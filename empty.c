/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * 小车固件 —— 整合版主程序(交接说明 §4)
 * 板: LP-MSPM0G3507   外设由 empty.syscfg 经 SysConfig 生成(11 个实例, 见交接说明 §3)
 *
 * 本版把 app 控制层(速度内环 + 循迹外环 + FSM)接进调度器, 取代之前的"电机开环 bring-up"版。
 * 上电后状态机处于 IDLE(电机不动), 按 START 键(PB19)才进入 RUN 开始跑线 —— 上电即静止是特意的安全设计。
 *
 * ⚠ NVIC 说明(容易漏, 别删):
 *   SysConfig 生成的 ti_msp_dl_config.c 只做"外设级"中断使能(DL_GPIO_enableInterrupt 等),
 *   NVIC(内核中断控制器)那一级 TI 的生成器从不代劳, 必须在 main 里手动 NVIC_EnableIRQ。
 *   漏掉的后果: 外设的中断标志会置起来, 但 CPU 永远不进 ISR —— 右编码器读数恒 0、
 *   K230 一个字节都收不到, 且编译/运行零报错。
 */
#include "ti_msp_dl_config.h"
#include "scheduler.h"
#include "bsp_i2c.h"    /* i2c_bus_recover: 开机 I2C 卡死从机自愈 */
#include "vofa.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "gray.h"
#include "servo.h"
#include "hmi.h"
#include "k230.h"
#include "app.h"

/* 全局 1ms 计数 —— 调度器时间基准(k230.c 超时判断也引用它) */
volatile uint32_t g_tick_ms = 0;

/* —— UART 底层发送 —— */
/* 发一段原始字节(VOFA 二进制帧用) */
static void uart_write(const unsigned char *buf, int len)
{
    for (int i = 0; i < len; i++) {
        DL_UART_transmitDataBlocking(UART_VOFA_INST, buf[i]);
    }
}
/* 发字符串(上电提示用) */
static void uart_puts(const char *s)
{
    while (*s) DL_UART_transmitDataBlocking(UART_VOFA_INST, (uint8_t)*s++);
}
/* 发无符号十进制数(诊断打数字用, 不引入 printf 这种重货) */
static void uart_print_u32(uint32_t v)
{
    char b[11];
    int  i = 10;
    b[10] = '\0';
    if (v == 0u) { uart_puts("0"); return; }
    while (v > 0u && i > 0) {
        b[--i] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    uart_puts(&b[i]);
}

/* —— 调度任务 —— */

/* 心跳灯: 每 500ms 翻一次 -> 1Hz 慢闪(活着的最直观证据, 死机/卡死一眼可见) */
static void task_heartbeat(void)
{
    led_heart_toggle();
}

/* 按键: 10ms 消抖扫描 + 事件分发(2026-07-04 定版语义, 满足 F4"按键设定运行模式"):
 * 待机态: START=按当前模式启动 | MODE=循环切模式(1→2→3→1, RUN灯闪模式号次作反馈)
 * 运动中: 任何键 = 急停(台架/现场安全铁律: 一伸手就能停) */
static void task_key(void)
{
    key_scan();
    if (key_pressed(KEY_START)) {
        if (app_get_state() == APP_ST_IDLE) app_start_request();
        else                                app_estop();
    }
    if (key_pressed(KEY_MODE)) {
        if (app_get_state() == APP_ST_IDLE) app_mode_cycle();
        else                                app_estop();
    }
}

/* K230 收帧推进: 5ms 把 RX 环形缓冲里的字节喂给解析状态机(非阻塞) */
static void task_k230(void)
{
    k230_poll();
}

/* ★灰度独占总线诊断开关(测完必改回 0!): 1 = 禁默 IMU 的一切 I2C 流量(init/校准/task_imu)。
 * 用途: 验证"感为自研 I2C 从机被发往其它地址(0x68)的流量搞死"假说——
 * 注意: 仅拔掉 MPU 设备不构成隔离, task_imu 仍会往 0x68 发地址包, 流量还在!(2026-07-04) */
#define GRAY_SOLO_TEST 0

/* IMU 航向积分推进: ★全工程唯一的 imu_update 调用点(多处调=积分重复推进=角度虚大)。
 * 盲走/到点判定/调试全部通过零副作用的 imu_get_yaw() 读缓存。 */
static void task_imu(void)
{
#if !GRAY_SOLO_TEST
    imu_update();
#endif
}

/* 速度内环: 10ms, 编码器实测速度 -> 增量式 PID -> motor_set */
static void task_vel(void)
{
    vel_loop_step();
}

/* 循迹外环: 20ms, 灰度偏差 -> 位置式 PID -> 更新左右目标速度(喂内环) */
static void task_track(void)
{
    track_loop_step();
}

/* 状态机: 10ms, IDLE/RUN/AIM/STOP/ESTOP 迁移 + 声光 */
static void task_fsm(void)
{
    app_fsm_step();
}

/* VOFA 调试波形: 只发"纯只读"量。
 * ⚠ 副作用规则: gray_get_error() 会做 I2C 读并搅动丢线状态, 只在 IDLE 代刷(见下);
 *   imu_get_yaw() 自 2026-07-04 重构后是纯读缓存(积分收口在 task_imu), 这里随便读;
 *   enc_get_count 是非破坏读(enc_get_delta 才是消费式), 安全。 */
static void task_vofa(void)
{
    float tl, ml, tr, mr, ol, or_;
    app_get_vel_debug(&tl, &ml, &tr, &mr);   /* 速度环只读快照(mm/s), 无副作用 */
    app_get_vel_out(&ol, &or_);              /* 两轮当前下发占空(-1000..1000) */

    /* 灰度偏差通道: IDLE 态没人刷新灰度(track 环只在 RUN 跑), 由这里代刷,
     * 方便台架"挥黑胶带看读数"; RUN 态只旁观缓存, 刷新权归 track 环独有。 */
    float gray_err;
    if (app_get_state() == APP_ST_IDLE) {
        gray_err = (float)gray_get_error();      /* IDLE: 代为刷新(含一次 I2C 读) */
    } else {
        gray_err = (float)gray_last_error();     /* 其它态: 纯读缓存, 零副作用 */
    }
    uint32_t g_ok, g_fail; uint8_t g_byte;
    gray_get_diag(&g_ok, &g_fail, &g_byte);      /* 灰度 I2C 诊断快照(纯读) */
    uint32_t imu_ok, imu_fail;
    imu_get_diag(&imu_ok, &imu_fail, 0);         /* IMU I2C 诊断快照(纯读) */

    float ch[19] = {
        (float)app_get_state(),          /* ch0: 状态机状态(0=IDLE 1=RUN 2=AIM 3=STOP 4=ESTOP) */
        (float)enc_get_count(ENC_LEFT),  /* ch1: 左轮累计计数(手转轮子应变化 -> 验 QEI) */
        (float)enc_get_count(ENC_RIGHT), /* ch2: 右轮累计计数(验 PA27 中断链路) */
        (float)k230_age_ms(),            /* ch3: 距上帧 K230 数据的毫秒数(不接 K230 时巨大值, 正常) */
        tl,                              /* ch4: 左轮目标速度 mm/s(RUN 态≈基速±转向量) */
        ml,                              /* ch5: 左轮实测速度 mm/s(低通后) —— 整定就看 ch4 vs ch5 */
        tr,                              /* ch6: 右轮目标速度 mm/s */
        mr,                              /* ch7: 右轮实测速度 mm/s —— 整定就看 ch6 vs ch7 */
        ol,                              /* ch8: 左轮占空(内环累加输出) —— 与 ch5 对照诊断积分/增益 */
        or_,                             /* ch9: 右轮占空 */
        gray_err,                        /* ch10: 灰度偏差(加权质心, 中线=0 左负右正) */
        (float)gray_is_lost(),           /* ch11: 丢线标志(1=丢线/全白全黑, 0=正常压线) */
        imu_get_yaw(),                   /* ch12: 航向角(°, 连续累计) —— 跑一圈应净变≈±360, 调 LAP 门限就看它 */
        (float)g_ok,                     /* ch13: 灰度 I2C 累计成功次数 —— 应持续上涨 */
        (float)g_fail,                   /* ch14: 灰度 I2C 累计失败次数 —— 独涨=电平/接触问题 */
        (float)g_byte,                   /* ch15: 灰度最近原始字节 —— 白纸=255, 盖住某路对应位变 0 */
        (float)app_get_mode(),           /* ch16: 运行模式(1=F1巡迹 2=F2定点瞄准 3=F3联动) */
        (float)imu_ok,                   /* ch17: IMU 突发读累计成功 —— 健康时每秒+100 */
        (float)imu_fail,                 /* ch18: IMU 突发读累计失败 —— 和 ch17 的比值=丢包率 */
    };
    vofa_send(ch, 19);
}

/* 任务表: { 函数, 周期ms, 计时器(初值=周期), 就绪标志 } —— 周期与交接说明 §4 / app.h 的 APP_*_DT_MS 一致 */
static task_t g_tasks[] = {
    { task_heartbeat, 500, 500, 0 },
    { task_key,        10,  10, 0 },
    { task_k230,        5,   5, 0 },
    { task_imu,        10,  10, 0 },   /* IMU 航向积分(唯一推进点), 排在用它的 track/fsm 之前 */
    { task_vel,        10,  10, 0 },   /* = APP_VEL_DT_MS */
    { task_track,      20,  20, 0 },   /* = APP_TRACK_DT_MS */
    { task_fsm,        10,  10, 0 },   /* = APP_FSM_DT_MS */
    { task_vofa,       20,  20, 0 },
};
#define N_TASKS ((uint8_t)(sizeof(g_tasks) / sizeof(g_tasks[0])))

int main(void)
{
    i2c_bus_recover();                /* I2C 卡死从机自愈: 必须在 SYSCFG_DL_init 之前(9时钟+STOP清场) */
    SYSCFG_DL_init();                 /* 时钟/GPIO/SysTick/UART×2/PWM×2/QEI/I2C 全部外设初始化 */

    /* NVIC 使能(见文件头说明, SysConfig 不代劳):
     * GPIO_IOA_INT_IRQN = GPIOA 组中断(右编码器 PA27, 向量名 GROUP1_IRQHandler, 实现在 encoder.c)
     * UART_K230_INST_INT_IRQN = UART2 RX(K230 收帧, ISR 在 k230.c) */
    NVIC_EnableIRQ(GPIO_IOA_INT_IRQN);
    NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);

    vofa_bind_writer(uart_write);     /* 把 VOFA 的底层发送口接到 UART(解耦关键) */

    /* 驱动初始化 —— hmi 提前, 让后面的初始化失败能用 ERR 灯报出来 */
    motor_init();                     /* STBY 使能、两轮 0 占空、启动 TIMA0 PWM */
    hmi_init();                       /* 蜂鸣器静音(PB18=高)、灯灭、按键状态清零 */
    enc_init();                       /* 启动 TIMG8 QEI 计数 + 右轮软件计数清零 */
    /* 感为灰度: 协议已按官方手册核定(0x4C+命令0xDD), init 内含 ping 在线检测 */
    if (gray_init() != 0) {
        led_err_set(1);
        uart_puts("[ERR] gray sensor offline: check 5V/pullup-3V3/PULL-cap-off/AD0-AD1-addr\r\n");
    }
    servo_init();                     /* TIMA1 50Hz 启动, 舵机轨 EN 保持断电 */
    k230_init();                      /* 清收帧状态 + 兜底使能 UART2 RX 外设级中断 */

#if GRAY_SOLO_TEST
    uart_puts("[TEST] GRAY_SOLO: IMU I2C traffic fully muted\r\n");
#else
    /* [DIAG] IMU 总线验尸探针(2026-07-04 排障用): 直接读 WHO_AM_I 并打出具体死法。
     * NACK    = 总线电气正常、但 0x68 无人应答 -> 模块没电 / 线没真到模块 / 地址不对;
     * TIMEOUT = 线被卡死或悬空                 -> 接触不良 / 排针孔没焊 / 短路;
     * BUS     = 仲裁丢失(SDA 被外力按住)。 */
    {
        int who = i2c_read_reg(0x68, 0x75);
        if (who >= 0) {
            uart_puts("[DIAG] IMU WHO_AM_I readable\r\n");
        } else if (who == BSP_I2C_ERR_NACK) {
            uart_puts("[DIAG] IMU probe: NACK - bus OK but no device answers (module power? wires reach module? addr?)\r\n");
        } else if (who == BSP_I2C_ERR_TIMEOUT) {
            uart_puts("[DIAG] IMU probe: TIMEOUT - line stuck/floating (bad contact? unsoldered header hole?)\r\n");
        } else if (who == BSP_I2C_ERR_BUS) {
            uart_puts("[DIAG] IMU probe: BUS/arb-lost - SDA held low\r\n");
        } else {
            uart_puts("[DIAG] IMU probe: other error\r\n");
        }

        /* 长度扫描探针(2026-07-04): "1字节通/14字节全灭"是硬分界而非随机丢包,
         * 扫 1/2/4/8/9/14 各5次找出精确死亡长度——若 8/9 之间断崖 = RX FIFO(深度8)边界问题。 */
        {
            static const uint8_t Ls[6] = { 1u, 2u, 4u, 8u, 9u, 14u };
            uint8_t tmp[14];
            for (int k = 0; k < 6; k++) {
                uint32_t okc = 0u;
                for (int t = 0; t < 5; t++) {
                    if (i2c_read_regs(0x68, 0x3B, tmp, Ls[k]) == 0) okc++;
                }
                uart_puts("[DIAG] IMU read len ");
                uart_print_u32(Ls[k]);
                uart_puts(": ");
                uart_print_u32(okc);
                uart_puts("/5\r\n");
            }
        }
    }

    /* IMU 初始化带自检(WHO_AM_I + I2C 应答), 失败点亮 ERR 红灯提示查线/上拉/AD0 地址 */
    if (imu_init() != 0) {
        led_err_set(1);
        uart_puts("[ERR] MPU6050 init failed: check wiring / 3.3V pullup / AD0\r\n");
    } else {
        /* 开机静止零漂校准(阻塞约 1 秒, 期间双灯同亮作指示; 上电时车天然静止)。
         * F1 整圈判定与丢线盲走都吃航向精度, 未校准的零漂 30s 能漂出好几度。 */
        led_run_set(1);
        led_err_set(1);
        uart_puts("IMU gyro calibrating, keep car still...\r\n");
        /* 校准 + 2 秒漂移自检, 不合格自动重校(最多 3 轮)。
         * 背景(2026-07-05): 航向锁正负符号都打转, 头号嫌疑=某些开机校准未生效导致 yaw
         * 匀速漂移, 航向环追着漂移参考跑必发散。此自检让每次开机自带体检报告:
         * [DIAG] yaw drift 行 <100(即 <1°/2s)才放行。典型 boot 增时 ~2s, 最坏 ~9s。 */
        int cal_ok = 0;
        for (int attempt = 1; attempt <= 3 && !cal_ok; attempt++) {
            if (imu_gyro_calibrate() != 0) {
                uint32_t cal_good;
                imu_get_diag(0, 0, &cal_good);
                uart_puts("[DIAG] cal attempt fail, good=");
                uart_print_u32(cal_good);
                uart_puts("/500, retry...\r\n");
                continue;
            }
            /* 2 秒静止漂移测量: 手动以 10ms 节拍推进积分(调度器此时还没开跑) */
            {
                float y0 = imu_get_yaw();
                uint32_t t0 = g_tick_ms, t_upd = t0;
                while ((g_tick_ms - t0) < 2000u) {
                    if ((g_tick_ms - t_upd) >= 10u) {
                        t_upd = g_tick_ms;
                        imu_update();
                    }
                }
                float dy = imu_get_yaw() - y0;              /* 2 秒净漂移(°) */
                float ady = (dy < 0.0f) ? -dy : dy;
                uart_puts("[DIAG] yaw drift x100: ");
                if (dy < 0.0f) uart_puts("-");
                uart_print_u32((uint32_t)(ady * 100.0f));   /* 单位: 百分之一度 / 2秒 */
                uart_puts(" cdeg per 2s\r\n");
                if (ady < 1.0f) {
                    cal_ok = 1;                             /* <1°/2s: 合格放行 */
                } else {
                    uart_puts("[DIAG] drift too high, recalibrating...\r\n");
                }
            }
        }
        if (cal_ok) {
            led_err_set(0);           /* 校准+漂移双合格: 红灯灭 */
            uart_puts("IMU calibrated & drift OK.\r\n");
        } else {
            uart_puts("[ERR] IMU calibrate/drift failed after 3 attempts\r\n");
        }
        led_run_set(0);
    }
#endif /* GRAY_SOLO_TEST */

    app_init();                       /* 建 3 个 PID + 状态置 IDLE(电机不动, 等 START 键) */

    /* 版本水印: 每轮整定改一次尾号, boot 一眼确认烧录生效(防"调了参数烧了个寂寞") */
    uart_puts("\r\n--- MSPM0 boot [tune-r8: IMU boot self-check] (IDLE, press START) ---\r\n");

    while (1) {
        sched_run(g_tasks, N_TASKS);  /* 跑所有"到点就绪"的任务 */
        __WFI();                      /* 没活就睡, 等下一次 1ms SysTick 唤醒 */
    }
}

/* 1ms 周期中断 (SysTick) —— 推进调度器时基 */
void SysTick_Handler(void)
{
    g_tick_ms++;
    sched_tick(g_tasks, N_TASKS);     /* 给每个任务计时器 -1, 到 0 置就绪并重装 */
}

/* MPU6050 六轴 IMU 驱动 —— firmware-scaffold driver 层
 *
 * 模块作用：经 I2C 读 MPU6050 加速度/陀螺原始值，并用“陀螺 Z 积分”给出航向角(yaw)。
 *
 * 用到的 SysConfig 实例/引脚（实际 I2C 时序由 bsp_i2c 负责，本文件不直接碰 DriverLib）：
 *   I2C_BUS(I2C1): SCL=PB2 SDA=PB3 -> I2C_BUS_INST（在 bsp_i2c.c 里使用，本模块只用 7 位设备地址）。
 *
 * 依赖的其它驱动：
 *   bsp_i2c —— 提供阻塞式“按寄存器”读写（见下方 extern 契约）。把 DL_I2C_* 时序细节收在 bsp_i2c，
 *   本模块只发 MPU6050 的寄存器协议，便于以后换总线实现/换 MCU（与 vofa 解耦 UART 同思路）。
 *
 * 设计约定：
 *   - 返回码统一 IMU_OK(0)/IMU_ERR(-1)，与 motor 风格一致。
 *   - 所有“量程换算系数 / 零漂 / 校准帧数”均为 #define 占位，标“待整定”，不填真值。
 *   - 2026-07-04 重构: 积分收口到 imu_update()(调度器唯一任务周期调, 用 g_tick_ms 真实 dt);
 *     imu_get_yaw() 变纯读缓存, 任意多处调用零副作用(修掉"每调一次多积一步"的隐患)。
 */
#include "mpu6050.h"
#include "bsp_i2c.h"   /* I2C 收口: i2c_write_reg / i2c_read_reg(返回值式) / i2c_read_regs */

/* empty.c 的 1ms 全局时基(SysTick 里自增), imu_update 用它算真实积分 dt */
extern volatile uint32_t g_tick_ms;

/* ===================== bsp_i2c 依赖契约（由 bsp_i2c.c 实现） =====================
 * 本模块不含任何 DL_I2C_* 调用；所有总线访问经 bsp_i2c.h 的阻塞接口：
 *   - addr7：7 位从机地址（不含读/写方向位），MPU6050 = MPU6050_I2C_ADDR。
 *   - reg  ：MPU6050 内部寄存器地址（先写该字节再读，即标准“写寄存器指针后重启读”时序）。
 *   - i2c_write_reg / i2c_read_regs 返回 0 成功、负值=NACK/仲裁丢失/总线超时等失败；
 *     i2c_read_reg 无出参指针：返回 >=0 即读到的字节(0~255)，<0 为错误码(调用方先判 <0)。 */

/* ===================== MPU6050 I2C 地址与寄存器地址（datasheet/RM-MPU-6000A） ===================== */
#define MPU6050_I2C_ADDR        0x68U   /**< 7 位地址：AD0 接地=0x68（接 VCC 则 0x69，本车 AD0 接地） */

#define MPU6050_REG_SMPLRT_DIV  0x19U   /**< 采样率分频：采样率 = 陀螺输出率 / (1+该值) */
#define MPU6050_REG_CONFIG      0x1AU   /**< DLPF 数字低通配置（含外部同步） */
#define MPU6050_REG_GYRO_CONFIG 0x1BU   /**< 陀螺量程配置（FS_SEL 在 bit4:3） */
#define MPU6050_REG_ACCEL_CONFIG 0x1CU  /**< 加速度量程配置（AFS_SEL 在 bit4:3） */
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU  /**< 数据区起始：ACCEL_XOUT_H，连续 14 字节到 GYRO_ZOUT_L */
#define MPU6050_REG_PWR_MGMT_1  0x6BU   /**< 电源管理1：bit6=SLEEP(休眠)，bit2:0=CLKSEL(时钟源) */
#define MPU6050_REG_WHO_AM_I    0x75U   /**< 器件 ID：正品 MPU6050 读回 0x68 */

/* WHO_AM_I 允许值集合：兼容片(MPU6500/9250 回 0x70/0x71、部分 MPU6000 回 0x68)放宽通过。
 * 待确认：实物到手用 imu_init 失败时打印实际 WHO_AM_I，再据芯片删减本集合。 */
#define MPU6050_WHOAMI_A        0x68U   /**< MPU6050/6000 */
#define MPU6050_WHOAMI_B        0x70U   /**< MPU6500 等兼容片 */
#define MPU6050_WHOAMI_C        0x71U   /**< MPU9250 等兼容片 */

/* ---- PWR_MGMT_1 写入值占位（解除休眠 + 选时钟源） ----
 * 0x00 = 解除 SLEEP + 用内部 8MHz 振荡器。常见更稳做法是选 PLL with X gyro = 0x01。
 * 待整定：先用 0x01（陀螺 X 轴 PLL，温漂更小），若启动异常回退 0x00。 */
#define MPU6050_PWR_WAKE_CLKSEL 0x01U   /* 待整定: 解除休眠并选时钟源(0x00=内部振荡, 0x01=PLL X 陀螺) */

/* ---- 量程配置寄存器写入值占位（默认最小量程，分辨率最高） ----
 * GYRO_CONFIG  FS_SEL: 0x00=±250dps 0x08=±500 0x10=±1000 0x18=±2000
 * ACCEL_CONFIG AFS_SEL:0x00=±2g    0x08=±4g  0x10=±8g    0x18=±16g
 * 待整定：转向角速度大时改 ±500/±1000dps（同时改下方 LSB 系数）。 */
#define MPU6050_GYRO_CONFIG_VAL  0x00U  /* 待整定: 陀螺量程, 默认 ±250dps */
#define MPU6050_ACCEL_CONFIG_VAL 0x00U  /* 待整定: 加速度量程, 默认 ±2g */

/* ---- DLPF / 采样率占位 ----
 * CONFIG=0x03: DLPF ~44Hz(陀螺), 抑制车体振动噪声; SMPLRT_DIV=0x09: 1kHz/(1+9)=100Hz 采样。
 * 待整定：采样率应与积分调用周期(IMU_DT_MS)匹配, 振动大可调 DLPF 更低带宽。 */
#define MPU6050_CONFIG_VAL       0x03U  /* 待整定: DLPF 带宽 */
#define MPU6050_SMPLRT_DIV_VAL   0x09U  /* 待整定: 采样率分频, 对应 100Hz */

/* ===================== 换算系数 / 积分参数 占位（全部待整定） ===================== */
/* 陀螺灵敏度：±250dps 量程下 = 131 LSB/(°/s)。改量程必须同步改这里：
 *   ±500->65.5  ±1000->32.8  ±2000->16.4 */
#define IMU_GYRO_LSB_PER_DPS    131.0f  /* 待整定: 须与 MPU6050_GYRO_CONFIG_VAL 量程一致 */
/* 加速度灵敏度：±2g 量程下 = 16384 LSB/g（当前仅供日后做互补滤波用，纯积分版用不到） */
#define IMU_ACC_LSB_PER_G       16384.0f /* 待整定: 须与 MPU6050_ACCEL_CONFIG_VAL 量程一致 */

/* 积分周期(毫秒)：imu_get_yaw 每被调用一次按此固定 dt 积分一次。
 * ⚠ 必须等于调用它的调度任务 period_ms（scheduler.h 的 task_t.period_ms），否则角度比例失真。 */
#define IMU_DT_MS               10      /* 待整定: 积分周期(ms), 须与调度任务周期一致(如 10ms=100Hz) */
#define IMU_DT_S                (IMU_DT_MS / 1000.0f)  /**< 积分步长(秒)，由 IMU_DT_MS 推得 */

/* 静止零漂校准参数占位 */
#define IMU_CAL_DISCARD_FRAMES  100     /* 待整定: 校准前丢弃的帧数(等芯片稳定) */
#define IMU_CAL_SAMPLE_FRAMES   500     /* 待整定: 求偏置平均用的采样帧数 */

/* 死区(°/s)：|去偏置后的角速度| 小于此值视为静止不积分，进一步压零漂。
 * 2026-07-04 验收后启用: 静止漂移可见(校准残余零漂), 0.3°/s 死区压掉它;
 * 循迹转弯角速度 ≥20°/s 远高于死区, 不受影响。若发现慢弯航向跟不上再调小。 */
#define IMU_GYRO_DEADBAND_DPS   0.3f    /* 待整定: 角速度死区(°/s) */

/* ===================== 模块内部状态 ===================== */
static float s_yaw_deg     = 0.0f;  /**< 累计航向角(度)，imu_get_yaw 积分结果，imu_reset_yaw 清零 */
static float s_gyro_z_bias = 0.0f;  /**< gyro_z 零漂偏置(LSB)，imu_gyro_calibrate 标定，积分时扣除 */

/* 把大端两字节(高字节在前)拼成 16 位带符号值。MPU6050 数据寄存器均为 H 在前、L 在后。
 * 用标准 static inline(不用 CMSIS 的 __STATIC_INLINE)：本模块刻意不引 DriverLib/CMSIS 头，
 * 仅依赖 bsp_i2c，便于移植(与 vofa 解耦同理)。 */
static inline int16_t be16_to_s16(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

/* 带重试的单笔读/写(2026-07-04 加): PA0/PA1 总线实测残留约 10~20% 散发丢包(长度扫描
 * len2~8 见 4/5), init 序列共 6 笔事务, 无重试时全过概率只有四成——纯掷硬币。
 * 每笔重试 5 次后, 单笔成功率 >99.99%, 序列成功率恢复确定性。 */
#define IMU_XFER_RETRIES  5
static int write_reg_retry(uint8_t reg, uint8_t val)
{
    for (int t = 0; t < IMU_XFER_RETRIES; t++) {
        if (i2c_write_reg(MPU6050_I2C_ADDR, reg, val) == 0) return 0;
    }
    return -1;
}
static int read_reg_retry(uint8_t reg)
{
    int r = -1;
    for (int t = 0; t < IMU_XFER_RETRIES; t++) {
        r = i2c_read_reg(MPU6050_I2C_ADDR, reg);
        if (r >= 0) return r;
    }
    return r;
}

int imu_init(void)
{
    uint8_t who = 0;

    /* 1) 读 WHO_AM_I 校验在不在允许集合里（确认地址/接线正确、芯片在线）。 */
    {
        int _t = read_reg_retry(MPU6050_REG_WHO_AM_I);
        if (_t < 0) {
            return IMU_ERR;   /* 重试后仍无应答：检查 AD0 地址、上拉、SCL/SDA 接线 */
        }
        who = (uint8_t)_t;
    }
    if (who != MPU6050_WHOAMI_A && who != MPU6050_WHOAMI_B && who != MPU6050_WHOAMI_C) {
        return IMU_ERR;   /* ID 不符：可能不是 MPU6050，或读到的是别的从机 */
    }

    /* 2) 解除休眠 + 选时钟源（复位后默认 SLEEP=1，不写这步读到的全是静止值/不更新） */
    if (write_reg_retry(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_WAKE_CLKSEL) != 0) {
        return IMU_ERR;
    }

    /* 3) 配置量程 / DLPF / 采样率（均为占位默认值，待整定） */
    if (write_reg_retry(MPU6050_REG_GYRO_CONFIG,  MPU6050_GYRO_CONFIG_VAL)  != 0) return IMU_ERR;
    if (write_reg_retry(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_VAL) != 0) return IMU_ERR;
    if (write_reg_retry(MPU6050_REG_CONFIG,       MPU6050_CONFIG_VAL)       != 0) return IMU_ERR;
    if (write_reg_retry(MPU6050_REG_SMPLRT_DIV,   MPU6050_SMPLRT_DIV_VAL)   != 0) return IMU_ERR;

    /* 4) 清零积分状态（偏置保持 0，待 imu_gyro_calibrate 静止标定） */
    s_yaw_deg     = 0.0f;
    s_gyro_z_bias = 0.0f;

    return IMU_OK;
}

/* ---- 运行时诊断计数(2026-07-04 加, 与 gray 同款仪表盘): 量化丢包率用 ---- */
static uint32_t s_imu_ok_cnt   = 0;   /**< 14字节突发读累计成功次数 */
static uint32_t s_imu_fail_cnt = 0;   /**< 累计失败次数 */
static uint32_t s_cal_good     = 0;   /**< 最近一次校准收集到的好样本数(fail时看它离500差多少) */

void imu_get_diag(uint32_t *ok_cnt, uint32_t *fail_cnt, uint32_t *cal_good)
{
    if (ok_cnt)   *ok_cnt   = s_imu_ok_cnt;
    if (fail_cnt) *fail_cnt = s_imu_fail_cnt;
    if (cal_good) *cal_good = s_cal_good;
}

int imu_read_raw(imu_raw_t *out)
{
    uint8_t buf[14];   /* 0x3B 起 14 字节：ACC_XYZ(6) + TEMP(2) + GYRO_XYZ(6) */

    if (out == 0) return IMU_ERR;

    /* 2026-07-04 改为 7+7 两段短读: I2C0(PA0/PA1) 上 >8 字节的长突发读 0/10 全灭
     * (单字节 100% 通 = 硬长度分界, 疑与 RX FIFO 深度 8 相关的控制器行为), 拆成两段
     * ≤7 字节绕开; 代价约 0.3ms/帧(100Hz 下可忽略), I2C1 上同样兼容。
     * 两段间隔 µs 级, MPU6050 寄存器按采样一致性锁存, 撕裂风险可忽略。 */
    if (i2c_read_regs(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H,
                      &buf[0], 7u) != 0 ||
        i2c_read_regs(MPU6050_I2C_ADDR, (uint8_t)(MPU6050_REG_ACCEL_XOUT_H + 7u),
                      &buf[7], 7u) != 0) {
        s_imu_fail_cnt++;
        return IMU_ERR;
    }
    s_imu_ok_cnt++;

    /* 大端拼装（每个量都是 H 字节在前） */
    out->accel_x = be16_to_s16(buf[0],  buf[1]);
    out->accel_y = be16_to_s16(buf[2],  buf[3]);
    out->accel_z = be16_to_s16(buf[4],  buf[5]);
    out->temp    = be16_to_s16(buf[6],  buf[7]);
    out->gyro_x  = be16_to_s16(buf[8],  buf[9]);
    out->gyro_y  = be16_to_s16(buf[10], buf[11]);
    out->gyro_z  = be16_to_s16(buf[12], buf[13]);

    return IMU_OK;
}

int imu_gyro_calibrate(void)
{
    /* 2026-07-04 重写为"容忍散发丢包"版: 旧版任何一次读失败就整体报废——但本车总线上
     * 挂着 5V 供电的感为灰度(其规格要求总线高电平≥72%供电=3.6V, 我们上拉 3.3V 在边缘),
     * 偶发 NACK/超时属于常态, 600 连读全成功的要求过于苛刻(实测真机就栽在这)。
     * 新逻辑: 只要在预算次数内凑够 IMU_CAL_SAMPLE_FRAMES 个"好样本"即成功;
     * 失败率超过 1/3(尝试预算耗尽)才判失败——那才是总线真有病。 */
    imu_raw_t r;
    float sum  = 0.0f;
    int   good = 0;
    int   tries;
    /* 尝试预算 = (丢弃 + 采样) * 1.5, 即允许约 1/3 的读取失败 */
    const int budget = (IMU_CAL_DISCARD_FRAMES + IMU_CAL_SAMPLE_FRAMES) * 3 / 2;
    int discard_left = IMU_CAL_DISCARD_FRAMES;

    for (tries = 0; tries < budget && good < IMU_CAL_SAMPLE_FRAMES; tries++) {
        if (imu_read_raw(&r) != IMU_OK) {
            continue;                    /* 散发失败: 跳过这帧, 不中止 */
        }
        if (discard_left > 0) {
            discard_left--;              /* 前若干个好帧只用于等芯片稳定, 不计入平均 */
            continue;
        }
        sum += (float)r.gyro_z;
        good++;
    }

    s_cal_good = (uint32_t)good;         /* 诊断: 无论成败都记录好样本数(量化丢包率) */
    if (good < IMU_CAL_SAMPLE_FRAMES) {
        return IMU_ERR;                  /* 预算内没凑够好样本: 失败率>1/3, 总线真有问题 */
    }
    s_gyro_z_bias = sum / (float)good;   /* LSB 偏置, 积分时扣除 */
    return IMU_OK;
}

void imu_update(void)
{
    imu_raw_t r;

    /* 真实经过时间(ms): 用 g_tick_ms 实测, 调度被拖延也不失真(与 app 测速同一套思路)。
     * 旧设计"每调一次积分固定 IMU_DT_S"有两个雷: ①多于一个调用方 -> 积分时间翻倍;
     * ②调度抖动 -> 角度比例失真。2026-07-04 重构为"唯一 update + 只读 get"。 */
    static uint32_t s_last_ms = 0u;
    uint32_t now_ms = g_tick_ms;
    uint32_t dt_ms  = now_ms - s_last_ms;

    /* 读一帧；读失败则本帧不积分、也**不更新时间戳**——这样丢帧期间的转角
     * 会由下一个成功帧按"跨越丢帧的真实 dt"一并补上(矩形近似), 而不是被静默吞掉。 */
    if (imu_read_raw(&r) != IMU_OK) {
        return;
    }
    s_last_ms = now_ms;   /* 只有读取成功才消耗时间窗 */
    if (dt_ms == 0u || dt_ms > 200u) {
        dt_ms = IMU_DT_MS;   /* 首拍/断点长暂停兜底 */
    }

    /* 去零漂 -> 转成 °/s（除以灵敏度）。当前为纯陀螺积分，长期必漂，见文件末尾改进说明。 */
    float gz_dps = ((float)r.gyro_z - s_gyro_z_bias) / IMU_GYRO_LSB_PER_DPS;

    /* 死区：极小角速度视为静止不积分，进一步压零漂（IMU_GYRO_DEADBAND_DPS=0 时不生效） */
    float gz_abs = (gz_dps < 0.0f) ? -gz_dps : gz_dps;   /* |角速度| */
    if (gz_abs >= IMU_GYRO_DEADBAND_DPS) {
        s_yaw_deg += gz_dps * ((float)dt_ms / 1000.0f);
    }
}

float imu_get_yaw(void)
{
    /* 纯读缓存, 零副作用: 任意多个调用方(盲走/到点判定/VOFA)随便读, 互不干扰。
     * 积分只发生在 imu_update()(由调度器唯一任务周期调)。 */
    return s_yaw_deg;
}

void imu_reset_yaw(void)
{
    s_yaw_deg = 0.0f;   /* 仅清角度，不动已标定的零漂偏置 */
}

/* ===================== 后续改进说明（占位，留给整定阶段） =====================
 * 当前 imu_get_yaw 是“纯陀螺 Z 积分”：
 *   优点：短时间响应快、对加速度计噪声不敏感；
 *   缺点：零漂会随时间累积，长跑必偏（即便做了静止校准）。
 * 升级路线（不改对外接口，只改 imu_get_yaw 内部）：
 *   1) 互补滤波：用加速度计算俯仰/横滚做长期参考，与陀螺积分加权融合
 *      （注意：单 MPU6050 无磁力计，yaw 无绝对参考，加速度计帮不了 yaw 的长期漂移，
 *       真要消 yaw 漂移需外加磁力计/视觉/编码器航迹推算做融合）。
 *   2) 卡尔曼：状态=角度+角速度偏置，过程/观测噪声为待整定参数。
 *   3) 工程折中：本车有左右编码器，可用差速里程计推 yaw 与陀螺积分互补，鲁棒性更好。
 */

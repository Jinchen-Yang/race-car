/* app —— 顶层应用骨架(控制环 + 状态机)  [firmware-scaffold APP 层]
 *
 * 见 app.h 顶部说明。本文件只搭"结构与接口", 控制逻辑/增益/阈值/几何公式全部占位:
 *   - 凡 #define 的调参常量都标 "待整定", 给的是占位数, 不是真值;
 *   - 凡需现场标定/题目回填的算法步骤都标 TODO, 骨架里走"安全的占位行为"(多为停/中位/直行)。
 *
 * 【依赖驱动对外函数】(若某驱动尚未由学长学姐建好, 该 #include 会报缺头, 属预期:
 *   按 app.h 列出的契约名建好对应 .h/.c 即可编译。本层只调对外函数, 不碰 DL_*。)
 *   motor.h   : motor_set / motor_stop_all / MOTOR_LEFT / MOTOR_RIGHT
 *   pid.h     : pid_t / pid_init / pid_reset / pid_update / pid_inc_update
 *   encoder.h : enc_get_delta / ENC_LEFT / ENC_RIGHT
 *   gray.h    : gray_get_error / gray_is_lost
 *   mpu6050.h : imu_get_yaw
 *   servo.h   : servo_set_angle(ch,deg) / servo_rail_enable(on) / SERVO_PAN / SERVO_TILT
 *   hmi.h     : beep_on / beep_off / led_run_set / led_err_set
 *   k230.h    : k230_get_aim
 */
#include <stdint.h>
#include <math.h>       /* aim_geometry: atan2f/cosf/sinf/sqrtf, ticlang 自带 */
#include "app.h"
#include "motor.h"
#include "pid.h"
#include "encoder.h"
#include "gray.h"
#include "mpu6050.h"
#include "servo.h"
#include "hmi.h"
#include "protocol.h"
#include "k230.h"

/* empty.c 的 1ms 全局时基(SysTick 里自增)。测速用真实经过时间而非名义周期:
 * track 环的 I2C 读失败超时会拖延调度, 若按名义 10ms 算速度会虚高 2~3 倍(2026-07-04 实测坐实)。 */
extern volatile uint32_t g_tick_ms;

/* ============================================================================
 *  占位调参常量 —— 全部 #define + "待整定", 不填真值
 * ==========================================================================*/

/* ★台架诊断开关(诊断完必须改回 0!): 1 = 速度环旁路——RUN 态两轮恒占空开环, 只测速不闭环。
 * 用途: 把"电机方向 / 编码器符号 / 测速换算 / 电气噪声"与 PID 动力学拆开定位。
 * 判读: 恒占空下轮子应匀速前进、I1/I2 应稳定上涨、I5/I7 应为稳定正值;
 *       哪一条不满足, 问题就在被控对象/传感器那一侧, 与 PID 无关。(2026-07-04 排障用) */
#define VEL_OPEN_LOOP_TEST   0
#define VEL_OPEN_LOOP_DUTY   200   /* 恒定占空(-1000..1000), 与六月电机开环实测同款值 */

/* --- 速度内环(增量式 PID, 每轮一个) ---
 * 2026-07-04 开环标定: 占空200 -> 左≈400/右≈380 mm/s => 对象增益 G≈2.0 mm/s每占空单位;
 * 跑 300mm/s 稳态占空≈150。增益按"回路增益 Kp*G≈0.3"设计, 积分时间常数≈170ms。
 * (历史: Kp=1/Ki=0 剧烈振荡; Kp=0.3/Ki=0.08 仍振荡——对 G=2.0 的对象回路增益过高。) */
#define VEL_KP            0.15f  /* 待整定: 空载已按实测对象增益设计; 带载后地面复整 */
#define VEL_KI            0.03f  /* 待整定: 同上; 收敛慢可加大, 振荡则减小 */
#define VEL_KD            0.0f   /* 待整定: 速度内环微分增益(占位) */
#define VEL_OUT_MIN   (-1000.0f) /* 待整定: 内环输出(占空)下限, 对应 motor duty 标度 */
#define VEL_OUT_MAX    (1000.0f) /* 待整定: 内环输出(占空)上限 */
#define VEL_INTEG_MAX   (500.0f) /* 待整定: 内环积分限幅(抗饱和) */
/* 编码器计数 -> 实测速度(mm/s)换算, 必须分轮——左右解码倍频不同(左 QEI 4倍频/右单相1倍频),
 * 每圈计数已实测并存 encoder.h(左 1017.4 / 右 265.2, 2026-07-03 十圈法)。
 * 每计数毫米数 = 轮周长(π·D) / 每圈计数; 实测速度 = delta * 每计数毫米数 / dt, 单位 mm/s。 */
#define WHEEL_DIAM_MM     47.0f  /* 2026-07-05 实测≈4.7cm; 再精一步的标法: 贴地划线推整 2.00m 读计数反推 */
#define ENC_L_MM_PER_CNT  (3.1415926f * WHEEL_DIAM_MM / ENC_L_CNT_PER_REV)  /* 左轮 mm/计数 */
#define ENC_R_MM_PER_CNT  (3.1415926f * WHEEL_DIAM_MM / ENC_R_CNT_PER_REV)  /* 右轮 mm/计数 */
#define VEL_MEAS_LPF      0.5f   /* 待整定: 测速一阶低通系数(0~1,越小越平滑),抑制编码器抖动(等效参考工程的多点滑窗) */

/* --- 循迹外环(位置式 PID) --- */
#define TRACK_KP          0.4f   /* 整定中: r1=1.0蛇形 r2=0.5蛇形 r3=0.3摆小但弯道拉不住 r4=0.4+KD */
#define TRACK_KD          1.0f   /* 整定中: r4 首次引入, 抑制 KP 回抬带来的摆动; 抖得高频再减半 */
#define TRACK_OUT_LIM   (400.0f) /* 待整定: 转向量限幅(占位) */
#define BASE_SPEED        300    /* 待整定: 巡迹基速, 单位 mm/s(=0.3m/s, 起步偏保守; 换算已定标, 单位从"当量"变真实物理量) */
/* 左右轮安装多为镜像, 整车直行需对其中一轮取反: 这两个符号现场标定。 */
#define WHEEL_SIGN_L     (+1)    /* 待整定: 左轮目标速度符号(占位, 可能取反) */
#define WHEEL_SIGN_R     (+1)    /* 待整定: 右轮目标速度符号(占位, 可能取反) */

/* --- 丢线盲走(IMU 航向锁定) --- */
#define HEADING_KP        0.0f   /* r7 暂时禁用(回到盲走直行): ⚠ +3/-3 两个符号都打转 —— 线性回路
                                  * 只有一个符号会发散, 两向皆疯说明疯的是航向反馈本身(头号嫌疑:
                                  * 摔机后某些开机 IMU 校准未过, yaw 持续漂移, 航向锁追着漂移参考跑)。
                                  * 待 ch12 静止漂移率/转90°响应两项体检通过后再启用并定符号。 */
#define HEADING_LPF_A     0.05f  /* 压线期航向滑动平均系数(20ms拍): τ≈0.4s, 滤掉蛇形摆动的
                                  * 相位, 得到黑线真实走向; 丢线时锁的是这个平均值而非瞬时值 */
#define LOST_BLIND_MAX    2000   /* 待整定: 盲走最大里程当量, 超限报错(占位) */

/* --- 瞄准 --- */
#define AIM_TIMEOUT_MS    5000   /* 待整定: 单点瞄准限时(题目 F2≤5s, 占位) */
#define AIM_TOL_PX        8      /* 待整定: K230 命中容差(像素)(占位) */
#define AIM_K_PAN         0.02f  /* 待整定: dx->pan 微调增益(符号待标)(占位) */
#define AIM_K_TILT        0.02f  /* 待整定: dy->tilt 微调增益(符号待标)(占位) */

/* --- 瞄准几何: 场地/车体/云台坐标(mm) ---
 * 世界坐标系: 原点=A, x 轴沿 A→B, y 轴垂直 AB 指向靶侧(y>0), z 垂直向上。
 * F2/F3 对靶时车都停在 B, 车头基本沿 A→B(+x)方向。IMU 出发航向 g_lap_yaw0
 * 作为世界系的 +x, 当前航向偏差量 = 车头相对 +x 的转角(度)。
 *
 * ★不做全场推航: IMU 已知 1°/2s 漂移, 累积到 B 会跑偏 20cm+ (车已跑 5s 以上),
 *   静态"车在 B"的常量近似比动态积分更靠谱(2026-07-04 firmware-codebase 记忆结论)。 */
#define AIM_PI            3.14159265358979f
#define AIM_DEG2RAD(d)    ((d) * (AIM_PI / 180.0f))
#define AIM_RAD2DEG(r)    ((r) * (180.0f / AIM_PI))

#define AB_LEN_MM         1400.0f  /* 待整定: A→B 直线长度(2024H 同款推测≈1400mm, 与 KP_GATES 一致) */
#define B_X_MM            AB_LEN_MM /* 车停 B 时的世界 x = AB 长度 */
#define B_Y_MM            0.0f      /* 车停 B 时的世界 y = 0 (B 在 x 轴上) */

/* 靶位: 题目 PDF 确认"靶距 AB 外侧 50cm, 与 AB 平行", 沿 x 方向落点未硬定 */
#define TARGET_X_MM       700.0f   /* 待整定: 靶心沿AB方向x坐标(占位=AB中点对面, 场地量) */
#define TARGET_Y_MM       500.0f   /* PDF 确定: 靶距 AB 外侧 500mm */
#define TARGET_Z_MM       500.0f   /* 待整定: 靶心离地高度(题目≤500mm, 占位=500) */

/* 云台安装(相对车中心, 车体坐标系): dx 前正、dy 右正、z 离地 */
#define CAR_GIMBAL_DX_MM   0.0f    /* 待整定: 云台安装前后偏移(前+, 占位=车中心正上) */
#define CAR_GIMBAL_DY_MM   0.0f    /* 待整定: 云台左右偏移(右+, 占位=车中线) */
#define CAR_GIMBAL_Z_MM  250.0f    /* 待整定: 云台旋转中心离地高度(车高 250mm, 占位=车顶) */

/* 舵机零点/符号: 与舵机装配朝向相关, 第一次通电试转标定
 * PAN_ZERO_DEG   = 车头正前方(pan 目标=0°几何角)对应的舵机输入角度
 * TILT_ZERO_DEG  = 云台水平(tilt=0°几何角)对应的舵机输入角度
 * _SIGN 若正是"几何角增大, 舵机角也增大"; 若装反了置 -1 */
#define AIM_PAN_ZERO_DEG   90.0f
#define AIM_TILT_ZERO_DEG  90.0f
#define AIM_PAN_SIGN      (+1.0f)  /* 待标: 第一次目视试转, 反了改 -1 */
#define AIM_TILT_SIGN     (+1.0f)  /* 待标: 同上 */

/* ===== 运行模式(F4: 按键/串口选模式; IDLE 态 MODE 键循环或串口发'1'~'4', RUN 灯闪"模式号"次) =====
 * 1 = F1 自动巡迹一圈回 A 停车(≤30s)
 * 2 = F2 定点瞄准(静止, 5s 内对靶)
 * 3 = F3 巡迹到靶联动(B 停车对靶 -> C/D 过点声光 -> 回 A 停车, ≤40s)
 * 4 = 发挥3 四圈连跑(每圈过 A 声光计圈, 跑满 4 圈停车) */
#define APP_MODE_MAX      4
#define APP_LAPS_MODE4    4      /* 发挥3 要求的连跑圈数 */

/* --- 关键点顺序门限表(B,C,D,A; 双条件"都超过"才判到点; 全部待整定) ---
 * 场地 = 2024H 同款(题目 Figure1 标注): A→B 直线, B→C 半弧(累计+180°), C→D 直线, D→A 半弧(+360°)。
 * 航向用 |净转角|(顺/逆时针通吃), 里程用右轮累计。理论: 直线≈1400mm, 半弧≈π*400≈1257mm。
 * 门限取"理论值 × ~0.8"留裕量, 靠顺序推进防误触; 真场地量了尺寸后回填。 */
typedef struct {
    float yaw_min_deg;    /**< 累计|航向|门限(°); 0 = 不设航向条件(纯里程) */
    float dist_min_mm;    /**< 累计里程门限(mm) */
} kp_gate_t;
static const kp_gate_t KP_GATES[4] = {
    /* B */ {   0.0f, 1100.0f },   /* 待整定: 第一直线尽头, 航向基本没转, 纯里程判 */
    /* C */ { 150.0f, 2100.0f },   /* 待整定: 第一个半弧走完(航向≥150°) */
    /* D */ { 150.0f, 3400.0f },   /* 待整定: 第二直线尽头 */
    /* A */ { 330.0f, 4000.0f },   /* 待整定: 整圈(原 F1 判定, F1/F3 共用) */
};

/* --- 到点声光 --- */
#define KEYPOINT_BEEP_MS  400    /* 待整定: 到点蜂鸣时长; 线上评审靠视频, 给足 400ms 让声光在片子里清晰可辨 */

/* ============================================================================
 *  模块内状态
 * ==========================================================================*/

/* 可在中断里被 app_estop 写、主循环 FSM 读, 必须 volatile(防编译器把它缓存到寄存器)。 */
static volatile app_state_e g_state = APP_ST_IDLE;   /**< 当前状态机状态 */

/* 三个 PID 控制器(占位增益, 真车整定) */
static pid_t g_pid_track;   /**< 循迹外环: 灰度/航向偏差 -> 转向量(位置式) */
static pid_t g_pid_vel_l;   /**< 速度内环-左: 编码器速度(增量式) */
static pid_t g_pid_vel_r;   /**< 速度内环-右: 编码器速度(增量式) */

/* 外环写、内环读的"左右目标速度"(速度当量, 非占空) */
static float g_vel_tgt_l = 0.0f;   /**< 左轮目标速度(track 外环设定) */
static float g_vel_tgt_r = 0.0f;   /**< 右轮目标速度 */

/* 内环累加输出(增量式 PID 的 Δu 累加到这里, 即下发给 motor 的占空) */
static float g_vel_out_l = 0.0f;   /**< 左轮当前下发占空(累加值) */
static float g_vel_out_r = 0.0f;   /**< 右轮当前下发占空(累加值) */

/* 可在中断里被 app_start_request 写、主循环 FSM 读, 必须 volatile。 */
static volatile uint8_t g_start_req = 0;   /**< 启动请求标志(app_start_request 置位, FSM 消费) */
static uint32_t g_aim_timer  = 0;   /**< 瞄准计时(ms), AIM 态累加用于超时判定 */
static uint32_t g_lost_dist  = 0;   /**< 丢线盲走累计里程当量(超 LOST_BLIND_MAX 报错) */

/* --- 速度内环测速滤波(每轮一阶低通状态) --- */
static float g_meas_filt[2] = {0.0f, 0.0f};   /**< [ENC_LEFT/RIGHT] 测速低通状态,抑抖 */
/* --- 丢线盲走(IMU 航向锁定)状态 --- */
static float   g_yaw_lock     = 0.0f;   /**< 丢线时锁定的航向基准(度, 取压线期滑动平均) */
static float   g_yaw_line_avg = 0.0f;   /**< 压线期航向滑动平均(滤蛇形): 黑线真实走向的估计 */
static int32_t g_blind_ref    = 0;      /**< 丢线瞬间的里程基准(右轮计数) */
static uint8_t g_blind_active = 0;      /**< 盲走中标志(抓丢线上升沿,基准只锁一次) */
/* --- 状态机入态检测 + STOP 声光一次性 --- */
static app_state_e g_prev_state = APP_ST_IDLE;   /**< 上一拍状态(检测"刚入态") */
static int32_t g_stop_beep_ms   = 0;             /**< STOP 到点蜂鸣剩余时长(ms),非阻塞关断 */
/* --- 关键点判定基准(进 RUN 时快照)与任务进度 --- */
static float   g_lap_yaw0 = 0.0f;   /**< 出发瞬间航向(°), 累计净转角的零点 */
static int32_t g_lap_odo0 = 0;      /**< 出发瞬间右轮累计计数, 累计里程的零点 */
static uint8_t g_run_mode = 1;      /**< 运行模式 1..APP_MODE_MAX(IDLE 态 MODE 键循环) */
static uint8_t g_kp_idx   = 0;      /**< 关键点序列进度: 0=还没到B ... 4=已回A */
static uint8_t g_lap_count = 0;     /**< 已完成圈数(模式4 四圈连跑用) */

/* --- 在线调参副本(串口实时调, #define 只当默认值; 现场整定免重编译烧录) --- */
static float g_track_kp   = TRACK_KP;    /**< 循迹外环 KP(运行时可调) */
static float g_track_kd   = TRACK_KD;    /**< 循迹外环 KD(运行时可调) */
static int   g_base_speed = BASE_SPEED;  /**< 巡迹基速 mm/s(运行时可调) */
static uint8_t g_aim_return_run = 0;/**< AIM 结束后回 RUN(F3 中途对靶)还是进 STOP(F2) */
static int32_t g_run_beep_ms = 0;   /**< RUN 态过点声光剩余时长(ms), 非阻塞关断 */

/* ============================================================================
 *  小工具
 * ==========================================================================*/

/* 浮点限幅(本层小工具, 不引外部数学库) */
static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* 角度归一到 [-180,180]：处理航向在 ±180 处的跨界翻转(借鉴参考工程的 Yaw wrap 处理),
 * 否则 (yaw0 - yaw) 在越过 ±180 时会突变一大截,导致盲走瞬间猛打方向。 */
static float wrap180(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* ============================================================================
 *  速度内环 —— 每轮一个增量式 PID
 * ==========================================================================*/

/**
 * @brief 单轮速度内环计算并下发(内部用)
 * @param ch     轮通道: ENC_LEFT/ENC_RIGHT(=MOTOR_LEFT/MOTOR_RIGHT, 编号同值 0/1,
 *               既给 enc_get_delta 取编码器增量, 也给 motor_set 下发占空)
 * @param p      该轮的 PID 控制器
 * @param target 目标速度(当量)
 * @param out    [in/out] 该轮累加输出(占空), 本函数把 Δu 累加进去并限幅
 * @note  实测速度(mm/s) = enc_get_delta(ch) * 该轮 mm/计数 / dt(按轮取 ENC_L/R_MM_PER_CNT,
 *        每圈计数已实测, 见 encoder.h)。增量式 PID 返回本次占空增量 Δu, 累加到 out 后限幅再 motor_set。
 *        ⚠ 轮径 WHEEL_DIAM_MM 与 PID 增益仍是占位, 待实测/整定。
 */
/**
 * @brief 单轮测速(内部用): 编码器增量 -> 低通后的实测速度(mm/s), 并刷新 g_meas_filt
 * @note  与控制解耦, 开环诊断模式下也照常调用(只测不控)。
 *        ⚠ 换算系数必须按轮取: 左右解码倍频差 3.84 倍, 用错轮的系数速度环立刻乱。
 *        ⚠ dt 用 g_tick_ms 实测而非名义 APP_VEL_DT_MS: 调度可能被 I2C 超时等拖延,
 *          按名义周期算会把速度虚高 2~3 倍, 环会追着假数据振荡(2026-07-04 实测坐实)。
 */
static float vel_measure(int ch)
{
    int32_t delta = enc_get_delta(ch);
    int fi = (ch == ENC_RIGHT) ? 1 : 0;
    static uint32_t s_last_ms[2] = {0u, 0u};
    uint32_t now_ms = g_tick_ms;
    uint32_t dt_ms  = now_ms - s_last_ms[fi];
    s_last_ms[fi]   = now_ms;
    if (dt_ms == 0u || dt_ms > 200u) {
        dt_ms = APP_VEL_DT_MS;   /* 首拍/长暂停(调试器断点等)兜底, 防除零与荒谬尖峰 */
    }
    float mm_per_cnt = (ch == ENC_RIGHT) ? ENC_R_MM_PER_CNT : ENC_L_MM_PER_CNT;
    float meas_raw = (float)delta * mm_per_cnt / ((float)dt_ms / 1000.0f);
    /* 尖峰保护: 本车物理极限约 ±1200mm/s(占空600, 12V空载外推), 超 ±1500 必是毛刺
     * (噪声/计数异常/时序抖动), 夹掉防止污染滤波器与控制环(2026-07-04 曾实测到 -1931 的假值)。 */
    if (meas_raw >  1500.0f) meas_raw =  1500.0f;
    if (meas_raw < -1500.0f) meas_raw = -1500.0f;
    /* 一阶低通滤波抑制编码器抖动/量化噪声(等效参考工程的多点滑窗测速),每轮各一份状态。 */
    g_meas_filt[fi] = g_meas_filt[fi] * (1.0f - VEL_MEAS_LPF) + meas_raw * VEL_MEAS_LPF;
    return g_meas_filt[fi];
}

static void vel_loop_one(int ch, pid_t *p, float target, float *out)
{
    /* 1) 测速(带低通, mm/s)。 */
    float meas = vel_measure(ch);

    /* 2) 增量式 PID: 返回本次占空"增量" Δu(增量式天然抗积分饱和)。 */
    float du = pid_inc_update(p, target, meas);

    /* 3) 累加到该轮下发占空并限幅(增量式必须自己维护累加量)。 */
    *out = clampf(*out + du, VEL_OUT_MIN, VEL_OUT_MAX);

    /* 4) "瞎油门保险丝"(2026-07-05, 摔坏左编码器实锤的教训): 编码器一瞎, 闭环变成
     *    瞎子踩油门——占空顶满而测速为零, 轮子实际全速失控。判据: |占空|≥90%上限
     *    且 |实测|<50mm/s 持续 500ms => 传感器/执行器故障, 急停 + 红灯。 */
    {
        static uint16_t s_stall_ms[2] = { 0u, 0u };
        int fi = (ch == ENC_RIGHT) ? 1 : 0;
        float mag_out  = (*out < 0.0f) ? -*out : *out;
        float mag_meas = (meas < 0.0f) ? -meas : meas;
        if (mag_out >= 0.9f * VEL_OUT_MAX && mag_meas < 50.0f) {
            s_stall_ms[fi] = (uint16_t)(s_stall_ms[fi] + APP_VEL_DT_MS);
            if (s_stall_ms[fi] >= 500u) {
                s_stall_ms[0] = s_stall_ms[1] = 0u;
                app_estop();   /* 内部停电机断舵机轨, FSM 进 ESTOP(红灯), RESET 才能退出 */
                return;
            }
        } else {
            s_stall_ms[fi] = 0u;
        }
    }

    /* 5) 下发到电机(motor_set 内部带方向/限幅; duty 标度 -1000..1000)。 */
    motor_set(ch, (int)(*out));
}

void vel_loop_step(void)
{
    /* 仅在"该出力"的态下闭环驱动; 其它态(IDLE/AIM/STOP/ESTOP)电机另行处理。 */
    if (g_state != APP_ST_RUN) {
        return;
    }
#if VEL_OPEN_LOOP_TEST
    /* ★开环诊断: 恒占空直驱 + 照常测速(只测不控)。VOFA 上 I5/I7 显示的是真实测速,
     * I4/I6 目标值此模式下无意义。诊断完把 VEL_OPEN_LOOP_TEST 改回 0! */
    (void)vel_measure(ENC_LEFT);
    (void)vel_measure(ENC_RIGHT);
    g_vel_out_l = g_vel_out_r = (float)VEL_OPEN_LOOP_DUTY;   /* 供调试观察 */
    motor_set(MOTOR_LEFT,  VEL_OPEN_LOOP_DUTY);
    motor_set(MOTOR_RIGHT, VEL_OPEN_LOOP_DUTY);
#else
    vel_loop_one(ENC_LEFT,  &g_pid_vel_l, g_vel_tgt_l, &g_vel_out_l);
    vel_loop_one(ENC_RIGHT, &g_pid_vel_r, g_vel_tgt_r, &g_vel_out_r);
#endif
}

/* ============================================================================
 *  循迹外环 —— 灰度闭环 + 丢线 IMU 航向锁定
 * ==========================================================================*/

/**
 * @brief 丢线盲走转向量(内部用): IMU 航向锁定占位
 * @return 转向量(叠加到基速做差速); 占位返回 0(直行)
 * @note  ⚠ 占位: 真实实现 = 锁定丢线前航向 yaw0, 用 imu_get_yaw() 算航向误差,
 *        HEADING_KP * 误差 出转向量; 并累加 g_lost_dist, 超 LOST_BLIND_MAX 报错。
 *        待整定: HEADING_KP、航向锁定基准、里程门限。
 */
static int track_blind_turn(void)
{
    /* 刚丢线(上升沿): 锁定航向基准 + 里程基准, 只锁一次(g_blind_active 由 track 压线时清)。
     * ★锁"压线期滑动平均航向"而非瞬时航向: 离线瞬间车头正处在蛇形摆动的随机相位上,
     * 带着几度偏角, 锁瞬时值会沿偏角走斜线(2026-07-05 实测坐实); 平均值滤掉摆动相位,
     * 剩下的就是黑线的真实走向, 按它直走才能命中对面胶带首端。 */
    if (!g_blind_active) {
        g_yaw_lock    = g_yaw_line_avg;            /* 锁"线的方向", 不是"车头此刻的方向" */
        g_blind_ref   = enc_get_count(ENC_RIGHT);  /* 里程基准: 右轮 int32 累加值,不回绕 */
        g_blind_active = 1;
    }

    /* 航向锁定: 用"基准航向 - 当前航向"的误差算转向量, 让车沿丢线前的方向直走,
     * 平稳穿过虚线/弧段(而非瞎猜)。wrap180 处理 ±180 跨界。HEADING_KP 待整定。 */
    float yaw  = imu_get_yaw();
    int   turn = (int)(HEADING_KP * wrap180(g_yaw_lock - yaw));

    /* 盲走里程 = |当前右轮计数 - 基准|; 超上限说明虚线/弧段异常(压根没接回线), 报错。 */
    int32_t d = enc_get_count(ENC_RIGHT) - g_blind_ref;
    g_lost_dist = (uint32_t)(d < 0 ? -d : d);
    if (g_lost_dist > LOST_BLIND_MAX) {
        led_err_set(1);   /* TODO: 盲走超限 -> 可选进 STOP/ESTOP, 现仅点错误灯示警 */
    }
    return turn;
}

void track_loop_step(void)
{
    if (g_state != APP_ST_RUN) {
        return;
    }

    /* 灰度加权质心偏差(居中≈0, 符号见标定); gray_get_error 内部刷新一次并更新丢线标志,
     * 紧接着用 gray_is_lost() 查本次是否全丢线(全黑/全白)。 */
    int err  = (int)gray_get_error();
    int lost = (int)gray_is_lost();

    int turn;
    if (!lost) {
        /* 压线: 外环位置式 PID, 目标偏差=0, 实测=err, 出转向量。 */
        g_lost_dist = 0;        /* 重新压到线: 清盲走里程 */
        g_blind_active = 0;     /* 解除航向锁, 下次丢线重新锁基准 */
        /* 持续更新"线的走向"估计: 航向滑动平均, 滤掉蛇形摆动相位(盲走锁它, 见 track_blind_turn) */
        g_yaw_line_avg += HEADING_LPF_A * (imu_get_yaw() - g_yaw_line_avg);
        turn = (int)pid_update(&g_pid_track, 0.0f, (float)err);
    } else {
        /* 丢线(虚线/弧段): 切 IMU 航向锁定盲走(占位)。 */
        turn = track_blind_turn();
    }

    /* 串级: 基速 ± 转向量 -> 左右目标速度(带安装符号补偿); 写给内环执行。
     * ⚠ WHEEL_SIGN_L/R 现场标定(镜像安装可能其一取反)。基速用运行时副本(串口可调)。 */
    g_vel_tgt_l = (float)(WHEEL_SIGN_L * (g_base_speed - turn));
    g_vel_tgt_r = (float)(WHEEL_SIGN_R * (g_base_speed + turn));
}

/* ============================================================================
 *  瞄准 —— 几何主路径(无视觉) + K230 末端精修
 * ==========================================================================*/

/**
 * @brief 几何反解云台指向(无视觉主路径, 2026-07-05 落地真公式)
 * @param pan_deg  [out] PAN 目标舵机角(度, 0..180)
 * @param tilt_deg [out] TILT 目标舵机角(度, 0..180)
 *
 * 【坐标系】原点=A, x沿A→B, y垂直AB指向靶(y>0), z 垂直向上。
 * 【假设】F2/F3 对靶时车停在 B; 车头相对世界 +x 的角度 = 当前IMU航向 − 出发航向(度)。
 *         若真实静止位置不在 B, 只需改 B_X/Y 或加"F2专用车位"常量, 计算不变。
 * 【流程】1) 由车头朝向把云台安装偏移(车体系)旋转到世界系, 得云台在场上坐标 G=(Gx,Gy);
 *         2) 靶 T=(TARGET_X, TARGET_Y, TARGET_Z), 相对G水平向量 (rx,ry) = (Tx-Gx, Ty-Gy);
 *         3) Pan(世界系) = atan2(ry, rx); Pan(车体系) = Pan(世界系) − 车头朝向;
 *         4) 水平距离 d = √(rx²+ry²), 垂直差 dz = Tz − 云台高;
 *         5) Tilt = atan2(dz, d);
 *         6) 加舵机零点/符号 → 输出目标舵机角, 夹在 0..180。
 * 【依赖占位】AB_LEN_MM、TARGET_X_MM、TARGET_Z_MM、CAR_GIMBAL_DX/DY/Z、
 *           AIM_PAN/TILT_ZERO_DEG、AIM_PAN/TILT_SIGN
 *           均在头部常量组标 "待整定/待标", 场地量了+首次通电试转后回填。
 */
static void aim_geometry(float *pan_deg, float *tilt_deg)
{
    /* 1) 车头朝向(世界系, 弧度): 当前IMU航向减出发航向, wrap 到 (-180,180] 防跨界 */
    float theta_deg = wrap180(imu_get_yaw() - g_lap_yaw0);
    float theta = AIM_DEG2RAD(theta_deg);
    float ct = cosf(theta), st = sinf(theta);

    /* 2) 云台旋转中心的世界坐标 G:
     *    车中心(B_X, B_Y) + 云台安装偏移(dx,dy)按车头旋转到世界系。 */
    float Gx = B_X_MM + CAR_GIMBAL_DX_MM * ct - CAR_GIMBAL_DY_MM * st;
    float Gy = B_Y_MM + CAR_GIMBAL_DX_MM * st + CAR_GIMBAL_DY_MM * ct;

    /* 3) 靶相对云台的水平向量(世界系) */
    float rx = TARGET_X_MM - Gx;
    float ry = TARGET_Y_MM - Gy;

    /* 4) Pan: 世界系"云台→靶"方向 − 车头方向 = 车体系水平指向角
     *    atan2 输出 (-π, π], 减 theta 后再 wrap 一次防溢出 servo 0..180 边界 */
    float pan_world = atan2f(ry, rx);          /* 弧度 */
    float pan_body  = pan_world - theta;
    while (pan_body >  AIM_PI) pan_body -= 2.0f * AIM_PI;
    while (pan_body < -AIM_PI) pan_body += 2.0f * AIM_PI;
    float pan = AIM_PAN_ZERO_DEG + AIM_PAN_SIGN * AIM_RAD2DEG(pan_body);

    /* 5) Tilt: 水平距离 vs 垂直高差; 靶高于云台=仰(正), 低于=俯(负) */
    float d_horiz = sqrtf(rx * rx + ry * ry);
    float dz = TARGET_Z_MM - CAR_GIMBAL_Z_MM;
    /* d_horiz≈0 保护: 车恰在靶正下方(几何上不可能, 但 F2 万一放错)则朝天 */
    float tilt_rad = (d_horiz < 1.0f) ? (AIM_PI * 0.5f) : atan2f(dz, d_horiz);
    float tilt = AIM_TILT_ZERO_DEG + AIM_TILT_SIGN * AIM_RAD2DEG(tilt_rad);

    /* 6) 夹在舵机可动区间。servo_set_angle 内部还会再限, 这里先夹防负数溢出 int 转换 */
    if (pan  <   0.0f) pan  =   0.0f;
    if (pan  > 180.0f) pan  = 180.0f;
    if (tilt <   0.0f) tilt =   0.0f;
    if (tilt > 180.0f) tilt = 180.0f;

    *pan_deg  = pan;
    *tilt_deg = tilt;
}

/**
 * @brief K230 末端精修(辅助; found=0 或无帧则不动几何指向)
 * @param pan_deg  [in/out] 在几何指向基础上微调 PAN
 * @param tilt_deg [in/out] 微调 TILT
 * @return 1=已命中(|dx|,|dy| 在容差内); 0=未命中/无新帧/无视觉(回退几何)
 * @note  无视觉独立性: 拔摄像头或 found=0 时本函数原样返回 0, 不破坏几何指向。
 *        ⚠ AIM_K_PAN/TILT 与 dx/dy 符号待标(test_plan §D, 与视觉 lane 对齐)。
 */
static int aim_refine_k230(float *pan_deg, float *tilt_deg)
{
    int16_t dx, dy;

    /* 取最新一帧瞄准偏差: k230 模块内部已跑协议状态机并判超时, 返回 found 标志。
     * found=0(未找到/无帧/失联) 一律回退几何指向, 绝不阻塞等视觉(拔摄像头也不卡死)。 */
    uint8_t found = k230_get_aim(&dx, &dy);
    if (!found) {
        return 0;   /* found=0: 回退几何, 不微调 */
    }

    /* 像素偏差 -> 角度微调(增益/符号占位待标)。 */
    *pan_deg  += AIM_K_PAN  * (float)dx;
    *tilt_deg += AIM_K_TILT * (float)dy;

    /* 命中判定: dx/dy 都进容差。 */
    return (dx > -AIM_TOL_PX && dx < AIM_TOL_PX &&
            dy > -AIM_TOL_PX && dy < AIM_TOL_PX);
}

/**
 * @brief 执行一次对靶(几何主 + K230 精修)并驱动云台
 * @return 1=命中; 0=未命中
 */
static int aim_step_once(void)
{
    float pan, tilt;
    aim_geometry(&pan, &tilt);                 /* 1) 无视觉几何指向(主) */
    int hit = aim_refine_k230(&pan, &tilt);    /* 2) K230 精修(辅, 自动回退) */
    servo_set_angle(SERVO_PAN,  (int)pan);
    servo_set_angle(SERVO_TILT, (int)tilt);
    return hit;
}

/* ============================================================================
 *  关键点判定 —— 里程 + 灰度特征
 * ==========================================================================*/

/**
 * @brief 关键点顺序判定(2026-07-04 实装, 由"仅整圈"扩展为 B/C/D/A 序列)
 * @return 0=无事件; 1=到B 2=到C 3=到D 4=回A(完赛)
 * @note  双确认判据(航向累计 + 里程累计, 都超过门限才触发), 顺序推进防跳点/误触:
 *        F3(模式3)按 B→C→D→A 逐点走; F1(模式1)只看最后一行(整圈), 中途不发事件。
 *        航向是 IMU 连续累计值直接做差取绝对值, 顺/逆时针跑圈通吃;
 *        里程取右轮 int32 累计(不回绕)。基准在 IDLE→RUN 迁移时快照。
 */
static int keypoint_event(void)
{
    if (g_kp_idx >= 4u) {
        return 0;   /* 序列已走完 */
    }

    float dyaw = imu_get_yaw() - g_lap_yaw0;               /* 出发以来净转角(°, 带符号) */
    if (dyaw < 0.0f) dyaw = -dyaw;                          /* 兼容顺/逆时针 */
    float dist_mm = (float)(enc_get_count(ENC_RIGHT) - g_lap_odo0) * ENC_R_MM_PER_CNT;
    if (dist_mm < 0.0f) dist_mm = -dist_mm;

    /* 模式3 逐点推进; 其它模式只关心"整圈"那一行(下标3) */
    uint8_t idx = (g_run_mode == 3u) ? g_kp_idx : 3u;
    const kp_gate_t *gate = &KP_GATES[idx];

    if (dyaw >= gate->yaw_min_deg && dist_mm >= gate->dist_min_mm) {
        g_kp_idx = (uint8_t)(idx + 1u);
        return (int)idx + 1;   /* 1=B 2=C 3=D 4=A */
    }
    return 0;
}

/* ============================================================================
 *  状态机
 * ==========================================================================*/

void app_fsm_step(void)
{
    /* 入态检测: 本拍状态 != 上一拍 = 刚进入该态, 用于到点声光等"只做一次"的动作。 */
    uint8_t entered = (g_state != g_prev_state);
    g_prev_state = g_state;

    switch (g_state) {

    case APP_ST_IDLE:
        /* 待机: 执行器保持停; RUN 灯每 2s 闪"模式号"次(F4 按键选模式的人机反馈)。 */
        motor_stop_all();
        {
            static uint16_t s_blink_ms = 0;
            s_blink_ms = (uint16_t)((s_blink_ms + APP_FSM_DT_MS) % 2000u);
            uint16_t slot = s_blink_ms / 200u;   /* 把 2s 切成 10 格, 每格 200ms */
            /* 前 mode*2 格里"亮灭交替" -> 闪 mode 次, 其余时间灭 */
            led_run_set((slot < (uint16_t)(g_run_mode * 2u)) && ((slot & 1u) == 0u));
        }
        if (g_start_req) {
            g_start_req = 0;
            /* 启动前复位: 清 PID 历史 + 内环累加 + 盲走里程 + 关键点进度, 防上次残留。 */
            pid_reset(&g_pid_track);
            pid_reset(&g_pid_vel_l);
            pid_reset(&g_pid_vel_r);
            g_vel_out_l = g_vel_out_r = 0.0f;
            g_vel_tgt_l = g_vel_tgt_r = 0.0f;
            g_lost_dist = 0;
            g_kp_idx = 0;
            g_lap_count = 0;
            g_run_beep_ms = 0;
            /* 关键点判定基准快照: 以"此刻"的航向/里程为零点(见 keypoint_event) */
            g_lap_yaw0 = imu_get_yaw();
            g_lap_odo0 = enc_get_count(ENC_RIGHT);
            g_yaw_line_avg = imu_get_yaw();   /* 线向估计从出发航向起步 */
            /* TODO: 上电自检(电源轨/编码器空转/IMU WHO_AM_I/K230 握手), 失败进 ESTOP。 */
            led_run_set(1);
            if (g_run_mode == 2u) {
                /* F2: 静止定点瞄准, 不巡迹, 直接进 AIM; 结束进 STOP */
                g_aim_timer = 0;
                g_aim_return_run = 0;
                g_state = APP_ST_AIM;
            } else {
                g_state = APP_ST_RUN;
            }
        }
        break;

    case APP_ST_RUN:
        /* 行驶: 跑线由 track_loop_step(外环) + vel_loop_step(内环) 各自任务完成,
         * 本函数管"到点事件"分流与过点声光。 */
        if (g_run_beep_ms > 0) {          /* 过点声光: 非阻塞脉冲, 到时自动关 */
            g_run_beep_ms -= APP_FSM_DT_MS;
            if (g_run_beep_ms <= 0) {
                beep_off();
            }
        }
        {
            int kp = keypoint_event();
            if (kp == 4) {
                if (g_run_mode == 4u && (g_lap_count + 1u) < APP_LAPS_MODE4) {
                    /* 发挥3 四圈连跑: 过 A 计圈+声光, 重置基准继续跑 */
                    g_lap_count++;
                    beep_on();
                    g_run_beep_ms = KEYPOINT_BEEP_MS;
                    g_lap_yaw0 = imu_get_yaw();
                    g_lap_odo0 = enc_get_count(ENC_RIGHT);
                    g_kp_idx = 0;
                } else {
                    /* 回到 A: 完赛停车(F1/F3/第4圈共用; STOP 入态自带声光) */
                    g_state = APP_ST_STOP;
                }
            } else if (kp == 1 && g_run_mode == 3u) {
                /* F3 到 B: 声光 + 停车对靶作业, 结束后回 RUN 续跑 */
                beep_on();
                g_run_beep_ms = KEYPOINT_BEEP_MS;
                g_aim_timer = 0;
                g_aim_return_run = 1;
                g_state = APP_ST_AIM;
            } else if (kp > 0) {
                /* C/D: 过点声光, 不停车(题目: 每经过关键点须声光提示) */
                beep_on();
                g_run_beep_ms = KEYPOINT_BEEP_MS;
            }
        }
        break;

    case APP_ST_AIM:
        /* 瞄准: 整车停, 云台几何指向 + K230 精修。命中或超时离开。 */
        motor_stop_all();
        {
            int hit = aim_step_once();
            g_aim_timer += APP_FSM_DT_MS;
            if (hit || g_aim_timer >= AIM_TIMEOUT_MS) {
                if (g_aim_return_run) {
                    /* F3: 对靶完成, 继续巡迹。重置内环状态, 从静止平滑重加速。 */
                    g_aim_return_run = 0;
                    pid_reset(&g_pid_vel_l);
                    pid_reset(&g_pid_vel_r);
                    g_vel_out_l = g_vel_out_r = 0.0f;
                    led_run_set(1);
                    g_state = APP_ST_RUN;
                } else {
                    g_state = APP_ST_STOP;   /* F2: 定点作业完成即收 */
                }
            }
        }
        break;

    case APP_ST_STOP:
        /* 正常停车: 电机停 + 到点声光(一次性, 非阻塞)。 */
        motor_stop_all();
        led_run_set(0);
        if (entered) {                 /* 到点只在"刚入 STOP"时响一次, 不每拍响 */
            beep_on();
            g_stop_beep_ms = KEYPOINT_BEEP_MS;
        }
        if (g_stop_beep_ms > 0) {      /* 非阻塞关断: 到时自动 beep_off, 不在此忙等 */
            g_stop_beep_ms -= APP_FSM_DT_MS;
            if (g_stop_beep_ms <= 0) {
                beep_off();
            }
        }
        /* TODO: 多圈/多点任务在此判是否回 RUN 续跑; 占位停在 STOP。 */
        break;

    case APP_ST_ESTOP:
        /* 急停安全态: 持续保持电机停 + 舵机轨断电, 直到外部清故障。 */
        motor_stop_all();
        servo_rail_enable(0);
        led_err_set(1);
        /* TODO: 清故障条件满足后 -> led_err_set(0); g_state = APP_ST_IDLE。 */
        break;

    default:
        g_state = APP_ST_ESTOP;   /* 异常态兜底进急停 */
        break;
    }
}

/* ============================================================================
 *  安全 / 外部事件入口
 * ==========================================================================*/

void app_estop(void)
{
    /* 最小动作, 可在中断里调: 切电机 + 断舵机轨 + 置态。耗时操作留给 FSM。 */
    motor_stop_all();        /* 两轮 0 占空 + TB6612 整片待机 */
    servo_rail_enable(0);     /* 断舵机轨(独立可控, 限制⑥) */
    g_state = APP_ST_ESTOP;
}

void app_start_request(void)
{
    g_start_req = 1;   /* 只置标志, 迁移交 app_fsm_step(避免中断里跑业务) */
}

app_state_e app_get_state(void)
{
    return g_state;
}

void app_mode_cycle(void)
{
    /* 只允许待机态换模式(运动中换模式没有安全语义); 1→2→...→APP_MODE_MAX→1 循环 */
    if (g_state == APP_ST_IDLE) {
        g_run_mode = (uint8_t)(g_run_mode % APP_MODE_MAX) + 1u;
    }
}

void app_mode_set(uint8_t mode)
{
    /* 串口直设模式(F4 "串口设定运行模式"); 同样只在待机态生效 */
    if (g_state == APP_ST_IDLE && mode >= 1u && mode <= APP_MODE_MAX) {
        g_run_mode = mode;
    }
}

uint8_t app_get_mode(void)
{
    return g_run_mode;
}

void app_estop_ack(void)
{
    /* 急停软清障: ESTOP 态下 2 秒内连按 3 次 START -> 回待机(红灯灭)。
     * 用"连按三次"作确认动作, 防误触; 清障后执行器仍全停, 需重新 START 才动。 */
    static uint8_t  s_cnt = 0;
    static uint32_t s_first_ms = 0;

    if (g_state != APP_ST_ESTOP) {
        s_cnt = 0;
        return;
    }
    if (s_cnt == 0u || (g_tick_ms - s_first_ms) > 2000u) {
        s_cnt = 1u;                 /* 窗口过期/首次: 重新开窗计数 */
        s_first_ms = g_tick_ms;
        return;
    }
    s_cnt++;
    if (s_cnt >= 3u) {
        s_cnt = 0;
        led_err_set(0);
        g_state = APP_ST_IDLE;      /* 清障回待机; IDLE 态会保持电机停 */
    }
}

void app_tune_step(char which, int dir)
{
    /* 串口在线调参(现场整定免重编译): 'p'=循迹KP ±0.05, 'd'=KD ±0.25, 'v'=基速 ±25mm/s。
     * KP/KD 直接热改 g_pid_track 的增益字段, 下一拍外环立即生效。 */
    float d = (dir >= 0) ? 1.0f : -1.0f;
    if (which == 'p') {
        g_track_kp += 0.05f * d;
        if (g_track_kp < 0.0f) g_track_kp = 0.0f;
        g_pid_track.kp = g_track_kp;
    } else if (which == 'd') {
        g_track_kd += 0.25f * d;
        if (g_track_kd < 0.0f) g_track_kd = 0.0f;
        g_pid_track.kd = g_track_kd;
    } else if (which == 'v') {
        g_base_speed += (dir >= 0) ? 25 : -25;
        if (g_base_speed < 100) g_base_speed = 100;   /* 下限: 太慢测速量化差 */
        if (g_base_speed > 600) g_base_speed = 600;   /* 上限: 未整定前安全帽 */
    }
}

void app_tune_get(float *kp, float *kd, int *base)
{
    if (kp)   *kp   = g_track_kp;
    if (kd)   *kd   = g_track_kd;
    if (base) *base = g_base_speed;
}

void app_get_vel_debug(float *tgt_l, float *meas_l, float *tgt_r, float *meas_r)
{
    /* 只读快照, 给 VOFA 调试波形用(main 的 task_vofa 调)。
     * 读的是内环真正在用的量: 目标速度(track/FSM 写入)与低通后的实测速度, 单位都是 mm/s。
     * 注: 这里不调 enc_get_delta/imu_get_yaw 等有副作用的函数, 不会干扰控制环。 */
    if (tgt_l)  *tgt_l  = g_vel_tgt_l;
    if (meas_l) *meas_l = g_meas_filt[0];
    if (tgt_r)  *tgt_r  = g_vel_tgt_r;
    if (meas_r) *meas_r = g_meas_filt[1];
}

void app_get_vel_out(float *out_l, float *out_r)
{
    /* 只读快照: 两轮当前下发占空(内环累加输出, -1000..1000)。
     * 整定时与实测速度对照看: 占空稳/爬/顶格 分别对应不同的诊断结论。 */
    if (out_l) *out_l = g_vel_out_l;
    if (out_r) *out_r = g_vel_out_r;
}

/* ============================================================================
 *  初始化
 * ==========================================================================*/

void app_init(void)
{
    /* ⚠ 占位增益, 必须真车整定(test_plan §B/§C)。
     * 外环位置式: 偏差->转向量; 内环增量式: 速度->占空增量。 */
    pid_init(&g_pid_track, g_track_kp, 0.0f, g_track_kd,   /* 用运行时副本(串口可调) */
             -TRACK_OUT_LIM, TRACK_OUT_LIM, 0.0f);          /* 外环(位置式) */
    pid_init(&g_pid_vel_l, VEL_KP, VEL_KI, VEL_KD,
             VEL_OUT_MIN, VEL_OUT_MAX, VEL_INTEG_MAX);       /* 内环左(增量式) */
    pid_init(&g_pid_vel_r, VEL_KP, VEL_KI, VEL_KD,
             VEL_OUT_MIN, VEL_OUT_MAX, VEL_INTEG_MAX);       /* 内环右(增量式) */

    g_vel_tgt_l = g_vel_tgt_r = 0.0f;
    g_vel_out_l = g_vel_out_r = 0.0f;
    g_start_req = 0;
    g_aim_timer = 0;
    g_lost_dist = 0;
    g_kp_idx = 0;
    g_run_beep_ms = 0;
    g_aim_return_run = 0;
    /* g_run_mode 不复位: 保持用户选择(app_init 只在上电跑一次, 默认 1=F1) */
    g_meas_filt[0] = g_meas_filt[1] = 0.0f;
    g_blind_active = 0;
    g_stop_beep_ms = 0;
    g_prev_state = APP_ST_IDLE;
    g_state = APP_ST_IDLE;
}

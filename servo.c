/* 云台舵机驱动 —— firmware-scaffold driver 层（实现）
 *
 * 硬件接线（与 empty.syscfg / SysConfig 生成宏一一对应）：
 *   PWM（单 TIMA1 两路边沿对齐, 50Hz/20ms）  SysConfig 实例名 = PWM_SERVO
 *     PAN  舵机信号 = PA17 = TIMA1_CCP0  -> GPIO_PWM_SERVO_C0_* / GPIO_PWM_SERVO_C0_IDX
 *     TILT 舵机信号 = PA16 = TIMA1_CCP1  -> GPIO_PWM_SERVO_C1_* / GPIO_PWM_SERVO_C1_IDX
 *   电源轨开关 GPIO（推挽输出, 初值 Low=断电）：
 *     SERVO_EN = PB23（GPIO_IOB 组）-> GPIO_IOB_PORT / GPIO_IOB_SERVO_EN_PIN
 *
 * PWM 极性（边沿对齐, 计数器 LOAD->0 下行；LOAD 处拉高、下行匹配 CCR 处拉低，同 motor.c）：
 *   高电平计数 = LOAD - CCR  =>  脉宽越宽需要的 CCR 越小。CCR 必须落在 [0, LOAD]。
 *
 * 50Hz 由 SysConfig 对 TIMA1 设分频得到（周期 20ms 对应 LOAD=getLoadValue()）。
 * 这里不硬编码分频/LOAD，运行时用 DL_TimerA_getLoadValue() 取真实 LOAD，
 * 再按 “脉宽/周期” 比例换算 CCR —— 改了 SysConfig 分频也不用动本文件。
 *
 * 依赖：ti_msp_dl_config.h（实例/引脚宏）。函数签名已对 dl_timera.h / dl_gpio.h 核实：
 *   DL_TimerA_getLoadValue / DL_TimerA_setCaptureCompareValue / DL_TimerA_startCounter,
 *   DL_GPIO_setPins / DL_GPIO_clearPins。
 */
#include "ti_msp_dl_config.h"
#include "servo.h"

/* ===== 调参/标定占位（一律 #define，不填真值，待标定） ===== */

/* PWM 周期：50Hz 舵机标准 20ms。须与 SysConfig 里 PWM_SERVO 的实际定时周期一致，
 * 否则脉宽换算会整体偏。若改 SysConfig 周期，这里同步改。 */
#define SERVO_PERIOD_US     20000u   /* 待整定: PWM 周期(us), 须与 SysConfig PWM_SERVO 周期一致 */

/* ===== PAN 舵机（360° 连续旋转，速度控制） =====
 * 脉宽 1000-2000µs：1500µs=停转，1500-2000µs=逆时针（正转），1000-1500µs=顺时针（反转）。
 * 死区 1400-1600µs（速度≈0）。本驱动只用"停转"语义（发 90°→1500µs），
 * 不做连续旋转调速——F2 瞄准时 PAN 方向由物理安装决定。 */
#define SERVO_PAN_PULSE_MIN_US  1000u   /* PAN 最小脉宽(us) = 全速顺时针 */
#define SERVO_PAN_PULSE_MAX_US  2000u   /* PAN 最大脉宽(us) = 全速逆时针 */
#define SERVO_PAN_STOP_US       1500u   /* PAN 停转脉宽(us) = 速度为0 */
#define SERVO_PAN_DEG_MIN       0       /* PAN 角度映射范围（仅用于 API 兼容，实际是速度控制） */
#define SERVO_PAN_DEG_MAX       180     /* PAN 90°=停转，0/180=全速正/反转 */

/* ===== TILT 舵机（180° 位置舵机） =====
 * 脉宽 500-2500µs 对应 0-180°。待标定：装好后扫两端实测机械极限再回填。 */
#define SERVO_TILT_PULSE_MIN_US  500u   /* 待整定: 对应 0° 的脉宽(us) */
#define SERVO_TILT_PULSE_MAX_US  2500u  /* 待整定: 对应 180° 的脉宽(us) */
#define SERVO_TILT_DEG_MIN       0      /* 待整定: 允许最小角度(度) */
#define SERVO_TILT_DEG_MAX       180    /* 待整定: 允许最大角度(度) */

/* 初始化安全中位角（占位）。两路上电先给中位，避免后续上电源轨时大幅扫动。 */
#define SERVO_INIT_DEG      90       /* 待整定: 初始中位角(度), 占位 90 */

/* 把“目标脉宽(us)”换算成要写入 CCR 的计数值。
 *
 * 反比原理（同 motor.c）：重装(LOAD)处拉高、下行匹配 CCR 处拉低 => 高电平计数 = LOAD - CCR。
 *   脉宽占空比 = pulse_us / SERVO_PERIOD_US；高电平计数 = LOAD * pulse_us / period。
 *   于是 CCR = LOAD - LOAD * pulse_us / period。脉宽越宽 -> CCR 越小。
 *
 * 用运行时 LOAD（=getLoadValue()=period-1）做基准，CCR 落在 [0, LOAD] 内才有效；
 * 因 pulse_us < period，pulse 项 <= LOAD，CCR>=0，且 pulse>0 时 CCR<LOAD，无边界翻车风险。
 * 这里先乘后除（uint32_t 量级：LOAD 约数千、pulse 数千，乘积不溢出 32 位）。 */
static uint32_t pulse_to_ccr(uint32_t pulse_us)
{
    uint32_t load = DL_TimerA_getLoadValue(PWM_SERVO_INST);   /* = period-1, 由 SysConfig 分频决定 */
    uint32_t high = (load * pulse_us) / SERVO_PERIOD_US;      /* 该脉宽对应的高电平计数 */
    if (high > load) high = load;                             /* 双保险: 夹在 LOAD 内 */
    return load - high;                                       /* 反比: 高电平越多 CCR 越小 */
}

/* 把角度按通道映射到脉宽(us)。
 * PAN(360°连续): 0-180 线性映射到 1000-2000µs, 90°=1500µs=停转。
 * TILT(180°位置): 0-180 线性映射到 500-2500µs。 */
static uint32_t deg_to_pulse_us(int ch, int deg)
{
    if (ch == SERVO_PAN) {
        /* PAN: 0°=1000µs(全速顺时针), 90°=1500µs(停转), 180°=2000µs(全速逆时针) */
        uint32_t span_us = SERVO_PAN_PULSE_MAX_US - SERVO_PAN_PULSE_MIN_US; /* 1000µs */
        uint32_t off = (uint32_t)deg * span_us / (uint32_t)(SERVO_PAN_DEG_MAX - SERVO_PAN_DEG_MIN);
        return SERVO_PAN_PULSE_MIN_US + off;
    } else {
        /* TILT: 0°=500µs, 180°=2500µs */
        uint32_t span_us = SERVO_TILT_PULSE_MAX_US - SERVO_TILT_PULSE_MIN_US; /* 2000µs */
        uint32_t off = (uint32_t)(deg - SERVO_TILT_DEG_MIN) * span_us
                       / (uint32_t)(SERVO_TILT_DEG_MAX - SERVO_TILT_DEG_MIN);
        return SERVO_TILT_PULSE_MIN_US + off;
    }
}

void servo_init(void)
{
    /* 两路先给安全中位(占位 90°)，使上电后即便随后通电源轨也从中位起步，不大幅扫动。
     * 此时电源轨默认关(SERVO_EN=Low)，舵机不动；写 CCR 只是把待输出脉宽备好。 */
    servo_set_angle(SERVO_PAN,  SERVO_INIT_DEG);
    servo_set_angle(SERVO_TILT, SERVO_INIT_DEG);

    /* 启动 TIMA1 计数, 开始出 50Hz PWM。SysConfig 生成的 PWM init 不会自动启动计数。 */
    DL_TimerA_startCounter(PWM_SERVO_INST);

    /* 不在此给电源轨上电: 题目限制⑥要求默认不通电, 上电由 servo_rail_enable 显式控制。 */
}

void servo_set_angle(int ch, int deg)
{
    if (ch != SERVO_PAN && ch != SERVO_TILT) return;   /* 非法通道直接忽略, 防越界写 */

    /* 按通道限位 */
    if (ch == SERVO_PAN) {
        if (deg < SERVO_PAN_DEG_MIN) deg = SERVO_PAN_DEG_MIN;
        if (deg > SERVO_PAN_DEG_MAX) deg = SERVO_PAN_DEG_MAX;
    } else {
        if (deg < SERVO_TILT_DEG_MIN) deg = SERVO_TILT_DEG_MIN;
        if (deg > SERVO_TILT_DEG_MAX) deg = SERVO_TILT_DEG_MAX;
    }

    uint32_t pulse_us = deg_to_pulse_us(ch, deg);      /* 角度 -> 脉宽(us), 按通道映射 */
    uint32_t ccr      = pulse_to_ccr(pulse_us);        /* 脉宽 -> CCR(与占空成反比) */

    /* ch=PAN -> CCP0 通道, ch=TILT -> CCP1 通道（写入对应 CC 寄存器索引） */
    if (ch == SERVO_PAN) {
        DL_TimerA_setCaptureCompareValue(PWM_SERVO_INST, ccr, GPIO_PWM_SERVO_C0_IDX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_SERVO_INST, ccr, GPIO_PWM_SERVO_C1_IDX);
    }
}

void servo_rail_enable(int on)
{
    if (on) {
        DL_GPIO_setPins(GPIO_IOB_PORT, GPIO_IOB_SERVO_EN_PIN);   /* 拉高: 给舵机电源轨通电 */
    } else {
        DL_GPIO_clearPins(GPIO_IOB_PORT, GPIO_IOB_SERVO_EN_PIN); /* 拉低: 断开舵机电源轨(默认态) */
    }
}

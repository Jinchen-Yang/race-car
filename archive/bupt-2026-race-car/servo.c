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

/* ===== 调参/标定参数 ===== */

/* PWM 周期：50Hz 舵机标准 20ms。须与 SysConfig 里 PWM_SERVO 的实际定时周期一致，
 * 否则脉宽换算会整体偏。若改 SysConfig 周期，这里同步改。 */
#define SERVO_PERIOD_US     20000u   /* PWM 周期(us), 与 empty.syscfg PWM_SERVO timerCount=40000 对应 */

/* 电气脉宽先收窄到 0.6~2.4ms，避开多数模拟舵机端点死区；确认型号/机构后再放宽。 */
#define SERVO_PAN_PULSE_MIN_US    600u
#define SERVO_PAN_PULSE_MAX_US   2400u
#define SERVO_TILT_PULSE_MIN_US   600u
#define SERVO_TILT_PULSE_MAX_US  2400u

/* 机械角度限位。PAN 目标解算约 133°，安全范围 10..170 足够；TILT 先给 40..140 防抬臂顶结构。
 * 现场扫角后，把真正不顶机械的范围回填到这里。 */
#define SERVO_PAN_DEG_MIN       10
#define SERVO_PAN_DEG_MAX       170
#define SERVO_PAN_INIT_DEG      90
#define SERVO_PAN_REVERSE        0

#define SERVO_TILT_DEG_MIN      40
#define SERVO_TILT_DEG_MAX     140
#define SERVO_TILT_INIT_DEG     90
#define SERVO_TILT_REVERSE       0

typedef struct {
    int deg_min;
    int deg_max;
    int init_deg;
    uint32_t pulse_min_us;
    uint32_t pulse_max_us;
    uint8_t reverse;
} servo_cal_t;

static const servo_cal_t s_cal[2] = {
    { SERVO_PAN_DEG_MIN,  SERVO_PAN_DEG_MAX,  SERVO_PAN_INIT_DEG,
      SERVO_PAN_PULSE_MIN_US,  SERVO_PAN_PULSE_MAX_US,  SERVO_PAN_REVERSE  },
    { SERVO_TILT_DEG_MIN, SERVO_TILT_DEG_MAX, SERVO_TILT_INIT_DEG,
      SERVO_TILT_PULSE_MIN_US, SERVO_TILT_PULSE_MAX_US, SERVO_TILT_REVERSE },
};

static int      s_angle_deg[2] = { SERVO_PAN_INIT_DEG, SERVO_TILT_INIT_DEG };
static uint32_t s_pulse_us[2]  = { 1500u, 1500u };
static uint8_t  s_rail_on      = 0;

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

static int clamp_angle(int ch, int deg)
{
    if (deg < s_cal[ch].deg_min) deg = s_cal[ch].deg_min;
    if (deg > s_cal[ch].deg_max) deg = s_cal[ch].deg_max;
    return deg;
}

/* 把角度(已限位)按通道标定线性映射到脉宽(us)。 */
static uint32_t deg_to_pulse_us(int ch, int deg)
{
    int span_deg = s_cal[ch].deg_max - s_cal[ch].deg_min;
    if (span_deg <= 0) return s_cal[ch].pulse_min_us;

    int phy_deg = s_cal[ch].reverse ? (s_cal[ch].deg_max + s_cal[ch].deg_min - deg) : deg;
    uint32_t span_us = s_cal[ch].pulse_max_us - s_cal[ch].pulse_min_us;
    uint32_t off = (uint32_t)(phy_deg - s_cal[ch].deg_min) * span_us / (uint32_t)span_deg;
    return s_cal[ch].pulse_min_us + off;
}

void servo_init(void)
{
    /* 两路先给安全中位(占位 90°)，使上电后即便随后通电源轨也从中位起步，不大幅扫动。
     * 此时电源轨默认关(SERVO_EN=Low)，舵机不动；写 CCR 只是把待输出脉宽备好。 */
    servo_set_angle(SERVO_PAN,  s_cal[SERVO_PAN].init_deg);
    servo_set_angle(SERVO_TILT, s_cal[SERVO_TILT].init_deg);

    /* 启动 TIMA1 计数, 开始出 50Hz PWM。SysConfig 生成的 PWM init 不会自动启动计数。 */
    DL_TimerA_startCounter(PWM_SERVO_INST);

    /* 显式保持电源轨断电: 防以后 SysConfig 初值被误改成高电平。 */
    servo_rail_enable(0);
}

void servo_set_angle(int ch, int deg)
{
    if (ch != SERVO_PAN && ch != SERVO_TILT) return;   /* 非法通道直接忽略, 防越界写 */

    deg = clamp_angle(ch, deg);                        /* 按通道软件限位，防舵机/结构互顶堵转 */
    uint32_t pulse_us = deg_to_pulse_us(ch, deg);      /* 角度 -> 脉宽(us) */
    uint32_t ccr      = pulse_to_ccr(pulse_us);        /* 脉宽 -> CCR(与占空成反比) */
    s_angle_deg[ch] = deg;
    s_pulse_us[ch]  = pulse_us;

    /* ch=PAN -> CCP0 通道, ch=TILT -> CCP1 通道（写入对应 CC 寄存器索引） */
    if (ch == SERVO_PAN) {
        DL_TimerA_setCaptureCompareValue(PWM_SERVO_INST, ccr, GPIO_PWM_SERVO_C0_IDX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_SERVO_INST, ccr, GPIO_PWM_SERVO_C1_IDX);
    }
}

void servo_set_pulse_us(int ch, uint32_t pulse_us)
{
    if (ch != SERVO_PAN && ch != SERVO_TILT) return;

    /* 直接按脉宽驱动(绕过角度限位/映射), 供 360° 连续舵机调速/停转微调。
     * 安全夹在 0.6~2.4ms, 防越界脉宽把舵机顶到电气死区。s_angle_deg 对连续舵机无意义, 不更新。 */
    if (pulse_us < 600u)  pulse_us = 600u;
    if (pulse_us > 2400u) pulse_us = 2400u;
    uint32_t ccr = pulse_to_ccr(pulse_us);
    s_pulse_us[ch] = pulse_us;

    if (ch == SERVO_PAN) {
        DL_TimerA_setCaptureCompareValue(PWM_SERVO_INST, ccr, GPIO_PWM_SERVO_C0_IDX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_SERVO_INST, ccr, GPIO_PWM_SERVO_C1_IDX);
    }
}

void servo_rail_enable(int on)
{
    if (on) {
        s_rail_on = 1u;
        DL_GPIO_setPins(GPIO_IOB_PORT, GPIO_IOB_SERVO_EN_PIN);   /* 拉高: 给舵机电源轨通电 */
    } else {
        s_rail_on = 0u;
        DL_GPIO_clearPins(GPIO_IOB_PORT, GPIO_IOB_SERVO_EN_PIN); /* 拉低: 断开舵机电源轨(默认态) */
    }
}

int servo_get_angle(int ch)
{
    if (ch != SERVO_PAN && ch != SERVO_TILT) return 0;
    return s_angle_deg[ch];
}

uint32_t servo_get_pulse_us(int ch)
{
    if (ch != SERVO_PAN && ch != SERVO_TILT) return 0u;
    return s_pulse_us[ch];
}

int servo_rail_is_enabled(void)
{
    return (int)s_rail_on;
}

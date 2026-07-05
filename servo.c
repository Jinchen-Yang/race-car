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

/* 角度->脉宽端点（占位，对应 0° 与 180°）。MG996R 名义 0.5~2.5ms，
 * 但各舵机端点有差异，装好后用 servo_set_angle 扫到两端实测机械极限再回填。 */
#define SERVO_PULSE_MIN_US  500u     /* 待整定: 对应 SERVO_DEG_MIN 的脉宽(us), 占位 0.5ms */
#define SERVO_PULSE_MAX_US  2500u    /* 待整定: 对应 SERVO_DEG_MAX 的脉宽(us), 占位 2.5ms */

/* 角度限位（占位）。先给满量程 0~180，标定后按云台机械行程收窄，防舵机/结构互顶堵转。 */
#define SERVO_DEG_MIN       0        /* 待整定: 允许最小角度(度), 占位 0 */
#define SERVO_DEG_MAX       180      /* 待整定: 允许最大角度(度), 占位 180 */

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

/* 把角度(已限位)线性映射到脉宽(us)：deg=MIN->PULSE_MIN, deg=MAX->PULSE_MAX。 */
static uint32_t deg_to_pulse_us(int deg)
{
    /* 线性插值: pulse = MIN + (deg-DEG_MIN)*(MAX-MIN)/(DEG_MAX-DEG_MIN) */
    int span_deg   = SERVO_DEG_MAX - SERVO_DEG_MIN;          /* 角度跨度, 占位 180 */
    uint32_t span_us = SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US; /* 脉宽跨度, 占位 2000us */
    uint32_t off   = (uint32_t)(deg - SERVO_DEG_MIN) * span_us / (uint32_t)span_deg;
    return SERVO_PULSE_MIN_US + off;
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

    if (deg < SERVO_DEG_MIN) deg = SERVO_DEG_MIN;      /* 软件限位: 夹紧到允许角度范围 */
    if (deg > SERVO_DEG_MAX) deg = SERVO_DEG_MAX;      /* 防舵机/结构互顶堵转 */

    uint32_t pulse_us = deg_to_pulse_us(deg);          /* 角度 -> 脉宽(us) */
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

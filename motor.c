/* TB6612FNG 双 H 桥电机驱动 —— firmware-scaffold driver 层
 *
 * 硬件接线（与 empty.syscfg / SysConfig 生成宏一一对应）：
 *   PWM（单 TIMA0 两路边沿对齐, 20kHz）  SysConfig 实例名 = PWM_MOTOR
 *     左轮 PWMA = PA8 = TIMA0_CCP0  -> GPIO_PWM_MOTOR_C0_* / GPIO_PWM_MOTOR_C0_IDX
 *     右轮 PWMB = PB9 = TIMA0_CCP1  -> GPIO_PWM_MOTOR_C1_* / GPIO_PWM_MOTOR_C1_IDX
 *   方向 GPIO（推挽输出, 初值 Low）。因 SysConfig 一个组只能单端口, 故拆两组：
 *     GPIOA 组 = MOTOR_A：AIN1=PA12, AIN2=PA13
 *     GPIOB 组 = MOTOR_B：BIN1=PB6,  BIN2=PB7,  STBY=PB0
 *   【硬件】STBY=PB0 必须外接 10k 下拉到 GND(见 motor.h 说明), 兜住复位/烧录上电窗口。
 *
 * TB6612 真值表：IN1=1/IN2=0 正转；IN1=0/IN2=1 反转；IN1=IN2=0 滑行停；STBY=0 整片待机。
 *
 * PWM 极性（边沿对齐, 计数器 LOAD->0 下行；LOAD 处拉高(LACT), 下行匹配 CCR 处拉低(CDACT)）：
 *   高电平计数 = LOAD - CCR  =>  占空% = (LOAD - CCR)/period，CCR 越大占空越小。
 *   ⚠ CCR 必须落在 [0, LOAD] 内：停(0%)=CCR=LOAD(恒低)，满(100%)=CCR=0(恒高)。详见 duty_to_ccr()。
 *   (依据 dl_timer.c DL_Timer_initTwoCCPWMMode L444-451: EDGE_ALIGN 下 LOAD=period-1,
 *    CC 配 LACT_CCP_HIGH|CDACT_CCP_LOW —— LOAD 处拉高、下行匹配 CCR 处拉低。)
 */
#include "ti_msp_dl_config.h"
#include "motor.h"

/* 软件占空限幅：电机 MG310p20 额定 7.4V，电池 12V。封顶 60% 防长时间超压伤电机。
 * 在 motor_set 里对“幅值”封顶（不影响方向）。 */
#define MOTOR_MAX_DUTY   600       /* 占空上限, 对应 duty 标度 0..1000 的 60% */
#define MOTOR_DUTY_SCALE 1000      /* duty 满标度 = 1000 */

/* 把 duty(0..1000, 已取绝对值并限幅) 换算成要写入 CCR 的计数值。
 *
 * 反比原理：PWM 在重装(LOAD)处由 LACT 拉高、下行匹配到 CCR 处由 CDACT 拉低，
 *   高电平计数 = LOAD - CCR。占空越大需要的 CCR 越小：
 *     duty=0    -> CCR=LOAD  高电平=0    -> 0%    (恒低, 电机停)
 *     duty=1000 -> CCR=0     高电平=LOAD -> ~100% (恒高, 满速)
 *   线性换算：CCR = LOAD - duty*LOAD/scale。
 *
 * ⚠ 致命边界(2026-06-28 实测踩坑改正)：基准必须取 LOAD(=getLoadValue()=1599)，
 *   不能取 period(=LOAD+1=1600)。CCR 必须落在 [0, LOAD] 内才有效：
 *   若以 period 为基准, duty=0 -> CCR=1600 > LOAD=1599, 计数器(1599..0)永远匹配不到,
 *   CDACT 不触发, 输出停在 LACT 的“高”上 => 恒高 100% => “停”变满速(致命!)。
 *   以 LOAD 为基准: duty=0 -> CCR=LOAD -> 重装即匹配拉低 => 恒低 = 真正的 0%。 */
__STATIC_INLINE uint32_t duty_to_ccr(uint32_t duty)
{
    uint32_t load = DL_TimerA_getLoadValue(PWM_MOTOR_INST);   /* = period-1 = 1599 */
    return load - (duty * load) / MOTOR_DUTY_SCALE;
}

/* 写某通道的占空（仅管 PWM 幅值, 方向由调用方先设好）。
 * ch=左 -> CCP0 通道, ch=右 -> CCP1 通道。 */
static void motor_pwm_write(int ch, uint32_t duty)
{
    if (ch == MOTOR_LEFT) {
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, duty_to_ccr(duty),
                                         GPIO_PWM_MOTOR_C0_IDX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, duty_to_ccr(duty),
                                         GPIO_PWM_MOTOR_C1_IDX);
    }
}

/* 设某电机方向脚（forward!=0 正转：IN1=1/IN2=0；否则反转：IN1=0/IN2=1）。
 * 左轮方向脚在 GPIOA 组(AIN1/AIN2)，右轮在 GPIOB 组(BIN1/BIN2)，端口分别取各组 _PORT。 */
static void motor_set_dir(int ch, int forward)
{
    if (ch == MOTOR_LEFT) {
        if (forward) {   /* 正转: AIN1=1, AIN2=0 */
            DL_GPIO_setPins(MOTOR_A_PORT, MOTOR_A_AIN1_PIN);
            DL_GPIO_clearPins(MOTOR_A_PORT, MOTOR_A_AIN2_PIN);
        } else {         /* 反转: AIN1=0, AIN2=1 */
            DL_GPIO_clearPins(MOTOR_A_PORT, MOTOR_A_AIN1_PIN);
            DL_GPIO_setPins(MOTOR_A_PORT, MOTOR_A_AIN2_PIN);
        }
    } else {
        if (forward) {   /* 正转: BIN1=1, BIN2=0 */
            DL_GPIO_setPins(MOTOR_B_PORT, MOTOR_B_BIN1_PIN);
            DL_GPIO_clearPins(MOTOR_B_PORT, MOTOR_B_BIN2_PIN);
        } else {         /* 反转: BIN1=0, BIN2=1 */
            DL_GPIO_clearPins(MOTOR_B_PORT, MOTOR_B_BIN1_PIN);
            DL_GPIO_setPins(MOTOR_B_PORT, MOTOR_B_BIN2_PIN);
        }
    }
}

void motor_init(void)
{
    /* 必须在 SYSCFG_DL_init() 之后调用：方向脚/PWM 引脚的 IOMUX 与 TIMA0 配置已就绪。
     * SysConfig 生成的 PWM init 不启动计数, 这里补 startCounter, 否则无任何波形(易误判)。 */

    /* 先确保两轮 0 占空(CCR=full=恒低), 再使能, 保证上电瞬间不乱转 */
    motor_pwm_write(MOTOR_LEFT, 0);
    motor_pwm_write(MOTOR_RIGHT, 0);

    /* 方向脚给个确定初值(正转方向, 但占空为 0 所以电机仍不转), 避免悬空 */
    motor_set_dir(MOTOR_LEFT, 1);
    motor_set_dir(MOTOR_RIGHT, 1);

    /* STBY=1 解除 TB6612 待机(STBY=0 时整片输出全关, H 桥不工作) */
    DL_GPIO_setPins(MOTOR_B_PORT, MOTOR_B_STBY_PIN);

    /* 启动 TIMA0 计数, 开始出 PWM(此刻占空 0, 电机不转) */
    DL_TimerA_startCounter(PWM_MOTOR_INST);
}

void motor_set(int ch, int duty)
{
    if (ch != MOTOR_LEFT && ch != MOTOR_RIGHT) return;   /* 非法通道直接忽略, 防越界写 */

    /* 确保驱动使能: motor_stop_all()/急停会拉低 STBY 闩住整片待机, 而 IDLE 态每拍都在调
     * stop_all —— 若这里不重新拉高, 进 RUN 后就是"PWM 在跑、驱动待机、轮子不动"。
     * (2026-07-04 真机排障坐实: PA8 有方波而 STBY 低。重复置位是幂等操作, 无副作用;
     *  停车/急停语义不变: stop_all 拉低待机, 下一次 motor_set 即重新武装。) */
    DL_GPIO_setPins(MOTOR_B_PORT, MOTOR_B_STBY_PIN);

    int forward = (duty >= 0);          /* 符号定方向: >=0 正转, <0 反转 */
    int mag = forward ? duty : -duty;   /* 取幅值(占空 0..1000) */

    if (mag > MOTOR_MAX_DUTY) mag = MOTOR_MAX_DUTY;   /* 软件限幅: 封顶 60% 防超压 */

    motor_set_dir(ch, forward);         /* 先定方向脚 */
    motor_pwm_write(ch, (uint32_t)mag); /* 再写占空(反比换算成 CCR) */
}

void motor_stop_all(void)
{
    /* 两轮 0 占空(CCR=full=恒低), 再拉低 STBY 让 TB6612 进待机(双保险: PWM 停 + 整片关) */
    motor_pwm_write(MOTOR_LEFT, 0);
    motor_pwm_write(MOTOR_RIGHT, 0);
    DL_GPIO_clearPins(MOTOR_B_PORT, MOTOR_B_STBY_PIN);
}

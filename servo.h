/* 云台舵机驱动层（纯驱动，不含比赛逻辑）—— firmware-scaffold driver 层
 *
 * 两路模拟舵机（MG996R 类，50Hz）由单个 TIMA1 出两路边沿对齐 PWM 驱动，
 * 外加一路 GPIO 控制舵机电源轨通断（题目限制⑥：默认关，需要时再供电）。
 *
 * 引脚/定时器由 empty.syscfg 经 SysConfig 生成（实例名见 servo.c 顶部）：
 *   PWM_SERVO = TIMA1, 50Hz：CC0=PA17 云台 PAN，CC1=PA16 云台 TILT
 *   SERVO_EN  = GPIO_IOB 组 PB23：舵机电源轨开关（高=通电；初值低=断电）
 *
 * 约定：ch 0=PAN 1=TILT；角度 deg 0~180°，按每路标定表限位后映射到安全脉宽。
 * 边沿对齐 PWM 极性同 motor.c：高电平计数 = LOAD - CCR，CCR 越大占空越小，
 *   故脉宽越宽 -> CCR 越小（详见 servo.c 的 pulse_to_ccr()）。
 *
 * 依赖：ti_msp_dl_config.h（SysConfig 生成的实例宏/引脚宏）。不依赖其它自写驱动。 */
#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#define SERVO_PAN    0   /**< ch=0 -> 云台 PAN（水平），PWM_SERVO CC0 = PA17 */
#define SERVO_TILT   1   /**< ch=1 -> 云台 TILT（俯仰），PWM_SERVO CC1 = PA16 */

/**
 * @brief 初始化云台舵机：启动 TIMA1 计数，两路给安全中位（占位 90°）。
 * @note  必须在 SYSCFG_DL_init() 之后调用（TIMA1 与 PA16/PA17 的 IOMUX 已配好）。
 *        SysConfig 生成的 PWM init 不启动计数，这里补 startCounter，否则无波形。
 *        本函数不主动给电源轨上电（电源轨默认关，由 servo_rail_enable 单独控制），
 *        故上电后即便有 PWM，舵机也不动，符合题目限制⑥的“默认不通电”。
 */
void servo_init(void);

/**
 * @brief 设置某一路舵机的目标角度。
 * @param ch  舵机通道：SERVO_PAN(0)=水平 / SERVO_TILT(1)=俯仰，其它值忽略。
 * @param deg 目标角度，单位度，范围 0~180（超出按该通道 DEG_MIN/MAX 限位夹紧）。
 * @note  内部按 角度->该通道安全脉宽端点->CCR 写入；CCR 与占空成反比同 motor.c。
 *        端点脉宽与角度限位在 servo.c 顶部集中配置，现场扫限位后回填。
 */
void servo_set_angle(int ch, int deg);

/**
 * @brief 直接按脉宽(us)驱动某路，绕过角度映射。
 * @param ch 通道 SERVO_PAN/SERVO_TILT；其它值忽略。
 * @param pulse_us 目标脉宽(us)，内部安全夹在 0.6~2.4ms。
 * @note  给 360° 连续旋转舵机用：脉宽=转速/方向(≈1.5ms 停)，不是角度。也用于停转脉宽现场微调。
 */
void servo_set_pulse_us(int ch, uint32_t pulse_us);

/** @brief 读取最近一次写入该通道的限位后角度；非法通道返回 0。 */
int servo_get_angle(int ch);

/** @brief 读取最近一次写入该通道的目标脉宽(us)；非法通道返回 0。 */
uint32_t servo_get_pulse_us(int ch);

/** @brief 读取舵机电源轨状态，非 0 = 已请求上电。 */
int servo_rail_is_enabled(void);

/**
 * @brief 控制舵机电源轨（SERVO_EN=PB23）通断。
 * @param on 非 0 = 给舵机供电（PB23 拉高）；0 = 断电（PB23 拉低）。
 * @note  对应题目限制⑥，电源轨默认关。断电后舵机失力（不再保持角度），
 *        通电前应先用 servo_set_angle 设好目标，避免上电瞬间大幅扫动。
 */
void servo_rail_enable(int on);

#endif /* SERVO_H */

/* TB6612FNG 双 H 桥电机驱动层（纯驱动，不含比赛逻辑）—— firmware-scaffold driver 层
 * 一片 TB6612 驱两电机：单 TIMA0 出两路 20kHz PWM（PA8/PB9）+ 每电机 2 个方向脚 + 公共 STBY。
 * 引脚/定时器由 empty.syscfg 经 SysConfig 生成（实例名见 motor.c 顶部）。
 *
 * 约定：ch 0=左轮 1=右轮；duty 带符号 -1000..1000，符号=方向、幅值=占空(0..1000)。
 * 软件限幅 MAX_DUTY=600(60%)：电机额定 7.4V 而电池 12V，封顶防超压烧机。
 *
 * 【强制硬件要求】STBY(PB0) 必须外接 10k 下拉到 GND。原因：MCU 从复位到
 * SYSCFG_DL_GPIO_init() 执行完之间, 所有脚是高阻输入(MSPM0 复位默认输入/高Z),
 * SysConfig 设的 Initial Output=Low 此时尚未生效; 复位/烧录/掉电重启这些窗口里
 * STBY 完全浮空。TB6612 的 STBY 内部无可靠下拉, 若窗口内被拉高即解除整片待机,
 * 而此时 IN1/IN2/PWM 也都浮空, H 桥输入不定可能瞬时驱动电机。软件初值 Low 覆盖
 * 不了这些窗口, 只有外部下拉能保证上电全程 TB6612 待机(STBY=0 时整片输出关断,
 * 其余脚浮空也无输出)。这是唯一硬性的上电安全依赖点, 不是可选建议。 */
#ifndef MOTOR_H
#define MOTOR_H

#define MOTOR_LEFT   0   /* ch=0 -> 左轮 (PWM CCP0=PA8, AIN1=PA12, AIN2=PA13) */
#define MOTOR_RIGHT  1   /* ch=1 -> 右轮 (PWM CCP1=PB9, BIN1=PB6,  BIN2=PB7) */

void motor_init(void);          /* STBY 使能、两轮 0 占空、启动 PWM 计数 */
void motor_set(int ch, int duty);  /* ch:0左/1右; duty:-1000..1000(符号=方向, 幅值=占空) */
void motor_stop_all(void);      /* 两轮 0 占空 + 拉低 STBY 进 TB6612 整片待机 */

#endif /* MOTOR_H */

/* hmi —— 人机交互三合一驱动层（蜂鸣器 + 状态灯 + 按键）firmware-scaffold driver 层
 *
 * 纯驱动, 不含比赛逻辑: 只提供"响/不响""灯亮/灭/翻""哪个键被按下"的原子动作,
 * 由 app 层组合成开机提示音、错误闪灯、模式切换等行为。
 *
 * 硬件接线（与 empty.syscfg / SysConfig 生成宏一一对应, 跨两个 GPIO 组）：
 *   蜂鸣器 BUZZER = PB18  -> GPIO_IOB_PORT / GPIO_IOB_BUZZER_PIN   【有源, 低触发: 拉低=响】
 *   运行灯 LED_RUN = PA24  -> GPIO_IOA_PORT / GPIO_IOA_LED_RUN_PIN
 *   错误灯 LED_ERR = PA26  -> GPIO_IOA_PORT / GPIO_IOA_LED_ERR_PIN
 *   心跳灯 LED_HEART = PB26 -> GPIO_LED_PORT / GPIO_LED_HEART_PIN  （与现有心跳灯同一脚）
 *   启动键 KEY_START = PB19 -> GPIO_IOB_PORT / GPIO_IOB_KEY_START_PIN 【内部上拉, 低=按下】
 *   模式键 KEY_MODE  = PA22 -> GPIO_IOA_PORT / GPIO_IOA_KEY_MODE_PIN  【内部上拉, 低=按下】
 *
 * 【硬件事实】
 *   - 蜂鸣器有源低触发: 初始化必须先拉高(静音), 否则上电就一直叫。
 *   - 按键 4 脚轻触接 GND + 内部上拉, 读到低电平=按下; 需软件消抖。
 *   - 两个按键分属不同 GPIO 组(PB19 / PA22), 各从自己的 _PORT 读, 不能合读。
 *   - LED 极性: SysConfig 里运行/错误灯设为推挽输出, 默认按"高=亮"处理;
 *     若实测板子是低有效, 改 led_run_set/led_err_set 里的 set/clear 即可(已就近标注)。
 *
 * 依赖: 仅依赖 DriverLib 的 DL_GPIO_*(setPins/clearPins/togglePins/readPins) 与
 *       SysConfig 生成的 ti_msp_dl_config.h; 不依赖本工程其它驱动。
 * 用法: SYSCFG_DL_init() 之后调一次 hmi_init(); key_scan() 需周期(建议 10ms)调用做消抖,
 *       led_heart_toggle() 由心跳任务周期调用; 其余动作函数随时可调。
 */
#ifndef HMI_H
#define HMI_H

#include <stdint.h>

/* —— 按键编号（key_pressed 的入参 / 内部按键数组下标）—— */
#define KEY_START   0    /**< 启动键 = PB19 */
#define KEY_MODE    1    /**< 模式键 = PA22 */
#define HMI_KEY_NUM 2    /**< 按键总数, 数组大小用 */

/* ============================ 蜂鸣器 ============================ */
void hmi_init(void);            /**< 初始化: 蜂鸣器静音(拉高)、运行/错误灯灭、清按键状态 */
void beep_on(void);             /**< 蜂鸣器响(有源低触发: 拉低 PB18) */
void beep_off(void);            /**< 蜂鸣器静音(拉高 PB18) */
void beep_set(int on);          /**< on!=0 响, on==0 静音(便于 app 用布尔量直接驱动) */

/* ============================ 状态灯 ============================ */
void led_run_set(int on);       /**< 运行灯 PA24: on!=0 亮, 0 灭 */
void led_err_set(int on);       /**< 错误灯 PA26: on!=0 亮, 0 灭 */
void led_heart_toggle(void);    /**< 心跳灯 PB26 翻转(放心跳任务里周期调) */

/* ============================ 按键 ============================ */
void key_scan(void);            /**< 周期(建议10ms)调: 消抖采样, 检出"按下边沿"并置位事件 */
int  key_pressed(int key);      /**< 查询并清除某键的"按下事件": 返回 1=自上次查询后有过一次按下 */

#endif /* HMI_H */

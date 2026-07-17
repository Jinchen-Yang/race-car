/* encoder —— 双轮编码器读数驱动层（纯驱动，不含里程/速度环逻辑）
 *
 * 模块作用：
 *   统一对外提供“左/右两轮的累计计数”和“距上次的增量”，供速度环每周期算速度、
 *   供里程统计积分。两轮硬件取数方式不同，本模块把差异封装掉，对上只暴露 ch 编号：
 *     ch=0 左轮：TIMG8 硬件正交解码(QEI)，直接读定时器计数寄存器（硬件自动加/减、判向）。
 *     ch=1 右轮：无硬件 QEI，用 GPIO 上升沿中断软件累加（中断里读另一相判方向，带符号 +/-1）。
 *
 * 用到的 SysConfig 实例 / 引脚（学长学姐照《命名契约》在 SysConfig 建实例后即可编译）：
 *   QEI_ENC_L (TIMG8, QEI 2 输入模式)：PHA->CCP0=PA29, PHB->CCP1=PA30 -> 实例宏 QEI_ENC_L_INST
 *       —— 在 SysConfig 里把该 TIMG8 配为 “QEI / Quadrature” 2-input 模式即可，本驱动只负责
 *          启动计数 + 读计数寄存器，不重复配模式。
 *   GPIO_IOA 组 (GPIOA)：ENC_R_A=PA27（★须在 SysConfig 对该脚开“上升沿”中断）
 *       -> GPIO_IOA_PORT, GPIO_IOA_ENC_R_A_PIN
 *   GPIO_IOB 组 (GPIOB)：ENC_R_B=PB24（普通输入，中断里读电平判方向）
 *       -> GPIO_IOB_PORT, GPIO_IOB_ENC_R_B_PIN
 *
 * 中断说明（重要，见 encoder.c 顶部详注）：
 *   右轮中断挂在 GPIOA 组上。本器件 GPIOA 的组中断向量名是 GROUP1_IRQHandler（GPIOA 与
 *   GPIOB 共用 GROUP1 这一根 NVIC 向量），故 ISR 必须实现成 GROUP1_IRQHandler，并在里面
 *   先用 DL_GPIO_getEnabledInterruptStatus 判定到底是不是 PA27 这一位再处理，避免误吃同组
 *   其它脚（如 GPIOB 上的按键）的中断。
 *
 * 依赖的其它驱动：无（仅依赖 SysConfig 生成的 ti_msp_dl_config.h 与 DriverLib）。
 *
 * 标定占位：每圈计数、mm/s per count 等物理量需手转一圈实测后回填，见下方 #define TODO。
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#define ENC_LEFT    0   /**< ch=0 -> 左轮 (TIMG8 硬件 QEI, PHA=PA29 PHB=PA30) */
#define ENC_RIGHT   1   /**< ch=1 -> 右轮 (GPIO 中断累加, A 相=PA27 B 相=PB24) */

/* ===== 每圈计数 =====
 * 左轮: 十圈法实测 (10290-116)/10 = 1017.4 计数/圈  [QEI 4 倍频解码, 硬件状态机抗抖, 金标准]
 * 右轮: 取理论值 1017.4/4 = 254.35 [单相上升沿 1 倍频, 同款编码器倍频比恒为 4]
 *   2026-07-05 推车对照实验定案: 手转标定的旧值 265.2 被接触抖动污染(+4%), 该系统偏差
 *   使右轮测速被低估 4% -> 闭环把右轮真实速度灌高 4% -> 白地持续左偏(实测坐实)。
 *   两遍干净推车归一比值 -1.0%/+2.8% 围绕 4.0, 理论值即真值。
 *   ⚠ 右轮单相计数仍有随机噪声(单次可达+14%), 治本靠编码器线与电机动力线分开走线;
 *     若仍不稳再上 ISR 微秒级毛刺滤波。
 * ⚠ 换电机、换编码器、或改 QEI/中断解码方式后必须重测左轮并按 /4 同步右轮。 */
#define ENC_L_CNT_PER_REV  1017.4f  /**< 左轮每圈计数(十圈法实测) */
#define ENC_R_CNT_PER_REV   254.35f /**< 右轮每圈计数(=左/4 理论值, 2026-07-05 定案) */

/**
 * @brief 初始化编码器读数：启动左轮 QEI 计数、清零右轮软件计数与两轮“上次值”
 * @note  必须在 SYSCFG_DL_init() 之后调用（此时 TIMG8 已被 SysConfig 配成 QEI 模式、
 *        PA27 的上升沿中断与 NVIC 已由 SysConfig 使能）。本函数只补“启动计数”这一步，
 *        因为 SysConfig 生成的 timer init 通常不自动 startCounter（与 motor_init 同理）。
 * @note  右轮中断的“使能”由 SysConfig 负责，这里不再开/关中断，只把软件累加器清零。
 */
void enc_init(void);

/**
 * @brief 读某轮的累计计数（带符号，正=前进方向，具体正负与接线相关，标定时确认）
 * @param ch 轮编号：ENC_LEFT(0)=左 / ENC_RIGHT(1)=右
 * @return   左轮：返回 TIMG8 当前计数寄存器原始值（已转成 int32；硬件计数会回绕，
 *           做里程/速度请优先用 enc_get_delta 取增量，不要直接对本返回值做差）。
 *           右轮：返回软件累加的带符号计数值。
 * @note     ch 非法时返回 0。
 */
int32_t enc_get_count(int ch);

/**
 * @brief 读某轮距“上次调用本函数”以来的计数增量（带符号，已处理硬件计数回绕）
 * @param ch 轮编号：ENC_LEFT(0)=左 / ENC_RIGHT(1)=右
 * @return   两次调用之间的计数变化量（前进为正/为负取决于接线，标定时确认）。
 * @note     供速度环每个控制周期固定调用一次：speed = delta * (mm/s per count)。
 *           左轮按硬件计数模数做回绕处理（见 .c 内 ENC_L_COUNT_MODULO）；
 *           右轮对软件累加值直接做差（int32 累加，正常运行远不会溢出）。
 * @note     每个 ch 各自维护一份“上次值”，左右互不影响；首次调用返回的是相对 enc_init
 *           时刻的增量。ch 非法时返回 0。
 */
int32_t enc_get_delta(int ch);

#endif /* ENCODER_H */

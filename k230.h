/* K230 视觉串口接收层（纯驱动 + 协议解析，不含比赛决策）—— firmware-scaffold driver 层
 *
 * 作用：通过 UART_K230(UART2) 非阻塞接收 K230 视觉模块发来的帧，逐字节喂状态机解析，
 *       对外只暴露"取最新一帧瞄准偏差"。中断里只搬字节进环形缓冲，主循环里跑解析，
 *       全程不阻塞——即使拔了摄像头/K230 不发数据，主控也不会卡死（见 k230_get_aim 的回退约定）。
 *
 * 用到的 SysConfig 实例/引脚（学长学姐须在 SysConfig 按命名契约建好后才能编译）：
 *   UART_K230 (UART2, 115200 8N1)  TX=PB17  RX=PB16
 *     -> UART_K230_INST、UART_K230_INST_IRQHandler、UART_K230_INST_INT_IRQN
 *     ⚠ 必须在 SysConfig 的 UART2 实例里勾选 "RX 中断"，否则收不到字节（k230_init 还会再软件使能一次兜底）。
 *
 * 依赖：
 *   - ti_msp_dl_config.h（SysConfig 生成的 UART_K230_INST 等宏 + DriverLib）。
 *   - 帧协议同源自 contracts/protocol.h（K230↔主控 UART 帧；本模块的解析/重同步逻辑即复用其
 *     proto_parse_byte 状态机；功能码 FUNC_AIM_OFFSET=0x05，负载 frame_aim_offset_t{dx,dy,found}）。
 *     ⚠ 协议是神圣契约，只能 lead 改 contracts/protocol.yaml 再 gen；本模块不得自定义帧格式。
 *
 * 典型用法（主循环）：
 *   k230_init();                 // SYSCFG_DL_init() 之后调，使能 RX 中断
 *   while (1) {
 *     k230_poll();               // 把缓冲里的字节喂进状态机，凑齐整帧就更新缓存
 *     int16_t dx, dy;
 *     if (k230_get_aim(&dx, &dy)) { ... 用视觉偏差精修 ... }
 *     else                       { ... 视觉不可用：回退几何瞄准（拔摄像头也不能卡死）... }
 *   }
 */
#ifndef K230_H
#define K230_H

#include <stdint.h>

/* ===== 调参/标定占位（不填真值，留待整定）===== */
#define K230_BAUD_RATE       115200u   /* 链路波特率, 须与 K230 端 + SysConfig 设的一致(protocol.h: 115200 8N1) */
#define K230_FRAME_TIMEOUT_MS 200u     /* 待整定: 多久没收到有效帧就判"视觉失联", get_aim 返回 found=0 让调用方回退几何瞄准 */

/**
 * @brief 初始化 K230 接收（清接收状态 + 使能 UART2 的 RX 中断）
 * @note 必须在 SYSCFG_DL_init() 之后调用（此时 UART_K230_INST 的时钟/IOMUX 已就绪）。
 *       NVIC 使能通常由 SysConfig 生成的 UART init 完成；这里再软件使能一次 RX 中断兜底，
 *       防止 SysConfig 里忘勾 RX 中断导致永远收不到字节。
 */
void k230_init(void);

/**
 * @brief 把环形缓冲里已收到的字节喂进协议状态机，凑齐一整有效帧就刷新内部缓存
 * @note 在主循环里高频调用（非阻塞，无字节就立即返回）。解析放主循环而非中断，
 *       让中断尽量短（只搬字节），避免长解析占用中断时间。
 */
void k230_poll(void);

/**
 * @brief 取最新一帧的瞄准像素偏差（靶心相对末端指向的偏差，用于末端闭环精修）
 * @param dx 出参：水平像素偏差（int16，可为 0；指针非空才写）
 * @param dy 出参：垂直像素偏差（int16，可为 0；指针非空才写）
 * @return found 标志：1=最近收到的有效帧里 K230 找到了目标且未超时；0=未找到/长时间无帧/未收到过帧
 * @note 【拔摄像头也不能卡死】返回 0 时 *dx/*dy 仍写为上一帧值(或 0)，但调用方必须以返回值为准——
 *       found=0 一律走几何解算回退，绝不能阻塞等视觉。超时由 K230_FRAME_TIMEOUT_MS 判定。
 */
uint8_t k230_get_aim(int16_t *dx, int16_t *dy);

/**
 * @brief 距上一次收到有效帧已过去多少毫秒（供上层做更细的失联策略/打印诊断）
 * @return 毫秒数；从未收到过帧时返回一个很大的值（视为长时间失联）
 * @note 基于全局 1ms 计数 g_tick_ms（empty.c 的 SysTick 维护）。
 */
uint32_t k230_age_ms(void);

#endif /* K230_H */

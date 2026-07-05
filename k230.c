/* K230 视觉串口接收层 —— firmware-scaffold driver 层
 *
 * 硬件/实例（与 SysConfig 命名契约一一对应，学长学姐照此建 UART2 实例）：
 *   UART_K230 = UART2, 115200 8N1, 3.3V TTL   TX=PB17  RX=PB16
 *     -> UART_K230_INST           : 寄存器基址（DL_UART_Main_* 第一个参数）
 *     -> UART_K230_INST_IRQHandler: 本文件实现的中断入口（名字由 SysConfig 生成，形如 UART2_IRQHandler）
 *     -> UART_K230_INST_INT_IRQN  : 该 UART 的 NVIC 中断号
 *
 * 收帧链路（全程非阻塞，拔摄像头也不卡死）：
 *   UART2 RX 中断 -> ISR 只做一件事：DL_UART_Main_receiveData 读 1 字节入环形缓冲 -> 立即返回
 *   主循环 k230_poll() -> 从环形缓冲逐字节喂 proto_parse_byte 状态机 -> 凑齐整帧(ready) -> 更新缓存
 *   主循环 k230_get_aim() -> 读缓存的 dx/dy/found；超过 K230_FRAME_TIMEOUT_MS 没新帧 -> 强制 found=0
 *
 * 帧协议同源：contracts/protocol.h（K230↔主控）。本文件 #include "protocol.h"（已随固件平铺一份，
 *   内容与 contracts/protocol.h 逐字一致，改协议须改 contracts/protocol.yaml 后重 gen 再覆盖此份；
 *   解析/重同步直接复用 protocol.h 里的 proto_parse_byte，不另写一套，避免两端漂移）。
 *
 * 依赖：ti_msp_dl_config.h（DriverLib + UART_K230_INST 宏）、protocol.h（帧定义+解析状态机）、
 *       empty.c 的全局 g_tick_ms（1ms 计数, 用于帧超时判定）。
 *
 * 核对依据（dl_uart_main.h / dl_uart.h, SDK 2.10）：
 *   - DL_UART_Main_receiveData(uart) -> uint8_t        读 1 字节(dl_uart.h L2111)
 *   - DL_UART_Main_getPendingInterrupt(uart) -> DL_UART_IIDX   取最高优先级待处理中断并清该标志(L2863)
 *   - DL_UART_IIDX_RX                                   RX 中断索引(dl_uart.h L192)
 *   - DL_UART_Main_enableInterrupt(uart, DL_UART_MAIN_INTERRUPT_RX)  使能 RX 中断(L2768/L105)
 *   读 IIDX 即清除该中断标志(MSPM0 IIDX 语义), 故 ISR 不再额外 clearInterruptStatus。
 */
#include "ti_msp_dl_config.h"
#include "protocol.h"
#include "k230.h"

/* empty.c 维护的全局 1ms 计数（SysTick 里 ++），本模块只读它做帧超时 */
extern volatile uint32_t g_tick_ms;

/* ===== 接收环形缓冲（中断写 head，主循环读 tail；容量取 2 的幂便于按位取模）===== */
#define K230_RXBUF_SIZE   128u                 /* 须为 2 的幂; 128 足够缓冲若干帧, 主循环来不及取也不溢出丢太多 */
#define K230_RXBUF_MASK   (K230_RXBUF_SIZE - 1u)

static volatile uint8_t  s_rxbuf[K230_RXBUF_SIZE]; /**< 原始字节环形缓冲 */
static volatile uint16_t s_rx_head = 0;            /**< 写指针(仅 ISR 改) */
static volatile uint16_t s_rx_tail = 0;            /**< 读指针(仅主循环改) */

/* ===== 解析出的最新一帧瞄准结果（k230_poll 写, k230_get_aim 读）===== */
static int16_t  s_aim_dx    = 0;     /**< 最新一帧水平像素偏差 */
static int16_t  s_aim_dy    = 0;     /**< 最新一帧垂直像素偏差 */
static uint8_t  s_aim_found = 0;     /**< 最新一帧 K230 是否找到目标(帧内 found 字段) */
static uint32_t s_last_ms   = 0;     /**< 最近一次收到"有效帧"的时间戳(g_tick_ms) */
static uint8_t  s_got_frame = 0;     /**< 是否曾收到过任何有效帧(没收到过则 age 视为极大) */

/* 协议状态机的帧容器（proto_parse_byte 收满一帧把 ready 置 1） */
static proto_frame_t s_frame = {0};

void k230_init(void)
{
    /* 必须在 SYSCFG_DL_init() 之后调用：此刻 UART_K230_INST 的时钟/IOMUX/波特率已由 SysConfig 配好。
     * 清空本模块软件状态，避免上电残留。 */
    s_rx_head   = 0;
    s_rx_tail   = 0;
    s_aim_dx    = 0;
    s_aim_dy    = 0;
    s_aim_found = 0;
    s_got_frame = 0;
    s_last_ms   = g_tick_ms;
    s_frame.ready = 0;

    /* 软件再使能一次 UART2 的 RX 中断兜底（SysConfig 勾了 RX 中断时这步幂等无害；
     * 万一 SysConfig 忘勾, 这里也能让 RX 中断真正生效）。
     * 注: NVIC 那一级的使能一般由 SysConfig 生成的 UART init 完成; 若发现仍不进中断,
     *     检查 SysConfig UART2 实例是否启用了中断 + 生成代码里有无 NVIC_EnableIRQ(UART_K230_INST_INT_IRQN)。 */
    DL_UART_Main_enableInterrupt(UART_K230_INST, DL_UART_MAIN_INTERRUPT_RX);
}

/* UART2 接收中断：只搬字节进环形缓冲，绝不在此做解析/阻塞（中断要短）。
 * 中断处理函数名由 SysConfig 生成（UART_K230_INST_IRQHandler，实际展开形如 UART2_IRQHandler）。 */
void UART_K230_INST_IRQHandler(void)
{
    /* 取最高优先级待处理中断（读 IIDX 同时清该标志，故无需再 clearInterruptStatus）。
     * 用 switch 而非 if，方便日后扩展 RX 超时/溢出等事件。 */
    switch (DL_UART_Main_getPendingInterrupt(UART_K230_INST)) {
    case DL_UART_MAIN_IIDX_RX: {
        uint8_t b = DL_UART_Main_receiveData(UART_K230_INST);   /* 读走 RX FIFO 1 字节 */
        uint16_t next = (uint16_t)((s_rx_head + 1u) & K230_RXBUF_MASK);
        if (next != s_rx_tail) {            /* 缓冲未满才写, 满了宁可丢新字节也不覆盖未读数据 */
            s_rxbuf[s_rx_head] = b;
            s_rx_head = next;
        }
        /* 缓冲满 = 主循环太久没 poll, 丢字节会让当前帧解析失败, 但状态机会在下一个帧头自动重同步, 不会卡死 */
        break;
    }
    default:
        /* 其它中断源（溢出/帧错等）这里不处理: 读 IIDX 已清标志, 直接返回即可, 不影响后续接收 */
        break;
    }
}

void k230_poll(void)
{
    /* 从环形缓冲把 ISR 收进来的字节逐个喂状态机。snap 一下 head, 避免循环里被中断不断推进而长占 CPU。 */
    uint16_t head = s_rx_head;
    while (s_rx_tail != head) {
        uint8_t c = s_rxbuf[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1u) & K230_RXBUF_MASK);

        proto_parse_byte(c, &s_frame);      /* 复用 protocol.h 的解析+重同步状态机 */

        if (s_frame.ready) {                 /* 收满一整有效帧(头/长度/校验/帧尾都过了) */
            s_frame.ready = 0;
            /* s_frame.data[0]=func, 其后为负载。只关心 0x05 aim_offset; 其它功能码这里忽略(可日后扩展) */
            if (s_frame.func == FUNC_AIM_OFFSET &&
                s_frame.len >= (uint8_t)(1u + sizeof(frame_aim_offset_t))) {
                /* 负载从 data[1] 起。protocol.h 里负载结构是 packed 小端, 直接按字段手取避免对齐/字节序坑：
                 *   dx = int16 LE, dy = int16 LE, found = uint8 */
                const uint8_t *p = &s_frame.data[1];
                s_aim_dx    = (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
                s_aim_dy    = (int16_t)((uint16_t)p[2] | ((uint16_t)p[3] << 8));
                s_aim_found = p[4];
                s_last_ms   = g_tick_ms;     /* 记一次"收到有效帧"时间, 供超时判定 */
                s_got_frame = 1;
            }
            /* 其它合法帧(如握手/巡线偏差)目前本模块不消费; 仍刷新 last_ms? —— 不刷:
             * 失联判定专指"没有可用的瞄准信息", 故只有 aim_offset 帧才算"视觉在线"。 */
        }
    }
}

uint8_t k230_get_aim(int16_t *dx, int16_t *dy)
{
    /* 先回填坐标(指针非空才写), 即使最终判失联也给上一帧值, 但调用方必须以返回值(found)为准 */
    if (dx) *dx = s_aim_dx;
    if (dy) *dy = s_aim_dy;

    /* 失联判定: 从未收过帧, 或距上一有效帧已超过超时 -> found=0, 让调用方回退几何瞄准。
     * 【拔摄像头也不能卡死】: 这里只比时间戳, 不等待、不阻塞, K230 静默时立刻返回 0。 */
    if (!s_got_frame) return 0;
    if ((uint32_t)(g_tick_ms - s_last_ms) > K230_FRAME_TIMEOUT_MS) return 0;

    return s_aim_found;   /* 未超时: 返回帧里真实的 found(可能仍是 0 表示"在线但没找到目标") */
}

uint32_t k230_age_ms(void)
{
    if (!s_got_frame) return 0xFFFFFFFFu;          /* 从未收过帧, 视为极大年龄(长时间失联) */
    return (uint32_t)(g_tick_ms - s_last_ms);      /* 无符号回绕安全 */
}

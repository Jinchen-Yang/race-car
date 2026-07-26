/* encoder —— 双轮编码器读数驱动层实现
 *
 * 硬件接线（与 SysConfig 生成宏一一对应，命名见《命名契约》）：
 *   左轮 = TIMG8 硬件正交解码(QEI)：PHA->CCP0=PA29, PHB->CCP1=PA30，实例 QEI_ENC_L_INST。
 *          QEI 模式由 SysConfig 配置（2 输入模式），硬件自动按 A/B 相位关系加/减计数、判方向，
 *          本驱动只负责 startCounter + 读 CTR 计数寄存器。
 *   右轮 = GPIO 中断累加：A 相 ENC_R_A=PA27（SysConfig 开“上升沿”中断），
 *          B 相 ENC_R_B=PB24（普通输入）。每来一个 PA27 上升沿进中断，读 PB24 电平判方向，
 *          对 g_encR_count 带符号 +1/-1。
 *
 * 【中断向量名核实 —— 这是本文件最易踩错的点，已对着 SDK 核实】
 *   GPIOA 组的 NVIC 向量在本器件(MSPM0G350x)启动文件里命名为 GROUP1_IRQHandler。
 *   依据：
 *     1) source/ti/devices/.../mspm0g350x.h 里 GPIOA_INT_IRQn = 1；
 *     2) ticlang 启动文件 startup_mspm0g350x_ticlang.c 第 2 项向量名是 GROUP1_IRQHandler；
 *     3) SysConfig 示例 gpio_simultaneous_interrupts 的 ti_msp_dl_config.h 里
 *        GPIOA_INT_IIDX / GPIOB_INT_IIDX 都是 DL_INTERRUPT_GROUP1_IIDX_*，
 *        说明 GPIOA 与 GPIOB 共用 GROUP1 这一根向量。
 *   => 本文件把右轮 ISR 实现为 GROUP1_IRQHandler。因为 GPIOA/GPIOB 同向量，ISR 里必须先
 *      DL_GPIO_getEnabledInterruptStatus(GPIO_IOA_PORT, GPIO_IOA_ENC_R_A_PIN) 判定确实是
 *      PA27 这一位再处理，处理完 DL_GPIO_clearInterruptStatus 只清这一位，互不影响同组其它脚。
 *   （若以后改用 SysConfig 生成的别名宏，对应名形如 GPIO_IOA_INT_IRQHandler/_IRQN；
 *    本文件直接用底层 GROUP1_IRQHandler，零依赖该别名是否生成，最稳。）
 *
 * 核实过的 DriverLib API（写前已在 dl_timer.h / dl_gpio.h 核对签名，不臆造）：
 *   DL_Timer_startCounter(GPTIMER_Regs*)                 启动定时器计数      (dl_timer.h:3737)
 *   DL_Timer_getTimerCount(const GPTIMER_Regs*) -> u32   读 CTR 计数寄存器   (dl_timer.h:2648)
 *   DL_Timer_setTimerCount(GPTIMER_Regs*, u32)           写 CTR（仅停表时用） (dl_timer.h:2670)
 *   DL_GPIO_getEnabledInterruptStatus(GPIO_Regs*, pins)  读已使能的中断状态   (dl_gpio.h:2628)
 *   DL_GPIO_clearInterruptStatus(GPIO_Regs*, pins)       清中断标志          (dl_gpio.h:2696)
 *   DL_GPIO_readPins(GPIO_Regs*, pins) -> (DIN & pins)    读引脚电平          (dl_gpio.h:2231)
 *
 * 依赖的其它驱动：无。
 */
#include "ti_msp_dl_config.h"
#include "encoder.h"

/* ===================== 标定占位（一律 #define + TODO，不填真值） ===================== */

/* 左轮 QEI 计数寄存器的“计数模数”，用于增量回绕处理。
 * TIMG8 作 QEI 时计数在 0..LOAD 之间回绕；2 输入模式常配满量程 LOAD=0xFFFF（16 位）。
 * 这里按 16 位回绕处理（最常见配置）。
 * /* 待整定：以 SysConfig 里 QEI_ENC_L 的实际 Period/LOAD 为准。若 SysConfig 配的不是 0xFFFF，
 *    把本值改成 (LOAD+1)。验证法：让左轮慢速单向转，逐周期看 enc_get_delta 是否始终同号、
 *    跨越 0/最大值时无突跳。* / */
#define ENC_L_COUNT_MODULO   65536L   /* = 0x10000，对应 16 位计数器满量程回绕 */

/* 每圈计数：✅ 已实测(2026-07-03 十圈法), 真值常量移到 encoder.h 供 app 层取用——
 * 左 ENC_L_CNT_PER_REV=1017.4(QEI 4倍频) / 右 ENC_R_CNT_PER_REV=265.2(单相1倍频), 左右不同属正常。
 * "计数 -> mm/s"的速度换算在 app.c 按轮分别做(还差轮径实测), 本文件不再保留重复常量, 防两处真值打架。 */

/* ============================ 模块内部状态 ============================ */

/* 右轮软件计数：在 GROUP1_IRQHandler 中断里被写，主循环里被读，必须 volatile。
 * 带符号 int32：以小车正常计数速率，运行数十分钟也远不会溢出。 */
static volatile int32_t g_encR_count = 0;   /**< 右轮累计计数（中断带符号累加） */

/* 两轮各自的“上次取增量时的计数”，供 enc_get_delta 做差。
 * 左轮存的是 QEI 计数寄存器的原始无符号值；右轮存的是软件累加值。 */
static uint32_t g_encL_last = 0;            /**< 左轮上次的 QEI 原始计数（无符号，用于回绕做差） */
static int32_t  g_encR_last = 0;            /**< 右轮上次的软件累计计数 */

void enc_init(void)
{
    /* 必须在 SYSCFG_DL_init() 之后调用：TIMG8 已被 SysConfig 配成 QEI 模式、
     * PA27 上升沿中断与 NVIC 已使能。这里只补“启动计数”和“清零软件状态”。 */

    /* 右轮软件累加器清零（中断的使能由 SysConfig 负责，这里不碰中断开关） */
    g_encR_count = 0;
    g_encR_last  = 0;

    /* 启动左轮 QEI 计数（SysConfig 的 timer init 通常不自动 startCounter，同 motor_init 经验）。
     * 启动后硬件即开始按 A/B 相正交关系自动加/减计数。 */
    DL_Timer_startCounter(QEI_ENC_L_INST);          /* 启动 TIMG8 QEI 计数 */

    /* 记录左轮“上次值”基准为当前计数，使首个 enc_get_delta 是相对此刻的增量 */
    g_encL_last = DL_Timer_getTimerCount(QEI_ENC_L_INST);   /* 读 CTR 当前计数 */
}

int32_t enc_get_count(int ch)
{
    if (ch == ENC_LEFT) {
        /* 左轮：直接返回 TIMG8 计数寄存器原始值（硬件计数会回绕，
         * 做里程/速度请用 enc_get_delta，勿对本值跨回绕直接做差） */
        return (int32_t)DL_Timer_getTimerCount(QEI_ENC_L_INST);   /* 读 CTR */
    } else if (ch == ENC_RIGHT) {
        /* 右轮：返回软件累加的带符号计数（读 volatile，单条 32 位读对 Cortex-M0+ 是原子的） */
        return g_encR_count;
    }
    return 0;   /* 非法 ch */
}

int32_t enc_get_delta(int ch)
{
    if (ch == ENC_LEFT) {
        /* 左轮：取当前 QEI 计数，与上次值做“带回绕”的有符号差。
         * 原理：QEI 计数在 [0, MODULO) 内回绕；无符号差再按模数折算到
         * [-MODULO/2, MODULO/2) 区间，即可正确表达正/反向跨界的小增量。 */
        uint32_t now  = DL_Timer_getTimerCount(QEI_ENC_L_INST);   /* 读 CTR */
        uint32_t prev = g_encL_last;
        g_encL_last = now;

        /* 无符号相减自然按 2^32 回绕，这里再折算到本计数器的实际模数 */
        int32_t raw = (int32_t)(now - prev);          /* 先得 2^32 模意义下的差 */
        if (raw >= (ENC_L_COUNT_MODULO / 2)) {
            raw -= ENC_L_COUNT_MODULO;                 /* 正向跨过最大值的回绕修正 */
        } else if (raw < -(ENC_L_COUNT_MODULO / 2)) {
            raw += ENC_L_COUNT_MODULO;                 /* 反向跨过 0 的回绕修正 */
        }
        return raw;
    } else if (ch == ENC_RIGHT) {
        /* 右轮：软件累加值是真·int32（不回绕），直接做差。
         * 先快照 volatile 再做差，避免做差中途被中断改动。 */
        int32_t now  = g_encR_count;
        int32_t prev = g_encR_last;
        g_encR_last = now;
        return now - prev;
    }
    return 0;   /* 非法 ch */
}

/* ============================ 右轮 GPIO 中断 ============================ */

/**
 * @brief 右轮编码器中断服务：PA27(ENC_R_A) 上升沿触发，读 PB24(ENC_R_B) 判方向并带符号累加
 * @note  函数名 GROUP1_IRQHandler 是本器件 GPIOA 组的 NVIC 向量名（GPIOA/GPIOB 共用，
 *        见文件头核实记录）。SysConfig 须已对 PA27 开“上升沿”中断并使能该向量。
 * @note  方向判定约定：A 相上升沿时，若 B 相为高记一个方向、为低记反方向。
 *        当前按 “B 高 -> +1，B 低 -> -1” 实现；/* 待整定：实车单向慢转，若计数反了就把这
 *        两个 +1/-1 对调（或把接线 A/B 互换）。* /
 */
void GROUP1_IRQHandler(void)
{
    /* 只关心 PA27(ENC_R_A) 这一位：先取“已使能且挂起”的状态，判定确实是它再处理，
     * 避免误吃同组(GROUP1)其它脚（如 GPIOB 上的按键）的中断。 */
    uint32_t gpioaStatus =
        DL_GPIO_getEnabledInterruptStatus(GPIO_IOA_PORT, GPIO_IOA_ENC_R_A_PIN);

    if (gpioaStatus & GPIO_IOA_ENC_R_A_PIN) {
        /* A 相上升沿到达，读 B 相(PB24)电平定方向 */
        if (DL_GPIO_readPins(GPIO_IOB_PORT, GPIO_IOB_ENC_R_B_PIN)) {
            g_encR_count++;     /* B 高：正向 +1（方向约定见函数注释，待实车确认） */
        } else {
            g_encR_count--;     /* B 低：反向 -1 */
        }

        /* 清 PA27 这一位的中断标志（只清自己，不动同组其它脚的标志） */
        DL_GPIO_clearInterruptStatus(GPIO_IOA_PORT, GPIO_IOA_ENC_R_A_PIN);
    }
}

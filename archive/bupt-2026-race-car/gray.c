/* gray —— 感为科技 8 路灰度循迹驱动实现  ——  firmware-scaffold driver 层
 *
 * 两层职责分明：
 *   1) gray_read_raw(): 「设备相关」层 —— 只这一处碰感为私有 I2C 协议, 用占位宏隔离。
 *   2) gray_get_error()/gray_is_lost(): 「通用算法」层 —— 加权质心 + 丢线判断,
 *      不关心传感器是哪家、地址是多少, 换任何 8 路数字灰度都能直接复用。
 *
 * 依赖 bsp_i2c 提供的阻塞读函数(本工程的 I2C 收口, 内部封装 DL_I2C_* 控制器时序)：
 *     int i2c_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len);
 *       dev_addr: 7 位从机地址(不含读写位); reg: 起始寄存器; buf/len: 读出缓冲与字节数(len 为 uint8_t);
 *       返回 0=成功, 非 0=失败(NACK/超时/总线错)。
 * 详细引脚/依赖见 gray.h 文件头。
 */
#include "gray.h"
#include "bsp_i2c.h"   /* 提供 i2c_read_regs(); 由 bsp_i2c 模块交付, 非本文件 */

/* ===================================================================== *
 *  ✅ 感为 ganv003 I2C 协议已按官方手册第 7 章核定(2026-07-04, 手册§7.4/7.6/7.7):
 *  - 语法是「命令 + 数据」会话式, 不是寄存器指针式; 但"写1字节命令再读"的线上时序
 *    与标准"写寄存器指针再读"完全相同, 故直接复用 i2c_read_regs(把命令当 reg 传)。
 *  - 7 位从机地址 = 0b1001_1_AD1_AD0(高5位软件地址出厂 0b10011, 低2位看跳线帽):
 *      AD1/AD0 都不装 = 0x4C | 只装AD0 = 0x4D | 只装AD1 = 0x4E | 都装 = 0x4F
 *      (手册例程用 0x4F=双跳线帽; ★以实物跳线帽为准, 都 ≠0x68 不与 MPU6050 冲突)
 *  - 命令 0xDD: 读 8 路数字量, 返回 1 字节, bit0=OUT1 ... bit7=OUT8; LED亮=1。
 *  - 命令 0xAA: ping, 设备在线会回 0x66(用于上电同步, 手册§7.13)。
 * ===================================================================== */
#define GRAY_I2C_ADDR     0x4C   /* ★按实物跳线帽核对: 不装跳线帽=0x4C(见上表), 装了要改! */
#define GRAY_CMD_DIGITAL  0xDD   /* 读数字量命令(Digital Data), 手册§7.7 */
#define GRAY_CMD_PING     0xAA   /* ping 命令, 手册§7.13 */
#define GRAY_PING_ACK     0x66   /* ping 应答: 在线返回 0x66 */
#define GRAY_PING_TRIES   100    /* 上电 ping 最多尝试次数(有界, 不学手册例程的无限等待,
                                  * 防传感器缺席时卡死整个 boot; 每次失败靠 bsp 超时兜底) */

/* 数字量打包(手册§7.7.1 已确认): 1 字节, bit0=OUT1(第1路) ... bit7=OUT8(第8路)。 */
#define GRAY_RAW_BYTES    1

/* ---- 算法标定常量(全部占位, 待现场整定) ---------------------------------- *
 * 物理含义见各行; 调参时只动这一块, 不碰算法本体。 */
#define GRAY_BLACK_LEVEL  0      /* 手册§7.7.1: LED亮=1。白地反光强->LED亮=1, 压到黑线反光弱->LED灭=0,
                                  * 故「压线」= bit==0 -> 本宏取 0。★仍需真车在赛道上验一次(拿黑胶带盖住
                                  * 一路, 看 gray_get_error 是否朝预期方向变), 探测高度/环境光会影响判定。 */
#define GRAY_POS_SCALE    100    /* 偏差放大倍数: 质心(单位=路间距)×此值取整, 便于 PID 用整数 待整定 */
#define GRAY_LOST_MIN_HIT 1      /* 至少几路压到线才算「没丢线」; 少于此判全白丢线 待整定 */
/* 全黑(GRAY_CH_NUM 路全压线)也算异常: 多为出库/交叉/抬起, 同样判丢线, 见算法。 */

/* ---- 8 路探头「物理左->右」与数组下标的对应(★现场务必确认) -------------- *
 *   约定: out[0] = 车头朝前看时「最左」探头, out[GRAY_CH_NUM-1] = 「最右」探头。
 *   质心权重据此从左到右线性给, 故下标顺序若装反, 偏差正负号会整体反 -> 转向反打。
 *   现场确认办法: 手动只挡住最左一路, 看 gray_get_error 应输出「负」(车偏右/线在左)。
 *   若发现装反, 最省事是在 gray_read_raw 出口处把数组首尾对调(留作 TODO 开关)。 */
#define GRAY_REVERSE_ORDER 0    /* 0=不翻转; 1=读出后首尾倒序(应对探头排线装反) 待现场确认 */

/* ---- 内部状态 ---------------------------------------------------------- */
static uint8_t  s_raw[GRAY_CH_NUM];   /**< 最近一次解析出的 0/1 数字量缓存 */
static int16_t  s_last_err;           /**< 上一次有效偏差(丢线时沿用) */
static uint8_t  s_lost;               /**< 丢线标志: 1=丢线 0=正常 */
/* ---- 运行时诊断(2026-07-04 加, 分辨"读取失败"vs"数据极性不对") ---- */
static uint32_t s_ok_cnt;             /**< 累计读取成功次数 */
static uint32_t s_fail_cnt;           /**< 累计读取失败次数(NACK/超时) */
static uint8_t  s_last_byte;          /**< 最近一次成功读到的原始字节(未解析) */
static uint8_t  s_fresh;              /**< r32: 最近一次 gray_get_error 是否真读到数据
                                       *  (0=I2C失败, s_last_byte 是冻结旧值, 调用方勿消费) */

int gray_init(void)
{
    /* 清软件状态; I2C 硬件由 SYSCFG_DL_init 配好, 此处不重复初始化总线。 */
    for (int i = 0; i < GRAY_CH_NUM; i++) s_raw[i] = 0;
    s_last_err = 0;
    s_lost     = 1;   /* 上电默认按「丢线」起步, 等第一次成功读到线再清 */

    /* ping 同步(手册§7.13): 主控常比传感器先初始化完, 立刻发命令可能丢。
     * 发 0xAA 读应答, 收到 0x66 = 设备在线且就绪。有界重试, 不在线不卡 boot。 */
    for (int t = 0; t < GRAY_PING_TRIES; t++) {
        uint8_t ack = 0;
        if (i2c_read_regs(GRAY_I2C_ADDR, GRAY_CMD_PING, &ack, 1) == 0 &&
            ack == GRAY_PING_ACK) {
            return 0;   /* 在线, 就绪 */
        }
    }
    return -1;          /* 缺席/地址不对/总线问题: 上层报警, 循迹将一直判丢线 */
}

int gray_read_raw(uint8_t out[GRAY_CH_NUM])
{
    uint8_t buf[GRAY_RAW_BYTES];

    /* === 唯一一处碰感为私有协议: 发命令 0xDD 后读 1 字节数字量(手册§7.7 方法1,
     *     i2c_read_regs 的"写reg再读"线上时序与之完全一致, 命令当 reg 传) === */
    int ret = i2c_read_regs(GRAY_I2C_ADDR, GRAY_CMD_DIGITAL, buf, GRAY_RAW_BYTES);
    if (ret != 0) {
        s_fail_cnt++;           /* 诊断: 运行时读取失败计数(VOFA 可视) */
        return ret;             /* I2C 失败: 不动 out, 让上层判丢线/沿用上次 */
    }
    s_ok_cnt++;
    s_last_byte = buf[0];       /* 诊断: 原始字节留档(白纸应 0xFF=255, 盖住某路对应位变 0) */

    /* 解析: 假设 1 字节里 bit_i 表示第 i 路是否压线(占位格式, 待协议确认)。
     * GRAY_BLACK_LEVEL 决定「bit=1」是黑还是白, 统一归一成「out[i]=1 表示压到黑线」。 */
    for (int i = 0; i < GRAY_CH_NUM; i++) {
        uint8_t bit = (uint8_t)((buf[0] >> i) & 0x01);
        out[i] = (bit == GRAY_BLACK_LEVEL) ? 1u : 0u;
    }

#if GRAY_REVERSE_ORDER
    /* 探头排线装反时启用: 首尾对调, 把数组对齐到「out[0]=物理最左」约定 */
    for (int i = 0; i < GRAY_CH_NUM / 2; i++) {
        uint8_t t = out[i];
        out[i] = out[GRAY_CH_NUM - 1 - i];
        out[GRAY_CH_NUM - 1 - i] = t;
    }
#endif
    return 0;
}

int16_t gray_get_error(void)
{
    /* 1) 刷新一次原始数字量; 读失败则保持丢线、沿用上次偏差(防总线抖动时方向乱跳)。 */
    if (gray_read_raw(s_raw) != 0) {
        s_fresh = 0;          /* r32: 标记本帧无效, 上层勿消费冻结的 s_last_byte */
        s_lost = 1;
        return s_last_err;
    }
    s_fresh = 1;

    /* 2) 加权质心: 给每路一个「位置权重」(以中线为 0 的对称坐标), 压线路求平均。
     *    8 路对称坐标(放大 2 倍避免分数): 路 i 的坐标 = (2*i - (N-1))。
     *    例 N=8 -> {-7,-5,-3,-1,+1,+3,+5,+7}, 左为负右为正, 中线天然落在 0。
     *    质心 = Σ(坐标_i * 压线_i) / Σ(压线_i)。 */
    int32_t weighted_sum = 0;   /* Σ 坐标*命中 */
    int32_t hit_count    = 0;   /* Σ 命中(压到线的路数) */
    for (int i = 0; i < GRAY_CH_NUM; i++) {
        if (s_raw[i]) {
            int32_t coord = (int32_t)(2 * i - (GRAY_CH_NUM - 1));   /* 该路对称坐标 */
            weighted_sum += coord;
            hit_count    += 1;
        }
    }

    /* 3) 丢线判断:
     *      全白(命中数 < GRAY_LOST_MIN_HIT) -> 没看到线;
     *      全黑(命中数 == GRAY_CH_NUM)      -> 路面全黑(交叉/出库/抬起), 偏差不可信。
     *    两种都判丢线, 返回上次有效偏差(让上层据上次方向继续微调冲出丢线区)。 */
    if (hit_count < GRAY_LOST_MIN_HIT || hit_count == GRAY_CH_NUM) {
        s_lost = 1;
        return s_last_err;
    }

    /* 4) 正常压线: 求质心并换算成偏差。
     *    质心(放大 2 倍坐标) -> 除命中数得平均, 再乘 GRAY_POS_SCALE/2 换成输出标度。
     *    合并写: err = weighted_sum * GRAY_POS_SCALE / (2 * hit_count)。 */
    int32_t err = (weighted_sum * (int32_t)GRAY_POS_SCALE) / (2 * hit_count);

    s_lost     = 0;
    s_last_err = (int16_t)err;
    return s_last_err;
}

uint8_t gray_is_lost(void)
{
    return s_lost;
}

uint8_t gray_frame_fresh(void)
{
    /* r32: 最近一次 gray_get_error 是否真读到数据。track 环每拍先调 gray_get_error
     * 再查本标志, 失败帧按"无数据"处理(不喂漏积分/查表), 堵"冻结字节拐死/段误切"。 */
    return s_fresh;
}

int16_t gray_last_error(void)
{
    /* 纯读缓存, 不发 I2C、不动丢线状态 —— 给调试波形(VOFA)在 RUN 态旁观用,
     * 避免与 track 环(gray_get_error 的唯一合法刷新者)抢总线/搅状态。 */
    return s_last_err;
}

void gray_get_diag(uint32_t *ok_cnt, uint32_t *fail_cnt, uint8_t *last_byte)
{
    /* 纯读诊断快照(VOFA 用): 成功/失败计数 + 最近原始字节。
     * 判读: fail 独涨=总线到灰度这段电平/接触有问题; ok 涨而 last_byte 与预期
     * 反(白纸读 0)=数据极性与手册相反, 翻 GRAY_BLACK_LEVEL 即可。 */
    if (ok_cnt)    *ok_cnt    = s_ok_cnt;
    if (fail_cnt)  *fail_cnt  = s_fail_cnt;
    if (last_byte) *last_byte = s_last_byte;
}

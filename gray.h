/* gray —— 感为科技 8 路灰度循迹模块(I2C/IIC 版)驱动层接口  ——  firmware-scaffold driver 层
 *
 * 模块作用：
 *   读取感为 8 路灰度传感器的「黑/白」数字量, 再用通用加权质心算法解出
 *   小车相对赛道中线的「位置偏差」(给上层 PID 当被控量), 并判断是否「丢线」。
 *
 * 用到的 SysConfig 实例 / 引脚：
 *   本模块自己不直接碰寄存器, 不依赖任何 SysConfig 引脚宏。
 *   它通过 bsp_i2c 走 I2C 总线读传感器。I2C 物理通道由 SysConfig 实例 I2C_BUS
 *   (I2C1: SCL=PB2 SDA=PB3, 见 MEMORY 契约)在 bsp_i2c 里初始化, 与本模块无关。
 *
 * 依赖的其它驱动：
 *   bsp_i2c —— 必须提供阻塞式寄存器读函数(签名见 gray.c 顶部说明)：
 *       int i2c_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len);
 *   ⚠ bsp_i2c 是另一个交付模块, 不在本文件内实现。
 *
 * 【硬件事实】
 *   - 感为 ganv003 8 路灰度, I2C 接口, 与 MPU6050(0x68) 共用一条总线, 上拉到 3.3V。
 *   - 模块输出 8 路结果(本驱动归一成「数字量: 1=压到黑线 / 0=白底」, 见 gray.c)。
 *   ✅ 协议已按官方手册第7章核定(2026-07-04): 地址 0b1001_1_AD1_AD0(无跳线帽=0x4C),
 *     命令 0xDD 读 1 字节数字量(bit0=OUT1, LED亮=1, 压黑线=0), 0xAA ping 应答 0x66。
 *     细节与电气注意(PULL跳线帽必须不装/上拉≥72%供电的说明)见 gray.c 顶部注释。
 */
#ifndef GRAY_H
#define GRAY_H

#include <stdint.h>

#define GRAY_CH_NUM   8   /**< 灰度路数(物理探头个数), 算法循环上限 */

/**
 * @brief 初始化灰度模块: 清软件状态 + ping 同步(感为手册§7.13)
 * @note  I2C 总线由 SYSCFG_DL_init 配好后调用。ping = 发 0xAA 等 0x66 应答,
 *        有界重试(约百次), 传感器缺席不会卡死 boot。
 * @return 0=传感器在线且就绪; -1=ping 失败(缺席/地址不对/总线电平问题),
 *         上层应报警; 此后 gray_read_raw 每次返回错误, 循迹一直判丢线。
 */
int gray_init(void);

/**
 * @brief 读「原始 8 路」灰度数字量(把感为私有协议隔离在这一个函数里)
 * @param out  长度 >= GRAY_CH_NUM(8) 的字节数组; 每元素写 0/1
 *             (1=该路探头压在黑线上, 0=白底)。下标与物理探头的对应见 gray.c 注释。
 * @return 0=读成功; 非 0=I2C 通信失败(此时 out 内容无效, 上层应保持上次状态)
 * @note  ✅ 协议已核定: 内部发命令 GRAY_CMD_DIGITAL(0xDD) 后读 1 字节,
 *        bit0=OUT1..bit7=OUT8, 按 GRAY_BLACK_LEVEL 归一成「1=压黑线」。
 *        换传感器时只动 gray.c 顶部协议宏区 + 本函数解析, 算法层无需动。
 */
int gray_read_raw(uint8_t out[GRAY_CH_NUM]);

/**
 * @brief 计算当前线位置偏差(加权质心法, 不依赖感为协议细节)
 * @return 偏差值: 中线对齐=0, 车偏左(线在右)为正, 车偏右(线在左)为负;
 *         量纲是「以路间距为单位」放大 GRAY_POS_SCALE 倍的整数(见 gray.c)。
 *         若本次丢线(全黑/全白), 返回上一次的有效偏差(便于丢线后沿用上次方向冲出)。
 * @note  内部会自动调 gray_read_raw 刷新一次; 同时更新丢线标志(用 gray_is_lost 查)。
 *        权重/中线零点/丢线阈值均为 #define 占位, 待现场标定。
 */
int16_t gray_get_error(void);

/**
 * @brief 查询最近一次 gray_get_error 是否判定为「丢线」
 * @return 1=丢线(全黑或全白, 偏差不可信); 0=正常压线
 * @note  需先调用过 gray_get_error 才有意义。
 */
uint8_t gray_is_lost(void);

/**
 * @brief 读最近一次有效偏差的缓存值(纯读, 不发 I2C、不动任何状态)
 * @return 上次 gray_get_error 算出的偏差
 * @note  给调试波形(VOFA)在 RUN 态旁观用; 刷新永远只由 gray_get_error 做。
 */
int16_t gray_last_error(void);

/**
 * @brief 最近一次 gray_get_error 是否真读到数据(r32)
 * @return 1=本帧有效; 0=I2C 失败, 缓存字节是冻结旧值, 调用方应按"无数据"处理
 * @note  堵"冻结字节被循迹环消费→拐死/段误切"的缺陷(2026-07-07 审计定案)。
 */
uint8_t gray_frame_fresh(void);

/**
 * @brief 读运行时诊断快照: I2C 成功/失败累计计数 + 最近一次原始字节(未解析)
 * @param ok_cnt/fail_cnt/last_byte 输出指针(不需要的传 NULL)
 * @note  纯读零副作用。fail 独涨=电平/接触问题; ok 涨但 last_byte 极性与手册相反
 *        (白纸应 0xFF)则翻 gray.c 的 GRAY_BLACK_LEVEL。2026-07-04 排障加。
 */
void gray_get_diag(uint32_t *ok_cnt, uint32_t *fail_cnt, uint8_t *last_byte);

#endif /* GRAY_H */

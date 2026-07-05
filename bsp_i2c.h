/* bsp_i2c —— I2C 控制器(主机)寄存器读写封装层  [firmware-scaffold BSP 层]
 *
 * 【模块作用】
 *   在 I2C_BUS_INST(=I2C1) 上提供"读/写从机某寄存器"的友好阻塞式 API,
 *   把 MSPM0 DriverLib 那套 startControllerTransfer / fillTXFIFO /
 *   receiveControllerData / getControllerStatus 的细节包起来, 供上层
 *   mpu6050、gray(感为8路灰度) 等 I2C 设备驱动共用, 避免每个驱动各写一份。
 *   全部为阻塞式 + 软件超时, 任一步卡死都会超时返回错误码而非死等,
 *   保证拔了传感器/总线被拉死时主程序不会卡住。
 *
 * 【依赖的 SysConfig 实例 / 引脚】(学长学姐需在 SysConfig 里建好同名实例)
 *   I2C_BUS (I2C1, 控制器模式): SCL=PB2  SDA=PB3
 *     -> 生成宏 I2C_BUS_INST (本模块直接用此宏寻址外设)
 *   总线上挂 2 个从机: MPU6050(0x68) + 感为8路灰度(地址待定); SCL/SDA 外部上拉到 3.3V。
 *   注意: 7 位地址传入时用"未移位"的原始地址(如 0x68), 移位/读写位由 DriverLib 处理。
 *
 * 【依赖的其它驱动】
 *   仅依赖 SysConfig 生成的 SYSCFG_DL_init() 已完成 I2C_BUS 的时钟/引脚/速率配置;
 *   本模块不自己 init 外设, 只在其上跑事务。
 *
 * 【对外约定】所有函数返回 int 错误码: 0=成功, 负值见下方 BSP_I2C_* 宏。
 */
#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stdint.h>

/* ---- 返回码(错误码) ---- */
#define BSP_I2C_OK            (0)    /**< 成功 */
#define BSP_I2C_ERR_NACK     (-1)    /**< 从机未应答(地址或数据 NACK), 多为设备不在/地址错 */
#define BSP_I2C_ERR_TIMEOUT  (-2)    /**< 软件超时(总线被拉死或从机无响应), 防死等 */
#define BSP_I2C_ERR_ARG      (-3)    /**< 入参非法(空指针 / 长度为 0 / 长度超 FIFO 上限) */
#define BSP_I2C_ERR_BUS      (-4)    /**< 总线异常(仲裁丢失 / 进入事务前总线一直忙) */

/**
 * @brief I2C 卡死从机自愈(9 时钟总线恢复), 在 main() 里 SYSCFG_DL_init() **之前**调用一次
 * @note  从机在半截传输中被打断会死按 SDA, 且按 RESET 不掉电、卡死状态穿越复位存活
 *        (2026-07-04 真机坐实)。本函数用 GPIO 手动敲时钟让从机释放总线并补 STOP;
 *        总线本来正常时调用无任何副作用, 所以每次开机都跑, 当保险。
 */
void i2c_bus_recover(void);

/**
 * @brief 向某从机的某寄存器写入 1 个字节(单字节寄存器写)
 * @param addr7 从机 7 位地址(未移位的原始地址, 如 MPU6050=0x68), 范围 0x00~0x7F
 * @param reg   目标寄存器地址(子地址)
 * @param val   要写入的字节值
 * @return 0=成功; 负值=错误码(见 BSP_I2C_ERR_*)
 * @note 总线时序: START - [addr+W] - [reg] - [val] - STOP, 一次性发出。
 */
int i2c_write_reg(uint8_t addr7, uint8_t reg, uint8_t val);

/**
 * @brief 从某从机的某寄存器起连续读取 len 个字节(寄存器读, 带重复起始)
 * @param addr7 从机 7 位地址(未移位, 如 0x68), 范围 0x00~0x7F
 * @param reg   起始寄存器地址(子地址), 多数 I2C 传感器读后地址自增
 * @param buf   接收缓冲区, 长度至少 len 字节(不可为空)
 * @param len   要读取的字节数, 1~255(受单次突发/FIFO 限制, 见实现内注释)
 * @return 0=成功; 负值=错误码(见 BSP_I2C_ERR_*)
 * @note 时序: START -[addr+W]-[reg]- ReStart -[addr+R]- 读 len 字节 - STOP。
 *       MPU6050 读寄存器、感为灰度读多路数据都走这个函数。
 */
int i2c_read_regs(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t len);

/**
 * @brief 读某从机某寄存器的 1 个字节(i2c_read_regs 的便捷封装)
 * @param addr7 从机 7 位地址(未移位)
 * @param reg   寄存器地址
 * @return >=0: 读到的字节值(0x00~0xFF); <0: 错误码(见 BSP_I2C_ERR_*)
 * @note 返回值用 int 承载, 是为了能同时表达"数据"和"错误"; 调用方先判 <0。
 */
int i2c_read_reg(uint8_t addr7, uint8_t reg);

#endif /* BSP_I2C_H */

/* MPU6050 六轴 IMU 驱动层（纯驱动，不含比赛逻辑）—— firmware-scaffold driver 层
 *
 * 作用：在 I2C 总线上读 MPU6050 的加速度/陀螺原始值，并提供一个“陀螺 Z 积分”得到的航向角(yaw)。
 *
 * 硬件事实（写驱动遵守）：
 *   - I2C 总线 I2C_BUS(I2C1): SCL=PB2 SDA=PB3，上拉到 3.3V，总线上同时挂感为灰度模块。
 *   - 本芯片 7 位地址 = 0x68（AD0 接地；若 AD0 接 VCC 则为 0x69）。
 *   - WHO_AM_I(0x75) 正品 MPU6050 应读回 0x68；部分兼容片(MPU6500/9250/6000)回 0x70/0x71/0x68，
 *     故校验放宽到“在允许集合里”，集合见 mpu6050.c 的 #define。
 *
 * 依赖的其它驱动：
 *   - bsp_i2c：本模块不直接调 DriverLib，所有总线读写都经 bsp_i2c.h 的阻塞式“寄存器读/写”接口
 *     （与 vofa 用回调解耦 UART 同理：把 I2C 时序/DriverLib 细节收在 bsp_i2c 里，IMU 只管协议）。
 *     bsp_i2c 提供如下契约（签名以 bsp_i2c.h 为准，本模块 #include "bsp_i2c.h" 直接用）：
 *       int i2c_write_reg(uint8_t addr7, uint8_t reg, uint8_t val);               // 0=成功, 负=错误码
 *       int i2c_read_reg (uint8_t addr7, uint8_t reg);                            // >=0 即数据, <0 错误码(无出参指针)
 *       int i2c_read_regs(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t len); // 0=成功, 负=错误(len 为 uint8_t)
 *     约定：addr7 为 7 位地址(不含读写位)；写/突发读返回 0=成功，负值=失败(NACK/总线错/超时)。
 *
 * 用法：上电后 SYSCFG_DL_init() + bsp_i2c 初始化完成后，调用 imu_init() 校验并唤醒芯片；
 *       之后每帧调 imu_read_raw() 取原始值；航向角由调用方“按固定周期”调 imu_get_yaw()
 *       做陀螺 Z 积分（周期见 IMU_DT_MS 占位，须与调度任务 period_ms 一致）。
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

/* ---- 返回码：与 motor 风格一致用裸 int，0=成功 ---- */
#define IMU_OK    0   /**< 操作成功 */
#define IMU_ERR  (-1) /**< 操作失败（I2C NACK / WHO_AM_I 不符 / 总线错误） */

/** @brief MPU6050 一帧原始数据（16 位带符号，未做量程换算）。
 *  @note  寄存器顺序即 0x3B 起连续 14 字节：ACCEL_XYZ、TEMP、GYRO_XYZ。
 *         温度仅顺带读出，单位换算见 .c 注释（默认未使用）。 */
typedef struct {
    int16_t accel_x;   /**< 加速度 X 原始值（LSB），换算系数见 IMU_ACC_LSB_PER_G */
    int16_t accel_y;   /**< 加速度 Y 原始值（LSB） */
    int16_t accel_z;   /**< 加速度 Z 原始值（LSB） */
    int16_t temp;      /**< 片上温度原始值（LSB），℃ = temp/340 + 36.53 */
    int16_t gyro_x;    /**< 角速度 X 原始值（LSB），换算系数见 IMU_GYRO_LSB_PER_DPS */
    int16_t gyro_y;    /**< 角速度 Y 原始值（LSB） */
    int16_t gyro_z;    /**< 角速度 Z 原始值（LSB），航向角积分用此轴 */
} imu_raw_t;

/** @brief 初始化 MPU6050：校验 WHO_AM_I、解除休眠、设量程，并清零航向积分。
 *  @return IMU_OK 成功；IMU_ERR 失败（WHO_AM_I 不符或 I2C 无应答）
 *  @note   必须在 SYSCFG_DL_init() 与 bsp_i2c 初始化之后调用。失败多为接线/上拉/地址(AD0)问题。 */
int imu_init(void);

/** @brief 读一帧加速度+陀螺原始值（0x3B 起 14 字节一次性突发读）。
 *  @param out 输出：填入解析后的原始值结构体；不可为 NULL
 *  @return IMU_OK 成功；IMU_ERR 失败（I2C 读失败，out 内容不可信）
 *  @note  这是“裸读”，不含滤波/换算；换算系数见 mpu6050.c 的 #define 占位。 */
int imu_read_raw(imu_raw_t *out);

/** @brief 取当前航向角（度）。
 *  @return 航向角（单位：度，连续累计、可超 ±180，不自动归一化——跑一圈约累计 ±360）
 *  @note  ✅ 纯读缓存、零副作用(2026-07-04 重构): 盲走/到点判定/调试可同时随便调。
 *         积分由 imu_update() 独家推进; 当前为纯陀螺积分, 必有零漂,
 *         开机静止校准见 imu_gyro_calibrate(), 长期漂移改善见 .c 末尾说明。 */
float imu_get_yaw(void);

/** @brief 推进航向积分：读一帧陀螺 -> 去零漂 -> 按 g_tick_ms 真实经过时间累加进航向角。
 *  @note  ★全工程只允许一个调用点（登记为一个调度任务, 建议 10ms）。
 *         多处调用 = 积分被重复推进 = 角度虚大, 这正是本次重构要杜绝的坑。 */
void imu_update(void);

/** @brief 把当前航向角清零（如出发前 / 到达路口重新对零）。 */
void imu_reset_yaw(void);

/** @brief 静止零漂校准：采样若干帧 gyro_z 求平均作为偏置，供 imu_get_yaw 扣除。
 *  @return IMU_OK 成功；IMU_ERR 采样期间 I2C 读失败
 *  @note  ⚠ 调用时小车必须完全静止。采样帧数/丢弃帧数见 .c 的 #define 占位待整定。 */
int imu_gyro_calibrate(void);

/** @brief 读运行时诊断: 14字节突发读的累计成功/失败次数 + 最近一次校准的好样本数。
 *  @note  纯读零副作用, 量化丢包率用(与 gray_get_diag 同款仪表盘)。2026-07-04 排障加。 */
void imu_get_diag(uint32_t *ok_cnt, uint32_t *fail_cnt, uint32_t *cal_good);

#endif /* MPU6050_H */

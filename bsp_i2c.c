/* bsp_i2c.c —— I2C 控制器(主机)寄存器读写封装  [firmware-scaffold BSP 层]
 *
 * 【模块作用】见 bsp_i2c.h 文件头。把 I2C_BUS_INST(I2C1) 上的"写寄存器/读寄存器"
 *   封成阻塞式带超时的友好函数, 供 mpu6050、gray 共用。
 *
 * 【用到的 SysConfig 实例/引脚】
 *   I2C_BUS (I2C1): SCL=PB2 SDA=PB3 -> 宏 I2C_BUS_INST。本文件全部事务都用此宏。
 *
 * 【依赖】SYSCFG_DL_init() 须已先把 I2C_BUS 配好(时钟/IOMUX/速率); 本模块不做外设 init。
 *
 * 【关键 DriverLib API(均已对照 SDK 2.10 source/ti/driverlib/dl_i2c.h 核实, 不臆造)】
 *   DL_I2C_getControllerStatus(i2c)            -> 读 MSR 状态寄存器(返回位或值)
 *     位: DL_I2C_CONTROLLER_STATUS_BUSY/_ERROR/_BUSY_BUS/_IDLE/_ARBITRATION_LOST
 *   DL_I2C_fillControllerTXFIFO(i2c,buf,cnt)   -> 把字节填进 TX FIFO, 返回成功写入数
 *   DL_I2C_flushControllerTXFIFO/RXFIFO(i2c)   -> 清空 TX/RX FIFO(事务前清场)
 *   DL_I2C_startControllerTransfer(i2c,addr,dir,len)
 *                                              -> 设地址+方向+突发长度并自动产生 START+STOP
 *   DL_I2C_enableControllerReadOnTXEmpty(i2c)  -> "发完 TX FIFO 后自动转向读", 用于
 *                                                 写寄存器指针->重复起始->读 的一气呵成
 *   DL_I2C_isControllerRXFIFOEmpty(i2c)        -> RX FIFO 是否空
 *   DL_I2C_receiveControllerData(i2c)          -> 从 RX FIFO 取 1 字节
 *   方向枚举: DL_I2C_CONTROLLER_DIRECTION_TX / _RX
 */
#include "ti_msp_dl_config.h"   /* SysConfig 生成: 含 I2C_BUS_INST 等宏 + DL_* 声明 */
#include "bsp_i2c.h"

/* ---- 软件超时计数(空转次数, 非毫秒) ----
 * 待整定: 经验值, 视 I2C 速率(100k/400k)与 CPU 主频而定。够大以容纳一次完整字节传输,
 *         又不至于卡太久。整定方法: 拔掉从机看是否能在可接受时间内返回 ERR_TIMEOUT。 */
#define BSP_I2C_TIMEOUT_LOOPS   (10000U)    /* 2026-07-04 整定: 原 100000≈30ms 拖垮调度(track 每拍读
                                             * 灰度/IMU 失败时速度环被拖慢 2~3 倍→测速虚高振荡);
                                             * 10000≈3ms, 仍是 100kHz 单字节事务(约0.1ms)的 30 倍余量。 */

/* 单次突发可读字节上限。MSPM0 I2C RX FIFO 深度有限(典型 8 字节), 这里用阻塞式边收
 * 边取的方式, 故 len 实际可大于 FIFO 深度; 仍设一个保守上限防异常入参。
 * 待整定: 若上层(如灰度一次读很多字节)需要更大, 再评估是否改用 DMA。 */
#define BSP_I2C_MAX_READ_LEN    (255U)

/* ===================== 双总线路由(2026-07-04 排障定案) =====================
 * 结论(GRAY_SOLO 实验实锤): 感为 ganv003 的自研 I2C 从机会被"发往其它地址的
 * 流量"搞死——总线上只有灰度时 0 失败; 一出现发往 0x68 的包它就宕机。
 * 处方: 灰度独占 I2C1(PB2/PB3); MPU6050 迁往 I2C0(PA0/PA1, ODIO 开漏脚,
 * 上拉靠 GY-521 板载电阻; ⚠ LaunchPad 的 LED1 挂在 PA0, 其跳线帽必须拔掉,
 * 否则 LED 负载会把高电平拉垮)。本层按 7 位地址路由, 上层驱动完全无感。 */
#ifndef I2C_IMU_INST
#warning "SysConfig 还没建 I2C_IMU 实例: MPU6050 暂与灰度同挤 I2C1(已知互相干扰, 仅供过渡编译)"
#define I2C_IMU_INST I2C_BUS_INST
#endif
static I2C_Regs *i2c_route(uint8_t addr7)
{
    return (addr7 == 0x68U) ? I2C_IMU_INST : I2C_BUS_INST;   /* 0x68=MPU6050 专线 */
}

/* ===================== I2C 卡死从机自愈(9 时钟总线恢复) =====================
 * 病理: 总线抖动使某次传输半途中断 -> 从机以为传输未完, 死按 SDA 不放 -> 之后每次
 *       通信都失败; 且按 RESET 只复位主控、从机不掉电, 卡死状态穿越复位存活
 *       (2026-07-04 真机坐实: 灰度/IMU 反复"值日式"掉线, RESET 无效, 断电才好)。
 * 药方(业界标准): 上电初期把 SCL 当 GPIO 手动敲若干时钟, 让从机把残字节移完释放
 *       SDA, 再补一个 STOP 条件清场; 之后 SYSCFG_DL_init 把引脚配回 I2C 功能。
 * 调用时机: main() 里 SYSCFG_DL_init() 之前(本函数自带 GPIOB 上电)。 */

/* 粗延时: 复位后默认时钟(约32MHz)下每次约 30~60us, 凑 I2C 慢时钟节拍用(精度无关紧要) */
static void recover_delay(void)
{
    for (volatile uint32_t i = 0u; i < 500u; i++) { }
}

/* 单条总线的 9 时钟恢复内核(引脚参数化, 两条总线共用)。
 * 注: PA0/PA1 是 ODIO 开漏脚, setPins=释放(靠外部上拉抬高), 时序照样成立。 */
static void recover_one_bus(GPIO_Regs *scl_port, uint32_t scl_pin, uint32_t scl_iomux,
                            GPIO_Regs *sda_port, uint32_t sda_pin, uint32_t sda_iomux)
{
    /* SCL=GPIO 输出(敲钟用), SDA=GPIO 输入(只观察, 总线靠外部/模块上拉) */
    DL_GPIO_initDigitalOutput(scl_iomux);
    DL_GPIO_initDigitalInput(sda_iomux);
    DL_GPIO_setPins(scl_port, scl_pin);
    DL_GPIO_enableOutput(scl_port, scl_pin);
    recover_delay();

    /* 最多敲 16 个时钟(标准要求 9 个, 加裕量): SDA 一旦被从机释放(读到高)即可停 */
    for (int i = 0; i < 16; i++) {
        if (DL_GPIO_readPins(sda_port, sda_pin)) {
            break;   /* SDA 已释放, 总线自由 */
        }
        DL_GPIO_clearPins(scl_port, scl_pin);
        recover_delay();
        DL_GPIO_setPins(scl_port, scl_pin);
        recover_delay();
    }

    /* 补一个 STOP 条件(SCL 高电平期间 SDA 由低到高), 让所有从机状态机回到空闲 */
    DL_GPIO_initDigitalOutput(sda_iomux);
    DL_GPIO_clearPins(sda_port, sda_pin);
    DL_GPIO_enableOutput(sda_port, sda_pin);
    recover_delay();
    DL_GPIO_setPins(sda_port, sda_pin);   /* SDA 低->高 = STOP */
    recover_delay();
    /* 之后 SYSCFG_DL_init() 会把引脚重配回 I2C 外设功能, 这里不用善后 */
}

void i2c_bus_recover(void)
{
    /* GPIOA+GPIOB 上电(此时 SYSCFG_DL_init 还没跑); 随后 SYSCFG 会再 reset+重配, 无冲突 */
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    recover_delay();

    /* 灰度总线 I2C1 (PB2/PB3) */
    recover_one_bus(GPIO_I2C_BUS_SCL_PORT, GPIO_I2C_BUS_SCL_PIN, GPIO_I2C_BUS_IOMUX_SCL,
                    GPIO_I2C_BUS_SDA_PORT, GPIO_I2C_BUS_SDA_PIN, GPIO_I2C_BUS_IOMUX_SDA);
#ifdef GPIO_I2C_IMU_IOMUX_SCL
    /* IMU 专线 I2C0 (PA0/PA1): 2026-07-04 迁线后 MPU6050 可能带着"半截事务卡死"状态
     * 过来(死按 SDA, LED1 常亮即此征兆), 开机同样敲 9 时钟解锁。 */
    recover_one_bus(GPIO_I2C_IMU_SCL_PORT, GPIO_I2C_IMU_SCL_PIN, GPIO_I2C_IMU_IOMUX_SCL,
                    GPIO_I2C_IMU_SDA_PORT, GPIO_I2C_IMU_SDA_PIN, GPIO_I2C_IMU_IOMUX_SDA);
#endif
}

/* 等待"控制器空闲(上一笔事务收尾完成)"。返回 0=空闲就绪, <0=超时/总线异常。
 * 进入新事务前必须确认总线不忙, 否则会破坏正在进行的传输。 */
static int i2c_wait_idle(I2C_Regs *inst)
{
    uint32_t guard = BSP_I2C_TIMEOUT_LOOPS;
    uint32_t st;

    /* BUSY=控制器正在跑事务; BUSY_BUS=总线层面被占(可能别的主机或被拉死)。
     * 两者都清掉才算可以安全开新事务。 */
    do {
        st = DL_I2C_getControllerStatus(inst);           /* 读 MSR 状态 */
        if (guard-- == 0U) {
            return BSP_I2C_ERR_TIMEOUT;                   /* 一直忙 -> 超时(总线被拉死?) */
        }
    } while (st & (DL_I2C_CONTROLLER_STATUS_BUSY |
                   DL_I2C_CONTROLLER_STATUS_BUSY_BUS));

    return BSP_I2C_OK;
}

/* 等"事务真的启动"(BUSY 置位)。★2026-07-04 排障核心修复:
 * startControllerTransfer 之后, MSR 可能仍显示【上一笔事务】的残留 ERROR 位;
 * 若立刻判错, 会把旧错误当成本笔失败——真机坐实: 对缺席 0x68 的失败读会"毒害"
 * 紧随其后的灰度读, 使其 100% 被误判 NACK(boot 时 ping 在 imu 之前跑所以幸免)。
 * 等 BUSY 置位 = 新事务已接管 MSR, 之后的判错才可信。
 * 超时不算错: 极短事务可能在首次查看前就已完成, 那时 MSR 同样已刷新。 */
static void i2c_wait_started(I2C_Regs *inst)
{
    uint32_t guard = 10000U;
    while (!(DL_I2C_getControllerStatus(inst) & DL_I2C_CONTROLLER_STATUS_BUSY)) {
        if (guard-- == 0U) {
            break;
        }
    }
}

/* 错误路径统一清场: ★中止硬件半截事务 + 关"自动转读" + 清两 FIFO。
 * 2026-07-04 深夜实锤的教训: 只清 FIFO 不中止事务是假清场——超时放弃的长突发读
 * 会把控制器留在 BUSY/拉钟状态, 之后所有事务全死在进门的 wait_idle 上
 * (症状: len9/14 探针跑完后, imu_init 带 5 重试仍 100% 必挂, 是被上一笔尸体堵门)。
 * resetControllerTransfer 强制终止当前传输状态机, 才是真火化。 */
static int i2c_fail(I2C_Regs *inst, int rc)
{
    DL_I2C_resetControllerTransfer(inst);       /* 强制中止半截事务(清 RUN/长度等状态) */
    DL_I2C_disableControllerReadOnTXEmpty(inst);
    DL_I2C_flushControllerTXFIFO(inst);
    DL_I2C_flushControllerRXFIFO(inst);
    DL_I2C_enableController(inst);              /* 防御性重挂使能位(幂等, 保证控制器仍在岗) */
    return rc;
}

/* 等待"当前事务结束(BUSY 撤销)", 并把 MSR 里的错误情况翻译成错误码。
 * 返回 0=事务成功完成; <0=NACK/仲裁丢失/超时。 */
static int i2c_wait_done(I2C_Regs *inst)
{
    uint32_t guard = BSP_I2C_TIMEOUT_LOOPS;
    uint32_t st;

    /* 事务期间 BUSY 置位; 等它清零表示这一笔(含 START/数据/STOP)已收尾。 */
    do {
        st = DL_I2C_getControllerStatus(inst);
        if (guard-- == 0U) {
            return BSP_I2C_ERR_TIMEOUT;        /* 事务迟迟不结束 -> 超时 */
        }
    } while (st & DL_I2C_CONTROLLER_STATUS_BUSY);

    /* 收尾后查错误位:
     *   ERROR  = 地址或数据未被 ACK(设备不在线/地址错/掉线), 归为 NACK 类。
     *   ARBITRATION_LOST = 多主机仲裁丢失(本系统单主机, 出现多半是硬件异常)。 */
    if (st & DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST) {
        return BSP_I2C_ERR_BUS;
    }
    if (st & DL_I2C_CONTROLLER_STATUS_ERROR) {
        return BSP_I2C_ERR_NACK;
    }
    return BSP_I2C_OK;
}

int i2c_write_reg(uint8_t addr7, uint8_t reg, uint8_t val)
{
    /* 入参检查: 7 位地址不得越界 */
    if (addr7 > 0x7FU) {
        return BSP_I2C_ERR_ARG;
    }
    I2C_Regs *inst = i2c_route(addr7);   /* 双总线: 按设备地址选控制器 */

    /* 1) 等总线空闲。若一直忙(上一笔尸体堵门), 走 i2c_fail 火化半截事务再报错,
     *    让上层的"重试"第二枪打在已复位的控制器上, 而不是永远撞同一堵墙。 */
    int rc = i2c_wait_idle(inst);
    if (rc != BSP_I2C_OK) {
        return i2c_fail(inst, rc);
    }

    /* 2) 清场: 防 FIFO 残留脏数据; 并确保"自动转读"是关的(它是粘性开关, 纯写事务
     *    若带着它跑, TX 空后控制器会自作主张转入读, 破坏时序)。 */
    DL_I2C_disableControllerReadOnTXEmpty(inst);
    DL_I2C_flushControllerTXFIFO(inst);
    DL_I2C_flushControllerRXFIFO(inst);

    /* 3) 把"寄存器地址 + 数据"两字节预装进 TX FIFO。
     *    时序将是 START -[addr+W]-[reg]-[val]- STOP。 */
    uint8_t tx[2];
    tx[0] = reg;
    tx[1] = val;
    /* fillControllerTXFIFO 返回实际写入数; FIFO 装不下 2 字节才会少于 2(异常) */
    if (DL_I2C_fillControllerTXFIFO(inst, tx, 2U) < 2U) {
        return i2c_fail(inst, BSP_I2C_ERR_BUS);
    }

    /* 4) 发起发送事务: 方向=TX(写), 突发长度=2; DL 会自动产生 START 与 STOP。 */
    DL_I2C_startControllerTransfer(inst, addr7,
                                  DL_I2C_CONTROLLER_DIRECTION_TX, 2U);
    i2c_wait_started(inst);   /* 等新事务接管 MSR, 防误读上一笔的残留错误位 */

    /* 5) 阻塞等事务结束并判错(NACK/超时); 失败时统一清场 */
    int done = i2c_wait_done(inst);
    if (done != BSP_I2C_OK) {
        return i2c_fail(inst, done);
    }
    return BSP_I2C_OK;
}

int i2c_read_regs(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* 入参检查: 地址越界 / 空缓冲 / 长度 0 / 长度超上限 一律拒绝, 防越界写与卡死 */
    if (addr7 > 0x7FU || buf == 0 || len == 0U || len > BSP_I2C_MAX_READ_LEN) {
        return BSP_I2C_ERR_ARG;
    }

    I2C_Regs *inst = i2c_route(addr7);   /* 双总线: 按设备地址选控制器 */

    /* 1) 等总线空闲。堵门(上一笔半截事务未终止)时同样火化后再报错, 给重试留活路。 */
    int rc = i2c_wait_idle(inst);
    if (rc != BSP_I2C_OK) {
        return i2c_fail(inst, rc);
    }

    /* 2) 清场 */
    DL_I2C_flushControllerTXFIFO(inst);
    DL_I2C_flushControllerRXFIFO(inst);

    /* 3) 先把"寄存器地址"放进 TX FIFO(作为读前的写阶段要发的子地址) */
    if (DL_I2C_fillControllerTXFIFO(inst, &reg, 1U) < 1U) {
        return i2c_fail(inst, BSP_I2C_ERR_BUS);
    }

    /* 4) 开启"TX 发空后自动转读"模式: 控制器会先发 START+[addr+W]+[reg],
     *    待 TX FIFO 空后自动发重复起始 ReStart+[addr+R] 并按下面设的方向/长度读。
     *    这正是 MPU6050/灰度"写寄存器指针再读"的标准时序, 省去中途中断翻转总线。 */
    DL_I2C_enableControllerReadOnTXEmpty(inst);

    /* 5) 发起读事务: 方向=RX, 突发长度=len; 因上面已开 ReadOnTXEmpty,
     *    DL 会先把 TX FIFO 里的 reg 发出去(写阶段), 再重复起始转入读阶段, 末尾给 STOP。 */
    DL_I2C_startControllerTransfer(inst, addr7,
                                  DL_I2C_CONTROLLER_DIRECTION_RX, len);
    i2c_wait_started(inst);   /* ★等新事务接管 MSR 再判错: 否则上一笔的残留 ERROR 位
                               * 会把本笔误判成 NACK(跨事务污染, 2026-07-04 实锤修复) */

    /* 6) 边收边取: 逐字节等 RX FIFO 非空后取出, 带超时防卡死。
     *    不能等收满再一次取, 因 FIFO 深度小于 len 时会溢出丢数据。 */
    uint8_t got = 0U;
    while (got < len) {
        uint32_t guard = BSP_I2C_TIMEOUT_LOOPS;

        /* 等 RX FIFO 里至少有 1 字节 */
        while (DL_I2C_isControllerRXFIFOEmpty(inst)) {
            /* 等待期间若出现错误(NACK/仲裁丢失), 提前退出别空转到超时 */
            uint32_t st = DL_I2C_getControllerStatus(inst);
            if (st & DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST) {
                return i2c_fail(inst, BSP_I2C_ERR_BUS);
            }
            if (st & DL_I2C_CONTROLLER_STATUS_ERROR) {
                return i2c_fail(inst, BSP_I2C_ERR_NACK);
            }
            if (guard-- == 0U) {
                return i2c_fail(inst, BSP_I2C_ERR_TIMEOUT);   /* 数据迟迟不来 -> 超时(从机掉线?) */
            }
        }

        /* 从 RX FIFO 取 1 字节存入用户缓冲 */
        buf[got] = DL_I2C_receiveControllerData(inst);
        got++;
    }

    /* 7) 收完后确认事务收尾(STOP 已发, BUSY 撤销)并复查错误位;
     *    无论成败都关掉"自动转读"(粘性开关), 不留给下一笔纯写事务。 */
    int done = i2c_wait_done(inst);
    DL_I2C_disableControllerReadOnTXEmpty(inst);
    if (done != BSP_I2C_OK) {
        return i2c_fail(inst, done);
    }
    return BSP_I2C_OK;
}

int i2c_read_reg(uint8_t addr7, uint8_t reg)
{
    uint8_t v = 0U;
    int rc = i2c_read_regs(addr7, reg, &v, 1U);
    if (rc != BSP_I2C_OK) {
        return rc;          /* 透传错误码(均为负值) */
    }
    return (int)v;          /* 成功: 返回 0x00~0xFF 的数据(非负) */
}

# 阶段0.5 第③步 — TB6612 双电机 PWM+方向 bring-up checklist

> 全部引脚/API/占空公式已对着本地 MSPM0 SDK 2.10 核实 + 对抗复查通过。
> 代码已就位：`motor.h` / `motor.c`（工程根目录，CCS 自动纳入构建）。
> **还差**：在 `empty.syscfg` 里加 1 个 PWM 实例 + 2 个 GPIO 组，保存生成，再 bench 验证。

## 引脚总表（已核 PINCM，无冲突）

| 功能 | 引脚 | PINCM | SysConfig 里选的功能项 |
|---|---|---|---|
| 左轮 PWM | PA8 | 19 | TIMA0_CCP0 (0x5) |
| 右轮 PWM | PB9 | 26 | TIMA0_CCP1 (0x4) |
| 左 AIN1 | PA12 | 34 | GPIOA_DIO12 |
| 左 AIN2 | PA13 | 35 | GPIOA_DIO13 |
| 右 BIN1 | PB6 | 23 | GPIOB_DIO06（别选成 UART1） |
| 右 BIN2 | PB7 | 24 | GPIOB_DIO07（别选成 UART1） |
| STBY | PB0 | 12 | GPIOB_DIO00 + **外接 10k 下拉到 GND** |

已占用、勿冲突：UART0 PA10/PA11，心跳灯 PB26。

## A. 新增 PWM 模块（实例名必须叫 `PWM_MOTOR`）

左侧 `/ti/driverlib/PWM` → ADD。逐字段：

1. **Name = `PWM_MOTOR`**（区分大小写）→ 决定所有宏前缀（`PWM_MOTOR_INST`、`GPIO_PWM_MOTOR_C0_IDX`…）。填错→motor.c 全部宏未定义。
2. **Timer = TIMA0** → 两路共一个定时器才能共享 20kHz 周期。
3. **启用 CCP0 + CCP1 两个通道** → 左右各一路。只开一路则另一个电机不能调速。
4. **CCP0 pin = PA8** → 下拉里挑 **TIMA0_CCP0(0x5)**。⚠陷阱：同脚还有 `TIMA1_CCP0_CMPL(0x6)`，别选错。
5. **CCP1 pin = PB9** → 下拉里挑 **TIMA0_CCP1(0x4)**。⚠⚠致命陷阱：同脚紧挨着 `TIMA0_CCP0_CMPL(0x5)`（左轮反相互补脚）。选错→右轮变左轮的反相联动，**编译/分配都不报错**，几乎无法排查。
6. **Dead Band = 不勾** → TB6612 是分立 H 桥，每路单端 PWM，不要互补对。
7. **Clock Source = BUSCLK**（=32MHz）。
8. **Divide Ratio = 1**。
9. **Prescale = 1**（GUI 是"除数"语义，填 1 生成寄存器 prescale=0=不分频；**别填 0**）。
10. **Timer Period / Count = `1600`**（计数值，不是时间）→ 32MHz/1600=20kHz，生成 LOAD=1599。
11. **两通道 Duty Cycle 都 = `0%`** → 上电恒低、电机停（已核 0% 即 CCR=1600，安全）。
12. **两通道 Update Method = Immediate** → 改速立即生效（PID 不滞后）。

## B. 新增方向 GPIO（两组，因为一个组只能单端口）

### 组 1：`/ti/driverlib/GPIO` → ADD，**Name = `MOTOR_A`**（GPIOA）
- 加脚 **AIN1 = PA12**，AIN2 = PA13
- 都设 **Output / Push-Pull**，**Initial = Low**

### 组 2：再 ADD 一个 GPIO，**Name = `MOTOR_B`**（GPIOB）
- 加脚 **BIN1 = PB6**，**BIN2 = PB7**，**STBY = PB0**
- 都设 **Output / Push-Pull**，**Initial = Low**
- ⚠ **STBY=PB0 必须外接 10k 下拉到 GND**：软件初值 Low 只在 `GPIO_init` 后生效，盖不住复位/烧录/掉电窗口（那时 PB0 浮空），只有外部下拉能保证上电全程 TB6612 待机。**硬性要求，非建议。**

## C. 保存生成 + 核对

`Ctrl+S` 让 SysConfig 重新生成。打开 `Debug/ti_msp_dl_config.h` 确认出现：
- `PWM_MOTOR_INST = TIMA0`
- `GPIO_PWM_MOTOR_C0_*`（PA8/PINCM19, FUNC=`IOMUX_PINCM19_PF_TIMA0_CCP0`）
- `GPIO_PWM_MOTOR_C1_*`（PB9/PINCM26, FUNC=`IOMUX_PINCM26_PF_TIMA0_CCP1` ← 不是 CCP0_CMPL！）
- `GPIO_PWM_MOTOR_C0_IDX / _C1_IDX`
- `GPIO_MOTOR_A_PORT(=GPIOA)`、`_AIN1_PIN`、`_AIN2_PIN`
- `GPIO_MOTOR_B_PORT(=GPIOB)`、`_BIN1_PIN`、`_BIN2_PIN`、`_STBY_PIN`

宏名对不上 → Name 没填对，回 A2/B 改名重存。

## D. empty.c 集成

```c
#include "motor.h"          // 顶部加

int main(void) {
    SYSCFG_DL_init();
    motor_init();           // ← 加这行：必须在 SYSCFG_DL_init 之后
    vofa_bind_writer(uart_write);
    ...
}
```

> `motor_init()` 里补了 `DL_TimerA_startCounter` —— SysConfig 生成的 PWM init **不启动计数器**，漏了它编译过但完全无波形（经典踩坑）。

bench 转动测试任务（**仅在 wheels-off-ground 时**加到 `g_tasks`）：

```c
static void task_motor_test(void) {   // 每 2s 切一次：停→慢正→停→慢反
    static int phase = 0; int cmd = 0;
    phase = (phase + 1) & 3;
    cmd = (phase == 1) ? 120 : (phase == 3) ? -120 : 0;   // 12% 占空, 远低于 60% 上限
    motor_set(MOTOR_LEFT,  cmd);
    motor_set(MOTOR_RIGHT, cmd);
}
// g_tasks 里加： { task_motor_test, 2000, 2000, 0 },
```

## E. 首次上电安全门（逐条，勿跳）

1. **先只接 TB6612 逻辑 VCC，断开电机动力 VM。**
2. 烧录后用示波器/万用表确认：PA8、PB9 各为 **20kHz 方波、motor_init 后占空 0（恒低）**；STBY(PB0) 实测为高（已解除待机，外部下拉在位）；4 个方向脚电平符合预期。
3. **全部正确，再接 VM**；首次 `motor_set` 用 ≤100/1000 极低占空、**空载**验证两轮转向与调速。
4. 确认无误后再逐步加占空、加负载。

理由：0% 恒低 / 极性 / 转向任一处接反，上负载即可能失控；这步把风险关在空载阶段。

## F. 仍需你现场确认的点

- SYSCTL 时钟图里 **BUSCLK 显示 32MHz**（period=1600→20kHz 的前提）。
- SysConfig 引脚视图里 7 个脚**无红色冲突**（尤其 PB6/PB7 别误选 UART1）。
- STBY 的 **10k 下拉电阻实物已接到 GND**（焊接项，代码/工具保证不了）。
- 生成头里 C1 的 FUNC 确为 `..._TIMA0_CCP1`（防 D5 那个互补脚陷阱）。

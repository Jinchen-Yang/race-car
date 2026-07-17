# Firmware

这里承载北京市赛的通用固件基线，不直接放某一道题的业务逻辑。

计划分层：

```text
bsp/       核心板、时钟、引脚、电源安全默认值
hal/       MSPM0 SDK 外设封装
drivers/   ADC/DAC/UART/SPI/I2C/PWM/QEI 等器件驱动
services/  采集、校准、参数、日志、故障、通信
app/       仅保留可复用状态机骨架
```

07-18 的首个基线必须同时具备：固件水印、非阻塞 UART 日志、1 ms 时基、ADC+DMA、DAC/PWM 输出、参数边界、故障码和上电安全关闭。完成前不从旧项目批量复制 car-specific 代码。

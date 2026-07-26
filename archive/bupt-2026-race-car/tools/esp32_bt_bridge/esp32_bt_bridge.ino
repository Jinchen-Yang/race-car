/* ESP32 经典蓝牙 SPP ↔ 车载 UART0 遥测桥（只听不说）
 *
 * 接线: 车 UART0 TX(XDS110 隔离跳线排的 TX 针脚, 3.3V 电平) → ESP32 GPIO16
 *       GND 与车共地; ESP32 TX 不接(不碰命令台)
 * 供电: 独立小充电宝或电池侧 buck —— 严禁挂主控/灰度那个充电宝(蓝牙发射
 *       是数百 mA 脉冲负载, 会污染"充电宝供电波动"这条判别实验)
 *
 * 用法: 烧录后电脑配对 "racecar-bt" → 出现虚拟串口(mac 下 /dev/tty.racecar-bt*)
 *       → VOFA+ 选它实时看波形, 或 tools/log_serial.py 录 .bin 离线解析。
 * 注意: 仅经典 ESP32(WROOM/WROVER)支持 BT SPP; C3/S3/C6 只有 BLE, 改走 WiFi TCP。
 */
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial2.setRxBufferSize(8192);                 // 蓝牙拥塞时的吸收缓冲(遥测 5KB/s, 可扛 1.6s)
  Serial2.begin(115200, SERIAL_8N1, 16, -1);     // RX=GPIO16, 不占 TX
  SerialBT.begin("racecar-bt");                  // SPP 从机, 无配对码
}

void loop() {
  uint8_t buf[512];
  size_t n = Serial2.available();
  if (n > 0) {
    n = Serial2.read(buf, n > sizeof(buf) ? sizeof(buf) : n);
    SerialBT.write(buf, n);                      // 未连接时静默丢弃, 连上即续流
  }
}

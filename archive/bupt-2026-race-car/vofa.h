/* VOFA+ JustFloat 上位机回传（KB08）：小端 float 数组 + 4 字节帧尾。
 * 整定时回传 {target, actual, output} 看波形。底层写口接 driver 的调试 UART（pinmap: log_tx）。 */
#ifndef VOFA_H
#define VOFA_H

/* driver 层在 init 时注册调试串口写函数（解耦：vofa 不直接调 DriverLib）。 */
void vofa_bind_writer(void (*writer)(const unsigned char *buf, int len));
void vofa_send(const float *ch, int n);   /* n 个通道；如 {target,actual,pwm} 整定 PID */

#endif

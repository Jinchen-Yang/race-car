#include "vofa.h"

/* 调试串口写函数由 driver 层注册（pinmap: log_tx / UART0）。未注册时丢弃，便于主机单测。 */
static void (*s_writer)(const unsigned char *buf, int len) = 0;

void vofa_bind_writer(void (*writer)(const unsigned char *buf, int len)) {
    s_writer = writer;
}

void vofa_send(const float *ch, int n) {
    static const unsigned char tail[4] = { 0x00, 0x00, 0x80, 0x7F };  /* JustFloat 帧尾 */
    if (!s_writer) return;
    s_writer((const unsigned char *)ch, n * 4);
    s_writer(tail, 4);
}

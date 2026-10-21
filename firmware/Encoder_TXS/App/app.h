#ifndef ENCODER_TXS_APP_H
#define ENCODER_TXS_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Giao dien C de main.c goi ung dung C++. */
void setup(void);

void loop(void);

/* Hook SysTick: chi lay mau encoder, khong in UART hay doi trong ngat. */
void appTick1ms(void);

#ifdef __cplusplus
}
#endif

#endif

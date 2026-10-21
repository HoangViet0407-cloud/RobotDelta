#ifndef ENCODER_TXS_SYSTEM_CONFIG_H
#define ENCODER_TXS_SYSTEM_CONFIG_H

#include "main.h"

/* ===================== ENCODER ===================== */
#define ENC_COUNTS_PER_MOTOR_REV 4000L
#define ENC_GEAR_RATIO           10L
#define ENC_A_PORT               ENC_A_GPIO_Port
#define ENC_A_PIN                ENC_A_Pin
#define ENC_B_PORT               ENC_B_GPIO_Port
#define ENC_B_PIN                ENC_B_Pin

/* ===================== COMMUNICATION ============== */
#define COMM_REPORT_PERIOD_MS    100U
#define COMM_TX_TIMEOUT_MS       20U
#define COMM_RX_BUFFER_BYTES     32U

/* ===================== MOTOR / TXS ================= */
#define MOTOR_DIR_PORT           MOTOR_DIR_GPIO_Port
#define MOTOR_OE_PORT            TXS_OE_GPIO_Port
#define MOTOR_OE_PIN             TXS_OE_Pin
#define MOTOR_STEPS_PER_REV      800U
#define MOTOR_MOVE_PULSES        80U
#define MOTOR_MAX_MOVE_PULSES    800U
#define MOTOR_DEFAULT_HZ         100U
#define MOTOR_MIN_HZ             50U
#define MOTOR_MAX_HZ             500U
#define MOTOR_FREQUENCY_STEP_HZ  50U
#define MOTOR_ARM_DIR_HOLD_MS    3U
#define MOTOR_FORWARD_IS_HIGH   true

/* Clock, baud UART, TIM1 ARR/filter va pin label cau hinh trong .ioc.
 * SysTick HAL hien tai goi appTick1ms moi 1 ms.
 * So dem/vong va ti so truyen phai duong.
 */
#if ENC_COUNTS_PER_MOTOR_REV <= 0 || ENC_GEAR_RATIO <= 0
#error "Encoder counts and gear ratio must be positive"
#endif
#if COMM_REPORT_PERIOD_MS == 0 || COMM_TX_TIMEOUT_MS == 0
#error "Communication intervals must be positive"
#endif
#if MOTOR_MOVE_PULSES == 0 || MOTOR_MOVE_PULSES > MOTOR_MAX_MOVE_PULSES
#error "Invalid motor burst size"
#endif
#if MOTOR_MIN_HZ == 0 || MOTOR_DEFAULT_HZ < MOTOR_MIN_HZ || MOTOR_DEFAULT_HZ > MOTOR_MAX_HZ
#error "Invalid motor frequency range"
#endif
#if COMM_RX_BUFFER_BYTES < 2
#error "UART ring requires at least two bytes"
#endif

#endif

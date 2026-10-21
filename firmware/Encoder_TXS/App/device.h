#ifndef ENCODER_TXS_DEVICE_H
#define ENCODER_TXS_DEVICE_H

#include "systemConfig.h"
#include "Libs/QuadratureEncoder.h"
#include "Libs/StepDirMotor.h"
#include <atomic>

/* Du lieu ung dung: them 'dev' vao Live Expressions de xem trang thai.
 * encoderReady publish object cho ISR bang atomic release/acquire.
 * Encoder publish snapshot; yeu cau zero duoc ISR xac nhan o nhip tiep theo.
 * Cac truong report/encoderZeroPending chi dung trong setup/loop.
 */
struct Device
{
  QuadratureEncoder *encoder = nullptr;
  StepDirMotor *motor = nullptr;
  UART_HandleTypeDef *uart = nullptr;
  std::atomic<bool> encoderReady{false};
  bool encoderZeroPending = false;
  volatile bool motorReady = false;
  volatile uint32_t stopEpoch = 0; // Invalidate a previously received arm command.
  uint32_t commandEpoch = 0;
  uint32_t motorFrequency_hz = MOTOR_DEFAULT_HZ;
  bool movePendingReport = false;
  bool streamEnabled = true;
  uint32_t lastReport_ms = 0;
  int32_t lastReport_count = 0;
  uint32_t lastPrint_ms = 0;
};

inline Device dev{}; // C++17: mot bien dung chung cho cac module.

bool XX_initEncoder();
void XX_serviceEncoder();
void XX_initComm();
bool XX_readCommand(uint8_t &command);
void XX_write(const char *text);
void XX_printHelp();
void XX_resetReportWindow();
void XX_reportEncoder();
bool XX_initMotor();
void XX_moveMotor(bool forward);
void XX_serviceMotor();
void XX_reportMotor();
void XX_stopMotorISR();
void XX_serviceComm();

#endif

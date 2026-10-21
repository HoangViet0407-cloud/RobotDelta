#include "device.h"
#include <stdio.h>

/* ===================== COMPONENTS ===================== */
extern "C" TIM_HandleTypeDef htim2;
static StepDirMotor motor(&htim2, MOTOR_DIR_PORT, MOTOR_DIR_Pin,
                          MOTOR_OE_PORT, MOTOR_OE_PIN);

/* ===================== FUNCTIONS ====================== */
bool XX_initMotor()
{
  dev.motorReady = false;
  dev.motor = &motor;
  HAL_GPIO_WritePin(MOTOR_OE_PORT, MOTOR_OE_PIN, GPIO_PIN_RESET);
  /* On debug halt, let the current single pulse finish LOW in hardware.
   * No ISR executes while halted, so no next pulse can start.
   */
  __HAL_DBGMCU_UNFREEZE_TIM2();

  /* Current CubeMX uses default TIMPRE=0. Reject a changed clock topology
   * instead of silently applying the wrong pulse frequency.
   */
  if ((RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) != 0U)
    return false;
  uint32_t clock_hz = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
    clock_hz *= 2U;
  const uint32_t divider = htim2.Instance->PSC + 1U;
  if (clock_hz % divider != 0U)
    return false;
  dev.motorReady = motor.init(clock_hz / divider, MOTOR_MIN_HZ, MOTOR_MAX_HZ,
      MOTOR_MAX_MOVE_PULSES, MOTOR_ARM_DIR_HOLD_MS, MOTOR_FORWARD_IS_HIGH);
  return dev.motorReady;
}

void XX_moveMotor(bool forward)
{
  if (!dev.motorReady || !dev.encoderReady || dev.encoderZeroPending ||
      dev.encoder->snapshot().overflow)
  {
    XX_write("MOVE rejected: motor/encoder not ready, zero pending or encoder overflow\r\n");
    return;
  }
  if (!motor.move(MOTOR_MOVE_PULSES, dev.motorFrequency_hz, forward))
  {
    XX_write("MOVE rejected: send e first, wait 3 ms, and wait until IDLE\r\n");
    return;
  }
  dev.movePendingReport = true;
  XX_reportMotor();
}

void XX_serviceMotor()
{
  if (!dev.motorReady)
    return;
  if (dev.encoderReady && dev.encoder->snapshot().overflow &&
      motor.snapshot().armed)
  {
    motor.abort();
    XX_write("STOP: encoder count overflow; motor disarmed\r\n");
  }
  if (dev.movePendingReport && !motor.snapshot().busy)
  {
    dev.movePendingReport = false;
    XX_reportMotor();
  }
}

void XX_reportMotor()
{
  const MotorSnapshot s = motor.snapshot();
  char line[160];
  (void)snprintf(line, sizeof(line),
      "MOTOR %s OE=%u DIR=%c pulses_done=%lu/%lu nominal_Hz=%lu next_Hz=%lu\r\n",
      !s.ready ? "ERROR" : s.busy ? "RUN" : s.aborted ? "ABORTED" : "IDLE",
      static_cast<unsigned>(s.armed), s.forward ? '+' : '-',
      static_cast<unsigned long>(s.completed), static_cast<unsigned long>(s.requested),
      static_cast<unsigned long>(s.frequency_hz),
      static_cast<unsigned long>(dev.motorFrequency_hz));
  XX_write(line);
}

void XX_stopMotorISR()
{
  if (dev.motorReady)
    motor.abort(); // Short GPIO/register operations only, no UART.
}

/* ===================== CALLBACKS ====================== */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2 && dev.motorReady)
    motor.onPeriodElapsedISR();
}

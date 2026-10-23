#include "device.h"
#include <stdio.h>

/* ===================== COMPONENTS ===================== */
extern "C" TIM_HandleTypeDef htim2;
static StepDirMotor motor(&htim2, MOTOR_DIR_PORT, MOTOR_DIR_Pin,
                          MOTOR_ENA_PORT, MOTOR_ENA_PIN);

/* ===================== FUNCTIONS ====================== */
bool XX_initMotor()
{
  dev.motorReady = false;
  dev.motor = &motor;
  HAL_GPIO_WritePin(MOTOR_ENA_PORT, MOTOR_ENA_PIN,
      MOTOR_ENA_RUN_HIGH ? GPIO_PIN_RESET : GPIO_PIN_SET);
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
      MOTOR_MAX_MOVE_PULSES, MOTOR_ARM_DIR_HOLD_MS, MOTOR_FORWARD_IS_HIGH,
      MOTOR_ENA_RUN_HIGH);
  return dev.motorReady;
}

void XX_selectMotorDirection(bool forward)
{
  if (!dev.motorReady || !motor.setDirection(forward))
  {
    XX_write("DIR rejected: motor unavailable or running; wait until IDLE\r\n");
    return;
  }
  XX_write("DIR selected; no STEP emitted. Send g after arming to move.\r\n");
  XX_reportMotor();
}

void XX_setMotorPulses(uint32_t pulses)
{
  if (pulses == 0U || pulses > MOTOR_MAX_MOVE_PULSES)
  {
    char line[112];
    (void)snprintf(line, sizeof(line),
        "PULSES rejected: use n1..n%lu then Enter; previous value unchanged\r\n",
        static_cast<unsigned long>(MOTOR_MAX_MOVE_PULSES));
    XX_write(line);
    return;
  }
  if (!dev.motorReady || motor.snapshot().busy || dev.commandEpoch != dev.stopEpoch)
  {
    XX_write("PULSES rejected: motor unavailable/running or command cancelled by STOP\r\n");
    return;
  }
  dev.motorPulseCount = pulses;
  XX_write("PULSES selected for next move; no STEP emitted.\r\n");
  XX_reportMotor();
}

void XX_moveMotor()
{
  if (!dev.motorReady || !dev.encoderReady || dev.encoderZeroPending ||
      dev.encoder->snapshot().overflow)
  {
    XX_write("MOVE rejected: motor/encoder not ready, zero pending or encoder overflow\r\n");
    return;
  }
  if (!motor.move(dev.motorPulseCount, dev.motorFrequency_hz, motor.snapshot().forward))
  {
    char line[104];
    (void)snprintf(line, sizeof(line),
        "MOVE rejected: send e first, wait %lu ms, and wait until IDLE\r\n",
        static_cast<unsigned long>(MOTOR_ARM_DIR_HOLD_MS));
    XX_write(line);
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
  char line[192];
  (void)snprintf(line, sizeof(line),
      "MOTOR %s ARMED=%u ENA_cmd=%s ENA_run=%s DIR=%c pulses_done=%lu/%lu nominal_Hz=%lu next_Hz=%lu next_pulses=%lu\r\n",
      !s.ready ? "ERROR" : s.busy ? "RUN" : s.aborted ? "ABORTED" : "IDLE",
      static_cast<unsigned>(s.armed), s.enaCommandHigh ? "HIGH" : "LOW",
      s.enaRunHigh ? "HIGH" : "LOW", s.forward ? '+' : '-',
      static_cast<unsigned long>(s.completed), static_cast<unsigned long>(s.requested),
      static_cast<unsigned long>(s.frequency_hz),
      static_cast<unsigned long>(dev.motorFrequency_hz),
      static_cast<unsigned long>(dev.motorPulseCount));
  XX_write(line);
}

void XX_toggleEnaPolarity()
{
  // A stop received after this command was read invalidates it too.
  const uint32_t saved = __get_PRIMASK();
  __disable_irq();
  const bool changed = dev.commandEpoch == dev.stopEpoch && dev.motorReady &&
      motor.setEnaRunHigh(!motor.snapshot().enaRunHigh);
  __set_PRIMASK(saved);
  if (!changed)
  {
    XX_write("ENA change rejected: send d, wait for STOP, then v while DISARMED\r\n");
    return;
  }
  XX_write("ENA polarity changed in RAM; STOP level changed too. No STEP emitted.\r\n");
  XX_reportMotor();
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

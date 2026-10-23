#include "app.h"
#include "device.h"

/* ===================== SETUP ========================== */
void setup(void)
{
  const bool motorOk = XX_initMotor(); // STEP low, ENA STOP level; never auto-arm.
  const bool encoderOk = XX_initEncoder();
  XX_initComm(); // Start asynchronous RX only after motor object is ready.
  if (!motorOk)
    XX_write("ERROR: TIM2 must be CH1 PWM2/upcounter, TIMPRE=0\r\n");
  if (!encoderOk)
  {
    XX_write("ERROR: TIM1 encoder init failed; check ARR=65535\r\n");
    return;
  }
  XX_resetReportWindow();
  XX_printHelp();
  XX_reportMotor();
}

/* ===================== LOOP =========================== */
void loop(void)
{
  XX_serviceComm();
  XX_serviceEncoder();
  XX_serviceMotor();

  uint8_t command;
  uint32_t argument;
  if (XX_readCommand(command, argument))
  {
    switch (command)
    {
      case 'z': case 'Z':
        if (!dev.encoderReady || dev.encoderZeroPending ||
            (dev.motorReady && dev.motor->snapshot().busy))
        {
          XX_write("ZERO rejected: encoder unavailable, zero pending or motor running\r\n");
          break;
        }
        dev.encoder->zero();
        dev.encoderZeroPending = true; // XX_serviceEncoder prints AFTER ISR ack.
        break;
      case 's': case 'S':
        dev.streamEnabled = true;
        XX_resetReportWindow();
        XX_write("STREAM ON\r\n");
        break;
      case 'p': case 'P':
        dev.streamEnabled = false;
        XX_write("STREAM OFF; encoder counting continues\r\n");
        break;
      case 'r': case 'R':
        if (dev.encoderZeroPending)
          XX_write("ZERO pending: wait for ZERO confirmation, then send r\r\n");
        else if (dev.encoderReady)
          XX_reportEncoder();
        break;
      case 'e': case 'E':
      {
        /* Do not re-arm an old 'e' if !/UART error arrived after it was read.
         * Only the short state transition is atomic; print with IRQs enabled.
         */
        const uint32_t saved = __get_PRIMASK();
        __disable_irq();
        const bool armed = dev.commandEpoch == dev.stopEpoch &&
            dev.motorReady && dev.encoderReady && !dev.encoderZeroPending &&
            !dev.encoder->snapshot().overflow && dev.motor->arm();
        __set_PRIMASK(saved);
        if (armed)
        {
          XX_write("ARMED: software STEP permission; ENA set to RUN level. Driver state unverified.\r\n");
          XX_reportMotor();
        }
        else
          XX_write("ARM rejected: check init/encoder; wait for IDLE and ZERO confirmation\r\n");
        break;
      }
      case 'v': case 'V':
        XX_toggleEnaPolarity();
        break;
      case 'f': case 'F':
        XX_selectMotorDirection(true);
        break;
      case 'b': case 'B':
        XX_selectMotorDirection(false);
        break;
      case 'n':
        XX_setMotorPulses(argument);
        break;
      case 'g': case 'G':
        XX_moveMotor();
        break;
      case '+':
      case '-':
        if (dev.motorReady && dev.motor->snapshot().busy)
          XX_write("Frequency change rejected while running\r\n");
        else
        {
          if (command == '+' && dev.motorFrequency_hz < MOTOR_MAX_HZ)
            dev.motorFrequency_hz += MOTOR_FREQUENCY_STEP_HZ;
          if (command == '-' && dev.motorFrequency_hz > MOTOR_MIN_HZ)
            dev.motorFrequency_hz -= MOTOR_FREQUENCY_STEP_HZ;
          XX_reportMotor();
        }
        break;
      case 'm': case 'M':
        XX_reportMotor();
        break;
      case '?':
        XX_printHelp();
        break;
      case '1':
        BSP_LED_On(LED2);
        break;
      case '0':
        BSP_LED_Off(LED2);
        break;
      default:
        break; // Bo qua CR/LF va ky tu khong ho tro.
    }
  }

  if (dev.encoderReady && !dev.encoderZeroPending && dev.streamEnabled &&
      static_cast<uint32_t>(HAL_GetTick() - dev.lastPrint_ms) >= COMM_REPORT_PERIOD_MS)
    XX_reportEncoder();
}

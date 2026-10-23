#include "device.h"
#include <stdio.h>
#include <string.h>

/* ===================== COMPONENTS ===================== */
extern "C" UART_HandleTypeDef huart2;
static uint8_t rxByte = 0;
static volatile uint8_t rxBuffer[COMM_RX_BUFFER_BYTES];
static volatile uint32_t rxHead = 0;
static volatile uint32_t rxTail = 0;
static volatile uint32_t rxNotice = 0; // 1=stop command, 2=UART/ring error.
static volatile bool rxListening = false;
// Text input belongs to foreground only; the RX ISR only queues bytes/stops.
static bool pulseEntry = false;
static bool pulseEntryInvalid = false;
static uint8_t pulseDigits = 0;
static uint32_t pulseValue = 0;
static uint32_t inputEpoch = 0;

static void resetPulseEntry()
{
  pulseEntry = pulseEntryInvalid = false;
  pulseDigits = 0;
  pulseValue = 0;
}

static void stopFromRx(uint32_t reason)
{
  XX_stopMotorISR();
  ++dev.stopEpoch;
  rxNotice |= reason;
  rxHead = rxTail = 0U; // Main consumer uses a short critical section.
}

static void startRx()
{
  HAL_StatusTypeDef status = HAL_UART_Receive_IT(dev.uart, &rxByte, 1U);
  if (status != HAL_OK)
  {
    (void)HAL_UART_AbortReceive(dev.uart); // No DMA in this project.
    status = HAL_UART_Receive_IT(dev.uart, &rxByte, 1U);
  }
  rxListening = status == HAL_OK;
  if (!rxListening)
    stopFromRx(2U);
}

/* ===================== FUNCTIONS ====================== */
void XX_initComm()
{
  dev.uart = &huart2; // CubeMX da khoi tao baud/pin; ung dung chi su dung.
  dev.streamEnabled = true;
  startRx();
}

static bool readRxByte(uint8_t &command)
{
  const uint32_t saved = __get_PRIMASK();
  __disable_irq();
  const bool available = rxNotice == 0U && rxHead != rxTail;
  if (available)
  {
    command = rxBuffer[rxTail];
    rxTail = (rxTail + 1U) % COMM_RX_BUFFER_BYTES;
    dev.commandEpoch = dev.stopEpoch;
  }
  __set_PRIMASK(saved);
  return available;
}

bool XX_readCommand(uint8_t &command, uint32_t &argument)
{
  argument = 0U;
  uint8_t ch;
  // Bound work per loop even if a sender keeps filling the RX ring.
  for (uint32_t consumed = 0; consumed < COMM_RX_BUFFER_BYTES && readRxByte(ch); ++consumed)
  {
    if (inputEpoch != dev.commandEpoch)
    {
      resetPulseEntry(); // !/d or UART error cancels unfinished numeric input.
      inputEpoch = dev.commandEpoch;
    }
    if (pulseEntry)
    {
      if (ch == 27U) // Escape: cancel without changing the configured count.
      {
        resetPulseEntry();
        XX_write("PULSES input cancelled; previous value unchanged\r\n");
      }
      else if (ch == '\r' || ch == '\n')
      {
        const bool valid = !pulseEntryInvalid && pulseDigits != 0U;
        const uint32_t value = pulseValue;
        resetPulseEntry();
        if (valid)
        {
          command = 'n';
          argument = value;
          return true;
        }
        XX_write("PULSES input invalid: send n<number> then Enter; previous value unchanged\r\n");
      }
      else if (ch == '\b' || ch == 127U)
      {
        if (!pulseEntryInvalid && pulseDigits != 0U)
        {
          pulseValue /= 10U;
          --pulseDigits;
        }
      }
      else if (ch >= '0' && ch <= '9')
      {
        const uint32_t digit = ch - '0';
        if (!pulseEntryInvalid)
        {
          if (pulseDigits >= 10U || pulseValue > (UINT32_MAX - digit) / 10U)
            pulseEntryInvalid = true;
          else
          {
            pulseValue = pulseValue * 10U + digit;
            ++pulseDigits;
          }
        }
      }
      else if (ch != ' ' && ch != '\t')
        pulseEntryInvalid = true; // No partial acceptance of e.g. n800x.
      continue;
    }
    if (ch == 'n' || ch == 'N')
    {
      resetPulseEntry();
      pulseEntry = true;
      XX_write("PULSES: type integer then Enter; Backspace edits, Esc cancels, ! stops\r\n");
      continue;
    }
    if (ch != 0U && strchr("zZsSpPrReEfFbBgGvVmM?10+-", static_cast<int>(ch)) != nullptr)
    {
      command = ch;
      return true;
    }
  }
  return false;
}

void XX_serviceComm()
{
  const uint32_t saved = __get_PRIMASK();
  __disable_irq();
  if (!rxListening)
    startRx();
  const uint32_t notice = rxNotice;
  if (rxListening)
    rxNotice = 0U;
  __set_PRIMASK(saved);
  if ((notice & 1U) != 0U)
  {
    XX_write("STOP: STEP low, software DISARMED; ENA set to STOP level. Driver power remains ON.\r\n");
    XX_reportMotor();
  }
  if ((notice & 2U) != 0U)
    XX_write("UART/ring error: motor disarmed; RX recovery attempted\r\n");
}

void XX_write(const char *text)
{
  /* Chi goi trong setup/loop. Trong luc TX cho, SysTick van cap nhat count. */
  (void)HAL_UART_Transmit(dev.uart, reinterpret_cast<const uint8_t *>(text),
                         static_cast<uint16_t>(strlen(text)), COMM_TX_TIMEOUT_MS);
}

/* Dinh dang so /1000; khong can printf float/long long cua newlib-nano. */
static void formatMilli(int64_t value, char out[32])
{
  char digits[24];
  unsigned n = 0, cursor = 0;
  const uint64_t magnitude = value < 0 ? static_cast<uint64_t>(-(value + 1)) + 1U
                                      : static_cast<uint64_t>(value);
  uint64_t whole = magnitude / 1000U;
  const uint32_t fraction = static_cast<uint32_t>(magnitude % 1000U);
  if (value < 0)
    out[cursor++] = '-';
  do
  {
    digits[n++] = static_cast<char>('0' + whole % 10U);
    whole /= 10U;
  } while (whole != 0U);
  while (n != 0U)
    out[cursor++] = digits[--n];
  out[cursor++] = '.';
  out[cursor++] = static_cast<char>('0' + fraction / 100U);
  out[cursor++] = static_cast<char>('0' + (fraction / 10U) % 10U);
  out[cursor++] = static_cast<char>('0' + fraction % 10U);
  out[cursor] = '\0';
}

void XX_printHelp()
{
  char line[112];
  XX_write("\r\nEncoder_TXS DIRECT_ENA v3: variable pulse count + preselected DIR; USART2 IRQ RX.\r\n");
  XX_write("Encoder v2: atomic snapshot, no encoder IRQ masking; z ack on next SysTick.\r\n");
  (void)snprintf(line, sizeof(line),
      "CPR=%ld/motor_rev; gear=%ld; report=%lu ms. Relative position.\r\n",
      static_cast<long>(ENC_COUNTS_PER_MOTOR_REV), static_cast<long>(ENC_GEAR_RATIO),
      static_cast<unsigned long>(COMM_REPORT_PERIOD_MS));
  XX_write(line);
  XX_write("z: zero | s: stream ON | p: stream OFF | r: read once | ?: help\r\n");
  XX_write("n<number> + Enter: next pulse count | f/b: select forward/reverse (NO motion)\r\n");
  XX_write("e: arm + ENA RUN | g: GO once using selected count/DIR | ! or d: stop + ENA STOP\r\n");
  XX_write("v: invert ENA RUN/STOP levels (DISARMED only; reset restores default)\r\n");
  XX_write("+/-: next speed (idle only) | m: motor status | 1/0: LED on/off\r\n");
  (void)snprintf(line, sizeof(line), "Pulses default=%lu, range=1..%lu; speed default=%lu Hz, range=%lu..%lu Hz\r\n",
      static_cast<unsigned long>(MOTOR_DEFAULT_PULSES),
      static_cast<unsigned long>(MOTOR_MAX_MOVE_PULSES),
      static_cast<unsigned long>(MOTOR_DEFAULT_HZ),
      static_cast<unsigned long>(MOTOR_MIN_HZ), static_cast<unsigned long>(MOTOR_MAX_HZ));
  XX_write(line);
  (void)snprintf(line, sizeof(line), "Motor=%lu pulses/rev; gearbox=1:%ld. Settings reset on boot.\r\n",
      static_cast<unsigned long>(MOTOR_STEPS_PER_REV), static_cast<long>(ENC_GEAR_RATIO));
  XX_write(line);
  XX_write("PA0=STEP, PA4=DIR, PB0=ENA; direct 3.3V GPIO. Boot DISARMED (software).\r\n");
  (void)snprintf(line, sizeof(line), "Default ENA RUN=%s, STOP=%s; v override is lost on reset.\r\n",
      MOTOR_ENA_RUN_HIGH ? "HIGH" : "LOW", MOTOR_ENA_RUN_HIGH ? "LOW" : "HIGH");
  XX_write(line);
  XX_write("ENA_cmd is GPIO latch, NOT driver status. 3.3V input electrical margins unverified.\r\n");
  XX_write("p only stops reports. ! cuts STEP and sets ENA STOP, NOT power. Zero at rest.\r\n");
}

void XX_resetReportWindow()
{
  const EncoderSnapshot sample = dev.encoder->snapshot();
  dev.lastReport_ms = sample.ms;
  dev.lastReport_count = sample.count;
  dev.lastPrint_ms = sample.ms;
}

void XX_reportEncoder()
{
  const EncoderSnapshot sample = dev.encoder->snapshot();
  const uint32_t elapsed_ms = sample.ms - dev.lastReport_ms;
  const int64_t delta_count = static_cast<int64_t>(sample.count) - dev.lastReport_count;
  const int64_t motor_mdeg = static_cast<int64_t>(sample.count) * 360000L /
                             ENC_COUNTS_PER_MOTOR_REV;
  const int64_t gear_mdeg = static_cast<int64_t>(sample.count) * 360000L /
                            (static_cast<int64_t>(ENC_COUNTS_PER_MOTOR_REV) * ENC_GEAR_RATIO);
  const int64_t rpm_milli = elapsed_ms == 0U ? 0 : delta_count * 60000000LL /
                            (static_cast<int64_t>(ENC_COUNTS_PER_MOTOR_REV) * elapsed_ms);
  char motor[32], gear[32], rpm[32], line[224];
  formatMilli(motor_mdeg, motor);
  formatMilli(gear_mdeg, gear);
  formatMilli(rpm_milli, rpm);
  const char *direction = delta_count > 0 ? "+" : delta_count < 0 ? "-" : "0";
  const int length = snprintf(line, sizeof(line),
      "t=%lu CNT=%ld motor_deg=%s gear_deg=%s motor_rpm=%s DIR=%s AB=%u%u OVF=%u\r\n",
      static_cast<unsigned long>(sample.ms), static_cast<long>(sample.count),
      motor, gear, rpm, direction, static_cast<unsigned>(sample.a),
      static_cast<unsigned>(sample.b), static_cast<unsigned>(sample.overflow));
  if (length > 0 && length < static_cast<int>(sizeof(line)))
    XX_write(line);
  dev.lastReport_ms = sample.ms;
  dev.lastReport_count = sample.count;
  dev.lastPrint_ms = HAL_GetTick();
}

/* ===================== CALLBACKS ====================== */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
    return;
  const uint8_t ch = rxByte;
  rxListening = false;
  if (huart->ErrorCode != HAL_UART_ERROR_NONE)
    stopFromRx(2U);
  else if (ch == '!' || ch == 'd' || ch == 'D')
    stopFromRx(1U); // Process even during a blocking terminal print.
  else if (rxNotice == 0U &&
           ((ch >= 32U && ch <= 126U) || ch == '\r' || ch == '\n' ||
            ch == '\t' || ch == '\b' || ch == 127U || ch == 27U))
  {
    const uint32_t next = (rxHead + 1U) % COMM_RX_BUFFER_BYTES;
    if (next == rxTail)
      stopFromRx(2U);
    else
    {
      rxBuffer[rxHead] = ch;
      rxHead = next;
    }
  }
  startRx(); // Re-arm reception after every byte, including ignored CR/LF.
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
    return;
  stopFromRx(2U);
  rxListening = false;
  (void)HAL_UART_AbortReceive(huart);
  startRx();
}

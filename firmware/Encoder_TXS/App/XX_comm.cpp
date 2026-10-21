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

bool XX_readCommand(uint8_t &command)
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
    XX_write("STOP: pulses aborted, TXS OE=0. Send e again to arm\r\n");
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
  XX_write("\r\nEncoder_TXS: TIM1 encoder + TIM2 single-pulse STEP; USART2 IRQ RX.\r\n");
  XX_write("Encoder v2: atomic snapshot, no encoder IRQ masking; z ack on next SysTick.\r\n");
  (void)snprintf(line, sizeof(line),
      "CPR=%ld/motor_rev; gear=%ld; report=%lu ms. Relative position.\r\n",
      static_cast<long>(ENC_COUNTS_PER_MOTOR_REV), static_cast<long>(ENC_GEAR_RATIO),
      static_cast<unsigned long>(COMM_REPORT_PERIOD_MS));
  XX_write(line);
  XX_write("z: zero | s: stream ON | p: stream OFF | r: read once | ?: help\r\n");
  XX_write("e: arm TXS | f/b: forward/reverse burst | ! or d: stop + disarm\r\n");
  XX_write("+/-: next speed (idle only) | m: motor status | 1/0: LED on/off\r\n");
  (void)snprintf(line, sizeof(line), "Burst=%lu pulses; default=%lu Hz; range=%lu..%lu Hz\r\n",
      static_cast<unsigned long>(MOTOR_MOVE_PULSES),
      static_cast<unsigned long>(MOTOR_DEFAULT_HZ),
      static_cast<unsigned long>(MOTOR_MIN_HZ), static_cast<unsigned long>(MOTOR_MAX_HZ));
  XX_write(line);
  XX_write("PA0=STEP, PA4=DIR, PB0=TXS OE (dedicated motor TXS). Boot DISARMED.\r\n");
  XX_write("p only stops reports. ! cuts pulses, NOT driver power/ENA. Zero at rest.\r\n");
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
           strchr("zZsSpPrReEfFbBmM?10+-", static_cast<int>(ch)) != nullptr &&
           ch != 0U)
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

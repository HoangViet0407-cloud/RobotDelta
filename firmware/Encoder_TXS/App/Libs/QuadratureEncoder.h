#ifndef QUADRATURE_ENCODER_H
#define QUADRATURE_ENCODER_H

#include "stm32f4xx_hal.h"
#include <atomic>

static_assert(std::atomic<uint32_t>::is_always_lock_free &&
              std::atomic<int32_t>::is_always_lock_free &&
              std::atomic<bool>::is_always_lock_free,
              "Encoder needs native lock-free 32-bit/bool atomics");

struct EncoderSnapshot
{
  int32_t count;
  uint32_t ms;
  uint8_t a;
  uint8_t b;
  bool overflow;
};

/* Driver doc timer quadrature modulo 65536. Khong phu thuoc App/config/UART.
 * CubeMX phai cau hinh encoder TI12, ARR=65535 truoc init().
 * updateISR(): goi deu, <32768 count giua hai lan goi.
 * init(): mot lan trong setup, truoc khi cho phep updateISR().
 * SysTick la writer duy nhat sau init. zero() gui yeu cau, khong cho.
 * snapshot()/zero()/zeroPending(): chi goi tu main/setup/loop, KHONG tu ISR.
 * snapshot() co the doc lai neu SysTick chen giua; khong khoa ngat.
 * Khong cap phat heap, khong dung atomic 64-bit hay mutex.
 */
class QuadratureEncoder
{
public:
  constexpr QuadratureEncoder(TIM_HandleTypeDef *timer,
      GPIO_TypeDef *portA, uint16_t pinA, GPIO_TypeDef *portB, uint16_t pinB)
    : timer_(timer), portA_(portA), portB_(portB), pinA_(pinA), pinB_(pinB) {}

  HAL_StatusTypeDef init();
  void updateISR();
  void zero();
  bool zeroPending() const;
  EncoderSnapshot snapshot() const;

private:
  void publishSample(); // Only init (before sampling) and the single writer ISR.

  TIM_HandleTypeDef *timer_;
  GPIO_TypeDef *portA_;
  GPIO_TypeDef *portB_;
  uint16_t pinA_;
  uint16_t pinB_;
  std::atomic<bool> ready_{false};
  std::atomic<bool> zeroRequested_{false};

  // Private writer state: main must never reset/read these after init.
  uint16_t previousRaw_ = 0;
  int32_t position_ = 0;
  bool overflow_ = false;

  // Even sequence = complete sample; odd = writer is publishing.
  // Payloads are atomic too: no C++ data race while the reader retries.
  std::atomic<uint32_t> sequence_{0};
  std::atomic<int32_t> sampleCount_{0};
  std::atomic<uint32_t> sampleMs_{0};
  std::atomic<uint32_t> sampleFlags_{0}; // bit 0=A, 1=B, 2=overflow.
};

#endif

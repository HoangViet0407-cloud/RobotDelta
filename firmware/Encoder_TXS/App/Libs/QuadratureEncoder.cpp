#include "QuadratureEncoder.h"
#include <stdint.h>

HAL_StatusTypeDef QuadratureEncoder::init()
{
  ready_.store(false, std::memory_order_release);
  if (timer_ == nullptr || timer_->Instance == nullptr ||
      portA_ == nullptr || portB_ == nullptr || pinA_ == 0 || pinB_ == 0)
    return HAL_ERROR;
  if (__HAL_TIM_GET_AUTORELOAD(timer_) != 65535U)
    return HAL_ERROR;

  const HAL_StatusTypeDef status = HAL_TIM_Encoder_Start(timer_, TIM_CHANNEL_ALL);
  if (status != HAL_OK)
    return status;

  previousRaw_ = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(timer_));
  position_ = 0;
  overflow_ = false;
  zeroRequested_.store(false, std::memory_order_relaxed);
  publishSample();
  ready_.store(true, std::memory_order_release);
  return HAL_OK;
}

void QuadratureEncoder::updateISR()
{
  if (!ready_.load(std::memory_order_acquire))
    return;

  const uint16_t raw = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(timer_));
  const bool reset = zeroRequested_.load(std::memory_order_acquire);
  if (reset)
  {
    // Zero takes effect at this sample, not when UART first received 'z'.
    previousRaw_ = raw;
    position_ = 0;
    overflow_ = false;
  }
  else
  {
    int32_t delta = static_cast<int32_t>(raw) - previousRaw_;
    previousRaw_ = raw;
    if (delta > 32767)
      delta -= 65536;
    else if (delta < -32768)
      delta += 65536;

    if (!overflow_)
    {
      const int64_t next = static_cast<int64_t>(position_) + delta;
      if (next > INT32_MAX || next < INT32_MIN)
        overflow_ = true;
      else
        position_ = static_cast<int32_t>(next);
    }
  }

  publishSample();
  if (reset)
    zeroRequested_.store(false, std::memory_order_release); // Ack AFTER publish.
}

void QuadratureEncoder::zero()
{
  if (ready_.load(std::memory_order_acquire))
    zeroRequested_.store(true, std::memory_order_release);
}

bool QuadratureEncoder::zeroPending() const
{
  return zeroRequested_.load(std::memory_order_acquire);
}

void QuadratureEncoder::publishSample()
{
  const uint32_t pinsA = portA_->IDR;
  const uint32_t pinsB = portA_ == portB_ ? pinsA : portB_->IDR;
  const uint32_t flags = ((pinsA & pinA_) != 0U ? 1U : 0U) |
                         ((pinsB & pinB_) != 0U ? 2U : 0U) |
                         (overflow_ ? 4U : 0U);
  const uint32_t odd = sequence_.load() + 1U;
  // seq_cst for both sequence and payload keeps the read/retry protocol ordered.
  sequence_.store(odd);
  sampleCount_.store(position_);
  sampleMs_.store(HAL_GetTick());
  sampleFlags_.store(flags);
  sequence_.store(odd + 1U);
}

EncoderSnapshot QuadratureEncoder::snapshot() const
{
  EncoderSnapshot sample{};
  if (!ready_.load(std::memory_order_acquire))
  {
    sample.ms = HAL_GetTick();
    return sample;
  }
  for (;;)
  {
    const uint32_t before = sequence_.load();
    if ((before & 1U) != 0U)
      continue;
    sample.count = sampleCount_.load();
    sample.ms = sampleMs_.load();
    const uint32_t flags = sampleFlags_.load();
    if (before == sequence_.load())
    {
      sample.a = static_cast<uint8_t>(flags & 1U);
      sample.b = static_cast<uint8_t>((flags >> 1U) & 1U);
      sample.overflow = (flags & 4U) != 0U;
      return sample;
    }
  }
}

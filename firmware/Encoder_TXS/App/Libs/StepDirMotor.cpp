#include "StepDirMotor.h"

namespace
{
/* Only short register/state copies. Never wait, print or divide in this scope.
 * Also serializes the UART stop callback against move()/arm() in main.
 * Hardware encoder counting continues while CPU interrupts are masked.
 */
class CriticalSection
{
public:
  CriticalSection() : saved_(__get_PRIMASK()) { __disable_irq(); }
  ~CriticalSection() { __set_PRIMASK(saved_); }
  CriticalSection(const CriticalSection &) = delete;
  CriticalSection &operator=(const CriticalSection &) = delete;
private:
  uint32_t saved_;
};
}

bool StepDirMotor::init(uint32_t counterClock_hz, uint32_t minFrequency_hz,
    uint32_t maxFrequency_hz, uint32_t maxPulses,
    uint32_t armAndDirectionHold_ms, bool forwardIsHigh, bool enaRunHigh)
{
  if (timer_ == nullptr || dirPort_ == nullptr || enaPort_ == nullptr ||
      dirPin_ == 0U || enaPin_ == 0U || counterClock_hz == 0U ||
      minFrequency_hz == 0U || maxFrequency_hz < minFrequency_hz ||
      maxPulses == 0U)
    return false;
  /* Restrict this implementation to general-purpose TIM2..TIM5, CH1. */
  if (timer_->Instance != TIM2 && timer_->Instance != TIM3 &&
      timer_->Instance != TIM4 && timer_->Instance != TIM5)
    return false;
  if (counterClock_hz / minFrequency_hz > 65535U ||
      counterClock_hz / maxFrequency_hz < 2U)
    return false;
  if ((timer_->Instance->CCMR1 & (TIM_CCMR1_CC1S | TIM_CCMR1_OC1M)) !=
      TIM_OCMODE_PWM2 || (timer_->Instance->CCER & TIM_CCER_CC1P) != 0U ||
      (timer_->Instance->CR1 & (TIM_CR1_DIR | TIM_CR1_CMS)) != 0U)
    return false;

  counterClock_hz_ = counterClock_hz;
  minFrequency_hz_ = minFrequency_hz;
  maxFrequency_hz_ = maxFrequency_hz;
  maxPulses_ = maxPulses;
  hold_ms_ = armAndDirectionHold_ms;
  forwardIsHigh_ = forwardIsHigh;
  enaRunHigh_ = enaRunHigh;

  CriticalSection lock;
  TIM_TypeDef *tim = timer_->Instance;
  CLEAR_BIT(tim->CR1, TIM_CR1_CEN | TIM_CR1_UDIS);
  tim->DIER = 0U; // This object owns the entire timer.
  SET_BIT(tim->CR1, TIM_CR1_OPM | TIM_CR1_URS | TIM_CR1_ARPE);
  MODIFY_REG(tim->CCMR1, TIM_CCMR1_OC1M, TIM_OCMODE_FORCED_INACTIVE);
  SET_BIT(tim->CCMR1, TIM_CCMR1_OC1PE);
  tim->CNT = 0U;
  tim->EGR = TIM_EGR_UG;
  tim->SR = 0U;
  SET_BIT(tim->CCER, TIM_CCER_CC1E); // Drive low even with CEN=0.
  forward_ = true;
  HAL_GPIO_WritePin(dirPort_, dirPin_, forwardIsHigh_ ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(enaPort_, enaPin_, enaRunHigh_ ? GPIO_PIN_RESET : GPIO_PIN_SET);
  ready_ = true;
  armed_ = busy_ = aborted_ = false;
  requested_ = completed_ = frequency_hz_ = 0U;
  idleSince_ms_ = HAL_GetTick();
  return true;
}

bool StepDirMotor::arm()
{
  CriticalSection lock;
  if (!ready_ || busy_)
    return false;
  if (!armed_)
  {
    idleSince_ms_ = HAL_GetTick();
    armed_ = true;
    HAL_GPIO_WritePin(enaPort_, enaPin_, enaRunHigh_ ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  return true;
}

bool StepDirMotor::setEnaRunHigh(bool high)
{
  CriticalSection lock;
  if (!ready_ || armed_ || busy_)
    return false;
  enaRunHigh_ = high;
  // STEP is already low; changing polarity changes the STOP output level too.
  HAL_GPIO_WritePin(enaPort_, enaPin_, enaRunHigh_ ? GPIO_PIN_RESET : GPIO_PIN_SET);
  idleSince_ms_ = HAL_GetTick();
  return true;
}

bool StepDirMotor::setDirection(bool forward)
{
  CriticalSection lock;
  if (!ready_ || busy_)
    return false;
  HAL_GPIO_WritePin(dirPort_, dirPin_,
      forward == forwardIsHigh_ ? GPIO_PIN_SET : GPIO_PIN_RESET);
  forward_ = forward;
  idleSince_ms_ = HAL_GetTick(); // Preserve DIR hold before a subsequent move.
  return true;
}

bool StepDirMotor::move(uint32_t pulses, uint32_t frequency_hz, bool forward)
{
  if (!ready_ || pulses == 0U || pulses > maxPulses_ ||
      frequency_hz < minFrequency_hz_ || frequency_hz > maxFrequency_hz_)
    return false;
  const uint32_t period = counterClock_hz_ / frequency_hz; // Outside IRQ mask.

  CriticalSection lock;
  if (!armed_ || busy_ ||
      static_cast<uint32_t>(HAL_GetTick() - idleSince_ms_) < hold_ms_)
    return false;

  TIM_TypeDef *tim = timer_->Instance;
  CLEAR_BIT(tim->CR1, TIM_CR1_CEN);
  tim->DIER = 0U;
  tim->CNT = 0U;
  tim->ARR = period - 1U;
  tim->CCR1 = period / 2U; // First half LOW (also DIR setup), second half HIGH.
  tim->EGR = TIM_EGR_UG;   // Load PSC/ARR/CCR before starting, no pulse counted.
  tim->SR = 0U;
  HAL_GPIO_WritePin(dirPort_, dirPin_,
      forward == forwardIsHigh_ ? GPIO_PIN_SET : GPIO_PIN_RESET);
  forward_ = forward;
  requested_ = pulses;
  completed_ = 0U;
  frequency_hz_ = frequency_hz;
  aborted_ = false;
  busy_ = true;
  MODIFY_REG(tim->CCMR1, TIM_CCMR1_OC1M, TIM_OCMODE_PWM2);
  SET_BIT(tim->DIER, TIM_DIER_UIE);
  SET_BIT(tim->CR1, TIM_CR1_CEN);
  return true;
}

void StepDirMotor::onPeriodElapsedISR()
{
  if (!ready_ || !busy_)
    return;
  /* HAL already cleared UIF. OPM already cleared CEN; CNT=0, STEP=LOW.
   * Delayed ISR => longer LOW gap, never uncontrolled extra pulses.
   */
  ++completed_;
  if (completed_ >= requested_ || !armed_)
  {
    CLEAR_BIT(timer_->Instance->DIER, TIM_DIER_UIE);
    busy_ = false;
    idleSince_ms_ = HAL_GetTick();
  }
  else
  {
    SET_BIT(timer_->Instance->CR1, TIM_CR1_CEN);
  }
}

void StepDirMotor::abort()
{
  CriticalSection lock;
  if (!ready_)
    return;
  TIM_TypeDef *tim = timer_->Instance;
  CLEAR_BIT(tim->CR1, TIM_CR1_CEN);
  tim->DIER = 0U;
  /* A completed pulse may be awaiting its update ISR. Account for it once. */
  if (busy_ && (tim->SR & TIM_SR_UIF) != 0U && completed_ < requested_)
    ++completed_;
  MODIFY_REG(tim->CCMR1, TIM_CCMR1_OC1M, TIM_OCMODE_FORCED_INACTIVE);
  tim->CNT = 0U;
  tim->SR = 0U;
  HAL_GPIO_WritePin(enaPort_, enaPin_, enaRunHigh_ ? GPIO_PIN_RESET : GPIO_PIN_SET);
  aborted_ = busy_;
  busy_ = armed_ = false;
  idleSince_ms_ = HAL_GetTick();
}

MotorSnapshot StepDirMotor::snapshot() const
{
  CriticalSection lock;
  const bool enaHigh = enaPort_ != nullptr && (enaPort_->ODR & enaPin_) != 0U;
  return {ready_, armed_, busy_, aborted_, forward_, enaHigh, enaRunHigh_,
          requested_, completed_, frequency_hz_};
}

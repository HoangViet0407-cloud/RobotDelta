#ifndef STEP_DIR_MOTOR_H
#define STEP_DIR_MOTOR_H

#include "stm32f4xx_hal.h"

struct MotorSnapshot
{
  bool ready;
  bool armed;
  bool busy;
  bool aborted;
  bool forward;
  bool enaCommandHigh; // GPIO output latch, NOT measured driver enable status.
  bool enaRunHigh;     // Configured RUN level; STOP uses the opposite level.
  uint32_t requested;
  uint32_t completed; // Complete timer pulses, NOT encoder counts.
  uint32_t frequency_hz; // Nominal frequency; ISR rearm adds a small gap.
};

/* One axis, exclusive general-purpose TIMx / CH1, PWM2 active-high.
 * CubeMX owns clock/pins/PWM configuration. init() adds single-shot OPM.
 * Hardware finishes each pulse and stops even if the CPU/ISR is delayed.
 * No heap, UART, global handle or project configuration inside this library.
 * Call onPeriodElapsedISR() ONLY from this timer's update callback.
 */
class StepDirMotor
{
public:
  constexpr StepDirMotor(TIM_HandleTypeDef *timer,
      GPIO_TypeDef *dirPort, uint16_t dirPin,
      GPIO_TypeDef *enaPort, uint16_t enaPin)
    : timer_(timer), dirPort_(dirPort), enaPort_(enaPort),
      dirPin_(dirPin), enaPin_(enaPin) {}

  bool init(uint32_t counterClock_hz, uint32_t minFrequency_hz,
            uint32_t maxFrequency_hz, uint32_t maxPulses,
            uint32_t armAndDirectionHold_ms, bool forwardIsHigh, bool enaRunHigh);
  bool arm();
  bool setEnaRunHigh(bool high); // Only while DISARMED and idle; never emits STEP.
  bool setDirection(bool forward); // Idle only; updates DIR without emitting STEP.
  bool move(uint32_t pulses, uint32_t frequency_hz, bool forward);
  void abort(); // Force STEP low, then ENA to STOP. May truncate one pulse.
  void onPeriodElapsedISR();
  MotorSnapshot snapshot() const;

private:
  TIM_HandleTypeDef *timer_;
  GPIO_TypeDef *dirPort_;
  GPIO_TypeDef *enaPort_;
  uint16_t dirPin_;
  uint16_t enaPin_;
  uint32_t counterClock_hz_ = 0;
  uint32_t minFrequency_hz_ = 0;
  uint32_t maxFrequency_hz_ = 0;
  uint32_t maxPulses_ = 0;
  uint32_t hold_ms_ = 0;
  bool forwardIsHigh_ = true;
  bool enaRunHigh_ = true;
  volatile bool ready_ = false;
  volatile bool armed_ = false;
  volatile bool busy_ = false;
  volatile bool aborted_ = false;
  volatile bool forward_ = true;
  volatile uint32_t requested_ = 0;
  volatile uint32_t completed_ = 0;
  volatile uint32_t frequency_hz_ = 0;
  volatile uint32_t idleSince_ms_ = 0;
};

#endif

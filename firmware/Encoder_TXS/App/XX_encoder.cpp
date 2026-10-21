#include "app.h"
#include "device.h"

/* ===================== COMPONENTS ===================== */
extern "C" TIM_HandleTypeDef htim1; // Handle CubeMX, khong tao timer thu hai.
static QuadratureEncoder encoder(&htim1, ENC_A_PORT, ENC_A_PIN, ENC_B_PORT, ENC_B_PIN);

/* ===================== FUNCTIONS ====================== */
bool XX_initEncoder()
{
  dev.encoderReady.store(false, std::memory_order_release);
  dev.encoderZeroPending = false;
  dev.encoder = &encoder;
  __HAL_DBGMCU_FREEZE_TIM1();
  const bool ok = encoder.init() == HAL_OK;
  // Publish the initialized pointer/object before allowing SysTick to use it.
  dev.encoderReady.store(ok, std::memory_order_release);
  return ok;
}

void XX_serviceEncoder()
{
  // Main context only: never wait for SysTick, and never print inside the ISR.
  if (dev.encoderZeroPending && !dev.encoder->zeroPending())
  {
    dev.encoderZeroPending = false;
    XX_resetReportWindow();
    XX_write("ZERO: relative origin set; this is NOT homing\r\n");
  }
}

/* ===================== CALLBACKS ====================== */
extern "C" void appTick1ms(void)
{
  if (dev.encoderReady.load(std::memory_order_acquire))
    dev.encoder->updateISR();
}

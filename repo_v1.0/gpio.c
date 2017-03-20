#include "gpio.h"
#include "em_cmu.h"

/* Function: GPIO_Init(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *    Initialize the GPIO pins for LED0 and LED1.
 */
void GPIO_Init(void)
{
  /* Init the clock */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Setup the config. for LED0 */
  GPIO_DriveModeSet(LED_PORT, gpioDriveModeStandard);
  GPIO_PinModeSet(LED_PORT, LED_0_PIN, gpioModePushPull, 0);

  /* Setup the config. for LED1 */
  GPIO_DriveModeSet(LED_PORT, gpioDriveModeStandard);
  GPIO_PinModeSet(LED_PORT, LED_1_PIN, gpioModePushPull, 0);

  /* Turn Off the LED for now */
  GPIO_PinOutToggle(LED_PORT, LED_0_PIN);
  GPIO_PinOutToggle(LED_PORT, LED_1_PIN);

  return;
}

/* Specify the GPIO pins:
 *  Power Pin: PD0
 *  Peripheral Interrupt: PD1
 *  SDA: PC4
 *  SCL: PC5
 */
void Set_I2C_GPIO_Pins(void)
{

  /* Enable the GPIO clock */
  //CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Set the power pin -- PD0 */
  GPIO_PinModeSet(I2C_GPIO_POWER_PORT, I2C_POWER_PIN, gpioModePushPull, 0);
  GPIO_PinOutClear(I2C_GPIO_POWER_PORT, I2C_POWER_PIN);

  
  /*XXX: Re-visit for Power Modules */
  //GPIO_PinOutSet(I2C_GPIO_POWER_PORT, I2C_POWER_PIN);

  /* Set the pin for the peripheral interrupt */
  /* The interrupt peripheral should be intput because you have to receive the value from
   * of that particular pin.
   */
  GPIO_PinModeSet(I2C_GPIO_INT_PORT, I2C_INT_PIN, gpioModeInput, 0);

  /* Set the pin for SDA */
  GPIO_PinModeSet(I2C_SDA_PORT, I2C_SDA_PIN,gpioModeWiredAnd, 1);
  GPIO_PinOutSet(I2C_SDA_PORT, I2C_SDA_PIN);
  
  /* Set the pin for SCL */
  GPIO_PinModeSet(I2C_SCL_PORT, I2C_SCL_PIN,gpioModeWiredAnd, 1);
  GPIO_PinOutSet(I2C_SCL_PORT, I2C_SCL_PIN);

  return;
}




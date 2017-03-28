#include "light_sensor.h"

/* Function: Light_Sensor_Init(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *    This function initializes the Light Sensor sense and excite paths
 */
void Light_Sensor_Init(void)
{
  GPIO_DriveModeSet(LS_SENSE_PORT, gpioDriveModeStandard);
  GPIO_DriveModeSet(LS_EXCITE_PORT, gpioDriveModeStandard);

  GPIO_PinModeSet(LS_SENSE_PORT, LS_PIN, gpioModeDisabled, 0);
  GPIO_PinModeSet(LS_EXCITE_PORT, LS_PIN, gpioModePushPull, 1);

  return;
}



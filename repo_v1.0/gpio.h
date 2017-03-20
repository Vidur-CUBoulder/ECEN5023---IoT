#include "em_gpio.h"

/* LED 0 */
#define LED_PORT gpioPortE
#define LED_0_PIN 2
#define LED_1_PIN 3

/* I2C GPIO Pins */
#define I2C_GPIO_POWER_PORT gpioPortD 
#define I2C_POWER_PIN 0

#define I2C_GPIO_INT_PORT gpioPortD
#define I2C_INT_PIN 1

#define I2C_SDA_PORT gpioPortC
#define I2C_SDA_PIN 4

#define I2C_SCL_PORT gpioPortC
#define I2C_SCL_PIN 5

void GPIO_Init(void);

void Set_I2C_GPIO_Pins(void);




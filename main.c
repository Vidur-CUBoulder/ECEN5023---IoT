/***************************************************************************//**
 * @file main.c
 * @brief Energy Management Unit (EMU) Peripheral API
 * @version 5.0.0
 *******************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
/********* CREDITS **********
 * For the functions/variables:
 * 					  void blockSleepMode(SLEEP_EnergyMode_t eMode)
 * 					  void unblockSleepMode(SLEEP_EnergyMode_t eMode)
 * 					  void sleep(void)
 * 					  static int8_t sleep_block_counter[NUM_SLEEP_MODES] = {0};
 *
 * CREDITS: Prof. Keith A. Graham
 * 			Lecture 3; ECEN5023-001
 *
 * enum SLEEP_EnergyMode_t
 * CREDITS: Silicon Labs
 *
 * Any other functions and/or variables except the definition
 * of the IRQ handler are all credited to Silicon Labs
 *
 */

#include "sleep_modes.h"
#include "letimer.h"

#include "em_dma.h"
#include "dmactrl.h"
#include "acmp.h"
#include "gpio.h"
#include "light_sensor.h"
#include "em_i2c.h"

#define DEBUG

/* To toggle the DMA transfer routines from the code */
#define WITHOUT_DMA

/* Macro used to toggle the ULFRCO calibration */
#define Calibrate_ULFRCO

/* Select the sleep mode to run the MCU in */
#define SEL_SLEEP_MODE sleepEM2

#define HIGH_LEVEL 61
#define TIMER_PERIOD 2.5

/* Misc. */

#define CYCLE_PERIOD 	  4.25
#define ON_PERIOD         0.004 
#define LETIMER_MAX_CNT   65535 
#define IDEAL_ULFRCO_CNT  1000

/*DMA*/
#define ADC0_DMA_Channel 0
#define MAX_CONVERSION 750
#define ADC_SLEEP_MODE sleepEM1
#define DEF_HPROT_VAL 0
#define CALC_PRESCALE_VAL 49
#define CONFIG_ADC_CHNL acmpChannel6

/* Temp. Sensor */
#define LOWER_TEMP_BOUND 15
#define UPPER_TEMP_BOUND 35
#define SET_TEMP_GRADIENT (-6.27)

/* I2C Macros */
#define I2C_READ 1
#define I2C_WRITE 0
#define I2C_SLAVE_ADDR 0x39
#define WAIT_FOR_SLAVE_ACK {\
                     while((I2C1->IF & I2C_IF_ACK) == 0);\
                     I2C1->IFC = I2C_IFC_ACK;\
                     }
#define MASTER_STOP {\
                      I2C1->CMD = I2C_CMD_STOP;\
                      while((I2C1->IF & I2C_IF_MSTOP) == 0);\
                      I2C1->IFC = 0xFF;\
                    }

/* Peripheral Macros */
#define REG_CONTROL 0x00
#define REG_TIMING 0x01
#define REG_THRESHLOWLOW 0x02
#define REG_THRESHLOWHIGH 0x03
#define REG_THRESHHIGHLOW 0x04
#define REG_THRESHHIGHHIGH 0x05
#define REG_INTERRUPT 0x06
#define REG_CRC 0x08
#define REG_ID 0xA
#define REG_DATA0LOW 0xC
#define REG_DATA0HIGH 0xD
#define REG_DATA1LOW 0xE
#define REG_DATA1HIGH 0xF

/* Misc. I2C Macros */


/* Global variables for use in the interrupt handler */
uint16_t irq_flag_set;
unsigned int acmp_value;
float output;
volatile uint16_t ADC0_DMArambuffer[MAX_CONVERSION] = {0};
int32_t conversion_val = 0;

static int8_t cycle_count = 0;

/* Function: convertToCelsius(int32_t adcSample)
 * Parameters:
 *    adcSample: pass the value returned by the ADC to this function
 * Return:
 *    - a value of the temperature converted to celsius
 * Description:
 *    - Use this function to convert the value that is read from the 
 *      adc to a celsius value in floating point.
 * IP Credits: 
 *      This routine is credited to Silicon Labs
 */
float convertToCelsius(int32_t adcSample)
{
  float temp = 0;

  /* Factory calibration of temperature from the device information page */
  float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK) >>\
                                _DEVINFO_CAL_TEMP_SHIFT);

  float cal_value_0 = (float)((DEVINFO->ADC0CAL2\
                              & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)\
                              >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

  /* Temperature gradient(from the datasheet) */
  float gradient = SET_TEMP_GRADIENT;

  temp = (cal_temp_0 - ((cal_value_0 - adcSample)/gradient));

  return temp;
}

/* Function: Get_Avg_Temperature(void)
 * Parameters:
 *    void
 * Return:
 *    - The average temperature value from amongst all the 
 *      values read from the ADC.
 * Description:
 *    - Use this function to get an average temperature reading 
 *      from amongst all the ADC values read. 
 */
float Get_Avg_Temperature(void)
{

  int16_t cnt = 0;

  /* Start the ADC count */
  ADC_Start(ADC0, adcStartSingle);

  while(cnt != MAX_CONVERSION) {
	while(!(ADC0->IF & ADC_IFS_SINGLE));

    /* Wait for the single conversion to complete */
    while(!(ADC0->IF & ADC_IFS_SINGLE));

    /*Clear the flag */
    ADC_IntClear(ADC0, ADC_IFC_SINGLE);

    /*Get the value*/
    conversion_val += ADC0->SINGLEDATA;

    cnt ++;
  }

  /* Stop the ADC conversion */
  ADC0->CMD = ADC_CMD_SINGLESTOP;

  /* ADC work done; Exit EM1 */
  unblockSleepMode(ADC_SLEEP_MODE);

  /* Get the average */
  conversion_val = conversion_val/MAX_CONVERSION;

  /* Return the value in Celsius */
  return (convertToCelsius(conversion_val));

}

/* Function:cb_ADC0_DMA(unsigned int channel, bool primary, void *user)
 * Parameters:
 *    - channel - the DMA channel over which the data is being transfered.
 *    - primary - whether or not the primary channel of the DMA is selected.
 *    - user - a user defined pointer to provide with the callback function.
 * Return:
 *    void
 * Description:
 *    This is the callback function that will get called once the DMA transfer
 *    has completed.
 */
void cb_ADC0_DMA(unsigned int channel, bool primary, void *user)
{
  
  uint16_t cnt = 0;
  uint32_t sum = 0;

  INT_Disable();

  /* Clear the DMA interrupt */
  DMA_IntClear(ADC0_DMA_Channel);

  /* Stop the ADC */
  ADC0->CMD = ADC_CMD_SINGLESTOP;
 
  /* unblock the EM1 sleep now, ADC ops done! */
  unblockSleepMode(ADC_SLEEP_MODE);

  /*Store the values in the DMA buffer */
  while(cnt != MAX_CONVERSION) {
    sum += ADC0_DMArambuffer[cnt++];
  }

  /*Get the average and convert to celsius */
  sum = sum/MAX_CONVERSION;

  float C_temp = convertToCelsius(sum);
  
  if ((C_temp < LOWER_TEMP_BOUND) || (C_temp > UPPER_TEMP_BOUND)) {
    /*Turn on the LED*/
    GPIO_PinOutSet(LED_PORT,LED_1_PIN);
  } else {
    /* Turn off the LED */
    GPIO_PinOutClear(LED_PORT,LED_1_PIN);
  }

  INT_Enable();

  return;
}

/* This is a global definition for the DMA callback function
 * configuration.
 */
static DMA_CB_TypeDef dma_cb_config = {
  .cbFunc = cb_ADC0_DMA,
  .userPtr = NULL,
  .primary = true
};

/* Function: ADC0_DMA_Setup(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *    - config. the ADC0 to run with/without DMA enabled
 */
void ADC0_DMA_Setup(void)
{
  /* Setup the initial DMA configuration */
  static DMA_CfgDescr_TypeDef dma_cfgdescr = {
    .arbRate = dmaArbitrate1,
    .dstInc = dmaDataInc2,
    .hprot = DEF_HPROT_VAL,
    .size = dmaDataSize2,
    .srcInc = dmaDataIncNone
  };

  DMA_CfgDescr(ADC0_DMA_Channel, true, &dma_cfgdescr);

  /* Setup the DMA channel */
  static DMA_CfgChannel_TypeDef dma_chnldescr = {
    .cb = &dma_cb_config,
	 .enableInt = true,
    .highPri = true,
    .select = DMAREQ_ADC0_SINGLE
  };

  /*Call the initialization functions */
  DMA_CfgChannel(ADC0_DMA_Channel, &dma_chnldescr);

  /*Clear and enable the DMA interrupt */
  DMA_IntClear(ADC0_DMA_Channel);
  DMA_IntEnable(ADC0_DMA_Channel);

  /* Initialize the main DMA setup */
  DMA_ActivateBasic(ADC0_DMA_Channel, true, false, (void *)ADC0_DMArambuffer,\
                      (void*)&(ADC0->SINGLEDATA), (MAX_CONVERSION-1));

  return;
}

/* Function: DMA_Initialize(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *    - Start the initialization process for the DMA.
 */
void DMA_Initialize(void)
{
  /* Initialize the structure */
  static DMA_Init_TypeDef Init_DMA = {
    .controlBlock = dmaControlBlock,
    .hprot = DEF_HPROT_VAL
  };
  
  DMA_Init(&Init_DMA);

  /* Setup the ADC to work with DMA */
  ADC0_DMA_Setup();
  
  /* Enable NVIC for the DMA*/
  NVIC_EnableIRQ(DMA_IRQn);

  return;
}

void Clear_Interrupt_Bit(void)
{
  /* Enable the peripheral for writing */
  I2C1->TXDATA = ((I2C_SLAVE_ADDR << 1) | I2C_WRITE);

  /* Start the communication over the SDA */
  for(int i=0; i<1000; i++);

  I2C1->CMD = I2C_CMD_START;

  WAIT_FOR_SLAVE_ACK;
  
  /* II. Loading the command register */
  I2C1->TXDATA = (0xC);

  /* Next, wait for the slave to respond */
  WAIT_FOR_SLAVE_ACK;
  
  /* IV. Start the STOP procedure */
  MASTER_STOP;
  
  return;
}

void Write_to_I2C_Peripheral(uint8_t addr, int8_t write_data)
{

  /* I.
   * Put the R/W bit along with the Slave addr. 
   * on the TXADDR register
   */
  I2C1->TXDATA = ((I2C_SLAVE_ADDR << 1) | I2C_WRITE);  

  /* Start the communication over the SDA */
  for(int i=0; i<1000; i++);

  I2C1->CMD = I2C_CMD_START;

  WAIT_FOR_SLAVE_ACK;

  /* II. Loading the command register */
  I2C1->TXDATA = (0x80 | addr);

  /* Next, wait for the slave to respond */
  WAIT_FOR_SLAVE_ACK;
  
  /* III. Sending the Data over to the peripheral register */
  I2C1->TXDATA = write_data;

  /* Next, wait for the slave to respond */
  WAIT_FOR_SLAVE_ACK;
 
  /* IV. Start the STOP procedure */
  MASTER_STOP;

  return;
}

void Power_On_Reset(void)
{
  /* Threshold Low register */
	Write_to_I2C_Peripheral(REG_THRESHLOWLOW, 0x0F);
  Write_to_I2C_Peripheral(REG_THRESHLOWHIGH, 0x00);

  /* Threshold High register */
  Write_to_I2C_Peripheral(REG_THRESHHIGHLOW, 0x00);
  Write_to_I2C_Peripheral(REG_THRESHHIGHHIGH, 0x08);

  /* Set the persistance value to 4 */
  Write_to_I2C_Peripheral(REG_INTERRUPT, 0x14);

  /* Set the Integration time to 101ms and LOW gain */
  Write_to_I2C_Peripheral(REG_TIMING, 0x11);

  return;
}

int8_t Read_from_I2C_Peripheral(int8_t addr)
{
  int8_t ret_data = 0;

  /* I. First pass the address of the register that you want to
   * read the data from */
  I2C1->TXDATA = ((I2C_SLAVE_ADDR << 1) | I2C_WRITE);

  /* Start the communication */
  I2C1->CMD = I2C_CMD_START;

  /* Next, wait for the slave to respond */
  WAIT_FOR_SLAVE_ACK;
  
  /* II. Send the address that you want to read */
  I2C1->TXDATA = (0x80 | addr);

  /* Next, wait for the slave to respond */
  WAIT_FOR_SLAVE_ACK;

  /* Re-start */
  I2C1->CMD = I2C_CMD_START;

  /* Change the config for read */
  I2C1->TXDATA = ((I2C_SLAVE_ADDR << 1) | I2C_READ);

  /* Next, wait for the slave to respond */
  WAIT_FOR_SLAVE_ACK;

  /*III. Get the data from the RX buffer */
  while((I2C1->IF & I2C_IF_RXDATAV) == 0);
  ret_data = I2C1->RXDATA;

  /*IV. Stop the data transfer from the slave */
  I2C1->CMD = I2C_CMD_NACK;

  MASTER_STOP;

  return ret_data;
}


void Power_Up_Peripheral(void)
{
  /* Turn on the GPIO pin */
  GPIO_PinOutSet(I2C_GPIO_POWER_PORT, I2C_POWER_PIN);
  
  /* Wait for some time */
  for(int i = 0; i<1000; i++);

  /* Do the misc. power on reset config here! */

  /* Write to the command register - peripheral */
  Write_to_I2C_Peripheral(REG_CONTROL, 0x03);

  Power_On_Reset();

#if 0
  int8_t read_val = 0;
  read_val = Read_from_I2C_Peripheral(REG_CONTROL);
#endif

  return;
}

void Power_Down_Peripheral(void)
{

  Clear_Interrupt_Bit();
  /* Clear the interrupts */
  //Write_to_I2C_Peripheral(REG_INTERRUPT, CLEAR_INTERRUPT_PIN);

	/* Push the power down to peripheral */
	Write_to_I2C_Peripheral(REG_CONTROL, 0x00);

	/* Clear the pin */
	GPIO_PinOutClear(I2C_GPIO_POWER_PORT, I2C_POWER_PIN);

	return;
}


void Setup_I2C_Peripheral(void)
{

  /* First setup the GPIO pins */
  //Set_I2C_GPIO_Pins();

  /*Next, enable the I2C clock */
  CMU_ClockEnable(cmuClock_I2C1, true);

  /* Initialize the I2C structures */
  I2C_Init_TypeDef init_I2C_1 = {
    .enable = true,
    .master = true, 
    .refFreq = 0,
    .freq = I2C_FREQ_STANDARD_MAX,
    .clhr = i2cClockHLRStandard
  };

  /* Next, set the route for I2C */
  I2C1->ROUTE = (I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN |\
		  	  	  I2C_ROUTE_LOCATION_LOC0);

  /* Initialize the I2C peripheral 
   * NOTE: This will not enable the I2C
   */
  I2C_Init(I2C1, &init_I2C_1);

  /*enable the I2C*/
  I2C_Enable(I2C1, true);
  
  /* Check for the busy state and reset the bus if true */
  if(I2C1->STATE & I2C_STATE_BUSY) {
    I2C1->CMD = I2C_CMD_ABORT;
  }

  /* Clear any interrupts from the I2C that may have been
   * inadvertently set.
   */
  I2C1->IFC = 0xFFFF;
  
  /* Enable the interrupts */
  I2C1->IEN = (I2C_IEN_ACK | I2C_IEN_NACK | I2C_IEN_MSTOP);
  
  return;
}

void Setup_GPIO_Interrupts(void)
{
  /* Clear all flags */
  GPIO->IFC = 0xFFFF;

  /* enable the external interrupts for GPIO */
  GPIO_ExtIntConfig(I2C_GPIO_INT_PORT,\
                    I2C_INT_PIN,\
                    1,\
                    false,\
                    true,\
                    true);     
  
  /* Enable the interrupt hadler for GPIO */
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

  return;
}

void Dump_All_Register_Values(void)
{
  int8_t temp_data[15] = {0};

  temp_data[0] = Read_from_I2C_Peripheral(REG_TIMING);
  temp_data[1] = Read_from_I2C_Peripheral(REG_THRESHLOWLOW);
  temp_data[2] = Read_from_I2C_Peripheral(REG_THRESHLOWHIGH);
  temp_data[3] = Read_from_I2C_Peripheral(REG_THRESHHIGHLOW);
  temp_data[4] = Read_from_I2C_Peripheral(REG_THRESHHIGHHIGH);
  temp_data[5] = Read_from_I2C_Peripheral(REG_INTERRUPT);
  temp_data[6] = Read_from_I2C_Peripheral(REG_CRC);
  temp_data[7] = Read_from_I2C_Peripheral(REG_ID);
  temp_data[8] = Read_from_I2C_Peripheral(REG_DATA0LOW);
  temp_data[9] = Read_from_I2C_Peripheral(REG_DATA0HIGH);
  temp_data[10] = Read_from_I2C_Peripheral(REG_DATA1LOW);
  temp_data[11] = Read_from_I2C_Peripheral(REG_DATA1HIGH);
  
  return;
}


/* Function: LETIMER0_IRQHandler(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *  - This is the global LETIMER0 IRQ handler definition.
 *  - Handles the general reading of the value from the ACMP 
 *  - Toggles the LED based on the value of the ACMP
 *  - Handles the toggle to LED1 after sensing the temperature
 */
void LETIMER0_IRQHandler(void)
{
  INT_Disable();
  
  irq_flag_set = LETIMER_IntGet(LETIMER0);

  /* If COMP1 flag is set */
  if(irq_flag_set & LETIMER_IF_COMP1) {

    /* Clear the COMP1 Interrupt Flag*/
    LETIMER0->IFC |= LETIMER_IFC_COMP1;

    /* Keep the ACMP0 enabled */
    ACMP0->CTRL |= ACMP_CTRL_EN;

    /* Enable the excitation */
    GPIO_PinOutSet(LS_EXCITE_PORT,LS_PIN);

    /* Wait for the warm-up to complete */
    while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT));
  } else { /* COMP0 flag is set */

#ifdef TEMPERATURE_SENSOR_ENABLE
    /* Add the functionality for the temperature sensor */
#ifdef WITHOUT_DMA
    /*Move out of the EM3 mode */
    blockSleepMode(ADC_SLEEP_MODE);
    
    /* Get the average temperature of the MCU */
    output = Get_Avg_Temperature();

    if ((output < LOWER_TEMP_BOUND) || (output > UPPER_TEMP_BOUND)) {
      /*Turn on LED1 */
      GPIO_PinOutSet(LED_PORT,LED_1_PIN);
    } else {
      /*Turn off LED1 */
      GPIO_PinOutClear(LED_PORT,LED_1_PIN);
    }

    /* ADC work done; Exit EM1 */
    unblockSleepMode(ADC_SLEEP_MODE);
#else

    /* Setup the DMA */
    DMA_Initialize();

    /* Block to EM1 */
    blockSleepMode(ADC_SLEEP_MODE);

    /* Initialize the ADC */
    ADC_Start(ADC0, adcStartSingle);

#endif
#endif

//#ifdef GPIO_INTERRUPT_SET
#if 1
    cycle_count++;
    if(cycle_count == 1){

      /* Setup the Pins and the I2C peripheral */ 
      Setup_I2C_Peripheral();
      Setup_GPIO_Interrupts();

      /* Next power up the peripheral and 
       * set the registers on it */
      Power_Up_Peripheral();

      Dump_All_Register_Values();

    } else if (cycle_count == 2) {
      /* Do nothing here just wait */
    } else {
      /* Disable the interrupts and stop the peripheral */
      Power_Down_Peripheral();
      
      /* reset the counter */
      cycle_count = 0;
    }

#endif

    /* Read the ACMP0 value and disable it */
    acmp_value = (ACMP0->STATUS & ACMP_STATUS_ACMPOUT);
    ACMP0->CTRL &= ~ACMP_CTRL_EN;

    /* Disable the excitation and Clear the interrupt */
    GPIO_PinOutClear(LS_EXCITE_PORT,LS_PIN);
    LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP0);

    if(acmp_value) {
      if(acmpinit.vddLevel == LOW_LEVEL)
      {
        /* Raise the level */
        acmpinit.vddLevel = HIGH_LEVEL;
        /* Initialize and set the channel for ACMP */
        ACMP_Init(ACMP0,&acmpinit);		
        ACMP_ChannelSet(ACMP0, acmpChannelVDD, CONFIG_ADC_CHNL);
        /* Set the LED */
        GPIO_PinOutSet(LED_PORT,LED_0_PIN);
      }
      else
      {
        /* Lower the Level */
        acmpinit.vddLevel = LOW_LEVEL;
        /* Initialize and set the channel for the ACMP */
        ACMP_Init(ACMP0,&acmpinit);
        ACMP_ChannelSet(ACMP0, CONFIG_ADC_CHNL, acmpChannelVDD);
        /* Clear the LED */
        GPIO_PinOutClear(LED_PORT,LED_0_PIN);
      }
    }
  }
  
  INT_Enable();
}

/* Function: int32_t Calc_Prescaler(int32_t *cycle_period, int32_t *on_period)
 * Parameters: 
 *    int32_t *cycle_period - period of the net cycle of the waveform
 *    int32_t *on_period - the period for which the peripheral should remain on
 * Return:
 *    Returns the prescaler value 
 * Description:
 *    A generic prescaler calculation module
 */
int32_t Calc_Prescaler(int32_t *cycle_period, int32_t *on_period) 
{
  
  int32_t prescaler = 0;
  
  while(*cycle_period > LETIMER_MAX_CNT) {
    prescaler++;							
    *cycle_period = *cycle_period/2;
    *on_period = *on_period/2;
  }

  return prescaler;
}

/******************** I2C Light Sensor Functions *****************/




/* Function: LETIMER0_Config_Cluster(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *    - Setup the default configuration of the LETIMER
 *    - Similar to the configurations done in assignment 3
 */
void LETIMER0_Config_Cluster(void)
{
  
  /* Initialize the GPIO and the Ambient Light Sensor */
  GPIO_Init();
  Light_Sensor_Init();
 
  /* Enable the clock for HFPERCO */
  //CMU_ClockEnable(cmuClock_HFPER, true);
  
  /*Enable the clock for the DMA */
  CMU_ClockEnable(cmuClock_DMA, true);

  /* Do the basic clock setup for the LFXO/ULFRCO clock */
#if (SEL_SLEEP_MODE == sleepEM3)
  LETIMER_ClockSetup(cmuSelect_ULFRCO);
#else
  LETIMER_ClockSetup(cmuSelect_LFXO);
#endif
  
  /* Setup and Enable the Clocks and all for ACMP0 */
  Setup_Enable_ACMP0();

  /* Continue with the LETIMER Setup */
  /* Disable the LETIMER before initializing it */
  LETIMER_Enable(LETIMER0, false);

  /* Initialize the LETIMER default structure values */
  const LETIMER_Init_TypeDef letimerInit = {
    .enable         = true,              /* Enable timer when init complete. */
    .debugRun       = false,             /* Stop counter during debug halt. */
    .rtcComp0Enable = false,             /* Do not start counting on RTC COMP0 match. */
    .rtcComp0Enable = false,             /* Do not start counting on RTC COMP1 match. */
    .comp0Top       = true,              /* Load COMP0 into CNT on underflow. */
    .bufTop         = false,             /* Do not load COMP1 into COMP0 when REP0 reaches 0. */
    .out0Pol        = IDLE_OUT_0,        /* Idle value 0 for output 0. */
    .out1Pol        = IDLE_OUT_1,        /* Idle value 0 for output 1. */
    .ufoa0          = letimerUFOANone,   /* No action on underflow on output 0. */
    .ufoa1          = letimerUFOANone,   /* No action on underflow on output 1. */
    .repMode        = letimerRepeatFree  /* Count until stopped by SW. */
  };

  /* Keep it disabled for the moment */
  LETIMER_Enable(LETIMER0, false);

#ifdef Calibrate_ULFRCO
  
  /* First Get the ratio */
  float ratio = Get_Osc_Ratio();

  /* Change the COMP0 and COMP1 values accordingly */
  int32_t cycle_period = ratio * (IDEAL_ULFRCO_CNT * CYCLE_PERIOD);
  int32_t on_period = ratio * (IDEAL_ULFRCO_CNT * ON_PERIOD);

#else

  int32_t cycle_period = 0;
  int32_t on_period = 0;

#ifdef PRESCALE_LFXO
  int32_t prescaler = Calc_Prescaler(&cycle_period, &on_period);

  /* Set the prescaler for the LFXO clk */
  CMU->LFAPRESC0 = (prescaler << 8);
#endif

  cycle_period = (IDEAL_ULFRCO_CNT * CYCLE_PERIOD);
  on_period = (IDEAL_ULFRCO_CNT * ON_PERIOD);

#endif

  /* Fill the Values in the Comparators */
  LETIMER_CompareSet(LETIMER0, 0, cycle_period);
  LETIMER_CompareSet(LETIMER0, 1, on_period);

  /* Clear all interrupts */
  LETIMER0->IFC = 0xFFFF;
 
  /* Enable the interrupts for COMP0 and COMP1 */
  LETIMER0->IEN = (LETIMER_IEN_COMP0 | LETIMER_IEN_COMP1);

  /* Set the NVIC to trigger the LETIMER0 interrupt */
  NVIC_EnableIRQ(LETIMER0_IRQn);
  
  /*Initialize the LETIMER and Enable it */ 
  LETIMER_Init(LETIMER0, &letimerInit);
  LETIMER_Enable(LETIMER0, true);

  return;
}

/* Function: ADC0_Init(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *    - Initialize the ADC0 and setup its default configurations
 */
void ADC0_Init(void)
{

  /* Enable the clock for HFPERCO */
  //CMU_ClockEnable(cmuClock_HFPER, true);
  
  /* Enable the clock to the ADC */
  CMU_ClockEnable(cmuClock_ADC0, true);

  /* Do the timebase calculation */
  int8_t calc_timebase = ADC_TimebaseCalc(CMU_ClockFreqGet(cmuClock_HFPER));

  ADC_Init_TypeDef adc_init = {
    .ovsRateSel       = adcOvsRateSel2, 
    .lpfMode          = adcLPFilterBypass,
    .warmUpMode       = adcWarmupNormal, 
    .timebase         = calc_timebase,
    .prescale         = CALC_PRESCALE_VAL,
    .tailgate         = false
  };

  /* Initialize the ADC */
  ADC_Init(ADC0, &adc_init);
  
  /* Config. the Single Input ADC mode */
  ADC_InitSingle_TypeDef adc_single_init = {
    .acqTime = adcAcqTime2,
    .diff = false,
    .input = adcSingleInputTemp,
    .leftAdjust = false,
    .prsEnable = false,
    .reference = adcRef1V25,
    .rep = true,
    .resolution = adcRes12Bit
  };
 
  /* Initialize the Single Input ADC Mode */
  ADC_InitSingle(ADC0, &adc_single_init);

  /* Do the ADC interrupt configuration */
  
  /* Clear all the ADC interrupts */
  ADC0->IFC = 0xFF;

  /* Enabling the requisite interrupts */
  ADC0->IEN = ADC_IFS_SINGLE;//0x1; //set the SINGLE bit in the register  

  return;
}

int8_t GPIO_IRQ_flag = 0;
int8_t adc_low = 0;
int8_t adc_high = 0;

void GPIO_ODD_IRQHandler(void)
{
  INT_Disable();

  /* Read the GPIO interrupt flags */
  GPIO_IRQ_flag = GPIO_IntGet();

  if(GPIO_IRQ_flag & 0x2) {
    /* Read the value from the ADC on the peripheral */
    adc_low = Read_from_I2C_Peripheral(REG_DATA0LOW);
    adc_high = Read_from_I2C_Peripheral(REG_DATA0HIGH);
    
    /* Clear the GPIO IF flags */
    GPIO->IFC = 0xFFFF;
  }
    
  INT_Enable();
}



/* Function: main(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *    This is the main function. This is where the execution starts from.
 */
int main(void)
{
  /* Chip errata */
  CHIP_Init();
  
  CMU_ClockEnable(cmuClock_HFPER, true);
  
  Set_I2C_GPIO_Pins();

  LETIMER0_Config_Cluster();
  
  ADC0_Init();

  


#if 0
  Write_to_I2C_Peripheral(REG_TIMING, 0x03);
  int8_t reg_out = Read_from_I2C_Peripheral(REG_TIMING);
#endif

  blockSleepMode(SEL_SLEEP_MODE);
  
  /* Infinite loop */
  while (1) {
	  sleep();
  }
}

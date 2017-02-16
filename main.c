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

//#include "em_adc.h"
#include "em_acmp.h"
#include "em_dma.h"
#include "dmactrl.h"

//#define WITHOUT_DMA

/* Macro used to toggle the ULFRCO calibration */
#define Calibrate_ULFRCO

/* Select the sleep mode to run the MCU in */
#define SEL_SLEEP_MODE sleepEM3

#define LOW_LEVEL 2
#define HIGH_LEVEL 61
#define TIMER_PERIOD 2.5

/* LED 0 */
#define LED_PORT gpioPortE
#define LED_0_PIN 2
#define LED_1_PIN 3

/* Light Sensor */
#define LS_EXCITE_PORT    gpioPortD
#define LS_SENSE_PORT     gpioPortC
#define LS_PIN            6

/* Misc. */
//#define CYCLE_PERIOD      2.5
#define CYCLE_PERIOD 	  5.5
#define ON_PERIOD         0.004 
#define LETIMER_MAX_CNT   65535 
#define IDEAL_ULFRCO_CNT  1000

/*DMA*/
#define ADC0_DMA_Channel 0
#define MAX_CONVERSION 750

/* Global variables for use in the interrupt handler */
uint16_t irq_flag_set;
unsigned int acmp_value;
float output;
volatile uint16_t ADC0_DMArambuffer[MAX_CONVERSION] = {0};

/* Global structure definition for the Analog Comparator */
static ACMP_Init_TypeDef acmpinit =
{
  .fullBias                 = false,
  .halfBias                 = true,
  .biasProg                 = 0x0,
  .interruptOnFallingEdge   = false,
  .interruptOnRisingEdge    = false,
  .warmTime                 = acmpWarmTime256,
  .hysteresisLevel          = acmpHysteresisLevel4,
  .inactiveValue            = false,
  .lowPowerReferenceEnabled = false,
  .vddLevel                 = LOW_LEVEL,
  .enable                   = true
};

/* Function: GPIO_Init(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *    Initialize the GPIO pins for LED0.
 */
void GPIO_Init(void)
{
  /* Init the clock */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Setup the config. for LED0 */
  GPIO_DriveModeSet(LED_PORT, gpioDriveModeStandard);
  GPIO_PinModeSet(LED_PORT, LED_0_PIN, gpioModePushPull, 1);

  /* Setup the config. for LED1 */
  GPIO_DriveModeSet(LED_PORT, gpioDriveModeStandard);
  GPIO_PinModeSet(LED_PORT, LED_1_PIN, gpioModePushPull, 1);

  /* Turn Off the LED for now */
  GPIO_PinOutToggle(LED_PORT, LED_0_PIN);
  GPIO_PinOutToggle(LED_PORT, LED_1_PIN);

  return;
}

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


/* credit silicon labs for this function ! */
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
  float gradient = -6.27;

  temp = (cal_temp_0 - ((cal_value_0 - adcSample)/gradient));

  return temp;
}

float Get_Avg_Temperature(void)
{

  int32_t conversion_val = 0;
  int16_t cnt = 0;

  /* Start the ADC count */
  ADC_Start(ADC0, adcStartSingle);

  while(cnt != MAX_CONVERSION) {

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

  /* Get the average */
  conversion_val = conversion_val/MAX_CONVERSION;

  /* Return the value in Celsius */
  return (convertToCelsius(conversion_val));

}

/*Define the callback function */
void cb_ADC0_DMA(unsigned int channel, bool primary, void *user) //<--- Check and experiment!!
{
  
  uint16_t cnt = 0;
  uint32_t sum = 0;

  INT_Disable();

  /* Clear the DMA interrupt */
  DMA_IntClear(ADC0_DMA_Channel);

  /* Initialize the Single-Stop DMA transfer */
  ADC0->CMD = ADC_CMD_SINGLESTOP;
  unblockSleepMode(sleepEM1);

  /*Store the values in the DMA buffer */
  while(cnt != MAX_CONVERSION) {
    sum += ADC0_DMArambuffer[cnt];
    cnt++;
  }
  //ADC0->CMD = ADC_CMD_SINGLESTOP;

  /*Get the average and convert to celsius */
  sum = sum/MAX_CONVERSION;
  float C_temp = convertToCelsius(sum);
#if 0
  if (C_temp > 23) {
    /*Turn on the LED*/
    GPIO_PinOutSet(LED_PORT,LED_1_PIN);
  } else {
    /* Turn off the LED */
    GPIO_PinOutClear(LED_PORT,LED_1_PIN);
  }
#endif

  INT_Enable();

  return;
}

/* Do the callback function configuration */
static DMA_CB_TypeDef dma_cb_config = {
  .cbFunc = cb_ADC0_DMA,
  .userPtr = NULL,
  .primary = true
};


void ADC0_DMA_Setup(void)
{
  /* Setup the initial DMA configuration */
  static DMA_CfgDescr_TypeDef dma_cfgdescr = {
    .arbRate = dmaArbitrate1,
    .dstInc = dmaDataInc2,
    .hprot = 0,
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
  //DMA_Initialize();
  DMA_ActivateBasic(ADC0_DMA_Channel, true, false, (void *)ADC0_DMArambuffer, (void*)&(ADC0->SINGLEDATA), 749);

  return;
}

void DMA_Initialize(void)
{
  /* Initialize the structure */
  static DMA_Init_TypeDef Init_DMA = {
    .controlBlock = dmaControlBlock,
    .hprot = 0
  };
  
  DMA_Init(&Init_DMA);

  ADC0_DMA_Setup();
  
  /* Enable NVIC */
  NVIC_EnableIRQ(DMA_IRQn);


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
 */

void LETIMER0_IRQHandler(void)
{
  INT_Disable();

  /* Find out which interrupt flag is set */
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

    /* Add the functionality for the temperature sensor */
#ifdef WITHOUT_DMA
    /*Move out of the EM3 mode */
    blockSleepMode(sleepEM1);
    
    /* Get the average temperature of the MCU */
    output = Get_Avg_Temperature();

    if(output > 24) {
      /*Turn on LED1 */
      GPIO_PinOutSet(LED_PORT,LED_1_PIN);
    } else {
      /*Turn off LED1 */
      GPIO_PinOutClear(LED_PORT,LED_1_PIN);
    }

    unblockSleepMode(sleepEM1);
#else

    /* Setup the DMA */
    //ADC0_DMA_Setup();
    DMA_Initialize();

    /* Block to EM1 */
    blockSleepMode(sleepEM1);

    /* Initialize the ADC */
    ADC_Start(ADC0, adcStartSingle);

    /* Activate the DMA conversion with the callback function */
    //DMA_ActivateBasic(ADC0_DMA_Channel, true, false, (void *)ADC0_DMArambuffer, (void*)&(ADC0->SINGLEDATA), 749);
    
    /* Stop the ADC */
    //ADC0->CMD = ADC_CMD_SINGLESTOP;


    /* Get the average temperature of the MCU */
    //output = Get_Avg_Temperature();

    /*Do the Basic DMA Transfer now! */
    //DMA_ActivateBasic(ADC0_DMA_Channel, true, false, (void *)ADC0_DMArambuffer, (void*)&(ADC0->SINGLEDATA), ADCSAMPLE-1);
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
        ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel6);
        /* Set the LED */
        GPIO_PinOutSet(LED_PORT,LED_0_PIN);
      }
      else
      {
        /* Lower the Level */
        acmpinit.vddLevel = LOW_LEVEL;
        /* Initialize and set the channel for the ACMP */
        ACMP_Init(ACMP0,&acmpinit);
        ACMP_ChannelSet(ACMP0, acmpChannel6, acmpChannelVDD);
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

/* Function: Setup_Enable_ACMP0(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *    Initialize the clock and channel of the ACMP and Enable it 
 */
void Setup_Enable_ACMP0(void)
{
  /* Setup the ACMP */
  CMU_ClockEnable(cmuClock_ACMP0, true);
  ACMP_Init(ACMP0,&acmpinit);								
  ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel6);
  ACMP_Enable(ACMP0);

  return;
}

void Assignment3_Cluster(void)
{

  /* Initialize the GPIO and the Ambient Light Sensor */
  GPIO_Init();
  Light_Sensor_Init();
 
  /* Enable the clock for HFPERCO */
  CMU_ClockEnable(cmuClock_HFPER, true);
  
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
#if 0
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
  LETIMER0->IFC = 0xFF;
 
  /* Enable the interrupts for COMP0 and COMP1 */
  LETIMER0->IEN = (LETIMER_IEN_COMP0 | LETIMER_IEN_COMP1);

  /* Set the NVIC to trigger the LETIMER0 interrupt */
  NVIC_EnableIRQ(LETIMER0_IRQn);
  
  /*Initialize the LETIMER and Enable it */ 
  LETIMER_Init(LETIMER0, &letimerInit);
  LETIMER_Enable(LETIMER0, true);
 
#ifdef Legacy_Assignemnt3
  /* Block in EM3 */
  blockSleepMode(SEL_SLEEP_MODE);
#endif
  blockSleepMode(sleepEM3);

  return;
}

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
    .prescale         = 49,
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

  Assignment3_Cluster();

  ADC0_Init();
  //float output = Get_Avg_Temperature();

  blockSleepMode(sleepEM3);
  
  /* Infinite loop */
  while (1) {
	  sleep();
  }
}

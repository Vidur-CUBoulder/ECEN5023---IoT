/*
 * adc.c
 *
 *  Created on: Feb 24, 2017
 *      Author: vidursarin
 */
/* Function: ADC0_Init(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *    - Initialize the ADC0 and setup its default configurations
 */

#include "adc.h"
#include "em_int.h"
#include "em_core.h"

volatile int16_t ADC0_DMArambuffer[MAX_CONVERSION] = {0};

/* This is a global definition for the DMA callback function
 * configuration.
 */
static DMA_CB_TypeDef dma_cb_config = {
  .cbFunc = cb_ADC0_DMA,
  .userPtr = NULL,
  .primary = true
};

void ADC0_Init(void)
{
  /* Do the timebase calculation */
  int8_t calc_timebase = ADC_TimebaseCalc(CMU_ClockFreqGet(cmuClock_HFPER));

  /* Initialize the ADC struct */
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




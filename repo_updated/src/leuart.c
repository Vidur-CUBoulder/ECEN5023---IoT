/*
 * leuart.c
 *
 *  Created on: Mar 11, 2017
 *      Author: vidursarin
 */
#include "leuart.h"
#include "sleep_modes.h"

#define DATA_BUFFER_SIZE 5
#define BAUD_RATE 9600
#define SEL_SLEEP_MODE    sleepEM3
#define LEUART_SLEEP_MODE sleepEM2
//extern uint8_t *data_buffer[DATA_BUFFER_SIZE];
extern uint8_t transfer_bytes = 0;
extern uint8_t counter;


/* Function: void Setup_LEUART(void)
 * Parameters:
 *      void
 * Return:
 *      void
 * Description:
 *    - Do the initial setup for the LEUART
 */
void Setup_LEUART(void)
{

  /* To avoid false start, configure output as high */
  GPIO_PinModeSet(LEUART_TXPORT, LEUART_TXPIN, gpioModePushPull, 1);
  GPIO_PinModeSet(LEUART_RXPORT, LEUART_RXPIN, gpioModeInput, 1);

  static LEUART_Init_TypeDef init_leuart = {
    .baudrate   = BAUD_RATE,        /* 9600 bits/s. */
    .databits   = leuartDatabits8,  /* 8 databits. */
    .enable     = leuartDisable,    /* Disable RX/TX when init completed. */
    .parity     = leuartNoParity,   /* No parity. */ 
    .refFreq    = 0,                /* Use current configured reference clock for configuring baudrate. */
    .stopbits   = leuartStopbits1   /* 1 stopbit. */
  };

  /* Select LFXO for LEUARTs (and wait for it to stabilize) */
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_LEUART0, true);

  /* Configure LEUART */
  init_leuart.enable = leuartDisable;

  /* In order to be cautious, reset the LEUART and start the init. procedures */
  LEUART_Reset(LEUART0);
  LEUART_Init(LEUART0, &init_leuart);
  
  /* Enable pins at default location */
  LEUART0->ROUTE = LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN | LEUART_LOCATION;
  
  /* Clear the interrupt flags */
  LEUART0->IFC = 0xFF;
 
  /* Enable the interrupt flag for the LEUART 
   * - Enable the interrupt on the completion of the TX.
   */
  LEUART0->IEN = LEUART_IEN_TXC;

  /* Finally enable it */
  LEUART_Enable(LEUART0, leuartEnable);
  NVIC_EnableIRQ(LEUART0_IRQn);

  /* Go to the EM3 sleep mode
   * XXX: LEUART does NOT operate in EM3!
   * LEUART will operate in EM2 and higher.
   */
  //blockSleepMode(sleepEM3);
  blockSleepMode(SEL_SLEEP_MODE);
  return;
}

/* Function: void Setup_LEUART_DMA(void)
 * Parameters:
 *      void
 * Return:
 *      void
 * Description:
 *    - Do the setup for the LEUART to work with DMA
 */
void Setup_LEUART_DMA(void)
{
  /* Initializing the DMA */
  static DMA_Init_TypeDef dmaInit = {
    .controlBlock = dmaControlBlock, 
    .hprot        = 0
  };

  /*Setting up DMA channel */
  static DMA_CfgChannel_TypeDef channelCfg = {
    .cb         = NULL, 
    .enableInt  = false,  
    .highPri    = false,
    .select     = DMAREQ_LEUART0_RXDATAV
  };

  /* Setting up channel descriptor */
  static DMA_CfgDescr_TypeDef descrCfg = {
    .arbRate  = dmaArbitrate1, 
    .dstInc   = dmaDataIncNone,
    .hprot    = 0,
    .size     = dmaDataSize1,
    .srcInc   = dmaDataIncNone
  };
 
  /* Configure loop transfer mode */
  static DMA_CfgLoop_TypeDef loopCfg = {
    .enable   = true,  
    .nMinus1  = 0     /* Single transfer per DMA cycle */
  };

  /* Call all the initialization functions now */
  DMA_Init(&dmaInit);
  DMA_CfgChannel(0, &channelCfg);
  DMA_CfgDescr(0, true, &descrCfg);
  DMA_CfgLoop(0, &loopCfg);

  /* Activate basic dma cycle using channel0 */
  DMA_ActivateBasic(0,\
      true,\
      false,\
      (void *)&LEUART0->TXDATA,\
      (void *)&LEUART0->RXDATA,\
      0);

  return;
}

/* Function:void LEUART0_IRQHandler(void)
 * Parameters:
 *      void
 * Return:
 *      void
 * Description:
 *    - Interrupt handler for LEUART0. This will be triggered on ever
 *      successful TXC event.
 */
void LEUART0_IRQHandler(void)
{
	INT_Disable();

	uint8_t ret_data = 0;
	debug_buf debug_handle;

	/* Clear the TXC flag */
	LEUART0->IFC = LEUART_IFC_TXC;

  /* Transfer all the data from the data buffer to the
   * peripheral device. In every other case, continue to 
   * block in the EM2 case.
   */
	if(counter != DATA_BUFFER_SIZE){
		/* Send the data again */
		//LEUART0->TXDATA = *data_buffer[counter++];
		if(remove_from_buffer(&buffer, &ret_data, sizeof(uint8_t)) != BUFFER_EMPTY) {
			LEUART0->TXDATA = ret_data;
		}
	} else {
		/* unblock in EM2 sleep mode */
		unblockSleepMode(LEUART_SLEEP_MODE);
	}

  INT_Enable();

  return;
}




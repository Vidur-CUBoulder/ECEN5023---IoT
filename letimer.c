#include "letimer.h"

int32_t net_time = 0;

/* Function: LETIMER_ClockSetup(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *    This function is used to setup the clock parameters for the LETIMER.
 */
void LETIMER_ClockSetup(void)
{
  /* For EM0-EM2, set the clock to LFXO */
#ifdef LFXO_CLK
  /* Enable the LFXO clock for the LETIMER by default */
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
  /* Select the LFXO for LFA */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  /* Block in EM2 */
  blockSleepMode(DEFINED_EM);
#endif

#ifdef ULFRCO_CLK
  /* Enable the ULFRCO clock for the LETIMER by default */
  CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);
  /* Select the ULFRCO for LFA */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
  /* Block in EM3 */
  blockSleepMode(DEFINED_EM);
#endif

  /* Enable the core LE clock */
  CMU_ClockEnable(cmuClock_CORELE, true);
  
  /* Enable the clock to the LETIMER */
  CMU_ClockEnable(cmuClock_LETIMER0, true);
}

/* Function: Enable_LETIMER_Interrupt(void)
 * Paramters:
 *    void
 * Return:
 *    void
 * Description:
 *    Setup the Interrupt parameters for the LETIMER Interrupt.
 *    You will not be able to use the IRQ handler function without 
 *    enabling these parameters
 */
void Enable_LETIMER_Interrupt(void)
{
  /* Clear the interrupts */
  LETIMER0->IFC = (LETIMER_IFC_COMP0 | LETIMER_IFC_COMP1 | LETIMER_IFC_UF);

  /* Enable the COMP0 and COMP1 Interrupt */
  LETIMER0->IEN = (LETIMER_IEN_UF);

  /* Enable Interrupts to the CPU via the NVIC */
  NVIC_EnableIRQ(LETIMER0_IRQn);

  /* Enable the LETIMER */
  LETIMER_Enable(LETIMER0, true);

}

/* Function: LETIMER_Setup(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *    - Setup the main parameters for the LETIMER along with the default 
 *      initialization values.
 *    - Also define the on and off times for the LEDs.
 */
void LETIMER_Setup_Comparator(void)
{

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

  /* Get the LETIMER clock frequency */
  float get_clk_value = 0;
  get_clk_value = CMU_ClockFreqGet(cmuClock_LETIMER0);

  /* Set the COMP0 and COMP1 values */
  int16_t off_time = (OFF_PERIOD * get_clk_value);
  int16_t on_time = (ON_PERIOD * get_clk_value);
  LETIMER_CompareSet(LETIMER0, 1, on_time);
  LETIMER_CompareSet(LETIMER0, 0, off_time);

  /* Initialize the LETIMER clock */
  LETIMER_Init(LETIMER0, &letimerInit);

  /* Next to the interrupt configuration */
  Enable_LETIMER_Interrupt();
}

void LETIMER_Setup_Counter(void) 
{

  /* Disable the LETIMER before initializing it */
  LETIMER_Enable(LETIMER0, false);

  /* Initialize the LETIMER default structure values */
  const LETIMER_Init_TypeDef letimerInit = {
    .enable         = true,              /* Enable timer when init complete. */
    .debugRun       = false,             /* Stop counter during debug halt. */
    .rtcComp0Enable = false,             /* Do not start counting on RTC COMP0 match. */
    .rtcComp1Enable = false,             /* Do not start counting on RTC COMP1 match. */
    .comp0Top       = false,              /* Load COMP0 into CNT on underflow. */
    .bufTop         = false,             /* Do not load COMP1 into COMP0 when REP0 reaches 0. */
    .out0Pol        = IDLE_OUT_0,        /* Idle value 0 for output 0. */
    .out1Pol        = IDLE_OUT_1,        /* Idle value 0 for output 1. */
    .ufoa0          = letimerUFOANone,   /* No action on underflow on output 0. */
    .ufoa1          = letimerUFOANone,   /* No action on underflow on output 1. */
    .repMode        = letimerRepeatOneshot  /* Count until stopped by SW. */
  };

  /* Keep it disabled for the moment */
  LETIMER_Enable(LETIMER0, false);

  /* Set the value in the CNT register */
  LETIMER0->CNT = 0x8000;
  
  /* Enable the LETIMER0 */
  LETIMER_Init(LETIMER0, &letimerInit);

  Enable_LETIMER_Interrupt();

}

void LETIMER0_IRQHandler(void)
{
  INT_Disable();
  LETIMER0->IFC |= (LETIMER_IFC_UF);
  
  //Disable the TIMERS...
  TIMER_Enable(TIMER0, false);
  TIMER_Enable(TIMER1, false);
  
  int16_t timer0 = TIMER0->CNT;
  net_time = TIMER1->CNT;
  
  net_time = (net_time << 16) | timer0; 

  INT_Enable();
}
 




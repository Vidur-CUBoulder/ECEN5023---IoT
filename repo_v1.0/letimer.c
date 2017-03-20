#include "letimer.h"

int32_t net_time = 0;



/* Function: LETIMER_ClockSetup(CMU_Osc_TypeDef clk_type)
 * Parameters:
 *    CMU_Osc_TypeDef clk_type - to define the type of clock 
 *                          to be configured
 * Return:
 *    void
 * Description:
 *    This function is used to setup the clock parameters for the LETIMER.
 */

void LETIMER_ClockSetup(CMU_Osc_TypeDef clk_type)
{
  /* Enable the LFXO clock for the LETIMER by default */
  CMU_OscillatorEnable(clk_type, true, true);
  /* Select the LFXO for LFA */
  CMU_Select_TypeDef cmuSelect = (clk_type == cmuOsc_LFXO) ? cmuSelect_LFXO:\
                                                               cmuSelect_ULFRCO;
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect);

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
void Enable_LETIMER_Interrupt(/*CMU_Osc_TypeDef clk_type void*/)
{

  /* Clear the interrupts */
  LETIMER0->IFC = (LETIMER_IFC_COMP0 | LETIMER_IFC_COMP1 | LETIMER_IFC_UF);

  /* Enable the COMP0 and COMP1 Interrupt */
  LETIMER0->IEN = (LETIMER_IEN_UF | LETIMER_IEN_COMP0 | LETIMER_IEN_COMP1);

  /* Enable Interrupts to the CPU via the NVIC */
  NVIC_EnableIRQ(LETIMER0_IRQn);

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
void LETIMER_Setup_Comparator(float new_value)
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
  int16_t off_time = (OFF_PERIOD * new_value);
  int16_t on_time = (ON_PERIOD * new_value);
  LETIMER_CompareSet(LETIMER0, 1, on_time);
  LETIMER_CompareSet(LETIMER0, 0, off_time);

  /* Initialize the LETIMER clock */
  LETIMER_Init(LETIMER0, &letimerInit);

  /* Next to the interrupt configuration */
#ifdef CALIBRATE_ULFRCO
  Enable_LETIMER_Interrupt();
#endif
}

/* Function: LETIMER_Setup_Counter(CMU_Osc_TypeDef clk_type)
 * Parameters:
 *    clk_type: the clk for which you want to setup/config. the counter
 * Return:
 *    void
 * Description:
 *    Setup and initialize the LETIMER counter for the given clock
 */
void LETIMER_Setup_Counter(CMU_Osc_TypeDef clk_type)
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

  /* Set the value in the CNT register */
  LETIMER0->CNT = (clk_type == cmuOsc_LFXO) ? 0x8000: 0x03E8;
  
  /* Keep it disabled for the moment */
  LETIMER_Enable(LETIMER0, false);

  /* Enable the LETIMER0 */
  LETIMER_Init(LETIMER0, &letimerInit);

}
/* Function: Calc_Freq(int32_t *net_time)
 * Parameters:
 *    - net_time - returns/stores the time that the function calculates
 * Return:
 *    void
 * Description:
 *    - Stop the timers and calc. the freq of the clock being run.
 */
void Calc_Freq(int32_t *net_time)
{
  /* Clear the UF flag in the IFC */
  LETIMER0->IFC |= (LETIMER_IFC_UF);

  /* Disable the timers first */
  TIMER_Enable(TIMER0, false);
  TIMER_Enable(TIMER1, false);

  /* Get the values from the counters and calc. the 
   * net freq value*/
  int32_t timer0 = TIMER0->CNT;
  *net_time = TIMER1->CNT;
  *net_time = (*net_time << 16) | timer0; 

}

/* Function: Get_Clk_Freq(CMU_Osc_TypeDef clk_type, float *freq)
 * Parameters:
 *  - CMU_Osc_TypeDef clk_type: specify the type of clock whose frequency is returned
 *  - freq: the frequency that is returned
 * Return:
 *    void
 * Description:
 *    Gets the freq. of the clk that is passed to the func. as a parameter 
 */
void Get_Clk_Freq(CMU_Osc_TypeDef clk_type,\
                  float *freq)
{
  LETIMER_Enable(LETIMER0, false);
  
  LETIMER_Setup_Counter(clk_type);
  
  LETIMER_Enable(LETIMER0, true);
  
  while(LETIMER0->CNT != 0x0);
  
  /* Do the calc. now!! */
  Calc_Freq(freq); 

  return;
}
 
/* Function: Reset_Peripherals(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *    Resets some peripherals to their initial state. The way it was when the hardware was initialized 
 */
void Reset_Peripherals(void)
{
  
  LETIMER_Reset(LETIMER0);
  TIMER_Reset(TIMER0);
  TIMER_Reset(TIMER1);
  
  return;
}


/* Function: float Get_Osc_Ratio(void)
 * Parameters:
 *    void
 * Return:
 *    float - returns the ratio as a float type
 * Description:
 *    The initial wrapper function that calc. and returns the calibration ratio 
 */

float Get_Osc_Ratio(void)
{
  float net_time_LFXO = 0;
  float net_time_ULFRCO = 0;
  float return_val = 0;

  /* Get the TIMER counts for LFXO */
  Configure_TIMERS(cmuOsc_LFXO);
  Get_Clk_Freq(cmuOsc_LFXO, &net_time_LFXO);

  /* Reset all peripherals */
  Reset_Peripherals();

  /* Get the TIMER counts with ULFRCO */
  Configure_TIMERS(cmuOsc_ULFRCO);
  Get_Clk_Freq(cmuOsc_ULFRCO, &net_time_ULFRCO);
 
  /* Do the math */
  return_val = (net_time_LFXO/net_time_ULFRCO); 
  
  return return_val;

}

#if ASSIGNMENT_1_IRQ_HANDLER
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
#endif




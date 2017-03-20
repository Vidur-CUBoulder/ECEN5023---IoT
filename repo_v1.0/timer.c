#include "letimer.h"
#include "sleep_modes.h"
#include "timer.h"

/* Function: Configure_TIMERS(CMU_Osc_TypeDef clk_type)
 * Parameters:
 *    clk_type: the clk type for which the timers should be configured
 * Return:
 *    void
 * Description:
 *    Configures the TIMER0 and TIMER1 for the specified OSC clk.
 */
void Configure_TIMERS(CMU_Osc_TypeDef clk_type)
{
  //LETIMER_ClockSetup(/*cmuOsc_LFXO*/cmuOsc_ULFRCO);
  LETIMER_ClockSetup(clk_type);

  /* Enable the clock for HFPERCO */
  CMU_ClockEnable(cmuClock_HFPER, true);

  /* Enable the clock for TIMER0  & TIMER1 */
  CMU_ClockEnable(cmuClock_TIMER0, true);
  CMU_ClockEnable(cmuClock_TIMER1, true);

  /* Setup the TIMER_Init struct */
  TIMER_Init_TypeDef timerInit_TIMER0 =
  {
    .enable     = true, 
    .debugRun   = false, 
    .prescale   = timerPrescale1, 
    .clkSel     = timerClkSelHFPerClk, 
    .fallAction = timerInputActionNone, 
    .riseAction = timerInputActionNone, 
    .mode       = timerModeUp, 
    .dmaClrAct  = false,
    .quadModeX4 = false, 
    .oneShot    = false, 
    .sync       = true, 
  };
  
  TIMER_Init_TypeDef timerInit_TIMER1 =
  {
    .enable     = true, 
    .debugRun   = false, 
    .prescale   = timerPrescale1, 
    .clkSel     = timerClkSelCascade, 
    .fallAction = timerInputActionNone, 
    .riseAction = timerInputActionNone, 
    .mode       = timerModeUp, 
    .dmaClrAct  = false,
    .quadModeX4 = false, 
    .oneShot    = false, 
    .sync       = true, 
  };
 
  /* Initialize the timers */
  TIMER_Init(TIMER0, &timerInit_TIMER0); 
  TIMER_Init(TIMER1, &timerInit_TIMER1); 

  /* The timers should only run in EM1 */
  //blockSleepMode(sleepEM0);

} 

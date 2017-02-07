#include "letimer.h"
#include "timer.h"

void Disable_Reset_TIMERS(void)
{
  TIMER_Enable(TIMER0, false);
  TIMER_Enable(TIMER1, false);

  /* Make the counts 0 */
  TIMER0->CNT = 0x00;
  TIMER1->CNT = 0x00;
}

void Configure_TIMERS(void)
{

  /* Enable the clock for HFPERCO */
  CMU_ClockEnable(cmuClock_HFPER, true);

  /* Enable the clock for TIMER0 */
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
  
  TIMER_Init(TIMER0, &timerInit_TIMER0); 
  TIMER_Init(TIMER1, &timerInit_TIMER1); 

  /* The timers should only run in EM1 */
  blockSleepMode(sleepEM0);

} 

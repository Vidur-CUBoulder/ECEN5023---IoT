#include "em_int.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_letimer.h"
#include "em_device.h"
#include "em_rtc.h"
#include "em_timer.h"
#include "em_adc.h"

#include "timer.h"


#define IDLE_OUT_0 0
#define IDLE_OUT_1 0
#define OFF_PERIOD 1.72
#define ON_PERIOD_LETIMER 0.03

/* Define the clock to be used */
//#define ULFRCO_CLK
#define LFXO_CLK

/* TIMER values; global variables */
//int timer0 = 0;
//int timer1 = 0;

void LETIMER_ClockSetup(CMU_Osc_TypeDef clk_type);

void Enable_LETIMER_Interrupt(void);

void LETIMER_Setup_Comparator(float new_value);

void LETIMER_Setup_Counter(CMU_Osc_TypeDef clk_type);

void Calc_Freq(int32_t *net_time);

float Get_Osc_Ratio(void);

void Get_Clk_Freq(CMU_Osc_TypeDef clk_type, float *freq);

void Reset_Peripherals(void);

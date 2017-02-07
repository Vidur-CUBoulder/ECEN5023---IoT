/***************************************************************************//**
 * @file em_emu.c
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

#if 0
#include "em_device.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "bsp.h"
#include "em_int.h"

#include "em_timer.h"

#include "sleep_modes.h"

#endif

#include "timer.h"
#include "letimer.h"
#include "udelay.h"

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

#if 0
  BSP_LedsInit();
  BSP_LedSet(1);

  LETIMER_ClockSetup();
  LETIMER_Setup();
#endif

  LETIMER_ClockSetup();
  Configure_TIMERS();
  LETIMER_Setup_Counter();
  for(int j = 0; j<1000; j++);
   
  /* Infinite loop */
  while (1) {
	  sleep();
  }
}

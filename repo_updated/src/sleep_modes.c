#include "sleep_modes.h"

/*Function:blockSleepMode(SLEEP_EnergyMode_t eMode)
 * Paratmers:
 *    SLEEP_EnergyMode_t  eMode: The energy mode to block the MCU in.
 * Return: void
 * Description: 
 *    This function is used to block the MCU in a specified Energy Mode.
 *    It acts like a semaphore that prevent the MCU from going into a lower
 *    energy state.
 */
void blockSleepMode(SLEEP_EnergyMode_t eMode)
{
  INT_Disable();
  sleep_block[eMode]++;
  INT_Enable();
}

/* Function: unblockSleepMode(SLEEP_EnergyMode_t eMode)
 * Parameters:
 *    SLEEP_EnergyMode_t  eMode: The energy mode to block the MCU in.
 *  Return: void
 *  Description:
 *    This function is used to unblock the MCU from a particular Energy Mode.
 */
void unblockSleepMode(SLEEP_EnergyMode_t eMode)
{
  INT_Disable();
  if(sleep_block[eMode] > 0) {
    sleep_block[eMode]--;
  }
  INT_Enable();
}

/* Function: sleep(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *    The function is used to put the MCU into a particular Energy Mode.
 *    It is dependent upon what EM is blocked by the blockSleepMode function.
 */
void sleep(void)
{
  if(sleep_block[sleepEM0] > 0) {
    /* remain in EM0 */
    return;
  } else if (sleep_block[sleepEM1] > 0) {
    EMU_EnterEM1();
  } else if (sleep_block[sleepEM2] > 0) {
    EMU_EnterEM2(true);
  } else if (sleep_block[sleepEM3] > 0) {
    EMU_EnterEM3(true);
  } else {
    /* Don't go beyond EM3 */
    EMU_EnterEM3(true);
  }
}

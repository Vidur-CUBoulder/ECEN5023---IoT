#include "acmp.h"
#include "em_cmu.h"

/* Function: Setup_Enable_ACMP0(void)
 * Parameters:
 *    void
 * Return:
 *    void
 * Description:
 *    Initialize the clock and channel of the ACMP and Enable it 
 */
void ACMP0_Init_Start(void)
{
  /* Setup the ACMP */
  //CMU_ClockEnable(cmuClock_ACMP0, true);
  ACMP_Init(ACMP0,&acmpinit);								
  ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel6);
  ACMP_Enable(ACMP0);

  return;
}




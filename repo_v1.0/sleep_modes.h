#include "em_int.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"

#define NUM_SLEEP_MODES 3

static int sleep_block[NUM_SLEEP_MODES] = {0};

typedef enum {
  /* Status value for EM0. */
  sleepEM0 = 0,

  /* Status value for EM1. */
  sleepEM1 = 1,

  /* Status value for EM2. */
  sleepEM2 = 2,

  /* Status value for EM3. */
  sleepEM3 = 3,

  /* Status value for EM4. */
  sleepEM4 = 4
} SLEEP_EnergyMode_t;

/* Function Prototypes */

void blockSleepMode(SLEEP_EnergyMode_t eMode);

void unblockSleepMode(SLEEP_EnergyMode_t eMode);

void sleep(void);




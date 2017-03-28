#include "em_acmp.h"

#define LOW_LEVEL 2
#define HIGH_LEVEL 61

/* Global structure definition for the Analog Comparator */
static ACMP_Init_TypeDef acmpinit =
{
  .fullBias                 = false,
  .halfBias                 = true,
  .biasProg                 = 0x0,
  .interruptOnFallingEdge   = false,
  .interruptOnRisingEdge    = false,
  .warmTime                 = acmpWarmTime256,
  .hysteresisLevel          = acmpHysteresisLevel4,
  .inactiveValue            = false,
  .lowPowerReferenceEnabled = false,
  .vddLevel                 = LOW_LEVEL,
  .enable                   = true
};

void Setup_Enable_ACMP0(void);

void ADC0_Init(void);




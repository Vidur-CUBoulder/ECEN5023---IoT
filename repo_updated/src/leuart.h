/*
 * leuart.h
 *
 *  Created on: Mar 11, 2017
 *      Author: vidursarin
 */

#ifndef SRC_LEUART_H_
#define SRC_LEUART_H_

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_leuart.h"
#include "em_dma.h"
#include "dmactrl.h"
#include "em_int.h"
#include "circular_buffer.h"

/** LEUART Rx/Tx Port/Pin Location */
#define LEUART_LOCATION    0
#define LEUART_TXPORT      gpioPortD            /* LEUART transmission port */
#define LEUART_TXPIN       4                    /* LEUART transmission pin */
#define LEUART_RXPORT      gpioPortD            /* LEUART reception port */
#define LEUART_RXPIN       5                    /* LEUART reception pin */

void Setup_LEUART(void);

void Setup_LEUART_DMA(void);

#endif /* SRC_LEUART_H_ */

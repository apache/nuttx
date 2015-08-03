/****************************************************************************
 * arch/arm/src/samv7/sam_mcan.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   SAMV7D3 Series Data Sheet
 *   Atmel sample code
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX, Atmel, nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/can.h>

#include "up_internal.h"
#include "up_arch.h"


#include "chip/sam_matrix.h"
#include "chip/sam_pinmap.h"
#include "sam_periphclks.h"
#include "sam_gpio.h"
#include "sam_mcan.h"

#if defined(CONFIG_CAN) && defined(CONFIG_SAMV7_MCAN)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Common definitions *******************************************************/

#ifndef MIN
#  define MIN(a,b) ((a < b) ? a : b)
#endif

#ifndef MAX
#  define MAX(a,b) ((a > b) ? a : b)
#endif

/* Clock source *************************************************************/

/* PCK5 is the programmable clock source, common to all MCAN controllers */

#if defined(CONFIG_SAMV7_MCAN_CLKSRC_SLOW)
#  define SAMV7_MCAN_CLKSRC           PMC_PCK_CSS_SLOW
#  define SAMV7_MCAN_CLKSRC_FREQUENCY BOARD_SLOWCLK_FREQUENCY
#elif defined(CONFIG_SAMV7_MCAN_CLKSRC_PLLA)
#  define SAMV7_MCAN_CLKSRC           PMC_PCK_CSS_PLLA
#  define SAMV7_MCAN_CLKSRC_FREQUENCY BOARD_PLLA_FREQUENCY
#elif defined(CONFIG_SAMV7_MCAN_CLKSRC_UPLL)
#  define SAMV7_MCAN_CLKSRC           PMC_PCK_CSS_UPLL
#  define SAMV7_MCAN_CLKSRC_FREQUENCY BOARD_UPLL_FREQUENCY
#elif defined(CONFIG_SAMV7_MCAN_CLKSRC_MCK)
#  define SAMV7_MCAN_CLKSRC           PMC_PCK_CSS_MCK
#  define SAMV7_MCAN_CLKSRC_FREQUENCY BOARD_MCK_FREQUENCY
#else /* if defined(CONFIG_SAMV7_MCAN_CLKSRC_MAIN */
#  define SAMV7_MCAN_CLKSRC           PMC_PCK_CSS_MAIN
#  define SAMV7_MCAN_CLKSRC_FREQUENCY BOARD_MAINOSC_FREQUENCY
#endif

#ifndef CONFIG_SAMV7_MCAN_CLKSRC_PRESCALER
#  define CONFIG_SAMV7_MCAN_CLKSRC_PRESCALER 1
#endif

#define SAMV7_MCANCLK_FREQUENCY \
  (SAMV7_MCAN_CLKSRC_FREQUENCY / CONFIG_SAMV7_MCAN_CLKSRC_PRESCALER)

/* MCAN0 Configuration ******************************************************/

#ifdef CONFIG_SAMV7_MCAN0

/* Bit timing */

#  define MCAN0_TSEG1  (CONFIG_SAMV7_MCAN0_PROPSEG + CONFIG_SAMV7_MCAN0_PHASESEG1)
#  define MCAN0_TSEG2  CONFIG_SAMV7_MCAN0_PHASESEG2
#  define MCAN0_BRP    ((uint32_t)(((float) SAMV7_MCANCLK_FREQUENCY / \
                       ((float)(MCAN0_TSEG1 + MCAN0_TSEG2 + 3) * \
                        (float)CONFIG_SAMV7_MCAN0_BITRATE)) - 1))
#  define MCAN0_SJW    (CONFIG_SAMV7_MCAN0_FSJW - 1)

#  if MCAN0_TSEG1 > 63
#    error Invalid MCAN0 TSEG1
#  endif
#  if MCAN0_TSEG2 > 15
#    error Invalid MCAN0 TSEG2
#  endif
#  if MCAN0_SJW > 15
#    error Invalid MCAN0 SJW
#  endif

#  define MCAN0_FTSEG1 (CONFIG_SAMV7_MCAN0_FPROPSEG + CONFIG_SAMV7_MCAN0_FPHASESEG1)
#  define MCAN0_FTSEG2 (CONFIG_SAMV7_MCAN0_FPHASESEG2)
#  define MCAN0_FBRP   ((uint32_t)(((float) SAMV7_MCANCLK_FREQUENCY / \
                       ((float)(MCAN0_FTSEG1 + MCAN0_FTSEG2 + 3) * \
                        (float)CONFIG_SAMV7_MCAN0_FBITRATE)) - 1))
#  define MCAN0_FSJW   (CONFIG_SAMV7_MCAN0_FFSJW - 1)

#  if MCAN0_FTSEG1 > 15
#    error Invalid MCAN0 FTSEG1
#  endif
#  if MCAN0_FTSEG2 > 7
#    error Invalid MCAN0 FTSEG2
#  endif
#  if MCAN0_FSJW > 3
#    error Invalid MCAN0 FSJW
#  endif

/* MCAN0 RX FIFO0 element size */

#  if defined(CONFIG_SAMV7_MCAN0_RXFIFO0_8BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO0_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO0_12BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO0_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO0_16BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO0_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO0_20BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO0_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO0_24BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO0_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO0_32BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO0_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO0_48BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO0_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO0_64BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO0_ENCODED_SIZE  7
#  else
#    error Undefined MCAN0 RX FIFO0 element size
#  endif

#  if CONFIG_SAMV7_MCAN0_RXFIFO0_SIZE > 64
#    error Invalid MCAN0 number of RX FIFO0 elements
#  endif

#  define MCAN0_RXFIFO0_WORDS \
    (CONFIG_SAMV7_MCAN0_RXFIFO0_SIZE * ((MCAN0_RXFIFO0_ELEMENT_SIZE/4) + 2))

/* MCAN0 RX FIFO1 element size */

#  if defined(CONFIG_SAMV7_MCAN0_RXFIFO1_8BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO1_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO1_12BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO1_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO1_16BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO1_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO1_20BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO1_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO1_24BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO1_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO1_32BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO1_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO1_48BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO1_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO1_64BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO1_ENCODED_SIZE  7
#  else
#    error Undefined MCAN0 RX FIFO1 element size
#  endif

#  if CONFIG_SAMV7_MCAN0_RXFIFO1_SIZE > 64
#    error Invalid MCAN0 number of RX FIFO1 elements
#  endif

#  define MCAN0_RXFIFO1_WORDS \
    (CONFIG_SAMV7_MCAN0_RXFIFO1_SIZE * ((MCAN1_RXFIFO1_ELEMENT_SIZE/4) + 2))

/* MCAN0 Filters */

#  if (CONFIG_SAMV7_MCAN0_NSTDFILTERS > 128)
#    error Invalid MCAN0 number of Standard Filters
#  endif

#  if (CONFIG_SAMV7_MCAN0_NEXTFILTERS > 64)
#    error Invalid MCAN0 number of Extended Filters
#  endif

#define MCAN0_STDFILTER_WORDS  CONFIG_SAMV7_MCAN0_NSTDFILTERS
#define MCAN0_EXTFILTER_WORDS  (CONFIG_SAMV7_MCAN0_NEXTFILTERS * 2)

/* MCAN0 RX buffer element size */

#  if defined(CONFIG_SAMV7_MCAN0_RXBUFFER_8BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_RXBUFFER_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN0_RXBUFFER_12BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_RXBUFFER_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN0_RXBUFFER_16BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_RXBUFFER_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN0_RXBUFFER_20BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_RXBUFFER_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN0_RXBUFFER_24BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_RXBUFFER_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN0_RXBUFFER_32BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_RXBUFFER_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN0_RXBUFFER_48BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_RXBUFFER_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN0_RXBUFFER_64BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_RXBUFFER_ENCODED_SIZE  7
#  else
#    error Undefined MCAN0 RX buffer element size
#  endif

#  if (CONFIG_SAMV7_MCAN0_DEDICATED_RXBUFFER_SIZE > 64)
#    error Invalid MCAN0 number of RX BUFFER elements
#  endif

#  define MCAN0_DEDICATED_RXBUFFER_WORDS \
    (CONFIG_SAMV7_MCAN0_DEDICATED_RXBUFFER_SIZE * \
    ((MCAN0_RXBUFFER_ELEMENT_SIZE/4) + 2))

/* MCAN0 TX buffer element size */

#  if defined(CONFIG_SAMV7_MCAN0_TXBUFFER_8BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_TXBUFFER_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN0_TXBUFFER_12BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_TXBUFFER_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN0_TXBUFFER_16BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_TXBUFFER_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN0_TXBUFFER_20BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_TXBUFFER_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN0_TXBUFFER_24BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_TXBUFFER_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN0_TXBUFFER_32BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_TXBUFFER_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN0_TXBUFFER_48BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_TXBUFFER_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN0_TXBUFFER_64BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_TXBUFFER_ENCODED_SIZE  7
#  else
#    error Undefined MCAN0 TX buffer element size
#  endif

#  if ((CONFIG_SAMV7_MCAN0_DEDICATED_TXBUFFER_SIZE + \
        CONFIG_SAMV7_MCAN0_TXFIFOQ_SIZE)  > 32)
#    error Invalid MCAN0 number of TX BUFFER elements
#  endif

#  define MCAN0_DEDICATED_TXBUFFER_WORDS \
    (CONFIG_SAMV7_MCAN0_DEDICATED_TXBUFFER_SIZE * \
    ((MCAN0_TXBUFFER_ELEMENT_SIZE/4) + 2))

/* MCAN0 TX FIFOs */

#  define MCAN0_TXEVENTFIFO_WORDS  (CONFIG_SAMV7_MCAN0_TXEVENTFIFO_SIZE * 2)
#  define MCAN0_TXFIFIOQ_WORDS \
    (CONFIG_SAMV7_MCAN0_TXFIFOQ_SIZE * ((MCAN0_TXBUFFER_ELEMENT_SIZE/4) + 2))

#  if CONFIG_SAMV7_MCAN0_TXEVENTFIFO_SIZE > 32
#    error Invalid MCAN0 number of TX EVENT FIFO elements
#  endif

/* MCAN0 Message RAM */

#  define MCAN0_STDFILTER_INDEX   0
#  define MCAN0_EXTFILTERS_INDEX  (MCAN0_STDFILTER_INDEX + MCAN0_STDFILTER_WORDS)
#  define MCAN0_RXFIFO0_INDEX     (MCAN0_EXTFILTERS_INDEX + MCAN0_EXTFILTER_WORDS)
#  define MCAN0_RXFIFO1_INDEX     (MCAN0_RXFIFO0_INDEX + MCAN0_RXFIFO0_WORDS)
#  define MCAN0_RXDEDICATED_INDEX (MCAN0_RXFIFO1_INDEX + MCAN0_RXFIFO1_WORDS)
#  define MCAN0_TXEVENTFIFO_INDEX (MCAN0_RXDEDICATED_INDEX + MCAN0_DEDICATED_RXBUFFER_WORDS)
#  define MCAN0_TXDEDICATED_INDEX (MCAN0_TXEVENTFIFO_INDEX + MCAN0_TXEVENTFIFO_WORDS)
#  define MCAN0_TXFIFOQ_INDEX     (MCAN0_TXDEDICATED_INDEX + MCAN0_DEDICATED_TXBUFFER_WORDS)
#  define MCAN0_MSGRAM_WORDS      (MCAN0_TXFIFOQ_INDEX + MCAN0_TXFIFIOQ_WORDS)

#endif /* CONFIG_SAMV7_MCAN0 */

/* Loopback mode */

#undef SAMV7_MCAN_LOOPBACK
#if defined(CONFIG_SAMV7_MCAN0_LOOPBACK) || defined(CONFIG_SAMV7_MCAN1_LOOPBACK)
#  define SAMV7_MCAN_LOOPBACK 1
#endif

/* MCAN1 Configuration ******************************************************/

#ifdef CONFIG_SAMV7_MCAN1
  /* Bit timing */

#  define MCAN1_TSEG1  (CONFIG_SAMV7_MCAN1_PROPSEG + CONFIG_SAMV7_MCAN1_PHASESEG1)
#  define MCAN1_TSEG2  CONFIG_SAMV7_MCAN1_PHASESEG2
#  define MCAN1_BRP    ((uint32_t)(((float) SAMV7_MCANCLK_FREQUENCY / \
                       ((float)(MCAN1_TSEG1 + MCAN1_TSEG2 + 3) * \
                        (float)CONFIG_SAMV7_MCAN1_BITRATE)) - 1))
#  define MCAN1_SJW    (CONFIG_SAMV7_MCAN1_FSJW - 1)

#  if MCAN1_TSEG1 > 63
#    error Invalid MCAN1 TSEG1
#  endif
#  if MCAN1_TSEG2 > 15
#    error Invalid MCAN1 TSEG2
#  endif
#  if MCAN1_SJW > 15
#    error Invalid MCAN1 SJW
#  endif

#  define MCAN1_FTSEG1 (CONFIG_SAMV7_MCAN1_FPROPSEG + CONFIG_SAMV7_MCAN1_FPHASESEG1)
#  define MCAN1_FTSEG2 (CONFIG_SAMV7_MCAN1_FPHASESEG2)
#  define MCAN1_FBRP   ((uint32_t)(((float) SAMV7_MCANCLK_FREQUENCY / \
                       ((float)(MCAN1_FTSEG1 + MCAN1_FTSEG2 + 3) * \
                        (float)CONFIG_SAMV7_MCAN1_FBITRATE)) - 1))
#  define MCAN1_FSJW   (CONFIG_SAMV7_MCAN1_FFSJW - 1)

#if MCAN1_FTSEG1 > 15
#  error Invalid MCAN1 FTSEG1
#endif
#if MCAN1_FTSEG2 > 7
#  error Invalid MCAN1 FTSEG2
#endif
#if MCAN1_FSJW > 3
#  error Invalid MCAN1 FSJW
#endif

/* MCAN1 RX FIFO0 element size */

#  if defined(CONFIG_SAMV7_MCAN1_RXFIFO0_8BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO0_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO0_12BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO0_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO0_16BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO0_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO0_20BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO0_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO0_24BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO0_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO0_32BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO0_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO0_48BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO0_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO0_64BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO0_ENCODED_SIZE  7
#  else
#    error Undefined MCAN1 RX FIFO0 element size
#  endif

#  if CONFIG_SAMV7_MCAN1_RXFIFO0_SIZE > 64
#    error Invalid MCAN1 number of RX FIFO 0 elements
#  endif

#  define MCAN1_RXFIFO0_WORDS \
     (CONFIG_SAMV7_MCAN1_RXFIFO0_SIZE * ((MCAN1_RXFIFO0_ELEMENT_SIZE/4) + 2))

/* MCAN1 RX FIFO1 element size */

#  if defined(CONFIG_SAMV7_MCAN1_RXFIFO1_8BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO1_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO1_12BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO1_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO1_16BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO1_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO1_20BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO1_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO1_24BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO1_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO1_32BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO1_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO1_48BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO1_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO1_64BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO1_ENCODED_SIZE  7
#  else
#    error Undefined MCAN1 RX FIFO1 element size
#  endif

#  if CONFIG_SAMV7_MCAN1_RXFIFO1_SIZE > 64
#    error Invalid MCAN1 number of RX FIFO 0 elements
#  endif

#  define MCAN1_RXFIFO1_WORDS \
     (CONFIG_SAMV7_MCAN1_RXFIFO1_SIZE * ((MCAN1_RXFIFO1_ELEMENT_SIZE/4) + 2))

/* MCAN1 Filters */

#  if CONFIG_SAMV7_MCAN1_NSTDFILTERS > 128
#    error Invalid MCAN1 number of Standard Filters
#  endif

#  if CONFIG_SAMV7_MCAN1_NEXTFILTERS > 64
#    error Invalid MCAN1 number of Extended Filters
#  endif

#  define MCAN1_STDFILTER_WORDS  CONFIG_SAMV7_MCAN1_NSTDFILTERS
#  define MCAN1_EXTFILTER_WORDS (CONFIG_SAMV7_MCAN1_NEXTFILTERS * 2)

/* MCAN1 RX buffer element size */

#  if defined(CONFIG_SAMV7_MCAN1_RXBUFFER_8BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_RXBUFFER_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN1_RXBUFFER_12BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_RXBUFFER_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN1_RXBUFFER_16BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_RXBUFFER_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN1_RXBUFFER_20BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_RXBUFFER_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN1_RXBUFFER_24BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_RXBUFFER_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN1_RXBUFFER_32BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_RXBUFFER_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN1_RXBUFFER_48BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_RXBUFFER_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN1_RXBUFFER_64BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_RXBUFFER_ENCODED_SIZE  7
#  else
#    error Undefined MCAN1 RX buffer element size
#  endif

#  if CONFIG_SAMV7_MCAN1_DEDICATED_RXBUFFER_SIZE > 64
#    error Invalid MCAN1 number of RX BUFFER elements
#  endif

#  define MCAN1_DEDICATED_RXBUFFER_WORDS \
     (CONFIG_SAMV7_MCAN1_DEDICATED_RXBUFFER_SIZE * \
     ((MCAN1_RXBUFFER_ELEMENT_SIZE/4) + 2))

/* MCAN1 TX buffer element size */

#  if defined(CONFIG_SAMV7_MCAN1_TXBUFFER_8BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_TXBUFFER_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN1_TXBUFFER_12BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_TXBUFFER_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN1_TXBUFFER_16BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_TXBUFFER_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN1_TXBUFFER_20BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_TXBUFFER_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN1_TXBUFFER_24BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_TXBUFFER_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN1_TXBUFFER_32BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_TXBUFFER_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN1_TXBUFFER_48BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_TXBUFFER_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN1_TXBUFFER_64BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_TXBUFFER_ENCODED_SIZE  7
#  else
#    error Undefined MCAN1 TX buffer element size
#  endif

#  if ((CONFIG_SAMV7_MCAN1_DEDICATED_TXBUFFER_SIZE + \
        CONFIG_SAMV7_MCAN1_TXFIFOQ_SIZE)  > 32)
#    error Invalid MCAN1 number of TX BUFFER elements
#  endif

#  define MCAN1_DEDICATED_TXBUFFER_WORDS \
     (CONFIG_SAMV7_MCAN1_DEDICATED_TXBUFFER_SIZE * \
     ((MCAN1_TXBUFFER_ELEMENT_SIZE/4) + 2))

/* MCAN1 TX FIFOs */

#  if CONFIG_SAMV7_MCAN1_TXEVENTFIFO_SIZE > 32
#    error Invalid MCAN1 number of TX EVENT FIFO elements
#  endif

#  define MCAN1_TXEVENTFIFO_WORDS (CONFIG_SAMV7_MCAN1_TXEVENTFIFO_SIZE * 2)
#  define MCAN1_TXFIFIOQ_WORDS \
     (CONFIG_SAMV7_MCAN1_TXFIFOQ_SIZE * ((MCAN0_TXBUFFER_ELEMENT_SIZE/4) + 2))

/* MCAN1 Message RAM */

#  define MCAN1_STDFILTER_INDEX   0
#  define MCAN1_EXTFILTERS_INDEX  (MCAN1_STDFILTER_INDEX + MCAN1_STDFILTER_WORDS)
#  define MCAN1_RXFIFO0_INDEX     (MCAN1_EXTFILTERS_INDEX + MCAN1_EXTFILTER_WORDS)
#  define MCAN1_RXFIFO1_INDEX     (MCAN1_RXFIFO0_INDEX + MCAN1_RXFIFO0_WORDS)
#  define MCAN1_RXDEDICATED_INDEX (MCAN1_RXFIFO1_INDEX + MCAN1_RXFIFO1_WORDS)
#  define MCAN1_TXEVENTFIFO_INDEX (MCAN1_RXDEDICATED_INDEX + MCAN1_DEDICATED_RXBUFFER_WORDS)
#  define MCAN1_TXDEDICATED_INDEX (MCAN1_TXEVENTFIFO_INDEX + MCAN1_TXEVENTFIFO_WORDS)
#  define MCAN1_TXFIFOQ_INDEX     (MCAN1_TXDEDICATED_INDEX + MCAN1_DEDICATED_TXBUFFER_WORDS)
#  define MCAN1_MSGRAM_WORDS      (MCAN1_TXFIFOQ_INDEX + MCAN1_TXFIFIOQ_WORDS)

#endif /* CONFIG_SAMV7_MCAN1 */

/* MCAN helpers *************************************************************/

#define MAILBOX_ADDRESS(a)        ((uint32_t)(a) & 0x0000fffc)

/* Interrupts ***************************************************************/

/* RX related interrupts.
 *
 *
 */

#define MCAN_RXINTERRUPTS 0 // To be provided

/* TX related interrupts.
 *
 * MCAN_INT_TC -  Transmission Completed
 */

#define MCAN_TXINTERRUPTS MCAN_INT_TC

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing CAN */

#ifdef CONFIG_DEBUG_CAN
#  define candbg    dbg
#  define canvdbg   vdbg
#  define canlldbg  lldbg
#  define canllvdbg llvdbg
#else
#  define candbg(x...)
#  define canvdbg(x...)
#  define canlldbg(x...)
#  define canllvdbg(x...)
#endif

#if !defined(CONFIG_DEBUG) || !defined(CONFIG_DEBUG_CAN)
#  undef CONFIG_SAMV7_MCAN_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* CAN mode of operation */

enum sam_canmode_e
{
  MCAN_ISO11898_1_MODE = 0, /* CAN operation according to ISO11898-1 */
  MCAN_FD_MODE         = 1, /* CAN FD operation */
  MCAN_FD_BSW_MODE     = 2  /* CAN FD operation with bit rate switching */
};

/* This structure describes the MCAN message RAM layout */

struct sam_msgram_s
{
  uint32_t *stdfilters;    /* Standard filters */
  uint32_t *extfilters;    /* Extended filters */
  uint32_t *rxfifo0;       /* RX FIFO0 */
  uint32_t *rxfifo1;       /* RX FIFO1 */
  uint32_t *rxdedicated;   /* RX dedicated buffers */
  uint32_t *txeventfifo;   /* TX event FIFO */
  uint32_t *txdedicated;   /* TX dedicated buffers */
  uint32_t *txfifoq;       /* TX FIFO queue */
};

/* This structure provides the constant configuration of a CAN peripheral */

struct sam_config_s
{
  gpio_pinset_t rxpinset;   /* RX pin configuration */
  gpio_pinset_t txpinset;   /* TX pin configuration */
  xcpt_t handler;           /* MCAN interrupt handler */
  uintptr_t base;           /* Base address of the CAN control registers */
  uint32_t baud;            /* Configured baud */
  uint32_t btp;             /* Bit timing/prescaler register setting */
  uint32_t fbtp;            /* Fast bit timing/prescaler register setting */
  uint8_t port;             /* MCAN port number (1 or 2) */
  uint8_t pid;              /* MCAN peripheral ID */
  uint8_t irq;              /* MCAN peripheral IRQ number */
  uint8_t mode;             /* See enum sam_canmode_e */
  uint8_t nstdfilters;      /* Number of standard filters (up to 128) */
  uint8_t nextfilters;      /* Number of extended filters (up to 64) */
  uint8_t nfifo0;           /* Number of FIFO0 elements (up to 64) */
  uint8_t nfifo1;           /* Number of FIFO1 elements (up to 64) */
  uint8_t nrxdedicated;     /* Number of dedicated RX buffers (up to 64) */
  uint8_t ntxeventfifo;     /* Number of TXevent FIFO elements (up to 32) */
  uint8_t ntxdedicated;     /* Number of dedicated TX buffers (up to 64) */
  uint8_t ntxfifoq;         /* Number of TX FIFO queue elements (up to 32) */
  uint8_t rxfifo0ecode;     /* Encoded RX FIFO0 element size */
  uint8_t rxfifo0esize;     /* RX FIFO0 element size */
  uint8_t rxfifo1ecode;     /* Encoded RX FIFO1 element size */
  uint8_t rxfifo1esize;     /* RX FIFO1 element size */
  uint8_t rxbufferecode;    /* Encoded RX buffer element size */
  uint8_t rxbufferesize;    /* RX buffer element size */
  uint8_t txbufferecode;    /* Encoded TX buffer element size */
  uint8_t txbufferesize;    /* TX buffer element size */
#ifdef SAMV7_MCAN_LOOPBACK
  bool loopback;            /* True: Loopback mode */
#endif

  /* MCAN message RAM layout */

  struct sam_msgram_s msgram;
};

/* This structure provides the current state of a CAN peripheral */

struct sam_mcan_s
{
  const struct sam_config_s *config; /* The constant configuration */
  bool initialized;         /* True: Device has been initialized */
  bool  txenabled;          /* True: TX interrupts have been enabled */
  sem_t exclsem;            /* Enforces mutually exclusive access */
#if 0 // REVISIT -- may apply only to SAMA5
  uint32_t frequency;       /* CAN clock frequency */
#endif

#ifdef CONFIG_SAMV7_MCAN_REGDEBUG
  uintptr_t regaddr;        /* Last register address read */
  uint32_t regval;          /* Last value read from the register */
  unsigned int count;       /* Number of times that the value was read */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MCAN Register access */

static uint32_t mcan_getreg(FAR struct sam_mcan_s *priv, int offset);
static void mcan_putreg(FAR struct sam_mcan_s *priv, int offset, uint32_t regval);
#ifdef CONFIG_SAMV7_MCAN_REGDEBUG
static void mcan_dumpregs(FAR struct sam_mcan_s *priv, FAR const char *msg);
#else
#  define mcan_dumpregs(priv,msg)
#endif

/* Semaphore helpers */

static void mcan_semtake(FAR struct sam_mcan_s *priv);
#define mcan_semgive(priv) sem_post(&priv->exclsem)

/* Mailboxes */

static int  mcan_recvsetup(FAR struct sam_mcan_s *priv);

/* CAN driver methods */

static void mcan_reset(FAR struct can_dev_s *dev);
static int  mcan_setup(FAR struct can_dev_s *dev);
static void mcan_shutdown(FAR struct can_dev_s *dev);
static void mcan_rxint(FAR struct can_dev_s *dev, bool enable);
static void mcan_txint(FAR struct can_dev_s *dev, bool enable);
static int  mcan_ioctl(FAR struct can_dev_s *dev, int cmd, unsigned long arg);
static int  mcan_remoterequest(FAR struct can_dev_s *dev, uint16_t id);
static int  mcan_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg);
static bool mcan_txready(FAR struct can_dev_s *dev);
static bool mcan_txempty(FAR struct can_dev_s *dev);

/* CAN interrupt handling */

static inline void mcan_rxinterrupt(FAR struct can_dev_s *dev, int mbndx,
                                    uint32_t msr);
static inline void mcan_txinterrupt(FAR struct can_dev_s *dev, int mbndx);
static void mcan_interrupt(FAR struct can_dev_s *dev);
#ifdef CONFIG_SAMV7_MCAN0
static int  mcan0_interrupt(int irq, void *context);
#endif
#ifdef CONFIG_SAMV7_MCAN1
static int  mcan1_interrupt(int irq, void *context);
#endif

/* Hardware initialization */

static int  mcan_hwinitialize(FAR struct sam_mcan_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_mcanops =
{
  .co_reset         = mcan_reset,
  .co_setup         = mcan_setup,
  .co_shutdown      = mcan_shutdown,
  .co_rxint         = mcan_rxint,
  .co_txint         = mcan_txint,
  .co_ioctl         = mcan_ioctl,
  .co_remoterequest = mcan_remoterequest,
  .co_send          = mcan_send,
  .co_txready       = mcan_txready,
  .co_txempty       = mcan_txempty,
};

#ifdef CONFIG_SAMV7_MCAN0
/* Message RAM allocation */

static uint32_t g_mcan0_msgram[MCAN0_MSGRAM_WORDS];

/* Constant configuration */

static const struct sam_config_s g_mcan0const =
{
  .rxpinset         = GPIO_MCAN0_RX,
  .txpinset         = GPIO_MCAN0_TX,
  .handler          = mcan0_interrupt,
  .base             = SAM_MCAN0_BASE,
  .baud             = CONFIG_SAMV7_MCAN0_BITRATE,
  .btp              = MCAN_BTP_BRP(MCAN0_BRP) | MCAN_BTP_TSEG1(MCAN0_TSEG1) |
                      MCAN_BTP_TSEG2(MCAN0_TSEG2) | MCAN_BTP_SJW(MCAN0_SJW),
  .fbtp             = MCAN_FBTP_FBRP(MCAN0_FBRP) | MCAN_FBTP_FTSEG1(MCAN0_FTSEG1) |
                      MCAN_FBTP_FTSEG2(MCAN0_FTSEG2) | MCAN_FBTP_FSJW(MCAN0_FSJW),
  .port             = 0,
  .pid              = SAM_PID_MCAN00,
  .irq              = SAM_IRQ_MCAN00,
#if defined(CONFIG_SAMV7_MCAN0_ISO11899_1)
  .mode             = MCAN_ISO11898_1_MODE,
#elif defined(CONFIG_SAMV7_MCAN0_FD)
  .mode             = MCAN_FD_MODE,
#else /* if defined(CONFIG_SAMV7_MCAN0_FD_BSW) */
  .mode             = MCAN_FD_BSW_MODE,
#endif
  .nstdfilters      = CONFIG_SAMV7_MCAN0_NSTDFILTERS,
  .nextfilters      = CONFIG_SAMV7_MCAN0_NEXTFILTERS,
  .nfifo0           = CONFIG_SAMV7_MCAN0_RXFIFO0_SIZE,
  .nfifo1           = CONFIG_SAMV7_MCAN0_RXFIFO1_SIZE,
  .nrxdedicated     = CONFIG_SAMV7_MCAN0_DEDICATED_RXBUFFER_SIZE,
  .ntxeventfifo     = CONFIG_SAMV7_MCAN0_TXEVENTFIFO_SIZE,
  .ntxdedicated     = CONFIG_SAMV7_MCAN0_DEDICATED_TXBUFFER_SIZE,
  .ntxfifoq         = CONFIG_SAMV7_MCAN0_TXFIFOQ_SIZE,
  .rxfifo0ecode     = MCAN0_RXFIFO0_ENCODED_SIZE,
  .rxfifo0esize     = (MCAN0_RXFIFO0_ELEMENT_SIZE / 4) + 2,
  .rxfifo1ecode     = MCAN0_RXFIFO1_ENCODED_SIZE,
  .rxfifo1esize     = (MCAN0_RXFIFO1_ELEMENT_SIZE / 4) + 2,
  .rxbufferecode    = MCAN0_RXBUFFER_ENCODED_SIZE,
  .rxbufferesize    = (MCAN0_RXBUFFER_ELEMENT_SIZE / 4) + 2,
  .txbufferecode    = MCAN0_TXBUFFER_ENCODED_SIZE,
  .txbufferesize    = (MCAN0_TXBUFFER_ELEMENT_SIZE / 4) + 2,

#ifdef CONFIG_SAMV7_MCAN0_LOOPBACK
  .loopback         = true,
#endif

  /* MCAN0 Message RAM */

  .msgram =
  {
     &g_mcan0_msgram[MCAN0_STDFILTER_INDEX],
     &g_mcan0_msgram[MCAN0_EXTFILTERS_INDEX],
     &g_mcan0_msgram[MCAN0_RXFIFO0_INDEX],
     &g_mcan0_msgram[MCAN0_RXFIFO1_INDEX],
     &g_mcan0_msgram[MCAN0_RXDEDICATED_INDEX],
     &g_mcan0_msgram[MCAN0_TXEVENTFIFO_INDEX],
     &g_mcan0_msgram[MCAN0_TXDEDICATED_INDEX],
     &g_mcan0_msgram[MCAN0_TXFIFOQ_INDEX]
  }
};

/* MCAN0 variable driver state */

static struct sam_mcan_s g_mcan0priv;
static struct can_dev_s g_mcan0dev;

#endif /* CONFIG_SAMV7_MCAN0 */

#ifdef CONFIG_SAMV7_MCAN1
/* MCAN1 message RAM allocation */

static uint32_t g_mcan1_msgram[MCAN1_MSGRAM_WORDS];

/* MCAN1 constant configuration */

static const struct sam_config_s g_mcan1const =
{
  .rxpinset         = GPIO_MCAN1_RX,
  .txpinset         = GPIO_MCAN1_TX,
  .handler          = mcan1_interrupt,
  .base             = SAM_MCAN1_BASE,
  .baud             = CONFIG_SAMV7_MCAN1_BITRATE,
  .btp              = MCAN_BTP_BRP(MCAN1_BRP) | MCAN_BTP_TSEG1(MCAN1_TSEG1) |
                      MCAN_BTP_TSEG2(MCAN1_TSEG2) | MCAN_BTP_SJW(MCAN1_SJW),
  .fbtp             = MCAN_FBTP_FBRP(MCAN1_FBRP) | MCAN_FBTP_FTSEG1(MCAN1_FTSEG1) |
                      MCAN_FBTP_FTSEG2(MCAN1_FTSEG2) | MCAN_FBTP_FSJW(MCAN1_FSJW),
  .port             = 1,
  .pid              = SAM_PID_MCAN10,
  .irq              = SAM_IRQ_MCAN10,
#if defined(CONFIG_SAMV7_MCAN1_ISO11899_1)
  .mode             = MCAN_ISO11898_1_MODE,
#elif defined(CONFIG_SAMV7_MCAN1_FD)
  .mode             = MCAN_FD_MODE,
#else /* if defined(CONFIG_SAMV7_MCAN1_FD_BSW) */
  .mode             = MCAN_FD_BSW_MODE,
#endif
  .nstdfilters      = CONFIG_SAMV7_MCAN1_NSTDFILTERS,
  .nextfilters      = CONFIG_SAMV7_MCAN1_NEXTFILTERS,
  .nfifo0           = CONFIG_SAMV7_MCAN1_RXFIFO0_SIZE,
  .nfifo1           = CONFIG_SAMV7_MCAN1_RXFIFO1_SIZE,
  .nrxdedicated     = CONFIG_SAMV7_MCAN0_DEDICATED_RXBUFFER_SIZE,
  .ntxeventfifo     = CONFIG_SAMV7_MCAN1_TXEVENTFIFO_SIZE,
  .ntxdedicated     = CONFIG_SAMV7_MCAN1_DEDICATED_TXBUFFER_SIZE,
  .ntxfifoq         = CONFIG_SAMV7_MCAN1_TXFIFOQ_SIZE,
  .rxfifo0ecode     = MCAN1_RXFIFO0_ENCODED_SIZE,
  .rxfifo0esize     = (MCAN1_RXFIFO0_ELEMENT_SIZE / 4) + 2,
  .rxfifo1ecode     = MCAN1_RXFIFO1_ENCODED_SIZE,
  .rxfifo1esize     = (MCAN1_RXFIFO1_ELEMENT_SIZE / 4) + 2,
  .rxbufferecode    = MCAN1_RXBUFFER_ENCODED_SIZE,
  .rxbufferesize    = (MCAN1_RXBUFFER_ELEMENT_SIZE / 4) + 2,
  .txbufferecode    = MCAN1_TXBUFFER_ENCODED_SIZE,
  .txbufferesize    = (MCAN0_TXBUFFER_ELEMENT_SIZE / 4) + 2,

#ifdef CONFIG_SAMV7_MCAN1_LOOPBACK
  .loopback         = true,
#endif
  /* MCAN0 Message RAM */

  .msgram =
  {
     &g_mcan1_msgram[MCAN1_STDFILTER_INDEX],
     &g_mcan1_msgram[MCAN1_EXTFILTERS_INDEX],
     &g_mcan1_msgram[MCAN1_RXFIFO0_INDEX],
     &g_mcan1_msgram[MCAN1_RXFIFO1_INDEX],
     &g_mcan1_msgram[MCAN1_RXDEDICATED_INDEX],
     &g_mcan1_msgram[MCAN1_TXEVENTFIFO_INDEX],
     &g_mcan1_msgram[MCAN1_TXDEDICATED_INDEX],
     &g_mcan1_msgram[MCAN1_TXFIFOQ_INDEX]
  }
};

/* MCAN0 variable driver state */

static struct sam_mcan_s g_mcan1priv;
static struct can_dev_s g_mcan1dev;

#endif /* CONFIG_SAMV7_MCAN1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcan_getreg
 *
 * Description:
 *   Read the value of a CAN register.
 *
 * Input Parameters:
 *   priv - A reference to the CAN peripheral state
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_MCAN_REGDEBUG
static uint32_t mcan_getreg(FAR struct sam_mcan_s *priv, int offset)
{
  FAR const struct sam_config_s *config = priv->config;
  uintptr_t regaddr;
  uint32_t regval;

  /* Read the value from the register */

  regaddr = config->base + offset;
  regval  = getreg32(regaddr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (regaddr == priv->regaddr && regval == priv->regval)
    {
      if (priv->count == 0xffffffff || ++priv->count > 3)
        {
           if (priv->count == 4)
             {
               lldbg("...\n");
             }

          return regval;
        }
    }

  /* No this is a new address or value */

  else
    {
       /* Did we print "..." for the previous value? */

       if (priv->count > 3)
         {
           /* Yes.. then show how many times the value repeated */

           lldbg("[repeats %d more times]\n", priv->count - 3);
         }

       /* Save the new address, value, and count */

       priv->regaddr = regaddr;
       priv->regval  = regval;
       priv->count   = 1;
    }

  /* Show the register value read */

  lldbg("%08x->%08x\n", regaddr, regval);
  return regval;
}

#else
static uint32_t mcan_getreg(FAR struct sam_mcan_s *priv, int offset)
{
  FAR const struct sam_config_s *config = priv->config;
  return getreg32(config->base + offset);
}

#endif

/****************************************************************************
 * Name: mcan_putreg
 *
 * Description:
 *   Set the value of a CAN register.
 *
 * Input Parameters:
 *   priv - A reference to the CAN peripheral state
 *   offset - The offset to the register to write
 *   regval - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_MCAN_REGDEBUG
static void mcan_putreg(FAR struct sam_mcan_s *priv, int offset, uint32_t regval)
{
  FAR const struct sam_config_s *config = priv->config;
  uintptr_t regaddr = config->base + offset;

  /* Show the register value being written */

  lldbg("%08x<-%08x\n", regaddr, regval);

  /* Write the value */

  putreg32(regval, regaddr);
}

#else
static void mcan_putreg(FAR struct sam_mcan_s *priv, int offset, uint32_t regval)
{
  FAR const struct sam_config_s *config = priv->config;
  putreg32(regval, config->base + offset);
}

#endif

/****************************************************************************
 * Name: mcan_dumpregs
 *
 * Description:
 *   Dump the contents of all CAN control registers
 *
 * Input Parameters:
 *   priv - A reference to the CAN peripheral state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_MCAN_REGDEBUG
static void mcan_dumpregs(FAR struct sam_mcan_s *priv, FAR const char *msg)
{
  FAR const struct sam_config_s *config = priv->config;
  unsigned long addr;

  lldbg("MCAN%d Registers: %s\n", config->port, msg);
  lldbg("   Base: %08x\n", config->base);

  lldbg("   CUST: %08x  FBTP: %08x TEST: %08x    RWD: %08x\n",
        getreg32(config->base + SAM_MCAN_CUST_OFFSET),
        getreg32(config->base + SAM_MCAN_FBTP_OFFSET),
        getreg32(config->base + SAM_MCAN_TEST_OFFSET),
        getreg32(config->base + SAM_MCAN_RWD_OFFSET));

  lldbg("  CCCR: %08x   BTP: %08x  TSCC: %08x   TSCV: %08x\n",
        getreg32(config->base + SAM_MCAN_CCCR_OFFSET),
        getreg32(config->base + SAM_MCAN_BTP_OFFSET),
        getreg32(config->base + SAM_MCAN_TSCC_OFFSET),
        getreg32(config->base + SAM_MCAN_TSCV_OFFSET));

  lldbg("  TOCC: %08x  TOCV: %08x   ECR: %08x    PSR: %08x\n",
        getreg32(config->base + SAM_MCAN_TOCC_OFFSET),
        getreg32(config->base + SAM_MCAN_TOCV_OFFSET),
        getreg32(config->base + SAM_MCAN_ECR_OFFSET),
        getreg32(config->base + SAM_MCAN_PSR_OFFSET));

  lldbg("    IR: %08x    IE: %08x   ILS: %08x    ILE: %08x\n",
        getreg32(config->base + SAM_MCAN_IR_OFFSET),
        getreg32(config->base + SAM_MCAN_IE_OFFSET),
        getreg32(config->base + SAM_MCAN_ILS_OFFSET),
        getreg32(config->base + SAM_MCAN_ILE_OFFSET));

  lldbg("   GFC: %08x SIDFC: %08x XIDFC: %08x  XIDAM: %08x\n",
        getreg32(config->base + SAM_MCAN_GFC_OFFSET),
        getreg32(config->base + SAM_MCAN_SIDFC_OFFSET),
        getreg32(config->base + SAM_MCAN_XIDFC_OFFSET),
        getreg32(config->base + SAM_MCAN_XIDAM_OFFSET));

  lldbg("  HPMS: %08x NDAT1: %08x NDAT2: %08x  RXF0C: %08x\n",
        getreg32(config->base + SAM_MCAN_HPMS_OFFSET),
        getreg32(config->base + SAM_MCAN_NDAT1_OFFSET),
        getreg32(config->base + SAM_MCAN_NDAT2_OFFSET),
        getreg32(config->base + SAM_MCAN_RXF0C_OFFSET));

  lldbg(" RXF0S: %08x FXF0A: %08x  RXBC: %08x  RXF1C: %08x\n",
        getreg32(config->base + SAM_MCAN_RXF0S_OFFSET),
        getreg32(config->base + SAM_MCAN_RXF0A_OFFSET),
        getreg32(config->base + SAM_MCAN_RXBC_OFFSET),
        getreg32(config->base + SAM_MCAN_RXF1C_OFFSET));

  lldbg(" RXF1S: %08x FXF1A: %08x RXESC: %08x   TXBC: %08x\n",
        getreg32(config->base + SAM_MCAN_RXF1S_OFFSET),
        getreg32(config->base + SAM_MCAN_RXF1A_OFFSET),
        getreg32(config->base + SAM_MCAN_RXESC_OFFSET),
        getreg32(config->base + SAM_MCAN_TXBC_OFFSET));

  lldbg(" TXFQS: %08x TXESC: %08x TXBRP: %08x  TXBAR: %08x\n",
        getreg32(config->base + SAM_MCAN_TXFQS_OFFSET),
        getreg32(config->base + SAM_MCAN_TXESC_OFFSET),
        getreg32(config->base + SAM_MCAN_TXBRP_OFFSET),
        getreg32(config->base + SAM_MCAN_TXBAR_OFFSET));

  lldbg(" TXBCR: %08x TXBTO: %08x TXBCF: %08x TXBTIE: %08x\n",
        getreg32(config->base + SAM_MCAN_TXBCR_OFFSET),
        getreg32(config->base + SAM_MCAN_TXBTO_OFFSET),
        getreg32(config->base + SAM_MCAN_TXBCF_OFFSET),
        getreg32(config->base + SAM_MCAN_TXBTIE_OFFSET));

  lldbg("TXBCIE: %08x TXEFC: %08x TXEFS: %08x  TXEFA: %08x\n",
        getreg32(config->base + SAM_MCAN_TXBCIE_OFFSET),
        getreg32(config->base + SAM_MCAN_TXEFC_OFFSET),
        getreg32(config->base + SAM_MCAN_TXEFS_OFFSET),
        getreg32(config->base + SAM_MCAN_TXEFA_OFFSET));
}
#endif

/****************************************************************************
 * Name: mcan_semtake
 *
 * Description:
 *   Take a semaphore handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the CAN peripheral state
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void mcan_semtake(FAR struct sam_mcan_s *priv)
{
  int ret;

  /* Wait until we successfully get the semaphore.  EINTR is the only
   * expected 'failure' (meaning that the wait for the semaphore was
   * interrupted by a signal.
   */

  do
    {
      ret = sem_wait(&priv->exclsem);
      DEBUGASSERT(ret == 0 || errno == EINTR);
    }
  while (ret < 0);
}

/****************************************************************************
 * Name: mcan_recvsetup
 *
 * Description:
 *   Configure and enable mailbox(es) for reception
 *
 * Input Parameter:
 *   priv - A pointer to the private data structure for this CAN peripheral
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Caller has exclusive access to the CAN data structures
 *   CAN interrupts are disabled at the NVIC
 *
 ****************************************************************************/

static int mcan_recvsetup(FAR struct sam_mcan_s *priv)
{
  FAR const struct sam_config_s *config = priv->config;

#warning Missing logic
  return OK;
}

/****************************************************************************
 * Name: mcan_reset
 *
 * Description:
 *   Reset the CAN device.  Called early to initialize the hardware. This
 *   function is called, before mcan_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void mcan_reset(FAR struct can_dev_s *dev)
{
  FAR struct sam_mcan_s *priv;
  FAR const struct sam_config_s *config;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  canllvdbg("CAN%d\n", config->port);
  UNUSED(config);

  /* Get exclusive access to the CAN peripheral */

  mcan_semtake(priv);

  /* Disable all interrupts */

  mcan_putreg(priv, SAM_MCAN_IE_OFFSET, 0);
  mcan_putreg(priv, SAM_MCAN_TXBTIE_OFFSET, 0);

  /* Disable the CAN controller */
#warning Missing logic
  mcan_semgive(priv);
}

/****************************************************************************
 * Name: mcan_setup
 *
 * Description:
 *   Configure the CAN. This method is called the first time that the CAN
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching CAN interrupts.
 *   All CAN interrupts are disabled upon return.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int mcan_setup(FAR struct can_dev_s *dev)
{
  FAR struct sam_mcan_s *priv;
  FAR const struct sam_config_s *config;
  int ret;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  canllvdbg("CAN%d pid: %d\n", config->port, config->pid);

  /* Get exclusive access to the CAN peripheral */

  mcan_semtake(priv);

  /* CAN hardware initialization */

  ret = mcan_hwinitialize(priv);
  if (ret < 0)
    {
      canlldbg("CAN%d H/W initialization failed: %d\n", config->port, ret);
      return ret;
    }

  mcan_dumpregs(priv, "After hardware initialization");

  /* Attach the CAN interrupt handler */

  ret = irq_attach(config->irq, config->handler);
  if (ret < 0)
    {
      canlldbg("Failed to attach CAN%d IRQ (%d)", config->port, config->irq);
      return ret;
    }

  /* Setup receive mailbox(es) (enabling receive interrupts) */

  ret = mcan_recvsetup(priv);
  if (ret < 0)
    {
      canlldbg("CAN%d H/W initialization failed: %d\n", config->port, ret);
      return ret;
    }

  mcan_dumpregs(priv, "After receive setup");

  /* Enable the interrupts at the NVIC (they are still disabled at the MCAN
   * peripheral). */

  up_enable_irq(config->irq);
  mcan_semgive(priv);
  return OK;
}

/****************************************************************************
 * Name: mcan_shutdown
 *
 * Description:
 *   Disable the CAN.  This method is called when the CAN device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mcan_shutdown(FAR struct can_dev_s *dev)
{
  FAR struct sam_mcan_s *priv;
  FAR const struct sam_config_s *config;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  canllvdbg("CAN%d\n", config->port);

  /* Get exclusive access to the CAN peripheral */

  mcan_semtake(priv);

  /* Disable the CAN interrupts */

  up_disable_irq(config->irq);

  /* Detach the CAN interrupt handler */

  irq_detach(config->irq);

  /* And reset the hardware */

  mcan_reset(dev);
  mcan_semgive(priv);
}

/****************************************************************************
 * Name: mcan_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mcan_rxint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct sam_mcan_s *priv = dev->cd_priv;
  irqstate_t flags;
  uint32_t regval;

  DEBUGASSERT(priv && priv->config);

  canllvdbg("CAN%d enable: %d\n", priv->config->port, enable);

  /* Enable/disable the receive interrupts */

  flags = irqsave();
  regval = mcan_getreg(priv, SAM_MCAN_IE_OFFSET);

  if (enable)
    {
      regval |= MCAN_RXINTERRUPTS;
    }
  else
    {
      regval &= ~MCAN_RXINTERRUPTS;
    }

  mcan_putreg(priv, SAM_MCAN_IE_OFFSET, regval);
  irqrestore(flags);
}

/****************************************************************************
 * Name: mcan_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mcan_txint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct sam_mcan_s *priv = dev->cd_priv;
  irqstate_t flags;
  uint32_t regval;

  DEBUGASSERT(priv && priv->config);

  canllvdbg("CAN%d enable: %d\n", priv->config->port, enable);

  /* Enable/disable the receive interrupts */

  flags = irqsave();
  regval = mcan_getreg(priv, SAM_MCAN_IE_OFFSET);

  if (enable)
    {
      regval |= MCAN_TXINTERRUPTS;
      priv->txenabled = true;
    }
  else
    {
      regval &= ~MCAN_TXINTERRUPTS;
      priv->txenabled = false;
    }

  mcan_putreg(priv, SAM_MCAN_IE_OFFSET, regval);
  irqrestore(flags);
}

/****************************************************************************
 * Name: mcan_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int mcan_ioctl(FAR struct can_dev_s *dev, int cmd, unsigned long arg)
{
  /* No CAN ioctls are supported */

  return -ENOTTY;
}

/****************************************************************************
 * Name: mcan_remoterequest
 *
 * Description:
 *   Send a remote request
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int mcan_remoterequest(FAR struct can_dev_s *dev, uint16_t id)
{
  /* REVISIT:  Remote request not implemented */

  return -ENOSYS;
}

/****************************************************************************
 * Name: mcan_send
 *
 * Description:
 *    Send one can message.
 *
 *    One CAN-message consists of a maximum of 10 bytes.  A message is
 *    composed of at least the first 2 bytes (when there are no data bytes).
 *
 *    Byte 0:      Bits 0-7: Bits 3-10 of the 11-bit CAN identifier
 *    Byte 1:      Bits 5-7: Bits 0-2 of the 11-bit CAN identifier
 *                 Bit 4:    Remote Tranmission Request (RTR)
 *                 Bits 0-3: Data Length Code (DLC)
 *    Bytes 2-10: CAN data
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int mcan_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg)
{
  FAR struct sam_mcan_s *priv;
  irqstate_t flags;
  uint32_t regval;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv && priv->config);

  canllvdbg("CAN%d\n", priv->config->port);
  canllvdbg("CAN%d ID: %d DLC: %d\n",
            priv->config->port, msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

#ifdef CONFIG_CAN_EXTID
  DEBUGASSERT(msg->cm_hdr.ch_extid);
  DEBUGASSERT(msg->cm_hdr.ch_id < (1 << 29));
#else
  DEBUGASSERT(!msg->cm_hdr.ch_extid);
  DEBUGASSERT(msg->cm_hdr.ch_id < (1 << 11));
#endif

  /* Get exclusive access to the CAN peripheral */

  mcan_semtake(priv);

#warning Missing logic

  /* If we have not been asked to suppress TX interrupts, then enable
   * TX interrupts now.
   */

  if (priv->txenabled)
    {
      flags   = irqsave();
      regval  = mcan_getreg(priv, SAM_MCAN_IE_OFFSET);
      regval |= MCAN_TXINTERRUPTS;
      mcan_putreg(priv, SAM_MCAN_IE_OFFSET, MCAN_TXINTERRUPTS);
      irqrestore(flags);
    }

  mcan_semgive(priv);
  return OK;
}

/****************************************************************************
 * Name: mcan_txready
 *
 * Description:
 *   Return true if the CAN hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if the CAN hardware is ready to accept another TX message.
 *
 ****************************************************************************/

static bool mcan_txready(FAR struct can_dev_s *dev)
{
  FAR struct sam_mcan_s *priv = dev->cd_priv;
  bool txready;

  /* Get exclusive access to the CAN peripheral */

  mcan_semtake(priv);

#warning Missing logic

  mcan_semgive(priv);
  return txready;
}

/****************************************************************************
 * Name: mcan_txempty
 *
 * Description:
 *   Return true if all message have been sent.  If for example, the CAN
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if there are no pending TX transfers in the CAN hardware.
 *
 ****************************************************************************/

static bool mcan_txempty(FAR struct can_dev_s *dev)
{
  FAR struct sam_mcan_s *priv = dev->cd_priv;
  bool txempty;

  /* Get exclusive access to the CAN peripheral */

  mcan_semtake(priv);

#warning Missing logic

  mcan_semgive(priv);
  return txempty;
}

/****************************************************************************
 * Name: mcan_rxinterrupt
 *
 * Description:
 *   CAN RX mailbox interrupt handler
 *
 * Input Parameters:
 *   priv - CAN-specific private data
 *   mbndx - The index of the mailbox that generated the interrupt
 *   msr - Applicable value from the mailbox status register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void mcan_rxinterrupt(FAR struct can_dev_s *dev, int mbndx,
                                    uint32_t msr)
{
  FAR struct sam_mcan_s *priv = dev->cd_priv;
  struct can_hdr_s hdr;
  int ret;

#warning Missing logic
}

/****************************************************************************
 * Name: mcan_txinterrupt
 *
 * Description:
 *   CAN TX mailbox interrupt handler
 *
 * Input Parameters:
 *   priv - CAN-specific private data
 *   mbndx - The index of the mailbox that generated the interrupt
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void mcan_txinterrupt(FAR struct can_dev_s *dev, int mbndx)
{
  FAR struct sam_mcan_s *priv = dev->cd_priv;

#warning Missing logic
}

/****************************************************************************
 * Name: mcan_interrupt
 *
 * Description:
 *   Common CAN interrupt handler
 *
 * Input Parameters:
 *   priv - CAN-specific private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mcan_interrupt(FAR struct can_dev_s *dev)
{
  FAR struct sam_mcan_s *priv = dev->cd_priv;
  uint32_t ir;
  uint32_t ie;
  uint32_t pending;

  DEBUGASSERT(priv && priv->config);

  /* Get the set of pending interrupts. */

  ir = mcan_getreg(priv, SAM_MCAN_IR_OFFSET);
  ie = mcan_getreg(priv, SAM_MCAN_IE_OFFSET);

  pending = (ir & ie);

  /* Check for transmission errors */
#warning Missing logic

  /* Check for successful completion of a transmission */

  if ((pending & MCAN_INT_TC) != 0)
    {
      /* Clear the pending TX completion interrupt */

      mcan_putreg(priv, SAM_MCAN_IE_OFFSET, MCAN_INT_TC);
    }

  /* Check for reception errors */
#warning Missing logic

  /* Check for successful reception of a message */
#warning Missing logic

}

/****************************************************************************
 * Name: mcan0_interrupt
 *
 * Description:
 *   MCAN0 interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_MCAN0
static int mcan0_interrupt(int irq, void *context)
{
  mcan_interrupt(&g_mcan0dev);
  return OK;
}
#endif

/****************************************************************************
 * Name: mcan1_interrupt
 *
 * Description:
 *   MCAN1 interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_MCAN1
static int mcan1_interrupt(int irq, void *context)
{
  mcan_interrupt(&g_mcan1dev);
  return OK;
}
#endif

/****************************************************************************
 * Name: mcan_hwinitialize
 *
 * Description:
 *   CAN cell initialization
 *
 * Input Parameter:
 *   priv - A pointer to the private data structure for this CAN peripheral
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int mcan_hwinitialize(struct sam_mcan_s *priv)
{
  FAR const struct sam_config_s *config = priv->config;
  FAR uint32_t *msgram;
  uint32_t regval;
  uint32_t cntr;
  uint32_t cmr;
  int ret;

  canllvdbg("CAN%d\n", config->port);

  /* Configure CAN pins */

  sam_configgpio(config->rxpinset);
  sam_configgpio(config->txpinset);

#if 0 // REVISIT -- may apply only to SAMA5
  /* Determine the maximum CAN peripheral clock frequency */

  mck = BOARD_MCK_FREQUENCY;
  if (mck <= SAM_MCAN_MAXPERCLK)
    {
      priv->frequency = mck;
      regval          = PMC_PCR_DIV1;
    }
  else if ((mck >> 1) <= SAM_MCAN_MAXPERCLK)
    {
      priv->frequency = (mck >> 1);
      regval          = PMC_PCR_DIV2;
    }
  else if ((mck >> 2) <= SAM_MCAN_MAXPERCLK)
    {
      priv->frequency = (mck >> 2);
      regval          = PMC_PCR_DIV4;
    }
  else if ((mck >> 3) <= SAM_MCAN_MAXPERCLK)
    {
      priv->frequency = (mck >> 3);
      regval          = PMC_PCR_DIV8;
    }
  else
    {
      candbg("ERROR: Cannot realize CAN input frequency\n");
      return -EINVAL;
    }

  /* Set the maximum CAN peripheral clock frequency */

  regval |= PMC_PCR_PID(config->pid) | PMC_PCR_CMD | PMC_PCR_EN;
  mcan_putreg(priv, SAM_PMC_PCR, regval);
#endif

  /* Enable peripheral clocking */

  sam_enableperiph1(config->pid);

  /* Enable the Initialization state */

  regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
  regval |= MCAN_CCCR_INIT;
  mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);

  /* Wait for initialization mode to take effect */

  while ((mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET) & MCAN_CCCR_INIT) == 0);

  /* Enable writing to configuration registers */

  regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
  regval |= (MCAN_CCCR_INIT | MCAN_CCCR_CCE);
  mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);

  /* Global Filter Configuration: Reject remote frames, reject non-matching
   * frames.
   */

  regval = MCAN_GFC_RRFE | MCAN_GFC_RRFS | MCAN_GFC_ANFE_REJECTED |
           MCAN_GFC_ANFS_REJECTED;
  mcan_putreg(priv, SAM_MCAN_GFC_OFFSET, regval);

  /* Extended ID Filter AND mask  */

  mcan_putreg(priv, SAM_MCAN_XIDAM_OFFSET, 0x1fffffff);

  /* Disable all interrupts  */

  mcan_putreg(priv, SAM_MCAN_IE_OFFSET, 0);
  mcan_putreg(priv, SAM_MCAN_TXBTIE_OFFSET, 0);

  /* All interrupts directed to Line 0.  But disable bot interrupt line 0
   * and 1 for now.
   */

  mcan_putreg(priv, SAM_MCAN_ILS_OFFSET, 0);
  mcan_putreg(priv, SAM_MCAN_ILE_OFFSET, 0);

  /* Clear all pending interrupts. */

  mcan_putreg(priv, SAM_MCAN_IR_OFFSET, MCAN_INT_ALL);

  /* Configure MCAN bit timing */

  mcan_putreg(priv, SAM_MCAN_BTP_OFFSET, config->btp);
  mcan_putreg(priv, SAM_MCAN_FBTP_OFFSET, config->fbtp);

  /* Configure message RAM starting addresses and sizes. */

  regval = MAILBOX_ADDRESS(config->msgram.stdfilters) |
           MCAN_SIDFC_LSS(config->nstdfilters);
  mcan_putreg(priv, SAM_MCAN_SIDFC_OFFSET, regval);

  regval = MAILBOX_ADDRESS(config->msgram.extfilters) |
           MCAN_XIDFC_LSE(config->nextfilters);
  mcan_putreg(priv, SAM_MCAN_XIDFC_OFFSET, regval);

  /* Configure RX FIFOs */

  regval = MAILBOX_ADDRESS(config->msgram.rxfifo0) |
           MCAN_RXF0C_F0S(config->nfifo0);
  mcan_putreg(priv, SAM_MCAN_RXF0C_OFFSET, regval);

  regval = MAILBOX_ADDRESS(config->msgram.rxfifo1) |
           MCAN_RXF1C_F1S(config->nfifo1);
  mcan_putreg(priv, SAM_MCAN_RXF1C_OFFSET, regval);

  /* Watermark interrupt off, blocking mode */

  regval = MAILBOX_ADDRESS(config->msgram.rxdedicated);
  mcan_putreg(priv, SAM_MCAN_RXBC_OFFSET, regval);

  regval = MAILBOX_ADDRESS(config->msgram.txeventfifo) |
           MCAN_TXEFC_EFS(config->ntxeventfifo);
  mcan_putreg(priv, SAM_MCAN_TXEFC_OFFSET, regval);

  /* Watermark interrupt off */

  regval = MAILBOX_ADDRESS(config->msgram.txdedicated) |
           MCAN_TXBC_NDTB(config->ntxdedicated) |
           MCAN_TXBC_TFQS(config->ntxfifoq);
  mcan_putreg(priv, SAM_MCAN_TXBC_OFFSET, regval);

  regval = MCAN_RXESC_RBDS(config->rxbufferecode) |
           MCAN_RXESC_F1DS(config->rxfifo1ecode) |
           MCAN_RXESC_F0DS(config->rxfifo0ecode);
  mcan_putreg(priv, SAM_MCAN_RXESC_OFFSET, regval);

  regval = MCAN_TXESC_TBDS(config->txbufferesize);
  mcan_putreg(priv, SAM_MCAN_TXESC_OFFSET, regval);

  /* Configure Message Filters */
  /* Disable all standard filters */

  msgram = config->msgram.stdfilters;
  cntr   = config->nstdfilters;
  while (cntr > 0)
    {
      *msgram++ = STDFILTER_S0_SFEC_DISABLE;
      cntr--;
    }

  /* Disable all extended filters */

  msgram = config->msgram.extfilters;
  cntr = config->nextfilters;
  while (cntr > 0)
    {
      *msgram = EXTFILTER_F0_EFEC_DISABLE;
      msgram = msgram + 2;
      cntr--;
    }

  /* Clear new RX data flags */

  mcan_putreg(priv, SAM_MCAN_NDAT1_OFFSET, 0xffffffff);
  mcan_putreg(priv, SAM_MCAN_NDAT2_OFFSET, 0xffffffff);

  /* Select ISO11898-1 mode or FD mode with or without fast bit rate
   * switching
   */

  regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
  regval &= ~(MCAN_CCCR_CME_MASK | MCAN_CCCR_CMR_MASK);

  switch (config->mode)
    {
    default:
    case MCAN_ISO11898_1_MODE:
      regval |= MCAN_CCCR_CME_ISO11898_1;
      cmr     = MCAN_CCCR_CMR_ISO11898_1;
      break;

    case MCAN_FD_MODE:
      regval |= MCAN_CCCR_CME_FD;
      cmr     = MCAN_CCCR_CMR_FD;
      break;

    case MCAN_FD_BSW_MODE:
      regval |= MCAN_CCCR_CME_FD_BSW;
      cmr     = MCAN_CCCR_CMR_FD_BSW;
      break;
    }

  /* Set the initial CAN mode */

  mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);

  /* Request the mode change */

  regval |= cmr;
  mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);

#if 0 /* Not necessary in initialization mode */
  /* Wait for the mode to take effect */

  while ((mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET) & (MCAN_CCCR_FDBS | MCAN_CCCR_FDO)) != 0);
#endif

  /* Enable FIFO/Queue mode */

  regval  = mcan_getreg(priv, SAM_MCAN_TXBC_OFFSET);
  regval |= MCAN_TXBC_TFQM;
  mcan_putreg(priv, SAM_MCAN_TXBC_OFFSET, regval);

#ifdef SAMV7_MCAN_LOOPBACK
  /* Is loopback mode selected for this peripheral? */

  if (config->loopback)
    {
     /* MCAN_CCCR_TEST  - Test mode enable
      * MCAN_CCCR_MON   - Bus monitoring mode (for internal loopback)
      * MCAN_TEST_LBCK  - Loopback mode
      */

      regval = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
      regval |= (MCAN_CCCR_TEST | MCAN_CCCR_MON);
      mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);

      regval = mcan_getreg(priv, SAM_MCAN_TEST_OFFSET);
      regval |= MCAN_TEST_LBCK;
      mcan_putreg(priv, SAM_MCAN_TEST_OFFSET, regval);
    }
#endif

  /* Disable initialization mode to enable normal operation */

  regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
  regval &= ~MCAN_CCCR_INIT;
  mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_mcan_initialize
 *
 * Description:
 *   Initialize the selected MCAN port
 *
 * Input Parameter:
 *   port - Port number (for hardware that has multiple CAN interfaces),
 *          0=MCAN0, 1=NCAN1
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct can_dev_s *sam_mcan_initialize(int port)
{
  FAR struct can_dev_s *dev;
  FAR struct sam_mcan_s *priv;
  FAR const struct sam_config_s *config;
  uint32_t regval;

  canvdbg("MCAN%d\n", port);

  /* Select PCK5 clock source and pre-scaler value.  Both MCAN controllers
   * use PCK5 to derive bit rate.
   */

  regval = PMC_PCK_PRES(CONFIG_SAMV7_MCAN_CLKSRC_PRESCALER) | SAMV7_MCAN_CLKSRC;
  putreg32(regval, SAM_PMC_PCK5);

  /* Enable PCK5 */

  putreg32(PMC_PCK5, SAM_PMC_SCER);

  /* Select MCAN peripheral to be initialized */

#ifdef CONFIG_SAMV7_MCAN0
  if (port == MCAN0)
    {
      /* Select the MCAN0 device structure */

      dev    = &g_mcan0dev;
      priv   = &g_mcan0priv;
      config = &g_mcan0const;

      /* Configure MCAN0 Message RAM Base Address */

      regval  = getreg32(SAM_MATRIX_CAN0);
      regval &= MATRIX_CAN0_RESERVED;
      regval |= (uint32_t)config->msgram.stdfilters & MATRIX_CAN0_CAN0DMABA_MASK;
      putreg32(regval, SAM_MATRIX_CAN0);
    }
  else
#endif
#ifdef CONFIG_SAMV7_MCAN1
  if (port == MCAN1)
    {
      /* Select the MCAN1 device structure */

      dev    = &g_mcan1dev;
      priv   = &g_mcan1priv;
      config = &g_mcan1const;

      /* Configure MCAN1 Message RAM Base Address */

      regval  = getreg32(SAM_MATRIX_CCFG_SYSIO);
      regval &= ~MATRIX_CCFG_CAN1DMABA_MASK;
      regval |= (uint32_t)config->msgram.stdfilters & MATRIX_CCFG_CAN1DMABA_MASK;
      putreg32(regval, SAM_MATRIX_CCFG_SYSIO);
    }
  else
#endif
    {
      candbg("ERROR: Unsupported port %d\n", port);
      return NULL;
    }

  /* Is this the first time that we have handed out this device? */

  if (!priv->initialized)
    {
      /* Yes, then perform one time data initialization */

      memset(priv, 0, sizeof(struct sam_mcan_s));
      priv->config      = config;
      priv->initialized = true;

      sem_init(&priv->exclsem, 0, 1);

      dev->cd_ops       = &g_mcanops;
      dev->cd_priv      = (FAR void *)priv;

      /* And put the hardware in the initial state */

      mcan_reset(dev);
    }

  return dev;
}

#endif /* CONFIG_CAN && CONFIG_SAMV7_MCAN */

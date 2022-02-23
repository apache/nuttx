/****************************************************************************
 * arch/arm/src/samv7/sam_mcan.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* References:
 *   SAMV7D3 Series Data Sheet
 *   Atmel sample code
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/can/can.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "hardware/sam_matrix.h"
#include "hardware/sam_chipid.h"
#include "hardware/sam_pinmap.h"
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

/* Buffer Alignment *********************************************************/

/* Buffer Alignment.
 *
 * The MCAN peripheral does not require any data be aligned.  However, if
 * the data cache is enabled then alignment is required.  That is because
 * the data will need to be invalidated and that cache invalidation will
 * occur in multiples of full change lines.
 */

#ifdef CONFIG_ARMV7M_DCACHE
#  define MCAN_ALIGN        ARMV7M_DCACHE_LINESIZE
#  define MCAN_ALIGN_MASK   (MCAN_ALIGN-1)
#  define MCAN_ALIGN_UP(n)  (((n) + MCAN_ALIGN_MASK) & ~MCAN_ALIGN_MASK)

#  ifndef CONFIG_ARMV7M_DCACHE_WRITETHROUGH
#    warning !!! This driver will not work without CONFIG_ARMV7M_DCACHE_WRITETHROUGH=y!!!
#  endif
#endif

/* General Configuration ****************************************************/

#ifndef CONFIG_CAN_TXREADY
#  warning WARNING!!! CONFIG_CAN_TXREADY is required by this driver
#endif

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
#    error Invalid MCAN0 NTSEG1
#  endif
#  if MCAN0_TSEG2 > 15
#    error Invalid MCAN0 NTSEG2
#  endif
#  if MCAN0_SJW > 15
#    error Invalid MCAN0 NSJW
#  endif

#  define MCAN0_DTSEG1 (CONFIG_SAMV7_MCAN0_FPROPSEG + CONFIG_SAMV7_MCAN0_FPHASESEG1)
#  define MCAN0_DTSEG2 (CONFIG_SAMV7_MCAN0_FPHASESEG2)
#  define MCAN0_DBRP   ((uint32_t)(((float) SAMV7_MCANCLK_FREQUENCY / \
                       ((float)(MCAN0_DTSEG1 + MCAN0_DTSEG2 + 3) * \
                        (float)CONFIG_SAMV7_MCAN0_FBITRATE)) - 1))
#  define MCAN0_DSJW   (CONFIG_SAMV7_MCAN0_FFSJW - 1)

#  if MCAN0_DTSEG1 > 15
#    error Invalid MCAN0 DTSEG1
#  endif
#  if MCAN0_DTSEG2 > 7
#    error Invalid MCAN0 DTSEG2
#  endif
#  if MCAN0_DSJW > 3
#    error Invalid MCAN0 DSJW
#  endif

/* MCAN0 RX FIFO0 element size */

#  if defined(CONFIG_SAMV7_MCAN0_RXFIFO0_8BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO0_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO0_12BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  12
#    define MCAN0_RXFIFO0_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO0_16BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  16
#    define MCAN0_RXFIFO0_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO0_20BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  20
#    define MCAN0_RXFIFO0_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO0_24BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  24
#    define MCAN0_RXFIFO0_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO0_32BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  32
#    define MCAN0_RXFIFO0_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO0_48BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  48
#    define MCAN0_RXFIFO0_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO0_64BYTES)
#    define MCAN0_RXFIFO0_ELEMENT_SIZE  64
#    define MCAN0_RXFIFO0_ENCODED_SIZE  7
#  else
#    error Undefined MCAN0 RX FIFO0 element size
#  endif

#  ifndef CONFIG_SAMV7_MCAN0_RXFIFO0_SIZE
#    define CONFIG_SAMV7_MCAN0_RXFIFO0_SIZE 0
#  endif

#  if CONFIG_SAMV7_MCAN0_RXFIFO0_SIZE > 64
#    error Invalid MCAN0 number of RX FIFO0 elements
#  endif

#  define MCAN0_RXFIFO0_BYTES \
     MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN0_RXFIFO0_SIZE * \
                   MCAN0_RXFIFO0_ELEMENT_SIZE + 8)
#  define MCAN0_RXFIFO0_WORDS (MCAN0_RXFIFO0_BYTES >> 2)

/* MCAN0 RX FIFO1 element size */

#  if defined(CONFIG_SAMV7_MCAN0_RXFIFO1_8BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN0_RXFIFO1_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO1_12BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  12
#    define MCAN0_RXFIFO1_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO1_16BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  16
#    define MCAN0_RXFIFO1_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO1_20BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  20
#    define MCAN0_RXFIFO1_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO1_24BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  24
#    define MCAN0_RXFIFO1_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO1_32BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  32
#    define MCAN0_RXFIFO1_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO1_48BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  48
#    define MCAN0_RXFIFO1_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN0_RXFIFO1_64BYTES)
#    define MCAN0_RXFIFO1_ELEMENT_SIZE  64
#    define MCAN0_RXFIFO1_ENCODED_SIZE  7
#  else
#    error Undefined MCAN0 RX FIFO1 element size
#  endif

#  ifndef CONFIG_SAMV7_MCAN0_RXFIFO1_SIZE
#    define CONFIG_SAMV7_MCAN0_RXFIFO1_SIZE 0
#  endif

#  if CONFIG_SAMV7_MCAN0_RXFIFO1_SIZE > 64
#    error Invalid MCAN0 number of RX FIFO1 elements
#  endif

#  define MCAN0_RXFIFO1_BYTES \
     MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN0_RXFIFO1_SIZE * \
                   MCAN0_RXFIFO1_ELEMENT_SIZE + 8)
#  define MCAN0_RXFIFO1_WORDS (MCAN0_RXFIFO1_BYTES >> 2)

/* MCAN0 Filters */

#  ifndef CONFIG_SAMV7_MCAN0_NSTDFILTERS
#    define CONFIG_SAMV7_MCAN0_NSTDFILTERS 0
#  endif

#  if (CONFIG_SAMV7_MCAN0_NSTDFILTERS > 128)
#    error Invalid MCAN0 number of Standard Filters
#  endif

#  ifndef CONFIG_SAMV7_MCAN0_NEXTFILTERS
#    define CONFIG_SAMV7_MCAN0_NEXTFILTERS 0
#  endif

#  if (CONFIG_SAMV7_MCAN0_NEXTFILTERS > 64)
#    error Invalid MCAN0 number of Extended Filters
#  endif

#  define MCAN0_STDFILTER_BYTES \
      MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN0_NSTDFILTERS << 2)
#  define MCAN0_STDFILTER_WORDS (MCAN0_STDFILTER_BYTES >> 2)

#  define MCAN0_EXTFILTER_BYTES \
      MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN0_NEXTFILTERS << 3)
#  define MCAN0_EXTFILTER_WORDS (MCAN0_EXTFILTER_BYTES >> 2)

/* MCAN0 RX buffer element size */

#  if defined(CONFIG_SAMV7_MCAN0_RXBUFFER_8BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_RXBUFFER_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN0_RXBUFFER_12BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  12
#    define MCAN0_RXBUFFER_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN0_RXBUFFER_16BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  16
#    define MCAN0_RXBUFFER_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN0_RXBUFFER_20BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  20
#    define MCAN0_RXBUFFER_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN0_RXBUFFER_24BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  24
#    define MCAN0_RXBUFFER_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN0_RXBUFFER_32BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  32
#    define MCAN0_RXBUFFER_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN0_RXBUFFER_48BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  48
#    define MCAN0_RXBUFFER_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN0_RXBUFFER_64BYTES)
#    define MCAN0_RXBUFFER_ELEMENT_SIZE  64
#    define MCAN0_RXBUFFER_ENCODED_SIZE  7
#  else
#    error Undefined MCAN0 RX buffer element size
#  endif

#  ifndef CONFIG_SAMV7_MCAN0_DEDICATED_RXBUFFER_SIZE
#    define CONFIG_SAMV7_MCAN0_DEDICATED_RXBUFFER_SIZE 0
#  endif

#  if CONFIG_SAMV7_MCAN0_DEDICATED_RXBUFFER_SIZE > 64
#    error Invalid MCAN0 number of RX BUFFER elements
#  endif

#  define MCAN0_DEDICATED_RXBUFFER_BYTES \
     MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN0_DEDICATED_RXBUFFER_SIZE * \
                   MCAN0_RXBUFFER_ELEMENT_SIZE + 8)
#  define MCAN0_DEDICATED_RXBUFFER_WORDS \
     (MCAN0_DEDICATED_RXBUFFER_BYTES >> 2)

/* MCAN0 TX buffer element size */

#  if defined(CONFIG_SAMV7_MCAN0_TXBUFFER_8BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN0_TXBUFFER_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN0_TXBUFFER_12BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  12
#    define MCAN0_TXBUFFER_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN0_TXBUFFER_16BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  16
#    define MCAN0_TXBUFFER_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN0_TXBUFFER_20BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  20
#    define MCAN0_TXBUFFER_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN0_TXBUFFER_24BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  24
#    define MCAN0_TXBUFFER_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN0_TXBUFFER_32BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  32
#    define MCAN0_TXBUFFER_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN0_TXBUFFER_48BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  48
#    define MCAN0_TXBUFFER_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN0_TXBUFFER_64BYTES)
#    define MCAN0_TXBUFFER_ELEMENT_SIZE  64
#    define MCAN0_TXBUFFER_ENCODED_SIZE  7
#  else
#    error Undefined MCAN0 TX buffer element size
#  endif

#  ifndef CONFIG_SAMV7_MCAN0_DEDICATED_TXBUFFER_SIZE
#    define CONFIG_SAMV7_MCAN0_DEDICATED_TXBUFFER_SIZE 0
#  endif

#  define MCAN0_DEDICATED_TXBUFFER_BYTES \
     MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN0_DEDICATED_TXBUFFER_SIZE * \
                   MCAN0_TXBUFFER_ELEMENT_SIZE + 8)
#  define MCAN0_DEDICATED_TXBUFFER_WORDS \
     (MCAN0_DEDICATED_TXBUFFER_BYTES >> 2)

/* MCAN0 TX FIFOs */

#  ifndef CONFIG_SAMV7_MCAN0_TXFIFOQ_SIZE
#    define CONFIG_SAMV7_MCAN0_TXFIFOQ_SIZE 0
#  endif

#  if (CONFIG_SAMV7_MCAN0_DEDICATED_TXBUFFER_SIZE + \
       CONFIG_SAMV7_MCAN0_TXFIFOQ_SIZE) > 32
#    error Invalid MCAN0 number of TX BUFFER elements
#  endif

#  ifndef CONFIG_SAMV7_MCAN0_TXEVENTFIFO_SIZE
#    define CONFIG_SAMV7_MCAN0_TXEVENTFIFO_SIZE 0
#  endif

#  if CONFIG_SAMV7_MCAN0_TXEVENTFIFO_SIZE > 32
#    error Invalid MCAN0 number of TX EVENT FIFO elements
#  endif

#  define MCAN0_TXEVENTFIFO_BYTES \
     MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN0_TXEVENTFIFO_SIZE << 3)
#  define MCAN0_TXEVENTFIFO_WORDS \
     (MCAN0_TXEVENTFIFO_BYTES >> 2)

#  define MCAN0_TXFIFIOQ_BYTES \
     MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN0_TXFIFOQ_SIZE *  \
                   MCAN0_TXBUFFER_ELEMENT_SIZE + 8)
#  define MCAN0_TXFIFIOQ_WORDS (MCAN0_TXFIFIOQ_BYTES >> 2)

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

#  if MCAN1_NTSEG1 > 63
#    error Invalid MCAN1 NTSEG1
#  endif
#  if MCAN1_NTSEG2 > 15
#    error Invalid MCAN1 NTSEG2
#  endif
#  if MCAN1_NSJW > 15
#    error Invalid MCAN1 NSJW
#  endif

#  define MCAN1_DTSEG1 (CONFIG_SAMV7_MCAN1_FPROPSEG + CONFIG_SAMV7_MCAN1_FPHASESEG1)
#  define MCAN1_DTSEG2 (CONFIG_SAMV7_MCAN1_FPHASESEG2)
#  define MCAN1_DBRP   ((uint32_t)(((float) SAMV7_MCANCLK_FREQUENCY / \
                       ((float)(MCAN1_DTSEG1 + MCAN1_DTSEG2 + 3) * \
                        (float)CONFIG_SAMV7_MCAN1_FBITRATE)) - 1))
#  define MCAN1_DSJW   (CONFIG_SAMV7_MCAN1_FFSJW - 1)

#if MCAN1_DTSEG1 > 15
#  error Invalid MCAN1 DTSEG1
#endif
#if MCAN1_DTSEG2 > 7
#  error Invalid MCAN1 DTSEG2
#endif
#if MCAN1_DSJW > 3
#  error Invalid MCAN1 DSJW
#endif

/* MCAN1 RX FIFO0 element size */

#  if defined(CONFIG_SAMV7_MCAN1_RXFIFO0_8BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO0_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO0_12BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  12
#    define MCAN1_RXFIFO0_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO0_16BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  16
#    define MCAN1_RXFIFO0_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO0_20BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  20
#    define MCAN1_RXFIFO0_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO0_24BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  24
#    define MCAN1_RXFIFO0_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO0_32BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  32
#    define MCAN1_RXFIFO0_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO0_48BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  48
#    define MCAN1_RXFIFO0_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO0_64BYTES)
#    define MCAN1_RXFIFO0_ELEMENT_SIZE  64
#    define MCAN1_RXFIFO0_ENCODED_SIZE  7
#  else
#    error Undefined MCAN1 RX FIFO0 element size
#  endif

#  ifndef CONFIG_SAMV7_MCAN1_RXFIFO0_SIZE
#    define CONFIG_SAMV7_MCAN1_RXFIFO0_SIZE 0
#  endif

#  if CONFIG_SAMV7_MCAN1_RXFIFO0_SIZE > 64
#    error Invalid MCAN1 number of RX FIFO 0 elements
#  endif

#  define MCAN1_RXFIFO0_BYTES \
     MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN1_RXFIFO0_SIZE * \
                   MCAN1_RXFIFO0_ELEMENT_SIZE + 8)
#  define MCAN1_RXFIFO0_WORDS (MCAN1_RXFIFO0_BYTES >> 2)

/* MCAN1 RX FIFO1 element size */

#  if defined(CONFIG_SAMV7_MCAN1_RXFIFO1_8BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  8
#    define MCAN1_RXFIFO1_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO1_12BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  12
#    define MCAN1_RXFIFO1_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO1_16BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  16
#    define MCAN1_RXFIFO1_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO1_20BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  20
#    define MCAN1_RXFIFO1_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO1_24BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  24
#    define MCAN1_RXFIFO1_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO1_32BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  32
#    define MCAN1_RXFIFO1_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO1_48BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  48
#    define MCAN1_RXFIFO1_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN1_RXFIFO1_64BYTES)
#    define MCAN1_RXFIFO1_ELEMENT_SIZE  64
#    define MCAN1_RXFIFO1_ENCODED_SIZE  7
#  else
#    error Undefined MCAN1 RX FIFO1 element size
#  endif

#  ifndef CONFIG_SAMV7_MCAN1_RXFIFO1_SIZE
#    define CONFIG_SAMV7_MCAN1_RXFIFO1_SIZE 0
#  endif

#  if CONFIG_SAMV7_MCAN1_RXFIFO1_SIZE > 64
#    error Invalid MCAN1 number of RX FIFO 0 elements
#  endif

#  define MCAN1_RXFIFO1_BYTES \
     MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN1_RXFIFO1_SIZE * \
                   MCAN1_RXFIFO1_ELEMENT_SIZE + 8)
#  define MCAN1_RXFIFO1_WORDS (MCAN1_RXFIFO1_BYTES >> 2)

/* MCAN1 Filters */

#  ifndef CONFIG_SAMV7_MCAN1_NSTDFILTERS
#    define CONFIG_SAMV7_MCAN1_NSTDFILTERS 0
#  endif

#  if CONFIG_SAMV7_MCAN1_NSTDFILTERS > 128
#    error Invalid MCAN1 number of Standard Filters
#  endif

#  ifndef CONFIG_SAMV7_MCAN1_NEXTFILTERS
#    define CONFIG_SAMV7_MCAN1_NEXTFILTERS 0
#  endif

#  if CONFIG_SAMV7_MCAN1_NEXTFILTERS > 64
#    error Invalid MCAN1 number of Extended Filters
#  endif

#  define MCAN1_STDFILTER_BYTES \
      MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN1_NSTDFILTERS << 2)
#  define MCAN1_STDFILTER_WORDS (MCAN1_STDFILTER_BYTES >> 2)

#  define MCAN1_EXTFILTER_BYTES \
      MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN1_NEXTFILTERS << 3)
#  define MCAN1_EXTFILTER_WORDS (MCAN1_EXTFILTER_BYTES >> 2)

/* MCAN1 RX buffer element size */

#  if defined(CONFIG_SAMV7_MCAN1_RXBUFFER_8BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_RXBUFFER_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN1_RXBUFFER_12BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  12
#    define MCAN1_RXBUFFER_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN1_RXBUFFER_16BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  16
#    define MCAN1_RXBUFFER_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN1_RXBUFFER_20BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  20
#    define MCAN1_RXBUFFER_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN1_RXBUFFER_24BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  24
#    define MCAN1_RXBUFFER_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN1_RXBUFFER_32BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  32
#    define MCAN1_RXBUFFER_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN1_RXBUFFER_48BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  48
#    define MCAN1_RXBUFFER_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN1_RXBUFFER_64BYTES)
#    define MCAN1_RXBUFFER_ELEMENT_SIZE  64
#    define MCAN1_RXBUFFER_ENCODED_SIZE  7
#  else
#    error Undefined MCAN1 RX buffer element size
#  endif

#  ifndef CONFIG_SAMV7_MCAN1_DEDICATED_RXBUFFER_SIZE
#    define CONFIG_SAMV7_MCAN1_DEDICATED_RXBUFFER_SIZE 0
#  endif

#  if CONFIG_SAMV7_MCAN1_DEDICATED_RXBUFFER_SIZE > 64
#    error Invalid MCAN1 number of RX BUFFER elements
#  endif

#  define MCAN1_DEDICATED_RXBUFFER_BYTES \
     MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN1_DEDICATED_RXBUFFER_SIZE * \
                   MCAN1_RXBUFFER_ELEMENT_SIZE + 8)
#  define MCAN1_DEDICATED_RXBUFFER_WORDS \
     (MCAN1_DEDICATED_RXBUFFER_BYTES >> 2)

/* MCAN1 TX buffer element size */

#  if defined(CONFIG_SAMV7_MCAN1_TXBUFFER_8BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  8
#    define MCAN1_TXBUFFER_ENCODED_SIZE  0
#  elif defined(CONFIG_SAMV7_MCAN1_TXBUFFER_12BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  12
#    define MCAN1_TXBUFFER_ENCODED_SIZE  1
#  elif defined(CONFIG_SAMV7_MCAN1_TXBUFFER_16BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  16
#    define MCAN1_TXBUFFER_ENCODED_SIZE  2
#  elif defined(CONFIG_SAMV7_MCAN1_TXBUFFER_20BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  20
#    define MCAN1_TXBUFFER_ENCODED_SIZE  3
#  elif defined(CONFIG_SAMV7_MCAN1_TXBUFFER_24BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  24
#    define MCAN1_TXBUFFER_ENCODED_SIZE  4
#  elif defined(CONFIG_SAMV7_MCAN1_TXBUFFER_32BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  32
#    define MCAN1_TXBUFFER_ENCODED_SIZE  5
#  elif defined(CONFIG_SAMV7_MCAN1_TXBUFFER_48BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  48
#    define MCAN1_TXBUFFER_ENCODED_SIZE  6
#  elif defined(CONFIG_SAMV7_MCAN1_TXBUFFER_64BYTES)
#    define MCAN1_TXBUFFER_ELEMENT_SIZE  64
#    define MCAN1_TXBUFFER_ENCODED_SIZE  7
#  else
#    error Undefined MCAN1 TX buffer element size
#  endif

#  ifndef CONFIG_SAMV7_MCAN1_DEDICATED_TXBUFFER_SIZE
#    define CONFIG_SAMV7_MCAN1_DEDICATED_TXBUFFER_SIZE 0
#  endif

#  define MCAN1_DEDICATED_TXBUFFER_BYTES \
     MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN1_DEDICATED_TXBUFFER_SIZE * \
                   MCAN1_TXBUFFER_ELEMENT_SIZE + 8)
#  define MCAN1_DEDICATED_TXBUFFER_WORDS \
     (MCAN1_DEDICATED_TXBUFFER_BYTES >> 2)

/* MCAN1 TX FIFOs */

#  ifndef CONFIG_SAMV7_MCAN1_TXFIFOQ_SIZE
#    define CONFIG_SAMV7_MCAN1_TXFIFOQ_SIZE 0
#  endif

#  if (CONFIG_SAMV7_MCAN1_DEDICATED_TXBUFFER_SIZE + \
       CONFIG_SAMV7_MCAN1_TXFIFOQ_SIZE) > 32
#    error Invalid MCAN1 number of TX BUFFER elements
#  endif

#  ifndef CONFIG_SAMV7_MCAN1_TXEVENTFIFO_SIZE
#    define CONFIG_SAMV7_MCAN1_TXEVENTFIFO_SIZE 0
#  endif

#  if CONFIG_SAMV7_MCAN1_TXEVENTFIFO_SIZE > 32
#    error Invalid MCAN1 number of TX EVENT FIFO elements
#  endif

#  define MCAN1_TXEVENTFIFO_BYTES \
     MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN1_TXEVENTFIFO_SIZE << 3)
#  define MCAN1_TXEVENTFIFO_WORDS \
     (MCAN1_TXEVENTFIFO_BYTES >> 2)

#  define MCAN1_TXFIFIOQ_BYTES \
     MCAN_ALIGN_UP(CONFIG_SAMV7_MCAN1_TXFIFOQ_SIZE *  \
                   MCAN1_TXBUFFER_ELEMENT_SIZE + 8)
#  define MCAN1_TXFIFIOQ_WORDS (MCAN1_TXFIFIOQ_BYTES >> 2)

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

/* Common interrupts
 *
 *   MCAN_INT_TSW  - Timestamp Wraparound
 *   MCAN_INT_MRAF - Message RAM Access Failure
 *   MCAN_INT_TOO  - Timeout Occurred
 *   MCAN_INT_ELO  - Error Logging Overflow
 *   MCAN_INT_EP   - Error Passive
 *   MCAN_INT_EW   - Warning Status
 *   MCAN_INT_BO   - Bus_Off Status
 *   MCAN_INT_WDI  - Watchdog Interrupt
 */

#define MCAN_CMNERR_INTS   (MCAN_INT_MRAF | MCAN_INT_TOO | MCAN_INT_EP | \
                            MCAN_INT_BO | MCAN_INT_WDI)
#define MCAN_COMMON_INTS   MCAN_CMNERR_INTS

/* RXFIFO mode interrupts
 *
 *   MCAN_INT_RF0N - Receive FIFO 0 New Message
 *   MCAN_INT_RF0W - Receive FIFO 0 Watermark Reached
 *   MCAN_INT_RF0F - Receive FIFO 0 Full
 *   MCAN_INT_RF0L - Receive FIFO 0 Message Lost
 *   MCAN_INT_RF1N - Receive FIFO 1 New Message
 *   MCAN_INT_RF1W - Receive FIFO 1 Watermark Reached
 *   MCAN_INT_RF1F - Receive FIFO 1 Full
 *   MCAN_INT_RF1L - Receive FIFO 1 Message Lost
 *   MCAN_INT_HPM  - High Priority Message Received
 *
 * Dedicated RX Buffer mode interrupts
 *
 *   MCAN_INT_DRX  - Message stored to Dedicated Receive Buffer
 *
  * Mode-independent RX-related interrupts for revision A
 *
 *   MCAN_INT_CRCE - Receive CRC Error
 *   MCAN_INT_FOE  - Format Error
 *   MCAN_INT_STE  - Stuff Error
 *
 * Mode-independent RX-related interrupts for revision B
 *
 *   MCAN_INT_PEA - Protocol Error in Arbitration Phase
 *   MCAN_INT_PED - Protocol Error in Data Phase
 */

#define MCAN_RXCOMMON_INTS_REVA (MCAN_INT_CRCE | MCAN_INT_FOE | MCAN_INT_STE)
#define MCAN_RXCOMMON_INTS (MCAN_INT_PEA | MCAN_INT_PED)
#define MCAN_RXFIFO0_INTS       (MCAN_INT_RF0N | MCAN_INT_RF0W | MCAN_INT_RF0L)
#define MCAN_RXFIFO1_INTS       (MCAN_INT_RF1N | MCAN_INT_RF1W | MCAN_INT_RF1L)
#define MCAN_RXFIFO_INTS        (MCAN_RXFIFO0_INTS | MCAN_RXFIFO1_INTS | \
                                 MCAN_INT_HPM | MCAN_RXCOMMON_INTS)
#define MCAN_RXDEDBUF_INTS      (MCAN_INT_DRX | MCAN_RXCOMMON_INTS)

#define MCAN_RXERR_INTS_REVA    (MCAN_INT_RF0L | MCAN_INT_RF1L | MCAN_INT_CRCE | \
                                 MCAN_INT_FOE | MCAN_INT_STE)
#define MCAN_RXERR_INTS    (MCAN_INT_RF0L | MCAN_INT_RF1L | MCAN_INT_PEA | \
                                 MCAN_INT_PED)

/* TX FIFOQ mode interrupts
 *
 *   MCAN_INT_TFE  - Tx FIFO Empty
 *
 * TX Event FIFO interrupts
 *
 *   MCAN_INT_TEFN - Tx Event FIFO New Entry
 *   MCAN_INT_TEFW - Tx Event FIFO Watermark Reached
 *   MCAN_INT_TEFF - Tx Event FIFO Full
 *   MCAN_INT_TEFL - Tx Event FIFO Element Lost
 *
 * Mode-independent TX-related interrupts
 *
 *   MCAN_INT_TC   - Transmission Completed
 *   MCAN_INT_TCF  - Transmission Cancellation Finished
 *   MCAN_INT_BE   - Bit Error (rev A)
 *   MCAN_INT_ACKE - Acknowledge Error (rev A)
 *   MCAN_INT_PEA  - Protocol Error in Arbitration Phase (rev B)
 *   MCAN_INT_PED  - Protocol Error in Data Phase (rev B)
 */

#define MCAN_TXCOMMON_INTS (MCAN_INT_TC | MCAN_INT_TCF | MCAN_INT_PEA | \
                            MCAN_INT_PED)
#define MCAN_TXFIFOQ_INTS  (MCAN_INT_TFE | MCAN_TXCOMMON_INTS)
#define MCAN_TXEVFIFO_INTS (MCAN_INT_TEFN | MCAN_INT_TEFW | MCAN_INT_TEFF | \
                            MCAN_INT_TEFL)
#define MCAN_TXDEDBUF_INTS MCAN_TXCOMMON_INTS

#define MCAN_TXERR_INTS    (MCAN_INT_TEFL | MCAN_INT_PEA | MCAN_INT_PED)

/* Common-, TX- and RX-Error-Mask */

#define MCAN_ANYERR_INTS (MCAN_CMNERR_INTS | MCAN_RXERR_INTS | MCAN_TXERR_INTS)

/* Debug ********************************************************************/

/* Debug configurations that may be enabled just for testing MCAN */

#ifndef CONFIG_DEBUG_CAN_INFO
#  undef CONFIG_SAMV7_MCAN_REGDEBUG
#endif

#ifdef CONFIG_SAMV7_MCAN_REGDEBUG
#  define reginfo caninfo
#else
#  define reginfo(x...)
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

/* CAN driver state */

enum can_state_s
{
  MCAN_STATE_UNINIT = 0,    /* Not yet initialized */
  MCAN_STATE_RESET,         /* Initialized, reset state */
  MCAN_STATE_SETUP,         /* can_setup() has been called */
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

/* This structure provides the constant configuration of a MCAN peripheral */

struct sam_config_s
{
  gpio_pinset_t rxpinset;   /* RX pin configuration */
  gpio_pinset_t txpinset;   /* TX pin configuration */
  uintptr_t base;           /* Base address of the MCAN registers */
  uint32_t baud;            /* Configured baud */
  uint32_t btp;             /* Bit timing/prescaler register setting */
  uint32_t fbtp;            /* Fast bit timing/prescaler register setting */
  uint32_t nbtp;            /* Nominal Bit timing/prescaler register setting */
  uint32_t dbtp;            /* Data bit timing/prescaler register setting */
  uint8_t rev;              /* Chip revision (0: A, 1: B) */
  uint8_t port;             /* MCAN port number (1 or 2) */
  uint8_t pid;              /* MCAN peripheral ID */
  uint8_t irq0;             /* MCAN peripheral IRQ number for interrupt line 0 */
  uint8_t irq1;             /* MCAN peripheral IRQ number for interrupt line 1 */
  uint8_t mode;             /* See enum sam_canmode_e */
  uint8_t nstdfilters;      /* Number of standard filters (up to 128) */
  uint8_t nextfilters;      /* Number of extended filters (up to 64) */
  uint8_t nrxfifo0;         /* Number of RX FIFO0 elements (up to 64) */
  uint8_t nrxfifo1;         /* Number of RX FIFO1 elements (up to 64) */
  uint8_t nrxdedicated;     /* Number of dedicated RX buffers (up to 64) */
  uint8_t ntxeventfifo;     /* Number of TXevent FIFO elements (up to 32) */
  uint8_t ntxdedicated;     /* Number of dedicated TX buffers (up to 64) */
  uint8_t ntxfifoq;         /* Number of TX FIFO queue elements (up to 32) */
  uint8_t rxfifo0ecode;     /* Encoded RX FIFO0 element size */
  uint8_t rxfifo0esize;     /* RX FIFO0 element size (words) */
  uint8_t rxfifo1ecode;     /* Encoded RX FIFO1 element size */
  uint8_t rxfifo1esize;     /* RX FIFO1 element size (words) */
  uint8_t rxbufferecode;    /* Encoded RX buffer element size */
  uint8_t rxbufferesize;    /* RX buffer element size (words) */
  uint8_t txbufferecode;    /* Encoded TX buffer element size */
  uint8_t txbufferesize;    /* TX buffer element size (words) */
#ifdef SAMV7_MCAN_LOOPBACK
  bool loopback;            /* True: Loopback mode */
#endif

  /* MCAN message RAM layout */

  struct sam_msgram_s msgram;
};

/* This structure provides the current state of a MCAN peripheral */

struct sam_mcan_s
{
  /* The constant configuration */

  const struct sam_config_s *config;

  uint8_t state;            /* See enum can_state_s */
#ifdef CONFIG_CAN_EXTID
  uint8_t nextalloc;        /* Number of allocated extended filters */
#endif
  uint8_t nstdalloc;        /* Number of allocated standard filters */
  sem_t locksem;            /* Enforces mutually exclusive access */
  sem_t txfsem;             /* Used to wait for TX FIFO availability */
  uint32_t btp;             /* Current bit timing */
  uint32_t fbtp;            /* Current fast bit timing */
  uint32_t rxints;          /* Configured RX interrupts */
  uint32_t txints;          /* Configured TX interrupts */
  uint8_t rev;              /* Chip revision (0: A, 1: B) */

#ifdef CONFIG_CAN_EXTID
  uint32_t extfilters[2];   /* Extended filter bit allocator.  2*32=64 */
#endif
  uint32_t stdfilters[4];   /* Standard filter bit allocator.  4*32=128 */

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
static void mcan_putreg(FAR struct sam_mcan_s *priv, int offset,
              uint32_t regval);
#ifdef CONFIG_SAMV7_MCAN_REGDEBUG
static void mcan_dumpregs(FAR struct sam_mcan_s *priv, FAR const char *msg);
#else
#  define mcan_dumpregs(priv,msg)
#endif

/* Semaphore helpers */

static int mcan_dev_lock(FAR struct sam_mcan_s *priv);
static int mcan_dev_lock_noncancelable(FAR struct sam_mcan_s *priv);
#define mcan_dev_unlock(priv) nxsem_post(&priv->locksem)

static void mcan_buffer_reserve(FAR struct sam_mcan_s *priv);
static void mcan_buffer_release(FAR struct sam_mcan_s *priv);

/* MCAN helpers */

static uint8_t mcan_dlc2bytes(FAR struct sam_mcan_s *priv, uint8_t dlc);
#if 0 /* Not used */
static uint8_t mcan_bytes2dlc(FAR struct sam_mcan_s *priv, uint8_t nbytes);
#endif

#ifdef CONFIG_CAN_EXTID
static int mcan_add_extfilter(FAR struct sam_mcan_s *priv,
              FAR struct canioc_extfilter_s *extconfig);
static int mcan_del_extfilter(FAR struct sam_mcan_s *priv, int ndx);
#endif
static int mcan_add_stdfilter(FAR struct sam_mcan_s *priv,
              FAR struct canioc_stdfilter_s *stdconfig);
static int mcan_del_stdfilter(FAR struct sam_mcan_s *priv, int ndx);

/* CAN driver methods */

static void mcan_reset(FAR struct can_dev_s *dev);
static int  mcan_setup(FAR struct can_dev_s *dev);
static void mcan_shutdown(FAR struct can_dev_s *dev);
static void mcan_rxint(FAR struct can_dev_s *dev, bool enable);
static void mcan_txint(FAR struct can_dev_s *dev, bool enable);
static int  mcan_ioctl(FAR struct can_dev_s *dev, int cmd,
              unsigned long arg);
static int  mcan_remoterequest(FAR struct can_dev_s *dev, uint16_t id);
static int  mcan_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg);
static bool mcan_txready(FAR struct can_dev_s *dev);
static bool mcan_txempty(FAR struct can_dev_s *dev);

/* MCAN interrupt handling */

#if 0 /* Not Used */
static bool mcan_dedicated_rxbuffer_available(FAR struct sam_mcan_s *priv,
              int bufndx);
#endif
#ifdef CONFIG_CAN_ERRORS
static void mcan_error(FAR struct can_dev_s *dev, uint32_t status);
#endif
static void mcan_receive(FAR struct can_dev_s *dev,
              FAR uint32_t *rxbuffer, unsigned long nwords);
static int  mcan_interrupt(int irq, void *context, FAR void *arg);

/* Hardware initialization */

static int  mcan_hw_initialize(FAR struct sam_mcan_s *priv);

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

static uint32_t g_mcan0_msgram[MCAN0_MSGRAM_WORDS]
#ifdef CONFIG_ARMV7M_DCACHE
  __attribute__((aligned(MCAN_ALIGN)));
#else
  ;
#endif

/* Constant configuration */

static struct sam_config_s g_mcan0const =
{
  .rxpinset         = GPIO_MCAN0_RX,
  .txpinset         = GPIO_MCAN0_TX,
  .base             = SAM_MCAN0_BASE,
  .baud             = CONFIG_SAMV7_MCAN0_BITRATE,
  .btp              = MCAN_REVA_BTP_BRP(MCAN0_BRP) |
                      MCAN_REVA_BTP_TSEG1(MCAN0_TSEG1) |
                      MCAN_REVA_BTP_TSEG2(MCAN0_TSEG2) |
                      MCAN_REVA_BTP_SJW(MCAN0_SJW),
  .fbtp             = MCAN_REVA_FBTP_FBRP(MCAN0_DBRP) |
                      MCAN_REVA_FBTP_FTSEG1(MCAN0_DTSEG1) |
                      MCAN_REVA_FBTP_FTSEG2(MCAN0_DTSEG2) |
                      MCAN_REVA_FBTP_FSJW(MCAN0_DSJW),
  .nbtp             = MCAN_NBTP_NBRP(MCAN0_BRP) |
                      MCAN_NBTP_NTSEG1(MCAN0_TSEG1) |
                      MCAN_NBTP_NTSEG2(MCAN0_TSEG2) |
                      MCAN_NBTP_NSJW(MCAN0_SJW),
  .dbtp             = MCAN_DBTP_DBRP(MCAN0_DBRP) |
                      MCAN_DBTP_DTSEG1(MCAN0_DTSEG1) |
                      MCAN_DBTP_DTSEG2(MCAN0_DTSEG2) |
                      MCAN_DBTP_DSJW(MCAN0_DSJW),
  .port             = 0,
  .pid              = SAM_PID_MCAN00,
  .irq0             = SAM_IRQ_MCAN00,
  .irq1             = SAM_IRQ_MCAN01,
#if defined(CONFIG_SAMV7_MCAN0_ISO11899_1)
  .mode             = MCAN_ISO11898_1_MODE,
#elif defined(CONFIG_SAMV7_MCAN0_FD)
  .mode             = MCAN_FD_MODE,
#else /* if defined(CONFIG_SAMV7_MCAN0_FD_BSW) */
  .mode             = MCAN_FD_BSW_MODE,
#endif
  .nstdfilters      = CONFIG_SAMV7_MCAN0_NSTDFILTERS,
  .nextfilters      = CONFIG_SAMV7_MCAN0_NEXTFILTERS,
  .nrxfifo0         = CONFIG_SAMV7_MCAN0_RXFIFO0_SIZE,
  .nrxfifo1         = CONFIG_SAMV7_MCAN0_RXFIFO1_SIZE,
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

static uint32_t g_mcan1_msgram[MCAN1_MSGRAM_WORDS]
#ifdef CONFIG_ARMV7M_DCACHE
  __attribute__((aligned(MCAN_ALIGN)));
#else
  ;
#endif

/* MCAN1 constant configuration */

static struct sam_config_s g_mcan1const =
{
  .rxpinset         = GPIO_MCAN1_RX,
  .txpinset         = GPIO_MCAN1_TX,
  .base             = SAM_MCAN1_BASE,
  .baud             = CONFIG_SAMV7_MCAN1_BITRATE,
  .btp              = MCAN_REVA_BTP_BRP(MCAN1_BRP) |
                      MCAN_REVA_BTP_TSEG1(MCAN1_TSEG1) |
                      MCAN_REVA_BTP_TSEG2(MCAN1_TSEG2) |
                      MCAN_REVA_BTP_SJW(MCAN1_SJW),
  .fbtp             = MCAN_REVA_FBTP_FBRP(MCAN1_DBRP) |
                      MCAN_REVA_FBTP_FTSEG1(MCAN1_DTSEG1) |
                      MCAN_REVA_FBTP_FTSEG2(MCAN1_DTSEG2) |
                      MCAN_REVA_FBTP_FSJW(MCAN1_DSJW),
  .nbtp             = MCAN_NBTP_NBRP(MCAN1_BRP) |
                      MCAN_NBTP_NTSEG1(MCAN1_TSEG1) |
                      MCAN_NBTP_NTSEG2(MCAN1_TSEG2) |
                      MCAN_NBTP_NSJW(MCAN1_SJW),
  .dbtp             = MCAN_DBTP_DBRP(MCAN1_DBRP) |
                      MCAN_DBTP_DTSEG1(MCAN1_DTSEG1) |
                      MCAN_DBTP_DTSEG2(MCAN1_DTSEG2) |
                      MCAN_DBTP_DSJW(MCAN1_DSJW),
  .port             = 1,
  .pid              = SAM_PID_MCAN10,
  .irq0             = SAM_IRQ_MCAN10,
  .irq1             = SAM_IRQ_MCAN11,
#if defined(CONFIG_SAMV7_MCAN1_ISO11899_1)
  .mode             = MCAN_ISO11898_1_MODE,
#elif defined(CONFIG_SAMV7_MCAN1_FD)
  .mode             = MCAN_FD_MODE,
#else /* if defined(CONFIG_SAMV7_MCAN1_FD_BSW) */
  .mode             = MCAN_FD_BSW_MODE,
#endif
  .nstdfilters      = CONFIG_SAMV7_MCAN1_NSTDFILTERS,
  .nextfilters      = CONFIG_SAMV7_MCAN1_NEXTFILTERS,
  .nrxfifo0         = CONFIG_SAMV7_MCAN1_RXFIFO0_SIZE,
  .nrxfifo1         = CONFIG_SAMV7_MCAN1_RXFIFO1_SIZE,
  .nrxdedicated     = CONFIG_SAMV7_MCAN1_DEDICATED_RXBUFFER_SIZE,
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
  .txbufferesize    = (MCAN1_TXBUFFER_ELEMENT_SIZE / 4) + 2,

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
 *   Read the value of a MCAN register.
 *
 * Input Parameters:
 *   priv - A reference to the MCAN peripheral state
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
              caninfo("...\n");
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

          caninfo("[repeats %d more times]\n", priv->count - 3);
        }

      /* Save the new address, value, and count */

      priv->regaddr = regaddr;
      priv->regval  = regval;
      priv->count   = 1;
    }

  /* Show the register value read */

  caninfo("%08x->%08x\n", regaddr, regval);
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
 *   Set the value of a MCAN register.
 *
 * Input Parameters:
 *   priv - A reference to the MCAN peripheral state
 *   offset - The offset to the register to write
 *   regval - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_MCAN_REGDEBUG
static void mcan_putreg(FAR struct sam_mcan_s *priv, int offset,
                        uint32_t regval)
{
  FAR const struct sam_config_s *config = priv->config;
  uintptr_t regaddr = config->base + offset;

  /* Show the register value being written */

  caninfo("%08x<-%08x\n", regaddr, regval);

  /* Write the value */

  putreg32(regval, regaddr);
}

#else
static void mcan_putreg(FAR struct sam_mcan_s *priv, int offset,
                        uint32_t regval)
{
  FAR const struct sam_config_s *config = priv->config;
  putreg32(regval, config->base + offset);
}

#endif

/****************************************************************************
 * Name: mcan_dumpregs
 *
 * Description:
 *   Dump the contents of all MCAN control registers
 *
 * Input Parameters:
 *   priv - A reference to the MCAN peripheral state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_MCAN_REGDEBUG
static void mcan_dumpregs(FAR struct sam_mcan_s *priv, FAR const char *msg)
{
  FAR const struct sam_config_s *config = priv->config;

  caninfo("MCAN%d Registers: %s\n", config->port, msg);
  caninfo("  Base: %08x\n", config->base);

  caninfo("  CUST: %08x  DBTP: %08x  TEST: %08x    RWD: %08x\n",
          getreg32(config->base + SAM_MCAN_CUST_OFFSET),
          getreg32(config->base + SAM_MCAN_DBTP_OFFSET),
          getreg32(config->base + SAM_MCAN_TEST_OFFSET),
          getreg32(config->base + SAM_MCAN_RWD_OFFSET));

  caninfo("  CCCR: %08x  NBTP: %08x  TSCC: %08x   TSCV: %08x\n",
          getreg32(config->base + SAM_MCAN_CCCR_OFFSET),
          getreg32(config->base + SAM_MCAN_NBTP_OFFSET),
          getreg32(config->base + SAM_MCAN_TSCC_OFFSET),
          getreg32(config->base + SAM_MCAN_TSCV_OFFSET));

  caninfo("  TOCC: %08x  TOCV: %08x   ECR: %08x    PSR: %08x\n",
          getreg32(config->base + SAM_MCAN_TOCC_OFFSET),
          getreg32(config->base + SAM_MCAN_TOCV_OFFSET),
          getreg32(config->base + SAM_MCAN_ECR_OFFSET),
          getreg32(config->base + SAM_MCAN_PSR_OFFSET));

  caninfo("    IR: %08x    IE: %08x   ILS: %08x    ILE: %08x\n",
          getreg32(config->base + SAM_MCAN_IR_OFFSET),
          getreg32(config->base + SAM_MCAN_IE_OFFSET),
          getreg32(config->base + SAM_MCAN_ILS_OFFSET),
          getreg32(config->base + SAM_MCAN_ILE_OFFSET));

  caninfo("   GFC: %08x SIDFC: %08x XIDFC: %08x  XIDAM: %08x\n",
          getreg32(config->base + SAM_MCAN_GFC_OFFSET),
          getreg32(config->base + SAM_MCAN_SIDFC_OFFSET),
          getreg32(config->base + SAM_MCAN_XIDFC_OFFSET),
          getreg32(config->base + SAM_MCAN_XIDAM_OFFSET));

  caninfo("  HPMS: %08x NDAT1: %08x NDAT2: %08x  RXF0C: %08x\n",
          getreg32(config->base + SAM_MCAN_HPMS_OFFSET),
          getreg32(config->base + SAM_MCAN_NDAT1_OFFSET),
          getreg32(config->base + SAM_MCAN_NDAT2_OFFSET),
          getreg32(config->base + SAM_MCAN_RXF0C_OFFSET));

  caninfo(" RXF0S: %08x FXF0A: %08x  RXBC: %08x  RXF1C: %08x\n",
          getreg32(config->base + SAM_MCAN_RXF0S_OFFSET),
          getreg32(config->base + SAM_MCAN_RXF0A_OFFSET),
          getreg32(config->base + SAM_MCAN_RXBC_OFFSET),
          getreg32(config->base + SAM_MCAN_RXF1C_OFFSET));

  caninfo(" RXF1S: %08x FXF1A: %08x RXESC: %08x   TXBC: %08x\n",
          getreg32(config->base + SAM_MCAN_RXF1S_OFFSET),
          getreg32(config->base + SAM_MCAN_RXF1A_OFFSET),
          getreg32(config->base + SAM_MCAN_RXESC_OFFSET),
          getreg32(config->base + SAM_MCAN_TXBC_OFFSET));

  caninfo(" TXFQS: %08x TXESC: %08x TXBRP: %08x  TXBAR: %08x\n",
          getreg32(config->base + SAM_MCAN_TXFQS_OFFSET),
          getreg32(config->base + SAM_MCAN_TXESC_OFFSET),
          getreg32(config->base + SAM_MCAN_TXBRP_OFFSET),
          getreg32(config->base + SAM_MCAN_TXBAR_OFFSET));

  caninfo(" TXBCR: %08x TXBTO: %08x TXBCF: %08x TXBTIE: %08x\n",
          getreg32(config->base + SAM_MCAN_TXBCR_OFFSET),
          getreg32(config->base + SAM_MCAN_TXBTO_OFFSET),
          getreg32(config->base + SAM_MCAN_TXBCF_OFFSET),
          getreg32(config->base + SAM_MCAN_TXBTIE_OFFSET));

  caninfo("TXBCIE: %08x TXEFC: %08x TXEFS: %08x  TXEFA: %08x\n",
          getreg32(config->base + SAM_MCAN_TXBCIE_OFFSET),
          getreg32(config->base + SAM_MCAN_TXEFC_OFFSET),
          getreg32(config->base + SAM_MCAN_TXEFS_OFFSET),
          getreg32(config->base + SAM_MCAN_TXEFA_OFFSET));
}
#endif

/****************************************************************************
 * Name: mcan_dev_lock
 *
 * Description:
 *   Take the semaphore that enforces mutually exclusive access to device
 *   structures, handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the MCAN peripheral state
 *
 * Returned Value:
 *  Normally success (OK) is returned, but the error -ECANCELED may be
 *  return in the event that task has been canceled.
 *
 ****************************************************************************/

static int mcan_dev_lock(FAR struct sam_mcan_s *priv)
{
  return nxsem_wait_uninterruptible(&priv->locksem);
}

/****************************************************************************
 * Name: mcan_dev_lock_noncancelable
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.  This version also
 *   ignores attempts to cancel the thread.
 *
 ****************************************************************************/

static int mcan_dev_lock_noncancelable(FAR struct sam_mcan_s *priv)
{
  int result;
  int ret = OK;

  do
    {
      result = nxsem_wait_uninterruptible(&priv->locksem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(result == OK || result == -ECANCELED);
      if (ret == OK && result < 0)
        {
          ret = result;
        }
    }
  while (result < 0);

  return ret;
}

/****************************************************************************
 * Name: mcan_buffer_reserve
 *
 * Description:
 *   Take the semaphore, decrementing the semaphore count to indicate that
 *   one fewer TX FIFOQ buffer is available.  Handles any exceptional
 *   conditions.
 *
 * Input Parameters:
 *   priv - A reference to the MCAN peripheral state
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  Called only non-interrupt logic via mcan_write().  We do not have
 *  exclusive access to the MCAN hardware and interrupts are not disabled.
 *  mcan_write() does lock the scheduler for reasons noted below.
 *
 ****************************************************************************/

static void mcan_buffer_reserve(FAR struct sam_mcan_s *priv)
{
  irqstate_t flags;
  uint32_t txfqs1;
  uint32_t txfqs2;
#ifndef CONFIG_SAMV7_MCAN_QUEUE_MODE
  int tffl;
#endif
  int sval;
  int ret;

  /* Wait until we successfully get the semaphore.  EINTR is the only
   * expected 'failure' (meaning that the wait for the semaphore was
   * interrupted by a signal.
   */

  do
    {
      /* We take some extra precautions here because it is possible that on
       * certain error conditions, the semaphore count could get out of
       * phase with the actual count of elements in the TX FIFO (I have
       * never seen this happen, however.  My paranoia).
       *
       * An missed TX interrupt could cause the semaphore count to fail to
       * be incremented and, hence, to be too low.
       */

      for (; ; )
        {
          /* Get the current queue status and semaphore count. */

          flags = enter_critical_section();
          txfqs1 = mcan_getreg(priv, SAM_MCAN_TXFQS_OFFSET);
          nxsem_get_value(&priv->txfsem, &sval);
          txfqs2 = mcan_getreg(priv, SAM_MCAN_TXFQS_OFFSET);

          /* If the semaphore count and the TXFQS samples are in
           * sync, then break out of the look with interrupts
           * disabled.
           */

          if (txfqs1 == txfqs2)
            {
              break;
            }

          /* Otherwise, re-enable interrupts to interrupts that may
           * resynchronize, the semaphore count and try again.
           */

          leave_critical_section(flags);
        }

#ifdef CONFIG_SAMV7_MCAN_QUEUE_MODE
      /* We only have one useful bit of information in the TXFQS:
       * Is the TX FIFOQ full or not?  We can only do limited checks
       * with that single bit of information.
       */

      if ((txfqs1 & MCAN_TXFQS_TFQF) != 0)
        {
          /* The TX FIFOQ is full.  The semaphore count should then be
           * less than or equal to zero.  If it is greater than zero,
           * then reinitialize it to 0.
           */

          if (sval > 0)
            {
              canerr("ERROR: TX FIFOQ full but txfsem is %d\n", sval);
              nxsem_reset(&priv->txfsem, 0);
            }
        }

      /* The FIFO is not full so the semaphore count should be greater
       * than zero.  If it is not, then we have missed a call to
       * mcan_buffer_release(0).
       *
       * NOTE: Since there is no mutual exclusion, it might be possible
       * that mcan_write() could be re-entered AFTER taking the semaphore
       * and dropping the count to zero, but BEFORE adding the message
       * to the TX FIFOQ.  That corner case is handled in mcan_write() by
       * locking the scheduler.
       */

      else if (sval <= 0)
        {
          canerr("ERROR: TX FIFOQ not full but txfsem is %d\n", sval);

          /* Less than zero means that another thread is waiting */

          if (sval < 0)
            {
              /* Bump up the count by one and try again */

              nxsem_post(&priv->txfsem);
              leave_critical_section(flags);
              continue;
            }

          /* Exactly zero but the FIFO is not full.  Just return without
           * decrementing the count.
           */

          leave_critical_section(flags);
          return;
        }
#else
      /* Tx FIFO Free Level */

      tffl = (txfqs1 & MCAN_TXFQS_TFFL_MASK) >> MCAN_TXFQS_TFFL_SHIFT;

      /* Check if the configured number is less than the number of buffers
       * in the chip
       */

      if (tffl > priv->config->ntxfifoq)
        {
          canerr("ERROR: TX FIFO reports %d but max is %d\n",
                 tffl, priv->config->ntxfifoq);
          tffl = priv->config->ntxfifoq;
        }

      /* REVISIT:  There may be issues with this logic in a multi-thread
       * environment.  If there is only a single thread, then certainly
       * sval and tff1 should match, but that may not be true in any multi-
       * threaded use of this driver.
       */

      if (sval != tffl)
        {
          canerr("ERROR: TX FIFO reports %d but txfsem is %d\n", tffl, sval);

          /* Reset the semaphore count to the Tx FIFO free level. */

          nxsem_reset(&priv->txfsem, tffl);
        }
#endif

      /* The semaphore value is reasonable. Wait for the next TC interrupt. */

      ret = nxsem_wait(&priv->txfsem);
      leave_critical_section(flags);
    }
  while (ret < 0);
}

/****************************************************************************
 * Name: mcan_buffer_release
 *
 * Description:
 *   Release the semaphore, increment the semaphore count to indicate that
 *   one more TX FIFOQ buffer is available.
 *
 * Input Parameters:
 *   priv - A reference to the MCAN peripheral state
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  This function is called only from the interrupt level in response to the
 *  complete of a transmission.
 *
 ****************************************************************************/

static void mcan_buffer_release(FAR struct sam_mcan_s *priv)
{
  int sval;

  /* We take some extra precautions here because it is possible that on
   * certain error conditions, the semaphore count could get out of phase
   * with the actual count of elements in the TX FIFO (I have never seen
   * this happen, however.  My paranoia).
   *
   * An extra TC interrupt could cause the count to be incremented too
   * many times.
   */

  nxsem_get_value(&priv->txfsem, &sval);
  if (sval < priv->config->ntxfifoq)
    {
      nxsem_post(&priv->txfsem);
    }
  else
    {
      canerr("ERROR: txfsem would increment beyond %d\n",
              priv->config->ntxfifoq);
    }
}

/****************************************************************************
 * Name: mcan_dlc2bytes
 *
 * Description:
 *   In the CAN FD format, the coding of the DLC differs from the standard
 *   CAN format. The DLC codes 0 to 8 have the same coding as in standard
 *   CAN.  But the codes 9 to 15 all imply a data field of 8 bytes with
 *   standard CAN.  In CAN FD mode, the values 9 to 15 are encoded to values
 *   in the range 12 to 64.
 *
 * Input Parameters:
 *   dlc    - the DLC value to convert to a byte count
 *
 * Returned Value:
 *   The number of bytes corresponding to the DLC value.
 *
 ****************************************************************************/

static uint8_t mcan_dlc2bytes(FAR struct sam_mcan_s *priv, uint8_t dlc)
{
  if (dlc > 8)
    {
#ifdef CONFIG_CAN_FD
      if (priv->config->mode == MCAN_ISO11898_1_MODE)
        {
          return 8;
        }
      else
        {
          switch (dlc)
            {
              case 9:
                return 12;
              case 10:
                return 16;
              case 11:
                return 20;
              case 12:
                return 24;
              case 13:
                return 32;
              case 14:
                return 48;
              default:
              case 15:
                return 64;
            }
        }
#else
      return 8;
#endif
    }

  return dlc;
}

/****************************************************************************
 * Name: mcan_bytes2dlc
 *
 * Description:
 *   In the CAN FD format, the coding of the DLC differs from the standard
 *   CAN format. The DLC codes 0 to 8 have the same coding as in standard
 *   CAN.  But the codes 9 to 15 all imply a data field of 8 bytes with
 *   standard CAN.  In CAN FD mode, the values 9 to 15 are encoded to values
 *   in the range 12 to 64.
 *
 * Input Parameters:
 *   nbytes - the byte count to convert to a DLC value
 *
 * Returned Value:
 *   The encoded DLC value corresponding to at least that number of bytes.
 *
 ****************************************************************************/

#if 0 /* Not used */
static uint8_t mcan_bytes2dlc(FAR struct sam_mcan_s *priv, uint8_t nbytes)
{
  if (nbytes <= 8)
    {
      return nbytes;
    }
#ifdef CONFIG_CAN_FD
  else if (priv->mode == MCAN_ISO11898_1_MODE)
    {
      return 8;
    }
  else if (nbytes <= 12)
    {
      return 9;
    }
  else if (nbytes <= 16)
    {
      return 10;
    }
  else if (nbytes <= 20)
    {
      return 11;
    }
  else if (nbytes <= 24)
    {
      return 12;
    }
  else if (nbytes <= 32)
    {
      return 13;
    }
  else if (nbytes <= 48)
    {
      return 14;
    }
  else /* if (nbytes <= 64) */
    {
      return 15;
    }
#else
  else
    {
      return 8;
    }
#endif
}
#endif

/****************************************************************************
 * Name: mcan_add_extfilter
 *
 * Description:
 *   Add an address filter for a extended 29 bit address.
 *
 * Input Parameters:
 *   priv      - An instance of the MCAN driver state structure.
 *   extconfig - The configuration of the extended filter
 *
 * Returned Value:
 *   A non-negative filter ID is returned on success.  Otherwise a negated
 *   errno value is returned to indicate the nature of the error.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static int mcan_add_extfilter(FAR struct sam_mcan_s *priv,
                              FAR struct canioc_extfilter_s *extconfig)
{
  FAR const struct sam_config_s *config;
  FAR uint32_t *extfilter;
  uint32_t regval;
  int word;
  int bit;
  int ndx;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL && extconfig != NULL);
  config = priv->config;

  /* Get exclusive excess to the MCAN hardware */

  ret = mcan_dev_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Find an unused standard filter */

  for (ndx = 0; ndx <  config->nextfilters; ndx++)
    {
      /* Is this filter assigned? */

      word = ndx >> 5;
      bit  = ndx & 0x1f;

      if ((priv->extfilters[word] & (1 << bit)) == 0)
        {
          /* No, assign the filter */

          DEBUGASSERT(priv->nextalloc < priv->config->nstdfilters);
          priv->extfilters[word] |= (1 << bit);
          priv->nextalloc++;

          extfilter = config->msgram.extfilters + (ndx << 1);

          /* Format and write filter word F0 */

          DEBUGASSERT(extconfig->xf_id1 <= CAN_MAX_EXTMSGID);
          regval = EXTFILTER_F0_EFID1(extconfig->xf_id1);

          if (extconfig->xf_prio == 0)
            {
              regval |= EXTFILTER_F0_EFEC_FIFO0;
            }
          else
            {
              regval |= EXTFILTER_F0_EFEC_FIFO0;
            }

          extfilter[0] = regval;

          /* Format and write filter word F1 */

          DEBUGASSERT(extconfig->xf_id2 <= CAN_MAX_EXTMSGID);
          regval = EXTFILTER_F1_EFID2(extconfig->xf_id2);

          switch (extconfig->xf_type)
            {
              default:
              case CAN_FILTER_DUAL:
                regval |= EXTFILTER_F1_EFT_DUAL;
                break;

              case CAN_FILTER_MASK:
                regval |= EXTFILTER_F1_EFT_CLASSIC;
                break;
              case CAN_FILTER_RANGE:
                regval |= EXTFILTER_F1_EFT_RANGE;
                break;
            }

          extfilter[1] = regval;

          /* Flush the filter entry into physical RAM */

          up_clean_dcache((uintptr_t)extfilter, (uintptr_t)extfilter + 8);

          /* Is this the first extended filter? */

          if (priv->nextalloc == 1)
            {
              /* Enable the Initialization state */

              regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
              regval |= MCAN_CCCR_INIT;
              mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);

              /* Wait for initialization mode to take effect */

              while ((mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET) &
                     MCAN_CCCR_INIT) == 0)
                {
                }

              /* Enable writing to configuration registers */

              regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
              regval |= (MCAN_CCCR_INIT | MCAN_CCCR_CCE);
              mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);

             /* Update the Global Filter Configuration so that received
              * messages are rejected if they do not match the acceptance
              * filter.
              *
              *   ANFE=2: Discard all rejected frames
              */

              regval  = mcan_getreg(priv, SAM_MCAN_GFC_OFFSET);
              regval &= ~MCAN_GFC_ANFE_MASK;
              regval |= MCAN_GFC_ANFE_REJECTED;
              mcan_putreg(priv, SAM_MCAN_GFC_OFFSET, regval);

              /* Disable writing to configuration registers */

              regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
              regval &= ~(MCAN_CCCR_INIT | MCAN_CCCR_CCE);
              mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);
            }

          mcan_dev_unlock(priv);
          return ndx;
        }
    }

  DEBUGASSERT(priv->nextalloc == priv->config->nextfilters);
  mcan_dev_unlock(priv);
  return -EAGAIN;
}
#endif

/****************************************************************************
 * Name: mcan_del_extfilter
 *
 * Description:
 *   Remove an address filter for a standard 29 bit address.
 *
 * Input Parameters:
 *   priv - An instance of the MCAN driver state structure.
 *   ndx  - The filter index previously returned by the mcan_add_extfilter().
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static int mcan_del_extfilter(FAR struct sam_mcan_s *priv, int ndx)
{
  FAR const struct sam_config_s *config;
  FAR uint32_t *extfilter;
  uint32_t regval;
  int word;
  int bit;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL);
  config = priv->config;

  /* Check user Parameters */

  DEBUGASSERT(ndx >= 0 || ndx < config->nextfilters);

  if (ndx < 0 || ndx >= config->nextfilters)
    {
      return -EINVAL;
    }

  /* Get exclusive excess to the MCAN hardware */

  ret = mcan_dev_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  word = ndx >> 5;
  bit  = ndx & 0x1f;

  /* Check if this filter is really assigned */

  if ((priv->extfilters[word] & (1 << bit)) == 0)
    {
      /* No, error out */

      mcan_dev_unlock(priv);
      return -ENOENT;
    }

  /* Release the filter */

  priv->extfilters[word] &= ~(1 << bit);

  DEBUGASSERT(priv->nextalloc > 0);
  priv->nextalloc--;

  /* Was that the last extended filter? */

  if (priv->nextalloc == 0)
    {
      /* Enable the Initialization state */

      regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
      regval |= MCAN_CCCR_INIT;
      mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);

      /* Wait for initialization mode to take effect */

      while ((mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET) &
             MCAN_CCCR_INIT) == 0)
        {
        }

      /* Enable writing to configuration registers */

      regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
      regval |= (MCAN_CCCR_INIT | MCAN_CCCR_CCE);
      mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);

      /* If there are no extended filters, then modify Global Filter
       * Configuration so that all rejected messages are places in RX
       * FIFO0.
       *
       *   ANFE=0: Store all rejected extended frame in RX FIFO0
       */

      regval  = mcan_getreg(priv, SAM_MCAN_GFC_OFFSET);
      regval &= ~MCAN_GFC_ANFE_MASK;
      regval |= MCAN_GFC_ANFE_RX_FIFO0;
      mcan_putreg(priv, SAM_MCAN_GFC_OFFSET, regval);

      /* Disable writing to configuration registers */

      regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
      regval &= ~(MCAN_CCCR_INIT | MCAN_CCCR_CCE);
      mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);
    }

  /* Deactivate the filter last so that no messages are lost. */

  extfilter    = config->msgram.extfilters + (ndx << 1);
  *extfilter++ = 0;
  *extfilter   = 0;

  mcan_dev_unlock(priv);
  return OK;
}
#endif

/****************************************************************************
 * Name: mcan_add_stdfilter
 *
 * Description:
 *   Add an address filter for a standard 11 bit address.
 *
 * Input Parameters:
 *   priv      - An instance of the MCAN driver state structure.
 *   stdconfig - The configuration of the standard filter
 *
 * Returned Value:
 *   A non-negative filter ID is returned on success.  Otherwise a negated
 *   errno value is returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int mcan_add_stdfilter(FAR struct sam_mcan_s *priv,
                              FAR struct canioc_stdfilter_s *stdconfig)
{
  FAR const struct sam_config_s *config;
  FAR uint32_t *stdfilter;
  uint32_t regval;
  int word;
  int bit;
  int ndx;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL);
  config = priv->config;

  /* Get exclusive excess to the MCAN hardware */

  ret = mcan_dev_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Find an unused standard filter */

  for (ndx = 0; ndx < config->nstdfilters; ndx++)
    {
      /* Is this filter assigned? */

      word = ndx >> 5;
      bit  = ndx & 0x1f;

      if ((priv->stdfilters[word] & (1 << bit)) == 0)
        {
          /* No, assign the filter */

          DEBUGASSERT(priv->nstdalloc < priv->config->nstdfilters);
          priv->stdfilters[word] |= (1 << bit);
          priv->nstdalloc++;

          /* Format and write filter word S0 */

          stdfilter = config->msgram.stdfilters + ndx;

          DEBUGASSERT(stdconfig->sf_id1 <= CAN_MAX_STDMSGID);
          regval = STDFILTER_S0_SFID1(stdconfig->sf_id1);

          DEBUGASSERT(stdconfig->sf_id2 <= CAN_MAX_STDMSGID);
          regval |= STDFILTER_S0_SFID2(stdconfig->sf_id2);

          if (stdconfig->sf_prio == 0)
            {
              regval |= STDFILTER_S0_SFEC_FIFO0;
            }
          else
            {
              regval |= STDFILTER_S0_SFEC_FIFO1;
            }

          switch (stdconfig->sf_type)
            {
              default:
              case CAN_FILTER_DUAL:
                regval |= STDFILTER_S0_SFT_DUAL;
                break;

              case CAN_FILTER_MASK:
                regval |= STDFILTER_S0_SFT_CLASSIC;
                break;
              case CAN_FILTER_RANGE:
                regval |= STDFILTER_S0_SFT_RANGE;
                break;
            }

          *stdfilter = regval;

          /* Flush the filter entry into physical RAM */

          up_clean_dcache((uintptr_t)stdfilter, (uintptr_t)stdfilter + 4);

          /* Is this the first standard filter? */

          if (priv->nstdalloc == 1)
            {
              /* Enable the Initialization state */

              regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
              regval |= MCAN_CCCR_INIT;
              mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);

              /* Wait for initialization mode to take effect */

              while ((mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET) &
                     MCAN_CCCR_INIT) == 0)
                {
                }

              /* Enable writing to configuration registers */

              regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
              regval |= (MCAN_CCCR_INIT | MCAN_CCCR_CCE);
              mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);

             /* Update the Global Filter Configuration so that received
              * messages are rejected if they do not match the acceptance
              * filter.
              *
              *   ANFS=2: Discard all rejected frames
              */

              regval  = mcan_getreg(priv, SAM_MCAN_GFC_OFFSET);
              regval &= ~MCAN_GFC_ANFS_MASK;
              regval |= MCAN_GFC_ANFS_REJECTED;
              mcan_putreg(priv, SAM_MCAN_GFC_OFFSET, regval);

              /* Disable writing to configuration registers */

              regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
              regval &= ~(MCAN_CCCR_INIT | MCAN_CCCR_CCE);
              mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);
            }

          mcan_dev_unlock(priv);
          return ndx;
        }
    }

  DEBUGASSERT(priv->nstdalloc == priv->config->nstdfilters);
  mcan_dev_unlock(priv);
  return -EAGAIN;
}

/****************************************************************************
 * Name: mcan_del_stdfilter
 *
 * Description:
 *   Remove an address filter for a standard 29 bit address.
 *
 * Input Parameters:
 *   priv - An instance of the MCAN driver state structure.
 *   ndx  - The filter index previously returned by the mcan_add_stdfilter().
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int mcan_del_stdfilter(FAR struct sam_mcan_s *priv, int ndx)
{
  FAR const struct sam_config_s *config;
  FAR uint32_t *stdfilter;
  uint32_t regval;
  int word;
  int bit;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL);
  config = priv->config;

  /* Check Userspace Parameters */

  DEBUGASSERT(ndx >= 0 || ndx < config->nstdfilters);

  if (ndx < 0 || ndx >= config->nstdfilters)
    {
      return -EINVAL;
    }

  /* Get exclusive excess to the MCAN hardware */

  ret = mcan_dev_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  word = ndx >> 5;
  bit  = ndx & 0x1f;

  /* Check if this filter is really assigned */

  if ((priv->stdfilters[word] & (1 << bit)) == 0)
    {
      /* No, error out */

      mcan_dev_unlock(priv);
      return -ENOENT;
    }

  /* Release the filter */

  priv->stdfilters[word] &= ~(1 << bit);

  DEBUGASSERT(priv->nstdalloc > 0);
  priv->nstdalloc--;

  /* Was that the last standard filter? */

  if (priv->nstdalloc == 0)
    {
      /* Enable the Initialization state */

      regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
      regval |= MCAN_CCCR_INIT;
      mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);

      /* Wait for initialization mode to take effect */

      while ((mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET) &
              MCAN_CCCR_INIT) == 0)
        {
        }

      /* Enable writing to configuration registers */

      regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
      regval |= (MCAN_CCCR_INIT | MCAN_CCCR_CCE);
      mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);

      /* If there are no standard filters, then modify Global Filter
       * Configuration so that all rejected messages are places in RX
       * FIFO0.
       *
       *   ANFS=0: Store all rejected extended frame in RX FIFO0
       */

      regval  = mcan_getreg(priv, SAM_MCAN_GFC_OFFSET);
      regval &= ~MCAN_GFC_ANFS_MASK;
      regval |= MCAN_GFC_ANFS_RX_FIFO0;
      mcan_putreg(priv, SAM_MCAN_GFC_OFFSET, regval);

      /* Disable writing to configuration registers */

      regval  = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
      regval &= ~(MCAN_CCCR_INIT | MCAN_CCCR_CCE);
      mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);
    }

  /* Deactivate the filter last so that no messages are lost. */

  stdfilter  = config->msgram.stdfilters + ndx;
  *stdfilter = 0;

  mcan_dev_unlock(priv);
  return OK;
}

/****************************************************************************
 * Name: mcan_start_busoff_recovery_sequence
 *
 * Description:
 *   This function initiates the BUS-OFF recovery sequence.
 *   CAN Specification Rev. 2.0 or ISO11898-1:2015
 *   According the SAMV71 datasheet:
 *
 *   "If the device goes Bus_Off, it will set MCAN_CCCR.INIT of its own
 *    accord, stopping all bus activities. Once MCAN_CCCR.INIT has been
 *    cleared by the processor (application), the device will then wait for
 *    129 occurrences of Bus Idle (129 * 11 consecutive recessive bits)
 *    before resuming normal operation. At the end of the Bus_Off recovery
 *    sequence, the Error Management Counters will be reset. During the
 *    waiting time after the resetting of MCAN_CCCR.INIT, each time a
 *    sequence of 11 recessive bits has been monitored, a Bit0 Error code is
 *    written to MCAN_PSR.LEC, enabling the processor to readily check up
 *    whether the CAN bus is stuck at dominant or continuously disturbed and
 *    to monitor the Bus_Off recovery sequence.  MCAN_ECR.REC is used to
 *    count these sequences."
 *
 * Input Parameters:
 *   priv - An instance of the MCAN driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int mcan_start_busoff_recovery_sequence(FAR struct sam_mcan_s *priv)
{
  uint32_t regval;
  int ret;

  DEBUGASSERT(priv);

  /* Get exclusive access to the MCAN peripheral */

  ret = mcan_dev_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* only start BUS-OFF recovery if we are in BUS-OFF state */

  regval = mcan_getreg(priv, SAM_MCAN_PSR_OFFSET);
  if (!(regval & MCAN_PSR_BO))
    {
      mcan_dev_unlock(priv);
      return -EPERM;
    }

  /* Disable initialization mode to issue the recovery sequence */

  regval = mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET);
  regval &= ~MCAN_CCCR_INIT;
  mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);

  mcan_dev_unlock(priv);
  return OK;
}

/****************************************************************************
 * Name: mcan_reset
 *
 * Description:
 *   Reset the MCAN device.  Called early to initialize the hardware. This
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

  caninfo("MCAN%d\n", config->port);
  UNUSED(config);

  /* Get exclusive access to the MCAN peripheral */

  mcan_dev_lock_noncancelable(priv);

  /* Disable all interrupts */

  mcan_putreg(priv, SAM_MCAN_IE_OFFSET, 0);
  mcan_putreg(priv, SAM_MCAN_TXBTIE_OFFSET, 0);

  /* Make sure that all buffers are released.
   *
   * REVISIT: What if a thread is waiting for a buffer?  The following
   * will not wake up any waiting threads.
   */

  nxsem_destroy(&priv->txfsem);
  nxsem_init(&priv->txfsem, 0, config->ntxfifoq);

  /* Disable peripheral clocking to the MCAN controller */

  sam_disableperiph1(priv->config->pid);
  priv->state = MCAN_STATE_RESET;
  mcan_dev_unlock(priv);
}

/****************************************************************************
 * Name: mcan_setup
 *
 * Description:
 *   Configure the MCAN. This method is called the first time that the MCAN
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching MCAN interrupts.
 *   All MCAN interrupts are disabled upon return.
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

  caninfo("MCAN%d pid: %d\n", config->port, config->pid);

  /* Get exclusive access to the MCAN peripheral */

  ret = mcan_dev_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* MCAN hardware initialization */

  ret = mcan_hw_initialize(priv);
  if (ret < 0)
    {
      canerr("ERROR: MCAN%d H/W initialization failed: %d\n",
             config->port, ret);
      return ret;
    }

  mcan_dumpregs(priv, "After hardware initialization");

  /* Attach the MCAN interrupt handlers */

  ret = irq_attach(config->irq0, mcan_interrupt, dev);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach MCAN%d line 0 IRQ (%d)",
      config->port, config->irq0);
      return ret;
    }

  ret = irq_attach(config->irq1, mcan_interrupt, dev);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach MCAN%d line 1 IRQ (%d)",
      config->port, config->irq1);
      return ret;
    }

  /* Enable receive interrupts */

  priv->state = MCAN_STATE_SETUP;
  mcan_rxint(dev, true);

  mcan_dumpregs(priv, "After receive setup");

  /* Enable the interrupts at the NVIC (they are still disabled at the MCAN
   * peripheral).
   */

  up_enable_irq(config->irq0);
  up_enable_irq(config->irq1);
  mcan_dev_unlock(priv);
  return OK;
}

/****************************************************************************
 * Name: mcan_shutdown
 *
 * Description:
 *   Disable the MCAN.  This method is called when the MCAN device is closed.
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

  caninfo("MCAN%d\n", config->port);

  /* Get exclusive access to the MCAN peripheral */

  mcan_dev_lock_noncancelable(priv);

  /* Disable MCAN interrupts at the NVIC */

  up_disable_irq(config->irq0);
  up_disable_irq(config->irq1);

  /* Disable all interrupts from the MCAN peripheral */

  mcan_putreg(priv, SAM_MCAN_IE_OFFSET, 0);
  mcan_putreg(priv, SAM_MCAN_TXBTIE_OFFSET, 0);

  /* Detach the MCAN interrupt handler */

  irq_detach(config->irq0);
  irq_detach(config->irq1);

  /* Disable peripheral clocking to the MCAN controller */

  sam_disableperiph1(priv->config->pid);
  mcan_dev_unlock(priv);
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

  caninfo("MCAN%d enable: %d\n", priv->config->port, enable);

  /* Enable/disable the receive interrupts */

  flags = enter_critical_section();
  regval = mcan_getreg(priv, SAM_MCAN_IE_OFFSET);

  if (enable)
    {
      regval |= priv->rxints | MCAN_COMMON_INTS;
    }
  else
    {
      regval &= ~priv->rxints;
    }

  mcan_putreg(priv, SAM_MCAN_IE_OFFSET, regval);
  leave_critical_section(flags);
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

  caninfo("MCAN%d enable: %d\n", priv->config->port, enable);

  /* Enable/disable the receive interrupts */

  flags = enter_critical_section();
  regval = mcan_getreg(priv, SAM_MCAN_IE_OFFSET);

  if (enable)
    {
      regval |= priv->txints | MCAN_COMMON_INTS;
    }
  else
    {
      regval &= ~priv->txints;
    }

  mcan_putreg(priv, SAM_MCAN_IE_OFFSET, regval);
  leave_critical_section(flags);
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
  FAR struct sam_mcan_s *priv;
  int ret = -ENOTTY;

  caninfo("cmd=%04x arg=%lu\n", cmd, arg);

  DEBUGASSERT(dev && dev->cd_priv);
  priv = dev->cd_priv;

  /* Handle the command */

  switch (cmd)
    {
      /* CANIOC_GET_BITTIMING:
       *   Description:    Return the current bit timing settings
       *   Argument:       A pointer to a write-able instance of struct
       *                   canioc_bittiming_s in which current bit timing
       *                   values will be returned.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       *   Dependencies:   None
       */

      case CANIOC_GET_BITTIMING:
        {
          FAR struct canioc_bittiming_s *bt =
            (FAR struct canioc_bittiming_s *)arg;
          uint32_t regval;
          uint32_t brp;

          DEBUGASSERT(bt != NULL);

          regval       = mcan_getreg(priv, SAM_MCAN_NBTP_OFFSET);

          if (priv->rev == 0)
            {
              /* Revision A */

              bt->bt_sjw   = ((regval & MCAN_REVA_BTP_SJW_MASK) >>
                              MCAN_REVA_BTP_SJW_SHIFT) + 1;
              bt->bt_tseg1 = ((regval & MCAN_REVA_BTP_TSEG1_MASK) >>
                              MCAN_REVA_BTP_TSEG1_SHIFT) + 1;
              bt->bt_tseg2 = ((regval & MCAN_REVA_BTP_TSEG2_MASK) >>
                              MCAN_REVA_BTP_TSEG2_SHIFT) + 1;
              brp          = ((regval & MCAN_REVA_BTP_BRP_MASK) >>
                              MCAN_REVA_BTP_BRP_SHIFT) + 1;
            }
          else
            {
              /* Revision B */

              bt->bt_sjw   = ((regval & MCAN_NBTP_NSJW_MASK) >>
                              MCAN_NBTP_NSJW_SHIFT) + 1;
              bt->bt_tseg1 = ((regval & MCAN_NBTP_NTSEG1_MASK) >>
                              MCAN_NBTP_NTSEG1_SHIFT) + 1;
              bt->bt_tseg2 = ((regval & MCAN_NBTP_NTSEG2_MASK) >>
                              MCAN_NBTP_NTSEG2_SHIFT) + 1;
              brp          = ((regval & MCAN_NBTP_NBRP_MASK) >>
                              MCAN_NBTP_NBRP_SHIFT) + 1;
            }

          bt->bt_baud  = SAMV7_MCANCLK_FREQUENCY / brp /
                        (bt->bt_tseg1 + bt->bt_tseg2 + 1);
          ret = OK;
        }
        break;

      /* CANIOC_SET_BITTIMING:
       *   Description:    Set new current bit timing values
       *   Argument:       A pointer to a read-able instance of struct
       *                   canioc_bittiming_s in which the new bit timing
       *                   values are provided.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       *   Dependencies:   None
       *
       * REVISIT: There is probably a limitation here:  If there are
       * multiple threads trying to send CAN packets, when one of these
       * threads reconfigures the bitrate, the MCAN hardware will be reset
       * and the context of operation will be lost.  Hence, this IOCTL can
       * only safely be executed in quiescent time periods.
       */

      case CANIOC_SET_BITTIMING:
        {
          FAR const struct canioc_bittiming_s *bt =
            (FAR const struct canioc_bittiming_s *)arg;
          irqstate_t flags;
          uint32_t brp;
          uint32_t tseg1;
          uint32_t tseg2;
          uint32_t sjw;
          uint32_t ie;
          uint8_t state;

          DEBUGASSERT(bt != NULL);
          DEBUGASSERT(bt->bt_baud < SAMV7_MCANCLK_FREQUENCY);
          DEBUGASSERT(bt->bt_sjw > 0 && bt->bt_sjw <= 16);
          DEBUGASSERT(bt->bt_tseg1 > 1 && bt->bt_tseg1 <= 64);
          DEBUGASSERT(bt->bt_tseg2 > 0 && bt->bt_tseg2 <= 16);

          /* Extract bit timing data */

          tseg1 = bt->bt_tseg1 - 1;
          tseg2 = bt->bt_tseg2 - 1;
          sjw   = bt->bt_sjw   - 1;

          brp = (uint32_t)
            (((float) SAMV7_MCANCLK_FREQUENCY /
             ((float)(tseg1 + tseg2 + 3) * (float)bt->bt_baud)) - 1);

          /* Save the value of the new bit timing register */

          flags = enter_critical_section();
          if (priv->rev == 0)
            {
              priv->btp = MCAN_REVA_BTP_BRP(brp) |
                          MCAN_REVA_BTP_TSEG1(tseg1) |
                          MCAN_REVA_BTP_TSEG2(tseg2) |
                          MCAN_REVA_BTP_SJW(sjw);
            }
          else
            {
              priv->btp = MCAN_NBTP_NBRP(brp) | MCAN_NBTP_NTSEG1(tseg1) |
                          MCAN_NBTP_NTSEG2(tseg2) | MCAN_NBTP_NSJW(sjw);
            }

          /* We need to reset to instantiate the new timing.  Save
           * current state information so that recover to this
           * state.
           */

          ie    = mcan_getreg(priv, SAM_MCAN_IE_OFFSET);
          state = priv->state;

          /* Reset the MCAN */

          mcan_reset(dev);
          ret = OK;

          /* If we have previously been setup, then setup again */

          if (state == MCAN_STATE_SETUP)
            {
              ret = mcan_setup(dev);
            }

          /* We we have successfully re-initialized, then restore the
           * interrupt state.
           *
           * REVISIT: Since the hardware was reset, any pending TX
           * activity was lost.  Should we disable TX interrupts?
           */

          if (ret == OK)
            {
              mcan_putreg(priv, SAM_MCAN_IE_OFFSET, ie & ~priv->txints);
            }

          leave_critical_section(flags);
        }
        break;

#ifdef CONFIG_CAN_EXTID
      /* CANIOC_ADD_EXTFILTER:
       *   Description:    Add an address filter for a extended 29 bit
       *                   address.
       *   Argument:       A reference to struct canioc_extfilter_s
       *   Returned Value: A non-negative filter ID is returned on success.
       *                   Otherwise -1 (ERROR) is returned with the errno
       *                   variable set to indicate the nature of the error.
       */

      case CANIOC_ADD_EXTFILTER:
        {
          DEBUGASSERT(arg != 0);

          ret = mcan_add_extfilter(priv,
                                   (FAR struct canioc_extfilter_s *)arg);
        }
        break;

      /* CANIOC_DEL_EXTFILTER:
       *   Description:    Remove an address filter for a standard 29 bit
       *                   address.
       *   Argument:       The filter index previously returned by the
       *                   CANIOC_ADD_EXTFILTER command
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       */

      case CANIOC_DEL_EXTFILTER:
        {
          DEBUGASSERT(arg <= priv->config->nextfilters);
          ret = mcan_del_extfilter(priv, (int)arg);
        }
        break;
#endif

      /* CANIOC_ADD_STDFILTER:
       *   Description:    Add an address filter for a standard 11 bit
       *                   address.
       *   Argument:       A reference to struct canioc_stdfilter_s
       *   Returned Value: A non-negative filter ID is returned on success.
       *                   Otherwise -1 (ERROR) is returned with the errno
       *                   variable set to indicate the nature of the error.
       */

      case CANIOC_ADD_STDFILTER:
        {
          DEBUGASSERT(arg != 0);

          ret = mcan_add_stdfilter(priv,
                                   (FAR struct canioc_stdfilter_s *)arg);
        }
        break;

      /* CANIOC_DEL_STDFILTER:
       *   Description:    Remove an address filter for a standard 11 bit
       *                   address.
       *   Argument:       The filter index previously returned by the
       *                   CANIOC_ADD_STDFILTER command
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       */

      case CANIOC_DEL_STDFILTER:
        {
          DEBUGASSERT(arg <= priv->config->nstdfilters);
          ret = mcan_del_stdfilter(priv, (int)arg);
        }
        break;

      /* CANIOC_BUSOFF_RECOVERY:
       *   Description : Initiates the BUS - OFF recovery sequence
       *   Argument : None
       *   Returned Value : Zero (OK) is returned on success. Otherwise -1
       *                    (ERROR) is returned with the errno variable set
       *                    to indicate the nature of the error.
       *   Dependencies : None
       */

      case CANIOC_BUSOFF_RECOVERY:
        {
          ret = mcan_start_busoff_recovery_sequence(priv);
        }
        break;

      /* Unsupported/unrecognized command */

      default:
        canerr("ERROR: Unrecognized command: %04x\n", cmd);
        break;
    }

  return ret;
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
 *                 Bit 4:    Remote Transmission Request (RTR)
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
  FAR const struct sam_config_s *config;
  FAR uint32_t *txbuffer = 0;
  FAR const uint8_t *src;
  FAR uint8_t *dest;
  uint32_t regval;
  unsigned int msglen;
  unsigned int ndx;
  unsigned int nbytes;
  unsigned int i;
  int ret;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  caninfo("MCAN%d\n", config->port);
  caninfo("MCAN%d ID: %"PRIu32" DLC: %u\n",
          config->port, (uint32_t)msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

  /* That that FIFO elements were configured.
   *
   * REVISIT: Dedicated TX buffers are not used by this driver.
   */

  DEBUGASSERT(config->ntxfifoq > 0);

  /* Reserve a buffer for the transmission, waiting if necessary.  When
   * mcan_buffer_reserve() returns, we are guaranteed that the TX FIFOQ is
   * not full and cannot become full at least until we add our packet to
   * the FIFO.
   *
   * We can't get exclusive access to MCAN resources here because that
   * lock the MCAN while we wait for a free buffer.  Instead, the
   * scheduler is locked here momentarily.  See discussion in
   * mcan_buffer_reserve() for an explanation.
   *
   * REVISIT: This needs to be extended in order to handler case where
   * the MCAN device was opened O_NONBLOCK.
   */

  sched_lock();
  mcan_buffer_reserve(priv);

  /* Get exclusive access to the MCAN peripheral */

  ret = mcan_dev_lock(priv);
  if (ret < 0)
    {
      mcan_buffer_release(priv);
      sched_unlock();
      return ret;
    }

  sched_unlock();

  /* Get our reserved Tx FIFO/queue put index */

  regval = mcan_getreg(priv, SAM_MCAN_TXFQS_OFFSET);
  DEBUGASSERT((regval & MCAN_TXFQS_TFQF) == 0);

  ndx = (regval & MCAN_TXFQS_TFQPI_MASK) >> MCAN_TXFQS_TFQPI_SHIFT;

  /* And the TX buffer corresponding to this index */

  txbuffer = config->msgram.txdedicated + ndx * config->txbufferesize;

  /* Format the TX FIFOQ entry
   *
   * Format word T0:
   *   Transfer message ID (ID)          - Value from message structure
   *   Remote Transmission Request (RTR) - Value from message structure
   *   Extended Identifier (XTD)         - Depends on configuration.
   */

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      DEBUGASSERT(msg->cm_hdr.ch_id <= CAN_MAX_EXTMSGID);

      regval = BUFFER_R0_EXTID(msg->cm_hdr.ch_id) | BUFFER_R0_XTD;
    }
  else
#endif
    {
      DEBUGASSERT(msg->cm_hdr.ch_id <= CAN_MAX_STDMSGID);

      regval = BUFFER_R0_STDID(msg->cm_hdr.ch_id);
    }

  if (msg->cm_hdr.ch_rtr)
    {
      regval |= BUFFER_R0_RTR;
    }

  txbuffer[0] = regval;
  reginfo("T0: %08x\n", regval);

  /* Format word T1:
   *   Data Length Code (DLC)            - Value from message structure
   *   Event FIFO Control (EFC)          - Do not store events.
   *   Message Marker (MM)               - Always zero
   */

  txbuffer[1] = BUFFER_R1_DLC(msg->cm_hdr.ch_dlc);

#ifdef CONFIG_CAN_FD
  if (msg->cm_hdr.ch_edl)
    {
      txbuffer[1] |= BUFFER_R1_EDL;
    }

  if (msg->cm_hdr.ch_brs)
    {
      txbuffer[1] |= BUFFER_R1_BRS;
    }
#endif

  reginfo("T1: %08x\n", txbuffer[1]);

  /* Followed by the amount of data corresponding to the DLC (T2..) */

  dest   = (FAR uint8_t *)&txbuffer[2];
  src    = msg->cm_data;
  nbytes = mcan_dlc2bytes(priv, msg->cm_hdr.ch_dlc);

  for (i = 0; i < nbytes; i++)
    {
      /* Little endian is assumed */

      *dest++ = *src++;
    }

  /* Flush the D-Cache to memory before initiating the transfer */

  msglen = 2 * sizeof(uint32_t) + nbytes;
  up_clean_dcache((uintptr_t)txbuffer, (uintptr_t)txbuffer + msglen);
  UNUSED(msglen);

  /* Enable transmit interrupts from the TX FIFOQ buffer by setting TC
   * interrupt bit in IR (also requires that the TC interrupt is enabled)
   */

  mcan_putreg(priv, SAM_MCAN_TXBTIE_OFFSET, (1 << ndx));

  /* And request to send the packet */

  mcan_putreg(priv, SAM_MCAN_TXBAR_OFFSET, (1 << ndx));
  mcan_dev_unlock(priv);

  /* Report that the TX transfer is complete to the upper half logic.  Of
   * course, the transfer is not complete, but this early notification
   * allows the upper half logic to free resources sooner.
   *
   * REVISIT:  Should we disable interrupts?  can_txdone() was designed to
   * be called from an interrupt handler and, hence, may be unsafe when
   * called from the tasking level.
   */

  can_txdone(dev);
  return OK;
}

/****************************************************************************
 * Name: mcan_txready
 *
 * Description:
 *   Return true if the MCAN hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if the MCAN hardware is ready to accept another TX message.
 *
 ****************************************************************************/

static bool mcan_txready(FAR struct can_dev_s *dev)
{
  FAR struct sam_mcan_s *priv = dev->cd_priv;
  uint32_t regval;
  bool notfull;
#ifdef CONFIG_DEBUG_FEATURES
  int sval;
#endif
  int ret;

  /* Get exclusive access to the MCAN peripheral */

  ret = mcan_dev_lock(priv);
  if (ret < 0)
    {
      return false;
    }

  /* Return the state of the TX FIFOQ.  Return TRUE if the TX FIFO/Queue is
   * not full.
   *
   * REVISIT: Dedicated TX buffers are not supported.
   */

  regval  = mcan_getreg(priv, SAM_MCAN_TXFQS_OFFSET);
  notfull = ((regval & MCAN_TXFQS_TFQF) == 0);

#ifdef CONFIG_DEBUG_FEATURES
  /* As a sanity check, the txfsem should also track the number of elements
   * the TX FIFO/queue.  Make sure that they are consistent.
   */

  nxsem_get_value(&priv->txfsem, &sval);
  DEBUGASSERT(((notfull && sval > 0) || (!notfull && sval <= 0)) &&
              (sval <= priv->config->ntxfifoq));
#endif

  mcan_dev_unlock(priv);
  return notfull;
}

/****************************************************************************
 * Name: mcan_txempty
 *
 * Description:
 *   Return true if all message have been sent.  If for example, the MCAN
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if there are no pending TX transfers in the MCAN hardware.
 *
 ****************************************************************************/

static bool mcan_txempty(FAR struct can_dev_s *dev)
{
  FAR struct sam_mcan_s *priv = dev->cd_priv;
  uint32_t regval;
#ifdef CONFIG_SAMV7_MCAN_QUEUE_MODE
  int sval;
#else
  int tffl;
#endif
  bool empty;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  /* Get exclusive access to the MCAN peripheral */

  ret = mcan_dev_lock(priv);
  if (ret < 0)
    {
      return false;
    }

  /* Return the state of the TX FIFOQ.  Return TRUE if the TX FIFO/Queue is
   * empty.  We don't have a reliable indication that the FIFO is empty, so
   * we have to use some heuristics.
   *
   * REVISIT: Dedicated TX buffers are not supported.
   */

  regval = mcan_getreg(priv, SAM_MCAN_TXFQS_OFFSET);
  if (((regval & MCAN_TXFQS_TFQF) != 0))
    {
      mcan_dev_unlock(priv);
      return false;
    }

#ifdef CONFIG_SAMV7_MCAN_QUEUE_MODE
  /* The TX FIFO/Queue is not full, but is it empty?  The txfsem should
   * track the number of elements the TX FIFO/queue in use.
   *
   * Since the FIFO is not full, the semaphore count should be greater
   * than zero.  If it is equal to the full count of TX FIFO/Queue
   * elements, then there is no transfer in progress.
   */

  nxsem_get_value(&priv->txfsem, &sval);
  DEBUGASSERT(sval > 0 && sval <= priv->config->ntxfifoq);

  empty = (sval ==  priv->config->ntxfifoq);
#else
  /* Tx FIFO Free Level */

  tffl  = (regval & MCAN_TXFQS_TFFL_MASK) >> MCAN_TXFQS_TFFL_SHIFT;
  empty = (tffl >= priv->config->ntxfifoq);
#endif

  mcan_dev_unlock(priv);
  return empty;
}

/****************************************************************************
 * Name: mcan_dedicated_rxbuffer_available
 *
 * Description:
 *   Check if data is available in a dedicated RX buffer.
 *
 * Input Parameters:
 *   priv   - MCAN-specific private data
 *   bufndx - Buffer index
 *
 *   None
 * Returned Value:
 *   True: Data is available
 *
 ****************************************************************************/

#if 0 /* Not Used */
bool mcan_dedicated_rxbuffer_available(FAR struct sam_mcan_s *priv,
                                       int bufndx)
{
  if (bufndx < 32)
    {
      return (bool)(mcan->MCAN_NDAT1 & (1 << bufndx));
    }
  else if (bufndx < 64)
    {
      return (bool)(mcan->MCAN_NDAT1 & (1 << (bufndx - 32)));
    }
  else
    {
      return false;
    }
}
#endif

/****************************************************************************
 * Name: mcan_error
 *
 * Description:
 *   Report a CAN error
 *
 * Input Parameters:
 *   dev        - CAN-common state data
 *   status     - Interrupt status with error bits set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_ERRORS
static void mcan_error(FAR struct can_dev_s *dev, uint32_t status)
{
  FAR struct sam_mcan_s *priv = dev->cd_priv;
  struct can_hdr_s hdr;
  uint32_t psr;
  uint16_t errbits;
  uint8_t data[CAN_ERROR_DLC];
  int ret;
  uint32_t lec;
  uint32_t dlec;

  /* Encode error bits */

  errbits = 0;
  memset(data, 0, sizeof(data));

  /* Always fill in "static" error conditions, but set the signaling bit
   * only if the condition has changed (see IRQ-Flags below)
   * They have to be filled in every time CAN_ERROR_CONTROLLER is set.
   */

  psr = mcan_getreg(priv, SAM_MCAN_PSR_OFFSET);
  if ((psr & MCAN_PSR_EP) != 0)
    {
      data[1] |= (CAN_ERROR1_RXPASSIVE | CAN_ERROR1_TXPASSIVE);
    }

  if (psr & MCAN_PSR_EW)
    {
      data[1] |= (CAN_ERROR1_RXWARNING | CAN_ERROR1_TXWARNING);
    }

  if ((status & (MCAN_INT_EP | MCAN_INT_EW)) != 0)
    {
      /* "Error Passive" or "Error Warning" status changed */

      errbits |= CAN_ERROR_CONTROLLER;
    }

  if ((status & MCAN_INT_BO) != 0)
    {
      /* Bus_Off Status changed */

      if ((psr & MCAN_PSR_BO) != 0)
        {
          errbits |= CAN_ERROR_BUSOFF;
        }
      else
        {
          errbits |= CAN_ERROR_RESTARTED;
        }
    }

  if ((status & (MCAN_INT_RF0L | MCAN_INT_RF1L)) != 0)
    {
      /* Receive FIFO 0/1 Message Lost
       * Receive FIFO 1 Message Lost
       */

      errbits |= CAN_ERROR_CONTROLLER;
      data[1] |= CAN_ERROR1_RXOVERFLOW;
    }

  if ((status & MCAN_INT_TEFL) != 0)
    {
      /* Tx Event FIFO Element Lost */

      errbits |= CAN_ERROR_CONTROLLER;
      data[1] |= CAN_ERROR1_TXOVERFLOW;
    }

  if ((status & MCAN_INT_TOO) != 0)
    {
      /* Timeout Occurred */

      errbits |= CAN_ERROR_TXTIMEOUT;
    }

  if ((status & (MCAN_INT_MRAF | MCAN_INT_ELO)) != 0)
    {
      /* Message RAM Access Failure
       * Error Logging Overflow
       */

      errbits |= CAN_ERROR_CONTROLLER;
      data[1] |= CAN_ERROR1_UNSPEC;
    }

  if ((status & (MCAN_INT_PEA | MCAN_INT_CRCE)) != 0)
    {
      lec = psr & MCAN_PSR_LEC_MASK;
      dlec = (psr & MCAN_PSR_DLEC_MASK) >> MCAN_PSR_DLEC_SHIFT;

      if ((lec == MCAN_PSR_EC_CRC_ERROR) || (dlec == MCAN_PSR_EC_CRC_ERROR))
        {
          /* Receive CRC Error */

          errbits |= CAN_ERROR_PROTOCOL;
          data[3] |= (CAN_ERROR3_CRCSEQ | CAN_ERROR3_CRCDEL);
        }

      if ((lec == MCAN_PSR_EC_BIT0_ERROR) || \
          (lec == MCAN_PSR_EC_BIT1_ERROR) || \
          (dlec == MCAN_PSR_EC_BIT0_ERROR) || \
          (dlec == MCAN_PSR_EC_BIT1_ERROR))
        {
          /* Bit Error */

          errbits |= CAN_ERROR_PROTOCOL;
          data[2] |= CAN_ERROR2_BIT;
        }

      if ((lec == MCAN_PSR_EC_ACK_ERROR) || (dlec == MCAN_PSR_EC_ACK_ERROR))
        {
          /* Acknowledge Error */

          errbits |= CAN_ERROR_NOACK;
        }

      if ((lec == MCAN_PSR_EC_FORM_ERROR) || \
         (dlec == MCAN_PSR_EC_FORM_ERROR))
        {
          /* Format Error */

          errbits |= CAN_ERROR_PROTOCOL;
          data[2] |= CAN_ERROR2_FORM;
        }

      if ((lec == MCAN_PSR_EC_STUFF_ERROR) || \
         (dlec == MCAN_PSR_EC_STUFF_ERROR))
        {
          /* Stuff Error */

          errbits |= CAN_ERROR_PROTOCOL;
          data[2] |= CAN_ERROR2_STUFF;
        }
    }

  if (errbits != 0)
    {
      /* Format the CAN header for the error report. */

      hdr.ch_id     = errbits;
      hdr.ch_dlc    = CAN_ERROR_DLC;
      hdr.ch_rtr    = 0;
      hdr.ch_error  = 1;
#ifdef CONFIG_CAN_EXTID
      hdr.ch_extid  = 0;
#endif
      hdr.ch_unused = 0;

      /* And provide the error report to the upper half logic */

      ret = can_receive(dev, &hdr, data);
      if (ret < 0)
        {
          canerr("ERROR: can_receive failed: %d\n", ret);
        }
    }
}
#endif /* CONFIG_CAN_ERRORS */

/****************************************************************************
 * Name: mcan_receive
 *
 * Description:
 *   Receive an MCAN messages
 *
 * Input Parameters:
 *   dev      - CAN-common state data
 *   rxbuffer - The RX buffer containing the received messages
 *   nwords   - The length of the RX buffer (element size in words).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mcan_receive(FAR struct can_dev_s *dev, FAR uint32_t *rxbuffer,
                         unsigned long nwords)
{
  struct can_hdr_s hdr;
  uint32_t regval;
  unsigned int nbytes;
  int ret;

  /* Invalidate the D-Cache to ensure
   * that we reread the RX buffer data from memory.
   */

  nbytes = (nwords << 2);
  up_invalidate_dcache((uintptr_t)rxbuffer, (uintptr_t)rxbuffer + nbytes);

  /* Format the CAN header */

  /* Work R0 contains the CAN ID */

  regval = *rxbuffer++;
  reginfo("R0: %08x\n", regval);

#ifdef CONFIG_CAN_ERRORS
  hdr.ch_error  = 0;
#endif
  hdr.ch_unused = 0;

  if ((regval & BUFFER_R0_RTR) != 0)
    {
      hdr.ch_rtr = true;
    }
  else
    {
      hdr.ch_rtr = false;
    }

#ifdef CONFIG_CAN_EXTID
  if ((regval & BUFFER_R0_XTD) != 0)
    {
      /* Save the extended ID of the newly received message */

      hdr.ch_id    = (regval & BUFFER_R0_EXTID_MASK) >>
                     BUFFER_R0_EXTID_SHIFT;
      hdr.ch_extid = true;
    }
  else
    {
      hdr.ch_id    = (regval & BUFFER_R0_STDID_MASK) >>
                     BUFFER_R0_STDID_SHIFT;
      hdr.ch_extid = false;
    }

#else
  if ((regval & BUFFER_R0_XTD) != 0)
    {
      /* Drop any messages with extended IDs */

      return;
    }

  /* Save the standard ID of the newly received message */

  hdr.ch_id = (regval & BUFFER_R0_STDID_MASK) >> BUFFER_R0_STDID_SHIFT;
#endif

  /* Word R1 contains the DLC and timestamp */

  regval = *rxbuffer++;
  reginfo("R1: %08x\n", regval);

  hdr.ch_dlc = (regval & BUFFER_R1_DLC_MASK) >> BUFFER_R1_DLC_SHIFT;

#ifdef CONFIG_CAN_FD
  hdr.ch_edl = (regval >> BUFFER_R1_EDL_SHIFT) & 1u;
  hdr.ch_brs = (regval >> BUFFER_R1_BRS_SHIFT) & 1u;
#endif

  /* And provide the CAN message to the upper half logic */

  ret = can_receive(dev, &hdr, (FAR uint8_t *)rxbuffer);
  if (ret < 0)
    {
      canerr("ERROR: can_receive failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: mcan_interrupt
 *
 * Description:
 *   Common MCAN interrupt handler
 *
 * Input Parameters:
 *   dev - CAN-common state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int mcan_interrupt(int irq, void *context, FAR void *arg)
{
  FAR struct can_dev_s *dev = (FAR struct can_dev_s *)arg;
  FAR struct sam_mcan_s *priv;
  FAR const struct sam_config_s *config;
  uint32_t ir;
  uint32_t ie;
  uint32_t pending;
  uint32_t regval;
  unsigned int nelem;
  unsigned int ndx;
  bool handled;
  uint32_t psr;

  DEBUGASSERT(dev != NULL);
  priv = dev->cd_priv;
  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  /* Loop while there are pending interrupt events */

  do
    {
      /* Get the set of pending interrupts. */

      ir = mcan_getreg(priv, SAM_MCAN_IR_OFFSET);
      ie = mcan_getreg(priv, SAM_MCAN_IE_OFFSET);

      pending = (ir & ie);
      handled = false;

      /* Check for any errors */

      if ((pending & MCAN_ANYERR_INTS) != 0)
        {
          /* Check for common errors */

          if ((pending & MCAN_CMNERR_INTS) != 0)
            {
              psr = mcan_getreg(priv, SAM_MCAN_PSR_OFFSET);
              canerr("ERROR: Common %08"PRIx32", PSR %08"PRIx32"\n",
                     pending & MCAN_CMNERR_INTS, psr);

              /* Clear the error indications */

              mcan_putreg(priv, SAM_MCAN_IR_OFFSET, MCAN_CMNERR_INTS);
            }

          /* Check for transmission errors */

          if ((pending & MCAN_TXERR_INTS) != 0)
            {
              psr = mcan_getreg(priv, SAM_MCAN_PSR_OFFSET);
              canerr("ERROR: TX %08"PRIx32", PSR %08"PRIx32"\n",
                     pending & MCAN_TXERR_INTS, psr);

              /* An Acknowledge-Error will occur if for example the device
               * is not connected to the bus.
               *
               * The CAN-Standard states that the Chip has to retry the
               * message forever, which will produce an ACKE every time.
               * To prevent this Interrupt-Flooding and the high CPU-Load
               * we disable the all errors here as long we didn't transfer at
               * least one message successfully (see MCAN_INT_TC below).
               */

              if (priv->rev == 0)
                {
                  ie &= ~MCAN_INT_ACKE;
                }
              else
                {
                  ie &= ~(MCAN_INT_PEA | MCAN_INT_PED);
                }

              mcan_putreg(priv, SAM_MCAN_IE_OFFSET, ie);

              /* Clear the error indications */

              mcan_putreg(priv, SAM_MCAN_IR_OFFSET, MCAN_TXERR_INTS);

              /* REVISIT:  Will MCAN_INT_TC also be set in the event of
               * a transmission error?  Each write must conclude with a
               * call to man_buffer_release(), whether or not the write
               * was successful.
               *
               * We assume that MCAN_INT_TC will be called for each
               * message buffer. Except the transfer is cancelled.
               * TODO: add handling for MCAN_INT_TCF
               */
            }

          /* Check for reception errors */

          if ((pending & MCAN_RXERR_INTS) != 0)
            {
              psr = mcan_getreg(priv, SAM_MCAN_PSR_OFFSET);
              canerr("ERROR: RX %08"PRIx32", PSR %08"PRIx32"\n",
                     pending & MCAN_RXERR_INTS, psr);

              /* To prevent Interrupt-Flooding the current active
               * RX error interrupts are disabled. After successfully
               * receiving at least one CAN packet all RX error interrupts
               * are turned back on.
               *
               * The Interrupt-Flooding can for example occur if the
               * configured CAN speed does not match the speed of the other
               * CAN nodes in the network.
               */

              ie &= ~(pending & MCAN_RXERR_INTS);
              mcan_putreg(priv, SAM_MCAN_IE_OFFSET, ie);

              /* Clear the error indications */

              mcan_putreg(priv, SAM_MCAN_IR_OFFSET, MCAN_RXERR_INTS);
            }

#ifdef CONFIG_CAN_ERRORS
          /* Report errors */

          mcan_error(dev, pending & MCAN_ANYERR_INTS);
#endif
          handled = true;
        }

      /* Check for successful completion of a transmission */

      if ((pending & MCAN_INT_TC) != 0)
        {
          /* Check if we have disabled the ACKE in the error-handling above
           * (see MCAN_TXERR_INTS) to prevent Interrupt-Flooding and
           * re-enable the error interrupt here again.
           */

          if ((priv->rev == 0) && ((ie & MCAN_INT_ACKE) == 0))
            {
                ie |= MCAN_INT_ACKE;
                mcan_putreg(priv, SAM_MCAN_IE_OFFSET, ie);
            }
          else if ((priv->rev == 1) && \
                  ((ie & (MCAN_INT_PEA | MCAN_INT_PED)) == 0))
            {
                ie |= MCAN_INT_PEA | MCAN_INT_PED;
                mcan_putreg(priv, SAM_MCAN_IE_OFFSET, ie);
            }

          /* Clear the pending TX completion interrupt (and all
           * other TX-related interrupts)
           */

          mcan_putreg(priv, SAM_MCAN_IR_OFFSET, priv->txints);

          /* Indicate that there is one more buffer free in the TX FIFOQ by
           * "releasing" it.  This may have the effect of waking up a thread
           * that has been waiting for a free TX FIFOQ buffer.
           *
           * REVISIT: TX dedicated buffers are not supported.
           */

          mcan_buffer_release(priv);
          handled = true;

#ifdef CONFIG_CAN_TXREADY
          /* Inform the upper half driver that we are again ready to accept
           * data in mcan_send().
           */

          can_txready(dev);
#endif
        }
      else if ((pending & priv->txints) != 0)
        {
          /* Clear unhandled TX events */

          mcan_putreg(priv, SAM_MCAN_IR_OFFSET, priv->txints);
          handled = true;
        }

#if 0 /* Not used */
      /* Check if a message has been stored to the dedicated RX buffer (DRX) */

      if ((pending & MCAN_INT_DRX) != 0)
        {
          int i;

          /* Clear the pending DRX interrupt */

          mcan_putreg(priv, SAM_MCAN_IR_OFFSET, MCAN_INT_DRX);

          /* Process each dedicated RX buffer */

          for (i = 0; i < config->nrxdedicated; i++)
            {
              uint32_t *rxdedicated = &config->rxdedicated[i];

              /* Check if datat is available in this dedicated RX buffer */

              if (mcan_dedicated_rxbuffer_available(priv, i))
                {
                  /* Yes.. Invalidate the D-Cache to that data will be re-
                   * fetched from RAM.
                   *
                   * REVISIT:  This will require 32-byte alignment.
                   */

                  arch_invalidata_dcache();
                  mcan_receive(priv, rxdedicated, config->rxbufferesize);

                  /* Clear the new data flag for the buffer */

                  if (i < 32)
                    {
                      sam_putreg(priv, SAM_MCAN_NDAT1_OFFSET,
                                 (1 << i);
                    }
                  else
                    {
                      sam_putreg(priv, SAM_MCAN_NDAT1_OFFSET,
                                 (1 << (i - 32));
                    }
                }
            }

          handled = true;
        }
#endif

      /* Clear the RX FIFO1 new message interrupt */

      mcan_putreg(priv, SAM_MCAN_IR_OFFSET, MCAN_INT_RF1N);
      pending &= ~MCAN_INT_RF1N;

      /* We treat RX FIFO1 as the "high priority" queue:  We will process
       * all messages in RX FIFO1 before processing any message from RX
       * FIFO0.
       */

      for (; ; )
        {
          /* Check if there is anything in RX FIFO1 */

          regval = mcan_getreg(priv, SAM_MCAN_RXF1S_OFFSET);
          nelem  = (regval & MCAN_RXF0S_F0FL_MASK) >> MCAN_RXF0S_F0FL_SHIFT;
          if (nelem == 0)
            {
              /* Break out of the loop if RX FIFO1 is empty */

              break;
            }

          /* Clear the RX FIFO1 interrupt (and all other FIFO1-related
           * interrupts)
           */

          /* Handle the newly received message in FIFO1 */

          ndx = (regval & MCAN_RXF1S_F1GI_MASK) >> MCAN_RXF1S_F1GI_SHIFT;

          if ((regval & MCAN_RXF0S_RF0L) != 0)
            {
              canerr("ERROR: Message lost: %08"PRIx32"\n", regval);
            }
          else
            {
              mcan_receive(dev,
                           config->msgram.rxfifo1 +
                             (ndx * priv->config->rxfifo1esize),
                           priv->config->rxfifo1esize);

              /* Turning back on all configured RX error interrupts */

              ie |= (priv->rxints & MCAN_RXERR_INTS);
              mcan_putreg(priv, SAM_MCAN_IE_OFFSET, ie);
            }

          /* Acknowledge reading the FIFO entry */

          mcan_putreg(priv, SAM_MCAN_RXF1A_OFFSET, ndx);
          handled = true;
        }

      /* Check for successful reception of a new message in RX FIFO0 */

      /* Clear the RX FIFO0 new message interrupt */

      mcan_putreg(priv, SAM_MCAN_IR_OFFSET, MCAN_INT_RF0N);
      pending &= ~MCAN_INT_RF0N;

      /* Check if there is anything in RX FIFO0 */

      regval = mcan_getreg(priv, SAM_MCAN_RXF0S_OFFSET);
      nelem  = (regval & MCAN_RXF0S_F0FL_MASK) >> MCAN_RXF0S_F0FL_SHIFT;
      if (nelem > 0)
        {
          /* Handle the newly received message in FIFO0 */

          ndx = (regval & MCAN_RXF0S_F0GI_MASK) >> MCAN_RXF0S_F0GI_SHIFT;

          if ((regval & MCAN_RXF0S_RF0L) != 0)
            {
              canerr("ERROR: Message lost: %08"PRIx32"\n", regval);
            }
          else
            {
              mcan_receive(dev,
                           config->msgram.rxfifo0 +
                             (ndx * priv->config->rxfifo0esize),
                           priv->config->rxfifo0esize);

              /* Turning back on all configured RX error interrupts */

              ie |= (priv->rxints & MCAN_RXERR_INTS);
              mcan_putreg(priv, SAM_MCAN_IE_OFFSET, ie);
            }

          /* Acknowledge reading the FIFO entry */

          mcan_putreg(priv, SAM_MCAN_RXF0A_OFFSET, ndx);
          handled = true;
        }

      /* Clear unhandled RX interrupts */

      if ((pending & priv->rxints) != 0)
        {
          mcan_putreg(priv, SAM_MCAN_IR_OFFSET, priv->rxints);
        }
    }
  while (handled);

  return OK;
}

/****************************************************************************
 * Name: mcan_hw_initialize
 *
 * Description:
 *   MCAN hardware initialization
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this MCAN peripheral
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int mcan_hw_initialize(struct sam_mcan_s *priv)
{
  FAR const struct sam_config_s *config = priv->config;
  FAR uint32_t *msgram;
  uint32_t regval;
  uint32_t cntr;
  uint32_t cmr = 0;

  caninfo("MCAN%d\n", config->port);

  /* Configure MCAN pins */

  sam_configgpio(config->rxpinset);
  sam_configgpio(config->txpinset);

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

  /* Global Filter Configuration:
   *
   *   ANFS=0: Store all non matching standard frame in RX FIFO0
   *   ANFE=0: Store all non matching extended frame in RX FIFO0
   */

  regval = MCAN_GFC_ANFE_RX_FIFO0 | MCAN_GFC_ANFS_RX_FIFO0;
  mcan_putreg(priv, SAM_MCAN_GFC_OFFSET, regval);

  /* Extended ID Filter AND mask  */

  mcan_putreg(priv, SAM_MCAN_XIDAM_OFFSET, 0x1fffffff);

  /* Disable all interrupts  */

  mcan_putreg(priv, SAM_MCAN_IE_OFFSET, 0);
  mcan_putreg(priv, SAM_MCAN_TXBTIE_OFFSET, 0);

  /* All interrupts directed to Line 0.  But disable both interrupt lines 0
   * and 1 for now.
   *
   * REVISIT: Only interrupt line 0 is used by this driver.
   */

  mcan_putreg(priv, SAM_MCAN_ILS_OFFSET, 0);
  mcan_putreg(priv, SAM_MCAN_ILE_OFFSET, 0);

  /* Clear all pending interrupts. */

  if (priv->rev == 0)
    {
      mcan_putreg(priv, SAM_MCAN_IR_OFFSET, MCAN_REVA_INT_ALL);
    }
  else
    {
      mcan_putreg(priv, SAM_MCAN_IR_OFFSET, MCAN_REVB_INT_ALL);
    }

  /* Configure MCAN bit timing */

  if (priv->rev == 0)
    {
      mcan_putreg(priv, SAM_MCAN_REVA_BTP_OFFSET, priv->btp);
      mcan_putreg(priv, SAM_MCAN_REVA_FBTP_OFFSET, priv->fbtp);
    }
  else
    {
      mcan_putreg(priv, SAM_MCAN_NBTP_OFFSET, priv->btp);
      mcan_putreg(priv, SAM_MCAN_DBTP_OFFSET, priv->fbtp);
    }

  /* Configure message RAM starting addresses and sizes. */

  regval = MAILBOX_ADDRESS(config->msgram.stdfilters) |
           MCAN_SIDFC_LSS(config->nstdfilters);
  mcan_putreg(priv, SAM_MCAN_SIDFC_OFFSET, regval);

  regval = MAILBOX_ADDRESS(config->msgram.extfilters) |
           MCAN_XIDFC_LSE(config->nextfilters);
  mcan_putreg(priv, SAM_MCAN_XIDFC_OFFSET, regval);

  /* Configure RX FIFOs */

  regval = MAILBOX_ADDRESS(config->msgram.rxfifo0) |
           MCAN_RXF0C_F0S(config->nrxfifo0);
  mcan_putreg(priv, SAM_MCAN_RXF0C_OFFSET, regval);

  regval = MAILBOX_ADDRESS(config->msgram.rxfifo1) |
           MCAN_RXF1C_F1S(config->nrxfifo1);
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

  regval = MCAN_TXESC_TBDS(config->txbufferecode);
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
  if (priv->rev == 0)
    {
      regval &= ~(MCAN_CCCR_CME_MASK | MCAN_CCCR_CMR_MASK);
    }
  else
    {
      regval &= ~(MCAN_CCCR_FDOE | MCAN_CCCR_BRSE | MCAN_CCCR_NISO);
    }

  switch (config->mode)
    {
    default:
    case MCAN_ISO11898_1_MODE:
      if (priv->rev == 0)
        {
          regval |= MCAN_CCCR_CME_ISO11898_1;
          cmr     = MCAN_CCCR_CMR_ISO11898_1;
        }
      break;

#ifdef CONFIG_CAN_FD
    case MCAN_FD_MODE:
      if (priv->rev == 0)
        {
          regval |= MCAN_CCCR_CME_FD;
          cmr     = MCAN_CCCR_CMR_FD;
        }
      else
        {
          regval |= MCAN_CCCR_FDOE;
        }
      break;

    case MCAN_FD_BSW_MODE:
      if (priv->rev == 0)
        {
          regval |= MCAN_CCCR_CME_FD_BSW;
          cmr     = MCAN_CCCR_CMR_FD_BSW;
        }
      else
        {
          regval |= (MCAN_CCCR_FDOE | MCAN_CCCR_BRSE);
        }
      break;
#endif
    }

  /* Set the initial CAN mode */

  mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);

  if (priv->rev == 0)
    {
      /* Request the mode change */

      regval |= cmr;
      mcan_putreg(priv, SAM_MCAN_CCCR_OFFSET, regval);
    }

#if 0 /* Not necessary in initialization mode */
  /* Wait for the mode to take effect */

  while ((mcan_getreg(priv, SAM_MCAN_CCCR_OFFSET) &
         (MCAN_CCCR_FDBS | MCAN_CCCR_FDO)) != 0);
#endif

  /* Enable FIFO/Queue mode
   *
   * REVISIT: Dedicated TX buffers are not used.
   */

  regval  = mcan_getreg(priv, SAM_MCAN_TXBC_OFFSET);
#ifdef CONFIG_SAMV7_MCAN_QUEUE_MODE
  regval |= MCAN_TXBC_TFQM;
#else
  regval &= ~MCAN_TXBC_TFQM;
#endif
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

  /* Configure interrupt lines */

  /* Select RX-related interrupts */

#if 0 /* Dedicated RX buffers are not used by this driver */
  priv->rxints = MCAN_RXDEDBUF_INTS;
#else
  priv->rxints = MCAN_RXFIFO_INTS;
#endif

  /* Select TX-related interrupts */

#if 0 /* Dedicated TX buffers are not used by this driver */
  priv->txints = MCAN_TXDEDBUF_INTS;
#else
  priv->txints = MCAN_TXFIFOQ_INTS;
#endif

  /* Direct all interrupts to Line 0.
   *
   * Bits in the ILS register correspond to each MCAN interrupt; A bit
   * set to '1' is directed to interrupt line 1; a bit cleared to '0'
   * is directed interrupt line 0.
   *
   * REVISIT: Nothing is done here.  Only interrupt line 0 is used by
   * this driver and ILS was already cleared above.
   */

  /* Enable only interrupt line 0. */

  mcan_putreg(priv, SAM_MCAN_ILE_OFFSET, MCAN_ILE_EINT0);

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
 * Input Parameters:
 *   port - Port number (for hardware that has multiple MCAN interfaces),
 *          0=MCAN0, 1=MCAN1
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

  caninfo("MCAN%d\n", port);

  /* Select PCK5 clock source and pre-scaler value.  Both MCAN controllers
   * use PCK5 to derive bit rate.
   */

  regval = PMC_PCK_PRES(CONFIG_SAMV7_MCAN_CLKSRC_PRESCALER - 1) |
           SAMV7_MCAN_CLKSRC;
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
      regval |= (uint32_t)config->msgram.stdfilters &
                MATRIX_CAN0_CAN0DMABA_MASK;
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
      regval |= (uint32_t)config->msgram.stdfilters &
                MATRIX_CCFG_CAN1DMABA_MASK;
      putreg32(regval, SAM_MATRIX_CCFG_SYSIO);
    }
  else
#endif
    {
      canerr("ERROR: Unsupported port %d\n", port);
      return NULL;
    }

  /* Is this the first time that we have handed out this device? */

  if (priv->state == MCAN_STATE_UNINIT)
    {
      /* Yes, then perform one time data initialization */

      memset(priv, 0, sizeof(struct sam_mcan_s));
      priv->config = config;

      /* Get the revision of the chip (A or B ) */

      regval = getreg32(SAM_CHIPID_CIDR);
      priv->rev = regval & CHIPID_CIDR_VERSION_MASK;

      /* Set the initial bit timing.  This might change subsequently
       * due to IOCTL command processing.
       */

      if (priv->rev == 0)
        {
          /* Revision A */

          priv->btp    = config->btp;
          priv->fbtp   = config->fbtp;
        }
      else if (priv->rev == 1)
        {
          /* Revision B */

          priv->btp    = config->nbtp;
          priv->fbtp   = config->dbtp;
        }
      else
        {
          canerr("ERROR: Incorrect chip revision: %d\n", priv->rev);
          return NULL;
        }

      /* Initialize semaphores */

      nxsem_init(&priv->locksem, 0, 1);
      nxsem_init(&priv->txfsem, 0, config->ntxfifoq);

      dev->cd_ops  = &g_mcanops;
      dev->cd_priv = (FAR void *)priv;

      /* And put the hardware in the initial state */

      mcan_reset(dev);
    }

  return dev;
}

#endif /* CONFIG_CAN && CONFIG_SAMV7_MCAN */

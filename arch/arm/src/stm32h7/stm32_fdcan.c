/****************************************************************************
 * arch/arm/src/stm32h7/stm32_fdcan.c
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
 *   ST32H7 Series Data Sheet
 *   SAMv7 MCAN NuttX driver
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
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

#include "stm32_fdcan.h"
#include "hardware/stm32_pinmap.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"

#if defined(CONFIG_CAN) && defined(CONFIG_STM32H7_FDCAN)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock source *************************************************************/

#define STM32H7_FDCANCLK_FREQUENCY STM32_FDCANCLK

/* Buffer Alignment *********************************************************/

/* Buffer Alignment.
 *
 * The FDCAN peripheral does not require any data be aligned.  However, if
 * the data cache is enabled then alignment is required.  That is because
 * the data will need to be invalidated and that cache invalidation will
 * occur in multiples of full change lines.
 */

// #ifdef CONFIG_ARMV7M_DCACHE
// #  define FDCAN_ALIGN        ARMV7M_DCACHE_LINESIZE
// #  define FDCAN_ALIGN_MASK   (FDCAN_ALIGN-1)
// #  define FDCAN_ALIGN_UP(n)  (((n) + FDCAN_ALIGN_MASK) & ~FDCAN_ALIGN_MASK)

// #  ifndef CONFIG_ARMV7M_DCACHE_WRITETHROUGH
// #    warning !!! This driver will not work without CONFIG_ARMV7M_DCACHE_WRITETHROUGH=y!!!
// #  endif
// #endif

/* General Configuration ****************************************************/

#ifndef CONFIG_CAN_TXREADY
#  warning WARNING!!! CONFIG_CAN_TXREADY is required by this driver
#endif

#define MSGRAM_MAX_WORDS 2560

/* FDCAN1 Configuration ******************************************************/

#ifdef CONFIG_STM32H7_FDCAN1

/* Bit timing */

#  define FDCAN1_NTSEG1  (CONFIG_STM32H7_FDCAN1_NTSEG1 - 1)
#  define FDCAN1_NTSEG2  (CONFIG_STM32H7_FDCAN1_NTSEG2 - 1)
#  define FDCAN1_NBRP    ((uint32_t)(((float) STM32H7_FDCANCLK_FREQUENCY / \
                       ((float)(FDCAN1_NTSEG1 + FDCAN1_NTSEG2 + 3) * \
                        (float)CONFIG_STM32H7_FDCAN1_BITRATE)) - 1))
#  define FDCAN1_NSJW    (CONFIG_STM32H7_FDCAN1_NSJW - 1)

#  if FDCAN1_NTSEG1 > 63
#    error Invalid FDCAN1 NTSEG1
#  endif
#  if FDCAN1_NTSEG2 > 15
#    error Invalid FDCAN1 NTSEG2
#  endif
#  if FDCAN1_NSJW > 15
#    error Invalid FDCAN1 NSJW
#  endif

#  ifdef STM32H7_FDCAN1_FD_BRS
#  define FDCAN1_DTSEG1 (CONFIG_STM32H7_FDCAN1_DTSEG1 - 1)
#  define FDCAN1_DTSEG2 (CONFIG_STM32H7_FDCAN1_DTSEG2 - 1)
#  define FDCAN1_DBRP   ((uint32_t)(((float) STM32H7_FDCANCLK_FREQUENCY / \
                       ((float)(FDCAN1_DTSEG1 + FDCAN1_DTSEG2 + 3) * \
                        (float)CONFIG_STM32H7_FDCAN1_DBITRATE)) - 1))
#  define FDCAN1_DSJW   (CONFIG_STM32H7_FDCAN1_DSJW - 1)
#  else
#  define FDCAN1_DTSEG1 1
#  define FDCAN1_DTSEG2 1
#  define FDCAN1_DBRP   1
#  define FDCAN1_DSJW   1
#  endif /* STM32H7_FDCAN1_FD_BRS */

#  if FDCAN1_DTSEG1 > 15
#    error Invalid FDCAN1 DTSEG1
#  endif
#  if FDCAN1_DTSEG2 > 7
#    error Invalid FDCAN1 DTSEG2
#  endif
#  if FDCAN1_DSJW > 3
#    error Invalid FDCAN1 DSJW
#  endif

/* FDCAN1 Message RAM Configuration ******************************************************/

/* FDCAN1 standard filters */

#  ifndef CONFIG_STM32H7_FDCAN1_NSTDFILTERS
#    define CONFIG_STM32H7_FDCAN1_NSTDFILTERS 0
#  endif

#  if (CONFIG_STM32H7_FDCAN1_NSTDFILTERS > 128)
#    error Invalid FDCAN1 number of Standard Filters
#  endif

#  define FDCAN1_STDFILTER_BYTES \
      (CONFIG_STM32H7_FDCAN1_NSTDFILTERS << 2)
      // FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN1_NSTDFILTERS << 2)
#  define FDCAN1_STDFILTER_WORDS (FDCAN1_STDFILTER_BYTES >> 2)


/* FDCAN1 extended filters */

#  ifndef CONFIG_STM32H7_FDCAN1_NEXTFILTERS
#    define CONFIG_STM32H7_FDCAN1_NEXTFILTERS 0
#  endif

#  if (CONFIG_STM32H7_FDCAN1_NEXTFILTERS > 64)
#    error Invalid FDCAN1 number of Extended Filters
#  endif

#  define FDCAN1_EXTFILTER_BYTES \
      (CONFIG_STM32H7_FDCAN1_NEXTFILTERS << 3)
      // FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN1_NEXTFILTERS << 3)
#  define FDCAN1_EXTFILTER_WORDS (FDCAN1_EXTFILTER_BYTES >> 2)


/* FDCAN1 RX FIFO0 */

#  ifndef CONFIG_STM32H7_FDCAN1_RXFIFO0_SIZE
#    define CONFIG_STM32H7_FDCAN1_RXFIFO0_SIZE 0
#  endif

#  if CONFIG_STM32H7_FDCAN1_RXFIFO0_SIZE > 64
#    error Invalid FDCAN1 number of RX FIFO0 elements
#  endif

#  if defined(CONFIG_STM32H7_FDCAN1_RXFIFO0_8BYTES)
#    define FDCAN1_RXFIFO0_ELEMENT_SIZE  8
#    define FDCAN1_RXFIFO0_ENCODED_SIZE  0
#  elif defined(CONFIG_STM32H7_FDCAN1_RXFIFO0_12BYTES)
#    define FDCAN1_RXFIFO0_ELEMENT_SIZE  12
#    define FDCAN1_RXFIFO0_ENCODED_SIZE  1
#  elif defined(CONFIG_STM32H7_FDCAN1_RXFIFO0_16BYTES)
#    define FDCAN1_RXFIFO0_ELEMENT_SIZE  16
#    define FDCAN1_RXFIFO0_ENCODED_SIZE  2
#  elif defined(CONFIG_STM32H7_FDCAN1_RXFIFO0_20BYTES)
#    define FDCAN1_RXFIFO0_ELEMENT_SIZE  20
#    define FDCAN1_RXFIFO0_ENCODED_SIZE  3
#  elif defined(CONFIG_STM32H7_FDCAN1_RXFIFO0_24BYTES)
#    define FDCAN1_RXFIFO0_ELEMENT_SIZE  24
#    define FDCAN1_RXFIFO0_ENCODED_SIZE  4
#  elif defined(CONFIG_STM32H7_FDCAN1_RXFIFO0_32BYTES)
#    define FDCAN1_RXFIFO0_ELEMENT_SIZE  32
#    define FDCAN1_RXFIFO0_ENCODED_SIZE  5
#  elif defined(CONFIG_STM32H7_FDCAN1_RXFIFO0_48BYTES)
#    define FDCAN1_RXFIFO0_ELEMENT_SIZE  48
#    define FDCAN1_RXFIFO0_ENCODED_SIZE  6
#  elif defined(CONFIG_STM32H7_FDCAN1_RXFIFO0_64BYTES)
#    define FDCAN1_RXFIFO0_ELEMENT_SIZE  64
#    define FDCAN1_RXFIFO0_ENCODED_SIZE  7
#  else
#    define FDCAN1_RXFIFO0_ELEMENT_SIZE  8
#    define FDCAN1_RXFIFO0_ENCODED_SIZE  0
#  endif

#  define FDCAN1_RXFIFO0_BYTES \
     (CONFIG_STM32H7_FDCAN1_RXFIFO0_SIZE * \
                  (FDCAN1_RXFIFO0_ELEMENT_SIZE + 8))
     /* ( FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN1_RXFIFO0_SIZE * \
      *               FDCAN1_RXFIFO0_ELEMENT_SIZE + 8)
      */
#  define FDCAN1_RXFIFO0_WORDS (FDCAN1_RXFIFO0_BYTES >> 2)


/* FDCAN1 RX FIFO1 */

#  ifndef CONFIG_STM32H7_FDCAN1_RXFIFO1_SIZE
#    define CONFIG_STM32H7_FDCAN1_RXFIFO1_SIZE 0
#  endif

#  if CONFIG_STM32H7_FDCAN1_RXFIFO1_SIZE > 64
#    error Invalid FDCAN1 number of RX FIFO1 elements
#  endif

#  if defined(CONFIG_STM32H7_FDCAN1_RXFIFO1_8BYTES)
#    define FDCAN1_RXFIFO1_ELEMENT_SIZE  8
#    define FDCAN1_RXFIFO1_ENCODED_SIZE  0
#  elif defined(CONFIG_STM32H7_FDCAN1_RXFIFO1_12BYTES)
#    define FDCAN1_RXFIFO1_ELEMENT_SIZE  12
#    define FDCAN1_RXFIFO1_ENCODED_SIZE  1
#  elif defined(CONFIG_STM32H7_FDCAN1_RXFIFO1_16BYTES)
#    define FDCAN1_RXFIFO1_ELEMENT_SIZE  16
#    define FDCAN1_RXFIFO1_ENCODED_SIZE  2
#  elif defined(CONFIG_STM32H7_FDCAN1_RXFIFO1_20BYTES)
#    define FDCAN1_RXFIFO1_ELEMENT_SIZE  20
#    define FDCAN1_RXFIFO1_ENCODED_SIZE  3
#  elif defined(CONFIG_STM32H7_FDCAN1_RXFIFO1_24BYTES)
#    define FDCAN1_RXFIFO1_ELEMENT_SIZE  24
#    define FDCAN1_RXFIFO1_ENCODED_SIZE  4
#  elif defined(CONFIG_STM32H7_FDCAN1_RXFIFO1_32BYTES)
#    define FDCAN1_RXFIFO1_ELEMENT_SIZE  32
#    define FDCAN1_RXFIFO1_ENCODED_SIZE  5
#  elif defined(CONFIG_STM32H7_FDCAN1_RXFIFO1_48BYTES)
#    define FDCAN1_RXFIFO1_ELEMENT_SIZE  48
#    define FDCAN1_RXFIFO1_ENCODED_SIZE  6
#  elif defined(CONFIG_STM32H7_FDCAN1_RXFIFO1_64BYTES)
#    define FDCAN1_RXFIFO1_ELEMENT_SIZE  64
#    define FDCAN1_RXFIFO1_ENCODED_SIZE  7
#  else
#    define FDCAN1_RXFIFO1_ELEMENT_SIZE  8
#    define FDCAN1_RXFIFO1_ENCODED_SIZE  0
#  endif

/* #  define FDCAN1_RXFIFO1_BYTES \
 *      FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN1_RXFIFO1_SIZE * \
 *                   FDCAN1_RXFIFO1_ELEMENT_SIZE + 8)
 */

#  define FDCAN1_RXFIFO1_BYTES (CONFIG_STM32H7_FDCAN1_RXFIFO1_SIZE * \
                   (FDCAN1_RXFIFO1_ELEMENT_SIZE + 8))
#  define FDCAN1_RXFIFO1_WORDS (FDCAN1_RXFIFO1_BYTES >> 2)


/* FDCAN1 RX dedicated buffer */

#  ifndef CONFIG_STM32H7_FDCAN1_DEDICATED_RXBUFFER_SIZE
#    define CONFIG_STM32H7_FDCAN1_DEDICATED_RXBUFFER_SIZE 0
#  endif

#  if CONFIG_STM32H7_FDCAN1_DEDICATED_RXBUFFER_SIZE > 64
#    error Invalid FDCAN1 number of RX BUFFER elements
#  endif

#  if defined(CONFIG_STM32H7_FDCAN1_RXBUFFER_8BYTES)
#    define FDCAN1_RXBUFFER_ELEMENT_SIZE  8
#    define FDCAN1_RXBUFFER_ENCODED_SIZE  0
#  elif defined(CONFIG_STM32H7_FDCAN1_RXBUFFER_12BYTES)
#    define FDCAN1_RXBUFFER_ELEMENT_SIZE  12
#    define FDCAN1_RXBUFFER_ENCODED_SIZE  1
#  elif defined(CONFIG_STM32H7_FDCAN1_RXBUFFER_16BYTES)
#    define FDCAN1_RXBUFFER_ELEMENT_SIZE  16
#    define FDCAN1_RXBUFFER_ENCODED_SIZE  2
#  elif defined(CONFIG_STM32H7_FDCAN1_RXBUFFER_20BYTES)
#    define FDCAN1_RXBUFFER_ELEMENT_SIZE  20
#    define FDCAN1_RXBUFFER_ENCODED_SIZE  3
#  elif defined(CONFIG_STM32H7_FDCAN1_RXBUFFER_24BYTES)
#    define FDCAN1_RXBUFFER_ELEMENT_SIZE  24
#    define FDCAN1_RXBUFFER_ENCODED_SIZE  4
#  elif defined(CONFIG_STM32H7_FDCAN1_RXBUFFER_32BYTES)
#    define FDCAN1_RXBUFFER_ELEMENT_SIZE  32
#    define FDCAN1_RXBUFFER_ENCODED_SIZE  5
#  elif defined(CONFIG_STM32H7_FDCAN1_RXBUFFER_48BYTES)
#    define FDCAN1_RXBUFFER_ELEMENT_SIZE  48
#    define FDCAN1_RXBUFFER_ENCODED_SIZE  6
#  elif defined(CONFIG_STM32H7_FDCAN1_RXBUFFER_64BYTES)
#    define FDCAN1_RXBUFFER_ELEMENT_SIZE  64
#    define FDCAN1_RXBUFFER_ENCODED_SIZE  7
#  else
#    define FDCAN1_RXBUFFER_ELEMENT_SIZE  8
#    define FDCAN1_RXBUFFER_ENCODED_SIZE  0
#  endif

#  define FDCAN1_DEDICATED_RXBUFFER_BYTES \
     (CONFIG_STM32H7_FDCAN1_DEDICATED_RXBUFFER_SIZE * \
                   (FDCAN1_RXBUFFER_ELEMENT_SIZE + 8))
     /* FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN1_DEDICATED_RXBUFFER_SIZE * \
       *            FDCAN1_RXBUFFER_ELEMENT_SIZE + 8)
  */
#  define FDCAN1_DEDICATED_RXBUFFER_WORDS \
     (FDCAN1_DEDICATED_RXBUFFER_BYTES >> 2)


/* FDCAN1 TX buffer element size */

#  if defined(CONFIG_STM32H7_FDCAN1_TXBUFFER_8BYTES)
#    define FDCAN1_TXBUFFER_ELEMENT_SIZE  8
#    define FDCAN1_TXBUFFER_ENCODED_SIZE  0
#  elif defined(CONFIG_STM32H7_FDCAN1_TXBUFFER_12BYTES)
#    define FDCAN1_TXBUFFER_ELEMENT_SIZE  12
#    define FDCAN1_TXBUFFER_ENCODED_SIZE  1
#  elif defined(CONFIG_STM32H7_FDCAN1_TXBUFFER_16BYTES)
#    define FDCAN1_TXBUFFER_ELEMENT_SIZE  16
#    define FDCAN1_TXBUFFER_ENCODED_SIZE  2
#  elif defined(CONFIG_STM32H7_FDCAN1_TXBUFFER_20BYTES)
#    define FDCAN1_TXBUFFER_ELEMENT_SIZE  20
#    define FDCAN1_TXBUFFER_ENCODED_SIZE  3
#  elif defined(CONFIG_STM32H7_FDCAN1_TXBUFFER_24BYTES)
#    define FDCAN1_TXBUFFER_ELEMENT_SIZE  24
#    define FDCAN1_TXBUFFER_ENCODED_SIZE  4
#  elif defined(CONFIG_STM32H7_FDCAN1_TXBUFFER_32BYTES)
#    define FDCAN1_TXBUFFER_ELEMENT_SIZE  32
#    define FDCAN1_TXBUFFER_ENCODED_SIZE  5
#  elif defined(CONFIG_STM32H7_FDCAN1_TXBUFFER_48BYTES)
#    define FDCAN1_TXBUFFER_ELEMENT_SIZE  48
#    define FDCAN1_TXBUFFER_ENCODED_SIZE  6
#  elif defined(CONFIG_STM32H7_FDCAN1_TXBUFFER_64BYTES)
#    define FDCAN1_TXBUFFER_ELEMENT_SIZE  64
#    define FDCAN1_TXBUFFER_ENCODED_SIZE  7
#  else
#    define FDCAN1_TXBUFFER_ELEMENT_SIZE  8
#    define FDCAN1_TXBUFFER_ENCODED_SIZE  0
#  endif


/* FDCAN1 TX dedicated buffer */

#  ifndef CONFIG_STM32H7_FDCAN1_DEDICATED_TXBUFFER_SIZE
#    define CONFIG_STM32H7_FDCAN1_DEDICATED_TXBUFFER_SIZE 0
#  endif

#  define FDCAN1_DEDICATED_TXBUFFER_BYTES \
     (CONFIG_STM32H7_FDCAN1_DEDICATED_TXBUFFER_SIZE * \
                   (FDCAN1_TXBUFFER_ELEMENT_SIZE + 8))
     /* FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN1_DEDICATED_TXBUFFER_SIZE * \
      *               FDCAN1_TXBUFFER_ELEMENT_SIZE + 8)
      */
#  define FDCAN1_DEDICATED_TXBUFFER_WORDS \
     (FDCAN1_DEDICATED_TXBUFFER_BYTES >> 2)


/* FDCAN1 TX FIFO/Queue*/

#  ifndef CONFIG_STM32H7_FDCAN1_TXFIFOQ_SIZE
#    define CONFIG_STM32H7_FDCAN1_TXFIFOQ_SIZE 0
#  endif

#  if (CONFIG_STM32H7_FDCAN1_DEDICATED_TXBUFFER_SIZE + \
       CONFIG_STM32H7_FDCAN1_TXFIFOQ_SIZE) > 32
#    error Invalid FDCAN1 number of TX BUFFER elements
#  endif

#  define FDCAN1_TXFIFIOQ_BYTES \
     (CONFIG_STM32H7_FDCAN1_TXFIFOQ_SIZE *  \
                   (FDCAN1_TXBUFFER_ELEMENT_SIZE + 8))
     /*FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN1_TXFIFOQ_SIZE *  \
       *            FDCAN1_TXBUFFER_ELEMENT_SIZE + 8)
*/
#  define FDCAN1_TXFIFIOQ_WORDS (FDCAN1_TXFIFIOQ_BYTES >> 2)


/* FDCAN1 TX Event FIFO */

#  ifndef CONFIG_STM32H7_FDCAN1_TXEVENTFIFO_SIZE
#    define CONFIG_STM32H7_FDCAN1_TXEVENTFIFO_SIZE 0
#  endif

#  if CONFIG_STM32H7_FDCAN1_TXEVENTFIFO_SIZE > 32
#    error Invalid FDCAN1 number of TX EVENT FIFO elements
#  endif

#  define FDCAN1_TXEVENTFIFO_BYTES \
     (CONFIG_STM32H7_FDCAN1_TXEVENTFIFO_SIZE << 3)
     // FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN1_TXEVENTFIFO_SIZE << 3)
#  define FDCAN1_TXEVENTFIFO_WORDS \
     (FDCAN1_TXEVENTFIFO_BYTES >> 2)


/* FDCAN1 Message RAM Layout*/

#  define FDCAN1_STDFILTER_INDEX   0
#  define FDCAN1_EXTFILTERS_INDEX  (FDCAN1_STDFILTER_INDEX + FDCAN1_STDFILTER_WORDS)
#  define FDCAN1_RXFIFO0_INDEX     (FDCAN1_EXTFILTERS_INDEX + FDCAN1_EXTFILTER_WORDS)
#  define FDCAN1_RXFIFO1_INDEX     (FDCAN1_RXFIFO0_INDEX + FDCAN1_RXFIFO0_WORDS)
#  define FDCAN1_RXDEDICATED_INDEX (FDCAN1_RXFIFO1_INDEX + FDCAN1_RXFIFO1_WORDS)
#  define FDCAN1_TXEVENTFIFO_INDEX (FDCAN1_RXDEDICATED_INDEX + FDCAN1_DEDICATED_RXBUFFER_WORDS)
#  define FDCAN1_TXDEDICATED_INDEX (FDCAN1_TXEVENTFIFO_INDEX + FDCAN1_TXEVENTFIFO_WORDS)
#  define FDCAN1_TXFIFOQ_INDEX     (FDCAN1_TXDEDICATED_INDEX + FDCAN1_DEDICATED_TXBUFFER_WORDS)
#  define FDCAN1_MSGRAM_WORDS      (FDCAN1_TXFIFOQ_INDEX + FDCAN1_TXFIFIOQ_WORDS)

#  if FDCAN1_MSGRAM_WORDS > MSGRAM_MAX_WORDS
#    error FDCAN1 config exceeds message RAM word limit
#  endif

#endif /* CONFIG_STM32H7_FDCAN1 */

/* FDCAN2 Configuration ******************************************************/

#ifdef CONFIG_STM32H7_FDCAN2

/* Bit timing */

#  define FDCAN2_NTSEG1  (CONFIG_STM32H7_FDCAN2_NTSEG1 - 1)
#  define FDCAN2_NTSEG2  (CONFIG_STM32H7_FDCAN2_NTSEG2 - 1)
#  define FDCAN2_NBRP    ((uint32_t)(((float) STM32H7_FDCANCLK_FREQUENCY / \
                       ((float)(FDCAN2_NTSEG1 + FDCAN2_NTSEG2 + 3) * \
                        (float)CONFIG_STM32H7_FDCAN2_BITRATE)) - 1))
#  define FDCAN2_NSJW    (CONFIG_STM32H7_FDCAN2_NSJW - 1)

#  if FDCAN2_NTSEG1 > 63
#    error Invalid FDCAN2 NTSEG1
#  endif
#  if FDCAN2_NTSEG2 > 15
#    error Invalid FDCAN2 NTSEG2
#  endif
#  if FDCAN2_NSJW > 15
#    error Invalid FDCAN2 NSJW
#  endif

#  ifdef STM32H7_FDCAN2_FD_BRS
#  define FDCAN2_DTSEG1 (CONFIG_STM32H7_FDCAN2_DTSEG1 - 1)
#  define FDCAN2_DTSEG2 (CONFIG_STM32H7_FDCAN2_DTSEG2 - 1)
#  define FDCAN2_DBRP   ((uint32_t)(((float) STM32H7_FDCANCLK_FREQUENCY / \
                       ((float)(FDCAN2_DTSEG1 + FDCAN2_DTSEG2 + 3) * \
                        (float)CONFIG_STM32H7_FDCAN2_DBITRATE)) - 1))
#  define FDCAN2_DSJW   (CONFIG_STM32H7_FDCAN2_DSJW - 1)
#  else
#  define FDCAN2_DTSEG1 1
#  define FDCAN2_DTSEG2 1
#  define FDCAN2_DBRP   1
#  define FDCAN2_DSJW   1
#  endif /* STM32H7_FDCAN2_FD_BRS */

#  if FDCAN2_DTSEG1 > 15
#    error Invalid FDCAN2 DTSEG1
#  endif
#  if FDCAN2_DTSEG2 > 7
#    error Invalid FDCAN2 DTSEG2
#  endif
#  if FDCAN2_DSJW > 3
#    error Invalid FDCAN2 DSJW
#  endif

/* FDCAN2 Message RAM Configuration ******************************************************/

/* FDCAN2 standard filters */

#  ifndef CONFIG_STM32H7_FDCAN2_NSTDFILTERS
#    define CONFIG_STM32H7_FDCAN2_NSTDFILTERS 0
#  endif

#  if (CONFIG_STM32H7_FDCAN2_NSTDFILTERS > 128)
#    error Invalid FDCAN2 number of Standard Filters
#  endif

#  define FDCAN2_STDFILTER_BYTES \
      (CONFIG_STM32H7_FDCAN2_NSTDFILTERS << 2)
      // FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN2_NSTDFILTERS << 2)
#  define FDCAN2_STDFILTER_WORDS (FDCAN2_STDFILTER_BYTES >> 2)


/* FDCAN2 extended filters */

#  ifndef CONFIG_STM32H7_FDCAN2_NEXTFILTERS
#    define CONFIG_STM32H7_FDCAN2_NEXTFILTERS 0
#  endif

#  if (CONFIG_STM32H7_FDCAN2_NEXTFILTERS > 64)
#    error Invalid FDCAN2 number of Extended Filters
#  endif

#  define FDCAN2_EXTFILTER_BYTES \
      (CONFIG_STM32H7_FDCAN2_NEXTFILTERS << 3)
      // FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN2_NEXTFILTERS << 3)
#  define FDCAN2_EXTFILTER_WORDS (FDCAN2_EXTFILTER_BYTES >> 2)


/* FDCAN2 RX FIFO0 */

#  ifndef CONFIG_STM32H7_FDCAN2_RXFIFO0_SIZE
#    define CONFIG_STM32H7_FDCAN2_RXFIFO0_SIZE 0
#  endif

#  if CONFIG_STM32H7_FDCAN2_RXFIFO0_SIZE > 64
#    error Invalid FDCAN2 number of RX FIFO0 elements
#  endif

#  if defined(CONFIG_STM32H7_FDCAN2_RXFIFO0_8BYTES)
#    define FDCAN2_RXFIFO0_ELEMENT_SIZE  8
#    define FDCAN2_RXFIFO0_ENCODED_SIZE  0
#  elif defined(CONFIG_STM32H7_FDCAN2_RXFIFO0_12BYTES)
#    define FDCAN2_RXFIFO0_ELEMENT_SIZE  12
#    define FDCAN2_RXFIFO0_ENCODED_SIZE  1
#  elif defined(CONFIG_STM32H7_FDCAN2_RXFIFO0_16BYTES)
#    define FDCAN2_RXFIFO0_ELEMENT_SIZE  16
#    define FDCAN2_RXFIFO0_ENCODED_SIZE  2
#  elif defined(CONFIG_STM32H7_FDCAN2_RXFIFO0_20BYTES)
#    define FDCAN2_RXFIFO0_ELEMENT_SIZE  20
#    define FDCAN2_RXFIFO0_ENCODED_SIZE  3
#  elif defined(CONFIG_STM32H7_FDCAN2_RXFIFO0_24BYTES)
#    define FDCAN2_RXFIFO0_ELEMENT_SIZE  24
#    define FDCAN2_RXFIFO0_ENCODED_SIZE  4
#  elif defined(CONFIG_STM32H7_FDCAN2_RXFIFO0_32BYTES)
#    define FDCAN2_RXFIFO0_ELEMENT_SIZE  32
#    define FDCAN2_RXFIFO0_ENCODED_SIZE  5
#  elif defined(CONFIG_STM32H7_FDCAN2_RXFIFO0_48BYTES)
#    define FDCAN2_RXFIFO0_ELEMENT_SIZE  48
#    define FDCAN2_RXFIFO0_ENCODED_SIZE  6
#  elif defined(CONFIG_STM32H7_FDCAN2_RXFIFO0_64BYTES)
#    define FDCAN2_RXFIFO0_ELEMENT_SIZE  64
#    define FDCAN2_RXFIFO0_ENCODED_SIZE  7
#  else
#    define FDCAN2_RXFIFO0_ELEMENT_SIZE  8
#    define FDCAN2_RXFIFO0_ENCODED_SIZE  0
#  endif

#  define FDCAN2_RXFIFO0_BYTES \
     (CONFIG_STM32H7_FDCAN2_RXFIFO0_SIZE * \
                  (FDCAN2_RXFIFO0_ELEMENT_SIZE + 8))
     /* ( FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN2_RXFIFO0_SIZE * \
      *               FDCAN2_RXFIFO0_ELEMENT_SIZE + 8)
      */               
#  define FDCAN2_RXFIFO0_WORDS (FDCAN2_RXFIFO0_BYTES >> 2)


/* FDCAN2 RX FIFO1 */

#  ifndef CONFIG_STM32H7_FDCAN2_RXFIFO1_SIZE
#    define CONFIG_STM32H7_FDCAN2_RXFIFO1_SIZE 0
#  endif

#  if CONFIG_STM32H7_FDCAN2_RXFIFO1_SIZE > 64
#    error Invalid FDCAN2 number of RX FIFO1 elements
#  endif

#  if defined(CONFIG_STM32H7_FDCAN2_RXFIFO1_8BYTES)
#    define FDCAN2_RXFIFO1_ELEMENT_SIZE  8
#    define FDCAN2_RXFIFO1_ENCODED_SIZE  0
#  elif defined(CONFIG_STM32H7_FDCAN2_RXFIFO1_12BYTES)
#    define FDCAN2_RXFIFO1_ELEMENT_SIZE  12
#    define FDCAN2_RXFIFO1_ENCODED_SIZE  1
#  elif defined(CONFIG_STM32H7_FDCAN2_RXFIFO1_16BYTES)
#    define FDCAN2_RXFIFO1_ELEMENT_SIZE  16
#    define FDCAN2_RXFIFO1_ENCODED_SIZE  2
#  elif defined(CONFIG_STM32H7_FDCAN2_RXFIFO1_20BYTES)
#    define FDCAN2_RXFIFO1_ELEMENT_SIZE  20
#    define FDCAN2_RXFIFO1_ENCODED_SIZE  3
#  elif defined(CONFIG_STM32H7_FDCAN2_RXFIFO1_24BYTES)
#    define FDCAN2_RXFIFO1_ELEMENT_SIZE  24
#    define FDCAN2_RXFIFO1_ENCODED_SIZE  4
#  elif defined(CONFIG_STM32H7_FDCAN2_RXFIFO1_32BYTES)
#    define FDCAN2_RXFIFO1_ELEMENT_SIZE  32
#    define FDCAN2_RXFIFO1_ENCODED_SIZE  5
#  elif defined(CONFIG_STM32H7_FDCAN2_RXFIFO1_48BYTES)
#    define FDCAN2_RXFIFO1_ELEMENT_SIZE  48
#    define FDCAN2_RXFIFO1_ENCODED_SIZE  6
#  elif defined(CONFIG_STM32H7_FDCAN2_RXFIFO1_64BYTES)
#    define FDCAN2_RXFIFO1_ELEMENT_SIZE  64
#    define FDCAN2_RXFIFO1_ENCODED_SIZE  7
#  else
#    define FDCAN2_RXFIFO1_ELEMENT_SIZE  8
#    define FDCAN2_RXFIFO1_ENCODED_SIZE  0
#  endif

/* #  define FDCAN2_RXFIFO1_BYTES \
 *      FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN2_RXFIFO1_SIZE * \
 *                   FDCAN2_RXFIFO1_ELEMENT_SIZE + 8)
 */

#  define FDCAN2_RXFIFO1_BYTES (CONFIG_STM32H7_FDCAN2_RXFIFO1_SIZE * \
                   (FDCAN2_RXFIFO1_ELEMENT_SIZE + 8))
#  define FDCAN2_RXFIFO1_WORDS (FDCAN2_RXFIFO1_BYTES >> 2)


/* FDCAN2 RX dedicated buffer */

#  ifndef CONFIG_STM32H7_FDCAN2_DEDICATED_RXBUFFER_SIZE
#    define CONFIG_STM32H7_FDCAN2_DEDICATED_RXBUFFER_SIZE 0
#  endif

#  if CONFIG_STM32H7_FDCAN2_DEDICATED_RXBUFFER_SIZE > 64
#    error Invalid FDCAN2 number of RX BUFFER elements
#  endif

#  if defined(CONFIG_STM32H7_FDCAN2_RXBUFFER_8BYTES)
#    define FDCAN2_RXBUFFER_ELEMENT_SIZE  8
#    define FDCAN2_RXBUFFER_ENCODED_SIZE  0
#  elif defined(CONFIG_STM32H7_FDCAN2_RXBUFFER_12BYTES)
#    define FDCAN2_RXBUFFER_ELEMENT_SIZE  12
#    define FDCAN2_RXBUFFER_ENCODED_SIZE  1
#  elif defined(CONFIG_STM32H7_FDCAN2_RXBUFFER_16BYTES)
#    define FDCAN2_RXBUFFER_ELEMENT_SIZE  16
#    define FDCAN2_RXBUFFER_ENCODED_SIZE  2
#  elif defined(CONFIG_STM32H7_FDCAN2_RXBUFFER_20BYTES)
#    define FDCAN2_RXBUFFER_ELEMENT_SIZE  20
#    define FDCAN2_RXBUFFER_ENCODED_SIZE  3
#  elif defined(CONFIG_STM32H7_FDCAN2_RXBUFFER_24BYTES)
#    define FDCAN2_RXBUFFER_ELEMENT_SIZE  24
#    define FDCAN2_RXBUFFER_ENCODED_SIZE  4
#  elif defined(CONFIG_STM32H7_FDCAN2_RXBUFFER_32BYTES)
#    define FDCAN2_RXBUFFER_ELEMENT_SIZE  32
#    define FDCAN2_RXBUFFER_ENCODED_SIZE  5
#  elif defined(CONFIG_STM32H7_FDCAN2_RXBUFFER_48BYTES)
#    define FDCAN2_RXBUFFER_ELEMENT_SIZE  48
#    define FDCAN2_RXBUFFER_ENCODED_SIZE  6
#  elif defined(CONFIG_STM32H7_FDCAN2_RXBUFFER_64BYTES)
#    define FDCAN2_RXBUFFER_ELEMENT_SIZE  64
#    define FDCAN2_RXBUFFER_ENCODED_SIZE  7
#  else
#    define FDCAN2_RXBUFFER_ELEMENT_SIZE  8
#    define FDCAN2_RXBUFFER_ENCODED_SIZE  0
#  endif

#  define FDCAN2_DEDICATED_RXBUFFER_BYTES \
     (CONFIG_STM32H7_FDCAN2_DEDICATED_RXBUFFER_SIZE * \
                   (FDCAN2_RXBUFFER_ELEMENT_SIZE + 8))
     /* FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN2_DEDICATED_RXBUFFER_SIZE * \
       *            FDCAN2_RXBUFFER_ELEMENT_SIZE + 8)
  */
#  define FDCAN2_DEDICATED_RXBUFFER_WORDS \
     (FDCAN2_DEDICATED_RXBUFFER_BYTES >> 2)


/* FDCAN2 TX buffer element size */

#  if defined(CONFIG_STM32H7_FDCAN2_TXBUFFER_8BYTES)
#    define FDCAN2_TXBUFFER_ELEMENT_SIZE  8
#    define FDCAN2_TXBUFFER_ENCODED_SIZE  0
#  elif defined(CONFIG_STM32H7_FDCAN2_TXBUFFER_12BYTES)
#    define FDCAN2_TXBUFFER_ELEMENT_SIZE  12
#    define FDCAN2_TXBUFFER_ENCODED_SIZE  1
#  elif defined(CONFIG_STM32H7_FDCAN2_TXBUFFER_16BYTES)
#    define FDCAN2_TXBUFFER_ELEMENT_SIZE  16
#    define FDCAN2_TXBUFFER_ENCODED_SIZE  2
#  elif defined(CONFIG_STM32H7_FDCAN2_TXBUFFER_20BYTES)
#    define FDCAN2_TXBUFFER_ELEMENT_SIZE  20
#    define FDCAN2_TXBUFFER_ENCODED_SIZE  3
#  elif defined(CONFIG_STM32H7_FDCAN2_TXBUFFER_24BYTES)
#    define FDCAN2_TXBUFFER_ELEMENT_SIZE  24
#    define FDCAN2_TXBUFFER_ENCODED_SIZE  4
#  elif defined(CONFIG_STM32H7_FDCAN2_TXBUFFER_32BYTES)
#    define FDCAN2_TXBUFFER_ELEMENT_SIZE  32
#    define FDCAN2_TXBUFFER_ENCODED_SIZE  5
#  elif defined(CONFIG_STM32H7_FDCAN2_TXBUFFER_48BYTES)
#    define FDCAN2_TXBUFFER_ELEMENT_SIZE  48
#    define FDCAN2_TXBUFFER_ENCODED_SIZE  6
#  elif defined(CONFIG_STM32H7_FDCAN2_TXBUFFER_64BYTES)
#    define FDCAN2_TXBUFFER_ELEMENT_SIZE  64
#    define FDCAN2_TXBUFFER_ENCODED_SIZE  7
#  else
#    define FDCAN2_TXBUFFER_ELEMENT_SIZE  8
#    define FDCAN2_TXBUFFER_ENCODED_SIZE  0
#  endif


/* FDCAN2 TX dedicated buffer */

#  ifndef CONFIG_STM32H7_FDCAN2_DEDICATED_TXBUFFER_SIZE
#    define CONFIG_STM32H7_FDCAN2_DEDICATED_TXBUFFER_SIZE 0
#  endif

#  define FDCAN2_DEDICATED_TXBUFFER_BYTES \
     (CONFIG_STM32H7_FDCAN2_DEDICATED_TXBUFFER_SIZE * \
                   (FDCAN2_TXBUFFER_ELEMENT_SIZE + 8))
     /* FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN2_DEDICATED_TXBUFFER_SIZE * \
      *               FDCAN2_TXBUFFER_ELEMENT_SIZE + 8)
      */
#  define FDCAN2_DEDICATED_TXBUFFER_WORDS \
     (FDCAN2_DEDICATED_TXBUFFER_BYTES >> 2)


/* FDCAN2 TX FIFO/Queue*/

#  ifndef CONFIG_STM32H7_FDCAN2_TXFIFOQ_SIZE
#    define CONFIG_STM32H7_FDCAN2_TXFIFOQ_SIZE 0
#  endif

#  if (CONFIG_STM32H7_FDCAN2_DEDICATED_TXBUFFER_SIZE + \
       CONFIG_STM32H7_FDCAN2_TXFIFOQ_SIZE) > 32
#    error Invalid FDCAN2 number of TX BUFFER elements
#  endif

#  define FDCAN2_TXFIFIOQ_BYTES \
     (CONFIG_STM32H7_FDCAN2_TXFIFOQ_SIZE *  \
                   (FDCAN2_TXBUFFER_ELEMENT_SIZE + 8))
     /*FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN2_TXFIFOQ_SIZE *  \
       *            FDCAN2_TXBUFFER_ELEMENT_SIZE + 8)
*/
#  define FDCAN2_TXFIFIOQ_WORDS (FDCAN2_TXFIFIOQ_BYTES >> 2)


/* FDCAN2 TX Event FIFO */

#  ifndef CONFIG_STM32H7_FDCAN2_TXEVENTFIFO_SIZE
#    define CONFIG_STM32H7_FDCAN2_TXEVENTFIFO_SIZE 0
#  endif

#  if CONFIG_STM32H7_FDCAN2_TXEVENTFIFO_SIZE > 32
#    error Invalid FDCAN2 number of TX EVENT FIFO elements
#  endif

#  define FDCAN2_TXEVENTFIFO_BYTES \
     (CONFIG_STM32H7_FDCAN2_TXEVENTFIFO_SIZE << 3)
     // FDCAN_ALIGN_UP(CONFIG_STM32H7_FDCAN2_TXEVENTFIFO_SIZE << 3)
#  define FDCAN2_TXEVENTFIFO_WORDS \
     (FDCAN2_TXEVENTFIFO_BYTES >> 2)


/* FDCAN2 Message RAM Layout*/

#  ifdef CONFIG_STM32H7_FDCAN1
#    define FDCAN2_STDFILTER_INDEX  FDCAN1_MSGRAM_WORDS
#  else
#    define FDCAN2_STDFILTER_INDEX  0
#  endif
#  define FDCAN2_EXTFILTERS_INDEX   (FDCAN2_STDFILTER_INDEX + FDCAN2_STDFILTER_WORDS)
#  define FDCAN2_RXFIFO0_INDEX      (FDCAN2_EXTFILTERS_INDEX + FDCAN2_EXTFILTER_WORDS)
#  define FDCAN2_RXFIFO1_INDEX      (FDCAN2_RXFIFO0_INDEX + FDCAN2_RXFIFO0_WORDS)
#  define FDCAN2_RXDEDICATED_INDEX  (FDCAN2_RXFIFO1_INDEX + FDCAN2_RXFIFO1_WORDS)
#  define FDCAN2_TXEVENTFIFO_INDEX  (FDCAN2_RXDEDICATED_INDEX + FDCAN2_DEDICATED_RXBUFFER_WORDS)
#  define FDCAN2_TXDEDICATED_INDEX  (FDCAN2_TXEVENTFIFO_INDEX + FDCAN2_TXEVENTFIFO_WORDS)
#  define FDCAN2_TXFIFOQ_INDEX      (FDCAN2_TXDEDICATED_INDEX + FDCAN2_DEDICATED_TXBUFFER_WORDS)
#  define FDCAN2_MSGRAM_WORDS       (FDCAN2_TXFIFOQ_INDEX + FDCAN2_TXFIFIOQ_WORDS)

#  ifdef CONFIG_STM32H7_FDCAN1 
#    if (FDCAN1_MSGRAM_WORDS + FDCAN2_MSGRAM_WORDS) > MSGRAM_MAX_WORDS
#      error FDCAN1 and FDCAN2 config exceeds message RAM word limit
#    endif
#  else
#    if FDCAN2_MSGRAM_WORDS > MSGRAM_MAX_WORDS
#      error FDCAN2 config exceeds message RAM word limit
#    endif
#  endif

#endif /* CONFIG_STM32H7_FDCAN2 */

/* Loopback mode */

#undef STM32H7_FDCAN_LOOPBACK
#if defined(CONFIG_STM32H7_FDCAN1_LOOPBACK) || defined(CONFIG_STM32H7_FDCAN2_LOOPBACK)
#  define STM32H7_FDCAN_LOOPBACK 1
#endif

/* Interrupts ***************************************************************/

/* Common interrupts
 *
 *   FDCAN_INT_TSW  - Timestamp Wraparound
 *   FDCAN_INT_MRAF - Message RAM Access Failure
 *   FDCAN_INT_TOO  - Timeout Occurred
 *   FDCAN_INT_ELO  - Error Logging Overflow
 *   FDCAN_INT_EP   - Error Passive
 *   FDCAN_INT_EW   - Warning Status
 *   FDCAN_INT_BO   - Bus_Off Status
 *   FDCAN_INT_WDI  - Watchdog Interrupt
 *   FDCAN_INT_PEA  - Protocol Error in Arbritration Phase
 *   FDCAN_INT_PED  - Protocol Error in Data Phase
 */

#define FDCAN_CMNERR_INTS   (FDCAN_INT_MRAF | FDCAN_INT_TOO | FDCAN_INT_EP | \
                            FDCAN_INT_BO | FDCAN_INT_WDI | FDCAN_INT_PEA | \
                            FDCAN_INT_PED)
#define FDCAN_COMMON_INTS   FDCAN_CMNERR_INTS

/* RXFIFO mode interrupts
 *
 *   FDCAN_INT_RF0N - Receive FIFO 0 New Message
 *   FDCAN_INT_RF0W - Receive FIFO 0 Watermark Reached
 *   FDCAN_INT_RF0F - Receive FIFO 0 Full
 *   FDCAN_INT_RF0L - Receive FIFO 0 Message Lost
 *   FDCAN_INT_RF1N - Receive FIFO 1 New Message
 *   FDCAN_INT_RF1W - Receive FIFO 1 Watermark Reached
 *   FDCAN_INT_RF1F - Receive FIFO 1 Full
 *   FDCAN_INT_RF1L - Receive FIFO 1 Message Lost
 *   FDCAN_INT_HPM  - High Priority Message Received
 *
 * Dedicated RX Buffer mode interrupts
 *
 *   FDCAN_INT_DRX  - Message stored to Dedicated Receive Buffer
 */

#define FDCAN_RXCOMMON_INTS  0
#define FDCAN_RXFIFO0_INTS  (FDCAN_INT_RF0N | FDCAN_INT_RF0W | FDCAN_INT_RF0L)
#define FDCAN_RXFIFO1_INTS  (FDCAN_INT_RF1N | FDCAN_INT_RF1W | FDCAN_INT_RF1L)
#define FDCAN_RXFIFO_INTS   (FDCAN_RXFIFO0_INTS | FDCAN_RXFIFO1_INTS | \
                            FDCAN_INT_HPM | FDCAN_RXCOMMON_INTS)
#define FDCAN_RXDEDBUF_INTS (FDCAN_INT_DRX | FDCAN_RXCOMMON_INTS)

#define FDCAN_RXERR_INTS    (FDCAN_INT_RF0L | FDCAN_INT_RF1L)

/* TX FIFOQ mode interrupts
 *
 *   FDCAN_INT_TFE  - Tx FIFO Empty
 *
 * TX Event FIFO interrupts
 *
 *   FDCAN_INT_TEFN - Tx Event FIFO New Entry
 *   FDCAN_INT_TEFW - Tx Event FIFO Watermark Reached
 *   FDCAN_INT_TEFF - Tx Event FIFO Full
 *   FDCAN_INT_TEFL - Tx Event FIFO Element Lost
 *
 * Mode-independent TX-related interrupts
 *
 *   FDCAN_INT_TC   - Transmission Completed
 *   FDCAN_INT_TCF  - Transmission Cancellation Finished
 */

#define FDCAN_TXCOMMON_INTS (FDCAN_INT_TC | FDCAN_INT_TCF)
#define FDCAN_TXFIFOQ_INTS  (FDCAN_INT_TFE | FDCAN_TXCOMMON_INTS)
#define FDCAN_TXEVFIFO_INTS (FDCAN_INT_TEFN | FDCAN_INT_TEFW | FDCAN_INT_TEFF | \
                            FDCAN_INT_TEFL)
#define FDCAN_TXDEDBUF_INTS FDCAN_TXCOMMON_INTS

#define FDCAN_TXERR_INTS    (FDCAN_INT_TEFL | FDCAN_INT_PEA | FDCAN_INT_PED)

/* Common-, TX- and RX-Error-Mask */

#define FDCAN_ANYERR_INTS (FDCAN_CMNERR_INTS | FDCAN_RXERR_INTS | FDCAN_TXERR_INTS)

/* Convenience macro for clearing all interrupts */

#define FDCAN_INT_ALL     0x3fcfffff

/* Debug ********************************************************************/

/* Debug configurations that may be enabled just for testing FDCAN */

#ifndef CONFIG_DEBUG_CAN_INFO
#  undef CONFIG_STM32H7_FDCAN_REGDEBUG
#endif

#ifdef CONFIG_STM32H7_FDCAN_REGDEBUG
#  define reginfo caninfo
#else
#  define reginfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CAN frame format */

enum stm32_frameformat_e
{
  FDCAN_ISO11898_1_FORMAT        = 0,  /* Frame format according to ISO11898-1 */
  FDCAN_NONISO_BOSCH_V1_FORMAT   = 1   /* Frame format according to Bosch CAN FD V1.0 */
};

/* CAN mode of operation */

enum stm32_canmode_e
{
  FDCAN_CLASSIC_MODE    = 0, /* Classic CAN operation */
  FDCAN_FD_MODE         = 1, /* CAN FD operation */
  FDCAN_FD_BRS_MODE     = 2  /* CAN FD operation with bit rate switching */
};

/* CAN driver state */

enum can_state_s
{
  FDCAN_STATE_UNINIT = 0,    /* Not yet initialized */
  FDCAN_STATE_RESET,         /* Initialized, reset state */
  FDCAN_STATE_SETUP,         /* fdcan_setup() has been called */
  FDCAN_STATE_DISABLED       /* Disabled by a fdcan_shutdown() */
};

/* This structure describes the FDCAN message RAM layout */

struct stm32_msgram_s
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

/* This structure provides the constant configuration of a FDCAN peripheral */

struct stm32_config_s
{
  uint32_t rxpinset;        /* RX pin configuration */
  uint32_t txpinset;        /* TX pin configuration */
  uintptr_t base;           /* Base address of the FDCAN registers */
  uint32_t baud;            /* Configured baud */
  uint32_t nbtp;            /* Nominal bit timing/prescaler register setting */
  uint32_t dbtp;            /* Data bit timing/prescaler register setting */
  uint8_t port;             /* FDCAN port number (1 or 2) */
  uint8_t irq0;             /* FDCAN peripheral IRQ number for interrupt line 0 */
  uint8_t irq1;             /* FDCAN peripheral IRQ number for interrupt line 1 */
  uint8_t mode;             /* See enum stm32_canmode_e */
  uint8_t format;           /* See enum stm32_frameformat_e */
  uint8_t nstdfilters;      /* Number of standard filters (up to 128) */
  uint8_t nextfilters;      /* Number of extended filters (up to 64) */
  uint8_t nrxfifo0;         /* Number of RX FIFO0 elements (up to 64) */
  uint8_t nrxfifo1;         /* Number of RX FIFO1 elements (up to 64) */
  uint8_t nrxdedicated;     /* Number of dedicated RX buffers (up to 64) */
  uint8_t ntxeventfifo;     /* Number of TXevent FIFO elements (up to 32) */
  uint8_t ntxdedicated;     /* Number of dedicated TX buffers (up to 64) */
  uint8_t ntxfifoq;         /* Number of TX FIFO queue elements (up to 32) */
  uint16_t stdfiltersstart; /* Start word of standard filters */
  uint16_t extfiltersstart; /* Start word of extended filters */
  uint16_t rxfifo0start;    /* Start word of RX FIFO 0 */
  uint16_t rxfifo1start;    /* Start word of RX FIFO 1 */
  uint16_t rxbufferstart;   /* Start word of RX dedicated buffers */
  uint16_t txeventstart;    /* Start word of TX event FIFO */
  uint16_t txbufferstart;   /* Start word of TX dedicated buffers */
  uint16_t txfifoqstart;    /* Start word of TX FIFO/Queue */
  uint8_t rxfifo0ecode;     /* Encoded RX FIFO0 element size */
  uint8_t rxfifo0esize;     /* RX FIFO0 element size (words) */
  uint8_t rxfifo1ecode;     /* Encoded RX FIFO1 element size */
  uint8_t rxfifo1esize;     /* RX FIFO1 element size (words) */
  uint8_t rxbufferecode;    /* Encoded RX buffer element size */
  uint8_t rxbufferesize;    /* RX buffer element size (words) */
  uint8_t txbufferecode;    /* Encoded TX buffer element size */
  uint8_t txbufferesize;    /* TX buffer element size (words) */
#ifdef STM32H7_FDCAN_LOOPBACK
  bool loopback;            /* True: Loopback mode */
#endif

  /* FDCAN message RAM layout */

  struct stm32_msgram_s msgram;
};

/* This structure provides the current state of a FDCAN peripheral */

struct stm32_fdcan_s
{
  /* The constant configuration */

  const struct stm32_config_s *config;

  uint8_t state;            /* See enum can_state_s */
#ifdef CONFIG_CAN_EXTID
  uint8_t nextalloc;        /* Number of allocated extended filters */
#endif
  uint8_t nstdalloc;        /* Number of allocated standard filters */
  sem_t locksem;            /* Enforces mutually exclusive access */
  sem_t txfsem;             /* Used to wait for TX FIFO availability */
  uint32_t nbtp;            /* Current nominal bit timing */
  uint32_t dbtp;            /* Current data bit timing */
  uint32_t rxints;          /* Configured RX interrupts */
  uint32_t txints;          /* Configured TX interrupts */

#ifdef CONFIG_CAN_EXTID
  uint32_t extfilters[2];   /* Extended filter bit allocator.  2*32=64 */
#endif
  uint32_t stdfilters[4];   /* Standard filter bit allocator.  4*32=128 */

#ifdef CONFIG_STM32H7_FDCAN_REGDEBUG
  uintptr_t regaddr;        /* Last register address read */
  uint32_t regval;          /* Last value read from the register */
  unsigned int count;       /* Number of times that the value was read */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* FDCAN Register access */

static uint32_t fdcan_getreg(FAR struct stm32_fdcan_s *priv, int offset);
static void fdcan_putreg(FAR struct stm32_fdcan_s *priv, int offset,
              uint32_t regval);
#ifdef CONFIG_STM32H7_FDCAN_REGDEBUG
static void fdcan_dumpregs(FAR struct stm32_fdcan_s *priv, FAR const char *msg);
static void fdcan_dumprxregs(FAR struct stm32_fdcan_s *priv, FAR const char *msg);
static void fdcan_dumptxregs(FAR struct stm32_fdcan_s *priv, FAR const char *msg);
static void fdcan_dumpramlayout(FAR struct stm32_fdcan_s *priv);
#else
#  define fdcan_dumpregs(priv,msg)
#  define fdcan_dumprxregs(priv,msg)
#  define fdcan_dumptxregs(priv,msg)
#  define fdcan_dumpramlayout(priv)
#endif

/* Semaphore helpers */

static int fdcan_dev_lock(FAR struct stm32_fdcan_s *priv);
static int fdcan_dev_lock_noncancelable(FAR struct stm32_fdcan_s *priv);
#define fdcan_dev_unlock(priv) nxsem_post(&priv->locksem)

static int fdcan_buffer_reserve(FAR struct stm32_fdcan_s *priv);
static void fdcan_buffer_release(FAR struct stm32_fdcan_s *priv);

/* FDCAN helpers */

static uint8_t fdcan_dlc2bytes(FAR struct stm32_fdcan_s *priv, uint8_t dlc);
#if 0 /* Not used */
static uint8_t fdcan_bytes2dlc(FAR struct stm32_fdcan_s *priv, uint8_t nbytes);
#endif

#ifdef CONFIG_CAN_EXTID
static int fdcan_add_extfilter(FAR struct stm32_fdcan_s *priv,
              FAR struct canioc_extfilter_s *extconfig);
static int fdcan_del_extfilter(FAR struct stm32_fdcan_s *priv, int ndx);
#endif
static int fdcan_add_stdfilter(FAR struct stm32_fdcan_s *priv,
              FAR struct canioc_stdfilter_s *stdconfig);
static int fdcan_del_stdfilter(FAR struct stm32_fdcan_s *priv, int ndx);

/* CAN driver methods */

static void fdcan_reset(FAR struct can_dev_s *dev);
static int  fdcan_setup(FAR struct can_dev_s *dev);
static void fdcan_shutdown(FAR struct can_dev_s *dev);
static void fdcan_rxint(FAR struct can_dev_s *dev, bool enable);
static void fdcan_txint(FAR struct can_dev_s *dev, bool enable);
static int  fdcan_ioctl(FAR struct can_dev_s *dev, int cmd,
              unsigned long arg);
static int  fdcan_remoterequest(FAR struct can_dev_s *dev, uint16_t id);
static int  fdcan_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg);
static bool fdcan_txready(FAR struct can_dev_s *dev);
static bool fdcan_txempty(FAR struct can_dev_s *dev);

/* FDCAN interrupt handling */

#if 0 /* Not Used */
static bool fdcan_dedicated_rxbuffer_available(FAR struct stm32_fdcan_s *priv,
              int bufndx);
#endif
#ifdef CONFIG_CAN_ERRORS
static void fdcan_error(FAR struct can_dev_s *dev, uint32_t status);
#endif
static void fdcan_receive(FAR struct can_dev_s *dev,
              FAR uint32_t *rxbuffer, unsigned long nwords);
static int  fdcan_interrupt(int irq, void *context, FAR void *arg);

/* Hardware initialization */

static int  fdcan_hw_initialize(FAR struct stm32_fdcan_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_fdcanops =
{
  .co_reset         = fdcan_reset,
  .co_setup         = fdcan_setup,
  .co_shutdown      = fdcan_shutdown,
  .co_rxint         = fdcan_rxint,
  .co_txint         = fdcan_txint,
  .co_ioctl         = fdcan_ioctl,
  .co_remoterequest = fdcan_remoterequest,
  .co_send          = fdcan_send,
  .co_txready       = fdcan_txready,
  .co_txempty       = fdcan_txempty,
};

#ifdef CONFIG_STM32H7_FDCAN1
/* Message RAM allocation */

// static uint32_t g_fdcan1_msgram[FDCAN1_MSGRAM_WORDS]
// #ifdef CONFIG_ARMV7M_DCACHE
//   __attribute__((aligned(FDCAN_ALIGN)));
// #else
//   ;
// #endif

/* Constant configuration */

static const struct stm32_config_s g_fdcan1const =
{
  .rxpinset         = GPIO_FDCAN1_RX,
  .txpinset         = GPIO_FDCAN1_TX,
  .base             = STM32_FDCAN1_BASE,
  .baud             = CONFIG_STM32H7_FDCAN1_BITRATE,
  .nbtp             = FDCAN_NBTP_NBRP(FDCAN1_NBRP) |
                      FDCAN_NBTP_NTSEG1(FDCAN1_NTSEG1) |
                      FDCAN_NBTP_NTSEG2(FDCAN1_NTSEG2) |
                      FDCAN_NBTP_NSJW(FDCAN1_NSJW),
  .dbtp             = FDCAN_DBTP_DBRP(FDCAN1_DBRP) |
                      FDCAN_DBTP_DTSEG1(FDCAN1_DTSEG1) |
                      FDCAN_DBTP_DTSEG2(FDCAN1_DTSEG2) |
                      FDCAN_DBTP_DSJW(FDCAN1_DSJW),
  .port             = 1,
  .irq0             = STM32_IRQ_FDCAN1_0,
  .irq1             = STM32_IRQ_FDCAN1_1,
#if defined(CONFIG_STM32H7_FDCAN1_CLASSIC)
  .mode             = FDCAN_CLASSIC_MODE,
#elif defined(CONFIG_STM32H7_FDCAN1_FD)
  .mode             = FDCAN_FD_MODE,
#else /* if defined(STM32H7_FDCAN1_ISO11898_1) */
  .mode             = FDCAN_FD_BRS_MODE,
#endif
#if defined(CONFIG_STM32H7_FDCAN1_NONISO_FORMAT)
  .format           = FDCAN_NONISO_BOSCH_V1_FORMAT,
#else
  .format           = FDCAN_ISO11898_1_FORMAT,
#endif
  .nstdfilters      = CONFIG_STM32H7_FDCAN1_NSTDFILTERS,
  .nextfilters      = CONFIG_STM32H7_FDCAN1_NEXTFILTERS,
  .nrxfifo0         = CONFIG_STM32H7_FDCAN1_RXFIFO0_SIZE,
  .nrxfifo1         = CONFIG_STM32H7_FDCAN1_RXFIFO1_SIZE,
  .nrxdedicated     = CONFIG_STM32H7_FDCAN1_DEDICATED_RXBUFFER_SIZE,
  .ntxeventfifo     = CONFIG_STM32H7_FDCAN1_TXEVENTFIFO_SIZE,
  .ntxdedicated     = CONFIG_STM32H7_FDCAN1_DEDICATED_TXBUFFER_SIZE,
  .ntxfifoq         = CONFIG_STM32H7_FDCAN1_TXFIFOQ_SIZE,
  .stdfiltersstart  = FDCAN1_STDFILTER_INDEX,
  .extfiltersstart  = FDCAN1_EXTFILTERS_INDEX,
  .rxfifo0start     = FDCAN1_RXFIFO0_INDEX,
  .rxfifo1start     = FDCAN1_RXFIFO1_INDEX,
  .rxbufferstart    = FDCAN1_RXDEDICATED_INDEX,
  .txeventstart     = FDCAN1_TXEVENTFIFO_INDEX,
  .txbufferstart    = FDCAN1_TXDEDICATED_INDEX,
  .txfifoqstart     = FDCAN1_TXFIFOQ_INDEX,
  .rxfifo0ecode     = FDCAN1_RXFIFO0_ENCODED_SIZE,
  .rxfifo0esize     = (FDCAN1_RXFIFO0_ELEMENT_SIZE / 4) + 2,
  .rxfifo1ecode     = FDCAN1_RXFIFO1_ENCODED_SIZE,
  .rxfifo1esize     = (FDCAN1_RXFIFO1_ELEMENT_SIZE / 4) + 2,
  .rxbufferecode    = FDCAN1_RXBUFFER_ENCODED_SIZE,
  .rxbufferesize    = (FDCAN1_RXBUFFER_ELEMENT_SIZE / 4) + 2,
  .txbufferecode    = FDCAN1_TXBUFFER_ENCODED_SIZE,
  .txbufferesize    = (FDCAN1_TXBUFFER_ELEMENT_SIZE / 4) + 2,

#ifdef CONFIG_STM32H7_FDCAN1_LOOPBACK
  .loopback         = true,
#endif

  /* FDCAN1 Message RAM */

  .msgram =
  {
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN1_STDFILTER_INDEX << 2)),
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN1_EXTFILTERS_INDEX << 2)),
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN1_RXFIFO0_INDEX << 2)),
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN1_RXFIFO1_INDEX << 2)),
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN1_RXDEDICATED_INDEX << 2)),
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN1_TXEVENTFIFO_INDEX << 2)),
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN1_TXDEDICATED_INDEX << 2)),
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN1_TXFIFOQ_INDEX << 2))
  }

  // .msgram =
  // {
  //    &g_fdcan1_msgram[FDCAN1_STDFILTER_INDEX],
  //    &g_fdcan1_msgram[FDCAN1_EXTFILTERS_INDEX],
  //    &g_fdcan1_msgram[FDCAN1_RXFIFO0_INDEX],
  //    &g_fdcan1_msgram[FDCAN1_RXFIFO1_INDEX],
  //    &g_fdcan1_msgram[FDCAN1_RXDEDICATED_INDEX],
  //    &g_fdcan1_msgram[FDCAN1_TXEVENTFIFO_INDEX],
  //    &g_fdcan1_msgram[FDCAN1_TXDEDICATED_INDEX],
  //    &g_fdcan1_msgram[FDCAN1_TXFIFOQ_INDEX]
  // }
};

/* FDCAN1 variable driver state */

static struct stm32_fdcan_s g_fdcan1priv;
static struct can_dev_s g_fdcan1dev;

#endif /* CONFIG_STM32H7_FDCAN1 */

#ifdef CONFIG_STM32H7_FDCAN2
/* FDCAN2 message RAM allocation */

// static uint32_t g_fdcan2_msgram[FDCAN2_MSGRAM_WORDS]
// #ifdef CONFIG_ARMV7M_DCACHE
//   __attribute__((aligned(FDCAN_ALIGN)));
// #else
//   ;
// #endif

/* FDCAN2 constant configuration */

static const struct stm32_config_s g_fdcan2const =
{
  .rxpinset         = GPIO_FDCAN2_RX,
  .txpinset         = GPIO_FDCAN2_TX,
  .base             = STM32_FDCAN2_BASE,
  .baud             = CONFIG_STM32H7_FDCAN2_BITRATE,
  .nbtp             = FDCAN_NBTP_NBRP(FDCAN2_NBRP) |
                      FDCAN_NBTP_NTSEG1(FDCAN2_NTSEG1) |
                      FDCAN_NBTP_NTSEG2(FDCAN2_NTSEG2) |
                      FDCAN_NBTP_NSJW(FDCAN2_NSJW),
  .dbtp             = FDCAN_DBTP_DBRP(FDCAN2_DBRP) |
                      FDCAN_DBTP_DTSEG1(FDCAN2_DTSEG1) |
                      FDCAN_DBTP_DTSEG2(FDCAN2_DTSEG2) |
                      FDCAN_DBTP_DSJW(FDCAN2_DSJW),
  .port             = 2,
  .irq0             = STM32_IRQ_FDCAN2_0,
  .irq1             = STM32_IRQ_FDCAN2_1,
#if defined(CONFIG_STM32H7_FDCAN2_CLASSIC)
  .mode             = FDCAN_CLASSIC_MODE,
#elif defined(CONFIG_STM32H7_FDCAN2_FD)
  .mode             = FDCAN_FD_MODE,
#else /* if defined(STM32H7_FDCAN2_ISO11898_1) */
  .mode             = FDCAN_FD_BRS_MODE,
#endif
#if defined(CONFIG_STM32H7_FDCAN2_NONISO_FORMAT)
  .format           = FDCAN_NONISO_BOSCH_V1_FORMAT,
#else
  .format           = FDCAN_ISO11898_1_FORMAT,
#endif
  .nstdfilters      = CONFIG_STM32H7_FDCAN2_NSTDFILTERS,
  .nextfilters      = CONFIG_STM32H7_FDCAN2_NEXTFILTERS,
  .nrxfifo0         = CONFIG_STM32H7_FDCAN2_RXFIFO0_SIZE,
  .nrxfifo1         = CONFIG_STM32H7_FDCAN2_RXFIFO1_SIZE,
  .nrxdedicated     = CONFIG_STM32H7_FDCAN2_DEDICATED_RXBUFFER_SIZE,
  .ntxeventfifo     = CONFIG_STM32H7_FDCAN2_TXEVENTFIFO_SIZE,
  .ntxdedicated     = CONFIG_STM32H7_FDCAN2_DEDICATED_TXBUFFER_SIZE,
  .ntxfifoq         = CONFIG_STM32H7_FDCAN2_TXFIFOQ_SIZE,
  .stdfiltersstart  = FDCAN2_STDFILTER_INDEX,
  .extfiltersstart  = FDCAN2_EXTFILTERS_INDEX,
  .rxfifo0start     = FDCAN2_RXFIFO0_INDEX,
  .rxfifo1start     = FDCAN2_RXFIFO1_INDEX,
  .rxbufferstart    = FDCAN2_RXDEDICATED_INDEX,
  .txeventstart     = FDCAN2_TXEVENTFIFO_INDEX,
  .txbufferstart    = FDCAN2_TXDEDICATED_INDEX,
  .txfifoqstart     = FDCAN2_TXFIFOQ_INDEX,
  .rxfifo0ecode     = FDCAN2_RXFIFO0_ENCODED_SIZE,
  .rxfifo0esize     = (FDCAN2_RXFIFO0_ELEMENT_SIZE / 4) + 2,
  .rxfifo1ecode     = FDCAN2_RXFIFO1_ENCODED_SIZE,
  .rxfifo1esize     = (FDCAN2_RXFIFO1_ELEMENT_SIZE / 4) + 2,
  .rxbufferecode    = FDCAN2_RXBUFFER_ENCODED_SIZE,
  .rxbufferesize    = (FDCAN2_RXBUFFER_ELEMENT_SIZE / 4) + 2,
  .txbufferecode    = FDCAN2_TXBUFFER_ENCODED_SIZE,
  .txbufferesize    = (FDCAN2_TXBUFFER_ELEMENT_SIZE / 4) + 2,

#ifdef CONFIG_STM32H7_FDCAN2_LOOPBACK
  .loopback         = true,
#endif

  /* FDCAN2 Message RAM */

  .msgram =
  {
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN2_STDFILTER_INDEX << 2)),
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN2_EXTFILTERS_INDEX << 2)),
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN2_RXFIFO0_INDEX << 2)),
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN2_RXFIFO1_INDEX << 2)),
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN2_RXDEDICATED_INDEX << 2)),
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN2_TXEVENTFIFO_INDEX << 2)),
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN2_TXDEDICATED_INDEX << 2)),
    (uint32_t*)(STM32_CANRAM_BASE + (FDCAN2_TXFIFOQ_INDEX << 2))
  }

  // .msgram =
  // {
  //    &g_FDCAN2_msgram[FDCAN2_STDFILTER_INDEX],
  //    &g_FDCAN2_msgram[FDCAN2_EXTFILTERS_INDEX],
  //    &g_FDCAN2_msgram[FDCAN2_RXFIFO0_INDEX],
  //    &g_FDCAN2_msgram[FDCAN2_RXFIFO1_INDEX],
  //    &g_FDCAN2_msgram[FDCAN2_RXDEDICATED_INDEX],
  //    &g_FDCAN2_msgram[FDCAN2_TXEVENTFIFO_INDEX],
  //    &g_FDCAN2_msgram[FDCAN2_TXDEDICATED_INDEX],
  //    &g_FDCAN2_msgram[FDCAN2_TXFIFOQ_INDEX]
  // }
};

/* FDCAN2 variable driver state */

static struct stm32_fdcan_s g_fdcan2priv;
static struct can_dev_s g_fdcan2dev;

#endif /* CONFIG_STM32H7_FDCAN2 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdcan_getreg
 *
 * Description:
 *   Read the value of a FDCAN register.
 *
 * Input Parameters:
 *   priv - A reference to the FDCAN peripheral state
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_FDCAN_REGDEBUG
static uint32_t fdcan_getreg(FAR struct stm32_fdcan_s *priv, int offset)
{
  FAR const struct stm32_config_s *config = priv->config;
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
static uint32_t fdcan_getreg(FAR struct stm32_fdcan_s *priv, int offset)
{
  FAR const struct stm32_config_s *config = priv->config;
  return getreg32(config->base + offset);
}

#endif

/****************************************************************************
 * Name: fdcan_putreg
 *
 * Description:
 *   Set the value of a FDCAN register.
 *
 * Input Parameters:
 *   priv - A reference to the FDCAN peripheral state
 *   offset - The offset to the register to write
 *   regval - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_FDCAN_REGDEBUG
static void fdcan_putreg(FAR struct stm32_fdcan_s *priv, int offset,
                        uint32_t regval)
{
  FAR const struct stm32_config_s *config = priv->config;
  uintptr_t regaddr = config->base + offset;

  /* Show the register value being written */

  caninfo("%08x<-%08x\n", regaddr, regval);

  /* Write the value */

  putreg32(regval, regaddr);
}

#else
static void fdcan_putreg(FAR struct stm32_fdcan_s *priv, int offset,
                        uint32_t regval)
{
  FAR const struct stm32_config_s *config = priv->config;
  putreg32(regval, config->base + offset);
}

#endif

/****************************************************************************
 * Name: fdcan_dumpctrlregs
 *
 * Description:
 *   Dump the contents of all CAN control registers
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_FDCAN_REGDEBUG
static void fdcan_dumpregs(FAR struct stm32_fdcan_s *priv,
                                  FAR const char *msg)
{  
  FAR const struct stm32_config_s *config = priv->config;

  caninfo("CAN%d Control and Status Registers: %s\n", config->port, msg);
  caninfo("  Base:  %08x\n", config->base);

  /* CAN control and status registers */

  caninfo("  CCCR:  %08x   TEST:  %08x\n",
          getreg32(config->base + STM32_FDCAN_CCCR_OFFSET),
          getreg32(config->base + STM32_FDCAN_TEST_OFFSET));

  caninfo("  NBTP:  %08x   DBTP:  %08x\n",
          getreg32(config->base + STM32_FDCAN_NBTP_OFFSET),
          getreg32(config->base + STM32_FDCAN_DBTP_OFFSET));

  caninfo("  IE:    %08x   TIE:   %08x\n",
          getreg32(config->base + STM32_FDCAN_IE_OFFSET),
          getreg32(config->base + STM32_FDCAN_TXBTIE_OFFSET));
  
  caninfo("  ILE:   %08x   ILS:   %08x\n",
          getreg32(config->base + STM32_FDCAN_ILE_OFFSET),
          getreg32(config->base + STM32_FDCAN_ILS_OFFSET));

  caninfo("  RXBC:  %08x   RXESC: %08x\n",
          getreg32(config->base + STM32_FDCAN_RXBC_OFFSET),
          getreg32(config->base + STM32_FDCAN_RXESC_OFFSET));

  caninfo("  RXF0C: %08x   RXF1C: %08x\n",
          getreg32(config->base + STM32_FDCAN_RXF0C_OFFSET),
          getreg32(config->base + STM32_FDCAN_RXF1C_OFFSET));

  caninfo("  TXBC:  %08x   TXESC: %08x   TXEFC: %08x\n",
          getreg32(config->base + STM32_FDCAN_TXBC_OFFSET),
          getreg32(config->base + STM32_FDCAN_TXESC_OFFSET),
          getreg32(config->base + STM32_FDCAN_TXEFC_OFFSET));
}
#endif

/****************************************************************************
 * Name: stm32can_dumprxregs
 *
 * Description:
 *   Dump the contents of all Rx status registers
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_FDCAN_REGDEBUG
static void fdcan_dumprxregs(FAR struct stm32_fdcan_s *priv,
                                FAR const char *msg)
{
  FAR const struct stm32_config_s *config = priv->config;

  caninfo("CAN%d Rx Registers: %s\n", config->port, msg);
  caninfo("  Base:  %08x\n", config->base);

  caninfo("  PSR:   %08x   ECR:   %08x   HPMS: %08x\n",
          getreg32(config->base + STM32_FDCAN_PSR_OFFSET),
          getreg32(config->base + STM32_FDCAN_ECR_OFFSET),
          getreg32(config->base + STM32_FDCAN_HPMS_OFFSET));

  caninfo("  RXF0S: %08x   RXF0A: %08x\n",
          getreg32(config->base + STM32_FDCAN_RXF0S_OFFSET),
          getreg32(config->base + STM32_FDCAN_RXF0A_OFFSET));

  caninfo("  RXF1S: %08x   RXF1A: %08x\n",
          getreg32(config->base + STM32_FDCAN_RXF1S_OFFSET),
          getreg32(config->base + STM32_FDCAN_RXF1A_OFFSET));
  
  caninfo("  NDAT1: %08x   NDAT2: %08x\n",
          getreg32(config->base + STM32_FDCAN_NDAT1_OFFSET),
          getreg32(config->base + STM32_FDCAN_NDAT2_OFFSET));

  caninfo("  IR:    %08x   IE:    %08x\n",
          getreg32(config->base + STM32_FDCAN_IR_OFFSET),
          getreg32(config->base + STM32_FDCAN_IE_OFFSET));
}
#endif

/****************************************************************************
 * Name: stm32can_dumptxregs
 *
 * Description:
 *   Dump the contents of all Tx buffer registers
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_FDCAN_REGDEBUG
static void fdcan_dumptxregs(FAR struct stm32_fdcan_s *priv,
                                  FAR const char *msg)
{
  FAR const struct stm32_config_s *config = priv->config;

  caninfo("CAN%d Tx Registers: %s\n", config->port, msg);
  caninfo("  Base:  %08x\n", config->base);

  caninfo("  PSR:   %08x   ECR:   %08x\n",
          getreg32(config->base + STM32_FDCAN_PSR_OFFSET),
          getreg32(config->base + STM32_FDCAN_ECR_OFFSET));

  caninfo("  TXQFS: %08x   TXBAR: %08x   TXBRP: %08x\n",
          getreg32(config->base + STM32_FDCAN_TXFQS_OFFSET),
          getreg32(config->base + STM32_FDCAN_TXBAR_OFFSET),
          getreg32(config->base + STM32_FDCAN_TXBRP_OFFSET));
  
  caninfo("  TXBTO: %08x   TXBCR: %08x   TXBCF: %08x\n",
          getreg32(config->base + STM32_FDCAN_TXBTO_OFFSET),
          getreg32(config->base + STM32_FDCAN_TXBCR_OFFSET),
          getreg32(config->base + STM32_FDCAN_TXBCF_OFFSET));

  caninfo("  TXEFC: %08x   TXEFS: %08x   TXEFA: %08x\n",
          getreg32(config->base + STM32_FDCAN_TXEFC_OFFSET),
          getreg32(config->base + STM32_FDCAN_TXEFS_OFFSET),
          getreg32(config->base + STM32_FDCAN_TXEFA_OFFSET));

  caninfo("  IR:    %08x   IE:    %08x   TIE:   %08x\n",
          getreg32(config->base + STM32_FDCAN_IR_OFFSET),
          getreg32(config->base + STM32_FDCAN_IE_OFFSET),
          getreg32(config->base + STM32_FDCAN_TXBTIE_OFFSET));
}
#endif

/****************************************************************************
 * Name: stm32can_dumpramlayout
 *
 * Description:
 *   Print the layout of the message RAM
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_FDCAN_REGDEBUG
static void fdcan_dumpramlayout(FAR struct stm32_fdcan_s *priv)
{
  FAR const struct stm32_config_s *config = priv->config;

  caninfo(" ******* FDCAN%d Message RAM layout *******\n", config->port);
  caninfo("                Start     # Elmnt  Elmnt size (words)\n");

  if(config->nstdfilters > 0)
    {
      caninfo("STD filters   %p   %4d        %2d\n",
              config->msgram.stdfilters,
              config->nstdfilters,
              1);
    }

  if(config->nextfilters)
    {
      caninfo("EXT filters   %p   %4d        %2d\n",
              config->msgram.extfilters,
              config->nextfilters,
              2);
    }

  if(config->nrxfifo0)
    {
      caninfo("RX FIFO 0     %p   %4d        %2d\n",
              config->msgram.rxfifo0,
              config->nrxfifo0,
              config->rxfifo0esize);
    }

  if(config->nrxfifo1)
    {
      caninfo("RX FIFO 1     %p   %4d        %2d\n",
              config->msgram.rxfifo1,
              config->nrxfifo1,
              config->rxfifo1esize);
    }

  if(config->nrxdedicated)
    {
      caninfo("RX Buffers    %p   %4d        %2d\n",
              config->msgram.rxdedicated,
              config->nrxdedicated,
              config->rxbufferesize);
    }

  if(config->ntxfifoq)
    {
      caninfo("TX FIFO       %p   %4d        %2d\n",
              config->msgram.txfifoq,
              config->ntxfifoq,
              config->txbufferesize);
    }
}
#endif

/****************************************************************************
 * Name: fdcan_dev_lock
 *
 * Description:
 *   Take the semaphore that enforces mutually exclusive access to device
 *   structures, handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the FDCAN peripheral state
 *
 * Returned Value:
 *  Normally success (OK) is returned, but the error -ECANCELED may be
 *  return in the event that task has been canceled.
 *
 ****************************************************************************/

static int fdcan_dev_lock(FAR struct stm32_fdcan_s *priv)
{
  return nxsem_wait_uninterruptible(&priv->locksem);
}

/****************************************************************************
 * Name: fdcan_dev_lock_noncancelable
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.  This version also
 *   ignores attempts to cancel the thread.
 *
 ****************************************************************************/

static int fdcan_dev_lock_noncancelable(FAR struct stm32_fdcan_s *priv)
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
 * Name: fdcan_buffer_reserve
 *
 * Description:
 *   Take the semaphore, decrementing the semaphore count to indicate that
 *   one fewer TX FIFOQ buffer is available.  Handles any exceptional
 *   conditions.
 *
 * Input Parameters:
 *   priv - A reference to the FDCAN peripheral state
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  Called only non-interrupt logic via fdcan_write().  We do not have
 *  exclusive access to the FDCAN hardware and interrupts are not disabled.
 *  fdcan_write() does lock the scheduler for reasons noted below.
 *
 ****************************************************************************/

static int fdcan_buffer_reserve(FAR struct stm32_fdcan_s *priv)
{
  irqstate_t flags;
  uint32_t txfqs1;
  uint32_t txfqs2;
#ifndef CONFIG_STM32H7_FDCAN_QUEUE_MODE
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
       *
       * REVISIT: Should we really wait here? In theory the time to wait is
       * small, but it is still wait time nonetheless. If the upper-level
       * driver is operating in non-blocking mode, is this acceptable?
       */

      for (; ; )
        {
          /* Get the current queue status and semaphore count. */

          flags = enter_critical_section();
          txfqs1 = fdcan_getreg(priv, STM32_FDCAN_TXFQS_OFFSET);
          nxsem_get_value(&priv->txfsem, &sval);
          txfqs2 = fdcan_getreg(priv, STM32_FDCAN_TXFQS_OFFSET);

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

#ifdef CONFIG_STM32H7_FDCAN_QUEUE_MODE
      /* We only have one useful bit of information in the TXFQS:
       * Is the TX FIFOQ full or not?  We can only do limited checks
       * with that single bit of information.
       */

      if ((txfqs1 & FDCAN_TXFQS_TFQF) != 0)
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

          return -ENOMEM;
        }

      /* The FIFO is not full so the semaphore count should be greater
       * than zero.  If it is not, then we have missed a call to
       * fdcan_buffer_release(0).
       *
       * NOTE: Since there is no mutual exclusion, it might be possible
       * that fdcan_write() could be re-entered AFTER taking the semaphore
       * and dropping the count to zero, but BEFORE adding the message
       * to the TX FIFOQ.  That corner case is handled in fdcan_write() by
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
          return OK;
        }
#else
      /* Tx FIFO Free Level */

      tffl = (txfqs1 & FDCAN_TXFQS_TFFL_MASK) >> FDCAN_TXFQS_TFFL_SHIFT;

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

      /* The semaphore value is reasonable. See if we can obtain it now. */

      ret = nxsem_trywait(&priv->txfsem);
      leave_critical_section(flags);
      return ret;
    }
  while (1); /* REVIST: I do not like infinite loops */
}

/****************************************************************************
 * Name: fdcan_buffer_release
 *
 * Description:
 *   Release the semaphore, increment the semaphore count to indicate that
 *   one more TX FIFOQ buffer is available.
 *
 * Input Parameters:
 *   priv - A reference to the FDCAN peripheral state
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  This function is called only from the interrupt level in response to the
 *  complete of a transmission.
 *
 ****************************************************************************/

static void fdcan_buffer_release(FAR struct stm32_fdcan_s *priv)
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
 * Name: fdcan_dlc2bytes
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

static uint8_t fdcan_dlc2bytes(FAR struct stm32_fdcan_s *priv, uint8_t dlc)
{
  if (dlc > 8)
    {
#ifdef CONFIG_CAN_FD
      if (priv->config->mode == FDCAN_CLASSIC_MODE)
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
 * Name: fdcan_bytes2dlc
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
static uint8_t fdcan_bytes2dlc(FAR struct stm32_fdcan_s *priv, uint8_t nbytes)
{
  if (nbytes <= 8)
    {
      return nbytes;
    }
#ifdef CONFIG_CAN_FD
  else if (priv->mode == FDCAN_ISO11898_1_MODE)
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
 * Name: fdcan_add_extfilter
 *
 * Description:
 *   Add an address filter for a extended 29 bit address.
 *
 * Input Parameters:
 *   priv      - An instance of the FDCAN driver state structure.
 *   extconfig - The configuration of the extended filter
 *
 * Returned Value:
 *   A non-negative filter ID is returned on success.  Otherwise a negated
 *   errno value is returned to indicate the nature of the error.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static int fdcan_add_extfilter(FAR struct stm32_fdcan_s *priv,
                              FAR struct canioc_extfilter_s *extconfig)
{
  FAR const struct stm32_config_s *config;
  FAR uint32_t *extfilter;
  uint32_t regval;
  int word;
  int bit;
  int ndx;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL && extconfig != NULL);
  config = priv->config;

  /* Get exclusive excess to the FDCAN hardware */

  ret = fdcan_dev_lock(priv);
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

          // up_clean_dcache((uintptr_t)extfilter, (uintptr_t)exfilter + 8);

          /* Is this the first extended filter? */

          if (priv->nextalloc == 1)
            {
              /* Enable the Initialization state */

              regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
              regval |= FDCAN_CCCR_INIT;
              fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

              /* Wait for initialization mode to take effect */

              while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) &
                     FDCAN_CCCR_INIT) == 0)
                {
                }

              /* Enable writing to configuration registers */

              regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
              regval |= (FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
              fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

             /* Update the Global Filter Configuration so that received
              * messages are rejected if they do not match the acceptance
              * filter.
              *
              *   ANFE=2: Discard all rejected frames
              */

              regval  = fdcan_getreg(priv, STM32_FDCAN_GFC_OFFSET);
              regval &= ~FDCAN_GFC_ANFE_MASK;
              regval |= FDCAN_GFC_ANFE_REJECTED;
              fdcan_putreg(priv, STM32_FDCAN_GFC_OFFSET, regval);

              /* Disable writing to configuration registers */

              regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
              regval &= ~(FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
              fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);
            }

          fdcan_dev_unlock(priv);
          return ndx;
        }
    }

  DEBUGASSERT(priv->nextalloc == priv->config->nextfilters);
  fdcan_dev_unlock(priv);
  return -EAGAIN;
}
#endif

/****************************************************************************
 * Name: fdcan_del_extfilter
 *
 * Description:
 *   Remove an address filter for a standard 29 bit address.
 *
 * Input Parameters:
 *   priv - An instance of the FDCAN driver state structure.
 *   ndx  - The filter index previously returned by the fdcan_add_extfilter().
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static int fdcan_del_extfilter(FAR struct stm32_fdcan_s *priv, int ndx)
{
  FAR const struct stm32_config_s *config;
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

  /* Get exclusive excess to the FDCAN hardware */

  ret = fdcan_dev_lock(priv);
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

      fdcan_dev_unlock(priv);
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

      regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
      regval |= FDCAN_CCCR_INIT;
      fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

      /* Wait for initialization mode to take effect */

      while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) &
             FDCAN_CCCR_INIT) == 0)
        {
        }

      /* Enable writing to configuration registers */

      regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
      regval |= (FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
      fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

      /* If there are no extended filters, then modify Global Filter
       * Configuration so that all rejected messages are places in RX
       * FIFO0.
       *
       *   ANFE=0: Store all rejected extended frame in RX FIFO0
       */

      regval  = fdcan_getreg(priv, STM32_FDCAN_GFC_OFFSET);
      regval &= ~FDCAN_GFC_ANFE_MASK;
      regval |= FDCAN_GFC_ANFE_RX_FIFO0;
      fdcan_putreg(priv, STM32_FDCAN_GFC_OFFSET, regval);

      /* Disable writing to configuration registers */

      regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
      regval &= ~(FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
      fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);
    }

  /* Deactivate the filter last so that no messages are lost. */

  extfilter    = config->msgram.extfilters + (ndx << 1);
  *extfilter++ = 0;
  *extfilter   = 0;

  fdcan_dev_unlock(priv);
  return OK;
}
#endif

/****************************************************************************
 * Name: fdcan_add_stdfilter
 *
 * Description:
 *   Add an address filter for a standard 11 bit address.
 *
 * Input Parameters:
 *   priv      - An instance of the FDCAN driver state structure.
 *   stdconfig - The configuration of the standard filter
 *
 * Returned Value:
 *   A non-negative filter ID is returned on success.  Otherwise a negated
 *   errno value is returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int fdcan_add_stdfilter(FAR struct stm32_fdcan_s *priv,
                              FAR struct canioc_stdfilter_s *stdconfig)
{
  FAR const struct stm32_config_s *config;
  FAR uint32_t *stdfilter;
  uint32_t regval;
  int word;
  int bit;
  int ndx;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL);
  config = priv->config;

  /* Get exclusive excess to the FDCAN hardware */

  ret = fdcan_dev_lock(priv);
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

          // up_clean_dcache((uintptr_t)stdfilter, (uintptr_t)stdfilter + 4);

          /* Is this the first standard filter? */

          if (priv->nstdalloc == 1)
            {
              /* Enable the Initialization state */

              regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
              regval |= FDCAN_CCCR_INIT;
              fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

              /* Wait for initialization mode to take effect */

              while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) &
                     FDCAN_CCCR_INIT) == 0)
                {
                }

              /* Enable writing to configuration registers */

              regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
              regval |= (FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
              fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

             /* Update the Global Filter Configuration so that received
              * messages are rejected if they do not match the acceptance
              * filter.
              *
              *   ANFS=2: Discard all rejected frames
              */

              regval  = fdcan_getreg(priv, STM32_FDCAN_GFC_OFFSET);
              regval &= ~FDCAN_GFC_ANFS_MASK;
              regval |= FDCAN_GFC_ANFS_REJECTED;
              fdcan_putreg(priv, STM32_FDCAN_GFC_OFFSET, regval);

              /* Disable writing to configuration registers */

              regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
              regval &= ~(FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
              fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);
            }

          fdcan_dev_unlock(priv);
          return ndx;
        }
    }

  DEBUGASSERT(priv->nstdalloc == priv->config->nstdfilters);
  fdcan_dev_unlock(priv);
  return -EAGAIN;
}

/****************************************************************************
 * Name: fdcan_del_stdfilter
 *
 * Description:
 *   Remove an address filter for a standard 29 bit address.
 *
 * Input Parameters:
 *   priv - An instance of the FDCAN driver state structure.
 *   ndx  - The filter index previously returned by the fdcan_add_stdfilter().
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int fdcan_del_stdfilter(FAR struct stm32_fdcan_s *priv, int ndx)
{
  FAR const struct stm32_config_s *config;
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

  /* Get exclusive excess to the FDCAN hardware */

  ret = fdcan_dev_lock(priv);
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

      fdcan_dev_unlock(priv);
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

      regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
      regval |= FDCAN_CCCR_INIT;
      fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

      /* Wait for initialization mode to take effect */

      while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) &
              FDCAN_CCCR_INIT) == 0)
        {
        }

      /* Enable writing to configuration registers */

      regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
      regval |= (FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
      fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

      /* If there are no standard filters, then modify Global Filter
       * Configuration so that all rejected messages are places in RX
       * FIFO0.
       *
       *   ANFS=0: Store all rejected extended frame in RX FIFO0
       */

      regval  = fdcan_getreg(priv, STM32_FDCAN_GFC_OFFSET);
      regval &= ~FDCAN_GFC_ANFS_MASK;
      regval |= FDCAN_GFC_ANFS_RX_FIFO0;
      fdcan_putreg(priv, STM32_FDCAN_GFC_OFFSET, regval);

      /* Disable writing to configuration registers */

      regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
      regval &= ~(FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
      fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);
    }

  /* Deactivate the filter last so that no messages are lost. */

  stdfilter  = config->msgram.stdfilters + ndx;
  *stdfilter = 0;

  fdcan_dev_unlock(priv);
  return OK;
}

/****************************************************************************
 * Name: fdcan_start_busoff_recovery_sequence
 *
 * Description:
 *   This function initiates the BUS-OFF recovery sequence.
 *   CAN Specification Rev. 2.0 or ISO11898-1:2015
 *   According the SAMV71 datasheet:
 *
 *   "If the device goes Bus_Off, it will set FDCAN_CCCR.INIT of its own
 *    accord, stopping all bus activities. Once FDCAN_CCCR.INIT has been
 *    cleared by the processor (application), the device will then wait for
 *    129 occurrences of Bus Idle (129 * 11 consecutive recessive bits)
 *    before resuming normal operation. At the end of the Bus_Off recovery
 *    sequence, the Error Management Counters will be reset. During the
 *    waiting time after the resetting of FDCAN_CCCR.INIT, each time a
 *    sequence of 11 recessive bits has been monitored, a Bit0 Error code is
 *    written to FDCAN_PSR.LEC, enabling the processor to readily check up
 *    whether the CAN bus is stuck at dominant or continuously disturbed and
 *    to monitor the Bus_Off recovery sequence.  FDCAN_ECR.REC is used to
 *    count these sequences."
 *
 * Input Parameters:
 *   priv - An instance of the FDCAN driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int fdcan_start_busoff_recovery_sequence(FAR struct stm32_fdcan_s *priv)
{
  uint32_t regval;
  int ret;

  DEBUGASSERT(priv);

  /* Get exclusive access to the FDCAN peripheral */

  ret = fdcan_dev_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* only start BUS-OFF recovery if we are in BUS-OFF state */

  regval = fdcan_getreg(priv, STM32_FDCAN_PSR_OFFSET);
  if (!(regval & FDCAN_PSR_BO))
    {
      fdcan_dev_unlock(priv);
      return -EPERM;
    }

  /* Disable initialization mode to issue the recovery sequence */

  regval = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
  regval &= ~FDCAN_CCCR_INIT;
  fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

  fdcan_dev_unlock(priv);
  return OK;
}

/****************************************************************************
 * Name: fdcan_reset
 *
 * Description:
 *   Reset the FDCAN device.  Called early to initialize the hardware. This
 *   function is called, before fdcan_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void fdcan_reset(FAR struct can_dev_s *dev)
{
  FAR struct stm32_fdcan_s *priv;
  FAR const struct stm32_config_s *config;
  uint32_t regval;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  caninfo("FDCAN%d\n", config->port);
  UNUSED(config);

  /* Get exclusive access to the FDCAN peripheral */

  fdcan_dev_lock_noncancelable(priv);

  /* Disable all interrupts */

  fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, 0);
  fdcan_putreg(priv, STM32_FDCAN_TXBTIE_OFFSET, 0);

  /* Make sure that all buffers are released.
   *
   * REVISIT: What if a thread is waiting for a buffer?  The following
   * will not wake up any waiting threads.
   */

  nxsem_destroy(&priv->txfsem);
  nxsem_init(&priv->txfsem, 0, config->ntxfifoq);

  /* Disable the FDCAN controller */
  /* REVISIT: Should fdcan_shutdown() be called here? */

  // fdcan_shutdown(dev);

  /* Reset the FD CAN */
  /* REVISIT:  Since there is only a single reset for both FDCAN
   * controllers, do we really want to use the RCC reset here?
   * This will nuke operation of the second controller if another
   * device is registered.
   */

  regval  = getreg32(STM32_RCC_APB1HRSTR);
  regval |= RCC_APB1HRSTR_FDCANRST;
  putreg32(regval, STM32_RCC_APB1HRSTR);

  regval &= ~RCC_APB1HRSTR_FDCANRST;
  putreg32(regval, STM32_RCC_APB1HRSTR);

  priv->state = FDCAN_STATE_RESET;
  fdcan_dev_unlock(priv);
}

/****************************************************************************
 * Name: fdcan_setup
 *
 * Description:
 *   Configure the FDCAN. This method is called the first time that the FDCAN
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching FDCAN interrupts.
 *   All FDCAN interrupts are disabled upon return.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int fdcan_setup(FAR struct can_dev_s *dev)
{
  FAR struct stm32_fdcan_s *priv;
  FAR const struct stm32_config_s *config;
  int ret;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  caninfo("FDCAN%d\n", config->port);

  /* Get exclusive access to the FDCAN peripheral */

  ret = fdcan_dev_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* FDCAN hardware initialization */

  ret = fdcan_hw_initialize(priv);
  if (ret < 0)
    {
      canerr("ERROR: FDCAN%d H/W initialization failed: %d\n",
             config->port, ret);
      return ret;
    }

  fdcan_dumpregs(priv, "After hardware initialization");

  /* Attach the FDCAN interrupt handlers */

  ret = irq_attach(config->irq0, fdcan_interrupt, dev);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach FDCAN%d line 0 IRQ (%d)",
      config->port, config->irq0);
      return ret;
    }

  ret = irq_attach(config->irq1, fdcan_interrupt, dev);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach FDCAN%d line 1 IRQ (%d)",
      config->port, config->irq1);
      return ret;
    }

  priv->state = FDCAN_STATE_SETUP;

  /* Enable receive interrupts */
  /* REVIST: this is done by can_open() as well. The comment below seems to
   * indicate that receive interrupts should be disabled on leaving here
   */

  /* fdcan_rxint(dev, true); */

  /* Enable the interrupts at the NVIC (they are still disabled at the FDCAN
   * peripheral).
   */

  up_enable_irq(config->irq0);
  up_enable_irq(config->irq1);
  fdcan_dev_unlock(priv);
  return OK;
}

/****************************************************************************
 * Name: fdcan_shutdown
 *
 * Description:
 *   Disable the FDCAN.  This method is called when the FDCAN device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void fdcan_shutdown(FAR struct can_dev_s *dev)
{
  FAR struct stm32_fdcan_s *priv;
  FAR const struct stm32_config_s *config;
  uint32_t regval;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  caninfo("FDCAN%d\n", config->port);

  /* Get exclusive access to the FDCAN peripheral */

  fdcan_dev_lock_noncancelable(priv);

  /* Disable FDCAN interrupts at the NVIC */

  up_disable_irq(config->irq0);
  up_disable_irq(config->irq1);

  /* Disable all interrupts from the FDCAN peripheral */

  fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, 0);
  fdcan_putreg(priv, STM32_FDCAN_TXBTIE_OFFSET, 0);

  /* Detach the FDCAN interrupt handler */

  irq_detach(config->irq0);
  irq_detach(config->irq1);

  /* Disable device by setting the Clock Stop Request bit */

  regval = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
  regval |= FDCAN_CCCR_CSR;
  fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

  /* Wait for Init and Clock Stop Acknowledge bits to verify
   * device is in the powered down state
   */

  while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) & FDCAN_CCCR_INIT) == 0);
  while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) & FDCAN_CCCR_CSA) == 0);
  priv->state = FDCAN_STATE_DISABLED;

  fdcan_dev_unlock(priv);
}

/****************************************************************************
 * Name: fdcan_rxint
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

static void fdcan_rxint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct stm32_fdcan_s *priv = dev->cd_priv;
  irqstate_t flags;
  uint32_t regval;

  DEBUGASSERT(priv && priv->config);

  caninfo("FDCAN%d enable: %d\n", priv->config->port, enable);

  /* Enable/disable the receive interrupts */

  flags = enter_critical_section();
  regval = fdcan_getreg(priv, STM32_FDCAN_IE_OFFSET);

  if (enable)
    {
      regval |= priv->rxints | FDCAN_COMMON_INTS;
    }
  else
    {
      regval &= ~priv->rxints;
    }

  fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, regval);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: fdcan_txint
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

static void fdcan_txint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct stm32_fdcan_s *priv = dev->cd_priv;
  irqstate_t flags;
  uint32_t regval;

  DEBUGASSERT(priv && priv->config);

  caninfo("FDCAN%d enable: %d\n", priv->config->port, enable);

  /* Enable/disable the receive interrupts */

  flags = enter_critical_section();
  regval = fdcan_getreg(priv, STM32_FDCAN_IE_OFFSET);

  if (enable)
    {
      regval |= priv->txints | FDCAN_COMMON_INTS;
    }
  else
    {
      regval &= ~priv->txints;
    }

  fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, regval);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: fdcan_ioctl
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

static int fdcan_ioctl(FAR struct can_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct stm32_fdcan_s *priv;
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
          uint32_t nbrp;

          DEBUGASSERT(bt != NULL);

          regval        = fdcan_getreg(priv, STM32_FDCAN_NBTP_OFFSET);
          bt->bt_sjw   = ((regval & FDCAN_NBTP_NSJW_MASK) >>
                          FDCAN_NBTP_NSJW_SHIFT) + 1;
          bt->bt_tseg1 = ((regval & FDCAN_NBTP_NTSEG1_MASK) >>
                          FDCAN_NBTP_NTSEG1_SHIFT) + 1;
          bt->bt_tseg2 = ((regval & FDCAN_NBTP_NTSEG2_MASK) >>
                          FDCAN_NBTP_NTSEG2_SHIFT) + 1;

          nbrp          = ((regval & FDCAN_NBTP_NBRP_MASK) >>
                          FDCAN_NBTP_NBRP_SHIFT) + 1;
          bt->bt_baud   = STM32H7_FDCANCLK_FREQUENCY / nbrp /
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
       * threads reconfigures the bitrate, the FDCAN hardware will be reset
       * and the context of operation will be lost.  Hence, this IOCTL can
       * only safely be executed in quiescent time periods.
       */

      case CANIOC_SET_BITTIMING:
        {
          FAR const struct canioc_bittiming_s *bt =
            (FAR const struct canioc_bittiming_s *)arg;
          irqstate_t flags;
          uint32_t nbrp;
          uint32_t ntseg1;
          uint32_t ntseg2;
          uint32_t nsjw;
          uint32_t ie;
          uint8_t state;

          DEBUGASSERT(bt != NULL);
          DEBUGASSERT(bt->bt_baud < STM32H7_FDCANCLK_FREQUENCY);
          DEBUGASSERT(bt->bt_sjw > 0 && bt->bt_sjw <= 16);
          DEBUGASSERT(bt->bt_tseg1 > 1 && bt->bt_tseg1 <= 64);
          DEBUGASSERT(bt->bt_tseg2 > 0 && bt->bt_tseg2 <= 16);

          /* Extract bit timing data */

          ntseg1 = bt->bt_tseg1 - 1;
          ntseg2 = bt->bt_tseg2 - 1;
          nsjw   = bt->bt_sjw   - 1;

          nbrp = (uint32_t)
            (((float) STM32H7_FDCANCLK_FREQUENCY /
             ((float)(ntseg1 + ntseg2 + 3) * (float)bt->bt_baud)) - 1);

          /* Save the value of the new bit timing register */

          flags = enter_critical_section();
          priv->nbtp = FDCAN_NBTP_NBRP(nbrp) | FDCAN_NBTP_NTSEG1(ntseg1) |
                      FDCAN_NBTP_NTSEG2(ntseg2) | FDCAN_NBTP_NSJW(nsjw);

          /* We need to reset to instantiate the new timing.  Save
           * current state information so that recover to this
           * state.
           */

          ie    = fdcan_getreg(priv, STM32_FDCAN_IE_OFFSET);
          state = priv->state;

          /* Reset the FDCAN */

          fdcan_reset(dev);
          ret = OK;

          /* If we have previously been setup, then setup again */

          if (state == FDCAN_STATE_SETUP)
            {
              ret = fdcan_setup(dev);
            }

          /* We we have successfully re-initialized, then restore the
           * interrupt state.
           *
           * REVISIT: Since the hardware was reset, any pending TX
           * activity was lost.  Should we disable TX interrupts?
           */

          if (ret == OK)
            {
              fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, ie & ~priv->txints);
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

          ret = fdcan_add_extfilter(priv,
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
          ret = fdcan_del_extfilter(priv, (int)arg);
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

          ret = fdcan_add_stdfilter(priv,
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
          ret = fdcan_del_stdfilter(priv, (int)arg);
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
          ret = fdcan_start_busoff_recovery_sequence(priv);
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
 * Name: fdcan_remoterequest
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

static int fdcan_remoterequest(FAR struct can_dev_s *dev, uint16_t id)
{
  /* REVISIT:  Remote request not implemented */

  return -ENOSYS;
}

/****************************************************************************
 * Name: fdcan_send
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

static int fdcan_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg)
{
  FAR struct stm32_fdcan_s *priv;
  FAR const struct stm32_config_s *config;
  FAR uint32_t *txbuffer = 0;
  FAR const uint8_t *src;
  FAR uint32_t *dest;
  uint32_t regval;
  irqstate_t flags;
  // unsigned int msglen;
  unsigned int ndx;
  unsigned int nbytes;
  uint32_t wordbuffer;
  unsigned int i;
  int ret;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  caninfo("FDCAN%d\n", config->port);
  caninfo("FDCAN%d ID: %d DLC: %d\n",
          config->port, msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

  /* That that FIFO elements were configured.
   *
   * REVISIT: Dedicated TX buffers are not used by this driver.
   */

  DEBUGASSERT(config->ntxfifoq > 0);

  /* Reserve a buffer for the transmission.  If fdcan_buffer_reserve()
   * returns succesful, we are guaranteed that the TX FIFOQ is not full
   * and cannot become full at least until we add our packet to the FIFO.
   * If it returns unsuccesful, we should exit now to ensure that non-blocking
   * operation is maintained.
   *
   * We can't get exclusive access to FDCAN resources here because that
   * locks the FDCAN while we wait for a free buffer.  Instead, the
   * scheduler is locked here momentarily.  See discussion in
   * fdcan_buffer_reserve() for an explanation.
   */

  sched_lock();
  ret = fdcan_buffer_reserve(priv);
  if (ret < 0)
    {
      sched_unlock();
      return ret;
    }

  /* Get exclusive access to the FDCAN peripheral */

  ret = fdcan_dev_lock(priv);
  if (ret < 0)
    {
      fdcan_buffer_release(priv);
      sched_unlock();
      return ret;
    }

  sched_unlock();

  /* Get our reserved Tx FIFO/queue put index */

  regval = fdcan_getreg(priv, STM32_FDCAN_TXFQS_OFFSET);
  DEBUGASSERT((regval & FDCAN_TXFQS_TFQF) == 0);

  ndx = (regval & FDCAN_TXFQS_TFQPI_MASK) >> FDCAN_TXFQS_TFQPI_SHIFT;

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
  reginfo("T1: %08x\n", txbuffer[1]);

  /* Followed by the amount of data corresponding to the DLC (T2..) */

  dest   = &txbuffer[2];
  src    = msg->cm_data;
  nbytes = fdcan_dlc2bytes(priv, msg->cm_hdr.ch_dlc);

  /* Writes must be word length */

  for (i = 0; i < nbytes; i += 4)
    {
      /* Little endian is assumed */

      wordbuffer = src[0] |
                  (src[1] << 8) |
                  (src[2] << 16) |
                  (src[3] << 24);
      src += 4;

      *dest++ = wordbuffer;
    }

  /* Flush the D-Cache to memory before initiating the transfer */
  /* REVISIT: Make sure D-Cache isn't actually used by FDCAN */

  /* msglen = 2 * sizeof(uint32_t) + nbytes;
   * up_clean_dcache((uintptr_t)txbuffer, (uintptr_t)txbuffer + msglen);
   * UNUSED(msglen);
   */

  /* Enable transmit interrupts from the TX FIFOQ buffer by setting TC
   * interrupt bit in IR (also requires that the TC interrupt is enabled)
   */

  fdcan_putreg(priv, STM32_FDCAN_TXBTIE_OFFSET, (1 << ndx));

  /* And request to send the packet */

  fdcan_putreg(priv, STM32_FDCAN_TXBAR_OFFSET, (1 << ndx));
  fdcan_dev_unlock(priv);

  /* Report that the TX transfer is complete to the upper half logic.  Of
   * course, the transfer is not complete, but this early notification
   * allows the upper half logic to free resources sooner.
   *
   * REVISIT:  Should we disable interrupts?  can_txdone() was designed to
   * be called from an interrupt handler and, hence, may be unsafe when
   * called from the tasking level.
   *
   * NOTE: The calls to enter/leave_critical_section() were added below to satisify
   * the requirement that interrupts are disabled when can_txdone() is called.
   */

  flags = enter_critical_section();
  can_txdone(dev);
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: fdcan_txready
 *
 * Description:
 *   Return true if the FDCAN hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if the FDCAN hardware is ready to accept another TX message.
 *
 ****************************************************************************/

static bool fdcan_txready(FAR struct can_dev_s *dev)
{
  FAR struct stm32_fdcan_s *priv = dev->cd_priv;
  uint32_t regval;
  bool notfull;
#ifdef CONFIG_DEBUG_FEATURES
  int sval;
#endif
  int ret;

  /* Get exclusive access to the FDCAN peripheral */

  ret = fdcan_dev_lock(priv);
  if (ret < 0)
    {
      return false;
    }

  /* Return the state of the TX FIFOQ.  Return TRUE if the TX FIFO/Queue is
   * not full.
   *
   * REVISIT: Dedicated TX buffers are not supported.
   */

  regval  = fdcan_getreg(priv, STM32_FDCAN_TXFQS_OFFSET);
  notfull = ((regval & FDCAN_TXFQS_TFQF) == 0);

#ifdef CONFIG_DEBUG_FEATURES
  /* As a sanity check, the txfsem should also track the number of elements
   * the TX FIFO/queue.  Make sure that they are consistent.
   */

  nxsem_get_value(&priv->txfsem, &sval);
  DEBUGASSERT(((notfull && sval > 0) || (!notfull && sval <= 0)) &&
              (sval <= priv->config->ntxfifoq));
#endif

  fdcan_dev_unlock(priv);
  return notfull;
}

/****************************************************************************
 * Name: fdcan_txempty
 *
 * Description:
 *   Return true if all message have been sent.  If for example, the FDCAN
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if there are no pending TX transfers in the FDCAN hardware.
 *
 ****************************************************************************/

static bool fdcan_txempty(FAR struct can_dev_s *dev)
{
  FAR struct stm32_fdcan_s *priv = dev->cd_priv;
  uint32_t regval;
  int ret;
#ifdef CONFIG_STM32H7_FDCAN_QUEUE_MODE
  int sval;
#else
  int tffl;
#endif
  bool empty;

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  /* Get exclusive access to the FDCAN peripheral */

  ret = fdcan_dev_lock(priv);
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

  regval = fdcan_getreg(priv, STM32_FDCAN_TXFQS_OFFSET);
  if (((regval & FDCAN_TXFQS_TFQF) != 0))
    {
      fdcan_dev_unlock(priv);
      return false;
    }

#ifdef CONFIG_STM32H7_FDCAN_QUEUE_MODE
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

  tffl  = (regval & FDCAN_TXFQS_TFFL_MASK) >> FDCAN_TXFQS_TFFL_SHIFT;
  empty = (tffl >= priv->config->ntxfifoq);
#endif

  fdcan_dev_unlock(priv);
  return empty;
}

/****************************************************************************
 * Name: fdcan_dedicated_rxbuffer_available
 *
 * Description:
 *   Check if data is available in a dedicated RX buffer.
 *
 * Input Parameters:
 *   priv   - FDCAN-specific private data
 *   bufndx - Buffer index
 *
 *   None
 * Returned Value:
 *   True: Data is available
 *
 ****************************************************************************/

#if 0 /* Not Used */
bool fdcan_dedicated_rxbuffer_available(FAR struct stm32_fdcan_s *priv,
                                       int bufndx)
{
  if (bufndx < 32)
    {
      return (bool)(fdcan->FDCAN_NDAT1 & (1 << bufndx));
    }
  else if (bufndx < 64)
    {
      return (bool)(fdcan->FDCAN_NDAT1 & (1 << (bufndx - 32)));
    }
  else
    {
      return false;
    }
}
#endif

/****************************************************************************
 * Name: fdcan_error
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
static void fdcan_error(FAR struct can_dev_s *dev, uint32_t status)
{
  FAR struct stm32_fdcan_s *priv = dev->cd_priv;
  struct can_hdr_s hdr;
  uint32_t psr;
  uint16_t errbits;
  uint8_t data[CAN_ERROR_DLC];
  int ret;

  /* Encode error bits */

  errbits = 0;
  memset(data, 0, sizeof(data));

  /* Always fill in "static" error conditions, but set the signaling bit
   * only if the condition has changed (see IRQ-Flags below)
   * They have to be filled in every time CAN_ERROR_CONTROLLER is set.
   */

  psr = fdcan_getreg(priv, STM32_FDCAN_PSR_OFFSET);
  if ((psr & FDCAN_PSR_EP) != 0)
    {
      data[1] |= (CAN_ERROR1_RXPASSIVE | CAN_ERROR1_TXPASSIVE);
    }

  if (psr & FDCAN_PSR_EW)
    {
      data[1] |= (CAN_ERROR1_RXWARNING | CAN_ERROR1_TXWARNING);
    }

  if ((status & (FDCAN_INT_EP | FDCAN_INT_EW)) != 0)
    {
      /* "Error Passive" or "Error Warning" status changed */

      errbits |= CAN_ERROR_CONTROLLER;
    }

  if ((status & FDCAN_INT_PEA) != 0)
    {
      /* Protocol Error in Arbitration Phase */

      if (psr & FDCAN_PSR_LEC_MASK)
        {
          /* Error code present */

          if ((psr & FDCAN_PSR_LEC(FDCAN_PSR_EC_STUFF_ERROR)) != 0)
            {
              /* Stuff Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[2] |= CAN_ERROR2_STUFF;
            }

          if ((psr & FDCAN_PSR_LEC(FDCAN_PSR_EC_FORM_ERROR)) != 0)
            {
              /* Format Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[2] |= CAN_ERROR2_FORM;
            }

          if ((psr & FDCAN_PSR_LEC(FDCAN_PSR_EC_ACK_ERROR)) != 0)
            {
              /* Acknowledge Error */

              errbits |= CAN_ERROR_NOACK;
            }

          if ((psr & FDCAN_PSR_LEC(FDCAN_PSR_EC_BIT0_ERROR)) != 0)
            {
              /* Bit0 Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[2] |= CAN_ERROR2_BIT0;
            }

          if ((psr & FDCAN_PSR_LEC(FDCAN_PSR_EC_BIT1_ERROR)) != 0)
            {
              /* Bit1 Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[2] |= CAN_ERROR2_BIT1;
            }

          if ((psr & FDCAN_PSR_LEC(FDCAN_PSR_EC_CRC_ERROR)) != 0)
            {
              /* Receive CRC Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[3] |= (CAN_ERROR3_CRCSEQ | CAN_ERROR3_CRCDEL);
            }

          if ((psr & FDCAN_PSR_LEC(FDCAN_PSR_EC_NO_CHANGE)) != 0)
            {
              /* No Change in Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[2] |= CAN_ERROR2_UNSPEC;
            }
        }
    }

  if ((status & FDCAN_INT_PED) != 0)
    {
      /* Protocol Error in Data Phase */

      if (psr & FDCAN_PSR_DLEC_MASK)
        {
          /* Error code present */

          if ((psr & FDCAN_PSR_DLEC(FDCAN_PSR_EC_STUFF_ERROR)) != 0)
            {
              /* Stuff Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[2] |= CAN_ERROR2_STUFF;
            }

          if ((psr & FDCAN_PSR_DLEC(FDCAN_PSR_EC_FORM_ERROR)) != 0)
            {
              /* Format Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[2] |= CAN_ERROR2_FORM;
            }

          if ((psr & FDCAN_PSR_DLEC(FDCAN_PSR_EC_ACK_ERROR)) != 0)
            {
              /* Acknowledge Error */

              errbits |= CAN_ERROR_NOACK;
            }

          if ((psr & FDCAN_PSR_DLEC(FDCAN_PSR_EC_BIT0_ERROR)) != 0)
            {
              /* Bit0 Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[2] |= CAN_ERROR2_BIT0;
            }

          if ((psr & FDCAN_PSR_DLEC(FDCAN_PSR_EC_BIT1_ERROR)) != 0)
            {
              /* Bit1 Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[2] |= CAN_ERROR2_BIT1;
            }

          if ((psr & FDCAN_PSR_DLEC(FDCAN_PSR_EC_CRC_ERROR)) != 0)
            {
              /* Receive CRC Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[3] |= (CAN_ERROR3_CRCSEQ | CAN_ERROR3_CRCDEL);
            }

          if ((psr & FDCAN_PSR_DLEC(FDCAN_PSR_EC_NO_CHANGE)) != 0)
            {
              /* No Change in Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[2] |= CAN_ERROR2_UNSPEC;
            }
        }
    }

  if ((status & FDCAN_INT_BO) != 0)
    {
      /* Bus_Off Status changed */

      if ((psr & FDCAN_PSR_BO) != 0)
        {
          errbits |= CAN_ERROR_BUSOFF;
        }
      else
        {
          errbits |= CAN_ERROR_RESTARTED;
        }
    }

  if ((status & (FDCAN_INT_RF0L | FDCAN_INT_RF1L)) != 0)
    {
      /* Receive FIFO 0/1 Message Lost
       * Receive FIFO 1 Message Lost
       */

      errbits |= CAN_ERROR_CONTROLLER;
      data[1] |= CAN_ERROR1_RXOVERFLOW;
    }

  if ((status & FDCAN_INT_TEFL) != 0)
    {
      /* Tx Event FIFO Element Lost */

      errbits |= CAN_ERROR_CONTROLLER;
      data[1] |= CAN_ERROR1_TXOVERFLOW;
    }

  if ((status & FDCAN_INT_TOO) != 0)
    {
      /* Timeout Occurred */

      errbits |= CAN_ERROR_TXTIMEOUT;
    }

  if ((status & (FDCAN_INT_MRAF | FDCAN_INT_ELO)) != 0)
    {
      /* Message RAM Access Failure
       * Error Logging Overflow
       */

      errbits |= CAN_ERROR_CONTROLLER;
      data[1] |= CAN_ERROR1_UNSPEC;
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
 * Name: fdcan_receive
 *
 * Description:
 *   Receive an FDCAN messages
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

static void fdcan_receive(FAR struct can_dev_s *dev, FAR uint32_t *rxbuffer,
                         unsigned long nwords)
{
  struct can_hdr_s hdr;
  uint32_t regval;
  /* unsigned int nbytes; */
  int ret;

  fdcan_dumprxregs(dev->cd_priv, "Before receive");

  /* Invalidate the D-Cache so that we reread the RX buffer data from memory. */
  /* REVISIT: Make sure D-Cache actually isn't used by FDCAN */

  /* nbytes = (nwords << 2);
   * up_invalidate_dcache((uintptr_t)rxbuffer, (uintptr_t)rxbuffer + nbytes);
   */

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

  /* And provide the CAN message to the upper half logic */

  ret = can_receive(dev, &hdr, (FAR uint8_t *)rxbuffer);
  if (ret < 0)
    {
      canerr("ERROR: can_receive failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: fdcan_interrupt
 *
 * Description:
 *   Common FDCAN interrupt handler
 *
 * Input Parameters:
 *   dev - CAN-common state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int fdcan_interrupt(int irq, void *context, FAR void *arg)
{
  FAR struct can_dev_s *dev = (FAR struct can_dev_s *)arg;
  FAR struct stm32_fdcan_s *priv;
  FAR const struct stm32_config_s *config;
  uint32_t ir;
  uint32_t ie;
  uint32_t pending;
  uint32_t regval;
  unsigned int nelem;
  unsigned int ndx;
  bool handled;

  DEBUGASSERT(dev != NULL);
  priv = dev->cd_priv;
  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  /* Loop while there are pending interrupt events */

  do
    {
      /* Get the set of pending interrupts. */

      ir = fdcan_getreg(priv, STM32_FDCAN_IR_OFFSET);
      ie = fdcan_getreg(priv, STM32_FDCAN_IE_OFFSET);

      pending = (ir & ie);
      handled = false;

      /* Check for any errors */

      if ((pending & FDCAN_ANYERR_INTS) != 0)
        {
          /* Check for common errors */

          if ((pending & FDCAN_CMNERR_INTS) != 0)
            {
              canerr("ERROR: Common %08x\n", pending & FDCAN_CMNERR_INTS);

              /* When a protocol error ocurrs, the problem is recorded in
               * the LEC/DLEC fields of the PSR register. In lieu of
               * seprate interrupt flags for each error, the hardware
               * groups procotol errors under a single interrupt each for
               * arbitration and data phases.
               *
               * These errors have a tendency to flood the system with
               * interrupts, so they are disabled here until we get a
               * successful transfer/receive on the hardware
               */
              
              uint32_t psr = fdcan_getreg(priv, STM32_FDCAN_PSR_OFFSET);
              
              if ((psr & FDCAN_PSR_LEC_MASK) != 0)
                {
                  ie &= ~(FDCAN_INT_PEA | FDCAN_INT_PED);
                  fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, ie);
                  caninfo("disabled protocol error intterupts\n");
                }

              /* Clear the error indications */

              fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, FDCAN_CMNERR_INTS);
            }

          /* Check for transmission errors */

          if ((pending & FDCAN_TXERR_INTS) != 0)
            {
              canerr("ERROR: TX %08x\n", pending & FDCAN_TXERR_INTS);

              /* An Acknowledge-Error will occur if for example the device
               * is not connected to the bus.
               *
               * The CAN-Standard states that the Chip has to retry the
               * message forever, which will produce an ACKE every time.
               * To prevent this Interrupt-Flooding and the high CPU-Load
               * we disable the ACKE here as long we didn't transfer at
               * least one message successfully (see FDCAN_INT_TC below).
               */

              // uint32_t psr = fdcan_getreg(priv, STM32_FDCAN_PSR_OFFSET);

              // if ((psr & FDCAN_PSR_LEC(FDCAN_PSR_EC_ACK_ERROR)) != 0)
              //   {
              //     ie &= ~(FDCAN_INT_PEA | FDCAN_INT_PED);
              //     fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, ie);
              //     caninfo("disabled pea/d\n");
              //   }


              /* Clear the error indications */

              fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, FDCAN_TXERR_INTS);

              /* REVISIT:  Will FDCAN_INT_TC also be set in the event of
               * a transmission error?  Each write must conclude with a
               * call to fdcan_buffer_release(), whether or not the write
               * was successful.
               *
               * We assume that FDCAN_INT_TC will be called for each
               * message buffer. Except the transfer is cancelled.
               * TODO: add handling for FDCAN_INT_TCF
               */
            }

          /* Check for reception errors */

          if ((pending & FDCAN_RXERR_INTS) != 0)
            {
              canerr("ERROR: RX %08x\n", pending & FDCAN_RXERR_INTS);

              /* To prevent Interrupt-Flooding the current active
               * RX error interrupts are disabled. After successfully
               * receiving at least one CAN packet all RX error interrupts
               * are turned back on.
               *
               * The Interrupt-Flooding can for example occur if the
               * configured CAN speed does not match the speed of the other
               * CAN nodes in the network.
               */

              ie &= ~(pending & FDCAN_RXERR_INTS);
              fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, ie);

              /* Clear the error indications */

              fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, FDCAN_RXERR_INTS);
            }

#ifdef CONFIG_CAN_ERRORS
          /* Report errors */

          fdcan_error(dev, pending & FDCAN_ANYERR_INTS);
#endif
          handled = true;
        }

      /* Check for successful completion of a transmission */

      if ((pending & FDCAN_INT_TC) != 0)
        {
          /* Check if we have disabled the ACKE in the error-handling above
           * (see FDCAN_TXERR_INTS) to prevent Interrupt-Flooding and
           * re-enable the error interrupt here again.
           */

          if ((ie & (FDCAN_INT_PEA | FDCAN_INT_PED)) == 0)
            {
                ie |= (FDCAN_INT_PEA | FDCAN_INT_PED);
                fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, ie);
                caninfo("Renabled protocol error intterupts\n");
            }

          /* Clear the pending TX completion interrupt (and all
           * other TX-related interrupts)
           */

          fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, priv->txints);

          /* Indicate that there is one more buffer free in the TX FIFOQ by
           * "releasing" it.  This may have the effect of waking up a thread
           * that has been waiting for a free TX FIFOQ buffer.
           *
           * REVISIT: TX dedicated buffers are not supported.
           */

          fdcan_buffer_release(priv);
          handled = true;

#ifdef CONFIG_CAN_TXREADY
          /* Inform the upper half driver that we are again ready to accept
           * data in fdcan_send().
           */

          can_txready(dev);
#endif
        }
      else if ((pending & priv->txints) != 0)
        {
          /* Clear unhandled TX events */

          fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, priv->txints);
          handled = true;
        }

#if 0 /* Not used */
      /* Check if a message has been stored to the dedicated RX buffer (DRX) */

      if ((pending & FDCAN_INT_DRX) != 0)
        {
          int i;

          /* Clear the pending DRX interrupt */

          fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, FDCAN_INT_DRX);

          /* Process each dedicated RX buffer */

          for (i = 0; i < config->nrxdedicated; i++)
            {
              uint32_t *rxdedicated = &config->rxdedicated[i];

              /* Check if datat is available in this dedicated RX buffer */

              if (fdcan_dedicated_rxbuffer_available(priv, i))
                {
                  /* Yes.. Invalidate the D-Cache to that data will be re-
                   * fetched from RAM.
                   *
                   * REVISIT:  This will require 32-byte alignment.
                   */

                  arch_invalidata_dcache();
                  fdcan_receive(priv, rxdedicated, config->rxbufferesize);

                  /* Clear the new data flag for the buffer */

                  if (i < 32)
                    {
                      sam_putreg(priv, STM32_FDCAN_NDAT1_OFFSET,
                                 (1 << i);
                    }
                  else
                    {
                      sam_putreg(priv, STM32_FDCAN_NDAT1_OFFSET,
                                 (1 << (i - 32));
                    }
                }
            }

          handled = true;
        }
#endif

      /* Clear the RX FIFO1 new message interrupt */

      fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, FDCAN_INT_RF1N);
      pending &= ~FDCAN_INT_RF1N;

      /* We treat RX FIFO1 as the "high priority" queue:  We will process
       * all messages in RX FIFO1 before processing any message from RX
       * FIFO0.
       */

      for (; ; )
        {
          /* Check if there is anything in RX FIFO1 */

          regval = fdcan_getreg(priv, STM32_FDCAN_RXF1S_OFFSET);
          nelem  = (regval & FDCAN_RXFS_FFL_MASK) >> FDCAN_RXFS_FFL_SHIFT;
          if (nelem == 0)
            {
              /* Break out of the loop if RX FIFO1 is empty */

              break;
            }

          /* Clear the RX FIFO1 interrupt (and all other FIFO1-related
           * interrupts)
           */

          /* Handle the newly received message in FIFO1 */

          ndx = (regval & FDCAN_RXFS_FGI_MASK) >> FDCAN_RXFS_FGI_SHIFT;

          if ((regval & FDCAN_RXFS_RFL) != 0)
            {
              canerr("ERROR: Message lost: %08x\n", regval);
            }
          else
            {
              fdcan_receive(dev,
                           config->msgram.rxfifo1 +
                             (ndx * priv->config->rxfifo1esize),
                           priv->config->rxfifo1esize);

              /* Turning back on all configured RX error interrupts */

              ie |= (priv->rxints & FDCAN_RXERR_INTS);
              fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, ie);
            }

          /* Acknowledge reading the FIFO entry */

          fdcan_putreg(priv, STM32_FDCAN_RXF1A_OFFSET, ndx);
          handled = true;
        }

      /* Check for successful reception of a new message in RX FIFO0 */

      /* Clear the RX FIFO0 new message interrupt */

      fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, FDCAN_INT_RF0N);
      pending &= ~FDCAN_INT_RF0N;

      /* Check if there is anything in RX FIFO0 */

      regval = fdcan_getreg(priv, STM32_FDCAN_RXF0S_OFFSET);
      nelem  = (regval & FDCAN_RXFS_FFL_MASK) >> FDCAN_RXFS_FFL_SHIFT;
      if (nelem > 0)
        {
          /* Handle the newly received message in FIFO0 */

          ndx = (regval & FDCAN_RXFS_FGI_MASK) >> FDCAN_RXFS_FGI_SHIFT;

          if ((regval & FDCAN_RXFS_RFL) != 0)
            {
              canerr("ERROR: Message lost: %08x\n", regval);
            }
          else
            {
              fdcan_receive(dev,
                           config->msgram.rxfifo0 +
                             (ndx * priv->config->rxfifo0esize),
                           priv->config->rxfifo0esize);

              /* Turning back on all configured RX error interrupts */

              ie |= (priv->rxints & FDCAN_RXERR_INTS);
              fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, ie);
            }

          /* Acknowledge reading the FIFO entry */

          fdcan_putreg(priv, STM32_FDCAN_RXF0A_OFFSET, ndx);
          handled = true;
        }

      /* Clear unhandled RX interrupts */

      if ((pending & priv->rxints) != 0)
        {
          fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, priv->rxints);
        }
    }
  while (handled);

  return OK;
}

/****************************************************************************
 * Name: fdcan_hw_initialize
 *
 * Description:
 *   FDCAN hardware initialization
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this FDCAN peripheral
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int fdcan_hw_initialize(struct stm32_fdcan_s *priv)
{
  FAR const struct stm32_config_s *config = priv->config;
  FAR uint32_t *msgram;
  uint32_t regval;
  uint32_t cntr;

  caninfo("FDCAN%d\n", config->port);

  /* Configure FDCAN pins */

  stm32_configgpio(config->rxpinset);
  stm32_configgpio(config->txpinset);

  /* Renable device if previosuly disabled in fdcan_shutdown() */

  if (priv->state == FDCAN_STATE_DISABLED)
  {
    /* Reset Clock Stop Request bit */

    regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
    regval &= ~FDCAN_CCCR_CSR;
    fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

    /* Wait for Clock Stop Acknowledge bit reset to indicate
     * device is operational
     */

    while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) & FDCAN_CCCR_CSA) == 1);
  }

  /* Enable the Initialization state */

  regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
  regval |= FDCAN_CCCR_INIT;
  fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

  /* Wait for initialization mode to take effect */

  while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) & FDCAN_CCCR_INIT) == 0);

  /* Enable writing to configuration registers */

  regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
  regval |= FDCAN_CCCR_CCE;
  fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

  /* Global Filter Configuration:
   *
   *   ANFS=0: Store all non matching standard frame in RX FIFO0
   *   ANFE=0: Store all non matching extended frame in RX FIFO0
   */

  regval = FDCAN_GFC_ANFE_RX_FIFO0 | FDCAN_GFC_ANFS_RX_FIFO0;
  fdcan_putreg(priv, STM32_FDCAN_GFC_OFFSET, regval);

  /* Extended ID Filter AND mask  */

  fdcan_putreg(priv, STM32_FDCAN_XIDAM_OFFSET, 0x1fffffff);

  /* Disable all interrupts  */

  fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, 0);
  fdcan_putreg(priv, STM32_FDCAN_TXBTIE_OFFSET, 0);

  /* All interrupts directed to Line 0.  But disable both interrupt lines 0
   * and 1 for now.
   *
   * REVISIT: Only interrupt line 0 is used by this driver.
   */

  fdcan_putreg(priv, STM32_FDCAN_ILS_OFFSET, 0);
  fdcan_putreg(priv, STM32_FDCAN_ILE_OFFSET, 0);

  /* Clear all pending interrupts. */

  fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, FDCAN_INT_ALL);

  /* Configure FDCAN bit timing */

  fdcan_putreg(priv, STM32_FDCAN_NBTP_OFFSET, priv->nbtp);
  fdcan_putreg(priv, STM32_FDCAN_DBTP_OFFSET, priv->dbtp);

  /* Configure message RAM starting addresses and sizes. */

  regval = FDCAN_SIDFC_FLSSA(config->stdfiltersstart) |
           FDCAN_SIDFC_LSS(config->nstdfilters);
  fdcan_putreg(priv, STM32_FDCAN_SIDFC_OFFSET, regval);

  regval = FDCAN_XIDFC_FLESA(config->extfiltersstart) |
           FDCAN_XIDFC_LSE(config->nextfilters);
  fdcan_putreg(priv, STM32_FDCAN_XIDFC_OFFSET, regval);

  /* Configure RX FIFOs */

  regval = FDCAN_RXFC_FSA(config->rxfifo0start) |
           FDCAN_RXFC_FS(config->nrxfifo0);
  fdcan_putreg(priv, STM32_FDCAN_RXF0C_OFFSET, regval);
  
  regval = FDCAN_RXFC_FSA(config->rxfifo1start) |
           FDCAN_RXFC_FS(config->nrxfifo1);
  fdcan_putreg(priv, STM32_FDCAN_RXF1C_OFFSET, regval);

  /* Watermark interrupt off, blocking mode */

  regval = FDCAN_RXBC_RBSA((uint32_t)config->rxbufferstart);
  fdcan_putreg(priv, STM32_FDCAN_RXBC_OFFSET, regval);

  regval = FDCAN_TXEFC_EFSA((uint32_t)config->txeventstart) |
           FDCAN_TXEFC_EFS(config->ntxeventfifo);
  fdcan_putreg(priv, STM32_FDCAN_TXEFC_OFFSET, regval);

  /* Watermark interrupt off */

  regval = FDCAN_TXBC_TBSA(config->txbufferstart) |
           FDCAN_TXBC_NDTB(config->ntxdedicated) |
           FDCAN_TXBC_TFQS(config->ntxfifoq);
  fdcan_putreg(priv, STM32_FDCAN_TXBC_OFFSET, regval);

  regval = FDCAN_RXESC_RBDS(config->rxbufferecode) |
           FDCAN_RXESC_F1DS(config->rxfifo1ecode) |
           FDCAN_RXESC_F0DS(config->rxfifo0ecode);
  fdcan_putreg(priv, STM32_FDCAN_RXESC_OFFSET, regval);

  regval = FDCAN_TXESC_TBDS(config->txbufferecode);
  fdcan_putreg(priv, STM32_FDCAN_TXESC_OFFSET, regval);

  fdcan_dumpramlayout(priv);

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

  fdcan_putreg(priv, STM32_FDCAN_NDAT1_OFFSET, 0xffffffff);
  fdcan_putreg(priv, STM32_FDCAN_NDAT2_OFFSET, 0xffffffff);

  /* TTCAN configuration (FDCAN1 only) */

  if(config->port == 1)
    {
      /* Disable TTCAN operation */

      regval = fdcan_getreg(priv, STM32_FDCAN_TTOCF_OFFSET);
      regval &= ~FDCAN_TTOCF_OM_MASK;
      fdcan_putreg(priv, STM32_FDCAN_TTOCF_OFFSET, regval);
    }
  
  /* CCU configuration
   * REVIST: CCU currently bypassed to used kernel clock
   * directly. Could be beneficial to get this working.
   */

  regval = getreg32(STM32_FDCAN_CCU_CCFG);
  regval |= FDCAN_CCU_CCFG_BCC;
  putreg32(regval, STM32_FDCAN_CCU_CCFG);

  regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
  regval &= ~(FDCAN_CCCR_NISO | FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE);

  /* Select ISO11898-1 or Non ISO Bosch CAN FD Specification V1.0 */

    switch (config->format)
    {
    default:
    case FDCAN_ISO11898_1_FORMAT:
      break;

    case FDCAN_NONISO_BOSCH_V1_FORMAT:
      regval |= FDCAN_CCCR_NISO;
      break;
    }

  /* Select Classic CAN mode or FD mode with or without fast bit rate
   * switching
   */

  switch (config->mode)
    {
    default:
    case FDCAN_CLASSIC_MODE:
      break;

#ifdef CONFIG_CAN_FD
    case FDCAN_FD_MODE:
      regval |= FDCAN_CCCR_FDOE;
      break;

    case FDCAN_FD_BRS_MODE:
      regval |= (FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE);
      break;
#endif
    }

  /* Set the initial CAN mode */

  fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

  /* Enable FIFO/Queue mode
   *
   * REVISIT: Dedicated TX buffers are not used.
   */

  regval  = fdcan_getreg(priv, STM32_FDCAN_TXBC_OFFSET);
#ifdef CONFIG_STM32H7_FDCAN_QUEUE_MODE
  regval |= FDCAN_TXBC_TFQM;
#else
  regval &= ~FDCAN_TXBC_TFQM;
#endif
  fdcan_putreg(priv, STM32_FDCAN_TXBC_OFFSET, regval);

#ifdef STM32H7_FDCAN_LOOPBACK
  /* Is loopback mode selected for this peripheral? */

  if (config->loopback)
    {
     /* FDCAN_CCCR_TEST  - Test mode enable
      * FDCAN_CCCR_MON   - Bus monitoring mode (for internal loopback)
      * FDCAN_TEST_LBCK  - Loopback mode
      */

      regval = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
      regval |= (FDCAN_CCCR_TEST | FDCAN_CCCR_MON);
      fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

      regval = fdcan_getreg(priv, STM32_FDCAN_TEST_OFFSET);
      regval |= FDCAN_TEST_LBCK;
      fdcan_putreg(priv, STM32_FDCAN_TEST_OFFSET, regval);
    }
#endif

  /* Configure interrupt lines */

  /* Select RX-related interrupts */

#if 0 /* Dedicated RX buffers are not used by this driver */
  priv->rxints = FDCAN_RXDEDBUF_INTS;
#else
  priv->rxints = FDCAN_RXFIFO_INTS;
#endif

  /* Select TX-related interrupts */

#if 0 /* Dedicated TX buffers are not used by this driver */
  priv->txints = FDCAN_TXDEDBUF_INTS;
#else
  priv->txints = FDCAN_TXFIFOQ_INTS;
#endif

  /* Direct all interrupts to Line 0.
   *
   * Bits in the ILS register correspond to each FDCAN interrupt; A bit
   * set to '1' is directed to interrupt line 1; a bit cleared to '0'
   * is directed interrupt line 0.
   *
   * REVISIT: Nothing is done here.  Only interrupt line 0 is used by
   * this driver and ILS was already cleared above.
   */

  /* Enable only interrupt line 0. */

  fdcan_putreg(priv, STM32_FDCAN_ILE_OFFSET, FDCAN_ILE_EINT0);

  /* Disable initialization mode to enable normal operation */

  regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
  regval &= ~FDCAN_CCCR_INIT;
  fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_fdcan_initialize
 *
 * Description:
 *   Initialize the selected FDCAN port
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple FDCAN interfaces),
 *          0=FDCAN1, 1=FDCAN2
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct can_dev_s *stm32_fdcan_initialize(int port)
{
  FAR struct can_dev_s *dev;
  FAR struct stm32_fdcan_s *priv;
  FAR const struct stm32_config_s *config;

  caninfo("FDCAN%d\n", port);

  /* Select FDCAN peripheral to be initialized */

#ifdef CONFIG_STM32H7_FDCAN1
  if (port == FDCAN1)
    {
      /* Select the FDCAN1 device structure */

      dev    = &g_fdcan1dev;
      priv   = &g_fdcan1priv;
      config = &g_fdcan1const;
    }
  else
#endif
#ifdef CONFIG_STM32H7_FDCAN2
  if (port == FDCAN2)
    {
      /* Select the FDCAN2 device structure */

      dev    = &g_fdcan2dev;
      priv   = &g_fdcan2priv;
      config = &g_fdcan2const;
    }
  else
#endif
    {
      canerr("ERROR: Unsupported port %d\n", port);
      return NULL;
    }

  /* Is this the first time that we have handed out this device? */

  if (priv->state == FDCAN_STATE_UNINIT)
    {
      /* Yes, then perform one time data initialization */

      memset(priv, 0, sizeof(struct stm32_fdcan_s));
      priv->config = config;

      /* Set the initial bit timing.  This might change subsequently
       * due to IOCTL command processing.
       */

      priv->nbtp   = config->nbtp;
      priv->dbtp   = config->dbtp;

      /* Initialize semaphores */

      nxsem_init(&priv->locksem, 0, 1);
      nxsem_init(&priv->txfsem, 0, config->ntxfifoq);

      dev->cd_ops  = &g_fdcanops;
      dev->cd_priv = (FAR void *)priv;

      /* And put the hardware in the initial state */

      fdcan_reset(dev);
    }

  return dev;
}

#endif /* CONFIG_CAN && CONFIG_STM32H7_FDCAN */

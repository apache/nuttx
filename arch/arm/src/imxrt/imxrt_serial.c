/****************************************************************************
 * arch/arm/src/imxrt/imxrt_serial.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/init.h>
#include <nuttx/power/pm.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "hardware/imxrt_lpuart.h"
#include "imxrt_gpio.h"
#include "imxrt_edma.h"
#include "hardware/imxrt_dmamux.h"
#include "hardware/imxrt_pinmux.h"
#include "imxrt_config.h"
#include "imxrt_lowputc.h"
#include "imxrt_serial.h"

#ifdef USE_SERIALDRIVER

/* The DMA buffer size when using RX DMA to emulate a FIFO.
 *
 * When streaming data, the generic serial layer will be called every time
 * the FIFO receives half this number of bytes.
 *
 * This buffer size should be an even multiple of the Cortex-M7 D-Cache line
 * size, ARMV7M_DCACHE_LINESIZE, so that it can be individually invalidated.
 *
 * Should there be a Cortex-M7 without a D-Cache, ARMV7M_DCACHE_LINESIZE
 * would be zero!
 */

#  if !defined(ARMV7M_DCACHE_LINESIZE) || ARMV7M_DCACHE_LINESIZE == 0
#    undef ARMV7M_DCACHE_LINESIZE
#    define ARMV7M_DCACHE_LINESIZE 32
#  endif

#  if !defined(CONFIG_IMXRT_SERIAL_RXDMA_BUFFER_SIZE) || \
      (CONFIG_IMXRT_SERIAL_RXDMA_BUFFER_SIZE < ARMV7M_DCACHE_LINESIZE)
#    undef CONFIG_IMXRT_SERIAL_RXDMA_BUFFER_SIZE
#    define CONFIG_IMXRT_SERIAL_RXDMA_BUFFER_SIZE ARMV7M_DCACHE_LINESIZE
#  endif

#  define RXDMA_BUFFER_MASK   (ARMV7M_DCACHE_LINESIZE - 1)
#  define RXDMA_BUFFER_SIZE   ((CONFIG_IMXRT_SERIAL_RXDMA_BUFFER_SIZE \
                                + RXDMA_BUFFER_MASK) & ~RXDMA_BUFFER_MASK)

/* The DMA buffer size when using TX DMA.
 *
 * This TX buffer size should be an even multiple of the Cortex-M7 D-Cache
 * line size, ARMV7M_DCACHE_LINESIZE, so that it can be individually
 * invalidated.
 *
 * Should there be a Cortex-M7 without a D-Cache, ARMV7M_DCACHE_LINESIZE
 * would be zero!
 */

#if !defined(ARMV7M_DCACHE_LINESIZE) || ARMV7M_DCACHE_LINESIZE == 0
#  undef ARMV7M_DCACHE_LINESIZE
#  define ARMV7M_DCACHE_LINESIZE 32
#endif

#define TXDMA_BUFFER_MASK   (ARMV7M_DCACHE_LINESIZE - 1)
#define TXDMA_BUFFER_SIZE   ((CONFIG_IMXRT_SERIAL_RXDMA_BUFFER_SIZE \
                              + RXDMA_BUFFER_MASK) & ~RXDMA_BUFFER_MASK)

/* If built with CONFIG_ARMV7M_DCACHE Buffers need to be aligned and
 * multiples of ARMV7M_DCACHE_LINESIZE
 */

#if defined(CONFIG_ARMV7M_DCACHE)
#  define TXDMA_BUF_SIZE(b) (((b) + TXDMA_BUFFER_MASK) & ~TXDMA_BUFFER_MASK)
#  define TXDMA_BUF_ALIGN   aligned_data(ARMV7M_DCACHE_LINESIZE);
#else
#  define TXDMA_BUF_SIZE(b)  (b)
#  define TXDMA_BUF_ALIGN
#endif

#if !defined(CONFIG_LPUART1_TXDMA)
#  define LPUART1_TXBUFSIZE_ADJUSTED  CONFIG_LPUART1_TXBUFSIZE
#  define LPUART1_TXBUFSIZE_ALGN
#else
#  define LPUART1_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART1_TXBUFSIZE)
#  define LPUART1_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

#if !defined(CONFIG_LPUART2_TXDMA)
#  define LPUART2_TXBUFSIZE_ADJUSTED  CONFIG_LPUART2_TXBUFSIZE
#  define LPUART2_TXBUFSIZE_ALGN
#else
#  define LPUART2_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART2_TXBUFSIZE)
#  define LPUART2_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

#if !defined(CONFIG_LPUART3_TXDMA)
#  define LPUART3_TXBUFSIZE_ADJUSTED  CONFIG_LPUART3_TXBUFSIZE
#  define LPUART3_TXBUFSIZE_ALGN
#else
#  define LPUART3_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART3_TXBUFSIZE)
#  define LPUART3_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

#if !defined(CONFIG_LPUART4_TXDMA)
#  define LPUART4_TXBUFSIZE_ADJUSTED  CONFIG_LPUART4_TXBUFSIZE
#  define LPUART4_TXBUFSIZE_ALGN
#else
#  define LPUART4_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART4_TXBUFSIZE)
#  define LPUART4_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

#if !defined(CONFIG_LPUART5_TXDMA)
#  define LPUART5_TXBUFSIZE_ADJUSTED  CONFIG_LPUART5_TXBUFSIZE
#  define LPUART5_TXBUFSIZE_ALGN
#else
#  define LPUART5_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART5_TXBUFSIZE)
#  define LPUART5_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

#if !defined(CONFIG_LPUART6_TXDMA)
#  define LPUART6_TXBUFSIZE_ADJUSTED  CONFIG_LPUART6_TXBUFSIZE
#  define LPUART6_TXBUFSIZE_ALGN
#else
#  define LPUART6_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART6_TXBUFSIZE)
#  define LPUART6_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

#if !defined(CONFIG_LPUART7_TXDMA)
#  define LPUART7_TXBUFSIZE_ADJUSTED  CONFIG_LPUART7_TXBUFSIZE
#  define LPUART7_TXBUFSIZE_ALGN
#else
#  define LPUART7_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART7_TXBUFSIZE)
#  define LPUART7_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

#if !defined(CONFIG_LPUART8_TXDMA)
#  define LPUART8_TXBUFSIZE_ADJUSTED  CONFIG_LPUART8_TXBUFSIZE
#  define LPUART8_TXBUFSIZE_ALGN
#else
#  define LPUART8_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART8_TXBUFSIZE)
#  define LPUART8_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

/* Which LPUART with be tty0/console and which tty1-7?  The console will
 * always be ttyS0.  If there is no console then will use the lowest
 * numbered UART.
 */

/* First pick the console and ttys0.  This could be any of LPUART1-8 */

#if defined(CONFIG_LPUART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart1priv /* LPUART1 is console */
#  define TTYS0_DEV           g_lpuart1priv /* LPUART1 is ttyS0 */
#  define LPUART1_ASSIGNED    1
#  if defined(CONFIG_LPUART1_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART1_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart2priv /* LPUART2 is console */
#  define TTYS0_DEV           g_lpuart2priv /* LPUART2 is ttyS0 */
#  define LPUART2_ASSIGNED    1
#  if defined(CONFIG_LPUART2_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART2_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART3_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart3priv /* LPUART3 is console */
#  define TTYS0_DEV           g_lpuart3priv /* LPUART3 is ttyS0 */
#  define LPUART3_ASSIGNED    1
#  if defined(CONFIG_LPUART3_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART3_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART4_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart4priv /* LPUART4 is console */
#  define TTYS0_DEV           g_lpuart4priv /* LPUART4 is ttyS0 */
#  define LPUART4_ASSIGNED    1
#  if defined(CONFIG_LPUART4_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART4_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART5_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart5priv /* LPUART5 is console */
#  define TTYS0_DEV           g_lpuart5priv /* LPUART5 is ttyS0 */
#  define LPUART5_ASSIGNED    1
#  if defined(CONFIG_LPUART5_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART5_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART6_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart6priv /* LPUART6 is console */
#  define TTYS0_DEV           g_lpuart6priv /* LPUART6 is ttyS0 */
#  define LPUART6_ASSIGNED    1
#  if defined(CONFIG_LPUART6_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART6_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART7_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart7priv /* LPUART7 is console */
#  define TTYS0_DEV           g_lpuart7priv /* LPUART7 is ttyS0 */
#  define LPUART7_ASSIGNED    1
#  if defined(CONFIG_LPUART7_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART7_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART8_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart8priv /* LPUART8 is console */
#  define TTYS0_DEV           g_lpuart8priv /* LPUART8 is ttyS0 */
#  define LPUART8_ASSIGNED    1
#  if defined(CONFIG_LPUART8_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART9_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#else
#  undef CONSOLE_DEV                      /* No console */
#  if defined(CONFIG_IMXRT_LPUART1)
#    define TTYS0_DEV         g_lpuart1priv /* LPUART1 is ttyS0 */
#    define UART1_ASSIGNED    1
#  elif defined(CONFIG_IMXRT_LPUART2)
#    define TTYS0_DEV         g_lpuart2priv /* LPUART2 is ttyS0 */
#    define UART2_ASSIGNED    1
#  elif defined(CONFIG_IMXRT_LPUART3)
#    define TTYS0_DEV         g_lpuart3priv /* LPUART3 is ttyS0 */
#    define UART3_ASSIGNED    1
#  elif defined(CONFIG_IMXRT_LPUART4)
#    define TTYS0_DEV         g_lpuart4priv /* LPUART4 is ttyS0 */
#    define UART4_ASSIGNED    1
#  elif defined(CONFIG_IMXRT_LPUART5)
#    define TTYS0_DEV         g_lpuart5priv /* LPUART5 is ttyS0 */
#    define UART5_ASSIGNED    1
#  elif defined(CONFIG_IMXRT_LPUART6)
#    define TTYS0_DEV         g_lpuart6priv /* LPUART6 is ttyS0 */
#    define UART6_ASSIGNED    1
#  elif defined(CONFIG_IMXRT_LPUART7)
#    define TTYS0_DEV         g_lpuart7priv /* LPUART7 is ttyS0 */
#    define UART7_ASSIGNED    1
#  elif defined(CONFIG_IMXRT_LPUART8)
#    define TTYS0_DEV         g_lpuart8priv /* LPUART8 is ttyS0 */
#    define UART8_ASSIGNED    1
#  endif
#endif

#if defined(SERIAL_HAVE_CONSOLE_RXDMA) || defined(SERIAL_HAVE_CONSOLE_TXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA
#endif

/* Pick ttys1.  This could be any of UART1-8 excluding the console UART.
 * One of UART1-8 could be the console; one of UART1-8 has already been
 * assigned to ttys0.
 */

#if defined(CONFIG_IMXRT_LPUART1) && !defined(LPUART1_ASSIGNED)
#  define TTYS1_DEV           g_lpuart1priv /* LPUART1 is ttyS1 */
#  define LPUART1_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART2) && !defined(LPUART2_ASSIGNED)
#  define TTYS1_DEV           g_lpuart2priv /* LPUART2 is ttyS1 */
#  define LPUART2_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART3) && !defined(LPUART3_ASSIGNED)
#  define TTYS1_DEV           g_lpuart3priv /* LPUART3 is ttyS1 */
#  define LPUART3_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART4) && !defined(LPUART4_ASSIGNED)
#  define TTYS1_DEV           g_lpuart4priv /* LPUART4 is ttyS1 */
#  define LPUART4_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART5) && !defined(LPUART5_ASSIGNED)
#  define TTYS1_DEV           g_lpuart5priv /* LPUART5 is ttyS1 */
#  define LPUART5_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART6) && !defined(LPUART6_ASSIGNED)
#  define TTYS1_DEV           g_lpuart6priv /* LPUART6 is ttyS1 */
#  define LPUART6_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART7) && !defined(LPUART7_ASSIGNED)
#  define TTYS1_DEV           g_lpuart7priv /* LPUART7 is ttyS1 */
#  define LPUART7_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART8) && !defined(LPUART8_ASSIGNED)
#  define TTYS1_DEV           g_lpuart8priv /* LPUART8 is ttyS1 */
#  define LPUART8_ASSIGNED    1
#endif

/* Pick ttys2.  This could be one of UART2-8. It can't be UART1 because that
 * was either assigned as ttyS0 or ttys1.  One of UART 1-8 could be the
 * console.  One of UART2-8 has already been assigned to ttys0 or ttyS1.
 */

#if defined(CONFIG_IMXRT_LPUART2) && !defined(LPUART2_ASSIGNED)
#  define TTYS2_DEV           g_lpuart2priv /* LPUART2 is ttyS2 */
#  define LPUART2_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART3) && !defined(LPUART3_ASSIGNED)
#  define TTYS2_DEV           g_lpuart3priv /* LPUART3 is ttyS2 */
#  define LPUART3_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART4) && !defined(LPUART4_ASSIGNED)
#  define TTYS2_DEV           g_lpuart4priv /* LPUART4 is ttyS2 */
#  define LPUART4_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART5) && !defined(LPUART5_ASSIGNED)
#  define TTYS2_DEV           g_lpuart5priv /* LPUART5 is ttyS2 */
#  define LPUART5_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART6) && !defined(LPUART6_ASSIGNED)
#  define TTYS2_DEV           g_lpuart6priv /* LPUART6 is ttyS2 */
#  define LPUART6_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART7) && !defined(LPUART7_ASSIGNED)
#  define TTYS2_DEV           g_lpuart7priv /* LPUART7 is ttyS2 */
#  define LPUART7_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART8) && !defined(LPUART8_ASSIGNED)
#  define TTYS2_DEV           g_lpuart8priv /* LPUART8 is ttyS2 */
#  define LPUART8_ASSIGNED    1
#endif

/* Pick ttys3. This could be one of UART3-8. It can't be UART1-2 because
 * those have already been assigned to ttsyS0, 1, or 2.  One of
 * UART3-8 could also be the console.  One of UART3-8 has already
 * been assigned to ttys0, 1, or 3.
 */

#if defined(CONFIG_IMXRT_LPUART3) && !defined(LPUART3_ASSIGNED)
#  define TTYS3_DEV           g_lpuart3priv /* LPUART3 is ttyS3 */
#  define LPUART3_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART4) && !defined(LPUART4_ASSIGNED)
#  define TTYS3_DEV           g_lpuart4priv /* LPUART4 is ttyS3 */
#  define LPUART4_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART5) && !defined(LPUART5_ASSIGNED)
#  define TTYS3_DEV           g_lpuart5priv /* LPUART5 is ttyS3 */
#  define LPUART5_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART6) && !defined(LPUART6_ASSIGNED)
#  define TTYS3_DEV           g_lpuart6priv /* LPUART6 is ttyS3 */
#  define LPUART6_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART7) && !defined(LPUART7_ASSIGNED)
#  define TTYS3_DEV           g_lpuart7priv /* LPUART7 is ttyS3 */
#  define LPUART7_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART8) && !defined(LPUART8_ASSIGNED)
#  define TTYS3_DEV           g_lpuart8priv /* LPUART8 is ttyS3 */
#  define LPUART8_ASSIGNED    1
#endif

/* Pick ttys4. This could be one of UART4-8. It can't be UART1-3 because
 * those have already been assigned to ttsyS0, 1, 2 or 3.  One of
 * UART 4-8 could be the console.  One of UART4-8 has already been
 * assigned to ttys0, 1, 3, or 4.
 */

#if defined(CONFIG_IMXRT_LPUART4) && !defined(LPUART4_ASSIGNED)
#  define TTYS4_DEV           g_lpuart4priv /* LPUART4 is ttyS4 */
#  define LPUART4_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART5) && !defined(LPUART5_ASSIGNED)
#  define TTYS4_DEV           g_lpuart5priv /* LPUART5 is ttyS4 */
#  define LPUART5_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART6) && !defined(LPUART6_ASSIGNED)
#  define TTYS4_DEV           g_lpuart6priv /* LPUART6 is ttyS4 */
#  define LPUART6_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART7) && !defined(LPUART7_ASSIGNED)
#  define TTYS4_DEV           g_lpuart7priv /* LPUART7 is ttyS4 */
#  define LPUART7_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART8) && !defined(LPUART8_ASSIGNED)
#  define TTYS4_DEV           g_lpuart8priv /* LPUART8 is ttyS4 */
#  define LPUART8_ASSIGNED    1
#endif

/* Pick ttys5. This could be one of UART5-8. It can't be UART1-4 because
 * those have already been assigned to ttsyS0, 1, 2, 3 or 4.  One of
 * UART 5-8 could be the console.  One of UART5-8 has already been
 * assigned to ttys0, 1, 2, 3, or 4.
 */

#if defined(CONFIG_IMXRT_LPUART5) && !defined(LPUART5_ASSIGNED)
#  define TTYS5_DEV           g_lpuart5priv /* LPUART5 is ttyS5 */
#  define LPUART5_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART6) && !defined(LPUART6_ASSIGNED)
#  define TTYS5_DEV           g_lpuart6priv /* LPUART6 is ttyS5 */
#  define LPUART6_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART7) && !defined(LPUART7_ASSIGNED)
#  define TTYS5_DEV           g_lpuart7priv /* LPUART7 is ttyS5 */
#  define LPUART7_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART8) && !defined(LPUART8_ASSIGNED)
#  define TTYS5_DEV           g_lpuart8priv /* LPUART8 is ttyS5 */
#  define LPUART8_ASSIGNED    1
#endif

/* Pick ttys6. This could be one of UART6-8. It can't be UART1-5 because
 * those have already been assigned to ttsyS0, 1, 2, 3, 4 or 5.  One of
 * UART 6-8 could be the console.  One of UART6-8 has already been
 * assigned to ttys0, 1, 2, 3, 4 or 5.
 */

#if defined(CONFIG_IMXRT_LPUART6) && !defined(LPUART6_ASSIGNED)
#  define TTYS6_DEV           g_lpuart6priv /* LPUART6 is ttyS5 */
#  define LPUART6_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART7) && !defined(LPUART7_ASSIGNED)
#  define TTYS6_DEV           g_lpuart7priv /* LPUART7 is ttyS5 */
#  define LPUART7_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART8) && !defined(LPUART8_ASSIGNED)
#  define TTYS6_DEV           g_lpuart8priv /* LPUART8 is ttyS5 */
#  define LPUART8_ASSIGNED    1
#endif

/* Pick ttys7. This could be one of UART7-8. It can't be UART1-6 because
 * those have already been assigned to ttsyS0, 1, 2, 3, 4, 5 or 6.  One of
 * UART 7-8 could be the console.  One of UART7-8 has already been
 * assigned to ttys0, 1, 2, 3, 4, 5 or 6.
 */

#if defined(CONFIG_IMXRT_LPUART7) && !defined(LPUART7_ASSIGNED)
#  define TTYS7_DEV           g_lpuart7priv /* LPUART7 is ttyS5 */
#  define LPUART7_ASSIGNED    1
#elif defined(CONFIG_IMXRT_LPUART8) && !defined(LPUART8_ASSIGNED)
#  define TTYS7_DEV           g_lpuart8port /* LPUART8 is ttyS5 */
#  define LPUART8_ASSIGNED    1
#endif

/* UART, if available, should have been assigned to ttyS0-7. */

/* Power management definitions */

#if defined(CONFIG_PM) && !defined(CONFIG_IMXRT_PM_SERIAL_ACTIVITY)
#  define CONFIG_IMXRT_PM_SERIAL_ACTIVITY 10
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imxrt_uart_s
{
  struct uart_dev_s dev;    /* Generic UART device */
  uint32_t uartbase;        /* Base address of UART registers */
  uint32_t baud;            /* Configured baud */
  uint32_t ie;              /* Saved enabled interrupts */
  uint8_t  irq;             /* IRQ associated with this UART */
  uint8_t  parity;          /* 0=none, 1=odd, 2=even */
  uint8_t  bits;            /* Number of bits (7 or 8) */
#if defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL)
  uint8_t  inviflow:1;      /* Invert RTS sense */
  const uint32_t rts_gpio;  /* LPUART RTS GPIO pin configuration */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  const uint32_t cts_gpio;  /* LPUART CTS GPIO pin configuration */
#endif
#ifdef CONFIG_IMXRT_LPUART_SINGLEWIRE
  const uint32_t tx_gpio;   /* TX GPIO pin configuration */
#endif

  uint8_t  stopbits2:1;     /* 1: Configure with 2 stop bits vs 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  uint8_t  iflow:1;         /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  uint8_t  oflow:1;         /* output flow control (CTS) enabled */
#endif
#ifdef CONFIG_SERIAL_RS485CONTROL
  uint8_t rs485mode:1;      /* We are in RS485 (RTS on TX) mode */
#endif
  /* TX DMA state */

#ifdef SERIAL_HAVE_TXDMA
  const unsigned int dma_txreqsrc;  /* DMAMUX source of TX DMA request */
  DMACH_HANDLE       txdma;         /* currently-open trasnmit DMA stream */
  sem_t              txdmasem;      /* Indicate TX DMA completion */
#endif

  /* RX DMA state */

#ifdef SERIAL_HAVE_RXDMA
  const unsigned int dma_rxreqsrc;  /* DMAMUX source of RX DMA request */
  DMACH_HANDLE       rxdma;         /* currently-open receive DMA stream */
  bool               rxenable;      /* DMA-based reception en/disable */
  uint32_t           rxdmanext;     /* Next byte in the DMA buffer to be read */
  char *const        rxfifo;        /* Receive DMA buffer */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t imxrt_serialin(struct imxrt_uart_s *priv,
                                      uint32_t offset);
static inline void imxrt_serialout(struct imxrt_uart_s *priv,
                                   uint32_t offset, uint32_t value);
static inline void imxrt_disableuartint(struct imxrt_uart_s *priv,
                                        uint32_t *ie);
static inline void imxrt_restoreuartint(struct imxrt_uart_s *priv,
                                        uint32_t ie);

static int  imxrt_setup(struct uart_dev_s *dev);
static void imxrt_shutdown(struct uart_dev_s *dev);
static int  imxrt_attach(struct uart_dev_s *dev);
static void imxrt_detach(struct uart_dev_s *dev);
static int  imxrt_interrupt(int irq, void *context, void *arg);
static int  imxrt_ioctl(struct file *filep, int cmd, unsigned long arg);
#if !defined(SERIAL_HAVE_ONLY_RXDMA)
static int  imxrt_receive(struct uart_dev_s *dev, unsigned int *status);
static void imxrt_rxint(struct uart_dev_s *dev, bool enable);
static bool imxrt_rxavailable(struct uart_dev_s *dev);
#endif
#if !defined(SERIAL_HAVE_ONLY_TXDMA)
static void imxrt_txint(struct uart_dev_s *dev, bool enable);
#endif

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool imxrt_rxflowcontrol(struct uart_dev_s *dev,
                                unsigned int nbuffered, bool upper);
#endif
static void imxrt_send(struct uart_dev_s *dev, int ch);

static bool imxrt_txready(struct uart_dev_s *dev);

#ifdef SERIAL_HAVE_TXDMA
static void imxrt_dma_send(struct uart_dev_s *dev);
static void imxrt_dma_txint(struct uart_dev_s *dev, bool enable);
static void imxrt_dma_txavailable(struct uart_dev_s *dev);
static void imxrt_dma_txcallback(DMACH_HANDLE handle, void *arg, bool done,
                                  int result);
#endif

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
static int  imxrt_dma_setup(struct uart_dev_s *dev);
static void imxrt_dma_shutdown(struct uart_dev_s *dev);
#endif

#ifdef SERIAL_HAVE_RXDMA
static int  imxrt_dma_receive(struct uart_dev_s *dev, unsigned int *status);
#ifdef CONFIG_PM
static void imxrt_dma_reenable(struct imxrt_uart_s *priv);
#endif
static void imxrt_dma_rxint(struct uart_dev_s *dev, bool enable);
static bool imxrt_dma_rxavailable(struct uart_dev_s *dev);

static void imxrt_dma_rxcallback(DMACH_HANDLE handle, void *arg, bool done,
                                  int result);
#endif

static bool imxrt_txempty(struct uart_dev_s *dev);

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int dowmin,
                         enum pm_state_e pmstate);
static int  up_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serial driver UART operations */

#if !defined(SERIAL_HAVE_ONLY_TXDMA) && !defined(SERIAL_HAVE_ONLY_RXDMA)
static const struct uart_ops_s g_lpuart_ops =
{
  .setup          = imxrt_setup,
  .shutdown       = imxrt_shutdown,
  .attach         = imxrt_attach,
  .detach         = imxrt_detach,
  .ioctl          = imxrt_ioctl,
  .receive        = imxrt_receive,
  .rxint          = imxrt_rxint,
  .rxavailable    = imxrt_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = imxrt_rxflowcontrol,
#endif
  .send           = imxrt_send,
  .txint          = imxrt_txint,
  .txready        = imxrt_txready,
  .txempty        = imxrt_txempty,
};
#endif

#if defined(SERIAL_HAVE_RXDMA) && defined(SERIAL_HAVE_TXDMA)
static const struct uart_ops_s g_lpuart_rxtxdma_ops =
{
  .setup          = imxrt_dma_setup,
  .shutdown       = imxrt_dma_shutdown,
  .attach         = imxrt_attach,
  .detach         = imxrt_detach,
  .ioctl          = imxrt_ioctl,
  .receive        = imxrt_dma_receive,
  .rxint          = imxrt_dma_rxint,
  .rxavailable    = imxrt_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = imxrt_rxflowcontrol,
#endif
  .send           = imxrt_send,
  .txint          = imxrt_dma_txint,
  .txready        = imxrt_txready,
  .txempty        = imxrt_txempty,
  .dmatxavail     = imxrt_dma_txavailable,
  .dmasend        = imxrt_dma_send,
};
#endif
#if !defined(SERIAL_HAVE_ONLY_DMA) && defined(SERIAL_HAVE_RXDMA)
static const struct uart_ops_s g_lpuart_rxdma_ops =
{
  .setup          = imxrt_dma_setup,
  .shutdown       = imxrt_dma_shutdown,
  .attach         = imxrt_attach,
  .detach         = imxrt_detach,
  .ioctl          = imxrt_ioctl,
  .receive        = imxrt_dma_receive,
  .rxint          = imxrt_dma_rxint,
  .rxavailable    = imxrt_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = imxrt_rxflowcontrol,
#endif
  .send           = imxrt_send,
  .txint          = imxrt_txint,
  .txready        = imxrt_txready,
  .txempty        = imxrt_txempty,
};
#endif

#if !defined(SERIAL_HAVE_ONLY_DMA) && defined(SERIAL_HAVE_TXDMA)
static const struct uart_ops_s g_lpuart_txdma_ops =
{
    .setup          = imxrt_dma_setup,
    .shutdown       = imxrt_dma_shutdown,
    .attach         = imxrt_attach,
    .detach         = imxrt_detach,
    .ioctl          = imxrt_ioctl,
    .receive        = imxrt_receive,
    .rxint          = imxrt_rxint,
    .rxavailable    = imxrt_rxavailable,
  #ifdef CONFIG_SERIAL_IFLOWCONTROL
    .rxflowcontrol  = imxrt_rxflowcontrol,
  #endif
    .send           = imxrt_send,
    .txint          = imxrt_dma_txint,
    .txready        = imxrt_txready,
    .txempty        = imxrt_txempty,
    .dmatxavail     = imxrt_dma_txavailable,
    .dmasend        = imxrt_dma_send,
};
#endif

/* Avoid unused warning */
#if !defined(SERIAL_HAVE_ONLY_DMA) && defined(SERIAL_HAVE_RXDMA)
const struct uart_ops_s *g_o0 = &g_lpuart_rxdma_ops;
#endif
#if !defined(SERIAL_HAVE_ONLY_DMA) && defined(SERIAL_HAVE_TXDMA)
const struct uart_ops_s *g_o1 = &g_lpuart_txdma_ops;
#endif

/* I/O buffers */

#ifdef CONFIG_LPUART1_RXDMA
static char g_lpuart1rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
#endif

# ifdef CONFIG_LPUART2_RXDMA
static char g_lpuart2rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
#endif

#ifdef CONFIG_LPUART3_RXDMA
static char g_lpuart3rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
#endif

#ifdef CONFIG_LPUART4_RXDMA
static char g_lpuart4rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
#endif

#ifdef CONFIG_LPUART5_RXDMA
static char g_lpuart5rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
#endif

#ifdef CONFIG_LPUART6_RXDMA
static char g_lpuart6rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
#endif

#ifdef CONFIG_LPUART7_RXDMA
static char g_lpuart7rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
#endif

#ifdef CONFIG_LPUART8_RXDMA
static char g_lpuart8rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
#endif

/* Receive/Transmit buffers */

#ifdef CONFIG_IMXRT_LPUART1
static char g_lpuart1rxbuffer[CONFIG_LPUART1_RXBUFSIZE];
static char g_lpuart1txbuffer[LPUART1_TXBUFSIZE_ADJUSTED] \
  LPUART1_TXBUFSIZE_ALGN;
#endif

#ifdef CONFIG_IMXRT_LPUART2
static char g_lpuart2rxbuffer[CONFIG_LPUART2_RXBUFSIZE];
static char g_lpuart2txbuffer[LPUART2_TXBUFSIZE_ADJUSTED] \
  LPUART2_TXBUFSIZE_ALGN;
#endif

#ifdef CONFIG_IMXRT_LPUART3
static char g_lpuart3rxbuffer[CONFIG_LPUART3_RXBUFSIZE];
static char g_lpuart3txbuffer[LPUART3_TXBUFSIZE_ADJUSTED] \
  LPUART3_TXBUFSIZE_ALGN;
#endif

#ifdef CONFIG_IMXRT_LPUART4
static char g_lpuart4rxbuffer[CONFIG_LPUART4_RXBUFSIZE];
static char g_lpuart4txbuffer[LPUART4_TXBUFSIZE_ADJUSTED] \
  LPUART4_TXBUFSIZE_ALGN;
#endif

#ifdef CONFIG_IMXRT_LPUART5
static char g_lpuart5rxbuffer[CONFIG_LPUART5_RXBUFSIZE];
static char g_lpuart5txbuffer[LPUART5_TXBUFSIZE_ADJUSTED] \
  LPUART5_TXBUFSIZE_ALGN;
#endif

#ifdef CONFIG_IMXRT_LPUART6
static char g_lpuart6rxbuffer[CONFIG_LPUART6_RXBUFSIZE];
static char g_lpuart6txbuffer[LPUART6_TXBUFSIZE_ADJUSTED] \
  LPUART6_TXBUFSIZE_ALGN;
#endif

#ifdef CONFIG_IMXRT_LPUART7
static char g_lpuart7rxbuffer[CONFIG_LPUART7_RXBUFSIZE];
static char g_lpuart7txbuffer[LPUART7_TXBUFSIZE_ADJUSTED] \
  LPUART7_TXBUFSIZE_ALGN;
#endif

#ifdef CONFIG_IMXRT_LPUART8
static char g_lpuart8rxbuffer[CONFIG_LPUART8_RXBUFSIZE];
static char g_lpuart8txbuffer[LPUART8_TXBUFSIZE_ADJUSTED] \
  LPUART8_TXBUFSIZE_ALGN;
#endif

/* This describes the state of the IMXRT lpuart1 port. */

#ifdef CONFIG_IMXRT_LPUART1
static struct imxrt_uart_s g_lpuart1priv =
{
  .dev =
    {
      .recv         =
      {
        .size       = CONFIG_LPUART1_RXBUFSIZE,
        .buffer     = g_lpuart1rxbuffer,
      },
      .xmit         =
      {
        .size       = CONFIG_LPUART1_TXBUFSIZE,
        .buffer     = g_lpuart1txbuffer,
      },
    #if defined(CONFIG_LPUART1_RXDMA) && defined(CONFIG_LPUART1_TXDMA)
        .ops       = &g_lpuart_rxtxdma_ops,
    #elif defined(CONFIG_LPUART1_RXDMA) && !defined(CONFIG_LPUART1_TXDMA)
        .ops       = &g_lpuart_rxdma_ops,
    #elif !defined(CONFIG_LPUART1_RXDMA) && defined(CONFIG_LPUART1_TXDMA)
        .ops       = &g_lpuart_txdma_ops,
    #else
        .ops       = &g_lpuart_ops,
    #endif
      .priv         = &g_lpuart1priv,
    },

  .uartbase     = IMXRT_LPUART1_BASE,
  .baud         = CONFIG_LPUART1_BAUD,
  .irq          = IMXRT_IRQ_LPUART1,
  .parity       = CONFIG_LPUART1_PARITY,
  .bits         = CONFIG_LPUART1_BITS,
  .stopbits2    = CONFIG_LPUART1_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART1_OFLOWCONTROL)
  .oflow        = 1,
  .cts_gpio     = GPIO_LPUART1_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART1_IFLOWCONTROL)
  .iflow        = 1,
#endif
# if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART1_RS485RTSCONTROL)) \
   || (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART1_IFLOWCONTROL)))
  .rts_gpio     = GPIO_LPUART1_RTS,
#endif
#ifdef CONFIG_IMXRT_LPUART_SINGLEWIRE
  .tx_gpio      = GPIO_LPUART1_TX,
#endif

#if (((defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL))) \
    && defined(CONFIG_LPUART1_INVERTIFLOWCONTROL))
  .inviflow     = 1,
#endif

#if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART1_RS485RTSCONTROL)
  .rs485mode    = 1,
#endif

#ifdef CONFIG_LPUART1_TXDMA
  .dma_txreqsrc = IMXRT_DMACHAN_LPUART1_TX,
  .txdmasem     = SEM_INITIALIZER(1),
#endif
#ifdef CONFIG_LPUART1_RXDMA
  .dma_rxreqsrc = IMXRT_DMACHAN_LPUART1_RX,
  .rxfifo        = g_lpuart1rxfifo,
#endif
};
#endif

/* This describes the state of the IMXRT lpuart2 port. */

#ifdef CONFIG_IMXRT_LPUART2
static struct imxrt_uart_s g_lpuart2priv =
{
    .dev =
      {
        .recv         =
        {
          .size       = CONFIG_LPUART2_RXBUFSIZE,
          .buffer     = g_lpuart2rxbuffer,
        },
        .xmit         =
        {
          .size       = CONFIG_LPUART2_TXBUFSIZE,
          .buffer     = g_lpuart2txbuffer,
        },
      #if defined(CONFIG_LPUART2_RXDMA) && defined(CONFIG_LPUART2_TXDMA)
          .ops       = &g_lpuart_rxtxdma_ops,
      #elif defined(CONFIG_LPUART2_RXDMA) && !defined(CONFIG_LPUART2_TXDMA)
          .ops       = &g_lpuart_rxdma_ops,
      #elif !defined(CONFIG_LPUART2_RXDMA) && defined(CONFIG_LPUART2_TXDMA)
          .ops       = &g_lpuart_txdma_ops,
      #else
          .ops       = &g_lpuart_ops,
      #endif
        .priv         = &g_lpuart2priv,
      },

  .uartbase     = IMXRT_LPUART2_BASE,
  .baud         = CONFIG_LPUART2_BAUD,
  .irq          = IMXRT_IRQ_LPUART2,
  .parity       = CONFIG_LPUART2_PARITY,
  .bits         = CONFIG_LPUART2_BITS,
  .stopbits2    = CONFIG_LPUART2_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART2_OFLOWCONTROL)
  .oflow        = 1,
  .cts_gpio     = GPIO_LPUART2_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART2_IFLOWCONTROL)
  .iflow        = 1,
#endif
# if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART2_RS485RTSCONTROL)) \
   || (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART2_IFLOWCONTROL)))
  .rts_gpio     = GPIO_LPUART2_RTS,
#endif
#ifdef CONFIG_IMXRT_LPUART_SINGLEWIRE
  .tx_gpio      = GPIO_LPUART2_TX,
#endif
#if (((defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL))) \
    && defined(CONFIG_LPUART2_INVERTIFLOWCONTROL))
  .inviflow     = 1,
#endif

#if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART2_RS485RTSCONTROL)
  .rs485mode    = 1,
#endif

#ifdef CONFIG_LPUART2_TXDMA
  .dma_txreqsrc = IMXRT_DMACHAN_LPUART2_TX,
  .txdmasem     = SEM_INITIALIZER(1),
#endif
#ifdef CONFIG_LPUART2_RXDMA
  .dma_rxreqsrc = IMXRT_DMACHAN_LPUART2_RX,
  .rxfifo        = g_lpuart2rxfifo,
#endif
};
#endif

#ifdef CONFIG_IMXRT_LPUART3
static struct imxrt_uart_s g_lpuart3priv =
{
  .dev =
    {
      .recv         =
      {
        .size       = CONFIG_LPUART3_RXBUFSIZE,
        .buffer     = g_lpuart3rxbuffer,
      },
      .xmit         =
      {
        .size       = CONFIG_LPUART3_TXBUFSIZE,
        .buffer     = g_lpuart3txbuffer,
      },
    #if defined(CONFIG_LPUART3_RXDMA) && defined(CONFIG_LPUART3_TXDMA)
        .ops       = &g_lpuart_rxtxdma_ops,
    #elif defined(CONFIG_LPUART3_RXDMA) && !defined(CONFIG_LPUART3_TXDMA)
        .ops       = &g_lpuart_rxdma_ops,
    #elif !defined(CONFIG_LPUART3_RXDMA) && defined(CONFIG_LPUART3_TXDMA)
        .ops       = &g_lpuart_txdma_ops,
    #else
        .ops       = &g_lpuart_ops,
    #endif
      .priv         = &g_lpuart3priv,
    },

  .uartbase     = IMXRT_LPUART3_BASE,
  .baud         = CONFIG_LPUART3_BAUD,
  .irq          = IMXRT_IRQ_LPUART3,
  .parity       = CONFIG_LPUART3_PARITY,
  .bits         = CONFIG_LPUART3_BITS,
  .stopbits2    = CONFIG_LPUART3_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART3_OFLOWCONTROL)
  .oflow        = 1,
  .cts_gpio     = GPIO_LPUART3_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART3_IFLOWCONTROL)
  .iflow        = 1,
#endif
# if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART3_RS485RTSCONTROL)) \
   || (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART3_IFLOWCONTROL)))
  .rts_gpio     = GPIO_LPUART3_RTS,
#endif
#ifdef CONFIG_IMXRT_LPUART_SINGLEWIRE
  .tx_gpio      = GPIO_LPUART3_TX,
#endif
#if (((defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL))) \
    && defined(CONFIG_LPUART3_INVERTIFLOWCONTROL))
  .inviflow     = 1,
#endif

#if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART3_RS485RTSCONTROL)
  .rs485mode    = 1,
#endif

#ifdef CONFIG_LPUART3_TXDMA
  .dma_txreqsrc = IMXRT_DMACHAN_LPUART3_TX,
  .txdmasem     = SEM_INITIALIZER(1),
#endif
#ifdef CONFIG_LPUART3_RXDMA
  .dma_rxreqsrc = IMXRT_DMACHAN_LPUART3_RX,
  .rxfifo        = g_lpuart3rxfifo,
#endif
};
#endif

#ifdef CONFIG_IMXRT_LPUART4
static struct imxrt_uart_s g_lpuart4priv =
{
  .dev =
    {
      .recv         =
      {
        .size       = CONFIG_LPUART4_RXBUFSIZE,
        .buffer     = g_lpuart4rxbuffer,
      },
      .xmit         =
      {
        .size       = CONFIG_LPUART4_TXBUFSIZE,
        .buffer     = g_lpuart4txbuffer,
      },
    #if defined(CONFIG_LPUART4_RXDMA) && defined(CONFIG_LPUART4_TXDMA)
        .ops       = &g_lpuart_rxtxdma_ops,
    #elif defined(CONFIG_LPUART4_RXDMA) && !defined(CONFIG_LPUART4_TXDMA)
        .ops       = &g_lpuart_rxdma_ops,
    #elif !defined(CONFIG_LPUART4_RXDMA) && defined(CONFIG_LPUART4_TXDMA)
        .ops       = &g_lpuart_txdma_ops,
    #else
        .ops       = &g_lpuart_ops,
    #endif
      .priv         = &g_lpuart4priv,
    },

  .uartbase     = IMXRT_LPUART4_BASE,
  .baud         = CONFIG_LPUART4_BAUD,
  .irq          = IMXRT_IRQ_LPUART4,
  .parity       = CONFIG_LPUART4_PARITY,
  .bits         = CONFIG_LPUART4_BITS,
  .stopbits2    = CONFIG_LPUART4_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART4_OFLOWCONTROL)
  .oflow        = 1,
  .cts_gpio     = GPIO_LPUART4_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART4_IFLOWCONTROL)
  .iflow        = 1,
#endif
# if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART4_RS485RTSCONTROL)) \
   || (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART4_IFLOWCONTROL)))
  .rts_gpio     = GPIO_LPUART4_RTS,
#endif
#ifdef CONFIG_IMXRT_LPUART_SINGLEWIRE
  .tx_gpio      = GPIO_LPUART4_TX,
#endif
#if (((defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL))) \
    && defined(CONFIG_LPUART4_INVERTIFLOWCONTROL))
  .inviflow     = 1,
#endif

#if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART4_RS485RTSCONTROL)
  .rs485mode    = 1,
#endif

#ifdef CONFIG_LPUART4_TXDMA
  .dma_txreqsrc = IMXRT_DMACHAN_LPUART4_TX,
  .txdmasem     = SEM_INITIALIZER(1),
#endif
#ifdef CONFIG_LPUART4_RXDMA
  .dma_rxreqsrc = IMXRT_DMACHAN_LPUART4_RX,
  .rxfifo        = g_lpuart4rxfifo,
#endif
};
#endif

#ifdef CONFIG_IMXRT_LPUART5
static struct imxrt_uart_s g_lpuart5priv =
{
  .dev =
    {
      .recv         =
        {
          .size       = CONFIG_LPUART5_RXBUFSIZE,
          .buffer     = g_lpuart5rxbuffer,
        },
        .xmit         =
        {
          .size       = CONFIG_LPUART5_TXBUFSIZE,
          .buffer     = g_lpuart5txbuffer,
        },
      #if defined(CONFIG_LPUART5_RXDMA) && defined(CONFIG_LPUART5_TXDMA)
          .ops       = &g_lpuart_rxtxdma_ops,
      #elif defined(CONFIG_LPUART5_RXDMA) && !defined(CONFIG_LPUART5_TXDMA)
          .ops       = &g_lpuart_rxdma_ops,
      #elif !defined(CONFIG_LPUART5_RXDMA) && defined(CONFIG_LPUART5_TXDMA)
          .ops       = &g_lpuart_txdma_ops,
      #else
          .ops       = &g_lpuart_ops,
      #endif
        .priv         = &g_lpuart5priv,
    },

  .uartbase     = IMXRT_LPUART5_BASE,
  .baud         = CONFIG_LPUART5_BAUD,
  .irq          = IMXRT_IRQ_LPUART5,
  .parity       = CONFIG_LPUART5_PARITY,
  .bits         = CONFIG_LPUART5_BITS,
  .stopbits2    = CONFIG_LPUART5_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART5_OFLOWCONTROL)
  .oflow        = 1,
  .cts_gpio     = GPIO_LPUART5_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART5_IFLOWCONTROL)
  .iflow        = 1,
#endif
# if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART5_RS485RTSCONTROL)) \
   || (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART5_IFLOWCONTROL)))
  .rts_gpio     = GPIO_LPUART5_RTS,
#endif
#ifdef CONFIG_IMXRT_LPUART_SINGLEWIRE
  .tx_gpio      = GPIO_LPUART5_TX,
#endif
#if (((defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL))) \
    && defined(CONFIG_LPUART5_INVERTIFLOWCONTROL))
  .inviflow     = 1,
#endif

#if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART5_RS485RTSCONTROL)
  .rs485mode    = 1,
#endif

#ifdef CONFIG_LPUART5_TXDMA
  .dma_txreqsrc = IMXRT_DMACHAN_LPUART5_TX,
  .txdmasem     = SEM_INITIALIZER(1),
#endif
#ifdef CONFIG_LPUART5_RXDMA
  .dma_rxreqsrc = IMXRT_DMACHAN_LPUART5_RX,
  .rxfifo        = g_lpuart5rxfifo,
#endif
};
#endif

#ifdef CONFIG_IMXRT_LPUART6
static struct imxrt_uart_s g_lpuart6priv =
{
  .dev =
    {
      .recv         =
      {
        .size       = CONFIG_LPUART6_RXBUFSIZE,
        .buffer     = g_lpuart6rxbuffer,
      },
      .xmit         =
      {
        .size       = CONFIG_LPUART6_TXBUFSIZE,
        .buffer     = g_lpuart6txbuffer,
      },
    #if defined(CONFIG_LPUART6_RXDMA) && defined(CONFIG_LPUART6_TXDMA)
        .ops       = &g_lpuart_rxtxdma_ops,
    #elif defined(CONFIG_LPUART6_RXDMA) && !defined(CONFIG_LPUART6_TXDMA)
        .ops       = &g_lpuart_rxdma_ops,
    #elif !defined(CONFIG_LPUART6_RXDMA) && defined(CONFIG_LPUART6_TXDMA)
        .ops       = &g_lpuart_txdma_ops,
    #else
        .ops       = &g_lpuart_ops,
    #endif
      .priv         = &g_lpuart6priv,
    },

  .uartbase     = IMXRT_LPUART6_BASE,
  .baud         = CONFIG_LPUART6_BAUD,
  .irq          = IMXRT_IRQ_LPUART6,
  .parity       = CONFIG_LPUART6_PARITY,
  .bits         = CONFIG_LPUART6_BITS,
  .stopbits2    = CONFIG_LPUART6_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART6_OFLOWCONTROL)
  .oflow        = 1,
  .cts_gpio     = GPIO_LPUART6_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART6_IFLOWCONTROL)
  .iflow        = 1,
#endif
# if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART6_RS485RTSCONTROL)) \
   || (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART6_IFLOWCONTROL)))
  .rts_gpio     = GPIO_LPUART6_RTS,
#endif
#ifdef CONFIG_IMXRT_LPUART_SINGLEWIRE
  .tx_gpio      = GPIO_LPUART6_TX,
#endif
#if (((defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL))) \
    && defined(CONFIG_LPUART6_INVERTIFLOWCONTROL))
  .inviflow     = 1,
#endif

#if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART6_RS485RTSCONTROL)
  .rs485mode    = 1,
#endif

#ifdef CONFIG_LPUART6_TXDMA
  .dma_txreqsrc = IMXRT_DMACHAN_LPUART6_TX,
  .txdmasem     = SEM_INITIALIZER(1),
#endif
#ifdef CONFIG_LPUART6_RXDMA
  .dma_rxreqsrc = IMXRT_DMACHAN_LPUART6_RX,
  .rxfifo        = g_lpuart6rxfifo,
#endif
};
#endif

#ifdef CONFIG_IMXRT_LPUART7
static struct imxrt_uart_s g_lpuart7priv =
{
  .dev =
    {
      .recv         =
      {
        .size       = CONFIG_LPUART7_RXBUFSIZE,
        .buffer     = g_lpuart7rxbuffer,
      },
      .xmit         =
      {
        .size       = CONFIG_LPUART7_TXBUFSIZE,
        .buffer     = g_lpuart7txbuffer,
      },
    #if defined(CONFIG_LPUART7_RXDMA) && defined(CONFIG_LPUART7_TXDMA)
        .ops       = &g_lpuart_rxtxdma_ops,
    #elif defined(CONFIG_LPUART7_RXDMA) && !defined(CONFIG_LPUART7_TXDMA)
        .ops       = &g_lpuart_rxdma_ops,
    #elif !defined(CONFIG_LPUART7_RXDMA) && defined(CONFIG_LPUART7_TXDMA)
        .ops       = &g_lpuart_txdma_ops,
    #else
        .ops       = &g_lpuart_ops,
    #endif
      .priv         = &g_lpuart7priv,
    },

  .uartbase     = IMXRT_LPUART7_BASE,
  .baud         = CONFIG_LPUART7_BAUD,
  .irq          = IMXRT_IRQ_LPUART7,
  .parity       = CONFIG_LPUART7_PARITY,
  .bits         = CONFIG_LPUART7_BITS,
  .stopbits2    = CONFIG_LPUART7_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART7_OFLOWCONTROL)
  .oflow        = 1,
  .cts_gpio     = GPIO_LPUART7_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART7_IFLOWCONTROL)
  .iflow        = 1,
#endif
# if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART7_RS485RTSCONTROL)) \
   || (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART7_IFLOWCONTROL)))
  .rts_gpio     = GPIO_LPUART7_RTS,
#endif
#ifdef CONFIG_IMXRT_LPUART_SINGLEWIRE
  .tx_gpio      = GPIO_LPUART7_TX,
#endif
#if (((defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL))) \
    && defined(CONFIG_LPUART7_INVERTIFLOWCONTROL))
  .inviflow     = 1,
#endif

#if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART7_RS485RTSCONTROL)
  .rs485mode    = 1,
#endif

#ifdef CONFIG_LPUART7_TXDMA
  .dma_txreqsrc = IMXRT_DMACHAN_LPUART7_TX,
  .txdmasem     = SEM_INITIALIZER(1),
#endif
#ifdef CONFIG_LPUART7_RXDMA
  .dma_rxreqsrc = IMXRT_DMACHAN_LPUART7_RX,
  .rxfifo        = g_lpuart7rxfifo,
#endif
};
#endif

#ifdef CONFIG_IMXRT_LPUART8
static struct imxrt_uart_s g_lpuart8priv =
{
  .dev =
    {
      .recv         =
      {
        .size       = CONFIG_LPUART8_RXBUFSIZE,
        .buffer     = g_lpuart8rxbuffer,
      },
      .xmit         =
      {
        .size       = CONFIG_LPUART8_TXBUFSIZE,
        .buffer     = g_lpuart8txbuffer,
      },
    #if defined(CONFIG_LPUART8_RXDMA) && defined(CONFIG_LPUART8_TXDMA)
        .ops       = &g_lpuart_rxtxdma_ops,
    #elif defined(CONFIG_LPUART8_RXDMA) && !defined(CONFIG_LPUART8_TXDMA)
        .ops       = &g_lpuart_rxdma_ops,
    #elif !defined(CONFIG_LPUART8_RXDMA) && defined(CONFIG_LPUART8_TXDMA)
        .ops       = &g_lpuart_txdma_ops,
    #else
        .ops       = &g_lpuart_ops,
    #endif
      .priv         = &g_lpuart8priv,
    },

  .uartbase     = IMXRT_LPUART8_BASE,
  .baud         = CONFIG_LPUART8_BAUD,
  .irq          = IMXRT_IRQ_LPUART8,
  .parity       = CONFIG_LPUART8_PARITY,
  .bits         = CONFIG_LPUART8_BITS,
  .stopbits2    = CONFIG_LPUART8_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART8_OFLOWCONTROL)
  .oflow        = 1,
  .cts_gpio     = GPIO_LPUART8_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART8_IFLOWCONTROL)
  .iflow        = 1,
#endif
# if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART8_RS485RTSCONTROL)) \
   || (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART8_IFLOWCONTROL)))
  .rts_gpio     = GPIO_LPUART8_RTS,
#endif
#ifdef CONFIG_IMXRT_LPUART_SINGLEWIRE
  .tx_gpio      = GPIO_LPUART8_TX,
#endif
#if (((defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL))) \
    && defined(CONFIG_LPUART8_INVERTIFLOWCONTROL))
  .inviflow     = 1,
#endif

#if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART8_RS485RTSCONTROL)
  .rs485mode    = 1,
#endif

#ifdef CONFIG_LPUART8_TXDMA
  .dma_txreqsrc = IMXRT_DMACHAN_LPUART8_TX,
  .txdmasem     = SEM_INITIALIZER(1),
#endif
#ifdef CONFIG_LPUART8_RXDMA
  .dma_rxreqsrc = IMXRT_DMACHAN_LPUART8_RX,
  .rxfifo        = g_lpuart8rxfifo,
#endif
};
#endif

#ifdef CONFIG_PM
static  struct pm_callback_s g_serial_pmcb =
{
  .notify       = up_pm_notify,
  .prepare      = up_pm_prepare,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_serialin
 ****************************************************************************/

static inline uint32_t imxrt_serialin(struct imxrt_uart_s *priv,
                                      uint32_t offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: imxrt_serialout
 ****************************************************************************/

static inline void imxrt_serialout(struct imxrt_uart_s *priv,
                                   uint32_t offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: imxrt_dma_nextrx
 *
 * Description:
 *   Returns the index into the RX FIFO where the DMA will place the next
 *   byte that it receives.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int imxrt_dma_nextrx(struct imxrt_uart_s *priv)
{
  int dmaresidual = imxrt_dmach_getcount(priv->rxdma);

  return RXDMA_BUFFER_SIZE - dmaresidual;
}
#endif

/****************************************************************************
 * Name: imxrt_disableuartint
 ****************************************************************************/

static inline void imxrt_disableuartint(struct imxrt_uart_s *priv,
                                        uint32_t *ie)
{
  irqstate_t flags;
  uint32_t regval;

  flags  = spin_lock_irqsave(NULL);
  regval = imxrt_serialin(priv, IMXRT_LPUART_CTRL_OFFSET);

  /* Return the current Rx and Tx interrupt state */

  if (ie != NULL)
    {
      *ie = regval & LPUART_ALL_INTS;
    }

  regval &= ~LPUART_ALL_INTS;
  imxrt_serialout(priv, IMXRT_LPUART_CTRL_OFFSET, regval);
  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name: imxrt_restoreuartint
 ****************************************************************************/

static inline void imxrt_restoreuartint(struct imxrt_uart_s *priv,
                                        uint32_t ie)
{
  irqstate_t flags;
  uint32_t regval;

  /* Enable/disable any interrupts that are currently disabled but should be
   * enabled/disabled.
   */

  flags   = spin_lock_irqsave(NULL);
  regval  = imxrt_serialin(priv, IMXRT_LPUART_CTRL_OFFSET);
  regval &= ~LPUART_ALL_INTS;
  regval |= ie;
  imxrt_serialout(priv, IMXRT_LPUART_CTRL_OFFSET, regval);
  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name: imxrt_dma_setup
 *
 * Description:
 *   Configure the LPUART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
static int imxrt_dma_setup(struct uart_dev_s *dev)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;
#if defined(SERIAL_HAVE_RXDMA)
  struct imxrt_edma_xfrconfig_s config;
#endif
  int result;

  /* Do the basic UART setup first, unless we are the console */

  if (!dev->isconsole)
    {
      result = imxrt_setup(dev);
      if (result != OK)
        {
          return result;
        }
    }

#if defined(SERIAL_HAVE_TXDMA)
  /* Acquire the Tx DMA channel.  This should always succeed. */

  if (priv->dma_txreqsrc != 0)
    {
      if (priv->txdma == NULL)
        {
          priv->txdma = imxrt_dmach_alloc(priv->dma_txreqsrc |
                                          DMAMUX_CHCFG_ENBL, 0);
          if (priv->txdma == NULL)
            {
              return -EBUSY;
            }
        }

      /* Enable Tx DMA for the UART */

      modifyreg32(priv->uartbase + IMXRT_LPUART_BAUD_OFFSET,
                  0, LPUART_BAUD_TDMAE);
    }
#endif

#if defined(SERIAL_HAVE_RXDMA)
  /* Acquire the Rx DMA channel.  This should always succeed. */

  if (priv->dma_rxreqsrc != 0)
    {
      if (priv->rxdma == NULL)
        {
          priv->rxdma = imxrt_dmach_alloc(priv->dma_rxreqsrc |
                                          DMAMUX_CHCFG_ENBL, 0);

          if (priv->rxdma == NULL)
            {
              return -EBUSY;
            }
        }
      else
        {
          imxrt_dmach_stop(priv->rxdma);
        }

      /* Configure for circular DMA reception into the RX FIFO */

      config.saddr  = priv->uartbase + IMXRT_LPUART_DATA_OFFSET;
      config.daddr  = (uint32_t) priv->rxfifo;
      config.soff   = 0;
      config.doff   = 1;
      config.iter   = RXDMA_BUFFER_SIZE;
      config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE |
                      EDMA_CONFIG_LOOPDEST |
                      EDMA_CONFIG_INTHALF  |
                      EDMA_CONFIG_INTMAJOR;
      config.ssize  = EDMA_8BIT;
      config.dsize  = EDMA_8BIT;
      config.nbytes = 1;
    #ifdef CONFIG_KINETIS_EDMA_ELINK
      config.linkch = 0;
    #endif

      imxrt_dmach_xfrsetup(priv->rxdma , &config);

      /* Reset our DMA shadow pointer and Rx data availability count to
       * match the address just programmed above.
       */

      priv->rxdmanext = 0;

      /* Enable receive Rx DMA for the UART */

      modifyreg32(priv->uartbase + IMXRT_LPUART_BAUD_OFFSET,
                  0, LPUART_BAUD_RDMAE);

      /* Enable itnerrupt on Idel and erros */

      modifyreg32(priv->uartbase + IMXRT_LPUART_CTRL_OFFSET, 0,
                  LPUART_CTRL_PEIE       |
                  LPUART_CTRL_FEIE       |
                  LPUART_CTRL_NEIE       |
                  LPUART_CTRL_ILIE);

      /* Start the DMA channel, and arrange for callbacks at the half and
       * full points in the FIFO.  This ensures that we have half a FIFO
       * worth of time to claim bytes before they are overwritten.
       */

      imxrt_dmach_start(priv->rxdma, imxrt_dma_rxcallback, (void *)priv);
    }
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: imxrt_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int imxrt_setup(struct uart_dev_s *dev)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;
#ifndef CONFIG_SUPPRESS_LPUART_CONFIG
  int ret;
  struct uart_config_s config =
  {
    0
  };

  /* Configure the UART */

  config.baud       = priv->baud;       /* Configured baud */
  config.parity     = priv->parity;     /* 0=none, 1=odd, 2=even */
  config.bits       = priv->bits;       /* Number of bits (5-9) */
  config.stopbits2  = priv->stopbits2;  /* true: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  config.usects     = priv->iflow;      /* Flow control on inbound side */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  /* Flow control on outbound side if not GPIO based */

  if ((priv->rts_gpio & GPIO_MODE_MASK) == GPIO_PERIPH)
    {
      config.userts = priv->oflow;
    }

#endif
#ifdef CONFIG_SERIAL_RS485CONTROL
  config.users485   = priv->rs485mode;  /* Switch into RS485 mode */
#endif
#if defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL)
  config.invrts     = priv->inviflow;   /* Inversion of outbound flow control */
#endif

  ret = imxrt_lpuart_configure(priv->uartbase, &config);

  priv->ie = imxrt_serialin(priv, IMXRT_LPUART_CTRL_OFFSET) & \
             LPUART_ALL_INTS;
  return ret;

#else
  priv->ie = imxrt_serialin(priv, IMXRT_LPUART_CTRL_OFFSET) & \
             LPUART_ALL_INTS;
  return OK;
#endif
}

/****************************************************************************
 * Name: imxrt_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void imxrt_shutdown(struct uart_dev_s *dev)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;

  /* Disable the UART */

  imxrt_serialout(priv, IMXRT_LPUART_GLOBAL_OFFSET, LPUART_GLOBAL_RST);
}

/****************************************************************************
 * Name: imxrt_dma_shutdown
 *
 * Description:
 *   Disable the LPUART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
static void imxrt_dma_shutdown(struct uart_dev_s *dev)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;

  /* Perform the normal UART shutdown */

  imxrt_shutdown(dev);

#if defined(SERIAL_HAVE_RXDMA)
  /* Stop the RX DMA channel */

  if (priv->dma_rxreqsrc != 0)
    {
      imxrt_dmach_stop(priv->rxdma);

      /* Release the RX DMA channel */

      imxrt_dmach_free(priv->rxdma);
      priv->rxdma = NULL;
    }
#endif

#if defined(SERIAL_HAVE_TXDMA)
  /* Stop the TX DMA channel */

  if (priv->dma_txreqsrc != 0)
    {
      imxrt_dmach_stop(priv->txdma);

      /* Release the TX DMA channel */

      imxrt_dmach_free(priv->txdma);
      priv->txdma = NULL;
    }
#endif
}
#endif

/****************************************************************************
 * Name: imxrt_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate
 *   in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int imxrt_attach(struct uart_dev_s *dev)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, imxrt_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: imxrt_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void imxrt_detach(struct uart_dev_s *dev)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: imxrt_interrupt (and front-ends)
 *
 * Description:
 *   This is the common UART interrupt handler.  It should call
 *   uart_transmitchars or uart_receivechar to perform the appropriate data
 *   transfers.
 *
 ****************************************************************************/

static int imxrt_interrupt(int irq, void *context, void *arg)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)arg;
  uint32_t usr;
  int passes = 0;
  bool handled;

  DEBUGASSERT(priv != NULL);

#if defined(CONFIG_PM) && CONFIG_IMXRT_PM_SERIAL_ACTIVITY > 0
  /* Report serial activity to the power management logic */

  pm_activity(PM_IDLE_DOMAIN, CONFIG_IMXRT_PM_SERIAL_ACTIVITY);
#endif

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the current UART status and check for loop
       * termination conditions
       */

      usr  = imxrt_serialin(priv, IMXRT_LPUART_STAT_OFFSET);
      usr &= (LPUART_STAT_RDRF | LPUART_STAT_TDRE | LPUART_STAT_OR |
              LPUART_STAT_FE | LPUART_STAT_NF | LPUART_STAT_PF |
              LPUART_STAT_IDLE);

      /* Clear serial overrun, parity and framing errors */

      if ((usr & LPUART_STAT_OR) != 0)
        {
          imxrt_serialout(priv, IMXRT_LPUART_STAT_OFFSET, LPUART_STAT_OR);
        }

      if ((usr & LPUART_STAT_PF) != 0)
        {
          imxrt_serialout(priv, IMXRT_LPUART_STAT_OFFSET, LPUART_STAT_NF);
        }

      if ((usr & LPUART_STAT_PF) != 0)
        {
          imxrt_serialout(priv, IMXRT_LPUART_STAT_OFFSET, LPUART_STAT_PF);
        }

      if ((usr & LPUART_STAT_FE) != 0)
        {
          imxrt_serialout(priv, IMXRT_LPUART_STAT_OFFSET, LPUART_STAT_FE);
        }

      if ((usr & (LPUART_STAT_FE | LPUART_STAT_PF | LPUART_STAT_NF)) != 0)
        {
          /* Discard data */

          imxrt_serialin(priv, IMXRT_LPUART_DATA_OFFSET);
        }

#ifdef SERIAL_HAVE_RXDMA
      /* The line going to idle, deliver any fractions of RX data */

      if ((usr & LPUART_STAT_IDLE) != 0)
        {
          imxrt_serialout(priv, IMXRT_LPUART_STAT_OFFSET, LPUART_STAT_IDLE);
          imxrt_dma_rxcallback(priv->rxdma, priv, false, LPUART_STAT_IDLE);
        }
#endif

      /* Handle incoming, receive bytes */

      if ((usr & LPUART_STAT_RDRF) != 0 &&
          (priv->ie & LPUART_CTRL_RIE) != 0)
        {
          uart_recvchars(&priv->dev);
          handled = true;
        }

      /* Handle outgoing, transmit bytes */

      if ((usr & LPUART_STAT_TDRE) != 0 &&
          (priv->ie & LPUART_CTRL_TIE) != 0)
        {
          uart_xmitchars(&priv->dev);
          handled = true;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int imxrt_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TIOCSERGSTRUCT) || defined(CONFIG_SERIAL_TERMIOS)
  struct inode *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  irqstate_t flags;
#endif
  int ret   = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
         struct imxrt_uart_s *user = (struct imxrt_uart_s *)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct imxrt_uart_s));
           }
       }
       break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Return parity */

        termiosp->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                            ((priv->parity == 1) ? PARODD : 0);

        /* Return stop bits */

        termiosp->c_cflag |= (priv->stopbits2) ? CSTOPB : 0;

        /* Return flow control */

#ifdef CONFIG_SERIAL_OFLOWCONTROL
        termiosp->c_cflag |= ((priv->oflow) ? CCTS_OFLOW : 0);
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        termiosp->c_cflag |= ((priv->iflow) ? CRTS_IFLOW : 0);
#endif
        /* Return baud */

        cfsetispeed(termiosp, priv->baud);

        /* Return number of bits */

        switch (priv->bits)
          {
          case 5:
            termiosp->c_cflag |= CS5;
            break;

          case 6:
            termiosp->c_cflag |= CS6;
            break;

          case 7:
            termiosp->c_cflag |= CS7;
            break;

          default:
          case 8:
            termiosp->c_cflag |= CS8;
            break;

#if defined(CS9)
          case 9:
            termiosp->c_cflag |= CS9;
            break;
#endif
          }
      }
      break;

    case TCSETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;
        uint32_t baud;
        uint32_t ie;
        uint8_t parity;
        uint8_t nbits;
        bool stop2;

        if ((!termiosp)
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            || ((termiosp->c_cflag & CCTS_OFLOW) && (priv->cts_gpio == 0))
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            || ((termiosp->c_cflag & CRTS_IFLOW) && (priv->rts_gpio == 0))
#endif
           )
          {
            ret = -EINVAL;
            break;
          }

        /* Decode baud. */

        ret = OK;
        baud = cfgetispeed(termiosp);

        /* Decode number of bits */

        switch (termiosp->c_cflag & CSIZE)
          {
          case CS5:
            nbits = 5;
            break;

          case CS6:
            nbits = 6;
            break;

          case CS7:
            nbits = 7;
            break;

          case CS8:
            nbits = 8;
            break;

#if defined(CS9)
          case CS9:
            nbits = 9;
            break;
#endif
          default:
            ret = -EINVAL;
            break;
          }

        /* Decode parity */

        if ((termiosp->c_cflag & PARENB) != 0)
          {
            parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            parity = 0;
          }

        /* Decode stop bits */

        stop2 = (termiosp->c_cflag & CSTOPB) != 0;

        /* Verify that all settings are valid before committing */

        if (ret == OK)
          {
            /* Commit */

            priv->baud      = baud;
            priv->parity    = parity;
            priv->bits      = nbits;
            priv->stopbits2 = stop2;
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            priv->oflow     = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            priv->iflow     = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif
            /* effect the changes immediately - note that we do not
             * implement TCSADRAIN / TCSAFLUSH
             */

            flags  = spin_lock_irqsave(NULL);
            imxrt_disableuartint(priv, &ie);
            ret = dev->ops->setup(dev);

            /* Restore the interrupt state */

            imxrt_restoreuartint(priv, ie);
            priv->ie = ie;
            spin_unlock_irqrestore(NULL, flags);
          }
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

#ifdef CONFIG_IMXRT_LPUART_SINGLEWIRE
    case TIOCSSINGLEWIRE:
      {
        uint32_t regval;
        irqstate_t flags;
        struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;

        flags  = spin_lock_irqsave(NULL);
        regval   = imxrt_serialin(priv, IMXRT_LPUART_CTRL_OFFSET);

        if ((arg & SER_SINGLEWIRE_ENABLED) != 0)
          {
            uint32_t gpio_val = IOMUX_OPENDRAIN;
            gpio_val |= (arg & SER_SINGLEWIRE_PULL_MASK) ==
                         SER_SINGLEWIRE_PULLUP ?
                                        IOMUX_PULL_UP_47K : IOMUX_PULL_NONE;
            gpio_val |= (arg & SER_SINGLEWIRE_PULL_MASK) ==
                         SER_SINGLEWIRE_PULLDOWN ?
                                     IOMUX_PULL_DOWN_100K : IOMUX_PULL_NONE;
            imxrt_config_gpio((priv->tx_gpio &
                          ~(IOMUX_PULL_MASK | IOMUX_OPENDRAIN)) | gpio_val);
            regval |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;
          }
        else
          {
            imxrt_config_gpio((priv->tx_gpio & ~(IOMUX_PULL_MASK |
                                                 IOMUX_OPENDRAIN)) |
                                                 IOMUX_PULL_NONE);
            regval &= ~(LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC);
          }

        imxrt_serialout(priv, IMXRT_LPUART_CTRL_OFFSET, regval);

        spin_unlock_irqrestore(NULL, flags);
      }
      break;
#endif

#ifdef CONFIG_IMXRT_LPUART_INVERT
    case TIOCSINVERT:
      {
        uint32_t ctrl;
        uint32_t stat;
        uint32_t regval;
        irqstate_t flags;
        struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;

        flags  = spin_lock_irqsave(NULL);
        ctrl   = imxrt_serialin(priv, IMXRT_LPUART_CTRL_OFFSET);
        stat   = imxrt_serialin(priv, IMXRT_LPUART_STAT_OFFSET);
        regval = ctrl;

        /* {R|T}XINV bit field can only be written when the receiver
         * is disabled (RE=0).
         */

        regval &= ~LPUART_CTRL_RE;

        imxrt_serialout(priv, IMXRT_LPUART_CTRL_OFFSET, regval);

        /* Enable/disable signal inversion. */

        if (arg & SER_INVERT_ENABLED_RX)
          {
            stat |= LPUART_STAT_RXINV;
          }
        else
          {
            stat &= ~LPUART_STAT_RXINV;
          }

        if (arg & SER_INVERT_ENABLED_TX)
          {
            ctrl |= LPUART_CTRL_TXINV;
          }
        else
          {
            ctrl &= ~LPUART_CTRL_TXINV;
          }

        imxrt_serialout(priv, IMXRT_LPUART_STAT_OFFSET, stat);
        imxrt_serialout(priv, IMXRT_LPUART_CTRL_OFFSET, ctrl);

        spin_unlock_irqrestore(NULL, flags);
      }
      break;
#endif

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: imxrt_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_RXDMA
static int imxrt_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;
  uint32_t rxd;

  rxd     = imxrt_serialin(priv, IMXRT_LPUART_DATA_OFFSET);
  *status = rxd >> LPUART_DATA_STATUS_SHIFT;
  return (rxd & LPUART_DATA_MASK) >> LPUART_DATA_SHIFT;
}
#endif

/****************************************************************************
 * Name: imxrt_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_RXDMA
static void imxrt_rxint(struct uart_dev_s *dev, bool enable)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;
  irqstate_t flags;
  uint32_t regval;

  /* Enable interrupts for data available at Rx */

  flags = spin_lock_irqsave(NULL);
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ie |= LPUART_CTRL_RIE | LPUART_CTRL_FEIE | LPUART_CTRL_ORIE;
#endif
    }
  else
    {
      priv->ie &= ~(LPUART_CTRL_RIE | LPUART_CTRL_FEIE | LPUART_CTRL_ORIE);
    }

  regval  = imxrt_serialin(priv, IMXRT_LPUART_CTRL_OFFSET);
  regval &= ~LPUART_ALL_INTS;
  regval |= priv->ie;
  imxrt_serialout(priv, IMXRT_LPUART_CTRL_OFFSET, regval);
  spin_unlock_irqrestore(NULL, flags);
}
#endif

/****************************************************************************
 * Name: imxrt_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_RXDMA
static bool imxrt_rxavailable(struct uart_dev_s *dev)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;
  uint32_t regval;

  /* Return true is data is ready in the Rx FIFO */

  regval = imxrt_serialin(priv, IMXRT_LPUART_STAT_OFFSET);
  return ((regval & LPUART_STAT_RDRF) != 0);
}
#endif

/****************************************************************************
 * Name: imxrt_rxflowcontrol
 *
 * Description:
 *   Called when Rx buffer is full (or exceeds configured watermark levels
 *   if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is defined).
 *   Return true if UART activated RX flow control to block more incoming
 *   data
 *
 * Input Parameters:
 *   dev       - UART device instance
 *   nbuffered - the number of characters currently buffered
 *               (if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is
 *               not defined the value will be 0 for an empty buffer or the
 *               defined buffer size for a full buffer)
 *   upper     - true indicates the upper watermark was crossed where
 *               false indicates the lower watermark has been crossed
 *
 * Returned Value:
 *   true if RX flow control activated.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool imxrt_rxflowcontrol(struct uart_dev_s *dev,
                             unsigned int nbuffered, bool upper)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;
  bool use_swhs = false;

#if defined(CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS)
  use_swhs = (priv->rts_gpio & GPIO_MODE_MASK) == GPIO_OUTPUT;
#endif

  if (use_swhs && priv->iflow && (priv->rts_gpio != 0))
    {
      /* Assert/de-assert nRTS set it high resume/stop sending */

      imxrt_gpio_write(priv->rts_gpio, upper);

      if (upper)
        {
          /* With heavy Rx traffic, RXNE might be set and data pending.
           * Returning 'true' in such case would cause RXNE left unhandled
           * and causing interrupt storm. Sending end might be also be slow
           * to react on nRTS, and returning 'true' here would prevent
           * processing that data.
           *
           * Therefore, return 'false' so input data is still being processed
           * until sending end reacts on nRTS signal and stops sending more.
           */

          return false;
        }

      return upper;
    }
  else
    {
      /* Is the RX buffer full? */

      if (upper)
        {
          /* Disable Rx interrupt to prevent more data being from
           * peripheral.  When hardware RTS is enabled, this will
           * prevent more data from coming in.
           *
           * This function is only called when UART recv buffer is full,
           * that is: "dev->recv.head + 1 == dev->recv.tail".
           *
           * Logic in "uart_read" will automatically toggle Rx interrupts
           * when buffer is read empty and thus we do not have to re-
           * enable Rx interrupts.
           */

          uart_disablerxint(dev);
          return true;
        }

      /* No.. The RX buffer is empty */

      else
        {
          /* We might leave Rx interrupt disabled if full recv buffer was
           * read empty.  Enable Rx interrupt to make sure that more input is
           * received.
           */

          uart_enablerxint(dev);
        }
    }

  return false;
}
#endif

/****************************************************************************
 * Name: imxrt_dma_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the LPUART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int imxrt_dma_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;
  uint32_t nextrx = imxrt_dma_nextrx(priv);
  int c = 0;

  /* Check if more data is available */

  if (nextrx != priv->rxdmanext)
    {
      /* Now read from the DMA buffer */

      c = priv->rxfifo[priv->rxdmanext];

      priv->rxdmanext++;

      if (priv->rxdmanext == RXDMA_BUFFER_SIZE)
        {
          priv->rxdmanext = 0;
        }
    }

  /* NOTE:  If no data is available, then we would return NULL which is,
   * of course, valid binary data.  The protocol is that the upper half
   * driver must call imxrt_dma_rxavailable prior to calling this function to
   * assure that this never happens.
   */

  return c;
}
#endif

/****************************************************************************
 * Name: imxrt_dma_reenable
 *
 * Description:
 *   Call to re-enable RX DMA.
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_RXDMA) && defined(CONFIG_PM)
static void imxrt_dma_reenable(struct imxrt_uart_s *priv)
{
  struct imxrt_edma_xfrconfig_s config;

  /* Stop an reset the RX DMA */

  imxrt_dmach_stop(priv->rxdma);

  /* Configure for circular DMA reception into the RX FIFO */

  config.saddr  = priv->uartbase + IMXRT_LPUART_DATA_OFFSET;
  config.daddr  = (uint32_t) priv->rxfifo;
  config.soff   = 0;
  config.doff   = 1;
  config.iter   = RXDMA_BUFFER_SIZE;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE |
                  EDMA_CONFIG_LOOPDEST |
                  EDMA_CONFIG_INTHALF |
                  EDMA_CONFIG_INTMAJOR;
  config.ssize  = EDMA_8BIT;
  config.dsize  = EDMA_8BIT;
  config.nbytes = 1;
#ifdef CONFIG_KINETIS_EDMA_ELINK
  config.linkch = 0;
#endif

  imxrt_dmach_xfrsetup(priv->rxdma, &config);

  /* Reset our DMA shadow pointer and Rx data availability count to match
   * the address just programmed above.
   */

  priv->rxdmanext = 0;

  /* Start the DMA channel, and arrange for callbacks at the half and
   * full points in the FIFO.  This ensures that we have half a FIFO
   * worth of time to claim bytes before they are overwritten.
   */

  imxrt_dmach_start(priv->rxdma, imxrt_dma_rxcallback, (void *)priv);

  /* Clear DMA suspended flag. */

  priv->rxdmasusp  = false;
}
#endif

/****************************************************************************
 * Name: imxrt_dma_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void imxrt_dma_rxint(struct uart_dev_s *dev, bool enable)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;

  /* Enable/disable DMA reception.
   *
   * Note that it is not safe to check for available bytes and immediately
   * pass them to uart_recvchars as that could potentially recurse back
   * to us again.  Instead, bytes must wait until the next up_dma_poll or
   * DMA event.
   */

  priv->rxenable = enable;
}
#endif

/****************************************************************************
 * Name: imxrt_dma_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static bool imxrt_dma_rxavailable(struct uart_dev_s *dev)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;

  /* Compare our receive pointer to the current DMA pointer, if they
   * do not match, then there are bytes to be received.
   */

  return (imxrt_dma_nextrx(priv) != priv->rxdmanext);
}
#endif

/****************************************************************************
 * Name: imxrt_dma_txcallback
 *
 * Description:
 *   This function clears dma buffer at complete of DMA transfer and wakes up
 *   threads waiting for space in buffer.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void imxrt_dma_txcallback(DMACH_HANDLE handle, void *arg, bool done,
                                  int result)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)arg;
  /* Update 'nbytes' indicating number of bytes actually transferred by DMA.
   * This is important to free TX buffer space by 'uart_xmitchars_done'.
   */

  priv->dev.dmatx.nbytes = priv->dev.dmatx.length + priv->dev.dmatx.nlength;

  /* Adjust the pointers */

  uart_xmitchars_done(&priv->dev);

  /* Release waiter */

  nxsem_post(&priv->txdmasem);
}
#endif

/****************************************************************************
 * Name: imxrt_dma_txavailable
 *
 * Description:
 *        Informs DMA that Tx data is available and is ready for transfer.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void imxrt_dma_txavailable(struct uart_dev_s *dev)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;

  /* Only send when the DMA is idle */

  nxsem_wait(&priv->txdmasem);

  uart_xmitchars_dma(dev);
}
#endif

/****************************************************************************
 * Name: imxrt_dma_send
 *
 * Description:
 *   Called (usually) from the interrupt level to start DMA transfer.
 *   (Re-)Configures DMA Stream updating buffer and buffer length.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void imxrt_dma_send(struct uart_dev_s *dev)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;
  struct imxrt_edma_xfrconfig_s config;

  /* We need to stop DMA before reconfiguration */

  imxrt_dmach_stop(priv->txdma);

  /* Reset the number sent */

  dev->dmatx.nbytes = 0;

  /* Make use of setup function to update buffer and its length for next
   * transfer
   */

  config.iter   = dev->dmatx.length;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = EDMA_8BIT;
  config.dsize  = EDMA_8BIT;
  config.nbytes = sizeof(dev->dmatx.buffer[0]);
  config.saddr  = (uint32_t) dev->dmatx.buffer;
  config.daddr  = priv->uartbase + IMXRT_LPUART_DATA_OFFSET;
  config.soff   = sizeof(dev->dmatx.buffer[0]);
  config.doff   = 0;
#ifdef CONFIG_IMXRT_EDMA_ELINK
  config.linkch  = 0;
#endif

  /* Flush the contents of the TX buffer into physical memory */

  up_clean_dcache((uintptr_t)dev->dmatx.buffer,
                  (uintptr_t)dev->dmatx.buffer + dev->dmatx.length);

  /* Setup first half */

  imxrt_dmach_xfrsetup(priv->txdma, &config);

  /* Is this a split transfer? */

  if (dev->dmatx.nbuffer)
    {
      config.iter   = priv->dev.dmatx.nlength;
      config.saddr  = (uint32_t) priv->dev.dmatx.nbuffer;

      /* Flush the contents of the next TX buffer into physical memory */

      up_clean_dcache((uintptr_t)dev->dmatx.nbuffer,
                      (uintptr_t)dev->dmatx.nbuffer + dev->dmatx.nlength);

      imxrt_dmach_xfrsetup(priv->txdma, &config);
    }

  /* Start transmission with the callback on DMA completion */

  imxrt_dmach_start(priv->txdma, imxrt_dma_txcallback, (void *)priv);
}
#endif

/****************************************************************************
 * Name: imxrt_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void imxrt_send(struct uart_dev_s *dev, int ch)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;
  imxrt_serialout(priv, IMXRT_LPUART_DATA_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: imxrt_dma_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts from the UART.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void imxrt_dma_txint(struct uart_dev_s *dev, bool enable)
{
  /* Nothing to do. */

  /* In case of DMA transfer we do not want to make use of UART interrupts.
   * Instead, we use DMA interrupts that are activated once during boot
   * sequence. Furthermore we can use imxrt_dma_txcallback() to handle stuff
   * at half DMA transfer or after transfer completion (depending on the
   *  configuration).
   */
}
#endif

/****************************************************************************
 * Name: imxrt_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

#if !defined(SERIAL_HAVE_ONLY_TXDMA)
static void imxrt_txint(struct uart_dev_s *dev, bool enable)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;
  irqstate_t flags;
  uint32_t regval;

  /* Enable interrupt for TX complete */

  flags = spin_lock_irqsave(NULL);
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ie |= LPUART_CTRL_TIE;
#endif
    }
  else
    {
      priv->ie &= ~LPUART_CTRL_TIE;
    }

  regval  = imxrt_serialin(priv, IMXRT_LPUART_CTRL_OFFSET);
  regval &= ~LPUART_ALL_INTS;
  regval |= priv->ie;
  imxrt_serialout(priv, IMXRT_LPUART_CTRL_OFFSET, regval);
  spin_unlock_irqrestore(NULL, flags);
}
#endif

/****************************************************************************
 * Name: imxrt_txready
 *
 * Description:
 *   Return true if the transmit register is available to be written to
 *
 ****************************************************************************/

static bool imxrt_txready(struct uart_dev_s *dev)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;
  uint32_t regval;

  regval = imxrt_serialin(priv, IMXRT_LPUART_STAT_OFFSET);
  return ((regval & LPUART_STAT_TDRE) != 0);
}

/****************************************************************************
 * Name: imxrt_txempty
 *
 * Description:
 *   Return true if the transmission has completed and been sent to line.
 *
 ****************************************************************************/

static bool imxrt_txempty(struct uart_dev_s *dev)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)dev;
  uint32_t regval;

  regval = imxrt_serialin(priv, IMXRT_LPUART_STAT_OFFSET);
  return ((regval & LPUART_STAT_TC) != 0);
}

/****************************************************************************
 * Name: imxrt_dma_rxcallback
 *
 * Description:
 *   This function checks the current DMA state and calls the generic
 *   serial stack when bytes appear to be available.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void imxrt_dma_rxcallback(DMACH_HANDLE handle, void *arg, bool done,
                                  int result)
{
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)arg;
  uint32_t sr;

  up_invalidate_dcache((uintptr_t)priv->rxfifo,
                       (uintptr_t)priv->rxfifo + RXDMA_BUFFER_SIZE);

  if (priv->rxenable && imxrt_dma_rxavailable(&priv->dev))
    {
      uart_recvchars(&priv->dev);
    }

  /* Get the masked LPUART status word to check and clear error flags.
   *
   * When wake-up from low power mode was not fast enough, UART is resumed
   * too late and sometimes exactly when character was coming over UART,
   * resulting to frame error.
   * If error flag is not cleared, Rx DMA will be stuck. Clearing errors
   * will release Rx DMA.
   */

  sr = imxrt_serialin(priv, IMXRT_LPUART_STAT_OFFSET);

  if ((sr & (LPUART_STAT_OR | LPUART_STAT_NF | LPUART_STAT_FE)) != 0)
    {
      imxrt_serialout(priv, IMXRT_LPUART_STAT_OFFSET,
                      sr & (LPUART_STAT_OR |
                            LPUART_STAT_NF |
                            LPUART_STAT_FE));
    }
}
#endif

/****************************************************************************
 * Name: up_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is  called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   None - The driver already agreed to transition to the low power
 *   consumption state when when it returned OK to the prepare() call.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case(PM_NORMAL):
        {
          /* Logic for PM_NORMAL goes here */
        }
        break;

      case(PM_IDLE):
        {
          /* Logic for PM_IDLE goes here */
        }
        break;

      case(PM_STANDBY):
        {
          /* Logic for PM_STANDBY goes here */
        }
        break;

      case(PM_SLEEP):
        {
          /* Logic for PM_SLEEP goes here */
        }
        break;

      default:

        /* Should not get here */

        break;
    }
}
#endif

/****************************************************************************
 * Name: up_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   Zero - (OK) means the event was successfully processed and that the
 *          driver is prepared for the PM state change.
 *
 *   Non-zero - means that the driver is not prepared to perform the tasks
 *              needed achieve this power setting and will cause the state
 *              change to be aborted. NOTE: The prepare() method will also
 *              be called when reverting from lower back to higher power
 *              consumption modes (say because another driver refused a
 *              lower power state change). Drivers are not permitted to
 *              return non-zero values when reverting back to higher power
 *              consumption modes!
 *
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int up_pm_prepare(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  /* Logic to prepare for a reduced power state goes here. */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

void imxrt_earlyserialinit(void)
{
  /* NOTE: This function assumes that low level hardware configuration
   * -- including all clocking and pin configuration -- was performed by the
   * function imxrt_lowsetup() earlier in the boot sequence.
   */

  /* Enable the console UART.  The other UARTs will be initialized if and
   * when they are first opened.
   */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.dev.isconsole = true;
  imxrt_setup(&CONSOLE_DEV.dev);
#endif
}

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that imxrt_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
#ifdef CONFIG_PM
  int ret;

  /* Register to receive power management callbacks */

  ret = pm_register(&g_serial_pmcb);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
#endif

#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV.dev);
#if defined(SERIAL_HAVE_CONSOLE_DMA)
  imxrt_dma_setup(&CONSOLE_DEV.dev);
#endif
#endif

  /* Register all UARTs */

  uart_register("/dev/ttyS0", &TTYS0_DEV.dev);
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV.dev);
#endif
#ifdef TTYS2_DEV
  uart_register("/dev/ttyS2", &TTYS2_DEV.dev);
#endif
#ifdef TTYS3_DEV
  uart_register("/dev/ttyS3", &TTYS3_DEV.dev);
#endif
#ifdef TTYS4_DEV
  uart_register("/dev/ttyS4", &TTYS4_DEV.dev);
#endif
#ifdef TTYS5_DEV
  uart_register("/dev/ttyS5", &TTYS5_DEV.dev);
#endif
#ifdef TTYS6_DEV
  uart_register("/dev/ttyS6", &TTYS6_DEV.dev);
#endif
#ifdef TTYS7_DEV
  uart_register("/dev/ttyS7", &TTYS7_DEV.dev);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef CONSOLE_DEV
  struct imxrt_uart_s *priv = (struct imxrt_uart_s *)CONSOLE_DEV.dev.priv;
  uint32_t ie;

  imxrt_disableuartint(priv, &ie);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      imxrt_lowputc('\r');
    }

  imxrt_lowputc(ch);
  imxrt_restoreuartint(priv, ie);
#endif

  return ch;
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#if CONSOLE_LPUART > 0
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
#endif

  return ch;
}

#endif /* USE_SERIALDRIVER */

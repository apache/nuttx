/****************************************************************************
 * arch/arm/src/stm32/stm32_fdcan.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *s
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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock source *************************************************************/

#define FDCANCLK_PDIV              (0)

#if FDCANCLK_PDIV == 0
#  define STM32_FDCANCLK_FREQUENCY (STM32_FDCAN_FREQUENCY / (1))
#else
#  define STM32_FDCANCLK_FREQUENCY (STM32_FDCAN_FREQUENCY / (2 * FDCANCLK_PDIV))
#endif

/* General Configuration ****************************************************/

#if defined(CONFIG_STM32_STM32G4XXX)

/* FDCAN Message RAM */

#  define FDCAN_MSGRAM_WORDS         (212)
#  define STM32_CANRAM1_BASE         (STM32_CANRAM_BASE + 0x0000)
#  define STM32_CANRAM2_BASE         (STM32_CANRAM_BASE + 1*(FDCAN_MSGRAM_WORDS * 4) + 4)
#  define STM32_CANRAM3_BASE         (STM32_CANRAM_BASE + 2*(FDCAN_MSGRAM_WORDS * 4) + 4)

#  ifdef CONFIG_STM32_FDCAN1
#    define FDCAN1_STDFILTER_SIZE    (28)
#    define FDCAN1_EXTFILTER_SIZE    (8)
#    define FDCAN1_RXFIFO0_SIZE      (3)
#    define FDCAN1_RXFIFO1_SIZE      (3)
#    define FDCAN1_TXEVENTFIFO_SIZE  (3)
#    define FDCAN1_TXFIFIOQ_SIZE     (3)

#    define FDCAN1_STDFILTER_WORDS   (28)
#    define FDCAN1_EXTFILTER_WORDS   (16)
#    define FDCAN1_RXFIFO0_WORDS     (54)
#    define FDCAN1_RXFIFO1_WORDS     (54)
#    define FDCAN1_TXEVENTFIFO_WORDS (6)
#    define FDCAN1_TXFIFIOQ_WORDS    (54)
#  endif
#  ifdef CONFIG_STM32_FDCAN2
#    define FDCAN2_STDFILTER_SIZE    (28)
#    define FDCAN2_EXTFILTER_SIZE    (8)
#    define FDCAN2_RXFIFO0_SIZE      (3)
#    define FDCAN2_RXFIFO1_SIZE      (3)
#    define FDCAN2_TXEVENTFIFO_SIZE  (3)
#    define FDCAN2_TXFIFIOQ_SIZE     (3)

#    define FDCAN2_STDFILTER_WORDS   (28)
#    define FDCAN2_EXTFILTER_WORDS   (16)
#    define FDCAN2_RXFIFO0_WORDS     (54)
#    define FDCAN2_RXFIFO1_WORDS     (54)
#    define FDCAN2_TXEVENTFIFO_WORDS (6)
#    define FDCAN2_TXFIFIOQ_WORDS    (54)
#  endif
#  ifdef CONFIG_STM32_FDCAN3
#    define FDCAN3_STDFILTER_SIZE    (28)
#    define FDCAN3_EXTFILTER_SIZE    (8)
#    define FDCAN3_RXFIFO0_SIZE      (3)
#    define FDCAN3_RXFIFO1_SIZE      (3)
#    define FDCAN3_TXEVENTFIFO_SIZE  (3)
#    define FDCAN3_TXFIFIOQ_SIZE     (3)

#    define FDCAN3_STDFILTER_WORDS   (28)
#    define FDCAN3_EXTFILTER_WORDS   (16)
#    define FDCAN3_RXFIFO0_WORDS     (54)
#    define FDCAN3_RXFIFO1_WORDS     (54)
#    define FDCAN3_TXEVENTFIFO_WORDS (6)
#    define FDCAN3_TXFIFIOQ_WORDS    (54)
#  endif
#else
#  error
#endif

/* FDCAN1 Configuration *****************************************************/

#ifdef CONFIG_STM32_FDCAN1

/* Bit timing */

#  define FDCAN1_NTSEG1  (CONFIG_STM32_FDCAN1_NTSEG1 - 1)
#  define FDCAN1_NTSEG2  (CONFIG_STM32_FDCAN1_NTSEG2 - 1)
#  define FDCAN1_NBRP    ((STM32_FDCANCLK_FREQUENCY /             \
                           ((FDCAN1_NTSEG1 + FDCAN1_NTSEG2 + 3) * \
                            CONFIG_STM32_FDCAN1_BITRATE)) - 1)
#  define FDCAN1_NSJW    (CONFIG_STM32_FDCAN1_NSJW - 1)

#  if FDCAN1_NTSEG1 > FDCAN_NBTP_NTSEG1_MAX
#    error Invalid FDCAN1 NTSEG1
#  endif
#  if FDCAN1_NTSEG2 > FDCAN_NBTP_NTSEG2_MAX
#    error Invalid FDCAN1 NTSEG2
#  endif
#  if FDCAN1_NSJW > FDCAN_NBTP_NSJW_MAX
#    error Invalid FDCAN1 NSJW
#  endif
#  if FDCAN1_NBRP > FDCAN_NBTP_NBRP_MAX
#    error Invalid FDCAN1 NBRP
#  endif

#  ifdef CONFIG_STM32_FDCAN1_FD_BRS
#    define FDCAN1_DTSEG1 (CONFIG_STM32_FDCAN1_DTSEG1 - 1)
#    define FDCAN1_DTSEG2 (CONFIG_STM32_FDCAN1_DTSEG2 - 1)
#    define FDCAN1_DBRP   ((STM32_FDCANCLK_FREQUENCY /             \
                            ((FDCAN1_DTSEG1 + FDCAN1_DTSEG2 + 3) * \
                             CONFIG_STM32_FDCAN1_DBITRATE)) - 1)
#    define FDCAN1_DSJW   (CONFIG_STM32_FDCAN1_DSJW - 1)
#  else
#    define FDCAN1_DTSEG1 1
#    define FDCAN1_DTSEG2 1
#    define FDCAN1_DBRP   1
#    define FDCAN1_DSJW   1
#  endif /* CONFIG_STM32_FDCAN1_FD_BRS */

#  if FDCAN1_DTSEG1 > FDCAN_DBTP_DTSEG1_MAX
#    error Invalid FDCAN1 DTSEG1
#  endif
#  if FDCAN1_DTSEG2 > FDCAN_DBTP_DTSEG2_MAX
#    error Invalid FDCAN1 DTSEG2
#  endif
#  if FDCAN1_DBRP > FDCAN_DBTP_DBRP_MAX
#    error Invalid FDCAN1 DBRP
#  endif
#  if FDCAN1_DSJW > FDCAN_DBTP_DSJW_MAX
#    error Invalid FDCAN1 DSJW
#  endif

/* FDCAN1 Message RAM Configuration *****************************************/

/* FDCAN1 Message RAM Layout */

#  define FDCAN1_STDFILTER_INDEX   0
#  define FDCAN1_EXTFILTERS_INDEX  (FDCAN1_STDFILTER_INDEX + FDCAN1_STDFILTER_WORDS)
#  define FDCAN1_RXFIFO0_INDEX     (FDCAN1_EXTFILTERS_INDEX + FDCAN1_EXTFILTER_WORDS)
#  define FDCAN1_RXFIFO1_INDEX     (FDCAN1_RXFIFO0_INDEX + FDCAN1_RXFIFO0_WORDS)
#  define FDCAN1_TXEVENTFIFO_INDEX (FDCAN1_RXFIFO1_INDEX + FDCAN1_RXFIFO1_WORDS)
#  define FDCAN1_TXFIFOQ_INDEX     (FDCAN1_TXEVENTFIFO_INDEX + FDCAN1_TXEVENTFIFO_WORDS)
#  define FDCAN1_MSGRAM_WORDS      (FDCAN1_TXFIFOQ_INDEX + FDCAN1_TXFIFIOQ_WORDS)

#endif /* CONFIG_STM32_FDCAN1 */

/* FDCAN2 Configuration *****************************************************/

#ifdef CONFIG_STM32_FDCAN2

/* Bit timing */

#  define FDCAN2_NTSEG1  (CONFIG_STM32_FDCAN2_NTSEG1 - 1)
#  define FDCAN2_NTSEG2  (CONFIG_STM32_FDCAN2_NTSEG2 - 1)
#  define FDCAN2_NBRP    (((STM32_FDCANCLK_FREQUENCY /              \
                            ((FDCAN2_NTSEG1 + FDCAN2_NTSEG2 + 3) *  \
                             CONFIG_STM32_FDCAN2_BITRATE)) - 1))
#  define FDCAN2_NSJW    (CONFIG_STM32_FDCAN2_NSJW - 1)

#  if FDCAN2_NTSEG1 > FDCAN_NBTP_NTSEG1_MAX
#    error Invalid FDCAN2 NTSEG1
#  endif
#  if FDCAN2_NTSEG2 > FDCAN_NBTP_NTSEG2_MAX
#    error Invalid FDCAN2 NTSEG2
#  endif
#  if FDCAN2_NSJW > FDCAN_NBTP_NSJW_MAX
#    error Invalid FDCAN2 NSJW
#  endif
#  if FDCAN2_NBRP > FDCAN_NBTP_NBRP_MAX
#    error Invalid FDCAN1 NBRP
#  endif

#  ifdef CONFIG_STM32_FDCAN2_FD_BRS
#    define FDCAN2_DTSEG1 (CONFIG_STM32_FDCAN2_DTSEG1 - 1)
#    define FDCAN2_DTSEG2 (CONFIG_STM32_FDCAN2_DTSEG2 - 1)
#    define FDCAN2_DBRP   (((STM32_FDCANCLK_FREQUENCY /                 \
                             ((FDCAN2_DTSEG1 + FDCAN2_DTSEG2 + 3) *     \
                              CONFIG_STM32_FDCAN2_DBITRATE)) - 1))
#    define FDCAN2_DSJW   (CONFIG_STM32_FDCAN2_DSJW - 1)
#  else
#    define FDCAN2_DTSEG1 1
#    define FDCAN2_DTSEG2 1
#    define FDCAN2_DBRP   1
#    define FDCAN2_DSJW   1
#  endif /* CONFIG_STM32_FDCAN2_FD_BRS */

#  if FDCAN2_DTSEG1 > FDCAN_DBTP_DTSEG1_MAX
#    error Invalid FDCAN2 DTSEG1
#  endif
#  if FDCAN2_DTSEG2 > FDCAN_DBTP_DTSEG2_MAX
#    error Invalid FDCAN2 DTSEG2
#  endif
#  if FDCAN2_DBRP > FDCAN_DBTP_DBRP_MAX
#    error Invalid FDCAN2 DBRP
#  endif
#  if FDCAN2_DSJW > FDCAN_DBTP_DSJW_MAX
#    error Invalid FDCAN2 DSJW
#  endif

/* FDCAN2 Message RAM Configuration *****************************************/

/* FDCAN2 Message RAM Layout */

#  define FDCAN2_STDFILTER_INDEX   0
#  define FDCAN2_EXTFILTERS_INDEX  (FDCAN2_STDFILTER_INDEX + FDCAN2_STDFILTER_WORDS)
#  define FDCAN2_RXFIFO0_INDEX     (FDCAN2_EXTFILTERS_INDEX + FDCAN2_EXTFILTER_WORDS)
#  define FDCAN2_RXFIFO1_INDEX     (FDCAN2_RXFIFO0_INDEX + FDCAN2_RXFIFO0_WORDS)
#  define FDCAN2_TXEVENTFIFO_INDEX (FDCAN2_RXFIFO1_INDEX + FDCAN2_RXFIFO1_WORDS)
#  define FDCAN2_TXFIFOQ_INDEX     (FDCAN2_TXEVENTFIFO_INDEX + FDCAN2_TXEVENTFIFO_WORDS)
#  define FDCAN2_MSGRAM_WORDS      (FDCAN2_TXFIFOQ_INDEX + FDCAN2_TXFIFIOQ_WORDS)

#endif /* CONFIG_STM32_FDCAN2 */

/* FDCAN3 Configuration *****************************************************/

#ifdef CONFIG_STM32_FDCAN3

/* Bit timing */

#  define FDCAN3_NTSEG1  (CONFIG_STM32_FDCAN3_NTSEG1 - 1)
#  define FDCAN3_NTSEG2  (CONFIG_STM32_FDCAN3_NTSEG2 - 1)
#  define FDCAN3_NBRP    (((STM32_FDCANCLK_FREQUENCY /              \
                            ((FDCAN3_NTSEG1 + FDCAN3_NTSEG2 + 3) *  \
                             CONFIG_STM32_FDCAN3_BITRATE)) - 1))
#  define FDCAN3_NSJW    (CONFIG_STM32_FDCAN3_NSJW - 1)

#  if FDCAN3_NTSEG1 > FDCAN_NBTP_NTSEG1_MAX
#    error Invalid FDCAN3 NTSEG1
#  endif
#  if FDCAN3_NTSEG2 > FDCAN_NBTP_NTSEG2_MAX
#    error Invalid FDCAN3 NTSEG2
#  endif
#  if FDCAN3_NSJW > FDCAN_NBTP_NSJW_MAX
#    error Invalid FDCAN3 NSJW
#  endif
#  if FDCAN3_NBRP > FDCAN_NBTP_NBRP_MAX
#    error Invalid FDCAN1 NBRP
#  endif

#  ifdef CONFIG_STM32_FDCAN3_FD_BRS
#    define FDCAN3_DTSEG1 (CONFIG_STM32_FDCAN3_DTSEG1 - 1)
#    define FDCAN3_DTSEG2 (CONFIG_STM32_FDCAN3_DTSEG2 - 1)
#    define FDCAN3_DBRP   (((STM32_FDCANCLK_FREQUENCY /                   \
                             ((FDCAN3_DTSEG1 + FDCAN3_DTSEG2 + 3) *     \
                              CONFIG_STM32_FDCAN3_DBITRATE)) - 1))
#    define FDCAN3_DSJW   (CONFIG_STM32_FDCAN3_DSJW - 1)
#  else
#    define FDCAN3_DTSEG1 1
#    define FDCAN3_DTSEG2 1
#    define FDCAN3_DBRP   1
#    define FDCAN3_DSJW   1
#  endif /* CONFIG_STM32_FDCAN3_FD_BRS */

#  if FDCAN3_DTSEG1 > FDCAN_DBTP_DTSEG1_MAX
#    error Invalid FDCAN3 DTSEG1
#  endif
#  if FDCAN3_DTSEG2 > FDCAN_DBTP_DTSEG2_MAX
#    error Invalid FDCAN3 DTSEG2
#  endif
#  if FDCAN3_DBRP > FDCAN_DBTP_DBRP_MAX
#    error Invalid FDCAN3 DBRP
#  endif
#  if FDCAN3_DSJW > FDCAN_DBTP_DSJW_MAX
#    error Invalid FDCAN3 DSJW
#  endif

/* FDCAN3 Message RAM Configuration *****************************************/

/* FDCAN3 Message RAM Layout */

#  define FDCAN3_STDFILTER_INDEX   0
#  define FDCAN3_EXTFILTERS_INDEX  (FDCAN3_STDFILTER_INDEX + FDCAN3_STDFILTER_WORDS)
#  define FDCAN3_RXFIFO0_INDEX     (FDCAN3_EXTFILTERS_INDEX + FDCAN3_EXTFILTER_WORDS)
#  define FDCAN3_RXFIFO1_INDEX     (FDCAN3_RXFIFO0_INDEX + FDCAN3_RXFIFO0_WORDS)
#  define FDCAN3_TXEVENTFIFO_INDEX (FDCAN3_RXFIFO1_INDEX + FDCAN3_RXFIFO1_WORDS)
#  define FDCAN3_TXFIFOQ_INDEX     (FDCAN3_TXEVENTFIFO_INDEX + FDCAN3_TXEVENTFIFO_WORDS)
#  define FDCAN3_MSGRAM_WORDS      (FDCAN3_TXFIFOQ_INDEX + FDCAN3_TXFIFIOQ_WORDS)

#endif /* CONFIG_STM32_FDCAN3 */

/* Loopback mode */

#undef STM32_FDCAN_LOOPBACK
#if defined(CONFIG_STM32_FDCAN1_LOOPBACK) ||   \
    defined(CONFIG_STM32_FDCAN2_LOOPBACK) ||   \
    defined(CONFIG_STM32_FDCAN3_LOOPBACK)
#  define STM32_FDCAN_LOOPBACK 1
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
 *   FDCAN_INT_RF0F - Receive FIFO 0 Full
 *   FDCAN_INT_RF0L - Receive FIFO 0 Message Lost
 *   FDCAN_INT_RF1N - Receive FIFO 1 New Message
 *   FDCAN_INT_RF1F - Receive FIFO 1 Full
 *   FDCAN_INT_RF1L - Receive FIFO 1 Message Lost
 *   FDCAN_INT_HPM  - High Priority Message Received
 *
 */

#define FDCAN_RXCOMMON_INTS  0
#define FDCAN_RXFIFO0_INTS  (FDCAN_INT_RF0N | FDCAN_INT_RF0L)
#define FDCAN_RXFIFO1_INTS  (FDCAN_INT_RF1N | FDCAN_INT_RF1L)
#define FDCAN_RXFIFO_INTS   (FDCAN_RXFIFO0_INTS | FDCAN_RXFIFO1_INTS | \
                             FDCAN_INT_HPM | FDCAN_RXCOMMON_INTS)

#define FDCAN_RXERR_INTS    (FDCAN_INT_RF0L | FDCAN_INT_RF1L)

/* TX FIFOQ mode interrupts
 *
 *   FDCAN_INT_TFE  - Tx FIFO Empty
 *
 * TX Event FIFO interrupts
 *
 *   FDCAN_INT_TEFN - Tx Event FIFO New Entry
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
#define FDCAN_TXEVFIFO_INTS (FDCAN_INT_TEFN | FDCAN_INT_TEFF | \
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
#  undef CONFIG_STM32_FDCAN_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CAN frame format */

enum stm32_frameformat_e
{
  FDCAN_ISO11898_1_FORMAT      = 0,  /* Frame format according to ISO11898-1 */
  FDCAN_NONISO_BOSCH_V1_FORMAT = 1   /* Frame format according to Bosch CAN FD V1.0 */
};

/* CAN mode of operation */

enum stm32_canmode_e
{
  FDCAN_CLASSIC_MODE = 0,   /* Classic CAN operation */
#ifdef CONFIG_CAN_FD
  FDCAN_FD_MODE      = 1,   /* CAN FD operation */
  FDCAN_FD_BRS_MODE  = 2    /* CAN FD operation with bit rate switching */
#endif
};

/* CAN driver state */

enum can_state_s
{
  FDCAN_STATE_UNINIT = 0,   /* Not yet initialized */
  FDCAN_STATE_RESET,        /* Initialized, reset state */
  FDCAN_STATE_SETUP,        /* fdcan_setup() has been called */
  FDCAN_STATE_DISABLED      /* Disabled by a fdcan_shutdown() */
};

/* This structure describes the FDCAN message RAM layout */

struct stm32_msgram_s
{
  volatile uint32_t *stdfilters;     /* Standard filters */
  volatile uint32_t *extfilters;     /* Extended filters */
  volatile uint32_t *rxfifo0;        /* RX FIFO0 */
  volatile uint32_t *rxfifo1;        /* RX FIFO1 */
  volatile uint32_t *txeventfifo;    /* TX event FIFO */
  volatile uint32_t *txfifoq;        /* TX FIFO queue */
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
  uint8_t nstdfilters;      /* Number of standard filters */
  uint8_t nextfilters;      /* Number of extended filters */
  uint8_t nrxfifo0;         /* Number of RX FIFO0 elements */
  uint8_t nrxfifo1;         /* Number of RX FIFO1 elements */
  uint8_t ntxeventfifo;     /* Number of TXevent FIFO elements */
  uint8_t ntxfifoq;         /* Number of TX FIFO queue elements */
  uint8_t rxfifo0esize;     /* RX FIFO0 element size (words) */
  uint8_t rxfifo1esize;     /* RX FIFO1 element size (words) */
  uint8_t txeventesize;     /* TXevent element size (words) */
  uint8_t txbufferesize;    /* TX buffer element size (words) */
#ifdef STM32_FDCAN_LOOPBACK
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
  uint32_t nbtp;            /* Current nominal bit timing */
  uint32_t dbtp;            /* Current data bit timing */
  uint32_t rxints;          /* Configured RX interrupts */
  uint32_t txints;          /* Configured TX interrupts */

#ifdef CONFIG_CAN_EXTID
  uint32_t extfilters[2];   /* Extended filter bit allocator.  2*32=64 */
#endif
  uint32_t stdfilters[4];   /* Standard filter bit allocator.  4*32=128 */

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
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
#ifdef CONFIG_STM32_FDCAN_REGDEBUG
static void fdcan_dumpregs(FAR struct stm32_fdcan_s *priv,
                           FAR const char *msg);
static void fdcan_dumprxregs(FAR struct stm32_fdcan_s *priv,
                             FAR const char *msg);
static void fdcan_dumptxregs(FAR struct stm32_fdcan_s *priv,
                             FAR const char *msg);
static void fdcan_dumpramlayout(FAR struct stm32_fdcan_s *priv);
#else
#  define fdcan_dumpregs(priv,msg)
#  define fdcan_dumprxregs(priv,msg)
#  define fdcan_dumptxregs(priv,msg)
#  define fdcan_dumpramlayout(priv)
#endif

/* FDCAN helpers */

static uint8_t fdcan_dlc2bytes(FAR struct stm32_fdcan_s *priv, uint8_t dlc);

#ifdef CONFIG_CAN_EXTID
static int fdcan_add_extfilter(FAR struct stm32_fdcan_s *priv,
                               FAR struct canioc_extfilter_s *extconfig);
static int fdcan_del_extfilter(FAR struct stm32_fdcan_s *priv, int ndx);
#endif
static int fdcan_add_stdfilter(FAR struct stm32_fdcan_s *priv,
                               FAR struct canioc_stdfilter_s *stdconfig);
static int fdcan_del_stdfilter(FAR struct stm32_fdcan_s *priv, int ndx);

static int
fdcan_start_busoff_recovery_sequence(FAR struct stm32_fdcan_s *priv);

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

#ifdef CONFIG_CAN_ERRORS
static void fdcan_error(FAR struct can_dev_s *dev, uint32_t status);
#endif
static void fdcan_receive(FAR struct can_dev_s *dev,
                          FAR volatile uint32_t *rxbuffer,
                          unsigned long nwords);
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

#ifdef CONFIG_STM32_FDCAN1
/* Message RAM allocation */

/* Constant configuration */

static const struct stm32_config_s g_fdcan1const =
{
  .rxpinset         = GPIO_FDCAN1_RX,
  .txpinset         = GPIO_FDCAN1_TX,
  .base             = STM32_FDCAN1_BASE,
  .baud             = CONFIG_STM32_FDCAN1_BITRATE,
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
#if defined(CONFIG_STM32_FDCAN1_CLASSIC)
  .mode             = FDCAN_CLASSIC_MODE,
#elif defined(CONFIG_STM32_FDCAN1_FD)
  .mode             = FDCAN_FD_MODE,
#else
  .mode             = FDCAN_FD_BRS_MODE,
#endif
#if defined(CONFIG_STM32_FDCAN1_NONISO_FORMAT)
  .format           = FDCAN_NONISO_BOSCH_V1_FORMAT,
#else
  .format           = FDCAN_ISO11898_1_FORMAT,
#endif
  .nstdfilters      = FDCAN1_STDFILTER_SIZE,
  .nextfilters      = FDCAN1_EXTFILTER_SIZE,
  .nrxfifo0         = FDCAN1_RXFIFO0_SIZE,
  .nrxfifo1         = FDCAN1_RXFIFO1_SIZE,
  .ntxeventfifo     = FDCAN1_TXEVENTFIFO_SIZE,
  .ntxfifoq         = FDCAN1_TXFIFIOQ_SIZE,
  .rxfifo0esize     = (FDCAN1_RXFIFO0_WORDS / FDCAN1_RXFIFO0_SIZE),
  .rxfifo1esize     = (FDCAN1_RXFIFO1_WORDS / FDCAN1_RXFIFO1_SIZE),
  .txeventesize     = (FDCAN1_TXEVENTFIFO_WORDS / FDCAN1_TXEVENTFIFO_SIZE),
  .txbufferesize    = (FDCAN1_TXFIFIOQ_WORDS / FDCAN1_TXFIFIOQ_SIZE),

#ifdef CONFIG_STM32_FDCAN1_LOOPBACK
  .loopback         = true,
#endif

  /* FDCAN1 Message RAM */

  .msgram =
  {
    (uint32_t *)(STM32_CANRAM1_BASE + (FDCAN1_STDFILTER_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM1_BASE + (FDCAN1_EXTFILTERS_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM1_BASE + (FDCAN1_RXFIFO0_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM1_BASE + (FDCAN1_RXFIFO1_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM1_BASE + (FDCAN1_TXEVENTFIFO_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM1_BASE + (FDCAN1_TXFIFOQ_INDEX << 2))
  }
};

/* FDCAN1 variable driver state */

static struct stm32_fdcan_s g_fdcan1priv;
static struct can_dev_s g_fdcan1dev;

#endif /* CONFIG_STM32_FDCAN1 */

#ifdef CONFIG_STM32_FDCAN2
/* FDCAN2 message RAM allocation */

/* FDCAN2 constant configuration */

static const struct stm32_config_s g_fdcan2const =
{
  .rxpinset         = GPIO_FDCAN2_RX,
  .txpinset         = GPIO_FDCAN2_TX,
  .base             = STM32_FDCAN2_BASE,
  .baud             = CONFIG_STM32_FDCAN2_BITRATE,
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
#if defined(CONFIG_STM32_FDCAN2_CLASSIC)
  .mode             = FDCAN_CLASSIC_MODE,
#elif defined(CONFIG_STM32_FDCAN2_FD)
  .mode             = FDCAN_FD_MODE,
#else
  .mode             = FDCAN_FD_BRS_MODE,
#endif
#if defined(CONFIG_STM32_FDCAN2_NONISO_FORMAT)
  .format           = FDCAN_NONISO_BOSCH_V1_FORMAT,
#else
  .format           = FDCAN_ISO11898_1_FORMAT,
#endif
  .nstdfilters      = FDCAN2_STDFILTER_SIZE,
  .nextfilters      = FDCAN2_EXTFILTER_SIZE,
  .nrxfifo0         = FDCAN2_RXFIFO0_SIZE,
  .nrxfifo1         = FDCAN2_RXFIFO1_SIZE,
  .ntxeventfifo     = FDCAN2_TXEVENTFIFO_SIZE,
  .ntxfifoq         = FDCAN2_TXFIFIOQ_SIZE,
  .rxfifo0esize     = (FDCAN2_RXFIFO0_WORDS / FDCAN2_RXFIFO0_SIZE),
  .rxfifo1esize     = (FDCAN2_RXFIFO1_WORDS / FDCAN2_RXFIFO1_SIZE),
  .txeventesize     = (FDCAN2_TXEVENTFIFO_WORDS / FDCAN2_TXEVENTFIFO_SIZE),
  .txbufferesize    = (FDCAN2_TXFIFIOQ_WORDS / FDCAN2_TXFIFIOQ_SIZE),

#ifdef CONFIG_STM32_FDCAN2_LOOPBACK
  .loopback         = true,
#endif

  /* FDCAN2 Message RAM */

  .msgram =
  {
    (uint32_t *)(STM32_CANRAM2_BASE + (FDCAN2_STDFILTER_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM2_BASE + (FDCAN2_EXTFILTERS_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM2_BASE + (FDCAN2_RXFIFO0_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM2_BASE + (FDCAN2_RXFIFO1_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM2_BASE + (FDCAN2_TXEVENTFIFO_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM2_BASE + (FDCAN2_TXFIFOQ_INDEX << 2))
  }
};

/* FDCAN2 variable driver state */

static struct stm32_fdcan_s g_fdcan2priv;
static struct can_dev_s g_fdcan2dev;

#endif /* CONFIG_STM32_FDCAN2 */

#ifdef CONFIG_STM32_FDCAN3
/* FDCAN3 message RAM allocation */

/* FDCAN3 constant configuration */

static const struct stm32_config_s g_fdcan3const =
{
  .rxpinset         = GPIO_FDCAN3_RX,
  .txpinset         = GPIO_FDCAN3_TX,
  .base             = STM32_FDCAN3_BASE,
  .baud             = CONFIG_STM32_FDCAN3_BITRATE,
  .nbtp             = FDCAN_NBTP_NBRP(FDCAN3_NBRP) |
                      FDCAN_NBTP_NTSEG1(FDCAN3_NTSEG1) |
                      FDCAN_NBTP_NTSEG2(FDCAN3_NTSEG2) |
                      FDCAN_NBTP_NSJW(FDCAN3_NSJW),
  .dbtp             = FDCAN_DBTP_DBRP(FDCAN3_DBRP) |
                      FDCAN_DBTP_DTSEG1(FDCAN3_DTSEG1) |
                      FDCAN_DBTP_DTSEG2(FDCAN3_DTSEG2) |
                      FDCAN_DBTP_DSJW(FDCAN3_DSJW),
  .port             = 3,
  .irq0             = STM32_IRQ_FDCAN3_0,
  .irq1             = STM32_IRQ_FDCAN3_1,
#if defined(CONFIG_STM32_FDCAN3_CLASSIC)
  .mode             = FDCAN_CLASSIC_MODE,
#elif defined(CONFIG_STM32_FDCAN3_FD)
  .mode             = FDCAN_FD_MODE,
#else
  .mode             = FDCAN_FD_BRS_MODE,
#endif
#if defined(CONFIG_STM32_FDCAN3_NONISO_FORMAT)
  .format           = FDCAN_NONISO_BOSCH_V1_FORMAT,
#else
  .format           = FDCAN_ISO11898_1_FORMAT,
#endif
  .nstdfilters      = FDCAN3_STDFILTER_SIZE,
  .nextfilters      = FDCAN3_EXTFILTER_SIZE,
  .nrxfifo0         = FDCAN3_RXFIFO0_SIZE,
  .nrxfifo1         = FDCAN3_RXFIFO1_SIZE,
  .ntxeventfifo     = FDCAN3_TXEVENTFIFO_SIZE,
  .ntxfifoq         = FDCAN3_TXFIFIOQ_SIZE,
  .rxfifo0esize     = (FDCAN3_RXFIFO0_WORDS / FDCAN3_RXFIFO0_SIZE),
  .rxfifo1esize     = (FDCAN3_RXFIFO1_WORDS / FDCAN3_RXFIFO1_SIZE),
  .txeventesize     = (FDCAN3_TXEVENTFIFO_WORDS / FDCAN3_TXEVENTFIFO_SIZE),
  .txbufferesize    = (FDCAN3_TXFIFIOQ_WORDS / FDCAN3_TXFIFIOQ_SIZE),

#ifdef CONFIG_STM32_FDCAN3_LOOPBACK
  .loopback         = true,
#endif

  /* FDCAN3 Message RAM */

  .msgram =
  {
    (uint32_t *)(STM32_CANRAM3_BASE + (FDCAN3_STDFILTER_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM3_BASE + (FDCAN3_EXTFILTERS_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM3_BASE + (FDCAN3_RXFIFO0_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM3_BASE + (FDCAN3_RXFIFO1_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM3_BASE + (FDCAN3_TXEVENTFIFO_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM3_BASE + (FDCAN3_TXFIFOQ_INDEX << 2))
  }
};

/* FDCAN3 variable driver state */

static struct stm32_fdcan_s g_fdcan3priv;
static struct can_dev_s g_fdcan3dev;

#endif /* CONFIG_STM32_FDCAN3 */

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

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
static uint32_t fdcan_getreg(FAR struct stm32_fdcan_s *priv, int offset)
{
  FAR const struct stm32_config_s *config  = priv->config;
  uintptr_t                        regaddr = 0;
  uint32_t                         regval  = 0;

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

  caninfo("%08" PRIx32 "->%08" PRIx32 "\n", regaddr, regval);
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

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
static void fdcan_putreg(FAR struct stm32_fdcan_s *priv, int offset,
                         uint32_t regval)
{
  FAR const struct stm32_config_s *config = priv->config;
  uintptr_t regaddr = config->base + offset;

  /* Show the register value being written */

  caninfo("%08" PRIx32 "->%08" PRIx32 "\n", regaddr, regval);

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

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
static void fdcan_dumpregs(FAR struct stm32_fdcan_s *priv,
                           FAR const char *msg)
{
  FAR const struct stm32_config_s *config = priv->config;

  caninfo("CAN%d Control and Status Registers: %s\n", config->port, msg);
  caninfo("  Base:  %08" PRIx32 "\n", config->base);

  /* CAN control and status registers */

  caninfo("  CCCR:  %08" PRIx32 "   TEST:  %08" PRIx32 "\n",
          getreg32(config->base + STM32_FDCAN_CCCR_OFFSET),
          getreg32(config->base + STM32_FDCAN_TEST_OFFSET));

  caninfo("  NBTP:  %08" PRIx32 "   DBTP:  %08" PRIx32 "\n",
          getreg32(config->base + STM32_FDCAN_NBTP_OFFSET),
          getreg32(config->base + STM32_FDCAN_DBTP_OFFSET));

  caninfo("  IE:    %08" PRIx32 "   TIE:   %08" PRIx32 "\n",
          getreg32(config->base + STM32_FDCAN_IE_OFFSET),
          getreg32(config->base + STM32_FDCAN_TXBTIE_OFFSET));

  caninfo("  ILE:   %08" PRIx32 "   ILS:   %08" PRIx32 "\n",
          getreg32(config->base + STM32_FDCAN_ILE_OFFSET),
          getreg32(config->base + STM32_FDCAN_ILS_OFFSET));

  caninfo("  TXBC:  %08" PRIx32 "\n",
          getreg32(config->base + STM32_FDCAN_TXBC_OFFSET));
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

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
static void fdcan_dumprxregs(FAR struct stm32_fdcan_s *priv,
                             FAR const char *msg)
{
  FAR const struct stm32_config_s *config = priv->config;

  caninfo("CAN%d Rx Registers: %s\n", config->port, msg);
  caninfo("  Base:  %08" PRIx32 "\n", config->base);

  caninfo("  PSR:   %08" PRIx32 "   ECR:   %08" PRIx32
          "   HPMS: %08" PRIx32 "\n",
          getreg32(config->base + STM32_FDCAN_PSR_OFFSET),
          getreg32(config->base + STM32_FDCAN_ECR_OFFSET),
          getreg32(config->base + STM32_FDCAN_HPMS_OFFSET));

  caninfo("  RXF0S: %08" PRIx32 "   RXF0A: %08" PRIx32 "\n",
          getreg32(config->base + STM32_FDCAN_RXF0S_OFFSET),
          getreg32(config->base + STM32_FDCAN_RXF0A_OFFSET));

  caninfo("  RXF1S: %08" PRIx32 "   RXF1A: %08" PRIx32 "\n",
          getreg32(config->base + STM32_FDCAN_RXF1S_OFFSET),
          getreg32(config->base + STM32_FDCAN_RXF1A_OFFSET));

  caninfo("  IR:    %08" PRIx32 "   IE:    %08" PRIx32 "\n",
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

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
static void fdcan_dumptxregs(FAR struct stm32_fdcan_s *priv,
                             FAR const char *msg)
{
  FAR const struct stm32_config_s *config = priv->config;

  caninfo("CAN%d Tx Registers: %s\n", config->port, msg);
  caninfo("  Base:  %08" PRIx32 "\n", config->base);

  caninfo("  PSR:   %08" PRIx32 "   ECR:   %08" PRIx32 "\n",
          getreg32(config->base + STM32_FDCAN_PSR_OFFSET),
          getreg32(config->base + STM32_FDCAN_ECR_OFFSET));

  caninfo("  TXQFS: %08" PRIx32 "   TXBAR: %08" PRIx32
          "   TXBRP: %08" PRIx32 "\n",
          getreg32(config->base + STM32_FDCAN_TXFQS_OFFSET),
          getreg32(config->base + STM32_FDCAN_TXBAR_OFFSET),
          getreg32(config->base + STM32_FDCAN_TXBRP_OFFSET));

  caninfo("  TXBTO: %08" PRIx32 "   TXBCR: %08" PRIx32 "\n",
          getreg32(config->base + STM32_FDCAN_TXBTO_OFFSET),
          getreg32(config->base + STM32_FDCAN_TXBCR_OFFSET));

  caninfo("   TXEFS: %08" PRIx32 "   TXEFA: %08" PRIx32 "\n",
          getreg32(config->base + STM32_FDCAN_TXEFS_OFFSET),
          getreg32(config->base + STM32_FDCAN_TXEFA_OFFSET));

  caninfo("  IR:    %08" PRIx32 "   IE:    %08" PRIx32
          "   TIE:   %08" PRIx32 "\n",
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

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
static void fdcan_dumpramlayout(FAR struct stm32_fdcan_s *priv)
{
  FAR const struct stm32_config_s *config = priv->config;

  caninfo(" ******* FDCAN%d Message RAM layout *******\n", config->port);
  caninfo("                Start     # Elmnt  Elmnt size (words)\n");

  if (config->nstdfilters > 0)
    {
      caninfo("STD filters   %p   %4d        %2d\n",
              config->msgram.stdfilters,
              config->nstdfilters,
              1);
    }

  if (config->nextfilters > 0)
    {
      caninfo("EXT filters   %p   %4d        %2d\n",
              config->msgram.extfilters,
              config->nextfilters,
              2);
    }

  if (config->nrxfifo0 > 0)
    {
      caninfo("RX FIFO 0     %p   %4d        %2d\n",
              config->msgram.rxfifo0,
              config->nrxfifo0,
              config->rxfifo0esize);
    }

  if (config->nrxfifo1 > 0)
    {
      caninfo("RX FIFO 1     %p   %4d        %2d\n",
              config->msgram.rxfifo1,
              config->nrxfifo1,
              config->rxfifo1esize);
    }

  if (config->ntxeventfifo > 0)
    {
      caninfo("TX EVENT      %p   %4d        %2d\n",
              config->msgram.txeventfifo,
              config->ntxeventfifo,
              config->txeventesize);
    }

  if (config->ntxfifoq > 0)
    {
      caninfo("TX FIFO       %p   %4d        %2d\n",
              config->msgram.txfifoq,
              config->ntxfifoq,
              config->txbufferesize);
    }
}
#endif

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
  FAR const struct stm32_config_s *config    = NULL;
  FAR volatile uint32_t           *extfilter = NULL;
  uint32_t                         regval    = 0;
  int                              word      = 0;
  int                              bit       = 0;
  int                              ndx       = 0;

  DEBUGASSERT(priv != NULL && priv->config != NULL && extconfig != NULL);
  config = priv->config;

  /* Find an unused standard filter */

  for (ndx = 0; ndx < config->nextfilters; ndx++)
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
              regval |= EXTFILTER_F0_EFEC_FIFO1;
            }

          extfilter[0] = regval;

          /* Format and write filter word F1 */

          DEBUGASSERT(extconfig->xf_id2 <= CAN_MAX_EXTMSGID);
          regval = EXTFILTER_F1_EFID2(extconfig->xf_id2);

          switch (extconfig->xf_type)
            {
              case CAN_FILTER_DUAL:
                {
                  regval |= EXTFILTER_F1_EFT_DUAL;
                  break;
                }

              case CAN_FILTER_MASK:
                {
                  regval |= EXTFILTER_F1_EFT_CLASSIC;
                  break;
                }

              case CAN_FILTER_RANGE:
                {
                  regval |= EXTFILTER_F1_EFT_RANGE;
                  break;
                }

              default:
                {
                  return -EINVAL;
                }
            }

          extfilter[1] = regval;

          /* Is this the first extended filter? */

          if (priv->nextalloc == 1)
            {
              /* Enable the Initialization state */

              regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
              regval |= FDCAN_CCCR_INIT;
              fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

              /* Wait for initialization mode to take effect */

              while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) &
                      FDCAN_CCCR_INIT) == 0);

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

              regval  = fdcan_getreg(priv, STM32_FDCAN_RXGFC_OFFSET);
              regval &= ~FDCAN_RXGFC_ANFE_MASK;
              regval |= FDCAN_RXGFC_ANFE_REJECTED;
              fdcan_putreg(priv, STM32_FDCAN_RXGFC_OFFSET, regval);

              /* Disable writing to configuration registers */

              regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
              regval &= ~(FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
              fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);
            }

          return ndx;
        }
    }

  DEBUGASSERT(priv->nextalloc == priv->config->nextfilters);

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
 *   ndx  - The filter index previously returned by the
 *          fdcan_add_extfilter().
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static int fdcan_del_extfilter(FAR struct stm32_fdcan_s *priv, int ndx)
{
  FAR const struct stm32_config_s *config    = NULL;
  FAR volatile uint32_t           *extfilter = NULL;
  uint32_t                         regval    = 0;
  int                              word      = 0;
  int                              bit       = 0;

  DEBUGASSERT(priv != NULL && priv->config != NULL);
  config = priv->config;

  /* Check user Parameters */

  DEBUGASSERT(ndx >= 0 || ndx < config->nextfilters);

  if (ndx < 0 || ndx >= config->nextfilters)
    {
      return -EINVAL;
    }

  word = ndx >> 5;
  bit  = ndx & 0x1f;

  /* Check if this filter is really assigned */

  if ((priv->extfilters[word] & (1 << bit)) == 0)
    {
      /* No, error out */

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
              FDCAN_CCCR_INIT) == 0);

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

      regval  = fdcan_getreg(priv, STM32_FDCAN_RXGFC_OFFSET);
      regval &= ~FDCAN_RXGFC_ANFE_MASK;
      regval |= FDCAN_RXGFC_ANFE_RX_FIFO0;
      fdcan_putreg(priv, STM32_FDCAN_RXGFC_OFFSET, regval);

      /* Disable writing to configuration registers */

      regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
      regval &= ~(FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
      fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);
    }

  /* Deactivate the filter last so that no messages are lost. */

  extfilter    = config->msgram.extfilters + (ndx << 1);
  *extfilter++ = 0;
  *extfilter   = 0;

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
  FAR const struct stm32_config_s *config    = NULL;
  FAR volatile uint32_t           *stdfilter = NULL;
  uint32_t                         regval    = 0;
  int                              word      = 0;
  int                              bit       = 0;
  int                              ndx       = 0;

  DEBUGASSERT(priv != NULL && priv->config != NULL);
  config = priv->config;

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
              case CAN_FILTER_DUAL:
                {
                  regval |= STDFILTER_S0_SFT_DUAL;
                  break;
                }

              case CAN_FILTER_MASK:
                {
                  regval |= STDFILTER_S0_SFT_CLASSIC;
                  break;
                }

              case CAN_FILTER_RANGE:
                {
                  regval |= STDFILTER_S0_SFT_RANGE;
                  break;
                }

              default:
                {
                  return -EINVAL;
                }
            }

          *stdfilter = regval;

          /* Is this the first standard filter? */

          if (priv->nstdalloc == 1)
            {
              /* Enable the Initialization state */

              regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
              regval |= FDCAN_CCCR_INIT;
              fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

              /* Wait for initialization mode to take effect */

              while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) &
                      FDCAN_CCCR_INIT) == 0);

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

              regval  = fdcan_getreg(priv, STM32_FDCAN_RXGFC_OFFSET);
              regval &= ~FDCAN_RXGFC_ANFS_MASK;
              regval |= FDCAN_RXGFC_ANFS_REJECTED;
              fdcan_putreg(priv, STM32_FDCAN_RXGFC_OFFSET, regval);

              /* Disable writing to configuration registers */

              regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
              regval &= ~(FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
              fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);
            }

          return ndx;
        }
    }

  DEBUGASSERT(priv->nstdalloc == priv->config->nstdfilters);
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
 *   ndx  - The filter index previously returned by the
 *          fdcan_add_stdfilter().
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int fdcan_del_stdfilter(FAR struct stm32_fdcan_s *priv, int ndx)
{
  FAR const struct stm32_config_s *config    = NULL;
  FAR volatile uint32_t           *stdfilter = NULL;
  uint32_t                         regval    = 0;
  int                              word      = 0;
  int                              bit       = 0;

  DEBUGASSERT(priv != NULL && priv->config != NULL);
  config = priv->config;

  /* Check Userspace Parameters */

  DEBUGASSERT(ndx >= 0 || ndx < config->nstdfilters);

  if (ndx < 0 || ndx >= config->nstdfilters)
    {
      return -EINVAL;
    }

  word = ndx >> 5;
  bit  = ndx & 0x1f;

  /* Check if this filter is really assigned */

  if ((priv->stdfilters[word] & (1 << bit)) == 0)
    {
      /* No, error out */

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
              FDCAN_CCCR_INIT) == 0);

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

      regval  = fdcan_getreg(priv, STM32_FDCAN_RXGFC_OFFSET);
      regval &= ~FDCAN_RXGFC_ANFS_MASK;
      regval |= FDCAN_RXGFC_ANFS_RX_FIFO0;
      fdcan_putreg(priv, STM32_FDCAN_RXGFC_OFFSET, regval);

      /* Disable writing to configuration registers */

      regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
      regval &= ~(FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
      fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);
    }

  /* Deactivate the filter last so that no messages are lost. */

  stdfilter  = config->msgram.stdfilters + ndx;
  *stdfilter = 0;

  return OK;
}

/****************************************************************************
 * Name: fdcan_start_busoff_recovery_sequence
 *
 * Description:
 *   This function initiates the BUS-OFF recovery sequence.
 *   CAN Specification Rev. 2.0 or ISO11898-1:2015.
 *   According the STM32G4 datasheet section 44.3.2 Software initialziation.
 *
 * Input Parameters:
 *   priv - An instance of the FDCAN driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int
fdcan_start_busoff_recovery_sequence(FAR struct stm32_fdcan_s *priv)
{
  uint32_t regval = 0;

  DEBUGASSERT(priv);

  /* Only start BUS-OFF recovery if we are in BUS-OFF state */

  regval = fdcan_getreg(priv, STM32_FDCAN_PSR_OFFSET);
  if ((regval & FDCAN_PSR_BO) == 0)
    {
      return -EPERM;
    }

  /* Disable initialization mode to issue the recovery sequence */

  regval = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
  regval &= ~FDCAN_CCCR_INIT;
  fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

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
  FAR struct stm32_fdcan_s        *priv   = NULL;
  FAR const struct stm32_config_s *config = NULL;
  uint32_t                         regval = 0;
  irqstate_t                       flags;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  caninfo("FDCAN%d\n", config->port);
  UNUSED(config);

  /* Disable all interrupts */

  fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, 0);
  fdcan_putreg(priv, STM32_FDCAN_TXBTIE_OFFSET, 0);

  /* Make sure that all buffers are released.
   *
   * REVISIT: What if a thread is waiting for a buffer?  The following
   * will not wake up any waiting threads.
   */

  /* Disable the FDCAN controller.
   * REVISIT: Should fdcan_shutdown() be called here?
   */

  /* Reset the FD CAN.
   * REVISIT:  Since there is only a single reset for both FDCAN
   * controllers, do we really want to use the RCC reset here?
   * This will nuke operation of the second controller if another
   * device is registered.
   */

  flags = enter_critical_section();
  regval  = getreg32(STM32_RCC_APB1RSTR1);
  regval |= RCC_APB1RSTR1_FDCANRST;
  putreg32(regval, STM32_RCC_APB1RSTR1);

  regval &= ~RCC_APB1RSTR1_FDCANRST;
  putreg32(regval, STM32_RCC_APB1RSTR1);
  leave_critical_section(flags);

  priv->state = FDCAN_STATE_RESET;
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
  FAR struct stm32_fdcan_s        *priv   = NULL;
  FAR const struct stm32_config_s *config = NULL;
  int                              ret    = 0;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  caninfo("FDCAN%d\n", config->port);

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

  /* Enable the interrupts at the NVIC (they are still disabled at the FDCAN
   * peripheral).
   */

  up_enable_irq(config->irq0);
  up_enable_irq(config->irq1);

  return OK;
}

/****************************************************************************
 * Name: fdcan_shutdown
 *
 * Description:
 *   Disable the FDCAN.  This method is called when the FDCAN device
 *   is closed. This method reverses the operation the setup method.
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
  FAR struct stm32_fdcan_s        *priv   = NULL;
  FAR const struct stm32_config_s *config = NULL;
  uint32_t                         regval = 0;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  caninfo("FDCAN%d\n", config->port);

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

  while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) & FDCAN_CCCR_INIT)
         == 0);
  while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) & FDCAN_CCCR_CSA)
         == 0);
  priv->state = FDCAN_STATE_DISABLED;
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
  FAR struct stm32_fdcan_s *priv   = dev->cd_priv;
  uint32_t                  regval = 0;

  DEBUGASSERT(priv && priv->config);

  caninfo("FDCAN%d enable: %d\n", priv->config->port, enable);

  /* Enable/disable the receive interrupts */

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
  uint32_t                  regval = 0;

  DEBUGASSERT(priv && priv->config);

  caninfo("FDCAN%d enable: %d\n", priv->config->port, enable);

  /* Enable/disable the receive interrupts */

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
  FAR struct stm32_fdcan_s *priv = NULL;
  int                       ret  = -ENOTTY;

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
          bt->bt_baud   = STM32_FDCANCLK_FREQUENCY / nbrp /
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
          uint32_t nbrp;
          uint32_t ntseg1;
          uint32_t ntseg2;
          uint32_t nsjw;
          uint32_t ie;
          uint8_t state;

          DEBUGASSERT(bt != NULL);
          DEBUGASSERT(bt->bt_baud < STM32_FDCANCLK_FREQUENCY);
          DEBUGASSERT(bt->bt_sjw > 0 && bt->bt_sjw <= 16);
          DEBUGASSERT(bt->bt_tseg1 > 1 && bt->bt_tseg1 <= 64);
          DEBUGASSERT(bt->bt_tseg2 > 0 && bt->bt_tseg2 <= 16);

          /* Extract bit timing data */

          ntseg1 = bt->bt_tseg1 - 1;
          ntseg2 = bt->bt_tseg2 - 1;
          nsjw   = bt->bt_sjw   - 1;

          nbrp = (uint32_t)
            (((float) STM32_FDCANCLK_FREQUENCY /
             ((float)(ntseg1 + ntseg2 + 3) * (float)bt->bt_baud)) - 1);

          /* Save the value of the new bit timing register */

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
  FAR struct stm32_fdcan_s        *priv       = NULL;
  FAR const struct stm32_config_s *config     = NULL;
  FAR volatile uint32_t           *txbuffer   = NULL;
  FAR const uint8_t               *src        = NULL;
  FAR uint32_t                    *dest       = NULL;
  uint32_t                         regval     = 0;
  unsigned int                     ndx        = 0;
  unsigned int                     nbytes     = 0;
  uint32_t                         wordbuffer = 0;
  unsigned int                     i          = 0;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  caninfo("CAN%" PRIu8 " ID: %" PRIu32 " DLC: %" PRIu8 "\n",
          config->port, (uint32_t)msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

  fdcan_dumptxregs(priv, "Before send");

  /* That that FIFO elements were configured */

  DEBUGASSERT(config->ntxfifoq > 0);

  /* Get our reserved Tx FIFO/queue put index */

  regval = fdcan_getreg(priv, STM32_FDCAN_TXFQS_OFFSET);
  DEBUGASSERT((regval & FDCAN_TXFQS_TFQF) == 0);

  ndx = (regval & FDCAN_TXFQS_TFQPI_MASK) >> FDCAN_TXFQS_TFQPI_SHIFT;

  /* And the TX buffer corresponding to this index */

  txbuffer = (config->msgram.txfifoq + ndx * config->txbufferesize);

  /* Format the TX FIFOQ entry
   *
   * Format word T0:
   *   Transfer message ID (ID)          - Value from message structure
   *   Remote Transmission Request (RTR) - Value from message structure
   *   Extended Identifier (XTD)         - Depends on configuration.
   *   Error state indicator (ESI)       - ESI bit in CAN FD
   *
   * Format word T1:
   *   Data Length Code (DLC)            - Value from message structure
   *   Bit Rate Switch (BRS)             - Bit rate switching for CAN FD
   *   FD format (FDF)                   - Frame transmited in CAN FD format
   *   Event FIFO Control (EFC)          - Do not store events.
   *   Message Marker (MM)               - Always zero
   */

  txbuffer[0] = 0;
  txbuffer[1] = 0;

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid == 1)
    {
      DEBUGASSERT(msg->cm_hdr.ch_id <= CAN_MAX_EXTMSGID);

      txbuffer[0] |= BUFFER_R0_EXTID(msg->cm_hdr.ch_id) | BUFFER_R0_XTD;
    }
  else
#endif
    {
      DEBUGASSERT(msg->cm_hdr.ch_id <= CAN_MAX_STDMSGID);

      txbuffer[0] |= BUFFER_R0_STDID(msg->cm_hdr.ch_id);
    }

  if (msg->cm_hdr.ch_rtr == 1)
    {
      txbuffer[0] |= BUFFER_R0_RTR;
    }

  txbuffer[1] |= BUFFER_R1_DLC(msg->cm_hdr.ch_dlc);

#ifdef CONFIG_CAN_FD
  /* CAN FD Format */

  if (msg->cm_hdr.ch_edl == 1)
    {
      txbuffer[1] |= BUFFER_R1_FDF;

      if (msg->cm_hdr.ch_brs == 1)
        {
          txbuffer[1] |= BUFFER_R1_BRS;
        }

      if (msg->cm_hdr.ch_esi == 1)
        {
          txbuffer[0] |= BUFFER_R0_ESI;
        }
    }
  else
#else
    {
      txbuffer[0] &= ~BUFFER_R0_ESI;
      txbuffer[1] &= ~BUFFER_R1_FDF;
      txbuffer[1] &= ~BUFFER_R1_BRS;
    }
#endif

  /* Followed by the amount of data corresponding to the DLC (T2..) */

  dest   = (FAR uint32_t *)&txbuffer[2];
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

  /* Enable transmit interrupts from the TX FIFOQ buffer by setting TC
   * interrupt bit in IR (also requires that the TC interrupt is enabled)
   */

  fdcan_putreg(priv, STM32_FDCAN_TXBTIE_OFFSET, (1 << ndx));

  /* And request to send the packet */

  fdcan_putreg(priv, STM32_FDCAN_TXBAR_OFFSET, (1 << ndx));

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
  FAR struct stm32_fdcan_s *priv    = dev->cd_priv;
  uint32_t                  regval  = 0;
  bool                      notfull = false;

  /* Return the state of the TX FIFOQ.  Return TRUE if the TX FIFO/Queue is
   * not full.
   */

  regval  = fdcan_getreg(priv, STM32_FDCAN_TXFQS_OFFSET);
  notfull = ((regval & FDCAN_TXFQS_TFQF) == 0);

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
  FAR struct stm32_fdcan_s *priv   = dev->cd_priv;
  uint32_t                  regval = 0;
  int                       tffl   = 0;
  bool                      empty  = false;

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  /* Return the state of the TX FIFOQ.  Return TRUE if the TX FIFO/Queue is
   * empty.  We don't have a reliable indication that the FIFO is empty, so
   * we have to use some heuristics.
   */

  regval = fdcan_getreg(priv, STM32_FDCAN_TXFQS_OFFSET);
  if ((regval & FDCAN_TXFQS_TFQF) != 0)
    {
      return false;
    }

  /* Tx FIFO Free Level */

  tffl  = (regval & FDCAN_TXFQS_TFFL_MASK) >> FDCAN_TXFQS_TFFL_SHIFT;
  empty = (tffl >= priv->config->ntxfifoq);

  return empty;
}

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
  FAR struct stm32_fdcan_s *priv    = dev->cd_priv;
  struct can_hdr_s          hdr;
  uint32_t                  psr     = 0;
  uint16_t                  errbits = 0;
  uint8_t                   data[CAN_ERROR_DLC];
  int                       ret     = 0;

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

  if ((psr & FDCAN_PSR_EW) != 0)
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

      if ((psr & FDCAN_PSR_LEC_MASK) != 0)
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

      if ((psr & FDCAN_PSR_DLEC_MASK) != 0)
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

static void fdcan_receive(FAR struct can_dev_s *dev,
                          FAR volatile uint32_t *rxbuffer,
                          unsigned long nwords)
{
  struct can_hdr_s hdr;
  int              ret    = 0;

  fdcan_dumprxregs(dev->cd_priv, "Before receive");

  /* Format the CAN header */

  /* Word R0 contains the CAN ID */

#ifdef CONFIG_CAN_ERRORS
  hdr.ch_error  = 0;
#endif
  hdr.ch_unused = 0;

  /* Extract the RTR bit */

  hdr.ch_rtr = ((rxbuffer[0] & BUFFER_R0_RTR) != 0);

#ifdef CONFIG_CAN_EXTID
  if ((rxbuffer[0] & BUFFER_R0_XTD) != 0)
    {
      /* Save the extended ID of the newly received message */

      hdr.ch_id    = (rxbuffer[0] & BUFFER_R0_EXTID_MASK) >>
                     BUFFER_R0_EXTID_SHIFT;
      hdr.ch_extid = 1;
    }
  else
    {
      hdr.ch_id    = (rxbuffer[0] & BUFFER_R0_STDID_MASK) >>
                     BUFFER_R0_STDID_SHIFT;
      hdr.ch_extid = 0;
    }

#else
  if ((rxbuffer[0] & BUFFER_R0_XTD) != 0)
    {
      /* Drop any messages with extended IDs */

      canerr("ERROR: Received message with extended identifier.  Dropped\n");

      return;
    }

  /* Save the standard ID of the newly received message */

  hdr.ch_id = (rxbuffer[0] & BUFFER_R0_STDID_MASK) >> BUFFER_R0_STDID_SHIFT;
#endif

  /* Word R1 contains the DLC and timestamp */

  hdr.ch_dlc = (rxbuffer[1] & BUFFER_R1_DLC_MASK) >> BUFFER_R1_DLC_SHIFT;

#ifdef CONFIG_CAN_FD
  /* CAN FD format */

  hdr.ch_esi = ((rxbuffer[0] & BUFFER_R0_ESI) != 0);
  hdr.ch_edl = ((rxbuffer[1] & BUFFER_R1_FDF) != 0);
  hdr.ch_brs = ((rxbuffer[1] & BUFFER_R1_BRS) != 0);
#else
  if ((rxbuffer[1] & BUFFER_R1_FDF) != 0)
    {
      /* Drop any FD CAN messages if not supported */

      canerr("ERROR: Received CAN FD message.  Dropped\n");

      return;
    }
#endif

  /* And provide the CAN message to the upper half logic */

  ret = can_receive(dev, &hdr, (FAR uint8_t *)&rxbuffer[2]);
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
  FAR struct can_dev_s            *dev     = (FAR struct can_dev_s *)arg;
  FAR struct stm32_fdcan_s        *priv    = NULL;
  FAR const struct stm32_config_s *config  = NULL;
  uint32_t                         ir      = 0;
  uint32_t                         ie      = 0;
  uint32_t                         pending = 0;
  uint32_t                         regval  = 0;
  uint32_t                         psr     = 0;
  unsigned int                     nelem   = 0;
  unsigned int                     ndx     = 0;

  DEBUGASSERT(dev != NULL);
  priv = dev->cd_priv;
  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  /* Get the set of pending interrupts. */

  ir = fdcan_getreg(priv, STM32_FDCAN_IR_OFFSET);
  ie = fdcan_getreg(priv, STM32_FDCAN_IE_OFFSET);

  pending = (ir & ie);

  /* Check for any errors */

  if ((pending & FDCAN_ANYERR_INTS) != 0)
    {
      /* Check for common errors */

      if ((pending & FDCAN_CMNERR_INTS) != 0)
        {
          canerr("ERROR: Common %08" PRIx32 "\n",
                 pending & FDCAN_CMNERR_INTS);

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

          psr = fdcan_getreg(priv, STM32_FDCAN_PSR_OFFSET);

          if ((psr & FDCAN_PSR_LEC_MASK) != 0)
            {
              canerr("ERROR: PSR %08" PRIx32 "\n", psr);
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
          canerr("ERROR: TX %08" PRIx32 "\n",
                 pending & FDCAN_TXERR_INTS);

          /* An Acknowledge-Error will occur if for example the device
           * is not connected to the bus.
           *
           * The CAN-Standard states that the Chip has to retry the
           * message forever, which will produce an ACKE every time.
           * To prevent this Interrupt-Flooding and the high CPU-Load
           * we disable the ACKE here as long we didn't transfer at
           * least one message successfully (see FDCAN_INT_TC below).
           */

          /* Clear the error indications */

          fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, FDCAN_TXERR_INTS);
        }

      /* Check for reception errors */

      if ((pending & FDCAN_RXERR_INTS) != 0)
        {
          canerr("ERROR: RX %08" PRIx32 "\n",
                 pending & FDCAN_RXERR_INTS);

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

      /* Check all TX buffers */

      regval = fdcan_getreg(priv, STM32_FDCAN_TXBTO_OFFSET);
      for (ndx = 0; ndx < config->ntxfifoq; ndx++)
        {
          if ((regval & (1 << ndx)) != 0)
            {
              /* Tell the upper half that the transfer is finished. */

              can_txdone(dev);
            }
        }
    }
  else if ((pending & priv->txints) != 0)
    {
      /* Clear unhandled TX events */

      fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, priv->txints);
    }

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
          canerr("ERROR: Message lost: %08" PRIx32 "\n", regval);
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
          canerr("ERROR: Message lost: %08" PRIx32 "\n", regval);
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
    }

  /* Clear unhandled RX interrupts */

  if ((pending & priv->rxints) != 0)
    {
      fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, priv->rxints);
    }

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
  FAR volatile uint32_t           *msgram = NULL;
  uint32_t                         regval = 0;
  uint32_t                         cntr   = 0;

  caninfo("FDCAN%d\n", config->port);

  /* Clean message RAM */

  msgram = config->msgram.stdfilters;
  cntr   = (FDCAN_MSGRAM_WORDS + 1);
  while (cntr > 0)
    {
      *msgram++ = 0;
      cntr--;
    }

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

      while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) & FDCAN_CCCR_CSA)
             != 0);
    }

  /* Enable the Initialization state */

  regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
  regval |= FDCAN_CCCR_INIT;
  fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

  /* Wait for initialization mode to take effect */

  while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) & FDCAN_CCCR_INIT)
         == 0);

  /* Enable writing to configuration registers */

  regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
  regval |= FDCAN_CCCR_CCE;
  fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

  /* Global Filter Configuration:
   *
   *   ANFS=0: Store all non matching standard frame in RX FIFO0
   *   ANFE=0: Store all non matching extended frame in RX FIFO0
   */

  regval = FDCAN_RXGFC_ANFE_RX_FIFO0 | FDCAN_RXGFC_ANFS_RX_FIFO0;
  fdcan_putreg(priv, STM32_FDCAN_RXGFC_OFFSET, regval);

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

  regval = FDCAN_RXGFC_LSS(config->nstdfilters);
  regval |= FDCAN_RXGFC_LSE(config->nextfilters);
  fdcan_putreg(priv, STM32_FDCAN_RXGFC_OFFSET, regval);

  /* Dump RAM layout */

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

  /* Input clock divider configuration */

  regval = FDCANCLK_PDIV;
  fdcan_putreg(priv, STM32_FDCAN_CKDIV_OFFSET, regval);

  /* CC control register */

  regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
  regval &= ~(FDCAN_CCCR_NISO | FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE);

  /* Select ISO11898-1 or Non ISO Bosch CAN FD Specification V1.0 */

  switch (config->format)
    {
      case FDCAN_ISO11898_1_FORMAT:
        {
          break;
        }

      case FDCAN_NONISO_BOSCH_V1_FORMAT:
        {
          regval |= FDCAN_CCCR_NISO;
          break;
        }

      default:
        {
          return -EINVAL;
        }
    }

  /* Select Classic CAN mode or FD mode with or without fast bit rate
   * switching
   */

  switch (config->mode)
    {
      case FDCAN_CLASSIC_MODE:
        {
          break;
        }

#ifdef CONFIG_CAN_FD
      case FDCAN_FD_MODE:
        {
          regval |= FDCAN_CCCR_FDOE;
          break;
        }

      case FDCAN_FD_BRS_MODE:
        {
          regval |= (FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE);
          break;
        }
#endif

      default:
        {
          return -EINVAL;
        }
    }

  /* Set the initial CAN mode */

  fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

  /* Enable FIFO/Queue mode */

  regval  = fdcan_getreg(priv, STM32_FDCAN_TXBC_OFFSET);
#ifdef CONFIG_STM32_FDCAN_QUEUE_MODE
  regval |= FDCAN_TXBC_TFQM;
#else
  regval &= ~FDCAN_TXBC_TFQM;
#endif
  fdcan_putreg(priv, STM32_FDCAN_TXBC_OFFSET, regval);

#ifdef STM32_FDCAN_LOOPBACK
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

  priv->rxints = FDCAN_RXFIFO_INTS;

  /* Select TX-related interrupts */

  priv->txints = FDCAN_TXFIFOQ_INTS;

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
 * Name: stm32_fdcaninitialize
 *
 * Description:
 *   Initialize the selected FDCAN port
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple FDCAN interfaces),
 *          1=FDCAN1, 2=FDCAN2, 3=FDCAN3
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct can_dev_s *stm32_fdcaninitialize(int port)
{
  FAR struct can_dev_s            *dev    = NULL;
  FAR struct stm32_fdcan_s        *priv   = NULL;
  FAR const struct stm32_config_s *config = NULL;

  caninfo("FDCAN%d\n", port);

  /* Select FDCAN peripheral to be initialized */

#ifdef CONFIG_STM32_FDCAN1
  if (port == FDCAN1)
    {
      /* Select the FDCAN1 device structure */

      dev    = &g_fdcan1dev;
      priv   = &g_fdcan1priv;
      config = &g_fdcan1const;
    }
  else
#endif
#ifdef CONFIG_STM32_FDCAN2
  if (port == FDCAN2)
    {
      /* Select the FDCAN2 device structure */

      dev    = &g_fdcan2dev;
      priv   = &g_fdcan2priv;
      config = &g_fdcan2const;
    }
  else
#endif
#ifdef CONFIG_STM32_FDCAN3
  if (port == FDCAN3)
    {
      /* Select the FDCAN3 device structure */

      dev    = &g_fdcan3dev;
      priv   = &g_fdcan3priv;
      config = &g_fdcan3const;
    }
  else
#endif
    {
      canerr("ERROR: Unsupported port %d\n", port);
      return NULL;
    }

  /* Perform one time data initialization */

  memset(priv, 0, sizeof(struct stm32_fdcan_s));
  priv->config = config;

  /* Set the initial bit timing.  This might change subsequently
   * due to IOCTL command processing.
   */

  priv->nbtp   = config->nbtp;
  priv->dbtp   = config->dbtp;

  dev->cd_ops  = &g_fdcanops;
  dev->cd_priv = (FAR void *)priv;

  /* And put the hardware in the initial state */

  fdcan_reset(dev);

  return dev;
}

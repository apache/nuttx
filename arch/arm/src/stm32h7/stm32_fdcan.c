/****************************************************************************
 * arch/arm/src/stm32h7/stm32_fdcan.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <string.h>
#include <errno.h>
#include <nuttx/debug.h>
#include <assert.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/can/can.h>

#include "arm_internal.h"
#include "stm32_fdcan.h"
#include "hardware/stm32_pinmap.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FDCAN kernel clock (fdcan_ker_ck), selected by the board with
 * STM32_RCC_D2CCIP1R_FDCANSEL.  Defaults to HSE as after reset.
 */

#ifndef STM32_FDCANCLK
#  define STM32_FDCANCLK           STM32_HSE_FREQUENCY
#endif

/* Bit timing register helpers */

#define FDCAN_NBTP_VALUE(brp, tseg1, tseg2, sjw)          \
  (((uint32_t)(brp) << FDCAN_NBTP_NBRP_SHIFT) |           \
   ((uint32_t)(tseg1) << FDCAN_NBTP_NTSEG1_SHIFT) |       \
   ((uint32_t)(tseg2) << FDCAN_NBTP_TSEG2_SHIFT) |        \
   ((uint32_t)(sjw) << FDCAN_NBTP_NSJW_SHIFT))
#define FDCAN_DBTP_VALUE(brp, tseg1, tseg2, sjw)          \
  (((uint32_t)(brp) << FDCAN_DBTP_DBRP_SHIFT) |           \
   ((uint32_t)(tseg1) << FDCAN_DBTP_DTSEG1_SHIFT) |       \
   ((uint32_t)(tseg2) << FDCAN_DBTP_DTSEG2_SHIFT) |       \
   ((uint32_t)(sjw) << FDCAN_DBTP_DSJW_SHIFT))

#define FDCAN_NBTP_NBRP_MAX        (511)
#define FDCAN_NBTP_NTSEG1_MAX      (255)
#define FDCAN_NBTP_NTSEG2_MAX      (127)
#define FDCAN_NBTP_NSJW_MAX        (127)
#define FDCAN_DBTP_DBRP_MAX        (31)
#define FDCAN_DBTP_DTSEG1_MAX      (31)
#define FDCAN_DBTP_DTSEG2_MAX      (15)
#define FDCAN_DBTP_DSJW_MAX        (15)
#define FDCAN_TDCR_TDCO_MAX        (127)

/* Message RAM **************************************************************/

/* 2560 words are shared by all FDCAN instances.  Each enabled port gets a
 * fixed region with the same layout.
 */

#define FDCAN_MSGRAM_TOTAL_WORDS   (2560)

#ifdef CONFIG_CAN_FD
#  define FDCAN_ELEMENT_WORDS      (18)  /* 2 header words + 64 data bytes */
#  define FDCAN_ELEMENT_DS         (7)   /* RXESC/TXESC: 64 byte data field */
#else
#  define FDCAN_ELEMENT_WORDS      (4)   /* 2 header words + 8 data bytes */
#  define FDCAN_ELEMENT_DS         (0)   /* RXESC/TXESC: 8 byte data field */
#endif

#define FDCAN_STDFILTER_SIZE       (32)
#define FDCAN_EXTFILTER_SIZE       (16)
#define FDCAN_RXFIFO0_SIZE         (16)
#define FDCAN_RXFIFO1_SIZE         (8)
#define FDCAN_TXFIFOQ_SIZE         (16)

/* The last element of each filter list is a reserved accept-all filter,
 * enabled while no user filter of that kind is allocated.
 */

#define FDCAN_NSTDFILTERS          (FDCAN_STDFILTER_SIZE - 1)
#define FDCAN_NEXTFILTERS          (FDCAN_EXTFILTER_SIZE - 1)

#define FDCAN_STDFILTER_INDEX      (0)
#define FDCAN_EXTFILTER_INDEX      (FDCAN_STDFILTER_INDEX + FDCAN_STDFILTER_SIZE)
#define FDCAN_RXFIFO0_INDEX        (FDCAN_EXTFILTER_INDEX + 2 * FDCAN_EXTFILTER_SIZE)
#define FDCAN_RXFIFO1_INDEX        (FDCAN_RXFIFO0_INDEX + \
                                    FDCAN_RXFIFO0_SIZE * FDCAN_ELEMENT_WORDS)
#define FDCAN_TXFIFOQ_INDEX        (FDCAN_RXFIFO1_INDEX + \
                                    FDCAN_RXFIFO1_SIZE * FDCAN_ELEMENT_WORDS)
#define FDCAN_MSGRAM_WORDS         (FDCAN_TXFIFOQ_INDEX + \
                                    FDCAN_TXFIFOQ_SIZE * FDCAN_ELEMENT_WORDS)

#if 3 * FDCAN_MSGRAM_WORDS > FDCAN_MSGRAM_TOTAL_WORDS
#  error FDCAN message RAM layout does not fit
#endif

#define FDCAN_MSGRAM_BASE(port)    (STM32_CANRAM_BASE + \
                                    ((port) - 1) * FDCAN_MSGRAM_WORDS * 4)

#define FDCAN_MSGRAM(port, index)  ((volatile uint32_t *) \
                                    (FDCAN_MSGRAM_BASE(port) + ((index) << 2)))

#define FDCAN_TXBUFFERS_ALL        ((1 << FDCAN_TXFIFOQ_SIZE) - 1)

/* FDCAN1 Configuration *****************************************************/

#ifdef CONFIG_STM32_FDCAN1
#  ifndef CONFIG_STM32_FDCAN1_AUTO_BIT_TIMING
#    define FDCAN1_NTSEG1  (CONFIG_STM32_FDCAN1_NTSEG1 - 1)
#    define FDCAN1_NTSEG2  (CONFIG_STM32_FDCAN1_NTSEG2 - 1)
#    define FDCAN1_NBRP    ((STM32_FDCANCLK /                        \
                             ((FDCAN1_NTSEG1 + FDCAN1_NTSEG2 + 3) * \
                              CONFIG_STM32_FDCAN1_BITRATE)) - 1)
#    define FDCAN1_NSJW    (CONFIG_STM32_FDCAN1_NSJW - 1)

#    if FDCAN1_NTSEG1 > FDCAN_NBTP_NTSEG1_MAX
#      error Invalid FDCAN1 NTSEG1
#    endif
#    if FDCAN1_NTSEG2 > FDCAN_NBTP_NTSEG2_MAX
#      error Invalid FDCAN1 NTSEG2
#    endif
#    if FDCAN1_NSJW > FDCAN_NBTP_NSJW_MAX
#      error Invalid FDCAN1 NSJW
#    endif
#    if FDCAN1_NBRP > FDCAN_NBTP_NBRP_MAX
#      error Invalid FDCAN1 NBRP
#    endif

#    ifdef CONFIG_STM32_FDCAN1_FD_BRS
#      define FDCAN1_DTSEG1 (CONFIG_STM32_FDCAN1_DTSEG1 - 1)
#      define FDCAN1_DTSEG2 (CONFIG_STM32_FDCAN1_DTSEG2 - 1)
#      define FDCAN1_DBRP   ((STM32_FDCANCLK /                        \
                              ((FDCAN1_DTSEG1 + FDCAN1_DTSEG2 + 3) * \
                               CONFIG_STM32_FDCAN1_DBITRATE)) - 1)
#      define FDCAN1_DSJW   (CONFIG_STM32_FDCAN1_DSJW - 1)
#    else
#      define FDCAN1_DTSEG1 1
#      define FDCAN1_DTSEG2 1
#      define FDCAN1_DBRP   1
#      define FDCAN1_DSJW   1
#    endif

#    if FDCAN1_DTSEG1 > FDCAN_DBTP_DTSEG1_MAX
#      error Invalid FDCAN1 DTSEG1
#    endif
#    if FDCAN1_DTSEG2 > FDCAN_DBTP_DTSEG2_MAX
#      error Invalid FDCAN1 DTSEG2
#    endif
#    if FDCAN1_DBRP > FDCAN_DBTP_DBRP_MAX
#      error Invalid FDCAN1 DBRP
#    endif
#    if FDCAN1_DSJW > FDCAN_DBTP_DSJW_MAX
#      error Invalid FDCAN1 DSJW
#    endif
#  endif
#endif /* CONFIG_STM32_FDCAN1 */

/* FDCAN2 Configuration *****************************************************/

#ifdef CONFIG_STM32_FDCAN2
#  ifndef CONFIG_STM32_FDCAN2_AUTO_BIT_TIMING
#    define FDCAN2_NTSEG1  (CONFIG_STM32_FDCAN2_NTSEG1 - 1)
#    define FDCAN2_NTSEG2  (CONFIG_STM32_FDCAN2_NTSEG2 - 1)
#    define FDCAN2_NBRP    ((STM32_FDCANCLK /                        \
                             ((FDCAN2_NTSEG1 + FDCAN2_NTSEG2 + 3) * \
                              CONFIG_STM32_FDCAN2_BITRATE)) - 1)
#    define FDCAN2_NSJW    (CONFIG_STM32_FDCAN2_NSJW - 1)

#    if FDCAN2_NTSEG1 > FDCAN_NBTP_NTSEG1_MAX
#      error Invalid FDCAN2 NTSEG1
#    endif
#    if FDCAN2_NTSEG2 > FDCAN_NBTP_NTSEG2_MAX
#      error Invalid FDCAN2 NTSEG2
#    endif
#    if FDCAN2_NSJW > FDCAN_NBTP_NSJW_MAX
#      error Invalid FDCAN2 NSJW
#    endif
#    if FDCAN2_NBRP > FDCAN_NBTP_NBRP_MAX
#      error Invalid FDCAN2 NBRP
#    endif

#    ifdef CONFIG_STM32_FDCAN2_FD_BRS
#      define FDCAN2_DTSEG1 (CONFIG_STM32_FDCAN2_DTSEG1 - 1)
#      define FDCAN2_DTSEG2 (CONFIG_STM32_FDCAN2_DTSEG2 - 1)
#      define FDCAN2_DBRP   ((STM32_FDCANCLK /                        \
                              ((FDCAN2_DTSEG1 + FDCAN2_DTSEG2 + 3) * \
                               CONFIG_STM32_FDCAN2_DBITRATE)) - 1)
#      define FDCAN2_DSJW   (CONFIG_STM32_FDCAN2_DSJW - 1)
#    else
#      define FDCAN2_DTSEG1 1
#      define FDCAN2_DTSEG2 1
#      define FDCAN2_DBRP   1
#      define FDCAN2_DSJW   1
#    endif

#    if FDCAN2_DTSEG1 > FDCAN_DBTP_DTSEG1_MAX
#      error Invalid FDCAN2 DTSEG1
#    endif
#    if FDCAN2_DTSEG2 > FDCAN_DBTP_DTSEG2_MAX
#      error Invalid FDCAN2 DTSEG2
#    endif
#    if FDCAN2_DBRP > FDCAN_DBTP_DBRP_MAX
#      error Invalid FDCAN2 DBRP
#    endif
#    if FDCAN2_DSJW > FDCAN_DBTP_DSJW_MAX
#      error Invalid FDCAN2 DSJW
#    endif
#  endif
#endif /* CONFIG_STM32_FDCAN2 */

/* FDCAN3 Configuration *****************************************************/

#ifdef CONFIG_STM32_FDCAN3
#  ifndef CONFIG_STM32_FDCAN3_AUTO_BIT_TIMING
#    define FDCAN3_NTSEG1  (CONFIG_STM32_FDCAN3_NTSEG1 - 1)
#    define FDCAN3_NTSEG2  (CONFIG_STM32_FDCAN3_NTSEG2 - 1)
#    define FDCAN3_NBRP    ((STM32_FDCANCLK /                        \
                             ((FDCAN3_NTSEG1 + FDCAN3_NTSEG2 + 3) * \
                              CONFIG_STM32_FDCAN3_BITRATE)) - 1)
#    define FDCAN3_NSJW    (CONFIG_STM32_FDCAN3_NSJW - 1)

#    if FDCAN3_NTSEG1 > FDCAN_NBTP_NTSEG1_MAX
#      error Invalid FDCAN3 NTSEG1
#    endif
#    if FDCAN3_NTSEG2 > FDCAN_NBTP_NTSEG2_MAX
#      error Invalid FDCAN3 NTSEG2
#    endif
#    if FDCAN3_NSJW > FDCAN_NBTP_NSJW_MAX
#      error Invalid FDCAN3 NSJW
#    endif
#    if FDCAN3_NBRP > FDCAN_NBTP_NBRP_MAX
#      error Invalid FDCAN3 NBRP
#    endif

#    ifdef CONFIG_STM32_FDCAN3_FD_BRS
#      define FDCAN3_DTSEG1 (CONFIG_STM32_FDCAN3_DTSEG1 - 1)
#      define FDCAN3_DTSEG2 (CONFIG_STM32_FDCAN3_DTSEG2 - 1)
#      define FDCAN3_DBRP   ((STM32_FDCANCLK /                        \
                              ((FDCAN3_DTSEG1 + FDCAN3_DTSEG2 + 3) * \
                               CONFIG_STM32_FDCAN3_DBITRATE)) - 1)
#      define FDCAN3_DSJW   (CONFIG_STM32_FDCAN3_DSJW - 1)
#    else
#      define FDCAN3_DTSEG1 1
#      define FDCAN3_DTSEG2 1
#      define FDCAN3_DBRP   1
#      define FDCAN3_DSJW   1
#    endif

#    if FDCAN3_DTSEG1 > FDCAN_DBTP_DTSEG1_MAX
#      error Invalid FDCAN3 DTSEG1
#    endif
#    if FDCAN3_DTSEG2 > FDCAN_DBTP_DTSEG2_MAX
#      error Invalid FDCAN3 DTSEG2
#    endif
#    if FDCAN3_DBRP > FDCAN_DBTP_DBRP_MAX
#      error Invalid FDCAN3 DBRP
#    endif
#    if FDCAN3_DSJW > FDCAN_DBTP_DSJW_MAX
#      error Invalid FDCAN3 DSJW
#    endif
#  endif
#endif /* CONFIG_STM32_FDCAN3 */

/* Interrupts ***************************************************************/

/* IR and IE share the same bit layout; the IR names are used for both.
 *
 * Common interrupts
 *
 *   FDCAN_IR_MRAF - Message RAM Access Failure
 *   FDCAN_IR_TOO  - Timeout Occurred
 *   FDCAN_IR_EP   - Error Passive
 *   FDCAN_IR_EW   - Warning Status
 *   FDCAN_IR_BO   - Bus_Off Status
 *   FDCAN_IR_WDI  - Watchdog Interrupt
 *   FDCAN_IR_PEA  - Protocol Error in Arbitration Phase
 *   FDCAN_IR_PED  - Protocol Error in Data Phase
 */

#define FDCAN_CMNERR_INTS   (FDCAN_IR_MRAF | FDCAN_IR_TOO | FDCAN_IR_EP | \
                             FDCAN_IR_EW | FDCAN_IR_BO | FDCAN_IR_WDI | \
                             FDCAN_IR_PEA | FDCAN_IR_PED)
#define FDCAN_COMMON_INTS   FDCAN_CMNERR_INTS

/* RX FIFO interrupts
 *
 *   FDCAN_IR_RF0N - Receive FIFO 0 New Message
 *   FDCAN_IR_RF0L - Receive FIFO 0 Message Lost
 *   FDCAN_IR_RF1N - Receive FIFO 1 New Message
 *   FDCAN_IR_RF1L - Receive FIFO 1 Message Lost
 */

#define FDCAN_RXFIFO0_INTS  (FDCAN_IR_RF0N | FDCAN_IR_RF0L)
#define FDCAN_RXFIFO1_INTS  (FDCAN_IR_RF1N | FDCAN_IR_RF1L)
#define FDCAN_RXFIFO_INTS   (FDCAN_RXFIFO0_INTS | FDCAN_RXFIFO1_INTS)

#define FDCAN_RXERR_INTS    (FDCAN_IR_RF0L | FDCAN_IR_RF1L)

/* TX FIFO/queue interrupts
 *
 *   FDCAN_IR_TC   - Transmission Completed
 *   FDCAN_IR_TCF  - Transmission Cancellation Finished
 *   FDCAN_IR_TFE  - Tx FIFO Empty
 */

#define FDCAN_TXFIFOQ_INTS  (FDCAN_IR_TC | FDCAN_IR_TCF | FDCAN_IR_TFE)

#define FDCAN_TXERR_INTS    (FDCAN_IR_TEFL | FDCAN_IR_PEA | FDCAN_IR_PED)

/* Common-, TX- and RX-Error-Mask */

#define FDCAN_ANYERR_INTS   (FDCAN_CMNERR_INTS | FDCAN_RXERR_INTS | \
                             FDCAN_TXERR_INTS)

/* All non-reserved IR bits */

#define FDCAN_INT_ALL       0x3fcfffff

/* Debug ********************************************************************/

#undef STM32_FDCAN_LOOPBACK
#if defined(CONFIG_STM32_FDCAN1_LOOPBACK) || \
    defined(CONFIG_STM32_FDCAN2_LOOPBACK) || \
    defined(CONFIG_STM32_FDCAN3_LOOPBACK)
#  define STM32_FDCAN_LOOPBACK 1
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
  FDCAN_FD_MODE      = 1,   /* CAN FD operation */
  FDCAN_FD_BRS_MODE  = 2    /* CAN FD operation with bit rate switching */
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
  volatile uint32_t *txfifoq;        /* TX FIFO queue */
};

/* This structure provides the constant configuration of a FDCAN peripheral */

struct stm32_config_s
{
  uint32_t rxpinset;        /* RX pin configuration */
  uint32_t txpinset;        /* TX pin configuration */
  uintptr_t base;           /* Base address of the FDCAN registers */
  uint32_t baud;            /* Configured baud */
  uint32_t data_baud;       /* Configured data baud */
  uint32_t nbtp;            /* Nominal bit timing/prescaler register setting */
  uint32_t dbtp;            /* Data bit timing/prescaler register setting */
  uint8_t port;             /* FDCAN port number (1, 2 or 3) */
  uint8_t irq0;             /* FDCAN peripheral IRQ number for interrupt line 0 */
  uint8_t irq1;             /* FDCAN peripheral IRQ number for interrupt line 1 */
  uint8_t mode;             /* See enum stm32_canmode_e */
  uint8_t format;           /* See enum stm32_frameformat_e */
  bool autotiming;          /* True: compute bit timing from the bitrates */
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
  bool loopback;            /* Connection mode: loopback */
  bool silent;              /* Connection mode: bus monitoring */
  uint32_t nbtp;            /* Current nominal bit timing */
  uint32_t dbtp;            /* Current data bit timing */
  uint32_t rxints;          /* Configured RX interrupts */
  uint32_t txints;          /* Configured TX interrupts */
  uint32_t txpending;       /* TX buffers waiting for completion */

#ifdef CONFIG_CAN_EXTID
  uint32_t extfilters[(FDCAN_EXTFILTER_SIZE + 31) / 32]; /* Extended filter bit allocator */
#endif
  uint32_t stdfilters[(FDCAN_STDFILTER_SIZE + 31) / 32]; /* Standard filter bit allocator */
};

struct fdcan_bitseg
{
  uint32_t bitrate;
  uint8_t sjw;
  uint8_t bs1;
  uint8_t bs2;
  uint16_t prescaler;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* FDCAN Register access */

static uint32_t fdcan_getreg(struct stm32_fdcan_s *priv, int offset);
static void fdcan_putreg(struct stm32_fdcan_s *priv, int offset,
                         uint32_t regval);

/* FDCAN helpers */

static uint8_t fdcan_dlc2bytes(struct stm32_fdcan_s *priv, uint8_t dlc);
static void fdcan_initmode(struct stm32_fdcan_s *priv, bool enable);
static void fdcan_acceptall(struct stm32_fdcan_s *priv);
static void fdcan_clearfilters(struct stm32_fdcan_s *priv);

#ifdef CONFIG_CAN_EXTID
static int fdcan_add_extfilter(struct stm32_fdcan_s *priv,
                               struct canioc_extfilter_s *extconfig);
static int fdcan_del_extfilter(struct stm32_fdcan_s *priv, int ndx);
#endif
static int fdcan_add_stdfilter(struct stm32_fdcan_s *priv,
                               struct canioc_stdfilter_s *stdconfig);
static int fdcan_del_stdfilter(struct stm32_fdcan_s *priv, int ndx);

static int
fdcan_start_busoff_recovery_sequence(struct stm32_fdcan_s *priv);
static int fdcan_restart(struct can_dev_s *dev);

/* CAN driver methods */

static void fdcan_reset(struct can_dev_s *dev);
static int  fdcan_setup(struct can_dev_s *dev);
static void fdcan_shutdown(struct can_dev_s *dev);
static void fdcan_rxint(struct can_dev_s *dev, bool enable);
static void fdcan_txint(struct can_dev_s *dev, bool enable);
static int  fdcan_ioctl(struct can_dev_s *dev, int cmd,
                        unsigned long arg);
static int  fdcan_remoterequest(struct can_dev_s *dev, uint16_t id);
static int  fdcan_send(struct can_dev_s *dev, struct can_msg_s *msg);
static bool fdcan_txready(struct can_dev_s *dev);
static bool fdcan_txempty(struct can_dev_s *dev);

/* FDCAN interrupt handling */

#ifdef CONFIG_CAN_ERRORS
static void fdcan_error(struct can_dev_s *dev, uint32_t status,
                        uint32_t psr);
#endif
static void fdcan_receive(struct can_dev_s *dev,
                          volatile uint32_t *rxbuffer);
static void fdcan_rxfifo(struct can_dev_s *dev, int offset_rxfs,
                         int offset_rxfa, volatile uint32_t *rxfifo,
                         uint32_t *ie);
static int  fdcan_interrupt(int irq, void *context, void *arg);

/* Hardware initialization */

static int  fdcan_hw_initialize(struct stm32_fdcan_s *priv);
static int  fdcan_bittiming(struct fdcan_bitseg *timing);

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

/* The FDCAN instances share one RCC reset and one message RAM */

static bool g_fdcan_rccreset;

#ifdef CONFIG_STM32_FDCAN1
/* FDCAN1 constant configuration */

static const struct stm32_config_s g_fdcan1const =
{
  .rxpinset         = GPIO_CAN1_RX,
  .txpinset         = GPIO_CAN1_TX,
  .base             = STM32_FDCAN1_BASE,
  .baud             = CONFIG_STM32_FDCAN1_BITRATE,
#if defined(CONFIG_STM32_FDCAN1_FD_BRS)
  .data_baud        = CONFIG_STM32_FDCAN1_DBITRATE,
#endif
#ifndef CONFIG_STM32_FDCAN1_AUTO_BIT_TIMING
  .nbtp             = FDCAN_NBTP_VALUE(FDCAN1_NBRP, FDCAN1_NTSEG1,
                                       FDCAN1_NTSEG2, FDCAN1_NSJW),
  .dbtp             = FDCAN_DBTP_VALUE(FDCAN1_DBRP, FDCAN1_DTSEG1,
                                       FDCAN1_DTSEG2, FDCAN1_DSJW),
#else
  .autotiming       = true,
#endif
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
#ifdef CONFIG_STM32_FDCAN1_LOOPBACK
  .loopback         = true,
#endif

  /* FDCAN1 Message RAM */

  .msgram =
  {
    FDCAN_MSGRAM(1, FDCAN_STDFILTER_INDEX),
    FDCAN_MSGRAM(1, FDCAN_EXTFILTER_INDEX),
    FDCAN_MSGRAM(1, FDCAN_RXFIFO0_INDEX),
    FDCAN_MSGRAM(1, FDCAN_RXFIFO1_INDEX),
    FDCAN_MSGRAM(1, FDCAN_TXFIFOQ_INDEX)
  }
};

/* FDCAN1 variable driver state */

static struct stm32_fdcan_s g_fdcan1priv;
static struct can_dev_s g_fdcan1dev;

#endif /* CONFIG_STM32_FDCAN1 */

#ifdef CONFIG_STM32_FDCAN2
/* FDCAN2 constant configuration */

static const struct stm32_config_s g_fdcan2const =
{
  .rxpinset         = GPIO_CAN2_RX,
  .txpinset         = GPIO_CAN2_TX,
  .base             = STM32_FDCAN2_BASE,
  .baud             = CONFIG_STM32_FDCAN2_BITRATE,
#if defined(CONFIG_STM32_FDCAN2_FD_BRS)
  .data_baud        = CONFIG_STM32_FDCAN2_DBITRATE,
#endif
#ifndef CONFIG_STM32_FDCAN2_AUTO_BIT_TIMING
  .nbtp             = FDCAN_NBTP_VALUE(FDCAN2_NBRP, FDCAN2_NTSEG1,
                                       FDCAN2_NTSEG2, FDCAN2_NSJW),
  .dbtp             = FDCAN_DBTP_VALUE(FDCAN2_DBRP, FDCAN2_DTSEG1,
                                       FDCAN2_DTSEG2, FDCAN2_DSJW),
#else
  .autotiming       = true,
#endif
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
#ifdef CONFIG_STM32_FDCAN2_LOOPBACK
  .loopback         = true,
#endif

  /* FDCAN2 Message RAM */

  .msgram =
  {
    FDCAN_MSGRAM(2, FDCAN_STDFILTER_INDEX),
    FDCAN_MSGRAM(2, FDCAN_EXTFILTER_INDEX),
    FDCAN_MSGRAM(2, FDCAN_RXFIFO0_INDEX),
    FDCAN_MSGRAM(2, FDCAN_RXFIFO1_INDEX),
    FDCAN_MSGRAM(2, FDCAN_TXFIFOQ_INDEX)
  }
};

/* FDCAN2 variable driver state */

static struct stm32_fdcan_s g_fdcan2priv;
static struct can_dev_s g_fdcan2dev;

#endif /* CONFIG_STM32_FDCAN2 */

#ifdef CONFIG_STM32_FDCAN3
/* FDCAN3 constant configuration */

static const struct stm32_config_s g_fdcan3const =
{
  .rxpinset         = GPIO_CAN3_RX,
  .txpinset         = GPIO_CAN3_TX,
  .base             = STM32_FDCAN3_BASE,
  .baud             = CONFIG_STM32_FDCAN3_BITRATE,
#if defined(CONFIG_STM32_FDCAN3_FD_BRS)
  .data_baud        = CONFIG_STM32_FDCAN3_DBITRATE,
#endif
#ifndef CONFIG_STM32_FDCAN3_AUTO_BIT_TIMING
  .nbtp             = FDCAN_NBTP_VALUE(FDCAN3_NBRP, FDCAN3_NTSEG1,
                                       FDCAN3_NTSEG2, FDCAN3_NSJW),
  .dbtp             = FDCAN_DBTP_VALUE(FDCAN3_DBRP, FDCAN3_DTSEG1,
                                       FDCAN3_DTSEG2, FDCAN3_DSJW),
#else
  .autotiming       = true,
#endif
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
#ifdef CONFIG_STM32_FDCAN3_LOOPBACK
  .loopback         = true,
#endif

  /* FDCAN3 Message RAM */

  .msgram =
  {
    FDCAN_MSGRAM(3, FDCAN_STDFILTER_INDEX),
    FDCAN_MSGRAM(3, FDCAN_EXTFILTER_INDEX),
    FDCAN_MSGRAM(3, FDCAN_RXFIFO0_INDEX),
    FDCAN_MSGRAM(3, FDCAN_RXFIFO1_INDEX),
    FDCAN_MSGRAM(3, FDCAN_TXFIFOQ_INDEX)
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
 *   The register value
 *
 ****************************************************************************/

static uint32_t fdcan_getreg(struct stm32_fdcan_s *priv, int offset)
{
  return getreg32(priv->config->base + offset);
}

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

static void fdcan_putreg(struct stm32_fdcan_s *priv, int offset,
                         uint32_t regval)
{
  putreg32(regval, priv->config->base + offset);
}

/****************************************************************************
 * Name: fdcan_dlc2bytes
 *
 * Description:
 *   In the CAN FD format, the coding of the DLC differs from the standard
 *   CAN format. The DLC codes 0 to 8 have the same coding as in standard
 *   CAN, the codes 9 to 15, which in standard CAN all code a data field of
 *   8 bytes, are encoded:
 *
 *     9->12, 10->16, 11->20, 12->24, 13->32, 14->48, 15->64
 *
 * Input Parameters:
 *   priv - FDCAN instance state
 *   dlc  - the DLC value to convert to a byte count
 *
 * Returned Value:
 *   The number of bytes corresponding to the DLC value.
 *
 ****************************************************************************/

static uint8_t fdcan_dlc2bytes(struct stm32_fdcan_s *priv, uint8_t dlc)
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
 * Name: fdcan_initmode
 *
 * Description:
 *   Enter (with configuration change enabled) or leave initialization mode.
 *
 * Input Parameters:
 *   priv   - An instance of the FDCAN driver state structure.
 *   enable - True: enter initialization mode
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void fdcan_initmode(struct stm32_fdcan_s *priv, bool enable)
{
  uint32_t regval;

  if (enable)
    {
      regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
      regval |= FDCAN_CCCR_INIT;
      fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

      /* Wait for initialization mode to take effect */

      while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) &
              FDCAN_CCCR_INIT) == 0);

      /* Enable writing to configuration registers */

      regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
      regval |= FDCAN_CCCR_CCE;
      fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);
    }
  else
    {
      regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
      regval &= ~(FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
      fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

      /* Wait for normal operation (synchronized to the bus) */

      while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) &
              FDCAN_CCCR_INIT) != 0);
    }
}

/****************************************************************************
 * Name: fdcan_acceptall
 *
 * Description:
 *   Enable the reserved accept-all filters (to RX FIFO0) for each kind of
 *   identifier that has no user filter allocated, disable them otherwise.
 *   Filter elements can be changed at runtime, no initialization mode.
 *
 * Input Parameters:
 *   priv - An instance of the FDCAN driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void fdcan_acceptall(struct stm32_fdcan_s *priv)
{
  const struct stm32_config_s *config    = priv->config;
  volatile uint32_t           *stdfilter = NULL;
  volatile uint32_t           *extfilter = NULL;
  bool                         extall    = true;

  stdfilter = config->msgram.stdfilters + FDCAN_NSTDFILTERS;
  if (priv->nstdalloc == 0)
    {
      *stdfilter = STDFILTER_S0_SFT_CLASSIC | STDFILTER_S0_SFEC_FIFO0;
    }
  else
    {
      *stdfilter = STDFILTER_S0_SFEC_DISABLE;
    }

#ifdef CONFIG_CAN_EXTID
  extall = (priv->nextalloc == 0);
#endif

  extfilter = config->msgram.extfilters + (FDCAN_NEXTFILTERS << 1);
  if (extall)
    {
      extfilter[1] = EXTFILTER_F1_EFT_CLASSIC;
      extfilter[0] = EXTFILTER_F0_EFEC_FIFO0;
    }
  else
    {
      extfilter[0] = EXTFILTER_F0_EFEC_DISABLE;
    }
}

/****************************************************************************
 * Name: fdcan_clearfilters
 *
 * Description:
 *   Disable and release all filters of this instance.
 *
 * Input Parameters:
 *   priv - An instance of the FDCAN driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void fdcan_clearfilters(struct stm32_fdcan_s *priv)
{
  volatile uint32_t *msgram = priv->config->msgram.stdfilters;
  unsigned int       cntr   = 0;

  for (cntr = 0; cntr < FDCAN_RXFIFO0_INDEX; cntr++)
    {
      *msgram++ = 0;
    }

  priv->nstdalloc = 0;
  memset(priv->stdfilters, 0, sizeof(priv->stdfilters));
#ifdef CONFIG_CAN_EXTID
  priv->nextalloc = 0;
  memset(priv->extfilters, 0, sizeof(priv->extfilters));
#endif
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
static int fdcan_add_extfilter(struct stm32_fdcan_s *priv,
                               struct canioc_extfilter_s *extconfig)
{
  const struct stm32_config_s *config    = NULL;
  volatile uint32_t           *extfilter = NULL;
  uint32_t                     regval    = 0;
  uint32_t                     f1        = 0;
  int                          word      = 0;
  int                          bit       = 0;
  int                          ndx       = 0;

  DEBUGASSERT(priv != NULL && priv->config != NULL && extconfig != NULL);
  config = priv->config;

  if (extconfig->xf_id1 > CAN_MAX_EXTMSGID ||
      extconfig->xf_id2 > CAN_MAX_EXTMSGID)
    {
      return -EINVAL;
    }

  /* Format filter word F1 */

  f1 = EXTFILTER_F1_EFID2(extconfig->xf_id2);

  switch (extconfig->xf_type)
    {
      case CAN_FILTER_DUAL:
        {
          f1 |= EXTFILTER_F1_EFT_DUAL;
          break;
        }

      case CAN_FILTER_MASK:
        {
          f1 |= EXTFILTER_F1_EFT_CLASSIC;
          break;
        }

      case CAN_FILTER_RANGE:
        {
          f1 |= EXTFILTER_F1_EFT_RANGE;
          break;
        }

      default:
        {
          return -EINVAL;
        }
    }

  /* Find an unused extended filter */

  for (ndx = 0; ndx < FDCAN_NEXTFILTERS; ndx++)
    {
      /* Is this filter assigned? */

      word = ndx >> 5;
      bit  = ndx & 0x1f;

      if ((priv->extfilters[word] & (1 << bit)) == 0)
        {
          /* No, assign the filter */

          DEBUGASSERT(priv->nextalloc < FDCAN_NEXTFILTERS);
          priv->extfilters[word] |= (1 << bit);
          priv->nextalloc++;

          extfilter = config->msgram.extfilters + (ndx << 1);

          /* Format and write filter word F0 */

          regval = EXTFILTER_F0_EFID1(extconfig->xf_id1);

          if (extconfig->xf_prio == 0)
            {
              regval |= EXTFILTER_F0_EFEC_FIFO0;
            }
          else
            {
              regval |= EXTFILTER_F0_EFEC_FIFO1;
            }

          /* Write F1 first, F0 enables the element */

          extfilter[1] = f1;
          extfilter[0] = regval;

          /* The first extended filter stops accepting all frames */

          fdcan_acceptall(priv);

          return ndx;
        }
    }

  DEBUGASSERT(priv->nextalloc == FDCAN_NEXTFILTERS);
  return -EAGAIN;
}
#endif

/****************************************************************************
 * Name: fdcan_del_extfilter
 *
 * Description:
 *   Remove an address filter for a extended 29 bit address.
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
static int fdcan_del_extfilter(struct stm32_fdcan_s *priv, int ndx)
{
  const struct stm32_config_s *config    = NULL;
  volatile uint32_t           *extfilter = NULL;
  int                          word      = 0;
  int                          bit       = 0;

  DEBUGASSERT(priv != NULL && priv->config != NULL);
  config = priv->config;

  /* Check user Parameters */

  if (ndx < 0 || ndx >= FDCAN_NEXTFILTERS)
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

  /* After the last extended filter accept all frames again */

  fdcan_acceptall(priv);

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

static int fdcan_add_stdfilter(struct stm32_fdcan_s *priv,
                               struct canioc_stdfilter_s *stdconfig)
{
  const struct stm32_config_s *config    = NULL;
  volatile uint32_t           *stdfilter = NULL;
  uint32_t                     s0        = 0;
  int                          word      = 0;
  int                          bit       = 0;
  int                          ndx       = 0;

  DEBUGASSERT(priv != NULL && priv->config != NULL && stdconfig != NULL);
  config = priv->config;

  if (stdconfig->sf_id1 > CAN_MAX_STDMSGID ||
      stdconfig->sf_id2 > CAN_MAX_STDMSGID)
    {
      return -EINVAL;
    }

  /* Format filter word S0 */

  s0  = STDFILTER_S0_SFID1(stdconfig->sf_id1);
  s0 |= STDFILTER_S0_SFID2(stdconfig->sf_id2);

  if (stdconfig->sf_prio == 0)
    {
      s0 |= STDFILTER_S0_SFEC_FIFO0;
    }
  else
    {
      s0 |= STDFILTER_S0_SFEC_FIFO1;
    }

  switch (stdconfig->sf_type)
    {
      case CAN_FILTER_DUAL:
        {
          s0 |= STDFILTER_S0_SFT_DUAL;
          break;
        }

      case CAN_FILTER_MASK:
        {
          s0 |= STDFILTER_S0_SFT_CLASSIC;
          break;
        }

      case CAN_FILTER_RANGE:
        {
          s0 |= STDFILTER_S0_SFT_RANGE;
          break;
        }

      default:
        {
          return -EINVAL;
        }
    }

  /* Find an unused standard filter */

  for (ndx = 0; ndx < FDCAN_NSTDFILTERS; ndx++)
    {
      /* Is this filter assigned? */

      word = ndx >> 5;
      bit  = ndx & 0x1f;

      if ((priv->stdfilters[word] & (1 << bit)) == 0)
        {
          /* No, assign the filter */

          DEBUGASSERT(priv->nstdalloc < FDCAN_NSTDFILTERS);
          priv->stdfilters[word] |= (1 << bit);
          priv->nstdalloc++;

          stdfilter  = config->msgram.stdfilters + ndx;
          *stdfilter = s0;

          /* The first standard filter stops accepting all frames */

          fdcan_acceptall(priv);

          return ndx;
        }
    }

  DEBUGASSERT(priv->nstdalloc == FDCAN_NSTDFILTERS);
  return -EAGAIN;
}

/****************************************************************************
 * Name: fdcan_del_stdfilter
 *
 * Description:
 *   Remove an address filter for a standard 11 bit address.
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

static int fdcan_del_stdfilter(struct stm32_fdcan_s *priv, int ndx)
{
  const struct stm32_config_s *config    = NULL;
  volatile uint32_t           *stdfilter = NULL;
  int                          word      = 0;
  int                          bit       = 0;

  DEBUGASSERT(priv != NULL && priv->config != NULL);
  config = priv->config;

  /* Check Userspace Parameters */

  if (ndx < 0 || ndx >= FDCAN_NSTDFILTERS)
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

  /* After the last standard filter accept all frames again */

  fdcan_acceptall(priv);

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
fdcan_start_busoff_recovery_sequence(struct stm32_fdcan_s *priv)
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
 * Name: fdcan_restart
 *
 * Description:
 *   Reset and, if it was set up, re-initialize the FDCAN so that new bit
 *   timing or connection modes take effect.  Pending TX frames are lost.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int fdcan_restart(struct can_dev_s *dev)
{
  struct stm32_fdcan_s *priv  = dev->cd_priv;
  uint32_t              ie    = 0;
  uint8_t               state = 0;
  int                   ret   = OK;

  /* Save the current state so that we can recover to it */

  ie    = fdcan_getreg(priv, STM32_FDCAN_IE_OFFSET);
  state = priv->state;

  fdcan_reset(dev);

  if (state == FDCAN_STATE_SETUP)
    {
      ret = fdcan_setup(dev);
      if (ret == OK)
        {
          /* Restore RX interrupts.  The hardware was reset, so TX
           * interrupts are re-enabled by the next transmission.
           */

          fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, ie & ~priv->txints);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: fdcan_reset
 *
 * Description:
 *   Reset the FDCAN device.  Called early to initialize the hardware. This
 *   function is called, before fdcan_setup() and on error conditions.
 *
 *   The RCC reset is shared by all FDCAN instances, so only this instance
 *   is stopped here.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void fdcan_reset(struct can_dev_s *dev)
{
  struct stm32_fdcan_s        *priv   = NULL;
  const struct stm32_config_s *config = NULL;
  irqstate_t                   flags;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  caninfo("FDCAN%d\n", config->port);
  UNUSED(config);

  flags = enter_critical_section();

  /* Disable all interrupts */

  fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, 0);
  fdcan_putreg(priv, STM32_FDCAN_ILE_OFFSET, 0);
  fdcan_putreg(priv, STM32_FDCAN_TXBTIE_OFFSET, 0);

  /* Stop the controller.  Entering configuration mode also clears pending
   * TX requests.
   */

  if (priv->state != FDCAN_STATE_DISABLED)
    {
      fdcan_initmode(priv, true);
    }

  fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, FDCAN_INT_ALL);

  priv->txpending = 0;
  if (priv->state != FDCAN_STATE_DISABLED)
    {
      priv->state = FDCAN_STATE_RESET;
    }

  leave_critical_section(flags);
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

static int fdcan_setup(struct can_dev_s *dev)
{
  struct stm32_fdcan_s        *priv   = NULL;
  const struct stm32_config_s *config = NULL;
  int                          ret    = 0;

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

static void fdcan_shutdown(struct can_dev_s *dev)
{
  struct stm32_fdcan_s        *priv   = NULL;
  const struct stm32_config_s *config = NULL;
  uint32_t                     regval = 0;

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

  /* Enter initialization mode first, this aborts pending transmissions
   * that would otherwise hold off the clock stop (e.g. no ACK on the bus).
   */

  fdcan_initmode(priv, true);

  /* Disable device by setting the Clock Stop Request bit */

  regval = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
  regval |= FDCAN_CCCR_CSR;
  fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

  /* Wait for Clock Stop Acknowledge to verify device is powered down */

  while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) & FDCAN_CCCR_CSA)
         == 0);

  /* Release all filters, the next setup accepts all frames again */

  fdcan_clearfilters(priv);

  priv->txpending = 0;
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

static void fdcan_rxint(struct can_dev_s *dev, bool enable)
{
  struct stm32_fdcan_s *priv   = dev->cd_priv;
  uint32_t              regval = 0;
  irqstate_t            flags;

  DEBUGASSERT(priv && priv->config);

  caninfo("FDCAN%d enable: %d\n", priv->config->port, enable);

  /* Enable/disable the receive interrupts */

  flags  = enter_critical_section();
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

static void fdcan_txint(struct can_dev_s *dev, bool enable)
{
  struct stm32_fdcan_s *priv   = dev->cd_priv;
  uint32_t              regval = 0;
  irqstate_t            flags;

  DEBUGASSERT(priv && priv->config);

  caninfo("FDCAN%d enable: %d\n", priv->config->port, enable);

  /* Enable/disable the transmit interrupts */

  flags  = enter_critical_section();
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

static int fdcan_ioctl(struct can_dev_s *dev, int cmd, unsigned long arg)
{
  struct stm32_fdcan_s *priv = NULL;
  int                   ret  = -ENOTTY;

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
          struct canioc_bittiming_s *bt =
            (struct canioc_bittiming_s *)arg;
          uint32_t regval;
          uint32_t brp;

          DEBUGASSERT(bt != NULL);

#ifdef CONFIG_CAN_FD
          if (bt->type == CAN_BITTIMING_DATA)
            {
              regval       = priv->dbtp;
              bt->bt_sjw   = ((regval & FDCAN_DBTP_DSJW_MASK) >>
                              FDCAN_DBTP_DSJW_SHIFT) + 1;
              bt->bt_tseg1 = ((regval & FDCAN_DBTP_DTSEG1_MASK) >>
                              FDCAN_DBTP_DTSEG1_SHIFT) + 1;
              bt->bt_tseg2 = ((regval & FDCAN_DBTP_DTSEG2_MASK) >>
                              FDCAN_DBTP_DTSEG2_SHIFT) + 1;
              brp          = ((regval & FDCAN_DBTP_DBRP_MASK) >>
                              FDCAN_DBTP_DBRP_SHIFT) + 1;
            }
          else
#endif
            {
              regval       = priv->nbtp;
              bt->bt_sjw   = ((regval & FDCAN_NBTP_NSJW_MASK) >>
                              FDCAN_NBTP_NSJW_SHIFT) + 1;
              bt->bt_tseg1 = ((regval & FDCAN_NBTP_NTSEG1_MASK) >>
                              FDCAN_NBTP_NTSEG1_SHIFT) + 1;
              bt->bt_tseg2 = ((regval & FDCAN_NBTP_TSEG2_MASK) >>
                              FDCAN_NBTP_TSEG2_SHIFT) + 1;
              brp          = ((regval & FDCAN_NBTP_NBRP_MASK) >>
                              FDCAN_NBTP_NBRP_SHIFT) + 1;
            }

          bt->bt_baud = STM32_FDCANCLK / brp /
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
       * The FDCAN is reset to apply the new timing, pending TX frames are
       * lost.  Use only while the bus is quiet.
       */

      case CANIOC_SET_BITTIMING:
        {
          const struct canioc_bittiming_s *bt =
            (const struct canioc_bittiming_s *)arg;
          uint32_t ntq;
          uint32_t brp;

          DEBUGASSERT(bt != NULL);

          if (bt->bt_baud == 0 || bt->bt_tseg1 == 0 || bt->bt_tseg2 == 0 ||
              bt->bt_sjw == 0)
            {
              ret = -EINVAL;
              break;
            }

          /* The bitrate must be an exact multiple of the time quanta */

          ntq = bt->bt_tseg1 + bt->bt_tseg2 + 1;
          if (STM32_FDCANCLK % (ntq * bt->bt_baud) != 0)
            {
              ret = -EINVAL;
              break;
            }

          brp = STM32_FDCANCLK / (ntq * bt->bt_baud);

#ifdef CONFIG_CAN_FD
          if (bt->type == CAN_BITTIMING_DATA)
            {
              if (brp - 1 > FDCAN_DBTP_DBRP_MAX ||
                  bt->bt_tseg1 - 1 > FDCAN_DBTP_DTSEG1_MAX ||
                  bt->bt_tseg2 - 1 > FDCAN_DBTP_DTSEG2_MAX ||
                  bt->bt_sjw - 1 > FDCAN_DBTP_DSJW_MAX)
                {
                  ret = -EINVAL;
                  break;
                }

              priv->dbtp = FDCAN_DBTP_VALUE(brp - 1, bt->bt_tseg1 - 1,
                                            bt->bt_tseg2 - 1,
                                            bt->bt_sjw - 1);
            }
          else
#endif
            {
              if (brp - 1 > FDCAN_NBTP_NBRP_MAX ||
                  bt->bt_tseg1 - 1 > FDCAN_NBTP_NTSEG1_MAX ||
                  bt->bt_tseg2 - 1 > FDCAN_NBTP_NTSEG2_MAX ||
                  bt->bt_sjw - 1 > FDCAN_NBTP_NSJW_MAX)
                {
                  ret = -EINVAL;
                  break;
                }

              priv->nbtp = FDCAN_NBTP_VALUE(brp - 1, bt->bt_tseg1 - 1,
                                            bt->bt_tseg2 - 1,
                                            bt->bt_sjw - 1);
            }

          ret = fdcan_restart(dev);
        }
        break;

      /* CANIOC_GET_CONNMODES:
       *   Description:    Get the current bus connection modes
       *   Argument:       A pointer to a write-able instance of struct
       *                   canioc_connmodes_s in which the new bus modes will
       *                   be returned.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR)is returned with the errno variable set
       *                   to indicate the nature of the error.
       *   Dependencies:   None
       */

      case CANIOC_GET_CONNMODES:
        {
          struct canioc_connmodes_s *bm =
            (struct canioc_connmodes_s *)arg;

          DEBUGASSERT(bm != NULL);

          bm->bm_loopback = priv->loopback;
          bm->bm_silent   = priv->silent;
          ret = OK;
        }
        break;

      /* CANIOC_SET_CONNMODES:
       *   Description:    Set new bus connection modes values
       *   Argument:       A pointer to a read-able instance of struct
       *                   canioc_connmodes_s in which the new bus modes
       *                   are provided.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       *   Dependencies:   None
       */

      case CANIOC_SET_CONNMODES:
        {
          const struct canioc_connmodes_s *bm =
            (const struct canioc_connmodes_s *)arg;

          DEBUGASSERT(bm != NULL);

          priv->loopback = bm->bm_loopback;
          priv->silent   = bm->bm_silent;

          ret = fdcan_restart(dev);
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
                                   (struct canioc_extfilter_s *)arg);
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
                                   (struct canioc_stdfilter_s *)arg);
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

static int fdcan_remoterequest(struct can_dev_s *dev, uint16_t id)
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
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *   msg - The message to send
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int fdcan_send(struct can_dev_s *dev, struct can_msg_s *msg)
{
  struct stm32_fdcan_s        *priv     = NULL;
  const struct stm32_config_s *config   = NULL;
  volatile uint32_t           *txbuffer = NULL;
  const uint8_t               *src      = NULL;
  uint32_t                     regval   = 0;
  uint32_t                     t0       = 0;
  uint32_t                     t1       = 0;
  unsigned int                 ndx      = 0;
  unsigned int                 nbytes   = 0;
  unsigned int                 i        = 0;
  irqstate_t                   flags;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  caninfo("CAN%" PRIu8 " ID: %" PRIu32 " DLC: %" PRIu8 "\n",
          config->port, (uint32_t)msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

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
   *   FD format (FDF)                   - Frame transmitted in CAN FD format
   *   Event FIFO Control (EFC)          - Do not store events.
   *   Message Marker (MM)               - Always zero
   */

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid == 1)
    {
      DEBUGASSERT(msg->cm_hdr.ch_id <= CAN_MAX_EXTMSGID);

      t0 = BUFFER_R0_EXTID(msg->cm_hdr.ch_id) | BUFFER_R0_XTD;
    }
  else
#endif
    {
      DEBUGASSERT(msg->cm_hdr.ch_id <= CAN_MAX_STDMSGID);

      t0 = BUFFER_R0_STDID(msg->cm_hdr.ch_id);
    }

  if (msg->cm_hdr.ch_rtr == 1)
    {
      t0 |= BUFFER_R0_RTR;
    }

  t1 = BUFFER_R1_DLC(msg->cm_hdr.ch_dlc);

#ifdef CONFIG_CAN_FD
  /* CAN FD Format */

  if (msg->cm_hdr.ch_edl == 1)
    {
      t1 |= BUFFER_R1_FDF;

      if (msg->cm_hdr.ch_brs == 1)
        {
          t1 |= BUFFER_R1_BRS;
        }

      if (msg->cm_hdr.ch_esi == 1)
        {
          t0 |= BUFFER_R0_ESI;
        }
    }
#endif

  flags = enter_critical_section();

  /* Get our reserved Tx FIFO/queue put index */

  regval = fdcan_getreg(priv, STM32_FDCAN_TXFQS_OFFSET);
  if ((regval & FDCAN_TXFQS_TFQF) != 0)
    {
      leave_critical_section(flags);
      return -EBUSY;
    }

  ndx = (regval & FDCAN_TXFQS_TFQPI_MASK) >> FDCAN_TXFQS_TFQPI_SHIFT;
  DEBUGASSERT(ndx < FDCAN_TXFIFOQ_SIZE);

  /* And the TX buffer corresponding to this index */

  txbuffer    = config->msgram.txfifoq + ndx * FDCAN_ELEMENT_WORDS;
  txbuffer[0] = t0;
  txbuffer[1] = t1;

  /* Followed by the amount of data corresponding to the DLC (T2..).
   * Message RAM is written in words, little endian is assumed.
   */

  src    = msg->cm_data;
  nbytes = fdcan_dlc2bytes(priv, msg->cm_hdr.ch_dlc);

  for (i = 0; i < nbytes; i += 4, src += 4)
    {
      txbuffer[2 + (i >> 2)] = src[0] | (src[1] << 8) |
                               (src[2] << 16) | ((uint32_t)src[3] << 24);
    }

  /* And request to send the packet */

  priv->txpending |= (1 << ndx);
  fdcan_putreg(priv, STM32_FDCAN_TXBAR_OFFSET, (1 << ndx));

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

static bool fdcan_txready(struct can_dev_s *dev)
{
  struct stm32_fdcan_s *priv   = dev->cd_priv;
  uint32_t              regval = 0;

  /* Return TRUE if the TX FIFO/Queue is not full */

  regval = fdcan_getreg(priv, STM32_FDCAN_TXFQS_OFFSET);
  return (regval & FDCAN_TXFQS_TFQF) == 0;
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

static bool fdcan_txempty(struct can_dev_s *dev)
{
  struct stm32_fdcan_s *priv = dev->cd_priv;

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  /* No pending transmission request and no completion left to report */

  return priv->txpending == 0 &&
         fdcan_getreg(priv, STM32_FDCAN_TXBRP_OFFSET) == 0;
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
 *   psr        - Protocol status read with the interrupt (LEC/DLEC are
 *                reset on read)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_ERRORS
static void fdcan_error(struct can_dev_s *dev, uint32_t status,
                        uint32_t psr)
{
  struct can_hdr_s      hdr;
  uint32_t              lec     = 0;
  uint16_t              errbits = 0;
  uint8_t               data[CAN_ERROR_DLC];
  int                   ret     = 0;

  /* Encode error bits */

  errbits = 0;
  memset(data, 0, sizeof(data));

  /* Always fill in "static" error conditions, but set the signaling bit
   * only if the condition has changed (see IRQ-Flags below)
   * They have to be filled in every time CAN_ERROR_CONTROLLER is set.
   */

  if ((psr & FDCAN_PSR_EP) != 0)
    {
      data[1] |= (CAN_ERROR1_RXPASSIVE | CAN_ERROR1_TXPASSIVE);
    }

  if ((psr & FDCAN_PSR_EW) != 0)
    {
      data[1] |= (CAN_ERROR1_RXWARNING | CAN_ERROR1_TXWARNING);
    }

  if ((status & (FDCAN_IR_EP | FDCAN_IR_EW)) != 0)
    {
      /* "Error Passive" or "Error Warning" status changed */

      errbits |= CAN_ERROR_CONTROLLER;
    }

  /* Protocol error in arbitration or data phase.  LEC/DLEC hold an error
   * code, not a bit mask.
   */

  if ((status & (FDCAN_IR_PEA | FDCAN_IR_PED)) != 0)
    {
      if ((status & FDCAN_IR_PED) != 0)
        {
          lec = (psr & FDCAN_PSR_DLEC_MASK) >> FDCAN_PSR_DLEC_SHIFT;
        }
      else
        {
          lec = (psr & FDCAN_PSR_LEC_MASK) >> FDCAN_PSR_LEC_SHIFT;
        }

      switch (lec)
        {
          case FDCAN_PSR_EC_STUFF_ERROR:
            errbits |= CAN_ERROR_PROTOCOL;
            data[2] |= CAN_ERROR2_STUFF;
            break;

          case FDCAN_PSR_EC_FORM_ERROR:
            errbits |= CAN_ERROR_PROTOCOL;
            data[2] |= CAN_ERROR2_FORM;
            break;

          case FDCAN_PSR_EC_ACK_ERROR:
            errbits |= CAN_ERROR_NOACK;
            break;

          case FDCAN_PSR_EC_BIT0_ERROR:
            errbits |= CAN_ERROR_PROTOCOL;
            data[2] |= CAN_ERROR2_BIT0;
            break;

          case FDCAN_PSR_EC_BIT1_ERROR:
            errbits |= CAN_ERROR_PROTOCOL;
            data[2] |= CAN_ERROR2_BIT1;
            break;

          case FDCAN_PSR_EC_CRC_ERROR:
            errbits |= CAN_ERROR_PROTOCOL;
            data[3] |= (CAN_ERROR3_CRCSEQ | CAN_ERROR3_CRCDEL);
            break;

          default:
            break;
        }
    }

  if ((status & FDCAN_IR_BO) != 0)
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

  if ((status & (FDCAN_IR_RF0L | FDCAN_IR_RF1L)) != 0)
    {
      /* Receive FIFO 0/1 Message Lost */

      errbits |= CAN_ERROR_CONTROLLER;
      data[1] |= CAN_ERROR1_RXOVERFLOW;
    }

  if ((status & FDCAN_IR_TEFL) != 0)
    {
      /* Tx Event FIFO Element Lost */

      errbits |= CAN_ERROR_CONTROLLER;
      data[1] |= CAN_ERROR1_TXOVERFLOW;
    }

  if ((status & FDCAN_IR_TOO) != 0)
    {
      /* Timeout Occurred */

      errbits |= CAN_ERROR_TXTIMEOUT;
    }

  if ((status & (FDCAN_IR_MRAF | FDCAN_IR_ELO)) != 0)
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

      memset(&hdr, 0, sizeof(hdr));
      hdr.ch_id     = errbits;
      hdr.ch_dlc    = CAN_ERROR_DLC;
      hdr.ch_error  = 1;

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
 *   rxbuffer - The RX FIFO element containing the received message
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void fdcan_receive(struct can_dev_s *dev,
                          volatile uint32_t *rxbuffer)
{
  struct stm32_fdcan_s *priv = dev->cd_priv;
  struct can_hdr_s      hdr;
  uint32_t              data[(FDCAN_ELEMENT_WORDS - 2)];
  uint32_t              r0;
  uint32_t              r1;
  unsigned int          nwords;
  unsigned int          i;
  int                   ret;
#ifdef CONFIG_CAN_TIMESTAMP
  struct timespec       ts;

  clock_systime_timespec(&ts);
#endif

  memset(&hdr, 0, sizeof(hdr));

  /* Word R0 contains the CAN ID, word R1 the DLC and frame format */

  r0 = rxbuffer[0];
  r1 = rxbuffer[1];

  /* Extract the RTR bit */

  hdr.ch_rtr = ((r0 & BUFFER_R0_RTR) != 0);

#ifdef CONFIG_CAN_EXTID
  if ((r0 & BUFFER_R0_XTD) != 0)
    {
      /* Save the extended ID of the newly received message */

      hdr.ch_id    = (r0 & BUFFER_R0_EXTID_MASK) >> BUFFER_R0_EXTID_SHIFT;
      hdr.ch_extid = 1;
    }
  else
    {
      hdr.ch_id    = (r0 & BUFFER_R0_STDID_MASK) >> BUFFER_R0_STDID_SHIFT;
      hdr.ch_extid = 0;
    }

#else
  if ((r0 & BUFFER_R0_XTD) != 0)
    {
      /* Drop any messages with extended IDs */

      canerr("ERROR: Received message with extended identifier.  Dropped\n");

      return;
    }

  /* Save the standard ID of the newly received message */

  hdr.ch_id = (r0 & BUFFER_R0_STDID_MASK) >> BUFFER_R0_STDID_SHIFT;
#endif

  hdr.ch_dlc = (r1 & BUFFER_R1_DLC_MASK) >> BUFFER_R1_DLC_SHIFT;

#ifdef CONFIG_CAN_FD
  /* CAN FD format */

  hdr.ch_esi = ((r0 & BUFFER_R0_ESI) != 0);
  hdr.ch_edl = ((r1 & BUFFER_R1_FDF) != 0);
  hdr.ch_brs = ((r1 & BUFFER_R1_BRS) != 0);
#else
  if ((r1 & BUFFER_R1_FDF) != 0)
    {
      /* Drop any FD CAN messages if not supported */

      canerr("ERROR: Received CAN FD message.  Dropped\n");

      return;
    }
#endif

#ifdef CONFIG_CAN_TIMESTAMP
  hdr.ch_ts.tv_sec  = ts.tv_sec;
  hdr.ch_ts.tv_usec = ts.tv_nsec / 1000;
#endif

  /* Copy the payload out of message RAM with word accesses */

  nwords = (fdcan_dlc2bytes(priv, hdr.ch_dlc) + 3) >> 2;
  for (i = 0; i < nwords; i++)
    {
      data[i] = rxbuffer[2 + i];
    }

  /* And provide the CAN message to the upper half logic */

  ret = can_receive(dev, &hdr, (uint8_t *)data);
  if (ret < 0)
    {
      canerr("ERROR: can_receive failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: fdcan_rxfifo
 *
 * Description:
 *   Drain one RX FIFO
 *
 * Input Parameters:
 *   dev         - CAN-common state data
 *   offset_rxfs - RX FIFO status register offset
 *   offset_rxfa - RX FIFO acknowledge register offset
 *   rxfifo      - RX FIFO in message RAM
 *   ie          - Current interrupt enable value (updated)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void fdcan_rxfifo(struct can_dev_s *dev, int offset_rxfs,
                         int offset_rxfa, volatile uint32_t *rxfifo,
                         uint32_t *ie)
{
  struct stm32_fdcan_s *priv   = dev->cd_priv;
  uint32_t              regval = 0;
  unsigned int          ndx    = 0;

  /* RXF0S and RXF1S share the same layout */

  for (; ; )
    {
      regval = fdcan_getreg(priv, offset_rxfs);
      if ((regval & FDCAN_RXF0S_F0FL_MASK) == 0)
        {
          break;
        }

      ndx = (regval & FDCAN_RXF0S_F0GI_MASK) >> FDCAN_RXF0S_F0GI_SHIFT;

      fdcan_receive(dev, rxfifo + ndx * FDCAN_ELEMENT_WORDS);

      /* Acknowledge reading the FIFO entry */

      fdcan_putreg(priv, offset_rxfa, ndx);

      /* Turning back on all configured RX error interrupts */

      if ((*ie & FDCAN_RXERR_INTS) != (priv->rxints & FDCAN_RXERR_INTS))
        {
          *ie |= (priv->rxints & FDCAN_RXERR_INTS);
          fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, *ie);
        }
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

static int fdcan_interrupt(int irq, void *context, void *arg)
{
  struct can_dev_s            *dev     = (struct can_dev_s *)arg;
  struct stm32_fdcan_s        *priv    = NULL;
  const struct stm32_config_s *config  = NULL;
  uint32_t                     ir      = 0;
  uint32_t                     ie      = 0;
  uint32_t                     pending = 0;
  uint32_t                     done    = 0;
  uint32_t                     psr     = 0;

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
      /* LEC/DLEC are reset on read, read PSR once */

      psr = fdcan_getreg(priv, STM32_FDCAN_PSR_OFFSET);

      /* Check for common errors */

      if ((pending & FDCAN_CMNERR_INTS) != 0)
        {
          canerr("ERROR: Common %08" PRIx32 "\n",
                 pending & FDCAN_CMNERR_INTS);

          /* When a protocol error occurs, the problem is recorded in
           * the LEC/DLEC fields of the PSR register. In lieu of
           * separate interrupt flags for each error, the hardware
           * groups protocol errors under a single interrupt each for
           * arbitration and data phases.
           *
           * These errors have a tendency to flood the system with
           * interrupts, so they are disabled here until we get a
           * successful transfer/receive on the hardware
           */

          if ((psr & FDCAN_PSR_LEC_MASK) != 0)
            {
              canerr("ERROR: PSR %08" PRIx32 "\n", psr);
              ie &= ~(FDCAN_IR_PEA | FDCAN_IR_PED);
              fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, ie);
              caninfo("disabled protocol error interrupts\n");
            }

          /* Clear the error indications */

          fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, FDCAN_CMNERR_INTS);
        }

      /* Check for transmission errors */

      if ((pending & FDCAN_TXERR_INTS) != 0)
        {
          canerr("ERROR: TX %08" PRIx32 "\n",
                 pending & FDCAN_TXERR_INTS);

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

      fdcan_error(dev, pending & FDCAN_ANYERR_INTS, psr);
#endif
    }

  /* Check for successful completion of a transmission */

  if ((pending & FDCAN_IR_TC) != 0)
    {
      /* Check if we have disabled the protocol error interrupts above
       * to prevent Interrupt-Flooding and re-enable them here again.
       */

      if ((ie & (FDCAN_IR_PEA | FDCAN_IR_PED)) == 0)
        {
          ie |= (FDCAN_IR_PEA | FDCAN_IR_PED);
          fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, ie);
          caninfo("Re-enabled protocol error interrupts\n");
        }

      /* Clear the pending TX completion interrupt (and all
       * other TX-related interrupts)
       */

      fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, priv->txints);

      /* TXBTO stays set until a buffer is reused, so only count buffers
       * that are still pending.  Release them all before notifying the
       * upper half, which may queue new frames into the released buffers.
       */

      done = fdcan_getreg(priv, STM32_FDCAN_TXBTO_OFFSET) & priv->txpending;
      priv->txpending &= ~done;

      for (; done != 0; done &= done - 1)
        {
          /* Tell the upper half that the transfer is finished. */

          can_txdone(dev);
        }
    }
  else if ((pending & priv->txints) != 0)
    {
      /* Clear unhandled TX events */

      fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, priv->txints);
    }

  /* Clear the RX new message interrupts before draining the FIFOs, so that
   * a message arriving meanwhile raises a new interrupt.
   */

  fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, FDCAN_IR_RF0N | FDCAN_IR_RF1N);

  /* We treat RX FIFO1 as the "high priority" queue:  We will process
   * all messages in RX FIFO1 before processing any message from RX
   * FIFO0.
   */

  fdcan_rxfifo(dev, STM32_FDCAN_RXF1S_OFFSET, STM32_FDCAN_RXF1A_OFFSET,
               config->msgram.rxfifo1, &ie);
  fdcan_rxfifo(dev, STM32_FDCAN_RXF0S_OFFSET, STM32_FDCAN_RXF0A_OFFSET,
               config->msgram.rxfifo0, &ie);

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
  const struct stm32_config_s *config = priv->config;
  uint32_t                     regval = 0;
  uint32_t                     dbtp   = 0;

  caninfo("FDCAN%d\n", config->port);

  /* Configure FDCAN pins */

  stm32_configgpio(config->rxpinset);
  stm32_configgpio(config->txpinset);

  /* Re-enable device if previously disabled in fdcan_shutdown() */

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

  /* Enable the Initialization state and configuration changes */

  fdcan_initmode(priv, true);

  /* Global Filter Configuration:
   *
   *   ANFS=2: Reject non matching standard frames
   *   ANFE=2: Reject non matching extended frames
   *
   * Accepting all frames is done by the reserved filters, so filters can
   * change without leaving normal operation.  Allocated filters are kept
   * across a restart.
   */

  fdcan_putreg(priv, STM32_FDCAN_GFC_OFFSET,
               (2 << FDCAN_GFC_ANFS_SHIFT) | (2 << FDCAN_GFC_ANFE_SHIFT));
  fdcan_acceptall(priv);

  /* Extended ID Filter AND mask  */

  fdcan_putreg(priv, STM32_FDCAN_XIDAM_OFFSET, 0x1fffffff);

  /* Disable all interrupts  */

  fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, 0);
  fdcan_putreg(priv, STM32_FDCAN_TXBTIE_OFFSET, 0);

  /* All interrupts directed to Line 0.  But disable both interrupt lines 0
   * and 1 for now.
   */

  fdcan_putreg(priv, STM32_FDCAN_ILS_OFFSET, 0);
  fdcan_putreg(priv, STM32_FDCAN_ILE_OFFSET, 0);

  /* Clear all pending interrupts. */

  fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, FDCAN_INT_ALL);

  /* Configure FDCAN bit timing.  Transmitter delay compensation is needed
   * for data phase bitrates above ~1 Mbit/s and works with a data
   * prescaler of 1 or 2 only.  The offset is the sample point in mtq.
   */

  dbtp = priv->dbtp;
  if (config->mode == FDCAN_FD_BRS_MODE)
    {
      uint32_t dbrp   = (dbtp & FDCAN_DBTP_DBRP_MASK) >>
                        FDCAN_DBTP_DBRP_SHIFT;
      uint32_t dtseg1 = (dbtp & FDCAN_DBTP_DTSEG1_MASK) >>
                        FDCAN_DBTP_DTSEG1_SHIFT;
      uint32_t tdco   = (dtseg1 + 2) * (dbrp + 1);

      if (dbrp <= 1 && tdco <= FDCAN_TDCR_TDCO_MAX)
        {
          fdcan_putreg(priv, STM32_FDCAN_TDCR_OFFSET,
                       tdco << FDCAN_TDCR_TDCO_SHIFT);
          dbtp |= FDCAN_DBTP_TDC;
        }
    }

  fdcan_putreg(priv, STM32_FDCAN_NBTP_OFFSET, priv->nbtp);
  fdcan_putreg(priv, STM32_FDCAN_DBTP_OFFSET, dbtp);

  /* Configure message RAM start addresses (byte offsets) and sizes */

  regval  = ((uintptr_t)config->msgram.stdfilters - STM32_CANRAM_BASE) &
            FDCAN_SIDFC_FLSSA_MASK;
  regval |= FDCAN_STDFILTER_SIZE << FDCAN_SIDFC_LSS_SHIFT;
  fdcan_putreg(priv, STM32_FDCAN_SIDFC_OFFSET, regval);

  regval  = ((uintptr_t)config->msgram.extfilters - STM32_CANRAM_BASE) &
            FDCAN_XIDFC_FLESA_MASK;
  regval |= FDCAN_EXTFILTER_SIZE << FDCAN_XIDFC_LSE_SHIFT;
  fdcan_putreg(priv, STM32_FDCAN_XIDFC_OFFSET, regval);

  regval  = ((uintptr_t)config->msgram.rxfifo0 - STM32_CANRAM_BASE) &
            FDCAN_RXF0C_F0SA_MASK;
  regval |= FDCAN_RXFIFO0_SIZE << FDCAN_RXF0C_F0S_SHIFT;
  fdcan_putreg(priv, STM32_FDCAN_RXF0C_OFFSET, regval);

  regval  = ((uintptr_t)config->msgram.rxfifo1 - STM32_CANRAM_BASE) &
            FDCAN_RXF1C_F1SA_MASK;
  regval |= FDCAN_RXFIFO1_SIZE << FDCAN_RXF1C_F1S_SHIFT;
  fdcan_putreg(priv, STM32_FDCAN_RXF1C_OFFSET, regval);

  /* No dedicated RX buffers and no TX event FIFO */

  fdcan_putreg(priv, STM32_FDCAN_RXBC_OFFSET, 0);
  fdcan_putreg(priv, STM32_FDCAN_TXEFC_OFFSET, 0);

  /* Element data field sizes */

  regval = (FDCAN_ELEMENT_DS << FDCAN_RXESC_F0DS_SHIFT) |
           (FDCAN_ELEMENT_DS << FDCAN_RXESC_F1DS_SHIFT) |
           (FDCAN_ELEMENT_DS << FDCAN_RXESC_RBDS_SHIFT);
  fdcan_putreg(priv, STM32_FDCAN_RXESC_OFFSET, regval);
  fdcan_putreg(priv, STM32_FDCAN_TXESC_OFFSET,
               FDCAN_ELEMENT_DS << FDCAN_TXESC_TBDS_SHIFT);

  /* TX FIFO/queue, no dedicated TX buffers */

  regval  = ((uintptr_t)config->msgram.txfifoq - STM32_CANRAM_BASE) &
            FDCAN_TXBC_TBSA_MASK;
  regval |= FDCAN_TXFIFOQ_SIZE << FDCAN_TXBC_TFQS_SHIFT;
#ifdef CONFIG_STM32_FDCAN_QUEUE_MODE
  regval |= FDCAN_TXBC_TFQM;
#endif
  fdcan_putreg(priv, STM32_FDCAN_TXBC_OFFSET, regval);

  /* CC control register */

  regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
  regval &= ~(FDCAN_CCCR_NISO | FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE |
              FDCAN_CCCR_TEST | FDCAN_CCCR_MON);

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

  /* Connection modes.  Loopback with bus monitoring is the internal
   * loopback (TX and RX disconnected from the bus).
   *
   *   FDCAN_CCCR_TEST  - Test mode enable
   *   FDCAN_CCCR_MON   - Bus monitoring mode
   *   FDCAN_TEST_LBCK  - Loopback mode
   */

  if (priv->silent)
    {
      regval |= FDCAN_CCCR_MON;
    }

  if (priv->loopback)
    {
      regval |= FDCAN_CCCR_TEST;
    }

  fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);
  fdcan_putreg(priv, STM32_FDCAN_TEST_OFFSET,
               priv->loopback ? FDCAN_TEST_LBCK : 0);

  /* Select RX-related interrupts */

  priv->rxints = FDCAN_RXFIFO_INTS;

  /* Select TX-related interrupts */

  priv->txints = FDCAN_TXFIFOQ_INTS;

  /* Transmission completed interrupts for all TX buffers */

  fdcan_putreg(priv, STM32_FDCAN_TXBTIE_OFFSET, FDCAN_TXBUFFERS_ALL);

  /* Enable only interrupt line 0.  ILS was cleared above, so all
   * interrupts are directed to line 0.
   */

  fdcan_putreg(priv, STM32_FDCAN_ILE_OFFSET, FDCAN_ILE_EINT0);

  /* Disable initialization mode to enable normal operation */

  priv->txpending = 0;
  fdcan_initmode(priv, false);

  return OK;
}

/****************************************************************************
 * Name: fdcan_bittiming
 *
 * Description:
 *   Convert desired bitrate to FDCAN bit segment values
 *   The computed values apply to both data and arbitration phases
 *
 * Input Parameters:
 *   timing - structure to store bit timing
 *
 * Returned Value:
 *   OK on success; >0 on failure.
 ****************************************************************************/

static int fdcan_bittiming(struct fdcan_bitseg *timing)
{
  /* Implementation ported from PX4's uavcan_drivers/stm32[h7]
   *
   * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe
   *  MicroControl GmbH & Co. KG
   *  CAN in Automation, 2003
   *
   * According to the source, optimal quanta per bit are:
   *   Bitrate        Optimal Maximum
   *   1000 kbps      8       10
   *   500  kbps      16      17
   *   250  kbps      16      17
   *   125  kbps      16      17
   */

  const uint32_t target_bitrate    = timing->bitrate;
  static const int32_t max_bs1     = 16;
  static const int32_t max_bs2     = 8;
  const uint8_t max_quanta_per_bit = (timing->bitrate >= 1000000) ? 10 : 17;
  static const int max_sp_location = 900;

  /* Computing (prescaler * BS):
   *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))
   *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))
   * let:
   *   BS = 1 + BS1 + BS2
   *     (BS == total number of time quanta per bit)
   *   PRESCALER_BS = PRESCALER * BS
   * ==>
   *   PRESCALER_BS = PCLK / BITRATE
   */

  if (target_bitrate == 0)
    {
      return 1;
    }

  const uint32_t prescaler_bs = STM32_FDCANCLK / target_bitrate;

  /* Find prescaler value such that the number of quanta per bit is highest */

  uint8_t bs1_bs2_sum = max_quanta_per_bit - 1;

  while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0)
    {
      if (bs1_bs2_sum <= 2)
        {
          canerr("Target bitrate too high - no solution possible.");
          return 1; /* No solution */
        }

      bs1_bs2_sum--;
    }

  const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);

  if ((prescaler < 1U) || (prescaler > 512U))
    {
      canerr("Target bitrate invalid - bad prescaler.");
      return 2; /* No solution */
    }

  /* Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
   * We need to find the values so that the sample point is as close as
   * possible to the optimal value.
   *
   *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]
   *     (Where 7/8 is 0.875, the recommended sample point location)
   *   {{bs2 -> (1 + bs1)/7}}
   *
   * Hence:
   *   bs2 = (1 + bs1) / 7
   *   bs1 = (7 * bs1_bs2_sum - 1) / 8
   *
   * Sample point location can be computed as follows:
   *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
   *
   * Since the optimal solution is so close to the maximum, we prepare two
   * solutions, and then pick the best one:
   *   - With rounding to nearest
   *   - With rounding to zero
   */

  /* First attempt with rounding to nearest */

  uint8_t bs1 = (uint8_t)((7 * bs1_bs2_sum - 1) + 4) / 8;
  uint8_t bs2 = (uint8_t)(bs1_bs2_sum - bs1);
  uint16_t sample_point_permill =
    (uint16_t)(1000 * (1 + bs1) / (1 + bs1 + bs2));

  if (sample_point_permill > max_sp_location)
    {
      /* Second attempt with rounding to zero */

      bs1 = (7 * bs1_bs2_sum - 1) / 8;
      bs2 = bs1_bs2_sum - bs1;
    }

  bool valid = (bs1 >= 1) && (bs1 <= max_bs1) && (bs2 >= 1) &&
    (bs2 <= max_bs2);

  /* Final validation */

  if (target_bitrate != (STM32_FDCANCLK /
                        (prescaler * (1 + bs1 + bs2))) || !valid)
    {
      canerr("Target bitrate invalid - solution does not match.");
      return 3; /* Solution not found */
    }

  timing->bs1 = (uint8_t)(bs1 - 1);
  timing->bs2 = (uint8_t)(bs2 - 1);
  timing->prescaler = (uint16_t)(prescaler - 1);
  timing->sjw = 0; /* Which means one */

  return 0;
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

struct can_dev_s *stm32_fdcaninitialize(int port)
{
  struct can_dev_s            *dev    = NULL;
  struct stm32_fdcan_s        *priv   = NULL;
  const struct stm32_config_s *config = NULL;
  struct fdcan_bitseg          timing;
  irqstate_t                   flags;

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

  /* The RCC reset is shared by all instances, reset only once */

  flags = enter_critical_section();
  if (!g_fdcan_rccreset)
    {
      modifyreg32(STM32_RCC_APB1HRSTR, 0, RCC_APB1HRSTR_FDCANRST);
      modifyreg32(STM32_RCC_APB1HRSTR, RCC_APB1HRSTR_FDCANRST, 0);
      g_fdcan_rccreset = true;
    }

  leave_critical_section(flags);

  /* Perform one time data initialization */

  memset(priv, 0, sizeof(struct stm32_fdcan_s));
  priv->config = config;
  fdcan_clearfilters(priv);

#ifdef STM32_FDCAN_LOOPBACK
  /* Internal loopback: TX and RX disconnected from the bus */

  priv->loopback = config->loopback;
  priv->silent   = config->loopback;
#endif

  /* Set the initial bit timing.  This might change subsequently
   * due to IOCTL command processing.
   */

  if (!config->autotiming)
    {
      priv->nbtp = config->nbtp;
      priv->dbtp = config->dbtp;
    }
  else
    {
      timing.bitrate = config->baud;
      if (fdcan_bittiming(&timing) != OK)
        {
          canerr("ERROR: Invalid CAN nominal phase timings\n");
          return NULL;
        }

      priv->nbtp = FDCAN_NBTP_VALUE(timing.prescaler, timing.bs1,
                                    timing.bs2, timing.sjw);

      if (config->mode == FDCAN_FD_BRS_MODE)
        {
          timing.bitrate = config->data_baud;
          if (fdcan_bittiming(&timing) != OK ||
              timing.prescaler > FDCAN_DBTP_DBRP_MAX ||
              timing.bs1 > FDCAN_DBTP_DTSEG1_MAX ||
              timing.bs2 > FDCAN_DBTP_DTSEG2_MAX)
            {
              canerr("ERROR: Invalid CAN data phase timings\n");
              return NULL;
            }

          priv->dbtp = FDCAN_DBTP_VALUE(timing.prescaler, timing.bs1,
                                        timing.bs2, timing.sjw);
        }
      else
        {
          priv->dbtp = 0;
        }
    }

  dev->cd_ops  = &g_fdcanops;
  dev->cd_priv = (void *)priv;

  /* And put the hardware in the initial state */

  fdcan_reset(dev);

  return dev;
}

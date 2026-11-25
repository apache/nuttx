/****************************************************************************
 * arch/arm/src/common/stm32/stm32_fdcan_mcan_v1_sock.c
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

#include <stdio.h>
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
#include <nuttx/semaphore.h>
#include <nuttx/can.h>
#include <nuttx/net/can.h>
#include <nuttx/net/netdev_lowerhalf.h>

#include "arm_internal.h"
#include "stm32_fdcan.h"
#include "hardware/stm32_pinmap.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NET_CAN_CANFD
#  define FDCAN_HAVE_FD 1
#endif

#ifdef CONFIG_NET_CAN_EXTID
#  define FDCAN_HAVE_EXTID 1
#endif

#ifdef CONFIG_NET_CAN_ERRORS
#  define FDCAN_HAVE_ERRORS 1
#endif

/* Configure FDCAN TQ Clock *************************************************/

/* Define STM32_FDCAN_PDIV (FDCAN_CKDIV PDIV[3:0] Value) in board.h */

#ifdef STM32_FDCAN_PDIV
#  define FDCANCLK_PDIV              STM32_FDCAN_PDIV
#else
#  define FDCANCLK_PDIV              (0)
#endif

/* Define STM32_FDCAN_FREQUENCY (fdcan_ker_ck) in board.h */

#if FDCANCLK_PDIV == 0
#  define STM32_FDCANCLK_FREQUENCY (STM32_FDCAN_FREQUENCY / (1))
#else
#  define STM32_FDCANCLK_FREQUENCY (STM32_FDCAN_FREQUENCY / (2 * FDCANCLK_PDIV))
#endif

/* General Configuration ****************************************************/

/* FDCAN Message RAM */

/* Each instance has 212 words of message RAM, one after the other */

#define FDCAN_MSGRAM_WORDS           (212)

#if defined(STM32_FDCAN_SRAM_BASE)
#  define FDCAN_SRAM_BASE            STM32_FDCAN_SRAM_BASE
#elif defined(STM32_SRAMCAN_BASE)
#  define FDCAN_SRAM_BASE            STM32_SRAMCAN_BASE
#else
#  define FDCAN_SRAM_BASE            STM32_CANRAM_BASE
#endif

#define STM32_CANRAM1_BASE           (FDCAN_SRAM_BASE + 0*(FDCAN_MSGRAM_WORDS * 4))
#define STM32_CANRAM2_BASE           (FDCAN_SRAM_BASE + 1*(FDCAN_MSGRAM_WORDS * 4))
#define STM32_CANRAM3_BASE           (FDCAN_SRAM_BASE + 2*(FDCAN_MSGRAM_WORDS * 4))

/* Interrupt lines */

#ifdef STM32_IRQ_FDCAN1_IT0
#  define FDCAN1_IRQ0                STM32_IRQ_FDCAN1_IT0
#  define FDCAN1_IRQ1                STM32_IRQ_FDCAN1_IT1
#  define FDCAN2_IRQ0                STM32_IRQ_FDCAN2_IT0
#  define FDCAN2_IRQ1                STM32_IRQ_FDCAN2_IT1
#  define FDCAN3_IRQ0                STM32_IRQ_FDCAN3_IT0
#  define FDCAN3_IRQ1                STM32_IRQ_FDCAN3_IT1
#else
#  define FDCAN1_IRQ0                STM32_IRQ_FDCAN1_0
#  define FDCAN1_IRQ1                STM32_IRQ_FDCAN1_1
#  define FDCAN2_IRQ0                STM32_IRQ_FDCAN2_0
#  define FDCAN2_IRQ1                STM32_IRQ_FDCAN2_1
#  define FDCAN3_IRQ0                STM32_IRQ_FDCAN3_0
#  define FDCAN3_IRQ1                STM32_IRQ_FDCAN3_1
#endif

/* The G0 headers define FDCAN1 only */

#if defined(CONFIG_ARCH_CHIP_STM32G0) && defined(CONFIG_STM32_FDCAN2)
#  error "FDCAN2 is not supported on STM32G0"
#endif

/* RCC reset of all FDCAN instances */

#if defined(RCC_APB1HRSTR_FDCANRST)
#  define FDCAN_RSTR                 STM32_RCC_APB1HRSTR
#  define FDCAN_RST                  RCC_APB1HRSTR_FDCANRST
#elif defined(RCC_APB1RSTR1_FDCANRST)
#  define FDCAN_RSTR                 STM32_RCC_APB1RSTR1
#  define FDCAN_RST                  RCC_APB1RSTR1_FDCANRST
#elif defined(RCC_APB1RSTR2_FDCANRST)
#  define FDCAN_RSTR                 STM32_RCC_APB1RSTR2
#  define FDCAN_RST                  RCC_APB1RSTR2_FDCANRST
#else
#  define FDCAN_RSTR                 STM32_RCC_APB1RSTR
#  define FDCAN_RST                  RCC_APB1RSTR_FDCANRST
#endif

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

/* FDCAN1 Configuration *****************************************************/

#ifdef CONFIG_STM32_FDCAN1

/* Bit timing */

#  ifndef CONFIG_STM32_FDCAN1_AUTO_BIT_TIMING

#    define FDCAN1_NTSEG1  (CONFIG_STM32_FDCAN1_NTSEG1 - 1)
#    define FDCAN1_NTSEG2  (CONFIG_STM32_FDCAN1_NTSEG2 - 1)
#    define FDCAN1_NBRP    ((STM32_FDCANCLK_FREQUENCY /             \
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
#      define FDCAN1_DBRP   ((STM32_FDCANCLK_FREQUENCY /             \
                              ((FDCAN1_DTSEG1 + FDCAN1_DTSEG2 + 3) * \
                               CONFIG_STM32_FDCAN1_DBITRATE)) - 1)
#      define FDCAN1_DSJW   (CONFIG_STM32_FDCAN1_DSJW - 1)
#    else
#      define FDCAN1_DTSEG1 1
#      define FDCAN1_DTSEG2 1
#      define FDCAN1_DBRP   1
#      define FDCAN1_DSJW   1
#    endif /* CONFIG_STM32_FDCAN1_FD_BRS */

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

#  ifndef CONFIG_STM32_FDCAN2_AUTO_BIT_TIMING

#    define FDCAN2_NTSEG1  (CONFIG_STM32_FDCAN2_NTSEG1 - 1)
#    define FDCAN2_NTSEG2  (CONFIG_STM32_FDCAN2_NTSEG2 - 1)
#    define FDCAN2_NBRP    (((STM32_FDCANCLK_FREQUENCY /              \
                            ((FDCAN2_NTSEG1 + FDCAN2_NTSEG2 + 3) *  \
                             CONFIG_STM32_FDCAN2_BITRATE)) - 1))
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
#      define FDCAN2_DBRP   (((STM32_FDCANCLK_FREQUENCY /                 \
                             ((FDCAN2_DTSEG1 + FDCAN2_DTSEG2 + 3) *     \
                              CONFIG_STM32_FDCAN2_DBITRATE)) - 1))
#      define FDCAN2_DSJW   (CONFIG_STM32_FDCAN2_DSJW - 1)
#    else
#      define FDCAN2_DTSEG1 1
#      define FDCAN2_DTSEG2 1
#      define FDCAN2_DBRP   1
#      define FDCAN2_DSJW   1
#    endif /* CONFIG_STM32_FDCAN2_FD_BRS */

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

#  ifndef CONFIG_STM32_FDCAN3_AUTO_BIT_TIMING

#    define FDCAN3_NTSEG1  (CONFIG_STM32_FDCAN3_NTSEG1 - 1)
#    define FDCAN3_NTSEG2  (CONFIG_STM32_FDCAN3_NTSEG2 - 1)
#    define FDCAN3_NBRP    (((STM32_FDCANCLK_FREQUENCY /              \
                            ((FDCAN3_NTSEG1 + FDCAN3_NTSEG2 + 3) *  \
                             CONFIG_STM32_FDCAN3_BITRATE)) - 1))
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
#      define FDCAN3_DBRP   (((STM32_FDCANCLK_FREQUENCY /                 \
                             ((FDCAN3_DTSEG1 + FDCAN3_DTSEG2 + 3) *     \
                              CONFIG_STM32_FDCAN3_DBITRATE)) - 1))
#      define FDCAN3_DSJW   (CONFIG_STM32_FDCAN3_DSJW - 1)
#    else
#      define FDCAN3_DTSEG1 1
#      define FDCAN3_DTSEG2 1
#      define FDCAN3_DBRP   1
#      define FDCAN3_DSJW   1
#    endif /* CONFIG_STM32_FDCAN3_FD_BRS */

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

/* Interrupts ***************************************************************/

/* Error interrupts
 *
 *   FDCAN_INT_MRAF - Message RAM Access Failure
 *   FDCAN_INT_TOO  - Timeout Occurred
 *   FDCAN_INT_EP   - Error Passive
 *   FDCAN_INT_BO   - Bus_Off Status
 *   FDCAN_INT_WDI  - Watchdog Interrupt
 *   FDCAN_INT_PEA  - Protocol Error in Arbritration Phase
 *   FDCAN_INT_PED  - Protocol Error in Data Phase
 */

#define FDCAN_CMNERR_INTS   (FDCAN_INT_MRAF | FDCAN_INT_TOO | FDCAN_INT_EP | \
                            FDCAN_INT_BO | FDCAN_INT_WDI | FDCAN_INT_PEA | \
                            FDCAN_INT_PED)

#define FDCAN_RXFIFO0_INTS  (FDCAN_INT_RF0N | FDCAN_INT_RF0L)
#define FDCAN_RXFIFO1_INTS  (FDCAN_INT_RF1N | FDCAN_INT_RF1L)

#define FDCAN_RXERR_INTS    (FDCAN_INT_RF0L | FDCAN_INT_RF1L)

#define FDCAN_TXERR_INTS    (FDCAN_INT_TEFL | FDCAN_INT_PEA | FDCAN_INT_PED)

/* Common-, TX- and RX-Error-Mask */

#define FDCAN_ANYERR_INTS (FDCAN_CMNERR_INTS | FDCAN_RXERR_INTS | FDCAN_TXERR_INTS)

/* All interrupt flags of the v1 IR register (bits 0-23) */

#define FDCAN_INT_ALL     0x00ffffff

/* Interrupts used by the SocketCAN mode */

#define FDCAN_SOCK_RX_INTS  (FDCAN_INT_RF0N | FDCAN_INT_RF1N)
#ifdef FDCAN_HAVE_ERRORS
#  define FDCAN_SOCK_ERR_INTS  FDCAN_ANYERR_INTS
#  define FDCAN_SOCK_PROT_INTS (FDCAN_INT_PEA | FDCAN_INT_PED)
#else
#  define FDCAN_SOCK_ERR_INTS  0
#  define FDCAN_SOCK_PROT_INTS 0
#endif

/* Debug ********************************************************************/

/* Debug configurations that may be enabled just for testing FDCAN */

#ifndef CONFIG_DEBUG_CAN_INFO
#  undef CONFIG_STM32_FDCAN_REGDEBUG
#endif

#undef STM32_FDCAN_LOOPBACK
#if defined(CONFIG_STM32_FDCAN1_LOOPBACK) ||   \
    defined(CONFIG_STM32_FDCAN2_LOOPBACK) ||   \
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
  uint32_t data_baud;       /* Configured data baud */
  uint32_t nbtp;            /* Nominal bit timing/prescaler register setting */
  uint32_t dbtp;            /* Data bit timing/prescaler register setting */
  uint8_t port;             /* FDCAN port number (1 to 3) */
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
  struct netdev_lowerhalf_s dev; /* Lower half network device, first */
  FAR netpkt_t *txpkt[3];        /* Packets in the TX FIFO/queue buffers */
#ifdef FDCAN_HAVE_ERRORS
  uint32_t errstatus;            /* Error interrupts for receive() */
#endif

  /* The constant configuration */

  const struct stm32_config_s *config;

  uint8_t state;            /* See enum can_state_s */
  uint32_t nbtp;            /* Current nominal bit timing */
  uint32_t dbtp;            /* Current data bit timing */

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
  uintptr_t regaddr;        /* Last register address read */
  uint32_t regval;          /* Last value read from the register */
  unsigned int count;       /* Number of times that the value was read */
#endif
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
#ifdef CONFIG_STM32_FDCAN_REGDEBUG
static void fdcan_dumpregs(struct stm32_fdcan_s *priv,
                           const char *msg);
static void fdcan_dumprxregs(struct stm32_fdcan_s *priv,
                             const char *msg);
static void fdcan_dumptxregs(struct stm32_fdcan_s *priv,
                             const char *msg);
static void fdcan_dumpramlayout(struct stm32_fdcan_s *priv);
#else
#  define fdcan_dumpregs(priv,msg)
#  define fdcan_dumprxregs(priv,msg)
#  define fdcan_dumptxregs(priv,msg)
#  define fdcan_dumpramlayout(priv)
#endif

/* SocketCAN methods */

#ifdef FDCAN_HAVE_ERRORS
static FAR netpkt_t *fdcan_sock_errframe(FAR struct stm32_fdcan_s *priv,
                                         uint32_t status);
#endif
static FAR netpkt_t *fdcan_sock_receive(FAR struct netdev_lowerhalf_s *dev);
static int  fdcan_sock_transmit(FAR struct netdev_lowerhalf_s *dev,
                                FAR netpkt_t *pkt);
static void fdcan_sock_reclaim(FAR struct netdev_lowerhalf_s *dev);
#ifdef CONFIG_NET_NETPOLL
static void fdcan_sock_txpoll(FAR struct netdev_lowerhalf_s *dev);
#endif
static int  fdcan_sock_interrupt(int irq, FAR void *context,
                                 FAR void *arg);
static int  fdcan_sock_ifup(FAR struct netdev_lowerhalf_s *dev);
static int  fdcan_sock_ifdown(FAR struct netdev_lowerhalf_s *dev);

/* Hardware initialization */

static int  fdcan_hw_initialize(struct stm32_fdcan_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* All instances share one RCC reset */

static bool g_fdcan_rccreset;

static const struct netdev_ops_s g_fdcan_sock_ops =
{
  .ifup     = fdcan_sock_ifup,
  .ifdown   = fdcan_sock_ifdown,
  .transmit = fdcan_sock_transmit,
  .receive  = fdcan_sock_receive,
#ifdef CONFIG_NET_NETPOLL
  .txpoll   = fdcan_sock_txpoll,
#endif
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
#if defined(CONFIG_STM32_FDCAN1_FD_BRS)
  .data_baud        = CONFIG_STM32_FDCAN1_DBITRATE,
#endif
#ifndef CONFIG_STM32_FDCAN1_AUTO_BIT_TIMING
  .nbtp             = FDCAN_NBTP_NBRP(FDCAN1_NBRP) |
                      FDCAN_NBTP_NTSEG1(FDCAN1_NTSEG1) |
                      FDCAN_NBTP_NTSEG2(FDCAN1_NTSEG2) |
                      FDCAN_NBTP_NSJW(FDCAN1_NSJW),
  .dbtp             = FDCAN_DBTP_DBRP(FDCAN1_DBRP) |
                      FDCAN_DBTP_DTSEG1(FDCAN1_DTSEG1) |
                      FDCAN_DBTP_DTSEG2(FDCAN1_DTSEG2) |
                      FDCAN_DBTP_DSJW(FDCAN1_DSJW),
#endif
  .port             = 1,
  .irq0             = FDCAN1_IRQ0,
  .irq1             = FDCAN1_IRQ1,
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
#if defined(CONFIG_STM32_FDCAN2_FD_BRS)
  .data_baud        = CONFIG_STM32_FDCAN2_DBITRATE,
#endif
#ifndef CONFIG_STM32_FDCAN2_AUTO_BIT_TIMING
  .nbtp             = FDCAN_NBTP_NBRP(FDCAN2_NBRP) |
                      FDCAN_NBTP_NTSEG1(FDCAN2_NTSEG1) |
                      FDCAN_NBTP_NTSEG2(FDCAN2_NTSEG2) |
                      FDCAN_NBTP_NSJW(FDCAN2_NSJW),
  .dbtp             = FDCAN_DBTP_DBRP(FDCAN2_DBRP) |
                      FDCAN_DBTP_DTSEG1(FDCAN2_DTSEG1) |
                      FDCAN_DBTP_DTSEG2(FDCAN2_DTSEG2) |
                      FDCAN_DBTP_DSJW(FDCAN2_DSJW),
#endif
  .port             = 2,
  .irq0             = FDCAN2_IRQ0,
  .irq1             = FDCAN2_IRQ1,
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
#if defined(CONFIG_STM32_FDCAN3_FD_BRS)
  .data_baud        = CONFIG_STM32_FDCAN3_DBITRATE,
#endif
#ifndef CONFIG_STM32_FDCAN3_AUTO_BIT_TIMING
  .nbtp             = FDCAN_NBTP_NBRP(FDCAN3_NBRP) |
                      FDCAN_NBTP_NTSEG1(FDCAN3_NTSEG1) |
                      FDCAN_NBTP_NTSEG2(FDCAN3_NTSEG2) |
                      FDCAN_NBTP_NSJW(FDCAN3_NSJW),
  .dbtp             = FDCAN_DBTP_DBRP(FDCAN3_DBRP) |
                      FDCAN_DBTP_DTSEG1(FDCAN3_DTSEG1) |
                      FDCAN_DBTP_DTSEG2(FDCAN3_DTSEG2) |
                      FDCAN_DBTP_DSJW(FDCAN3_DSJW),
#endif
  .port             = 3,
  .irq0             = FDCAN3_IRQ0,
  .irq1             = FDCAN3_IRQ1,
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
static uint32_t fdcan_getreg(struct stm32_fdcan_s *priv, int offset)
{
  const struct stm32_config_s *config  = priv->config;
  uintptr_t                    regaddr = 0;
  uint32_t                     regval  = 0;

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
static uint32_t fdcan_getreg(struct stm32_fdcan_s *priv, int offset)
{
  const struct stm32_config_s *config = priv->config;

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
static void fdcan_putreg(struct stm32_fdcan_s *priv, int offset,
                         uint32_t regval)
{
  const struct stm32_config_s *config = priv->config;
  uintptr_t regaddr = config->base + offset;

  /* Show the register value being written */

  caninfo("%08" PRIx32 "->%08" PRIx32 "\n", regaddr, regval);

  /* Write the value */

  putreg32(regval, regaddr);
}

#else
static void fdcan_putreg(struct stm32_fdcan_s *priv, int offset,
                         uint32_t regval)
{
  const struct stm32_config_s *config = priv->config;

  putreg32(regval, config->base + offset);
}

#endif

/****************************************************************************
 * Name: fdcan_dumpregs
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
static void fdcan_dumpregs(struct stm32_fdcan_s *priv,
                           const char *msg)
{
  const struct stm32_config_s *config = priv->config;

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
static void fdcan_dumprxregs(struct stm32_fdcan_s *priv,
                             const char *msg)
{
  const struct stm32_config_s *config = priv->config;

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
static void fdcan_dumptxregs(struct stm32_fdcan_s *priv,
                             const char *msg)
{
  const struct stm32_config_s *config = priv->config;

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
static void fdcan_dumpramlayout(struct stm32_fdcan_s *priv)
{
  const struct stm32_config_s *config = priv->config;

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
  volatile uint32_t           *msgram = NULL;
  uint32_t                     regval = 0;
  uint32_t                     cntr   = 0;

  caninfo("FDCAN%d\n", config->port);

  /* Clean message RAM */

  msgram = config->msgram.stdfilters;
  cntr   = FDCAN_MSGRAM_WORDS;
  while (cntr > 0)
    {
      *msgram++ = 0;
      cntr--;
    }

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

  /* Extended ID Filter AND mask  */

  fdcan_putreg(priv, STM32_FDCAN_XIDAM_OFFSET, 0x1fffffff);

  /* Disable all interrupts  */

  fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, 0);
  fdcan_putreg(priv, STM32_FDCAN_TXBTIE_OFFSET, 0);

  /* All interrupts directed to line 0, both lines disabled for now */

  fdcan_putreg(priv, STM32_FDCAN_ILS_OFFSET, 0);
  fdcan_putreg(priv, STM32_FDCAN_ILE_OFFSET, 0);

  /* Clear all pending interrupts. */

  fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, FDCAN_INT_ALL);

  /* Configure FDCAN bit timing */

  fdcan_putreg(priv, STM32_FDCAN_NBTP_OFFSET, priv->nbtp);
  fdcan_putreg(priv, STM32_FDCAN_DBTP_OFFSET, priv->dbtp);

  /* Filter list sizes; non matching frames go to RX FIFO0 */

  regval  = FDCAN_RXGFC_LSS(config->nstdfilters);
  regval |= FDCAN_RXGFC_LSE(config->nextfilters);
  regval |= FDCAN_RXGFC_ANFE_RX_FIFO0 | FDCAN_RXGFC_ANFS_RX_FIFO0;
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
  regval &= ~(FDCAN_CCCR_NISO | FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE |
              FDCAN_CCCR_DAR);

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

#ifdef FDCAN_HAVE_FD
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

  /* Enable only interrupt line 0 (ILS is cleared above) */

  fdcan_putreg(priv, STM32_FDCAN_ILE_OFFSET, FDCAN_ILE_EINT0);

  /* Disable initialization mode to enable normal operation */

  regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
  regval &= ~FDCAN_CCCR_INIT;
  fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

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

static int32_t fdcan_bittiming(struct fdcan_bitseg *timing,
                               uint32_t maxpresc)
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

  const uint32_t prescaler_bs = STM32_FDCANCLK_FREQUENCY / target_bitrate;

  /* Find prescaler value such that the number of quanta per bit is highest */

  uint8_t bs1_bs2_sum = max_quanta_per_bit - 1;

  while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0)
    {
      if (bs1_bs2_sum <= 2)
        {
          nerr("Target bitrate too high - no solution possible.");
          return 1; /* No solution */
        }

      bs1_bs2_sum--;
    }

  const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);

  if ((prescaler < 1U) || (prescaler > maxpresc))
    {
      nerr("Target bitrate invalid - bad prescaler.");
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

  /* Final validation
   * Helpful Python:
   * def sample_point_from_btr(x):
   *     assert 0b0011110010000000111111000000000 & x == 0
   *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
   *     return (1+ts1+1)/(1+ts1+1+ts2+1)
   */

  if (target_bitrate != (STM32_FDCANCLK_FREQUENCY /
                        (prescaler * (1 + bs1 + bs2))) || !valid)
    {
      nerr("Target bitrate invalid - solution does not match.");
      return 3; /* Solution not found */
    }

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
  ninfo("[fdcan] CLK_FREQ %" PRIu32 ", target_bitrate %" PRIu32
        ", prescaler %" PRIu32 ", bs1 %d, bs2 %d\n",
        (uint32_t)STM32_FDCANCLK_FREQUENCY, target_bitrate, prescaler_bs,
        bs1 - 1,
        bs2 - 1);
#endif

  timing->bs1 = (uint8_t)(bs1 - 1);
  timing->bs2 = (uint8_t)(bs2 - 1);
  timing->prescaler = (uint16_t)(prescaler - 1);
  timing->sjw = 0; /* Which means one */

  return 0;
}

/****************************************************************************
 * Name: fdcan_sock_lec
 *
 * Description:
 *   Decode a last error code into the error frame bits.
 *
 ****************************************************************************/

#ifdef FDCAN_HAVE_ERRORS
static void fdcan_sock_lec(uint32_t lec, FAR uint16_t *errbits,
                           FAR uint8_t *data)
{
  switch (lec)
    {
      case FDCAN_PSR_EC_STUFF_ERROR:
        *errbits |= CAN_ERR_PROT;
        data[2]  |= CAN_ERR_PROT_STUFF;
        break;

      case FDCAN_PSR_EC_FORM_ERROR:
        *errbits |= CAN_ERR_PROT;
        data[2]  |= CAN_ERR_PROT_FORM;
        break;

      case FDCAN_PSR_EC_ACK_ERROR:
        *errbits |= CAN_ERR_ACK;
        break;

      case FDCAN_PSR_EC_BIT1_ERROR:
        *errbits |= CAN_ERR_PROT;
        data[2]  |= CAN_ERR_PROT_BIT1;
        break;

      case FDCAN_PSR_EC_BIT0_ERROR:
        *errbits |= CAN_ERR_PROT;
        data[2]  |= CAN_ERR_PROT_BIT0;
        break;

      case FDCAN_PSR_EC_CRC_ERROR:
        *errbits |= CAN_ERR_PROT;
        data[3]  |= (CAN_ERR_PROT_LOC_CRC_SEQ | CAN_ERR_PROT_LOC_CRC_DEL);
        break;

      default:
        break;
    }
}

static FAR netpkt_t *fdcan_sock_errframe(FAR struct stm32_fdcan_s *priv,
                                         uint32_t status)
{
  FAR struct netdev_lowerhalf_s *dev = &priv->dev;
  FAR struct can_frame          *frame;
  FAR netpkt_t                  *pkt;
  uint32_t                       psr;
  uint16_t                       errbits = 0;
  uint8_t                        data[CAN_ERR_DLC];

  memset(data, 0, sizeof(data));
  psr = fdcan_getreg(priv, STM32_FDCAN_PSR_OFFSET);

  if ((psr & FDCAN_PSR_EP) != 0)
    {
      data[1] |= (CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE);
    }

  if ((psr & FDCAN_PSR_EW) != 0)
    {
      data[1] |= (CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING);
    }

  if ((status & (FDCAN_INT_EP | FDCAN_INT_EW)) != 0)
    {
      errbits |= CAN_ERR_CRTL;
    }

  if ((status & FDCAN_INT_PEA) != 0)
    {
      fdcan_sock_lec((psr & FDCAN_PSR_LEC_MASK) >> FDCAN_PSR_LEC_SHIFT,
                     &errbits, data);
    }

  if ((status & FDCAN_INT_PED) != 0)
    {
      fdcan_sock_lec((psr & FDCAN_PSR_DLEC_MASK) >> FDCAN_PSR_DLEC_SHIFT,
                     &errbits, data);
    }

  if ((status & FDCAN_INT_BO) != 0)
    {
      errbits |= (psr & FDCAN_PSR_BO) != 0 ? CAN_ERR_BUSOFF :
                                             CAN_ERR_RESTARTED;
    }

  if ((status & (FDCAN_INT_RF0L | FDCAN_INT_RF1L)) != 0)
    {
      errbits |= CAN_ERR_CRTL;
      data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
    }

  if ((status & FDCAN_INT_TEFL) != 0)
    {
      errbits |= CAN_ERR_CRTL;
      data[1] |= CAN_ERR_CRTL_TX_OVERFLOW;
    }

  if ((status & FDCAN_INT_TOO) != 0)
    {
      errbits |= CAN_ERR_TX_TIMEOUT;
    }

  if ((status & (FDCAN_INT_MRAF | FDCAN_INT_ELO)) != 0)
    {
      errbits |= CAN_ERR_CRTL;
      data[1] |= CAN_ERR_CRTL_UNSPEC;
    }

  if (errbits == 0)
    {
      return NULL;
    }

  pkt = netpkt_alloc(dev, NETPKT_RX);
  if (pkt == NULL)
    {
      return NULL;
    }

  netpkt_setdatalen(dev, pkt, sizeof(struct can_frame));
  frame = (FAR struct can_frame *)netpkt_getdata(dev, pkt);
  memset(frame, 0, sizeof(*frame));
  frame->can_id  = CAN_ERR_FLAG | errbits;
  frame->can_dlc = CAN_ERR_DLC;
  memcpy(frame->data, data, CAN_ERR_DLC);

  NETDEV_ERRORS(&dev->netdev);
  return pkt;
}
#endif /* FDCAN_HAVE_ERRORS */

/****************************************************************************
 * Name: fdcan_sock_rxframe
 *
 * Description:
 *   Copy one RX FIFO element into a new packet.
 *
 ****************************************************************************/

static FAR netpkt_t *fdcan_sock_rxframe(FAR struct stm32_fdcan_s *priv,
                                        volatile uint32_t *rxbuffer)
{
  FAR struct netdev_lowerhalf_s *dev = &priv->dev;
  FAR netpkt_t                  *pkt;
  canid_t                        id;
  uint8_t                        dlc;

#ifndef FDCAN_HAVE_EXTID
  if ((rxbuffer[0] & BUFFER_R0_XTD) != 0)
    {
      /* Extended IDs are not supported, drop the message */

      return NULL;
    }
#endif

  pkt = netpkt_alloc(dev, NETPKT_RX);
  if (pkt == NULL)
    {
      return NULL;
    }

  if ((rxbuffer[0] & BUFFER_R0_XTD) != 0)
    {
      id = ((rxbuffer[0] & BUFFER_R0_EXTID_MASK) >> BUFFER_R0_EXTID_SHIFT) |
           CAN_EFF_FLAG;
    }
  else
    {
      id = (rxbuffer[0] & BUFFER_R0_STDID_MASK) >> BUFFER_R0_STDID_SHIFT;
    }

  if ((rxbuffer[0] & BUFFER_R0_RTR) != 0)
    {
      id |= CAN_RTR_FLAG;
    }

  dlc = (rxbuffer[1] & BUFFER_R1_DLC_MASK) >> BUFFER_R1_DLC_SHIFT;

#ifdef FDCAN_HAVE_FD
  if ((rxbuffer[1] & BUFFER_R1_FDF) != 0)
    {
      FAR struct canfd_frame *frame;

      netpkt_setdatalen(dev, pkt, sizeof(struct canfd_frame));
      frame = (FAR struct canfd_frame *)netpkt_getdata(dev, pkt);
      memset(frame, 0, sizeof(*frame));
      frame->can_id = id;
      frame->len    = can_dlc2bytes(dlc);

      if ((rxbuffer[0] & BUFFER_R0_ESI) != 0)
        {
          frame->flags |= CANFD_ESI;
        }

      if ((rxbuffer[1] & BUFFER_R1_BRS) != 0)
        {
          frame->flags |= CANFD_BRS;
        }

      memcpy(frame->data, (FAR void *)&rxbuffer[2], frame->len);
    }
  else
#endif
    {
      FAR struct can_frame *frame;

      netpkt_setdatalen(dev, pkt, sizeof(struct can_frame));
      frame = (FAR struct can_frame *)netpkt_getdata(dev, pkt);
      memset(frame, 0, sizeof(*frame));
      frame->can_id  = id;
      frame->can_dlc = dlc > CAN_MAX_DLC ? CAN_MAX_DLC : dlc;
      memcpy(frame->data, (FAR void *)&rxbuffer[2], frame->can_dlc);
    }

  return pkt;
}

/****************************************************************************
 * Name: fdcan_sock_receive
 *
 * Description:
 *   Return the next received frame: latched errors first, then RX FIFO1,
 *   then RX FIFO0.  RX interrupts are enabled again when both FIFOs are
 *   empty.
 *
 ****************************************************************************/

static FAR netpkt_t *fdcan_sock_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct stm32_fdcan_s    *priv   = (FAR struct stm32_fdcan_s *)dev;
  const struct stm32_config_s *config = priv->config;
  FAR netpkt_t                *pkt    = NULL;
  irqstate_t                   flags;
  uint32_t                     regval;
  unsigned int                 ndx;
  int                          fifo;

#ifdef FDCAN_HAVE_ERRORS
  flags = enter_critical_section();
  regval = priv->errstatus;
  priv->errstatus = 0;
  leave_critical_section(flags);

  if (regval != 0)
    {
      pkt = fdcan_sock_errframe(priv, regval);

      /* Re-enable the error interrupts, except the protocol errors:
       * they stay off until the next transfer, so a failing bus does not
       * flood the receivers with error frames.
       */

      flags = enter_critical_section();
      fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET,
                   fdcan_getreg(priv, STM32_FDCAN_IE_OFFSET) |
                   (FDCAN_SOCK_ERR_INTS & ~FDCAN_SOCK_PROT_INTS));
      leave_critical_section(flags);

      if (pkt != NULL)
        {
          return pkt;
        }
    }
#endif

  for (fifo = 1; fifo >= 0; fifo--)
    {
      uint32_t statoff = fifo ? STM32_FDCAN_RXF1S_OFFSET :
                                STM32_FDCAN_RXF0S_OFFSET;
      uint32_t ackoff  = fifo ? STM32_FDCAN_RXF1A_OFFSET :
                                STM32_FDCAN_RXF0A_OFFSET;

      for (; ; )
        {
          regval = fdcan_getreg(priv, statoff);
          if ((regval & FDCAN_RXFS_FFL_MASK) == 0)
            {
              break;
            }

          ndx = (regval & FDCAN_RXFS_FGI_MASK) >> FDCAN_RXFS_FGI_SHIFT;
          pkt = fdcan_sock_rxframe(priv,
                                   (fifo ? config->msgram.rxfifo1 :
                                           config->msgram.rxfifo0) +
                                   ndx * (fifo ? config->rxfifo1esize :
                                                 config->rxfifo0esize));

          /* Acknowledge the element.  Without a free packet the message
           * is dropped, so the FIFO keeps receiving.
           */

          fdcan_putreg(priv, ackoff, ndx);
          if (pkt != NULL)
            {
              return pkt;
            }

          NETDEV_RXDROPPED(&dev->netdev);
        }
    }

  /* Both FIFOs are empty, enable the RX interrupts again */

  flags = enter_critical_section();
  fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET,
               fdcan_getreg(priv, STM32_FDCAN_IE_OFFSET) |
               FDCAN_SOCK_RX_INTS);
  leave_critical_section(flags);
  return NULL;
}

/****************************************************************************
 * Name: fdcan_sock_transmit
 *
 * Description:
 *   Copy a frame into the next TX FIFO/queue buffer and request it.
 *
 ****************************************************************************/

static int fdcan_sock_transmit(FAR struct netdev_lowerhalf_s *dev,
                               FAR netpkt_t *pkt)
{
  FAR struct stm32_fdcan_s    *priv   = (FAR struct stm32_fdcan_s *)dev;
  const struct stm32_config_s *config = priv->config;
  volatile uint32_t           *txbuffer;
  FAR const uint8_t           *src;
  canid_t                      id;
  uint32_t                     regval;
  irqstate_t                   flags;
  unsigned int                 ndx;
  unsigned int                 nbytes;
  unsigned int                 i;

  /* The socket checks the frame size only, check the data length */

#ifdef FDCAN_HAVE_FD
  if (netpkt_getdatalen(dev, pkt) == sizeof(struct canfd_frame))
    {
      if (((FAR struct canfd_frame *)netpkt_getdata(dev, pkt))->len >
          CANFD_MAX_DLEN)
        {
          return -EINVAL;
        }
    }
  else
#endif
  if (((FAR struct can_frame *)netpkt_getdata(dev, pkt))->can_dlc >
      CAN_MAX_DLEN)
    {
      return -EINVAL;
    }

  /* The TX interrupt also frees the finished packets */

  flags = enter_critical_section();
  fdcan_sock_reclaim(dev);

  regval = fdcan_getreg(priv, STM32_FDCAN_TXFQS_OFFSET);
  if ((regval & FDCAN_TXFQS_TFQF) != 0)
    {
      leave_critical_section(flags);
      return -EBUSY;
    }

  ndx      = (regval & FDCAN_TXFQS_TFQPI_MASK) >> FDCAN_TXFQS_TFQPI_SHIFT;
  txbuffer = config->msgram.txfifoq + ndx * config->txbufferesize;

  txbuffer[0] = 0;
  txbuffer[1] = 0;

#ifdef FDCAN_HAVE_FD
  if (netpkt_getdatalen(dev, pkt) == sizeof(struct canfd_frame))
    {
      FAR struct canfd_frame *frame =
        (FAR struct canfd_frame *)netpkt_getdata(dev, pkt);

      id          = frame->can_id;
      nbytes      = frame->len;
      src         = frame->data;
      txbuffer[1] = BUFFER_R1_FDF | BUFFER_R1_DLC(can_bytes2dlc(nbytes));

      if ((frame->flags & CANFD_BRS) != 0)
        {
          txbuffer[1] |= BUFFER_R1_BRS;
        }

      if ((frame->flags & CANFD_ESI) != 0)
        {
          txbuffer[0] |= BUFFER_R0_ESI;
        }
    }
  else
#endif
    {
      FAR struct can_frame *frame =
        (FAR struct can_frame *)netpkt_getdata(dev, pkt);

      id          = frame->can_id;
      nbytes      = frame->can_dlc;
      src         = frame->data;
      txbuffer[1] = BUFFER_R1_DLC(nbytes);
    }

#ifdef FDCAN_HAVE_EXTID
  if ((id & CAN_EFF_FLAG) != 0)
    {
      txbuffer[0] |= BUFFER_R0_EXTID(id & CAN_EFF_MASK) | BUFFER_R0_XTD;
    }
  else
#endif
    {
      txbuffer[0] |= BUFFER_R0_STDID(id & CAN_SFF_MASK);
    }

  if ((id & CAN_RTR_FLAG) != 0)
    {
      txbuffer[0] |= BUFFER_R0_RTR;
    }

  /* The message RAM takes word writes */

  for (i = 0; i < nbytes; i += 4)
    {
      txbuffer[2 + i / 4] = src[i] | (src[i + 1] << 8) |
                            (src[i + 2] << 16) | (src[i + 3] << 24);
    }

  priv->txpkt[ndx] = pkt;

  fdcan_putreg(priv, STM32_FDCAN_TXBAR_OFFSET, 1 << ndx);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: fdcan_sock_reclaim
 *
 * Description:
 *   Free the packets of the finished TX buffers.  Called with interrupts
 *   disabled: from transmit, the TX interrupt and fdcan_sock_txpoll().
 *
 ****************************************************************************/

static void fdcan_sock_reclaim(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct stm32_fdcan_s *priv = (FAR struct stm32_fdcan_s *)dev;
  uint32_t                  pending;
  uint32_t                  done;
  unsigned int              i;

  pending = fdcan_getreg(priv, STM32_FDCAN_TXBRP_OFFSET);
  done    = fdcan_getreg(priv, STM32_FDCAN_TXBTO_OFFSET);

  for (i = 0; i < priv->config->ntxfifoq; i++)
    {
      if (priv->txpkt[i] != NULL && (pending & (1 << i)) == 0)
        {
          if ((done & (1 << i)) == 0)
            {
              NETDEV_TXERRORS(&dev->netdev);
            }

          netpkt_free(dev, priv->txpkt[i], NETPKT_TX);
          priv->txpkt[i] = NULL;
        }
    }
}

/****************************************************************************
 * Name: fdcan_sock_txpoll
 *
 * Description:
 *   NuttX callback: free the packets of the finished TX buffers for
 *   netpoll, which may run with or without interrupts disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_NETPOLL
static void fdcan_sock_txpoll(FAR struct netdev_lowerhalf_s *dev)
{
  irqstate_t flags = enter_critical_section();

  fdcan_sock_reclaim(dev);
  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: fdcan_sock_interrupt
 *
 * Description:
 *   FDCAN interrupt handler for the SocketCAN mode: frees the finished TX
 *   packets, the reception is done in the upper half receive call.
 *
 ****************************************************************************/

static int fdcan_sock_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct stm32_fdcan_s *priv = (FAR struct stm32_fdcan_s *)arg;
  uint32_t                  ie;
  uint32_t                  pending;

  ie      = fdcan_getreg(priv, STM32_FDCAN_IE_OFFSET);
  pending = fdcan_getreg(priv, STM32_FDCAN_IR_OFFSET) & ie;
  fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, pending);

  if ((pending & FDCAN_SOCK_RX_INTS) != 0)
    {
      /* receive() enables them again when the FIFOs are empty */

      ie &= ~FDCAN_SOCK_RX_INTS;
    }

#ifdef FDCAN_HAVE_ERRORS
  if ((pending & FDCAN_SOCK_ERR_INTS) != 0)
    {
      /* Latch the errors for receive(), which enables them again */

      priv->errstatus |= pending & FDCAN_SOCK_ERR_INTS;
      ie &= ~FDCAN_SOCK_ERR_INTS;
    }
#endif

  /* A transfer completed: report protocol errors again */

  if ((pending & (FDCAN_SOCK_RX_INTS | FDCAN_INT_TC)) != 0)
    {
      ie |= FDCAN_SOCK_PROT_INTS & ~(pending & FDCAN_SOCK_ERR_INTS);
    }

  fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, ie);

  if ((pending & (FDCAN_SOCK_RX_INTS | FDCAN_SOCK_ERR_INTS)) != 0)
    {
      netdev_lower_rxready(&priv->dev);
    }

  if ((pending & FDCAN_INT_TC) != 0)
    {
      fdcan_sock_reclaim(&priv->dev);
      netdev_lower_txdone(&priv->dev);
    }

  return OK;
}

/****************************************************************************
 * Name: fdcan_sock_ifup
 ****************************************************************************/

static int fdcan_sock_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct stm32_fdcan_s    *priv   = (FAR struct stm32_fdcan_s *)dev;
  const struct stm32_config_s *config = priv->config;
  int                          ret;

  ret = fdcan_hw_initialize(priv);
  if (ret < 0)
    {
      nerr("ERROR: FDCAN%d H/W initialization failed: %d\n",
           config->port, ret);
      return ret;
    }

  ret = irq_attach(config->irq0, fdcan_sock_interrupt, priv);
  if (ret >= 0)
    {
      ret = irq_attach(config->irq1, fdcan_sock_interrupt, priv);
    }

  if (ret < 0)
    {
      return ret;
    }

  /* All TX buffers raise TC, RX and errors on interrupt line 0 */

  fdcan_putreg(priv, STM32_FDCAN_TXBTIE_OFFSET,
               (1 << config->ntxfifoq) - 1);
  fdcan_putreg(priv, STM32_FDCAN_IR_OFFSET, FDCAN_INT_ALL);
  fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET,
               FDCAN_SOCK_RX_INTS | FDCAN_INT_TC | FDCAN_SOCK_ERR_INTS);

  priv->state = FDCAN_STATE_SETUP;
  up_enable_irq(config->irq0);
  up_enable_irq(config->irq1);

  fdcan_dumpregs(priv, "ifup");
  fdcan_dumprxregs(priv, "ifup");
  fdcan_dumptxregs(priv, "ifup");

  netdev_lower_carrier_on(dev);
  return OK;
}

/****************************************************************************
 * Name: fdcan_sock_ifdown
 ****************************************************************************/

static int fdcan_sock_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct stm32_fdcan_s    *priv   = (FAR struct stm32_fdcan_s *)dev;
  const struct stm32_config_s *config = priv->config;
  uint32_t                     regval;
  unsigned int                 i;

  netdev_lower_carrier_off(dev);

  up_disable_irq(config->irq0);
  up_disable_irq(config->irq1);
  fdcan_putreg(priv, STM32_FDCAN_IE_OFFSET, 0);
  fdcan_putreg(priv, STM32_FDCAN_TXBTIE_OFFSET, 0);
  irq_detach(config->irq0);
  irq_detach(config->irq1);

  if (priv->state == FDCAN_STATE_SETUP)
    {
      /* Stop the controller: request clock stop and wait for it */

      regval  = fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET);
      regval |= FDCAN_CCCR_CSR;
      fdcan_putreg(priv, STM32_FDCAN_CCCR_OFFSET, regval);

      while ((fdcan_getreg(priv, STM32_FDCAN_CCCR_OFFSET) &
              FDCAN_CCCR_CSA) == 0);

      priv->state = FDCAN_STATE_DISABLED;
    }

  for (i = 0; i < config->ntxfifoq; i++)
    {
      if (priv->txpkt[i] != NULL)
        {
          netpkt_free(dev, priv->txpkt[i], NETPKT_TX);
          priv->txpkt[i] = NULL;
        }
    }

#ifdef FDCAN_HAVE_ERRORS
  priv->errstatus = 0;
#endif
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_fdcansockinitialize
 *
 * Description:
 *   Initialize the selected FDCAN port as a SocketCAN interface
 *
 * Input Parameters:
 *   port - Port number, 1=FDCAN1, 2=FDCAN2, 3=FDCAN3
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32_fdcansockinitialize(int port)
{
  struct stm32_fdcan_s        *priv   = NULL;
  const struct stm32_config_s *config = NULL;
  bool                         auto_bit_timing = true;
  struct fdcan_bitseg          timing;
  int                          ret;

  ninfo("FDCAN%d\n", port);

#ifdef CONFIG_STM32_FDCAN1
  if (port == FDCAN1)
    {
      priv   = &g_fdcan1priv;
      config = &g_fdcan1const;
#ifndef CONFIG_STM32_FDCAN1_AUTO_BIT_TIMING
      auto_bit_timing = false;
#endif
    }
  else
#endif
#ifdef CONFIG_STM32_FDCAN2
  if (port == FDCAN2)
    {
      priv   = &g_fdcan2priv;
      config = &g_fdcan2const;
#ifndef CONFIG_STM32_FDCAN2_AUTO_BIT_TIMING
      auto_bit_timing = false;
#endif
    }
  else
#endif
#ifdef CONFIG_STM32_FDCAN3
  if (port == FDCAN3)
    {
      priv   = &g_fdcan3priv;
      config = &g_fdcan3const;
#ifndef CONFIG_STM32_FDCAN3_AUTO_BIT_TIMING
      auto_bit_timing = false;
#endif
    }
  else
#endif
    {
      nerr("ERROR: Unsupported port %d\n", port);
      return -ENODEV;
    }

  memset(priv, 0, sizeof(struct stm32_fdcan_s));
  priv->config = config;

  if (auto_bit_timing == false)
    {
      priv->nbtp = config->nbtp;
      priv->dbtp = config->dbtp;
    }
  else
    {
      timing.bitrate = config->baud;
      if (fdcan_bittiming(&timing, FDCAN_NBTP_NBRP_MAX + 1) != OK)
        {
          nerr("ERROR: Invalid CAN nominal phase timings\n");
          return -EINVAL;
        }

      priv->nbtp = FDCAN_NBTP_NBRP(timing.prescaler) |
                   FDCAN_NBTP_NTSEG1(timing.bs1) |
                   FDCAN_NBTP_NTSEG2(timing.bs2) |
                   FDCAN_NBTP_NSJW(timing.sjw);

      if (config->mode == FDCAN_FD_BRS_MODE)
        {
          timing.bitrate = config->data_baud;
          if (fdcan_bittiming(&timing, FDCAN_DBTP_DBRP_MAX + 1) != OK)
            {
              nerr("ERROR: Invalid CAN data phase timings\n");
              return -EINVAL;
            }

          priv->dbtp = FDCAN_DBTP_DBRP(timing.prescaler) |
                       FDCAN_DBTP_DTSEG1(timing.bs1) |
                       FDCAN_DBTP_DTSEG2(timing.bs2) |
                       FDCAN_DBTP_DSJW(timing.sjw);
        }
    }

  /* The RCC reset is common to all instances: do it only once */

  if (!g_fdcan_rccreset)
    {
      irqstate_t flags = enter_critical_section();
      uint32_t   regval;

      regval = getreg32(FDCAN_RSTR);
      putreg32(regval | FDCAN_RST, FDCAN_RSTR);
      putreg32(regval & ~FDCAN_RST, FDCAN_RSTR);
      g_fdcan_rccreset = true;
      leave_critical_section(flags);
    }

  priv->state = FDCAN_STATE_RESET;

  priv->dev.ops = &g_fdcan_sock_ops;

  ret = netdev_lower_register(&priv->dev, NET_LL_CAN);
  if (ret < 0)
    {
      nerr("ERROR: netdev_lower_register failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Initialize the enabled FDCAN ports when the board does not do it.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETDEV_LATEINIT)
#  ifdef CONFIG_STM32_ETHMAC
#    error "FDCAN SocketCAN and Ethernet need CONFIG_NETDEV_LATEINIT"
#  endif

void arm_netinitialize(void)
{
#ifdef CONFIG_STM32_FDCAN1
  stm32_fdcansockinitialize(FDCAN1);
#endif

#ifdef CONFIG_STM32_FDCAN2
  stm32_fdcansockinitialize(FDCAN2);
#endif

#ifdef CONFIG_STM32_FDCAN3
  stm32_fdcansockinitialize(FDCAN3);
#endif
}
#endif

/****************************************************************************
 * arch/arm/src/stm32h5/stm32_fdcan_socket.c
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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/can.h>
#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/can.h>

#if defined(CONFIG_NET_CAN_RAW_TX_DEADLINE) || defined(CONFIG_NET_TIMESTAMP)
#  include <sys/time.h>
#endif

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32_fdcan.h"
#include "hardware/stm32_pinmap.h"
#include "chip.h"
#include "stm32.h"

/* Workaround to enable CANFD globally.
 * The legacy SocketCan driver needs CANFD enabled but this driver will not
 * compile with CANFD enabled, disabling it locally only for this driver.
 */
#ifdef CONFIG_NET_CAN_CANFD
#  undef CONFIG_NET_CAN_CANFD
#endif

#ifdef CONFIG_STM32_FDCAN1
#  ifndef STM32H5_FDCAN1_PWR
#    error "STM32H5_FDCAN1_PWR for FDCAN1 must be defined in board.h!"
#  endif
#  ifndef STM32H5_FDCAN1_PWR_STANDBY
#    error "STM32H5_FDCAN1_PWR_STANDBY for FDCAN1 must be defined in board.h!"
#  endif
#endif

#ifdef CONFIG_STM32_FDCAN2
#  ifndef STM32H5_FDCAN2_PWR
#    error "STM32H5_FDCAN2_PWR for FDCAN2 must be defined in board.h!"
#  endif
#  ifndef STM32H5_FDCAN2_PWR_STANDBY
#    error "STM32H5_FDCAN2_PWR_STANDBY for FDCAN2 must be defined in board.h!"
#  endif
#endif

#ifdef CONFIG_STM32_FDCAN3
#  ifndef STM32H5_FDCAN3_PWR
#    error "STM32H5_FDCAN3_PWR for FDCAN3 must be defined in board.h!"
#  endif
#  ifndef STM32H5_FDCAN3_PWR_STANDBY
#    error "STM32H5_FDCAN3_PWR_STANDBY for FDCAN2 must be defined in board.h!"
#  endif
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FDCAN_CMNERR_INTS   (FDCAN_INT_MRAF | FDCAN_INT_TOO | FDCAN_INT_EP | \
                            FDCAN_INT_BO | FDCAN_INT_WDI | FDCAN_INT_PEA | \
                            FDCAN_INT_PED)

#define FDCAN_RXERR_INTS    (FDCAN_INT_RF0L | FDCAN_INT_RF1L)

#define FDCAN_TXERR_INTS    (FDCAN_INT_TEFL | FDCAN_INT_PEA | FDCAN_INT_PED)

/* Common-, TX- and RX-Error-Mask */

#define FDCAN_ANYERR_INTS (FDCAN_CMNERR_INTS | FDCAN_RXERR_INTS | FDCAN_TXERR_INTS)

#define CAN_ERROR_PASSIVE_THRESHOLD 128

#define CAN_ERROR_WARNING_THRESHOLD 96

#define FDCAN_MSGRAM_WORDS         (212)
#define STM32_CANRAM1_BASE         (STM32_FDCAN_SRAM_BASE + 0x0000)
#define STM32_CANRAM2_BASE         (STM32_FDCAN_SRAM_BASE + 1*(FDCAN_MSGRAM_WORDS * 4))

#ifdef CONFIG_STM32_FDCAN1
#  define FDCAN1_STDFILTER_SIZE    (28)
#  define FDCAN1_EXTFILTER_SIZE    (8)
#  define FDCAN1_RXFIFO0_SIZE      (3)
#  define FDCAN1_RXFIFO1_SIZE      (3)
#  define FDCAN1_TXEVENTFIFO_SIZE  (3)
#  define FDCAN1_TXFIFIOQ_SIZE     (3)

#  define FDCAN1_STDFILTER_WORDS   (28)
#  define FDCAN1_EXTFILTER_WORDS   (16)
#  define FDCAN1_RXFIFO0_WORDS     (54)
#  define FDCAN1_RXFIFO1_WORDS     (54)
#  define FDCAN1_TXEVENTFIFO_WORDS (6)
#  define FDCAN1_TXFIFIOQ_WORDS    (54)
#endif
#ifdef CONFIG_STM32_FDCAN2
#  define FDCAN2_STDFILTER_SIZE    (28)
#  define FDCAN2_EXTFILTER_SIZE    (8)
#  define FDCAN2_RXFIFO0_SIZE      (3)
#  define FDCAN2_RXFIFO1_SIZE      (3)
#  define FDCAN2_TXEVENTFIFO_SIZE  (3)
#  define FDCAN2_TXFIFIOQ_SIZE     (3)

#  define FDCAN2_STDFILTER_WORDS   (28)
#  define FDCAN2_EXTFILTER_WORDS   (16)
#  define FDCAN2_RXFIFO0_WORDS     (54)
#  define FDCAN2_RXFIFO1_WORDS     (54)
#  define FDCAN2_TXEVENTFIFO_WORDS (6)
#  define FDCAN2_TXFIFIOQ_WORDS    (54)
#endif

/* FDCAN1 Message RAM Layout */

#define FDCAN1_STDFILTER_INDEX   0
#define FDCAN1_EXTFILTERS_INDEX  (FDCAN1_STDFILTER_INDEX + FDCAN1_STDFILTER_WORDS)
#define FDCAN1_RXFIFO0_INDEX     (FDCAN1_EXTFILTERS_INDEX + FDCAN1_EXTFILTER_WORDS)
#define FDCAN1_RXFIFO1_INDEX     (FDCAN1_RXFIFO0_INDEX + FDCAN1_RXFIFO0_WORDS)
#define FDCAN1_TXEVENTFIFO_INDEX (FDCAN1_RXFIFO1_INDEX + FDCAN1_RXFIFO1_WORDS)
#define FDCAN1_TXFIFOQ_INDEX     (FDCAN1_TXEVENTFIFO_INDEX + FDCAN1_TXEVENTFIFO_WORDS)
#define FDCAN1_MSGRAM_WORDS      (FDCAN1_TXFIFOQ_INDEX + FDCAN1_TXFIFIOQ_WORDS)

#define FDCAN2_STDFILTER_INDEX   0
#define FDCAN2_EXTFILTERS_INDEX  (FDCAN2_STDFILTER_INDEX + FDCAN2_STDFILTER_WORDS)
#define FDCAN2_RXFIFO0_INDEX     (FDCAN2_EXTFILTERS_INDEX + FDCAN2_EXTFILTER_WORDS)
#define FDCAN2_RXFIFO1_INDEX     (FDCAN2_RXFIFO0_INDEX + FDCAN2_RXFIFO0_WORDS)
#define FDCAN2_TXEVENTFIFO_INDEX (FDCAN2_RXFIFO1_INDEX + FDCAN2_RXFIFO1_WORDS)
#define FDCAN2_TXFIFOQ_INDEX     (FDCAN2_TXEVENTFIFO_INDEX + FDCAN2_TXEVENTFIFO_WORDS)
#define FDCAN2_MSGRAM_WORDS      (FDCAN2_TXFIFOQ_INDEX + FDCAN2_TXFIFIOQ_WORDS)

/* General Configuration ****************************************************/

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required; HPWORK is recommended
#else

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 *
 * The high-priority work queue is suggested in order to minimize latency of
 * critical Rx/Tx transactions on the CAN bus.
 */

#  if defined(CONFIG_STM32_FDCAN_HPWORK)
#    define CANWORK HPWORK
#  elif defined(CONFIG_STM32_FDCAN_LPWORK)
#    define CANWORK LPWORK
#  else
#    define CANWORK LPWORK
#  endif
#endif

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
#  define TX_TIMEOUT_WQ
#endif

/* Message RAM Configuration ************************************************/

#define WORD_LENGTH         4U

/* Define number of Rx / Tx elements in message RAM; note that elements are
 * given sizes in number of words (4-byte chunks)
 *
 * Up to 64 Rx elements and 32 Tx elements may be configured per interface
 *
 * Note there are a total of 2560 words available shared between all FDCAN
 * interfaces for Rx, Tx, and filter storage
 */

#ifdef CONFIG_NET_CAN_CANFD
#  error "Not supported"
#else
#  define FIFO_ELEMENT_SIZE 4  /* size (in Words) of a FIFO element in message RAM (CAN_MTU / 4) */
#  define NUM_RX_FIFO0      3  /* 64 elements max for RX FIFO0 */
#  define NUM_RX_FIFO1      3  /* No elements for RX FIFO1 */
#  define NUM_TX_FIFO       3  /* 32 elements max for TX FIFO */
#endif

/* Intermediate message buffering *******************************************/

#define TX_POOL_SIZE        1
#define RX_POOL_SIZE        256

#if defined(CONFIG_NET_CAN_RAW_TX_DEADLINE) || defined(CONFIG_NET_TIMESTAMP)
#  define MSG_DATA            sizeof(struct timeval)
#else
#  define MSG_DATA            0
#endif

#ifdef CONFIG_NET_CAN_CANFD
#  define FRAME_TYPE struct canfd_frame
#else
#  define FRAME_TYPE struct can_frame
#endif

/* CAN Clock Configuration **************************************************/

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

/* Interrupts ***************************************************************/

#define FDCAN_IR_MASK 0x3fcfffff /* Mask of all non-reserved bits in FDCAN_IR */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CAN ID word, as defined by FDCAN device (Note xtd/rtr/esi bit positions) */

union can_id_u
{
  volatile uint32_t can_id;
  struct
  {
    volatile uint32_t extid : 29;
    volatile uint32_t resex : 3;
  };
  struct
  {
    volatile uint32_t res   : 18;
    volatile uint32_t stdid : 11;
    volatile uint32_t rtr   : 1;
    volatile uint32_t xtd   : 1;
    volatile uint32_t esi   : 1;
  };
};

/* Union of 4 bytes as 1 register */

union payload_u
{
  volatile uint32_t word;
  struct
  {
    volatile uint32_t b00 : 8;
    volatile uint32_t b01 : 8;
    volatile uint32_t b02 : 8;
    volatile uint32_t b03 : 8;
  };
};

/* Message RAM Structures ***************************************************/

/* Rx FIFO Element Header -- RM0433 pg 2536 */

union rx_fifo_header_u
{
  struct
  {
    volatile uint32_t w0;
    volatile uint32_t w1;
  };

  struct
  {
    /* First word */

    union can_id_u id;

    /* Second word */

    volatile uint32_t rxts : 16; /* Rx timestamp */
    volatile uint32_t dlc  : 4;  /* Data length code */
    volatile uint32_t brs  : 1;  /* Bitrate switching */
    volatile uint32_t fdf  : 1;  /* FD frame */
    volatile uint32_t res  : 2;  /* Reserved for Tx Event */
    volatile uint32_t fidx : 7;  /* Filter index */
    volatile uint32_t anmf : 1;  /* Accepted non-matching frame */
  };
};

/* Tx FIFO Element Header -- RM0433 pg 2538 */

union tx_fifo_header_u
{
  struct
  {
    volatile uint32_t w0;
    volatile uint32_t w1;
  };

  struct
  {
    /* First word */

    union can_id_u id;

    /* Second word */

    volatile uint32_t res1 : 16; /* Reserved for Tx Event timestamp */
    volatile uint32_t dlc  : 4;  /* Data length code */
    volatile uint32_t brs  : 1;  /* Bitrate switching */
    volatile uint32_t fdf  : 1;  /* FD frame */
    volatile uint32_t res2 : 1;  /* Reserved for Tx Event */
    volatile uint32_t efc  : 1;  /* Event FIFO control */
    volatile uint32_t mm   : 8;  /* Message marker (user data; copied to Tx Event) */
  };
};

/* Rx FIFO Element */

struct rx_fifo_s
{
  union rx_fifo_header_u header;
#ifdef CONFIG_NET_CAN_CANFD
  union payload_u data[16]; /* 64-byte FD payload */
#else
  union payload_u data[2];  /* 8-byte Classic payload */
#endif
};

/* Tx FIFO Element */

struct tx_fifo_s
{
  union tx_fifo_header_u header;
#ifdef CONFIG_NET_CAN_CANFD
  union payload_u data[16]; /* 64-byte FD payload */
#else
  union payload_u data[2];  /* 8-byte Classic payload */
#endif
};

/* Tx Mailbox Status Tracking */

#define TX_ABORT -1
#define TX_FREE   0
#define TX_BUSY   1

struct txmbstats
{
#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  struct timeval deadline;
  struct wdog_s txtimeout;
#endif
  int8_t pending;
};

/* FDCAN Device hardware configuration **************************************/

struct stm32_msgram_s
{
  volatile uint32_t * stdfilters;    /* Standard filters */
  volatile uint32_t * extfilters;    /* Extended filters */
  volatile uint32_t * rxfifo0;       /* RX FIFO0 */
  volatile uint32_t * rxfifo1;       /* RX FIFO1 */
  volatile uint32_t * txeventfifo;   /* TX event FIFO */
  volatile uint32_t * txfifoq;       /* TX FIFO queue */
};

struct fdcan_config_s
{
  uint32_t tx_pin;           /* GPIO configuration for TX */
  uint32_t rx_pin;           /* GPIO configuration for RX */
  uint32_t mb_irq[2];        /* FDCAN Interrupt line 0 (RX + errors), line 1 (TX completions) */

  struct stm32_msgram_s msgram;
  int8_t nstdfilters;        /* Number of standard filters */
  uint8_t rxfifo0esize;      /* RX FIFO0 element size (words) */
  uint8_t rxfifo1esize;      /* RX FIFO1 element size (words) */
  uint8_t nextfilters;       /* Number of extended filters */
  uint8_t txbufferesize;     /* TX buffer element size (words) */
};

struct fdcan_bitseg
{
  uint32_t bitrate;
  uint8_t sjw;
  uint8_t bs1;
  uint8_t bs2;
  uint8_t prescaler;
};

struct fdcan_message_ram
{
  uint32_t filt_stdid_addr;
  uint32_t filt_extid_addr;
  uint32_t rxfifo0_addr;
  uint32_t rxfifo1_addr;
  uint32_t txfifo_addr;
  uint8_t n_stdfilt;
  uint8_t n_extfilt;
  uint8_t n_rxfifo0;
  uint8_t n_rxfifo1;
  uint8_t n_txfifo;
};

/* FDCAN device structures **************************************************/

#ifdef CONFIG_STM32_FDCAN1
static const struct fdcan_config_s stm32_fdcan0_config =
{
  .tx_pin      = GPIO_FDCAN1_TX,
  .rx_pin      = GPIO_FDCAN1_RX,
  .mb_irq      =
  {
    STM32_IRQ_FDCAN1_IT0,
    STM32_IRQ_FDCAN1_IT1,
  },

  .msgram =
  {
    (uint32_t *)(STM32_CANRAM1_BASE + (FDCAN1_STDFILTER_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM1_BASE + (FDCAN1_EXTFILTERS_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM1_BASE + (FDCAN1_RXFIFO0_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM1_BASE + (FDCAN1_RXFIFO1_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM1_BASE + (FDCAN1_TXEVENTFIFO_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM1_BASE + (FDCAN1_TXFIFOQ_INDEX << 2))
  },
  .nstdfilters      = FDCAN1_STDFILTER_SIZE,
  .nextfilters      = FDCAN1_EXTFILTER_SIZE,
  .rxfifo0esize     = (FDCAN1_RXFIFO0_WORDS / FDCAN1_RXFIFO0_SIZE),
  .rxfifo1esize     = (FDCAN1_RXFIFO1_WORDS / FDCAN1_RXFIFO1_SIZE),
  .txbufferesize    = (FDCAN1_TXFIFIOQ_WORDS / FDCAN1_TXFIFIOQ_SIZE),
};
#endif

#ifdef CONFIG_STM32_FDCAN2
static const struct fdcan_config_s stm32_fdcan1_config =
{
  .tx_pin      = GPIO_FDCAN2_TX,
  .rx_pin      = GPIO_FDCAN2_RX,
  .mb_irq      =
  {
    STM32_IRQ_FDCAN2_IT0,
    STM32_IRQ_FDCAN2_IT1 ,
  },

  .msgram =
  {
    (uint32_t *)(STM32_CANRAM2_BASE + (FDCAN2_STDFILTER_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM2_BASE + (FDCAN2_EXTFILTERS_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM2_BASE + (FDCAN2_RXFIFO0_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM2_BASE + (FDCAN2_RXFIFO1_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM2_BASE + (FDCAN2_TXEVENTFIFO_INDEX << 2)),
    (uint32_t *)(STM32_CANRAM2_BASE + (FDCAN2_TXFIFOQ_INDEX << 2))
  },
  .nstdfilters      = FDCAN2_STDFILTER_SIZE,
  .nextfilters      = FDCAN2_EXTFILTER_SIZE,
  .rxfifo0esize     = (FDCAN2_RXFIFO0_WORDS / FDCAN2_RXFIFO0_SIZE),
  .rxfifo1esize     = (FDCAN2_RXFIFO1_WORDS / FDCAN2_RXFIFO1_SIZE),
  .txbufferesize    = (FDCAN2_TXFIFIOQ_WORDS / FDCAN2_TXFIFIOQ_SIZE),
};
#endif

#ifdef CONFIG_STM32_FDCAN3
#  error "FDCAN3 support not yet added to stm32h5xxx header files (pinmap, irq, etc.)"
#endif

/* The fdcan_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct fdcan_driver_s
{
  const struct fdcan_config_s *config;  /* Pin config */
  uint8_t iface_idx;                    /* FDCAN interface index (0 or 1) */
  uint32_t base;                        /* FDCAN base address */

  struct fdcan_bitseg arbi_timing;      /* Timing for arbitration phase */
#ifdef CONFIG_NET_CAN_CANFD
  struct fdcan_bitseg data_timing;      /* Timing for data phase */
#endif

  struct fdcan_message_ram message_ram; /* Start addresses for each reagion of Message RAM */

  /* Work queue configs for deferring interrupt and poll work */

  struct work_s rxwork;
  struct work_s txcwork;
  struct work_s txdwork;
  struct work_s pollwork;
#ifdef CONFIG_NET_CAN_ERRORS
  struct work_s irqwork;
  volatile uint32_t saved_psr;      /* PSR captured in ISR before any TXBCR write */
  volatile uint32_t saved_errflags; /* Error IR flags captured and W1C-cleared in ISR; used by fdcan_error_work */
#endif

  /* Intermediate storage of Tx / Rx frames outside of Message RAM */

  uint8_t tx_pool[(sizeof(FRAME_TYPE)+MSG_DATA)*TX_POOL_SIZE];
  uint8_t rx_pool[(sizeof(FRAME_TYPE)+MSG_DATA)*RX_POOL_SIZE];
  volatile uint8_t rx_pool_head;    /* Next slot to write (ISR producer) */
  volatile uint8_t rx_pool_tail;    /* Next slot to read  (worker consumer) */
  volatile uint8_t rx_pool_count;   /* Number of frames in the pool */

  struct net_driver_s dev;              /* Interface understood by the network */
  bool bifup;                           /* true:ifup false:ifdown */
  bool listen_only_mode;                /* true:listen-only false:normal */
  bool abort_tx_on_error;               /* Aborts TX MBs on Error Interrupt */
  struct txmbstats txmb[NUM_TX_FIFO];   /* Track deadline and status of every Tx entry */

  uint32_t pwr_pin;      /* GPIO for transceiver power */
  bool pwr_pin_standby;  /* value to place transceiver in standby, opposite is active */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32_FDCAN1
static struct fdcan_driver_s g_fdcan0;
#endif

#ifdef CONFIG_STM32_FDCAN2
static struct fdcan_driver_s g_fdcan1;
#endif

#ifdef CONFIG_STM32_FDCAN3
static struct fdcan_driver_s g_fdcan2;
#endif

static bool g_apb1h_init = false;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static bool fdcan_txringfull(struct fdcan_driver_s *priv);
static int  fdcan_transmit(struct fdcan_driver_s *priv);
static int  fdcan_txpoll(struct net_driver_s *dev);

/* Helper functions */

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
static void fdcan_dumpregs(struct fdcan_driver_s *priv);
#endif

int32_t fdcan_bittiming(struct fdcan_bitseg *timing);

static void fdcan_setinit(uint32_t base, uint32_t init);
static void fdcan_setenable(uint32_t base, uint32_t enable);
static void fdcan_setconfig(uint32_t base, uint32_t config_enable);
static bool fdcan_waitccr_change(uint32_t base,
                                 uint32_t mask,
                                 uint32_t target_state);

static void fdcan_enable_interrupts(struct fdcan_driver_s *priv);
static void fdcan_disable_interrupts(struct fdcan_driver_s *priv);

/* Interrupt handling */

static void fdcan_receive(struct fdcan_driver_s *priv);
static void fdcan_receive_work(void *arg);
static void fdcan_txdone(struct fdcan_driver_s *priv);
static void fdcan_txdone_work(void *arg);

static int  fdcan_interrupt(int irq, void *context,
                            void *arg);

static void fdcan_check_errors(struct fdcan_driver_s *priv);

/* Watchdog timer expirations */

#ifdef TX_TIMEOUT_WQ
static void fdcan_txtimeout_work(void *arg);
static void fdcan_txtimeout_expiry(wdparm_t arg);
#endif

/* NuttX networking stack callback functions */

static int fdcan_ifup(struct net_driver_s *dev);
static int fdcan_ifdown(struct net_driver_s *dev);

static void fdcan_txavail_work(void *arg);
static int  fdcan_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NETDEV_IOCTL
static int  fdcan_netdev_ioctl(struct net_driver_s *dev, int cmd,
                               unsigned long arg);
#endif

/* Initialization and Reset */

static int  fdcan_initialize(struct fdcan_driver_s *priv);
static void fdcan_reset(struct fdcan_driver_s *priv);

#ifdef CONFIG_NET_CAN_ERRORS

/* CAN errors interrupt handling */

static void fdcan_error_work(void *arg);
static void fdcan_error(struct fdcan_driver_s *priv, uint32_t status,
                        uint32_t psr);
static void fdcan_errint(struct fdcan_driver_s *priv, bool enable);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdcan_dumpregs
 *
 * Dump common register values to the console for debugging purposes.
 ****************************************************************************/

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
static void fdcan_dumpregs(struct fdcan_driver_s *priv)
{
  printf("-------------- FDCAN Reg Dump ----------------\n");
  printf("CAN%d Base: 0x%lx\n", priv->iface_idx, priv->base);

  uint32_t regval;

  regval = getreg32(priv->base + STM32_FDCAN_CCCR_OFFSET);
  printf("CCCR = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_ECR_OFFSET);
  printf("ECR  = 0x%lx\n", regval);

  regval = getreg32(priv->base + STM32_FDCAN_NBTP_OFFSET);
  printf("NBTP = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_DBTP_OFFSET);
  printf("DBTP = 0x%lx\n", regval);

  regval = getreg32(priv->base + STM32_FDCAN_TXBC_OFFSET);
  printf("TXBC = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_RXF0C_OFFSET);
  printf("RXF0C = 0x%lx\n", regval);

  regval = getreg32(priv->base + STM32_FDCAN_TXESC_OFFSET);
  printf("TXESC = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_RXESC_OFFSET);
  printf("RXESC = 0x%lx\n", regval);

  regval = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);
  printf("IE   = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_ILE_OFFSET);
  printf("ILE = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_ILS_OFFSET);
  printf("ILS = 0x%lx\n", regval);

  /* Print out some possibly interesting unhandled interrupts */

  regval = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);
  printf("IR   = 0x%lx\n", regval);

  if (regval & FDCAN_INT_PEA || regval & FDCAN_INT_PED)
    {
      /* Protocol error -- check protocol status register for details */

      regval = getreg32(priv->base + STM32_FDCAN_PSR_OFFSET);
      printf("--PSR.LEC = %d\n", regval & FDCAN_PSR_LEC);
    }
}
#endif

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

int32_t fdcan_bittiming(struct fdcan_bitseg *timing)
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

  if ((prescaler < 1U) || (prescaler > 1024U))
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
  ninfo("[fdcan] STM32_FDCANCLK_FREQUENCY %lu, target_bitrate %lu, "
        "prescaler %lu, bs1 %d, bs2 %d\n",
        STM32_FDCANCLK_FREQUENCY, target_bitrate, prescaler_bs, bs1 - 1,
        bs2 - 1);
#endif

  timing->bs1 = (uint8_t)(bs1 - 1);
  timing->bs2 = (uint8_t)(bs2 - 1);
  timing->prescaler = (uint16_t)(prescaler - 1);
  timing->sjw = 0; /* Which means one */

  return 0;
}

/****************************************************************************
 * Function: fdcan_txringfull
 *
 * Description:
 *   Check if all of the TX descriptors are in use.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   true is the TX ring is full; false if there are free slots at the
 *   head index.
 *
 ****************************************************************************/

static bool fdcan_txringfull(struct fdcan_driver_s *priv)
{
  /* Check if the Tx queue is full */

  uint32_t regval = getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET);

  if ((regval & FDCAN_TXFQS_TFQF) == FDCAN_TXFQS_TFQF)
    {
      return true; /* Sorry, out of room, try back later */
    }

  return false;
}

/****************************************************************************
 * Function: fdcan_transmit
 *
 * Description:
 *   Start hardware transmission of the data contained in priv->d_buf. Called
 *   either from the txdone interrupt handling or from watchdog based polling
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int fdcan_transmit(struct fdcan_driver_s *priv)
{
  irqstate_t flags = enter_critical_section();

  /* First, check if there are any slots available in the queue */

  uint32_t regval = getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET);

  if ((regval & FDCAN_TXFQS_TFQF) == FDCAN_TXFQS_TFQF)
    {
      /* Tx FIFO / Queue is full */

      leave_critical_section(flags);
      return -EBUSY;
    }

  /* Next, get the next available FIFO index from the controller */

  regval = getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET);
  const uint8_t mbi = (regval & FDCAN_TXFQS_TFQPI_MASK) >>
                       FDCAN_TXFQS_TFQPI_SHIFT;

  /* Now, we can copy the CAN frame to the FIFO (in message RAM) */

  if (mbi >= NUM_TX_FIFO)
    {
      nerr("Invalid Tx mailbox index encountered in transmit\n");
      leave_critical_section(flags);
      return -EIO;
    }

  /* struct tx_fifo_s *mb = &priv->tx[mbi]; */

  struct tx_fifo_s *mb =
    (struct tx_fifo_s *)(priv->config->msgram.txfifoq +
                         priv->config->txbufferesize * mbi);

  /* Setup timeout deadline if enabled */

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  int32_t timeout = 0;
  struct timespec ts;

  clock_systime_timespec(&ts);

  if (priv->dev.d_sndlen > priv->dev.d_len)
    {
      /* Tx deadline is stored in d_buf after frame data */

      struct timeval *tv =
             (struct timeval *)(priv->dev.d_buf + priv->dev.d_len);

      priv->txmb[mbi].deadline = *tv;
      timeout  = (tv->tv_sec - ts.tv_sec)*CLK_TCK
                 + ((tv->tv_usec - ts.tv_nsec / 1000)*CLK_TCK) / 1000000;
      if (timeout < 0)
        {
          leave_critical_section(flags);
          return 0;  /* No transmission for you! */
        }
    }
  else
    {
      /* Default TX deadline defined in NET_CAN_RAW_DEFAULT_TX_DEADLINE */

      if (CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE > 0)
        {
          timeout = ((CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE / 1000000)
              *CLK_TCK);
          priv->txmb[mbi].deadline.tv_sec = ts.tv_sec +
              CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE / 1000000;
          priv->txmb[mbi].deadline.tv_usec = (ts.tv_nsec / 1000) +
              CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE % 1000000;
        }
      else
        {
          priv->txmb[mbi].deadline.tv_sec = 0;
          priv->txmb[mbi].deadline.tv_usec = 0;
        }
    }
#endif

  /* Attempt to write frame */

  union tx_fifo_header_u header;

  if (priv->dev.d_len == sizeof(struct can_frame))
    {
      struct can_frame *frame = (struct can_frame *)priv->dev.d_buf;

      if (frame->can_id & CAN_EFF_FLAG)
        {
          header.id.xtd = 1;
          header.id.extid = frame->can_id & CAN_EFF_MASK;
        }
      else
        {
          header.id.xtd = 0;
          header.id.stdid = frame->can_id & CAN_SFF_MASK;
        }

      header.id.esi = frame->can_id & CAN_ERR_FLAG ? 1 : 0;
      header.id.rtr = frame->can_id & CAN_RTR_FLAG ? 1 : 0;
      header.dlc = frame->can_dlc;
      header.brs = 0;  /* No bitrate switching */
      header.fdf = 0;  /* Classic CAN frame, not CAN-FD */
      header.efc = 0;  /* Don't store Tx events */
      header.mm = mbi; /* Mailbox Marker for our own use; just store FIFO index */

      /* Store into message RAM */

      mb->header.w0 = header.w0;
      mb->header.w1 = header.w1;
      mb->data[0].word = *(uint32_t *)&frame->data[0];
      mb->data[1].word = *(uint32_t *)&frame->data[4];
    }
#ifdef CONFIG_NET_CAN_CANFD
  else /* CAN FD frame */
    {
      struct canfd_frame *frame = (struct canfd_frame *)priv->dev.d_buf;

      if (frame->can_id & CAN_EFF_FLAG)
        {
          header.id.xtd = 1;
          header.id.extid = frame->can_id & CAN_EFF_MASK;
        }
      else
        {
          header.id.xtd = 0;
          header.id.stdid = frame->can_id & CAN_SFF_MASK;
        }

      const bool brs =
        (priv->arbi_timing.bitrate == priv->data_timing.bitrate) ? 0 : 1;

      header.id.esi = (frame->can_id & CAN_ERR_FLAG) ? 1 : 0;
      header.id.rtr = (frame->can_id & CAN_RTR_FLAG) ? 1 : 0;
      header.dlc = len_to_can_dlc[frame->len];
      header.brs = brs; /* Bitrate switching */
      header.fdf = 1;   /* CAN-FD frame */
      header.efc = 0;   /* Don't store Tx events */
      header.mm = mbi;  /* Mailbox Marker for our own use; just store FIFO index */

      /* Store into message RAM */

      mb->header.w1 = header.w1;
      mb->header.w0 = header.w0;

      uint32_t *frame_data_word = (uint32_t *)&frame->data[0];

      for (int i = 0; i < (frame->len + 4 - 1) / 4; i++)
        {
          mb->data[i].word = frame_data_word[i];
        }
    }
#endif

  /* GO - Submit the transmission request for this element */

  putreg32((1 << mbi), priv->base + STM32_FDCAN_TXBTIE_OFFSET);

  putreg32(1 << mbi, priv->base + STM32_FDCAN_TXBAR_OFFSET);

  /* Increment statistics */

  NETDEV_TXPACKETS(&priv->dev);

  priv->txmb[mbi].pending = TX_BUSY;

#ifdef TX_TIMEOUT_WQ
  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  if (timeout > 0)
    {
      wd_start(&priv->txmb[mbi].txtimeout, timeout + 1,
               fdcan_txtimeout_expiry, (wdparm_t)priv);
    }
#endif

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Function: fdcan_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int fdcan_txpoll(struct net_driver_s *dev)
{
  struct fdcan_driver_s *priv =
    (struct fdcan_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Send the packet */

      fdcan_transmit(priv);

      /* Check if there is room in the device to hold another packet. If
       * not, return a non-zero value to terminate the poll.
       */

      if (fdcan_txringfull(priv))
        {
          return -EBUSY;
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: fdcan_receive
 *
 * Description:
 *   Called from interrupt context.  Drains both RX FIFO0 and FIFO1 into
 *   the rx_pool ring buffer, acks the RX interrupt, and schedules
 *   fdcan_receive_work to pass the frames up to the upper layer.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 ****************************************************************************/

static void fdcan_receive(struct fdcan_driver_s *priv)
{
  /* Check the interrupt value to determine which FIFO(s) to read */

  uint32_t ir = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);

  const uint32_t ir_fifo0 = FDCAN_INT_RF0N | FDCAN_INT_RF0F;
  const uint32_t ir_fifo1 = FDCAN_INT_RF1N | FDCAN_INT_RF1F;

  uint32_t ack_flags = 0;

  /* Drain both FIFOs if they have pending frames */

  for (int fifo_id = 0; fifo_id <= 1; fifo_id++)
    {
      uint32_t ir_mask = (fifo_id == 0) ? ir_fifo0 : ir_fifo1;

      if ((ir & ir_mask) == 0)
        {
          continue;
        }

      uint32_t offset_rxfns = (fifo_id == 0) ? STM32_FDCAN_RXF0S_OFFSET
                                              : STM32_FDCAN_RXF1S_OFFSET;
      uint32_t offset_rxfna = (fifo_id == 0) ? STM32_FDCAN_RXF0A_OFFSET
                                              : STM32_FDCAN_RXF1A_OFFSET;

      volatile uint32_t *const rxfns =
        (uint32_t *)(priv->base + offset_rxfns);
      volatile uint32_t *const rxfna =
        (uint32_t *)(priv->base + offset_rxfna);

      /* Drain all available elements from this FIFO into rx_pool */

      while ((*rxfns & FDCAN_RXFS_FFL_MASK) > 0)
        {
          if (priv->rx_pool_count >= RX_POOL_SIZE)
            {
              /* Pool full -- cannot store more; frames remain in HW FIFO */

              NETDEV_RXERRORS(&priv->dev);
              break;
            }

          const uint8_t gi = (*rxfns & FDCAN_RXFS_FGI_MASK) >>
                              FDCAN_RXFS_FGI_SHIFT;
          struct rx_fifo_s *rf;

          if (fifo_id == 0)
            {
              rf = (struct rx_fifo_s *)(priv->config->msgram.rxfifo0 +
                    priv->config->rxfifo0esize * gi);
            }
          else
            {
              rf = (struct rx_fifo_s *)(priv->config->msgram.rxfifo1 +
                    priv->config->rxfifo1esize * gi);
            }

          /* Copy frame from message RAM into the rx_pool slot */

          const size_t frame_slot_size = sizeof(FRAME_TYPE) + MSG_DATA;
          uint8_t *slot =
            &priv->rx_pool[priv->rx_pool_head * frame_slot_size];

#ifdef CONFIG_NET_CAN_CANFD
          if (rf->header.fdf)
            {
              struct canfd_frame *frame = (struct canfd_frame *)slot;

              if (rf->header.id.xtd)
                {
                  frame->can_id  = CAN_EFF_MASK & rf->header.id.extid;
                  frame->can_id |= CAN_EFF_FLAG;
                }
              else
                {
                  frame->can_id = CAN_SFF_MASK & rf->header.id.stdid;
                }

              if (rf->header.id.rtr)
                {
                  frame->can_id |= CAN_RTR_FLAG;
                }

              frame->len = can_dlc_to_len[rf->header.dlc];

              uint32_t *frame_data_word = (uint32_t *)&frame->data[0];

              for (int i = 0; i < (frame->len + 4 - 1) / 4; i++)
                {
                  frame_data_word[i] = rf->data[i].word;
                }
            }
          else
#endif
            {
              struct can_frame *frame = (struct can_frame *)slot;

              if (rf->header.id.xtd)
                {
                  frame->can_id  = CAN_EFF_MASK & rf->header.id.extid;
                  frame->can_id |= CAN_EFF_FLAG;
                }
              else
                {
                  frame->can_id = CAN_SFF_MASK & rf->header.id.stdid;
                }

              if (rf->header.id.rtr)
                {
                  frame->can_id |= CAN_RTR_FLAG;
                }

              frame->can_dlc = rf->header.dlc;

              *(uint32_t *)&frame->data[0] = rf->data[0].word;
              *(uint32_t *)&frame->data[4] = rf->data[1].word;
            }

          /* Acknowledge receipt of this FIFO element in the HW */

          putreg32(gi, rxfna);

          /* Advance the pool head */

          priv->rx_pool_head = (priv->rx_pool_head + 1) % RX_POOL_SIZE;
          priv->rx_pool_count++;
        }

      if ((*rxfns & FDCAN_RXFS_FFL_MASK) == 0)
        {
          /* FIFO fully drained -- safe to ack its interrupt flags */

          ack_flags |= ir_mask;
        }
      else
        {
          /* Pool was full, FIFO still has frames.  Disable this FIFO's
           * interrupts to prevent an interrupt storm.  The worker will
           * re-enable them after consuming pool entries; the un-acked IR
           * flag will then immediately re-trigger this ISR.
           */

          uint32_t ie = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);

          ie &= ~ir_mask;
          putreg32(ie, priv->base + STM32_FDCAN_IE_OFFSET);
        }
    }

  /* Only ack interrupt flags for FIFOs that were fully drained */

  if (ack_flags != 0)
    {
      putreg32(ack_flags, priv->base + STM32_FDCAN_IR_OFFSET);
    }

  /* Schedule the worker to consume the pooled frames */

  if (priv->rx_pool_count > 0)
    {
      work_queue(CANWORK, &priv->rxwork, fdcan_receive_work, priv, 0);
    }
}

/****************************************************************************
 * Function: fdcan_receive_work
 *
 * Description:
 *   Consume frames previously stored into the rx_pool ring buffer by the
 *   fdcan_receive ISR and pass them up to the upper-half driver via
 *   can_input().  Also check for errors and timeouts.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Scheduled in worker thread (HPWORK / LPWORK)
 *
 ****************************************************************************/

static void fdcan_receive_work(void *arg)
{
  net_lock();
  irqstate_t flags = enter_critical_section();

  struct fdcan_driver_s *priv = (struct fdcan_driver_s *)arg;
  const size_t frame_slot_size = sizeof(FRAME_TYPE) + MSG_DATA;

  /* Consume all frames that the ISR placed into rx_pool */

  while (priv->rx_pool_count > 0)
    {
      uint8_t *slot = &priv->rx_pool[priv->rx_pool_tail * frame_slot_size];

#ifdef CONFIG_NET_CAN_CANFD
      /* Peek at the slot to determine frame type.  The ISR stored either
       * a canfd_frame (fdf=1) or a can_frame into the slot.  We use
       * sizeof to decide -- the ISR always wrote the correct struct.
       * For simplicity we pass the larger type and set d_len accordingly.
       */

      struct canfd_frame *fd_frame = (struct canfd_frame *)slot;

      /* A CAN-FD frame will have been stored with its full canfd_frame
       * layout.  A classic frame will have been stored as can_frame.
       * We distinguish by checking if the slot was written as canfd_frame
       * (len > 8) vs can_frame.  However, since the ISR doesn't mark
       * the type, we always pass as can_frame when CANFD is enabled
       * unless the length indicates otherwise.  For now, mirror the
       * original code: try canfd first, fall back to can.
       */

      if (fd_frame->len > 8)
        {
          priv->dev.d_len = sizeof(struct canfd_frame);
          priv->dev.d_buf = slot;
        }
      else
#endif
        {
          priv->dev.d_len = sizeof(struct can_frame);
          priv->dev.d_buf = slot;
        }

      /* Send to socket interface */

      can_input(&priv->dev);

      /* Update iface statistics */

      NETDEV_RXPACKETS(&priv->dev);

      /* Advance the pool tail */

      priv->rx_pool_tail = (priv->rx_pool_tail + 1) % RX_POOL_SIZE;
      priv->rx_pool_count--;
    }

  /* Point the packet buffer back to the Tx pool */

  priv->dev.d_buf = priv->tx_pool;

  /* Check for errors and abort-transmission requests */

  fdcan_check_errors(priv);

  /* Re-enable RX FIFO interrupts that may have been disabled by the ISR
   * when the pool was full.  The un-acked IR flags will immediately
   * re-trigger the ISR once we leave the critical section.
   */

  {
    uint32_t regval = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);

    regval |= FDCAN_INT_RF0N | FDCAN_INT_RF0F
              | FDCAN_INT_RF1N | FDCAN_INT_RF1F;
#ifdef CONFIG_NET_CAN_ERRORS
    regval |= FDCAN_RXERR_INTS;
#endif
    putreg32(regval, priv->base + STM32_FDCAN_IE_OFFSET);
  }

  leave_critical_section(flags);
  net_unlock();
}

/****************************************************************************
 * Function: fdcan_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from interrupt context
 *
 ****************************************************************************/

static void fdcan_txdone(struct fdcan_driver_s *priv)
{
  /* Read and reset the interrupt flag */

  uint32_t ir = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);

  if (ir & FDCAN_INT_TC)
    {
      putreg32(FDCAN_INT_TC, priv->base + STM32_FDCAN_IR_OFFSET);
    }
  else
    {
      nerr("Unexpected FCAN interrupt on line 1\n");
      return;
    }

  /* Schedule to perform the TX timeout processing on the worker thread */

  work_queue(CANWORK, &priv->txdwork, fdcan_txdone_work, priv, 0);
}

/****************************************************************************
 * Function: fdcan_txdone_work
 *
 * Description:
 *   Process completed transmissions, including canceling their watchdog
 *   timers if applicable
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Scheduled in worker thread (HPWORK / LPWORK)
 *
 ****************************************************************************/

static void fdcan_txdone_work(void *arg)
{
  net_lock();

  irqstate_t flags = enter_critical_section();

  struct fdcan_driver_s *priv = (struct fdcan_driver_s *)arg;

  /* Update counters for successful transmissions */

  for (uint8_t i = 0; i < NUM_TX_FIFO; i++)
    {
      if ((getreg32(priv->base + STM32_FDCAN_TXBTO_OFFSET) & (1 << i)) > 0)
        {
          /* Transmission Occurred in buffer i
           *   (Not necessarily a 'new' transmission, however)
           * Check that it's a new transmission, not a previously handled
           * transmission
           */

          struct txmbstats *txi = &priv->txmb[i];

          if (txi->pending == TX_BUSY)
            {
              /* This is a transmission that just now completed */

              NETDEV_TXDONE(&priv->dev);

              txi->pending = TX_FREE;

#ifdef TX_TIMEOUT_WQ
              /* We are here because a transmission completed, so the
               * corresponding watchdog can be canceled.
               */

              wd_cancel(&priv->txmb[i].txtimeout);
#endif
            }
        }
    }

  /* Check for errors and abort-transmission requests */

  fdcan_check_errors(priv);

#ifdef CONFIG_NET_CAN_ERRORS
  uint32_t regval = 0;

  /* Turning back on PEA and PED error interrupts */

  regval = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);
  regval |= (FDCAN_INT_PEA | FDCAN_INT_PED);
  putreg32(regval, priv->base + STM32_FDCAN_IE_OFFSET);
#endif

  /* There should be space for a new TX in any event
   * Poll the network for new data to transmit
   */

  devif_poll(&priv->dev, fdcan_txpoll);

  leave_critical_section(flags);
  net_unlock();
}

/****************************************************************************
 * Function: fdcan_interrupt
 *
 * Description:
 *   Common handler for all enabled FDCAN interrupts
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *   arg     - Unused
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int fdcan_interrupt(int irq, void *context,
                           void *arg)
{
  struct fdcan_driver_s *priv    = (struct fdcan_driver_s *)arg;

  DEBUGASSERT(priv != NULL);

  if (irq == priv->config->mb_irq[0])
    {
      /* Interrupt Line 0: RX FIFO events and all error groups.
       *
       * The ILS register is configured with FDCAN_ILS_SMG only, which routes
       * the SMG group (HPM, TC, TCF, TFE) to Line 1.  All other interrupt
       * groups — RXFIFO0, RXFIFO1, TFERR, MISC, BERR, PERR — remain on
       * Line 0.  Every bit in FDCAN_ANYERR_INTS belongs to one of those
       * groups, so the error check below is exhaustive on this line and can
       * never fire on Line 1.
       */

#ifdef CONFIG_NET_CAN_ERRORS
      uint32_t pending = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);

      if ((pending & FDCAN_ANYERR_INTS) != 0)
        {
          /* Disable further CAN ERROR interrupts and schedule
           * to perform the interrupt processing on the worker
           * thread
           */

          fdcan_errint(priv, false);

          /* Save error IR flags and clear them (W1C) before this ISR
           * returns.
           *
           * Before fdcan_error_work runs, this ISR may be re-entered for a
           * completely unrelated reason (e.g. a valid RX frame arriving on
           * Line 0 again, or a TX completion on Line 1).  The error block
           * guard above checks IR directly, so without the W1C clear below,
           * a re-entered Line 0 ISR would re-read the same error flags,
           * overwrite saved_errflags/saved_psr, and enqueue a second
           * spurious work item.  Clearing FDCAN_ANYERR_INTS from IR now
           * ensures any re-entry sees no pending error flags.
           *
           * PSR is snapshotted here rather than in fdcan_error_work because
           * a TXBCR write below immediately transitions PSR.ACT away from
           * TRANSMITTING; a live read in the work handler would miss that
           * state.
           */

          priv->saved_errflags = pending; /* pending already holds the IR snapshot */
          putreg32(FDCAN_ANYERR_INTS, priv->base + STM32_FDCAN_IR_OFFSET);

          priv->saved_psr = getreg32(priv->base + STM32_FDCAN_PSR_OFFSET);

          /* Abort all TX MBs if an error has occurred and the
           * abort_tx_on_error flag has been set via the ioctl call
           */

          if (priv->abort_tx_on_error)
            {
              uint32_t regval =
                getreg32(priv->base + STM32_FDCAN_TXBRP_OFFSET);
              uint32_t i = 0;

              for (i = 0; i < NUM_TX_FIFO; i++)
                {
                  struct txmbstats *txi = &priv->txmb[i];

                  if (txi->pending == TX_BUSY && ((1 << i) & regval))
                    {
                      putreg32(1 << i,
                               priv->base + STM32_FDCAN_TXBCR_OFFSET);
                      txi->pending = TX_FREE;
                      NETDEV_TXERRORS(&priv->dev);
                    }
                }
            }

          work_queue(CANWORK, &priv->irqwork, fdcan_error_work, priv, 0);
        }
#endif /* CONFIG_NET_CAN_ERRORS */

      fdcan_receive(priv);
    }
  else if (irq == priv->config->mb_irq[1])
    {
      /* Interrupt Line 1: TX completion events
       * (HPM, TC, TCF, TFE via SMG).
       */

      fdcan_txdone(priv);
    }
  else
    {
      nerr("Unexpected IRQ [%d]\n", irq);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Function: fdcan_check_errors
 *
 * Description:
 *   Check error flags and cancel any timed out transmissions
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static void fdcan_check_errors(struct fdcan_driver_s *priv)
{
  /* Read CAN Error Logging counter (This also resets the error counter) */

  uint32_t regval = getreg32(priv->base + STM32_FDCAN_ECR_OFFSET)
                  & FDCAN_ECR_CEL_MASK;
  const uint8_t cel = (uint8_t)(regval >> FDCAN_ECR_CEL_SHIFT);

  if (cel > 0)
    {
      /* We've had some errors; check the status of the device */

      regval = getreg32(priv->base + STM32_FDCAN_CCCR_OFFSET);
      bool restricted_op_mode = (regval & FDCAN_CCCR_ASM) > 0;

      if (restricted_op_mode)
        {
          nerr("Tx handler message RAM error -- resctricted mode enabled\n");

          /* Reset the CCCR.ASM register to exit restricted op mode */

          putreg32(regval & ~FDCAN_CCCR_ASM,
                   priv->base + STM32_FDCAN_CCCR_OFFSET);
        }
    }

  /* Serve abort requests */

  for (uint8_t i = 0; i < NUM_TX_FIFO; i++)
    {
      struct txmbstats *txi = &priv->txmb[i];

      regval = getreg32(priv->base + STM32_FDCAN_TXBRP_OFFSET);
      if (txi->pending == TX_ABORT && ((1 << i) & regval))
        {
          /* Request to Cancel Tx item */

          putreg32(1 << i, priv->base + STM32_FDCAN_TXBCR_OFFSET);
          txi->pending = TX_FREE;
          NETDEV_TXERRORS(&priv->dev);
#ifdef TX_TIMEOUT_WQ
          wd_cancel(&priv->txmb[i].txtimeout);
#endif
        }
    }
}

#ifdef TX_TIMEOUT_WQ
/****************************************************************************
 * Function: fdcan_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void fdcan_txtimeout_work(void *arg)
{
  struct fdcan_driver_s *priv = (struct fdcan_driver_s *)arg;

  struct timespec ts;
  struct timeval *now = (struct timeval *)&ts;

  clock_systime_timespec(&ts);
  now->tv_usec = ts.tv_nsec / 1000; /* timespec to timeval conversion */

  /* The watchdog timed out, yet we still check mailboxes in case the
   * transmit function transmitted a new frame
   */

  for (int mbi = 0; mbi < NUM_TX_FIFO; mbi++)
    {
      if (priv->txmb[mbi].deadline.tv_sec != 0
          && (now->tv_sec > priv->txmb[mbi].deadline.tv_sec
          || now->tv_usec > priv->txmb[mbi].deadline.tv_usec))
        {
          NETDEV_TXTIMEOUTS(&priv->dev);
          priv->txmb[mbi].pending = TX_ABORT;
        }
    }

  fdcan_check_errors(priv);
}

/****************************************************************************
 * Function: fdcan_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *
 * Input Parameters:
 *   arg - Pointer to the private FDCAN driver state structure
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void fdcan_txtimeout_expiry(wdparm_t arg)
{
  struct fdcan_driver_s *priv = (struct fdcan_driver_s *)arg;

  /* Schedule to perform the TX timeout processing on the worker thread */

  work_queue(CANWORK, &priv->txcwork, fdcan_txtimeout_work, priv, 0);
}
#endif /* TX_TIMEOUT_WQ */

/****************************************************************************
 * Function: fdcan_apb1hreset
 *
 * Description:
 *   Reset the periheral bus clock used by FDCAN
 *   Note that this will reset all configuration of all FDCAN peripherals
 *
 ****************************************************************************/

void fdcan_apb1hreset(void)
{
  /* Reset the FDCAN's peripheral bus clock */

  modifyreg32(STM32_RCC_APB1HRSTR, 0, RCC_APB1HRSTR_FDCANRST);
  modifyreg32(STM32_RCC_APB1HRSTR, RCC_APB1HRSTR_FDCANRST, 0);
}

/****************************************************************************
 * Function: fdcan_setinit
 *
 * Description:
 *   Enter / Exit initialization mode
 *
 * Input Parameters:
 *   base - The base pointer of the FDCAN peripheral
 *   init - true: Enter init mode; false: Exit init mode
 *
 ****************************************************************************/

static void fdcan_setinit(uint32_t base, uint32_t init)
{
  if (init)
    {
      /* Enter hardware initialization mode */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_INIT);
      fdcan_waitccr_change(base, FDCAN_CCCR_INIT, FDCAN_CCCR_INIT);
    }
  else
    {
      /* Exit hardware initialization mode */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_INIT, 0);
      fdcan_waitccr_change(base, FDCAN_CCCR_INIT, 0);
    }
}

/****************************************************************************
 * Function: fdcan_setenable
 *
 * Description:
 *   Power On / Power Off the device with the Clock Stop Request bit
 *
 * Input Parameters:
 *   base - The base pointer of the FDCAN peripheral
 *   init - true: Power on the device; false: Power off the device
 *
 ****************************************************************************/

static void fdcan_setenable(uint32_t base, uint32_t enable)
{
  if (enable)
    {
      /* Clear CSR bit */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_CSR, 0);
      fdcan_waitccr_change(base, FDCAN_CCCR_CSA, 0);
    }
  else
    {
      /* Set CSR bit */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_CSR);
      fdcan_waitccr_change(base, FDCAN_CCCR_CSA, 1);
    }
}

/****************************************************************************
 * Function: fdcan_setconfig
 *
 * Description:
 *   Enter / Exit Configuration Changes Enabled mode
 *
 * Input Parameters:
 *   base - The base pointer of the FDCAN peripheral
 *   init - true: Enter config mode; false: Exit config mode
 *
 ****************************************************************************/

static void fdcan_setconfig(uint32_t base, uint32_t config_enable)
{
  if (config_enable)
    {
      /* Configuration Changes Enabled (CCE) mode */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_CCE);
      fdcan_waitccr_change(base, FDCAN_CCCR_CCE, 1);
    }
  else
    {
      /* Exit CCE mode */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_CCE, 0);
      fdcan_waitccr_change(base, FDCAN_CCCR_CCE, 0);
    }
}

/****************************************************************************
 * Function: fdcan_waitccr_change
 *
 * Description:
 *   Wait for the CCR register to accept a requested change.
 *   Timeout after ~10ms.
 *
 * Input Parameters:
 *   base         - The base pointer of the FDCAN peripheral
 *   mask         - Mask to apply to the CCR register value
 *   target_state - Target masked value to wait for
 *
 * Returned Value:
 *   true on success; false on timeout
 *
 ****************************************************************************/

static bool fdcan_waitccr_change(uint32_t base, uint32_t mask,
                                     uint32_t target_state)
{
  const unsigned timeout = 1000;

  for (unsigned wait_ack = 0; wait_ack < timeout; wait_ack++)
    {
      const bool state = (getreg32(base + STM32_FDCAN_CCCR_OFFSET) & mask);

      if (state == target_state)
        {
          return true;
        }

      up_udelay(10);
    }

  return false;
}

/****************************************************************************
 * Function: fdcan_enable_interrupts
 *
 * Description:
 *   Enable all interrupts used by this driver
 *
 * Input Parameters:
 *   priv - Pointer to the private FDCAN driver state structure
 *
 * Assumptions:
 *   The peripheral is in Configuration Changes Enabled (CCE) mode
 *
 ****************************************************************************/

static void fdcan_enable_interrupts(struct fdcan_driver_s *priv)
{
  /* Enable both interrupt lines at the device level */

  const uint32_t ile = FDCAN_ILE_EINT0 | FDCAN_ILE_EINT1;

  modifyreg32(priv->base + STM32_FDCAN_ILE_OFFSET, 0, ile);

  /* Enable both lines at the NVIC level */

  up_enable_irq(priv->config->mb_irq[0]);
  up_enable_irq(priv->config->mb_irq[1]);
}

/****************************************************************************
 * Function: fdcan_disable_interrupts
 *
 * Description:
 *   Disable all interrupts used by this driver
 *
 * Input Parameters:
 *   priv - Pointer to the private FDCAN driver state structure
 *
 * Assumptions:
 *   The peripheral is in Configuration Changes Enabled (CCE) mode
 *
 ****************************************************************************/

static void fdcan_disable_interrupts(struct fdcan_driver_s *priv)
{
  /* Disable both lines at the NVIC level */

  up_disable_irq(priv->config->mb_irq[0]);
  up_disable_irq(priv->config->mb_irq[1]);

  /* Disable both interrupt lines at the device level */

  const uint32_t ile = FDCAN_ILE_EINT0 | FDCAN_ILE_EINT1;

  modifyreg32(priv->base + STM32_FDCAN_ILE_OFFSET, ile, 0);
}

/****************************************************************************
 * Function: fdcan_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the CAN interface when a socket is opened
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The device is initialized and waiting to be brought online
 ****************************************************************************/

static int fdcan_ifup(struct net_driver_s *dev)
{
  struct fdcan_driver_s *priv =
    (struct fdcan_driver_s *)dev->d_private;

  /* Wake up the device and perform all initialization */

  irqstate_t flags = enter_critical_section();

  fdcan_initialize(priv);

  fdcan_setinit(priv->base, 1);
  fdcan_setconfig(priv->base, 1);

  /* Enable interrupts (at both device and NVIC level) */

  fdcan_enable_interrupts(priv);

  /* Leave init mode */

  fdcan_setinit(priv->base, 0);

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
  fdcan_dumpregs(priv);
#endif

  leave_critical_section(flags);

  /* Not standby = active */

  stm32_gpiowrite(priv->pwr_pin, !priv->pwr_pin_standby);

  priv->bifup = true;

  return OK;
}

/****************************************************************************
 * Function: fdcan_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int fdcan_ifdown(struct net_driver_s *dev)
{
  struct fdcan_driver_s *priv =
    (struct fdcan_driver_s *)dev->d_private;

  fdcan_reset(priv);

  priv->bifup = false;
  stm32_gpiowrite(priv->pwr_pin, priv->pwr_pin_standby);

  return OK;
}

/****************************************************************************
 * Function: fdcan_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void fdcan_txavail_work(void *arg)
{
  struct fdcan_driver_s *priv = (struct fdcan_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      if (!fdcan_txringfull(priv))
        {
          /* There is space for another transfer.  Poll the network for
           * new XMIT data.
           */

          devif_poll(&priv->dev, fdcan_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: fdcan_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus to perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int fdcan_txavail(struct net_driver_s *dev)
{
  struct fdcan_driver_s *priv =
    (struct fdcan_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      fdcan_txavail_work(priv);
    }

  return OK;
}

/****************************************************************************
 * Function: fdcan_netdev_ioctl
 *
 * Description:
 *   PHY ioctl command handler
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   cmd  - ioctl command
 *   arg  - Argument accompanying the command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int fdcan_netdev_ioctl(struct net_driver_s *dev, int cmd,
                       unsigned long arg)
{
  struct fdcan_driver_s *priv = dev->d_private;

  int ret;

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_CAN_IOCTL
      case SIOCGCANBITRATE: /* Get bitrate from a CAN controller */
        {
          struct can_ioctl_data_s *req =
              (struct can_ioctl_data_s *)((uintptr_t)arg);

          req->arbi_bitrate = priv->arbi_timing.bitrate;
#ifdef CONFIG_NET_CAN_CANFD
          req->data_bitrate = priv->data_timing.bitrate;
#else
          req->data_bitrate = 0;
#endif
          ret = OK;
        }
        break;

      case SIOCSCANBITRATE: /* Set bitrate of a CAN controller */
        {
          struct can_ioctl_data_s *req =
              (struct can_ioctl_data_s *)((uintptr_t)arg);

          /* Apply the new timings (interface is guaranteed to be down) */

          priv->arbi_timing.bitrate = req->arbi_bitrate;
#ifdef CONFIG_NET_CAN_CANFD
          priv->data_timing.bitrate = req->data_bitrate;
#endif
          ret = OK;
        }
        break;

      case SIOCSCANLISTENONLY:
        {
          /* set listenonly mode */

          priv->listen_only_mode = *(uint32_t *)arg;

          if (priv->bifup)
            {
              /* Reset CAN controller and start in the correct mode */

              ret = fdcan_initialize(priv);

              if (ret == OK)
                {
                  ret = fdcan_ifup(dev);
                }
            }
          else
            {
              ret = OK;
            }
        }
        break;

      case SIOCSCANABORTTX:
        {
          priv->abort_tx_on_error = *(uint32_t *)(uintptr_t)arg;
          ret = OK;
        }
        break;
#endif  /* CONFIG_NETDEV_CAN_IOCTL */

      default:
        ret = -ENOTSUP;
        break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Function: fdcan_initialize
 *
 * Description:
 *   Initialize FDCAN device
 *
 * Input Parameters:
 *   priv - Pointer to the private FDCAN driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int fdcan_initialize(struct fdcan_driver_s *priv)
{
  uint32_t regval;
  volatile uint32_t      *     msgram = NULL;
  uint32_t                     cntr   = 0;

  irqstate_t flags = enter_critical_section();

  /* Reset the rx_pool ring buffer indices */

  priv->rx_pool_head  = 0;
  priv->rx_pool_tail  = 0;
  priv->rx_pool_count = 0;

  /* Reset the peripheral clock bus (only do this once) */

  if (!g_apb1h_init)
    {
      fdcan_apb1hreset();
      g_apb1h_init = true;
    }

  /* Exit Power-down / Sleep mode */

  fdcan_setenable(priv->base, 1);

  /* Enter Initialization mode */

  fdcan_setinit(priv->base, 1);

  /* Enter Configuration Changes Enabled mode */

  fdcan_setconfig(priv->base, 1);

  /* Disable interrupts while we configure the hardware */

  putreg32(0, priv->base + STM32_FDCAN_IE_OFFSET);

  /* Compute CAN bit timings for this bitrate */

  /* Nominal / arbitration phase bitrate */

  if (fdcan_bittiming(&priv->arbi_timing) != OK)
    {
      fdcan_setinit(priv->base, 0);
      leave_critical_section(flags);
      return -EIO;
    }

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
  const fdcan_bitseg *tim = &priv->arbi_timing;

  ninfo("[fdcan][arbi] Timings: presc=%u sjw=%u bs1=%u bs2=%u\r\n",
        tim->prescaler, tim->sjw, tim->bs1, tim->bs2);
#endif

  /* Set bit timings and prescalers (Nominal bitrate) */

  regval = ((priv->arbi_timing.sjw << FDCAN_NBTP_NSJW_SHIFT)  |
            (priv->arbi_timing.bs1 << FDCAN_NBTP_NTSEG1_SHIFT) |
            (priv->arbi_timing.bs2 << FDCAN_NBTP_NTSEG2_SHIFT)  |
            (priv->arbi_timing.prescaler << FDCAN_NBTP_NBRP_SHIFT));
  putreg32(regval, priv->base + STM32_FDCAN_NBTP_OFFSET);

#ifdef CONFIG_NET_CAN_CANFD
  /* CAN-FD Data phase bitrate */

  if (fdcan_bittiming(&priv->data_timing) != OK)
    {
      fdcan_setinit(priv->base, 0);
      leave_critical_section(flags);
      return -EIO;
    }

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
  const fdcan_bitseg *tim = &priv->data_timing;

  ninfo("[fdcan][data] Timings: presc=%u sjw=%u bs1=%u bs2=%u\r\n",
        tim->prescaler, tim->sjw, tim->bs1, tim->bs2);
#endif

  /* Set bit timings and prescalers (Data bitrate) */

  regval = ((priv->data_timing.sjw << FDCAN_DBTP_DSJW_SHIFT)  |
            (priv->data_timing.bs1 << FDCAN_DBTP_DTSEG1_SHIFT) |
            (priv->data_timing.bs2 << FDCAN_DBTP_DTSEG2_SHIFT)  |
            (priv->data_timing.prescaler << FDCAN_DBTP_DBRP_SHIFT));
#endif /* CONFIG_NET_CAN_CANFD */

  /* Be sure to fill data-phase register even if we're not using CAN FD */

  putreg32(regval, priv->base + STM32_FDCAN_DBTP_OFFSET);

  /* Operation Configuration */

#ifdef CONFIG_STM32_FDCAN_LOOPBACK
  /* Enable External Loopback Mode (Rx pin disconnected) (RM0433 pg 2494) */

  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_TEST);
  modifyreg32(priv->base + STM32_FDCAN_TEST_OFFSET, 0, FDCAN_TEST_LBCK);
#endif

#if defined(CONFIG_STM32_FDCAN_LOOPBACK_INTERNAL)
  /* Enable Bus Monitoring / Restricted Op Mode (RM0433 pg 2492, 2494) */

  priv->listen_only_mode = true;
#endif

  if (priv->listen_only_mode)
    {
      modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_MON);
    }
  else
    {
      modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_MON, 0);
    }

  regval = FDCANCLK_PDIV;
  putreg32(regval, priv->base + STM32_FDCAN_CKDIV_OFFSET);

#ifdef CONFIG_NET_CAN_CANFD
  /* Enable CAN-FD frames, including bitrate switching if needed */

  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_FDOE);
  if (priv->arbi_timing.bitrate != priv->data_timing.bitrate)
    {
      modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_BRSE);
    }
#else
  /* Disable CAN-FD communications ("classic" CAN only) */

  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_FDOE, 0);
#endif

#if 0
  /* Disable Automatic Retransmission of frames upon error
   * NOTE: This will even disable automatic retry due to lost arbitration!!
   */

  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_DAR);
#endif

  /* Configure Interrupts */

  /* Clear all interrupt flags
   * Note: A flag is cleared by writing a 1 to the corresponding bit position
   */

  putreg32(FDCAN_IR_MASK, priv->base + STM32_FDCAN_IR_OFFSET);

  /* Enable relevant interrupts */

  regval = FDCAN_INT_TC     /* Transmit Complete */
         | FDCAN_INT_RF0N   /* Rx FIFO 0 new message */
         | FDCAN_INT_RF0F   /* Rx FIFO 0 FIFO full */
         | FDCAN_INT_RF1N   /* Rx FIFO 1 new message */
         | FDCAN_INT_RF1F;  /* Rx FIFO 1 FIFO full */
  putreg32(regval, priv->base + STM32_FDCAN_IE_OFFSET);

#ifdef CONFIG_NET_CAN_ERRORS
  fdcan_errint(priv, true);
#endif

  /* Keep Rx interrupts on Line 0; move Tx to Line 1
   * TC (Tx Complete) interrupt on line 1
   */

  regval = getreg32(priv->base + STM32_FDCAN_ILS_OFFSET);
  regval |= FDCAN_ILS_SMG;
  putreg32(FDCAN_ILS_SMG, priv->base + STM32_FDCAN_ILS_OFFSET);

  /* Enable Tx buffer transmission interrupts
   * Note: Still need fdcan_enable_interrupts() to set ILE (IR line enable)
   */

  putreg32(FDCAN_TXBTIE_TIE_MASK, priv->base + STM32_FDCAN_TXBTIE_OFFSET);

  /* RAM CONFIGURATION */

  const struct fdcan_config_s * config = priv->config;

  regval = FDCAN_RXGFC_LSS(config->nstdfilters);
  regval |= FDCAN_RXGFC_LSE(config->nextfilters);
  putreg32(regval, priv->base + STM32_FDCAN_RXGFC_OFFSET);

  msgram = config->msgram.stdfilters;
  cntr   = config->nstdfilters;
  while (cntr > 0)
    {
      *msgram++ = STDFILTER_S0_SFEC_DISABLE;
      cntr--;
    }

  msgram = config->msgram.stdfilters;

  /* Filter 0 — High priority → FIFO0 */

  msgram[0] =
      (STDFILTER_S0_SFEC_FIFO0) |      /* Store in FIFO0 */
      (STDFILTER_S0_SFT_RANGE)   |     /* Range filter */
      (0x000 << STDFILTER_S0_SFID1_SHIFT)       |
      (0x1ff << STDFILTER_S0_SFID2_SHIFT);

  /* Filter 1 — Low priority → FIFO1 */

  msgram[1] =
      (STDFILTER_S0_SFEC_FIFO1) |
      (STDFILTER_S0_SFT_RANGE)  |
      (0x200 << STDFILTER_S0_SFID1_SHIFT)      |
      (0x2ff << STDFILTER_S0_SFID2_SHIFT);

  /* Disable all extended filters */

  msgram = config->msgram.extfilters;
  cntr = config->nextfilters;
  while (cntr > 0)
    {
      *msgram = EXTFILTER_F0_EFEC_DISABLE;
      msgram = msgram + 2;
      cntr--;
    }

  msgram = config->msgram.extfilters;

  /* Filter 0 — High priority → FIFO0 */

  msgram[0] =
      (EXTFILTER_F0_EFEC_FIFO0) |
      (0x00000000 << 0);

  msgram[1] =
      (EXTFILTER_F1_EFT_RANGE) |
      (0x0fffffff << 0);

  /* Filter 1 — Low priority → FIFO1 */

  msgram[2] =
      (EXTFILTER_F0_EFEC_FIFO1) |
      (0x10000000 << 0);

  msgram[3] =
      (EXTFILTER_F1_EFT_RANGE) |
      (0x1fffffff << 0);

  /* END RAM CONFIGURATION */

  regval = getreg32(priv->base + STM32_FDCAN_TXBC_OFFSET);
  regval &= ~FDCAN_TXBC_TFQM;  /* Use FIFO */
  putreg32(regval, priv->base + STM32_FDCAN_TXBC_OFFSET);

  /* Default filter configuration - Accept all messages into Rx FIFO1 */

  regval = getreg32(priv->base + STM32_FDCAN_RXGFC_OFFSET);
  regval &= ~FDCAN_RXGFC_ANFS_MASK;     /* Clear */
  regval &= ~FDCAN_RXGFC_ANFE_MASK;     /* Clear */
  regval |= FDCAN_RXGFC_ANFS_RX_FIFO1;  /* Accept non-matching stdid frames into FIFO1 */
  regval |= FDCAN_RXGFC_ANFE_RX_FIFO1;  /* Accept non-matching extid frames into FIFO1 */
  putreg32(regval, priv->base + STM32_FDCAN_RXGFC_OFFSET);

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
  fdcan_dumpregs(priv);
#endif

  /* Exit Initialization mode */

  fdcan_setinit(priv->base, 0);

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
  fdcan_dumpregs(priv);
#endif

  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Function: fdcan_reset
 *
 * Description:
 *   Put the device in the non-operational, reset state
 *
 * Input Parameters:
 *   priv - Pointer to the private FDCAN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The device has previously been initialized, including message RAM
 ****************************************************************************/

static void fdcan_reset(struct fdcan_driver_s *priv)
{
  /* Request Init Mode */

  irqstate_t flags = enter_critical_section();

  fdcan_setenable(priv->base, 1);
  fdcan_setinit(priv->base, 1);

  /* Enable Configuration Change Mode */

  fdcan_setconfig(priv->base, 1);

  /* Disable interrupts and clear all interrupt flags */

  fdcan_disable_interrupts(priv);

  putreg32(FDCAN_IR_MASK, priv->base + STM32_FDCAN_IR_OFFSET);

  /* Clear all message RAM mailboxes if initialized */

#ifdef CONFIG_NET_CAN_CANFD
  const uint8_t n_data_words = 16;
#else
  const uint8_t n_data_words = 2;
#endif

  for (uint32_t f = 0; f < 2; f++)
    {
      for (uint32_t i = 0; i < NUM_RX_FIFO0; i++)
        {
          struct rx_fifo_s *mb;

          if (f == 0)
            {
              mb = (struct rx_fifo_s *)(priv->config->msgram.rxfifo0 +
                                        priv->config->rxfifo0esize * i);
            }
          else
            {
              mb = (struct rx_fifo_s *)(priv->config->msgram.rxfifo1 +
                                        priv->config->rxfifo1esize * i);
            }

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
          ninfo("[fdcan] MB RX%i %i %p\r\n", f, i, mb);
#endif
          mb->header.w1 = 0x0;
          mb->header.w0 = 0x0;
          for (uint8_t j = 0; j < n_data_words; j++)
            {
              mb->data[j].word = 0x0;
            }
        }
    }

  for (uint32_t i = 0; i < NUM_TX_FIFO; i++)
    {
      struct tx_fifo_s *mb =
        (struct tx_fifo_s *)(priv->config->msgram.txfifoq +
                             priv->config->txbufferesize * i);

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
      ninfo("[fdcan] MB TX %i %p\r\n", i, mb);
#endif
      mb->header.w1 = 0x0;
      mb->header.w0 = 0x0;
      for (uint8_t j = 0; j < n_data_words; j++)
        {
          mb->data[j].word = 0x0;
        }
    }

  /* Power off the device -- See RM0433 pg 2493 */

  fdcan_setinit(priv->base, 0);
  fdcan_setenable(priv->base, 0);

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: stm32_fdcanh5sockinitialize
 *
 * Description:
 *   Initialize the selected CAN peripheral and network (socket) interface
 *
 * Input Parameters:
 *   intf - In the case where there are multiple interfaces, this value
 *          identifies which interface is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int stm32_fdcanh5sockinitialize(int intf, char *ifname)
{
  struct fdcan_driver_s *priv;

  switch (intf)
    {
#ifdef CONFIG_STM32_FDCAN1
      case 1:
        priv             = &g_fdcan0;
        memset(priv, 0, sizeof(struct fdcan_driver_s));
        priv->base       = STM32_FDCAN1_BASE;
        priv->iface_idx  = 0;
        priv->config     = &stm32_fdcan0_config;
        priv->pwr_pin = STM32H5_FDCAN1_PWR;
        priv->pwr_pin_standby = STM32H5_FDCAN1_PWR_STANDBY;

        /* Default bitrate configuration */

#  ifdef CONFIG_NET_CAN_CANFD
        priv->arbi_timing.bitrate = CONFIG_FDCAN1_ARBI_BITRATE;
        priv->data_timing.bitrate = CONFIG_FDCAN1_DATA_BITRATE;
#  else
        priv->arbi_timing.bitrate = CONFIG_STM32_FDCAN1_BITRATE;
#  endif
        break;
#endif /* CONFIG_STM32_FDCAN1 */

#ifdef CONFIG_STM32_FDCAN2
      case 2:
        priv             = &g_fdcan1;
        memset(priv, 0, sizeof(struct fdcan_driver_s));
        priv->base       = STM32_FDCAN2_BASE;
        priv->iface_idx  = 1;
        priv->config     = &stm32_fdcan1_config;
        priv->pwr_pin = STM32H5_FDCAN2_PWR;
        priv->pwr_pin_standby = STM32H5_FDCAN2_PWR_STANDBY;

        /* Default bitrate configuration */

#  ifdef CONFIG_NET_CAN_CANFD
        priv->arbi_timing.bitrate = CONFIG_FDCAN2_ARBI_BITRATE;
        priv->data_timing.bitrate = CONFIG_FDCAN2_DATA_BITRATE;
#  else
        priv->arbi_timing.bitrate = CONFIG_STM32_FDCAN2_BITRATE;
#  endif
        break;
#endif /* CONFIG_STM32_FDCAN2 */

#ifdef CONFIG_STM32_FDCAN3
      case 2:
        priv             = &g_fdcan2
        memset(priv, 0, sizeof(struct fdcan_driver_s));
        priv->base       = STM32_FDCAN3_BASE;
        priv->iface_idx  = 2;
        priv->config     = &stm32_fdcan2_config;
        priv->pwr_pin = STM32H5_FDCAN3_PWR;
        priv->pwr_pin_standby = STM32H5_FDCAN3_PWR_STANDBY;

        /* Default bitrate configuration */

#  ifdef CONFIG_NET_CAN_CANFD
        priv->arbi_timing.bitrate = CONFIG_FDCAN3_ARBI_BITRATE;
        priv->data_timing.bitrate = CONFIG_FDCAN3_DATA_BITRATE;
#  else
        priv->arbi_timing.bitrate = CONFIG_FDCAN3_BITRATE;
#  endif
        break;
#endif /* CONFIG_STM32_FDCAN3 */

      default:
        return -ENODEV;
    }

  if (fdcan_bittiming(&priv->arbi_timing) != OK)
    {
      printf("ERROR: Invalid CAN timings\n");
      return -1;
    }

#ifdef CONFIG_NET_CAN_CANFD
  if (fdcan_bittiming(&priv->data_timing) != OK)
    {
      printf("ERROR: Invalid CAN data phase timings\n");
      return -1;
    }
#endif

  /* Configure the pins we're using to interface to the controller */

  stm32_configgpio(priv->config->tx_pin);
  stm32_configgpio(priv->config->rx_pin);

  stm32_configgpio(priv->pwr_pin);
  stm32_gpiowrite(priv->pwr_pin, priv->pwr_pin_standby);

  /* Attach the fdcan interrupt handlers */

  if (irq_attach(priv->config->mb_irq[0], fdcan_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      printf("ERROR: Failed to attach CAN RX IRQ\n");
      return -EAGAIN;
    }

  if (irq_attach(priv->config->mb_irq[1], fdcan_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      printf("ERROR: Failed to attach CAN TX IRQ\n");
      return -EAGAIN;
    }

  /* Initialize the driver structure */

  strlcpy(priv->dev.d_ifname, ifname, IFNAMSIZ);
  priv->dev.d_ifup    = fdcan_ifup;     /* I/F up callback */
  priv->dev.d_ifdown  = fdcan_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = fdcan_txavail;  /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = fdcan_netdev_ioctl;    /* Support CAN ioctl() calls */
#endif
  priv->dev.d_private = (void *)priv;   /* Used to recover private state from dev */

  priv->dev.d_buf = priv->tx_pool;

  /* Put the interface in the down state (disable interrupts, power off) */

  fdcan_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_CAN);

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
  fdcan_dumpregs(priv);
#endif

  return OK;
}

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Initialize the CAN device interfaces.  If there is more than one device
 *   interface in the chip, then board-specific logic will have to provide
 *   this function to determine which, if any, CAN interfaces should be
 *   initialized.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
#ifdef CONFIG_STM32_FDCAN1
  stm32_fdcanh5sockinitialize(1, "can0");
#endif

#ifdef CONFIG_STM32_FDCAN2
  stm32_fdcanh5sockinitialize(2, "can1");
#endif

#ifdef CONFIG_STM32_FDCAN3
  stm32_fdcanh5sockinitialize(3, "can2");
#endif
}
#endif /* !defined(CONFIG_NETDEV_LATEINIT) */

#ifdef CONFIG_NET_CAN_ERRORS
/****************************************************************************
 * Name: fdcan_error_work
 ****************************************************************************/

static void fdcan_error_work(void *arg)
{
  struct fdcan_driver_s *priv    = (struct fdcan_driver_s *)arg;
  uint32_t              pending = 0;
  uint32_t              ir      = 0;
  uint32_t              psr     = 0;

  ir  = priv->saved_errflags; /* Error IR flags captured and cleared in ISR */
  psr = priv->saved_psr;      /* PSR captured at ISR time, before TXBCR write */

  pending = ir;

  /* Check for common errors */

  if ((pending & FDCAN_CMNERR_INTS) != 0)
    {
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

      /* Clear the error indications */

      putreg32(FDCAN_CMNERR_INTS, priv->base + STM32_FDCAN_IR_OFFSET);
    }

  /* Check for transmission errors */

  if ((pending & FDCAN_TXERR_INTS) != 0)
    {
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

      putreg32(FDCAN_TXERR_INTS, priv->base + STM32_FDCAN_IR_OFFSET);
    }

  /* Check for reception errors */

  if ((pending & FDCAN_RXERR_INTS) != 0)
    {
      /* To prevent Interrupt-Flooding the current active
       * RX error interrupts are disabled. After successfully
       * receiving at least one CAN packet all RX error interrupts
       * are turned back on.
       *
       * The Interrupt-Flooding can for example occur if the
       * configured CAN speed does not match the speed of the other
       * CAN nodes in the network.
       */

      /* Clear the error indications */

      putreg32(FDCAN_RXERR_INTS, priv->base + STM32_FDCAN_IR_OFFSET);
    }

  /* Report errors */

  net_lock();
  fdcan_error(priv, pending & FDCAN_ANYERR_INTS, psr);
  net_unlock();

  /* Re-enable ERROR interrupts */

  fdcan_errint(priv, true);
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
 *   psr        - psr register content
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void fdcan_error(struct fdcan_driver_s *priv, uint32_t status,
                        uint32_t psr)
{
  irqstate_t flags = enter_critical_section();
  struct can_frame *frame = (struct can_frame *)priv->rx_pool;
  uint32_t          errbits = 0;
  uint8_t           data[CAN_ERR_DLC];
  uint32_t regval;

  DEBUGASSERT(priv != NULL);

  /* Encode error bits */

  errbits = 0;
  memset(data, 0, sizeof(data));

  /* Always fill in "static" error conditions, but set the signaling bit
   * only if the condition has changed (see IRQ-Flags below)
   * They have to be filled in every time CAN_ERROR_CONTROLLER is set.
   */

  errbits |= CAN_ERR_CNT;
  regval = getreg32(priv->base + STM32_FDCAN_ECR_OFFSET);
  data[6] = (uint8_t)((regval & FDCAN_ECR_TEC_MASK) >> FDCAN_ECR_TEC_SHIFT);
  data[7] = (uint8_t)((regval & FDCAN_ECR_REC_MASK) >>
                      FDCAN_ECR_REC_SHIFT);

  if ((psr & FDCAN_PSR_EP) != 0)
    {
      if ((regval & FDCAN_ECR_RP) != 0)
        {
          data[1] |= (CAN_ERR_CRTL_RX_PASSIVE);
        }

      if (data[6] >= CAN_ERROR_PASSIVE_THRESHOLD)
        {
          data[1] |= (CAN_ERR_CRTL_TX_PASSIVE);
        }
    }

  if ((psr & FDCAN_PSR_EW) != 0)
    {
      if ((data[6] >= CAN_ERROR_WARNING_THRESHOLD))
        {
          data[1] |= CAN_ERR_CRTL_TX_WARNING;
        }

      if ((data[7] >= CAN_ERROR_WARNING_THRESHOLD))
        {
          data[1] |= CAN_ERR_CRTL_RX_WARNING;
        }
    }

  if ((status & (FDCAN_INT_EP | FDCAN_INT_EW)) != 0)
    {
      /* "Error Passive" or "Error Warning" status changed */

      errbits |= CAN_ERR_CRTL;
    }

  if ((status & FDCAN_INT_PEA) != 0)
    {
      /* Protocol Error in Arbitration Phase */

      if ((psr & FDCAN_PSR_ACT_MASK) == FDCAN_PSR_ACT_TRANS)
        {
          /* transmit error */

          data[2] |= CAN_ERR_PROT_TX;

          /* Used to communicate to the upper layer that a TX error has
           * occurred, which led to the TX frame being aborted
           */

          if (priv->abort_tx_on_error)
            {
              errbits |= CAN_ERR_TX_TIMEOUT;
            }
        }

      switch (psr & FDCAN_PSR_LEC_MASK)
        {
          case FDCAN_PSR_LEC(FDCAN_PSR_EC_STUFF_ERROR):

            /* Stuff Error */

            errbits |= CAN_ERR_PROT;
            data[2] |= CAN_ERR_PROT_STUFF;
            break;

          case FDCAN_PSR_LEC(FDCAN_PSR_EC_FORM_ERROR):

            /* Format Error */

            errbits |= CAN_ERR_PROT;
            data[2] |= CAN_ERR_PROT_FORM;
            break;

          case FDCAN_PSR_LEC(FDCAN_PSR_EC_ACK_ERROR):

            /* Acknowledge Error */

            errbits |= (CAN_ERR_PROT | CAN_ERR_ACK);
            break;

          case FDCAN_PSR_LEC(FDCAN_PSR_EC_BIT0_ERROR):

            /* Bit0 Error */

            errbits |= CAN_ERR_PROT;
            data[2] |= CAN_ERR_PROT_BIT0;
            break;

          case FDCAN_PSR_LEC(FDCAN_PSR_EC_BIT1_ERROR):

            /* Bit1 Error */

            errbits |= CAN_ERR_PROT;
            data[2] |= CAN_ERR_PROT_BIT1;
            break;

          case FDCAN_PSR_LEC(FDCAN_PSR_EC_CRC_ERROR):

            /* Receive CRC Error */

            errbits |= CAN_ERR_PROT;
            data[3] |= (CAN_ERR_PROT_LOC_CRC_SEQ | CAN_ERR_PROT_LOC_CRC_DEL);
            break;

          case FDCAN_PSR_LEC(FDCAN_PSR_EC_NO_CHANGE):

            /* No change (nothing has cleared the error) */

            errbits |= CAN_ERR_PROT;
            data[2] |= CAN_ERR_PROT_UNSPEC;
            break;

          default:

            /* no error  */

            break;
        }
    }

  if ((status & FDCAN_INT_PED) != 0)
    {
      /* Protocol Error in Data Phase */

      if ((psr & FDCAN_PSR_ACT_MASK) == FDCAN_PSR_ACT_TRANS)
        {
          /* transmit error */

          data[2] |= CAN_ERR_PROT_TX;

          /* Used to communicate to the upper layer that a TX error has
           * occurred, which led to the TX frame being aborted
           */

          if (priv->abort_tx_on_error)
            {
              errbits |= CAN_ERR_TX_TIMEOUT;
            }
        }

      switch (psr & FDCAN_PSR_DLEC_MASK)
        {
          case FDCAN_PSR_DLEC(FDCAN_PSR_EC_STUFF_ERROR):

            /* Stuff Error */

            errbits |= CAN_ERR_PROT;
            data[2] |= CAN_ERR_PROT_STUFF;
            break;

          case FDCAN_PSR_DLEC(FDCAN_PSR_EC_FORM_ERROR):

            /* Format Error */

            errbits |= CAN_ERR_PROT;
            data[2] |= CAN_ERR_PROT_FORM;
            break;

          case FDCAN_PSR_DLEC(FDCAN_PSR_EC_ACK_ERROR):

            /* Acknowledge Error */

            errbits |= (CAN_ERR_ACK | CAN_ERR_PROT);
            break;

          case FDCAN_PSR_DLEC(FDCAN_PSR_EC_BIT0_ERROR):

            /* Bit0 Error */

            errbits |= CAN_ERR_PROT;
            data[2] |= CAN_ERR_PROT_BIT0;
            break;

          case FDCAN_PSR_DLEC(FDCAN_PSR_EC_BIT1_ERROR):

            /* Bit1 Error */

            errbits |= CAN_ERR_PROT;
            data[2] |= CAN_ERR_PROT_BIT1;
            break;

          case FDCAN_PSR_DLEC(FDCAN_PSR_EC_CRC_ERROR):

            /* Receive CRC Error */

            errbits |= CAN_ERR_PROT;
            data[3] |= (CAN_ERR_PROT_LOC_CRC_SEQ | CAN_ERR_PROT_LOC_CRC_DEL);
            break;

          case FDCAN_PSR_DLEC(FDCAN_PSR_EC_NO_CHANGE):

            /* No change (nothing has cleared the error) */

            errbits |= CAN_ERR_PROT;
            data[2] |= CAN_ERR_PROT_UNSPEC;
            break;

          default:

            /* no error */

            break;
        }
    }

  if ((status & FDCAN_INT_BO) != 0)
    {
      /* Bus_Off Status changed */

      if ((psr & FDCAN_PSR_BO) != 0)
        {
          errbits |= CAN_ERR_BUSOFF;
        }
      else
        {
          errbits |= CAN_ERR_RESTARTED;
        }
    }

  if ((status & (FDCAN_INT_RF0L | FDCAN_INT_RF1L)) != 0)
    {
      /* Receive FIFO 0/1 Message Lost
       * Receive FIFO 1 Message Lost
       */

      errbits |= CAN_ERR_CRTL;
      data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
    }

  if ((status & FDCAN_INT_TEFL) != 0)
    {
      /* Tx Event FIFO Element Lost */

      errbits |= CAN_ERR_CRTL;
      data[1] |= CAN_ERR_CRTL_TX_OVERFLOW;
    }

  if ((status & FDCAN_INT_TOO) != 0)
    {
      /* Timeout Occurred */

      errbits |= CAN_ERR_TX_TIMEOUT;
    }

  if ((status & (FDCAN_INT_MRAF | FDCAN_INT_ELO)) != 0)
    {
      /* Message RAM Access Failure
       * Error Logging Overflow
       */

      errbits |= CAN_ERR_CRTL;
      data[1] |= CAN_ERR_CRTL_UNSPEC;
    }

  errbits |= CAN_ERR_FLAG;

  if (errbits != 0)
    {
      nerr("ERROR: errbits = %lx" PRIx16 "\n", errbits);

      /* Copy frame */

      frame->can_id  = errbits;
      frame->can_dlc = CAN_ERR_DLC;

      memcpy(frame->data, data, CAN_ERR_DLC);

      /* Copy the buffer pointer to priv->dev..  Set amount of data
       * in priv->dev.d_len
       */

      priv->dev.d_len = sizeof(struct can_frame);
      priv->dev.d_buf = (uint8_t *)frame;

      /* Send to socket interface */

      NETDEV_ERRORS(&priv->dev);

      can_input(&priv->dev);

      /* Point the packet buffer back to the next Tx buffer that will be
       * used during the next write.  If the write queue is full, then
       * this will point at an active buffer, which must not be written
       * to.  This is OK because devif_poll won't be called unless the
       * queue is not full.
       */

      priv->dev.d_buf = (uint8_t *)priv->tx_pool;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: fdcan_errint
 *
 * Description:
 *   Call to enable or disable CAN error interrupts.
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *   enable - enable or disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void fdcan_errint(struct fdcan_driver_s *priv, bool enable)
{
  const struct fdcan_config_s *config = NULL;
  uint32_t regval = 0;

  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  /* Enable/disable the transmit mailbox interrupt */

  regval  = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);
  if (enable)
    {
      regval |= FDCAN_ANYERR_INTS;
    }
  else
    {
      regval &= ~FDCAN_ANYERR_INTS;
    }

  putreg32(regval, priv->base + STM32_FDCAN_IE_OFFSET);
}
#endif /* CONFIG_NET_CAN_ERRORS */


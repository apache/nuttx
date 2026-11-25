/****************************************************************************
 * arch/arm/src/common/stm32/stm32_fdcan_mcan_v2_sock.c
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
#include <nuttx/debug.h>
#include <errno.h>

#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/clock.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/net/can.h>

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
#  include <sys/time.h>
#endif

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FDCAN_CMNERR_INTS   (FDCAN_IE_MRAFE | FDCAN_IE_TOOE | FDCAN_IE_EPE | \
                            FDCAN_IE_BOE | FDCAN_IE_WDIE | FDCAN_IE_PEAE | \
                            FDCAN_IE_PEDE)

#define FDCAN_RXERR_INTS    (FDCAN_IE_RF0LE | FDCAN_IE_RF1LE)

#define FDCAN_TXERR_INTS    (FDCAN_IE_TEFLE | FDCAN_IE_PEAE | FDCAN_IE_PEDE)

/* Common-, TX- and RX-Error-Mask */

#define FDCAN_ANYERR_INTS (FDCAN_CMNERR_INTS | FDCAN_RXERR_INTS | FDCAN_TXERR_INTS)

#define CAN_ERROR_PASSIVE_THRESHOLD 128

#define CAN_ERROR_WARNING_THRESHOLD 96

#define WORD_LENGTH         4U

#define CANRAM_WORDS        2560U  /* Total words in Message RAM (RM0433) */

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

/* Define number of Rx / Tx elements in message RAM; note that elements are
 * given sizes in number of words (4-byte chunks)
 *
 * Up to 64 Rx elements and 32 Tx elements may be configured per interface
 *
 * Note there are a total of 2560 words available shared between all FDCAN
 * interfaces for Rx, Tx, and filter storage
 */

#ifdef CONFIG_NET_CAN_CANFD
#  define FIFO_ELEMENT_SIZE 18 /* size (in Words) of a FIFO element in message RAM (CANFD_MTU / 4) */
#  define NUM_RX_FIFO0      14 /* 14 elements max for RX FIFO0 */
#  define NUM_RX_FIFO1      0  /* No elements for RX FIFO1 */
#  define NUM_TX_FIFO       7  /* 7 elements max for TX FIFO */
#else
#  define FIFO_ELEMENT_SIZE 4  /* size (in Words) of a FIFO element in message RAM (CAN_MTU / 4) */
#  define NUM_RX_FIFO0      64 /* 64 elements max for RX FIFO0 */
#  define NUM_RX_FIFO1      0  /* No elements for RX FIFO1 */
#  define NUM_TX_FIFO       32 /* 32 elements max for TX FIFO */
#endif

/* Message RAM of one interface: 128 standard filters (1 word), 128
 * extended filters (2 words), RX FIFO0 and the TX FIFO.
 */

#ifdef CONFIG_STM32_FDCAN3
#  define FDCAN_NIFACES     3
#else
#  define FDCAN_NIFACES     2
#endif

#define FDCAN_RAM_WORDS     (CANRAM_WORDS / FDCAN_NIFACES)

#if 128 + 2 * 128 + (NUM_RX_FIFO0 + NUM_TX_FIFO) * FIFO_ELEMENT_SIZE > \
    FDCAN_RAM_WORDS
#  error "FDCAN message RAM layout does not fit"
#endif

/* CAN Clock Configuration **************************************************/

#ifndef STM32_FDCANCLK
#  define STM32_FDCANCLK    STM32_HSE_FREQUENCY
#endif
#define CLK_FREQ            STM32_FDCANCLK

#define PRESDIV_MAX         256

/* Interrupts ***************************************************************/

#define FDCAN_IR_MASK 0x3fcfffff /* Mask of all non-reserved bits in FDCAN_IR */

/* RX interrupts, masked until receive() empties the FIFOs */

#define FDCAN_RX_INTS (FDCAN_IE_RF0NE | FDCAN_IE_RF0FE)

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
  struct timespec deadline;
  struct wdog_s txtimeout;
#endif
  netpkt_t *pkt;
  int8_t pending;
};

/* FDCAN Device hardware configuration **************************************/

struct fdcan_config_s
{
  uint32_t tx_pin;           /* GPIO configuration for TX */
  uint32_t rx_pin;           /* GPIO configuration for RX */
  uint32_t mb_irq[2];        /* FDCAN Interrupt 0, 1 (Rx, Tx) */
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
  .tx_pin      = GPIO_CAN1_TX,
  .rx_pin      = GPIO_CAN1_RX,
  .mb_irq      =
  {
    STM32_IRQ_FDCAN1_0,
    STM32_IRQ_FDCAN1_1,
  },
};
#endif

#ifdef CONFIG_STM32_FDCAN2
static const struct fdcan_config_s stm32_fdcan1_config =
{
  .tx_pin      = GPIO_CAN2_TX,
  .rx_pin      = GPIO_CAN2_RX,
  .mb_irq      =
  {
    STM32_IRQ_FDCAN2_0,
    STM32_IRQ_FDCAN2_1,
  },
};
#endif

#ifdef CONFIG_STM32_FDCAN3
static const struct fdcan_config_s stm32_fdcan2_config =
{
  .tx_pin      = GPIO_CAN3_TX,
  .rx_pin      = GPIO_CAN3_RX,
  .mb_irq      =
  {
    STM32_IRQ_FDCAN3_0,
    STM32_IRQ_FDCAN3_1,
  },
};
#endif

/* The fdcan_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct fdcan_driver_s
{
  struct netdev_lowerhalf_s dev;        /* Must be first */
  const struct fdcan_config_s *config;  /* Pin config */
  uint8_t iface_idx;                    /* FDCAN interface index (0 to 2) */
  uint32_t base;                        /* FDCAN base address */

  struct fdcan_bitseg arbi_timing;      /* Timing for arbitration phase */
#ifdef CONFIG_NET_CAN_CANFD
  struct fdcan_bitseg data_timing;      /* Timing for data phase */
#endif

  struct fdcan_message_ram message_ram; /* Start addresses for each region of Message RAM */
  struct rx_fifo_s *rx;                 /* Pointer to Rx FIFO0 in Message RAM */
  struct tx_fifo_s *tx;                 /* Pointer to Tx mailboxes in Message RAM */

#ifdef TX_TIMEOUT_WQ
  struct work_s txcwork;                /* TX deadline expiry work */
#endif
#ifdef CONFIG_NET_CAN_ERRORS
  uint32_t errstatus;                   /* Error interrupts for receive() */
#endif

  struct txmbstats txmb[NUM_TX_FIFO];   /* Track deadline and status of every Tx entry */
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

static int  fdcan_transmit(struct netdev_lowerhalf_s *dev, netpkt_t *pkt);
static void fdcan_reclaim(struct netdev_lowerhalf_s *dev);
#ifdef CONFIG_NET_NETPOLL
static void fdcan_txpoll(struct netdev_lowerhalf_s *dev);
#endif

/* Helper functions */

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
static void fdcan_dumpregs(struct fdcan_driver_s *priv);
#endif

int32_t fdcan_bittiming(struct fdcan_bitseg *timing);

static void fdcan_apb1hreset(void);
static void fdcan_setinit(uint32_t base, uint32_t init);
static void fdcan_setenable(uint32_t base, uint32_t enable);
static void fdcan_setconfig(uint32_t base, uint32_t config_enable);
static bool fdcan_waitccr_change(uint32_t base,
                                 uint32_t mask,
                                 uint32_t target_state);

static void fdcan_enable_interrupts(struct fdcan_driver_s *priv);
static void fdcan_disable_interrupts(struct fdcan_driver_s *priv);

/* Interrupt handling */

static netpkt_t *fdcan_receive(struct netdev_lowerhalf_s *dev);
static void fdcan_txdone(struct fdcan_driver_s *priv);

static int  fdcan_interrupt(int irq, void *context,
                            void *arg);

static void fdcan_check_errors(struct fdcan_driver_s *priv);

/* Watchdog timer expirations */

#ifdef TX_TIMEOUT_WQ
static void fdcan_txtimeout_work(void *arg);
static void fdcan_txtimeout_expiry(wdparm_t arg);
#endif

/* NuttX networking stack callback functions */

static int fdcan_ifup(struct netdev_lowerhalf_s *dev);
static int fdcan_ifdown(struct netdev_lowerhalf_s *dev);

#ifdef CONFIG_NETDEV_IOCTL
static int  fdcan_netdev_ioctl(struct netdev_lowerhalf_s *dev, int cmd,
                               unsigned long arg);
#endif

/* Initialization and Reset */

static int  fdcan_initialize(struct fdcan_driver_s *priv);
static void fdcan_reset(struct fdcan_driver_s *priv);

#ifdef CONFIG_NET_CAN_ERRORS

/* CAN errors interrupt handling */

static netpkt_t *fdcan_error_handle(struct fdcan_driver_s *priv,
                                    uint32_t pending);
static netpkt_t *fdcan_error(struct fdcan_driver_s *priv, uint32_t status,
                             uint32_t psr);
static void fdcan_errint(struct fdcan_driver_s *priv, bool enable);
#endif

static const struct netdev_ops_s g_fdcan_ops =
{
  .ifup     = fdcan_ifup,
  .ifdown   = fdcan_ifdown,
  .transmit = fdcan_transmit,
  .receive  = fdcan_receive,
#ifdef CONFIG_NETDEV_IOCTL
  .ioctl    = fdcan_netdev_ioctl,
#endif
#ifdef CONFIG_NET_NETPOLL
  .txpoll   = fdcan_txpoll,
#endif
};

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
  ninfo("-------------- FDCAN Reg Dump ----------------\n");
  ninfo("CAN%d Base: 0x%lx\n", priv->iface_idx, priv->base);

  uint32_t regval;

  regval = getreg32(priv->base + STM32_FDCAN_CCCR_OFFSET);
  ninfo("CCCR = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_ECR_OFFSET);
  ninfo("ECR  = 0x%lx\n", regval);

  regval = getreg32(priv->base + STM32_FDCAN_NBTP_OFFSET);
  ninfo("NBTP = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_DBTP_OFFSET);
  ninfo("DBTP = 0x%lx\n", regval);

  regval = getreg32(priv->base + STM32_FDCAN_TXBC_OFFSET);
  ninfo("TXBC = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET);
  ninfo("TXFQS = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_RXF0C_OFFSET);
  ninfo("RXF0C = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_RXF0S_OFFSET);
  ninfo("RXF0S = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_RXF1C_OFFSET);
  ninfo("RXF1C = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_RXF1S_OFFSET);
  ninfo("RXF1S = 0x%lx\n", regval);

  regval = getreg32(priv->base + STM32_FDCAN_TXESC_OFFSET);
  ninfo("TXESC = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_RXESC_OFFSET);
  ninfo("RXESC = 0x%lx\n", regval);

  regval = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);
  ninfo("IE   = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_ILE_OFFSET);
  ninfo("ILE = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_ILS_OFFSET);
  ninfo("ILS = 0x%lx\n", regval);

  /* Print out some possibly interesting unhandled interrupts */

  regval = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);
  ninfo("IR   = 0x%lx\n", regval);

  if (regval & FDCAN_IR_PEA || regval & FDCAN_IR_PED)
    {
      /* Protocol error -- check protocol status register for details */

      regval = getreg32(priv->base + STM32_FDCAN_PSR_OFFSET);
      ninfo("--PSR.LEC = %" PRId32 "\n", regval & FDCAN_PSR_LEC_MASK);
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

  const uint32_t prescaler_bs = CLK_FREQ / target_bitrate;

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

  if (target_bitrate != (CLK_FREQ / (prescaler * (1 + bs1 + bs2))) || !valid)
    {
      nerr("Target bitrate invalid - solution does not match.");
      return 3; /* Solution not found */
    }

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
  ninfo("[fdcan] CLK_FREQ %lu, target_bitrate %lu, prescaler %lu, bs1 %d"
        ", bs2 %d\n", CLK_FREQ, target_bitrate, prescaler_bs, bs1 - 1,
        bs2 - 1);
#endif

  timing->bs1 = (uint8_t)(bs1 - 1);
  timing->bs2 = (uint8_t)(bs2 - 1);
  timing->prescaler = (uint16_t)(prescaler - 1);
  timing->sjw = 0; /* Which means one */

  return 0;
}

/****************************************************************************
 * Function: fdcan_deadline_get
 *
 * Description:
 *   Get the TX deadline of a packet: the struct timeval after the frame
 *   (CAN_RAW_TX_DEADLINE), else CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE
 *   microseconds from now, or none if that is 0.
 *
 * Input Parameters:
 *   dev      - Reference to the lower half driver state structure
 *   pkt      - Packet holding one frame
 *   len      - Packet length in bytes
 *   deadline - Returns the deadline in CLOCK_MONOTONIC time, 0 for none
 *
 * Returned Value:
 *   Packet length without the struct timeval
 *
 ****************************************************************************/

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
static unsigned int fdcan_deadline_get(struct netdev_lowerhalf_s *dev,
                                       netpkt_t *pkt, unsigned int len,
                                       struct timespec *deadline)
{
  struct timeval tv;
  struct timespec delay;

  if (len == sizeof(struct can_frame) + sizeof(tv) ||
      len == sizeof(struct canfd_frame) + sizeof(tv))
    {
      len -= sizeof(tv);
      netpkt_copyout(dev, (uint8_t *)&tv, pkt, sizeof(tv), len);

      deadline->tv_sec  = tv.tv_sec;
      deadline->tv_nsec = tv.tv_usec * NSEC_PER_USEC;
      return len;
    }

  if (CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE > 0)
    {
      nxclock_gettime(CLOCK_MONOTONIC, deadline);
      clock_usec2time(&delay, CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE);
      clock_timespec_add(deadline, &delay, deadline);
    }
  else
    {
      deadline->tv_sec  = 0;
      deadline->tv_nsec = 0;
    }

  return len;
}
#endif

/****************************************************************************
 * Function: fdcan_transmit
 *
 * Description:
 *   Copy a frame into the next TX FIFO element and request it.
 *
 * Input Parameters:
 *   dev - Reference to the lower half driver state structure
 *   pkt - The packet to send
 *
 * Returned Value:
 *   OK on success; -EBUSY if the TX FIFO is full; a negated errno on
 *   failure
 *
 ****************************************************************************/

static int fdcan_transmit(struct netdev_lowerhalf_s *dev, netpkt_t *pkt)
{
  struct fdcan_driver_s *priv = (struct fdcan_driver_s *)dev;
  unsigned int len = netpkt_getdatalen(dev, pkt);
  uint8_t *buf = netpkt_getdata(dev, pkt);
  union tx_fifo_header_u header;
  struct tx_fifo_s *mb;
  irqstate_t flags;
  uint32_t regval;
  uint8_t mbi;
#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  struct timespec deadline;
  clock_t timeout = 0;

  len = fdcan_deadline_get(dev, pkt, len, &deadline);
  if (deadline.tv_sec != 0 || deadline.tv_nsec != 0)
    {
      clock_abstime2ticks(CLOCK_MONOTONIC, &deadline, &timeout);

      /* The deadline has passed: drop the frame unsent */

      if (timeout == 0)
        {
          NETDEV_TXTIMEOUTS(&dev->netdev);
          netpkt_free(dev, pkt, NETPKT_TX);
          return OK;
        }
    }
#endif

  /* The socket checks the frame size only, check the data length */

#ifdef CONFIG_NET_CAN_CANFD
  if (len != sizeof(struct can_frame))
    {
      if (((struct canfd_frame *)buf)->len > CANFD_MAX_DLEN)
        {
          return -EINVAL;
        }
    }
  else
#endif
  if (((struct can_frame *)buf)->can_dlc > CAN_MAX_DLEN)
    {
      return -EINVAL;
    }

  /* The TX interrupt also frees the finished packets */

  flags = enter_critical_section();
  fdcan_reclaim(dev);

  regval = getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET);
  if ((regval & FDCAN_TXFQS_TFQF) == FDCAN_TXFQS_TFQF)
    {
      leave_critical_section(flags);
      return -EBUSY;
    }

  mbi = (regval & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_SHIFT;
  if (mbi >= NUM_TX_FIFO || priv->txmb[mbi].pkt != NULL)
    {
      leave_critical_section(flags);
      return -EBUSY;
    }

  mb = &priv->tx[mbi];

  if (len == sizeof(struct can_frame))
    {
      struct can_frame *frame = (struct can_frame *)buf;

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
      struct canfd_frame *frame = (struct canfd_frame *)buf;

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
      header.dlc = g_len_to_can_dlc[frame->len];
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

  priv->txmb[mbi].pkt = pkt;
  priv->txmb[mbi].pending = TX_BUSY;
#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  priv->txmb[mbi].deadline = deadline;
#endif

  /* GO - Submit the transmission request for this element */

  putreg32(1 << mbi, priv->base + STM32_FDCAN_TXBAR_OFFSET);

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
 * Function: fdcan_reclaim
 *
 * Description:
 *   Free the packets of the finished or cancelled TX elements.  Called with
 *   interrupts disabled: from transmit, the TX interrupt, the TX deadline
 *   work and fdcan_txpoll().
 *
 ****************************************************************************/

static void fdcan_reclaim(struct netdev_lowerhalf_s *dev)
{
  struct fdcan_driver_s *priv = (struct fdcan_driver_s *)dev;
  uint32_t pending = getreg32(priv->base + STM32_FDCAN_TXBRP_OFFSET);
  uint32_t done = getreg32(priv->base + STM32_FDCAN_TXBTO_OFFSET);
  uint8_t i;

  for (i = 0; i < NUM_TX_FIFO; i++)
    {
      struct txmbstats *txi = &priv->txmb[i];

      if (txi->pkt == NULL || (pending & (1 << i)) != 0)
        {
          continue;
        }

      /* A timed out element is counted in TXTIMEOUTS already */

      if (txi->pending == TX_BUSY && (done & (1 << i)) == 0)
        {
          NETDEV_TXERRORS(&dev->netdev);
        }

#ifdef TX_TIMEOUT_WQ
      wd_cancel(&txi->txtimeout);
#endif
      netpkt_free(dev, txi->pkt, NETPKT_TX);
      txi->pkt = NULL;
      txi->pending = TX_FREE;
    }
}

/****************************************************************************
 * Function: fdcan_txpoll
 *
 * Description:
 *   NuttX callback: free the packets of the finished TX elements for
 *   netpoll, which may run with or without interrupts disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_NETPOLL
static void fdcan_txpoll(struct netdev_lowerhalf_s *dev)
{
  irqstate_t flags = enter_critical_section();

  fdcan_reclaim(dev);
  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Function: fdcan_rxframe
 *
 * Description:
 *   Copy one RX FIFO0 element into a new packet.
 *
 ****************************************************************************/

static netpkt_t *fdcan_rxframe(struct fdcan_driver_s *priv,
                               struct rx_fifo_s *rf)
{
  struct netdev_lowerhalf_s *dev = &priv->dev;
  netpkt_t *pkt = netpkt_alloc(dev, NETPKT_RX);

  if (pkt == NULL)
    {
      return NULL;
    }

#ifdef CONFIG_NET_CAN_CANFD
  if (rf->header.fdf)
    {
      /* CAN FD frame */

      struct canfd_frame *frame =
        (struct canfd_frame *)netpkt_getdata(dev, pkt);

      memset(frame, 0, sizeof(*frame));

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

      frame->len = g_can_dlc_to_len[rf->header.dlc];

      if (rf->header.brs)
        {
          frame->flags |= CANFD_BRS;
        }

      if (rf->header.id.esi)
        {
          frame->flags |= CANFD_ESI;
        }

      uint32_t *frame_data_word = (uint32_t *)&frame->data[0];

      for (int i = 0; i < (frame->len + 4 - 1) / 4; i++)
        {
          frame_data_word[i] = rf->data[i].word;
        }

      netpkt_setdatalen(dev, pkt, sizeof(struct canfd_frame));
    }
  else
#endif
    {
      /* CAN 2.0 Frame */

      struct can_frame *frame =
        (struct can_frame *)netpkt_getdata(dev, pkt);

      memset(frame, 0, sizeof(*frame));

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

      netpkt_setdatalen(dev, pkt, sizeof(struct can_frame));
    }

  return pkt;
}

/****************************************************************************
 * Function: fdcan_receive
 *
 * Description:
 *   NuttX callback: return one received frame or error frame.  When the
 *   FIFO is empty, enable the RX interrupts again and return NULL.
 *
 * Input Parameters:
 *   dev - Reference to the lower half driver state structure
 *
 * Returned Value:
 *   The received packet, or NULL
 *
 ****************************************************************************/

static netpkt_t *fdcan_receive(struct netdev_lowerhalf_s *dev)
{
  struct fdcan_driver_s *priv = (struct fdcan_driver_s *)dev;
  netpkt_t *pkt = NULL;
  irqstate_t flags;
  uint32_t regval;

#ifdef CONFIG_NET_CAN_ERRORS
  flags = enter_critical_section();
  regval = priv->errstatus;
  priv->errstatus = 0;
  leave_critical_section(flags);

  if (regval != 0)
    {
      pkt = fdcan_error_handle(priv, regval);
      if (pkt != NULL)
        {
          return pkt;
        }
    }
#endif

  /* Only RX FIFO0 is allocated */

  flags = enter_critical_section();

  regval = getreg32(priv->base + STM32_FDCAN_RXF0S_OFFSET);
  if ((regval & FDCAN_RXF0S_RF0L) != 0)
    {
      NETDEV_RXERRORS(&dev->netdev);
#ifndef CONFIG_NET_CAN_ERRORS
      /* Without error frames nothing else clears the flag */

      putreg32(FDCAN_IR_RF0L, priv->base + STM32_FDCAN_IR_OFFSET);
#endif
    }

  while ((regval & FDCAN_RXF0S_F0FL) > 0)
    {
      const uint8_t index = (regval & FDCAN_RXF0S_F0GI) >>
                             FDCAN_RXF0S_F0GI_SHIFT;

      pkt = fdcan_rxframe(priv, &priv->rx[index]);

      /* Acknowledge receipt of this FIFO element */

      putreg32(index, priv->base + STM32_FDCAN_RXF0A_OFFSET);

      if (pkt != NULL)
        {
#ifdef CONFIG_NET_CAN_ERRORS
          /* A frame was received: report protocol errors again */

          modifyreg32(priv->base + STM32_FDCAN_IE_OFFSET, 0,
                      FDCAN_IE_PEAE | FDCAN_IE_PEDE);
#endif
          leave_critical_section(flags);
          return pkt;
        }

      /* No free packet: drop the frame and keep draining, so that the RX
       * interrupts are enabled again when the FIFO is empty.
       */

      NETDEV_RXDROPPED(&dev->netdev);
      regval = getreg32(priv->base + STM32_FDCAN_RXF0S_OFFSET);
    }

  /* Check for errors and abort-transmission requests */

  fdcan_check_errors(priv);

  /* The FIFO is empty: enable the RX interrupts again */

  regval = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);
  regval |= FDCAN_RX_INTS;
#ifdef CONFIG_NET_CAN_ERRORS
  regval |= FDCAN_RXERR_INTS;
#endif
  putreg32(regval, priv->base + STM32_FDCAN_IE_OFFSET);

  leave_critical_section(flags);
  return NULL;
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

  if (ir & (FDCAN_IR_TC | FDCAN_IR_TCF))
    {
      putreg32(ir & (FDCAN_IR_TC | FDCAN_IR_TCF),
               priv->base + STM32_FDCAN_IR_OFFSET);
    }
  else
    {
      nerr("Unexpected FCAN interrupt on line 1\n");
      return;
    }

  fdcan_reclaim(&priv->dev);

  /* Check for errors and abort-transmission requests */

  fdcan_check_errors(priv);

#ifdef CONFIG_NET_CAN_ERRORS
  /* Turning back on PEA and PED error interrupts */

  modifyreg32(priv->base + STM32_FDCAN_IE_OFFSET, 0,
              FDCAN_IE_PEAE | FDCAN_IE_PEDE);
#endif

  netdev_lower_txdone(&priv->dev);
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
 *   arg     - Pointer to the private FDCAN driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int fdcan_interrupt(int irq, void *context,
                           void *arg)
{
  struct fdcan_driver_s *priv    = (struct fdcan_driver_s *)arg;
  bool                   rxready = false;
  uint32_t               pending;

  DEBUGASSERT(priv != NULL);

  pending = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);

#ifdef CONFIG_NET_CAN_ERRORS
  /* Latch the errors for receive(), which enables them again */

  if ((pending & FDCAN_ANYERR_INTS &
       getreg32(priv->base + STM32_FDCAN_IE_OFFSET)) != 0)
    {
      fdcan_errint(priv, false);
      priv->errstatus |= pending & FDCAN_ANYERR_INTS;
      rxready = true;
    }
#endif

  if (irq == priv->config->mb_irq[0])
    {
      if ((pending & FDCAN_RX_INTS) != 0)
        {
          /* receive() enables them again when the FIFO is empty */

          modifyreg32(priv->base + STM32_FDCAN_IE_OFFSET, FDCAN_RX_INTS, 0);
          putreg32(pending & FDCAN_RX_INTS,
                   priv->base + STM32_FDCAN_IR_OFFSET);
          rxready = true;
        }
    }
  else if (irq == priv->config->mb_irq[1])
    {
      fdcan_txdone(priv);
    }
  else
    {
      nerr("Unexpected IRQ [%d]\n", irq);
      return -EINVAL;
    }

  if (rxready)
    {
      netdev_lower_rxready(&priv->dev);
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
                  & FDCAN_ECR_CEL;
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
          /* Request to Cancel Tx item; reclaim frees its packet */

          putreg32(1 << i, priv->base + STM32_FDCAN_TXBCR_OFFSET);
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
  irqstate_t flags = enter_critical_section();
  clock_t ticks;

  /* The watchdog timed out, yet we still check mailboxes in case the
   * transmit function transmitted a new frame
   */

  for (int mbi = 0; mbi < NUM_TX_FIFO; mbi++)
    {
      struct txmbstats *txi = &priv->txmb[mbi];

      if (txi->pending != TX_BUSY ||
          (txi->deadline.tv_sec == 0 && txi->deadline.tv_nsec == 0))
        {
          continue;
        }

      clock_abstime2ticks(CLOCK_MONOTONIC, &txi->deadline, &ticks);
      if (ticks == 0)
        {
          NETDEV_TXTIMEOUTS(&priv->dev.netdev);
          txi->pending = TX_ABORT;
        }
    }

  /* Cancel the aborted elements, then free what is finished */

  fdcan_check_errors(priv);
  fdcan_reclaim(&priv->dev);
  leave_critical_section(flags);

  netdev_lower_txdone(&priv->dev);
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
#endif

/****************************************************************************
 * Function: fdcan_apb1hreset
 *
 * Description:
 *   Reset the periheral bus clock used by FDCAN
 *   Note that this will reset all configuration of all FDCAN peripherals
 *
 ****************************************************************************/

static void fdcan_apb1hreset(void)
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

static int fdcan_ifup(struct netdev_lowerhalf_s *dev)
{
  struct fdcan_driver_s *priv = (struct fdcan_driver_s *)dev;

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

  netdev_lower_carrier_on(dev);

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

static int fdcan_ifdown(struct netdev_lowerhalf_s *dev)
{
  struct fdcan_driver_s *priv = (struct fdcan_driver_s *)dev;

  netdev_lower_carrier_off(dev);

  fdcan_reset(priv);

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
static int fdcan_netdev_ioctl(struct netdev_lowerhalf_s *dev, int cmd,
                              unsigned long arg)
{
  struct fdcan_driver_s *priv = (struct fdcan_driver_s *)dev;

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
#endif

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

  irqstate_t flags = enter_critical_section();

  /* Reset the peripheral clock bus (only do this once) */

  if (!g_apb1h_init)
    {
      /* Clear Message RAM (shared between FDCAN1/2/3) */

      memset((void *)STM32_CANRAM_BASE, 0,
             CANRAM_WORDS * WORD_LENGTH);

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
  const struct fdcan_bitseg *tim = &priv->arbi_timing;

  ninfo("[fdcan][arbi] Timings: presc=%u sjw=%u bs1=%u bs2=%u\r\n",
        tim->prescaler, tim->sjw, tim->bs1, tim->bs2);
#endif

  /* Set bit timings and prescalers (Nominal bitrate) */

  regval = ((priv->arbi_timing.sjw << FDCAN_NBTP_NSJW_SHIFT)  |
            (priv->arbi_timing.bs1 << FDCAN_NBTP_NTSEG1_SHIFT) |
            (priv->arbi_timing.bs2 << FDCAN_NBTP_TSEG2_SHIFT)  |
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
  tim = &priv->data_timing;
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

#ifdef CONFIG_STM32_FDCAN_LOOPBACK_INTERNAL
  /* Enable Bus Monitoring / Restricted Op Mode (RM0433 pg 2492, 2494) */

  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_MON);
#endif

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

  regval = FDCAN_IE_TCE     /* Transmit Complete */
         | FDCAN_IE_TCFE    /* Transmission Cancellation Finished */
         | FDCAN_IE_RF0NE   /* Rx FIFO 0 new message */
         | FDCAN_IE_RF0FE;  /* Rx FIFO 0 FIFO full */
  putreg32(regval, priv->base + STM32_FDCAN_IE_OFFSET);

#ifdef CONFIG_NET_CAN_ERRORS
  fdcan_errint(priv, true);
#endif

  /* Keep Rx interrupts on Line 0; move Tx to Line 1
   * TC (Tx Complete) interrupt on line 1
   */

  regval = getreg32(priv->base + STM32_FDCAN_ILS_OFFSET);
  regval |= FDCAN_ILS_TCL | FDCAN_ILS_TCFL;
  putreg32(regval, priv->base + STM32_FDCAN_ILS_OFFSET);

  /* Enable Tx buffer transmission interrupts
   * Note: Still need fdcan_enable_interrupts() to set ILE (IR line enable)
   */

  putreg32(FDCAN_TXBTIE_TIE, priv->base + STM32_FDCAN_TXBTIE_OFFSET);
  putreg32(FDCAN_TXBCIE_CF, priv->base + STM32_FDCAN_TXBCIE_OFFSET);

  /* Configure Message RAM
   *
   * The available 2560 words (10 kiB) of RAM are shared between both FDCAN
   * interfaces. It is up to us to ensure each interface has its own non-
   * overlapping region of RAM assigned to it by properly assigning the start
   * and end addresses for all regions of RAM.
   *
   * Each enabled interface index gets an equal part of the RAM.
   *
   * Rx buffers are only used in conjunction with acceptance filters; we
   * don't have any specific need for this, so we will only use Rx FIFOs.
   *
   * Each FIFO can hold up to 64 elements, where each element (for a classic
   * CAN 2.0B frame) is up to 4 words long (8 bytes data + header bits)
   *
   * Let's make use of the full 64 FIFO elements for FIFO0.  We have no need
   * to separate messages between FIFO0 and FIFO1, so ignore FIFO1 for
   * simplicity.
   *
   * Note that the start addresses given to FDCAN are in terms of _words_,
   * not  bytes, so when we go to read/write to/from the message RAM, there
   * will be a factor of 4 necessary in the address relative to the SA
   * register values.
   */

  /* Location of this interface's message RAM - address in CPU memory address
   * and relative address (in words) used for configuration
   */

  const uint32_t iface_ram_base = FDCAN_RAM_WORDS * priv->iface_idx;

  const uint32_t gl_ram_base = STM32_CANRAM_BASE;
  uint32_t ram_offset = iface_ram_base;

  /* Standard ID Filters: Allow space for 128 filters (128 words) */

  const uint8_t n_stdid = 128;

  priv->message_ram.filt_stdid_addr = gl_ram_base + ram_offset * WORD_LENGTH;

  regval  = (n_stdid << FDCAN_SIDFC_LSS_SHIFT) & FDCAN_SIDFC_LSS_MASK;
  regval |= ram_offset << FDCAN_SIDFC_FLSSA_SHIFT;
  putreg32(regval, priv->base + STM32_FDCAN_SIDFC_OFFSET);
  ram_offset += n_stdid;

  /* Extended ID Filters: Allow space for 64 filters (64 words)
   * The register definition of FDCAN_XIDFC:LSE - List size extended, in the
   * reference manual (RM0433 Rev 8) is defined as 8 bits and claims that a
   * value of 0-128 (Number of extended ID filter elements) are possible, but
   * in the text in section 56.4.22 and in STM32CubeH7's HAL it's only 64.
   *
   * HAL source:
   * https://github.com/STMicroelectronics/STM32CubeH7/blob/
   * 43c9e552ba1c038577c48723d96ca8c825b11987/Drivers/STM32H7xx_HAL_Driver/
   * Inc/stm32h7xx_hal_fdcan.h#L112-L113
   *
   * Discussion:
   * https://community.st.com/s/question/0D73W000001nzqFSAQ
   *
   * vmay23
   * Using 64  --> some messages are being received but some are not
   * Using 128 --> according to the tests, everything is working fine
   */

  const uint8_t n_extid = 128;

  priv->message_ram.filt_extid_addr = gl_ram_base + ram_offset * WORD_LENGTH;

  regval = (n_extid << FDCAN_XIDFC_LSE_SHIFT) & FDCAN_XIDFC_LSE_MASK;
  regval |= ram_offset << FDCAN_XIDFC_FLESA_SHIFT;
  putreg32(regval, priv->base + STM32_FDCAN_XIDFC_OFFSET);
  ram_offset += 2 * n_extid;  /* Extended filter elements are 2 words */

  /* Set size of each element in the Rx/Tx buffers and FIFOs */

#ifdef CONFIG_NET_CAN_CANFD
  /* Set full 64 byte space for every Rx/Tx FIFO element */

  modifyreg32(priv->base + STM32_FDCAN_RXESC_OFFSET, 0, FDCAN_RXESC_RBDS); /* Rx Buffer */
  modifyreg32(priv->base + STM32_FDCAN_RXESC_OFFSET, 0, FDCAN_RXESC_F0DS); /* Rx FIFO 0 */
  modifyreg32(priv->base + STM32_FDCAN_RXESC_OFFSET, 0, FDCAN_RXESC_F1DS); /* Rx FIFO 1 */
  modifyreg32(priv->base + STM32_FDCAN_TXESC_OFFSET, 0, FDCAN_TXESC_TBDS); /* Tx Buffer */
#else
  putreg32(0, priv->base + STM32_FDCAN_RXESC_OFFSET);  /* 8 byte space for every element (Rx buffer, FIFO1, FIFO0) */
  putreg32(0, priv->base + STM32_FDCAN_TXESC_OFFSET);  /* 8 byte space for every element (Tx buffer) */
#endif

  priv->message_ram.n_rxfifo0 = NUM_RX_FIFO0;
  priv->message_ram.n_rxfifo1 = NUM_RX_FIFO1;
  priv->message_ram.n_txfifo = NUM_TX_FIFO;

  /* Assign Rx Mailbox pointer in the driver structure */

  priv->message_ram.rxfifo0_addr = gl_ram_base + ram_offset * WORD_LENGTH;
  priv->rx = (struct rx_fifo_s *)(priv->message_ram.rxfifo0_addr);

  /* Set Rx FIFO0 size (64 elements max) */

  regval = (ram_offset << FDCAN_RXF0C_F0SA_SHIFT) & FDCAN_RXF0C_F0SA_MASK;
  regval |= (NUM_RX_FIFO0 << FDCAN_RXF0C_F0S_SHIFT) & FDCAN_RXF0C_F0S_MASK;

  putreg32(regval, priv->base + STM32_FDCAN_RXF0C_OFFSET);

  ram_offset += NUM_RX_FIFO0 * FIFO_ELEMENT_SIZE;

  /* Not using Rx FIFO1 */

  /* Assign Tx Mailbox pointer in the driver structure */

  priv->message_ram.txfifo_addr = gl_ram_base + ram_offset * WORD_LENGTH;
  priv->tx = (struct tx_fifo_s *)(priv->message_ram.txfifo_addr);

  /* Set Tx FIFO size (32 elements max) */

  regval = (NUM_TX_FIFO << FDCAN_TXBC_TFQS_SHIFT) & FDCAN_TXBC_TFQS_MASK;
  regval &= ~FDCAN_TXBC_TFQM;  /* Use FIFO */
  regval |= (ram_offset << FDCAN_TXBC_TBSA_SHIFT) & FDCAN_TXBC_TBSA_MASK;
  putreg32(regval, priv->base + STM32_FDCAN_TXBC_OFFSET);

  /* Default filter configuration - Accept all messages into Rx FIFO0 */

  regval = getreg32(priv->base + STM32_FDCAN_GFC_OFFSET);
  regval &= ~FDCAN_GFC_ANFS;  /* Accept non-matching stdid frames into FIFO0 */
  regval &= ~FDCAN_GFC_ANFE;  /* Accept non-matching extid frames into FIFO0 */
  putreg32(regval, priv->base + STM32_FDCAN_GFC_OFFSET);

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

  if (priv->rx)
    {
      for (uint32_t i = 0; i < NUM_RX_FIFO0; i++)
        {
    #ifdef CONFIG_STM32_FDCAN_REGDEBUG
          ninfo("[fdcan] MB RX %i %p\r\n", i, &priv->rx[i]);
    #endif
          priv->rx[i].header.w1 = 0x0;
          priv->rx[i].header.w0 = 0x0;
          for (uint8_t j = 0; j < n_data_words; j++)
            {
              priv->rx[i].data[j].word = 0x0;
            }
        }
    }

  if (priv->tx)
    {
      for (uint32_t i = 0; i < NUM_TX_FIFO; i++)
        {
    #ifdef CONFIG_STM32_FDCAN_REGDEBUG
          ninfo("[fdcan] MB TX %i %p\r\n", i, &priv->tx[i]);
    #endif
          priv->tx[i].header.w1 = 0x0;
          priv->tx[i].header.w0 = 0x0;
          for (uint8_t j = 0; j < n_data_words; j++)
            {
              priv->tx[i].data[j].word = 0x0;
            }
        }
    }

#ifdef TX_TIMEOUT_WQ
  work_cancel(CANWORK, &priv->txcwork);
#endif

  /* Free the packets of the TX elements */

  for (uint32_t i = 0; i < NUM_TX_FIFO; i++)
    {
#ifdef TX_TIMEOUT_WQ
      wd_cancel(&priv->txmb[i].txtimeout);
#endif
      if (priv->txmb[i].pkt != NULL)
        {
          netpkt_free(&priv->dev, priv->txmb[i].pkt, NETPKT_TX);
          priv->txmb[i].pkt = NULL;
        }

      priv->txmb[i].pending = TX_FREE;
    }

#ifdef CONFIG_NET_CAN_ERRORS
  priv->errstatus = 0;
#endif

  /* Power off the device -- See RM0433 pg 2493 */

  fdcan_setinit(priv->base, 0);
  fdcan_setenable(priv->base, 0);

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: stm32_fdcansockinitialize
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

int stm32_fdcansockinitialize(int intf)
{
  struct fdcan_driver_s *priv;

  switch (intf)
    {
#ifdef CONFIG_STM32_FDCAN1
      case 0:
        priv             = &g_fdcan0;
        memset(priv, 0, sizeof(struct fdcan_driver_s));
        priv->base       = STM32_FDCAN1_BASE;
        priv->iface_idx  = 0;
        priv->config     = &stm32_fdcan0_config;

        /* Default bitrate configuration */

#  ifdef CONFIG_NET_CAN_CANFD
        priv->arbi_timing.bitrate = CONFIG_FDCAN1_ARBI_BITRATE;
        priv->data_timing.bitrate = CONFIG_FDCAN1_DATA_BITRATE;
#  else
        priv->arbi_timing.bitrate = CONFIG_FDCAN1_BITRATE;
#  endif
        break;
#endif

#ifdef CONFIG_STM32_FDCAN2
      case 1:
        priv             = &g_fdcan1;
        memset(priv, 0, sizeof(struct fdcan_driver_s));
        priv->base       = STM32_FDCAN2_BASE;
        priv->iface_idx  = 1;
        priv->config     = &stm32_fdcan1_config;

        /* Default bitrate configuration */

#  ifdef CONFIG_NET_CAN_CANFD
        priv->arbi_timing.bitrate = CONFIG_FDCAN2_ARBI_BITRATE;
        priv->data_timing.bitrate = CONFIG_FDCAN2_DATA_BITRATE;
#  else
        priv->arbi_timing.bitrate = CONFIG_FDCAN2_BITRATE;
#  endif
        break;
#endif

#ifdef CONFIG_STM32_FDCAN3
      case 2:
        priv             = &g_fdcan2;
        memset(priv, 0, sizeof(struct fdcan_driver_s));
        priv->base       = STM32_FDCAN3_BASE;
        priv->iface_idx  = 2;
        priv->config     = &stm32_fdcan2_config;

        /* Default bitrate configuration */

#  ifdef CONFIG_NET_CAN_CANFD
        priv->arbi_timing.bitrate = CONFIG_FDCAN3_ARBI_BITRATE;
        priv->data_timing.bitrate = CONFIG_FDCAN3_DATA_BITRATE;
#  else
        priv->arbi_timing.bitrate = CONFIG_FDCAN3_BITRATE;
#  endif
        break;
#endif

      default:
        return -ENODEV;
    }

  if (fdcan_bittiming(&priv->arbi_timing) != OK)
    {
      nerr("ERROR: Invalid CAN timings\n");
      return -1;
    }

#ifdef CONFIG_NET_CAN_CANFD
  if (fdcan_bittiming(&priv->data_timing) != OK)
    {
      nerr("ERROR: Invalid CAN data phase timings\n");
      return -1;
    }
#endif

  /* Configure the pins we're using to interface to the controller */

  stm32_configgpio(priv->config->tx_pin);
  stm32_configgpio(priv->config->rx_pin);

  /* Attach the fdcan interrupt handlers */

  if (irq_attach(priv->config->mb_irq[0], fdcan_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach CAN RX IRQ\n");
      return -EAGAIN;
    }

  if (irq_attach(priv->config->mb_irq[1], fdcan_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach CAN TX IRQ\n");
      return -EAGAIN;
    }

  /* Initialize the driver structure */

  priv->dev.ops      = &g_fdcan_ops;
  priv->dev.rxtype   = NETDEV_RX_WORK;
  priv->dev.priority = CANWORK;

  priv->rx = NULL;
  priv->tx = NULL;

  /* Put the controller in the reset state (interrupts off, power off) */

  fdcan_reset(priv);

  /* Register the device with the OS so that socket IOCTLs can be performed */

#ifdef CONFIG_STM32_FDCAN_REGDEBUG
  fdcan_dumpregs(priv);
#endif

  return netdev_lower_register(&priv->dev, NET_LL_CAN);
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
#  ifdef CONFIG_STM32_ETHMAC
#    error "FDCAN SocketCAN and Ethernet need CONFIG_NETDEV_LATEINIT"
#  endif

void arm_netinitialize(void)
{
#ifdef CONFIG_STM32_FDCAN1
  stm32_fdcansockinitialize(0);
#endif

#ifdef CONFIG_STM32_FDCAN2
  stm32_fdcansockinitialize(1);
#endif

#ifdef CONFIG_STM32_FDCAN3
  stm32_fdcansockinitialize(2);
#endif
}
#endif

#ifdef CONFIG_NET_CAN_ERRORS
/****************************************************************************
 * Name: fdcan_error_handle
 *
 * Description:
 *   Clear the latched error interrupts, build the error frame and enable
 *   the error interrupts again.  Called from receive().
 *
 ****************************************************************************/

static netpkt_t *fdcan_error_handle(struct fdcan_driver_s *priv,
                                    uint32_t pending)
{
  irqstate_t            flags;
  netpkt_t             *pkt;
  uint32_t              ie;
  uint32_t              psr;

  flags = enter_critical_section();
  ie  = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);
  psr = getreg32(priv->base + STM32_FDCAN_PSR_OFFSET);

  ie |= FDCAN_ANYERR_INTS;

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

      if ((psr & FDCAN_PSR_LEC_MASK) != 0)
        {
          ie &= ~(FDCAN_IE_PEAE | FDCAN_IE_PEDE);
        }

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

      ie &= ~(pending & FDCAN_RXERR_INTS);

      /* Clear the error indications */

      putreg32(FDCAN_RXERR_INTS, priv->base + STM32_FDCAN_IR_OFFSET);
    }

  /* Re-enable ERROR interrupts */

  putreg32(ie, priv->base + STM32_FDCAN_IE_OFFSET);
  leave_critical_section(flags);

  /* Report errors */

  pkt = fdcan_error(priv, pending & FDCAN_ANYERR_INTS, psr);
  return pkt;
}

/****************************************************************************
 * Name: fdcan_error
 *
 * Description:
 *   Report a CAN error
 *
 * Input Parameters:
 *   priv       - Reference to the driver state structure
 *   status     - Interrupt status with error bits set
 *   psr        - psr register content
 *
 * Returned Value:
 *   The error frame packet, or NULL if no packet is free
 *
 ****************************************************************************/

static netpkt_t *fdcan_error(struct fdcan_driver_s *priv, uint32_t status,
                             uint32_t psr)
{
  struct can_frame *frame;
  netpkt_t         *pkt;
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
  data[7] = (uint8_t)((regval & FDCAN_ECR_TREC_MASK) >>
                      FDCAN_ECR_TREC_SHIFT);

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

  if ((status & (FDCAN_IR_EP | FDCAN_IR_EW)) != 0)
    {
      /* "Error Passive" or "Error Warning" status changed */

      errbits |= CAN_ERR_CRTL;
    }

  if ((status & FDCAN_IR_PEA) != 0)
    {
      /* Protocol Error in Arbitration Phase */

      if ((psr & FDCAN_PSR_ACT) == FDCAN_PSR_ACT)
        {
          /* transmit error */

          data[2] |= CAN_ERR_PROT_TX;
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

  if ((status & FDCAN_IR_PED) != 0)
    {
      /* Protocol Error in Data Phase */

      if ((psr & FDCAN_PSR_ACT) == FDCAN_PSR_ACT)
        {
          /* transmit error */

          data[2] |= CAN_ERR_PROT_TX;
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

  if ((status & FDCAN_IR_BO) != 0)
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

  if ((status & (FDCAN_IR_RF0L | FDCAN_IR_RF1L)) != 0)
    {
      /* Receive FIFO 0/1 Message Lost
       * Receive FIFO 1 Message Lost
       */

      errbits |= CAN_ERR_CRTL;
      data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
    }

  if ((status & FDCAN_IR_TEFL) != 0)
    {
      /* Tx Event FIFO Element Lost */

      errbits |= CAN_ERR_CRTL;
      data[1] |= CAN_ERR_CRTL_TX_OVERFLOW;
    }

  if ((status & FDCAN_IR_TOO) != 0)
    {
      /* Timeout Occurred */

      errbits |= CAN_ERR_TX_TIMEOUT;
    }

  if ((status & (FDCAN_IR_MRAF | FDCAN_IR_ELO)) != 0)
    {
      /* Message RAM Access Failure
       * Error Logging Overflow
       */

      errbits |= CAN_ERR_CRTL;
      data[1] |= CAN_ERR_CRTL_UNSPEC;
    }

  errbits |= CAN_ERR_FLAG;

  ninfo("errbits = %08" PRIx32 "\n", errbits);

  pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
  if (pkt == NULL)
    {
      return NULL;
    }

  netpkt_setdatalen(&priv->dev, pkt, sizeof(struct can_frame));
  frame = (struct can_frame *)netpkt_getdata(&priv->dev, pkt);
  memset(frame, 0, sizeof(*frame));

  frame->can_id  = errbits;
  frame->can_dlc = CAN_ERR_DLC;
  memcpy(frame->data, data, CAN_ERR_DLC);

  NETDEV_ERRORS(&priv->dev.netdev);
  return pkt;
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
#endif

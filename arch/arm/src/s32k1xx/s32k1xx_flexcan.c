/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_flexcan.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
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

#include "arm_internal.h"
#include "chip.h"
#include "s32k1xx_config.h"
#include "hardware/s32k1xx_flexcan.h"
#include "hardware/s32k1xx_pinmux.h"
#include "s32k1xx_periphclocks.h"
#include "s32k1xx_pin.h"
#include "s32k1xx_flexcan.h"

#include <arch/board/board.h>

#include <sys/time.h>

#ifdef CONFIG_S32K1XX_FLEXCAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#define CANWORK LPWORK

/* CONFIG_S32K1XX_FLEXCAN_NETHIFS determines the number of physical
 * interfaces that will be supported.
 */

#define MASKSTDID                   0x000007ff
#define MASKEXTID                   0x1fffffff
#define FLAGEFF                     (1 << 31) /* Extended frame format */
#define FLAGRTR                     (1 << 30) /* Remote transmission request */

#define RXMBCOUNT                   5
#define TXMBCOUNT                   2
#define TOTALMBCOUNT                RXMBCOUNT + TXMBCOUNT

#define IFLAG1_RX                   ((1 << RXMBCOUNT)-1)
#define IFLAG1_TX                   (((1 << TXMBCOUNT)-1) << RXMBCOUNT)

#define CAN_FIFO_NE                 (1 << 5)
#define CAN_FIFO_OV                 (1 << 6)
#define CAN_FIFO_WARN               (1 << 7)
#define CAN_EFF_FLAG                0x80000000 /* EFF/SFF is set in the MSB */

#define POOL_SIZE                   1

#define MSG_DATA                    sizeof(struct timeval)

/* CAN bit timing values  */
#define CLK_FREQ                    80000000
#define PRESDIV_MAX                 256

#define SEG_MAX                     8
#define SEG_MIN                     1
#define TSEG_MIN                    2
#define TSEG1_MAX                   17
#define TSEG2_MAX                   9
#define NUMTQ_MAX                   26

#define SEG_FD_MAX                  32
#define SEG_FD_MIN                  1
#define TSEG_FD_MIN                 2
#define TSEG1_FD_MAX                39
#define TSEG2_FD_MAX                9
#define NUMTQ_FD_MAX                49

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE

#  if !defined(CONFIG_SCHED_WORKQUEUE)
#    error Work queue support is required
#  endif

#define TX_TIMEOUT_WQ
#endif

/* Interrupt flags for RX fifo */
#define IFLAG1_RXFIFO               (CAN_FIFO_NE | CAN_FIFO_WARN | CAN_FIFO_OV)

static int peak_tx_mailbox_index_ = 0;

/****************************************************************************
 * Private Types
 ****************************************************************************/

union cs_e
{
  volatile uint32_t cs;
  struct
  {
    volatile uint32_t time_stamp : 16;
    volatile uint32_t dlc : 4;
    volatile uint32_t rtr : 1;
    volatile uint32_t ide : 1;
    volatile uint32_t srr : 1;
    volatile uint32_t res : 1;
    volatile uint32_t code : 4;
    volatile uint32_t res2 : 1;
    volatile uint32_t esi : 1;
    volatile uint32_t brs : 1;
    volatile uint32_t edl : 1;
  };
};

union id_e
{
  volatile uint32_t w;
  struct
  {
    volatile uint32_t ext : 29;
    volatile uint32_t resex : 3;
  };
  struct
  {
    volatile uint32_t res : 18;
    volatile uint32_t std : 11;
    volatile uint32_t resstd : 3;
  };
};

union data_e
{
  volatile uint32_t w00;
  struct
  {
    volatile uint32_t b03 : 8;
    volatile uint32_t b02 : 8;
    volatile uint32_t b01 : 8;
    volatile uint32_t b00 : 8;
  };
};

struct mb_s
{
  union cs_e cs;
  union id_e id;
#ifdef CONFIG_NET_CAN_CANFD
  union data_e data[16];
#else
  union data_e data[2];
#endif
};

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
#define TX_ABORT -1
#define TX_FREE 0
#define TX_BUSY 1

struct txmbstats
{
  struct timeval deadline;
  uint32_t pending; /* -1 = abort, 0 = free, 1 = busy  */
};
#endif

/* FlexCAN Device hardware configuration */

struct flexcan_config_s
{
  uint32_t tx_pin;           /* GPIO configuration for TX */
  uint32_t rx_pin;           /* GPIO configuration for RX */
  uint32_t enable_pin;       /* Optional enable pin */
  uint32_t enable_high;      /* Optional enable high/low */
  uint32_t bus_irq;          /* BUS IRQ */
  uint32_t error_irq;        /* ERROR IRQ */
  uint32_t lprx_irq;         /* LPRX IRQ */
  uint32_t mb_irq;           /* MB 0-15 IRQ */
};

struct flexcan_timeseg
{
  uint32_t bitrate;
  int32_t samplep;
  uint8_t propseg;
  uint8_t pseg1;
  uint8_t pseg2;
  uint8_t presdiv;
};

/* FlexCAN device structures */

#ifdef CONFIG_S32K1XX_FLEXCAN0
static const struct flexcan_config_s s32k1xx_flexcan0_config =
{
  .tx_pin      = PIN_CAN0_TX,
  .rx_pin      = PIN_CAN0_RX,
#ifdef PIN_CAN0_ENABLE
  .enable_pin  = PIN_CAN0_ENABLE,
  .enable_high = CAN0_ENABLE_OUT,
#else
  .enable_pin  = 0,
  .enable_high = 0,
#endif
  .bus_irq     = S32K1XX_IRQ_CAN0_BUS,
  .error_irq   = S32K1XX_IRQ_CAN0_ERROR,
  .lprx_irq    = S32K1XX_IRQ_CAN0_LPRX,
  .mb_irq      = S32K1XX_IRQ_CAN0_0_15,
};
#endif

#ifdef CONFIG_S32K1XX_FLEXCAN1
static const struct flexcan_config_s s32k1xx_flexcan1_config =
{
  .tx_pin      = PIN_CAN1_TX,
  .rx_pin      = PIN_CAN1_RX,
#ifdef PIN_CAN1_ENABLE
  .enable_pin  = PIN_CAN1_ENABLE,
  .enable_high = CAN1_ENABLE_OUT,
#else
  .enable_pin  = 0,
  .enable_high = 0,
#endif
  .bus_irq     = S32K1XX_IRQ_CAN1_BUS,
  .error_irq   = S32K1XX_IRQ_CAN1_ERROR,
  .lprx_irq    = 0,
  .mb_irq      = S32K1XX_IRQ_CAN1_0_15,
};
#endif

#ifdef CONFIG_S32K1XX_FLEXCAN2
static const struct flexcan_config_s s32k1xx_flexcan2_config =
{
  .tx_pin    = PIN_CAN2_TX,
  .rx_pin    = PIN_CAN2_RX,
#ifdef PIN_CAN2_ENABLE
  .enable_pin = PIN_CAN2_ENABLE,
  .rx_pin     = CAN2_ENABLE_HIGH,
#else
  .enable_pin = 0,
  .rx_pin     = 0,
#endif
  .bus_irq   = S32K1XX_IRQ_CAN2_BUS,
  .error_irq = S32K1XX_IRQ_CAN2_ERROR,
  .lprx_irq  = 0,
  .mb_irq    = S32K1XX_IRQ_CAN2_0_15,
};
#endif

/* The s32k1xx_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct s32k1xx_driver_s
{
  uint32_t base;                /* FLEXCAN base address */
  bool bifup;                   /* true:ifup false:ifdown */
#ifdef TX_TIMEOUT_WQ
  struct wdog_s txtimeout[TXMBCOUNT]; /* TX timeout timer */
#endif
  struct work_s irqwork;        /* For deferring interrupt work to the wq */
  struct work_s pollwork;       /* For deferring poll work to the work wq */
#ifdef CONFIG_NET_CAN_CANFD
  struct canfd_frame *txdesc;   /* A pointer to the list of TX descriptor */
  struct canfd_frame *rxdesc;   /* A pointer to the list of RX descriptors */
#else
  struct can_frame *txdesc;     /* A pointer to the list of TX descriptor */
  struct can_frame *rxdesc;     /* A pointer to the list of RX descriptors */
#endif

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;      /* Interface understood by the network */

  struct mb_s *rx;
  struct mb_s *tx;

  struct flexcan_timeseg arbi_timing; /* Timing for arbitration phase */
#ifdef CONFIG_NET_CAN_CANFD
  struct flexcan_timeseg data_timing; /* Timing for data phase */
#endif

  const struct flexcan_config_s *config;

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  struct txmbstats txmb[TXMBCOUNT];
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_FLEXCAN0
static struct s32k1xx_driver_s g_flexcan0;
#endif

#ifdef CONFIG_S32K1XX_FLEXCAN1
static struct s32k1xx_driver_s g_flexcan1;
#endif

#ifdef CONFIG_S32K1XX_FLEXCAN2
static struct s32k1xx_driver_s g_flexcan2;
#endif

#ifdef CONFIG_NET_CAN_CANFD
static uint8_t g_tx_pool[(sizeof(struct canfd_frame)+MSG_DATA)*POOL_SIZE];
static uint8_t g_rx_pool[(sizeof(struct canfd_frame)+MSG_DATA)*POOL_SIZE];
#else
static uint8_t g_tx_pool[sizeof(struct can_frame)*POOL_SIZE];
static uint8_t g_rx_pool[sizeof(struct can_frame)*POOL_SIZE];
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: arm_lsb
 *
 * Description:
 *   Calculate position of lsb that's equal to 1
 *
 * Input Parameters:
 *   value - The value to perform the operation on
 *
 * Returned Value:
 *   location of lsb which is equal to 1, returns 32 when value is 0
 *
 ****************************************************************************/

static inline uint32_t arm_lsb(unsigned int value)
{
  uint32_t ret;
  volatile uint32_t rvalue = value;
  __asm__ __volatile__ ("rbit %1,%0" : "=r" (rvalue) : "r" (rvalue));
  __asm__ __volatile__ ("clz %0, %1" : "=r"(ret) : "r"(rvalue));
  return ret;
}

/****************************************************************************
 * Name: s32k1xx_bitratetotimeseg
 *
 * Description:
 *   Convert bitrate to timeseg
 *
 * Input Parameters:
 *   timeseg - structure to store bit timing
 *   sp_tolerance - allowed difference in sample point from calculated
 *                  bit timings (recommended value: 1)
 *   can_fd - if set to calculate CAN FD bit timings, otherwise calculate
 *            classical can timings
 *
 * Returned Value:
 *   return 1 on success, return 0 on failure
 *
 ****************************************************************************/

uint32_t s32k1xx_bitratetotimeseg(struct flexcan_timeseg *timeseg,
                                                int32_t sp_tolerance,
                                                uint32_t can_fd)
{
  int32_t tmppresdiv;
  int32_t numtq;
  int32_t tmpsample;
  int32_t tseg1;
  int32_t tseg2;
  int32_t tmppseg1;
  int32_t tmppseg2;
  int32_t tmppropseg;

  const int32_t TSEG1MAX = (can_fd ? TSEG1_FD_MAX : TSEG1_MAX);
  const int32_t TSEG2MAX = (can_fd ? TSEG2_FD_MAX : TSEG2_MAX);
  const int32_t SEGMAX = (can_fd ? SEG_FD_MAX : SEG_MAX);
  const int32_t NUMTQMAX = (can_fd ? NUMTQ_FD_MAX : NUMTQ_MAX);

  for (tmppresdiv = 0; tmppresdiv < PRESDIV_MAX; tmppresdiv++)
    {
      numtq = (CLK_FREQ / ((tmppresdiv + 1) * timeseg->bitrate));

      if (numtq == 0)
        {
          continue;
        }

      /* The number of time quanta in 1 bit time must be
       * lower than the one supported
       */

      if ((CLK_FREQ / ((tmppresdiv + 1) * numtq) == timeseg->bitrate)
          && (numtq >= 8) && (numtq < NUMTQMAX))
        {
          /* Compute time segments based on the value of the sampling point */

          tseg1 = (numtq * timeseg->samplep / 100) - 1;
          tseg2 = numtq - 1 - tseg1;

          /* Adjust time segment 1 and time segment 2 */

          while (tseg1 >= TSEG1MAX || tseg2 < TSEG_MIN)
            {
              tseg2++;
              tseg1--;
            }

          tmppseg2 = tseg2 - 1;

          /* Start from pseg1 = pseg2 and adjust until propseg is valid */

          tmppseg1 = tmppseg2;
          tmppropseg = tseg1 - tmppseg1 - 2;

          while (tmppropseg <= 0)
            {
              tmppropseg++;
              tmppseg1--;
            }

          while (tmppropseg >= SEGMAX)
            {
              tmppropseg--;
              tmppseg1++;
            }

          if (((tseg1 >= TSEG1MAX) || (tseg2 >= TSEG2MAX) ||
              (tseg2 < TSEG_MIN) || (tseg1 < TSEG_MIN)) ||
              ((tmppropseg >= SEGMAX) || (tmppseg1 >= SEGMAX) ||
                  (tmppseg2 < SEG_MIN) || (tmppseg2 >= SEGMAX)))
            {
              continue;
            }

          tmpsample = ((tseg1 + 1) * 100) / numtq;

          if ((tmpsample - timeseg->samplep) <= sp_tolerance &&
              (timeseg->samplep - tmpsample) <= sp_tolerance)
            {
              if (can_fd == 1)
                {
                  timeseg->propseg = tmppropseg + 1;
                }
              else
                {
                  timeseg->propseg = tmppropseg;
                }
              timeseg->pseg1 = tmppseg1;
              timeseg->pseg2 = tmppseg2;
              timeseg->presdiv = tmppresdiv;
              timeseg->samplep = tmpsample;
              return 1;
            }
        }
    }

  return 0;
}

/* Common TX logic */

static bool s32k1xx_txringfull(struct s32k1xx_driver_s *priv);
static int  s32k1xx_transmit(struct s32k1xx_driver_s *priv);
static int  s32k1xx_txpoll(struct net_driver_s *dev);

/* Helper functions */

static void s32k1xx_setenable(uint32_t base, uint32_t enable);
static void s32k1xx_setfreeze(uint32_t base, uint32_t freeze);
static uint32_t s32k1xx_waitmcr_change(uint32_t base,
                                       uint32_t mask,
                                       uint32_t target_state);
static uint32_t s32k1xx_waitesr2_change(uint32_t base,
                                       uint32_t mask,
                                       uint32_t target_state);

/* Interrupt handling */

static void s32k1xx_receive(struct s32k1xx_driver_s *priv,
                            uint32_t flags);
static void s32k1xx_txdone_work(void *arg);
static void s32k1xx_txdone(struct s32k1xx_driver_s *priv);

static int  s32k1xx_flexcan_interrupt(int irq, void *context,
                                      void *arg);

/* Watchdog timer expirations */
#ifdef TX_TIMEOUT_WQ
static void s32k1xx_txtimeout_work(void *arg);
static void s32k1xx_txtimeout_expiry(wdparm_t arg);
#endif

/* NuttX callback functions */

static int  s32k1xx_ifup(struct net_driver_s *dev);
static int  s32k1xx_ifdown(struct net_driver_s *dev);

static void s32k1xx_txavail_work(void *arg);
static int  s32k1xx_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NETDEV_IOCTL
static int  s32k1xx_ioctl(struct net_driver_s *dev, int cmd,
                          unsigned long arg);
#endif

/* Initialization */

static int  s32k1xx_initialize(struct s32k1xx_driver_s *priv);
static void s32k1xx_reset(struct s32k1xx_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: s32k1xx_txringfull
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

static bool s32k1xx_txringfull(struct s32k1xx_driver_s *priv)
{
  uint32_t mbi = 0;

  while (mbi < TXMBCOUNT)
    {
      if (priv->tx[mbi].cs.code != CAN_TXMB_DATAORREMOTE)
        {
          return 0;
        }

      mbi++;
    }

  return 1;
}

/****************************************************************************
 * Function: s32k1xx_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
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

static int s32k1xx_transmit(struct s32k1xx_driver_s *priv)
{
  /* Attempt to write frame */

  uint32_t mbi = 0;
  uint32_t mb_bit;
  uint32_t regval;
#ifdef CONFIG_NET_CAN_CANFD
  uint32_t *frame_data_word;
  uint32_t i;
#endif
#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  int32_t timeout;
#endif

  if ((getreg32(priv->base + S32K1XX_CAN_ESR2_OFFSET) &
      (CAN_ESR2_IMB | CAN_ESR2_VPS)) ==
      (CAN_ESR2_IMB | CAN_ESR2_VPS))
    {
      mbi  = ((getreg32(priv->base + S32K1XX_CAN_ESR2_OFFSET) &
        CAN_ESR2_LPTM_MASK) >> CAN_ESR2_LPTM_SHIFT);
      mbi -= RXMBCOUNT;
    }

  mb_bit = 1 << (RXMBCOUNT + mbi);

  while (mbi < TXMBCOUNT)
    {
      if (priv->tx[mbi].cs.code != CAN_TXMB_DATAORREMOTE)
        {
          putreg32(mb_bit, priv->base + S32K1XX_CAN_IFLAG1_OFFSET);
          break;
        }

      mb_bit <<= 1;
      mbi++;
    }

  if (mbi == TXMBCOUNT)
    {
      nwarn("No TX MB available mbi %" PRIu32 "\n", mbi);
      NETDEV_TXERRORS(&priv->dev);
      return 0;       /* No transmission for you! */
    }

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  struct timespec ts;
  clock_systime_timespec(&ts);

  if (priv->dev.d_sndlen > priv->dev.d_len)
    {
      struct timeval *tv =
             (struct timeval *)(priv->dev.d_buf + priv->dev.d_len);
      priv->txmb[mbi].deadline = *tv;
      timeout  = (tv->tv_sec - ts.tv_sec)*CLK_TCK
                 + ((tv->tv_usec - ts.tv_nsec / 1000)*CLK_TCK) / 1000000;
      if (timeout < 0)
        {
          return 0;       /* No transmission for you! */
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
          timeout = -1;
        }
    }
#endif

  peak_tx_mailbox_index_ =
    (peak_tx_mailbox_index_ > mbi ? peak_tx_mailbox_index_ : mbi);

  union cs_e cs;
  cs.code = CAN_TXMB_DATAORREMOTE;
  struct mb_s *mb = &priv->tx[mbi];
  mb->cs.code = CAN_TXMB_INACTIVE;

  if (priv->dev.d_len <= sizeof(struct can_frame))
    {
      struct can_frame *frame = (struct can_frame *)priv->dev.d_buf;

      if (frame->can_id & CAN_EFF_FLAG)
        {
          cs.ide = 1;
          mb->id.ext = frame->can_id & MASKEXTID;
        }
      else
        {
          mb->id.std = frame->can_id & MASKSTDID;
        }

      cs.rtr = frame->can_id & FLAGRTR ? 1 : 0;
      cs.dlc = frame->can_dlc;

      mb->data[0].w00 = __builtin_bswap32(*(uint32_t *)&frame->data[0]);
      mb->data[1].w00 = __builtin_bswap32(*(uint32_t *)&frame->data[4]);
    }
#ifdef CONFIG_NET_CAN_CANFD
  else /* CAN FD frame */
    {
      struct canfd_frame *frame = (struct canfd_frame *)priv->dev.d_buf;

      cs.edl = 1; /* CAN FD Frame */

      if (frame->can_id & CAN_EFF_FLAG)
        {
          cs.ide = 1;
          mb->id.ext = frame->can_id & MASKEXTID;
        }
      else
        {
          mb->id.std = frame->can_id & MASKSTDID;
        }

      cs.rtr = frame->can_id & FLAGRTR ? 1 : 0;

      cs.dlc = len_to_can_dlc[frame->len];

      frame_data_word = (uint32_t *)&frame->data[0];

      for (i = 0; i < (frame->len + 4 - 1) / 4; i++)
        {
          mb->data[i].w00 = __builtin_bswap32(frame_data_word[i]);
        }
    }
#endif

  mb->cs = cs; /* Go. */

  regval = getreg32(priv->base + S32K1XX_CAN_IMASK1_OFFSET);
  regval |= mb_bit;
  putreg32(regval, priv->base + S32K1XX_CAN_IMASK1_OFFSET);

  /* Increment statistics */

  NETDEV_TXPACKETS(&priv->dev);

#ifdef TX_TIMEOUT_WQ
  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  if (timeout >= 0)
    {
      wd_start(&priv->txtimeout[mbi], timeout + 1,
               s32k1xx_txtimeout_expiry, (wdparm_t)priv);
    }
#endif

  return OK;
}

/****************************************************************************
 * Function: s32k1xx_txpoll
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

static int s32k1xx_txpoll(struct net_driver_s *dev)
{
  struct s32k1xx_driver_s *priv =
    (struct s32k1xx_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      if (!devif_loopback(&priv->dev))
        {
          s32k1xx_txdone(priv);

          /* Send the packet */

          s32k1xx_transmit(priv);

          /* Check if there is room in the device to hold another packet. If
           * not, return a non-zero value to terminate the poll.
           */

          if ((getreg32(priv->base + S32K1XX_CAN_ESR2_OFFSET) &
              (CAN_ESR2_IMB | CAN_ESR2_VPS)) ==
              (CAN_ESR2_IMB | CAN_ESR2_VPS))
            {
              if (s32k1xx_txringfull(priv))
                {
                  return -EBUSY;
                }
            }
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: s32k1xx_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void s32k1xx_receive(struct s32k1xx_driver_s *priv,
                            uint32_t flags)
{
  uint32_t mb_index;
  struct mb_s *rf;
#ifdef CONFIG_NET_CAN_CANFD
  uint32_t *frame_data_word;
  uint32_t i;
#endif

  while ((mb_index = arm_lsb(flags)) != 32)
    {
      rf = &priv->rx[mb_index];

      /* Read the frame contents */

#ifdef CONFIG_NET_CAN_CANFD
      if (rf->cs.edl) /* CAN FD frame */
        {
        struct canfd_frame *frame = (struct canfd_frame *)priv->rxdesc;

          if (rf->cs.ide)
            {
              frame->can_id = MASKEXTID & rf->id.ext;
              frame->can_id |= FLAGEFF;
            }
          else
            {
              frame->can_id = MASKSTDID & rf->id.std;
            }

          if (rf->cs.rtr)
            {
              frame->can_id |= FLAGRTR;
            }

          frame->len = can_dlc_to_len[rf->cs.dlc];

          frame_data_word = (uint32_t *)&frame->data[0];

          for (i = 0; i < (frame->len + 4 - 1) / 4; i++)
            {
              frame_data_word[i] = __builtin_bswap32(rf->data[i].w00);
            }

          /* Clear MB interrupt flag */

          putreg32(1 << mb_index,
                   priv->base + S32K1XX_CAN_IFLAG1_OFFSET);

          /* Copy the buffer pointer to priv->dev..  Set amount of data
           * in priv->dev.d_len
           */

          priv->dev.d_len = sizeof(struct canfd_frame);
          priv->dev.d_buf = (uint8_t *)frame;
        }
      else /* CAN 2.0 Frame */
#endif
        {
        struct can_frame *frame = (struct can_frame *)priv->rxdesc;

          if (rf->cs.ide)
            {
              frame->can_id = MASKEXTID & rf->id.ext;
              frame->can_id |= FLAGEFF;
            }
          else
            {
              frame->can_id = MASKSTDID & rf->id.std;
            }

          if (rf->cs.rtr)
            {
              frame->can_id |= FLAGRTR;
            }

          frame->can_dlc = rf->cs.dlc;

          *(uint32_t *)&frame->data[0] = __builtin_bswap32(rf->data[0].w00);
          *(uint32_t *)&frame->data[4] = __builtin_bswap32(rf->data[1].w00);

          /* Clear MB interrupt flag */

          putreg32(1 << mb_index,
                   priv->base + S32K1XX_CAN_IFLAG1_OFFSET);

          /* Copy the buffer pointer to priv->dev..  Set amount of data
           * in priv->dev.d_len
           */

          priv->dev.d_len = sizeof(struct can_frame);
          priv->dev.d_buf = (uint8_t *)frame;
        }

      /* Send to socket interface */

      NETDEV_RXPACKETS(&priv->dev);

      can_input(&priv->dev);

      /* Point the packet buffer back to the next Tx buffer that will be
       * used during the next write.  If the write queue is full, then
       * this will point at an active buffer, which must not be written
       * to.  This is OK because devif_poll won't be called unless the
       * queue is not full.
       */

      priv->dev.d_buf = (uint8_t *)priv->txdesc;

      flags &= ~(1 << mb_index);

      /* Reread interrupt flags and process them in this loop */

      if (flags == 0)
        {
          flags  = getreg32(priv->base + S32K1XX_CAN_IFLAG1_OFFSET);
          flags &= IFLAG1_RX;
        }
    }
}

/****************************************************************************
 * Function: s32k1xx_txdone
 *
 * Description:
 *   Check transmit interrupt flags and clear them
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   None
 *
 ****************************************************************************/

static void s32k1xx_txdone(struct s32k1xx_driver_s *priv)
{
  uint32_t flags;
  uint32_t mbi;
  uint32_t mb_bit;

  flags  = getreg32(priv->base + S32K1XX_CAN_IFLAG1_OFFSET);
  flags &= IFLAG1_TX;

  /* TODO First Process Error aborts */

  /* Process TX completions */

  mb_bit = 1 << RXMBCOUNT;
  for (mbi = 0; flags && mbi < TXMBCOUNT; mbi++)
    {
      if (flags & mb_bit)
        {
          putreg32(mb_bit, priv->base + S32K1XX_CAN_IFLAG1_OFFSET);
          flags &= ~mb_bit;
          NETDEV_TXDONE(&priv->dev);
#ifdef TX_TIMEOUT_WQ
          /* We are here because a transmission completed, so the
           * corresponding watchdog can be canceled
           * mailbox be set to inactive
           */

          wd_cancel(&priv->txtimeout[mbi]);
          struct mb_s *mb = &priv->tx[mbi];
          mb->cs.code = CAN_TXMB_INACTIVE;
#endif
        }

      mb_bit <<= 1;
    }
}

/****************************************************************************
 * Function: s32k1xx_txdone_work
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
 *   Global interrupts are disabled by the watchdog logic.
 *   We are not in an interrupt context so that we can lock the network.
 *
 ****************************************************************************/

static void s32k1xx_txdone_work(void *arg)
{
  struct s32k1xx_driver_s *priv = (struct s32k1xx_driver_s *)arg;

  s32k1xx_txdone(priv);

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  net_lock();
  devif_poll(&priv->dev, s32k1xx_txpoll);
  net_unlock();
}

/****************************************************************************
 * Function: s32k1xx_flexcan_interrupt
 *
 * Description:
 *   Three interrupt sources will vector to this function:
 *   1. CAN MB transmit interrupt handler
 *   2. CAN MB receive interrupt handler
 *   3.
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int s32k1xx_flexcan_interrupt(int irq, void *context,
                                     void *arg)
{
  struct s32k1xx_driver_s *priv = (struct s32k1xx_driver_s *)arg;

  if (irq == priv->config->mb_irq)
    {
      uint32_t flags;
      flags  = getreg32(priv->base + S32K1XX_CAN_IFLAG1_OFFSET);
      flags &= IFLAG1_RX;

      if (flags)
        {
          /* Process immediately since scheduling a workqueue is too slow
           * which causes us to drop CAN frames
           */

          s32k1xx_receive(priv, flags);
        }

      flags  = getreg32(priv->base + S32K1XX_CAN_IFLAG1_OFFSET);
      flags &= IFLAG1_TX;

      if (flags)
        {
          /* Disable further TX MB CAN interrupts. here can be no race
           * condition here.
           */

          flags  = getreg32(priv->base + S32K1XX_CAN_IMASK1_OFFSET);
          flags &= ~(IFLAG1_TX);
          putreg32(flags, priv->base + S32K1XX_CAN_IMASK1_OFFSET);
          work_queue(CANWORK, &priv->irqwork, s32k1xx_txdone_work, priv, 0);
        }
    }

  return OK;
}

/****************************************************************************
 * Function: s32k1xx_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/
#ifdef TX_TIMEOUT_WQ

static void s32k1xx_txtimeout_work(void *arg)
{
  struct s32k1xx_driver_s *priv = (struct s32k1xx_driver_s *)arg;
  uint32_t flags;
  uint32_t mbi;
  uint32_t mb_bit;

  struct timespec ts;
  struct timeval *now = (struct timeval *)&ts;
  clock_systime_timespec(&ts);
  now->tv_usec = ts.tv_nsec / 1000; /* timespec to timeval conversion */

  /* The watchdog timed out, yet we still check mailboxes in case the
   * transmit function transmitted a new frame
   */

  flags  = getreg32(priv->base + S32K1XX_CAN_IFLAG1_OFFSET);

  for (mbi = 0; mbi < TXMBCOUNT; mbi++)
    {
      if (priv->txmb[mbi].deadline.tv_sec != 0
          && (now->tv_sec > priv->txmb[mbi].deadline.tv_sec
          || now->tv_usec > priv->txmb[mbi].deadline.tv_usec))
        {
          NETDEV_TXTIMEOUTS(&priv->dev);

          mb_bit = 1 << (RXMBCOUNT +  mbi);

          if (flags & mb_bit)
            {
              putreg32(mb_bit, priv->base + S32K1XX_CAN_IFLAG1_OFFSET);
            }

          struct mb_s *mb = &priv->tx[mbi];
          mb->cs.code = CAN_TXMB_ABORT;
          priv->txmb[mbi].pending = TX_ABORT;
        }
    }
}

/****************************************************************************
 * Function: s32k1xx_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void s32k1xx_txtimeout_expiry(wdparm_t arg)
{
  struct s32k1xx_driver_s *priv = (struct s32k1xx_driver_s *)arg;

  /* Schedule to perform the TX timeout processing on the worker thread
   */

  work_queue(CANWORK, &priv->irqwork, s32k1xx_txtimeout_work, priv, 0);
}

#endif

static void s32k1xx_setenable(uint32_t base, uint32_t enable)
{
  uint32_t regval;

  if (enable)
    {
      regval  = getreg32(base + S32K1XX_CAN_MCR_OFFSET);
      regval &= ~(CAN_MCR_MDIS);
      putreg32(regval, base + S32K1XX_CAN_MCR_OFFSET);
    }
  else
    {
      regval  = getreg32(base + S32K1XX_CAN_MCR_OFFSET);
      regval |= CAN_MCR_MDIS;
      putreg32(regval, base + S32K1XX_CAN_MCR_OFFSET);
    }

  s32k1xx_waitmcr_change(base, CAN_MCR_LPMACK, 1);
}

static uint32_t s32k1xx_waitesr2_change(uint32_t base, uint32_t mask,
                                       uint32_t target_state)
{
  const uint32_t timeout = 1000;
  uint32_t wait_ack;

  for (wait_ack = 0; wait_ack < timeout; wait_ack++)
    {
      uint32_t state = (getreg32(base + S32K1XX_CAN_ESR2_OFFSET) & mask);
      if (state == target_state)
        {
          return true;
        }

      up_udelay(10);
    }

  return false;
}

static void s32k1xx_setfreeze(uint32_t base, uint32_t freeze)
{
  uint32_t regval;
  if (freeze)
    {
      /* Enter freeze mode */

      regval  = getreg32(base + S32K1XX_CAN_MCR_OFFSET);
      regval |= (CAN_MCR_HALT | CAN_MCR_FRZ);
      putreg32(regval, base + S32K1XX_CAN_MCR_OFFSET);
    }
  else
    {
      /* Exit freeze mode */

      regval  = getreg32(base + S32K1XX_CAN_MCR_OFFSET);
      regval &= ~(CAN_MCR_HALT | CAN_MCR_FRZ);
      putreg32(regval, base + S32K1XX_CAN_MCR_OFFSET);
    }
}

static uint32_t s32k1xx_waitmcr_change(uint32_t base, uint32_t mask,
                                       uint32_t target_state)
{
  const uint32_t timeout = 1000;
  uint32_t wait_ack;

  for (wait_ack = 0; wait_ack < timeout; wait_ack++)
    {
      const bool state = (getreg32(base + S32K1XX_CAN_MCR_OFFSET) & mask)
          != 0;
      if (state == target_state)
        {
          return true;
        }

      up_udelay(10);
    }

  return false;
}

static uint32_t s32k1xx_waitfreezeack_change(uint32_t base,
                                             uint32_t target_state)
{
  return s32k1xx_waitmcr_change(base, CAN_MCR_FRZACK, target_state);
}

/****************************************************************************
 * Function: s32k1xx_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
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

static int s32k1xx_ifup(struct net_driver_s *dev)
{
  struct s32k1xx_driver_s *priv =
    (struct s32k1xx_driver_s *)dev->d_private;

  if (!s32k1xx_initialize(priv))
    {
      nerr("initialize failed");
      return -1;
    }

  priv->bifup = true;

#ifdef CONFIG_NET_CAN_CANFD
  priv->txdesc = (struct canfd_frame *)&g_tx_pool;
  priv->rxdesc = (struct canfd_frame *)&g_rx_pool;
#else
  priv->txdesc = (struct can_frame *)&g_tx_pool;
  priv->rxdesc = (struct can_frame *)&g_rx_pool;
#endif

  priv->dev.d_buf = (uint8_t *)priv->txdesc;

  /* Set interrupts */

  up_enable_irq(priv->config->bus_irq);
  up_enable_irq(priv->config->error_irq);
  if (priv->config->lprx_irq > 0)
    {
      up_enable_irq(priv->config->lprx_irq);
    }

  up_enable_irq(priv->config->mb_irq);

  return OK;
}

/****************************************************************************
 * Function: s32k1xx_ifdown
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

static int s32k1xx_ifdown(struct net_driver_s *dev)
{
  struct s32k1xx_driver_s *priv =
    (struct s32k1xx_driver_s *)dev->d_private;

  s32k1xx_reset(priv);

  priv->bifup = false;
  return OK;
}

/****************************************************************************
 * Function: s32k1xx_txavail_work
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

static void s32k1xx_txavail_work(void *arg)
{
  struct s32k1xx_driver_s *priv = (struct s32k1xx_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      if (s32k1xx_waitesr2_change(priv->base,
                             (CAN_ESR2_IMB | CAN_ESR2_VPS),
                             (CAN_ESR2_IMB | CAN_ESR2_VPS)))
        {
          /* No, there is space for another transfer.  Poll the network for
           * new XMIT data.
           */

          devif_poll(&priv->dev, s32k1xx_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: s32k1xx_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
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

static int s32k1xx_txavail(struct net_driver_s *dev)
{
  struct s32k1xx_driver_s *priv =
    (struct s32k1xx_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      s32k1xx_txavail_work(priv);
    }

  return OK;
}

/****************************************************************************
 * Function: s32k1xx_ioctl
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

#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
static int s32k1xx_ioctl(struct net_driver_s *dev, int cmd,
                         unsigned long arg)
{
  struct s32k1xx_driver_s *priv =
      (struct s32k1xx_driver_s *)dev->d_private;

  int ret;

  switch (cmd)
    {
      case SIOCGCANBITRATE: /* Get bitrate from a CAN controller */
        {
          struct can_ioctl_data_s *req =
              (struct can_ioctl_data_s *)((uintptr_t)arg);
          req->arbi_bitrate = priv->arbi_timing.bitrate / 1000; /* kbit/s */
          req->arbi_samplep = priv->arbi_timing.samplep;
#ifdef CONFIG_NET_CAN_CANFD
          req->data_bitrate = priv->data_timing.bitrate / 1000; /* kbit/s */
          req->data_samplep = priv->data_timing.samplep;
#else
          req->data_bitrate = 0;
          req->data_samplep = 0;
#endif
          ret = OK;
        }
        break;

      case SIOCSCANBITRATE: /* Set bitrate of a CAN controller */
        {
          struct can_ioctl_data_s *req =
              (struct can_ioctl_data_s *)((uintptr_t)arg);

          struct flexcan_timeseg arbi_timing;
          arbi_timing.bitrate = req->arbi_bitrate * 1000;
          arbi_timing.samplep = req->arbi_samplep;

          if (s32k1xx_bitratetotimeseg(&arbi_timing, 10, 0))
            {
              ret = OK;
            }
          else
            {
              ret = -EINVAL;
            }

#ifdef CONFIG_NET_CAN_CANFD
          struct flexcan_timeseg data_timing;
          data_timing.bitrate = req->data_bitrate * 1000;
          data_timing.samplep = req->data_samplep;

          if (ret == OK && s32k1xx_bitratetotimeseg(&data_timing, 10, 1))
            {
              ret = OK;
            }
          else
            {
              ret = -EINVAL;
            }
#endif

          if (ret == OK)
            {
              /* Reset CAN controller and start with new timings */

              priv->arbi_timing = arbi_timing;
#ifdef CONFIG_NET_CAN_CANFD
              priv->data_timing = data_timing;
#endif
              s32k1xx_ifup(dev);
            }
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Function: s32k1xx_initalize
 *
 * Description:
 *   Initialize FLEXCAN device
 *
 * Input Parameters:
 *   priv - Reference to the private FLEXCAN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int s32k1xx_initialize(struct s32k1xx_driver_s *priv)
{
  uint32_t regval;
  uint32_t i;

  /* initialize CAN device */

  s32k1xx_setenable(priv->base, 0);

  /* Set SYS_CLOCK src */

  regval  = getreg32(priv->base + S32K1XX_CAN_CTRL1_OFFSET);
  regval |= CAN_CTRL1_CLKSRC;
  putreg32(regval, priv->base + S32K1XX_CAN_CTRL1_OFFSET);

  s32k1xx_setenable(priv->base, 1);

  s32k1xx_reset(priv);

  /* Enter freeze mode */

  s32k1xx_setfreeze(priv->base, 1);
  if (!s32k1xx_waitfreezeack_change(priv->base, 1))
    {
      ninfo("FLEXCAN: freeze fail\n");
      return -1;
    }

  /* Reset CTRL1 register to reset value */

  regval  = getreg32(priv->base + S32K1XX_CAN_CTRL1_OFFSET);
  regval &= ~(CAN_CTRL1_LOM | CAN_CTRL1_LBUF | CAN_CTRL1_TSYN |
              CAN_CTRL1_BOFFREC | CAN_CTRL1_SMP | CAN_CTRL1_RWRNMSK |
              CAN_CTRL1_TWRNMSK | CAN_CTRL1_LPB | CAN_CTRL1_ERRMSK |
              CAN_CTRL1_BOFFMSK);
  putreg32(regval, priv->base + S32K1XX_CAN_CTRL1_OFFSET);

#ifndef CONFIG_NET_CAN_CANFD
  regval  = getreg32(priv->base + S32K1XX_CAN_CTRL1_OFFSET);

  regval &= ~(CAN_CTRL1_TIMINGMSK); /* Reset timings */

  regval |= CAN_CTRL1_PRESDIV(priv->arbi_timing.presdiv) | /* Prescaler divisor factor */
            CAN_CTRL1_PROPSEG(priv->arbi_timing.propseg) | /* Propagation segment */
            CAN_CTRL1_PSEG1(priv->arbi_timing.pseg1) |     /* Phase buffer segment 1 */
            CAN_CTRL1_PSEG2(priv->arbi_timing.pseg2) |     /* Phase buffer segment 2 */
            CAN_CTRL1_RJW(1);                              /* Resynchronization jump width */
  putreg32(regval, priv->base + S32K1XX_CAN_CTRL1_OFFSET);

#else

  regval  = CAN_CBT_BTF |                                 /* Enable extended bit timing
                                                           * configurations for CAN-FD for setting up
                                                           * separately nominal and data phase */
            CAN_CBT_EPRESDIV(priv->arbi_timing.presdiv) | /* Prescaler divisor factor */
            CAN_CBT_EPROPSEG(priv->arbi_timing.propseg) | /* Propagation segment */
            CAN_CBT_EPSEG1(priv->arbi_timing.pseg1) |     /* Phase buffer segment 1 */
            CAN_CBT_EPSEG2(priv->arbi_timing.pseg2) |     /* Phase buffer segment 2 */
            CAN_CBT_ERJW(1);                              /* Resynchronization jump width */
  putreg32(regval, priv->base + S32K1XX_CAN_CBT_OFFSET);

  /* Enable CAN FD feature */

  regval  = getreg32(priv->base + S32K1XX_CAN_MCR_OFFSET);
  regval |= CAN_MCR_FDEN;
  putreg32(regval, priv->base + S32K1XX_CAN_MCR_OFFSET);

  regval  = CAN_FDCBT_FPRESDIV(priv->data_timing.presdiv) |  /* Prescaler divisor factor of 1 */
            CAN_FDCBT_FPROPSEG(priv->data_timing.propseg) |  /* Propagation
                                                              * segment (only register that doesn't add 1) */
            CAN_FDCBT_FPSEG1(priv->data_timing.pseg1) |      /* Phase buffer segment 1 */
            CAN_FDCBT_FPSEG2(priv->data_timing.pseg2) |      /* Phase buffer segment 2 */
            CAN_FDCBT_FRJW(priv->data_timing.pseg2);         /* Resynchorinzation jump width same as PSEG2 */
  putreg32(regval, priv->base + S32K1XX_CAN_FDCBT_OFFSET);

  /* Additional CAN-FD configurations */

  regval  = CAN_FDCTRL_FDRATE |     /* Enable bit rate switch in data phase of frame */
            CAN_FDCTRL_TDCEN |      /* Enable transceiver delay compensation */
            CAN_FDCTRL_TDCOFF(5) |  /* Setup 5 cycles for data phase sampling delay */
            CAN_FDCTRL_MBDSR0(3);   /* Setup 64 bytes per message buffer (7 MB's) */
  putreg32(regval, priv->base + S32K1XX_CAN_FDCTRL_OFFSET);

  regval  = getreg32(priv->base + S32K1XX_CAN_CTRL2_OFFSET);
  regval |= CAN_CTRL2_ISOCANFDEN;
  putreg32(regval, priv->base + S32K1XX_CAN_CTRL2_OFFSET);
#endif

  for (i = TXMBCOUNT; i < TOTALMBCOUNT; i++)
    {
      priv->rx[i].id.w = 0x0;

      /* FIXME sometimes we get a hard fault here */
    }

  putreg32(0x0, priv->base + S32K1XX_CAN_RXFGMASK_OFFSET);

  for (i = 0; i < S32K1XX_CAN_RXIMR_COUNT; i++)
    {
      putreg32(0, priv->base + S32K1XX_CAN_RXIMR_OFFSET(i));
    }

  for (i = 0; i < RXMBCOUNT; i++)
    {
      ninfo("Set MB%" PRIu32 " to receive %p\n", i, &priv->rx[i]);
      priv->rx[i].cs.edl = 0x1;
      priv->rx[i].cs.brs = 0x1;
      priv->rx[i].cs.esi = 0x0;
      priv->rx[i].cs.code = 4;
      priv->rx[i].cs.srr = 0x0;
      priv->rx[i].cs.ide = 0x1;
      priv->rx[i].cs.rtr = 0x0;
    }

  putreg32(IFLAG1_RX, priv->base + S32K1XX_CAN_IFLAG1_OFFSET);
  putreg32(IFLAG1_RX, priv->base + S32K1XX_CAN_IMASK1_OFFSET);

  /* Exit freeze mode */

  s32k1xx_setfreeze(priv->base, 0);
  if (!s32k1xx_waitfreezeack_change(priv->base, 0))
    {
      ninfo("FLEXCAN: unfreeze fail\n");
      return -1;
    }

  return 1;
}

/****************************************************************************
 * Function: s32k1xx_reset
 *
 * Description:
 *   Put the EMAC in the non-operational, reset state
 *
 * Input Parameters:
 *   priv - Reference to the private FLEXCAN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void s32k1xx_reset(struct s32k1xx_driver_s *priv)
{
  uint32_t regval;
  uint32_t i;

  regval  = getreg32(priv->base + S32K1XX_CAN_MCR_OFFSET);
  regval |= CAN_MCR_SOFTRST;
  putreg32(regval, priv->base + S32K1XX_CAN_MCR_OFFSET);

  if (!s32k1xx_waitmcr_change(priv->base, CAN_MCR_SOFTRST, 0))
    {
      nerr("Reset failed");
      return;
    }

  regval  = getreg32(priv->base + S32K1XX_CAN_MCR_OFFSET);
  regval &= ~(CAN_MCR_SUPV);
  putreg32(regval, priv->base + S32K1XX_CAN_MCR_OFFSET);

  /* Initialize all MB rx and tx */

  for (i = 0; i < TOTALMBCOUNT; i++)
    {
      ninfo("MB %" PRIu32 " %p\n", i, &priv->rx[i]);
      ninfo("MB %" PRIu32 " %p\n", i, &priv->rx[i].id.w);
      priv->rx[i].cs.cs = 0x0;
      priv->rx[i].id.w = 0x0;
      priv->rx[i].data[0].w00 = 0x0;
      priv->rx[i].data[1].w00 = 0x0;
    }

  regval  = getreg32(priv->base + S32K1XX_CAN_MCR_OFFSET);
  regval |= CAN_MCR_SLFWAK | CAN_MCR_WRNEN | CAN_MCR_SRXDIS |
            CAN_MCR_IRMQ | CAN_MCR_AEN |
            (((TOTALMBCOUNT - 1) << CAN_MCR_MAXMB_SHIFT) &
            CAN_MCR_MAXMB_MASK);
  putreg32(regval, priv->base + S32K1XX_CAN_MCR_OFFSET);

  regval  = CAN_CTRL2_RRS | CAN_CTRL2_EACEN;
  putreg32(regval, priv->base + S32K1XX_CAN_CTRL2_OFFSET);

  for (i = 0; i < TOTALMBCOUNT; i++)
    {
      putreg32(0, priv->base + S32K1XX_CAN_RXIMR_OFFSET(i));
    }

  /* Filtering catchall */

  putreg32(0x3fffffff, priv->base + S32K1XX_CAN_RX14MASK_OFFSET);
  putreg32(0x3fffffff, priv->base + S32K1XX_CAN_RX15MASK_OFFSET);
  putreg32(0x3fffffff, priv->base + S32K1XX_CAN_RXMGMASK_OFFSET);
  putreg32(0x0, priv->base + S32K1XX_CAN_RXFGMASK_OFFSET);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: s32k1xx_caninitialize
 *
 * Description:
 *   Initialize the CAN controller and driver
 *
 * Input Parameters:
 *   intf - In the case where there are multiple CAN devices, this value
 *          identifies which CAN device is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int s32k1xx_caninitialize(int intf)
{
  struct s32k1xx_driver_s *priv;
  int ret;

  switch (intf)
    {
#ifdef CONFIG_S32K1XX_FLEXCAN0
    case 0:
      priv               = &g_flexcan0;
      memset(priv, 0, sizeof(struct s32k1xx_driver_s));
      priv->base         = S32K1XX_FLEXCAN0_BASE;
      priv->config       = &s32k1xx_flexcan0_config;

      /* Default bitrate configuration */

#  ifdef CONFIG_NET_CAN_CANFD
      priv->arbi_timing.bitrate = CONFIG_FLEXCAN0_ARBI_BITRATE;
      priv->arbi_timing.samplep = CONFIG_FLEXCAN0_ARBI_SAMPLEP;
      priv->data_timing.bitrate = CONFIG_FLEXCAN0_DATA_BITRATE;
      priv->data_timing.samplep = CONFIG_FLEXCAN0_DATA_SAMPLEP;
#  else
      priv->arbi_timing.bitrate = CONFIG_FLEXCAN0_BITRATE;
      priv->arbi_timing.samplep = CONFIG_FLEXCAN0_SAMPLEP;
#  endif
      break;
#endif

#ifdef CONFIG_S32K1XX_FLEXCAN1
    case 1:
      priv         = &g_flexcan1;
      memset(priv, 0, sizeof(struct s32k1xx_driver_s));
      priv->base   = S32K1XX_FLEXCAN1_BASE;
      priv->config = &s32k1xx_flexcan1_config;

      /* Default bitrate configuration */

#  ifdef CONFIG_NET_CAN_CANFD
      priv->arbi_timing.bitrate = CONFIG_FLEXCAN1_ARBI_BITRATE;
      priv->arbi_timing.samplep = CONFIG_FLEXCAN1_ARBI_SAMPLEP;
      priv->data_timing.bitrate = CONFIG_FLEXCAN1_DATA_BITRATE;
      priv->data_timing.samplep = CONFIG_FLEXCAN1_DATA_SAMPLEP;
#  else
      priv->arbi_timing.bitrate = CONFIG_FLEXCAN1_BITRATE;
      priv->arbi_timing.samplep = CONFIG_FLEXCAN1_SAMPLEP;
#  endif
      break;
#endif

#ifdef CONFIG_S32K1XX_FLEXCAN2
    case 2:
      priv         = &g_flexcan2;
      memset(priv, 0, sizeof(struct s32k1xx_driver_s));
      priv->base   = S32K1XX_FLEXCAN2_BASE;
      priv->config = &s32k1xx_flexcan2_config;

      /* Default bitrate configuration */

#  ifdef CONFIG_NET_CAN_CANFD
      priv->arbi_timing.bitrate = CONFIG_FLEXCAN2_ARBI_BITRATE;
      priv->arbi_timing.samplep = CONFIG_FLEXCAN2_ARBI_SAMPLEP;
      priv->data_timing.bitrate = CONFIG_FLEXCAN2_DATA_BITRATE;
      priv->data_timing.samplep = CONFIG_FLEXCAN2_DATA_SAMPLEP;
#  else
      priv->arbi_timing.bitrate = CONFIG_FLEXCAN2_BITRATE;
      priv->arbi_timing.samplep = CONFIG_FLEXCAN2_SAMPLEP;
#  endif
      break;
#endif

    default:
      return -ENODEV;
    }

  if (!s32k1xx_bitratetotimeseg(&priv->arbi_timing, 1, 0))
    {
      nerr("ERROR: Invalid CAN timings please try another sample point "
           "or refer to the reference manual\n");
      return -1;
    }

#ifdef CONFIG_NET_CAN_CANFD
  if (!s32k1xx_bitratetotimeseg(&priv->data_timing, 1, 1))
    {
      nerr("ERROR: Invalid CAN data phase timings please try another "
           "sample point or refer to the reference manual\n");
      return -1;
    }
#endif

  s32k1xx_pinconfig(priv->config->tx_pin);
  s32k1xx_pinconfig(priv->config->rx_pin);
  if (priv->config->enable_pin > 0)
    {
      s32k1xx_pinconfig(priv->config->enable_pin);
      s32k1xx_gpiowrite(priv->config->enable_pin, priv->config->enable_high);
    }

  /* Attach the flexcan interrupt handler */

  if (irq_attach(priv->config->bus_irq, s32k1xx_flexcan_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach CAN bus IRQ\n");
      return -EAGAIN;
    }

  if (irq_attach(priv->config->error_irq, s32k1xx_flexcan_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach CAN error IRQ\n");
      return -EAGAIN;
    }

  if (priv->config->lprx_irq > 0)
    {
      if (irq_attach(priv->config->lprx_irq,
                     s32k1xx_flexcan_interrupt, priv))
        {
          /* We could not attach the ISR to the interrupt */

          nerr("ERROR: Failed to attach CAN LPRX IRQ\n");
          return -EAGAIN;
        }
    }

  if (irq_attach(priv->config->mb_irq, s32k1xx_flexcan_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach CAN OR'ed Message buffer (0-15) IRQ\n");
      return -EAGAIN;
    }

  /* Initialize the driver structure */

  priv->dev.d_ifup    = s32k1xx_ifup;      /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = s32k1xx_ifdown;    /* I/F down callback */
  priv->dev.d_txavail = s32k1xx_txavail;   /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = s32k1xx_ioctl;     /* Support CAN ioctl() calls */
#endif
  priv->dev.d_private = priv;              /* Used to recover private state from dev */
  priv->rx            = (struct mb_s *)(priv->base + S32K1XX_CAN_MB_OFFSET);
  priv->tx            = (struct mb_s *)(priv->base + S32K1XX_CAN_MB_OFFSET +
                          (sizeof(struct mb_s) * RXMBCOUNT));

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling s32k1xx_ifdown().
   */

  ninfo("callbacks done\n");

  s32k1xx_initialize(priv);

  s32k1xx_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_CAN);

  UNUSED(ret);
  return OK;
}

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Initialize the enabled CAN device interfaces.  If there are more
 *   different network devices in the chip, then board-specific logic will
 *   have to provide this function to determine which, if any, network
 *   devices should be initialized.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
#ifdef CONFIG_S32K1XX_FLEXCAN0
  s32k1xx_caninitialize(0);
#endif

#ifdef CONFIG_S32K1XX_FLEXCAN1
  s32k1xx_caninitialize(1);
#endif

#ifdef CONFIG_S32K1XX_FLEXCAN2
  s32k1xx_caninitialize(2);
#endif
}
#endif

#endif /* CONFIG_S32K1XX_FLEXCAN */

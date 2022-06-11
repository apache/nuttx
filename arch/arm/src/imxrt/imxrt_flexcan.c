/****************************************************************************
 * arch/arm/src/imxrt/imxrt_flexcan.c
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/can.h>

#include "arm_internal.h"
#include "chip.h"
#include "imxrt_config.h"
#include "imxrt_flexcan.h"
#include "imxrt_periphclks.h"
#include "hardware/imxrt_flexcan.h"
#include "hardware/imxrt_pinmux.h"
#include "hardware/imxrt_ccm.h"

#include <arch/board/board.h>

#include <sys/time.h>

#ifdef CONFIG_IMXRT_FLEXCAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#define CANWORK LPWORK

/* CONFIG_IMXRT_FLEXCAN_NETHIFS determines the number of physical
 * interfaces that will be supported.
 */

#define MASKSTDID                   0x000007ff
#define MASKEXTID                   0x1fffffff
#define FLAGEFF                     (1 << 31) /* Extended frame format */
#define FLAGRTR                     (1 << 30) /* Remote transmission request */

#define RXMBCOUNT                   CONFIG_IMXRT_FLEXCAN_RXMB
#define TXMBCOUNT                   (CONFIG_IMXRT_FLEXCAN_TXMB + 1)
#define TOTALMBCOUNT                RXMBCOUNT + TXMBCOUNT

#define IFLAG1_RX                   ((1 << RXMBCOUNT)-1)
#define IFLAG1_TX                   (((1 << TXMBCOUNT)-2) << RXMBCOUNT)

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

#if (CONFIG_IMXRT_FLEXCAN_RXMB + CONFIG_IMXRT_FLEXCAN_TXMB) > 13
# error Only 13 MB are allowed to be used
#endif

/* Interrupt flags for RX fifo */
#define IFLAG1_RXFIFO               (CAN_FIFO_NE | CAN_FIFO_WARN | CAN_FIFO_OV)

static int peak_tx_mailbox_index_ = 0;

static uint8_t mb_address[] =
                              {
                               0x10, 0x12, 0x14, 0x16, 0x18, 0x1a, 0x1c,
                               0x1e, 0x20, 0x22, 0x24, 0x26, 0x28, 0x2a,
                               0x10, 0x19, 0x22, 0x2b, 0x34, 0x3d, 0x46,
                               0x50, 0x59, 0x62, 0x6b, 0x74, 0x7d, 0x86
                              };

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
  union data_e data[];
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
  uint32_t irq;              /* Combined interrupt */
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

#ifdef CONFIG_IMXRT_FLEXCAN1
static const struct flexcan_config_s imxrt_flexcan1_config =
{
  .tx_pin      = GPIO_FLEXCAN1_TX,
  .rx_pin      = GPIO_FLEXCAN1_RX,
  .irq         = IMXRT_IRQ_CAN1,
};
#endif

#ifdef CONFIG_IMXRT_FLEXCAN2
static const struct flexcan_config_s imxrt_flexcan2_config =
{
  .tx_pin      = GPIO_FLEXCAN2_TX,
  .rx_pin      = GPIO_FLEXCAN2_RX,
  .irq         = IMXRT_IRQ_CAN2,
};
#endif

#ifdef CONFIG_IMXRT_FLEXCAN3
static const struct flexcan_config_s imxrt_flexcan3_config =
{
  .tx_pin      = GPIO_FLEXCAN3_TX,
  .rx_pin      = GPIO_FLEXCAN3_RX,
  .irq         = IMXRT_IRQ_CAN3,
};
#endif

/* The imxrt_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct imxrt_driver_s
{
  uint32_t base;                /* FLEXCAN base address */
  bool bifup;                   /* true:ifup false:ifdown */
  bool canfd_capable;
  int mb_address_offset;
#ifdef TX_TIMEOUT_WQ
  WDOG_ID txtimeout[TXMBCOUNT]; /* TX timeout timer */
#endif
  struct work_s irqwork;            /* For deferring interrupt work to the wq */
  struct work_s pollwork;           /* For deferring poll work to the work wq */
  struct canfd_frame *txdesc_fd;    /* A pointer to the list of TX descriptor for FD frames */
  struct canfd_frame *rxdesc_fd;    /* A pointer to the list of RX descriptors for FD frames */
  struct can_frame *txdesc;         /* A pointer to the list of TX descriptor */
  struct can_frame *rxdesc;         /* A pointer to the list of RX descriptors */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;      /* Interface understood by the network */

  struct flexcan_timeseg arbi_timing; /* Timing for arbitration phase */
  struct flexcan_timeseg data_timing; /* Timing for data phase */

  const struct flexcan_config_s *config;

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  struct txmbstats txmb[TXMBCOUNT];
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_IMXRT_FLEXCAN1
static struct imxrt_driver_s g_flexcan1;
#endif

#ifdef CONFIG_IMXRT_FLEXCAN2
static struct imxrt_driver_s g_flexcan2;
#endif

#ifdef CONFIG_IMXRT_FLEXCAN3
static struct imxrt_driver_s g_flexcan3;
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
 * Name: imxrt_bitratetotimeseg
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

uint32_t imxrt_bitratetotimeseg(struct flexcan_timeseg *timeseg,
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

static bool imxrt_txringfull(struct imxrt_driver_s *priv);
static int  imxrt_transmit(struct imxrt_driver_s *priv);
static int  imxrt_txpoll(struct net_driver_s *dev);

/* Helper functions */

static void imxrt_setenable(uint32_t base, uint32_t enable);
static void imxrt_setfreeze(uint32_t base, uint32_t freeze);
static uint32_t imxrt_waitmcr_change(uint32_t base,
                                       uint32_t mask,
                                       uint32_t target_state);
static struct mb_s *flexcan_get_mb(struct imxrt_driver_s *priv,
                                    uint32_t mbi);

/* Interrupt handling */

static void imxrt_receive(struct imxrt_driver_s *priv,
                            uint32_t flags);
static void imxrt_txdone_work(void *arg);
static void imxrt_txdone(struct imxrt_driver_s *priv);

static int  imxrt_flexcan_interrupt(int irq, void *context,
                                      void *arg);

/* Watchdog timer expirations */
#ifdef TX_TIMEOUT_WQ
static void imxrt_txtimeout_work(void *arg);
static void imxrt_txtimeout_expiry(int argc, uint32_t arg, ...);
#endif

/* NuttX callback functions */

static int  imxrt_ifup(struct net_driver_s *dev);
static int  imxrt_ifdown(struct net_driver_s *dev);

static void imxrt_txavail_work(void *arg);
static int  imxrt_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NETDEV_IOCTL
static int  imxrt_ioctl(struct net_driver_s *dev, int cmd,
                          unsigned long arg);
#endif

/* Initialization */

static int  imxrt_initialize(struct imxrt_driver_s *priv);
static void imxrt_reset(struct imxrt_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: imxrt_txringfull
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

static bool imxrt_txringfull(struct imxrt_driver_s *priv)
{
  uint32_t mbi = RXMBCOUNT + 1;
  struct mb_s *mb;

  while (mbi < TOTALMBCOUNT)
    {
      mb = flexcan_get_mb(priv, mbi);
      if (mb->cs.code != CAN_TXMB_DATAORREMOTE)
        {
          return 0;
        }

      mbi++;
    }

  return 1;
}

/****************************************************************************
 * Function: imxrt_transmit
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

static int imxrt_transmit(struct imxrt_driver_s *priv)
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

  mbi = RXMBCOUNT + 1;
  mb_bit = 1 << mbi;

  while (mbi < TOTALMBCOUNT)
    {
      /* Check whether message buffer is not currently transmitting */

      struct mb_s *mb = flexcan_get_mb(priv, mbi);
      if (mb->cs.code != CAN_TXMB_DATAORREMOTE)
        {
          putreg32(mb_bit, priv->base + IMXRT_CAN_IFLAG1_OFFSET);
          break;
        }

      mb_bit <<= 1;
      mbi++;
    }

  if (mbi == TOTALMBCOUNT)
    {
      nwarn("No TX MB available mbi %" PRIi32 "\n", mbi);
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
  struct mb_s *mb = flexcan_get_mb(priv, mbi);
  mb->cs.code = CAN_TXMB_INACTIVE;

  if (priv->dev.d_len == sizeof(struct can_frame))
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

  /* Errata ER005829 step 8: Write twice into the first TX MB
   * Errata mentions writng 0x8 value, but this one couses
   * the ESR2_LPTM register to choose the reserved MB for
   * transmiting the package, hence we write 0x3
   */

  struct mb_s *buffer = flexcan_get_mb(priv, RXMBCOUNT);
  buffer->cs.code = 0x3;
  buffer->cs.code = 0x3;

  regval = getreg32(priv->base + IMXRT_CAN_IMASK1_OFFSET);
  regval |= mb_bit;
  putreg32(regval, priv->base + IMXRT_CAN_IMASK1_OFFSET);

  /* Increment statistics */

  NETDEV_TXPACKETS(&priv->dev);

#ifdef TX_TIMEOUT_WQ
  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  if (timeout > 0)
    {
      wd_start(priv->txtimeout[mbi], timeout + 1, imxrt_txtimeout_expiry,
                1, (wdparm_t)priv);
    }
#endif

  return OK;
}

/****************************************************************************
 * Function: imxrt_txpoll
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

static int imxrt_txpoll(struct net_driver_s *dev)
{
  struct imxrt_driver_s *priv =
    (struct imxrt_driver_s *)dev->d_private;
  irqstate_t flags;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  flags = spin_lock_irqsave(NULL);

  if (priv->dev.d_len > 0)
    {
      if (!devif_loopback(&priv->dev))
        {
          imxrt_txdone(priv);

          /* Send the packet */

          imxrt_transmit(priv);

          /* Check if there is room in the device to hold another packet. If
           * not, return a non-zero value to terminate the poll.
           */

          if (imxrt_txringfull(priv))
            {
              spin_unlock_irqrestore(NULL, flags);
              return -EBUSY;
            }
        }
    }

  spin_unlock_irqrestore(NULL, flags);

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: imxrt_receive
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

static void imxrt_receive(struct imxrt_driver_s *priv,
                            uint32_t flags)
{
  uint32_t mbi;
  uint32_t mbj;
  struct mb_s *rf;
# ifdef CONFIG_NET_CAN_CANFD
  uint32_t *frame_data_word;
  uint32_t i;
# endif
  uint32_t f;

  while ((f = flags) != 0)
    {
      mbj = mbi = arm_lsb(f);
      rf = flexcan_get_mb(priv, mbi);
      uint32_t t = rf->cs.time_stamp;
      while ((f &= ~(1 << mbj)) != 0)
        {
          mbj = arm_lsb(f);
          struct mb_s *rf_next = flexcan_get_mb(priv, mbj);
          uint16_t t_next = rf_next->cs.time_stamp;
          if ((int16_t)(t - t_next) > 0)
            {
              t = t_next;
              mbi = mbj;
            }
        }

      rf = flexcan_get_mb(priv, mbi);

      /* Read the frame contents */

#ifdef CONFIG_NET_CAN_CANFD
      if (rf->cs.edl) /* CAN FD frame */
        {
        struct canfd_frame *frame = (struct canfd_frame *)priv->rxdesc_fd;

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

          putreg32(1 << mbi,
                   priv->base + IMXRT_CAN_IFLAG1_OFFSET);

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

          putreg32(1 << mbi,
                   priv->base + IMXRT_CAN_IFLAG1_OFFSET);

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

      if (priv->canfd_capable)
        {
          priv->dev.d_buf = (uint8_t *)priv->txdesc_fd;
        }
      else
        {
          priv->dev.d_buf = (uint8_t *)priv->txdesc;
        }

      flags &= ~(1 << mbi);

      /* Reread interrupt flags and process them in this loop */

      if (flags == 0)
        {
          flags  = getreg32(priv->base + IMXRT_CAN_IFLAG1_OFFSET);
          flags &= IFLAG1_RX;
        }
    }
}

/****************************************************************************
 * Function: imxrt_txdone
 *
 * Description:
 *   Check transmit interrupt flags and clear them
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void imxrt_txdone(struct imxrt_driver_s *priv)
{
  uint32_t flags;
  uint32_t mbi;
  uint32_t mb_bit;

  flags  = getreg32(priv->base + IMXRT_CAN_IFLAG1_OFFSET);
  flags &= IFLAG1_TX;

  /* TODO First Process Error aborts */

  /* Process TX completions */

  mb_bit = 1 << RXMBCOUNT;
  for (mbi = 0; flags && mbi < TXMBCOUNT; mbi++)
    {
      if (flags & mb_bit)
        {
          putreg32(mb_bit, priv->base + IMXRT_CAN_IFLAG1_OFFSET);
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
 * Function: imxrt_txdone_work
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

static void imxrt_txdone_work(void *arg)
{
  struct imxrt_driver_s *priv = (struct imxrt_driver_s *)arg;

  imxrt_txdone(priv);

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  net_lock();
  devif_poll(&priv->dev, imxrt_txpoll);
  net_unlock();
}

/****************************************************************************
 * Function: imxrt_flexcan_interrupt
 *
 * Description:
 *   Three interrupt sources will vector this this function:
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

static int imxrt_flexcan_interrupt(int irq, void *context,
                                     void *arg)
{
  struct imxrt_driver_s *priv = (struct imxrt_driver_s *)arg;

  if (irq == priv->config->irq)
    {
      uint32_t flags;
      flags  = getreg32(priv->base + IMXRT_CAN_IFLAG1_OFFSET);
      flags &= IFLAG1_RX;

      if (flags)
        {
          /* Process immediately since scheduling a workqueue is too slow
           * which causes us to drop CAN frames
           */

          imxrt_receive(priv, flags);
        }

      flags  = getreg32(priv->base + IMXRT_CAN_IFLAG1_OFFSET);
      flags &= IFLAG1_TX;

      if (flags)
        {
          /* Disable further TX MB CAN interrupts. here can be no race
           * condition here.
           */

          flags  = getreg32(priv->base + IMXRT_CAN_IMASK1_OFFSET);
          flags &= ~(IFLAG1_TX);
          putreg32(flags, priv->base + IMXRT_CAN_IMASK1_OFFSET);
          work_queue(CANWORK, &priv->irqwork, imxrt_txdone_work, priv, 0);
        }
    }

  return OK;
}

/****************************************************************************
 * Function: imxrt_txtimeout_work
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

static void imxrt_txtimeout_work(void *arg)
{
  struct imxrt_driver_s *priv = (struct imxrt_driver_s *)arg;
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

  flags  = getreg32(priv->base + IMXRT_CAN_IFLAG1_OFFSET);

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
              putreg32(mb_bit, priv->base + IMXRT_CAN_IFLAG1_OFFSET);
            }

          struct mb_s *mb = &priv->tx[mbi];
          mb->cs.code = CAN_TXMB_ABORT;
          priv->txmb[mbi].pending = TX_ABORT;
        }
    }
}

/****************************************************************************
 * Function: imxrt_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Input Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void imxrt_txtimeout_expiry(int argc, uint32_t arg, ...)
{
  struct imxrt_driver_s *priv = (struct imxrt_driver_s *)arg;

  /* Schedule to perform the TX timeout processing on the worker thread
   */

  work_queue(CANWORK, &priv->irqwork, imxrt_txtimeout_work, priv, 0);
}

#endif

static void imxrt_setenable(uint32_t base, uint32_t enable)
{
  uint32_t regval;

  if (enable)
    {
      regval  = getreg32(base + IMXRT_CAN_MCR_OFFSET);
      regval &= ~(CAN_MCR_MDIS);
      putreg32(regval, base + IMXRT_CAN_MCR_OFFSET);
    }
  else
    {
      regval  = getreg32(base + IMXRT_CAN_MCR_OFFSET);
      regval |= CAN_MCR_MDIS;
      putreg32(regval, base + IMXRT_CAN_MCR_OFFSET);
    }

  imxrt_waitmcr_change(base, CAN_MCR_LPMACK, 1);
}

static void imxrt_setfreeze(uint32_t base, uint32_t freeze)
{
  uint32_t regval;
  if (freeze)
    {
      /* Enter freeze mode */

      regval  = getreg32(base + IMXRT_CAN_MCR_OFFSET);
      regval |= (CAN_MCR_HALT | CAN_MCR_FRZ);
      putreg32(regval, base + IMXRT_CAN_MCR_OFFSET);
    }
  else
    {
      /* Exit freeze mode */

      regval  = getreg32(base + IMXRT_CAN_MCR_OFFSET);
      regval &= ~(CAN_MCR_HALT | CAN_MCR_FRZ);
      putreg32(regval, base + IMXRT_CAN_MCR_OFFSET);
    }
}

static uint32_t imxrt_waitmcr_change(uint32_t base, uint32_t mask,
                                       uint32_t target_state)
{
  const uint32_t timeout = 1000;
  uint32_t wait_ack;

  for (wait_ack = 0; wait_ack < timeout; wait_ack++)
    {
      const bool state = (getreg32(base + IMXRT_CAN_MCR_OFFSET) & mask)
          != 0;
      if (state == target_state)
        {
          return true;
        }

      up_udelay(10);
    }

  return false;
}

static uint32_t imxrt_waitfreezeack_change(uint32_t base,
                                             uint32_t target_state)
{
  return imxrt_waitmcr_change(base, CAN_MCR_FRZACK, target_state);
}

static struct mb_s *flexcan_get_mb(struct imxrt_driver_s *priv,
                                    uint32_t mbi)
{
  return (struct mb_s *)(priv->base +
                        (mb_address[priv->mb_address_offset + mbi] << 3));
}

/****************************************************************************
 * Function: imxrt_ifup
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

static int imxrt_ifup(struct net_driver_s *dev)
{
  struct imxrt_driver_s *priv =
    (struct imxrt_driver_s *)dev->d_private;

  if (!imxrt_initialize(priv))
    {
      nerr("initialize failed");
      return -1;
    }

  priv->bifup = true;
  priv->txdesc = (struct can_frame *)&g_tx_pool;
  priv->rxdesc = (struct can_frame *)&g_rx_pool;
  if (priv->canfd_capable)
    {
      priv->txdesc_fd = (struct canfd_frame *)&g_tx_pool;
      priv->rxdesc_fd = (struct canfd_frame *)&g_rx_pool;
      priv->dev.d_buf = (uint8_t *)priv->txdesc_fd;
    }
  else
    {
      priv->dev.d_buf = (uint8_t *)priv->txdesc;
    }

  /* Set interrupts */

  up_enable_irq(priv->config->irq);

  return OK;
}

/****************************************************************************
 * Function: imxrt_ifdown
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

static int imxrt_ifdown(struct net_driver_s *dev)
{
  struct imxrt_driver_s *priv =
    (struct imxrt_driver_s *)dev->d_private;

  imxrt_reset(priv);

  priv->bifup = false;
  return OK;
}

/****************************************************************************
 * Function: imxrt_txavail_work
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

static void imxrt_txavail_work(void *arg)
{
  struct imxrt_driver_s *priv = (struct imxrt_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      if (!imxrt_txringfull(priv))
        {
          /* No, there is space for another transfer.  Poll the network for
           * new XMIT data.
           */

          devif_poll(&priv->dev, imxrt_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: imxrt_txavail
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

static int imxrt_txavail(struct net_driver_s *dev)
{
  struct imxrt_driver_s *priv =
    (struct imxrt_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      imxrt_txavail_work(priv);
    }

  return OK;
}

/****************************************************************************
 * Function: imxrt_ioctl
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
static int imxrt_ioctl(struct net_driver_s *dev, int cmd,
                         unsigned long arg)
{
  struct imxrt_driver_s *priv =
      (struct imxrt_driver_s *)dev->d_private;
  struct flexcan_timeseg data_timing;
  int ret;

  switch (cmd)
    {
      case SIOCGCANBITRATE: /* Get bitrate from a CAN controller */
        {
          struct can_ioctl_data_s *req =
              (struct can_ioctl_data_s *)((uintptr_t)arg);
          req->arbi_bitrate = priv->arbi_timing.bitrate / 1000; /* kbit/s */
          req->arbi_samplep = priv->arbi_timing.samplep;
          if (priv->canfd_capable)
            {
              req->data_bitrate = priv->data_timing.bitrate / 1000; /* kbit/s */
              req->data_samplep = priv->data_timing.samplep;
            }
          else
            {
              req->data_bitrate = 0;
              req->data_samplep = 0;
            }

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

          if (imxrt_bitratetotimeseg(&arbi_timing, 10, 0))
            {
              ret = OK;
            }
          else
            {
              ret = -EINVAL;
            }

          if (priv->canfd_capable)
          {
            data_timing.bitrate = req->data_bitrate * 1000;
            data_timing.samplep = req->data_samplep;

            if (ret == OK && imxrt_bitratetotimeseg(&data_timing, 10, 1))
              {
                ret = OK;
              }
            else
              {
                ret = -EINVAL;
              }
          }

          if (ret == OK)
            {
              /* Reset CAN controller and start with new timings */

              priv->arbi_timing = arbi_timing;
              if (priv->canfd_capable)
              {
                priv->data_timing = data_timing;
              }

              imxrt_ifup(dev);
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
 * Function: imxrt_initalize
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

static int imxrt_initialize(struct imxrt_driver_s *priv)
{
  uint32_t regval;
  uint32_t i;

  /* initialize CAN device */

#ifdef CONFIG_IMXRT_FLEXCAN3
  imxrt_setenable(priv->base, 0);

  /* Set SYS_CLOCK src */

  regval  = getreg32(priv->base + IMXRT_CAN_CTRL1_OFFSET);
  regval |= (CAN_CTRL1_CLKSRC);
  putreg32(regval, priv->base + IMXRT_CAN_CTRL1_OFFSET);
#endif

  imxrt_setenable(priv->base, 1);

  imxrt_reset(priv);

  /* Enter freeze mode */

  imxrt_setfreeze(priv->base, 1);
  if (!imxrt_waitfreezeack_change(priv->base, 1))
    {
      ninfo("FLEXCAN: freeze fail\n");
      return -1;
    }

  if (!priv->canfd_capable)
    {
      regval  = getreg32(priv->base + IMXRT_CAN_CTRL1_OFFSET);
      regval &= ~(CAN_CTRL1_PRESDIV_MASK | CAN_CTRL1_PROPSEG_MASK |
                  CAN_CTRL1_PSEG1_MASK | CAN_CTRL1_PSEG2_MASK |
                 CAN_CTRL1_RJW_MASK);
      regval |= CAN_CTRL1_PRESDIV(priv->arbi_timing.presdiv) | /* Prescaler divisor factor */
                CAN_CTRL1_PROPSEG(priv->arbi_timing.propseg) | /* Propagation segment */
                CAN_CTRL1_PSEG1(priv->arbi_timing.pseg1) |     /* Phase buffer segment 1 */
                CAN_CTRL1_PSEG2(priv->arbi_timing.pseg2) |     /* Phase buffer segment 2 */
                CAN_CTRL1_RJW(1);                              /* Resynchronization jump width */
      putreg32(regval, priv->base + IMXRT_CAN_CTRL1_OFFSET);
    }
  else
    {
      regval  = getreg32(priv->base + IMXRT_CAN_CBT_OFFSET);
      regval &= ~(CAN_CBT_EPRESDIV_MASK | CAN_CBT_EPROPSEG_MASK |
                  CAN_CBT_EPSEG1_MASK | CAN_CBT_EPSEG2_MASK |
                  CAN_CBT_ERJW_MASK);
      regval |= CAN_CBT_BTF |                                 /* Enable extended bit timing
                                                               * configurations for CAN-FD for setting up
                                                               * separately nominal and data phase */
                CAN_CBT_EPRESDIV(priv->arbi_timing.presdiv) | /* Prescaler divisor factor */
                CAN_CBT_EPROPSEG(priv->arbi_timing.propseg) | /* Propagation segment */
                CAN_CBT_EPSEG1(priv->arbi_timing.pseg1) |     /* Phase buffer segment 1 */
                CAN_CBT_EPSEG2(priv->arbi_timing.pseg2) |     /* Phase buffer segment 2 */
                CAN_CBT_ERJW(1);                              /* Resynchronization jump width */
      putreg32(regval, priv->base + IMXRT_CAN_CBT_OFFSET);

      /* Enable CAN FD feature */

      regval  = getreg32(priv->base + IMXRT_CAN_MCR_OFFSET);
      regval |= CAN_MCR_FDEN;
      putreg32(regval, priv->base + IMXRT_CAN_MCR_OFFSET);

      regval  = getreg32(priv->base + IMXRT_CAN_FDCBT_OFFSET);
      regval &= ~(CAN_FDCBT_FPRESDIV_MASK | CAN_FDCBT_FPROPSEG_MASK |
                  CAN_FDCBT_FPSEG1_MASK | CAN_FDCBT_FPSEG2_MASK |
                 CAN_FDCBT_FRJW_MASK);
      regval |= CAN_FDCBT_FPRESDIV(priv->data_timing.presdiv) |  /* Prescaler divisor factor of 1 */
                CAN_FDCBT_FPROPSEG(priv->data_timing.propseg) |  /* Propagation
                                                                  * segment (only register that doesn't add 1) */
                CAN_FDCBT_FPSEG1(priv->data_timing.pseg1) |      /* Phase buffer segment 1 */
                CAN_FDCBT_FPSEG2(priv->data_timing.pseg2) |      /* Phase buffer segment 2 */
                CAN_FDCBT_FRJW(priv->data_timing.pseg2);         /* Resynchorinzation jump width same as PSEG2 */
      putreg32(regval, priv->base + IMXRT_CAN_FDCBT_OFFSET);

      /* Additional CAN-FD configurations */

      regval  = getreg32(priv->base + IMXRT_CAN_FDCTRL_OFFSET);

      regval |= CAN_FDCTRL_FDRATE |     /* Enable bit rate switch in data phase of frame */
                CAN_FDCTRL_TDCEN |      /* Enable transceiver delay compensation */
                CAN_FDCTRL_TDCOFF(5) |  /* Setup 5 cycles for data phase sampling delay */
                CAN_FDCTRL_MSBSR0(3) |  /* Setup 64 bytes per MB 0-6 */
                CAN_FDCTRL_MSBSR1(3);   /* Setup 64 bytes per MB 7-13 */
      putreg32(regval, priv->base + IMXRT_CAN_FDCTRL_OFFSET);

      regval  = getreg32(priv->base + IMXRT_CAN_CTRL2_OFFSET);
      regval |= CAN_CTRL2_ISOCANFDEN;
      putreg32(regval, priv->base + IMXRT_CAN_CTRL2_OFFSET);
    }

  /*  Errata ER005829 step 7: Reserve first TX MB
   *  Errata mentions writng 0x8 value, but this one couses
   *  the ESR2_LPTM register to choose the reserved MB for
   *  transmiting the package, hence we write 0x3
   */

      struct mb_s *buffer = flexcan_get_mb(priv, RXMBCOUNT);
      buffer->cs.code = 0x3;

  for (i = RXMBCOUNT + 1; i < TOTALMBCOUNT; i++)
    {
      struct mb_s *rx = flexcan_get_mb(priv, i);
      rx->id.w = 0x0;

      /* FIXME sometimes we get a hard fault here */
    }

  putreg32(0x0, priv->base + IMXRT_CAN_RXFGMASK_OFFSET);

  for (i = 0; i < TOTALMBCOUNT; i++)
    {
      putreg32(0, priv->base + IMXRT_CAN_RXIMR_OFFSET(i));
    }

  for (i = 0; i < RXMBCOUNT; i++)
    {
      struct mb_s *rx = flexcan_get_mb(priv, i);
      ninfo("Set MB%" PRIi32 " to receive %p\n", i, rx);
      rx->cs.edl = 0x1;
      rx->cs.brs = 0x1;
      rx->cs.esi = 0x0;
      rx->cs.code = 4;
      rx->cs.srr = 0x0;
      rx->cs.ide = 0x1;
      rx->cs.rtr = 0x0;
    }

  putreg32(IFLAG1_RX, priv->base + IMXRT_CAN_IFLAG1_OFFSET);
  putreg32(IFLAG1_RX, priv->base + IMXRT_CAN_IMASK1_OFFSET);

  /* Exit freeze mode */

  imxrt_setfreeze(priv->base, 0);
  if (!imxrt_waitfreezeack_change(priv->base, 0))
    {
      ninfo("FLEXCAN: unfreeze fail\n");
      return -1;
    }

  return 1;
}

/****************************************************************************
 * Function: imxrt_reset
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

static void imxrt_reset(struct imxrt_driver_s *priv)
{
  uint32_t regval;
  uint32_t i;

  regval  = getreg32(priv->base + IMXRT_CAN_MCR_OFFSET);
  regval |= CAN_MCR_SOFTRST;
  putreg32(regval, priv->base + IMXRT_CAN_MCR_OFFSET);

  if (!imxrt_waitmcr_change(priv->base, CAN_MCR_SOFTRST, 0))
    {
      nerr("Reset failed");
      return;
    }

  regval  = getreg32(priv->base + IMXRT_CAN_MCR_OFFSET);
  regval &= ~(CAN_MCR_SUPV);
  putreg32(regval, priv->base + IMXRT_CAN_MCR_OFFSET);

  /* Initialize all MB rx and tx */

  for (i = 0; i < TOTALMBCOUNT; i++)
    {
      struct mb_s *rx = flexcan_get_mb(priv, i);
      ninfo("MB %" PRIi32 " %p\n", i, rx);
      ninfo("MB %" PRIi32 " %p\n", i, &rx->id.w);
      rx->cs.cs = 0x0;
      rx->id.w = 0x0;
      rx->data[0].w00 = 0x0;
      rx->data[1].w00 = 0x0;
    }

  regval  = getreg32(priv->base + IMXRT_CAN_MCR_OFFSET);
  regval |= CAN_MCR_SLFWAK | CAN_MCR_WRNEN | CAN_MCR_SRXDIS |
            CAN_MCR_IRMQ | CAN_MCR_AEN |
            (((TOTALMBCOUNT - 1) << CAN_MCR_MAXMB_SHIFT) &
            CAN_MCR_MAXMB_MASK);
  putreg32(regval, priv->base + IMXRT_CAN_MCR_OFFSET);

  regval  = CAN_CTRL2_RRS | CAN_CTRL2_EACEN;
  putreg32(regval, priv->base + IMXRT_CAN_CTRL2_OFFSET);

  for (i = 0; i < TOTALMBCOUNT; i++)
    {
      putreg32(0, priv->base + IMXRT_CAN_RXIMR_OFFSET(i));
    }

  /* Filtering catchall */

  putreg32(0x3fffffff, priv->base + IMXRT_CAN_RX14MASK_OFFSET);
  putreg32(0x3fffffff, priv->base + IMXRT_CAN_RX15MASK_OFFSET);
  putreg32(0x3fffffff, priv->base + IMXRT_CAN_RXMGMASK_OFFSET);
  putreg32(0x0, priv->base + IMXRT_CAN_RXFGMASK_OFFSET);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: imxrt_caninitialize
 *
 * Description:
 *   Initialize the CAN controller and driver
 *
 * Input Parameters:
 *   intf - In the case where there are multiple CAN, this value
 *          identifies which CAN is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int imxrt_caninitialize(int intf)
{
  struct imxrt_driver_s *priv;
  int ret;
#ifdef TX_TIMEOUT_WQ
  uint32_t i;
#endif

  switch (intf)
    {
#ifdef CONFIG_IMXRT_FLEXCAN1
    case 1:
      imxrt_clockall_can1();
      imxrt_clockall_can1_serial();
      priv               = &g_flexcan1;
      memset(priv, 0, sizeof(struct imxrt_driver_s));
      priv->base         = IMXRT_CAN1_BASE;
      priv->config       = &imxrt_flexcan1_config;
      priv->canfd_capable = false;
      priv->mb_address_offset = 0;

      /* Default bitrate configuration */

      priv->arbi_timing.bitrate = CONFIG_FLEXCAN1_BITRATE;
      priv->arbi_timing.samplep = CONFIG_FLEXCAN1_SAMPLEP;
      break;
#endif

#ifdef CONFIG_IMXRT_FLEXCAN2
    case 2:
      imxrt_clockall_can2();
      imxrt_clockall_can2_serial();
      priv         = &g_flexcan2;
      memset(priv, 0, sizeof(struct imxrt_driver_s));
      priv->base   = IMXRT_CAN2_BASE;
      priv->config = &imxrt_flexcan2_config;
      priv->canfd_capable = false;
      priv->mb_address_offset = 0;

      /* Default bitrate configuration */

      priv->arbi_timing.bitrate = CONFIG_FLEXCAN2_BITRATE;
      priv->arbi_timing.samplep = CONFIG_FLEXCAN2_SAMPLEP;
      break;
#endif

#ifdef CONFIG_IMXRT_FLEXCAN3
    case 3:
      imxrt_clockall_can3();
      imxrt_clockall_can3_serial();
      priv         = &g_flexcan3;
      memset(priv, 0, sizeof(struct imxrt_driver_s));
      priv->base   = IMXRT_CAN3_BASE;
      priv->config = &imxrt_flexcan3_config;
#  ifdef CONFIG_NET_CAN_CANFD
      priv->canfd_capable = true;
      priv->mb_address_offset = 14;
#  else
      priv->canfd_capable = false;
      priv->mb_address_offset = 0;
#  endif

      /* Default bitrate configuration */

#  ifdef CONFIG_NET_CAN_CANFD
      priv->arbi_timing.bitrate = CONFIG_FLEXCAN3_ARBI_BITRATE;
      priv->arbi_timing.samplep = CONFIG_FLEXCAN3_ARBI_SAMPLEP;
      priv->data_timing.bitrate = CONFIG_FLEXCAN3_DATA_BITRATE;
      priv->data_timing.samplep = CONFIG_FLEXCAN3_DATA_SAMPLEP;
#  else
      priv->arbi_timing.bitrate = CONFIG_FLEXCAN3_BITRATE;
      priv->arbi_timing.samplep = CONFIG_FLEXCAN3_SAMPLEP;
#  endif
      break;
#endif

    default:
      return -ENODEV;
    }

  if (!imxrt_bitratetotimeseg(&priv->arbi_timing, 1, 0))
    {
      nerr("ERROR: Invalid CAN timings please try another sample point "
           "or refer to the reference manual\n");
      return -1;
    }

  if (priv->canfd_capable)
    {
      if (!imxrt_bitratetotimeseg(&priv->data_timing, 1, 1))
        {
          nerr("ERROR: Invalid CAN data phase timings please try another "
               "sample point or refer to the reference manual\n");
          return -1;
        }
    }

  imxrt_config_gpio(priv->config->tx_pin);
  imxrt_config_gpio(priv->config->rx_pin);

  if (irq_attach(priv->config->irq, imxrt_flexcan_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach CAN bus IRQ\n");
      return -EAGAIN;
    }

  /* Initialize the driver structure */

  priv->dev.d_ifup    = imxrt_ifup;      /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = imxrt_ifdown;    /* I/F down callback */
  priv->dev.d_txavail = imxrt_txavail;   /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = imxrt_ioctl;     /* Support CAN ioctl() calls */
#endif
  priv->dev.d_private = (void *)priv;      /* Used to recover private state from dev */

#ifdef TX_TIMEOUT_WQ
  for (i = 0; i < TXMBCOUNT; i++)
    {
      priv->txtimeout[i] = wd_create();    /* Create TX timeout timer */
    }

#endif

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling imxrt_ifdown().
   */

  ninfo("callbacks done\n");

  imxrt_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_CAN);

  UNUSED(ret);
  return OK;
}

/****************************************************************************
 * Name: arm_caninitialize
 *
 * Description:
 *   Initialize the first network interface.  If there are more than one
 *   interface in the chip, then board-specific logic will have to provide
 *   this function to determine which, if any, Ethernet controllers should
 *   be initialized.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
#ifdef CONFIG_IMXRT_FLEXCAN1
  imxrt_caninitialize(1);
#endif

#ifdef CONFIG_IMXRT_FLEXCAN2
  imxrt_caninitialize(2);
#endif

#ifdef CONFIG_IMXRT_FLEXCAN3
  imxrt_caninitialize(3);
#endif
}
#endif

#endif /* CONFIG_IMXRT_FLEXCAN */

/****************************************************************************
 * arch/arm64/src/imx9/imx9_flexcan.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/spinlock.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/can.h>

#include "arm64_internal.h"
#include "imx9_flexcan.h"
#include "imx9_ccm.h"
#include "imx9_iomuxc.h"
#include "hardware/imx9_ccm.h"
#include "hardware/imx9_pinmux.h"

#include <arch/barriers.h>
#include <arch/board/board.h>

#include <sys/time.h>

#ifdef CONFIG_IMX9_FLEXCAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#define CANWORK    LPWORK
#define CANRCVWORK HPWORK

#define RXMBCOUNT                   CONFIG_IMX9_FLEXCAN_RXMB
#define TXMBCOUNT                   CONFIG_IMX9_FLEXCAN_TXMB
#define TOTALMBCOUNT                (RXMBCOUNT + TXMBCOUNT)

#define IFLAG1_RX                   ((1 << RXMBCOUNT)-1)
#define IFLAG1_TX                   (((1 << TXMBCOUNT)-1) << RXMBCOUNT)

#define POOL_SIZE                   1

#define MSG_DATA                    sizeof(struct timeval)

#define TSEG_MIN                    2

/* Classical can CTRL1 bit timing */

#define TSEG1_MAX                   16
#define TSEG2_MAX                   8
#define NUMTQ_MIN                   8
#define NUMTQ_MAX                   25

/* CAN FD CBT and FDCBT bit timing */

#define TSEG1_FD_MAX                96
#define TSEG2_FD_MAX                32
#define NUMTQ_FD_MIN                8
#define NUMTQ_FD_MAX                129

#define TSEG1_FD_DATAMAX            39
#define TSEG2_FD_DATAMAX            8
#define NUMTQ_FD_DATAMIN            5
#define NUMTQ_FD_DATAMAX            48

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE

#  if !defined(CONFIG_SCHED_WORKQUEUE)
#    error Work queue support is required
#  endif

#define TX_TIMEOUT_WQ
#endif

#if (CONFIG_IMX9_FLEXCAN_RXMB + CONFIG_IMX9_FLEXCAN_TXMB) > 21
# error Only 21 MB are allowed to be used
#endif

#ifdef CONFIG_NET_CAN_CANFD
#  define TX_POOL_SIZE ((sizeof(struct canfd_frame) + MSG_DATA) * POOL_SIZE)
#  define RX_POOL_SIZE ((sizeof(struct canfd_frame) + MSG_DATA) * POOL_SIZE)
#else
#  define TX_POOL_SIZE (sizeof(struct can_frame) * POOL_SIZE)
#  define RX_POOL_SIZE (sizeof(struct can_frame) * POOL_SIZE)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mb_s
{
  volatile uint32_t cs;
  volatile uint32_t id;
  volatile uint32_t data[];
};

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
#define TX_ABORT -1
#define TX_FREE 0
#define TX_BUSY 1

struct txmbstats
{
  struct timeval deadline;
};
#endif

/* FlexCAN Device hardware configuration */

struct flexcan_timeseg
{
  uint32_t bitrate;
  int32_t samplep;
  uint8_t propseg;
  uint8_t pseg1;
  uint8_t pseg2;
  uint8_t presdiv;
};

/* The imx9_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct imx9_driver_s
{
  struct net_driver_s dev;            /* Interface understood by the network */

  const uintptr_t base;               /* FLEXCAN base address */
  const int irq;                      /* irq number */
  const bool canfd_capable;
  const uint32_t *txdesc;             /* A pointer to the list of TX descriptor */
  const uint32_t *rxdesc;             /* A pointer to the list of RX descriptors */
  const bool srxdis;                  /* Self reception disable */

  uint32_t clk_freq;                  /* Peripheral clock frequency */
  bool bifup;                         /* true:ifup false:ifdown */
#ifdef TX_TIMEOUT_WQ
  struct wdog_s txtimeout[TXMBCOUNT]; /* TX timeout timer */
#endif
  struct work_s rcvwork;              /* For deferring interrupt work to the wq */
  struct work_s irqwork;              /* For deferring interrupt work to the wq */
  struct work_s pollwork;             /* For deferring poll work to the work wq */
  struct flexcan_timeseg arbi_timing; /* Timing for arbitration phase */
  struct flexcan_timeseg data_timing; /* Timing for data phase */

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  struct txmbstats txmb[TXMBCOUNT];
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_IMX9_FLEXCAN1

static uint32_t g_tx_pool_can1[(TX_POOL_SIZE + 3) / 4];
static uint32_t g_rx_pool_can1[(RX_POOL_SIZE + 3) / 4];

static struct imx9_driver_s g_flexcan1 =
  {
    .base              = IMX9_CAN1_BASE,
    .irq               = IMX9_IRQ_CAN1,
#  if defined(CONFIG_NET_CAN_CANFD)
    .canfd_capable     = true,
#  else
    .canfd_capable     = false,
#  endif

    /* Default bitrate configuration */

#  if defined(CONFIG_NET_CAN_CANFD)
    .arbi_timing =
      {
        .bitrate = CONFIG_IMX9_FLEXCAN1_ARBI_BITRATE,
        .samplep = CONFIG_IMX9_FLEXCAN1_ARBI_SAMPLEP,
      },
    .data_timing =
      {
        .bitrate = CONFIG_IMX9_FLEXCAN1_DATA_BITRATE,
        .samplep = CONFIG_IMX9_FLEXCAN1_DATA_SAMPLEP,
      },
#  else
    .arbi_timing =
      {
        .bitrate = CONFIG_IMX9_FLEXCAN1_BITRATE,
        .samplep = CONFIG_IMX9_FLEXCAN1_SAMPLEP,
      },
#  endif

    .txdesc = g_tx_pool_can1,
    .rxdesc = g_rx_pool_can1,

#  if defined(CONFIG_IMX9_FLEXCAN1_SRXDIS)
    .srxdis = true,
#  endif
  };
#endif

#ifdef CONFIG_IMX9_FLEXCAN2

static uint32_t g_tx_pool_can2[(TX_POOL_SIZE + 3) / 4];
static uint32_t g_rx_pool_can2[(RX_POOL_SIZE + 3) / 4];

static struct imx9_driver_s g_flexcan2 =
  {
    .base              = IMX9_CAN2_BASE,
    .irq               = IMX9_IRQ_CAN2,
#  if defined(CONFIG_NET_CAN_CANFD)
    .canfd_capable     = true,
#  else
    .canfd_capable     = false,
#  endif

    /* Default bitrate configuration */

#  if defined(CONFIG_NET_CAN_CANFD)
    .arbi_timing =
      {
        .bitrate = CONFIG_IMX9_FLEXCAN2_ARBI_BITRATE,
        .samplep = CONFIG_IMX9_FLEXCAN2_ARBI_SAMPLEP,
      },
    .data_timing =
      {
        .bitrate = CONFIG_IMX9_FLEXCAN2_DATA_BITRATE,
        .samplep = CONFIG_IMX9_FLEXCAN2_DATA_SAMPLEP,
      },
#  else
    .arbi_timing =
      {
        .bitrate = CONFIG_IMX9_FLEXCAN2_BITRATE,
        .samplep = CONFIG_IMX9_FLEXCAN2_SAMPLEP,
      },
#  endif

    .txdesc = g_tx_pool_can2,
    .rxdesc = g_rx_pool_can2,

#  if defined(CONFIG_IMX9_FLEXCAN2_SRXDIS)
    .srxdis = true,
#  endif
  };
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_lsb
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

static inline uint32_t arm64_lsb(unsigned int value)
{
  uint32_t ret;
  volatile uint32_t rvalue = value;
  __asm__ __volatile__ ("rbit %w1,%w0" : "=r" (rvalue) : "r" (rvalue));
  __asm__ __volatile__ ("clz %w0, %w1" : "=r"(ret) : "r"(rvalue));
  return ret;
}

/****************************************************************************
 * Name: imx9_bitratetotimeseg
 *
 * Description:
 *   Convert bitrate to timeseg
 *
 * Input Parameters:
 *   timeseg - structure to store bit timing
 *                  bit timings (recommended value: 1)
 *   can_fd_data - if set to calculate CAN FD data bit timings,
 *                  otherwise calculate classical or arbitration can
 *                  timings
 *
 * Returned Value:
 *   return OK on success, negated error number on failure
 *
 ****************************************************************************/

static uint32_t imx9_bitratetotimeseg(struct imx9_driver_s *priv,
                                      struct flexcan_timeseg *timeseg,
                                      bool can_fd_data)
{
#if defined(CONFIG_NET_CAN_CANFD)
  /* Max SEG1 & SEG2 values in TQ */

  const int32_t TSEG1MAX = can_fd_data ? TSEG1_FD_DATAMAX : TSEG1_FD_MAX;
  const int32_t TSEG2MAX = can_fd_data ? TSEG2_FD_DATAMAX : TSEG2_FD_MAX;

  /* Min and max bit length in TQ */

  const int32_t NUMTQMIN = can_fd_data ? NUMTQ_FD_DATAMIN : NUMTQ_FD_MIN;
  const int32_t NUMTQMAX = can_fd_data ? NUMTQ_FD_DATAMAX : NUMTQ_FD_MAX;

  /* Max register field values */

  /* Max register field value for presdiv */

  const uint32_t PRESDIVMAX = can_fd_data ?
    CAN_FDCBT_FPRESDIV_MASK >> CAN_FDCBT_FPRESDIV_SHIFT :
    CAN_CBT_EPRESDIV_MASK >> CAN_CBT_EPRESDIV_SHIFT;

  /* Max register field values from PSEG and PROPSEG */

  const int32_t PSEGMAX = can_fd_data ?
    CAN_FDCBT_FPSEG1_MASK >> CAN_FDCBT_FPSEG1_SHIFT :
    CAN_CBT_EPSEG1_MASK >> CAN_CBT_EPSEG1_SHIFT;
  const int32_t PROPSEGMAX = can_fd_data ?
    (CAN_FDCBT_FPROPSEG_MASK >> CAN_FDCBT_FPROPSEG_SHIFT) - 1 :
    CAN_CBT_EPROPSEG_MASK >> CAN_CBT_EPROPSEG_SHIFT;
#else
  /* Max SEG1 & SEG2 values in TQ */

  const int32_t TSEG1MAX = TSEG1_MAX;
  const int32_t TSEG2MAX = TSEG2_MAX;

  /* Min and max bit length in TQ */

  const int32_t NUMTQMIN = NUMTQ_MIN;
  const int32_t NUMTQMAX = NUMTQ_MAX;

  /* Max register field values */

  /* Max register field value for presdiv */

  const uint32_t PRESDIVMAX =
    CAN_CTRL1_PRESDIV_MASK >> CAN_CTRL1_PRESDIV_SHIFT;

  /* Max register field values from PSEG and PROPSEG */

  const int32_t PSEGMAX = CAN_CTRL1_PSEG1_MASK >> CAN_CTRL1_PSEG1_SHIFT;
  const int32_t PROPSEGMAX =
    CAN_CTRL1_PROPSEG_MASK >> CAN_CTRL1_PROPSEG_SHIFT;
#endif

  int32_t presdiv = PRESDIVMAX;
  int32_t tmppresdiv;
  int32_t numtq;
  int32_t tmpnumtq;
  int32_t tmpsample;
  int32_t tseg1;
  int32_t tseg2;
  int32_t tmppseg1;
  int32_t tmppseg2;
  int32_t tmppropseg;
  int32_t bitrate = 0;

  int32_t bitrate_tmp = 0;
  int32_t bitrate_err = INT32_MAX;

  for (tmppresdiv = 0; tmppresdiv < PRESDIVMAX; tmppresdiv++)
    {
      tmpnumtq = (priv->clk_freq / ((tmppresdiv + 1) * timeseg->bitrate));

      /* if number of time quanta per bit is too high, continue */

      if (tmpnumtq > NUMTQMAX)
        {
          continue;
        }

      /* if number of time quanta per bit is too small, break out */

      if (tmpnumtq < NUMTQMIN)
        {
          break;
        }

      bitrate_tmp = priv->clk_freq / ((tmppresdiv + 1) * tmpnumtq);
      if (abs(bitrate - bitrate_tmp) < bitrate_err)
        {
          bitrate_err = abs(bitrate - bitrate_tmp);
          bitrate = bitrate_tmp;
          numtq = tmpnumtq;
          presdiv = tmppresdiv;
        }
    }

  if (bitrate != timeseg->bitrate)
    {
      canwarn("bitrate set to %" PRId32 " instead of %" PRId32 "\n",
              bitrate, timeseg->bitrate);
    }

  /* Compute time segments based on the value of the sampling point */

  tseg1 = (numtq * timeseg->samplep / 100) - 1;
  tseg2 = numtq - 1 - tseg1;

  /* Adjust time segment 1 and time segment 2 */

  while (tseg1 >= TSEG1MAX || tseg2 < TSEG_MIN)
    {
      tseg2++;
      tseg1--;
    }

  if (tseg1 > TSEG1MAX || tseg2 > TSEG2MAX ||
      tseg2 < TSEG_MIN || tseg1 < TSEG_MIN)
    {
      canerr("tseg1 %" PRId32 ", max %" PRId32 "\n", tseg1, TSEG1MAX);
      canerr("tseg2 %" PRId32 ", max %" PRId32 "\n", tseg2, TSEG2MAX);
      return -EINVAL;
    }

  DEBUGASSERT(1 + tseg1 + tseg2 == numtq);

  tmppseg2 = tseg2 - 1;

  /* Start from pseg1 = pseg2 and adjust until propseg is valid */

  tmppseg1 = tmppseg2;
  tmppropseg = tseg1 - tmppseg1 - 2;

  while (tmppropseg <= 0)
    {
      tmppropseg++;
      tmppseg1--;
    }

  while (tmppropseg >= PROPSEGMAX)
    {
      tmppropseg--;
      tmppseg1++;
    }

  if (tmppseg1 > PSEGMAX || tmppseg2 > PSEGMAX)
    {
      canerr("tmppseg1 %" PRId32 ", max %" PRId32 "\n", tmppseg1, PSEGMAX);
      canerr("tmppseg2 %" PRId32 ", max %" PRId32 "\n", tmppseg2, PSEGMAX);
      return -EINVAL;
    }

  tmpsample = (1 + tseg1) * 100 / numtq;

  /* Allow 5% tolerance in sample point */

  if (abs(tmpsample - timeseg->samplep) <= 5)
    {
      if (can_fd_data)
        {
          timeseg->propseg = tmppropseg + 1;
        }
      else
        {
          timeseg->propseg = tmppropseg;
        }

      timeseg->pseg1 = tmppseg1;
      timeseg->pseg2 = tmppseg2;
      timeseg->presdiv = presdiv;
      timeseg->samplep = tmpsample;

      return OK;
    }

  canerr("sample point %" PRId32 ", configured %" PRId32 "\n",
         tmpsample, timeseg->samplep);

  return -EINVAL;
}

/* Common TX logic */

static bool imx9_txringfull(struct imx9_driver_s *priv);
static int  imx9_transmit(struct imx9_driver_s *priv);
static int  imx9_txpoll(struct net_driver_s *dev);

/* Helper functions */

static bool imx9_setenable(uint32_t base, bool enable);
static bool imx9_setfreeze(uint32_t base, bool freeze);
static bool imx9_waitmcr_change(uint32_t base,
                                uint32_t mask,
                                bool target_state);
static volatile struct mb_s *flexcan_get_mb(struct imx9_driver_s *priv,
                                            int mbi);

/* Interrupt handling */

static void imx9_rxdone_work(void *arg);
static void imx9_receive(struct imx9_driver_s *priv);
static void imx9_txdone_work(void *arg);
static void imx9_txdone(struct imx9_driver_s *priv, uint32_t flags);

static int  imx9_flexcan_interrupt(int irq, void *context,
                                   void *arg);

/* Watchdog timer expirations */
#ifdef TX_TIMEOUT_WQ
static void imx9_txtimeout_work(void *arg);
static void imx9_txtimeout_expiry(wdparm_t arg);
#endif

/* NuttX callback functions */

static int  imx9_ifup(struct net_driver_s *dev);
static int  imx9_ifdown(struct net_driver_s *dev);

static void imx9_txavail_work(void *arg);
static int  imx9_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NETDEV_IOCTL
static int  imx9_ioctl(struct net_driver_s *dev, int cmd,
                       unsigned long arg);
#endif

/* Initialization */

static int  imx9_initialize(struct imx9_driver_s *priv);
static void imx9_reset(struct imx9_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: imx9_txringfull
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

static bool imx9_txringfull(struct imx9_driver_s *priv)
{
  int mbi;

  for (mbi = RXMBCOUNT; mbi < TOTALMBCOUNT; mbi++)
    {
      volatile struct mb_s *mb = flexcan_get_mb(priv, mbi);
      if (CAN_MB_CS_CODE(mb->cs) != CAN_TXMB_DATAORREMOTE)
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Function: imx9_transmit
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

static int imx9_transmit(struct imx9_driver_s *priv)
{
  volatile struct mb_s *mb;
  uint32_t mbi = 0;
  uint32_t *frame_data_word;
  uint32_t i;
#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  int32_t timeout;
  uint32_t txmb = 0;
#endif
  uint32_t cs = 0;
  canid_t can_id;
  uint32_t can_dlc;
  uint8_t len;

  for (mbi = RXMBCOUNT; mbi < TOTALMBCOUNT; mbi++)
    {
      /* Check whether message buffer is not currently transmitting */

      mb = flexcan_get_mb(priv, mbi);
      if (CAN_MB_CS_CODE(mb->cs) != CAN_TXMB_DATAORREMOTE)
        {
          break;
        }

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
      txmb++;
#endif
    }

  if (mbi >= TOTALMBCOUNT)
    {
      nwarn("No TX MB available mbi %" PRIi32 "\n", mbi);
      NETDEV_TXERRORS(&priv->dev);
      return ERROR;
    }

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  struct timespec ts;
  clock_systime_timespec(&ts);

  if (priv->dev.d_sndlen > priv->dev.d_len)
    {
      struct timeval *tv =
             (struct timeval *)(priv->dev.d_buf + priv->dev.d_len);

      priv->txmb[txmb].deadline = *tv;
      timeout  = (tv->tv_sec - ts.tv_sec)*CLK_TCK
                 + ((tv->tv_usec - ts.tv_nsec / 1000)*CLK_TCK) / 1000000;
      if (timeout < 0)
        {
          return ERROR;
        }
    }
  else
    {
      /* Default TX deadline defined in NET_CAN_RAW_DEFAULT_TX_DEADLINE */

      if (CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE > 0)
        {
          timeout = ((CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE / 1000000)
              *CLK_TCK);
          priv->txmb[txmb].deadline.tv_sec = ts.tv_sec +
              CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE / 1000000;
          priv->txmb[txmb].deadline.tv_usec = (ts.tv_nsec / 1000) +
              CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE % 1000000;
        }
      else
        {
          priv->txmb[txmb].deadline.tv_sec = 0;
          priv->txmb[txmb].deadline.tv_usec = 0;
          timeout = -1;
        }
    }
#endif

  if (priv->dev.d_len == sizeof(struct can_frame))
    {
      struct can_frame *frame = (struct can_frame *)priv->dev.d_buf;
      can_id = frame->can_id;
      len = 8;
      can_dlc = frame->can_dlc;
      frame_data_word = (uint32_t *)&frame->data[0];
    }
#ifdef CONFIG_NET_CAN_CANFD
  else
    {
      struct canfd_frame *frame = (struct canfd_frame *)priv->dev.d_buf;
      cs |= CAN_MB_CS_EDL;
      cs |= frame->flags & CANFD_BRS ? CAN_MB_CS_BRS : 0;
      can_id = frame->can_id;
      len = frame->len;
      can_dlc = g_len_to_can_dlc[len];
      frame_data_word = (uint32_t *)&frame->data[0];
    }
#endif

  if (can_id & CAN_EFF_FLAG)
    {
      cs |= CAN_MB_CS_IDE;
      mb->id = can_id & CAN_MB_ID_ID_MASK;
    }
  else
    {
      mb->id = ((can_id & CAN_SFF_MASK) << CAN_MB_ID_ID_STD_SHIFT) &
        CAN_MB_ID_ID_STD_MASK;
    }

  cs |= (can_id & CAN_RTR_FLAG) ? CAN_MB_CS_RTR : 0;
  cs |= (can_dlc << CAN_MB_CS_DLC_SHIFT) & CAN_MB_CS_DLC_MASK;

  for (i = 0; i < (len + 4 - 1) / 4; i++)
    {
      mb->data[i] = __builtin_bswap32(frame_data_word[i]);
    }

  /* Go */

  cs |= CAN_TXMB_DATAORREMOTE << CAN_MB_CS_CODE_SHIFT;
  mb->cs = cs;

  /* Increment statistics */

  NETDEV_TXPACKETS(&priv->dev);

#ifdef TX_TIMEOUT_WQ
  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  if (timeout > 0)
    {
      wd_start(&priv->txtimeout[txmb], timeout + 1,
               imx9_txtimeout_expiry, (wdparm_t)priv);
    }
#endif

  return OK;
}

/****************************************************************************
 * Function: imx9_txpoll
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

static int imx9_txpoll(struct net_driver_s *dev)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)dev;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (dev->d_len > 0)
    {
      /* Send the packet */

      if (imx9_transmit(priv) != OK)
        {
          return -EINVAL;
        }

      /* Check if there is room in the device to hold another packet. If
       * not, return a non-zero value to terminate the poll.
       */

      if (imx9_txringfull(priv))
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
 * Function: imx9_get_oldest_mbi
 *
 * Description:
 *   Find the oldest MB in the message buffers, based on timestamp
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *   flags - Bitmask of MBs which should be checked
 *
 * Returned Value:
 *   index of the MB with oldest received data
 *
 * Assumptions:
 *   Always called with at least one RX buffer to be checked
 *
 ****************************************************************************/

static int imx9_get_oldest_mbi(struct imx9_driver_s *priv, uint32_t flags)
{
  int mbi;
  int oldest = -1;
  bool first = true;
  uint16_t t;
  uint16_t t_oldest;
  volatile struct mb_s *mb;

  /* When this is called, there is always at least one received buffer */

  DEBUGASSERT((flags & IFLAG1_RX) != 0);

  for (mbi = 0; mbi < RXMBCOUNT; mbi++)
    {
      if (flags & (1 << mbi))
        {
          mb = flexcan_get_mb(priv, mbi);
          t = CAN_MB_CS_TIMESTAMP(mb->cs);

          if ((int16_t)(t_oldest - t) > 0 || first)
            {
              first = false;
              t_oldest = t;
              oldest = mbi;
            }
        }
    }

  return oldest;
}

/****************************************************************************
 * Function: imx9_receive
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

static void imx9_receive(struct imx9_driver_s *priv)
{
  int mbi;
  volatile struct mb_s *rf;
#ifdef CONFIG_NET_CAN_CANFD
  uint32_t *frame_data_word;
  uint32_t i;
#endif
  size_t frame_len;

  uint32_t flags = getreg32(priv->base + IMX9_CAN_IFLAG1_OFFSET);
  flags &= IFLAG1_RX;

  while (flags != 0)
    {
      mbi = imx9_get_oldest_mbi(priv, flags);

      /* Make sure the MB is locked */

      do
        {
          rf = flexcan_get_mb(priv, mbi);
        }
      while ((CAN_MB_CS_CODE(rf->cs) & CAN_RXMB_BUSY_BIT) != 0);

      DEBUGASSERT(CAN_MB_CS_CODE(rf->cs) != CAN_RXMB_EMPTY);

      if (CAN_MB_CS_CODE(rf->cs) == CAN_RXMB_OVERRUN)
        {
          canwarn("RX overrun\n");
          NETDEV_RXERRORS(dev);
        }

      /* Read the frame contents */

#ifdef CONFIG_NET_CAN_CANFD
      if (rf->cs & CAN_MB_CS_EDL)
        {
          /* CAN FD frame */

          struct canfd_frame *frame = (struct canfd_frame *)priv->rxdesc;

          if (rf->cs & CAN_MB_CS_IDE)
            {
              frame->can_id = ((rf->id & CAN_MB_ID_ID_MASK) >>
                               CAN_MB_ID_ID_SHIFT);
              frame->can_id |= CAN_EFF_FLAG;
            }
          else
            {
              frame->can_id = ((rf->id & CAN_MB_ID_ID_STD_MASK) >>
                               CAN_MB_ID_ID_STD_SHIFT);
            }

          if (rf->cs & CAN_MB_CS_RTR)
            {
              frame->can_id |= CAN_RTR_FLAG;
            }

          /* Set bitrate switch by default if frame is CANFD */

          frame->flags = CANFD_BRS;
          if (rf->cs & CAN_MB_CS_ESI)
            {
              frame->flags |= CANFD_ESI;
            }

          frame->len = g_can_dlc_to_len[CAN_MB_CS_DLC(rf->cs)];

          frame_data_word = (uint32_t *)&frame->data[0];

          for (i = 0; i < (frame->len + 4 - 1) / 4; i++)
            {
              frame_data_word[i] = __builtin_bswap32(rf->data[i]);
            }

          frame_len = sizeof(struct canfd_frame);
        }
      else
#endif
        {
          /* CAN 2.0 Frame */

          struct can_frame *frame = (struct can_frame *)priv->rxdesc;

          if (rf->cs & CAN_MB_CS_IDE)
            {
              frame->can_id = ((rf->id & CAN_MB_ID_ID_MASK) >>
                               CAN_MB_ID_ID_SHIFT);
              frame->can_id |= CAN_EFF_FLAG;
            }
          else
            {
              frame->can_id = ((rf->id & CAN_MB_ID_ID_STD_MASK) >>
                               CAN_MB_ID_ID_STD_SHIFT);
            }

          if (rf->cs & CAN_MB_CS_RTR)
            {
              frame->can_id |= CAN_RTR_FLAG;
            }

          frame->can_dlc = CAN_MB_CS_DLC(rf->cs);

          *(uint32_t *)&frame->data[0] = __builtin_bswap32(rf->data[0]);
          *(uint32_t *)&frame->data[4] = __builtin_bswap32(rf->data[1]);

          frame_len = sizeof(struct can_frame);
        }

      net_lock();

      /* Copy the buffer pointer to priv->dev..  Set amount of data
       * in priv->dev.d_len
       */

      priv->dev.d_buf = (uint8_t *)priv->rxdesc;
      priv->dev.d_len = frame_len;

      /* Send to socket interface */

      can_input(&priv->dev);

      net_unlock();

      /* Clear MB interrupt flag */

      putreg32(1 << mbi, priv->base + IMX9_CAN_IFLAG1_OFFSET);

      /* Re-activate the buffer */

      rf->cs = (CAN_RXMB_EMPTY << CAN_MB_CS_CODE_SHIFT) | CAN_MB_CS_IDE;

      /* Re-enable interrupt */

      modifyreg32(priv->base + IMX9_CAN_IMASK1_OFFSET, 0, 1 << mbi);

      flags &= ~(1 << mbi);
    }
}

/****************************************************************************
 * Function: imx9_txdone
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

static void imx9_txdone(struct imx9_driver_s *priv, uint32_t flags)
{
  volatile struct mb_s *mb;
  uint32_t mbi;
  uint32_t mb_bit;
#ifdef TX_TIMEOUT_WQ
  uint32_t txmb;
#endif
  int code;

  /* Process TX completions */

#ifdef TX_TIMEOUT_WQ
  txmb = 0;
#endif

  for (mbi = RXMBCOUNT; mbi < TOTALMBCOUNT; mbi++)
    {
      mb_bit = 1 << mbi;
      if (flags & mb_bit)
        {
          mb = flexcan_get_mb(priv, mbi);
          code = CAN_MB_CS_CODE(mb->cs);

          /* Clear interrupt */

          putreg32(mb_bit, priv->base + IMX9_CAN_IFLAG1_OFFSET);

          /* After RTR transmission, the MB transitions into RX MB.
           * Check for RX empty w. busy bit or RX full - this should
           * not happen.
           */

          if (((code & ~CAN_RXMB_BUSY_BIT) == CAN_RXMB_EMPTY &&
               (code & CAN_RXMB_BUSY_BIT) != 0) ||
              code == CAN_RXMB_FULL)
            {
              /* Received something in this buffer?
               * This should only happen if we sent RTR and then did
               * run out of RX MBs (which are at lower indecies).
               * Or perhaps this shouldn't happen at all when AEN=1. This
               * is unclear in the RM.
               */

              NETDEV_RXDROPPED(priv->dev);
              canerr("RCV in TX MB, code %x\n", code);
            }

          /* Only possible TX codes after transmission are ABORT or
           * INACTIVE. If it transitioned to RX MB after RTR sent,
           * inactivate it.
           */

          if (code != CAN_TXMB_ABORT && code != CAN_TXMB_INACTIVE)
            {
              mb->cs = CAN_TXMB_INACTIVE << CAN_MB_CS_CODE_SHIFT;
            }

          if (code == CAN_TXMB_ABORT)
            {
              NETDEV_TXERRORS(dev);
            }
          else
            {
              NETDEV_TXDONE(dev);
            }

#ifdef TX_TIMEOUT_WQ
          /* We are here because a transmission completed, so the
           * corresponding watchdog can be canceled
           */

          wd_cancel(&priv->txtimeout[txmb]);
#endif
        }

#ifdef TX_TIMEOUT_WQ
      txmb++;
#endif
    }

  /* Schedule worker to poll for more data */

  work_queue(CANWORK, &priv->irqwork, imx9_txdone_work, priv, 0);
}

/****************************************************************************
 * Function: imx9_txdone_work
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

static void imx9_txdone_work(void *arg)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)arg;

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  net_lock();
  devif_poll(&priv->dev, imx9_txpoll);
  net_unlock();
}

/****************************************************************************
 * Function: imx9_rxdone_work
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

static void imx9_rxdone_work(void *arg)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)arg;

  imx9_receive(priv);
}

/****************************************************************************
 * Function: imx9_flexcan_interrupt
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

static int imx9_flexcan_interrupt(int irq, void *context,
                                     void *arg)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)arg;
  uint32_t esr1;

  if (irq == priv->irq)
    {
      uint32_t flags = getreg32(priv->base + IMX9_CAN_IFLAG1_OFFSET);

      if (flags & IFLAG1_RX)
        {
          work_queue(CANRCVWORK, &priv->rcvwork, imx9_rxdone_work, priv, 0);

          /* Mask RX interrupts until handled in the work queue */

          modifyreg32(priv->base + IMX9_CAN_IMASK1_OFFSET,
                      flags & IFLAG1_RX, 0);
        }

      if (flags & IFLAG1_TX)
        {
          imx9_txdone(priv, flags);
        }
    }
  else
    {
      /* Error interrupt */

      esr1 = getreg32(priv->base + IMX9_CAN_ESR1_OFFSET);
      canerr("ESR1 %x\n", esr1);
      canerr("ERRSR: %x\n", getreg32(priv->base + IMX9_CAN_ERRSR_OFFSET));
      canerr("RERRAR: %x\n", getreg32(priv->base + IMX9_CAN_RERRAR_OFFSET));

      /* Clear the interrupt */

      putreg32(esr1 & (CAN_ESR1_PTA | CAN_ESR1_ATP | CAN_ESR1_ERROVR |
                       CAN_ESR1_ERRINTFAST | CAN_ESR1_BOFFDONEINT |
                       CAN_ESR1_TWRNINT | CAN_ESR1_RWRNINT |
                       CAN_ESR1_BOFFINT | CAN_ESR1_ERRINT | CAN_ESR1_WAKINT),
               priv->base + IMX9_CAN_ESR1_OFFSET);
    }

  return OK;
}

/****************************************************************************
 * Function: imx9_txtimeout_work
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

static void imx9_txtimeout_work(void *arg)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)arg;
  int mbi;
  int txmbi = 0;
  uint32_t mb_bit;
  volatile struct mb_s *mb;
  struct timespec ts;
  struct timeval *now = (struct timeval *)&ts;
  clock_systime_timespec(&ts);
  now->tv_usec = ts.tv_nsec / 1000; /* timespec to timeval conversion */

  /* The watchdog timed out, yet we still check mailboxes in case the
   * transmit function transmitted a new frame
   */

  for (mbi = RXMBCOUNT; mbi < TOTALMBCOUNT; mbi++)
    {
      mb_bit = 1 << mbi;

      /* Disable interrupt for this MB */

      modifyreg32(priv->base + IMX9_CAN_IMASK1_OFFSET, mb_bit, 0);
      UP_DSB();

      if (priv->txmb[txmbi].deadline.tv_sec != 0
          && (now->tv_sec > priv->txmb[txmbi].deadline.tv_sec
          || now->tv_usec > priv->txmb[txmbi].deadline.tv_usec))
        {
          /* This MB was timed out */

          NETDEV_TXTIMEOUTS(&priv->dev);

          mb = flexcan_get_mb(priv, mbi);
          mb->cs = CAN_TXMB_ABORT << CAN_MB_CS_CODE_SHIFT;
        }

      /* Re-enable interrupt for this MB */

      modifyreg32(priv->base + IMX9_CAN_IMASK1_OFFSET, 0, mb_bit);
      txmbi++;
    }
}

/****************************************************************************
 * Function: imx9_txtimeout_expiry
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

static void imx9_txtimeout_expiry(wdparm_t arg)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)arg;

  /* Schedule to perform the TX timeout processing on the worker thread */

  work_queue(CANWORK, &priv->irqwork, imx9_txtimeout_work, priv, 0);
}

#endif

static bool imx9_setenable(uint32_t base, bool enable)
{
  if (enable)
    {
      modifyreg32(base + IMX9_CAN_MCR_OFFSET, CAN_MCR_MDIS, 0);
    }
  else
    {
      modifyreg32(base + IMX9_CAN_MCR_OFFSET, 0, CAN_MCR_MDIS);
    }

  return imx9_waitmcr_change(base, CAN_MCR_LPMACK, !enable);
}

static bool imx9_setfreeze(uint32_t base, bool freeze)
{
  if (freeze)
    {
      modifyreg32(base + IMX9_CAN_MCR_OFFSET, 0, CAN_MCR_HALT | CAN_MCR_FRZ);
    }
  else
    {
      modifyreg32(base + IMX9_CAN_MCR_OFFSET, CAN_MCR_HALT | CAN_MCR_FRZ, 0);
    }

  return imx9_waitmcr_change(base, CAN_MCR_FRZACK, freeze);
}

static bool imx9_waitmcr_change(uint32_t base, uint32_t mask,
                                bool target_state)
{
  const uint32_t timeout = 1000;
  uint32_t wait_ack;
  bool state;

  for (wait_ack = 0; wait_ack < timeout; wait_ack++)
    {
      state = (getreg32(base + IMX9_CAN_MCR_OFFSET) & mask) != 0;
      if (state == target_state)
        {
          return true;
        }

      up_udelay(10);
    }

  return false;
}

/****************************************************************************
 * Function: flexcan_get_mb
 *
 * Description:
 *   Get message buffer start address by buffer index. Message buffers
 *   are allocated in 512-byte ramblocks.
 *
 * Input Parameters:
 *   priv - Reference to the private FLEXCAN driver state structure
 *   mbi  - Message buffer index
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static volatile struct mb_s *flexcan_get_mb(struct imx9_driver_s *priv,
                                            int mbi)
{
  uintptr_t mb_offset;
  size_t data_bytes = priv->canfd_capable ? 64 : 8;
  size_t mb_bytes = sizeof(struct mb_s) + data_bytes;
  int mbs_per_block = 512 / mb_bytes;          /* n of buffers in one ramblock */
  int ramblock = mbi / mbs_per_block;          /* ramblock in which the mb resides */
  int mb_off = mbi - ramblock * mbs_per_block; /* idx of the mb within ramblock */

  mb_offset = IMX9_CAN_MB_OFFSET + (ramblock * 512) + mb_off * mb_bytes;

  DEBUGASSERT(mb_offset < IMX9_CAN_MB_END);

  return (volatile struct mb_s *)(priv->base + mb_offset);
}

/****************************************************************************
 * Function: imx9_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK or ERROR
 *
 * Assumptions:
 *
 ****************************************************************************/

static int imx9_ifup(struct net_driver_s *dev)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)dev;

  if (imx9_initialize(priv) != OK)
    {
      canerr("initialize failed");
      return ERROR;
    }

  priv->bifup = true;
  priv->dev.d_buf = (uint8_t *)priv->txdesc;

  /* Set interrupts */

  up_enable_irq(priv->irq);
  up_enable_irq(priv->irq + 1);

  return OK;
}

/****************************************************************************
 * Function: imx9_ifdown
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

static int imx9_ifdown(struct net_driver_s *dev)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)dev;

  /* Disable interrupts */

  up_disable_irq(priv->irq);
  up_disable_irq(priv->irq + 1);

  imx9_reset(priv);

  priv->bifup = false;
  return OK;
}

/****************************************************************************
 * Function: imx9_txavail_work
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

static void imx9_txavail_work(void *arg)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      if (!imx9_txringfull(priv))
        {
          /* No, there is space for another transfer.  Poll the network for
           * new XMIT data.
           */

          devif_poll(&priv->dev, imx9_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: imx9_txavail
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

static int imx9_txavail(struct net_driver_s *dev)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)dev;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      imx9_txavail_work(priv);
    }

  return OK;
}

/****************************************************************************
 * Function: imx9_ioctl
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
static int imx9_ioctl(struct net_driver_s *dev, int cmd,
                      unsigned long arg)
{
  int ret;

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
      case SIOCGCANBITRATE: /* Get bitrate from a CAN controller */
        {
          struct imx9_driver_s *priv = (struct imx9_driver_s *)dev;
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
          struct imx9_driver_s *priv = (struct imx9_driver_s *)dev;
          struct can_ioctl_data_s *req =
              (struct can_ioctl_data_s *)((uintptr_t)arg);
          struct flexcan_timeseg arbi_timing;
          struct flexcan_timeseg data_timing;

          arbi_timing.bitrate = req->arbi_bitrate * 1000;
          arbi_timing.samplep = req->arbi_samplep;
          ret = imx9_bitratetotimeseg(priv, &arbi_timing, false);
          if (ret == OK && priv->canfd_capable)
            {
              data_timing.bitrate = req->data_bitrate * 1000;
              data_timing.samplep = req->data_samplep;
              ret = imx9_bitratetotimeseg(priv, &data_timing, true);
            }

          if (ret == OK)
            {
              /* Reset CAN controller and start with new timings */

              priv->arbi_timing = arbi_timing;
              if (priv->canfd_capable)
              {
                priv->data_timing = data_timing;
              }

              imx9_ifup(dev);
            }
        }
        break;
#endif
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Function: imx9_init_eccram
 *
 * Description:
 *   Initialize FLEXCAN ECC RAM
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

static int imx9_init_eccram(struct imx9_driver_s *priv)
{
  volatile uint32_t *data = (uint32_t *)(priv->base + IMX9_CAN_MB_OFFSET);
  volatile uint32_t *data_end = (uint32_t *)(priv->base + IMX9_CAN_MB_END);
  int i;

  /* Set WRMFRZ bit in CTRL2 Register to grant write access to memory */

  modifyreg32(priv->base + IMX9_CAN_CTRL2_OFFSET, 0, CAN_CTRL2_WRMFRZ);

  /* Fill in the whole MB area as inactive RX buffers */

  while (data < data_end)
    {
      *data++ = 0;
    }

  /* Clear Mask registers - allow all for RX MBs */

  for (i = 0; i < RXMBCOUNT; i++)
    {
      putreg32(0, priv->base + IMX9_CAN_RXIMR_OFFSET(i));
    }

  /* Set Mask registers to compare all for TX MBs;
   * sending an RTR will result TX MB to become RX EMPTY.
   * We don't want to match anything in this case
   */

  for (; i < IMX9_CAN_N_RXIMR; i++)
    {
      putreg32(0xffffffff, priv->base + IMX9_CAN_RXIMR_OFFSET(i));
    }

  /* Clear legacy fifo information registers */

  for (i = 0; i < IMX9_CAN_N_RXFMB; i++)
    {
      putreg32(0, priv->base + IMX9_CAN_RXFMB_OFFSET(i));
    }

  /* Configure RX*MASKs at 0xaa0-> */

  putreg32(0x3fffffff, priv->base + IMX9_CAN_RX14MASK_OFFSET);
  putreg32(0x3fffffff, priv->base + IMX9_CAN_RX15MASK_OFFSET);
  putreg32(0x3fffffff, priv->base + IMX9_CAN_RXMGMASK_OFFSET);
  putreg32(0x0, priv->base + IMX9_CAN_RXFGMASK_OFFSET);

  /* Clear Tx_SMB, Rx_SMB0 and Rx_SMB1 */

  if (!priv->canfd_capable)
    {
      data = (uint32_t *)(priv->base + IMX9_CAN_TXSMB_OFFSET);
      data_end = &data[3 * (sizeof(struct mb_s) + 8) / 4];
    }
  else
    {
      data = (uint32_t *)(priv->base + IMX9_CAN_TXSMBFD_OFFSET);
      data_end = &data[3 * (sizeof(struct mb_s) + 64) / 4];
    }

  while (data < data_end)
    {
      *data++ = 0;
    }

  /* Clear WRMFRZ bit in CTRL2 Register */

  modifyreg32(priv->base + IMX9_CAN_CTRL2_OFFSET, CAN_CTRL2_WRMFRZ, 0);

  return 0;
}

/****************************************************************************
 * Function: imx9_initalize
 *
 * Description:
 *   Initialize FLEXCAN device
 *
 * Input Parameters:
 *   priv - Reference to the private FLEXCAN driver state structure
 *
 * Returned Value:
 *   OK or ERROR
 *
 * Assumptions:
 *
 ****************************************************************************/

static int imx9_initialize(struct imx9_driver_s *priv)
{
  uint32_t tdcoff;
  int i;
  volatile struct mb_s *mb;

  /* Enable module */

  if (!imx9_setenable(priv->base, true))
    {
      canerr("FLEXCAN: enable fail\n");
      return ERROR;
    }

  /* Enter freeze mode */

  if (!imx9_setfreeze(priv->base, true))
    {
      canerr("FLEXCAN: freeze fail\n");
      return ERROR;
    }

  /* Initialize memory buffers */

  imx9_init_eccram(priv);

  /* Configure MCR */

  modifyreg32(priv->base + IMX9_CAN_MCR_OFFSET, CAN_MCR_MAXMB_MASK,
              CAN_MCR_SLFWAK | CAN_MCR_WRNEN | CAN_MCR_WAKSRC |
              CAN_MCR_IRMQ | CAN_MCR_LPRIOEN | CAN_MCR_AEN |
              (((TOTALMBCOUNT - 1) << CAN_MCR_MAXMB_SHIFT) &
               CAN_MCR_MAXMB_MASK));

  if (priv->srxdis)
    {
      modifyreg32(priv->base + IMX9_CAN_MCR_OFFSET, 0, CAN_MCR_SRXDIS);
    }

  if (!priv->canfd_capable)
    {
      modifyreg32(priv->base + IMX9_CAN_CTRL1_OFFSET,
                  CAN_CTRL1_PRESDIV_MASK | CAN_CTRL1_PROPSEG_MASK |
                  CAN_CTRL1_PSEG1_MASK | CAN_CTRL1_PSEG2_MASK |
                  CAN_CTRL1_RJW_MASK,
                  CAN_CTRL1_PRESDIV(priv->arbi_timing.presdiv) | /* Prescaler divisor factor */
                  CAN_CTRL1_PROPSEG(priv->arbi_timing.propseg) | /* Propagation segment */
                  CAN_CTRL1_PSEG1(priv->arbi_timing.pseg1) |     /* Phase buffer segment 1 */
                  CAN_CTRL1_PSEG2(priv->arbi_timing.pseg2) |     /* Phase buffer segment 2 */
                  CAN_CTRL1_RJW(1));                             /* Resynchronization jump width */
    }
  else
    {
      modifyreg32(priv->base + IMX9_CAN_CBT_OFFSET,
                  CAN_CBT_EPRESDIV_MASK | CAN_CBT_EPROPSEG_MASK |
                  CAN_CBT_EPSEG1_MASK | CAN_CBT_EPSEG2_MASK |
                  CAN_CBT_ERJW_MASK,
                  CAN_CBT_BTF |                                 /* Enable extended bit timing
                                                                 * configurations for CAN-FD for setting up
                                                                 * separately nominal and data phase */
                  CAN_CBT_EPRESDIV(priv->arbi_timing.presdiv) | /* Prescaler divisor factor */
                  CAN_CBT_EPROPSEG(priv->arbi_timing.propseg) | /* Propagation segment */
                  CAN_CBT_EPSEG1(priv->arbi_timing.pseg1) |     /* Phase buffer segment 1 */
                  CAN_CBT_EPSEG2(priv->arbi_timing.pseg2) |     /* Phase buffer segment 2 */
                  CAN_CBT_ERJW(1));                             /* Resynchronization jump width */

      /* Enable CAN FD feature */

      modifyreg32(priv->base + IMX9_CAN_MCR_OFFSET, 0, CAN_MCR_FDEN);

      modifyreg32(priv->base + IMX9_CAN_FDCBT_OFFSET,
                  CAN_FDCBT_FPRESDIV_MASK | CAN_FDCBT_FPROPSEG_MASK |
                  CAN_FDCBT_FPSEG1_MASK | CAN_FDCBT_FPSEG2_MASK |
                  CAN_FDCBT_FRJW_MASK,
                  CAN_FDCBT_FPRESDIV(priv->data_timing.presdiv) |  /* Prescaler divisor factor of 1 */
                  CAN_FDCBT_FPROPSEG(priv->data_timing.propseg) |  /* Propagation
                                                                    * segment (only register that doesn't add 1) */
                  CAN_FDCBT_FPSEG1(priv->data_timing.pseg1) |      /* Phase buffer segment 1 */
                  CAN_FDCBT_FPSEG2(priv->data_timing.pseg2) |      /* Phase buffer segment 2 */
                  CAN_FDCBT_FRJW(priv->data_timing.pseg2));        /* Resynchorinzation jump width same as PSEG2 */

      /* Additional CAN-FD configurations */

      tdcoff = (priv->data_timing.pseg1 + priv->data_timing.pseg2 + 2) *
        (priv->data_timing.presdiv + 1);

      modifyreg32(priv->base + IMX9_CAN_FDCTRL_OFFSET, 0,
                  CAN_FDCTRL_FDRATE |          /* Enable bit rate switch in data phase of frame */
                  CAN_FDCTRL_TDCEN |           /* Enable transceiver delay compensation */
                  CAN_FDCTRL_TDCOFF(tdcoff) |  /* Setup 5 cycles for data phase sampling delay */
                  CAN_FDCTRL_MBDSR0(3) |       /* Setup 64 bytes per MB 0-6 */
                  CAN_FDCTRL_MBDSR1(3) |       /* Setup 64 bytes per MB 7-13 */
                  CAN_FDCTRL_MBDSR2(3));       /* Setup 64 bytes per MB 14-20 */

      modifyreg32(priv->base + IMX9_CAN_CTRL2_OFFSET, 0,
                  CAN_CTRL2_ISOCANFDEN);
    }

  /* Exit supervisor mode */

  modifyreg32(priv->base + IMX9_CAN_MCR_OFFSET, CAN_MCR_SUPV, 0);

  /* Always compare also IDE and RTR bits to mask in RX */

  modifyreg32(priv->base + IMX9_CAN_CTRL2_OFFSET, CAN_CTRL2_RETRY_MASK,
              CAN_CTRL2_RRS | CAN_CTRL2_EACEN |
              3 << CAN_CTRL2_RETRY_SHIFT);

  /* Clear MB interrupts */

  putreg32(IFLAG1_TX | IFLAG1_RX, priv->base + IMX9_CAN_IFLAG1_OFFSET);

  /* Enable MB interrupts */

  putreg32(IFLAG1_TX | IFLAG1_RX, priv->base + IMX9_CAN_IMASK1_OFFSET);

  /* Set RX buffers to receive */

  for (i = 0; i < RXMBCOUNT; i++)
    {
      mb = flexcan_get_mb(priv, i);
      mb->cs = (CAN_RXMB_EMPTY << CAN_MB_CS_CODE_SHIFT) | CAN_MB_CS_IDE;
    }

  /* Exit freeze mode */

  if (!imx9_setfreeze(priv->base, false))
    {
      canerr("FLEXCAN: unfreeze fail\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Function: imx9_reset
 *
 * Description:
 *   Reset the flexcan and put it into disabled state
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

static void imx9_reset(struct imx9_driver_s *priv)
{
  /* Make sure module is enabled */

  if (!imx9_setenable(priv->base, true))
    {
      canerr("Enable fail\n");
      return;
    }

  modifyreg32(priv->base + IMX9_CAN_MCR_OFFSET, 0, CAN_MCR_SOFTRST);

  if (!imx9_waitmcr_change(priv->base, CAN_MCR_SOFTRST, false))
    {
      canerr("Reset failed");
      return;
    }

  /* Disable module */

  if (!imx9_setenable(priv->base, false))
    {
      canerr("Disable fail\n");
      return;
    }
}

/****************************************************************************
 * Function: imx9_canpinmux
 *
 * Description:
 *   Mux the pins used for CAN RX&TX
 *§
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void imx9_canpinmux(void)
{
#ifdef CONFIG_IMX9_FLEXCAN1
  imx9_iomux_configure(MUX_FLEXCAN1_TX);
  imx9_iomux_configure(MUX_FLEXCAN1_RX);
#endif

#ifdef CONFIG_IMX9_FLEXCAN2
  imx9_iomux_configure(MUX_FLEXCAN2_TX);
  imx9_iomux_configure(MUX_FLEXCAN2_RX);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: imx9_caninitialize
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

int imx9_caninitialize(int intf)
{
  struct imx9_driver_s *priv;
  int ret;

  /* Select device, configure root clock to PLL1PFD1DIV2 / 5 and enable
   * peripheral clock
   */

  switch (intf)
    {
#ifdef CONFIG_IMX9_FLEXCAN1
    case 1:
      imx9_ccm_configure_root_clock(CCM_CR_CAN1, SYS_PLL1PFD1DIV2, 5);
      imx9_ccm_gate_on(CCM_LPCG_CAN1, true);
      priv = &g_flexcan1;
      break;
#endif

#ifdef CONFIG_IMX9_FLEXCAN2
    case 2:
      imx9_ccm_configure_root_clock(CCM_CR_CAN2, SYS_PLL1PFD1DIV2, 5);
      imx9_ccm_gate_on(CCM_LPCG_CAN2, true);
      priv = &g_flexcan2;
      break;
#endif

    default:
      return -ENODEV;
    }

  /* Get and store the clock (should be 80 MHz now) */

  imx9_get_rootclock(CCM_CR_CAN1, &priv->clk_freq);

  if (imx9_bitratetotimeseg(priv, &priv->arbi_timing, false) != OK)
    {
      canerr("ERROR: Invalid CAN timings please try another sample point "
             "or refer to the reference manual\n");
      return -1;
    }

  if (priv->canfd_capable)
    {
      if (imx9_bitratetotimeseg(priv, &priv->data_timing, true) != OK)
        {
          canerr("ERROR: Invalid CAN data phase timings please try another "
                 "sample point or refer to the reference manual\n");
          return -1;
        }
    }

  /* Mux the can RX&TX pins */

  imx9_canpinmux();

  if (irq_attach(priv->irq, imx9_flexcan_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      canerr("ERROR: Failed to attach CAN bus IRQ\n");
      return -EAGAIN;
    }

  if (irq_attach(priv->irq + 1, imx9_flexcan_interrupt, priv))
    {
      /* We could not attach the error ISR to the interrupt */

      canerr("ERROR: Failed to attach CAN bus error IRQ\n");
      irq_detach(priv->irq);
      return -EAGAIN;
    }

  /* Disable */

  imx9_setenable(priv->base, false);

  /* Initialize the driver structure */

  priv->dev.d_ifup    = imx9_ifup;      /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = imx9_ifdown;    /* I/F down callback */
  priv->dev.d_txavail = imx9_txavail;   /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = imx9_ioctl;     /* Support CAN ioctl() calls */
#endif

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling imx9_ifdown().
   */

  ninfo("callbacks done\n");

  imx9_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_CAN);

  UNUSED(ret);
  return OK;
}

/****************************************************************************
 * Name: arm64_caninitialize
 *
 * Description:
 *   Initialize the can network interfaces.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETDEV_LATEINIT)
void arm64_caninitialize(void)
{
#ifdef CONFIG_IMX9_FLEXCAN1
  imx9_caninitialize(1);
#endif

#ifdef CONFIG_IMX9_FLEXCAN2
  imx9_caninitialize(2);
#endif
}
#endif

#endif /* CONFIG_IMX9_FLEXCAN */

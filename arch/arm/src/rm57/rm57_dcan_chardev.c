/****************************************************************************
 * arch/arm/src/rm57/rm57_dcan_chardev.c
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

/* Character-device (drivers/can) frontend for the RM57 DCAN core.  One
 * instance of this frontend serves every DCAN configured with
 * RM57_DCANn_CHARDRIVER; instances configured for SocketCAN are served
 * by rm57_dcan_sock.c instead.
 *
 * TX model: the D_CAN mailbox window is a hardware TX FIFO, so co_send
 * completes the frame immediately (can_txdone at the end of co_send)
 * and the core's txempty event maps to can_txready() - the topology
 * described in include/nuttx/can/can.h for CONFIG_CAN_TXREADY.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/can/can.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/rm57_dcan.h"
#include "rm57_dcan.h"

#if defined(CONFIG_RM57_DCAN) && defined(CONFIG_CAN)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CAN driver methods */

static void rm57can_co_reset(struct can_dev_s *dev);
static int  rm57can_co_setup(struct can_dev_s *dev);
static void rm57can_co_shutdown(struct can_dev_s *dev);
static void rm57can_co_rxint(struct can_dev_s *dev, bool enable);
static void rm57can_co_txint(struct can_dev_s *dev, bool enable);
static int  rm57can_co_ioctl(struct can_dev_s *dev, int cmd,
                             unsigned long arg);
static int  rm57can_co_send(struct can_dev_s *dev, struct can_msg_s *msg);
static bool rm57can_co_txready(struct can_dev_s *dev);
static bool rm57can_co_txempty(struct can_dev_s *dev);

/* Core callbacks */

static void rm57can_cd_rxframe(void *ctx, uint32_t arb, uint32_t mctl,
                               uint32_t da, uint32_t db);
static void rm57can_cd_txempty(void *ctx);
static void rm57can_cd_errevent(void *ctx, uint32_t es, uint32_t errc);
static void rm57can_cd_rxoverflow(void *ctx);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_rm57can_ops =
{
  .co_reset         = rm57can_co_reset,
  .co_setup         = rm57can_co_setup,
  .co_shutdown      = rm57can_co_shutdown,
  .co_rxint         = rm57can_co_rxint,
  .co_txint         = rm57can_co_txint,
  .co_ioctl         = rm57can_co_ioctl,
  .co_remoterequest = NULL,             /* CONFIG_CAN_USE_RTR instead */
  .co_send          = rm57can_co_send,
  .co_txready       = rm57can_co_txready,
  .co_txempty       = rm57can_co_txempty,
};

static const struct rm57can_cbs_s g_rm57can_cd_cbs =
{
  .rxframe  = rm57can_cd_rxframe,
  .txdone   = NULL,                     /* can_txdone runs in co_send */
  .txempty    = rm57can_cd_txempty,
  .errevent   = rm57can_cd_errevent,
  .rxoverflow = rm57can_cd_rxoverflow,
};

/* One character device per DCAN instance (only those configured as
 * character drivers are ever registered).
 */

static struct can_dev_s g_rm57can_dev[RM57_NCAN];

/* Set when can_receive() has refused a frame because the upper-half FIFO
 * was full.  The overflow is reported once there is room again: emitting
 * the report at the moment of failure is pointless, since the report
 * itself would be the next thing refused.
 */

static bool g_rxovf_pending[RM57_NCAN];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57can_cd_ovfframe
 *
 * Description:
 *   Report a dropped receive frame as a CAN error frame.  The two
 *   sources are kept distinct because they point at different fixes:
 *   a controller overflow (MCTL.MsgLst) means the interrupt did not run
 *   in time, while a queue overflow means the application did not read
 *   in time.
 *
 ****************************************************************************/

static void rm57can_cd_ovfframe(struct can_dev_s *dev, bool controller)
{
#ifdef CONFIG_CAN_ERRORS
  struct can_hdr_s hdr;
  uint8_t data[CAN_ERROR_DLC];

  memset(&hdr, 0, sizeof(hdr));
  memset(data, 0, sizeof(data));

  if (controller)
    {
      hdr.ch_id = CAN_ERROR_CONTROLLER;
      data[1]   = CAN_ERROR1_RXOVERFLOW;
    }
  else
    {
      hdr.ch_id = CAN_ERROR_INTERNAL;
      data[5]   = CAN_ERROR5_RXOVERFLOW;
    }

  hdr.ch_dlc   = CAN_ERROR_DLC;
  hdr.ch_error = true;

  can_receive(dev, &hdr, data);
#else
  UNUSED(dev);
  UNUSED(controller);
#endif
}

/****************************************************************************
 * Name: rm57can_cd_rxoverflow
 *
 * Description:
 *   Core callback (interrupt context): the controller overwrote a frame
 *   in its message object before the driver read it.
 *
 ****************************************************************************/

static void rm57can_cd_rxoverflow(void *ctx)
{
  rm57can_cd_ovfframe((struct can_dev_s *)ctx, true);
}

/****************************************************************************
 * Name: rm57can_cd_rxframe
 *
 * Description:
 *   Core RX callback (interrupt context): convert the raw message
 *   object registers into a can_hdr_s and hand it to the upper half.
 *
 ****************************************************************************/

static void rm57can_cd_rxframe(void *ctx, uint32_t arb, uint32_t mctl,
                               uint32_t da, uint32_t db)
{
  struct can_dev_s *dev = (struct can_dev_s *)ctx;
  struct can_hdr_s hdr;
  uint8_t data[8];
  uint8_t dlc;
  int i;

  memset(&hdr, 0, sizeof(hdr));

  dlc = mctl & DCAN_IFMCTL_DLC_MASK;
  if (dlc > 8)
    {
      dlc = 8;
    }

  hdr.ch_dlc = dlc;

  /* Note: the D_CAN stores received frames with Dir = 0, so the RTR
   * bit of a received frame cannot be recovered - ch_rtr is always 0
   * (same limitation as the Tiva C_CAN driver).
   */

  if ((arb & DCAN_IFARB_XTD) != 0)
    {
#ifdef CONFIG_CAN_EXTID
      hdr.ch_id    = arb & DCAN_IFARB_ID_MASK;
      hdr.ch_extid = true;
#else
      /* Extended frames cannot be represented; drop */

      return;
#endif
    }
  else
    {
      hdr.ch_id = (arb >> DCAN_IFARB_ID_STD_SHIFT) & 0x7ff;
    }

  for (i = 0; i < 4; i++)
    {
      data[i]     = (uint8_t)(da >> (8 * i));
      data[i + 4] = (uint8_t)(db >> (8 * i));
    }

  /* A refused frame is already gone: the message object was released by
   * the transfer that read it, so there is nothing left to retry.  All
   * that can be done is to make the loss visible.
   */

  if (can_receive(dev, &hdr, data) < 0)
    {
      g_rxovf_pending[dev - g_rm57can_dev] = true;
      canerr("ERROR: DCAN RX queue full, frame 0x%" PRIx32 " dropped\n",
             (uint32_t)hdr.ch_id);
    }
  else if (g_rxovf_pending[dev - g_rm57can_dev])
    {
      g_rxovf_pending[dev - g_rm57can_dev] = false;
      rm57can_cd_ovfframe(dev, false);
    }
}

/****************************************************************************
 * Name: rm57can_cd_txempty
 *
 * Description:
 *   Core TX-drain callback (interrupt context): the hardware TX FIFO
 *   is empty again, wake the upper half in case its software FIFO
 *   still holds messages.
 *
 ****************************************************************************/

static void rm57can_cd_txempty(void *ctx)
{
#ifdef CONFIG_CAN_TXREADY
  can_txready((struct can_dev_s *)ctx);
#endif
}

/****************************************************************************
 * Name: rm57can_cd_errevent
 *
 * Description:
 *   Core status/error callback (interrupt context): translate the ES
 *   snapshot and error counters into a chardev error report frame.
 *
 ****************************************************************************/

static void rm57can_cd_errevent(void *ctx, uint32_t es, uint32_t errc)
{
#ifdef CONFIG_CAN_ERRORS
  struct can_dev_s *dev = (struct can_dev_s *)ctx;
  struct can_hdr_s hdr;
  uint8_t data[CAN_ERROR_DLC];
  uint16_t errbits = 0;
  uint8_t rec = (errc & DCAN_ERRC_REC_MASK) >> DCAN_ERRC_REC_SHIFT;
  uint8_t tec = (errc & DCAN_ERRC_TEC_MASK) >> DCAN_ERRC_TEC_SHIFT;

  memset(data, 0, sizeof(data));

  /* Controller state */

  if ((es & DCAN_ES_BOFF) != 0)
    {
      errbits |= CAN_ERROR_BUSOFF;
    }
  else
    {
      if ((es & DCAN_ES_EWARN) != 0)
        {
          errbits |= CAN_ERROR_CONTROLLER;
          data[1] |= (rec >= 96) ? CAN_ERROR1_RXWARNING : 0;
          data[1] |= (tec >= 96) ? CAN_ERROR1_TXWARNING : 0;
        }

      if ((es & DCAN_ES_EPASS) != 0)
        {
          errbits |= CAN_ERROR_CONTROLLER;
          data[1] |= ((errc & DCAN_ERRC_RP) != 0) ?
                     CAN_ERROR1_RXPASSIVE : CAN_ERROR1_TXPASSIVE;
        }
    }

  /* Last error code */

  switch (es & DCAN_ES_LEC_MASK)
    {
      case DCAN_ES_LEC_STUFF:
        errbits |= CAN_ERROR_PROTOCOL;
        data[2] |= CAN_ERROR2_STUFF;
        break;

      case DCAN_ES_LEC_FORM:
        errbits |= CAN_ERROR_PROTOCOL;
        data[2] |= CAN_ERROR2_FORM;
        break;

      case DCAN_ES_LEC_ACK:
        errbits |= CAN_ERROR_NOACK;
        data[3]  = CAN_ERROR3_ACK;
        break;

      case DCAN_ES_LEC_BIT1:
        errbits |= CAN_ERROR_PROTOCOL;
        data[2] |= CAN_ERROR2_BIT1 | CAN_ERROR2_TX;
        break;

      case DCAN_ES_LEC_BIT0:
        errbits |= CAN_ERROR_PROTOCOL;
        data[2] |= CAN_ERROR2_BIT0 | CAN_ERROR2_TX;
        break;

      case DCAN_ES_LEC_CRC:
        errbits |= CAN_ERROR_PROTOCOL;
        data[3]  = CAN_ERROR3_CRCSEQ;
        break;

      default:
        break;
    }

  /* Message RAM parity/ECC event */

  if ((es & DCAN_ES_PER) != 0)
    {
      errbits |= CAN_ERROR_INTERNAL;
    }

  if (errbits == 0)
    {
      /* Nothing to report (e.g. a TxOK/RxOK status interrupt) */

      return;
    }

  memset(&hdr, 0, sizeof(hdr));
  hdr.ch_id    = errbits;
  hdr.ch_dlc   = CAN_ERROR_DLC;
  hdr.ch_error = true;

  can_receive(dev, &hdr, data);
#endif /* CONFIG_CAN_ERRORS */
}

/****************************************************************************
 * Name: rm57can_co_reset
 ****************************************************************************/

static void rm57can_co_reset(struct can_dev_s *dev)
{
  struct rm57can_s *priv = (struct rm57can_s *)dev->cd_priv;

  rm57can_shutdown(priv);
}

/****************************************************************************
 * Name: rm57can_co_setup / rm57can_co_shutdown
 ****************************************************************************/

static int rm57can_co_setup(struct can_dev_s *dev)
{
  g_rxovf_pending[dev - g_rm57can_dev] = false;
  return rm57can_setup((struct rm57can_s *)dev->cd_priv);
}

static void rm57can_co_shutdown(struct can_dev_s *dev)
{
  rm57can_shutdown((struct rm57can_s *)dev->cd_priv);
}

/****************************************************************************
 * Name: rm57can_co_rxint / rm57can_co_txint
 ****************************************************************************/

static void rm57can_co_rxint(struct can_dev_s *dev, bool enable)
{
  rm57can_rxint((struct rm57can_s *)dev->cd_priv, enable);
}

static void rm57can_co_txint(struct can_dev_s *dev, bool enable)
{
  rm57can_txint((struct rm57can_s *)dev->cd_priv, enable);
}

/****************************************************************************
 * Name: rm57can_co_ioctl
 ****************************************************************************/

static int rm57can_co_ioctl(struct can_dev_s *dev, int cmd,
                            unsigned long arg)
{
  struct rm57can_s *priv = (struct rm57can_s *)dev->cd_priv;

  switch (cmd)
    {
      case CANIOC_GET_BITTIMING:
        {
          struct canioc_bittiming_s *bt =
            (struct canioc_bittiming_s *)(uintptr_t)arg;

          if (bt == NULL)
            {
              return -EINVAL;
            }

          bt->bt_baud  = priv->timing.baud;
          bt->bt_tseg1 = priv->timing.tseg1;
          bt->bt_tseg2 = priv->timing.tseg2;
          bt->bt_sjw   = priv->timing.sjw;
          return OK;
        }

      case CANIOC_SET_BITTIMING:
        {
          const struct canioc_bittiming_s *bt =
            (const struct canioc_bittiming_s *)(uintptr_t)arg;
          struct rm57can_timing_s timing;
          int ret;

          if (bt == NULL || bt->bt_baud == 0)
            {
              return -EINVAL;
            }

          if (bt->bt_tseg1 != 0 || bt->bt_tseg2 != 0)
            {
              /* Explicit segment values: derive the prescaler */

              uint32_t ntq = 1 + bt->bt_tseg1 + bt->bt_tseg2;
              uint32_t div = bt->bt_baud * ntq;

              timing.baud  = bt->bt_baud;
              timing.brp   = (BOARD_VCLK_FREQUENCY + div / 2) / div;
              timing.tseg1 = bt->bt_tseg1;
              timing.tseg2 = bt->bt_tseg2;
              timing.sjw   = (bt->bt_sjw != 0) ? bt->bt_sjw : 1;
            }
          else
            {
              ret = rm57can_timing_from_baud(bt->bt_baud, &timing);
              if (ret < 0)
                {
                  return ret;
                }
            }

          return rm57can_set_timing(priv, &timing);
        }

      case CANIOC_ADD_STDFILTER:
        {
          const struct canioc_stdfilter_s *sf =
            (const struct canioc_stdfilter_s *)(uintptr_t)arg;

          if (sf == NULL || sf->sf_type != CAN_FILTER_MASK)
            {
              return -EINVAL;
            }

          return rm57can_add_filter(priv, false, sf->sf_id1, sf->sf_id2);
        }

      case CANIOC_DEL_STDFILTER:
        {
          return rm57can_del_filter(priv, (int)arg);
        }

#ifdef CONFIG_CAN_EXTID
      case CANIOC_ADD_EXTFILTER:
        {
          const struct canioc_extfilter_s *xf =
            (const struct canioc_extfilter_s *)(uintptr_t)arg;

          if (xf == NULL || xf->xf_type != CAN_FILTER_MASK)
            {
              return -EINVAL;
            }

          return rm57can_add_filter(priv, true, xf->xf_id1, xf->xf_id2);
        }

      case CANIOC_DEL_EXTFILTER:
        {
          return rm57can_del_filter(priv, (int)arg);
        }
#endif

      case CANIOC_GET_CONNMODES:
        {
          struct canioc_connmodes_s *modes =
            (struct canioc_connmodes_s *)(uintptr_t)arg;

          if (modes == NULL)
            {
              return -EINVAL;
            }

          modes->bm_loopback = priv->loopback;
          modes->bm_silent   = priv->silent;
          return OK;
        }

      case CANIOC_SET_CONNMODES:
        {
          const struct canioc_connmodes_s *modes =
            (const struct canioc_connmodes_s *)(uintptr_t)arg;

          if (modes == NULL)
            {
              return -EINVAL;
            }

          return rm57can_connmodes(priv, modes->bm_loopback != 0,
                                   modes->bm_silent != 0);
        }

      case CANIOC_BUSOFF_RECOVERY:
        {
          return rm57can_busoff_recovery(priv);
        }

      case CANIOC_SET_NART:
        {
          return rm57can_set_nart(priv, arg != 0);
        }

      case CANIOC_SET_ABOM:
        {
          return rm57can_set_abom(priv, arg != 0);
        }

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: rm57can_co_send
 ****************************************************************************/

static int rm57can_co_send(struct can_dev_s *dev, struct can_msg_s *msg)
{
  struct rm57can_s *priv = (struct rm57can_s *)dev->cd_priv;
  bool extid = false;
  bool rtr   = false;
  int ret;

#ifdef CONFIG_CAN_EXTID
  extid = (msg->cm_hdr.ch_extid != 0);
#endif
#ifdef CONFIG_CAN_USE_RTR
  rtr = (msg->cm_hdr.ch_rtr != 0);
#endif

  ret = rm57can_send(priv, extid, rtr, msg->cm_hdr.ch_id,
                     msg->cm_hdr.ch_dlc, msg->cm_data);
  if (ret < 0)
    {
      return ret;
    }

  /* The frame is wholly owned by the hardware FIFO now - complete it
   * so the upper half can queue the next one (see the TXREADY topology
   * notes in include/nuttx/can/can.h).
   */

  can_txdone(dev);
  return OK;
}

/****************************************************************************
 * Name: rm57can_co_txready / rm57can_co_txempty
 ****************************************************************************/

static bool rm57can_co_txready(struct can_dev_s *dev)
{
  return rm57can_txready((struct rm57can_s *)dev->cd_priv);
}

static bool rm57can_co_txempty(struct can_dev_s *dev)
{
  return rm57can_txempty((struct rm57can_s *)dev->cd_priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_dcan_chardev_initialize
 *
 * Description:
 *   Bind DCAN instance n (0..3) to the character-device upper half and
 *   register it at 'path' (e.g. "/dev/can0").
 *
 ****************************************************************************/

int rm57_dcan_chardev_initialize(const char *path, int n)
{
  struct rm57can_s *priv = rm57can_instance(n);
  struct can_dev_s *dev;
  int ret;

  if (priv == NULL || n < 0 || n >= RM57_NCAN)
    {
      return -ENODEV;
    }

  dev = &g_rm57can_dev[n];
  dev->cd_ops  = &g_rm57can_ops;
  dev->cd_priv = priv;

  ret = rm57can_init(priv, &g_rm57can_cd_cbs, dev);
  if (ret < 0)
    {
      return ret;
    }

  ret = can_register(path, dev);
  if (ret < 0)
    {
      canerr("DCAN%d: can_register(%s) failed: %d\n", n + 1, path, ret);
      return ret;
    }

  caninfo("DCAN%d registered at %s\n", n + 1, path);
  return OK;
}

#endif /* CONFIG_RM57_DCAN && CONFIG_CAN */

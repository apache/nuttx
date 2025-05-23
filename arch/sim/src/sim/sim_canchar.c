/****************************************************************************
 * arch/sim/src/sim/sim_canchar.c
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

#include <debug.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>

#include <nuttx/can/can.h>
#include <nuttx/can.h>

#include "sim_hostcan.h"
#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIM_CAN_WORK_DELAY  USEC2TICK(1000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_canchar_s
{
  struct can_dev_s dev;    /* CAN character device */
  struct sim_can_s host;   /* Host CAN handler */
  struct work_s    worker; /* Work queue for RX */
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* CAN driver methods */

static void sim_can_reset(struct can_dev_s *dev);
static int  sim_can_setup(struct can_dev_s *dev);
static void sim_can_shutdown(struct can_dev_s *dev);
static void sim_can_rxint(struct can_dev_s *dev, bool enable);
static void sim_can_txint(struct can_dev_s *dev, bool enable);
static int  sim_can_ioctl(struct can_dev_s *dev, int cmd, unsigned long arg);
static int  sim_can_remoterequest(struct can_dev_s *dev, uint16_t id);
static int  sim_can_send(struct can_dev_s *dev, struct can_msg_s *msg);
static bool sim_can_txready(struct can_dev_s *dev);
static bool sim_can_txempty(struct can_dev_s *dev);
static void sim_can_work(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_sim_can_ops =
{
  .co_reset         = sim_can_reset,
  .co_setup         = sim_can_setup,
  .co_shutdown      = sim_can_shutdown,
  .co_rxint         = sim_can_rxint,
  .co_txint         = sim_can_txint,
  .co_ioctl         = sim_can_ioctl,
  .co_remoterequest = sim_can_remoterequest,
  .co_send          = sim_can_send,
  .co_txready       = sim_can_txready,
  .co_txempty       = sim_can_txempty,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_can_reset
 ****************************************************************************/

static void sim_can_reset(struct can_dev_s *dev)
{
  /* Nothing is here */
}

/****************************************************************************
 * Name: sim_can_setup
 ****************************************************************************/

static int sim_can_setup(struct can_dev_s *dev)
{
  struct sim_canchar_s *priv = dev->cd_priv;
  int ret;

  ret = host_can_ifup(&priv->host);
  if (ret < 0)
    {
      return ret;
    }

  /* Start RX work */

  return work_queue(HPWORK, &priv->worker, sim_can_work, priv, 0);
}

/****************************************************************************
 * Name: sim_can_shutdown
 ****************************************************************************/

static void sim_can_shutdown(struct can_dev_s *dev)
{
  struct sim_canchar_s *priv = dev->cd_priv;

  /* Cancel work */

  work_cancel(HPWORK, &priv->worker);

  host_can_ifdown(&priv->host);
}

/****************************************************************************
 * Name: sim_can_rxint
 ****************************************************************************/

static void sim_can_rxint(struct can_dev_s *dev, bool enable)
{
  /* Nothing is here */
}

/****************************************************************************
 * Name: sim_can_txint
 ****************************************************************************/

static void sim_can_txint(struct can_dev_s *dev, bool enable)
{
  /* Nothing is here */
}

/****************************************************************************
 * Name: sim_can_ioctl
 ****************************************************************************/

static int sim_can_ioctl(struct can_dev_s *dev, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: sim_can_remoterequest
 ****************************************************************************/

static int sim_can_remoterequest(struct can_dev_s *dev, uint16_t id)
{
  return -ENOTSUP;
}

/****************************************************************************
 * Name: sim_can_send
 ****************************************************************************/

static int sim_can_send(struct can_dev_s *dev, struct can_msg_s *msg)
{
  struct sim_canchar_s *priv = dev->cd_priv;
  int ret;

#ifdef CONFIG_CAN_FD
  if (msg->cm_hdr.ch_edl)
    {
      struct canfd_frame frame;

      frame.can_id = msg->cm_hdr.ch_id;

      if (msg->cm_hdr.ch_rtr)
        {
          frame.can_id |= CAN_RTR_FLAG;
        }

#ifdef CONFIG_CAN_EXTID
      if (msg->cm_hdr.ch_extid)
        {
          frame.can_id |= CAN_EFF_FLAG;
        }
#endif

      frame.flags = CANFD_FDF;

      if (msg->cm_hdr.ch_brs)
        {
          frame.flags |= CANFD_BRS;
        }

      if (msg->cm_hdr.ch_esi)
        {
          frame.flags |= CANFD_ESI;
        }

      frame.len = can_dlc2bytes(msg->cm_hdr.ch_dlc);

      memcpy(frame.data, msg->cm_data, frame.len);

      ret = host_can_send(&priv->host, &frame, sizeof(struct canfd_frame));
    }
  else
#endif
    {
      struct can_frame frame;

      frame.can_id  = msg->cm_hdr.ch_id;
      frame.can_dlc = msg->cm_hdr.ch_dlc;

      if (msg->cm_hdr.ch_rtr)
        {
          frame.can_id |= CAN_RTR_FLAG;
        }

#ifdef CONFIG_CAN_EXTID
      /* Extended frame */

      if (msg->cm_hdr.ch_extid)
        {
          frame.can_id |= CAN_EFF_FLAG;
        }
#endif

      memcpy(frame.data, msg->cm_data, frame.can_dlc);

      ret = host_can_send(&priv->host, &frame, sizeof(struct can_frame));
    }

  if (ret > 0)
    {
      /* Tell the upper half that the transfer is finished. */

      can_txdone(dev);
    }

  return ret;
}

/****************************************************************************
 * Name: sim_can_txready
 ****************************************************************************/

static bool sim_can_txready(struct can_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: sim_can_txempty
 ****************************************************************************/

static bool sim_can_txempty(struct can_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: sim_can_work
 *
 * Description:
 *   Feed pending packets on the host sockets into the CAN stack.
 *
 * Assumption:
 *   NuttX canfd_frame is byte compatible with host canfd_frame
 *
 ****************************************************************************/

static void sim_can_work(void *arg)
{
  struct sim_canchar_s *priv = arg;
  struct canfd_frame    frame;
  struct can_hdr_s      hdr;
  int                   ret;

  if (host_can_avail(&priv->host))
    {
      /* Wait for data from host stack */

      ret = host_can_read(&priv->host, &frame);
      if (ret < 0)
        {
          canerr("host_can_read failed %d\n", ret);
          goto nodata;
        }

      /* Get header */

      hdr.ch_id    = frame.can_id & CAN_ERR_MASK;
      hdr.ch_dlc   = can_bytes2dlc(frame.len);
      hdr.ch_rtr   = frame.can_id & CAN_RTR_FLAG;
#ifdef CONFIG_CAN_ERRORS
      hdr.ch_error = frame.can_id & CAN_ERR_FLAG;
#endif
#ifdef CONFIG_CAN_EXTID
      hdr.ch_extid = frame.can_id & CAN_EFF_FLAG;
#endif
#ifdef CONFIG_CAN_FD
      hdr.ch_edl   = frame.flags & CANFD_FDF;
      hdr.ch_brs   = frame.flags & CANFD_BRS;
      hdr.ch_esi   = frame.flags & CANFD_ESI;
#endif
      hdr.ch_tcf   = 0;
#ifdef CONFIG_CAN_TIMESTAMP
      hdr.ch_ts    = 0;
#endif

      /* Notify upper-half */

      can_receive(&priv->dev, &hdr, frame.data);
    }

nodata:
  work_queue(HPWORK, &priv->worker, sim_can_work, priv, SIM_CAN_WORK_DELAY);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_canchar_initialize
 ****************************************************************************/

int sim_canchar_initialize(int devidx, int devno)
{
  struct sim_canchar_s *priv;
  char                  devpath[PATH_MAX];
  int                   ret;

  /* Allocate device */

  priv = kmm_zalloc(sizeof(struct sim_canchar_s));
  if (!priv)
    {
      return -ENOMEM;
    }

  /* Get host interface */

  ret = host_can_init(&priv->host, devidx);
  if (ret < 0)
    {
      canerr("host_can_init failed %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* Initialzie CAN character driver */

  priv->dev.cd_ops  = &g_sim_can_ops;
  priv->dev.cd_priv = priv;

  /* Register CAN device */

  snprintf(devpath, PATH_MAX, "/dev/can%d", devno);
  ret = can_register(devpath, &priv->dev);
  if (ret < 0)
    {
      canerr("can_register failed %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  return OK;
}

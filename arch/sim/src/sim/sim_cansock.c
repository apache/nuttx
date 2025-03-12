/****************************************************************************
 * arch/sim/src/sim/sim_cansock.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/can.h>

#include "sim_hostcan.h"
#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIM_CAN_WORK_DELAY  USEC2TICK(1000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_cansock_s
{
  struct net_driver_s dev;    /* Interface understood by the network */
  struct sim_can_s    host;   /* Host CAN handler */
  struct work_s       worker; /* Work queue for RX */
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int sim_can_ifup(struct net_driver_s *dev);
static int sim_can_ifdown(struct net_driver_s *dev);
static int sim_can_txavail(struct net_driver_s *dev);
#ifdef CONFIG_NETDEV_IOCTL
static int sim_can_netdev_ioctl(struct net_driver_s *dev, int cmd,
                                unsigned long arg);
#endif
static void sim_can_work(void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_can_ifup
 ****************************************************************************/

static int sim_can_ifup(struct net_driver_s *dev)
{
  struct sim_cansock_s *priv = (struct sim_cansock_s *)dev;
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
 * Name: sim_can_ifdown
 ****************************************************************************/

static int sim_can_ifdown(struct net_driver_s *dev)
{
  struct sim_cansock_s *priv = (struct sim_cansock_s *)dev;

  /* Cancel work */

  work_cancel(HPWORK, &priv->worker);

  return host_can_ifdown(&priv->host);
}

/****************************************************************************
 * Name: sim_can_txpoll
 ****************************************************************************/

static int sim_can_txpoll(FAR struct net_driver_s *dev)
{
  struct sim_cansock_s *priv = (struct sim_cansock_s *)dev;

  if (priv->dev.d_len > 0)
    {
      /* Send the packet */

      return host_can_send(&priv->host, priv->dev.d_buf, priv->dev.d_len);
    }

  return OK;
}

/****************************************************************************
 * Name: sim_can_txavail
 *
 * Assumption:
 *   NuttX canfd_frame is byte compatible with host canfd_frame.
 *
 ****************************************************************************/

static int sim_can_txavail(struct net_driver_s *dev)
{
  struct sim_cansock_s *priv = (struct sim_cansock_s *)dev;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (IFF_IS_UP(priv->dev.d_flags))
    {
      devif_poll(&priv->dev, sim_can_txpoll);
    }

  net_unlock();

  return OK;
}

#ifdef CONFIG_NETDEV_IOCTL
/****************************************************************************
 * Name: sim_can_netdev_ioctl
 ****************************************************************************/

static int sim_can_netdev_ioctl(struct net_driver_s *dev, int cmd,
                                unsigned long arg)
{
  return -ENOTTY;
}
#endif

/****************************************************************************
 * Name: sim_can_work
 *
 * Description:
 *   Feed pending packets on the host sockets into the CAN stack.
 *
 * Assumption:
 *   NuttX canfd_frame is byte compatible with host canfd_frame.
 *
 ****************************************************************************/

static void sim_can_work(void *arg)
{
  struct sim_cansock_s *priv = arg;
  struct canfd_frame    hframe;
  int                   ret;

  if (host_can_avail(&priv->host))
    {
      /* Wait for data from host stack */

      ret = host_can_read(&priv->host, &hframe);
      if (ret < 0)
        {
          canerr("host_can_read failed %d\n", ret);
          goto nodata;
        }

      /* Copy the buffer pointer to priv->dev..  Set amount of data
       * in priv->dev.d_len
       */

      net_lock();
      priv->dev.d_len = ret;
      priv->dev.d_buf = (FAR uint8_t *)&hframe;

      /* Send to socket interface */

      NETDEV_RXPACKETS(&priv->dev);

      can_input(&priv->dev);
      net_unlock();
    }

nodata:
  work_queue(HPWORK, &priv->worker, sim_can_work, priv, SIM_CAN_WORK_DELAY);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_cansock_initialize
 ****************************************************************************/

int sim_cansock_initialize(int devidx)
{
  struct sim_cansock_s *priv;
  int                   ret;

  /* Allocate device */

  priv = kmm_zalloc(sizeof(struct sim_cansock_s));
  if (!priv)
    {
      return -ENOMEM;
    }

  /* Initialize host interface */

  ret = host_can_init(&priv->host, devidx);
  if (ret < 0)
    {
      canerr("host_can_init failed %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* Initialize the driver structure */

  priv->dev.d_ifup    = sim_can_ifup;
  priv->dev.d_ifdown  = sim_can_ifdown;
  priv->dev.d_txavail = sim_can_txavail;
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = sim_can_netdev_ioctl;
#endif
  priv->dev.d_private = priv;

  ret = netdev_register(&priv->dev, NET_LL_CAN);
  if (ret < 0)
    {
      canerr("netdev_register failed %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  return OK;
}

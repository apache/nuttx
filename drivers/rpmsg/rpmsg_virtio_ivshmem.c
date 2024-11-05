/****************************************************************************
 * drivers/rpmsg/rpmsg_virtio_ivshmem.c
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

#include <debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/drivers/addrenv.h>
#include <nuttx/pci/pci_ivshmem.h>
#include <nuttx/rpmsg/rpmsg_virtio.h>
#include <nuttx/rpmsg/rpmsg_virtio_ivshmem.h>
#include <nuttx/wdog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define rpmsg_virtio_ivshmem_from(dev) \
  container_of(ivshmem_get_driver(dev), struct rpmsg_virtio_ivshmem_dev_s, drv)

#define RPMSG_VIRTIO_IVSHMEM_WDOG_DELAY    MSEC2TICK(1)

#define RPMSG_VIRTIO_VRING_ALIGNMENT       8

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsg_virtio_ivshmem_mem_s
{
  volatile uint64_t         basem;
  volatile uint32_t         seqs;
  volatile uint32_t         seqm;
  struct rpmsg_virtio_rsc_s rsc;
};

struct rpmsg_virtio_ivshmem_dev_s
{
  struct rpmsg_virtio_s                  dev;
  struct ivshmem_driver_s                drv;
  FAR struct ivshmem_device_s           *ivdev;
  rpmsg_virtio_callback_t                callback;
  FAR void                              *arg;
  uint32_t                               seq;
  FAR struct rpmsg_virtio_ivshmem_mem_s *shmem;
  size_t                                 shmem_size;
  struct simple_addrenv_s                addrenv[2];
  int                                    master;
  char                                   cpuname[RPMSG_NAME_SIZE + 1];
  FAR struct pci_device_s               *ivshmem;

  /* Wdog for transmit */

  struct wdog_s                          wdog;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static const FAR char *
rpmsg_virtio_ivshmem_get_cpuname(FAR struct rpmsg_virtio_s *dev);
static FAR struct rpmsg_virtio_rsc_s *
rpmsg_virtio_ivshmem_get_resource(FAR struct rpmsg_virtio_s *dev);
static int
rpmsg_virtio_ivshmem_is_master(FAR struct rpmsg_virtio_s *dev);
static int rpmsg_virtio_ivshmem_notify(FAR struct rpmsg_virtio_s *dev,
                                       uint32_t notifyid);
static int
rpmsg_virtio_ivshmem_register_callback(FAR struct rpmsg_virtio_s *dev,
                                       rpmsg_virtio_callback_t callback,
                                       FAR void *arg);
static int rpmsg_virtio_ivshmem_probe(FAR struct ivshmem_device_s *ivdev);
static void rpmsg_virtio_ivshmem_remove(FAR struct ivshmem_device_s *ivdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rpmsg_virtio_ops_s g_rpmsg_virtio_ivshmem_ops =
{
  .get_cpuname       = rpmsg_virtio_ivshmem_get_cpuname,
  .get_resource      = rpmsg_virtio_ivshmem_get_resource,
  .is_master         = rpmsg_virtio_ivshmem_is_master,
  .notify            = rpmsg_virtio_ivshmem_notify,
  .register_callback = rpmsg_virtio_ivshmem_register_callback,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const FAR char *
rpmsg_virtio_ivshmem_get_cpuname(FAR struct rpmsg_virtio_s *dev)
{
  FAR struct rpmsg_virtio_ivshmem_dev_s *priv =
    (FAR struct rpmsg_virtio_ivshmem_dev_s *)dev;
  return priv->cpuname;
}

static FAR struct rpmsg_virtio_rsc_s *
rpmsg_virtio_ivshmem_get_resource(FAR struct rpmsg_virtio_s *dev)
{
  FAR struct rpmsg_virtio_ivshmem_dev_s *priv =
    (FAR struct rpmsg_virtio_ivshmem_dev_s *)dev;
  FAR struct rpmsg_virtio_rsc_s *rsc;
  FAR struct rpmsg_virtio_cmd_s *cmd;

  rsc = &priv->shmem->rsc;
  cmd = RPMSG_VIRTIO_RSC2CMD(rsc);

  if (priv->master)
    {
      memset(priv->shmem, 0, priv->shmem_size);
      rsc->rpmsg_vdev.id            = VIRTIO_ID_RPMSG;
      rsc->rpmsg_vdev.dfeatures     = 1 << VIRTIO_RPMSG_F_NS |
                                      1 << VIRTIO_RPMSG_F_ACK;
      rsc->rpmsg_vdev.config_len    = sizeof(struct fw_rsc_config);
      rsc->rpmsg_vdev.num_of_vrings = 2;
      rsc->rpmsg_vring0.da          = 0;
      rsc->rpmsg_vring0.align       = RPMSG_VIRTIO_VRING_ALIGNMENT;
      rsc->rpmsg_vring0.num         = CONFIG_RPMSG_VIRTIO_IVSHMEM_BUFFNUM;
      rsc->rpmsg_vring1.da          = 0;
      rsc->rpmsg_vring1.align       = RPMSG_VIRTIO_VRING_ALIGNMENT;
      rsc->rpmsg_vring1.num         = CONFIG_RPMSG_VIRTIO_IVSHMEM_BUFFNUM;
      rsc->config.r2h_buf_size      = CONFIG_RPMSG_VIRTIO_IVSHMEM_BUFFSIZE;
      rsc->config.h2r_buf_size      = CONFIG_RPMSG_VIRTIO_IVSHMEM_BUFFSIZE;
      cmd->cmd_slave                = 0;

      priv->shmem->basem = (uint64_t)(uintptr_t)priv->shmem;
    }
  else
    {
      /* Wait untils master is ready, salve need use master base to
       * initialize addrenv.
       */

      while (priv->shmem->basem == 0)
        {
          usleep(1000);
        }

      cmd->cmd_master       = 0;
      priv->addrenv[0].va   = (uint64_t)(uintptr_t)priv->shmem;
      priv->addrenv[0].pa   = priv->shmem->basem;
      priv->addrenv[0].size = priv->shmem_size;

      simple_addrenv_initialize(&priv->addrenv[0]);

      priv->shmem->basem = 0;
    }

  return rsc;
}

static int rpmsg_virtio_ivshmem_is_master(FAR struct rpmsg_virtio_s *dev)
{
  FAR struct rpmsg_virtio_ivshmem_dev_s *priv =
    (FAR struct rpmsg_virtio_ivshmem_dev_s *)dev;
  return priv->master;
}

static int rpmsg_virtio_ivshmem_notify(FAR struct rpmsg_virtio_s *dev,
                                       uint32_t vqid)
{
  FAR struct rpmsg_virtio_ivshmem_dev_s *priv =
    (FAR struct rpmsg_virtio_ivshmem_dev_s *)dev;

  if (ivshmem_support_irq(priv->ivdev))
    {
      ivshmem_kick_peer(priv->ivdev);
    }
  else if (priv->master)
    {
      priv->shmem->seqm++;
    }
  else
    {
      priv->shmem->seqs++;
    }

  return 0;
}

static int
rpmsg_virtio_ivshmem_register_callback(FAR struct rpmsg_virtio_s *dev,
                                       rpmsg_virtio_callback_t callback,
                                       FAR void *arg)
{
  FAR struct rpmsg_virtio_ivshmem_dev_s *priv =
    (FAR struct rpmsg_virtio_ivshmem_dev_s *)dev;

  priv->callback = callback;
  priv->arg      = arg;
  return 0;
}

/****************************************************************************
 * Name: rpmsg_virtio_ivshmem_interrupt
 ****************************************************************************/

static int rpmsg_virtio_ivshmem_interrupt(int irq, FAR void *context,
                                          FAR void *arg)
{
  FAR struct rpmsg_virtio_ivshmem_dev_s *priv = arg;

  if (priv->callback != NULL)
    {
      priv->callback(priv->arg, RPMSG_VIRTIO_NOTIFY_ALL);
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsg_virtio_ivshmem_wdog
 ****************************************************************************/

static void rpmsg_virtio_ivshmem_wdog(wdparm_t arg)
{
  FAR struct rpmsg_virtio_ivshmem_dev_s *priv =
    (FAR struct rpmsg_virtio_ivshmem_dev_s *)arg;
  bool should_notify = false;

  if (priv->master && priv->seq != priv->shmem->seqs)
    {
      priv->seq = priv->shmem->seqs;
      should_notify = true;
    }
  else if (!priv->master && priv->seq != priv->shmem->seqm)
    {
      priv->seq = priv->shmem->seqm;
      should_notify = true;
    }

  if (should_notify && priv->callback != NULL)
    {
      priv->callback(priv->arg, RPMSG_VIRTIO_NOTIFY_ALL);
    }

  wd_start(&priv->wdog, RPMSG_VIRTIO_IVSHMEM_WDOG_DELAY,
           rpmsg_virtio_ivshmem_wdog, (wdparm_t)priv);
}

/****************************************************************************
 * Name: rpmsg_virtio_ivshmem_probe
 ****************************************************************************/

static int rpmsg_virtio_ivshmem_probe(FAR struct ivshmem_device_s *ivdev)
{
  FAR struct rpmsg_virtio_ivshmem_dev_s *priv =
    rpmsg_virtio_ivshmem_from(ivdev);
  int ret;

  /* Do the rpmsg virtio ivshmem init */

  priv->ivdev = ivdev;
  priv->dev.ops = &g_rpmsg_virtio_ivshmem_ops;
  priv->shmem = ivshmem_get_shmem(ivdev, &priv->shmem_size);

  ivshmem_attach_irq(ivdev, rpmsg_virtio_ivshmem_interrupt, priv);
  ivshmem_control_irq(ivdev, true);

  /* Do rpmsg virtio initialize */

  ret = rpmsg_virtio_initialize(&priv->dev);
  if (ret < 0)
    {
      rpmsgerr("Rpmsg virtio intialize failed, ret=%d\n", ret);
      goto err;
    }

  if (!ivshmem_support_irq(ivdev))
    {
      ret = wd_start(&priv->wdog, RPMSG_VIRTIO_IVSHMEM_WDOG_DELAY,
                     rpmsg_virtio_ivshmem_wdog, (wdparm_t)priv);
      if (ret < 0)
        {
          rpmsgerr("ERROR: wd_start failed: %d\n", ret);
          goto err;
        }
    }

  return ret;

err:
  ivshmem_control_irq(ivdev, false);
  ivshmem_detach_irq(ivdev);
  return ret;
}

/****************************************************************************
 * Name: rpmsg_virtio_ivshmem_remove
 ****************************************************************************/

static void rpmsg_virtio_ivshmem_remove(FAR struct ivshmem_device_s *ivdev)
{
  FAR struct rpmsg_virtio_ivshmem_dev_s *priv =
    rpmsg_virtio_ivshmem_from(ivdev);

  if (!ivshmem_support_irq(ivdev))
    {
      wd_cancel(&priv->wdog);
    }

  ivshmem_control_irq(ivdev, false);
  ivshmem_detach_irq(ivdev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pci_register_rpmsg_virtio_ivshmem_driver(void)
{
  FAR char *name = CONFIG_RPMSG_VIRTIO_IVSHMEM_NAME;

  while (name != NULL)
    {
      FAR const char *str;

      FAR struct rpmsg_virtio_ivshmem_dev_s * priv =
        kmm_zalloc(sizeof(*priv));
      if (priv == NULL)
        {
          return -ENOMEM;
        }

      priv->drv.id = strtoul(name, &name, 0);
      str = strchr(++name, ':');
      snprintf(priv->cpuname, RPMSG_NAME_SIZE, "%.*s", (int)(str - name),
               name);
      priv->master = *(str + 1) == 'm';

      pciinfo("Register ivshmem driver, id=%d, cpuname=%s, master=%d\n",
              priv->drv.id, priv->cpuname, priv->master);

      priv->drv.probe = rpmsg_virtio_ivshmem_probe;
      priv->drv.remove = rpmsg_virtio_ivshmem_remove;
      if (ivshmem_register_driver(&priv->drv) < 0)
        {
          kmm_free(priv);
        }

      name = strchr(str, ';');
      if (name != NULL)
        {
          name++;
        }
    }

  return 0;
}

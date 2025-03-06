/****************************************************************************
 * drivers/rptun/rptun_ivshmem.c
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

#include <nuttx/nuttx.h>
#include <nuttx/drivers/addrenv.h>
#include <nuttx/pci/pci_ivshmem.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/rptun/rptun_ivshmem.h>
#include <nuttx/wdog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define rptun_ivshmem_from_ivdev(dev) \
  container_of(ivshmem_get_driver(dev), struct rptun_ivshmem_dev_s, drv)

#define RPTUN_IVSHMEM_SHMEM_BAR   2
#define RPTUN_IVSHMEM_SHMEM_SIZE  0x10000
#define RPTUN_IVSHMEM_WDOG_DELAY  USEC2TICK(100)

#define RPTUN_IVSHMEM_VIRTIO_NUM  3
#define RPTUN_IVSHMEM_RSC_NUM     (2 * RPTUN_IVSHMEM_VIRTIO_NUM)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct aligned_data(8) rptun_ivshmem_rsc_s
{
  struct resource_table      hdr;
  uint32_t                   offset[RPTUN_IVSHMEM_RSC_NUM];
  struct fw_rsc_vdev         rng0;
  struct fw_rsc_vdev_vring   rng0_vring;
  struct fw_rsc_carveout     rng0_carveout;
  char                       rng0_shm[RPTUN_IVSHMEM_SHMEM_SIZE];
  struct fw_rsc_vdev         rng1;
  struct fw_rsc_vdev_vring   rng1_vring;
  struct fw_rsc_carveout     rng1_carveout;
  char                       rng1_shm[RPTUN_IVSHMEM_SHMEM_SIZE];
  struct fw_rsc_vdev         rpmsg0;
  struct fw_rsc_vdev_vring   rpmsg0_vring0;
  struct fw_rsc_vdev_vring   rpmsg0_vring1;
  struct fw_rsc_config       rpmsg0_config;
  struct fw_rsc_carveout     rpmsg0_carveout;
  char                       rpmsg0_shm[RPTUN_IVSHMEM_SHMEM_SIZE];
};

struct rptun_ivshmem_mem_s
{
  volatile uint64_t          basem;
  volatile uint32_t          seqs;
  volatile uint32_t          seqm;
  volatile uint32_t          reserved;
  volatile uint32_t          rsc_size;
  struct rptun_ivshmem_rsc_s rsc;
};

struct rptun_ivshmem_dev_s
{
  struct rptun_dev_s              rptun;
  struct ivshmem_driver_s         drv;
  rptun_callback_t                callback;
  FAR void                       *arg;
  uint32_t                        seq;
  FAR struct rptun_ivshmem_mem_s *shmem;
  size_t                          shmem_size;
  struct simple_addrenv_s         addrenv[2];
  struct rptun_addrenv_s          raddrenv[2];
  bool                            master;
  char                            cpuname[RPMSG_NAME_SIZE + 1];
  FAR struct ivshmem_device_s    *ivdev;

  /* Wdog for transmit */

  struct wdog_s                   wdog;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static const char *rptun_ivshmem_get_cpuname(FAR struct rptun_dev_s *dev);
static const FAR struct rptun_addrenv_s *
rptun_ivshmem_get_addrenv(FAR struct rptun_dev_s *dev);
static FAR struct resource_table *
rptun_ivshmem_get_resource(FAR struct rptun_dev_s *dev);
static bool rptun_ivshmem_is_autostart(FAR struct rptun_dev_s *dev);
static bool rptun_ivshmem_is_master(FAR struct rptun_dev_s *dev);
static int rptun_ivshmem_start(FAR struct rptun_dev_s *dev);
static int rptun_ivshmem_stop(FAR struct rptun_dev_s *dev);
static int rptun_ivshmem_notify(FAR struct rptun_dev_s *dev,
                                uint32_t notifyid);
static int rptun_ivshmem_register_callback(FAR struct rptun_dev_s *dev,
                                           rptun_callback_t callback,
                                           FAR void *arg);

static void rptun_ivshmem_wdog(wdparm_t arg);
static int rptun_ivshmem_probe(FAR struct ivshmem_device_s *dev);
static void rptun_ivshmem_remove(FAR struct ivshmem_device_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_rptun_ivshmem_ops =
{
  .get_cpuname       = rptun_ivshmem_get_cpuname,
  .get_addrenv       = rptun_ivshmem_get_addrenv,
  .get_resource      = rptun_ivshmem_get_resource,
  .is_autostart      = rptun_ivshmem_is_autostart,
  .is_master         = rptun_ivshmem_is_master,
  .start             = rptun_ivshmem_start,
  .stop              = rptun_ivshmem_stop,
  .notify            = rptun_ivshmem_notify,
  .register_callback = rptun_ivshmem_register_callback,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const FAR char *rptun_ivshmem_get_cpuname(FAR struct rptun_dev_s *dev)
{
  FAR struct rptun_ivshmem_dev_s *priv =
    (FAR struct rptun_ivshmem_dev_s *)dev;
  return priv->cpuname;
}

static const FAR struct rptun_addrenv_s *
rptun_ivshmem_get_addrenv(FAR struct rptun_dev_s *dev)
{
  FAR struct rptun_ivshmem_dev_s *priv =
    (FAR struct rptun_ivshmem_dev_s *)dev;
  return &priv->raddrenv[0];
}

static FAR struct resource_table *
rptun_ivshmem_get_resource(FAR struct rptun_dev_s *dev)
{
  FAR struct rptun_ivshmem_dev_s *priv =
    (FAR struct rptun_ivshmem_dev_s *)dev;
  FAR struct rptun_cmd_s *cmd = RPTUN_RSC2CMD(&priv->shmem->rsc);

  if (priv->master)
    {
      /* Wait until salve is ready */

      while (RPTUN_GET_CMD(cmd->cmd_slave) != RPTUN_CMD_READY)
        {
          usleep(1000);
        }

      cmd->cmd_slave = 0;
      priv->shmem->basem = (uint64_t)(uintptr_t)priv->shmem;
    }
  else
    {
      FAR struct rptun_ivshmem_rsc_s *rsc = &priv->shmem->rsc;
      memset(priv->shmem, 0, priv->shmem_size);

      rsc->hdr.ver                    = 1;
      rsc->hdr.num                    = RPTUN_IVSHMEM_RSC_NUM;

      /* Virtio Driver 0, VIRTIO_ID_ENTROPY */

      rsc->offset[0]                  = offsetof(struct rptun_ivshmem_rsc_s,
                                                 rng0);
      rsc->rng0.type                  = RSC_VDEV;
      rsc->rng0.id                    = VIRTIO_ID_ENTROPY;
      rsc->rng0.notifyid              = RSC_NOTIFY_ID_ANY;
      rsc->rng0.dfeatures             = 0;
      rsc->rng0.config_len            = 0;
      rsc->rng0.num_of_vrings         = 1;
      rsc->rng0.reserved[0]           = VIRTIO_DEV_DRIVER;
      rsc->rng0.reserved[1]           = 0;
      rsc->rng0_vring.align           = 8;
      rsc->rng0_vring.num             = 8;
      rsc->rng0_vring.notifyid        = RSC_NOTIFY_ID_ANY;
      rsc->rng0_vring.da              = FW_RSC_U32_ADDR_ANY;

      /* Virtio Rng0 share memory buffer */

      rsc->offset[1]                  = offsetof(struct rptun_ivshmem_rsc_s,
                                                 rng0_carveout);
      rsc->rng0_carveout.type         = RSC_CARVEOUT;
      rsc->rng0_carveout.da           = offsetof(struct rptun_ivshmem_mem_s,
                                                 rsc.rng0_shm);
      rsc->rng0_carveout.pa           = (uint32_t)METAL_BAD_PHYS;
      rsc->rng0_carveout.len          = sizeof(priv->shmem->rsc.rng0_shm);
      memcpy(rsc->rng0_carveout.name, "vdev0buffer", 11);

      /* Virtio Driver 1, VIRTIO_ID_ENTROPY */

      rsc->offset[2]                  = offsetof(struct rptun_ivshmem_rsc_s,
                                                 rng1);
      rsc->rng1.type                  = RSC_VDEV;
      rsc->rng1.id                    = VIRTIO_ID_ENTROPY;
      rsc->rng1.notifyid              = RSC_NOTIFY_ID_ANY;
      rsc->rng1.dfeatures             = 0;
      rsc->rng1.config_len            = 0;
      rsc->rng1.num_of_vrings         = 1;
      rsc->rng1.reserved[0]           = VIRTIO_DEV_DRIVER;
      rsc->rng1.reserved[1]           = 0;
      rsc->rng1_vring.align           = 8;
      rsc->rng1_vring.num             = 8;
      rsc->rng1_vring.notifyid        = RSC_NOTIFY_ID_ANY;
      rsc->rng1_vring.da              = FW_RSC_U32_ADDR_ANY;

      /* Virtio Rng1 share memory buffer */

      rsc->offset[3]                  = offsetof(struct rptun_ivshmem_rsc_s,
                                                 rng1_carveout);
      rsc->rng1_carveout.type         = RSC_CARVEOUT;
      rsc->rng1_carveout.da           = offsetof(struct rptun_ivshmem_mem_s,
                                                 rsc.rng1_shm);
      rsc->rng1_carveout.pa           = (uint32_t)METAL_BAD_PHYS;
      rsc->rng1_carveout.len          = sizeof(priv->shmem->rsc.rng1_shm);
      memcpy(rsc->rng1_carveout.name, "vdev1buffer", 11);

      /* Virtio Driver 2, VIRTIO_ID_RPMSG */

      rsc->offset[4]                  = offsetof(struct rptun_ivshmem_rsc_s,
                                                 rpmsg0);
      rsc->rpmsg0.type                = RSC_VDEV;
      rsc->rpmsg0.id                  = VIRTIO_ID_RPMSG;
      rsc->rpmsg0.notifyid            = RSC_NOTIFY_ID_ANY;
      rsc->rpmsg0.dfeatures           = (1 << VIRTIO_RPMSG_F_NS) |
                                        (1 << VIRTIO_RPMSG_F_ACK) |
                                        (1 << VIRTIO_RPMSG_F_BUFSZ) |
                                        (1 << VIRTIO_RPMSG_F_CPUNAME);
      rsc->rpmsg0.config_len          = sizeof(struct fw_rsc_config);
      rsc->rpmsg0.num_of_vrings       = 2;
      rsc->rpmsg0.reserved[0]         = VIRTIO_DEV_DRIVER;
      rsc->rpmsg0.reserved[1]         = 0;
      rsc->rpmsg0_vring0.align        = 8;
      rsc->rpmsg0_vring0.num          = 8;
      rsc->rpmsg0_vring0.notifyid     = RSC_NOTIFY_ID_ANY;
      rsc->rpmsg0_vring0.da           = FW_RSC_U32_ADDR_ANY;
      rsc->rpmsg0_vring1.align        = 8;
      rsc->rpmsg0_vring1.num          = 8;
      rsc->rpmsg0_vring1.notifyid     = RSC_NOTIFY_ID_ANY;
      rsc->rpmsg0_vring1.da           = FW_RSC_U32_ADDR_ANY;
      rsc->rpmsg0_config.h2r_buf_size = 0x600;
      rsc->rpmsg0_config.r2h_buf_size = 0x600;
      strlcpy((FAR char *)rsc->rpmsg0_config.host_cpuname,
              priv->cpuname, VIRTIO_RPMSG_CPUNAME_SIZE);
      strlcpy((FAR char *)rsc->rpmsg0_config.remote_cpuname,
              CONFIG_RPMSG_LOCAL_CPUNAME, VIRTIO_RPMSG_CPUNAME_SIZE);

      /* Virtio Rpmsg0 share memory buffer */

      rsc->offset[5]                  = offsetof(struct rptun_ivshmem_rsc_s,
                                                 rpmsg0_carveout);
      rsc->rpmsg0_carveout.type       = RSC_CARVEOUT;
      rsc->rpmsg0_carveout.da         = offsetof(struct rptun_ivshmem_mem_s,
                                                 rsc.rpmsg0_shm);
      rsc->rpmsg0_carveout.pa         = (uint32_t)METAL_BAD_PHYS;
      rsc->rpmsg0_carveout.len        = sizeof(priv->shmem->rsc.rpmsg0_shm);
      memcpy(rsc->rpmsg0_carveout.name, "vdev2buffer", 11);

      priv->shmem->rsc_size           = sizeof(struct rptun_ivshmem_rsc_s);
      cmd->cmd_slave                  = RPTUN_CMD(RPTUN_CMD_READY, 0);

      /* Wait until master is ready, slave needs to use master base to
       * initialize addrenv.
       */

      while (priv->shmem->basem == 0)
        {
          usleep(1000);
        }

      priv->addrenv[0].va   = (uint64_t)(uintptr_t)priv->shmem;
      priv->addrenv[0].pa   = priv->shmem->basem;
      priv->addrenv[0].size = priv->shmem_size;

      simple_addrenv_initialize(&priv->addrenv[0]);
    }

  priv->raddrenv[0].pa   = priv->master ? (uintptr_t)priv->shmem :
                                          (uintptr_t)priv->shmem->basem;
  priv->raddrenv[0].da   = 0;
  priv->raddrenv[0].size = priv->shmem_size;

  return &priv->shmem->rsc.hdr;
}

static bool rptun_ivshmem_is_autostart(FAR struct rptun_dev_s *dev)
{
  return true;
}

static bool rptun_ivshmem_is_master(FAR struct rptun_dev_s *dev)
{
  FAR struct rptun_ivshmem_dev_s *priv =
    (FAR struct rptun_ivshmem_dev_s *)dev;
  return priv->master;
}

static int rptun_ivshmem_start(FAR struct rptun_dev_s *dev)
{
  FAR struct rptun_ivshmem_dev_s *priv =
    (FAR struct rptun_ivshmem_dev_s *)dev;

  if (ivshmem_support_irq(priv->ivdev))
    {
      return 0;
    }

  return wd_start(&priv->wdog, 0, rptun_ivshmem_wdog, (wdparm_t)priv);
}

static int rptun_ivshmem_stop(FAR struct rptun_dev_s *dev)
{
  FAR struct rptun_ivshmem_dev_s *priv =
    (FAR struct rptun_ivshmem_dev_s *)dev;

  if (ivshmem_support_irq(priv->ivdev))
    {
      return 0;
    }

  return wd_cancel(&priv->wdog);
}

static int rptun_ivshmem_notify(FAR struct rptun_dev_s *dev, uint32_t vqid)
{
  FAR struct rptun_ivshmem_dev_s *priv =
    (FAR struct rptun_ivshmem_dev_s *)dev;

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

static int rptun_ivshmem_register_callback(FAR struct rptun_dev_s *dev,
                                           rptun_callback_t callback,
                                           FAR void *arg)
{
  FAR struct rptun_ivshmem_dev_s *priv =
    (FAR struct rptun_ivshmem_dev_s *)dev;

  priv->callback = callback;
  priv->arg      = arg;
  return 0;
}

/****************************************************************************
 * Name: rptun_ivshmem_interrupt
 ****************************************************************************/

static int rptun_ivshmem_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct rptun_ivshmem_dev_s *priv = arg;

  if (priv->callback != NULL)
    {
      priv->callback(priv->arg, RPTUN_NOTIFY_ALL);
    }

  return 0;
}

/****************************************************************************
 * Name: rptun_ivshmem_wdog
 ****************************************************************************/

static void rptun_ivshmem_wdog(wdparm_t arg)
{
  FAR struct rptun_ivshmem_dev_s *priv =
    (FAR struct rptun_ivshmem_dev_s *)arg;
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
      priv->callback(priv->arg, RPTUN_NOTIFY_ALL);
    }

  wd_start(&priv->wdog, RPTUN_IVSHMEM_WDOG_DELAY, rptun_ivshmem_wdog,
           (wdparm_t)priv);
}

/****************************************************************************
 * Name: rptun_ivshmem_probe
 ****************************************************************************/

static int rptun_ivshmem_probe(FAR struct ivshmem_device_s *ivdev)
{
  FAR struct rptun_ivshmem_dev_s *priv = rptun_ivshmem_from_ivdev(ivdev);
  int ret;

  /* Do the rptun ivshmem init */

  priv->rptun.ops = &g_rptun_ivshmem_ops;
  priv->ivdev = ivdev;
  priv->shmem = ivshmem_get_shmem(ivdev, &priv->shmem_size);

  ivshmem_attach_irq(ivdev, rptun_ivshmem_interrupt, priv);
  ivshmem_control_irq(ivdev, true);

  pciinfo("shmem addr=%p size=%zu\n", priv->shmem, priv->shmem_size);

  /* Do rptun initialize */

  ret = rptun_initialize(&priv->rptun);
  if (ret < 0)
    {
      pcierr("rptun initialize failed, ret=%d\n", ret);
      goto err;
    }

  if (!priv->master && !ivshmem_support_irq(ivdev))
    {
      pciinfo("Start the wdog\n");
      wd_start(&priv->wdog, 0, rptun_ivshmem_wdog, (wdparm_t)priv);
    }

  return ret;

err:
  ivshmem_unregister_driver(&priv->drv);
  ivshmem_control_irq(ivdev, false);
  ivshmem_detach_irq(ivdev);
  return ret;
}

/****************************************************************************
 * Name: rptun_ivshmem_remove
 ****************************************************************************/

static void rptun_ivshmem_remove(FAR struct ivshmem_device_s *ivdev)
{
  FAR struct rptun_ivshmem_dev_s *priv = rptun_ivshmem_from_ivdev(ivdev);

  ivshmem_unregister_driver(&priv->drv);
  ivshmem_control_irq(ivdev, false);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pci_register_rptun_ivshmem_driver(void)
{
  FAR struct rptun_ivshmem_dev_s *priv;
  FAR const char *str;
  FAR char *name = CONFIG_RPTUN_IVSHMEM_NAME;

  while (name != NULL)
    {
      priv = kmm_zalloc(sizeof(*priv));
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

      priv->drv.probe = rptun_ivshmem_probe;
      priv->drv.remove = rptun_ivshmem_remove;
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


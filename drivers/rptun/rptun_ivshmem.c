/****************************************************************************
 * drivers/rptun/rptun_ivshmem.c
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
#include <nuttx/pci/pci.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/rptun/rptun_ivshmem.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPTUN_IVSHMEM_SHMEM_BAR   2
#define RPTUN_IVSHMEM_READY       1
#define RPTUN_IVSHMEM_WORK_DELAY  1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rptun_ivshmem_mem_s
{
  volatile uint64_t  basem;
  volatile uint32_t  seqs;
  volatile uint32_t  seqm;
  volatile uint32_t  cmds;
  volatile uint32_t  cmdm;
  volatile uint32_t  reserved;
  volatile uint32_t  rsc_size;
  struct rptun_rsc_s rsc;
};

struct rptun_ivshmem_dev_s
{
  struct rptun_dev_s              rptun;
  rptun_callback_t                callback;
  FAR void                       *arg;
  uint32_t                        seq;
  FAR struct rptun_ivshmem_mem_s *shmem;
  size_t                          shmem_size;
  struct simple_addrenv_s         addrenv;
  struct rptun_addrenv_s          raddrenv;
  bool                            master;
  char                            cpuname[RPMSG_NAME_SIZE + 1];
  FAR struct pci_device_s        *ivshmem;

  /* Work queue for transmit */

  struct work_s                   worker;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static const char *rptun_ivshmem_get_cpuname(FAR struct rptun_dev_s *dev);
static const FAR struct rptun_addrenv_s *
rptun_ivshmem_get_addrenv(FAR struct rptun_dev_s *dev);
static FAR struct rptun_rsc_s *
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

static void rptun_ivshmem_work(FAR void *arg);
static int rptun_ivshmem_probe(FAR struct pci_device_s *dev);
static void rptun_ivshmem_remove(FAR struct pci_device_s *dev);

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

static const struct pci_device_id_s g_rptun_ivshmem_ids[] =
{
  { PCI_DEVICE(0x1af4, 0x1110) },
  { 0, }
};

static struct pci_driver_s g_rptun_ivshmem_drv =
{
  g_rptun_ivshmem_ids,  /* PCI id_tables */
  rptun_ivshmem_probe,  /* Probe function */
  rptun_ivshmem_remove, /* Remove function */
};

static int g_rptun_ivshmem_idx = 0;

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
  return &priv->raddrenv;
}

static FAR struct rptun_rsc_s *
rptun_ivshmem_get_resource(FAR struct rptun_dev_s *dev)
{
  FAR struct rptun_ivshmem_dev_s *priv =
    (FAR struct rptun_ivshmem_dev_s *)dev;

  priv->raddrenv.da   = 0;
  priv->raddrenv.pa   = (uintptr_t)priv->shmem;
  priv->raddrenv.size = priv->shmem_size;

  if (priv->master)
    {
      /* Wait untils salve is ready */

      while (priv->shmem->cmds != RPTUN_IVSHMEM_READY)
        {
          usleep(1000);
        }

      priv->shmem->cmds  = 0;
      priv->shmem->basem = (uint64_t)priv->shmem;
    }
  else
    {
      FAR struct rptun_rsc_s *rsc = &priv->shmem->rsc;

      memset(priv->shmem, 0, priv->shmem_size);

      rsc->rsc_tbl_hdr.ver          = 1;
      rsc->rsc_tbl_hdr.num          = 1;
      rsc->offset[0]                = offsetof(struct rptun_rsc_s,
                                               rpmsg_vdev);
      rsc->rpmsg_vdev.type          = RSC_VDEV;
      rsc->rpmsg_vdev.id            = VIRTIO_ID_RPMSG;
      rsc->rpmsg_vdev.notifyid      = 20;
      rsc->rpmsg_vdev.dfeatures     = 1 << VIRTIO_RPMSG_F_NS |
                                      1 << VIRTIO_RPMSG_F_ACK |
                                      1 << VIRTIO_RPMSG_F_BUFSZ;
      rsc->rpmsg_vdev.config_len    = sizeof(struct fw_rsc_config);
      rsc->rpmsg_vdev.num_of_vrings = 2;
      rsc->rpmsg_vring0.da          = FW_RSC_U32_ADDR_ANY;
      rsc->rpmsg_vring0.align       = 8;
      rsc->rpmsg_vring0.num         = CONFIG_RPTUN_IVSHMEM_BUFFNUM;
      rsc->rpmsg_vring0.notifyid    = RSC_NOTIFY_ID_ANY;
      rsc->rpmsg_vring1.da          = FW_RSC_U32_ADDR_ANY;
      rsc->rpmsg_vring1.align       = 8;
      rsc->rpmsg_vring1.num         = CONFIG_RPTUN_IVSHMEM_BUFFNUM;
      rsc->rpmsg_vring1.notifyid    = RSC_NOTIFY_ID_ANY;
      rsc->config.r2h_buf_size      = CONFIG_RPTUN_IVSHMEM_BUFFSIZE;
      rsc->config.h2r_buf_size      = CONFIG_RPTUN_IVSHMEM_BUFFSIZE;

      priv->shmem->rsc_size         = sizeof(struct rptun_rsc_s);
      priv->shmem->cmds             = RPTUN_IVSHMEM_READY;

      /* Wait untils master is ready, salve need use master base to
       * initialize addrenv.
       */

      while (priv->shmem->basem == 0)
        {
          usleep(1000);
        }

      priv->addrenv.va   = (uint64_t)priv->shmem;
      priv->addrenv.pa   = priv->shmem->basem;
      priv->addrenv.size = priv->shmem_size;

      simple_addrenv_initialize(&priv->addrenv);
    }

  return &priv->shmem->rsc;
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
  return work_queue(HPWORK, &priv->worker, rptun_ivshmem_work, priv, 0);
}

static int rptun_ivshmem_stop(FAR struct rptun_dev_s *dev)
{
  FAR struct rptun_ivshmem_dev_s *priv =
    (FAR struct rptun_ivshmem_dev_s *)dev;
  return work_cancel_sync(HPWORK, &priv->worker);
}

static int rptun_ivshmem_notify(FAR struct rptun_dev_s *dev, uint32_t vqid)
{
  FAR struct rptun_ivshmem_dev_s *priv =
    (FAR struct rptun_ivshmem_dev_s *)dev;

  if (priv->master)
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
 * Name: rptun_ivshmem_work
 ****************************************************************************/

static void rptun_ivshmem_work(FAR void *arg)
{
  FAR struct rptun_ivshmem_dev_s *priv = arg;
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

  work_queue(HPWORK, &priv->worker, rptun_ivshmem_work, priv,
             RPTUN_IVSHMEM_WORK_DELAY);
}

/****************************************************************************
 * Name: rptun_ivshmem_get_info
 ****************************************************************************/

static int rptun_ivshmem_get_info(FAR char *cpuname, FAR bool *master)
{
  FAR const char *name = CONFIG_RPTUN_IVSHMEM_NAME;
  int start = 0;
  int i;
  int j;

  for (i = 0, j = 0; name[start] != '\0'; i++)
    {
      if (name[i] == ';' || name[i] == '\0')
        {
          if (j++ == g_rptun_ivshmem_idx)
            {
              snprintf(cpuname, RPMSG_NAME_SIZE, "%.*s", i - start - 2,
                       &name[start]);
              *master = name[i - 1] == 'm';
              return 0;
            }

          start = i + 1;
        }
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: rptun_ivshmem_probe
 ****************************************************************************/

static int rptun_ivshmem_probe(FAR struct pci_device_s *dev)
{
  FAR struct rptun_ivshmem_dev_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  /* Do the rptun ivshmem init */

  priv->rptun.ops = &g_rptun_ivshmem_ops;
  priv->ivshmem = dev;
  ret = rptun_ivshmem_get_info(priv->cpuname, &priv->master);
  if (ret < 0)
    {
      goto err_priv;
    }

  /* Configure the ivshmem device and get share memory address */

  ret = pci_enable_device(dev);
  if (ret < 0)
    {
      pcierr("Enable device failed, ret=%d\n", ret);
      goto err_priv;
    }

  pci_set_master(dev);

  priv->shmem = (FAR struct rptun_ivshmem_mem_s *)
    pci_map_bar(dev, RPTUN_IVSHMEM_SHMEM_BAR);
  if (priv->shmem == NULL)
    {
      ret = -ENOTSUP;
      pcierr("Device not support share memory bar\n");
      goto err_master;
    }

  priv->shmem_size = pci_resource_len(dev, RPTUN_IVSHMEM_SHMEM_BAR);

  pciinfo("shmem addr=%p size=%zu\n", priv->shmem, priv->shmem_size);

  /* Do rptun initialize */

  ret = rptun_initialize(&priv->rptun);
  if (ret < 0)
    {
      pcierr("rptun intialize failed, ret=%d\n", ret);
      goto err_master;
    }

  if (!priv->master)
    {
      /* Queue the worker for slave, master will do this in ops->start() */

      work_queue(HPWORK, &priv->worker, rptun_ivshmem_work, priv, 0);
    }

  g_rptun_ivshmem_idx++;
  return ret;

err_master:
  pci_clear_master(dev);
  pci_disable_device(dev);
err_priv:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: rptun_ivshmem_remove
 ****************************************************************************/

static void rptun_ivshmem_remove(FAR struct pci_device_s *dev)
{
  pciwarn("Not support remove for now\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pci_register_rptun_ivshmem_driver(void)
{
  return pci_register_driver(&g_rptun_ivshmem_drv);
}


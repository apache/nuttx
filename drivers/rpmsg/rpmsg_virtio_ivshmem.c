/****************************************************************************
 * drivers/rpmsg/rpmsg_virtio_ivshmem.c
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
#include <nuttx/rpmsg/rpmsg_virtio.h>
#include <nuttx/rpmsg/rpmsg_virtio_ivshmem.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSG_VIRTIO_IVSHMEM_SHMEM_BAR     2
#define RPMSG_VIRTIO_IVSHMEM_READY         0x1
#define RPMSG_VIRTIO_IVSHMEM_WORK_DELAY    1

#define RPMSG_VIRTIO_VRING_ALIGNMENT       8

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsg_virtio_ivshmem_mem_s
{
  volatile uint64_t         basem;
  volatile uint32_t         seqs;
  volatile uint32_t         seqm;
  volatile uint32_t         cmds;
  volatile uint32_t         cmdm;
  struct rpmsg_virtio_rsc_s rsc;
};

struct rpmsg_virtio_ivshmem_dev_s
{
  struct rpmsg_virtio_s                  dev;
  rpmsg_virtio_callback_t                callback;
  FAR void                              *arg;
  uint32_t                               seq;
  FAR struct rpmsg_virtio_ivshmem_mem_s *shmem;
  size_t                                 shmem_size;
  struct simple_addrenv_s                addrenv;
  int                                    master;
  char                                   cpuname[RPMSG_NAME_SIZE + 1];
  FAR struct pci_device_s               *ivshmem;

  /* Work queue for transmit */

  struct work_s                          worker;
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

static int rpmsg_virtio_ivshmem_probe(FAR struct pci_device_s *dev);
static void rpmsg_virtio_ivshmem_remove(FAR struct pci_device_s *dev);

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

static const struct pci_device_id_s g_rpmsg_virtio_ivshmem_ids[] =
{
  { PCI_DEVICE(0x1af4, 0x1110) },
  { 0, }
};

static struct pci_driver_s g_rpmsg_virtio_ivshmem_drv =
{
  g_rpmsg_virtio_ivshmem_ids,  /* PCI id_tables */
  rpmsg_virtio_ivshmem_probe,  /* Probe function */
  rpmsg_virtio_ivshmem_remove, /* Remove function */
};

static int g_rpmsg_virtio_ivshmem_idx = 0;

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

  rsc = &priv->shmem->rsc;

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

      priv->addrenv.va   = (uint64_t)(uintptr_t)priv->shmem;
      priv->addrenv.pa   = priv->shmem->basem;
      priv->addrenv.size = priv->shmem_size;

      simple_addrenv_initialize(&priv->addrenv);

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
 * Name: rpmsg_virtio_ivshmem_work
 ****************************************************************************/

static void rpmsg_virtio_ivshmem_work(FAR void *arg)
{
  FAR struct rpmsg_virtio_ivshmem_dev_s *priv = arg;
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

  work_queue(HPWORK, &priv->worker, rpmsg_virtio_ivshmem_work, priv,
             RPMSG_VIRTIO_IVSHMEM_WORK_DELAY);
}

/****************************************************************************
 * Name: rpmsg_virtio_ivshmem_get_info
 ****************************************************************************/

static int rpmsg_virtio_ivshmem_get_info(FAR char *cpuname, FAR int *master)
{
  FAR const char *name = CONFIG_RPMSG_VIRTIO_IVSHMEM_NAME;
  int start = 0;
  int i;
  int j;

  for (i = 0, j = 0; name[start] != '\0'; i++)
    {
      if (name[i] == ';' || name[i] == '\0')
        {
          if (j++ == g_rpmsg_virtio_ivshmem_idx)
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
 * Name: rpmsg_virtio_ivshmem_probe
 ****************************************************************************/

static int rpmsg_virtio_ivshmem_probe(FAR struct pci_device_s *dev)
{
  FAR struct rpmsg_virtio_ivshmem_dev_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  /* Do the rpmsg virtio ivshmem init */

  priv->dev.ops = &g_rpmsg_virtio_ivshmem_ops;
  priv->ivshmem = dev;
  ret = rpmsg_virtio_ivshmem_get_info(priv->cpuname, &priv->master);
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

  priv->shmem = (FAR struct rpmsg_virtio_ivshmem_mem_s *)
    pci_map_bar(dev, RPMSG_VIRTIO_IVSHMEM_SHMEM_BAR);
  if (priv->shmem == NULL)
    {
      ret = -ENOTSUP;
      pcierr("Device not support share memory bar\n");
      goto err_master;
    }

  priv->shmem_size = pci_resource_len(dev, RPMSG_VIRTIO_IVSHMEM_SHMEM_BAR);

  pciinfo("shmem addr=%p size=%zu\n", priv->shmem, priv->shmem_size);

  /* Do rpmsg virtio initialize */

  ret = rpmsg_virtio_initialize(&priv->dev);
  if (ret < 0)
    {
      pcierr("rpmsg virtio intialize failed, ret=%d\n", ret);
      goto err_master;
    }

  work_queue(HPWORK, &priv->worker, rpmsg_virtio_ivshmem_work, priv, 0);
  g_rpmsg_virtio_ivshmem_idx++;
  return ret;

err_master:
  pci_clear_master(dev);
  pci_disable_device(dev);
err_priv:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: rpmsg_virtio_ivshmem_remove
 ****************************************************************************/

static void rpmsg_virtio_ivshmem_remove(FAR struct pci_device_s *dev)
{
  pciwarn("Not support remove for now\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pci_register_rpmsg_virtio_ivshmem_driver(void)
{
  return pci_register_driver(&g_rpmsg_virtio_ivshmem_drv);
}

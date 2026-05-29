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

#include <nuttx/debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/nuttx.h>
#include <nuttx/drivers/addrenv.h>
#include <nuttx/pci/pci_ivshmem.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/rptun/rptun_ivshmem.h>
#include <nuttx/virtio/virtio-config.h>
#include <nuttx/virtio/virtio-vsock.h>
#include <nuttx/wdog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define rptun_ivshmem_from_ivdev(dev) \
  container_of(ivshmem_get_driver(dev), struct rptun_ivshmem_dev_s, drv)

#define RPTUN_IVSHMEM_SHMEM_BAR   2
#define RPTUN_IVSHMEM_SHMEM_SIZE  0x10000
#define RPTUN_IVSHMEM_WDOG_DELAY  USEC2TICK(100)

#ifdef CONFIG_DRIVERS_VHOST_RNG
#  define RPTUN_IVSHMEM_RNG       1
#else
#  define RPTUN_IVSHMEM_RNG       0
#endif

#ifdef CONFIG_DRIVERS_VHOST_RPMSG
#  define RPTUN_IVSHMEM_RPMSG     1
#else
#  define RPTUN_IVSHMEM_RPMSG     0
#endif

#ifdef CONFIG_NET_VSOCK_VHOST
#  define RPTUN_IVSHMEM_VSOCK     1
#else
#  define RPTUN_IVSHMEM_VSOCK     0
#endif

#ifdef CONFIG_DRIVERS_VHOST_NET
#  define RPTUN_IVSHMEM_NET       1
#else
#  define RPTUN_IVSHMEM_NET       0
#endif

#define RPTUN_IVSHMEM_VIRTIO_NUM  (RPTUN_IVSHMEM_RNG + \
                                   RPTUN_IVSHMEM_RPMSG + \
                                   RPTUN_IVSHMEM_VSOCK + \
                                   RPTUN_IVSHMEM_NET)
#define RPTUN_IVSHMEM_RSC_NUM     (2 * RPTUN_IVSHMEM_VIRTIO_NUM)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct aligned_data(8) rptun_ivshmem_rsc_s
{
  struct resource_table        hdr;
  uint32_t                     offset[RPTUN_IVSHMEM_RSC_NUM];
#ifdef CONFIG_DRIVERS_VHOST_RNG
  struct fw_rsc_vdev           rng;
  struct fw_rsc_vdev_vring     rng_vring;
  struct fw_rsc_carveout       rng_carveout;
  char                         rng_shm[RPTUN_IVSHMEM_SHMEM_SIZE];
#endif
#ifdef CONFIG_DRIVERS_VHOST_RPMSG
  struct fw_rsc_vdev           rpmsg;
  struct fw_rsc_vdev_vring     rpmsg_vring0;
  struct fw_rsc_vdev_vring     rpmsg_vring1;
  struct fw_rsc_config         rpmsg_config;
  struct fw_rsc_carveout       rpmsg_carveout;
  char                         rpmsg_shm[RPTUN_IVSHMEM_SHMEM_SIZE];
#endif
#ifdef CONFIG_NET_VSOCK_VHOST
  struct fw_rsc_vdev           vsock;
  struct fw_rsc_vdev_vring     vsock_vring0;
  struct fw_rsc_vdev_vring     vsock_vring1;
  struct fw_rsc_vdev_vring     vsock_vring2;
  struct virtio_vsock_config_s vsock_config;
  struct fw_rsc_carveout       vsock_carveout;
  char                         vsock_shm[8 * RPTUN_IVSHMEM_SHMEM_SIZE];
#endif
#ifdef CONFIG_DRIVERS_VHOST_NET
  struct fw_rsc_vdev           net;
  struct fw_rsc_vdev_vring     net_vring0;
  struct fw_rsc_vdev_vring     net_vring1;
  struct fw_rsc_carveout       net_carveout;
  char                         net_shm[RPTUN_IVSHMEM_SHMEM_SIZE];
#endif
};

struct rptun_ivshmem_mem_s
{
  volatile uint64_t     basem;
  volatile uint32_t     seqs;
  volatile uint32_t     seqm;
  volatile uint32_t     reserved;
  volatile uint32_t     rsc_size;
  struct resource_table rsc_table;
};

struct rptun_ivshmem_dev_s
{
  struct rptun_dev_s               rptun;
  struct ivshmem_driver_s          drv;
  rptun_callback_t                 callback;
  FAR void                        *arg;
  uint32_t                         seq;
  FAR struct rptun_ivshmem_mem_s  *shmem;
  size_t                           shmem_size;
  struct simple_addrenv_s          addrenv[2];
  struct rptun_addrenv_s           raddrenv[2];
  char                             cpuname[RPMSG_NAME_SIZE + 1];
  FAR struct ivshmem_device_s     *ivdev;
  FAR const struct resource_table *rsc_table;
  size_t                           rsc_size;

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

static const struct rptun_ivshmem_rsc_s g_rsc_table =
{
  .hdr =
    {
      .ver = 1,
      .num = RPTUN_IVSHMEM_RSC_NUM,
    },
  .offset =
    {
#ifdef CONFIG_DRIVERS_VHOST_RNG
      offsetof(struct rptun_ivshmem_rsc_s, rng),
      offsetof(struct rptun_ivshmem_rsc_s, rng_carveout),
#endif
#ifdef CONFIG_DRIVERS_VHOST_RPMSG
      offsetof(struct rptun_ivshmem_rsc_s, rpmsg),
      offsetof(struct rptun_ivshmem_rsc_s, rpmsg_carveout),
#endif
#ifdef CONFIG_NET_VSOCK_VHOST
      offsetof(struct rptun_ivshmem_rsc_s, vsock),
      offsetof(struct rptun_ivshmem_rsc_s, vsock_carveout),
#endif
#ifdef CONFIG_DRIVERS_VHOST_NET
      offsetof(struct rptun_ivshmem_rsc_s, net),
      offsetof(struct rptun_ivshmem_rsc_s, net_carveout),
#endif
    },

#ifdef CONFIG_DRIVERS_VHOST_RNG
  .rng =
    {
      .type       = RSC_VDEV,
      .id         = VIRTIO_ID_ENTROPY,
      .notifyid   = RSC_NOTIFY_ID_ANY,
      .dfeatures  = 0,
      .config_len = 0,
      .num_of_vrings = 1,
      .reserved   =
        {
          VIRTIO_DEV_DRIVER, 0
        },
    },
  .rng_vring =
    {
      .align     = 8,
      .num       = 8,
      .notifyid  = RSC_NOTIFY_ID_ANY,
      .da        = FW_RSC_U32_ADDR_ANY,
    },
  .rng_carveout =
    {
      .type = RSC_CARVEOUT,
      .da   = offsetof(struct rptun_ivshmem_rsc_s, rng_shm) +
              offsetof(struct rptun_ivshmem_mem_s, rsc_table),
      .pa   = (uint32_t)METAL_BAD_PHYS,
      .len  = sizeof(g_rsc_table.rng_shm),
    },
#endif
#ifdef CONFIG_DRIVERS_VHOST_RPMSG
  .rpmsg =
    {
      .type          = RSC_VDEV,
      .id            = VIRTIO_ID_RPMSG,
      .notifyid      = RSC_NOTIFY_ID_ANY,
      .dfeatures     = (1 << VIRTIO_RPMSG_F_NS) |
                       (1 << VIRTIO_RPMSG_F_ACK) |
                       (1 << VIRTIO_RPMSG_F_BUFSZ) |
                       (1 << VIRTIO_RPMSG_F_CPUNAME),
      .config_len    = sizeof(struct fw_rsc_config),
      .num_of_vrings = 2,
      .reserved      =
        {
          VIRTIO_DEV_DRIVER, 0
        },
    },
  .rpmsg_vring0 =
    {
      .align     = 8,
      .num       = 8,
      .notifyid  = RSC_NOTIFY_ID_ANY,
      .da        = FW_RSC_U32_ADDR_ANY,
    },
  .rpmsg_vring1 =
    {
      .align     = 8,
      .num       = 8,
      .notifyid  = RSC_NOTIFY_ID_ANY,
      .da        = FW_RSC_U32_ADDR_ANY,
    },
  .rpmsg_config =
    {
      .h2r_buf_size   = 0x600,
      .r2h_buf_size   = 0x600,
      .remote_cpuname = CONFIG_RPMSG_LOCAL_CPUNAME,
    },
  .rpmsg_carveout =
    {
      .type = RSC_CARVEOUT,
      .da   = offsetof(struct rptun_ivshmem_rsc_s, rpmsg_shm) +
              offsetof(struct rptun_ivshmem_mem_s, rsc_table),
      .pa   = (uint32_t)METAL_BAD_PHYS,
      .len  = sizeof(g_rsc_table.rpmsg_shm),
    },
#endif
#ifdef CONFIG_NET_VSOCK_VHOST
  .vsock =
    {
      .type       = RSC_VDEV,
      .id         = VIRTIO_ID_VSOCK,
      .notifyid   = RSC_NOTIFY_ID_ANY,
      .dfeatures  = (1 << VIRTIO_VSOCK_F_HOST_CID),
      .config_len = sizeof(struct virtio_vsock_config_s),
      .num_of_vrings = 3,
      .reserved   =
        {
          VIRTIO_DEV_DRIVER, 0
        },
    },
  .vsock_vring0 =
    {
     .align = 8,
     .num = 32,
     .notifyid = RSC_NOTIFY_ID_ANY,
     .da = FW_RSC_U32_ADDR_ANY
    },
  .vsock_vring1 =
    {
      .align = 8,
      .num = 32,
      .notifyid = RSC_NOTIFY_ID_ANY,
      .da = FW_RSC_U32_ADDR_ANY
    },
  .vsock_vring2 =
    {
      .align = 8,
      .num = 32,
      .notifyid = RSC_NOTIFY_ID_ANY,
      .da = FW_RSC_U32_ADDR_ANY
    },
  .vsock_carveout =
    {
      .type = RSC_CARVEOUT,
      .da   = offsetof(struct rptun_ivshmem_rsc_s, vsock_shm) +
              offsetof(struct rptun_ivshmem_mem_s, rsc_table),
      .pa   = (uint32_t)METAL_BAD_PHYS,
      .len  = sizeof(g_rsc_table.vsock_shm),
    },
#endif
#ifdef CONFIG_DRIVERS_VHOST_NET
  .net =
    {
      .type       = RSC_VDEV,
      .id         = VIRTIO_ID_NETWORK,
      .notifyid   = RSC_NOTIFY_ID_ANY,
      .dfeatures  = (1 << VIRTIO_F_ANY_LAYOUT),
      .config_len = sizeof(struct fw_rsc_config),
      .num_of_vrings = 2,
      .reserved   =
        {
          VIRTIO_DEV_DRIVER, 0
        },
    },
  .net_vring0 =
    {
      .align     = 8,
      .num       = 32,
      .notifyid  = RSC_NOTIFY_ID_ANY,
      .da        = FW_RSC_U32_ADDR_ANY,
    },
  .net_vring1 =
    {
      .align     = 8,
      .num       = 32,
      .notifyid  = RSC_NOTIFY_ID_ANY,
      .da        = FW_RSC_U32_ADDR_ANY,
    },
  .net_carveout =
    {
      .type = RSC_CARVEOUT,
      .da   = offsetof(struct rptun_ivshmem_rsc_s, net_shm) +
              offsetof(struct rptun_ivshmem_mem_s, rsc_table),
      .pa   = (uint32_t)METAL_BAD_PHYS,
      .len  = sizeof(g_rsc_table.net_shm),
    },
#endif
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

  if (!priv->rsc_table)
    {
      /* Wait until salve is ready */

      while (priv->shmem->rsc_size == 0)
        {
          nxsig_usleep(100);
        }

      priv->shmem->basem = (uint64_t)(uintptr_t)priv->shmem;
    }
  else
    {
      memcpy(&priv->shmem->rsc_table, priv->rsc_table, priv->rsc_size);
      if (priv->rsc_table == (FAR const struct resource_table *)&g_rsc_table)
        {
          FAR struct rptun_ivshmem_rsc_s *rsc =
            (FAR struct rptun_ivshmem_rsc_s *)&priv->shmem->rsc_table;
          int i = 0;

#ifdef CONFIG_DRIVERS_VHOST_RNG
          snprintf((FAR char *)rsc->rng_carveout.name, RPROC_MAX_NAME_LEN,
                   "vdev%dbuffer", i++);
#endif
#ifdef CONFIG_DRIVERS_VHOST_RPMSG
          snprintf((FAR char *)rsc->rpmsg_carveout.name, RPROC_MAX_NAME_LEN,
                   "vdev%dbuffer", i++);
          strlcpy((FAR char *)rsc->rpmsg_config.host_cpuname, priv->cpuname,
                  sizeof(rsc->rpmsg_config.host_cpuname));
#endif
#ifdef CONFIG_NET_VSOCK_VHOST
          snprintf((FAR char *)rsc->vsock_carveout.name, RPROC_MAX_NAME_LEN,
                   "vdev%dbuffer", i++);
          strlcpy((FAR char *)&rsc->vsock_config.guest_cid,
                  priv->cpuname, 5);
          strlcpy((FAR char *)&rsc->vsock_config.host_cid,
                  CONFIG_RPMSG_LOCAL_CPUNAME, 5);
#endif
#ifdef CONFIG_DRIVERS_VHOST_NET
          snprintf((FAR char *)rsc->net_carveout.name, RPROC_MAX_NAME_LEN,
                   "vdev%dbuffer", i++);
#endif
          UNUSED(rsc);
          UNUSED(i);
        }

      priv->shmem->rsc_size = priv->rsc_size;

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

  priv->raddrenv[0].pa   = !priv->rsc_table ? (uintptr_t)priv->shmem :
                                              (uintptr_t)priv->shmem->basem;
  priv->raddrenv[0].da   = 0;
  priv->raddrenv[0].size = priv->shmem_size;

  return &priv->shmem->rsc_table;
}

static bool rptun_ivshmem_is_autostart(FAR struct rptun_dev_s *dev)
{
  return true;
}

static bool rptun_ivshmem_is_master(FAR struct rptun_dev_s *dev)
{
  FAR struct rptun_ivshmem_dev_s *priv =
    (FAR struct rptun_ivshmem_dev_s *)dev;

  return !priv->rsc_table;
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
  else if (!priv->rsc_table)
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

  if (!priv->rsc_table && priv->seq != priv->shmem->seqs)
    {
      priv->seq = priv->shmem->seqs;
      should_notify = true;
    }
  else if (priv->rsc_table && priv->seq != priv->shmem->seqm)
    {
      priv->seq = priv->shmem->seqm;
      should_notify = true;
    }

  if (should_notify && priv->callback != NULL)
    {
      priv->callback(priv->arg, RPTUN_NOTIFY_ALL);
    }

  wd_start_next(&priv->wdog, RPTUN_IVSHMEM_WDOG_DELAY, rptun_ivshmem_wdog,
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

  if (priv->rsc_table && !ivshmem_support_irq(ivdev))
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
  FAR const char *str;
  FAR char *name = CONFIG_RPTUN_IVSHMEM_NAME;
  char cpuname[RPMSG_NAME_SIZE];
  bool master;
  int drvid;
  int ret;

  while (name != NULL && *name != '\0')
    {
      drvid = strtoul(name, &name, 0);
      str = strchr(++name, ':');
      snprintf(cpuname, RPMSG_NAME_SIZE, "%.*s", (int)(str - name), name);
      master = *(str + 1) == 'm';
      if (master)
        {
          ret = pci_register_rptun_ivshmem_slave_driver(cpuname, drvid,
                                                        NULL, 0);
        }
      else
        {
          ret = pci_register_rptun_ivshmem_slave_driver(
                  cpuname, drvid,
                  (FAR const struct resource_table *)&g_rsc_table,
                  sizeof(g_rsc_table));
        }

      if (ret < 0)
        {
          return ret;
        }

      name = strchr(str, ';');
      if (name != NULL)
        {
          name++;
        }
    }

  return OK;
}

int pci_register_rptun_ivshmem_slave_driver(
      FAR const char *cpuname, int drvid,
      FAR const struct resource_table *rsc_table,
      size_t rsc_size)
{
  FAR struct rptun_ivshmem_dev_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->drv.id = drvid;
  strlcpy(priv->cpuname, cpuname, sizeof(priv->cpuname));

  priv->rsc_table = rsc_table;
  priv->rsc_size = rsc_size;

  pciinfo("Register ivshmem driver, id=%d, cpuname=%s, master=%d\n",
          priv->drv.id, priv->cpuname, !rsc_table);

  priv->drv.probe = rptun_ivshmem_probe;
  priv->drv.remove = rptun_ivshmem_remove;
  ret = ivshmem_register_driver(&priv->drv);
  if (ret < 0)
    {
      kmm_free(priv);
    }

  return ret;
}

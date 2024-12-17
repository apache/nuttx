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
#include <nuttx/board.h>
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
#define RPTUN_IVSHMEM_WDOG_DELAY  MSEC2TICK(1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rptun_ivshmem_mem_s
{
  volatile uint64_t  basem;
  volatile uint32_t  seqs;
  volatile uint32_t  seqm;
  volatile uint32_t  reserved;
  volatile uint32_t  rsc_size;
  struct rptun_rsc_s rsc;
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

static FAR struct rptun_rsc_s *
rptun_ivshmem_get_resource(FAR struct rptun_dev_s *dev)
{
  FAR struct rptun_ivshmem_dev_s *priv =
    (FAR struct rptun_ivshmem_dev_s *)dev;
  FAR struct rptun_cmd_s *cmd = RPTUN_RSC2CMD(&priv->shmem->rsc);

  priv->raddrenv[0].da   = 0;
  priv->raddrenv[0].size = priv->shmem_size;

  if (priv->master)
    {
      priv->raddrenv[0].pa = (uintptr_t)priv->shmem;

      /* Wait untils salve is ready */

      while (RPTUN_GET_CMD(cmd->cmd_slave) != RPTUN_CMD_READY)
        {
          usleep(1000);
        }

      cmd->cmd_slave = 0;
      priv->shmem->basem = (uint64_t)(uintptr_t)priv->shmem;
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
      cmd->cmd_master               = 0;
      cmd->cmd_slave                = RPTUN_CMD(RPTUN_CMD_READY, 0);

      /* Wait untils master is ready, salve need use master base to
       * initialize addrenv.
       */

      while (priv->shmem->basem == 0)
        {
          usleep(1000);
        }

      priv->raddrenv[0].pa  = (uintptr_t)priv->shmem->basem;

      priv->addrenv[0].va   = (uint64_t)(uintptr_t)priv->shmem;
      priv->addrenv[0].pa   = priv->shmem->basem;
      priv->addrenv[0].size = priv->shmem_size;

      simple_addrenv_initialize(&priv->addrenv[0]);
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
 * Name: rptun_ivshmem_check_cmd
 ****************************************************************************/

static void rptun_ivshmem_check_cmd(FAR struct rptun_ivshmem_dev_s *priv)
{
  FAR struct rptun_cmd_s *rptun_cmd = RPTUN_RSC2CMD(&priv->shmem->rsc);
  uint32_t cmd;

  if (priv->master)
    {
      cmd = RPTUN_GET_CMD(rptun_cmd->cmd_slave);
      rptun_cmd->cmd_slave = RPTUN_CMD(RPTUN_CMD_DEFAULT, 0);
    }
  else
    {
      cmd = RPTUN_GET_CMD(rptun_cmd->cmd_master);
      rptun_cmd->cmd_master = RPTUN_CMD(RPTUN_CMD_DEFAULT, 0);
    }

  switch (cmd)
    {
      case RPTUN_CMD_RESTART:
#ifdef CONFIG_BOARDCTL_RESET
        board_reset(0);
#endif
        break;
      default:
        break;
    }
}

/****************************************************************************
 * Name: rptun_ivshmem_interrupt
 ****************************************************************************/

static int rptun_ivshmem_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct rptun_ivshmem_dev_s *priv = arg;

  rptun_ivshmem_check_cmd(priv);

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

  rptun_ivshmem_check_cmd(priv);

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


/****************************************************************************
 * arch/arm/src/nrf53/nrf53_rptun.c
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

#include <nuttx/nuttx.h>
#include <nuttx/kthread.h>
#include <nuttx/rptun/rptun.h>

#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "hardware/nrf53_spu.h"

#include "nrf53_ipc.h"

#ifdef CONFIG_NRF53_APPCORE
#  include "nrf53_cpunet.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Vring configuration parameters */

#define VRINGS                   (2)           /* Number of vrings */
#define VRING_ALIGN              (8)           /* Vring alignment */
#define VRING_NR                 (8)           /* Number of descriptors */
#define VRING_SIZE               (512)         /* Size of one descriptor */

/* This is the RPMSG default channel used with only one RPMSG channel.
 * We use the last 32kB of the App core RAM as a shared memory.
 */

#define VRING_SHMEM              (0x20078000)         /* Vring shared memory start */
#define VRING0_NOTIFYID          (RSC_NOTIFY_ID_ANY)  /* Vring0 id */
#define VRING1_NOTIFYID          (RSC_NOTIFY_ID_ANY)  /* Vring1 id */

/* IPC configuration */

#define RPTUN_IPC_CHAN_MASTER_RX    (0) /* RX for master is ready */
#define RPTUN_IPC_CHAN_SLAVE_RX     (1) /* RX for slave is ready */
#define RPTUN_IPC_CHAN_SLAVE_RESET  (2)
#define RPTUN_IPC_CHAN_SLAVE_PANIC  (3)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* NRF53 rptun sharred memory */

struct nrf53_rptun_shmem_s
{
  volatile uintptr_t         base;
  struct rptun_rsc_s         rsc;
};

/* NRF53 rptun device */

struct nrf53_rptun_dev_s
{
  struct rptun_dev_s          rptun;
  rptun_callback_t            callback;
  void                       *arg;
  bool                        master;
  struct nrf53_rptun_shmem_s *shmem;
  char                        cpuname[RPMSG_NAME_SIZE + 1];
  char                        shmemname[RPMSG_NAME_SIZE + 1];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static const char *nrf53_rptun_get_cpuname(struct rptun_dev_s *dev);
static const char *nrf53_rptun_get_firmware(struct rptun_dev_s *dev);
static const struct rptun_addrenv_s *
nrf53_rptun_get_addrenv(struct rptun_dev_s *dev);
static struct rptun_rsc_s *
nrf53_rptun_get_resource(struct rptun_dev_s *dev);
static bool nrf53_rptun_is_autostart(struct rptun_dev_s *dev);
static bool nrf53_rptun_is_master(struct rptun_dev_s *dev);
static int nrf53_rptun_start(struct rptun_dev_s *dev);
static int nrf53_rptun_stop(struct rptun_dev_s *dev);
static int nrf53_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid);
static int nrf53_rptun_register_callback(struct rptun_dev_s *dev,
                                         rptun_callback_t callback,
                                         void *arg);

#ifdef CONFIG_NRF53_APPCORE
static void nrf53_rptun_reset(struct rptun_dev_s *dev, int value);
static void nrf53_rptun_panic(struct rptun_dev_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_nrf53_rptun_ops =
{
  .get_cpuname       = nrf53_rptun_get_cpuname,
  .get_firmware      = nrf53_rptun_get_firmware,
  .get_addrenv       = nrf53_rptun_get_addrenv,
  .get_resource      = nrf53_rptun_get_resource,
  .is_autostart      = nrf53_rptun_is_autostart,
  .is_master         = nrf53_rptun_is_master,
  .start             = nrf53_rptun_start,
  .stop              = nrf53_rptun_stop,
  .notify            = nrf53_rptun_notify,
  .register_callback = nrf53_rptun_register_callback,
#ifdef CONFIG_NRF53_APPCORE
  .reset             = nrf53_rptun_reset,
  .panic             = nrf53_rptun_panic
#endif
};

#ifdef CONFIG_NRF53_APPCORE
/* Allocate shared memory on the App core side */

static struct nrf53_rptun_shmem_s g_shmem __attribute__((section(".shmem")));
#endif

struct nrf53_rptun_dev_s          g_rptun_dev;
static sem_t                      g_nrf53_rx_sig = SEM_INITIALIZER(0);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_rptun_get_cpuname
 ****************************************************************************/

static const char *nrf53_rptun_get_cpuname(struct rptun_dev_s *dev)
{
  struct nrf53_rptun_dev_s *priv = container_of(dev,
                                 struct nrf53_rptun_dev_s, rptun);

  return priv->cpuname;
}

/****************************************************************************
 * Name: nrf53_rptun_get_firmware
 ****************************************************************************/

static const char *nrf53_rptun_get_firmware(struct rptun_dev_s *dev)
{
  return NULL;
}

/****************************************************************************
 * Name: nrf53_rptun_get_addrenv
 ****************************************************************************/

static const struct rptun_addrenv_s *
nrf53_rptun_get_addrenv(struct rptun_dev_s *dev)
{
  return NULL;
}

/****************************************************************************
 * Name: nrf53_rptun_get_resource
 ****************************************************************************/

static struct rptun_rsc_s *
nrf53_rptun_get_resource(struct rptun_dev_s *dev)
{
  struct nrf53_rptun_dev_s *priv = container_of(dev,
                                   struct nrf53_rptun_dev_s, rptun);
  struct rptun_rsc_s *rsc;

  if (priv->shmem != NULL)
    {
      return &priv->shmem->rsc;
    }

#ifdef CONFIG_NRF53_APPCORE
  priv->shmem = &g_shmem;
#else
  priv->shmem = (struct nrf53_rptun_shmem_s *)VRING_SHMEM;
#endif

  if (priv->master)
    {
      /* Perform initial setup */

      rsc = &priv->shmem->rsc;

      priv->shmem->base             = (uintptr_t)priv->shmem;

      rsc->rsc_tbl_hdr.ver          = 1;
      rsc->rsc_tbl_hdr.num          = 1;
      rsc->rsc_tbl_hdr.reserved[0]  = 0;
      rsc->rsc_tbl_hdr.reserved[1]  = 0;
      rsc->offset[0]                = offsetof(struct rptun_rsc_s,
                                               rpmsg_vdev);

      rsc->rpmsg_vdev.type          = RSC_VDEV;
      rsc->rpmsg_vdev.id            = VIRTIO_ID_RPMSG;
      rsc->rpmsg_vdev.dfeatures     = 1 << VIRTIO_RPMSG_F_NS
                                    | 1 << VIRTIO_RPMSG_F_ACK
                                    | 1 << VIRTIO_RPMSG_F_BUFSZ;
      rsc->rpmsg_vdev.config_len    = sizeof(struct fw_rsc_config);
      rsc->rpmsg_vdev.num_of_vrings = VRINGS;

      rsc->rpmsg_vring0.align       = VRING_ALIGN;
      rsc->rpmsg_vring0.num         = VRING_NR;
      rsc->rpmsg_vring0.notifyid    = VRING0_NOTIFYID;
      rsc->rpmsg_vring1.align       = VRING_ALIGN;
      rsc->rpmsg_vring1.num         = VRING_NR;
      rsc->rpmsg_vring1.notifyid    = VRING1_NOTIFYID;
      rsc->config.r2h_buf_size      = VRING_SIZE;
      rsc->config.h2r_buf_size      = VRING_SIZE;
    }
  else
    {
      /* TODO: use IPC */

      while (priv->shmem->base == 0)
        {
          usleep(100);
        }
    }

  return &priv->shmem->rsc;
}

/****************************************************************************
 * Name: nrf53_rptun_is_autostart
 ****************************************************************************/

static bool nrf53_rptun_is_autostart(struct rptun_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: nrf53_rptun_is_master
 ****************************************************************************/

static bool nrf53_rptun_is_master(struct rptun_dev_s *dev)
{
  struct nrf53_rptun_dev_s *priv = container_of(dev,
                                 struct nrf53_rptun_dev_s, rptun);
  return priv->master;
}

/****************************************************************************
 * Name: nrf53_rptun_start
 ****************************************************************************/

static int nrf53_rptun_start(struct rptun_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: nrf53_rptun_stop
 ****************************************************************************/

static int nrf53_rptun_stop(struct rptun_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: nrf53_rptun_notify
 ****************************************************************************/

static int nrf53_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
#ifdef CONFIG_NRF53_APPCORE
  /* Notify slave that RX is ready */

  nrf53_ipc_signal(RPTUN_IPC_CHAN_SLAVE_RX);
#else
  /* Notify master that RX is ready */

  nrf53_ipc_signal(RPTUN_IPC_CHAN_MASTER_RX);
#endif

  return 0;
}

/****************************************************************************
 * Name: nrf53_rptun_register_callback
 ****************************************************************************/

static int nrf53_rptun_register_callback(struct rptun_dev_s *dev,
                                       rptun_callback_t callback,
                                       void *arg)
{
  struct nrf53_rptun_dev_s *priv = container_of(dev,
                                 struct nrf53_rptun_dev_s, rptun);

  priv->callback = callback;
  priv->arg      = arg;

  return 0;
}

#ifdef CONFIG_NRF53_APPCORE
/****************************************************************************
 * Name: nrf53_rptun_reset
 ****************************************************************************/

static void nrf53_rptun_reset(struct rptun_dev_s *dev, int value)
{
  if (value == 0)
    {
      /* Hard reset with Forceoff */

      nrf53_cpunet_power(false);
      nrf53_cpunet_power(true);
    }
  else
    {
      /* Soft reset */

      nrf53_ipc_signal(RPTUN_IPC_CHAN_SLAVE_RESET);
    }
}

/****************************************************************************
 * Name: nrf53_rptun_panic
 ****************************************************************************/

static void nrf53_rptun_panic(struct rptun_dev_s *dev)
{
  nrf53_ipc_signal(RPTUN_IPC_CHAN_SLAVE_PANIC);
}
#endif

#ifdef CONFIG_NRF53_APPCORE
/****************************************************************************
 * Name: nrf53_ipc_master_callback
 ****************************************************************************/

static void nrf53_ipc_master_callback(int id, void *arg)
{
  _info("Rptun IPC master %d\n", id);

  switch (id)
    {
      case RPTUN_IPC_CHAN_MASTER_RX:
        {
          nxsem_post(&g_nrf53_rx_sig);
          break;
        }

      default:
        {
          DEBUGASSERT(0);
        }
    }
}

/****************************************************************************
 * Name: nrf53_rptun_ipc_app
 ****************************************************************************/

static void nrf53_rptun_ipc_app(struct nrf53_rptun_dev_s *dev)
{
  DEBUGASSERT(dev);

  nrf53_ipc_subscribe(RPTUN_IPC_CHAN_MASTER_RX,
                      nrf53_ipc_master_callback,
                      dev);
  nrf53_ipc_send_cfg(RPTUN_IPC_CHAN_SLAVE_RX);
  nrf53_ipc_send_cfg(RPTUN_IPC_CHAN_SLAVE_RESET);
  nrf53_ipc_send_cfg(RPTUN_IPC_CHAN_SLAVE_PANIC);
}
#else
/****************************************************************************
 * Name: nrf53_ipc_slave_callback
 ****************************************************************************/

static void nrf53_ipc_slave_callback(int id, void *arg)
{
  _info("Rptun IPC slave %d\n", id);

  switch (id)
    {
      case RPTUN_IPC_CHAN_SLAVE_RX:
        {
          nxsem_post(&g_nrf53_rx_sig);
          break;
        }

      case RPTUN_IPC_CHAN_SLAVE_RESET:
        {
          /* TODO: Soft reset */

          break;
        }

      case RPTUN_IPC_CHAN_SLAVE_PANIC:
        {
          PANIC();
        }

      default:
        {
          DEBUGASSERT(0);
        }
    }
}

/****************************************************************************
 * Name: nrf53_rptun_ipc_net
 ****************************************************************************/

static void nrf53_rptun_ipc_net(struct nrf53_rptun_dev_s *dev)
{
  DEBUGASSERT(dev);

  nrf53_ipc_subscribe(RPTUN_IPC_CHAN_SLAVE_RX,
                      nrf53_ipc_slave_callback,
                      dev);
  nrf53_ipc_subscribe(RPTUN_IPC_CHAN_SLAVE_RESET,
                      nrf53_ipc_slave_callback,
                      dev);
  nrf53_ipc_subscribe(RPTUN_IPC_CHAN_SLAVE_PANIC,
                      nrf53_ipc_slave_callback,
                      dev);
  nrf53_ipc_send_cfg(RPTUN_IPC_CHAN_MASTER_RX);
}
#endif

/****************************************************************************
 * Name: nrf53_rptun_thread
 ****************************************************************************/

static int nrf53_rptun_thread(int argc, char *argv[])
{
  struct nrf53_rptun_dev_s *dev = &g_rptun_dev;

  while (1)
    {
      if (dev->callback != NULL)
        {
          dev->callback(dev->arg, RPTUN_NOTIFY_ALL);
        }

      nxsem_wait(&g_nrf53_rx_sig);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int nrf53_rptun_init(const char *shmemname, const char *cpuname)
{
  struct nrf53_rptun_dev_s *dev = &g_rptun_dev;
  int                       ret = OK;

  /* Initialize IPC */

  nrf53_ipc_init();

  /* The App core always master */

#ifdef CONFIG_NRF53_APPCORE
  memset(&g_shmem, 0, sizeof(struct nrf53_rptun_shmem_s));
  dev->master = true;
#else
  dev->master = false;
#endif

#ifdef CONFIG_NRF53_APPCORE
  /* Set secure domain - this allows net core to access shared mem */

  putreg32(SPU_EXTDOMAIN_SECUREMAPPING_SECATTR, NRF53_SPU_EXTDOMAIN(0));
#endif

  /* Subscribe to IPC */

#ifdef CONFIG_NRF53_APPCORE
  nrf53_rptun_ipc_app(dev);
#else
  nrf53_rptun_ipc_net(dev);
#endif

  /* TODO: handle net core reset */

  /* Configure device */

  dev->rptun.ops = &g_nrf53_rptun_ops;
  strncpy(dev->cpuname, cpuname, RPMSG_NAME_SIZE);
  strncpy(dev->shmemname, shmemname, RPMSG_NAME_SIZE);

  ret = rptun_initialize(&dev->rptun);
  if (ret < 0)
    {
      _err("ERROR: rptun_initialize failed %d!\n", ret);
      goto errout;
    }

  /* Create rptun RX thread */

  ret = kthread_create("nrf53-rptun", CONFIG_RPTUN_PRIORITY,
                       CONFIG_RPTUN_STACKSIZE, nrf53_rptun_thread, NULL);
  if (ret < 0)
    {
      _err("ERROR: kthread_create failed %d\n", ret);
    }

errout:
  return ret;
}

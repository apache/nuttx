/****************************************************************************
 * arch/arm/src/am67/am67_rptun.c
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

/* rptun driver for the AM67A (J722S) main-R5FSS0-0 Cortex-R5F core.
 *
 * The resource table (g_am67_rsc_table, defined in am67_boot.c and placed
 * in the .resource_table ELF section) is read by Linux's remoteproc driver
 * when it loads the R5F firmware.  This driver registers the same table
 * with NuttX's rptun/OpenAMP stack so that NuttX can also discover and
 * use the virtio devices (RPMsg + virtio-net).
 *
 * Inter-processor notification (Linux <-> R5F):
 *   Uses mailbox0_cluster3 (base 0x29030000).
 *   R5F -> Linux: write vqid to FIFO 0 -> Linux GIC SPI 109 (user 0)
 *   Linux -> R5F: Linux writes to FIFO 1 -> R5F VIM IRQ 116 (user 3)
 *   Mailbox assignment confirmed from:
 *     k3-j722s-evm.dts (mbox_main_r5_0) and
 *     mcu_plus_sdk_j722s cslr_intr_r5fss0_core0.h (IRQ 116).
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <arch/barriers.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/wqueue.h>

#include "am67_rptun.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NAVSS mailbox0_cluster3 — the J722S EVM DTS assigns this cluster to
 * main_r5fss0_core0:
 *   &mailbox0_cluster3 { mbox_main_r5_0: { ti,mbox-rx=<0 0 0>;
 *                                           ti,mbox-tx=<1 0 0>; }; }
 *   &main_r5fss0_core0 { mboxes = <&mailbox0_cluster3 &mbox_main_r5_0>; }
 *
 * From the perspective of each side:
 *   Linux TX -> R5F RX : write/read FIFO 1 (Linux sends, R5F receives)
 *   R5F  TX -> Linux RX: write/read FIFO 0 (R5F sends, Linux receives)
 *
 * The NAVSS interrupt router wires cluster3 user-3 to R5FSS0 VIM 116
 * (CSLR_R5FSS0_CORE0_INTR_MAILBOX0_MAILBOX_CLUSTER_3
 *  _MAILBOX_CLUSTER_PEND_3, from mcu_plus_sdk_j722s
 *  source/drivers/hw_include/j722s/cslr_intr_r5fss0_core0.h).
 * Linux uses user-0 of the same cluster (GIC SPI 109).
 */

#define AM67_MBOX_BASE          (0x29030000ul) /* mailbox0_cluster3 */

/* FIFO assignments (from Linux DTS mbox_main_r5_0) */

#define AM67_MBOX_TX_FIFO       (0U)  /* R5F writes here -> Linux user-0 IRQ fires */
#define AM67_MBOX_RX_FIFO       (1U)  /* Linux writes here -> R5F user-3 IRQ fires */

/* R5F uses mailbox user-3 (interrupt router wires cluster3/user3 -> VIM) */

#define AM67_MBOX_USER          (3U)

/* Control messages the Linux side exchanges in the mailbox payload.  Any
 * value outside [READY, END_MSG) is a virtqueue index instead.  Values are
 * from the TI kernel's drivers/remoteproc/omap_remoteproc.h; SHUTDOWN and
 * SHUTDOWN_ACK are TI additions used by ti_k3_r5_remoteproc.c.
 */

#define RP_MBOX_READY           (0xffffff00ul)
#define RP_MBOX_SHUTDOWN        (0xffffff14ul)
#define RP_MBOX_SHUTDOWN_ACK    (0xffffff15ul)
#define RP_MBOX_END_MSG         (0xffffff16ul)

/* VIM IRQ number on MAIN_R5FSS0_0 for mailbox0_cluster3/user3 */

#ifndef CONFIG_AM67_RPTUN_IRQ
#  define AM67_RPTUN_IRQ_EVENT  (116)
#else
#  define AM67_RPTUN_IRQ_EVENT  CONFIG_AM67_RPTUN_IRQ
#endif

/* Register offsets (OMAP4-style mailbox, ti,am64-mailbox compatible):
 *   MESSAGE(n)     = base + 0x040 + 4*n  (read/write FIFO n)
 *   MSG_STATUS(n)  = base + 0x0C0 + 4*n  (number of msgs in FIFO n)
 *   IRQSTATUS(u)   = base + 0x104 + 16*u
 *   IRQENABLE(u)   = base + 0x108 + 16*u
 *   IRQDISABLE(u)  = base + 0x10C + 16*u
 *   EOI            = base + 0x140
 *   NEW_MSG_INT(n) = (1 << (2*n))  — bit to enable/clear new-message IRQ
 */

#define AM67_MBOX_MESSAGE(fifo)    (AM67_MBOX_BASE + 0x040u + 0x4u * (fifo))
#define AM67_MBOX_MSG_STATUS(fifo) (AM67_MBOX_BASE + 0x0c0u + 0x4u * (fifo))
#define AM67_MBOX_IRQSTATUS(usr)   (AM67_MBOX_BASE + 0x104u + 0x10u * (usr))
#define AM67_MBOX_IRQENABLE(usr)   (AM67_MBOX_BASE + 0x108u + 0x10u * (usr))
#define AM67_MBOX_IRQDISABLE(usr)  (AM67_MBOX_BASE + 0x10cu + 0x10u * (usr))
#define AM67_MBOX_EOI              (AM67_MBOX_BASE + 0x140u)
#define AM67_MBOX_NEW_MSG_INT(n)   (1u << ((n) * 2u))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct am67_rptun_dev_s
{
  struct rptun_dev_s rptun;
  rptun_callback_t   callback;
  void              *arg;
  struct work_s      work;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static const char *am67_rptun_get_cpuname(struct rptun_dev_s *dev);
static const char *am67_rptun_get_firmware(struct rptun_dev_s *dev);
static const struct rptun_addrenv_s *
am67_rptun_get_addrenv(struct rptun_dev_s *dev);
static struct resource_table *
am67_rptun_get_resource(struct rptun_dev_s *dev);
static bool am67_rptun_is_autostart(struct rptun_dev_s *dev);
static bool am67_rptun_is_master(struct rptun_dev_s *dev);
static int am67_rptun_start(struct rptun_dev_s *dev);
static int am67_rptun_stop(struct rptun_dev_s *dev);
static int am67_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid);
static int am67_rptun_register_callback(struct rptun_dev_s *dev,
                                        rptun_callback_t callback,
                                        void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_am67_rptun_ops =
{
  .get_cpuname       = am67_rptun_get_cpuname,
  .get_firmware      = am67_rptun_get_firmware,
  .get_addrenv       = am67_rptun_get_addrenv,
  .get_resource      = am67_rptun_get_resource,
  .is_autostart      = am67_rptun_is_autostart,
  .is_master         = am67_rptun_is_master,
  .start             = am67_rptun_start,
  .stop              = am67_rptun_stop,
  .notify            = am67_rptun_notify,
  .register_callback = am67_rptun_register_callback,
};

static struct am67_rptun_dev_s g_am67_rptun_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *am67_rptun_get_cpuname(struct rptun_dev_s *dev)
{
  return AM67_RPTUN_CPUNAME;
}

static const char *am67_rptun_get_firmware(struct rptun_dev_s *dev)
{
  return NULL; /* Linux is master; R5F does not load a firmware file */
}

static const struct rptun_addrenv_s *
am67_rptun_get_addrenv(struct rptun_dev_s *dev)
{
  return NULL; /* physical == virtual (no MMU on R5F) */
}

static struct resource_table *
am67_rptun_get_resource(struct rptun_dev_s *dev)
{
  /* The resource table is placed at a fixed address (0xA2100000) by the
   * linker script (.resource_table section -> ddr_rsctable region).
   * Linux remoteproc populates the vdev status/features fields in-place
   * before kicking the R5F, so we return a non-const pointer.
   */

  return (struct resource_table *)&g_am67_rsc_table;
}

static bool am67_rptun_is_autostart(struct rptun_dev_s *dev)
{
  return true; /* start rptun thread automatically */
}

static bool am67_rptun_is_master(struct rptun_dev_s *dev)
{
  return false; /* Linux A53 is the remoteproc master */
}

static int am67_rptun_start(struct rptun_dev_s *dev)
{
  return 0; /* nothing to do — Linux starts us */
}

static int am67_rptun_stop(struct rptun_dev_s *dev)
{
  return 0;
}

static int am67_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
  /* Kick Linux A53: write vqid into FIFO 0 of mailbox0_cluster3.
   * Linux has user-0 new-message interrupt enabled on FIFO 0
   * (GIC SPI 109), so this fires rproc_vq_interrupt() on the A53.
   */

  putreg32(vqid, AM67_MBOX_MESSAGE(AM67_MBOX_TX_FIFO));
  UP_DSB();
  return 0;
}

static int am67_rptun_register_callback(struct rptun_dev_s *dev,
                                        rptun_callback_t callback,
                                        void *arg)
{
  struct am67_rptun_dev_s *priv =
      container_of(dev, struct am67_rptun_dev_s, rptun);

  priv->callback = callback;
  priv->arg      = arg;

  if (callback != NULL)
    {
      putreg32(AM67_MBOX_NEW_MSG_INT(AM67_MBOX_RX_FIFO),
               AM67_MBOX_IRQENABLE(AM67_MBOX_USER));
      up_enable_irq(AM67_RPTUN_IRQ_EVENT);
    }
  else
    {
      up_disable_irq(AM67_RPTUN_IRQ_EVENT);
      putreg32(AM67_MBOX_NEW_MSG_INT(AM67_MBOX_RX_FIFO),
               AM67_MBOX_IRQDISABLE(AM67_MBOX_USER));
    }

  return 0;
}

/****************************************************************************
 * Name: am67_rptun_notify_work
 *
 * Description:
 *   Deferred half of the mailbox interrupt: runs on the high-priority work
 *   queue and delivers the notification to the rptun/OpenAMP stack.  The
 *   OpenAMP receive path takes mutexes and allocates memory, so it must
 *   not run in interrupt context; RPTUN_NOTIFY_ALL re-scans every
 *   virtqueue, so any number of coalesced mailbox kicks collapse safely
 *   into one pass.
 *
 ****************************************************************************/

static void am67_rptun_notify_work(void *arg)
{
  struct am67_rptun_dev_s *priv = (struct am67_rptun_dev_s *)arg;

  if (priv->callback != NULL)
    {
      priv->callback(priv->arg, RPTUN_NOTIFY_ALL);
    }
}

/****************************************************************************
 * Name: am67_rptun_interrupt
 *
 * Description:
 *   VIM IRQ 116 handler — fires when Linux A53 writes to mailbox0_cluster3
 *   FIFO 1 (user-3 new-message interrupt).  Drains the FIFO, clears the
 *   interrupt, and queues am67_rptun_notify_work() to process the
 *   virtqueues in thread context.
 *
 ****************************************************************************/

static int am67_rptun_interrupt(int irq, void *context, void *arg)
{
  struct am67_rptun_dev_s *priv = (struct am67_rptun_dev_s *)arg;
  bool shutdown = false;
  bool kick = false;

  /* Drain all messages Linux wrote into FIFO 1.  Each read pops one
   * entry; stop when MSG_STATUS reports 0 pending messages.  A single
   * interrupt can carry both control messages and virtqueue kicks, so
   * classify every entry rather than the batch.
   */

  while (getreg32(AM67_MBOX_MSG_STATUS(AM67_MBOX_RX_FIFO)) != 0)
    {
      uint32_t msg = getreg32(AM67_MBOX_MESSAGE(AM67_MBOX_RX_FIFO));

      if (msg == RP_MBOX_SHUTDOWN)
        {
          shutdown = true;
        }
      else if (msg < RP_MBOX_READY || msg >= RP_MBOX_END_MSG)
        {
          kick = true;
        }
    }

  /* Clear the new-message interrupt status for user-3 / FIFO-1.
   * Write the bit mask to the IRQSTATUS register (write-1-to-clear).
   */

  putreg32(AM67_MBOX_NEW_MSG_INT(AM67_MBOX_RX_FIFO),
           AM67_MBOX_IRQSTATUS(AM67_MBOX_USER));

  /* Acknowledge the interrupt to the mailbox EOI register so the
   * controller can re-assert on the next incoming message.
   */

  putreg32(0, AM67_MBOX_EOI);
  UP_DSB();

  /* Honour a shutdown request: acknowledge it, then park the core.  Linux
   * polls the TI-SCI WFI status for 2 ms after the ACK and only halts the
   * R5F once it sees us in WFI, so this must never return.
   */

  if (shutdown)
    {
      putreg32(RP_MBOX_SHUTDOWN_ACK, AM67_MBOX_MESSAGE(AM67_MBOX_TX_FIFO));
      UP_DSB();

      up_irq_save();

      for (; ; )
        {
          __asm__ __volatile__ ("wfi");
        }
    }

  if (priv != NULL && priv->callback != NULL &&
      work_available(&priv->work) && kick)
    {
      work_queue(HPWORK, &priv->work, am67_rptun_notify_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_rptun_init
 ****************************************************************************/

int am67_rptun_init(void)
{
  struct am67_rptun_dev_s *dev = &g_am67_rptun_dev;
  int ret;

  memset(dev, 0, sizeof(*dev));
  dev->rptun.ops = &g_am67_rptun_ops;

  /* Enable the mailbox interrupt: R5F user-3, FIFO 1 (Linux->R5F).
   * Linux enables its own user-0/FIFO-0 side independently.
   * Disable first to start from a clean state, then clear any stale
   * status before attaching the IRQ.
   */

  putreg32(AM67_MBOX_NEW_MSG_INT(AM67_MBOX_RX_FIFO),
           AM67_MBOX_IRQDISABLE(AM67_MBOX_USER));
  putreg32(AM67_MBOX_NEW_MSG_INT(AM67_MBOX_RX_FIFO),
           AM67_MBOX_IRQSTATUS(AM67_MBOX_USER));

  ret = irq_attach(AM67_RPTUN_IRQ_EVENT, am67_rptun_interrupt, dev);
  if (ret < 0)
    {
      ipcerr("ERROR: irq_attach failed: %d\n", ret);
      return ret;
    }

  /* Enable new-message interrupt for FIFO 1, user 3 */

  putreg32(AM67_MBOX_NEW_MSG_INT(AM67_MBOX_RX_FIFO),
           AM67_MBOX_IRQENABLE(AM67_MBOX_USER));

  ret = rptun_initialize(&dev->rptun);
  if (ret < 0)
    {
      irq_detach(AM67_RPTUN_IRQ_EVENT);
      ipcerr("ERROR: rptun_initialize failed: %d\n", ret);
    }

  return ret;
}

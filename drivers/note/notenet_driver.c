/****************************************************************************
 * drivers/note/notenet_driver.c
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

#include <nuttx/net/netpoll.h>
#include <nuttx/note/note_driver.h>
#include <nuttx/note/notenet_driver.h>
#include <nuttx/panic_notifier.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define NOTENET_WORK_DELAY MSEC2TICK(CONFIG_DRIVERS_NOTENET_WORK_DELAY)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct notenet_driver_s
{
  struct note_driver_s  driver;  /* Note driver part, must be first element */
  spinlock_t            lock;    /* Protects pkt and next */
  FAR netpkt_t         *pkt;     /* Packet being filled by note_netdev_add */
  size_t                next;    /* The next index (where data is added) */
  struct work_s         work;    /* Used for deferred send */
  struct netpoll_s      np;      /* Netpoll */
  struct notifier_block nb;      /* Panic notifier */
  bool                  bound;   /* Netpoll is bound to a device */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void notenetdev_work(FAR void *arg);
static void note_netdev_add(FAR struct note_driver_s *driver,
                            FAR const void *note, size_t notelen);
static int netdev_note_panic_callback(FAR struct notifier_block *nb,
                                      unsigned long action,
                                      FAR void *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct note_driver_ops_s g_notenetdev_ops =
{
  note_netdev_add
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct notenet_driver_s g_notenet =
{
  {
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
    "net",
    {
      {
        CONFIG_SCHED_INSTRUMENTATION_FILTER_DEFAULT_MODE,
#  ifdef CONFIG_SMP
        CONFIG_SCHED_INSTRUMENTATION_CPUSET
#  endif
      },
    },
#endif
    &g_notenetdev_ops
  },
  .lock = SP_UNLOCKED
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: notenetdev_take
 *
 * Description:
 *   Detach the packet being filled, so it can be sent without holding
 *   drv->lock.  Notes added before notenetdev_refill() provides a new
 *   packet are dropped.
 *
 * Returned Value:
 *   The packet to send, or NULL if there is nothing to send.
 *
 ****************************************************************************/

static FAR netpkt_t *notenetdev_take(FAR struct notenet_driver_s *drv)
{
  FAR netpkt_t *pkt = NULL;
  irqstate_t flags;

  flags = spin_lock_irqsave_notrace(&drv->lock);

  if (drv->pkt != NULL && drv->next > 0)
    {
      pkt       = drv->pkt;
      drv->pkt  = NULL;
      drv->next = 0;
    }

  spin_unlock_irqrestore_notrace(&drv->lock, flags);

  return pkt;
}

/****************************************************************************
 * Name: notenetdev_send
 *
 * Description:
 *   Send one packet.  The packet is freed when the driver refuses it, for
 *   example because the interface is down.
 *
 ****************************************************************************/

static void notenetdev_send(FAR struct notenet_driver_s *drv,
                            FAR netpkt_t *pkt)
{
  if (netpoll_udp_pkt_send(&drv->np, pkt) < 0)
    {
      netpkt_free(drv->np.lower, pkt, NETPKT_TX);
    }
}

/****************************************************************************
 * Name: notenetdev_refill
 *
 * Description:
 *   Allocate a packet for note_netdev_add() if the previous allocation
 *   failed.
 *
 ****************************************************************************/

static void notenetdev_refill(FAR struct notenet_driver_s *drv)
{
  FAR netpkt_t *pkt;
  irqstate_t flags;

  if (drv->pkt != NULL)
    {
      return;
    }

  pkt = netpoll_udp_pkt_alloc(&drv->np);
  if (pkt == NULL)
    {
      return;
    }

  flags = spin_lock_irqsave_notrace(&drv->lock);
  if (drv->pkt == NULL)
    {
      drv->pkt  = pkt;
      drv->next = 0;
      pkt       = NULL;
    }

  spin_unlock_irqrestore_notrace(&drv->lock, flags);

  if (pkt != NULL)
    {
      netpkt_free(drv->np.lower, pkt, NETPKT_TX);
    }
}

/****************************************************************************
 * Name: notenetdev_work
 *
 * Description:
 *   Send the buffered notes.  The driver's transmit() is called with
 *   netdev_lock() held, the same lock the upper half holds when it calls
 *   transmit(), so the two never run in the driver at the same time.
 *
 ****************************************************************************/

static void notenetdev_work(FAR void *arg)
{
  FAR struct notenet_driver_s *drv = arg;
  FAR struct net_driver_s *dev = &drv->np.lower->netdev;
  FAR netpkt_t *pkt;

  netdev_lock(dev);

  pkt = notenetdev_take(drv);
  if (pkt != NULL)
    {
      notenetdev_send(drv, pkt);
    }

  notenetdev_refill(drv);

  netdev_unlock(dev);
}

/****************************************************************************
 * Name: note_netdev_add
 *
 * Description:
 *   Add the variable length note to the packet being filled.  This can be
 *   called from any context, also from an interrupt handler, so it never
 *   sends: sending is deferred to the work queue.  When the note does not
 *   fit, it is dropped.
 *
 * Input Parameters:
 *   driver  - The note driver
 *   note    - The note buffer
 *   notelen - The buffer length
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void note_netdev_add(FAR struct note_driver_s *driver,
                            FAR const void *note, size_t notelen)
{
  FAR struct notenet_driver_s *drv =
    (FAR struct notenet_driver_s *)driver;
  clock_t      delay  = NOTENET_WORK_DELAY;
  size_t       maxlen = netpoll_get_maxlen(&drv->np);
  irqstate_t   flags;

  flags = spin_lock_irqsave_notrace(&drv->lock);

  if (!drv->bound)
    {
      spin_unlock_irqrestore_notrace(&drv->lock, flags);
      return;
    }

  /* Drop the note if there is no packet or the packet is full, and let
   * the work send what is buffered.
   */

  if (drv->pkt == NULL || drv->next + notelen > maxlen)
    {
      spin_unlock_irqrestore_notrace(&drv->lock, flags);
      delay = 0;
      goto kick;
    }

  if (netpoll_pkt_copy(&drv->np, drv->pkt, note, notelen, drv->next) < 0)
    {
      /* Out of IOBs: drop the note, and trim what was partly copied */

      netpkt_setdatalen(drv->np.lower, drv->pkt,
                        drv->np.hdrlen + drv->next);
      spin_unlock_irqrestore_notrace(&drv->lock, flags);
      delay = 0;
      goto kick;
    }

  drv->next += notelen;

  /* Start work immediately when the packet is more than 75% full */

  if (maxlen - drv->next < maxlen / 4)
    {
      delay = 0;
    }

  spin_unlock_irqrestore_notrace(&drv->lock, flags);

kick:
  if (work_available(&drv->work))
    {
      work_queue(HPWORK, &drv->work, notenetdev_work, drv, delay);
    }
}

/****************************************************************************
 * Name: netdev_note_panic_callback
 *
 * Description:
 *   Flush the buffered notes on panic.  Nothing else runs at this point,
 *   so the packet is sent inline without netdev_lock().
 *
 ****************************************************************************/

static int netdev_note_panic_callback(FAR struct notifier_block *nb,
                                      unsigned long action,
                                      FAR void *data)
{
  FAR struct notenet_driver_s *drv =
    container_of(nb, struct notenet_driver_s, nb);
  FAR netpkt_t *pkt;

  if (action == PANIC_KERNEL_FINAL)
    {
      pkt = notenetdev_take(drv);
      if (pkt != NULL)
        {
          notenetdev_send(drv, pkt);
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: notenet_init
 *
 * Description:
 *   Register a netdev channel to note.
 *
 * Input Parameters:
 *   dev - The lower half netdev driver structure to be bound.
 *
 * Returned Value:
 *   Zero on success. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

int notenet_init(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct notenet_driver_s *note = &g_notenet;
  FAR netpkt_t                *pkt;
  irqstate_t                   flags;
  int                          ret;

  /* Bind only once: ifup runs again after every ifdown, and the packet
   * and panic notifier set up the first time stay valid.
   */

  if (note->bound)
    {
      return note->np.lower == dev ? OK : -EBUSY;
    }

  /* Initialize netpoll */

  ret = netpoll_udp_init(&note->np, dev,
                         CONFIG_DRIVERS_NOTENET_PORT,
                         CONFIG_DRIVERS_NOTENET_IPADDR,
                         CONFIG_DRIVERS_NOTENET_PORT,
                         CONFIG_DRIVERS_NOTENET_MAC);
  if (ret < 0)
    {
      return ret;
    }

  pkt = netpoll_udp_pkt_alloc(&note->np);

  flags = spin_lock_irqsave_notrace(&note->lock);
  note->pkt   = pkt;
  note->next  = 0;
  note->bound = true;
  spin_unlock_irqrestore_notrace(&note->lock, flags);

  /* Register panic notifier */

  note->nb.notifier_call = netdev_note_panic_callback;
  panic_notifier_chain_register(&note->nb);

  return OK;
}

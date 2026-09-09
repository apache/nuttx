/****************************************************************************
 * drivers/reset/reset_ipc.c
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

#include <assert.h>
#include <debug.h>
#include <limits.h>
#include <stdbool.h>
#include <string.h>
#include <sys/boardctl.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/nuttx.h>
#include <nuttx/panic_notifier.h>
#include <nuttx/reboot_notifier.h>
#include <nuttx/reset/reset-controller.h>
#include <nuttx/reset/reset.h>
#include <nuttx/reset/reset_ipc.h>
#include <nuttx/signal.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int reset_ipc_rc_reset(FAR struct reset_controller_dev *rcdev,
                              unsigned int id, unsigned int reason);
static void reset_ipc_check_peer(FAR struct reset_ipc_s *ipc);
static int reset_ipc_set_status(FAR struct reset_ipc_s *ipc,
                                unsigned int reason);
static int reset_ipc_panic_notifier(FAR struct notifier_block *nb,
                                    unsigned long action, FAR void *data);
static int reset_ipc_reboot_notifier(FAR struct notifier_block *nb,
                                     unsigned long action, FAR void *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Only the self-deasserting reset op. */

static const struct reset_control_ops g_reset_ipc_rc_ops =
{
  .reset = reset_ipc_rc_reset,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: reset_ipc_send
 *
 * Description:
 *   Send a reset cause or RESET_REASON_PRIVATE to the peer.  A panic also
 *   dumps the peer's trace buffer.
 *
 ****************************************************************************/

static int reset_ipc_send(FAR struct reset_ipc_s *ipc, unsigned int reason)
{
  int ret;

  if (reason > RESET_REASON_PRIVATE)
    {
      return -EINVAL;
    }

  ret = reset_ipc_set_status(ipc, reason);

  if (reason == BOARDIOC_SOFTRESETCAUSE_PANIC && ipc->ops->dump != NULL)
    {
      ipc->ops->dump(ipc);
    }

  return ret;
}

/****************************************************************************
 * Name: reset_ipc_rc_reset
 *
 * Description:
 *   reset framework op, passing the reason on to the peer.  id is unused.
 *
 ****************************************************************************/

static int reset_ipc_rc_reset(FAR struct reset_controller_dev *rcdev,
                              unsigned int id, unsigned int reason)
{
  FAR struct reset_ipc_s *ipc =
    container_of(rcdev, struct reset_ipc_s, rcdev);

  return reset_ipc_send(ipc, reason);
}

/****************************************************************************
 * Name: reset_ipc_get_peer_status
 ****************************************************************************/

static inline_function unsigned int
reset_ipc_get_peer_status(FAR struct reset_ipc_s *ipc)
{
  return *ipc->remote;
}

/****************************************************************************
 * Name: reset_ipc_set_local_notify
 ****************************************************************************/

static inline_function void
reset_ipc_set_local_notify(FAR struct reset_ipc_s *ipc,
                           unsigned int reason)
{
  *ipc->local = reason;
  ipc->ops->notify(ipc);
}

/****************************************************************************
 * Name: reset_ipc_boardctl_work
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL_RESET) || defined(CONFIG_BOARDCTL_POWEROFF)
static void reset_ipc_boardctl_work(FAR void *arg)
{
  if ((unsigned long)arg == BOARDIOC_SOFTRESETCAUSE_POWEROFF)
    {
      boardctl(BOARDIOC_POWEROFF, 0u);
    }
  else
    {
      boardctl(BOARDIOC_RESET, (unsigned long)arg);
    }
}
#endif

/****************************************************************************
 * Name: reset_ipc_check_peer
 *
 * Description:
 *   React to a new value in the peer's status word.  Run by the transport
 *   from its notification path.  A value already acted on, or the echo of
 *   what this core sent, is ignored.
 *
 ****************************************************************************/

static void reset_ipc_check_peer(FAR struct reset_ipc_s *ipc)
{
  unsigned int reason;
  unsigned int status;

  status = reset_ipc_get_peer_status(ipc);
  if (status == ipc->shadow || status == *ipc->local)
    {
      return;
    }

  ipc->shadow = status;

  if (status == RESET_IPC_PAUSE)
    {
      int timeout = CONFIG_RESET_IPC_STATUS_TIMEOUT_MS;

      /* Wait for the crashing core to finish printing its log */

      while (timeout-- > 0 &&
             status == RESET_IPC_PAUSE)
        {
          status = reset_ipc_get_peer_status(ipc);
          up_udelay(USEC_PER_MSEC);
        }

      if (timeout <= 0)
        {
          _emerg("FATAL: Timeout waiting for remote core panic: %s\n",
                 ipc->rcdev.name);
          return;
        }

      ipc->shadow = status;
    }

  reason = status;

  if (reason < BOARDIOC_SOFTRESETCAUSE_MAX)
    {
      if (reason == BOARDIOC_SOFTRESETCAUSE_PANIC)
        {
          char msg[32];

#ifdef CONFIG_RESET_IPC_REMOTE_PANIC_SILENT
          /* Silence this passive panic; restored at PANIC_KERNEL_FINAL. */

          nx_setlogmask(LOG_NONE);
#endif

          snprintf(msg, sizeof(msg), "Panic by remote core: %s",
                   ipc->rcdev.name);
          PANIC_WITH_MSG(msg);
        }

      _emerg("FATAL: Reset by remote core: %s reason: %u\n",
             ipc->rcdev.name, reason);
      if (reason == BOARDIOC_SOFTRESETCAUSE_POWEROFF)
        {
#ifdef CONFIG_BOARDCTL_POWEROFF
          work_queue(HPWORK, &ipc->work, reset_ipc_boardctl_work,
                     (FAR void *)BOARDIOC_SOFTRESETCAUSE_POWEROFF, 0);
#else
          reset_ipc_set_local_notify(ipc, BOARDIOC_SOFTRESETCAUSE_POWEROFF);
#endif
        }
      else
        {
#ifdef CONFIG_BOARDCTL_RESET
          work_queue(HPWORK, &ipc->work, reset_ipc_boardctl_work,
                     (FAR void *)(uintptr_t)reason, 0);
#else
          reset_ipc_set_local_notify(ipc, reason);
#endif
        }
    }
  else if (reason == RESET_REASON_PRIVATE &&
           ipc->ops->handle_status != NULL)
    {
      /* Transport-private: only it knows what this means. */

      ipc->ops->handle_status(ipc, reason);
    }
}

/****************************************************************************
 * Name: reset_ipc_set_status
 ****************************************************************************/

static int reset_ipc_set_status(FAR struct reset_ipc_s *ipc,
                                unsigned int reason)
{
  unsigned int status = reason;
  int timeout = CONFIG_RESET_IPC_STATUS_TIMEOUT_MS;
  int ret = -ENOTSUP;

  /* Hardware delivery first; -ENOTSUP falls back to the handshake below. */

  if (ipc->ops->set_status != NULL)
    {
      _emerg("Set status to remote %s reason: %u\n",
             ipc->rcdev.name, reason);
      ret = ipc->ops->set_status(ipc, reason);
      _emerg("Set status to remote %s ret %d\n", ipc->rcdev.name, ret);
    }

  if (ret != -ENOTSUP)
    {
      return ret;
    }

  /* Already sent, or already sent by the peer: no re-notify. */

  if (status == *ipc->local || status == ipc->shadow)
    {
      return OK;
    }

  reset_ipc_set_local_notify(ipc, reason);

  /* Only a reset is echoed back; PAUSE and PRIVATE are fire-and-forget. */

  if (reason >= BOARDIOC_SOFTRESETCAUSE_MAX)
    {
      return OK;
    }

  ret = -ETIMEDOUT;
  while (timeout-- > 0)
    {
      if (reset_ipc_get_peer_status(ipc) == status)
        {
          ret = OK;
          break;
        }

      if (timeout % MSEC_PER_SEC == 0)
        {
          _emerg("Waiting %s reset, timeout left %d ms\n",
                 ipc->rcdev.name, timeout);
        }

      if (up_interrupt_context())
        {
          up_udelay(USEC_PER_MSEC);
        }
      else
        {
          nxsig_usleep(USEC_PER_MSEC);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: reset_ipc_panic_notifier
 *
 * Description:
 *   Panic chain entry of one channel, so its peer comes down with us.
 *
 ****************************************************************************/

static int reset_ipc_panic_notifier(FAR struct notifier_block *nb,
                                    unsigned long action, FAR void *data)
{
  FAR struct reset_ipc_s *ipc =
    container_of(nb, struct reset_ipc_s, panic_nb);

  if (action == PANIC_KERNEL)
    {
      /* Only park the peer for now: this core still has logging to do. */

      reset_ipc_set_status(ipc, RESET_IPC_PAUSE);
    }
#ifndef CONFIG_BMP
  else if (action == PANIC_KERNEL_FINAL)
    {
      /* This core collects the peers' dumps, so panic them here.  Under
       * BMP each core dumps itself and the peer waits for the reboot.
       */

      reset_ipc_send(ipc, BOARDIOC_SOFTRESETCAUSE_PANIC);
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: reset_ipc_reboot_notifier
 *
 * Description:
 *   Reboot chain entry of one channel, mapping the event onto a cause.
 *
 ****************************************************************************/

static int reset_ipc_reboot_notifier(FAR struct notifier_block *nb,
                                     unsigned long action, FAR void *data)
{
  FAR struct reset_ipc_s *ipc =
    container_of(nb, struct reset_ipc_s, reboot_nb);
  unsigned int reason;

  if (action == SYS_POWER_OFF)
    {
      reason = BOARDIOC_SOFTRESETCAUSE_POWEROFF;
    }
  else if (action == SYS_HALT)
    {
      reason = BOARDIOC_SOFTRESETCAUSE_PANIC;
    }
  else
    {
      reason = (uintptr_t)data;
    }

  reset_ipc_send(ipc, reason);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: reset_ipc_register
 ****************************************************************************/

FAR struct reset_control *
reset_ipc_register(FAR struct reset_ipc_s *ipc,
                   FAR const struct reset_ipc_ops_s *ops,
                   FAR const char *cpuname,
                   uintptr_t status, bool master)
{
  FAR uint32_t *sts = (FAR uint32_t *)status;
  FAR struct reset_control *rstc;

  DEBUGASSERT(ipc != NULL && ops != NULL && cpuname != NULL &&
              status != 0 && (status % sizeof(uint32_t)) == 0);

  /* Word 0 belongs to the master, word 1 to the slave. */

  ipc->local      = master ? &sts[0] : &sts[1];
  ipc->remote     = master ? &sts[1] : &sts[0];
  ipc->ops        = ops;
  ipc->rcdev.name = cpuname;
  ipc->rcdev.ops  = &g_reset_ipc_rc_ops;

  /* Start clean, so a status from a previous cycle is driven again. */

  *ipc->local = UINT32_MAX;
  ipc->shadow = UINT32_MAX;

  if (reset_controller_register(&ipc->rcdev) < 0)
    {
      return NULL;
    }

  /* The callback the transport runs on every peer notification. */

  if (ops->register_callback != NULL)
    {
      ops->register_callback(ipc, reset_ipc_check_peer);
    }

  /* Put this channel on both chains, last of all, so the peer is only taken
   * down once everything else on this core has had its turn.
   */

  ipc->panic_nb.notifier_call  = reset_ipc_panic_notifier;
#ifdef CONFIG_BMP
  ipc->panic_nb.priority       = INT_MIN + 1;
#else
  ipc->panic_nb.priority       = INT_MIN + 2;
#endif
  ipc->reboot_nb.notifier_call = reset_ipc_reboot_notifier;
  ipc->reboot_nb.priority      = INT_MIN;

  panic_notifier_chain_register(&ipc->panic_nb);
  register_reboot_notifier(&ipc->reboot_nb);

  /* Hand back a reset control, so the caller drives the peer through the
   * framework.  Roll back on failure: NULL means nothing is registered.
   */

  rstc = reset_control_get_exclusive(cpuname);
  if (rstc == NULL)
    {
      rsterr("reset_ipc get reset control failed: %s\n", cpuname);
      reset_ipc_unregister(ipc);
    }

  return rstc;
}

/****************************************************************************
 * Name: reset_ipc_unregister
 ****************************************************************************/

void reset_ipc_unregister(FAR struct reset_ipc_s *ipc)
{
  /* Leave the chains first: they hold pointers into this channel. */

  panic_notifier_chain_unregister(&ipc->panic_nb);
  unregister_reboot_notifier(&ipc->reboot_nb);

  /* Take the callback back, so no late notification reaches us. */

  if (ipc->ops->register_callback != NULL)
    {
      ipc->ops->register_callback(ipc, NULL);
    }

  reset_controller_unregister(&ipc->rcdev);

  /* Drop a pending boardctl and wait out a running one. */

  work_cancel_sync(HPWORK, &ipc->work);

  /* Nothing references the channel now; a stale use faults on NULL. */

  memset(ipc, 0, sizeof(*ipc));
}

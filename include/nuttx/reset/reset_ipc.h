/****************************************************************************
 * include/nuttx/reset/reset_ipc.h
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

#ifndef __INCLUDE_NUTTX_RESET_RESET_IPC_H
#define __INCLUDE_NUTTX_RESET_RESET_IPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_RESET_IPC

#include <stdbool.h>
#include <stdint.h>
#include <sys/boardctl.h>

#include <nuttx/compiler.h>
#include <nuttx/notifier.h>
#include <nuttx/reset/reset-controller.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Reason reserved above the BOARDIOC_SOFTRESETCAUSE_xxx range: ask for
 * whatever action the transport keeps for itself instead of resetting the
 * peer.  The reset framework only carries it.
 */

#define RESET_REASON_PRIVATE     (BOARDIOC_SOFTRESETCAUSE_MAX + 0)

/* Reason reset_ipc sends on its own to park the peer while this core logs
 * a panic.  Never accepted from the reset framework.
 */

#define RESET_IPC_PAUSE          (BOARDIOC_SOFTRESETCAUSE_MAX + 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct reset_ipc_s;

/* Run by the transport from its notification path, to pick up whatever the
 * peer has put in the shared status word.
 */

typedef CODE void (*reset_ipc_callback_t)(FAR struct reset_ipc_s *ipc);

struct reset_ipc_ops_s
{
  /* Trigger the inter-processor interrupt towards the peer core. */

  CODE void (*notify)(FAR struct reset_ipc_s *ipc);

  /* Optional hardware delivery of a reason, tried first by
   * reset_ipc_set_status; return -ENOTSUP to fall back to the shared-word
   * handshake.
   */

  CODE int (*set_status)(FAR struct reset_ipc_s *ipc,
                         unsigned int reason);

  /* Optional handler for RESET_REASON_PRIVATE from the peer, which
   * reset_ipc reserves but cannot act on itself since only the transport
   * knows what it means.
   */

  CODE void (*handle_status)(FAR struct reset_ipc_s *ipc,
                             unsigned int reason);

  /* Optional dump of the peer's trace buffer at reset time. */

  CODE void (*dump)(FAR struct reset_ipc_s *ipc);

  /* Take the callback to run whenever the peer notifies this core.  Called
   * once from reset_ipc_register(), before the transport starts delivering
   * notifications.  A transport that never notifies may leave it NULL.
   */

  CODE void (*register_callback)(FAR struct reset_ipc_s *ipc,
                                 reset_ipc_callback_t callback);
};

/* One channel per peer core, embedded in the transport's per-peer private
 * data.  rcdev is the reset framework device fronting that peer and comes
 * first, so container_of() back from the framework is a plain cast; its name
 * doubles as the peer cpu name.  local/remote are the two halves of the
 * shared status pair and stay non-NULL while registered, so the status paths
 * never check them.  shadow holds the last peer word acted on, for dedup,
 * and starts clean at every reset_ipc_register() like local does.  The two
 * notifier blocks put this channel on the panic and reboot chains, which is
 * what brings the peer down with this core.
 */

struct reset_ipc_s
{
  struct reset_controller_dev       rcdev;     /* Reset framework device */
  struct notifier_block             panic_nb;  /* Panic chain entry */
  struct notifier_block             reboot_nb; /* Reboot chain entry */
  FAR const struct reset_ipc_ops_s *ops;       /* Transport callbacks */
  FAR uint32_t                     *local;     /* Local status word */
  FAR uint32_t                     *remote;    /* Peer status word */
  uint32_t                          shadow;    /* Last peer word seen */
  struct work_s                     work;      /* Deferred boardctl */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: reset_ipc_register
 *
 * Description:
 *   Register a channel, exposing the peer core to the reset framework so it
 *   can be reset through reset_control_* and joining the framework's
 *   panic/reboot fan-out on the calling core.
 *
 *   Call before the transport starts delivering notifications, and from the
 *   core that owns the channel.  Balance with reset_ipc_unregister() from
 *   the stop path: the reset controller registration is not idempotent.
 *
 * Input Parameters:
 *   ipc     - The channel embedded in the transport private data
 *   ops     - Transport callbacks, must stay valid while registered
 *   cpuname - Peer cpu name, used for log messages and as the reset
 *             controller device name.  Not copied.
 *   status  - Address of a uint32_t pair shared with the peer, same memory
 *             on both sides.  Word 0 is written by the master, word 1 by the
 *             slave, each core reading only the other's.  Must be uint32_t
 *             aligned and non-cacheable (the words share a cache line), and
 *             ideally survive a peer reset so the reason stays readable.
 *   master  - True on the master core, false on the slave.
 *
 * Returned Value:
 *   An exclusive reset control on the channel, to drive the peer with
 *   reset_control_reset_reason(), or NULL on failure.  Release it with
 *   reset_control_put() before reset_ipc_unregister().  A NULL return leaves
 *   nothing registered, so there is nothing to undo.
 *
 ****************************************************************************/

FAR struct reset_control *
reset_ipc_register(FAR struct reset_ipc_s *ipc,
                   FAR const struct reset_ipc_ops_s *ops,
                   FAR const char *cpuname,
                   uintptr_t status, bool master);

/****************************************************************************
 * Name: reset_ipc_unregister
 *
 * Description:
 *   Remove a channel from the reset framework, which also drops it from the
 *   panic/reboot fan-out, and drop its status words.  Takes a mutex, so not
 *   callable from the interrupt path.  The transport must have stopped
 *   delivering notifications first:
 *   the callback is taken back here, but one already running races with it.
 *
 ****************************************************************************/

void reset_ipc_unregister(FAR struct reset_ipc_s *ipc);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_RESET_IPC */
#endif /* __INCLUDE_NUTTX_RESET_RESET_IPC_H */

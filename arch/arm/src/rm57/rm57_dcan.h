/****************************************************************************
 * arch/arm/src/rm57/rm57_dcan.h
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

/* Framework-agnostic DCAN core driver.  The character-device frontend
 * (rm57_dcan_chardev.c) and the SocketCAN frontend (rm57_dcan_sock.c)
 * both sit on top of this API; each DCAN instance is bound to exactly
 * one frontend, per the RM57_DCANn_CHARDRIVER/RM57_DCANn_SOCKET choice.
 *
 * Locking/context rules:
 *  - IF1 transfers are used by thread-context entry points only and are
 *    serialized by the per-instance iflock mutex.
 *  - IF2 transfers are used exclusively by the interrupt handlers.
 *  - The rxframe/txdone/txempty/errevent callbacks run in interrupt
 *    context (or with interrupts disabled) - same rules as can_receive()
 *    and can_input().
 */

#ifndef __ARCH_ARM_SRC_RM57_RM57_DCAN_H
#define __ARCH_ARM_SRC_RM57_RM57_DCAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/mutex.h>
#include <nuttx/wdog.h>

#ifdef CONFIG_RM57_DCAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* At least one instance uses the IF3 DMA RX path */

#if defined(CONFIG_RM57_DCAN1_RX_IF3DMA) || \
    defined(CONFIG_RM57_DCAN2_RX_IF3DMA) || \
    defined(CONFIG_RM57_DCAN3_RX_IF3DMA) || \
    defined(CONFIG_RM57_DCAN4_RX_IF3DMA)
#  define RM57_DCAN_HAVE_IF3DMA 1
#endif

#define RM57_DCAN_NTXMBOX      CONFIG_RM57_DCAN_TXFIFO_DEPTH
#define RM57_DCAN_NRXMBOX      (64 - RM57_DCAN_NTXMBOX)
#define RM57_DCAN_TXMBOX_FIRST (RM57_DCAN_NRXMBOX + 1) /* 1-based */

/* RX path selection per instance (mutually exclusive Kconfig choice) */

#define RM57_DCAN_RXPATH_PIO    0 /* IF2 programmed I/O from line-1 ISR */
#define RM57_DCAN_RXPATH_IF3IRQ 1 /* IF3 auto-update, IF3 interrupt */
#define RM57_DCAN_RXPATH_IF3DMA 2 /* IF3 auto-update, DMA (EXPERIMENTAL) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Bit timing in raw (1-based) terms: bit time = 1 + tseg1 + tseg2 time
 * quanta, tq = brp / CAN_CLK.  Constraints: brp 1..1024, tseg1 2..16,
 * tseg2 1..8, sjw 1..4 (and sjw <= tseg2), 8 <= 1+tseg1+tseg2 <= 25.
 */

struct rm57can_timing_s
{
  uint32_t baud;   /* Nominal bit rate in bits/second */
  uint16_t brp;    /* Baud rate prescaler (1..1024) */
  uint8_t  tseg1;  /* Time quanta before sample point, excl. sync (2..16) */
  uint8_t  tseg2;  /* Time quanta after sample point (1..8) */
  uint8_t  sjw;    /* Synchronization jump width (1..4) */
};

/* Callbacks a frontend registers with the core.  All run in interrupt
 * context unless noted.
 */

struct rm57can_cbs_s
{
  /* One received frame.  arb/mctl/da/db are the raw IFxARB/IFxMCTL/
   * IFxDATA/IFxDATB register images of the message object.
   */

  void (*rxframe)(void *ctx, uint32_t arb, uint32_t mctl,
                  uint32_t da, uint32_t db);

  /* One TX mailbox completed transmission */

  void (*txdone)(void *ctx);

  /* The hardware TX FIFO fully drained (all pending TX done) */

  void (*txempty)(void *ctx);

  /* Status/error event: snapshot of ES (read-cleared by the ISR) and
   * the current ERRC value.  Bus-off entry/recovery included (ES.BOff).
   */

  void (*errevent)(void *ctx, uint32_t es, uint32_t errc);

  /* A received frame was overwritten in its message object before the
   * driver could read it (MCTL.MsgLst): the controller's own receive
   * buffer overflowed.  Distinct from the frontend's software queue
   * overflowing, which the frontend reports itself.
   */

  void (*rxoverflow)(void *ctx);
};

/* One DCAN instance.  Frontends treat this as opaque except through the
 * functions below; it is defined here so they can embed/allocate it.
 */

struct rm57can_s
{
  /* Fixed configuration */

  uint32_t base;         /* Control register base address */
  uint8_t  port;         /* 1..4 */
  uint8_t  irq0;         /* VIM channel of interrupt line 0 (level 0) */
  uint8_t  irq1;         /* VIM channel of interrupt line 1 (level 1) */
  uint8_t  irqif3;       /* VIM channel of the IF3 interrupt */
  uint8_t  msinena;      /* SYS MSINENA bit for message RAM auto-init */
  uint8_t  rxpath;       /* RM57_DCAN_RXPATH_* */
  uint32_t initial_baud; /* CONFIG_RM57_DCANn_BAUD */
#ifdef RM57_DCAN_HAVE_IF3DMA
  uint8_t  dmareq;       /* DMA request line of the IF3 interface */
  uint32_t *if3buf;      /* 32-byte aligned 4-word IF3 DMA landing buffer */
#endif

  /* Frontend binding */

  const struct rm57can_cbs_s *cbs;
  void *ctx;

  /* State */

  mutex_t  iflock;       /* Serializes thread-context IF1 use */
  uint64_t mbox_used;    /* RX mailbox allocation bitmap (bit n = mbox n+1) */

  /* Per-slot mailbox bitmaps */

  uint64_t filters[CONFIG_RM57_DCAN_NFILTERS_MAX];
  bool     filter_ext[CONFIG_RM57_DCAN_NFILTERS_MAX];
  uint64_t catchall_map; /* Mailboxes claimed by the catch-all filter */
  bool     catchall;     /* Default catch-all filter installed */
  uint8_t  nfilters;     /* Active user filter count */
  uint8_t  tx_used;      /* TX mailboxes filled since last full drain */
  uint8_t  tx_inflight;  /* TX mailboxes not yet completed */
  bool     setup;        /* Controller is out of Init */
  bool     boff;         /* Bus-off state (ES.BOff seen, not recovered) */
  bool     loopback;     /* TEST.LBack active */
  bool     silent;       /* TEST.Silent active */
  bool     nart;         /* CTL.DAR active */
  bool     abom;         /* CTL.ABO active */
  struct rm57can_timing_s timing;
  uint32_t rxlost;       /* MsgLst overflow event count */
#ifdef RM57_DCAN_HAVE_IF3DMA
  void    *dma;          /* DMA_HANDLE of the armed IF3 channel */
  struct wdog_s if3wd;   /* Stuck-request watchdog */
  bool     if3stuck;     /* IF3OBS.UPD seen pending on last watchdog tick */
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Instance lookup: n = 0..3 for DCAN1..DCAN4.  Returns NULL when the
 * instance is not enabled in the configuration.
 */

struct rm57can_s *rm57can_instance(int n);

/* Bind a frontend and reset driver state.  Must be called once before
 * rm57can_setup().
 */

int rm57can_init(struct rm57can_s *priv,
                 const struct rm57can_cbs_s *cbs, void *ctx);

/* Full hardware bring-up: message RAM auto-init, bit timing, mailbox
 * partitioning, interrupt attach/enable, exit Init.  Thread context.
 */

int rm57can_setup(struct rm57can_s *priv);

/* Reverse of setup: enter Init, disable and detach interrupts */

void rm57can_shutdown(struct rm57can_s *priv);

/* Bit timing.  rm57can_timing_from_baud() fills in brp/tseg1/tseg2/sjw
 * for the given bit rate; rm57can_set_timing() programs the BTR (round
 * trips through Init+CCE, briefly disturbing the bus).
 */

int rm57can_timing_from_baud(uint32_t baud, struct rm57can_timing_s *timing);
int rm57can_set_timing(struct rm57can_s *priv,
                       const struct rm57can_timing_s *timing);

/* Transmission.  rm57can_send() returns -EBUSY when no TX mailbox is
 * available (poll rm57can_txready() first) and -ENETDOWN during bus-off.
 */

int  rm57can_send(struct rm57can_s *priv, bool extid, bool rtr,
                  uint32_t id, uint8_t dlc, const uint8_t *data);
bool rm57can_txready(struct rm57can_s *priv);
bool rm57can_txempty(struct rm57can_s *priv);

/* Acceptance filters.  Adding the first user filter replaces the
 * default catch-all; deleting the last restores it.  Returns the filter
 * handle (>= 0) or a negated errno.
 */

int rm57can_add_filter(struct rm57can_s *priv, bool ext,
                       uint32_t id, uint32_t mask);
int rm57can_del_filter(struct rm57can_s *priv, int handle);

/* Modes and recovery */

int rm57can_connmodes(struct rm57can_s *priv, bool loopback, bool silent);
int rm57can_set_nart(struct rm57can_s *priv, bool nart);
int rm57can_set_abom(struct rm57can_s *priv, bool abom);
int rm57can_busoff_recovery(struct rm57can_s *priv);

/* Interrupt gating for the character driver's rxint/txint callbacks.
 * rxint masks interrupt line 1 (RX mailboxes) at the VIM; txint is
 * currently a no-op because line 0 also carries status/error interrupts.
 */

void rm57can_rxint(struct rm57can_s *priv, bool enable);
void rm57can_txint(struct rm57can_s *priv, bool enable);

/* Board-visible frontend entry points */

#ifdef CONFIG_CAN
int rm57_dcan_chardev_initialize(const char *path, int n);
#endif
#ifdef CONFIG_NET_CAN
int rm57_dcan_sock_initialize(int n);
#endif

#endif /* CONFIG_RM57_DCAN */
#endif /* __ARCH_ARM_SRC_RM57_RM57_DCAN_H */

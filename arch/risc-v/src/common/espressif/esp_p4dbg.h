/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_p4dbg.h
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
 * Debug counters for the protected build, CONFIG_ESPRESSIF_P4DBG.
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_P4DBG_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_P4DBG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#ifdef CONFIG_ESPRESSIF_P4DBG
#  define P4DBG 1

extern volatile uint32_t g_p4dbg_irq_n;
extern volatile uint32_t g_p4dbg_exc_n;
extern volatile uint32_t g_p4dbg_last_mcause;
extern volatile uint32_t g_p4dbg_tick_n;
extern volatile uint32_t g_p4dbg_idle_n;
extern volatile uint32_t g_p4dbg_idle_mint;
extern volatile uint32_t g_p4dbg_idle_mstatus;

/* Onset detector for the stuck mintstatus.mil.  mcause.mpil sampled at
 * tick entry is the CLIC level of the context the tick INTERRUPTED, which
 * for task context must be 0.  The first tick that sees it non-zero is
 * where the level started sticking.
 */

extern volatile uint32_t g_p4dbg_mil_first_tick;
extern volatile uint32_t g_p4dbg_mil_first_mcause;
extern volatile uint32_t g_p4dbg_mil_first_epc;
extern volatile uint32_t g_p4dbg_mil_ticks;
extern volatile uint32_t g_p4dbg_mil_last_zero_tick;

/* The pre-rev3 ESP32-P4 gates interrupt delivery on a MEMORY-MAPPED CLIC
 * threshold (INTTHRESH_STANDARD = 0), not on a CSR, and the whole CLIC
 * block reads back as zero over the JTAG debug bus -- so it has to be
 * sampled by the target itself.  Snapshot it from the tick handler (last
 * healthy state) and from up_idle (hung state) and diff the two.
 */

#define P4DBG_CLIC_BASE   0x20800000  /* CLIC_INT_CONFIG / INFO / THRESH   */
#define P4DBG_CLIC_CTRL   0x20801000  /* per-source IP/IE/ATTR/CTL, 1 word */
#define P4DBG_CLIC_N      48

extern volatile uint32_t g_p4dbg_tick_thresh;
extern volatile uint32_t g_p4dbg_tick_mint;
extern volatile uint32_t g_p4dbg_tick_clic[P4DBG_CLIC_N];
extern volatile uint32_t g_p4dbg_idle_thresh;
extern volatile uint32_t g_p4dbg_idle_clic[P4DBG_CLIC_N];

/* mintstatus sampled from up_idle -- TASK context, not a handler -- every
 * time it changes.  g_p4dbg_tick_mint is sampled inside the tick handler,
 * where mil is legitimately raised, so it cannot answer "was mil ever 0 in
 * task context".  This can.
 */

#define P4DBG_MINTLOG_N   12

extern volatile uint32_t g_p4dbg_mintlog[P4DBG_MINTLOG_N][2];  /* n, mint */
extern volatile uint32_t g_p4dbg_mintlog_n;

/* DIAGNOSTIC, not a fix: try writing mintstatus.mil down to canonical
 * level 0 from the idle loop and see (a) whether the CSR is writable and
 * (b) whether interrupt delivery resumes.  Answers "is the stuck level the
 * blocker" directly, instead of inferring it from another failed fix.
 */

extern volatile uint32_t g_p4dbg_force_mil0;     /* 1 = attempt the write */
extern volatile uint32_t g_p4dbg_mil_before;
extern volatile uint32_t g_p4dbg_mil_after;
#endif

#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_P4DBG_H */

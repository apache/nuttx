/****************************************************************************
 * arch/risc-v/src/common/riscv_debug.c
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
 * Notice:
 *
 * This driver is based on the RISC-V Debug Specification, version 0.13.2.
 * The latest version of the specification can be found at:
 * https://github.com/riscv/riscv-debug-spec
 *
 * The 1.0 version of the specification is still in RC phase, so there are
 * no chips that support it yet. The 0.13.2 version is the latest stable
 * version and some chips support it (e.g. QEMU RV, ESP32C3, BL602 etc).
 *
 * So this driver may needs to be updated when there is a new chip that
 * supports the 1.0 version of the specification.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include <arch/chip/chip.h>
#include <arch/csr.h>

#include <stdbool.h>

#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check the essential definition that must from chip vendor */

#define TRIGGER_TYPE_NONE          0 /* There is no trigger at this tselect */
#define TRIGGER_TYPE_LEGACY_SIFIVE 1 /* Legacy SiFive address match trigger */
#define TRIGGER_TYPE_ADDRESS_DATA  2 /* Address/data match trigger */
#define TRIGGER_TYPE_ICOUNT        3 /* Instruction count trigger */
#define TRIGGER_TYPE_ITRIGGER      4 /* Interrupt trigger */
#define TRIGGER_TYPE_ETRIGGER      5 /* Exception trigger */

#define MATCH_TYPE_EQUAL           0 /* Value equals to tdata2 */
#define MATCH_TYPE_TOPBITS         1 /* Match top M bits of tdata2 */
#define MATCH_TYPE_GREAT           2 /* Value great than tdata2 */
#define MATCH_TYPE_LESS            3 /* Value less than tdata2 */
#define MATCH_TYPE_LOWERHALF       4 /* Lower half of the value equals */
#define MATCH_TYPE_UPPERHALF       5 /* Upper half of the value equals */

#define ACTION_TYPE_EXCEPTION      0 /* Raise a breakpoint exception */
#define ACTION_TYPE_DEBUGMODE      1 /* Enter debug mode */

#define DMODE_TYPE_BOTH            0 /* Both Debug and M-mode can write the tdata */
#define DMODE_TYPE_ONLY            1 /* Only Debug Mode can write the tdata */

/****************************************************************************
 * Private Type
 ****************************************************************************/

/* Represent the tdata1 register, from the RISC-V Debug Specification 0.13.2
 * Read https://riscv.org/wp-content/uploads/2019/03/riscv-debug-release.pdf
 * for more information
 */

union tdata1
{
  uintptr_t reg;
  struct
    {
      uintptr_t load : 1;
      uintptr_t store : 1;
      uintptr_t execute : 1;
      uintptr_t u : 1;
      uintptr_t s : 1;
      uintptr_t reserved0 : 1;
      uintptr_t m : 1;
      uintptr_t match : 4;
      uintptr_t chain : 1;
      uintptr_t action : 4;
      uintptr_t sizelo : 2;
      uintptr_t timing : 1;
      uintptr_t select : 1;
      uintptr_t hit : 1;
#ifdef CONFIG_ARCH_RV64
      uintptr_t sizehi : 2;
      uintptr_t reserved1 : 30;
#endif
      uintptr_t maskmax : 6;
      uintptr_t dmode : 1;
      uintptr_t type : 4;
    } match;

  struct
    {
      uintptr_t action: 6;
      uintptr_t u : 1;
      uintptr_t s : 1;
      uintptr_t reserved0 : 1;
      uintptr_t m : 1;
      uintptr_t count : 14;
      uintptr_t hit : 1;
      uintptr_t reserved1 : sizeof(uintptr_t) * 8 - 30;
      uintptr_t dmode : 1;
      uintptr_t type : 4;
    } icount;
};

struct riscv_debug_trigger
{
  int type;                  /* Trigger type */
  void *address;             /* Trigger address */
  size_t size;               /* Trigger region size */
  debug_callback_t callback; /* Debug callback */
  void *arg;                 /* Debug callback argument */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Save the trigger address info */

static int g_trigger_count = 0;
static struct riscv_debug_trigger *g_trigger_map;
static bool g_support_napot = false;
static bool g_debug_initiliazed = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_debug_find_slot
 *
 * Description:
 * Find the trigger slot by type, address and size, return the index of the
 * slot or -ENOENT if not found. And if address is NULL meand to find the
 * first empty slot.
 ****************************************************************************/

static int riscv_debug_find_slot(int type, void *address, size_t size)
{
  int i;

  for (i = 0; i < g_trigger_count; i++)
    {
      if (g_trigger_map[i].type == type &&
          g_trigger_map[i].address == address &&
          g_trigger_map[i].size == size)
        {
          return i;
        }
    }

  return -ENOENT;
}

static int riscv_debug_handler(int irq, void *context, void *arg)
{
  /* Get the trigger index */

  int slot = READ_CSR(CSR_TSELECT);

  DEBUGASSERT(slot >= 0);
  DEBUGASSERT(slot < g_trigger_count);

  /* Call the trigger callback */

  if (g_trigger_map[slot].callback)
    {
      g_trigger_map[slot].callback(g_trigger_map[slot].type,
                                   g_trigger_map[slot].address,
                                   g_trigger_map[slot].size,
                                   g_trigger_map[slot].arg);
    }

  return 0;
}

/****************************************************************************
 * Name: riscv_debug_init
 ****************************************************************************/

static int riscv_debug_init(void)
{
  union tdata1 reg;

  /* Attach the debug exception handler */

  irq_attach(RISCV_IRQ_BPOINT, riscv_debug_handler, NULL);

  /* Detect the number of triggers by write a huge value
   * to tselect and read it back
   */

  WRITE_CSR(CSR_TSELECT, 0xffffffff);

  g_trigger_count = READ_CSR(CSR_TSELECT);

  if (g_trigger_count == 0)
    {
      return -ENOENT;
    }

  /* Allocate the trigger map */

  g_trigger_map = kmm_zalloc(sizeof(struct riscv_debug_trigger) *
                             g_trigger_count);

  if (!g_trigger_map)
    {
      return -ENOMEM;
    }

  /* Detect the support of NAPOT by trigger 0 */

  WRITE_CSR(CSR_TSELECT, 0);

  reg.reg = READ_CSR(CSR_TDATA1);

  /* REVISIT: NAPOT match is supported and tested on
   * QEMU and ESP32C3, prefer to use it.
   */

  reg.match.type = TRIGGER_TYPE_ADDRESS_DATA;
  reg.match.match = MATCH_TYPE_TOPBITS;

  /* Write it to tdata1 and read back
   * to check if the NAPOT is supported
   */

  WRITE_CSR(CSR_TDATA1, reg.reg);
  reg.reg = READ_CSR(CSR_TDATA1);

  if (reg.match.match == MATCH_TYPE_TOPBITS)
    {
      g_support_napot = true;
    }

  /* Special handling for QEMU since it does not implement
   * the TCONTROL register, verified on QEMU 9.0.1.
   */

#ifndef CONFIG_ARCH_CHIP_QEMU_RV
  /* Enable trigger in M-mode */

  WRITE_CSR(CSR_TCONTROL, CSR_TCONTROL_MPTE | CSR_TCONTROL_MTE);
#endif

  return 0;
}

/****************************************************************************
 * Name: riscv_debug_setup_match
 ****************************************************************************/

static uintptr_t riscv_debug_setup_match(int type, void *addr, size_t size)
{
  union tdata1 reg;
  uintptr_t    addr_napot;

  if (type == DEBUGPOINT_BREAKPOINT)
    {
      reg.match.execute = 1;
    }
  else if (type == DEBUGPOINT_WATCHPOINT_RO)
    {
      reg.match.load = 1;
    }
  else if (type == DEBUGPOINT_WATCHPOINT_WO)
    {
      reg.match.store = 1;
    }
  else if (type == DEBUGPOINT_WATCHPOINT_RW)
    {
      reg.match.load = 1;
      reg.match.store = 1;
    }

  reg.match.type = TRIGGER_TYPE_ADDRESS_DATA;
  reg.match.m = 1;
  reg.match.u = 1;
  reg.match.hit = 0;
  reg.match.dmode = DMODE_TYPE_BOTH;
  reg.match.action = ACTION_TYPE_EXCEPTION;

  /* From RISC-V Debug Specification:
   * tdata1(mcontrol) match = 0 : Exact byte match
   *
   * tdata1(mcontrol) match = 1 : NAPOT (Naturally Aligned Power-Of-Two):
   *
   * Examples for understanding how to calculate match pattern to tdata2:
   *
   * nnnn...nnnnn 1-byte  Exact byte match
   * nnnn...nnnn0 2-byte  NAPOT range
   * nnnn...nnn01 4-byte  NAPOT range
   * nnnn...nn011 8-byte  NAPOT range
   * nnnn...n0111 16-byte NAPOT range
   * nnnn...01111 32-byte NAPOT range
   * ...
   * n011...11111 2^31 byte NAPOT range
   * where n are bits from original address
   */

  if (size > 1 && g_support_napot)
    {
      reg.match.match = MATCH_TYPE_TOPBITS;
      addr_napot = ((uintptr_t)addr & ~(size - 1)) |
                    ((size - 1) >> 1);
      WRITE_CSR(CSR_TDATA2, addr_napot);
    }
  else
    {
      reg.match.match = MATCH_TYPE_EQUAL;
      WRITE_CSR(CSR_TDATA2, (uintptr_t)addr);
    }

  return reg.reg;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_debugpoint_add
 ****************************************************************************/

int up_debugpoint_add(int type, void *addr, size_t size,
                      debug_callback_t callback, void *arg)
{
  int slot;
  union tdata1 reg;
  int ret = OK;

  /* Initialize the debug module if it is not initialized yet */

  if (g_debug_initiliazed == false)
    {
      ret = riscv_debug_init();
      if (ret < 0)
        {
          return ret;
        }

      g_debug_initiliazed = true;
    }

  /* Find a free slot */

  slot = riscv_debug_find_slot(0, 0, 0);
  if (slot < 0)
    {
      return slot;
    }

  /* Select the trigger */

  WRITE_CSR(CSR_TSELECT, slot);

  switch (type)
    {
      case DEBUGPOINT_STEPPOINT:
        /* NOTICE: STEPPOINT implemented but not tested since no such
         * hardware that implemented icount is available.
         */

        reg.reg = 0;
        reg.icount.type = TRIGGER_TYPE_ICOUNT;
        reg.icount.dmode = DMODE_TYPE_BOTH;
        reg.icount.count = 1;
        reg.icount.hit = 0;
        reg.icount.m = 1;
        reg.icount.s = 1;
        reg.icount.u = 1;
        reg.icount.action = ACTION_TYPE_EXCEPTION;
        break;
      case DEBUGPOINT_BREAKPOINT:
      case DEBUGPOINT_WATCHPOINT_RO:
      case DEBUGPOINT_WATCHPOINT_RW:
      case DEBUGPOINT_WATCHPOINT_WO:
        reg.reg = riscv_debug_setup_match(type, addr, size);
        break;
      default:
        return -EINVAL;
    }

  /* Register the callback and arg */

  g_trigger_map[slot].type     = type;
  g_trigger_map[slot].address  = addr;
  g_trigger_map[slot].size     = size;
  g_trigger_map[slot].callback = callback;
  g_trigger_map[slot].arg      = arg;
  WRITE_CSR(CSR_TDATA1, reg.reg);

  return 0;
}

/****************************************************************************
 * Name: up_debugpoint_remove
 ****************************************************************************/

int up_debugpoint_remove(int type, void *addr, size_t size)
{
  int slot;

  /* Find the existing debugpoint */

  slot = riscv_debug_find_slot(type, addr, size);
  if (slot < 0)
    {
      return slot;
    }

  /* Select the trigger and clear setting by write 0 to tdata1 */

  WRITE_CSR(CSR_TSELECT, slot);
  WRITE_CSR(CSR_TDATA1, 0);

  /* Clear the callback and arg */

  memset(&g_trigger_map[slot], 0, sizeof(g_trigger_map[slot]));

  return 0;
}

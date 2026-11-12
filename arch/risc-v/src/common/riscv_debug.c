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
 * This driver implements the native trigger interface described by the
 * RISC-V Debug Specification, version 1.0 (Sdtrig). The specification can
 * be found at:
 * https://docs.riscv.org/reference/debug/v1.0/index.html
 *
 * mcontrol (type 2) is deprecated in 1.0 but is still what most current
 * cores implement (e.g. QEMU RV, ESP32-C3). mcontrol6 (type 6) is used when
 * the trigger only accepts type 6. Breakpoints always use an exact address
 * match; watchpoints use NAPOT only when the hardware reports a maskmax
 * large enough for the requested size.
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
#include <string.h>

#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Trigger types from the RISC-V Debug Specification 1.0 */

#define TRIGGER_TYPE_NONE          0  /* No trigger at this tselect */
#define TRIGGER_TYPE_LEGACY_SIFIVE 1  /* Legacy SiFive address match */
#define TRIGGER_TYPE_MCONTROL      2  /* Address/data match trigger */
#define TRIGGER_TYPE_ICOUNT        3  /* Instruction count trigger */
#define TRIGGER_TYPE_ITRIGGER      4  /* Interrupt trigger */
#define TRIGGER_TYPE_ETRIGGER      5  /* Exception trigger */
#define TRIGGER_TYPE_MCONTROL6     6  /* Address/data match trigger, 1.0 */
#define TRIGGER_TYPE_DISABLED      15 /* Trigger exists but is disabled */

#define MATCH_TYPE_EQUAL           0  /* Value equals tdata2 */
#define MATCH_TYPE_TOPBITS         1  /* NAPOT, top M bits match tdata2 */

/* Match capabilities discovered by writing an inert trigger to each slot */

#define TRIGGER_CAP_EXECUTE        (1 << 0)
#define TRIGGER_CAP_LOAD           (1 << 1)
#define TRIGGER_CAP_STORE          (1 << 2)
#define TRIGGER_CAP_ICOUNT         (1 << 3)

#define ACTION_TYPE_EXCEPTION      0  /* Raise a breakpoint exception */
#define ACTION_TYPE_DEBUGMODE      1  /* Enter debug mode */

#define DMODE_TYPE_BOTH            0  /* Both Debug and M-mode can write */

/* The specification does not define a way to read the number of triggers.
 * Enumeration stops when tselect no longer reads back the written index,
 * so this only bounds a broken implementation.
 */

#define RISCV_MAX_TRIGGERS         32

/****************************************************************************
 * Private Type
 ****************************************************************************/

/* Represent the tdata1 register from the RISC-V Debug Specification 1.0.
 * The type field is the top 4 bits of every layout.
 */

union tdata1
{
  uintptr_t reg;
  struct
    {
      uintptr_t load : 1;        /* 0 */
      uintptr_t store : 1;       /* 1 */
      uintptr_t execute : 1;     /* 2 */
      uintptr_t u : 1;           /* 3 */
      uintptr_t s : 1;           /* 4 */
      uintptr_t reserved0 : 1;   /* 5 */
      uintptr_t m : 1;           /* 6 */
      uintptr_t match : 4;       /* 10:7 */
      uintptr_t chain : 1;       /* 11 */
      uintptr_t action : 4;      /* 15:12 */
      uintptr_t sizelo : 2;      /* 17:16 */
      uintptr_t timing : 1;      /* 18 */
      uintptr_t select : 1;      /* 19 */
      uintptr_t hit : 1;         /* 20 */
#ifdef CONFIG_ARCH_RV64
      uintptr_t sizehi : 2;      /* 22:21 */
      uintptr_t reserved1 : 30;  /* 52:23 */
#endif
      uintptr_t maskmax : 6;     /* XLEN-6:XLEN-11 */
      uintptr_t dmode : 1;       /* XLEN-5 */
      uintptr_t type : 4;        /* XLEN-1:XLEN-4 */
    } match;

  struct
    {
      uintptr_t load : 1;        /* 0 */
      uintptr_t store : 1;       /* 1 */
      uintptr_t execute : 1;     /* 2 */
      uintptr_t u : 1;           /* 3 */
      uintptr_t s : 1;           /* 4 */
      uintptr_t uncertainen : 1; /* 5 */
      uintptr_t m : 1;           /* 6 */
      uintptr_t match : 4;       /* 10:7 */
      uintptr_t chain : 1;       /* 11 */
      uintptr_t action : 4;      /* 15:12 */
      uintptr_t size : 3;        /* 18:16 */
      uintptr_t reserved0 : 2;   /* 20:19 */
      uintptr_t select : 1;      /* 21 */
      uintptr_t hit0 : 1;        /* 22 */
      uintptr_t vu : 1;          /* 23 */
      uintptr_t vs : 1;          /* 24 */
      uintptr_t hit1 : 1;        /* 25 */
      uintptr_t uncertain : 1;   /* 26 */
#ifdef CONFIG_ARCH_RV64
      uintptr_t reserved1 : 32;  /* 58:27 */
#endif
      uintptr_t dmode : 1;       /* XLEN-5 */
      uintptr_t type : 4;        /* XLEN-1:XLEN-4 */
    } mcontrol6;

  struct
    {
      uintptr_t action : 6;      /* 5:0 */
      uintptr_t u : 1;           /* 6 */
      uintptr_t s : 1;           /* 7 */
      uintptr_t pending : 1;     /* 8 */
      uintptr_t m : 1;           /* 9 */
      uintptr_t count : 14;      /* 23:10 */
      uintptr_t hit : 1;         /* 24 */
      uintptr_t reserved1 : sizeof(uintptr_t) * 8 - 30;
      uintptr_t dmode : 1;       /* XLEN-5 */
      uintptr_t type : 4;        /* XLEN-1:XLEN-4 */
    } icount;
};

struct riscv_debug_trigger
{
  int type;                  /* Trigger type */
  int hwtype;                /* Trigger type programmed in hardware */
  void *address;             /* Trigger address */
  size_t size;               /* Trigger region size */
  bool napot;                /* tdata2 holds a NAPOT pattern */
  debug_callback_t callback; /* Debug callback */
  void *arg;                 /* Debug callback argument */
};

struct riscv_debug_s
{
  int count;                       /* Number of trigger slots */
  size_t napot_max;                /* Largest supported NAPOT range */
  int hwtype;                      /* Address/data trigger type */
  struct riscv_debug_trigger *map; /* Slot state */
  uint8_t *caps;                   /* Per-slot match capability */
  bool initialized;                /* Per-CPU state is ready */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Trigger CSRs are per hart, so the slot bookkeeping must be per CPU too.
 * g_debug is indexed by the hart ID, which is dense on the RISC-V chips
 * that implement Sdtrig (QEMU RV and ESP32-C3).
 */

static struct riscv_debug_s g_debug[CONFIG_NCPUS];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int riscv_debug_probe_icount(int slot);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_debug_state
 ****************************************************************************/

static struct riscv_debug_s *riscv_debug_state(void)
{
#ifdef CONFIG_SMP
  int cpu = riscv_hartid_to_cpuid(up_cpu_index());
#else
  int cpu = 0;
#endif

  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_NCPUS);
  return &g_debug[cpu];
}

/****************************************************************************
 * Name: riscv_debug_tdata1_type
 *
 * Description:
 *   Extract the trigger type from a tdata1 value. Every trigger layout
 *   stores it in the top four bits.
 *
 ****************************************************************************/

static int riscv_debug_tdata1_type(uintptr_t tdata1)
{
  return (tdata1 >> (sizeof(uintptr_t) * 8 - 4)) & 0xf;
}

/****************************************************************************
 * Name: riscv_debug_find_slot
 *
 * Description:
 *   Find the trigger slot by type, address and size, return the index of the
 *   slot or -ENOENT if not found. If address is NULL means to find the first
 *   empty slot.
 ****************************************************************************/

static int riscv_debug_find_slot(struct riscv_debug_s *dbg, int type,
                                 void *address, size_t size)
{
  int i;

  if (dbg->map == NULL)
    {
      return -ENOENT;
    }

  for (i = 0; i < dbg->count; i++)
    {
      if (dbg->map[i].type == type &&
          dbg->map[i].address == address &&
          dbg->map[i].size == size)
        {
          return i;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: riscv_debug_find_free_slot
 *
 * Description:
 *   Find a free slot that implements the match operation the debugpoint
 *   needs. Slots are not assumed to be identical: a core may tie load and
 *   store to zero on some slots, and a watchpoint placed there would never
 *   fire.
 *
 ****************************************************************************/

static int riscv_debug_find_free_slot(struct riscv_debug_s *dbg, int type)
{
  uint8_t need;
  bool capable = false;
  int i;

  switch (type)
    {
      case DEBUGPOINT_BREAKPOINT:
        need = TRIGGER_CAP_EXECUTE;
        break;

      case DEBUGPOINT_WATCHPOINT_RO:
        need = TRIGGER_CAP_LOAD;
        break;

      case DEBUGPOINT_WATCHPOINT_WO:
        need = TRIGGER_CAP_STORE;
        break;

      case DEBUGPOINT_WATCHPOINT_RW:
        need = TRIGGER_CAP_LOAD | TRIGGER_CAP_STORE;
        break;

      case DEBUGPOINT_STEPPOINT:
        need = TRIGGER_CAP_ICOUNT;
        break;

      default:
        need = 0;
        break;
    }

  for (i = 0; i < dbg->count; i++)
    {
      if (need != 0 &&
          (dbg->caps == NULL || (dbg->caps[i] & need) != need))
        {
          continue;
        }

      capable = true;

      if (dbg->map[i].type != 0 || dbg->map[i].address != NULL ||
          dbg->map[i].size != 0)
        {
          continue;
        }

      return i;
    }

  return capable ? -ENOSPC : -ENOSYS;
}

/****************************************************************************
 * Name: riscv_debug_disable
 *
 * Description:
 *   Disable the selected trigger. The specification guarantees that writing
 *   0 to tdata1 disables any trigger, but some implementations do not honor
 *   it: QEMU dispatches tdata1 writes by the type carried in the written
 *   value, so a write of 0 (type 0) is dropped and the trigger stays armed.
 *   Verify the result and clear the bits that enable matching if needed.
 *
 ****************************************************************************/

static void riscv_debug_disable(int slot)
{
  union tdata1 reg;

  WRITE_CSR(CSR_TSELECT, slot);
  WRITE_CSR(CSR_TDATA1, 0);

  reg.reg = READ_CSR(CSR_TDATA1);

  switch (riscv_debug_tdata1_type(reg.reg))
    {
      case TRIGGER_TYPE_ICOUNT:
        if (reg.icount.m || reg.icount.s || reg.icount.u)
          {
            reg.icount.m = 0;
            reg.icount.s = 0;
            reg.icount.u = 0;
            WRITE_CSR(CSR_TDATA1, reg.reg);
          }
        break;

      case TRIGGER_TYPE_MCONTROL:
      case TRIGGER_TYPE_MCONTROL6:
        if (reg.match.load || reg.match.store || reg.match.execute)
          {
            /* load, store and execute are bits 2:0 in both layouts */

            reg.reg &= ~(uintptr_t)0x7;
            WRITE_CSR(CSR_TDATA1, reg.reg);
          }
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: riscv_debug_clear_hit
 *
 * Description:
 *   Clear the hit bit of the slot that just fired. hit is WARL and may not
 *   be implemented, in which case writing 0 has no effect.
 *
 ****************************************************************************/

static void riscv_debug_clear_hit(int slot, int hwtype)
{
  union tdata1 reg;
  uintptr_t tselect = READ_CSR(CSR_TSELECT);

  WRITE_CSR(CSR_TSELECT, slot);
  reg.reg = READ_CSR(CSR_TDATA1);

  if (hwtype == TRIGGER_TYPE_ICOUNT)
    {
      reg.icount.hit = 0;
    }
  else if (hwtype == TRIGGER_TYPE_MCONTROL6)
    {
      reg.mcontrol6.hit0 = 0;
      reg.mcontrol6.hit1 = 0;
    }
  else
    {
      reg.match.hit = 0;
    }

  WRITE_CSR(CSR_TDATA1, reg.reg);
  WRITE_CSR(CSR_TSELECT, tselect);
}

/****************************************************************************
 * Name: riscv_debug_match
 *
 * Description:
 *   Check whether a registered debugpoint is the one that fired. tselect is
 *   only a selector and is not updated by hardware when a trigger fires;
 *   an execute trigger leaves the faulting PC in EPC and a load/store
 *   trigger leaves the accessed address in TVAL.
 *
 ****************************************************************************/

static bool riscv_debug_match(struct riscv_debug_trigger *trg,
                              uintptr_t epc, uintptr_t tval)
{
  uintptr_t base;

  if (trg->type == DEBUGPOINT_BREAKPOINT)
    {
      return epc == (uintptr_t)trg->address;
    }

  if (trg->napot)
    {
      base = (uintptr_t)trg->address & ~((uintptr_t)trg->size - 1);
      return tval >= base && tval < base + trg->size;
    }

  return tval == (uintptr_t)trg->address;
}

static int riscv_debug_handler(int irq, void *context, void *arg)
{
  struct riscv_debug_s *dbg = riscv_debug_state();
  uintptr_t epc   = READ_CSR(CSR_EPC);
  uintptr_t tval  = READ_CSR(CSR_TVAL);
  uintptr_t tselect;
  int step = -ENOENT;
  int hit  = -ENOENT;
  int i;

  if (dbg->map == NULL)
    {
      return riscv_exception(irq, context, arg);
    }

  tselect = READ_CSR(CSR_TSELECT);

  for (i = 0; i < dbg->count; i++)
    {
      if (dbg->map[i].callback == NULL)
        {
          continue;
        }

      if (dbg->map[i].type == DEBUGPOINT_STEPPOINT)
        {
          step = i;
        }
      else if (riscv_debug_match(&dbg->map[i], epc, tval))
        {
          hit = i;
          break;
        }
    }

  WRITE_CSR(CSR_TSELECT, tselect);

  if (hit < 0)
    {
      /* An icount steppoint has no address to compare against. The
       * specification also lets icount match a trap, so while a step is
       * in progress it is the only remaining candidate.
       */

      hit = step;
    }

  if (hit < 0)
    {
      /* Not one of our triggers (e.g. an ebreak): keep the default
       * behavior instead of silently returning to the same instruction.
       */

      return riscv_exception(irq, context, arg);
    }

  riscv_debug_clear_hit(hit, dbg->map[hit].hwtype);

  if (dbg->map[hit].callback != NULL)
    {
      dbg->map[hit].callback(dbg->map[hit].type, dbg->map[hit].address,
                             dbg->map[hit].size, dbg->map[hit].arg);
    }

  return 0;
}

/****************************************************************************
 * Name: riscv_debug_probe_count
 *
 * Description:
 *   Enumerate the trigger slots as described in the specification: write an
 *   index to tselect, read it back, and stop when the readback differs or
 *   when tdata1.type reports that the slot does not exist.
 *
 ****************************************************************************/

static int riscv_debug_probe_count(struct riscv_debug_s *dbg)
{
  uintptr_t i;

  for (i = 0; i < RISCV_MAX_TRIGGERS; i++)
    {
      WRITE_CSR(CSR_TSELECT, i);
      if (READ_CSR(CSR_TSELECT) != i)
        {
          break;
        }

      if (riscv_debug_tdata1_type(READ_CSR(CSR_TDATA1)) ==
          TRIGGER_TYPE_NONE)
        {
          break;
        }
    }

  dbg->count = i;
  return i > 0 ? OK : -ENOENT;
}

/****************************************************************************
 * Name: riscv_debug_probe_hwtype
 *
 * Description:
 *   Find out which address/data trigger type slot 0 accepts and how large a
 *   NAPOT range it supports. All slots are assumed to be identical.
 *
 ****************************************************************************/

static void riscv_debug_probe_hwtype(struct riscv_debug_s *dbg)
{
  union tdata1 reg;
  uintptr_t tdata1;
  unsigned int shift;

  dbg->hwtype    = TRIGGER_TYPE_NONE;
  dbg->napot_max = 0;

  riscv_debug_disable(0);

  reg.reg = 0;
  reg.match.type = TRIGGER_TYPE_MCONTROL;
  WRITE_CSR(CSR_TDATA1, reg.reg);
  tdata1 = READ_CSR(CSR_TDATA1);

  if (riscv_debug_tdata1_type(tdata1) == TRIGGER_TYPE_MCONTROL)
    {
      dbg->hwtype = TRIGGER_TYPE_MCONTROL;

      /* maskmax is read-only: 0 means NAPOT is not supported */

      reg.reg = tdata1;
      shift = reg.match.maskmax;
      if (shift >= sizeof(uintptr_t) * 8)
        {
          shift = sizeof(uintptr_t) * 8 - 1;
        }

      if (shift > 0)
        {
          dbg->napot_max = (size_t)1 << shift;
        }
    }

  /* Probe mcontrol6 as well: when mcontrol is rejected this is the only
   * address/data trigger the core may offer.
   */

  reg.reg = 0;
  reg.mcontrol6.type = TRIGGER_TYPE_MCONTROL6;
  WRITE_CSR(CSR_TDATA1, reg.reg);
  tdata1 = READ_CSR(CSR_TDATA1);

  if (riscv_debug_tdata1_type(tdata1) == TRIGGER_TYPE_MCONTROL6)
    {
      /* NAPOT on mcontrol6 needs a separate maskmax6 probe; exact match is
       * always supported, so keep napot_max at 0.
       */

      if (dbg->hwtype == TRIGGER_TYPE_NONE)
        {
          dbg->hwtype = TRIGGER_TYPE_MCONTROL6;
        }
    }

  riscv_debug_disable(0);
}

/****************************************************************************
 * Name: riscv_debug_probe_caps
 *
 * Description:
 *   Find out which match operations every slot implements. A trigger with
 *   no privilege mode enabled cannot fire, so the probe leaves each slot
 *   inert and the tdata1 readback shows the bits the hardware kept.
 *
 ****************************************************************************/

static void riscv_debug_probe_caps(struct riscv_debug_s *dbg)
{
  union tdata1 reg;
  uintptr_t tdata1;
  uintptr_t tselect;
  uintptr_t probe;
  uintptr_t i;

  if (dbg->hwtype == TRIGGER_TYPE_NONE || dbg->caps == NULL)
    {
      return;
    }

  tselect = READ_CSR(CSR_TSELECT);
  probe = (uintptr_t)dbg->hwtype << (sizeof(uintptr_t) * 8 - 4);

  for (i = 0; i < (uintptr_t)dbg->count; i++)
    {
      uint8_t caps = 0;

      WRITE_CSR(CSR_TSELECT, i);

      reg.reg = probe | 0x7;
      WRITE_CSR(CSR_TDATA1, reg.reg);
      tdata1 = READ_CSR(CSR_TDATA1);

      if (riscv_debug_tdata1_type(tdata1) == dbg->hwtype)
        {
          if ((tdata1 & 0x1) != 0)
            {
              caps |= TRIGGER_CAP_LOAD;
            }

          if ((tdata1 & 0x2) != 0)
            {
              caps |= TRIGGER_CAP_STORE;
            }

          if ((tdata1 & 0x4) != 0)
            {
              caps |= TRIGGER_CAP_EXECUTE;
            }
        }

      if (riscv_debug_probe_icount((int)i) == OK)
        {
          caps |= TRIGGER_CAP_ICOUNT;
        }

      dbg->caps[i] = caps;

      riscv_debug_disable((int)i);
    }

  WRITE_CSR(CSR_TSELECT, tselect);
}

/****************************************************************************
 * Name: riscv_debug_probe_icount
 *
 * Description:
 *   Check whether the selected slot accepts an icount trigger. Writing a
 *   count of 1 with no privilege mode enabled leaves it disabled.
 *
 ****************************************************************************/

static int riscv_debug_probe_icount(int slot)
{
  union tdata1 reg;

  reg.reg = 0;
  riscv_debug_disable(slot);

  reg.icount.type  = TRIGGER_TYPE_ICOUNT;
  reg.icount.count = 1;
  WRITE_CSR(CSR_TDATA1, reg.reg);

  if (riscv_debug_tdata1_type(READ_CSR(CSR_TDATA1)) !=
      TRIGGER_TYPE_ICOUNT)
    {
      riscv_debug_disable(slot);
      return -ENOSYS;
    }

  return OK;
}

/****************************************************************************
 * Name: riscv_debug_init
 ****************************************************************************/

static int riscv_debug_init(struct riscv_debug_s *dbg)
{
  int ret;

  ret = riscv_debug_probe_count(dbg);
  if (ret < 0)
    {
      return ret;
    }

  dbg->map = kmm_zalloc(sizeof(struct riscv_debug_trigger) * dbg->count);
  if (dbg->map == NULL)
    {
      dbg->count = 0;
      return -ENOMEM;
    }

  dbg->caps = kmm_zalloc(dbg->count);
  if (dbg->caps == NULL)
    {
      kmm_free(dbg->map);
      dbg->map = NULL;
      dbg->count = 0;
      return -ENOMEM;
    }

  riscv_debug_probe_hwtype(dbg);
  riscv_debug_probe_caps(dbg);

#ifndef CONFIG_ARCH_RISCV_NO_TCONTROL
  /* MTE lets action=0 triggers fire in M-mode. MPTE keeps MTE across a trap
   * into M-mode, otherwise the first trap would clear MTE for good.
   */

  WRITE_CSR(CSR_TCONTROL, CSR_TCONTROL_MPTE | CSR_TCONTROL_MTE);
#endif

  ret = irq_attach(RISCV_IRQ_BPOINT, riscv_debug_handler, NULL);
  if (ret < 0)
    {
      kmm_free(dbg->caps);
      kmm_free(dbg->map);
      dbg->caps = NULL;
      dbg->map = NULL;
      dbg->count = 0;
      return ret;
    }

  dbg->initialized = true;
  return OK;
}

/****************************************************************************
 * Name: riscv_debug_check_size
 *
 * Description:
 *   A hardware watchpoint can only describe a power-of-two range. Sizes
 *   above napot_max fall back to an exact address match, which still sees
 *   every access whose lowest address is the watched address.
 *
 ****************************************************************************/

static int riscv_debug_check_size(int type, size_t size)
{
  if (type == DEBUGPOINT_BREAKPOINT || size <= 1)
    {
      return OK;
    }

  if ((size & (size - 1)) != 0)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: riscv_debug_setup_match
 ****************************************************************************/

static void riscv_debug_setup_match(int hwtype, int type, void *addr,
                                    size_t size, size_t napot_max,
                                    uintptr_t *tdata1, uintptr_t *tdata2,
                                    bool *napot)
{
  union tdata1 reg;
  uintptr_t value = (uintptr_t)addr;
  bool load = false;
  bool store = false;
  bool execute = false;

  reg.reg = 0;

  if (type == DEBUGPOINT_BREAKPOINT)
    {
      execute = true;
    }
  else if (type == DEBUGPOINT_WATCHPOINT_RO)
    {
      load = true;
    }
  else if (type == DEBUGPOINT_WATCHPOINT_WO)
    {
      store = true;
    }
  else
    {
      load  = true;
      store = true;
    }

  *napot = type != DEBUGPOINT_BREAKPOINT && size > 1 &&
           napot_max >= size;

  if (*napot)
    {
      value = (value & ~(size - 1)) | ((size - 1) >> 1);
    }

  if (hwtype == TRIGGER_TYPE_MCONTROL6)
    {
      reg.mcontrol6.type      = TRIGGER_TYPE_MCONTROL6;
      reg.mcontrol6.load      = load;
      reg.mcontrol6.store     = store;
      reg.mcontrol6.execute   = execute;
      reg.mcontrol6.m         = 1;
      reg.mcontrol6.u         = 1;
      reg.mcontrol6.action    = ACTION_TYPE_EXCEPTION;
      reg.mcontrol6.match     = *napot ? MATCH_TYPE_TOPBITS :
                                         MATCH_TYPE_EQUAL;
    }
  else
    {
      reg.match.type    = TRIGGER_TYPE_MCONTROL;
      reg.match.load    = load;
      reg.match.store   = store;
      reg.match.execute = execute;
      reg.match.m       = 1;
      reg.match.u       = 1;
      reg.match.action  = ACTION_TYPE_EXCEPTION;
      reg.match.match   = *napot ? MATCH_TYPE_TOPBITS : MATCH_TYPE_EQUAL;
    }

  *tdata1 = reg.reg;
  *tdata2 = value;
}

/****************************************************************************
 * Name: riscv_debug_verify
 *
 * Description:
 *   Check that the hardware kept the match-control bits the driver depends
 *   on. tdata is WARL, so a watchpoint on a slot whose load/store bits are
 *   tied to zero would otherwise register successfully and never fire.
 *
 ****************************************************************************/

static bool riscv_debug_verify(int hwtype, uintptr_t want_tdata1,
                               uintptr_t got_tdata1, uintptr_t want_tdata2,
                               uintptr_t got_tdata2)
{
  uintptr_t mask;

  if (hwtype != TRIGGER_TYPE_MCONTROL &&
      hwtype != TRIGGER_TYPE_MCONTROL6)
    {
      return true;
    }

  /* hit is set by the hardware and timing may be forced by the
   * implementation, so neither is part of the check. The privilege bits are
   * omitted as well: a core without U/S mode legally ties them to zero.
   */

  mask = (uintptr_t)0x7 |          /* load, store, execute */
         (uintptr_t)0x1 << 6 |     /* m */
         (uintptr_t)0xf << 7 |     /* match */
         (uintptr_t)0x1 << 11 |    /* chain */
         (uintptr_t)0xf << 12;     /* action */

  if (hwtype == TRIGGER_TYPE_MCONTROL)
    {
      mask |= (uintptr_t)0x3 << 16;    /* sizelo */
      mask |= (uintptr_t)0x1 << 19;    /* select */
#ifdef CONFIG_ARCH_RV64
      mask |= (uintptr_t)0x3 << 21;    /* sizehi */
#endif
    }
  else
    {
      mask |= (uintptr_t)0x7 << 16;    /* size */
      mask |= (uintptr_t)0x1 << 21;    /* select */
    }

  return (want_tdata1 & mask) == (got_tdata1 & mask) &&
         want_tdata2 == got_tdata2;
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
  struct riscv_debug_s *dbg = riscv_debug_state();
  union tdata1 reg;
  uintptr_t tdata1 = 0;
  uintptr_t tdata2 = 0;
  bool napot = false;
  int hwtype;
  int slot;
  int ret;

  /* Initialize the trigger module if it is not initialized yet */

  if (dbg->initialized == false)
    {
      ret = riscv_debug_init(dbg);
      if (ret < 0)
        {
          return ret;
        }
    }

  switch (type)
    {
      case DEBUGPOINT_STEPPOINT:
        break;

      case DEBUGPOINT_BREAKPOINT:
      case DEBUGPOINT_WATCHPOINT_RO:
      case DEBUGPOINT_WATCHPOINT_WO:
      case DEBUGPOINT_WATCHPOINT_RW:
        if (dbg->hwtype == TRIGGER_TYPE_NONE)
          {
            return -ENOSYS;
          }

        ret = riscv_debug_check_size(type, size);
        if (ret < 0)
          {
            return ret;
          }

        break;

      default:
        return -EINVAL;
    }

  slot = riscv_debug_find_free_slot(dbg, type);
  if (slot < 0)
    {
      return slot;
    }

  if (type == DEBUGPOINT_STEPPOINT)
    {
      ret = riscv_debug_probe_icount(slot);
      if (ret < 0)
        {
          return ret;
        }

      reg.reg = 0;
      reg.icount.type   = TRIGGER_TYPE_ICOUNT;
      reg.icount.dmode  = DMODE_TYPE_BOTH;
      reg.icount.count  = 1;
      reg.icount.m      = 1;
      reg.icount.u      = 1;
      reg.icount.action = ACTION_TYPE_EXCEPTION;
      hwtype = TRIGGER_TYPE_ICOUNT;
      tdata1 = reg.reg;
    }
  else
    {
      hwtype = dbg->hwtype;
      riscv_debug_setup_match(hwtype, type, addr, size, dbg->napot_max,
                              &tdata1, &tdata2, &napot);
    }

  /* tdata1 must be disabled while tdata2 is written. Writing 0 is not
   * enough on implementations that drop it, so program a disabled trigger
   * of the target type first; that also routes the following tdata2 write
   * to the right trigger type on implementations that dispatch by the
   * current type.
   */

  WRITE_CSR(CSR_TSELECT, slot);
  riscv_debug_disable(slot);

  reg.reg = 0;
  if (hwtype == TRIGGER_TYPE_MCONTROL6)
    {
      reg.mcontrol6.type = TRIGGER_TYPE_MCONTROL6;
    }
  else if (hwtype == TRIGGER_TYPE_MCONTROL)
    {
      reg.match.type = TRIGGER_TYPE_MCONTROL;
    }
  else
    {
      reg.icount.type  = TRIGGER_TYPE_ICOUNT;
      reg.icount.count = 1;
    }

  WRITE_CSR(CSR_TDATA1, reg.reg);

  if (type != DEBUGPOINT_STEPPOINT)
    {
      WRITE_CSR(CSR_TDATA2, tdata2);
    }

  /* Publish the callback before arming so a trigger that fires right after
   * the final write is never dispatched to an empty slot.
   */

  dbg->map[slot].type     = type;
  dbg->map[slot].hwtype   = hwtype;
  dbg->map[slot].address  = addr;
  dbg->map[slot].size     = size;
  dbg->map[slot].napot    = napot;
  dbg->map[slot].callback = callback;
  dbg->map[slot].arg      = arg;

  WRITE_CSR(CSR_TDATA1, tdata1);

  /* tdata is WARL: verify that the trigger was actually accepted */

  reg.reg = READ_CSR(CSR_TDATA1);

  if (riscv_debug_tdata1_type(reg.reg) != hwtype)
    {
      riscv_debug_disable(slot);
      memset(&dbg->map[slot], 0, sizeof(dbg->map[slot]));
      return -ENOSYS;
    }

  if (type != DEBUGPOINT_STEPPOINT &&
      !riscv_debug_verify(hwtype, tdata1, reg.reg, tdata2,
                          READ_CSR(CSR_TDATA2)))
    {
      riscv_debug_disable(slot);
      memset(&dbg->map[slot], 0, sizeof(dbg->map[slot]));
      return -ENOSYS;
    }

  return OK;
}

/****************************************************************************
 * Name: up_debugpoint_remove
 ****************************************************************************/

int up_debugpoint_remove(int type, void *addr, size_t size)
{
  struct riscv_debug_s *dbg = riscv_debug_state();
  int slot;

  /* Find the existing debugpoint */

  slot = riscv_debug_find_slot(dbg, type, addr, size);
  if (slot < 0)
    {
      return slot;
    }

  riscv_debug_disable(slot);

  /* Clear the callback and arg */

  memset(&dbg->map[slot], 0, sizeof(dbg->map[slot]));

  return OK;
}

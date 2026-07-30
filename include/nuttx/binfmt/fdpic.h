/****************************************************************************
 * include/nuttx/binfmt/fdpic.h
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

#ifndef __INCLUDE_NUTTX_BINFMT_FDPIC_H
#define __INCLUDE_NUTTX_BINFMT_FDPIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <elf.h>
#include <stdint.h>

#include <nuttx/fs/fs.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A function descriptor is the pair that makes FDPIC work: a callee cannot
 * find its own data from its code address alone, because the two segments
 * are placed independently, so every function pointer carries the data base
 * to install alongside the entry point.
 */

#define FDPIC_DESC_WORDS   2

/* Shared libraries a single object may name in DT_NEEDED */

#define FDPIC_MAX_NEEDED   8

/* How deep a chain of libraries depending on libraries may go.
 *
 * A cycle cannot run away -- an object joins the load's list before its own
 * dependencies are walked, so coming back around finds it already there --
 * but a chain of distinct names has nothing to stop it.  The walk recurses
 * once per link with a path buffer on each frame, so without a bound a
 * malformed module set overflows the stack of whichever task called the
 * loader rather than being rejected.
 */

#define FDPIC_MAX_DEPTH    8

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct fdpic_desc_s
{
  uintptr_t entry;      /* Address of the code */
  uintptr_t got;        /* Data/GOT base to install in the FDPIC register */
};

/* One loaded object: the module itself, or a shared library it needs.
 *
 * A load produces a list of these -- the module first, then whatever its
 * DT_NEEDED entries pulled in.  Text is mapped and therefore shared between
 * every instance; the writable segment is private to this load, so two
 * tasks using the same library each get their own copy of its data, which
 * is the behaviour a library with state has to have.
 */

struct fdpic_loadinfo_s
{
  FAR struct fdpic_loadinfo_s *flink;   /* Next object in this load       */
  char name[32];                        /* For diagnostics and DT_NEEDED  */

  struct file file;                 /* The module being loaded            */
  Elf32_Ehdr  ehdr;                 /* Copy of the ELF header             */

  /* Read-only segment.  This is mapped, not copied: on a filesystem that
   * can expose the media directly this pointer is the flash address and
   * the text is never in RAM at all.
   */

  uintptr_t textaddr;               /* Where the RX segment actually is   */
  uintptr_t textvaddr;              /* p_vaddr it was linked at           */
  size_t    textsize;
  bool      textmapped;             /* True if it came from mmap          */

  /* Read-write segment.  Always a private RAM copy, one per instance. */

  uintptr_t dataaddr;               /* Where the RW segment was placed    */
  uintptr_t datavaddr;              /* p_vaddr it was linked at           */
  size_t    datafilesz;             /* Bytes to read from the file        */
  size_t    datamemsz;              /* Including .bss                     */
  size_t    dataalloc;              /* Total allocation incl. descriptors */

  uintptr_t gotaddr;                /* Runtime address of the GOT         */
  uintptr_t entry;                  /* Runtime entry point                */

  /* Reference counted container the scheduler installs into the FDPIC
   * register on every context switch.  Allocated here; freed by
   * sched_releasetcb() once the last thread using it is gone.
   */

  FAR struct dspace_s *dspace;

  /* Pool used to satisfy R_ARM_FUNCDESC, which asks the loader to
   * manufacture a descriptor and hand back its address.
   */

  uintptr_t descpool;
  uint16_t  ndesc;                  /* Capacity                           */
  uint16_t  usedesc;                /* Next free slot                     */

  /* Dynamic information, all as link-time virtual addresses */

  /* DT_NEEDED entries, as offsets into DT_STRTAB.  Resolved to names once
   * the read-only segment holding the string table is mapped.
   */

  uint32_t  needed[FDPIC_MAX_NEEDED];
  uint8_t   nneeded;

  uintptr_t relvaddr;
  size_t    relsize;

  /* The PLT relocation table.  Separate from DT_REL only by where the
   * linker chose to put an entry: without -z now the imported function
   * descriptors land here instead, and the loader binds both tables
   * eagerly rather than requiring one particular layout.
   */

  uintptr_t pltrelvaddr;
  size_t    pltrelsize;

  /* Constructors and destructors.  Both arrays live in the writable
   * segment and their entries arrive as link-time addresses carrying an
   * R_ARM_RELATIVE relocation, so they are real code addresses -- Thumb
   * bit included -- only after fdpic_bind() has run.
   */

  uintptr_t initvaddr;
  size_t    initsize;
  uintptr_t finivaddr;
  size_t    finisize;

  uintptr_t symtabvaddr;
  uintptr_t strtabvaddr;
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdpic_callback
 *
 * Description:
 *   Resolve a function pointer that arrived from a caller which may be an
 *   FDPIC module.
 *
 *   This exists because the two sides disagree about what a function
 *   pointer *is*.  Base firmware is not built FDPIC, so to it a function
 *   pointer is a code address and it simply branches to it.  An FDPIC
 *   module passes the address of a two-word descriptor instead.  A base
 *   firmware routine that takes a callback -- qsort() and bsearch() are the
 *   ones that matter -- therefore branches straight into the module's data
 *   segment and faults.
 *
 *   The caller is identified by the FDPIC register: a module task runs with
 *   its GOT there, and a plain kernel task runs with zero, because
 *   up_initial_state() only installs a value when the task has a D-Space.
 *
 *   Only the entry point is taken from the descriptor.  The data base is
 *   already correct in the register: the base firmware is built with that
 *   register reserved, so the module's GOT survives the call in.
 *
 * Input Parameters:
 *   fn - The pointer as it was received.
 *
 * Returned Value:
 *   An address that can be branched to directly.
 *
 ****************************************************************************/

#if defined(CONFIG_FDPIC) && defined(__thumb__)
static inline FAR void *fdpic_callback(FAR void *fn)
{
  uintptr_t base;

  __asm__ __volatile__ ("mov %0, r9" : "=r"(base));

  if (base != 0 && fn != NULL)
    {
      return (FAR void *)((FAR struct fdpic_desc_s *)fn)->entry;
    }

  return fn;
}

/****************************************************************************
 * Name: fdpic_base
 *
 * Description:
 *   The FDPIC data base of the calling context, taken from the FDPIC
 *   register.  Non-zero means the caller is an FDPIC module; zero means base
 *   firmware.  This is the same test fdpic_callback() makes, exposed for a
 *   caller that must decide whether a pointer it was handed is a descriptor
 *   before it stores it somewhere the register will no longer be correct --
 *   a callback that will run on a work-queue thread, for instance.
 *
 ****************************************************************************/

static inline uintptr_t fdpic_base(void)
{
  uintptr_t base;

  __asm__ __volatile__ ("mov %0, r9" : "=r"(base));

  return base;
}

/****************************************************************************
 * Name: fdpic_invoke
 *
 * Description:
 *   Call a resolved module entry point with the module's data base installed
 *   in the FDPIC register, and restore the caller's base afterwards.
 *
 *   This is for the one case the register cannot already be correct: a
 *   callback that a module registered but that runs on a shared thread --
 *   the signal-notification work queue -- which carries no module's base.
 *   The base is captured at registration (fdpic_base(), in the module's
 *   context) and installed here around the call.  Everywhere else the
 *   callback runs in a task that inherited the module's data space and only
 *   the entry point has to be resolved; there fdpic_callback() is enough.
 *
 *   A context switch or interrupt during the call is safe: the FDPIC
 *   register is REG_PIC in the saved register context, so it is preserved
 *   and restored across a switch, and base firmware is built with it
 *   reserved so no interrupt handler disturbs it.
 *
 * Input Parameters:
 *   entry - The code address to enter (already resolved from the
 *           descriptor).
 *   arg   - The single word argument, passed in r0.
 *   got   - The module data base to install in the FDPIC register.
 *
 ****************************************************************************/

static inline void fdpic_invoke(uintptr_t entry, uintptr_t arg,
                                uintptr_t got)
{
  register uintptr_t r0v __asm__ ("r0") = arg;

  /* arg is pinned in r0 (the first argument and the call's scratch), so the
   * asm needs registers only for entry and got -- kept deliberately few so
   * the allocator has room on builds that reserve a frame pointer.  r9 is
   * saved on the stack rather than in a scratch register; r4 rides along
   * only to keep the push 8-byte aligned and is restored untouched.
   */

  __asm__ __volatile__
  (
    "push {r4, r9}\n"     /* Save the caller's FDPIC register            */
    "mov r9, %[got]\n"    /* Install the module's data base             */
    "blx %[entry]\n"      /* Enter the module                           */
    "pop {r4, r9}\n"      /* Restore the caller's FDPIC register        */
    : "+r" (r0v)
    : [entry] "r" (entry), [got] "r" (got)
    : "r1", "r2", "r3", "r12", "lr", "cc", "memory"
  );
}
#else
#  define fdpic_callback(fn) (fn)
#  define fdpic_base()       (0)
#  define fdpic_invoke(entry, arg, got) \
          ((void)(got), (((CODE void (*)(uintptr_t))(uintptr_t)(entry))(arg)))
#endif

#endif /* __INCLUDE_NUTTX_BINFMT_FDPIC_H */

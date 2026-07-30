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
#else
#  define fdpic_callback(fn) (fn)
#endif

#endif /* __INCLUDE_NUTTX_BINFMT_FDPIC_H */

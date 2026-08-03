/****************************************************************************
 * include/nuttx/fdpic.h
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

#ifndef __INCLUDE_NUTTX_FDPIC_H
#define __INCLUDE_NUTTX_FDPIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A function descriptor: what a function pointer is under FDPIC.
 *
 * The base firmware is not built FDPIC, so to it a function pointer is a
 * code address and it simply branches there.  An FDPIC module passes the
 * address of one of these instead, because its code and data are placed
 * independently and a bare code address would leave the callee unable to
 * find its own data.
 */

struct fdpic_desc_s
{
  uintptr_t entry;      /* Address of the code */
  uintptr_t got;        /* Data base to install before branching */
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifdef CONFIG_FDPIC

/****************************************************************************
 * Name: fdpic_base
 *
 * Description:
 *   The data base of the calling context, read from the PIC base register.
 *   Non-zero means the caller is an FDPIC module; zero means base firmware,
 *   because up_initial_state() only installs a value for a task that has a
 *   D-Space.
 *
 *   This is what lets a shared entry point tell the two apart without being
 *   told, so that a plain kernel task calling qsort() is unaffected.
 *
 ****************************************************************************/

static inline uintptr_t fdpic_base(void)
{
  uintptr_t base;

  up_getpicbase(&base);
  return base;
}

/****************************************************************************
 * Name: fdpic_callback
 *
 * Description:
 *   Resolve a function pointer that arrived from a caller which may be an
 *   FDPIC module.
 *
 *   Only the entry point is taken from the descriptor.  The data base is
 *   already correct in the register: the base firmware is built with that
 *   register reserved, so the module's own base survives the call in, and
 *   any task the module creates inherits its D-Space.
 *
 * Input Parameters:
 *   fn - The pointer as it was received.
 *
 * Returned Value:
 *   An address that can be branched to directly.
 *
 ****************************************************************************/

static inline FAR void *fdpic_callback(FAR void *fn)
{
  if (fn != NULL && fdpic_base() != 0)
    {
      return (FAR void *)((FAR struct fdpic_desc_s *)fn)->entry;
    }

  return fn;
}

/****************************************************************************
 * Name: fdpic_invoke
 *
 * Description:
 *   Call a resolved module entry point with the module data base in the PIC
 *   base register.  Put the caller data base back after the call.
 *
 *   This is for the one case where the register cannot be correct already: a
 *   callback that a module registers and that runs on a shared thread -- the
 *   signal notification work queue -- which carries no module base.  The
 *   loader records the base at registration time, in the module context.
 *   Everywhere else the callback runs on a task that inherited the module
 *   D-Space, and fdpic_callback() is sufficient.
 *
 *   The register sequence is architecture specific, thus up_fdpic_invoke()
 *   supplies it from <arch/arch.h>.
 *
 * Input Parameters:
 *   entry - The code address to enter, already resolved from the descriptor.
 *   arg   - The one word argument.
 *   got   - The module data base to install.
 *
 ****************************************************************************/

static inline void fdpic_invoke(uintptr_t entry, uintptr_t arg,
                                uintptr_t got)
{
  up_fdpic_invoke(entry, arg, got);
}

#else

#  define fdpic_base()       (0)
#  define fdpic_callback(fn) (fn)
#  define fdpic_invoke(entry, arg, got) \
          ((void)(got), (((CODE void (*)(uintptr_t))(uintptr_t)(entry))(arg)))

#endif /* CONFIG_FDPIC */

#endif /* __INCLUDE_NUTTX_FDPIC_H */

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

/* A function descriptor: what a function pointer is under FDPIC.  The
 * firmware branches to a code address; a module passes one of these.
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
 *   The data base of the calling context, from the PIC base register.
 *   Non-zero means the caller is an FDPIC module, zero means firmware.
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
 *   Resolve a function pointer from a caller that may be an FDPIC module.
 *   Only the entry point is taken: the data base is already in the register.
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
 * Name: fdpic_init
 *
 * Description:
 *   Record a callback for a later call from a thread that carries no module
 *   base.  Runs in the caller's own context, which is the only place the
 *   answer can be had.
 *
 *   A module's function pointer is the address of a descriptor, and the base
 *   comes from there: a module can hand over a callback that belongs to
 *   another one.  A firmware pointer is a code address and has no base.
 *
 * Input Parameters:
 *   desc - The descriptor to fill.
 *   fn   - The callback, as the caller received it.
 *
 ****************************************************************************/

static inline void fdpic_init(FAR struct fdpic_desc_s *desc,
                              FAR void *fn)
{
  if (fn != NULL && fdpic_base() != 0)
    {
      *desc = *(FAR struct fdpic_desc_s *)fn;
    }
  else
    {
      desc->entry = (uintptr_t)fn;
      desc->got   = 0;
    }
}

/****************************************************************************
 * Name: fdpic_invoke
 *
 * Description:
 *   Enter a callback recorded by fdpic_init(), with the data base it
 *   carries in the PIC base register.  For a callback that runs on a shared
 *   thread, which has no base of its own.  Elsewhere fdpic_callback() is
 *   enough.
 *
 *   A zero base means the callback is not a module's, and it is branched to
 *   directly.
 *
 * Input Parameters:
 *   arg  - The one word argument.
 *   desc - The recorded callback.
 *
 ****************************************************************************/

static inline void fdpic_invoke(uintptr_t arg,
                                FAR const struct fdpic_desc_s *desc)
{
  if (desc->got != 0)
    {
      up_fdpic_invoke(arg, desc->entry, desc->got);
    }
  else
    {
      ((CODE void (*)(uintptr_t))desc->entry)(arg);
    }
}

#else

#  define fdpic_base()       (0)
#  define fdpic_callback(fn) (fn)
#  define fdpic_init(desc, fn) \
          ((desc)->entry = (uintptr_t)(fn), (desc)->got = 0)
#  define fdpic_invoke(arg, desc) \
          (((CODE void (*)(uintptr_t))(desc)->entry)(arg))

#endif /* CONFIG_FDPIC */

#endif /* __INCLUDE_NUTTX_FDPIC_H */

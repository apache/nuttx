/****************************************************************************
 * binfmt/binfmt_dumpmodule.c
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sched.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/binfmt/binfmt.h>

#include "binfmt.h"

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_BINFMT) && !defined(CONFIG_BINFMT_DISABLE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dump_module
 *
 * Description:
 *   Load a module into memory and prep it for execution.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int dump_module(FAR const struct binary_s *bin)
{
  if (bin)
    {
      binfo("Module:\n");
      binfo("  filename:  %s\n", bin->filename);
      binfo("  argv:      %p\n", bin->argv);
      binfo("  entrypt:   %p\n", bin->entrypt);
      binfo("  mapped:    %p size=%d\n", bin->mapped, bin->mapsize);
      binfo("  alloc:     %p %p %p\n", bin->alloc[0],
                                       bin->alloc[1],
                                       bin->alloc[2]);
#ifdef CONFIG_BINFMT_CONSTRUCTORS
      binfo("  ctors:     %p nctors=%d\n", bin->ctors, bin->nctors);
      binfo("  dtors:     %p ndtors=%d\n", bin->dtors, bin->ndtors);
#endif
#ifdef CONFIG_ARCH_ADDRENV
      binfo("  addrenv:   %p\n", bin->addrenv);
#endif
      binfo("  stacksize: %d\n", bin->stacksize);
      binfo("  unload:    %p\n", bin->unload);
    }

  return OK;
}
#endif

/****************************************************************************
 * binfmt/binfmt_unloadmodule.c
 *
 *   Copyright (C) 2009, 2012-2013, 2017 Gregory Nutt. All rights reserved.
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

#include <sys/mman.h>
#include <stdlib.h>
#include <sched.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/binfmt/binfmt.h>

#include "binfmt.h"

#ifndef CONFIG_BINFMT_DISABLE

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exec_dtors
 *
 * Description:
 *   Execute C++ static destructors.
 *
 * Input Parameters:
 *   binp - Load state information
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BINFMT_CONSTRUCTORS
static inline int exec_dtors(FAR struct binary_s *binp)
{
  binfmt_dtor_t *dtor = binp->dtors;
#ifdef CONFIG_ARCH_ADDRENV
  save_addrenv_t oldenv;
  int ret;
#endif
  int i;

  /* Instantiate the address environment containing the destructors */

#ifdef CONFIG_ARCH_ADDRENV
  ret = up_addrenv_select(&binp->addrenv, &oldenv);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_select() failed: %d\n", ret);
      return ret;
    }
#endif

  /* Execute each destructor */

  for (i = 0; i < binp->ndtors; i++)
    {
      binfo("Calling dtor %d at %p\n", i, (FAR void *)dtor);

      (*dtor)();
      dtor++;
    }

  /* Restore the address environment */

#ifdef CONFIG_ARCH_ADDRENV
  return up_addrenv_restore(&oldenv);
#else
  return OK;
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: unload_module
 *
 * Description:
 *   Unload a (non-executing) module from memory.  If the module has
 *   been started (via exec_module) and has not exited, calling this will
 *   be fatal.
 *
 *   However, this function must be called after the module exits.  How
 *   this is done is up to your logic.  Perhaps you register it to be
 *   called by on_exit()?
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int unload_module(FAR struct binary_s *binp)
{
  int ret;
  int i;

  if (binp)
    {
      /* Perform any format-specific unload operations */

      if (binp->unload)
        {
          ret = binp->unload(binp);
          if (ret < 0)
            {
              berr("binp->unload() failed: %d\n", ret);
              return ret;
            }
        }

#ifdef CONFIG_BINFMT_CONSTRUCTORS
      /* Execute C++ destructors */

      ret = exec_dtors(binp);
      if (ret < 0)
        {
          berr("exec_ctors() failed: %d\n", ret);
          return ret;
        }
#endif

      /* Free any allocated argv[] strings */

      binfmt_freeargv(binp);

      /* Unmap mapped address spaces */

      if (binp->mapped)
        {
          binfo("Unmapping address space: %p\n", binp->mapped);

          munmap(binp->mapped, binp->mapsize);
        }

      /* Free allocated address spaces */

      for (i = 0; i < BINFMT_NALLOC; i++)
        {
          if (binp->alloc[i])
            {
              binfo("Freeing alloc[%d]: %p\n", i, binp->alloc[i]);
              kumm_free((FAR void *)binp->alloc[i]);
            }
        }

      /* Notice that the address environment is not destroyed.  This should
       * happen automatically when the task exits.
       */
    }

  return OK;
}

#endif /* CONFIG_BINFMT_DISABLE */

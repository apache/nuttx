/****************************************************************************
 * binfmt/binfmt_unloadmodule.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mman.h>
#include <stdlib.h>
#include <sched.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/addrenv.h>
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
  int ret;
#endif
  int i;

  /* Instantiate the address environment containing the destructors */

#ifdef CONFIG_ARCH_ADDRENV
  ret = addrenv_select(&binp->addrenv);
  if (ret < 0)
    {
      berr("ERROR: addrenv_select() failed: %d\n", ret);
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
  return addrenv_restore();
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

      /* Unmap mapped address spaces */

      if (binp->mapped)
        {
          binfo("Unmapping address space: %p\n", binp->mapped);

          file_munmap(binp->mapped, binp->mapsize);
        }

      /* Free allocated address spaces */

      for (i = 0; i < BINFMT_NALLOC; i++)
        {
          if (binp->alloc[i])
            {
              binfo("Freeing alloc[%d]: %p\n", i, binp->alloc[i]);
#if defined(CONFIG_ARCH_USE_TEXT_HEAP)
              if (i == 0)
                {
                  up_textheap_free((FAR void *)binp->alloc[i]);
                }
              else
#endif
                {
                  kumm_free((FAR void *)binp->alloc[i]);
                }
            }
        }

      /* Notice that the address environment is not destroyed.  This should
       * happen automatically when the task exits.
       */
    }

  return OK;
}

#endif /* CONFIG_BINFMT_DISABLE */

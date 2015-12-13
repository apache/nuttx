/****************************************************************************
 * sched/module/module.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/module.h>

#include "module/module.h"

#ifdef CONFIG_MODULE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rmmod
 *
 * Description:
 *   Remove a previously installed module from memory.
 *
 * Input Parameters:
 *
 *   modulename - The module name.  This is the name module name that was
 *     provided to insmod when the module was loaded.
 *
 * Returned Value:
 *   Zero (OK) on success.  On any failure, -1 (ERROR) is returned the
 *   errno value is set appropriately.
 *
 ****************************************************************************/

int rmmod(FAR const char *modulename)
{
  FAR struct module_s *modp;
  int ret = OK;

  DEBUGASSERT(modulename != NULL);

  /* Get exclusive access to the module registry */

  mod_registry_lock();

  /* Find the module entry for this modulename in the registry */

  modp = mod_registry_find(modulename);
  if (modp == NULL)
    {
      sdbg("ERROR: Failed to find module %s: %d\n", modulename, ret);
      ret = -ENOENT;
      goto errout_with_lock;
    }

  /* Is there an uninitializer? */

  if (modp->uninitializer != NULL)
    {
      /* Try to uninitializer the module */

      ret = modp->uninitializer(modp->arg);

      /* Did the module sucessfully uninitialize? */

      if (ret < 0)
        {
          sdbg("ERROR: Failed to uninitialize the module: %d\n", ret);
          goto errout_with_lock;
        }

      /* Nullify so that the uninitializer cannot be called again */

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
      modp->initializer = NULL;
#endif
      modp->uninitializer = NULL;
      modp->arg = NULL;
    }

  /* Release resources held by the module */

  if (modp->alloc != NULL)
    {
      /* Free the module memory */

      kmm_free((FAR void *)modp->alloc);

      /* Nullify so that the memory cannot be freed again */

      modp->alloc = NULL;
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
      modp->textsize  = 0;
      modp->datasize  = 0;
#endif
    }

  /* Remove the module from the registry */

  ret = mod_registry_del(modp);
  if (ret < 0)
    {
      sdbg("ERROR: Failed to remove the module from the registry: %d\n", ret);
      goto errout_with_lock;
    }

  mod_registry_unlock();

  /* And free the registry entry */

  kmm_free(modp);
  return OK;

errout_with_lock:
  mod_registry_unlock();
  set_errno(-ret);
  return ERROR;
}

#endif /* CONFIG_MODULE */

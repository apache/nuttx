/****************************************************************************
 * sched/module/mod_rmmod.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
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
#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/module.h>
#include <nuttx/lib/modlib.h>

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
 *   handle - The module handler previously returned by insmod().
 *
 * Returned Value:
 *   Zero (OK) on success.  On any failure, -1 (ERROR) is returned the
 *   errno value is set appropriately.
 *
 ****************************************************************************/

int rmmod(FAR void *handle)
{
  FAR struct module_s *modp = (FAR struct module_s *)handle;
  int ret;

  DEBUGASSERT(modp != NULL);

  /* Get exclusive access to the module registry */

  modlib_registry_lock();

  /* Verify that the module is in the registry */

  ret = modlib_registry_verify(modp);
  if (ret < 0)
    {
      berr("ERROR: Failed to verify module: %d\n", ret);
      goto errout_with_lock;
    }

#if CONFIG_MODLIB_MAXDEPEND > 0
  /* Refuse to remove any module that other modules may depend upon. */

  if (modp->dependents > 0)
    {
      berr("ERROR: Module has dependents: %d\n", modp->dependents);
      ret = -EBUSY;
      goto errout_with_lock;
    }
#endif

  /* Is there an uninitializer? */

  if (modp->modinfo.uninitializer != NULL)
    {
      /* Try to uninitialize the module */

      ret = modp->modinfo.uninitializer(modp->modinfo.arg);

      /* Did the module successfully uninitialize? */

      if (ret < 0)
        {
          berr("ERROR: Failed to uninitialize the module: %d\n", ret);
          goto errout_with_lock;
        }

      /* Nullify so that the uninitializer cannot be called again */

      modp->modinfo.uninitializer = NULL;
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
      modp->initializer           = NULL;
      modp->modinfo.arg           = NULL;
      modp->modinfo.exports       = NULL;
      modp->modinfo.nexports      = 0;
#endif
    }

  /* Release resources held by the module */

#if defined(CONFIG_ARCH_USE_MODULE_TEXT)
  if (modp->textalloc != NULL || modp->dataalloc != NULL)
#else
  if (modp->alloc != NULL)
#endif
    {
      /* Free the module memory
       * and nullify so that the memory cannot be freed again
       */

#if defined(CONFIG_ARCH_USE_MODULE_TEXT)
      up_module_text_free((FAR void *)modp->textalloc);
      kmm_free((FAR void *)modp->dataalloc);
      modp->textalloc = NULL;
      modp->dataalloc = NULL;
#else
      kmm_free((FAR void *)modp->alloc);
      modp->alloc = NULL;
#endif
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
      modp->textsize  = 0;
      modp->datasize  = 0;
#endif
    }

  /* Remove the module from the registry */

  ret = modlib_registry_del(modp);
  if (ret < 0)
    {
      berr("ERROR: Failed to remove the module from the registry: %d\n",
           ret);
      goto errout_with_lock;
    }

#if CONFIG_MODLIB_MAXDEPEND > 0
  /* Eliminate any dependencies that this module has on other modules */

  modlib_undepend(modp);
#endif
  modlib_registry_unlock();

  /* And free the registry entry */

  kmm_free(modp);
  return OK;

errout_with_lock:
  modlib_registry_unlock();
  set_errno(-ret);
  return ERROR;
}

#endif /* CONFIG_MODULE */

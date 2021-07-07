/****************************************************************************
 * sched/module/mod_rmmod.c
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

#include <sys/types.h>
#include <assert.h>
#include <debug.h>
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

  if (modp->textalloc != NULL || modp->dataalloc != NULL)
    {
      /* Free the module memory
       * and nullify so that the memory cannot be freed again
       */

#if defined(CONFIG_ARCH_USE_TEXT_HEAP)
      up_textheap_free((FAR void *)modp->textalloc);
#else
      kmm_free((FAR void *)modp->textalloc);
#endif
      kmm_free((FAR void *)modp->dataalloc);
      modp->textalloc = NULL;
      modp->dataalloc = NULL;
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

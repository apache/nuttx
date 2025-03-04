/****************************************************************************
 * libs/libc/modlib/modlib_remove.c
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
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/lib/lib.h>
#include <nuttx/lib/modlib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_uninit
 *
 * Description:
 *   Uninitialize module resources.
 *
 ****************************************************************************/

int modlib_uninit(FAR struct module_s *modp)
{
  FAR void (**array)(void);
  int ret = OK;
  int i;

#if CONFIG_MODLIB_MAXDEPEND > 0
  /* Refuse to remove any module that other modules may depend upon. */

  if (modp->dependents > 0)
    {
      berr("ERROR: Module has dependents: %d\n", modp->dependents);
      return -EBUSY;
    }
#endif

  /* Is there an uninitializer? */

  array = (FAR void (**)(void))modp->finiarr;
  for (i = 0; i < modp->nfini; i++)
    {
      array[i]();
    }

  if (modp->modinfo.uninitializer != NULL)
    {
      /* Try to uninitialize the module */

      ret = modp->modinfo.uninitializer(modp->modinfo.arg);

      /* Did the module successfully uninitialize? */

      if (ret < 0)
        {
          berr("ERROR: Failed to uninitialize the module: %d\n", ret);
          return ret;
        }

      modlib_freesymtab(modp);

      /* Nullify so that the uninitializer cannot be called again */

      modp->modinfo.uninitializer = NULL;
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
      modp->modinfo.arg           = NULL;
      modp->modinfo.exports       = NULL;
      modp->modinfo.nexports      = 0;
#endif
    }

  /* Release resources held by the module */

  if (modp->textalloc != NULL || modp->dataalloc != NULL)
    {
      /* Free the module memory and nullify so that the memory cannot
       * be freed again
       *
       * NOTE: For dynamic shared objects there is only a single
       * allocation: the text/data were allocated in one operation
       */

      if (!modp->dynamic)
        {
#ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
          for (i = 0; i < modp->nsect && modp->sectalloc[i] != NULL; i++)
            {
#  ifdef CONFIG_ARCH_USE_TEXT_HEAP
              if (up_textheap_heapmember(modp->sectalloc[i]))
                {
                  up_textheap_free(modp->sectalloc[i]);
                  continue;
                }
#  endif

#  ifdef CONFIG_ARCH_USE_DATA_HEAP
              if (up_dataheap_heapmember(modp->sectalloc[i]))
                {
                  up_dataheap_free(modp->sectalloc[i]);
                  continue;
                }
#  endif

              lib_free(modp->sectalloc[i]);
            }

          lib_free(modp->sectalloc);
          modp->sectalloc = NULL;
          modp->nsect = 0;
#else
          if (modp->xipbase == 0)
            {
#  if defined(CONFIG_ARCH_USE_TEXT_HEAP)
              up_textheap_free((FAR void *)modp->textalloc);
#  else
              lib_free((FAR void *)modp->textalloc);
#  endif
            }

#  if defined(CONFIG_ARCH_USE_DATA_HEAP)
          up_dataheap_free((FAR void *)modp->dataalloc);
#  else
          lib_free((FAR void *)modp->dataalloc);
#  endif
#endif
        }
      else
        {
          lib_free((FAR void *)modp->textalloc);
        }

      modp->textalloc = NULL;
      modp->dataalloc = NULL;
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
      modp->textsize  = 0;
      modp->datasize  = 0;
#endif
    }

#if CONFIG_MODLIB_MAXDEPEND > 0
  /* Eliminate any dependencies that this module has on other modules */

  modlib_undepend(modp);
#endif

  return ret;
}

/****************************************************************************
 * Name: modlib_remove
 *
 * Description:
 *   Remove a previously installed module from memory.
 *
 * Input Parameters:
 *   handle - The module handler previously returned by modlib_insert().
 *
 * Returned Value:
 *   Zero (OK) on success.  On any failure, -1 (ERROR) is returned the
 *   errno value is set appropriately.
 *
 ****************************************************************************/

int modlib_remove(FAR void *handle)
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

  ret = modlib_uninit(modp);
  if (ret < 0)
    {
      berr("ERROR: Failed to uninitialize module %d\n", ret);
      goto errout_with_lock;
    }

  /* Remove the module from the registry */

  ret = modlib_registry_del(modp);
  if (ret < 0)
    {
      berr("ERROR: Failed to remove the module from the registry: %d\n",
           ret);
      goto errout_with_lock;
    }

  modlib_registry_unlock();

  /* And free the registry entry */

  return ret;

errout_with_lock:
  modlib_registry_unlock();
  set_errno(-ret);
  return ERROR;
}

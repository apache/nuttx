/****************************************************************************
 * libs/libc/dlfcn/lib_dlclose.c
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

#include <dlfcn.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/module.h>
#include <nuttx/lib/modlib.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dlremove
 *
 * Description:
 *   Remove a previously installed shared library from memory.
 *
 * Input Parameters:
 *   handle - The shared library handle previously returned by dlopen().
 *
 * Returned Value:
 *   Zero (OK) on success.  On any failure, -1 (ERROR) is returned the
 *   errno value is set appropriately.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
static inline int dlremove(FAR void *handle)
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
      serr("ERROR: Failed to verify module: %d\n", ret);
      goto errout_with_lock;
    }

#if CONFIG_MODLIB_MAXDEPEND > 0
  /* Refuse to remove any module that other modules may depend upon. */

  if (modp->dependents > 0)
    {
      serr("ERROR: Module has dependents: %d\n", modp->dependents);
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
          serr("ERROR: Failed to uninitialize the module: %d\n", ret);
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

  if (modp->textalloc != NULL)
    {
      /* Free the module memory */

      lib_free((FAR void *)modp->textalloc);

      /* Nullify so that the memory cannot be freed again */

      modp->textalloc = NULL;
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
      modp->textsize  = 0;
#endif
    }

  if (modp->dataalloc != NULL)
    {
      /* Free the module memory */

      lib_free((FAR void *)modp->dataalloc);

      /* Nullify so that the memory cannot be freed again */

      modp->dataalloc = NULL;
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
      modp->datasize  = 0;
#endif
    }

  /* Remove the module from the registry */

  ret = modlib_registry_del(modp);
  if (ret < 0)
    {
      serr("ERROR: Failed to remove the module from the registry: %d\n",
           ret);
      goto errout_with_lock;
    }

#if CONFIG_MODLIB_MAXDEPEND > 0
  /* Eliminate any dependencies that this module has on other modules */

  modlib_undepend(modp);
#endif
  modlib_registry_unlock();

  /* And free the registry entry */

  lib_free(modp);
  return OK;

errout_with_lock:
  modlib_registry_unlock();
  set_errno(-ret);
  return ERROR;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dlclose
 *
 * Description:
 *   dlclose() is used to inform the system that the object referenced by a
 *   handle returned from a previous dlopen() invocation is no longer needed
 *   by the application.
 *
 *   The use of dlclose() reflects a statement of intent on the part of the
 *   process, but does not create any requirement upon the implementation,
 *   such as removal of the code or symbols referenced by handle. Once an
 *   object has been closed using dlclose() an application should assume
 *   that its symbols are no longer available to dlsym(). All objects loaded
 *   automatically as a result of invoking dlopen() on the referenced object
 *   are also closed.
 *
 *   Although a dlclose() operation is not required to remove structures
 *   from an address space, neither is an implementation prohibited from
 *   doing so. The only restriction on such a removal is that no object will
 *   be removed to which references have been relocated, until or unless all
 *   such references are removed. For instance, an object that had been
 *   loaded with a dlopen() operation specifying the RTLD_GLOBAL flag might
 *   provide a target for dynamic relocations performed in the processing of
 *   other objects - in such environments, an application may assume that no
 *   relocation, once made, will be undone or remade unless the object
 *   requiring the relocation has itself been removed.
 *
 * Input Parameters:
 *   handle - The opaque, non-NULL value returned by a previous successful
 *            call to dlopen().
 *
 * Returned Value:
 *   If the referenced object was successfully closed, dlclose() returns 0.
 *   If the object could not be closed, or if handle does not refer to an
 *   open object, dlclose() returns a non-zero value. More detailed
 *   diagnostic information will be available through dlerror().
 *
 * Reference: OpenGroup.org
 *
 ****************************************************************************/

int dlclose(FAR void *handle)
{
#if defined(CONFIG_BUILD_FLAT)
  /* In the FLAT build, a shared library is essentially the same as a kernel
   * module.
   */

  return rmmod(handle);

#elif defined(CONFIG_BUILD_PROTECTED)
  /* The PROTECTED build is equivalent to the FLAT build EXCEPT that there
   * must be two copies of the module logic:  One residing in kernel
   * space and using the kernel symbol table and one residing in user space
   * using the user space symbol table.
   *
   * dlremove() is essentially a clone of rmmod().
   */

  return dlremove(handle);

#else /* if defined(CONFIG_BUILD_KERNEL) */
  /* The KERNEL build is considerably more complex:  In order to be shared,
   * the .text portion of the module must be (1) build for PIC/PID operation
   * and (2) must like in a shared memory region accessible from all
   * processes.  The .data/.bss portion of the module must be allocated in
   * the user space of each process, but must lie at the same virtual address
   * so that it can be referenced from the one copy of the text in the shared
   * memory region.
   */

  /* #warning Missing logic */

  return -ENOSYS;
#endif
}

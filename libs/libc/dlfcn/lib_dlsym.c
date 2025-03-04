/****************************************************************************
 * libs/libc/dlfcn/lib_dlsym.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <dlfcn.h>
#include <errno.h>

#include <nuttx/lib/modlib.h>
#include <nuttx/symtab.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dlsym
 *
 * Description:
 *   dlsym() allows a process to obtain the address of a symbol defined
 *   within an object made accessible through a dlopen() call. handle is the
 *   value returned from a call to dlopen() (and which has not since been
 *   released via a call to dlclose()), name is the symbol's name as a
 *   character string.
 *
 *   dlsym() will search for the named symbol in all objects loaded
 *   automatically as a result of loading the object referenced by handle
 *   (see dlopen()). Load ordering is used in dlsym() operations upon the
 *   global symbol object. The symbol resolution algorithm used will be
 *   dependency order as described in dlopen().
 *
 * Input Parameters:
 *   handle - The opaque, non-NULL value returned by a previous successful
 *            call to dlopen().
 *   name   - A pointer to the symbol name string.
 *
 * Returned Value:
 *   If handle does not refer to a valid object opened by dlopen(), or if
 *   the named symbol cannot be found within any of the objects associated
 *   with handle, dlsym() will return NULL. More detailed diagnostic
 *   information will be available through dlerror().
 *
 * Reference: OpenGroup.org
 *
 ****************************************************************************/

FAR void *dlsym(FAR void *handle, FAR const char *name)
{
#if defined(CONFIG_BUILD_FLAT) || defined(CONFIG_BUILD_PROTECTED)
  /* In the FLAT build, a shared library is essentially the same as a kernel
   * module.
   *
   * The PROTECTED build is equivalent to the FLAT build EXCEPT that there
   * must be two copies of the module logic:  One residing in kernel
   * space and using the kernel symbol table and one residing in user space
   * using the user space symbol table.
   *
   * dlgetsem() is essentially a clone of modsym().
   */

  return (FAR void *)modlib_getsymbol(handle, name);

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

  return NULL;
#endif
}

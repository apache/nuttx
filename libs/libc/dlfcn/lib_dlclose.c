/****************************************************************************
 * libs/libc/dlfcn/lib_dlclose.c
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

#include <dlfcn.h>

#include <nuttx/lib/modlib.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
#if defined(CONFIG_BUILD_FLAT) || defined(CONFIG_BUILD_PROTECTED)
  /* In the FLAT build, a shared library is essentially the same as a kernel
   * module.
   *
   * The PROTECTED build is equivalent to the FLAT build EXCEPT that there
   * must be two copies of the module logic:  One residing in kernel
   * space and using the kernel symbol table and one residing in user space
   * using the user space symbol table.
   *
   * dlremove() is essentially a clone of rmmod().
   */

  return modlib_remove(handle);

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

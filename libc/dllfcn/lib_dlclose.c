/****************************************************************************
 * libc/dllfcn/lib_dlclose.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <dllfcn.h>

#include <nuttx/module.h>

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
   * The brute force way to accomplish this is by just copying the kernel
   * module code into libc/module.
   */

#warning Missing logic
  return -ENOSYS;

#else /* if defined(CONFIG_BUILD_KERNEL) */
  /* The KERNEL build is considerably more complex:  In order to be shared,
   * the .text portion of the module must be (1) build for PIC/PID operation
   * and (2) must like in a shared memory region accessible from all
   * processes.  The .data/.bss portion of the module must be allocated in
   * the user space of each process, but must lie at the same virtual address
   * so that it can be referenced from the one copy of the text in the shared
   * memory region.
   */

#warning Missing logic
  return -ENOSYS;
#endif
}

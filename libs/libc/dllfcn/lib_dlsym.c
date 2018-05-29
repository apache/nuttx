/****************************************************************************
 * libs/libc/dllfcn/lib_dlsym.c
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
#include <errno.h>

#include <nuttx/module.h>
#include <nuttx/symtab.h>
#include <nuttx/lib/modlib.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dlgetsym
 *
 * Description:
 *   dlgetsym() implements dlsym() for the PROTECTED build.
 *
 * Input Parameters:
 *   handle - The opaque, non-NULL value returned by a previous successful
 *            call to insmod().
 *   name   - A pointer to the symbol name string.
 *
 * Returned Value:
 *   See dlsym().
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
static inline FAR const void *dlgetsym(FAR void *handle,
                                       FAR const char *name)
{
  FAR struct module_s *modp = (FAR struct module_s *)handle;
  FAR const struct symtab_s *symbol;
  int err;
  int ret;

  /* Verify that the module is in the registry */

  modlib_registry_lock();
  ret = modlib_registry_verify(modp);
  if (ret < 0)
    {
      serr("ERROR: Failed to verify module: %d\n", ret);
      err = -ret;
      goto errout_with_lock;
    }

  /* Does the module have a symbol table? */

  if (modp->modinfo.exports == NULL || modp->modinfo.nexports == 0)
    {
      serr("ERROR: Module has no symbol table\n");
      err = ENOENT;
      goto errout_with_lock;
    }

  /* Search the symbol table for the matching symbol */

  symbol = symtab_findbyname(modp->modinfo.exports, name,
                             modp->modinfo.nexports);
  if (symbol == NULL)
    {
      serr("ERROR: Failed to find symbol in symbol \"$s\" in table\n", name);
      err = ENOENT;
      goto errout_with_lock;
    }

  /* Return the address within the module assoicated with the symbol */

  modlib_registry_unlock();
  DEBUGASSERT(symbol->sym_value != NULL);
  return symbol->sym_value;

errout_with_lock:
  modlib_registry_unlock();
  set_errno(err);
  return NULL;
}
#endif

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
#if defined(CONFIG_BUILD_FLAT)
  /* In the FLAT build, a shared library is essentially the same as a kernel
   * module.
   */

  return (FAR void *)modsym(handle, name);

#elif defined(CONFIG_BUILD_PROTECTED)
  /* The PROTECTED build is equivalent to the FLAT build EXCEPT that there
   * must be two copies of the module logic:  One residing in kernel
   * space and using the kernel symbol table and one residing in user space
   * using the user space symbol table.
   *
   * dlgetsem() is essentially a clone of modsym().
   */

  return (FAR void *)dlgetsym(handle, name);

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
  return NULL;
#endif
}

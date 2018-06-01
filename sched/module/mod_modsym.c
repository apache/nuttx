/****************************************************************************
 * sched/module/mod_modsym.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/module.h>
#include <nuttx/symtab.h>
#include <nuttx/lib/modlib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modsym
 *
 * Description:
 *   modsym() returns the address of a symbol defined within the object that
 *   was previously made accessible through a insmod() call.  handle is the
 *   value returned from a call to insmod() (and which has not since been
 *   released via a call to rmmod()), name is the symbol's name as a
 *   character string.
 *
 *   The returned symbol address will remain valid until rmmod() is called.
 *
 * Input Parameters:
 *   handle - The opaque, non-NULL value returned by a previous successful
 *            call to insmod().
 *   name   - A pointer to the symbol name string.
 *
 * Returned Value:
 *   The address associated with the symbol is returned on success.
 *   If handle does not refer to a valid module opened by insmod(), or if
 *   the named symbol cannot be found within any of the objects associated
 *   with handle, modsym() will return NULL and the errno variable will be
 *   set appropriately.
 *
 *   NOTE: This means that the address zero can never be a valid return
 *   value.
 *
 ****************************************************************************/

FAR const void *modsym(FAR void *handle, FAR const char *name)
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
      berr("ERROR: Failed to verify module: %d\n", ret);
      err = -ret;
      goto errout_with_lock;
    }

  /* Does the module have a symbol table? */

  if (modp->modinfo.exports == NULL || modp->modinfo.nexports == 0)
    {
      berr("ERROR: Module has no symbol table\n");
      err = ENOENT;
      goto errout_with_lock;
    }

  /* Search the symbol table for the matching symbol */

  symbol = symtab_findbyname(modp->modinfo.exports, name,
                             modp->modinfo.nexports);
  if (symbol == NULL)
    {
      berr("ERROR: Failed to find symbol in symbol \"$s\" in table\n", name);
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

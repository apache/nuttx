/****************************************************************************
 * libs/libc/modlib/modlib_getsymbol.c
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

#include <nuttx/lib/modlib.h>
#include <nuttx/symtab.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_getsymbol
 *
 * Description:
 *   modlib_getsymbol() returns the address of a symbol defined within the
 *   object that was previously made accessible through a modlib_getsymbol()
 *   call.  handle is the value returned from a call to modlib_insert() (and
 *   which has not since been released via a call to modlib_remove()),
 *   name is the symbol's name as a character string.
 *
 *   The returned symbol address will remain valid until modlib_remove() is
 *   called.
 *
 * Input Parameters:
 *   handle - The opaque, non-NULL value returned by a previous successful
 *            call to modlib_insert().
 *   name   - A pointer to the symbol name string.
 *
 * Returned Value:
 *   The address associated with the symbol is returned on success.
 *   If handle does not refer to a valid module opened by modlib_insert(),
 *   or if the named modlib_symbol cannot be found within any of the objects
 *   associated with handle, modlib_getsymbol() will return NULL and the
 *   errno variable will be set appropriately.
 *
 *   NOTE: This means that the address zero can never be a valid return
 *   value.
 *
 ****************************************************************************/

FAR const void *modlib_getsymbol(FAR void *handle, FAR const char *name)
{
  FAR struct module_s *modp = handle;
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

  modlib_registry_unlock();
  if (symbol == NULL)
    {
      berr("ERROR: Failed to find symbol in symbol \"%s\" in table\n", name);
      set_errno(ENOENT);
      return NULL;
    }

  /* Return the address within the module associated with the symbol */

  DEBUGASSERT(symbol->sym_value != NULL);
  return symbol->sym_value;

errout_with_lock:
  modlib_registry_unlock();
  set_errno(err);
  return NULL;
}

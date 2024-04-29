/****************************************************************************
 * libs/libc/modlib/modlib_gethandle.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_modhandle
 *
 * Description:
 *   modlib_modhandle() returns the module handle for the installed
 *   module with the provided name.  A secondary use of this function is to
 *   determine if a module has been loaded or not.
 *
 * Input Parameters:
 *   name   - A pointer to the module name string.
 *
 * Returned Value:
 *   The non-NULL module handle previously returned by modlib_insert() is
 *   returned on success.  If no module with that name is installed,
 *   modlib_modhandle() will return a NULL handle and the errno variable
 *   will be set appropriately.
 *
 ****************************************************************************/

#ifdef HAVE_MODLIB_NAMES

FAR void *modlib_gethandle(FAR const char *name)
{
  FAR struct module_s *modp;

  DEBUGASSERT(name != NULL);

  /* Get exclusive access to the module registry */

  modlib_registry_lock();

  /* Find the module entry for this name in the registry */

  modp = modlib_registry_find(name);
  if (modp == NULL)
    {
      berr("ERROR: Failed to find module %s\n", name);
      set_errno(ENOENT);
    }

  modlib_registry_unlock();
  return modp;
}

#endif /* HAVE_MODLIB_NAMES */

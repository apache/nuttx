/****************************************************************************
 * libs/libc/modlib/modlib_registry.c
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

#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/mutex.h>
#include <nuttx/lib/modlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static rmutex_t g_modlock = RMUTEX_INITIALIZER;

static FAR struct module_s *g_mod_registry;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_registry_lock
 *
 * Description:
 *   Get exclusive access to the module registry.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void modlib_registry_lock(void)
{
  _RMUTEX_LOCK(&g_modlock);
}

/****************************************************************************
 * Name: modlib_registry_unlock
 *
 * Description:
 *   Relinquish the lock on the module registry
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void modlib_registry_unlock(void)
{
  _RMUTEX_UNLOCK(&g_modlock);
}

/****************************************************************************
 * Name: modlib_registry_add
 *
 * Description:
 *   Add a new entry to the module registry.
 *
 * Input Parameters:
 *   modp - The module data structure to be registered.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller holds the lock on the module registry.
 *
 ****************************************************************************/

void modlib_registry_add(FAR struct module_s *modp)
{
  DEBUGASSERT(modp);
  modp->flink = g_mod_registry;
  g_mod_registry = modp;
}

/****************************************************************************
 * Name: modlib_registry_del
 *
 * Description:
 *   Remove a module entry from the registry
 *
 * Input Parameters:
 *   modp - The registry entry to be removed.
 *
 * Returned Value:
 *   Zero (OK) is returned if the registry entry was deleted.  Otherwise,
 *   a negated errno value is returned.
 *
 * Assumptions:
 *   The caller holds the lock on the module registry.
 *
 ****************************************************************************/

int modlib_registry_del(FAR struct module_s *modp)
{
  FAR struct module_s *prev;
  FAR struct module_s *curr;

  for (prev = NULL, curr = g_mod_registry;
       curr != NULL && curr != modp;
       prev = curr, curr = curr->flink);

  if (curr == NULL)
    {
      berr("ERROR: Could not find module entry\n");
      return -ENOENT;
    }

  if (prev == NULL)
    {
      g_mod_registry = modp->flink;
    }
  else
    {
      prev->flink = modp->flink;
    }

  modp->flink = NULL;
  return OK;
}

/****************************************************************************
 * Name: modlib_registry_find
 *
 * Description:
 *   Find an entry in the module registry using the name of the module.
 *
 * Input Parameters:
 *   modname - The name of the module to be found
 *
 * Returned Value:
 *   If the registry entry is found, a pointer to the module entry is
 *   returned.  NULL is returned if the entry is not found.
 *
 * Assumptions:
 *   The caller holds the lock on the module registry.
 *
 ****************************************************************************/

#ifdef HAVE_MODLIB_NAMES
FAR struct module_s *modlib_registry_find(FAR const char *modname)
{
  FAR struct module_s *modp;

  for (modp = g_mod_registry;
       modp != NULL && strncmp(modp->modname, modname, MODLIB_NAMEMAX) != 0;
       modp = modp->flink);

  return modp;
}
#endif

/****************************************************************************
 * Name: modlib_registry_verify
 *
 * Description:
 *   Verify that a module handle is valid by traversing the module list and
 *   assuring that the module still resides in the list.  If it does not,
 *   the handle is probably a stale pointer.
 *
 * Input Parameters:
 *   modp - The registry entry to be verified.
 *
 * Returned Value:
 *   Returns OK is the module is valid; -ENOENT otherwise.
 *
 * Assumptions:
 *   The caller holds the lock on the module registry.
 *
 ****************************************************************************/

int modlib_registry_verify(FAR struct module_s *modp)
{
  FAR struct module_s *node;

  for (node = g_mod_registry; node != NULL; node = node->flink)
    {
      if (node == modp)
        {
          return OK;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: modlib_registry_foreach
 *
 * Description:
 *   Visit each module in the registry
 *
 * Input Parameters:
 *   callback - This callback function was be called for each entry in the
 *     registry.
 *   arg - This opaque argument will be passed to the callback function.
 *
 * Returned Value:
 *   This function normally returns zero (OK).  If, however, any callback
 *   function returns a non-zero value, the traversal will be terminated and
 *   that non-zero value will be returned.
 *
 ****************************************************************************/

int modlib_registry_foreach(mod_callback_t callback, FAR void *arg)
{
  FAR struct module_s *modp;
  int ret = OK;

  /* Get exclusive access to the module registry */

  modlib_registry_lock();

  /* Visit each installed module */

  for (modp = g_mod_registry; modp != NULL; modp = modp->flink)
    {
      /* Perform the callback */

      ret = callback(modp, arg);
      if (ret != 0)
        {
          break;
        }
    }

  modlib_registry_unlock();
  return ret;
}

/****************************************************************************
 * sched/module/mod_registry.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <semaphore.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/module.h>

#include "module.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_mod_lock = SEM_INITIALIZER(1);
static FAR struct module_s *g_mod_registry;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mod_registry_lock
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

void mod_registry_lock(void)
{
  while (sem_post(&g_mod_lock) < 0)
    {
      DEBUGASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: mod_registry_unlock
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

void mod_registry_unlock(void)
{
  sem_post(&g_mod_lock);
}

/****************************************************************************
 * Name: mod_registry_add
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

void mod_registry_add(FAR struct module_s *modp)
{
  DEBUGASSERT(modp);
  modp->flink = g_mod_registry;
  g_mod_registry = modp;
}

/****************************************************************************
 * Name: mod_registry_del
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

int mod_registry_del(FAR struct module_s *modp)
{
  FAR struct module_s *prev;
  FAR struct module_s *curr;

  for (prev = NULL, curr = g_mod_registry;
       curr != NULL && curr != modp;
       prev = curr, curr = curr->flink);

  if (curr == NULL)
    {
      sdbg("ERROR: Could not find module entry\n");
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
 * Name: mod_registry_find
 *
 * Description:
 *   Find an entry in the module registry using the name of the module.
 *
 * Input Parameters:
 *   modulename - The name of the module to be found
 *
 * Returned Value:
 *   If the registry entry is found, a pointer to the module entry is
 *   returned.  NULL is returned if the they entry is not found.
 *
 * Assumptions:
 *   The caller holds the lock on the module registry.
 *
 ****************************************************************************/

FAR struct module_s *mod_registry_find(FAR const char *modulename)
{
  FAR struct module_s *modp;

  for (modp = g_mod_registry;
       modp != NULL && strncmp(modp->modulename, modulename, MODULENAME_MAX) != 0;
       modp = modp->flink);

  return modp;
}

/****************************************************************************
 * Name: mod_registry_foreach
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
 * Assumptions:
 *   The caller does *NOT* hold the lock on the module registry.
 *
 ****************************************************************************/

int mod_registry_foreach(mod_callback_t callback, FAR void *arg)
{
  FAR struct module_s *modp;
  int ret;

  /* Get exclusive access to the module registry */

  mod_registry_lock();

  /* Visit each installed module */

  for (modp = g_mod_registry; modp != NULL; modp = modp->flink)
    {
      /* Perform the callback */

      ret = callback(modp, arg);
      if (ret != 0)
        {
          return ret;
        }
    }

  mod_registry_unlock();
  return OK;
}
/****************************************************************************
 * libs/libc/elf/elf_registry.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/list.h>
#include <nuttx/lib/elf.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: libelf_registry_add
 *
 * Description:
 *   Add a new entry to the module registry.
 *
 * Input Parameters:
 *   registry - The registry.
 *   modp     - The module data structure to be registered.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller holds the lock of the registry.
 *
 ****************************************************************************/

void libelf_registry_add(FAR struct list_node *registry,
                         FAR struct module_s *modp)
{
  DEBUGASSERT(registry != NULL && modp != NULL);
  list_add_head(registry, &modp->node);
}

/****************************************************************************
 * Name: libelf_registry_del
 *
 * Description:
 *   Remove a module entry from its registry.
 *
 * Input Parameters:
 *   modp - The registry entry to be removed.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller holds the lock of the registry.
 *
 ****************************************************************************/

void libelf_registry_del(FAR struct module_s *modp)
{
  DEBUGASSERT(modp != NULL);
  list_delete_init(&modp->node);
}

/****************************************************************************
 * Name: libelf_registry_find
 *
 * Description:
 *   Find an entry in the module registry using the name of the module.
 *
 * Input Parameters:
 *   registry - The registry.
 *   modname  - The name of the module to be found
 *
 * Returned Value:
 *   If the registry entry is found, a pointer to the module entry is
 *   returned.  NULL is returned if the entry is not found.
 *
 * Assumptions:
 *   The caller holds the lock of the registry.
 *
 ****************************************************************************/

#ifdef HAVE_LIBC_ELF_NAMES
FAR struct module_s *libelf_registry_find(FAR struct list_node *registry,
                                          FAR const char *modname)
{
  FAR struct module_s *modp;

  list_for_every_entry(registry, modp, struct module_s, node)
    {
      if (strncmp(modp->modname, modname, LIBC_ELF_NAMEMAX) == 0)
        {
          return modp;
        }
    }

  return NULL;
}
#endif

/****************************************************************************
 * Name: libelf_registry_verify
 *
 * Description:
 *   Verify that a module handle is valid by traversing the module list and
 *   assuring that the module still resides in the list.  If it does not,
 *   the handle is probably a stale pointer.
 *
 * Input Parameters:
 *   registry - The registry.
 *   modp     - The registry entry to be verified.
 *
 * Returned Value:
 *   Returns OK is the module is valid; -ENOENT otherwise.
 *
 * Assumptions:
 *   The caller holds the lock of the registry.
 *
 ****************************************************************************/

int libelf_registry_verify(FAR struct list_node *registry,
                           FAR struct module_s *modp)
{
  FAR struct module_s *node;

  list_for_every_entry(registry, node, struct module_s, node)
    {
      if (node == modp)
        {
          return OK;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: libelf_registry_foreach
 *
 * Description:
 *   Visit each module in the registry
 *
 * Input Parameters:
 *   registry - The registry, or NULL for none.
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
 *   The caller holds the lock of the registry.
 *
 ****************************************************************************/

int libelf_registry_foreach(FAR struct list_node *registry,
                            mod_callback_t callback, FAR void *arg)
{
  FAR struct module_s *modp;
  int ret = OK;

  if (registry == NULL)
    {
      return OK;
    }

  list_for_every_entry(registry, modp, struct module_s, node)
    {
      ret = callback(modp, arg);
      if (ret != 0)
        {
          break;
        }
    }

  return ret;
}

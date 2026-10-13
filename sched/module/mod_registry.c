/****************************************************************************
 * sched/module/mod_registry.c
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

#include <errno.h>

#include <nuttx/list.h>
#include <nuttx/mutex.h>

#include "module/module.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The kernel modules that insmod() loads.  What dlopen() loads is on the
 * registry of the task group.
 */

static rmutex_t g_mod_lock = NXRMUTEX_INITIALIZER;
static struct list_node g_mod_registry = LIST_INITIAL_VALUE(g_mod_registry);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mod_registry_lock
 *
 * Description:
 *   Lock the registry of the kernel modules that insmod() loads, and
 *   return it.  The lock is recursive: the constructors of a module run
 *   with it held.
 *
 ****************************************************************************/

FAR struct list_node *mod_registry_lock(void)
{
  int ret;

  /* A signal must not make the caller run without the lock */

  do
    {
      ret = nxrmutex_lock(&g_mod_lock);
    }
  while (ret == -EINTR);

  return &g_mod_registry;
}

/****************************************************************************
 * Name: mod_registry_unlock
 *
 * Description:
 *   Unlock the registry of the kernel modules.
 *
 ****************************************************************************/

void mod_registry_unlock(void)
{
  nxrmutex_unlock(&g_mod_lock);
}

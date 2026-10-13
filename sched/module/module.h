/****************************************************************************
 * sched/module/module.h
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

#ifndef __SCHED_MODULE_MODULE_H
#define __SCHED_MODULE_MODULE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/list.h>

#ifdef CONFIG_MODULE

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mod_registry_lock
 *
 * Description:
 *   Lock the registry of the kernel modules that insmod() loads, and
 *   return it.
 *
 ****************************************************************************/

FAR struct list_node *mod_registry_lock(void);

/****************************************************************************
 * Name: mod_registry_unlock
 *
 * Description:
 *   Unlock the registry of the kernel modules.
 *
 ****************************************************************************/

void mod_registry_unlock(void);

#endif /* CONFIG_MODULE */
#endif /* __SCHED_MODULE_MODULE_H */

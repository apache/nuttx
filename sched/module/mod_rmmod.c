/****************************************************************************
 * sched/module/mod_rmmod.c
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

#include <nuttx/module.h>
#include <nuttx/lib/modlib.h>

#ifdef CONFIG_MODULE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rmmod
 *
 * Description:
 *   Remove a previously installed module from memory.
 *
 * Input Parameters:
 *   handle - The module handler previously returned by insmod().
 *
 * Returned Value:
 *   Zero (OK) on success.  On any failure, -1 (ERROR) is returned the
 *   errno value is set appropriately.
 *
 ****************************************************************************/

int rmmod(FAR void *handle)
{
  return modlib_remove(handle);
}

#endif /* CONFIG_MODULE */

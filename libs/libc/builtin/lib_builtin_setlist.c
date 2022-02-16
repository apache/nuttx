/****************************************************************************
 * libs/libc/builtin/lib_builtin_setlist.c
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

#include <nuttx/lib/builtin.h>

#if defined(CONFIG_BUILD_PROTECTED) && defined(__KERNEL__)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR const struct builtin_s *g_builtins;
int g_builtin_count;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: builtin_setlist
 *
 * Description:
 *   Saves the user-space list of built-in applications for use by BINFS in
 *   protected mode.  Normally this is small set of globals provided by
 *   user-space logic.  It provides name-value pairs for associating
 *   built-in application names with user-space entry point addresses.
 *   These globals are only needed for use by BINFS which executes built-in
 *   applications from kernel-space in PROTECTED mode.  In the FLAT build,
 *   the user space globals are readily available.  (BINFS is not
 *   supportable in KERNEL mode since user-space address have no general
 *   meaning that configuration).
 *
 * Input Parameters:
 *   builtins - The list of built-in functions.  Each entry is a name-value
 *              pair that maps a built-in function name to its user-space
 *              entry point address.
 *   count    - The number of name-value pairs in the built-in list.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void builtin_setlist(FAR const struct builtin_s *builtins, int count)
{
  g_builtins      = builtins;
  g_builtin_count = count;
}

#endif /* CONFIG_BUILD_PROTECTED && __KERNEL__ */

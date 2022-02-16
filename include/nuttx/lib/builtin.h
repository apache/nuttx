/****************************************************************************
 * include/nuttx/lib/builtin.h
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

#ifndef __INCLUDE_NUTTX_LIB_BUILTIN_H
#define __INCLUDE_NUTTX_LIB_BUILTIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#ifdef CONFIG_BUILTIN

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct builtin_s
{
  const char *name;         /* Invocation name and as seen under /sbin/ */
  int         priority;     /* Use: SCHED_PRIORITY_DEFAULT */
  int         stacksize;    /* Desired stack size */
  main_t      main;         /* Entry point: main(int argc, char *argv[]) */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#if defined(CONFIG_BUILD_PROTECTED) && defined(__KERNEL__)
/* In the PROTECTED build, the builtin arrays are only needed by BINFS.
 * in this case, the user-space globals are not accessible and must be
 * provided to the OS via the boardctl(BOARDIOC_BUILTINS) call.  In this
 * case, the PROTECTED kernel will keep its own copy of the user-space
 * array.
 *
 * The call to boardctl(BOARDIOC_BUILTINS) must be provided by the
 * application layer.
 */

EXTERN FAR const struct builtin_s *g_builtins;
EXTERN int g_builtin_count;

#else
/* In the FLAT build, the builtin list is just a global global array and
 * count exported from user space via the backdoor left open by the FLAT
 * address space.  These globals must be provided the application layer.
 */

EXTERN FAR const struct builtin_s g_builtins[];
EXTERN const int g_builtin_count;
#endif

/****************************************************************************
 * Public Function Prototypes
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

#if defined(CONFIG_BUILD_PROTECTED) && defined(__KERNEL__)
void builtin_setlist(FAR const struct builtin_s *builtins, int count);
#endif

/****************************************************************************
 * Name: builtin_isavail
 *
 * Description:
 *   Checks for availability of an application named 'appname' registered
 *   during compile time and, if available, returns the index into the table
 *   of built-in applications.
 *
 * Input Parameters:
 *   filename - Name of the linked-in binary to be started.
 *
 * Returned Value:
 *   This is an internal function, used by by the NuttX binfmt logic and
 *   by the application built-in logic.  It returns a non-negative index to
 *   the application entry in the table of built-in applications on success
 *   or a negated errno value in the event of a failure.
 *
 ****************************************************************************/

int builtin_isavail(FAR const char *appname);

/****************************************************************************
 * Name: builtin_getname
 *
 * Description:
 *   Returns pointer to a name of built-in application pointed by the
 *   index.
 *
 * Input Parameters:
 *   index, from 0 and on ...
 *
 * Returned Value:
 *   Returns valid pointer pointing to the app name if index is valid.
 *   Otherwise NULL is returned.
 *
 ****************************************************************************/

FAR const char *builtin_getname(int index);

/****************************************************************************
 * Name: builtin_for_index
 *
 * Description:
 *   Returns the builtin_s structure for the selected built-in.
 *   If support for built-in functions is enabled in the NuttX
 *   configuration, then this function must be provided by the application
 *   code.
 *
 * Input Parameters:
 *   index, from 0 and on...
 *
 * Returned Value:
 *   Returns valid pointer pointing to the builtin_s structure if index is
 *   valid.
 *   Otherwise, NULL is returned.
 *
 ****************************************************************************/

FAR const struct builtin_s *builtin_for_index(int index);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_BUILTIN */
#endif /* __INCLUDE_NUTTX_LIB_BUILTIN_H */

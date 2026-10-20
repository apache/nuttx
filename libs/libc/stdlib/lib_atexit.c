/****************************************************************************
 * libs/libc/stdlib/lib_atexit.c
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

#include <stdlib.h>

#include <nuttx/atexit.h>
#include <nuttx/lib/lib.h>
#include <nuttx/mutex.h>
#include <nuttx/tls.h>

#if CONFIG_LIBC_MAX_EXITFUNS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of entries that each heap allocation adds */

#define ATEXIT_GROW 8

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_exitfuncs
 *
 * Description:
 *    Obtain the list of exit functions.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Pointer to the list of exit functions.
 *
 ****************************************************************************/

static FAR struct atexit_list_s *get_exitfuncs(void)
{
  FAR struct task_info_s *info;

  info = task_get_info();
  return &info->ta_exit;
}

/****************************************************************************
 * Name: get_entry
 *
 * Description:
 *    Return entry 'idx' of the list: in funcs[] or in the heap part.
 *
 ****************************************************************************/

static inline FAR struct atexit_s *
get_entry(FAR struct atexit_list_s *aehead, int idx)
{
  if (idx < CONFIG_LIBC_MAX_EXITFUNS)
    {
      return &aehead->funcs[idx];
    }

  return &aehead->extra[idx - CONFIG_LIBC_MAX_EXITFUNS];
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int atexit_register(int type, CODE void (*func)(void), FAR void *arg,
                    FAR void *dso)
{
  FAR struct task_info_s   *info = task_get_info();
  FAR struct atexit_list_s *aehead;
  FAR struct atexit_s      *entry;
  int                       ret;

  /* REVISIT: Missing logic */

  UNUSED(dso);

  if (func == NULL)
    {
      return ERROR;
    }

  aehead = get_exitfuncs();

  ret = nxrmutex_lock(&info->ta_lock);
  if (ret < 0)
    {
      return ERROR;
    }

  /* Grow the heap part when the list is full */

  if (aehead->nfuncs >= CONFIG_LIBC_MAX_EXITFUNS + aehead->nextra)
    {
      entry = lib_realloc(aehead->extra, (aehead->nextra + ATEXIT_GROW) *
                                         sizeof(struct atexit_s));
      if (entry == NULL)
        {
          nxrmutex_unlock(&info->ta_lock);
          return ERROR;
        }

      aehead->extra   = entry;
      aehead->nextra += ATEXIT_GROW;
    }

  entry       = get_entry(aehead, aehead->nfuncs);
  entry->type = type;
  entry->func = func;
  entry->arg  = arg;
  aehead->nfuncs++;

  nxrmutex_unlock(&info->ta_lock);
  return OK;
}

void atexit_call_exitfuncs(int status, bool quick)
{
  FAR struct atexit_list_s *aehead;
  FAR struct task_info_s   *info = task_get_info();
  FAR struct atexit_s      *entry;
  CODE void               (*func)(void);
  FAR void                 *arg;
  int                       type;

  /* Call exit functions in reverse order */

  aehead = get_exitfuncs();

  while (aehead->nfuncs > 0)
    {
      /* Claim the newest entry under the lock.  A handler may register
       * further functions during exit processing; those land in the slot
       * just freed here and are executed on the next iteration, i.e.
       * before the older remaining ones, as documented in exit(3).
       */

      if (nxrmutex_lock(&info->ta_lock) < 0)
        {
          break;
        }

      entry = get_entry(aehead, aehead->nfuncs - 1);

      type = entry->type;
      func = entry->func;
      arg  = entry->arg;

      /* Remove the function to prevent recursive call to it */

      entry->func = NULL;
      entry->arg  = NULL;
      aehead->nfuncs--;

      nxrmutex_unlock(&info->ta_lock);

      if (!func)
        {
          continue;
        }

      if (quick != (type == ATTYPE_ATQUICKEXIT))
        {
          continue;
        }

      /* Call the atexit/on_exit/cxa_atexit() function */

      if (type == ATTYPE_ATEXIT || type == ATTYPE_ATQUICKEXIT)
        {
          (*func)();
        }
      else if (type == ATTYPE_ONEXIT)
        {
          (*((CODE void (*)(int, FAR void *))func))(status, arg);
        }
      else if (type == ATTYPE_CXA)
        {
          (*((CODE void (*)(FAR void *))func))(arg);
        }
    }

  /* Release the heap part */

  if (nxrmutex_lock(&info->ta_lock) >= 0)
    {
      if (aehead->nfuncs == 0)
        {
          lib_free(aehead->extra);
          aehead->extra  = NULL;
          aehead->nextra = 0;
        }

      nxrmutex_unlock(&info->ta_lock);
    }
}

/****************************************************************************
 * Name: atexit
 *
 * Description:
 *    Registers a function to be called at program exit.
 *    The atexit() function registers the given function to be called
 *    at normal process termination, whether via exit or via return from
 *    the program's main().
 *
 *    Limitations in the current implementation:
 *
 *      1. atexit functions are not inherited when a new task is
 *         created.
 *
 * Input Parameters:
 *   func - A pointer to the function to be called when the task exits.
 *
 * Returned Value:
 *   Zero on success. Non-zero on failure.
 *
 ****************************************************************************/

int atexit(CODE void (*func)(void))
{
  return atexit_register(ATTYPE_ATEXIT, func, NULL, NULL);
}

/****************************************************************************
 * Name: at_quick_exit
 *
 * Description:
 *    Registers the function pointed to by func to be called on quick
 *    program termination (via quick_exit).
 *
 * Input Parameters:
 *   func - A pointer to the function to be called when the task exits.
 *
 * Returned Value:
 *   Zero on success. Non-zero on failure.
 *
 ****************************************************************************/

int at_quick_exit(CODE void (*func)(void))
{
  return atexit_register(ATTYPE_ATQUICKEXIT, func, NULL, NULL);
}

/****************************************************************************
 * Name: on_exit
 *
 * Description:
 *    Registers a function to be called at program exit.
 *    The on_exit() function registers the given function to be called
 *    at normal process termination, whether via exit or via return from
 *    the program's main(). The function is passed the status argument
 *    given to the last call to exit and the arg argument from on_exit().
 *
 *    NOTE 1: This function comes from SunOS 4, but is also present in
 *    libc4, libc5 and glibc. It no longer occurs in Solaris (SunOS 5).
 *    Avoid this function, and use the standard atexit() instead.
 *
 *    Limitations in the current implementation:
 *
 *      1. on_exit functions are not inherited when a new task is
 *         created.
 *
 * Input Parameters:
 *   func - A pointer to the function to be called when the task exits.
 *   arg -  An argument that will be provided to the on_exit() function when
 *          the task exits.
 *
 * Returned Value:
 *   Zero on success. Non-zero on failure.
 *
 ****************************************************************************/

int on_exit(CODE void (*func)(int, FAR void *), FAR void *arg)
{
  return atexit_register(ATTYPE_ONEXIT, (CODE void (*)(void))func, arg,
                         NULL);
}

#endif /* CONFIG_LIBC_MAX_EXITFUNS > 0 */

/****************************************************************************
 * Name: __cxa_atexit
 *
 * Description:
 *   __cxa_atexit() registers a destructor function to be called by exit().
 *   On a call to exit(), the registered functions should be called with
 *   the single argument 'arg'. Destructor functions shall always be
 *   called in the reverse order to their registration (i.e. the most
 *   recently registered function shall be called first),
 *
 *   If shared libraries were supported, the callbacks should be invoked
 *   when the shared library is unloaded as well.
 *
 * Reference:
 *   Linux base
 *
 ****************************************************************************/

int __cxa_atexit(CODE void (*func)(FAR void *), FAR void *arg,
                 FAR void *dso_handle)
{
  return atexit_register(ATTYPE_CXA, (CODE void (*)(void))func, arg,
                         dso_handle);
}

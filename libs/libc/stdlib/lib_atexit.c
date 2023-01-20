/****************************************************************************
 * libs/libc/stdlib/lib_atexit.c
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
#include <nuttx/mutex.h>
#include <nuttx/tls.h>

#if CONFIG_LIBC_MAX_EXITFUNS > 0

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
 * Public Functions
 ****************************************************************************/

int atexit_register(int type, CODE void (*func)(void), FAR void *arg,
                    FAR void *dso)
{
  FAR struct task_info_s   *info = task_get_info();
  FAR struct atexit_list_s *aehead;
  int                       idx;
  int                       ret = ERROR;

  /* REVISIT: Missing logic */

  UNUSED(dso);

  /* The following must be atomic */

  aehead = get_exitfuncs();

  if (func)
    {
      ret = nxmutex_lock(&info->ta_lock);
      if (ret < 0)
        {
          return -ret;
        }

      if ((idx = aehead->nfuncs) < ATEXIT_MAX)
        {
          aehead->funcs[idx].type = type;
          aehead->funcs[idx].func = func;
          aehead->funcs[idx].arg  = arg;
          aehead->nfuncs++;
          ret = OK;
        }
      else
        {
          ret = ERROR;
        }

      nxmutex_unlock(&info->ta_lock);
    }

  return ret;
}

void atexit_call_exitfuncs(int status, bool quick)
{
  FAR struct atexit_list_s *aehead;
  CODE void               (*func)(void);
  FAR void                 *arg;
  int                       idx;
  int                       type;

  /* Call exit functions in reverse order */

  aehead = get_exitfuncs();

  for (idx = aehead->nfuncs - 1; idx >= 0; idx--)
    {
      /* Remove the function to prevent recursive call to it */

      type = aehead->funcs[idx].type;

      func = aehead->funcs[idx].func;
      arg  = aehead->funcs[idx].arg;

      aehead->funcs[idx].func = NULL;
      aehead->funcs[idx].arg  = NULL;

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
}

#endif

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
 *      1. Only a single atexit function can be registered unless
 *         CONFIG_LIBC_MAX_EXITFUNS defines a larger number.
 *      2. atexit functions are not inherited when a new task is
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

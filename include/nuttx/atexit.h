/****************************************************************************
 * include/nuttx/atexit.h
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

#ifndef __INCLUDE_NUTTX_ATEXIT_H
#define __INCLUDE_NUTTX_ATEXIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Amount of exit functions */

#define ATEXIT_MAX (CONFIG_LIBC_MAX_EXITFUNS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum atexit_type_e
{
  ATTYPE_NONE,
  ATTYPE_ATEXIT,
  ATTYPE_ONEXIT,
  ATTYPE_CXA
};

struct atexit_s
{
  int         type;
  CODE void (*func)(void);
  FAR void   *arg;
};

struct atexit_list_s
{
  int             nfuncs;
  struct atexit_s funcs[ATEXIT_MAX];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

#if CONFIG_LIBC_MAX_EXITFUNS > 0

/****************************************************************************
 * Name: atexit_register
 *
 * Description:
 *   atexit_register registers a function function to be called by exit().
 *
 * Input Parameters:
 *   type - Type of exit function. Available types in atexit_type_e.
 *   func - Function to be called on exit.
 *   arg  - Optional argument to be passed to function on exit.
 *   dso  - Dso handle, called when shared library is unloaded.
 *
 * Returned value:
 *   OK  on success; ERROR on failure
 *
 ****************************************************************************/

int atexit_register(int type, CODE void (*func)(void), FAR void *arg,
                    FAR void *dso);

/****************************************************************************
 * Name: atexit_call_exitfuncs
 *
 * Description:
 *   Execute the registered exit functions. Call this in exit().
 *
 * Input Parameters:
 *   status - Process exit status code.
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void atexit_call_exitfuncs(int status);
#else
#  define atexit_register(type, func, arg, dso) (0)
#  define atexit_call_exitfuncs(status)
#endif /* CONFIG_LIBC_MAX_EXITFUNS */

#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_NUTTX_ATEXIT_H */

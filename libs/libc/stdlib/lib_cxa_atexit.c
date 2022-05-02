/****************************************************************************
 * libs/libc/stdlib/lib_cxa_atexit.c
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

#include <nuttx/atexit.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

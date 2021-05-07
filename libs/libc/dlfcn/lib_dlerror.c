/****************************************************************************
 * libs/libc/dlfcn/lib_dlerror.c
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
#include <dlfcn.h>
#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dlerror
 *
 * Description:
 *   dlerror() returns a null-terminated character string (with no trailing
 *   newline) that describes the last error that occurred during dynamic
 *   linking processing. If no dynamic linking errors have occurred since
 *   the last invocation of dlerror(), dlerror() returns NULL. Thus,
 *   invoking dlerror() a second time, immediately following a prior
 *   invocation, will result in NULL being returned.
 *
 * Input Parameters:
 *   If successful, dlerror() returns a null-terminated character string.
 *   Otherwise, NULL is returned.
 *
 * Returned Value:
 *
 * Reference: OpenGroup.org
 *
 ****************************************************************************/

FAR char *dlerror(void)
{
  return (FAR char *)strerror(get_errno());
}

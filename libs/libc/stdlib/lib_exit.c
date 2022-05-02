/****************************************************************************
 * libs/libc/stdlib/lib_exit.c
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
#include <nuttx/compiler.h>

#include <stdlib.h>
#include <unistd.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern FAR void *__dso_handle weak_data;
FAR void *__dso_handle = &__dso_handle;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void exit(int status)
{
  atexit_call_exitfuncs(status);

  /* REVISIT: Need to flush files and streams */

  _exit(status);
}

void _Exit(int status)
{
  _exit(status);
}

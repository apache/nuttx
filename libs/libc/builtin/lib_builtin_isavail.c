/****************************************************************************
 * libs/libc/builtin/lib_builtin_isavail.c
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

#include <string.h>
#include <limits.h>
#include <errno.h>

#include <nuttx/lib/builtin.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

int builtin_isavail(FAR const char *appname)
{
  FAR const char *name;
  int i;

  for (i = 0; (name = builtin_getname(i)) != NULL; i++)
    {
      if (strcmp(name, appname) == 0)
        {
          return i;
        }
    }

  return -ENOENT;
}

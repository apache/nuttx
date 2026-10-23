/****************************************************************************
 * libs/libc/stdio/lib_ctermid.c
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

#include <stdio.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The controlling terminal of every task.  L_ctermid in stdio.h is the
 * size of this string.
 */

#define CTERMID_PATH "/dev/console"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ctermid
 *
 * Description:
 *   The ctermid() function generates a string that is the pathname of the
 *   controlling terminal of the current process.  On NuttX this is always
 *   the system console.
 *
 *   If s is a null pointer, ctermid() writes the string into an internal
 *   static object and returns a pointer to it.  Otherwise s must point to
 *   an array of at least L_ctermid bytes; ctermid() writes the string into
 *   that array and returns s.
 *
 * Returned Value:
 *   A pointer to the pathname of the controlling terminal.
 *
 ****************************************************************************/

FAR char *ctermid(FAR char *s)
{
  static char path[sizeof(CTERMID_PATH)];

  if (s == NULL)
    {
      s = path;
    }

  return strcpy(s, CTERMID_PATH);
}

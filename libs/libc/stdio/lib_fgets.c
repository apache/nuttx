/****************************************************************************
 * libs/libc/stdio/lib_fgets.c
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

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fgets
 *
 * Description:
 *   fgets() reads in at most one less than 'buflen' characters from stream
 *   and stores them into the buffer pointed to by 'buf'. Reading stops after
 *   an EOF or a newline.  If a newline is read, it is stored into the
 *   buffer.  A null terminator is stored after the last character in the
 *   buffer.
 *
 ****************************************************************************/

char *fgets(FAR char *buf, int buflen, FAR FILE *stream)
{
  /* Handle negative buffer size */

  if (buflen < 0)
    {
      return NULL;
    }

  /* Let lib_fgets() do the heavy lifting */

  else
    {
      return lib_fgets(buf, buflen, stream, true, false);
    }
}

/****************************************************************************
 * libs/libc/stdio/lib_gets.c
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

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gets
 *
 * Description:
 *   gets() reads a line from stdin into the buffer pointed to by s until
 *   either a terminating newline or EOF, which it replaces with '\0'.  No
 *   check for buffer overrun is performed
 *
 *   This API should not be used because it is inherently unsafe.  Consider
 *   using fgets which is safer and slightly more efficient.
 *
 ****************************************************************************/

FAR char *gets(FAR char *s)
{
  /* Let lib_fgets() do the heavy lifting */

#ifdef CONFIG_FILE_STREAM
  return lib_fgets(s, SIZE_MAX, stdin, false, false);
#else
  return lib_dgets(s, SIZE_MAX, STDIN_FILENO, false, false);
#endif
}

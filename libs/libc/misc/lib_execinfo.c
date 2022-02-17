/****************************************************************************
 * libs/libc/misc/lib_execinfo.c
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

#include <execinfo.h>
#include <stdio.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR char **backtrace_malloc(FAR void *const *buffer, int size)
{
  size_t length = 0;

  if (size <= 0)
    {
      return NULL;
    }

  while (size-- > 0)
    {
      int ret = snprintf(NULL, 0, "%pS", *buffer++);
      if (ret < 0)
        {
          return NULL;
        }

      length += sizeof(FAR char *) + ret + 1;
    }

  return lib_malloc(length);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR char **backtrace_symbols(FAR void *const *buffer, int size)
{
  FAR char **syms;
  FAR char *buf;
  int i;

  syms = backtrace_malloc(buffer, size);
  if (syms != NULL)
    {
      buf = (FAR char *)&syms[size];
      for (i = 0; i < size; i++)
        {
          syms[i] = buf;
          buf += sprintf(buf, "%pS", buffer[i]);
          buf += 1;
        }
    }

  return syms;
}

void backtrace_symbols_fd(FAR void *const *buffer, int size, int fd)
{
  int i;

  for (i = 0; i < size; i++)
    {
      dprintf(fd, "%pS\n", buffer[i]);
    }
}

/****************************************************************************
 * libs/libc/sched/sched_dumpstack.c
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

#include <sys/types.h>

#include <stdio.h>
#include <syslog.h>
#include <execinfo.h>

#define DUMP_FORMAT "%*p"
#define DUMP_WIDTH  (int)(2 * sizeof(FAR void *) + 3)

#define DUMP_DEPTH  16
#define DUMP_NITEM  8
#define DUMP_LINESIZE (DUMP_NITEM * DUMP_WIDTH)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_dumpstack
 *
 * Description:
 *  Dump thread backtrace from specified tid.
 *
 ****************************************************************************/

void sched_dumpstack(pid_t tid)
{
  FAR void *address[DUMP_DEPTH];
  char line[DUMP_LINESIZE + 1];
  int ret = 0;
  int size;
  int i;

  size = sched_backtrace(tid, address, DUMP_DEPTH);
  if (size <= 0)
    {
      return;
    }

  for (i = 0; i < size; i++)
    {
      ret += snprintf(line + ret, sizeof(line) - ret,
                      DUMP_FORMAT, DUMP_WIDTH, address[i]);
      if (i == size - 1 || ret % DUMP_LINESIZE == 0)
        {
          syslog(LOG_EMERG, "backtrace|%2d: %s\n", tid, line);
          ret = 0;
        }
    }
}

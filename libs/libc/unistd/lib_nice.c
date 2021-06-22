/****************************************************************************
 * libs/libc/unistd/lib_nice.c
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

#include <sys/resource.h>
#include <unistd.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nice
 *
 * Description:
 *  The nice() function shall add the value of incr to the nice value of the
 *  calling process. A process' nice value is a non-negative number for which
 *  a more positive value shall result in less favorable scheduling.
 *
 ****************************************************************************/

int nice(int inc)
{
  int prio;
  int ret;

  set_errno(0);
  ret = getpriority(PRIO_PROCESS, 0);
  if (get_errno() != 0)
    {
      return ret;
    }

  prio = ret + inc;
  ret = setpriority(PRIO_PROCESS, 0, prio);
  if (ret < 0)
    {
      return ret;
    }

  return prio;
}

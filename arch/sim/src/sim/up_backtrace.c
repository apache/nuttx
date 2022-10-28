/****************************************************************************
 * arch/sim/src/sim/up_backtrace.c
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

#include <nuttx/arch.h>
#include <sched/sched.h>

#include <string.h>

#include "up_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

nosanitize_address
int up_backtrace(struct tcb_s *tcb, void **buffer, int size, int skip)
{
  void *buf[skip + size];
  int ret = 0;
  int i;

  if (tcb == running_task())
    {
      ret = host_backtrace(buf, skip + size);
    }

  if (ret <= skip)
    {
      return ret < 0 ? ret : 0;
    }

  ret -= skip;
  for (i = 0; i < ret; i++)
    {
      buffer[i] = buf[skip + i];
    }

  return ret;
}

/****************************************************************************
 * libs/libc/misc/lib_fdcheck.c
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

#include <nuttx/fdcheck.h>
#include <nuttx/lib/math32.h>
#include <nuttx/sched.h>
#include <nuttx/spinlock.h>
#include <sys/ioctl.h>

#include <debug.h>
#include <stdio.h>

#ifdef CONFIG_FDCHECK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FD_SHIFT  0
#define FD_BITS   LOG2_CEIL(OPEN_MAX)
#define FD_MASK   ((1 << FD_BITS) - 1)

#define TAG_SHIFT (FD_BITS + FD_SHIFT)
#define TAG_BITS  8
#define TAG_MASK  ((1 << TAG_BITS) - 1)

#define PID_SHIFT (TAG_BITS + TAG_SHIFT)
#define PID_BITS  (8 * sizeof(int) - 1 - PID_SHIFT)
#define PID_MASK  ((1 << PID_BITS) - 1)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_fdcheck_lock = SP_UNLOCKED;
static uint8_t    g_fdcheck_tag = 0;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdcheck_restore
 *
 * Description: Obtain original fd information
 *
 * Val carries the pid, tag and fd information.
 * The original fd information is stored in low bit of val.
 * The pid and tag information is stored in the high bit of val.
 * For ease of understanding, let's give an example where
 * the following information is represented in 32-bit binary format
 *
 *  val       00000000 01010101 00000001 10001010
 *  fd        00000000 00000000 00000000 10001010
 *  pid       00000000 00000000 00000000 01010101
 *  tag       00000000 00000000 00000000 00000001
 *
 * In this function, we also check if the pid and tag information is correct.
 * If there is an error, it will panic.
 *
 * Input Parameters:
 *   val - this val carrying pid, tag and original fd information
 *
 * Returned Value: none
 *
 ****************************************************************************/

int fdcheck_restore(int val)
{
  int pid_expect;
  int ppid_now;
  int pid_now;

  if (val <= 2)
    {
      return val;
    }

  pid_expect = (val >> PID_SHIFT) & PID_MASK;
  pid_now = _SCHED_GETPID() & PID_MASK;
  ppid_now = _SCHED_GETPPID() & PID_MASK;
  if (pid_expect != pid_now && pid_expect != ppid_now && pid_expect != 0)
    {
      ferr("pid_expect %d pid_now %d ppid_now %d\n",
           pid_expect, pid_now, ppid_now);
      PANIC();
    }

  if (pid_expect != 0)
    {
      uint8_t tag_store;
      int ret = ioctl(val & FD_MASK, FIOC_GETTAG_FDCHECK, &tag_store);
      if (ret >= 0)
        {
          uint8_t tag_expect = (val >> TAG_SHIFT) & TAG_MASK;
          if (tag_expect != tag_store)
            {
              ferr("tag_expect 0x%x tag_store 0x%x\n",
                   tag_expect, tag_store);
              PANIC();
            }
        }
    }

  return val & FD_MASK;
}

/****************************************************************************
 * Name: fdcheck_protect
 *
 * Description: Obtain the combined value of fd, pid and tag
 *
 * the return value carries the pid, tag and fd information.
 * The original fd information is stored in low bit of val.
 * The pid and tag information is stored in high bit of val.
 * For ease of understanding, let's give an example where
 * the following information is represented in 32-bit binary format
 *
 *  fd        00000000 00000000 00000000 10001010
 *  pid       00000000 00000000 00000000 01010101
 *  tag       00000000 00000000 00000000 00000001
 *  val       00000000 01010101 00000001 10001010
 *
 * Input Parameters:
 *   fd - original fd
 *
 * Returned Value: the combined value of fd and pid
 *
 ****************************************************************************/

int fdcheck_protect(int fd)
{
  int protect_fd;
  uint8_t tag;
  int ret;

  if (fd <= 2)
    {
      return fd;
    }

  protect_fd = fd & FD_MASK;
  protect_fd |= (_SCHED_GETPID() & PID_MASK) << PID_SHIFT;

  ret = ioctl(fd, FIOC_GETTAG_FDCHECK, &tag);
  DEBUGASSERT(ret >= 0);
  if (tag == 0)
    {
      irqstate_t flags = spin_lock_irqsave(&g_fdcheck_lock);
      if ((++g_fdcheck_tag & TAG_MASK) == 0)
        {
          ++g_fdcheck_tag;
        }

      g_fdcheck_tag &= TAG_MASK;
      protect_fd |= g_fdcheck_tag << TAG_SHIFT;
      ret = ioctl(fd, FIOC_SETTAG_FDCHECK, &g_fdcheck_tag);
      DEBUGASSERT(ret == 0);
      spin_unlock_irqrestore(&g_fdcheck_lock, flags);
    }
  else
    {
      protect_fd |= (tag & TAG_MASK) << TAG_SHIFT;
    }

  return protect_fd;
}

#endif

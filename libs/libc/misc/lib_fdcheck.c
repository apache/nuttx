/****************************************************************************
 * libs/libc/misc/lib_fdcheck.c
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

#define TAG_SHIFT 0
#define TAG_BITS  8
#define TAG_MASK  ((1 << TAG_BITS) - 1)

#define FD_SHIFT  (TAG_SHIFT + TAG_BITS)
#define FD_BITS   LOG2_CEIL(OPEN_MAX)
#define FD_MASK   ((1 << FD_BITS) - 1)

static_assert(FD_BITS <= TAG_BITS, "FD_BITS is too long");

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_fdcheck_lock = SP_UNLOCKED;
static uint8_t    g_fdcheck_tag = 0;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int fdcheck_restore(int val)
{
  uint8_t tag_store;
  int fd;

  /* If val is a bare fd（0~255）, we should return it directly  */

  fd = (val >> FD_SHIFT) & FD_MASK;
  if (fd == 0 || val < 0)
    {
      return val;
    }

  int ret = ioctl(fd, FIOC_GETTAG_FDCHECK, &tag_store);
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

  return fd;
}

int fdcheck_protect(int fd)
{
  int protect_fd;
  uint8_t tag;
  int ret;

  if (fd <= 2)
    {
      return fd;
    }

  protect_fd = (fd & FD_MASK) << FD_SHIFT;
  ret = ioctl(fd, FIOC_GETTAG_FDCHECK, &tag);
  DEBUGASSERT(ret >= 0);
  if (tag == 0)
    {
      uint8_t fdcheck_tag;

      irqstate_t flags = spin_lock_irqsave(&g_fdcheck_lock);
      if ((++g_fdcheck_tag & TAG_MASK) == 0)
        {
          ++g_fdcheck_tag;
        }

      g_fdcheck_tag &= TAG_MASK;
      protect_fd |= g_fdcheck_tag << TAG_SHIFT;
      fdcheck_tag = g_fdcheck_tag;
      spin_unlock_irqrestore(&g_fdcheck_lock, flags);

      ret = ioctl(fd, FIOC_SETTAG_FDCHECK, &fdcheck_tag);
      DEBUGASSERT(ret == 0);
    }
  else
    {
      protect_fd |= (tag & TAG_MASK) << TAG_SHIFT;
    }

  return protect_fd;
}

#endif

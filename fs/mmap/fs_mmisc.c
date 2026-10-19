/****************************************************************************
 * fs/mmap/fs_mmisc.c
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

#include <sys/mman.h>
#include <errno.h>
#include <unistd.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/mm/map.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* The empty implementation is enough since the paging isn't supported yet. */

int mlock(FAR const void *addr, size_t len)
{
  return 0;
}

int mlockall(int flags)
{
  return 0;
}

int munlock(FAR const void *addr, size_t len)
{
  return 0;
}

int munlockall(void)
{
  return 0;
}

/* Without an address environment the arguments are checked, but there are
 * no page permissions to change.
 */

int mprotect(FAR void *addr, size_t len, int prot)
{
#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_MM_PGALLOC)
  FAR struct addrenv_s *addrenv = nxsched_self()->addrenv_own;
#endif
  uintptr_t start = (uintptr_t)addr;
  long pagesize = sysconf(_SC_PAGESIZE);
  int ret;

  if ((prot & ~(PROT_READ | PROT_WRITE | PROT_EXEC)) != 0 ||
      (pagesize > 0 && start % pagesize != 0))
    {
      ret = -EINVAL;
      goto errout;
    }

  if (len == 0)
    {
      return OK;
    }

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_MM_PGALLOC)
  len = MM_PGALIGNUP(len);
  if (len == 0 || start + len < start || addrenv == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* Hold the mapping list so that munmap() cannot remove pages meanwhile */

  ret = mm_map_lock();
  if (ret < 0)
    {
      goto errout;
    }

  ret = up_addrenv_mprot(&addrenv->addrenv, start, len, prot);
  mm_map_unlock();
  if (ret < 0)
    {
      goto errout;
    }

  if ((prot & PROT_EXEC) != 0)
    {
      up_coherent_dcache(start, len);
    }
#endif

  return OK;

errout:
  set_errno(-ret);
  return ERROR;
}

/* Ignore the advice since there is no alternative strategy to select. */

int posix_madvise(FAR void *addr, size_t len, int advice)
{
  return 0;
}

/****************************************************************************
 * fs/mmap/fs_mmisc.c
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

#include <sys/mman.h>

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

/* The empty implementation is enough since MMU/MPU mayn't exist. */

int mprotect(FAR void *addr, size_t len, int prot)
{
  return 0;
}

/* Ignore the advice since there is no alternative strategy to select. */

int posix_madvise(FAR void *addr, size_t len, int advice)
{
  return 0;
}

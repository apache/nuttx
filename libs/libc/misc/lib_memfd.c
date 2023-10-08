/****************************************************************************
 * libs/libc/misc/lib_memfd.c
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
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#if defined(CONFIG_LIBC_MEMFD_TMPFS) || defined(CONFIG_LIBC_MEMFD_SHMFS)
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LIBC_MEMFD_TMPFS
#  define LIBC_MEM_FD_VFS_PATH CONFIG_LIBC_TMPDIR "/" CONFIG_LIBC_MEM_FD_VFS_PATH
#else
#  define LIBC_MEM_FD_VFS_PATH CONFIG_LIBC_MEM_FD_VFS_PATH
#endif

#define LIBC_MEM_FD_VFS_PATH_FMT LIBC_MEM_FD_VFS_PATH "/%s"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int memfd_create(FAR const char *name, unsigned int flags)
{
#ifdef CONFIG_LIBC_MEMFD_ERROR
  set_errno(ENOSYS);
  return -1;
#else
  char path[PATH_MAX];
  int ret;

  snprintf(path, sizeof(path), LIBC_MEM_FD_VFS_PATH_FMT, name);
#  ifdef CONFIG_LIBC_MEMFD_SHMFS
  ret = shm_open(path, O_RDWR | flags, 0660);
  if (ret >= 0)
    {
      shm_unlink(path);
    }
#  else
  mkdir(LIBC_MEM_FD_VFS_PATH, 0666);
  ret = open(path, O_RDWR | flags, 0660);
  if (ret >= 0)
    {
      unlink(path);
    }
#  endif

  return ret;
#endif
}
#endif
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

#include <sys/memfd.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LIBC_MEM_FD_VFS_PATH \
          CONFIG_LIBC_TMPDIR "/" CONFIG_LIBC_MEM_FD_VFS_PATH "/%s"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int memfd_create(FAR const char *name, unsigned int flags)
{
#ifdef CONFIG_FS_TMPFS
  char path[PATH_MAX];

  snprintf(path, sizeof(path), LIBC_MEM_FD_VFS_PATH, name);
  return open(path, O_RDWR | flags);
#else
  set_errno(ENOSYS);
  return -1;
#endif
}

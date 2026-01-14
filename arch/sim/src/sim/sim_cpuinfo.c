/****************************************************************************
 * arch/sim/src/sim/sim_cpuinfo.c
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

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <nuttx/fs/hostfs.h>

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_CPUINFO)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_show_cpuinfo
 ****************************************************************************/

ssize_t up_show_cpuinfo(char *buf, size_t buf_size, off_t file_off)
{
  int fd;
  off_t ret;

  fd = host_open("/proc/cpuinfo", O_RDONLY, 0644);
  if (fd < 0)
    {
      return fd;
    }

  ret = host_lseek(fd, 0, file_off, SEEK_SET);
  if (ret < 0)
    {
      host_close(fd);
      return ret;
    }

  file_off = host_read(fd, buf, buf_size);

  host_close(fd);

  return file_off;
}
#endif /* CONFIG_FS_PROCFS && !CONFIG_FS_PROCFS_EXCLUDE_CPUINFO */

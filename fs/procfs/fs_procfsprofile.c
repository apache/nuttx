/****************************************************************************
 * fs/procfs/fs_procfsprofile.c
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

#if defined(CONFIG_FS_PROCFS) && defined(CONFIG_FS_PROFILER) && \
    defined(CONFIG_FS_PROCFS_PROFILER)

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/procfs.h>
#include "../vfs/vfs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int profile_open(FAR struct file *filep, FAR const char *relpath,
                        int oflags, mode_t mode)
{
  return OK;
}

static int profile_close(FAR struct file *filep)
{
  return OK;
}

static ssize_t profile_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  char buf[256];
  size_t linesize;

  procfs_snprintf(buf, sizeof(buf),
           "FS Performance Profile:\n"
           "  Reads:  %10" PRId32 " (Total time: %" PRId64 " ns)\n"
           "  Writes: %10" PRId32 " (Total time: %" PRId64 " ns)\n"
           "  Opens:  %10" PRId32 " (Total time: %" PRId64 " ns)\n"
           "  Closes: %10" PRId32 " (Total time: %" PRId64 " ns)\n",
           atomic_read(&g_fs_profile.reads),
           atomic64_read(&g_fs_profile.total_read_time),
           atomic_read(&g_fs_profile.writes),
           atomic64_read(&g_fs_profile.total_write_time),
           atomic_read(&g_fs_profile.opens),
           atomic64_read(&g_fs_profile.total_open_time),
           atomic_read(&g_fs_profile.closes),
           atomic64_read(&g_fs_profile.total_close_time));

  linesize = strlen(buf);
  return procfs_memcpy(buf, linesize, buffer, buflen, &filep->f_pos);
}

static int profile_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  return OK;
}

static int profile_stat(FAR const char *relpath, FAR struct stat *buf)
{
  memset(buf, 0, sizeof(struct stat));
  buf->st_mode = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
  return OK;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct procfs_operations g_fsprofile_operations =
{
  profile_open,       /* open */
  profile_close,      /* close */
  profile_read,       /* read */
  NULL,               /* write */
  NULL,               /* poll */
  profile_dup,        /* dup */
  NULL,               /* opendir */
  NULL,               /* closedir */
  NULL,               /* readdir */
  NULL,               /* rewinddir */
  profile_stat        /* stat */
};

#endif


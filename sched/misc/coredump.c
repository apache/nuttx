/****************************************************************************
 * sched/misc/coredump.c
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

#include <nuttx/binfmt/binfmt.h>
#include <nuttx/coredump.h>
#include <syslog.h>
#include <debug.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_BOARD_COREDUMP_COMPRESSION
static struct lib_lzfoutstream_s  g_lzfstream;
#endif

#ifdef CONFIG_BOARD_COREDUMP_SYSLOG
static struct lib_syslogstream_s  g_syslogstream;
static struct lib_hexdumpstream_s g_hexstream;
#endif

#ifdef CONFIG_BOARD_COREDUMP_BLKDEV
static struct lib_blkoutstream_s  g_blockstream;
static unsigned char *g_blockinfo;
#endif

static struct memory_region_s *g_regions;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: coredump_dump_syslog
 *
 * Description:
 *   Put coredump to block device.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_COREDUMP_SYSLOG
static void coredump_dump_syslog(pid_t pid)
{
  FAR void *stream;
  int logmask;

  logmask = setlogmask(LOG_ALERT);

  _alert("Start coredump:\n");

  /* Initialize hex output stream */

  lib_syslogstream(&g_syslogstream, LOG_EMERG);
  stream = &g_syslogstream;
  lib_hexdumpstream(&g_hexstream, stream);
  stream = &g_hexstream;
#  ifdef CONFIG_BOARD_COREDUMP_COMPRESSION

  /* Initialize LZF compression stream */

  lib_lzfoutstream(&g_lzfstream, stream);
  stream = &g_lzfstream;
#  endif

  /* Do core dump */

  core_dump(g_regions, stream, pid);

#  ifdef CONFIG_BOARD_COREDUMP_COMPRESSION
  _alert("Finish coredump (Compression Enabled).\n");
#  else
  _alert("Finish coredump.\n");
#  endif

  setlogmask(logmask);
}
#endif

/****************************************************************************
 * Name: coredump_dump_blkdev
 *
 * Description:
 *   Save coredump to block device.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_COREDUMP_BLKDEV
static void coredump_dump_blkdev(pid_t pid)
{
  FAR void *stream = &g_blockstream;
  FAR struct coredump_info_s *info;
  blkcnt_t nsectors;
  int ret;

  if (g_blockstream.inode == NULL)
    {
      _alert("Coredump device not found\n");
      return;
    }

  nsectors = (sizeof(struct coredump_info_s) +
              g_blockstream.geo.geo_sectorsize - 1) /
             g_blockstream.geo.geo_sectorsize;

  ret = g_blockstream.inode->u.i_bops->read(g_blockstream.inode,
           g_blockinfo, g_blockstream.geo.geo_nsectors - nsectors, nsectors);
  if (ret < 0)
    {
      _alert("Coredump information read fail\n");
      return;
    }

  info = (FAR struct coredump_info_s *)g_blockinfo;

#ifdef CONFIG_BOARD_COREDUMP_COMPRESSION
  lib_lzfoutstream(&g_lzfstream,
                   (FAR struct lib_outstream_s *)&g_blockstream);
  stream = &g_lzfstream;
#endif

  ret = core_dump(g_regions, stream, pid);
  if (ret < 0)
    {
      _alert("Coredump fail\n");
      return;
    }

  info->magic = COREDUMP_MAGIC;
  info->size  = g_blockstream.common.nput;
  info->time  = time(NULL);
  uname(&info->name);
  ret = g_blockstream.inode->u.i_bops->write(g_blockstream.inode,
      (FAR void *)info, g_blockstream.geo.geo_nsectors - nsectors, nsectors);
  if (ret < 0)
    {
      _alert("Coredump information write fail\n");
      return;
    }

  _alert("Finish coredump, write %d bytes to %s\n",
         info->size, CONFIG_BOARD_COREDUMP_BLKDEV_PATH);
}
#endif

/****************************************************************************
 * Name: coredump_set_memory_region
 *
 * Description:
 *   Set do coredump memory region.
 *
 ****************************************************************************/

int coredump_set_memory_region(FAR struct memory_region_s *region)
{
  /* Not free g_regions, because allow call this fun when crash */

  g_regions = region;
  return 0;
}

/****************************************************************************
 * Name: coredump_initialize
 *
 * Description:
 *   Initialize the coredump facility.  Called once and only from
 *   nx_start_application.
 *
 ****************************************************************************/

int coredump_initialize(void)
{
  blkcnt_t nsectors;
  int ret = 0;

  if (CONFIG_BOARD_MEMORY_RANGE[0] != '\0')
    {
      coredump_set_memory_region(
         alloc_memory_region(CONFIG_BOARD_MEMORY_RANGE));
      if (g_regions == NULL)
        {
          _alert("Coredump memory region alloc fail\n");
          return -ENOMEM;
        }
    }

#ifdef CONFIG_BOARD_COREDUMP_BLKDEV
  ret = lib_blkoutstream_open(&g_blockstream,
                              CONFIG_BOARD_COREDUMP_BLKDEV_PATH);
  if (ret < 0)
    {
      _alert("%s Coredump device not found\n",
             CONFIG_BOARD_COREDUMP_BLKDEV_PATH);
      free_memory_region(g_regions);
      g_regions = NULL;
      return ret;
    }

  nsectors = (sizeof(struct coredump_info_s) +
              g_blockstream.geo.geo_sectorsize - 1) /
             g_blockstream.geo.geo_sectorsize;

  g_blockinfo = kmm_malloc(g_blockstream.geo.geo_sectorsize * nsectors);
  if (g_blockinfo == NULL)
    {
      _alert("Coredump device memory alloc fail\n");
      free_memory_region(g_regions);
      g_regions = NULL;
      lib_blkoutstream_close(&g_blockstream);
      return -ENOMEM;
    }
#endif

  UNUSED(nsectors);
  return ret;
}

/****************************************************************************
 * Name: coredump_dump
 *
 * Description:
 *   Do coredump of the task specified by pid.
 *
 * Input Parameters:
 *   pid - The task/thread ID of the thread to dump
 *
 ****************************************************************************/

void coredump_dump(pid_t pid)
{
#ifdef CONFIG_BOARD_COREDUMP_SYSLOG
  coredump_dump_syslog(pid);
#endif

#ifdef CONFIG_BOARD_COREDUMP_BLKDEV
  coredump_dump_blkdev(pid);
#endif
}

/****************************************************************************
 * boards/arm64/imx9/imx95-a55-evk/src/imx9_bringup.c
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

#include <sys/types.h>
#include <stdio.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/partition.h>

#include "imx9_dma_alloc.h"

#include "imx95-evk.h"

#if defined(CONFIG_GPT_PARTITION) || defined(CONFIG_MBR_PARTITION)
static void mmcsd_partition_handler(FAR struct partition_s *part,
                                    FAR void *arg)
{
  char devname[32];

  snprintf(devname, sizeof(devname), "/dev/mmcsd0p%zu", part->index);
  register_blockpartition(devname, 0660, "/dev/mmcsd0",
                          part->firstblock, part->nblocks);
  syslog(LOG_INFO, "Registered partition %s (start=%lu nblocks=%lu)\n",
         devname, (unsigned long)part->firstblock,
         (unsigned long)part->nblocks);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int imx9_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_IMX9_DMA_ALLOC
  /* Initialize the DMA memory allocator */

  ret = imx9_dma_alloc_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed initialize DMA allocator: %d\n", ret);
    }
#endif

#ifdef CONFIG_MMCSD
  ret = imx9_usdhc_init();

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to init MMCSD driver: %d\n", ret);
    }

#if defined(CONFIG_GPT_PARTITION) || defined(CONFIG_MBR_PARTITION)
  else
    {
      ret = parse_block_partition("/dev/mmcsd0",
                                  mmcsd_partition_handler, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "Failed to parse partition table: %d\n", ret);
        }
    }
#endif
#endif

  UNUSED(ret);
  return OK;
}

/****************************************************************************
 * include/nuttx/fs/partition.h
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

#ifndef __INCLUDE_NUTTX_FS_PARTITION_H
#define __INCLUDE_NUTTX_FS_PARTITION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <limits.h>
#include <sys/types.h>

#ifndef CONFIG_DISABLE_MOUNTPOINT

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct partition_s
{
  char   name[NAME_MAX + 1];
  size_t index;
  size_t firstblock;
  size_t nblocks;
  size_t blocksize;
};

typedef CODE void
  (*partition_handler_t)(FAR struct partition_s *part, FAR void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: parse_block_partition
 *
 * Description:
 *   parse the partition table on a block device.
 *
 * Input Parameters:
 *   path    - The block device to be parsed
 *   handler - The function to be called for each found partition
 *   arg     - A caller provided value to return with the handler
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on a failure
 *
 ****************************************************************************/

int parse_block_partition(FAR const char *path,
                          partition_handler_t handler,
                          FAR void *arg);

/****************************************************************************
 * Name: parse_mtd_partition
 *
 * Description:
 *   parse the partition table on a mtd device.
 *
 * Input Parameters:
 *   mtd     - The MTD device to be parsed
 *   handler - The function to be called for each found partition
 *   arg     - A caller provided value to return with the handler
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on a failure
 *
 ****************************************************************************/

struct mtd_dev_s;
int parse_mtd_partition(FAR struct mtd_dev_s *mtd,
                        partition_handler_t handler,
                        FAR void *arg);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_DISABLE_MOUNTPOINT */

#endif /* __INCLUDE_NUTTX_FS_PARTITION_H */

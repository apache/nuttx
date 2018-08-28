/****************************************************************************
 * include/nuttx/fs/partition.h
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiangg@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
  char   name[NAME_MAX+1];
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

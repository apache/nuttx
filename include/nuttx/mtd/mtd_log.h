/****************************************************************************
 * include/nuttx/mtd/mtd_log.h
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

#ifndef __INCLUDE_NUTTX_MTD_MTD_LOG_H
#define __INCLUDE_NUTTX_MTD_MTD_LOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* IOCTL Commands to store and read configuration for user application.
 *
 * MTDLOGIOC_LOG_COUNT - Get the total number of log entries.
 *
 *   ioctl argument:  Pointer to a unsigned long variable to receive the
 *                    the total number of log entries.
 *
 * MTDLOGIOC_LOG_SEEK_CUR - Seek to the relative log entry.
 *
 *   ioctl argument:  Integer offset to seek to the relative log entry.
 *
 * MTDLOGIOC_LOG_SEEK_SET - Seek to the absolute log entry.
 *
 *   ioctl argument:  Integer offset to seek to the absolute log entry.
 *
 * MTDLOGIOC_GRP_COUNT - Get the total number of blk groups.
 *
 *   ioctl argument:  Pointer to a unsigned long variable to receive the
 *                    the total number of blk groups.
 *
 * MTDLOGIOC_GRP_SEEK_CUR - Seek to the relative blk group.
 *
 *   ioctl argument:  Integer offset to seek to the relative blk group.
 *
 * MTDLOGIOC_GRP_SEEK_SET - Seek to the absolute blk group.
 *
 *   ioctl argument:  Integer offset to seek to the absolute blk group.
 *
 */

#define MTDLOGIOC_GET_ENTRYINFO   _MTDLOGIOC(1)
#define MTDLOGIOC_GET_BLOCKINFO   _MTDLOGIOC(2)
#define MTDLOGIOC_STATUS          _MTDLOGIOC(3)

#define MTDLOGIOC_LOG_COUNT       _MTDLOGIOC(4)
#define MTDLOGIOC_LOG_SEEK_CUR    _MTDLOGIOC(5)
#define MTDLOGIOC_LOG_SEEK_SET    _MTDLOGIOC(6)

#define MTDLOGIOC_GRP_COUNT       _MTDLOGIOC(7)
#define MTDLOGIOC_GRP_SEEK_CUR    _MTDLOGIOC(8)
#define MTDLOGIOC_GRP_SEEK_SET    _MTDLOGIOC(9)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mtdlog_blkinfo_s
{
  uint16_t blkseq;  /* The block sequence number of current block */
  uint16_t blkgrp;  /* The block group number of current block */
  uint8_t  valid;   /* Indicates whether the current blkinfo is valid */
};

struct mtdlog_loginfo_s
{
  uint32_t offset;  /* The offset of the log in the device */
  uint32_t length;  /* The length of the log entry */
  uint8_t  valid;   /* Indicates whether the current loginfo is valid */
};

struct mtdlog_status_s
{
  uint32_t logsize;  /* total size of the log */
  uint32_t logcnt;   /* total number of log entries */
  uint32_t logoff;   /* offset of the log entry */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mtdlog_register
 *
 * Description:
 *   This function binds an instance of an MTD device to the path specified
 *   device.
 *
 *   When this function is called, the MTD device pass in should already
 *   be initialized appropriately to access the physical device or partition.
 *
 * Input Parameters:
 *   path - Path name of the file backing the MTD device
 *   mtd - Pointer to the MTD device to bind with the path device
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mtdlog_register(FAR const char *path, FAR struct mtd_dev_s *mtd);

/****************************************************************************
 * Name: mtdlog_unregister
 *
 * Description:
 *   This function unregisters path device.
 *
 * Input Parameters:
 *   path - Path name of the file backing the MTD device
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mtdlog_unregister(FAR const char *path);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_MTD_MTD_LOG_H */

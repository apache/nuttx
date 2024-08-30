/****************************************************************************
 * include/nuttx/virtio/virtio-blk.h
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

#ifndef __INCLUDE_NUTTX_VIRTIO_VIRTIO_BLK_H
#define __INCLUDE_NUTTX_VIRTIO_VIRTIO_BLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <limits.h>
#include <stdint.h>

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_BLK_REQ_HEADER_SIZE  sizeof(struct virtio_blk_req_s)
#define VIRTIO_BLK_RESP_HEADER_SIZE sizeof(struct virtio_blk_resp_s)

/* Block feature bits */

#define VIRTIO_BLK_F_RO             5  /* Disk is read-only */
#define VIRTIO_BLK_F_BLK_SIZE       6  /* Block size of disk is available */
#define VIRTIO_BLK_F_FLUSH          9  /* Cache flush command support */

/* Block request type */

#define VIRTIO_BLK_T_IN             0  /* READ */
#define VIRTIO_BLK_T_OUT            1  /* WRITE */
#define VIRTIO_BLK_T_FLUSH          4  /* FLUSH */

/* Block request return status */

#define VIRTIO_BLK_S_OK             0
#define VIRTIO_BLK_S_IOERR          1
#define VIRTIO_BLK_S_UNSUPP         2

/* Block device sector size */

#define VIRTIO_BLK_SECTOR_BITS      9
#define VIRTIO_BLK_SECTOR_SIZE      (1UL << VIRTIO_BLK_SECTOR_BITS)

#define VIRTIO_BLK_PATH_SIZE        PATH_MAX

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Block request out header */

begin_packed_struct struct virtio_blk_req_s
{
  uint32_t type;
  uint32_t reserved;
  uint64_t sector;
} end_packed_struct;

/* Block request in header */

begin_packed_struct struct virtio_blk_resp_s
{
  uint8_t status;
} end_packed_struct;

begin_packed_struct struct virtio_blk_config_s
{
  uint64_t capacity;
  uint32_t size_max;
  uint32_t seg_max;
  uint16_t cylinders; /* block geometry */
  uint8_t  heads;     /* block geometry */
  uint8_t  sectors;   /* block geometry */
  uint32_t blk_size;
  uint8_t  physical_block_exp;
  uint8_t  alignment_offset;
  uint16_t min_io_size;
  uint32_t opt_io_size;
  uint8_t  writeback;
  uint8_t  unused0;
  uint16_t num_queues;
  uint32_t max_discard_sectors;
  uint32_t max_discard_seg;
  uint32_t discard_sector_alignment;
  uint32_t max_write_zeroes_sectors;
  uint32_t max_write_zeroes_seg;
  uint8_t  write_zeroes_may_unmap;
  uint8_t  unused1[3];
  uint32_t max_secure_erase_sectors;
  uint32_t max_secure_erase_seg;
  uint32_t secure_erase_sector_alignment;
} end_packed_struct;

begin_packed_struct struct vhost_blk_config_s
{
  struct virtio_blk_config_s config;
  char path[VIRTIO_BLK_PATH_SIZE];
} end_packed_struct;

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
 * Name: register_virtio_pci_driver
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIRTIO_VIRTIO_BLK_H */

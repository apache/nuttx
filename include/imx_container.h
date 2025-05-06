/****************************************************************************
 * include/imx_container.h
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

#ifndef __INCLUDE_IMX_CONTAINER_H
#define __INCLUDE_IMX_CONTAINER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IV_MAX_LEN                 32
#define HASH_MAX_LEN               64

#define CONTAINER_HDR_ALIGNMENT    0x400
#define CONTAINER_HDR_EMMC_OFFSET  0
#define CONTAINER_HDR_MMCSD_OFFSET SZ_32K
#define CONTAINER_HDR_QSPI_OFFSET  SZ_4K
#define CONTAINER_HDR_NAND_OFFSET  SZ_128M

#define CONTAINER_HDR_TAG          0x87
#define CONTAINER_HDR_VERSION      0

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* i.MX AHAB container header. */

begin_packed_struct struct container_hdr
{
  uint8_t  version;
  uint8_t  length_lsb;
  uint8_t  length_msb;
  uint8_t  tag;
  uint32_t flags;
  uint16_t sw_version;
  uint8_t  fuse_version;
  uint8_t  num_images;
  uint16_t sig_blk_offset;
  uint16_t reserved;
} end_packed_struct;

/* i.MX AHAB container, image header. */

begin_packed_struct struct boot_img_hdr
{
  uint32_t offset;
  uint32_t size;
  uint64_t dst;
  uint64_t entry;
  uint32_t hab_flags;
  uint32_t meta;
  uint8_t  hash[HASH_MAX_LEN];
  uint8_t  iv[IV_MAX_LEN];
} end_packed_struct;

/* i.MX AHAB container, signature block header. */

begin_packed_struct struct signature_block_hdr
{
  uint8_t  version;
  uint8_t  length_lsb;
  uint8_t  length_msb;
  uint8_t  tag;
  uint16_t srk_table_offset;
  uint16_t cert_offset;
  uint16_t blob_offset;
  uint16_t signature_offset;
  uint32_t reserved;
} end_packed_struct;

/* Header used for the generation of an encrypted key blob from ELE. */

begin_packed_struct struct generate_key_blob_hdr
{
  uint8_t version;
  uint8_t length_lsb;
  uint8_t length_msb;
  uint8_t tag;
  uint8_t flags;
  uint8_t size;
  uint8_t algorithm;
  uint8_t mode;
} end_packed_struct;

/****************************************************************************
 * Name: imx_valid_container_hdr
 *
 * Description:
 *   Simple validation of the container header.
 *
 * Returned Value:
 *   true for Success, otherwise false
 *
 ****************************************************************************/

static inline bool imx_valid_container_hdr(struct container_hdr *container)
{
    return container->tag == CONTAINER_HDR_TAG &&
           container->version == CONTAINER_HDR_VERSION;
}
#endif /* __INCLUDE_IMX_CONTAINER_H */

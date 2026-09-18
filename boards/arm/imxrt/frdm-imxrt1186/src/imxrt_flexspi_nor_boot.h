/****************************************************************************
 * boards/arm/imxrt/frdm-imxrt1186/src/imxrt_flexspi_nor_boot.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __BOARDS_ARM_IMXRT_FRDM_IMXRT1186_SRC_IMXRT_FLEXSPI_NOR_BOOT_H
#define __BOARDS_ARM_IMXRT_FRDM_IMXRT1186_SRC_IMXRT_FLEXSPI_NOR_BOOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMXRT_CONTAINER_VERSION        0x00
#define IMXRT_CONTAINER_TAG            0x87
#define IMXRT_CONTAINER_FLAGS          0x00000000
#define IMXRT_CONTAINER_SW_VERSION     0
#define IMXRT_CONTAINER_FUSE_VERSION   0
#define IMXRT_CONTAINER_IMAGE_COUNT    1

/* Non-encrypted, SHA-512, Cortex-M33, executable. */

#define IMXRT_CONTAINER_IMAGE_FLAGS    0x00000213

#define IMXRT_SIGNATURE_BLOCK_VERSION  0x00
#define IMXRT_SIGNATURE_BLOCK_TAG      0x90

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct imxrt_container_header_s
{
  uint8_t version;
  uint16_t length;
  uint8_t tag;
  uint32_t flags;
  uint16_t sw_version;
  uint8_t fuse_version;
  uint8_t image_count;
  uint16_t signature_block_offset;
  uint16_t reserved;
} end_packed_struct;

begin_packed_struct struct imxrt_container_image_s
{
  uint32_t offset;
  uint32_t size;
  uint32_t load_address;
  uint32_t reserved1;
  uint32_t entry;
  uint32_t reserved2;
  uint32_t flags;
  uint32_t metadata;
  uint8_t hash[64];
  uint8_t iv[32];
} end_packed_struct;

begin_packed_struct struct imxrt_signature_block_s
{
  uint8_t version;
  uint16_t length;
  uint8_t tag;
  uint16_t certificate_offset;
  uint16_t srk_table_offset;
  uint16_t signature_offset;
  uint16_t blob_offset;
  uint32_t reserved;
} end_packed_struct;

begin_packed_struct struct imxrt_boot_container_s
{
  struct imxrt_container_header_s header;
  struct imxrt_container_image_s image[IMXRT_CONTAINER_IMAGE_COUNT];
  struct imxrt_signature_block_s signature_block;
} end_packed_struct;

#endif /* __BOARDS_ARM_IMXRT_FRDM_IMXRT1186_SRC_IMXRT_FLEXSPI_NOR_BOOT_H */

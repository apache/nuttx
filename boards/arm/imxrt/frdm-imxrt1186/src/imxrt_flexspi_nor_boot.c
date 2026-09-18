/****************************************************************************
 * boards/arm/imxrt/frdm-imxrt1186/src/imxrt_flexspi_nor_boot.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include "imxrt_flexspi_nor_boot.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

extern const uint8_t __container_image_offset[];
extern const uint8_t __container_image_size[];
extern const uint8_t __container_load_address[];
extern const uint8_t __container_entry[];

/****************************************************************************
 * Public Data
 ****************************************************************************/

locate_data(".boot_hdr.container")
const struct imxrt_boot_container_s g_boot_container =
{
  .header =
    {
      .version = IMXRT_CONTAINER_VERSION,
      .length = sizeof(struct imxrt_boot_container_s),
      .tag = IMXRT_CONTAINER_TAG,
      .flags = IMXRT_CONTAINER_FLAGS,
      .sw_version = IMXRT_CONTAINER_SW_VERSION,
      .fuse_version = IMXRT_CONTAINER_FUSE_VERSION,
      .image_count = IMXRT_CONTAINER_IMAGE_COUNT,
      .signature_block_offset =
        sizeof(struct imxrt_container_header_s) +
        sizeof(struct imxrt_container_image_s),
      .reserved = 0
    },
  .image =
    {
      {
        .offset = (uint32_t)(uintptr_t)__container_image_offset,
        .size = (uint32_t)(uintptr_t)__container_image_size,
        .load_address = (uint32_t)(uintptr_t)__container_load_address,
        .reserved1 = 0,
        .entry = (uint32_t)(uintptr_t)__container_entry,
        .reserved2 = 0,
        .flags = IMXRT_CONTAINER_IMAGE_FLAGS,
        .metadata = 0,
        .hash =
          {
            0
          },
        .iv =
          {
            0
          }
      }
    },
  .signature_block =
    {
      .version = IMXRT_SIGNATURE_BLOCK_VERSION,
      .length = sizeof(struct imxrt_signature_block_s),
      .tag = IMXRT_SIGNATURE_BLOCK_TAG,
      .certificate_offset = 0,
      .srk_table_offset = 0,
      .signature_offset = 0,
      .blob_offset = 0,
      .reserved = 0
    }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* None */

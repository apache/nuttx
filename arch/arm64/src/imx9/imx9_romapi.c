/****************************************************************************
 * arch/arm64/src/imx9/imx9_romapi.c
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

#include <arch/imx9/imx9_romapi.h>
#include <nuttx/nuttx.h>
#include <imx_container.h>
#include <debug.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* i.MX9 ROM API structure. */

struct romapi_s
{
  uint16_t ver;
  uint16_t tag;
  uint32_t reserved1;
  uint32_t (*load_image)(uint8_t *dest, uint32_t offset, uint32_t size,
                         uint32_t xor);
  uint32_t (*query_boot_infor)(uint32_t infor_type, uint32_t *infor,
                               uint32_t xor);
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Physical address of the ROM api. */

static struct romapi_s *imx9_romapi_s = (struct romapi_s *)0x1980;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t imx9_romapi_load_image(uint8_t *dest, uint32_t offset,
                                uint32_t size)
{
  uint32_t xor;
  uint32_t ret;

  xor = (uintptr_t)dest ^ offset ^ size;
  ret = imx9_romapi_s->load_image(dest, offset, size, xor);

  if (ret != ROMAPI_OK)
    {
      _err("[%s]: Couldn't download image.", __func__);
    }

  return ret;
}

uint32_t imx9_romapi_query_boot_infor(uint32_t infor_type, uint32_t *infor)
{
  uint32_t ret;
  uint32_t xor;

  xor = infor_type ^ (uintptr_t)infor;
  ret = imx9_romapi_s->query_boot_infor(infor_type, infor, xor);

  if (ret != ROMAPI_OK)
    {
      _err("[%s]: Couldn't query boot info.", __func__);
    }

  return ret;
}

enum imx9_romapi_bootdev_e imx9_romapi_get_boot_device(void)
{
  int ret;
  uint32_t boot;
  enum imx9_romapi_bootdev_e boot_type;
  uint32_t xor = (uintptr_t)&boot ^ QUERY_BOOT_DEV;

  ret = imx9_romapi_s->query_boot_infor(QUERY_BOOT_DEV, &boot, xor);

  if (ret != ROMAPI_OK)
    {
      return BOOTDEV_INVALID;
    }

  boot_type = boot >> 16;

  switch (boot_type)
    {
      case BOOTDEV_SD:
        break;
      case BOOTDEV_MMC:
        break;
      case BOOTDEV_NAND:
        break;
      case BOOTDEV_FLEXSPI_NOR:
        break;
      case BOOTDEV_SPI_NOR:
        break;
      case BOOTDEV_FLEXSPI_NAND:
        break;
      case BOOTDEV_USB:
        break;
      case BOOTDEV_MEM_DEV:
        break;
      default:
        _err("[%s]: Invalid boot device.\n", __func__);
        boot_type = BOOTDEV_INVALID;
        break;
    }

  return boot_type;
}

int64_t imx9_romapi_get_seq_cntr_base(uint32_t image_offset,
                                      uint32_t pagesize,
                                      uint8_t cntr_idx)
{
  uint32_t size;
  uint32_t ret;
  uint16_t cntr_size;
  struct container_hdr *hdr;

  size = ALIGN_UP(sizeof(struct container_hdr), pagesize);
  hdr = (struct container_hdr *)(CONFIG_IMX_AHAB_CNTR_ADDR);

  if (CONTAINER_HDR_ALIGNMENT % pagesize != 0)
    {
      _err("[%s]: Unsupported pagesize.\n", __func__);
      return -1;
    }

  for (uint8_t i = 0; i < cntr_idx; i++)
    {
      ret = imx9_romapi_load_image((uint8_t *)CONFIG_IMX_AHAB_CNTR_ADDR,
                                       image_offset, size);
      if (ret != ROMAPI_OK)
        {
          _err("[%s]: Failure at iteration %hhu\n", __func__, i);
          return -1;
        }

      cntr_size = hdr->length_lsb + (hdr->length_msb << 8);
      image_offset = ALIGN_UP((cntr_size + image_offset),
                              CONTAINER_HDR_ALIGNMENT);
    }

  return image_offset;
}

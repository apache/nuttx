/****************************************************************************
 * arch/arm64/src/imx9/imx9_ahab.c
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

#include <stdbool.h>
#include <debug.h>
#include <stdio.h>
#include <nuttx/nuttx.h>
#include <nuttx/arch.h>
#include "imx9_ele.h"
#include <arch/imx9/imx9_romapi.h>
#include <arch/imx9/imx9_ahab.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct boot_img_hdr *ahab_read_auth_image(struct container_hdr *container,
                                          int image_index,
                                          unsigned long container_offset)
{
  struct boot_img_hdr *images;
  unsigned long offset;
  unsigned long size;

  _info("Image index is %d number of container images are %hhu\n",
        image_index, container->num_images);
  if (image_index > container->num_images)
    {
      _info("[%s]: Invalid image number\n", __func__);
      return NULL;
    }

  images = (struct boot_img_hdr *)((uint8_t *)container +
           sizeof(struct container_hdr));

  uint32_t ret;
  uint32_t pagesize;
  ret = imx9_romapi_query_boot_infor(QUERY_PAGE_SZ, &pagesize);
  if (ret != ROMAPI_OK)
    {
      _err("[%s]: Failed to query pagesize\n", __func__);
      return NULL;
    }

  if (!IS_ALIGNED(images[image_index].offset, pagesize))
    {
      _err("[%s]: image%d offset not aligned to %u\n", __func__,
           image_index, pagesize);
      return NULL;
    }

  size = ALIGN_UP(images[image_index].size, pagesize);

  offset = images[image_index].offset + container_offset;

  _info("[%s]: container: %p offset: %lu size: %lu\n", __func__,
        container, offset, size);

  imx9_romapi_load_image((uint8_t *)images[image_index].dst, offset,
                          size);

  /* Flush the cache so ELE can read from RAM the updated values. */

  up_flush_dcache((unsigned long)images[image_index].dst,
                  images[image_index].dst + size);

  uint32_t respo;
  if (imx9_ele_verify_image(image_index, &respo))
    {
      _err("[%s]: Failed with response %x\n", __func__, respo);
      return NULL;
    }
  else
    {
      _info("[%s]: Image successfully verified.", __func__);
    }

  return &images[image_index];
}

int ahab_auth_release(void)
{
  int err;
  uint32_t resp;

  err = imx9_ele_release_container(&resp);
  if (err)
    {
      _err("[%s]: ELE error 0x%x\n", __func__, resp);
    }
  else
    {
      _info("[%s]: Container successfully released\n", __func__);
    }

  return err;
}

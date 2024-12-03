/****************************************************************************
 * drivers/coresight/coresight_tmc_core.c
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

#include <errno.h>
#include <debug.h>
#include <nuttx/bits.h>
#include <nuttx/kmalloc.h>

#include <nuttx/coresight/coresight_tmc.h>

#include "coresight_common.h"
#include "coresight_tmc_core.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmc_etf_get_memwidth
 ****************************************************************************/

static enum tmc_mem_intf_width_e tmc_etf_get_memwidth(uint32_t devid)
{
  /* Indicate the minimum alignemnt for RRR/RURP/RWP/DBA etc registers. */

  switch (BMVAL(devid, 8, 10))
  {
    case 0x2:
      return TMC_MEM_INTF_WIDTH_32BITS;
    case 0x3:
      return TMC_MEM_INTF_WIDTH_64BITS;
    case 0x4:
      return TMC_MEM_INTF_WIDTH_128BITS;
    case 0x5:
      return TMC_MEM_INTF_WIDTH_256BITS;
    default:
      return 0;
  }
}

/****************************************************************************
 * Name: tmc_etr_get_memwidth
 ****************************************************************************/

static enum tmc_mem_intf_width_e tmc_etr_get_memwidth(uint32_t devid)
{
  uint32_t val = (BMVAL(devid, 14, 15) << 3) | BMVAL(devid, 8, 10);

  /* Indicate the minimum alignemnt for RRR/RURP/RWP/DBA etc registers. */

  switch (val)
  {
    case 0x5:
      return TMC_MEM_INTF_WIDTH_32BITS;
    case 0x6:
      return TMC_MEM_INTF_WIDTH_64BITS;
    case 0x7:
      return TMC_MEM_INTF_WIDTH_128BITS;
    case 0x8:
      return TMC_MEM_INTF_WIDTH_256BITS;
    default:
      return 0;
  }
}

/****************************************************************************
 * Name: tmc_init_arch_data
 ****************************************************************************/

static void tmc_init_arch_data(FAR struct coresight_tmc_dev_s *tmcdev,
                               FAR const struct coresight_desc_s *desc)
{
  uint32_t devid;

  coresight_unlock(desc->addr);
  devid = coresight_get32(desc->addr + CORESIGHT_DEVID);
  tmcdev->config_type = BMVAL(devid, 6, 7);
  if (tmcdev->config_type == TMC_CONFIG_TYPE_ETR)
    {
      tmcdev->size = desc->buffer_size;
      tmcdev->burst_size = desc->burst_size;
      tmcdev->mmwidth = tmc_etr_get_memwidth(devid);
    }
  else
    {
      tmcdev->size = coresight_get32(desc->addr + TMC_RSZ) * 4;
      tmcdev->mmwidth = tmc_etf_get_memwidth(devid);
    }

  coresight_lock(desc->addr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmc_register
 *
 * Description:
 *   Register a TMC devices.
 *
 * Input Parameters:
 *   desc  - A description of this coresight device.
 *
 * Returned Value:
 *   Pointer to a TMC device on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct coresight_tmc_dev_s *
tmc_register(FAR const struct coresight_desc_s *desc)
{
  FAR struct coresight_tmc_dev_s *tmcdev;
  int ret = -EINVAL;

  tmcdev = kmm_zalloc(sizeof(struct coresight_tmc_dev_s));
  if (tmcdev == NULL)
    {
      cserr("%s:malloc failed!\n", desc->name);
      return NULL;
    }

  tmc_init_arch_data(tmcdev, desc);
  tmcdev->caps = desc->caps;

  switch (tmcdev->config_type)
    {
      case TMC_CONFIG_TYPE_ETB:
      case TMC_CONFIG_TYPE_ETF:
        ret = tmc_etf_register(tmcdev, desc);
        break;

      case TMC_CONFIG_TYPE_ETR:
        ret = tmc_etr_register(tmcdev, desc);
        break;

      default:
        cserr("config type error\n");
        break;
    }

  if (ret < 0)
    {
      kmm_free(tmcdev);
      return NULL;
    }

  nxmutex_init(&tmcdev->lock);
  return tmcdev;
}

/****************************************************************************
 * Name: tmc_unregister
 *
 * Description:
 *   Unregister a TMC devices.
 *
 * Input Parameters:
 *   tmcdev  - Pointer to the TMC device.
 *
 ****************************************************************************/

void tmc_unregister(FAR struct coresight_tmc_dev_s *tmcdev)
{
  switch (tmcdev->config_type)
    {
      case TMC_CONFIG_TYPE_ETB:
      case TMC_CONFIG_TYPE_ETF:
        tmc_etf_unregister(tmcdev);
        break;

      case TMC_CONFIG_TYPE_ETR:
        tmc_etr_unregister(tmcdev);
        break;

      default:
        cserr("wrong config type\n");
        break;
    }

  nxmutex_destroy(&tmcdev->lock);
  kmm_free(tmcdev);
}

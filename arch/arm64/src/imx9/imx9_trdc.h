/****************************************************************************
 * arch/arm64/src/imx9/imx9_trdc.h
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
#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_TRDC_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_TRDC_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <stdbool.h>
#include "hardware/imx9_trdc.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
#define MBC_BLK_ALL 255
#define MRC_REG_ALL 16

struct trdc_glbac_config
{
  uint8_t mbc_mrc_id;
  uint8_t glbac_id;
  uint32_t glbac_val;
};

struct trdc_mbc_config
{
  uint8_t mbc_id;
  uint8_t dom_id;
  uint8_t mem_id;
  uint8_t blk_id;
  uint8_t glbac_id;
  bool secure;
};

struct trdc_mrc_config
{
  uint8_t mrc_id;
  uint8_t dom_id;
  uint8_t region_id;
  uint32_t region_start;
  uint32_t region_size;
  uint8_t glbac_id;
  bool secure;
};

/****************************************************************************
 * Name: imx9_trdc_config
 *
 * Description:
 *   Trusted Resource Domain Controller configuration function. This gives
 *   accesses to various resources before jumping to EL1
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

void imx9_trdc_config(void);

/****************************************************************************
 * Name: imx9_trdc_init
 *
 * Description:
 *   Trusted Resource Domain Controller initialization function. This gives
 *   accesses to various resources.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success, a negated error value otherwise
 *
 ****************************************************************************/

int imx9_trdc_init(void);

#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_TRDC_H */
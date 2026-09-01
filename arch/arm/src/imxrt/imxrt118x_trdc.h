/****************************************************************************
 * arch/arm/src/imxrt/imxrt118x_trdc.h
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
#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT118X_TRDC_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT118X_TRDC_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <stdbool.h>
#include "hardware/rt118x/imxrt118x_trdc.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
#define MBC_BLK_ALL 255
#define MRC_REG_ALL 16

/* TRDC MDA attribute encodings. */

#define TRDC_MDA_DID_FROM_MDA       0
#define TRDC_MDA_DID_FROM_INPUT     1
#define TRDC_MDA_FORCE_SECURE       0
#define TRDC_MDA_FORCE_NONSECURE    1
#define TRDC_MDA_USE_MASTER_SECURE  2
#define TRDC_MDA_FORCE_USER         0
#define TRDC_MDA_FORCE_PRIVILEGE    1
#define TRDC_MDA_USE_MASTER_PRIV    2

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
 * Name: imxrt118x_trdc_config
 *
 * Description:
 *   Configure TRDC resource access.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

void imxrt118x_trdc_config(void);

/****************************************************************************
 * Name: imxrt118x_trdc_init
 *
 * Description:
 *   Take ownership of the TRDCs.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success, a negated error value otherwise
 *
 ****************************************************************************/

int imxrt118x_trdc_init(void);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT118X_TRDC_H */
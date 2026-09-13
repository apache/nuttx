/****************************************************************************
 * arch/arm/src/imxrt/imxrt118x_ele.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT118X_ELE_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT118X_ELE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/rt118x/imxrt118x_ele.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_IMXRT_ELE_LOAD_FW
/* Embedded ELE firmware container, incbin'd directly from
 * CONFIG_IMXRT_ELE_FW_PATH (see imxrt118x_ele.c).  Only needed when the
 * driver is responsible for handing the FW to the ELE at runtime; when
 * the FW is instead packed into the boot AHAB container for the ROM to
 * load automatically, this blob is not built into the image at all.
 */

extern const uint8_t imxrt118x_ele_fw[];
extern const uint8_t imxrt118x_ele_fw_end[];
#endif

/****************************************************************************
 * Name: imxrt118x_ele_init
 *
 * Description:
 *   Bring the EdgeLock Enclave up.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imxrt118x_ele_init(void);

/****************************************************************************
 * Name: imxrt118x_ele_load_fw
 *
 * Description:
 *   Load the EdgeLock Enclave firmware.
 *
 * Input Parameters:
 *   fw_addr - Address of the ELE firmware container.
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.
 *
 ****************************************************************************/

int imxrt118x_ele_load_fw(uint32_t fw_addr);

/****************************************************************************
 * Name: imxrt118x_ele_release_rdc
 *
 * Description:
 *   Request ownership of one TRDC.
 *
 * Input Parameters:
 *   rdc_id - Packed TRDC and core identifier.
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.
 *
 ****************************************************************************/

int imxrt118x_ele_release_rdc(uint32_t rdc_id);

/****************************************************************************
 * Name: imxrt118x_ele_enable_apc
 *
 * Description:
 *   Enable Access Permission Control for the M7.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.
 *
 ****************************************************************************/

int imxrt118x_ele_enable_apc(void);

/****************************************************************************
 * Name: imxrt118x_ele_check_fw_version
 *
 * Description:
 *   Query the EdgeLock Enclave firmware version (GET_FW_VERSION) and log
 *   it, so it can be verified that the intended ELE firmware is active -
 *   whether it was loaded by NuttX (CONFIG_IMXRT_ELE_LOAD_FW) or by the
 *   ROM itself from a properly packed AHAB container.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.
 *
 ****************************************************************************/

int imxrt118x_ele_check_fw_version(void);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT118X_ELE_H */

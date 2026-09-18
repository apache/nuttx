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

#include <sys/types.h>
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
 * Name: imxrt118x_ele_read_common_fuse
 *
 * Description:
 *   Read a common fuse value.
 *
 * Input Parameters:
 *   fuse_id - Fuse ID
 *
 * Returned Value:
 *   Fuse value. Returns zero if read failed.
 *
 ****************************************************************************/

uint32_t imxrt118x_ele_read_common_fuse(uint32_t fuse_id);

/****************************************************************************
 * Name: imxrt118x_ele_get_key
 *
 * Description:
 *   Return HW unique key value.
 *
 * Input Parameters:
 *   key           -  Key buffer, must be cache line aligned
 *   key_size      -  Key size in bytes, must be 16 or 32
 *   ctx           -  Context buffer, must be cache line aligned
 *   ctx_size      -  Context buffer size
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.
 *
 ****************************************************************************/

int imxrt118x_ele_get_key(uint8_t *key, size_t key_size,
                          uint8_t *ctx, size_t ctx_size);

/****************************************************************************
 * Name: imxrt118x_ele_get_events
 *
 * Description:
 *   Return ELE events.
 *
 * Input Parameters:
 *   buffer        -  Event buffer
 *   buffer_size   -  Event buffer size
 *
 * Returned Value:
 *   Zero (OK) is returned if no events. A negated errno value is returned
 *   on failure. Positive value is number of events read.
 *
 ****************************************************************************/

int imxrt118x_ele_get_events(uint32_t *buffer, size_t buffer_size);

/****************************************************************************
 * Name: imxrt118x_ele_close_device
 *
 * Description:
 *   Set device to OEM close state. This operation is irreversible.
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.
 *
 ****************************************************************************/

int imxrt118x_ele_close_device(void);

/****************************************************************************
 * Name: imxrt118x_ele_get_lifecycle
 *
 * Description:
 *   Return the device's lifecycle value, queried from the ELE via
 *   GET_INFO (RT118x has no i.MX9-style FSB lifecycle register).
 *
 * Returned Value:
 *   Lifecycle value.
 *
 ****************************************************************************/

uint32_t imxrt118x_ele_get_lifecycle(void);

/****************************************************************************
 * Name: imxrt118x_ele_auth_oem_ctnr
 *
 * Description:
 *   Authenticate container header.
 *
 * Input Parameters:
 *   ctnr_addr - Address of the container header.
 *
 * Output Parameters:
 *   response - ELE response, can be used for debugging.
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.
 *
 ****************************************************************************/

int imxrt118x_ele_auth_oem_ctnr(unsigned long ctnr_addr, uint32_t *response);

/****************************************************************************
 * Name: imxrt118x_ele_release_container
 *
 * Description:
 *   Release the container from the ELE, used after
 *   imxrt118x_ele_auth_oem_ctnr().
 *
 * Output Parameters:
 *   response - ELE response, can be used for debugging.
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.
 *
 ****************************************************************************/

int imxrt118x_ele_release_container(uint32_t *response);

/****************************************************************************
 * Name: imxrt118x_ele_verify_image
 *
 * Description:
 *   Verify the specified image, for the current container.
 *
 * Input Parameters:
 *   img_id - The id of the image in the context of the current container.
 *
 * Output Parameters:
 *   response - ELE response, can be used for debugging.
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.
 *
 ****************************************************************************/

int imxrt118x_ele_verify_image(uint32_t img_id, uint32_t *response);

/****************************************************************************
 * Name: imxrt118x_ele_start_rng
 *
 * Description:
 *   Sends command to initialize the ELE RNG context.
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.
 *
 ****************************************************************************/

int imxrt118x_ele_start_rng(void);

/****************************************************************************
 * Name: imxrt118x_ele_get_trng_state
 *
 * Description:
 *   Query the state of the True Random Number Generator.
 *
 * Returned Value:
 *   Zero is returned if the Random Number Generator (RNG) is ready for
 *   use. A negated errno value (-EBUSY) or another is returned on
 *   failure.
 *
 ****************************************************************************/

int imxrt118x_ele_get_trng_state(void);

/****************************************************************************
 * Name: imxrt118x_ele_get_random
 *
 * Description:
 *   Request from the ELE the generation of a random number of specified
 *   length.
 *
 * Input Parameters:
 *   paddr  -  32bit physical address to store the random number.
 *   len    -  Length in bytes of the random number.
 *
 * Returned Value:
 *   Zero is returned if ELE successfully generated the random number.
 *   A negated errno value (-EBUSY) or another is returned on failure.
 *
 ****************************************************************************/

int imxrt118x_ele_get_random(uint32_t paddr, size_t len);

/****************************************************************************
 * Name: imxrt118x_ele_commit
 *
 * Description:
 *   Sends commit command to the ELE.
 *
 * Input Parameters:
 *   info - Information type to be committed
 *
 * Output Parameters:
 *   response - ELE response, can be used for debugging.
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.
 *
 ****************************************************************************/

int imxrt118x_ele_commit(uint32_t info, uint32_t *response);

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
 * Name: imxrt118x_ele_voltage_change_start
 *
 * Description:
 *   Begin an ELE-guarded VDD1P0 voltage change (VOLTAGE_CHANGE_START,
 *   0x12).  When the digital glitch detector (GDET) is enabled, the DCDC
 *   target must only be reprogrammed between this command and
 *   imxrt118x_ele_voltage_change_finish(); otherwise the glitch detector
 *   can trip on the transition and put the SoC into an unpredictable
 *   (lockup/reset) state.  While the change is in progress the ELE accepts
 *   no other command and aborts if FINISH is not issued within 50 ms.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.  ELE firmware that
 *   does not implement the command simply rejects it, in which case the
 *   caller may proceed with the (unguarded) voltage change.
 *
 ****************************************************************************/

int imxrt118x_ele_voltage_change_start(void);

/****************************************************************************
 * Name: imxrt118x_ele_voltage_change_finish
 *
 * Description:
 *   Complete an ELE-guarded VDD1P0 voltage change (VOLTAGE_CHANGE_FINISH,
 *   0x13), removing the GDET isolation started by
 *   imxrt118x_ele_voltage_change_start().  Must be issued within 50 ms of
 *   the START command.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.
 *
 ****************************************************************************/

int imxrt118x_ele_voltage_change_finish(void);

/****************************************************************************
 * Name: imxrt118x_ele_check_fw_version
 *
 * Description:
 *   Query the EdgeLock Enclave firmware version (GET_FW_VERSION) and log
 *   it, so it can be verified that the intended ELE firmware is active.
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

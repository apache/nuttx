/****************************************************************************
 * arch/arm64/src/imx9/imx9_ele.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_ELE_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_ELE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/imx9_ele.h"
#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_ele_init
 *
 * Description:
 *   This function disable interrupts from AHAB
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx9_ele_init(void);

/****************************************************************************
 * Name: imx9_ele_release_rdc
 *
 * Description:
 *   Trusted Resource Domain Controller AHAB interface.  This function
 *   communicates with the Advanced High Assurance Boot (AHAB) image that
 *   should reside in the particular address. This releases particular
 *   resources.
 *
 * Input Parameters:
 *   xrdc    -  RDC index
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ele_release_rdc(uint32_t rdc_id);

/****************************************************************************
 * Name: imx9_ele_read_common_fuse
 *
 * Description:
 *   Trusted Resource Domain Controller AHAB interface.  This function
 *   communicates with the Advanced High Assurance Boot (AHAB) image that
 *   should reside in the particular address. This reads particular
 *   fuse
 *
 * Input Parameters:
 *   fuse_id    -  Fuse ID
 *
 * Returned Value:
 *   Fuse value. Returns zero if read failed.
 *
 ****************************************************************************/

uint32_t imx9_ele_read_common_fuse(uint32_t fuse_id);

/****************************************************************************
 * Name: imx9_ele_get_key
 *
 * Description:
 *   Trusted Resource Domain Controller AHAB interface.  This function
 *   communicates with the Advanced High Assurance Boot (AHAB) image that
 *   should reside in the particular address. This returns HW unique
 *   key value.
 *
 * Input Parameters:
 *   key           -  Key buffer, must be cache line aligned
 *   key_size      -  Key size in bytes, must be 16 or 32
 *   ctx           -  Context buffer, must be cache line aligned
 *   ctx_size      -  Context buffer size
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ele_get_key(uint8_t *key, size_t key_size,
                     uint8_t *ctx, size_t ctx_size);

/****************************************************************************
 * Name: imx9_ele_get_events
 *
 * Description:
 *   Trusted Resource Domain Controller AHAB interface.  This function
 *   communicates with the Advanced High Assurance Boot (AHAB) image that
 *   should reside in the particular address. This returns ELE events.
 *
 * Input Parameters:
 *   buffer        -  Event buffer
 *   buffer_size   -  Event buffer size
 *
 * Returned Value:
 *  Zero (OK) is returned if no events success. A negated errno value
 *  is returned on failure. Positive value is number of events read.
 *
 ****************************************************************************/

int imx9_ele_get_events(uint32_t *buffer, size_t buffer_size);

/****************************************************************************
 * Name: imx9_ele_close_device
 *
 * Description:
 *   Trusted Resource Domain Controller AHAB interface.  This function
 *   communicates with the Advanced High Assurance Boot (AHAB) image that
 *   should reside in the particular address. This sets device to
 *   OEM close state. This operation is irreversible.
 *
 * Returned Value:
 *  Zero (OK) is returned if no events success. A negated errno value
 *  is returned on failure.
 *
 ****************************************************************************/

int imx9_ele_close_device(void);

/****************************************************************************
 * Name: imx9_ele_get_lifecycle
 *
 * Description:
 *   This returns devices lifecycle value
 *
 * Returned Value:
 *  Lifecycle value
 *
 ****************************************************************************/

uint32_t imx9_ele_get_lifecycle(void);

/****************************************************************************
 * Name: imx9_ele_auth_oem_ctnr
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
 *   Zero (OK) is returned for success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ele_auth_oem_ctnr(unsigned long ctnr_addr, uint32_t *response);

/****************************************************************************
 * Name: imx9_ele_release_container
 *
 * Description:
 *   Release the container from the ELE, used after imx9_ele_auth_oem_ctnr().
 *
 * Output Parameters:
 *   response - ELE response, can be used for debugging.
 *
 * Returned Value:
 *   Zero (OK) is returned for success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ele_release_container(uint32_t *response);

/****************************************************************************
 * Name: imx9_ele_verify_image
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
 *   Zero (OK) is returned for success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ele_verify_image(uint32_t img_id, uint32_t *response);

/****************************************************************************
 * Name: imx9_ele_start_rng
 *
 * Description:
 *   Sends command to initialize the ELE RNG context.
 *
 * Returned Value:
 *   Zero (OK) is returned for success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ele_start_rng(void);

/****************************************************************************
 * Name: imx9_ele_get_trng_state
 *
 * Description:
 *   Query the state of the True Random Number Generator.
 *
 * Returned Value:
 *   Zero is returned if the Random Number Generator (RNG) is ready for use.
 *   A negated errno value (-EBUSY) or another is returned on failure.
 *
 ****************************************************************************/

int imx9_ele_get_trng_state(void);

/****************************************************************************
 * Name: imx9_ele_get_random
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

int imx9_ele_get_random(uint32_t paddr, size_t len);
#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_ELE_H */

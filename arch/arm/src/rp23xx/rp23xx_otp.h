/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_otp.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_RP23XX_OTP_H
#define __ARCH_ARM_SRC_RP23XX_RP23XX_OTP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The OTP holds 4096 rows.  Through the ECC translation used by this driver
 * each row carries 16 bits of data, so the flat bit address space that the
 * efuse field descriptors index into is:
 *
 *   bit = row * RP23XX_OTP_ROW_BITS + bit_within_row
 */

#define RP23XX_OTP_NUM_ROWS      4096
#define RP23XX_OTP_ROW_BITS      16
#define RP23XX_OTP_TOTAL_BITS    (RP23XX_OTP_NUM_ROWS * RP23XX_OTP_ROW_BITS)

/* Rows are locked in pages of 64 rows */

#define RP23XX_OTP_PAGE_ROWS     64

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: rp23xx_otp_initialize
 *
 * Description:
 *   Register the OTP as an efuse character device.
 *
 * Input Parameters:
 *   devpath - The path to the device, e.g. "/dev/efuse"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rp23xx_otp_initialize(FAR const char *devpath);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP23XX_RP23XX_OTP_H */

/****************************************************************************
 * arch/arm/src/imxrt/imxrt_ocotp.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_OCOTP_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_OCOTP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_ocotp.h"

#include <sys/types.h>
#include <stdint.h>

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_ocotp_read
 *
 * Description:
 *   Read one value from the OTP.
 *
 * Input Parameters:
 *   otp_index - otp index (0-63)
 *   data      - a pointer to store the retrieved data
 *
 * Returned Value
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int imxrt_ocotp_read(uint32_t otp_index, uint32_t *data);

/****************************************************************************
 * Name: imxrt_ocotp_write
 *
 * Description:
 *   Write one value to the OTP.
 *
 * Input Parameters:
 *   otp_index - otp index (0-63)
 *   data      - data to write to OTP
 *
 * Returned Value
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 *
 ****************************************************************************/

int imxrt_ocotp_write(uint32_t otp_index, uint32_t data);

/****************************************************************************
 * Name: imxrt_ocotp_reload
 *
 * Description:
 *   Reload the Shadow from OTP.
 *
 ****************************************************************************/

int imxrt_ocotp_reload(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_OCOTP_H */

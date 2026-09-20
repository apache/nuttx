/*****************************************************************************
 * arch/arm/src/imxrt/imxrt_caam.h
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
 *****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_CAAM_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_CAAM_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <stdint.h>

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/*****************************************************************************
 * Name: imxrt_caam_initialize
 *
 * Description:
 *   Bring up job ring zero and, if the boot ROM has not already done so,
 *   instantiate the RNG state handle.  Idempotent.
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure.
 *
 *****************************************************************************/

int imxrt_caam_initialize(void);

/*****************************************************************************
 * Name: imxrt_caam_get_random
 *
 * Description:
 *   Fill a buffer from the CAAM true random number generator.
 *
 * Input Parameters:
 *   buffer - Where to put the bytes
 *   buflen - How many to fetch
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure.  On failure the buffer
 *   holds nothing a caller may use.
 *
 *****************************************************************************/

int imxrt_caam_get_random(uint8_t *buffer, size_t buflen);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_CAAM_H */

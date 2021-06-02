/****************************************************************************
 * arch/arm/src/sam34/sam4cm_supc.h
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

#ifndef __ARCH_ARM_SRC_SAM34_SAM4CM_SUPC_H
#define __ARCH_ARM_SRC_SAM34_SAM4CM_SUPC_H

#include <nuttx/config.h>

#include "chip.h"

#if defined(CONFIG_ARCH_CHIP_SAM4CM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
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
 * Public Functions Prototypes
 ****************************************************************************/

uint32_t supc_get_slcd_power_mode(void);
void supc_set_slcd_power_mode(uint32_t mode);
void supc_set_slcd_ldo_output(uint32_t vrout);

#undef EXTERN
#if defined(__cplusplus)
}
#endif /* __cplusplus */
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM3U_SUPC_H */
#endif /* __ARCH_ARM_SRC_SAM34_SAM4CM_SUPC_H */

/****************************************************************************
 * arch/arm/src/samd2l2/sam_ac.h
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

#ifndef __ARCH_ARM_SRC_SAMD2L2_SAM_AC_H
#define __ARCH_ARM_SRC_SAMD2L2_SAM_AC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "sam_config.h"
#include "sam_port.h"

#if defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)
#  include "hardware/samd_ac.h"
#elif defined(CONFIG_ARCH_FAMILY_SAML21)
#  include "hardware/saml_ac.h"
#else
#  error Unrecognized SAMD/L architecture
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int sam_ac_initialize(uint8_t gclkgen);
int sam_ac_config(uint8_t channel, uint32_t compctrl);
int sam_ac_enable(uint8_t channel, bool enable);

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ARCH_ARM_SRC_SAMD2L2_SAM_AC_H */

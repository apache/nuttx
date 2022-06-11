/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_firewall.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_FIREWALL_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_FIREWALL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "chip.h"

/* Include the correct firewall register definitions for this STM32L4
 * family
 */

#if defined(CONFIG_STM32L4_STM32L4X3)
#  include "hardware/stm32l4x3xx_firewall.h"
#elif defined(CONFIG_STM32L4_STM32L4X5)
#  include "hardware/stm32l4x5xx_firewall.h"
#elif defined(CONFIG_STM32L4_STM32L4X6)
#  include "hardware/stm32l4x6xx_firewall.h"
#elif defined(CONFIG_STM32L4_STM32L4XR)
#  include "hardware/stm32l4xrxx_firewall.h"
#else
#  error "Unsupported STM32L4 chip"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct stm32l4_firewall_t
{
  uintptr_t  codestart;
  size_t     codelen;
  uintptr_t  nvdatastart;
  size_t     nvdatalen;
  uintptr_t  datastart;
  size_t     datalen;
  uint8_t    datashared : 1;
  uint8_t    dataexec   : 1;
};

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

/****************************************************************************
 * Name: stm32l4_firewallsetup
 *
 * Description:
 *   Configure the STM32L4 firewall. After this, protected code will only
 *   be accessible via the "entry gate".
 *   Once enabled, the firewall cannot be enabled until the next reset.
 *   Returns 0 when OK, -1 when addresses and length are not properly
 *   aligned.
 *
 ****************************************************************************/

int stm32l4_firewallsetup(struct stm32l4_firewall_t *setup);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_FIREWALL_H */

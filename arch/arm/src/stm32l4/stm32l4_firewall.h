/************************************************************************************
 * arch/arm/src/stm32l4/stm32l4_firewall.h
 *
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_FIREWALL_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_FIREWALL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "chip.h"

/* Include the correct firewall register definitions for this STM32L4 family */

#if defined(CONFIG_STM32L4_STM32L4X3)
#  include "chip/stm32l4x3xx_firewall.h"
#elif defined(CONFIG_STM32L4_STM32L4X5)
#  include "chip/stm32l4x5xx_firewall.h"
#elif defined(CONFIG_STM32L4_STM32L4X6)
#  include "chip/stm32l4x6xx_firewall.h"
#elif defined(CONFIG_STM32L4_STM32L4XR)
#  include "chip/stm32l4xrxx_firewall.h"
#else
#  error "Unsupported STM32L4 chip"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

struct stm32l4_firewall_t
  {
  uintptr_t   codestart;
  size_t      codelen;
  uintptr_t   nvdatastart;
  size_t      nvdatalen;
  uintptr_t   datastart;
  size_t      datalen;
  uint8_t     datashared:1;
  uint8_t     dataexec  :1;
  };

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: stm32l4_firewallsetup
 *
 * Description:
 *   Configure the STM32L4 firewall. After this, protected code will only
 *   be accessible via the "entry gate".
 *   Once enabled, the firewall cannot be enabled until the next reset.
 *   Returns 0 when OK, -1 when addresses and length are not properly aligned.
 *
 ****************************************************************************/

int stm32l4_firewallsetup(FAR struct stm32l4_firewall_t *setup);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_FIREWALL_H */


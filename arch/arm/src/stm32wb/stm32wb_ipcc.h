/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_ipcc.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_STM32WB_IPCC_H
#define __ARCH_ARM_SRC_STM32WB_STM32WB_IPCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "chip.h"
#include "arm_internal.h"
#include "hardware/stm32wb_ipcc.h"

/****************************************************************************
 * Pre-processor Definitions
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_ipccreset
 *
 * Description:
 *   Reset the IPCC registers to default state
 *
 ****************************************************************************/

void stm32wb_ipccreset(void);

/****************************************************************************
 * Name: stm32wb_ipccenable
 *
 * Description:
 *   Enable the IPCC and start CPU2
 *
 ****************************************************************************/

void stm32wb_ipccenable(void);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_ipcc_rxactive
 *
 * Description:
 *   Check channel receive active flag.
 *
 ****************************************************************************/

static inline bool stm32wb_ipcc_rxactive(uint8_t chan)
{
  return (getreg32(STM32WB_IPCC_C2TOC1SR) & IPCC_C2TOC1SR_BIT(chan)) != 0;
}

/****************************************************************************
 * Name: stm32wb_ipcc_txactive
 *
 * Description:
 *   Check channel transmit active flag.
 *
 ****************************************************************************/

static inline bool stm32wb_ipcc_txactive(uint8_t chan)
{
  return (getreg32(STM32WB_IPCC_C1TOC2SR) & IPCC_C1TOC2SR_BIT(chan)) != 0;
}

/****************************************************************************
 * Name: stm32wb_ipcc_settxactive
 *
 * Description:
 *   Set channel transmit active flag.
 *
 ****************************************************************************/

static inline void stm32wb_ipcc_settxactive(uint8_t chan)
{
  putreg32(IPCC_C1SCR_SET_BIT(chan), STM32WB_IPCC_C1SCR);
}

/****************************************************************************
 * Name: stm32wb_ipcc_masktxf
 *
 * Description:
 *   Mask channel transmit free interrupt.
 *
 ****************************************************************************/

static inline void stm32wb_ipcc_masktxf(uint8_t chan)
{
  uint32_t regval = getreg32(STM32WB_IPCC_C1MR);
  regval |= IPCC_C1MR_FM_BIT(chan);
  putreg32(regval, STM32WB_IPCC_C1MR);
}

/****************************************************************************
 * Name: stm32wb_ipcc_unmasktxf
 *
 * Description:
 *   Unmask channel transmit free interrupt.
 *
 ****************************************************************************/

static inline void stm32wb_ipcc_unmasktxf(uint8_t chan)
{
  uint32_t regval = getreg32(STM32WB_IPCC_C1MR);
  regval &= ~IPCC_C1MR_FM_BIT(chan);
  putreg32(regval, STM32WB_IPCC_C1MR);
}

/****************************************************************************
 * Name: stm32wb_ipcc_maskrxo
 *
 * Description:
 *   Mask channel receive occupied interrupt.
 *
 ****************************************************************************/

static inline void stm32wb_ipcc_maskrxo(uint8_t chan)
{
  uint32_t regval = getreg32(STM32WB_IPCC_C1MR);
  regval |= IPCC_C1MR_OM_BIT(chan);
  putreg32(regval, STM32WB_IPCC_C1MR);
}

/****************************************************************************
 * Name: stm32wb_ipcc_maskrxo
 *
 * Description:
 *   Unmask channel receive occupied interrupt.
 *
 ****************************************************************************/

static inline void stm32wb_ipcc_unmaskrxo(uint8_t chan)
{
  uint32_t regval = getreg32(STM32WB_IPCC_C1MR);
  regval &= ~IPCC_C1MR_OM_BIT(chan);
  putreg32(regval, STM32WB_IPCC_C1MR);
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32WB_STM32WB_IPCC_H */

/****************************************************************************
 * arch/arm/src/efm32/efm32_rmu.h
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_RMU_H
#define __ARCH_ARM_SRC_EFM32_EFM32_RMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/efm32_rmu.h"

#ifdef CONFIG_EFM32_RMU

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_DEBUG_ERROR
#  undef CONFIG_EFM32_RMU_DEBUG
#endif

#ifdef CONFIG_EFM32_RMU_DEBUG
#  define rmuerr   _err
#  define rmuwarn  _warn
#  define rmuinfo  _info
#else
#  define rmuerr(x...)
#  define rmuwarn(x...)
#  define rmuinfo(x...)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t g_efm32_rstcause;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_rmu_initialize
 *
 * Description:
 *    Store reset cause into g_efm32_rstcause then clear reset cause
 *    register.
 *
 ****************************************************************************/

void efm32_rmu_initialize(void);
const char *efm32_reset_cause_list_str(uint32_t reg, unsigned int *idx);

/****************************************************************************
 * Name: efm32_reset_cause_list_str
 *
 * Description:
 *    Return next reset cause string, NULL if no more reset cause.
 *
 * Input Parameters:
 *   reg: reset cause register to decode (like g_efm32_rstcause)
 *   idx: Use to keep in maind reset cause decoding position.
 *        set *idx to zero before first call.
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_RMU_DEBUG
const char *efm32_reset_cause_list_str(uint32_t reg, unsigned int *idx);
#endif

#endif /* CONFIG_EFM32_RMU */
#endif /* __ARCH_ARM_SRC_EFM32_EFM32_RMU_H */

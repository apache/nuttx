/****************************************************************************
 * arch/arm/src/common/stm32/stm32_ptp_pps.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_STM32_PTP_PPS_H
#define __ARCH_ARM_SRC_COMMON_STM32_STM32_PTP_PPS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_STM32_PTP_PPS

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: stm32_ptp_pps_initialize
 *
 * Description:
 *   Start the 32-bit timer selected by CONFIG_STM32_PTP_PPS_TIMx and
 *   register a PPS disciplined PTP clock as /dev/ptp<devno>.  The PPS of
 *   the GNSS receiver must be wired to the GPIO_TIMx_CHyIN pin defined in
 *   the board.h file.  Called by the board bring-up logic.
 *
 * Input Parameters:
 *   devno - The device number, the clock is registered as /dev/ptp<devno>
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_ptp_pps_initialize(int devno);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_STM32_PTP_PPS */
#endif /* __ARCH_ARM_SRC_COMMON_STM32_STM32_PTP_PPS_H */

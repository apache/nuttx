/****************************************************************************
 * arch/arm/src/stm32f7/stm32_can.h
 *
 *   Copyright (C) 2009, 2011, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_STM32F7_CAN_H
#define __ARCH_ARM_SRC_STM32_STM32F7_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32_can.h"

#include <nuttx/can/can.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if defined(CONFIG_CAN) && (defined(CONFIG_STM32F7_CAN1) || \
    defined(CONFIG_STM32F7_CAN2) || defined(CONFIG_STM32F7_CAN3))

/* CAN BAUD */

#if defined(CONFIG_STM32F7_CAN1) && !defined(CONFIG_STM32F7_CAN1_BAUD)
#  error "CONFIG_STM32F7_CAN1_BAUD is not defined"
#endif

#if defined(CONFIG_STM32F7_CAN2) && !defined(CONFIG_STM32F7_CAN2_BAUD)
#  error "CONFIG_STM32F7_CAN2_BAUD is not defined"
#endif

#if defined(CONFIG_STM32F7_CAN3) && !defined(CONFIG_STM32F7_CAN3_BAUD)
#  error "CONFIG_STM32F7_CAN3_BAUD is not defined"
#endif

/* User-defined TSEG1 and TSEG2 settings may be used.
 *
 * CONFIG_STM32F7_CAN_TSEG1 = the number of CAN time quanta in segment 1
 * CONFIG_STM32F7_CAN_TSEG2 = the number of CAN time quanta in segment 2
 * CAN_BIT_QUANTA   = The number of CAN time quanta in on bit time
 */

#ifndef CONFIG_STM32F7_CAN_TSEG1
#  define CONFIG_STM32F7_CAN_TSEG1 6
#endif

#if CONFIG_STM32F7_CAN_TSEG1 < 1 || \
    CONFIG_STM32F7_CAN_TSEG1 > CAN_BTR_TSEG1_MAX
#  error "CONFIG_STM32_CAN_TSEG1 is out of range"
#endif

#ifndef CONFIG_STM32F7_CAN_TSEG2
#  define CONFIG_STM32F7_CAN_TSEG2 7
#endif

#if CONFIG_STM32F7_CAN_TSEG2 < 1 || \
    CONFIG_STM32F7_CAN_TSEG2 > CAN_BTR_TSEG2_MAX
#  error "CONFIG_STM32_CAN_TSEG2 is out of range"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 * Name: stm32_caninitialize
 *
 * Description:
 *   Initialize the selected CAN port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple CAN interfaces)
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s;
FAR struct can_dev_s *stm32_caninitialize(int port);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_CAN && (CONFIG_STM32_CAN1 || CONFIG_STM32_CAN2) */
#endif /* __ARCH_ARM_SRC_STM32_STM32_CAN_H */

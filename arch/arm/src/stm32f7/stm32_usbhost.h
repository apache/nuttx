/****************************************************************************
 * arch/arm/src/stm32f7/stm32_usbhost.h
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

#ifndef __ARCH_ARM_SRC_STM32F7_STM32_USBHOST_H
#define __ARCH_ARM_SRC_STM32F7_STM32_USBHOST_H

/* STM32 USB OTG Host Driver Support
 *
 * Pre-requisites
 *
 *  CONFIG_USBHOST        - Enable general USB host support
 *  CONFIG_STM32F7_OTGFS  - Enable the STM32 USB OTG FS block
 *     or
 *  CONFIG_STM32F7_OTGFSHS  - Enable the STM32 USB OTG HS block
 *  CONFIG_STM32F7_SYSCFG - Needed
 *
 * Options:
 *
 *  CONFIG_STM32F7_OTG_RXFIFO_SIZE - Size of the RX FIFO in 32-bit words.
 *    Default 128 (512 bytes)
 *  CONFIG_STM32F7_OTG_NPTXFIFO_SIZE - Size of the non-periodic Tx FIFO
 *    in 32-bit words.  Default 96 (384 bytes)
 *  CONFIG_STM32F7_OTG_PTXFIFO_SIZE - Size of the periodic Tx FIFO in 32-bit
 *    words.  Default 96 (384 bytes)
 *  CONFIG_STM32F7_OTG_SOFINTR - Enable SOF interrupts.  Why would you ever
 *    want to do that?
 *
 *  CONFIG_STM32F7_USBHOST_REGDEBUG - Enable very low-level register access
 *    debug.  Depends on CONFIG_DEBUG_FEATURES.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#if (defined(CONFIG_STM32F7_OTGFS) || defined(CONFIG_STM32F7_OTGFSHS)) && \
    defined(CONFIG_USBHOST)

/****************************************************************************
 * Public Function Prototypes
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
 * Name: stm32_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be
 *   provided be each platform that implements the STM32 OTG FS host
 *   interface
 *
 *   "On-chip 5 V VBUS generation is not supported. For this reason, a
 *    charge pump or, if 5 V are available on the application board, a basic
 *    power switch, must be added externally to drive the 5 V VBUS line. The
 *    external charge pump can be driven by any GPIO output. When the
 *    application decides to power on VBUS using the chosen GPIO, it must
 *    also set the port power bit in the host port control and status
 *    register (PPWR bit in OTG_FS_HPRT).
 *
 *   "The application uses this field to control power to this port, and the
 *    core clears this bit on an overcurrent condition."
 *
 * Input Parameters:
 *   iface - For future growth to handle multiple USB host interface.
 *           Should be zero.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_usbhost_vbusdrive(int iface, bool enable);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* (CONFIG_STM32F7_OTGFS || CONFIG_STM32F7_OTGFSHS) && CONFIG_USBHOST */
#endif /* __ARCH_ARM_SRC_STM32F7_STM32_USBHOST_H */

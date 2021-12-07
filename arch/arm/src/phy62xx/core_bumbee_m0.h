/****************************************************************************
 * arch/arm/src/phy62xx/core_bumbee_m0.h
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

#ifndef PHY_BUMBEE_M0_H
#define PHY_BUMBEE_M0_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Start of section using anonymous unions and disabling warnings  */
#if   defined (__CC_ARM)
#pragma push
#pragma anon_unions
#elif defined (__ICCARM__)
#pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc11-extensions"
#pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined (__GNUC__)
/* anonymous unions are enabled by default */
#elif defined (__TMS470__)
/* anonymous unions are enabled by default */
#elif defined (__TASKING__)
#pragma warning 586
#elif defined (__CSMC__)
/* anonymous unions are enabled by default */
#else
#warning Not supported compiler type
#endif

/* Configuration of the Cortex-M0 Processor and Core Peripherals */

/* #define __CM0_REV                 0x0000U */  /* Core revision r0p0 */

/* #define __MPU_PRESENT             0U  */      /* MPU present or not */

/* #define __VTOR_PRESENT            0U  */      /* no VTOR present */

#define __NVIC_PRIO_BITS          2U        /* Number of Bits used for Priority Levels */

/* #define __Vendor_SysTickConfig    0U  */     /* Set to 1 if different SysTick Config is used */

#include "core_cm0.h"                       /* Processor and core peripherals */
#include "system_ARMCM0.h"                  /* System Header */

#define NVIC_GetPendingIRQs()       (NVIC->ISPR[0U])
#define NVIC_ClearPendingIRQs(icpr) (NVIC->ICPR[0U] = (unsigned int)icpr)
#define NVIC_SetPendingIRQs(ispr)   (NVIC->ISPR[0U] = (unsigned int)ispr)

#define NVIC_GetEnableIRQs()        (NVIC->ISER[0U])
#define NVIC_DisableIRQs(irqs)      (NVIC->ICER[0U] = (unsigned int)irqs)
#define NVIC_EnableIRQs(iser)       (NVIC->ISER[0U] = (unsigned int)iser)

#define NVIC_ClearWakeupIRQ(irqn)
#define NVIC_SetWakeupIRQ(irqn)

/* End of section using anonymous unions and disabling warnings */

#if   defined (__CC_ARM)
#pragma pop
#elif defined (__ICCARM__)
/* leave anonymous unions enabled */
#elif (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic pop
#elif defined (__GNUC__)
/* anonymous unions are enabled by default */
#elif defined (__TMS470__)
/* anonymous unions are enabled by default */
#elif defined (__TASKING__)
#pragma warning restore
#elif defined (__CSMC__)
/* anonymous unions are enabled by default */
#else
#warning Not supported compiler type
#endif

#ifdef __cplusplus
}
#endif

#endif  /* PHY_BUMBEE_M0 */

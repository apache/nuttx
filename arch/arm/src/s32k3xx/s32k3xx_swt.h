/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_swt.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_S32K3XX_WDOG_H
#define __ARCH_ARM_SRC_S32K3XX_S32K3XX_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
#include "hardware/s32k3xx_swt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: s32k3xx_swt_disable
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during boot-up.  This must be called
 *   before arm_serialinit.
 *
 *   REVISIT:  Hardcoded assumption that WDOG clock derives for LPO_CLK
 *
 ****************************************************************************/

static inline void s32k3xx_swt_disable(void)
{
  /* Unlock soft lock */

  putreg32(0xc520, S32K3XX_SWT0_SR);
  putreg32(0xd928, S32K3XX_SWT0_SR);

  /* Enable access */

  putreg32(SWT_CR_MAP0 |
           SWT_CR_MAP1 |
           SWT_CR_MAP2 |
           SWT_CR_MAP3 |
           SWT_CR_MAP4 |
           SWT_CR_MAP5 |
           SWT_CR_MAP6 |
           SWT_CR_MAP7 |
           SWT_CR_WND, S32K3XX_SWT0_CR);
}

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_S32K3XX_S32K3XX_WDOG_H */

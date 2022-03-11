/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_wdog.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K1XX_WDOG_H
#define __ARCH_ARM_SRC_S32K1XX_S32K1XX_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
#include "hardware/s32k1xx_wdog.h"

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
 * Name: s32k1xx_wdog_disable
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during boot-up.  This must be called
 *   before arm_serialinit.
 *
 *   REVISIT:  Hardcoded assumption that WDOG clock derives for LPO_CLK
 *
 ****************************************************************************/

static inline void s32k1xx_wdog_disable(void)
{
  uint32_t regval;

  /* Write of the WDOG unlock key to CNT register.  This must be done in
   * order to allow any modifications to the WDOG configuration.
   */

  putreg32(WDOG_CNT_UNLOCK, S32K1XX_WDOG_CNT);

  /* The dummy read is used in order to make sure that the WDOG registers
   * will be configured only after the write of the unlock value was
   * completed.
   */

  getreg32(S32K1XX_WDOG_CNT);

  /* Initial write of WDOG configuration register:  Enables support for
   * 32-bit refresh/unlock command write words, clock select from LPO,
   * update enable, watchdog disabled.
   */

  regval = (WDOG_CS_CMD32EN | WDOG_CS_CLK_LPOCLK | WDOG_CS_UPDATE);
  putreg32(regval, S32K1XX_WDOG_CS);

  /* Configure timeout to the maximum: */

  putreg32(0xffff, S32K1XX_WDOG_TOVAL);
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
#endif /* __ARCH_ARM_SRC_S32K1XX_S32K1XX_WDOG_H */

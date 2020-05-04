/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_wdog.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K1XX_WDOG_H
#define __ARCH_ARM_SRC_S32K1XX_S32K1XX_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_arch.h"
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

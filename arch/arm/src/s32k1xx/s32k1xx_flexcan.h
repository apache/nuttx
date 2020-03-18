/************************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_flexcan.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K1XX_FLEXCAN_H
#define __ARCH_ARM_SRC_S32K1XX_S32K1XX_FLEXCAN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "hardware/s32k1xx_flexcan.h"

#ifdef CONFIG_S32K1XX_FLEXCAN

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
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
 * Function: up_netinitialize
 *
 * Description:
 *   Initialize the first network interface.  If there are more than one
 *   interface in the chip, then board-specific logic will have to provide
 *   this function to determine which, if any, Ethernet controllers should
 *   be initialized.  Also prototyped in up_internal.h.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called very early in the initialization sequence.
 *
 ************************************************************************************/

void up_netinitialize(void);

/************************************************************************************
 * Function: s32k1xx_phy_boardinitialize
 *
 * Description:
 *   Some boards require specialized initialization of the PHY before it can be
 *   used.  This may include such things as configuring GPIOs, resetting the PHY,
 *   etc.  If CONFIG_S32K1XX_FLEXCAN_PHYINIT is defined in the configuration then the
 *   board specific logic must provide s32k1xx_phyinitialize();  The i.MX RT Ethernet
 *   driver will call this function one time before it first uses the PHY.
 *
 * Input Parameters:
 *   intf - Always zero for now.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_S32K1XX_FLEXCAN */
#endif /* __ARCH_ARM_SRC_S32K1XX_S32K1XX_FLEXCAN_H */

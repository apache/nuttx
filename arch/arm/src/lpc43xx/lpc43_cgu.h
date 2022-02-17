/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_cgu.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_LPC43_CGU_H
#define __ARCH_ARM_SRC_LPC43XX_LPC43_CGU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "hardware/lpc43_cgu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
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
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_pll0usbdisable
 *
 * Description:
 *   Take PLL0USB to power-down mode.
 *
 ****************************************************************************/

EXTERN void lpc43_pll0usbdisable(void);

/****************************************************************************
 * Name: lpc43_pll0usbenable
 *
 * Description:
 *   Take PLL0USB out of power-down mode and wait until it is locked onto the
 *   input clock.
 *
 ****************************************************************************/

EXTERN void lpc43_pll0usbenable(void);

/****************************************************************************
 * Name: lpc43_pll0usbconfig
 *
 * Description:
 *   Config USB0 PLL
 *
 ****************************************************************************/

EXTERN void lpc43_pll0usbconfig(void);

/****************************************************************************
 * Name: lpc43_clockconfig
 *
 * Description:
 *   Called to initialize the LPC43XX.  This does whatever setup is needed to
 *   put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void lpc43_clockconfig(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LPC43XX_LPC43_CGU_H */

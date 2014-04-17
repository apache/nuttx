/****************************************************************************
 * arch/arm/src/sama5/sam_clockconfig.h
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_SAMA5_SAM_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "up_internal.h"

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
 * Name: sam_clockconfig
 *
 * Description:
 *   Called to initialize the SAM3/4.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void __ramfunc__ sam_clockconfig(void);

/****************************************************************************
 * Name: board_norflash_config
 *
 * Description:
 *   If CONFIG_SAMA5_BOOT_CS0FLASH, then the system is boot directly off
 *   CS0 NOR FLASH.  In this case, we assume that we get here from the
 *   primary boot loader under these conditions:
 *
 *     "If BMS signal is tied to 0, BMS_BIT is read at 1.  The ROM Code
 *      allows execution of the code contained into the memory connected to
 *      Chip Select 0 of the External Bus Interface.
 *
 *     "To achieve that, the following sequence is preformed by the ROM
 *      Code:
 *
 *        - The main clock is the on-chip 12 MHz RC oscillator,
 *        - The Static Memory Controller is configured with timing allowing
 *          code execution inCS0 external memory at 12 MHz
 *        - AXI matrix is configured to remap EBI CS0 address at 0x0
 *        - 0x0 is loaded in the Program Counter register
 *
 *     "The user software in the external memory must perform the next
 *      operation in order to complete the clocks and SMC timings
 *      configuration to run at a higher clock frequency:
 *
 *        - Enable the 32768 Hz oscillator if best accuracy is needed
 *        - Reprogram the SMC setup, cycle, hold, mode timing registers
 *          for EBI CS0, to adapt them to the new clock
 *        - Program the PMC (Main Oscillator Enable or Bypass mode)
 *        - Program and Start the PLL
 *        - Switch the system clock to the new value"
 *
 *   This function provides the board-specific implementation of the logic
 *   to reprogram the SMC.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_BOOT_CS0FLASH
void board_norflash_config(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_CLOCKCONFIG_H */

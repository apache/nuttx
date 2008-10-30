/************************************************************************************
 * arch/arm/src/str71x/str71x_pcu.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_PCU_H
#define __ARCH_ARM_SRC_STR71X_STR71X_PCU_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include "str71x_map.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Power Control Unit (PCU) registers ***********************************************/

#define STR71X_PCU_MDIVR        (STR71X_PCU_BASE + 0x0000)  /* 16-bits wide */
#define STR71X_PCU_PDIVR        (STR71X_PCU_BASE + 0x0004)  /* 16-bits wide */
#define STR71X_PCU_RSTR         (STR71X_PCU_BASE + 0x0008)  /* 16-bits wide */
#define STR71X_PCU_PLL2CR       (STR71X_PCU_BASE + 0x000c)  /* 16-bits wide */
#define STR71X_PCU_BOOTCR       (STR71X_PCU_BASE + 0x0010)  /* 16-bits wide */
#define STR71X_PCU_PWRCR        (STR71X_PCU_BASE + 0x0014)  /* 16-bits wide */

/* Register bit settings ************************************************************/

/* PCU flags */

#define STR71X_PCU_WREN         (0x8000)
#define STR71X_PCU_VROK         (0x1000)

/* PCU VR status */

#define STR71X_PCU_STABLE       (0)
#define STR71X_PCU_UNSTABLE     (1)

/* PCU VR */

#define STR71X_PCU_MVR          (0x0008)
#define STR71X_PCU_LPR          (0x0010)

/* WFI Clocks */

#define STR71X_PCU_WFICLOCK216  (0)
#define STR71X_PCU_WFICkAF      (1)

/* LPWFI Clocks */

#define STR71X_PCU_LPWFICLK216  (0)
#define STR71X_PCU_LPWFICKAF    (1)

/* RCCU_CCR register bits definition */

#define STR71X_PCUCCR_ENHALT    (0x00000800) /* Bit 11: Enable Halt bit */
#define STR71X_PCUCCR_LPOWFI    (0x00000001) /* Bit 0: Low Power Mode in Wait For interrupt Mode */
 
/* PCU_PWRCR register bits definition */

#define STR71X_PCUPWRCR_VRBYP   (0x0008) /* Bit 3: Main Regulator Bypass bit */
#define STR71X_PCUPWRCR_LPRWFI  (0x0010) /* Bit 4: Low Power Regulator in Wait For interrupt Mode bit */
#define STR71X_PCUPWRCR_LVDDIS  (0x0100) /* Bit 8: Low Voltage Detector Disable bit */
#define STR71X_PCUPWRCR_VROK    (0x1000) /* Bit 12: Voltage Regulator OK flag */
#define STR71X_PCUPWRCR_BUSY    (0x4000) /* Bit 14: PCU register Backup logic Busy flag */
#define STR71X_PCUPWRCR_WREN    (0x8000) /* Bit 15: PCU register Write Enable Bit */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_PCU_H */

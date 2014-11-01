/*******************************************************************************************************************************
 * arch/arm/src/efm32/chip/efm32_wdog.h
 *
 *  Copyright 2014 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.@n
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.@n
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Laboratories, Inc.
 * has no obligation to support this Software. Silicon Laboratories, Inc. is
 * providing the Software "AS IS", with no express or implied warranties of any
 * kind, including, but not limited to, any implied warranties of
 * merchantability or fitness for any particular purpose or warranties against
 * infringement of any proprietary rights of a third party.
 *
 * Silicon Laboratories, Inc. will not be liable for any consequential,
 * incidental, or special damages, or any other relief, or for any claim by
 * any third party, arising from your use of this Software.
 *
 *   Copyright (C) 2014 Pierre-noel Bouteville . All rights reserved.
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Pierre-noel Bouteville <pnb990@gmail.com>
 *            Gregory Nutt <gnutt@nuttx.org>
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
 *******************************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_EFM32_CHIP_EFM32_WDOG_H
#define __ARCH_ARM_SRC_EFM32_CHIP_EFM32_WDOG_H

/*******************************************************************************************************************************
 * Included Files
 *******************************************************************************************************************************/

#include <nuttx/config.h>
#include "chip/efm32_memorymap.h"

#if !defined(CONFIG_EFM32_EFM32GG)
#  warning This is the EFM32GG header file; Review/modification needed for this archtecture
#endif

/*******************************************************************************************************************************
 * Pre-processor Definitions
 *******************************************************************************************************************************/
/* WDOG Register Offsets *******************************************************************************************************/

#define EFM32_WDOG_CTRL_OFFSET           0x0000  /* Control Register */
#define EFM32_WDOG_CMD_OFFSET            0x0004  /* Command Register */
#define EFM32_WDOG_SYNCBUSY_OFFSET       0x0008  /* Synchronization Busy Register */

/* WDOG Register Addresses *****************************************************************************************************/

#define EFM32_WDOG_CTRL                  (EFM32_WDOG_BASE+EFM32_WDOG_CTRL_OFFSET)
#define EFM32_WDOG_CMD                   (EFM32_WDOG_BASE+EFM32_WDOG_CMD_OFFSET)
#define EFM32_WDOG_SYNCBUSY              (EFM32_WDOG_BASE+EFM32_WDOG_SYNCBUSY_OFFSET)

/* WDOG Register Bit Field Definitions *****************************************************************************************/

/* Bit fields for WDOG CTRL */

#define _WDOG_CTRL_RESETVALUE            0x00000F00UL                         /* Default value for WDOG_CTRL */
#define _WDOG_CTRL_MASK                  0x00003F7FUL                         /* Mask for WDOG_CTRL */

#define WDOG_CTRL_EN                     (0x1UL << 0)                         /* Watchdog Timer Enable */
#define _WDOG_CTRL_EN_SHIFT              0                                    /* Shift value for WDOG_EN */
#define _WDOG_CTRL_EN_MASK               0x1UL                                /* Bit mask for WDOG_EN */
#define _WDOG_CTRL_EN_DEFAULT            0x00000000UL                         /* Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_EN_DEFAULT             (_WDOG_CTRL_EN_DEFAULT << 0)         /* Shifted mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_DEBUGRUN               (0x1UL << 1)                         /* Debug Mode Run Enable */
#define _WDOG_CTRL_DEBUGRUN_SHIFT        1                                    /* Shift value for WDOG_DEBUGRUN */
#define _WDOG_CTRL_DEBUGRUN_MASK         0x2UL                                /* Bit mask for WDOG_DEBUGRUN */
#define _WDOG_CTRL_DEBUGRUN_DEFAULT      0x00000000UL                         /* Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_DEBUGRUN_DEFAULT       (_WDOG_CTRL_DEBUGRUN_DEFAULT << 1)   /* Shifted mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_EM2RUN                 (0x1UL << 2)                         /* Energy Mode 2 Run Enable */
#define _WDOG_CTRL_EM2RUN_SHIFT          2                                    /* Shift value for WDOG_EM2RUN */
#define _WDOG_CTRL_EM2RUN_MASK           0x4UL                                /* Bit mask for WDOG_EM2RUN */
#define _WDOG_CTRL_EM2RUN_DEFAULT        0x00000000UL                         /* Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_EM2RUN_DEFAULT         (_WDOG_CTRL_EM2RUN_DEFAULT << 2)     /* Shifted mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_EM3RUN                 (0x1UL << 3)                         /* Energy Mode 3 Run Enable */
#define _WDOG_CTRL_EM3RUN_SHIFT          3                                    /* Shift value for WDOG_EM3RUN */
#define _WDOG_CTRL_EM3RUN_MASK           0x8UL                                /* Bit mask for WDOG_EM3RUN */
#define _WDOG_CTRL_EM3RUN_DEFAULT        0x00000000UL                         /* Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_EM3RUN_DEFAULT         (_WDOG_CTRL_EM3RUN_DEFAULT << 3)     /* Shifted mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_LOCK                   (0x1UL << 4)                         /* Configuration lock */
#define _WDOG_CTRL_LOCK_SHIFT            4                                    /* Shift value for WDOG_LOCK */
#define _WDOG_CTRL_LOCK_MASK             0x10UL                               /* Bit mask for WDOG_LOCK */
#define _WDOG_CTRL_LOCK_DEFAULT          0x00000000UL                         /* Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_LOCK_DEFAULT           (_WDOG_CTRL_LOCK_DEFAULT << 4)       /* Shifted mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_EM4BLOCK               (0x1UL << 5)                         /* Energy Mode 4 Block */
#define _WDOG_CTRL_EM4BLOCK_SHIFT        5                                    /* Shift value for WDOG_EM4BLOCK */
#define _WDOG_CTRL_EM4BLOCK_MASK         0x20UL                               /* Bit mask for WDOG_EM4BLOCK */
#define _WDOG_CTRL_EM4BLOCK_DEFAULT      0x00000000UL                         /* Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_EM4BLOCK_DEFAULT       (_WDOG_CTRL_EM4BLOCK_DEFAULT << 5)   /* Shifted mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_SWOSCBLOCK             (0x1UL << 6)                         /* Software Oscillator Disable Block */
#define _WDOG_CTRL_SWOSCBLOCK_SHIFT      6                                    /* Shift value for WDOG_SWOSCBLOCK */
#define _WDOG_CTRL_SWOSCBLOCK_MASK       0x40UL                               /* Bit mask for WDOG_SWOSCBLOCK */
#define _WDOG_CTRL_SWOSCBLOCK_DEFAULT    0x00000000UL                         /* Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_SWOSCBLOCK_DEFAULT     (_WDOG_CTRL_SWOSCBLOCK_DEFAULT << 6) /* Shifted mode DEFAULT for WDOG_CTRL */
#define _WDOG_CTRL_PERSEL_SHIFT          8                                    /* Shift value for WDOG_PERSEL */
#define _WDOG_CTRL_PERSEL_MASK           0xF00UL                              /* Bit mask for WDOG_PERSEL */
#define _WDOG_CTRL_PERSEL_DEFAULT        0x0000000FUL                         /* Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_PERSEL_DEFAULT         (_WDOG_CTRL_PERSEL_DEFAULT << 8)     /* Shifted mode DEFAULT for WDOG_CTRL */
#define _WDOG_CTRL_CLKSEL_SHIFT          12                                   /* Shift value for WDOG_CLKSEL */
#define _WDOG_CTRL_CLKSEL_MASK           0x3000UL                             /* Bit mask for WDOG_CLKSEL */
#define _WDOG_CTRL_CLKSEL_DEFAULT        0x00000000UL                         /* Mode DEFAULT for WDOG_CTRL */
#define _WDOG_CTRL_CLKSEL_ULFRCO         0x00000000UL                         /* Mode ULFRCO for WDOG_CTRL */
#define _WDOG_CTRL_CLKSEL_LFRCO          0x00000001UL                         /* Mode LFRCO for WDOG_CTRL */
#define _WDOG_CTRL_CLKSEL_LFXO           0x00000002UL                         /* Mode LFXO for WDOG_CTRL */
#define WDOG_CTRL_CLKSEL_DEFAULT         (_WDOG_CTRL_CLKSEL_DEFAULT << 12)    /* Shifted mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_CLKSEL_ULFRCO          (_WDOG_CTRL_CLKSEL_ULFRCO << 12)     /* Shifted mode ULFRCO for WDOG_CTRL */
#define WDOG_CTRL_CLKSEL_LFRCO           (_WDOG_CTRL_CLKSEL_LFRCO << 12)      /* Shifted mode LFRCO for WDOG_CTRL */
#define WDOG_CTRL_CLKSEL_LFXO            (_WDOG_CTRL_CLKSEL_LFXO << 12)       /* Shifted mode LFXO for WDOG_CTRL */

/* Bit fields for WDOG CMD */

#define _WDOG_CMD_RESETVALUE             0x00000000UL                     /* Default value for WDOG_CMD */
#define _WDOG_CMD_MASK                   0x00000001UL                     /* Mask for WDOG_CMD */

#define WDOG_CMD_CLEAR                   (0x1UL << 0)                     /* Watchdog Timer Clear */
#define _WDOG_CMD_CLEAR_SHIFT            0                                /* Shift value for WDOG_CLEAR */
#define _WDOG_CMD_CLEAR_MASK             0x1UL                            /* Bit mask for WDOG_CLEAR */
#define _WDOG_CMD_CLEAR_DEFAULT          0x00000000UL                     /* Mode DEFAULT for WDOG_CMD */
#define _WDOG_CMD_CLEAR_UNCHANGED        0x00000000UL                     /* Mode UNCHANGED for WDOG_CMD */
#define _WDOG_CMD_CLEAR_CLEARED          0x00000001UL                     /* Mode CLEARED for WDOG_CMD */
#define WDOG_CMD_CLEAR_DEFAULT           (_WDOG_CMD_CLEAR_DEFAULT << 0)   /* Shifted mode DEFAULT for WDOG_CMD */
#define WDOG_CMD_CLEAR_UNCHANGED         (_WDOG_CMD_CLEAR_UNCHANGED << 0) /* Shifted mode UNCHANGED for WDOG_CMD */
#define WDOG_CMD_CLEAR_CLEARED           (_WDOG_CMD_CLEAR_CLEARED << 0)   /* Shifted mode CLEARED for WDOG_CMD */

/* Bit fields for WDOG SYNCBUSY */

#define _WDOG_SYNCBUSY_RESETVALUE        0x00000000UL                       /* Default value for WDOG_SYNCBUSY */
#define _WDOG_SYNCBUSY_MASK              0x00000003UL                       /* Mask for WDOG_SYNCBUSY */

#define WDOG_SYNCBUSY_CTRL               (0x1UL << 0)                       /* CTRL Register Busy */
#define _WDOG_SYNCBUSY_CTRL_SHIFT        0                                  /* Shift value for WDOG_CTRL */
#define _WDOG_SYNCBUSY_CTRL_MASK         0x1UL                              /* Bit mask for WDOG_CTRL */
#define _WDOG_SYNCBUSY_CTRL_DEFAULT      0x00000000UL                       /* Mode DEFAULT for WDOG_SYNCBUSY */
#define WDOG_SYNCBUSY_CTRL_DEFAULT       (_WDOG_SYNCBUSY_CTRL_DEFAULT << 0) /* Shifted mode DEFAULT for WDOG_SYNCBUSY */
#define WDOG_SYNCBUSY_CMD                (0x1UL << 1)                       /* CMD Register Busy */
#define _WDOG_SYNCBUSY_CMD_SHIFT         1                                  /* Shift value for WDOG_CMD */
#define _WDOG_SYNCBUSY_CMD_MASK          0x2UL                              /* Bit mask for WDOG_CMD */
#define _WDOG_SYNCBUSY_CMD_DEFAULT       0x00000000UL                       /* Mode DEFAULT for WDOG_SYNCBUSY */
#define WDOG_SYNCBUSY_CMD_DEFAULT        (_WDOG_SYNCBUSY_CMD_DEFAULT << 1)  /* Shifted mode DEFAULT for WDOG_SYNCBUSY */

#endif /* __ARCH_ARM_SRC_EFM32_CHIP_EFM32_WDOG_H */

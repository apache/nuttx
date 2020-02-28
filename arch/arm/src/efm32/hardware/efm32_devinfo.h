/*******************************************************************************************************************************
 * arch/arm/src/efm32/chip/efm32_devinfo.h
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

#ifndef __ARCH_ARM_SRC_EFM32_CHIP_EFM32_DEVINFO_H
#define __ARCH_ARM_SRC_EFM32_CHIP_EFM32_DEVINFO_H

/*******************************************************************************************************************************
 * Included Files
 *******************************************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/efm32_memorymap.h"

#if !defined(CONFIG_EFM32_EFM32GG)
#  warning This is the EFM32GG header file; Review/modification needed for this architecture
#endif

/*******************************************************************************************************************************
 * Pre-processor Definitions
 *******************************************************************************************************************************/


/* MSC Register Offsets ********************************************************************************************************/

#define EFM32_DEVINFO_CAL_OFFSET                    0x0000
#define EFM32_DEVINFO_ADC0CALn_OFFSET(n)            (0x0004+(n)*4)
#define EFM32_DEVINFO_DAC0CALn_OFFSET(n)            (0x0018+(n)*4)
#define EFM32_DEVINFO_AUXHFRCOCALn_OFFSET(n)        (0x0024+(n)*4)
#define EFM32_DEVINFO_HFRCOCALn_OFFSET(n)           (0x002c+(n)*4)
#define EFM32_DEVINFO_MEMINFO_PAGE_SIZE_OFFSET      0x0034
#define EFM32_DEVINFO_UNIQUEL_OFFSET                0x0040
#define EFM32_DEVINFO_UNIQUEH_OFFSET                0x0044
#define EFM32_DEVINFO_MEMINFO_SIZE_OFFSET           0x0048
#define EFM32_DEVINFO_PART_OFFSET                   0x004c

/* MSC Register Addresses ******************************************************************************************************/

#define EFM32_DEVINFO_CAL                           (EFM32_DEVINFO_BASE+EFM32_DEVINFO_CAL_OFFSET)

#define EFM32_DEVINFO_ADC0CALn(n)                   (EFM32_DEVINFO_BASE+EFM32_DEVINFO_ADC0CALn_OFFSET(n))
#define EFM32_DEVINFO_ADC0CAL0                      (EFM32_DEVINFO_BASE+EFM32_DEVINFO_ADC0CALn_OFFSET(0))
#define EFM32_DEVINFO_ADC0CAL1                      (EFM32_DEVINFO_BASE+EFM32_DEVINFO_ADC0CALn_OFFSET(1))
#define EFM32_DEVINFO_ADC0CAL2                      (EFM32_DEVINFO_BASE+EFM32_DEVINFO_ADC0CALn_OFFSET(2))

#define EFM32_DEVINFO_DAC0CALn(n)                   (EFM32_DEVINFO_BASE+EFM32_DEVINFO_DAC0CALn_OFFSET(n))
#define EFM32_DEVINFO_DAC0CAL0                      (EFM32_DEVINFO_BASE+EFM32_DEVINFO_DAC0CALn_OFFSET(0))
#define EFM32_DEVINFO_DAC0CAL1                      (EFM32_DEVINFO_BASE+EFM32_DEVINFO_DAC0CALn_OFFSET(1))
#define EFM32_DEVINFO_DAC0CAL2                      (EFM32_DEVINFO_BASE+EFM32_DEVINFO_DAC0CALn_OFFSET(2))
#define EFM32_DEVINFO_AUXHFRCOCALn(n)               (EFM32_DEVINFO_BASE+EFM32_DEVINFO_AUXHFRCOCALn_OFFSET(n))
#define EFM32_DEVINFO_AUXHFRCOCAL0                  (EFM32_DEVINFO_BASE+EFM32_DEVINFO_AUXHFRCOCALn_OFFSET(0))
#define EFM32_DEVINFO_AUXHFRCOCAL1                  (EFM32_DEVINFO_BASE+EFM32_DEVINFO_AUXHFRCOCALn_OFFSET(1))
#define EFM32_DEVINFO_HFRCOCALn(n)                  (EFM32_DEVINFO_BASE+EFM32_DEVINFO_HFRCOCALn_OFFSET(n))
#define EFM32_DEVINFO_HFRCOCAL0                     (EFM32_DEVINFO_BASE+EFM32_DEVINFO_HFRCOCALn_OFFSET(0))
#define EFM32_DEVINFO_HFRCOCAL1                     (EFM32_DEVINFO_BASE+EFM32_DEVINFO_HFRCOCALn_OFFSET(1))
#define EFM32_DEVINFO_MEMINFO_PAGE_SIZE             (EFM32_DEVINFO_BASE+EFM32_DEVINFO_MEMINFO_PAGE_SIZE_OFFSET)
#define EFM32_DEVINFO_UNIQUEL                       (EFM32_DEVINFO_BASE+EFM32_DEVINFO_UNIQUEL_OFFSET)
#define EFM32_DEVINFO_UNIQUEH                       (EFM32_DEVINFO_BASE+EFM32_DEVINFO_UNIQUEH_OFFSET)
#define EFM32_DEVINFO_MEMINFO_SIZE                  (EFM32_DEVINFO_BASE+EFM32_DEVINFO_MEMINFO_SIZE_OFFSET)
#define EFM32_DEVINFO_PART                          (EFM32_DEVINFO_BASE+EFM32_DEVINFO_PART_OFFSET)

/* Bit fields for struct efm32_devinfo_s */

#define _DEVINFO_CAL_CRC_MASK                       0x0000FFFFUL /* Integrity CRC checksum mask */
#define _DEVINFO_CAL_CRC_SHIFT                      0            /* Integrity CRC checksum shift */
#define _DEVINFO_CAL_TEMP_MASK                      0x00FF0000UL /* Calibration temperature, DegC, mask */
#define _DEVINFO_CAL_TEMP_SHIFT                     16           /* Calibration temperature shift */

#define _DEVINFO_ADC0CAL0_1V25_GAIN_MASK            0x00007F00UL /* Gain for 1V25 reference, mask */
#define _DEVINFO_ADC0CAL0_1V25_GAIN_SHIFT           8            /* Gain for 1V25 reference, shift */
#define _DEVINFO_ADC0CAL0_1V25_OFFSET_MASK          0x0000007FUL /* Offset for 1V25 reference, mask */
#define _DEVINFO_ADC0CAL0_1V25_OFFSET_SHIFT         0            /* Offset for 1V25 reference, shift */
#define _DEVINFO_ADC0CAL0_2V5_GAIN_MASK             0x7F000000UL /* Gain for 2V5 reference, mask */
#define _DEVINFO_ADC0CAL0_2V5_GAIN_SHIFT            24           /* Gain for 2V5 reference, shift */
#define _DEVINFO_ADC0CAL0_2V5_OFFSET_MASK           0x007F0000UL /* Offset for 2V5 reference, mask */
#define _DEVINFO_ADC0CAL0_2V5_OFFSET_SHIFT          16           /* Offset for 2V5 reference, shift */

#define _DEVINFO_ADC0CAL1_VDD_GAIN_MASK             0x00007F00UL /* Gain for VDD reference, mask */
#define _DEVINFO_ADC0CAL1_VDD_GAIN_SHIFT            8            /* Gain for VDD reference, shift */
#define _DEVINFO_ADC0CAL1_VDD_OFFSET_MASK           0x0000007FUL /* Offset for VDD reference, mask */
#define _DEVINFO_ADC0CAL1_VDD_OFFSET_SHIFT          0            /* Offset for VDD reference, shift */
#define _DEVINFO_ADC0CAL1_5VDIFF_GAIN_MASK          0x7F000000UL /* Gain 5VDIFF for 5VDIFF reference, mask */
#define _DEVINFO_ADC0CAL1_5VDIFF_GAIN_SHIFT         24           /* Gain for 5VDIFF reference, mask */
#define _DEVINFO_ADC0CAL1_5VDIFF_OFFSET_MASK        0x007F0000UL /* Offset for 5VDIFF reference, mask */
#define _DEVINFO_ADC0CAL1_5VDIFF_OFFSET_SHIFT       16           /* Offset for 5VDIFF reference, shift */

#define _DEVINFO_ADC0CAL2_2XVDDVSS_OFFSET_MASK      0x0000007FUL /* Offset for 2XVDDVSS reference, mask */
#define _DEVINFO_ADC0CAL2_2XVDDVSS_OFFSET_SHIFT     0            /* Offset for 2XVDDVSS reference, shift */
#define _DEVINFO_ADC0CAL2_TEMP1V25_MASK             0xFFF00000UL /* Temperature reading at 1V25 reference, mask */
#define _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT            20           /* Temperature reading at 1V25 reference, DegC */

#define _DEVINFO_DAC0CAL0_1V25_GAIN_MASK            0x007F0000UL /* Gain for 1V25 reference, mask */
#define _DEVINFO_DAC0CAL0_1V25_GAIN_SHIFT           16           /* Gain for 1V25 reference, shift */
#define _DEVINFO_DAC0CAL0_1V25_CH1_OFFSET_MASK      0x00003F00UL /* Channel 1 offset for 1V25 reference, mask */
#define _DEVINFO_DAC0CAL0_1V25_CH1_OFFSET_SHIFT     8            /* Channel 1 offset for 1V25 reference, shift */
#define _DEVINFO_DAC0CAL0_1V25_CH0_OFFSET_MASK      0x0000003FUL /* Channel 0 offset for 1V25 reference, mask */
#define _DEVINFO_DAC0CAL0_1V25_CH0_OFFSET_SHIFT     0            /* Channel 0 offset for 1V25 reference, shift */

#define _DEVINFO_DAC0CAL1_2V5_GAIN_MASK             0x007F0000UL /* Gain for 2V5 reference, mask */
#define _DEVINFO_DAC0CAL1_2V5_GAIN_SHIFT            16           /* Gain for 2V5 reference, shift */
#define _DEVINFO_DAC0CAL1_2V5_CH1_OFFSET_MASK       0x00003F00UL /* Channel 1 offset for 2V5 reference, mask */
#define _DEVINFO_DAC0CAL1_2V5_CH1_OFFSET_SHIFT      8            /* Channel 1 offset for 2V5 reference, shift */
#define _DEVINFO_DAC0CAL1_2V5_CH0_OFFSET_MASK       0x0000003FUL /* Channel 0 offset for 2V5 reference, mask */
#define _DEVINFO_DAC0CAL1_2V5_CH0_OFFSET_SHIFT      0            /* Channel 0 offset for 2V5 reference, shift */

#define _DEVINFO_DAC0CAL2_VDD_GAIN_MASK             0x007F0000UL /* Gain for VDD reference, mask */
#define _DEVINFO_DAC0CAL2_VDD_GAIN_SHIFT            16           /* Gain for VDD reference, shift */
#define _DEVINFO_DAC0CAL2_VDD_CH1_OFFSET_MASK       0x00003F00UL /* Channel 1 offset for VDD reference, mask */
#define _DEVINFO_DAC0CAL2_VDD_CH1_OFFSET_SHIFT      8            /* Channel 1 offset for VDD reference, shift */
#define _DEVINFO_DAC0CAL2_VDD_CH0_OFFSET_MASK       0x0000003FUL /* Channel 0 offset for VDD reference, mask */
#define _DEVINFO_DAC0CAL2_VDD_CH0_OFFSET_SHIFT      0            /* Channel 0 offset for VDD reference, shift*/

#define _DEVINFO_AUXHFRCOCAL0_BAND1_MASK            0x000000FFUL /* 1MHz tuning value for AUXHFRCO, mask */
#define _DEVINFO_AUXHFRCOCAL0_BAND1_SHIFT           0            /* 1MHz tuning value for AUXHFRCO, shift */
#define _DEVINFO_AUXHFRCOCAL0_BAND7_MASK            0x0000FF00UL /* 7MHz tuning value for AUXHFRCO, mask */
#define _DEVINFO_AUXHFRCOCAL0_BAND7_SHIFT           8            /* 7MHz tuning value for AUXHFRCO, shift */
#define _DEVINFO_AUXHFRCOCAL0_BAND11_MASK           0x00FF0000UL /* 11MHz tuning value for AUXHFRCO, mask */
#define _DEVINFO_AUXHFRCOCAL0_BAND11_SHIFT          16           /* 11MHz tuning value for AUXHFRCO, shift */
#define _DEVINFO_AUXHFRCOCAL0_BAND14_MASK           0xFF000000UL /* 14MHz tuning value for AUXHFRCO, mask */
#define _DEVINFO_AUXHFRCOCAL0_BAND14_SHIFT          24           /* 14MHz tuning value for AUXHFRCO, shift */

#define _DEVINFO_AUXHFRCOCAL1_BAND21_MASK           0x000000FFUL /* 21MHz tuning value for AUXHFRCO, mask */
#define _DEVINFO_AUXHFRCOCAL1_BAND21_SHIFT          0            /* 21MHz tuning value for AUXHFRCO, shift */
#define _DEVINFO_AUXHFRCOCAL1_BAND28_MASK           0x0000FF00UL /* 28MHz tuning value for AUXHFRCO, shift */
#define _DEVINFO_AUXHFRCOCAL1_BAND28_SHIFT          8            /* 28MHz tuning value for AUXHFRCO, mask */

#define _DEVINFO_HFRCOCAL0_BAND1_MASK               0x000000FFUL /* 1MHz tuning value for HFRCO, mask */
#define _DEVINFO_HFRCOCAL0_BAND1_SHIFT              0            /* 1MHz tuning value for HFRCO, shift */
#define _DEVINFO_HFRCOCAL0_BAND7_MASK               0x0000FF00UL /* 7MHz tuning value for HFRCO, mask */
#define _DEVINFO_HFRCOCAL0_BAND7_SHIFT              8            /* 7MHz tuning value for HFRCO, shift */
#define _DEVINFO_HFRCOCAL0_BAND11_MASK              0x00FF0000UL /* 11MHz tuning value for HFRCO, mask */
#define _DEVINFO_HFRCOCAL0_BAND11_SHIFT             16           /* 11MHz tuning value for HFRCO, shift */
#define _DEVINFO_HFRCOCAL0_BAND14_MASK              0xFF000000UL /* 14MHz tuning value for HFRCO, mask */
#define _DEVINFO_HFRCOCAL0_BAND14_SHIFT             24           /* 14MHz tuning value for HFRCO, shift */

#define _DEVINFO_HFRCOCAL1_BAND21_MASK              0x000000FFUL /* 21MHz tuning value for HFRCO, mask */
#define _DEVINFO_HFRCOCAL1_BAND21_SHIFT             0            /* 21MHz tuning value for HFRCO, shift */
#define _DEVINFO_HFRCOCAL1_BAND28_MASK              0x0000FF00UL /* 28MHz tuning value for HFRCO, shift */
#define _DEVINFO_HFRCOCAL1_BAND28_SHIFT             8            /* 28MHz tuning value for HFRCO, mask */

#define _DEVINFO_MEMINFO_FLASH_PAGE_SIZE_MASK       0xFF000000UL /* Flash page size (refer to ref.man for encoding) mask */
#define _DEVINFO_MEMINFO_FLASH_PAGE_SIZE_SHIFT      24           /* Flash page size shift */

#define _DEVINFO_UNIQUEL_MASK                       0xFFFFFFFFUL /* Lower part of  64-bit device unique number */
#define _DEVINFO_UNIQUEL_SHIFT                      0            /* Unique Low 32-bit shift */

#define _DEVINFO_UNIQUEH_MASK                       0xFFFFFFFFUL /* High part of  64-bit device unique number */
#define _DEVINFO_UNIQUEH_SHIFT                      0            /* Unique High 32-bit shift */

#define _DEVINFO_MEMINFO_SIZE_SRAM_MASK             0xFFFF0000UL /* Flash size in kilobytes */
#define _DEVINFO_MEMINFO_SIZE_SRAM_SHIFT            16           /* Bit position for flash size */
#define _DEVINFO_MEMINFO_SIZE_FLASH_MASK            0x0000FFFFUL /* SRAM size in kilobytes */
#define _DEVINFO_MEMINFO_SIZE_FLASH_SHIFT           0            /* Bit position for SRAM size */

#define _DEVINFO_PART_PROD_REV_MASK                 0xFF000000UL /* Production revision */
#define _DEVINFO_PART_PROD_REV_SHIFT                24           /* Bit position for production revision */
#define _DEVINFO_PART_DEVICE_FAMILY_MASK            0x00FF0000UL /* Device Family, 0x47 for Gecko */
#define _DEVINFO_PART_DEVICE_FAMILY_SHIFT           16           /* Bit position for device family */
#define _DEVINFO_PART_DEVICE_FAMILY_G               71           /* Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_GG              72           /* Giant Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_TG              73           /* Tiny Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_LG              74           /* Leopard Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_WG              75           /* Wonder Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_ZG              76           /* Zero Gecko Device Family */
#define _DEVINFO_PART_DEVICE_NUMBER_MASK            0x0000FFFFUL /* Device number */
#define _DEVINFO_PART_DEVICE_NUMBER_SHIFT           0            /* Bit position for device number */

/*******************************************************************************************************************************
 * Public Type Definitions
 *******************************************************************************************************************************/


#endif /* __ARCH_ARM_SRC_EFM32_CHIP_EFM32_DEVINFO_H */

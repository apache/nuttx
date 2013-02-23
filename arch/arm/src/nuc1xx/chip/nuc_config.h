/********************************************************************************************
 * arch/arm/src/nuc1xx/chip/nuc_config.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_NUC1XX_CHIP_NUC_CONFIG_H
#define __ARCH_ARM_SRC_NUC1XX_CHIP_NUC_CONFIG_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* Register offsets *************************************************************************/

#define NUC_CONFIG0_OFFSET     0x0000 /* Config 0 */
#define NUC_CONFIG1_OFFSET     0x0004 /* Config 1 */

/* Register addresses ***********************************************************************/

#define NUC_CONFIG0            (NUC_CONFIG_BASE+NUC_CONFIG0_OFFSET)
#define NUC_CONFIG1            (NUC_CONFIG_BASE+NUC_CONFIG1_OFFSET)

/* Register bit-field definitions ***********************************************************/

/* Config 0 */

#define CONFIG0_DFEN           (1 << 0)  /* Bit 0:  Data FLASH enable */
#define CONFIG0_LOCK           (1 << 1)  /* Bit 1:  Security lock */
#define CONFIG0_CBS            (1 << 7)  /* Bit 7:  Chip boot selection. 1=LDROM 0=APROM */
#define CONFIG0_CBORST         (1 << 20) /* Bit 20: Brown-out reset enable */
#define CONFIG0_CBOV_SHIFT     (21)      /* Bits 21-22: Brown-out voltage selection */
#define CONFIG0_CBOV_MASK      (3 << CONFIG0_CBOV_SHIFT)
#  define CONFIG0_CBOV_2p2V    (0 << CONFIG0_CBOV_SHIFT) /* 2.2V */
#  define CONFIG0_CBOV_2p7V    (1 << CONFIG0_CBOV_SHIFT) /* 2.7V */
#  define CONFIG0_CBOV_3p8V    (2 << CONFIG0_CBOV_SHIFT) /* 3.8V */
#  define CONFIG0_CBOV_4p5V    (3 << CONFIG0_CBOV_SHIFT) /* 4.5V */
#define CONFIG0_CBODEN         (1 << 23) /* Bit 23: Brown out detector enable */
#define CONFIG0_CFOSC_SHIFT    (24)      /* Bits 24-26: CPU clock source selection after reset */
#define CONFIG0_CFOSC_MASK     (7 << CONFIG0_CFOSC_SHIFT)
#  define CONFIG0_CFOSC_XTALHI (0 << CONFIG0_CFOSC_SHIFT)
#  define CONFIG0_CFOSC_INTHI  (7 << CONFIG0_CFOSC_SHIFT)
#define CONFIG0_CKF            (1 << 28) /* Bit 28: XT1 clock filter enable */

/* My NuTiny-SDK-NUC120 was programmed this way when I got it:
 * CKF=1
 * CFOSC=0
 * CBODEN=1
 * CBOV=3
 * CBORST=1
 * CBS=0
 * LOCK=1
 * DFEN=1
 */

#define CONFIG0_FACTORY_DEFAULT (0xf8ffff7f)

/* Config 1 */

#define CONFIG1_DFBADR_SHIFT   (0)       /* Bits 0-19: Data FLASH base address */
#define CONFIG1_DFBADR_MASK    (0x000fffff << CONFIG1_DFBADR_SHIFT)

/* My NuTiny-SDK-NUC120 was programmed this way when I got it:
 * DFBADR=0x05000
 */

#define CONFIG1_FACTORY_DEFAULT (0x00005000)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_NUC1XX_CHIP_NUC_CONFIG_H */

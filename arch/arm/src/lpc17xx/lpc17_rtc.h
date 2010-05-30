/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_rtc.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_RTC_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_RTC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "lp17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/
/* Miscellaneous registers */

#define LPC17_RTC_ILR_OFFSET    0x0000 /* Interrupt Location Register */
#define LPC17_RTC_CCR_OFFSET    0x0008 /* Clock Control Register */
#define LPC17_RTC_CIIR_OFFSET   0x000c /* Counter Increment Interrupt Register */
#define LPC17_RTC_AMR_OFFSET    0x0010 /* Alarm Mask Register */
#define LPC17_RTC_AUXEN_OFFSET  0x0058 /* RTC Auxiliary Enable register */
#define LPC17_RTC_AUX_OFFSET    0x005c /* RTC Auxiliary control register */

/* Consolidated time registers */

#define LPC17_RTC_CTIME0_OFFSET 0x0014 /* Consolidated Time Register 0 */
#define LPC17_RTC_CTIME1_OFFSET 0x0018 /* Consolidated Time Register 1 */
#define LPC17_RTC_CTIME2_OFFSET 0x001c /* Consolidated Time Register 2 */

/* Time counter registers */

#define LPC17_RTC_SEC_OFFSET    0x0020 /* Seconds Counter */
#define LPC17_RTC_MIN_OFFSET    0x0024 /* Minutes Register */
#define LPC17_RTC_HOUR_OFFSET   0x0028 /* Hours Register */
#define LPC17_RTC_DOM_OFFSET    0x002c /* Day of Month Register */
#define LPC17_RTC_DOW_OFFSET    0x0030 /* Day of Week Register */
#define LPC17_RTC_DOY_OFFSET    0x0034 /* Day of Year Register */
#define LPC17_RTC_MONTH_OFFSET  0x0038 /* Months Register */
#define LPC17_RTC_YEAR_OFFSET   0x003c /* Years Register */
#define LPC17_RTC_CALIB_OFFSET  0x0040 /* Calibration Value Register */

/* General purpose registers */

#define LPC17_RTC_GPREG0_OFFSET 0x0044 /* General Purpose Register 0 */
#define LPC17_RTC_GPREG1_OFFSET 0x0048 /* General Purpose Register 1 */
#define LPC17_RTC_GPREG2_OFFSET 0x004c /* General Purpose Register 2 */
#define LPC17_RTC_GPREG3_OFFSET 0x0050 /* General Purpose Register 3 */
#define LPC17_RTC_GPREG4_OFFSET 0x0054 /* General Purpose Register 4 */

/* Alarm register group */

#define LPC17_RTC_ALSEC_OFFSET  0x0060 /* Alarm value for Seconds */
#define LPC17_RTC_ALMIN_OFFSET  0x0064 /* Alarm value for Minutes */
#define LPC17_RTC_ALHOUR_OFFSET 0x0068 /* Alarm value for Hours */
#define LPC17_RTC_ALDOM_OFFSET  0x006c /* Alarm value for Day of Month */
#define LPC17_RTC_ALDOW_OFFSET  0x0070 /* Alarm value for Day of Week */
#define LPC17_RTC_ALDOY_OFFSET  0x0074 /* Alarm value for Day of Year */
#define LPC17_RTC_ALMON_OFFSET  0x0078 /* Alarm value for Months  */
#define LPC17_RTC_ALYEAR_OFFSET 0x007c /* Alarm value for Year */

/* Register addresses ***************************************************************/
/* Miscellaneous registers */

#define LPC17_RTC_ILR           (LPC17_RTC_BASE+LPC17_RTC_ILR_OFFSET)
#define LPC17_RTC_CCR           (LPC17_RTC_BASE+LPC17_RTC_CCR_OFFSET)
#define LPC17_RTC_CIIR          (LPC17_RTC_BASE+LPC17_RTC_CIIR_OFFSET)
#define LPC17_RTC_AMR           (LPC17_RTC_BASE+LPC17_RTC_AMR_OFFSET)
#define LPC17_RTC_AUXEN         (LPC17_RTC_BASE+LPC17_RTC_AUXEN_OFFSET)
#define LPC17_RTC_AUX           (LPC17_RTC_BASE+LPC17_RTC_AUX_OFFSET)

/* Consolidated time registers */

#define LPC17_RTC_CTIME0        (LPC17_RTC_BASE+LPC17_RTC_CTIME0_OFFSET)
#define LPC17_RTC_CTIME1        (LPC17_RTC_BASE+LPC17_RTC_CTIME1_OFFSET)
#define LPC17_RTC_CTIME2        (LPC17_RTC_BASE+LPC17_RTC_CTIME2_OFFSET)

/* Time counter registers */

#define LPC17_RTC_SEC           (LPC17_RTC_BASE+LPC17_RTC_SEC_OFFSET)
#define LPC17_RTC_MIN           (LPC17_RTC_BASE+LPC17_RTC_MIN_OFFSET)
#define LPC17_RTC_HOUR          (LPC17_RTC_BASE+LPC17_RTC_HOUR_OFFSET)
#define LPC17_RTC_DOM           (LPC17_RTC_BASE+LPC17_RTC_DOM_OFFSET)
#define LPC17_RTC_DOW           (LPC17_RTC_BASE+LPC17_RTC_DOW_OFFSET)
#define LPC17_RTC_DOY           (LPC17_RTC_BASE+LPC17_RTC_DOY_OFFSET)
#define LPC17_RTC_MONTH         (LPC17_RTC_BASE+LPC17_RTC_MONTH_OFFSET)
#define LPC17_RTC_YEAR          (LPC17_RTC_BASE+LPC17_RTC_YEAR_OFFSET)
#define LPC17_RTC_CALIB         (LPC17_RTC_BASE+LPC17_RTC_CALIB_OFFSET)

/* General purpose registers */

#define LPC17_RTC_GPREG0        (LPC17_RTC_BASE+LPC17_RTC_GPREG0_OFFSET)
#define LPC17_RTC_GPREG1        (LPC17_RTC_BASE+LPC17_RTC_GPREG1_OFFSET)
#define LPC17_RTC_GPREG2        (LPC17_RTC_BASE+LPC17_RTC_GPREG2_OFFSET)
#define LPC17_RTC_GPREG3        (LPC17_RTC_BASE+LPC17_RTC_GPREG3_OFFSET)
#define LPC17_RTC_GPREG4        (LPC17_RTC_BASE+LPC17_RTC_GPREG4_OFFSET)

/* Alarm register group */

#define LPC17_RTC_ALSEC         (LPC17_RTC_BASE+LPC17_RTC_ALSEC_OFFSET)
#define LPC17_RTC_ALMIN         (LPC17_RTC_BASE+LPC17_RTC_ALMIN_OFFSET)
#define LPC17_RTC_ALHOUR        (LPC17_RTC_BASE+LPC17_RTC_ALHOUR_OFFSET)
#define LPC17_RTC_ALDOM         (LPC17_RTC_BASE+LPC17_RTC_ALDOM_OFFSET)
#define LPC17_RTC_ALDOW         (LPC17_RTC_BASE+LPC17_RTC_ALDOW_OFFSET)
#define LPC17_RTC_ALDOY         (LPC17_RTC_BASE+LPC17_RTC_ALDOY_OFFSET)
#define LPC17_RTC_ALMON         (LPC17_RTC_BASE+LPC17_RTC_ALMON_OFFSET)
#define LPC17_RTC_ALYEAR        (LPC17_RTC_BASE+LPC17_RTC_ALYEAR_OFFSET)

/* Register bit definitions *********************************************************/
/* Miscellaneous registers */
/* Interrupt Location Register */

#define RTC_ILR_

/* Clock Control Register */

#define RTC_CCR_

/* Counter Increment Interrupt Register */

#define RTC_CIIR_

/* Alarm Mask Register */

#define RTC_AMR_

/* RTC Auxiliary Enable register */

#define RTC_AUXEN_

/* RTC Auxiliary control register */

#define RTC_AUX_

/* Consolidated time registers */
/* Consolidated Time Register 0 */

#define RTC_CTIME0_

/* Consolidated Time Register 1 */

#define RTC_CTIME1_

/* Consolidated Time Register 2 */

#define RTC_CTIME2_

/* Time counter registers */
/* Seconds Counter */

#define RTC_SEC_

/* Minutes Register */

#define RTC_MIN_

/* Hours Register */

#define RTC_HOUR_

/* Day of Month Register */

#define RTC_DOM_

/* Day of Week Register */

#define RTC_DOW_

/* Day of Year Register */

#define RTC_DOY_

/* Months Register */

#define RTC_MONTH_

/* Years Register */

#define RTC_YEAR_

/* Calibration Value Register */

#define RTC_CALIB_

/* General purpose registers */
/* General Purpose Register 0 */

#define RTC_GPREG0_

/* General Purpose Register 1 */

#define RTC_GPREG1_

/* General Purpose Register 2 */

#define RTC_GPREG2_

/* General Purpose Register 3 */

#define RTC_GPREG3_

/* General Purpose Register 4 */

#define RTC_GPREG4_

/* Alarm register group */
/* Alarm value for Seconds */

#define RTC_ALSEC_

/* Alarm value for Minutes */

#define RTC_ALMIN_

/* Alarm value for Hours */

#define RTC_ALHOUR_

/* Alarm value for Day of Month */

#define RTC_ALDOM_

/* Alarm value for Day of Week */

#define RTC_ALDOW_

/* Alarm value for Day of Year */

#define RTC_ALDOY_

/* Alarm value for Months  */

#define RTC_ALMON_

/* Alarm value for Year */

#define RTC_ALYEAR_

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_RTC_H */

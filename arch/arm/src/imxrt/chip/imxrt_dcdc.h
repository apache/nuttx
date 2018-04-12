/****************************************************************************************************
 * arch/arm/src/imxrt/imxrt_dcdc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Janne Rosberg <janne@offcode.fi>
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_DCDC_H
#define __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_DCDC_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/imxrt_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register offsets *********************************************************************************/

#define IMXRT_DCDC_REG0_OFFSET              0x0000  /* DCDC Register 0 */
#define IMXRT_DCDC_REG1_OFFSET              0x0004  /* DCDC Register 1 */
#define IMXRT_DCDC_REG2_OFFSET              0x0008  /* DCDC Register 2 */
#define IMXRT_DCDC_REG3_OFFSET              0x000c  /* DCDC Register 3 */

/* Register addresses *******************************************************************************/

#define IMXRT_DCDC_REG0                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG0_OFFSET)
#define IMXRT_DCDC_REG1                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG1_OFFSET)
#define IMXRT_DCDC_REG2                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG2_OFFSET)
#define IMXRT_DCDC_REG3                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG3_OFFSET)

/* Register bit definitions *************************************************************************/

/* Register 0 */

#define DCDC_REG0_PWD_ZCD                   (1 << 0)  /* Bit 0:  Power down the zero cross detection */
#define DCDC_REG0_DISABLE_AUTO_CLK_SWITCH   (1 << 1)  /* Bit 1:  Disable automatic clock switch */
#define DCDC_REG0_SEL_CLK                   (1 << 2)  /* Bit 2:  Select 24 MHz Crystal clock */
#define DCDC_REG0_PWD_OSC_INT               (1 << 3)  /* Bit 3:  Power down internal osc */
#define DCDC_REG0_PWD_CUR_SNS_CMP           (1 << 4)  /* Bit 4:  The power down signal of the current detector */
#define DCDC_REG0_CUR_SNS_THRSH_SHIFT       (5)       /* Bits 5-7: threshold of current detector */
#define DCDC_REG0_CUR_SNS_THRSH_MASK        (0x7 << DCDC_REG0_CUR_SNS_THRSH_SHIFT)
#  define DCDC_REG0_CUR_SNS_THRSH_150MA     ((uint32_t)(0) << DCDC_REG0_CUR_SNS_THRSH_SHIFT)
#  define DCDC_REG0_CUR_SNS_THRSH_250MA     ((uint32_t)(1) << DCDC_REG0_CUR_SNS_THRSH_SHIFT)
#  define DCDC_REG0_CUR_SNS_THRSH_350MA     ((uint32_t)(2) << DCDC_REG0_CUR_SNS_THRSH_SHIFT)
#  define DCDC_REG0_CUR_SNS_THRSH_450MA     ((uint32_t)(3) << DCDC_REG0_CUR_SNS_THRSH_SHIFT)
#  define DCDC_REG0_CUR_SNS_THRSH_550MA     ((uint32_t)(4) << DCDC_REG0_CUR_SNS_THRSH_SHIFT)
#  define DCDC_REG0_CUR_SNS_THRSH_650MA     ((uint32_t)(5) << DCDC_REG0_CUR_SNS_THRSH_SHIFT)
#define DCDC_REG0_PWD_OVERCUR_DET           (1 << 8)  /* Bit 8:  Power down overcurrent detection comparator */
#define DCDC_REG0_OVERCUR_TIRG_ADJ_SHIFT    (9)       /* Bits 9-10:  The threshold of over current detection */
#define DCDC_REG0_OVERCUR_TIRG_ADJ_MASK     (0x3 << DCDC_REG0_OVERCUR_TIRG_ADJ_SHIFT)
#  define DCDC_REG0_OVERCUR_TIRG_ADJ_1A_025 ((uint32_t)(0) << DCDC_REG0_OVERCUR_TIRG_ADJ_SHIFT)
#  define DCDC_REG0_OVERCUR_TIRG_ADJ_2A_025 ((uint32_t)(1) << DCDC_REG0_OVERCUR_TIRG_ADJ_SHIFT)
#  define DCDC_REG0_OVERCUR_TIRG_ADJ_1A_02  ((uint32_t)(2) << DCDC_REG0_OVERCUR_TIRG_ADJ_SHIFT)
#  define DCDC_REG0_OVERCUR_TIRG_ADJ_2A_02  ((uint32_t)(3) << DCDC_REG0_OVERCUR_TIRG_ADJ_SHIFT)
#define DCDC_REG0_PWD_CMP_BATT_DET          (1 << 11) /* Bit 11: Power down the low voltage detection comparator */
#define DCDC_REG0_ADJ_POSLIMIT_BUCK_SHIFT   (12)      /* Bits 12-15: Adjust value to poslimit_buck register */
#define DCDC_REG0_ADJ_POSLIMIT_BUCK_MASK    (0xf << DCDC_REG0_ADJ_POSLIMIT_BUCK_SHIFT)
#  define DCDC_REG0_ADJ_POSLIMIT_BUCK(n)    ((uint32_t)(n) << DCDC_REG0_ADJ_POSLIMIT_BUCK_SHIFT)
#define DCDC_REG0_EN_LP_OVERLOAD_SNS        (1 << 16) /* Bit 16: Enable the overload detection in power save mode */
#define DCDC_REG0_PWD_HIGH_VOLT_DET         (1 << 17) /* Bit 17: Power down overvoltage detection comparator */
#define DCDC_REG0_LP_OVERLOAD_THRSH_SHIFT   (18)      /* Bits 18-19: the threshold of the counting number */
#define DCDC_REG0_LP_OVERLOAD_THRSH_MASK    (0x3 << DCDC_REG0_LP_OVERLOAD_THRSH_SHIFT)
#  define DCDC_REG0_LP_OVERLOAD_THRSH_32    ((uint32_t)(0) << DCDC_REG0_LP_OVERLOAD_THRSH_SHIFT)
#  define DCDC_REG0_LP_OVERLOAD_THRSH_64    ((uint32_t)(1) << DCDC_REG0_LP_OVERLOAD_THRSH_SHIFT)
#  define DCDC_REG0_LP_OVERLOAD_THRSH_16    ((uint32_t)(2) << DCDC_REG0_LP_OVERLOAD_THRSH_SHIFT)
#  define DCDC_REG0_LP_OVERLOAD_THRSH_8     ((uint32_t)(3) << DCDC_REG0_LP_OVERLOAD_THRSH_SHIFT)
#define DCDC_REG0_LP_OVERLOAD_FREQ_SEL      (1 << 20) /* Bit 20: The period of counting the charging times in power save mode */
#define DCDC_REG0_LP_OVERLOAD_FREQ_SEL_8    (0 << 20) /* Bit 20: The period of counting the charging times in power save mode */
#define DCDC_REG0_LP_OVERLOAD_FREQ_SEL_16   (1 << 20) /* Bit 20: Fhe period of counting the charging times in power save mode */
#define DCDC_REG0_LP_HIGH_HYS               (1 << 21) /* Bit 21: Adjust hysteretic value in low power from 12.5mV to 25mV */
                                                      /* Bits 22-26  Reserved */
#define DCDC_REG0_XTALOK_DISABLE            (1 << 27) /* Bit 27: Disable xtalok detection circuit */
#define DCDC_REG0_CURRENT_ALERT_RESET       (1 << 28) /* Bit 28: Reset current alert signal */
#define DCDC_REG0_XTAL_24M_OK               (1 << 29) /* Bit 29: Set to 1 to switch internal ring osc to xtal 24M */
                                                      /* Bit 30: Reserved */
#define DCDC_REG0_STS_DC_OK                 (1 << 31) /* Bit 31: Status register to indicate DCDC status */

/* Register 1 */

#define DCDC_REG1_POSLIMIT_BUCK_IN_SHIFT    (0)       /* Bits 0-6: Upper limit duty cycle limit in DC-DC converter */
#define DCDC_REG1_POSLIMIT_BUCK_IN_MASK     (0x7f << DCDC_REG1_POSLIMIT_BUCK_IN_SHIFT)
#  define DCDC_REG1_POSLIMIT_BUCK_IN(n)     ((uint32_t)(n) << DCDC_REG1_POSLIMIT_BUCK_IN_SHIFT)
#define DCDC_REG1_REG_FBK_SEL_SHIFT         (7)       /* Bits 7-8: Select the feedback point of the internal regulator */
#define DCDC_REG1_REG_FBK_SEL_MASK          (0x3 << DCDC_REG1_REG_FBK_SEL_SHIFT)
#  define DCDC_REG1_REG_FBK_SEL(n)          ((uint32_t)(n) << DCDC_REG1_REG_FBK_SEL_SHIFT)
                                                      /* Bits 9-11: Reserved */
#define DCDC_REG1_LP_CMP_ISRC_SEL_SHIFT     (12)      /* Bits 12-13: Set the current bias of low power comparator */
#define DCDC_REG1_LP_CMP_ISRC_SEL_MASK      (0x3 << DCDC_REG1_LP_CMP_ISRC_SEL_SHIFT)
#  define DCDC_REG1_LP_CMP_ISRC_SEL(n)      ((uint32_t)(n) << DCDC_REG1_LP_CMP_ISRC_SEL_SHIFT)
#define DCDC_REG1_NEGLIMIT_IN_SHIFT         (14)      /* Bits 14-20: Set the current bias of low power comparator */
#define DCDC_REG1_NEGLIMIT_IN_MASK          (0x3f << DCDC_REG1_NEGLIMIT_IN_SHIFT)
#  define DCDC_REG1_NEGLIMIT_IN(n)          ((uint32_t)(n) << DCDC_REG1_NEGLIMIT_IN_SHIFT)
#define DCDC_REG1_LOOPCTRL_HST_THRESH       (1 << 21) /* Bit 21: Increase the threshold detection for common mode analog comparator */
                                                      /* Bit 22: Reserved */
#define DCDC_REG1_LOOPCTRL_EN_HYST          (1 << 23) /* Bit 23: Enable hysteresis in switching converter */
#define DCDC_REG1_VBG_TRIM_SHIFT            (24)      /* Bits 24-28: Trim bandgap voltage */
#define DCDC_REG1_VBG_TRIM_MASK             (0x1f << DCDC_REG1_VBG_TRIM_SHIFT)
#  define DCDC_REG1_VBG_TRIM(n)             ((uint32_t)(n) << DCDC_REG1_VBG_TRIM_SHIFT)
                                                      /* Bit 29-31: Reserved */

/* Register 3 */

#define DCDC_REG3_TRG_SHIFT                 (0)       /* Bits 0-4: Target value of VDD_SOC, 25 mV each step */
#define DCDC_REG3_TRG_MASK                  (0x1f << DCDC_REG3_TRG_SHIFT)
#  define DCDC_REG3_TRG(n)                  ((uint32_t)(n) << DCDC_REG3_TRG_SHIFT)
                                                      /* Bit 5-7: Reserved */
#define DCDC_REG3_TARGET_LP_SHIFT           (8)       /* Bits 8-10: Target value of standby (low power) mode */
#define DCDC_REG3_TARGET_LP_MASK            (0x7 << DCDC_REG3_TARGET_LP_SHIFT)
#  define DCDC_REG3_TARGET_LP_(n)           ((uint32_t)(n) << DCDC_REG3_TARGET_LP_SHIFT)
                                                      /* Bit 11-23: Reserved */
#define DCDC_REG3_MINPWR_DC_HALFCLK         (1 << 24) /* Bit 24: Set DCDC clock to half freqeuncy for continuous mode */
                                                      /* Bit 25-26: Reserved */
#define DCDC_REG3_MISC_DELAY_TIMING         (1 << 27) /* Bit 27: Adjust delay to reduce ground noise */
#define DCDC_REG3_MISC_DISABLE_FET_LOGIC    (1 << 28) /* Bit 28: Datasheet: reserved? */
                                                      /* Bit 29: Reserved */
#define DCDC_REG3_DISABLE_STEP              (1 << 30) /* Bit 30: Disable stepping */
                                                      /* Bit 31: Reserved */

#endif /* __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_DCDC_H */

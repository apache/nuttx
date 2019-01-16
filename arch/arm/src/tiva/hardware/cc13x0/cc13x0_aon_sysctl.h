/********************************************************************************************************************
 * arch/arm/src/tiva/hardware/cc13x0/cc13x0_aon_sysctl.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible BSD license:
 *
 *   Copyright (c) 2015-2017, Texas Instruments Incorporated
 *   All rights reserved.
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
 ********************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_AON_SYSCTL_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_AON_SYSCTL_H

/********************************************************************************************************************
 * Included Files
 ********************************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/********************************************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************************************/

/* AON SYSCTL Register Offsets **************************************************************************************/

#define TIVA_AON_SYSCTL_PWRCTL_OFFSET             0x0000  /* Power Management */
#define TIVA_AON_SYSCTL_RESETCTL_OFFSET           0x0004  /* Reset Management */
#define TIVA_AON_SYSCTL_SLEEPCTL_OFFSET           0x0008  /* Sleep Mode */

/* AON SYSCTL Register Addresses ************************************************************************************/

#define TIVA_AON_SYSCTL_PWRCTL                    (TIVA_AON_SYSCTL_BASE + TIVA_AON_SYSCTL_PWRCTL_OFFSET)
#define TIVA_AON_SYSCTL_RESETCTL                  (TIVA_AON_SYSCTL_BASE + TIVA_AON_SYSCTL_RESETCTL_OFFSET)
#define TIVA_AON_SYSCTL_SLEEPCTL                  (TIVA_AON_SYSCTL_BASE + TIVA_AON_SYSCTL_SLEEPCTL_OFFSET)

/* AON SYSCTL Register Bitfield Definitions *************************************************************************/

/* AON_SYSCTL_PWRCTL */

#define AON_SYSCTL_PWRCTL_DCDC_EN                 (1 << 0)  /* Bit 0: Select to use DCDC regulator during recharge of VDDR */
#  define AON_SYSCTL_PWRCTL_DCDC_EN_GLD0          (0)
#  define AON_SYSCTL_PWRCTL_DCDC_EN_DCDC          AON_SYSCTL_PWRCTL_DCDC_EN
#define AON_SYSCTL_PWRCTL_EXT_REG_MODE            (1 << 1)  /* Bit 1: Status of source for VDDR supply */
#  define AON_SYSCTL_PWRCTL_EXT_REG_INTERNAL      (0)
#  define AON_SYSCTL_PWRCTL_EXT_REG_EXTERNAL      AON_SYSCTL_PWRCTL_EXT_REG_MODE
#define AON_SYSCTL_PWRCTL_DCDC_ACTIVE             (1 << 2)  /* Bit 2: Select to use DCDC regulator for VDDR in active mode */
#  define AON_SYSCTL_PWRCTL_DCDC_ACTIVE_GLD0      (0)
#  define AON_SYSCTL_PWRCTL_DCDC_ACTIVE_DCDC      AON_SYSCTL_PWRCTL_DCDC_ACTIVE

/* AON_SYSCTL_RESETCTL */

#define AON_SYSCTL_RESETCTL_RESET_SRC_SHIFT       (1)     /* Bits 1-3: Shows the source of the last system reset */
#define AON_SYSCTL_RESETCTL_RESET_SRC_MASK        (7 << AON_SYSCTL_RESETCTL_RESET_SRC_SHIFT)
#  define AON_SYSCTL_RESETCTL_RESET_SRC_PWR_ON    (0 << AON_SYSCTL_RESETCTL_RESET_SRC_SHIFT)
#  define AON_SYSCTL_RESETCTL_RESET_SRC_PIN_RESET (1 << AON_SYSCTL_RESETCTL_RESET_SRC_SHIFT)
#  define AON_SYSCTL_RESETCTL_RESET_SRC_VDDS_LOSS (2 << AON_SYSCTL_RESETCTL_RESET_SRC_SHIFT)
#  define AON_SYSCTL_RESETCTL_RESET_SRC_VDD_LOSS  (3 << AON_SYSCTL_RESETCTL_RESET_SRC_SHIFT)
#  define AON_SYSCTL_RESETCTL_RESET_SRC_VDDR_LOSS (4 << AON_SYSCTL_RESETCTL_RESET_SRC_SHIFT)
#  define AON_SYSCTL_RESETCTL_RESET_SRC_CLK_LOSS  (5 << AON_SYSCTL_RESETCTL_RESET_SRC_SHIFT)
#  define AON_SYSCTL_RESETCTL_RESET_SRC_SYSRESET  (6 << AON_SYSCTL_RESETCTL_RESET_SRC_SHIFT)
#  define AON_SYSCTL_RESETCTL_RESET_SRC_WARMRESET (7 << AON_SYSCTL_RESETCTL_RESET_SRC_SHIFT)
#define AON_SYSCTL_RESETCTL_CLK_LOSS_EN           (1 << 4)  /* Bit 4:  Controls reset generation in case SCLK_LF is lost */
#  define AON_SYSCTL_RESETCTL_CLK_LOSS_IGNORE     (0)
#  define AON_SYSCTL_RESETCTL_CLK_LOSS_RESET      AON_SYSCTL_RESETCTL_CLK_LOSS_EN
#define AON_SYSCTL_RESETCTL_VDD_LOSS_EN           (1 << 5)  /* Bit 5:  Controls reset generation in case VDD is lost */
#  define AON_SYSCTL_RESETCTL_VDD_LOSS_IGNORE     (0)
#  define AON_SYSCTL_RESETCTL_VDD_LOSS_RESET      AON_SYSCTL_RESETCTL_VDD_LOSS_EN
#define AON_SYSCTL_RESETCTL_VDDR_LOSS_EN          (1 << 6)  /* Bit 6:  Controls reset generation in case VDDR is lost */
#  define AON_SYSCTL_RESETCTL_VDDR_LOSS_IGNORE    (0)
#  define AON_SYSCTL_RESETCTL_VDDR_LOSS_RESET     AON_SYSCTL_RESETCTL_VDDR_LOSS_EN
#define AON_SYSCTL_RESETCTL_VDDS_LOSS_EN          (1 << 7)  /* Bit 7:  Controls reset generation in case VDDS is lost */
#  define AON_SYSCTL_RESETCTL_VDDS_LOSS_IGNORE    (0)
#  define AON_SYSCTL_RESETCTL_VDDS_LOSS_RESET     AON_SYSCTL_RESETCTL_VDDS_LOSS_EN
#define AON_SYSCTL_RESETCTL_VDD_LOSS_EN_OVR       (1 << 9)  /* Bit 9:  Override of VDD_LOSS_EN */
#  define AON_SYSCTL_RESETCTL_VDD_OVR_IGNORE      (0)
#  define AON_SYSCTL_RESETCTL_VDD_OVR_RESET       AON_SYSCTL_RESETCTL_VDD_LOSS_EN_OVR
#define AON_SYSCTL_RESETCTL_VDDR_LOSS_EN_OVR      (1 << 10) /* Bit 10: Override of VDDR_LOSS_EN */
#  define AON_SYSCTL_RESETCTL_VDDR_OVR_IGNORE     (0)
#  define AON_SYSCTL_RESETCTL_VDDR_OVR_RESET      AON_SYSCTL_RESETCTL_VDDR_LOSS_EN_OVR
#define AON_SYSCTL_RESETCTL_VDDS_LOSS_EN_OVR      (1 << 11) /* Bit 11: Override of VDDS_LOSS_EN */
#  define AON_SYSCTL_RESETCTL_VDDS_OVR_IGNORE     (0)
#  define AON_SYSCTL_RESETCTL_VDDS_OVR_RESET      AON_SYSCTL_RESETCTL_VDDS_LOSS_EN_OVR
#define AON_SYSCTL_RESETCTL_BOOT_DET_0            (1 << 12) /* Bit 12 */
#define AON_SYSCTL_RESETCTL_BOOT_DET_1            (1 << 13) /* Bit 13 */
#define AON_SYSCTL_RESETCTL_GPIO_WU_FROM_SD       (1 << 14) /* Bit 14: Wakeup from shutdown, IO event */
#define AON_SYSCTL_RESETCTL_WU_FROM_SD            (1 << 15) /* Bit 15: Wakeup from shutdown, IO event or debugger */
#  define AON_SYSCTL_RESETCTL_WU_FROM_COLDRESET   (0)
#  define AON_SYSCTL_RESETCTL_WU_FROM_SHUTDOWN    AON_SYSCTL_RESETCTL_WU_FROM_SD
#define AON_SYSCTL_RESETCTL_BOOT_DET_0_SET        (1 << 16) /* Bit 16 */
#define AON_SYSCTL_RESETCTL_BOOT_DET_1_SET        (1 << 17) /* Bit 17 */
#define AON_SYSCTL_RESETCTL_BOOT_DET_0_CLR        (1 << 24) /* Bit 24 */
#define AON_SYSCTL_RESETCTL_BOOT_DET_1_CLR        (1 << 25) /* Bit 25 */
#define AON_SYSCTL_RESETCTL_SYSRESET              (1 << 31) /* Bit 31: Cold reset */

/* AON_SYSCTL_SLEEPCTL */

#define AON_SYSCTL_SLEEPCTL_IO_PAD_SLEEP_DIS      (1 << 0)  /* Bit 0: Controls the I/O pad sleep mode */
#  define AON_SYSCTL_SLEEPCTL_IO_PAD_SLEEP_ENABLE   (0)
#  define AON_SYSCTL_SLEEPCTL_IO_PAD_SLEEP_DISABLE  AON_SYSCTL_SLEEPCTL_IO_PAD_SLEEP_DIS

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_AON_SYSCTL_H */

/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_hbn.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_HBN_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_HBN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_HBN_CTL_OFFSET           0x000000  /* HBN_CTL */
#define BL602_HBN_TIME_L_OFFSET        0x000004  /* HBN_TIME_L */
#define BL602_HBN_TIME_H_OFFSET        0x000008  /* HBN_TIME_H */
#define BL602_HBN_RTC_TIME_L_OFFSET    0x00000c  /* RTC_TIME_L */
#define BL602_HBN_RTC_TIME_H_OFFSET    0x000010  /* RTC_TIME_H */
#define BL602_HBN_IRQ_MODE_OFFSET      0x000014  /* HBN_IRQ_MODE */
#define BL602_HBN_IRQ_STAT_OFFSET      0x000018  /* HBN_IRQ_STAT */
#define BL602_HBN_IRQ_CLR_OFFSET       0x00001c  /* HBN_IRQ_CLR */
#define BL602_HBN_PIR_CFG_OFFSET       0x000020  /* HBN_PIR_CFG */
#define BL602_HBN_PIR_VTH_OFFSET       0x000024  /* HBN_PIR_VTH */
#define BL602_HBN_PIR_INTERVAL_OFFSET  0x000028  /* HBN_PIR_INTERVAL */
#define BL602_HBN_BOR_CFG_OFFSET       0x00002c  /* HBN_BOR_CFG */
#define BL602_HBN_GLB_OFFSET           0x000030  /* HBN_GLB */
#define BL602_HBN_SRAM_OFFSET          0x000034  /* HBN_SRAM */
#define BL602_HBN_RSV0_OFFSET          0x000100  /* HBN_RSV0 */
#define BL602_HBN_RSV1_OFFSET          0x000104  /* HBN_RSV1 */
#define BL602_HBN_RSV2_OFFSET          0x000108  /* HBN_RSV2 */
#define BL602_HBN_RSV3_OFFSET          0x00010c  /* HBN_RSV3 */
#define BL602_HBN_RC32K_CTRL0_OFFSET   0x000200  /* rc32k_ctrl0 */
#define BL602_HBN_XTAL32K_OFFSET       0x000204  /* xtal32k */

/* Register definitions *****************************************************/

#define BL602_HBN_CTL           (BL602_HBN_BASE + BL602_HBN_CTL_OFFSET)
#define BL602_HBN_TIME_L        (BL602_HBN_BASE + BL602_HBN_TIME_L_OFFSET)
#define BL602_HBN_TIME_H        (BL602_HBN_BASE + BL602_HBN_TIME_H_OFFSET)
#define BL602_HBN_RTC_TIME_L    (BL602_HBN_BASE + BL602_HBN_RTC_TIME_L_OFFSET)
#define BL602_HBN_RTC_TIME_H    (BL602_HBN_BASE + BL602_HBN_RTC_TIME_H_OFFSET)
#define BL602_HBN_IRQ_MODE      (BL602_HBN_BASE + BL602_HBN_IRQ_MODE_OFFSET)
#define BL602_HBN_IRQ_STAT      (BL602_HBN_BASE + BL602_HBN_IRQ_STAT_OFFSET)
#define BL602_HBN_IRQ_CLR       (BL602_HBN_BASE + BL602_HBN_IRQ_CLR_OFFSET)
#define BL602_HBN_PIR_CFG       (BL602_HBN_BASE + BL602_HBN_PIR_CFG_OFFSET)
#define BL602_HBN_PIR_VTH       (BL602_HBN_BASE + BL602_HBN_PIR_VTH_OFFSET)
#define BL602_HBN_PIR_INTERVAL  (BL602_HBN_BASE + BL602_HBN_PIR_INTERVAL_OFFSET)
#define BL602_HBN_BOR_CFG       (BL602_HBN_BASE + BL602_HBN_BOR_CFG_OFFSET)
#define BL602_HBN_GLB           (BL602_HBN_BASE + BL602_HBN_GLB_OFFSET)
#define BL602_HBN_SRAM          (BL602_HBN_BASE + BL602_HBN_SRAM_OFFSET)
#define BL602_HBN_RSV0          (BL602_HBN_BASE + BL602_HBN_RSV0_OFFSET)
#define BL602_HBN_RSV1          (BL602_HBN_BASE + BL602_HBN_RSV1_OFFSET)
#define BL602_HBN_RSV2          (BL602_HBN_BASE + BL602_HBN_RSV2_OFFSET)
#define BL602_HBN_RSV3          (BL602_HBN_BASE + BL602_HBN_RSV3_OFFSET)
#define BL602_HBN_RC32K_CTRL0   (BL602_HBN_BASE + BL602_HBN_RC32K_CTRL0_OFFSET)
#define BL602_HBN_XTAL32K       (BL602_HBN_BASE + BL602_HBN_XTAL32K_OFFSET)

/* Register bit definitions *************************************************/

#define HBN_CTL_HBN_STATE_SHIFT                  (28)
#define HBN_CTL_HBN_STATE_MASK                   (0x0f << HBN_CTL_HBN_STATE_SHIFT)
#define HBN_CTL_SRAM_SLP                         (1 << 27)
#define HBN_CTL_SRAM_SLP_OPTION                  (1 << 26)
#define HBN_CTL_PWR_ON_OPTION                    (1 << 25)
#define HBN_CTL_RTC_DLY_OPTION                   (1 << 24)
#define HBN_CTL_PU_DCDC18_AON                    (1 << 23)
#define HBN_CTL_HBN_LDO11_AON_VOUT_SEL_SHIFT     (19)
#define HBN_CTL_HBN_LDO11_AON_VOUT_SEL_MASK      (0x0f << HBN_CTL_HBN_LDO11_AON_VOUT_SEL_SHIFT)
#define HBN_CTL_HBN_LDO11_RT_VOUT_SEL_SHIFT      (15)
#define HBN_CTL_HBN_LDO11_RT_VOUT_SEL_MASK       (0x0f << HBN_CTL_HBN_LDO11_RT_VOUT_SEL_SHIFT)
#define HBN_CTL_HBN_DIS_PWR_OFF_LDO11_RT         (1 << 14)
#define HBN_CTL_HBN_DIS_PWR_OFF_LDO11            (1 << 13)
#define HBN_CTL_SW_RST                           (1 << 12)
#define HBN_CTL_PWRDN_HBN_RTC                    (1 << 11)
#define HBN_CTL_PWRDN_HBN_CORE                   (1 << 9)
#define HBN_CTL_TRAP_MODE                        (1 << 8)
#define HBN_CTL_HBN_MODE                         (1 << 7)
#define HBN_CTL_RTC_CTL_MASK                     (0x7f)

#define HBN_TIME_H_HBN_TIME_H_MASK               (0xff)

#define HBN_RTC_TIME_H_RTC_TIME_LATCH                (1 << 31)
#define HBN_RTC_TIME_H_RTC_TIME_LATCH_H_MASK         (0xff)

#define HBN_IRQ_MODE_PIN_WAKEUP_EN               (1 << 27)
#define HBN_IRQ_MODE_PIN_WAKEUP_SEL_SHIFT        (24)
#define HBN_IRQ_MODE_PIN_WAKEUP_SEL_MASK         (0x07 << HBN_IRQ_MODE_PIN_WAKEUP_SEL_SHIFT)
#define HBN_IRQ_MODE_IRQ_ACOMP1_EN_SHIFT         (22)
#define HBN_IRQ_MODE_IRQ_ACOMP1_EN_MASK          (0x03 << HBN_IRQ_MODE_IRQ_ACOMP1_EN_SHIFT)
#define HBN_IRQ_MODE_IRQ_ACOMP0_EN_SHIFT         (20)
#define HBN_IRQ_MODE_IRQ_ACOMP0_EN_MASK          (0x03 << HBN_IRQ_MODE_IRQ_ACOMP0_EN_SHIFT)
#define HBN_IRQ_MODE_IRQ_BOR_EN                  (1 << 18)
#define HBN_IRQ_MODE_REG_EN_HW_PU_PD             (1 << 16)
#define HBN_IRQ_MODE_REG_AON_PAD_IE_SMT          (1 << 8)
#define HBN_IRQ_MODE_HBN_PIN_WAKEUP_MASK_SHIFT   (3)
#define HBN_IRQ_MODE_HBN_PIN_WAKEUP_MASK_MASK    (0x03 << HBN_IRQ_MODE_HBN_PIN_WAKEUP_MASK_SHIFT)
#define HBN_IRQ_MODE_HBN_PIN_WAKEUP_MODE_MASK    (0x07)

#define HBN_PIR_CFG_GPADC_NOSYNC                 (1 << 9)
#define HBN_PIR_CFG_GPADC_CGEN                   (1 << 8)
#define HBN_PIR_CFG_PIR_EN                       (1 << 7)
#define HBN_PIR_CFG_PIR_DIS_SHIFT                (4)
#define HBN_PIR_CFG_PIR_DIS_MASK                 (0x03 << HBN_PIR_CFG_PIR_DIS_SHIFT)
#define HBN_PIR_CFG_PIR_LPF_SEL                  (1 << 2)
#define HBN_PIR_CFG_PIR_HPF_SEL_MASK             (0x03)

#define HBN_PIR_VTH_PIR_VTH_MASK                 (0x3fff)

#define HBN_PIR_INTERVAL_PIR_INTERVAL_MASK       (0xfff)

#define HBN_BOR_CFG_R_BOR_OUT                    (1 << 3)
#define HBN_BOR_CFG_PU_BOR                       (1 << 2)
#define HBN_BOR_CFG_BOR_VTH                      (1 << 1)
#define HBN_BOR_CFG_BOR_SEL                      (1 << 0)

#define HBN_GLB_SW_LDO11_AON_VOUT_SEL_SHIFT      (28)
#define HBN_GLB_SW_LDO11_AON_VOUT_SEL_MASK       (0x0f << HBN_GLB_SW_LDO11_AON_VOUT_SEL_SHIFT)
#define HBN_GLB_SW_LDO11_RT_VOUT_SEL_SHIFT       (24)
#define HBN_GLB_SW_LDO11_RT_VOUT_SEL_MASK        (0x0f << HBN_GLB_SW_LDO11_RT_VOUT_SEL_SHIFT)
#define HBN_GLB_SW_LDO11SOC_VOUT_SEL_AON_SHIFT   (16)
#define HBN_GLB_SW_LDO11SOC_VOUT_SEL_AON_MASK    (0x0f << HBN_GLB_SW_LDO11SOC_VOUT_SEL_AON_SHIFT)
#define HBN_GLB_HBN_PU_RC32K                     (1 << 5)
#define HBN_GLB_HBN_F32K_SEL_SHIFT               (3)
#define HBN_GLB_HBN_F32K_SEL_MASK                (0x03 << HBN_GLB_HBN_F32K_SEL_SHIFT)
#define HBN_GLB_HBN_UART_CLK_SEL                 (1 << 2)
#define HBN_GLB_HBN_ROOT_CLK_SEL_MASK            (0x03)

#define HBN_SRAM_RETRAM_SLP                      (1 << 7)
#define HBN_SRAM_RETRAM_RET                      (1 << 6)

#define HBN_RC32K_CTRL0_RC32K_CODE_FR_EXT_SHIFT      (22)
#define HBN_RC32K_CTRL0_RC32K_CODE_FR_EXT_MASK       (0x3ff << HBN_RC32K_CTRL0_RC32K_CODE_FR_EXT_SHIFT)
#define HBN_RC32K_CTRL0_RC32K_CAL_EN                 (1 << 20)
#define HBN_RC32K_CTRL0_RC32K_EXT_CODE_EN            (1 << 19)
#define HBN_RC32K_CTRL0_RC32K_ALLOW_CAL              (1 << 18)
#define HBN_RC32K_CTRL0_RC32K_VREF_DLY_SHIFT         (16)
#define HBN_RC32K_CTRL0_RC32K_VREF_DLY_MASK          (0x03 << HBN_RC32K_CTRL0_RC32K_VREF_DLY_SHIFT)
#define HBN_RC32K_CTRL0_RC32K_DIG_CODE_FR_CAL_SHIFT  (6)
#define HBN_RC32K_CTRL0_RC32K_DIG_CODE_FR_CAL_MASK   (0x3ff << HBN_RC32K_CTRL0_RC32K_DIG_CODE_FR_CAL_SHIFT)
#define HBN_RC32K_CTRL0_RC32K_CAL_PRECHARGE          (1 << 5)
#define HBN_RC32K_CTRL0_RC32K_CAL_DIV_SHIFT          (3)
#define HBN_RC32K_CTRL0_RC32K_CAL_DIV_MASK           (0x03 << HBN_RC32K_CTRL0_RC32K_CAL_DIV_SHIFT)
#define HBN_RC32K_CTRL0_RC32K_CAL_INPROGRESS         (1 << 2)
#define HBN_RC32K_CTRL0_RC32K_RDY                    (1 << 1)
#define HBN_RC32K_CTRL0_RC32K_CAL_DONE               (1 << 0)

#define HBN_XTAL32K_PU_XTAL32K                       (1 << 19)
#define HBN_XTAL32K_PU_XTAL32K_BUF                   (1 << 18)
#define HBN_XTAL32K_XTAL32K_AC_CAP_SHORT             (1 << 17)
#define HBN_XTAL32K_XTAL32K_CAPBANK_SHIFT            (11)
#define HBN_XTAL32K_XTAL32K_CAPBANK_MASK             (0x3f << HBN_XTAL32K_XTAL32K_CAPBANK_SHIFT)
#define HBN_XTAL32K_XTAL32K_INV_STRE_SHIFT           (9)
#define HBN_XTAL32K_XTAL32K_INV_STRE_MASK            (0x03 << HBN_XTAL32K_XTAL32K_INV_STRE_SHIFT)
#define HBN_XTAL32K_XTAL32K_OTF_SHORT                (1 << 8)
#define HBN_XTAL32K_XTAL32K_OUTBUF_STRE              (1 << 7)
#define HBN_XTAL32K_XTAL32K_REG_SHIFT                (5)
#define HBN_XTAL32K_XTAL32K_REG_MASK                 (0x03 << HBN_XTAL32K_XTAL32K_REG_SHIFT)
#define HBN_XTAL32K_XTAL32K_AMP_CTRL_SHIFT           (3)
#define HBN_XTAL32K_XTAL32K_AMP_CTRL_MASK            (0x03 << HBN_XTAL32K_XTAL32K_AMP_CTRL_SHIFT)
#define HBN_XTAL32K_XTAL32K_EXT_SEL                  (1 << 2)

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_HBN_H */

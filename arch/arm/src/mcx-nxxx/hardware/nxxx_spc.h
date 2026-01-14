/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/nxxx_spc.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_SPC_H
#define ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_SPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/nxxx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NXXX_SPC_VERID               (NXXX_SPC0_BASE + 0x0000)  /* Version ID */
#define NXXX_SPC_SC                  (NXXX_SPC0_BASE + 0x0010)  /* Status Control */
#define NXXX_SPC_CNTRL               (NXXX_SPC0_BASE + 0x0014)  /* SPC Regulator Control */
#define NXXX_SPC_LPREQ_CFG           (NXXX_SPC0_BASE + 0x001C)  /* Low-Power Request Configuration */
#define NXXX_SPC_PD_STATUS0          (NXXX_SPC0_BASE + 0x0030)  /* SPC Power Domain Mode Status */
#define NXXX_SPC_PD_STATUS1          (NXXX_SPC0_BASE + 0x0034)  /* SPC Power Domain Mode Status */
#define NXXX_SPC_SRAMCTL             (NXXX_SPC0_BASE + 0x0040)  /* SRAM Control */
#define NXXX_SPC_ACTIVE_CFG          (NXXX_SPC0_BASE + 0x0100)  /* Active Power Mode Configuration */
#define NXXX_SPC_ACTIVE_CFG1         (NXXX_SPC0_BASE + 0x0104)  /* Active Power Mode Configuration 1 */
#define NXXX_SPC_LP_CFG              (NXXX_SPC0_BASE + 0x0108)  /* Low-Power Mode Configuration */
#define NXXX_SPC_LP_CFG1             (NXXX_SPC0_BASE + 0x010C)  /* Low Power Mode Configuration 1 */
#define NXXX_SPC_LPWKUP_DELAY        (NXXX_SPC0_BASE + 0x0120)  /* Low Power Wake-Up Delay */
#define NXXX_SPC_ACTIVE_VDELAY       (NXXX_SPC0_BASE + 0x0124)  /* Active Voltage Trim Delay */
#define NXXX_SPC_VD_STAT             (NXXX_SPC0_BASE + 0x0130)  /* Voltage Detect Status */
#define NXXX_SPC_VD_CORE_CFG         (NXXX_SPC0_BASE + 0x0134)  /* Core Voltage Detect Configuration */
#define NXXX_SPC_VD_SYS_CFG          (NXXX_SPC0_BASE + 0x0138)  /* System Voltage Detect Configuration */
#define NXXX_SPC_VD_IO_CFG           (NXXX_SPC0_BASE + 0x013C)  /* IO Voltage Detect Configuration */
#define NXXX_SPC_EVD_CFG             (NXXX_SPC0_BASE + 0x0140)  /* External Voltage Domain Configuration */
#define NXXX_SPC_GLITCH_DETECT_SC    (NXXX_SPC0_BASE + 0x0144)  /* Glitch Detect Status Control */
#define NXXX_SPC_CORELDO_CFG         (NXXX_SPC0_BASE + 0x0300)  /* LDO_CORE Configuration */
#define NXXX_SPC_SYSLDO_CFG          (NXXX_SPC0_BASE + 0x0400)  /* LDO_SYS Configuration */
#define NXXX_SPC_DCDC_CFG            (NXXX_SPC0_BASE + 0x0500)  /* DCDC Configuration */
#define NXXX_SPC_DCDC_BURST_CFG      (NXXX_SPC0_BASE + 0x0504)  /* DCDC Burst Configuration */

/* Status Control (SC) */

#define SPC_SC_BUSY_SHIFT                        (0)
#define SPC_SC_BUSY_MASK                         (0x01 << SPC_SC_BUSY_SHIFT)
#define SPC_SC_BUSY(x)                           (((x) << SPC_SC_BUSY_SHIFT) & SPC_SC_BUSY_MASK)

#define SPC_SC_SPC_LP_REQ_SHIFT                  (1)
#define SPC_SC_SPC_LP_REQ_MASK                   (0x01 < SPC_SC_SPC_LP_REQ_SHIFT)
#define SPC_SC_SPC_LP_REQ(x)                     ((((x) << SPC_SC_SPC_LP_REQ_SHIFT) & SPC_SC_SPC_LP_REQ_MASK)

#define SPC_SC_SPC_LP_MODE_SHIFT                 (4)
#define SPC_SC_SPC_LP_MODE_MASK                  (0x0f << SPC_SC_SPC_LP_MODE_SHIFT)
#define SPC_SC_SPC_LP_MODE(x)                    (((x) << SPC_SC_SPC_LP_MODE_SHIFT) & SPC_SC_SPC_LP_MODE_MASK)

#define SPC_SC_ISO_CLR_SHIFT                     (16)
#define SPC_SC_ISO_CLR_MASK                      (0x03 << SPC_SC_ISO_CLR_SHIFT)
#define SPC_SC_ISO_CLR(x)                        (((x) << SPC_SC_ISO_CLR_SHIFT) & SPC_SC_ISO_CLR_MASK)

/* Regulator Control (CNTRL) */

#define SPC_CNTRL_CORELDO_EN_SHIFT               (0)
#define SPC_CNTRL_CORELDO_EN_MASK                (0x01 << SPC_CNTRL_CORELDO_EN_SHIFT)
#define SPC_CNTRL_CORELDO_EN(x)                  (((x) << SPC_CNTRL_CORELDO_EN_SHIFT) & SPC_CNTRL_CORELDO_EN_MASK)

#define SPC_CNTRL_SYSLDO_EN_SHIFT                (1)
#define SPC_CNTRL_SYSLDO_EN_MASK                 (0x01 << SPC_CNTRL_SYSLDO_EN_SHIFT)
#define SPC_CNTRL_SYSLDO_EN(x)                   (((x) << SPC_CNTRL_SYSLDO_EN_SHIFT) & SPC_CNTRL_SYSLDO_EN_MASK)

#define SPC_CNTRL_DCDC_EN_SHIFT                  (2)
#define SPC_CNTRL_DCDC_EN_MASK                   (0x01 << SPC_CNTRL_DCDC_EN_SHIFT)
#define SPC_CNTRL_DCDC_EN(x)                     (((x) << SPC_CNTRL_DCDC_EN_SHIFT) & SPC_CNTRL_DCDC_EN_MASK)

/* Low-Power Request Configuration (LPREQ_CFG) */

#define SPC_LPREQ_CFG_LPREQOE_SHIFT              (0)
#define SPC_LPREQ_CFG_LPREQOE_MASK               (0x01 << SPC_LPREQ_CFG_LPREQOE_SHIFT)
#define SPC_LPREQ_CFG_LPREQOE(x)                 (((x) << SPC_LPREQ_CFG_LPREQOE_SHIFT) & SPC_LPREQ_CFG_LPREQOE_MASK)

#define SPC_LPREQ_CFG_LPREQPOL_SHIFT             (1)
#define SPC_LPREQ_CFG_LPREQPOL_MASK              (0x01)
#define SPC_LPREQ_CFG_LPREQPOL(x)                (((x) << SPC_LPREQ_CFG_LPREQPOL_SHIFT) & SPC_LPREQ_CFG_LPREQPOL_MASK)

#define SPC_LPREQ_CFG_LPREQOV_SHIFT              (2)
#define SPC_LPREQ_CFG_LPREQOV_MASK               (0x03 << SPC_LPREQ_CFG_LPREQOV_SHIFT)
#define SPC_LPREQ_CFG_LPREQOV(x)                 (((x) << SPC_LPREQ_CFG_LPREQOV_SHIFT) & SPC_LPREQ_CFG_LPREQOV_MASK)

/* Power Domain Mode Status (PD_STATUS) */

#define SPC_PD_STATUS_PWR_REQ_STATUS_SHIFT       (0)
#define SPC_PD_STATUS_PWR_REQ_STATUS_MASK        (0x01 << SPC_PD_STATUS_PWR_REQ_STATUS_SHIFT)
#define SPC_PD_STATUS_PWR_REQ_STATUS(x)          (((x) << SPC_PD_STATUS_PWR_REQ_STATUS_SHIFT) & SPC_PD_STATUS_PWR_REQ_STATUS_MASK)

#define SPC_PD_STATUS_PD_LP_REQ_SHIFT            (4)
#define SPC_PD_STATUS_PD_LP_REQ_MASK             (0x01 << SPC_PD_STATUS_PD_LP_REQ_SHIFT)
#define SPC_PD_STATUS_PD_LP_REQ(x)               (((x) << SPC_PD_STATUS_PD_LP_REQ_SHIFT) & SPC_PD_STATUS_PD_LP_REQ_MASK)

#define SPC_PD_STATUS_LP_MODE_SHIFT              (8)
#define SPC_PD_STATUS_LP_MODE_MASK               (0x0f << SPC_PD_STATUS_LP_MODE_SHIFT)
#define SPC_PD_STATUS_LP_MODE(x)                 (((x) << SPC_PD_STATUS_LP_MODE_SHIFT) & SPC_PD_STATUS_LP_MODE_MASK)

/* SRAM Control (SRAMCTL) */

#define SPC_SRAMCTL_VSM_SHIFT                    (0)
#define SPC_SRAMCTL_VSM_MASK                     (0x03 << SPC_SRAMCTL_VSM_SHIFT)
#define SPC_SRAMCTL_VSM(x)                       (((x) << SPC_SRAMCTL_VSM_SHIFT) & SPC_SRAMCTL_VSM_MASK)

#define SPC_SRAM1V0                              (0x01) /* SRAM configured for 1.0V operation. */
#define SPC_SRAM1V1                              (0x02) /* SRAM configured for 1.1V operation. */
#define SPC_SRAM1V2                              (0x03) /* SRAM configured for 1.2V operation. */

#define SPC_SRAMCTL_REQ_SHIFT                    (30)
#define SPC_SRAMCTL_REQ_MASK                     (0x01 << SPC_SRAMCTL_REQ_SHIFT)
#define SPC_SRAMCTL_REQ(x)                       (((x) << SPC_SRAMCTL_REQ_SHIFT) & SPC_SRAMCTL_REQ_MASK)

#define SPC_SRAMCTL_ACK_SHIFT                    (31)
#define SPC_SRAMCTL_ACK_MASK                     (0x01 << SPC_SRAMCTL_ACK_SHIFT)
#define SPC_SRAMCTL_ACK(x)                       (((x) << SPC_SRAMCTL_ACK_SHIFT) & SPC_SRAMCTL_ACK_MASK)

/* Active Power Mode Configuration (ACTIVE_CFG) */

#define SPC_ACTIVE_CFG_CORELDO_VDD_DS_SHIFT      (0)
#define SPC_ACTIVE_CFG_CORELDO_VDD_DS_MASK       (0x01 << SPC_ACTIVE_CFG_CORELDO_VDD_DS_SHIFT)
#define SPC_ACTIVE_CFG_CORELDO_VDD_DS(x)         (((x) << SPC_ACTIVE_CFG_CORELDO_VDD_DS_SHIFT) & SPC_ACTIVE_CFG_CORELDO_VDD_DS_MASK)

#define SPC_ACTIVE_CFG_CORELDO_VDD_LVL_SHIFT     (2)
#define SPC_ACTIVE_CFG_CORELDO_VDD_LVL_MASK      (0x03 << SPC_ACTIVE_CFG_CORELDO_VDD_LVL_SHIFT)
#define SPC_ACTIVE_CFG_CORELDO_VDD_LVL(x)        (((x) << SPC_ACTIVE_CFG_CORELDO_VDD_LVL_SHIFT) & SPC_ACTIVE_CFG_CORELDO_VDD_LVL_MASK)

#define SPC_ACTIVE_CFG_SYSLDO_VDD_DS_SHIFT       (4)
#define SPC_ACTIVE_CFG_SYSLDO_VDD_DS_MASK        (0x01 << SPC_ACTIVE_CFG_SYSLDO_VDD_DS_SHIFT)
#define SPC_ACTIVE_CFG_SYSLDO_VDD_DS(x)          (((x) << SPC_ACTIVE_CFG_SYSLDO_VDD_DS_SHIFT) & SPC_ACTIVE_CFG_SYSLDO_VDD_DS_MASK)

#define SPC_ACTIVE_CFG_SYSLDO_VDD_LVL_SHIFT      (6)
#define SPC_ACTIVE_CFG_SYSLDO_VDD_LVL_MASK       (0x01 << SPC_ACTIVE_CFG_SYSLDO_VDD_LVL_SHIFT)
#define SPC_ACTIVE_CFG_SYSLDO_VDD_LVL(x)         (((x) << SPC_ACTIVE_CFG_SYSLDO_VDD_LVL_SHIFT) & SPC_ACTIVE_CFG_SYSLDO_VDD_LVL_MASK)

#define SPC_ACTIVE_CFG_DCDC_VDD_DS_SHIFT         (8)
#define SPC_ACTIVE_CFG_DCDC_VDD_DS_MASK          (0x03 << SPC_ACTIVE_CFG_DCDC_VDD_DS_SHIFT)
#define SPC_ACTIVE_CFG_DCDC_VDD_DS(x)            (((x) << SPC_ACTIVE_CFG_DCDC_VDD_DS_SHIFT) & SPC_ACTIVE_CFG_DCDC_VDD_DS_MASK)

#define SPC_DCDC_PULSEREFRESHMODE                (0) /* DCDC_CORE Regulator Drive Strength set to Pulse Refresh Mode */
#define SPC_DCDC_LOWDRIVESTRENGTH                (1) /* DCDC_CORE regulator Drive Strength set to low. */
#define SPC_DCDC_NORMALDRIVESTRENGTH             (2) /* DCDC_CORE regulator Drive Strength set to Normal. */

#define SPC_ACTIVE_CFG_DCDC_VDD_LVL_SHIFT        (10)
#define SPC_ACTIVE_CFG_DCDC_VDD_LVL_MASK         (0x03 << SPC_ACTIVE_CFG_DCDC_VDD_LVL_SHIFT)
#define SPC_ACTIVE_CFG_DCDC_VDD_LVL(x)           (((x) << SPC_ACTIVE_CFG_DCDC_VDD_LVL_SHIFT) & SPC_ACTIVE_CFG_DCDC_VDD_LVL_MASK)

#define SPC_DCDC_RETENTIONVOLTAGE                (0) /* DCDC_CORE Regulator regulate to retention Voltage */
#define SPC_DCDC_MIDVOLTAGE                      (1) /* DCDC_CORE Regulator regulate to Mid Voltage(1.0V). */
#define SPC_DCDC_NORMALVOLTAGE                   (2) /* DCDC_CORE Regulator regulate to Normal Voltage(1.1V). */
#define SPC_DCDC_OVERDRIVEVOLTAGE                (3) /* DCDC_CORE Regulator regulate to Safe-Mode Voltage(1.2V). */

#define SPC_ACTIVE_CFG_GLITCH_DETECT_DISABLE_SHIFT (12)
#define SPC_ACTIVE_CFG_GLITCH_DETECT_DISABLE_MASK  (0x01 << SPC_ACTIVE_CFG_GLITCH_DETECT_DISABLE_SHIFT)
#define SPC_ACTIVE_CFG_GLITCH_DETECT_DISABLE(x)    (((x) << SPC_ACTIVE_CFG_GLITCH_DETECT_DISABLE_SHIFT) & SPC_ACTIVE_CFG_GLITCH_DETECT_DISABLE_MASK)

#define SPC_ACTIVE_CFG_LPBUFF_EN_SHIFT           (18)
#define SPC_ACTIVE_CFG_LPBUFF_EN_MASK            (0x01 << SPC_ACTIVE_CFG_LPBUFF_EN_SHIFT)
#define SPC_ACTIVE_CFG_LPBUFF_EN(x)              (((x) << SPC_ACTIVE_CFG_LPBUFF_EN_SHIFT) & SPC_ACTIVE_CFG_LPBUFF_EN_MASK)

#define SPC_ACTIVE_CFG_BGMODE_SHIFT              (20)
#define SPC_ACTIVE_CFG_BGMODE_MASK               (0x03 << SPC_ACTIVE_CFG_BGMODE_SHIFT)
#define SPC_ACTIVE_CFG_BGMODE(x)                 (((x) << SPC_ACTIVE_CFG_BGMODE_SHIFT) & SPC_ACTIVE_CFG_BGMODE_MASK)

#define SPC_ACTIVE_CFG_VDD_VD_DISABLE_SHIFT      (23)
#define SPC_ACTIVE_CFG_VDD_VD_DISABLE_MASK       (0x01 << SPC_ACTIVE_CFG_VDD_VD_DISABLE_SHIFT)
#define SPC_ACTIVE_CFG_VDD_VD_DISABLE(x)         (((x) << SPC_ACTIVE_CFG_VDD_VD_DISABLE_SHIFT) & SPC_ACTIVE_CFG_VDD_VD_DISABLE_MASK)

#define SPC_ACTIVE_CFG_CORE_LVDE_SHIFT           (24)
#define SPC_ACTIVE_CFG_CORE_LVDE_MASK            (0x01 << SPC_ACTIVE_CFG_CORE_LVDE_SHIFT)
#define SPC_ACTIVE_CFG_CORE_LVDE(x)              (((x) << SPC_ACTIVE_CFG_CORE_LVDE_SHIFT) & SPC_ACTIVE_CFG_CORE_LVDE_MASK)

#define SPC_ACTIVE_CFG_SYS_LVDE_SHIFT            (25)
#define SPC_ACTIVE_CFG_SYS_LVDE_MASK             (0x01 << SPC_ACTIVE_CFG_SYS_LVDE_SHIFT)
#define SPC_ACTIVE_CFG_SYS_LVDE(x)               (((x) << SPC_ACTIVE_CFG_SYS_LVDE_SHIFT) & SPC_ACTIVE_CFG_SYS_LVDE_MASK)

#define SPC_ACTIVE_CFG_IO_LVDE_SHIFT             (26)
#define SPC_ACTIVE_CFG_IO_LVDE_MASK              (0x01)
#define SPC_ACTIVE_CFG_IO_LVDE(x)                (((x) << SPC_ACTIVE_CFG_IO_LVDE_SHIFT) & SPC_ACTIVE_CFG_IO_LVDE_MASK)

#define SPC_ACTIVE_CFG_CORE_HVDE_SHIFT           (27)
#define SPC_ACTIVE_CFG_CORE_HVDE_MASK            (0x01 << SPC_ACTIVE_CFG_CORE_HVDE_SHIFT)
#define SPC_ACTIVE_CFG_CORE_HVDE(x)              (((x) << SPC_ACTIVE_CFG_CORE_HVDE_SHIFT) & SPC_ACTIVE_CFG_CORE_HVDE_MASK)

#define SPC_ACTIVE_CFG_SYS_HVDE_SHIFT            (28)
#define SPC_ACTIVE_CFG_SYS_HVDE_MASK             (0x01 << SPC_ACTIVE_CFG_SYS_HVDE_SHIFT)
#define SPC_ACTIVE_CFG_SYS_HVDE(x)               (((x) << SPC_ACTIVE_CFG_SYS_HVDE_SHIFT) & SPC_ACTIVE_CFG_SYS_HVDE_MASK)

#define SPC_ACTIVE_CFG_IO_HVDE_SHIFT             (29)
#define SPC_ACTIVE_CFG_IO_HVDE_MASK              (0x01)
#define SPC_ACTIVE_CFG_IO_HVDE(x)                (((x) << SPC_ACTIVE_CFG_IO_HVDE_SHIFT) & SPC_ACTIVE_CFG_IO_HVDE_MASK)

/* Active Power Mode Configuration 1 (ACTIVE_CFG1) */

#define SPC_ACTIVE_CFG1_SOC_CNTRL_SHIFT          (0)
#define SPC_ACTIVE_CFG1_SOC_CNTRL_MASK           (0xffffffff << SPC_ACTIVE_CFG1_SOC_CNTRL_SHIFT)
#define SPC_ACTIVE_CFG1_SOC_CNTRL(x)             (((x) << SPC_ACTIVE_CFG1_SOC_CNTRL_SHIFT) & SPC_ACTIVE_CFG1_SOC_CNTRL_MASK)

/* Low-Power Mode Configuration (LP_CFG) */

#define SPC_LP_CFG_CORELDO_VDD_DS_SHIFT          (0)
#define SPC_LP_CFG_CORELDO_VDD_DS_MASK           (0x01 << SPC_LP_CFG_CORELDO_VDD_DS_SHIFT)
#define SPC_LP_CFG_CORELDO_VDD_DS(x)             (((x) << SPC_LP_CFG_CORELDO_VDD_DS_SHIFT) & SPC_LP_CFG_CORELDO_VDD_DS_MASK)

#define SPC_CORELDO_LOWDRIVESTRENGTH             (0) /* Core LDO VDD regulator Drive Strength set to low. */
#define SPC_CORELDO_NORMALDRIVESTRENGTH          (1) /* Core LDO VDD regulator Drive Strength set to Normal. */

#define SPC_LP_CFG_CORELDO_VDD_LVL_SHIFT         (2)
#define SPC_LP_CFG_CORELDO_VDD_LVL_MASK          (0x3 << SPC_LP_CFG_CORELDO_VDD_LVL_SHIFT)
#define SPC_LP_CFG_CORELDO_VDD_LVL(x)            (((x) << SPC_LP_CFG_CORELDO_VDD_LVL_SHIFT) & SPC_LP_CFG_CORELDO_VDD_LVL_MASK)

#define SPC_CORELDO_RETENTIONVOLTAGE             (0) /* Core LDO VDD regulator regulate to retention voltage. */
#define SPC_CORELDO_MIDDRIVEVOLTAGE              (1) /* Core LDO VDD regulator regulate to Mid Drive Voltage. */
#define SPC_CORELDO_NORMALVOLTAGE                (2) /* Core LDO VDD regulator regulate to Normal Voltage. */
#define SPC_CORELDO_OVERDRIVEVOLTAGE             (3) /* Core LDO VDD regulator regulate to overdrive Voltage. */

#define SPC_LP_CFG_SYSLDO_VDD_DS_SHIFT           (4)
#define SPC_LP_CFG_SYSLDO_VDD_DS_MASK            (0x01 << SPC_LP_CFG_SYSLDO_VDD_DS_SHIFT)
#define SPC_LP_CFG_SYSLDO_VDD_DS(x)              (((x) << SPC_LP_CFG_SYSLDO_VDD_DS_SHIFT) & SPC_LP_CFG_SYSLDO_VDD_DS_MASK)

#define SPC_LP_CFG_DCDC_VDD_DS_SHIFT             (8)
#define SPC_LP_CFG_DCDC_VDD_DS_MASK              (0x03 << SPC_LP_CFG_DCDC_VDD_DS_SHIFT)
#define SPC_LP_CFG_DCDC_VDD_DS(x)                (((x) << SPC_LP_CFG_DCDC_VDD_DS_SHIFT) & SPC_LP_CFG_DCDC_VDD_DS_MASK)

#define SPC_LP_CFG_DCDC_VDD_LVL_SHIFT            (10)
#define SPC_LP_CFG_DCDC_VDD_LVL_MASK             (0x03 << SPC_LP_CFG_DCDC_VDD_LVL_SHIFT)
#define SPC_LP_CFG_DCDC_VDD_LVL(x)               (((x) << SPC_LP_CFG_DCDC_VDD_LVL_SHIFT) & SPC_LP_CFG_DCDC_VDD_LVL_MASK)

#define SPC_LP_CFG_GLITCH_DETECT_DISABLE_SHIFT   (12)
#define SPC_LP_CFG_GLITCH_DETECT_DISABLE_MASK    (0x01 << SPC_LP_CFG_GLITCH_DETECT_DISABLE_SHIFT)
#define SPC_LP_CFG_GLITCH_DETECT_DISABLE(x)      (((x) << SPC_LP_CFG_GLITCH_DETECT_DISABLE_SHIFT) & SPC_LP_CFG_GLITCH_DETECT_DISABLE_MASK)

#define SPC_LP_CFG_COREVDD_IVS_EN_SHIFT          (17)
#define SPC_LP_CFG_COREVDD_IVS_EN_MASK           (0x01 << SPC_LP_CFG_COREVDD_IVS_EN_SHIFT)
#define SPC_LP_CFG_COREVDD_IVS_EN(x)             (((x) << SPC_LP_CFG_COREVDD_IVS_EN_SHIFT) & SPC_LP_CFG_COREVDD_IVS_EN_MASK)

#define SPC_LP_CFG_LPBUFF_EN_SHIFT               (18)
#define SPC_LP_CFG_LPBUFF_EN_MASK                (0x01 << SPC_LP_CFG_LPBUFF_EN_SHIFT)
#define SPC_LP_CFG_LPBUFF_EN(x)                  (((x) << SPC_LP_CFG_LPBUFF_EN_SHIFT) & SPC_LP_CFG_LPBUFF_EN_MASK)

#define SPC_LP_CFG_BGMODE_SHIFT                  (20)
#define SPC_LP_CFG_BGMODE_MASK                   (0x03 << SPC_LP_CFG_BGMODE_SHIFT)
#define SPC_LP_CFG_BGMODE(x)                     (((x) << SPC_LP_CFG_BGMODE_SHIFT) & SPC_LP_CFG_BGMODE_MASK)

#define SPC_LP_CFG_LP_IREFEN_SHIFT               (23)
#define SPC_LP_CFG_LP_IREFEN_MASK                (0x01 << SPC_LP_CFG_LP_IREFEN_SHIFT)
#define SPC_LP_CFG_LP_IREFEN(x)                  (((x) << SPC_LP_CFG_LP_IREFEN_SHIFT) & SPC_LP_CFG_LP_IREFEN_MASK)

#define SPC_LP_CFG_CORE_LVDE_SHIFT               (24)
#define SPC_LP_CFG_CORE_LVDE_MASK                (0x01 << SPC_LP_CFG_CORE_LVDE_SHIFT)
#define SPC_LP_CFG_CORE_LVDE(x)                  (((x) << SPC_LP_CFG_CORE_LVDE_SHIFT) & SPC_LP_CFG_CORE_LVDE_MASK)

#define SPC_LP_CFG_SYS_LVDE_SHIFT                (25)
#define SPC_LP_CFG_SYS_LVDE_MASK                 (0x01 << SPC_LP_CFG_SYS_LVDE_SHIFT)
#define SPC_LP_CFG_SYS_LVDE(x)                   (((x) << SPC_LP_CFG_SYS_LVDE_SHIFT) & SPC_LP_CFG_SYS_LVDE_MASK)

#define SPC_LP_CFG_IO_LVDE_SHIFT                 (26)
#define SPC_LP_CFG_IO_LVDE_MASK                  (0x01 << SPC_LP_CFG_IO_LVDE_SHIFT)
#define SPC_LP_CFG_IO_LVDE(x)                    (((x) << SPC_LP_CFG_IO_LVDE_SHIFT) & SPC_LP_CFG_IO_LVDE_MASK)

#define SPC_LP_CFG_CORE_HVDE_SHIFT               (27)
#define SPC_LP_CFG_CORE_HVDE_MASK                (0x01 << SPC_LP_CFG_CORE_HVDE_SHIFT)
#define SPC_LP_CFG_CORE_HVDE(x)                  (((x) << SPC_LP_CFG_CORE_HVDE_SHIFT) & SPC_LP_CFG_CORE_HVDE_MASK)

#define SPC_LP_CFG_SYS_HVDE_SHIFT                (28)
#define SPC_LP_CFG_SYS_HVDE_MASK                 (0x01 << SPC_LP_CFG_SYS_HVDE_SHIFT)
#define SPC_LP_CFG_SYS_HVDE(x)                   (((x) << SPC_LP_CFG_SYS_HVDE_SHIFT) & SPC_LP_CFG_SYS_HVDE_MASK)

#define SPC_LP_CFG_IO_HVDE_SHIFT                 (29)
#define SPC_LP_CFG_IO_HVDE_MASK                  (0x01 << SPC_LP_CFG_IO_HVDE_SHIFT)
#define SPC_LP_CFG_IO_HVDE(x)                    (((x) << SPC_LP_CFG_IO_HVDE_SHIFT) & SPC_LP_CFG_IO_HVDE_MASK)

/* Low Power Mode Configuration 1 (LP_CFG1) */

#define SPC_LP_CFG1_SOC_CNTRL_SHIFT              (0)
#define SPC_LP_CFG1_SOC_CNTRL_MASK               (0xffffffff << SPC_LP_CFG1_SOC_CNTRL_SHIFT)
#define SPC_LP_CFG1_SOC_CNTRL(x)                 (((x) << SPC_LP_CFG1_SOC_CNTRL_SHIFT) & SPC_LP_CFG1_SOC_CNTRL_MASK)

/* Low Power Wake-Up Delay (LPWKUP_DELAY) */

#define SPC_LPWKUP_DELAY_LPWKUP_DELAY_SHIFT      (0)
#define SPC_LPWKUP_DELAY_LPWKUP_DELAY_MASK       (0xffff << SPC_LPWKUP_DELAY_LPWKUP_DELAY_SHIFT)
#define SPC_LPWKUP_DELAY_LPWKUP_DELAY(x)         (((x) << SPC_LPWKUP_DELAY_LPWKUP_DELAY_SHIFT) & SPC_LPWKUP_DELAY_LPWKUP_DELAY_MASK)

/* Active Voltage Trim Delay (ACTIVE_VDELAY) */

#define SPC_ACTIVE_VDELAY_ACTIVE_VDELAY_SHIFT    (0)
#define SPC_ACTIVE_VDELAY_ACTIVE_VDELAY_MASK     (0xffff << SPC_ACTIVE_VDELAY_ACTIVE_VDELAY_SHIFT)
#define SPC_ACTIVE_VDELAY_ACTIVE_VDELAY(x)       (((x) << SPC_ACTIVE_VDELAY_ACTIVE_VDELAY_SHIFT) & SPC_ACTIVE_VDELAY_ACTIVE_VDELAY_MASK)

/* Voltage Detect Status (VD_STAT) */

#define SPC_VD_STAT_COREVDD_LVDF_SHIFT           (0)
#define SPC_VD_STAT_COREVDD_LVDF_MASK            (0x01 << SPC_VD_STAT_COREVDD_LVDF_SHIFT)
#define SPC_VD_STAT_COREVDD_LVDF(x)              (((x) << SPC_VD_STAT_COREVDD_LVDF_SHIFT) & SPC_VD_STAT_COREVDD_LVDF_MASK)

#define SPC_VD_STAT_SYSVDD_LVDF_SHIFT            (1)
#define SPC_VD_STAT_SYSVDD_LVDF_MASK             (0x01 << SPC_VD_STAT_SYSVDD_LVDF_SHIFT)
#define SPC_VD_STAT_SYSVDD_LVDF(x)               (((x) << SPC_VD_STAT_SYSVDD_LVDF_SHIFT) & SPC_VD_STAT_SYSVDD_LVDF_MASK)

#define SPC_VD_STAT_IOVDD_LVDF_SHIFT             (2)
#define SPC_VD_STAT_IOVDD_LVDF_MASK              (0x01 << SPC_VD_STAT_IOVDD_LVDF_SHIFT)
#define SPC_VD_STAT_IOVDD_LVDF(x)                (((x) << SPC_VD_STAT_IOVDD_LVDF_SHIFT) & SPC_VD_STAT_IOVDD_LVDF_MASK)

#define SPC_VD_STAT_COREVDD_HVDF_SHIFT           (4)
#define SPC_VD_STAT_COREVDD_HVDF_MASK            (0x01 << SPC_VD_STAT_COREVDD_HVDF_SHIFT)
#define SPC_VD_STAT_COREVDD_HVDF(x)              (((x) << SPC_VD_STAT_COREVDD_HVDF_SHIFT) & SPC_VD_STAT_COREVDD_HVDF_MASK)

#define SPC_VD_STAT_SYSVDD_HVDF_SHIFT            (5)
#define SPC_VD_STAT_SYSVDD_HVDF_MASK             (0x01 << SPC_VD_STAT_SYSVDD_HVDF_SHIFT)
#define SPC_VD_STAT_SYSVDD_HVDF(x)               (((x) << SPC_VD_STAT_SYSVDD_HVDF_SHIFT) & SPC_VD_STAT_SYSVDD_HVDF_MASK)

#define SPC_VD_STAT_IOVDD_HVDF_SHIFT             (6)
#define SPC_VD_STAT_IOVDD_HVDF_MASK              (0x01 << SPC_VD_STAT_IOVDD_HVDF_SHIFT)
#define SPC_VD_STAT_IOVDD_HVDF(x)                (((x) << SPC_VD_STAT_IOVDD_HVDF_SHIFT) & SPC_VD_STAT_IOVDD_HVDF_MASK)

/* Core Voltage Detect Configuration (VD_CORE_CFG) */

#define SPC_VD_CORE_CFG_LVDRE_SHIFT              (0)
#define SPC_VD_CORE_CFG_LVDRE_MASK               (0x01 << SPC_VD_CORE_CFG_LVDRE_SHIFT)
#define SPC_VD_CORE_CFG_LVDRE(x)                 (((x) << SPC_VD_CORE_CFG_LVDRE_SHIFT) & SPC_VD_CORE_CFG_LVDRE_MASK)

#define SPC_VD_CORE_CFG_LVDIE_SHIFT              (1)
#define SPC_VD_CORE_CFG_LVDIE_MASK               (0x01 << SPC_VD_CORE_CFG_LVDIE_SHIFT)
#define SPC_VD_CORE_CFG_LVDIE(x)                 (((x) << SPC_VD_CORE_CFG_LVDIE_SHIFT) & SPC_VD_CORE_CFG_LVDIE_MASK)

#define SPC_VD_CORE_CFG_HVDRE_SHIFT              (2)
#define SPC_VD_CORE_CFG_HVDRE_MASK               (0x01 << SPC_VD_CORE_CFG_HVDRE_SHIFT)
#define SPC_VD_CORE_CFG_HVDRE(x)                 (((x) << SPC_VD_CORE_CFG_HVDRE_SHIFT) & SPC_VD_CORE_CFG_HVDRE_MASK)

#define SPC_VD_CORE_CFG_HVDIE_SHIFT              (3)
#define SPC_VD_CORE_CFG_HVDIE_MASK               (0x01 << SPC_VD_CORE_CFG_HVDIE_SHIFT)
#define SPC_VD_CORE_CFG_HVDIE(x)                 (((x) << SPC_VD_CORE_CFG_HVDIE_SHIFT) & SPC_VD_CORE_CFG_HVDIE_MASK)

#define SPC_VD_CORE_CFG_LOCK_SHIFT               (16)
#define SPC_VD_CORE_CFG_LOCK_MASK                (0x01 << SPC_VD_CORE_CFG_LOCK_SHIFT)
#define SPC_VD_CORE_CFG_LOCK(x)                  (((x) << SPC_VD_CORE_CFG_LOCK_SHIFT) & SPC_VD_CORE_CFG_LOCK_MASK)

/* System Voltage Detect Configuration (VD_SYS_CFG) */

#define SPC_VD_SYS_CFG_LVDRE_SHIFT               (0)
#define SPC_VD_SYS_CFG_LVDRE_MASK                (0x01 << SPC_VD_SYS_CFG_LVDRE_SHIFT)
#define SPC_VD_SYS_CFG_LVDRE(x)                  (((x) << SPC_VD_SYS_CFG_LVDRE_SHIFT) & SPC_VD_SYS_CFG_LVDRE_MASK)

#define SPC_VD_SYS_CFG_LVDIE_SHIFT               (1)
#define SPC_VD_SYS_CFG_LVDIE_MASK                (0x01 << SPC_VD_SYS_CFG_LVDIE_SHIFT)
#define SPC_VD_SYS_CFG_LVDIE(x)                  (((x) << SPC_VD_SYS_CFG_LVDIE_SHIFT) & SPC_VD_SYS_CFG_LVDIE_MASK)

#define SPC_VD_SYS_CFG_HVDRE_SHIFT               (2)
#define SPC_VD_SYS_CFG_HVDRE_MASK                (0x01 << SPC_VD_SYS_CFG_HVDRE_SHIFT)
#define SPC_VD_SYS_CFG_HVDRE(x)                  (((x) << SPC_VD_SYS_CFG_HVDRE_SHIFT) & SPC_VD_SYS_CFG_HVDRE_MASK)

#define SPC_VD_SYS_CFG_HVDIE_SHIFT               (3)
#define SPC_VD_SYS_CFG_HVDIE_MASK                (0x01 << SPC_VD_SYS_CFG_HVDIE_SHIFT)
#define SPC_VD_SYS_CFG_HVDIE(x)                  (((x) << SPC_VD_SYS_CFG_HVDIE_SHIFT) & SPC_VD_SYS_CFG_HVDIE_MASK)

#define SPC_VD_SYS_CFG_LVSEL_SHIFT               (8)
#define SPC_VD_SYS_CFG_LVSEL_MASK                (0x01 << SPC_VD_SYS_CFG_LVSEL_SHIFT)
#define SPC_VD_SYS_CFG_LVSEL(x)                  (((x) << SPC_VD_SYS_CFG_LVSEL_SHIFT) & SPC_VD_SYS_CFG_LVSEL_MASK)

#define SPC_VD_SYS_CFG_LOCK_SHIFT                (16)
#define SPC_VD_SYS_CFG_LOCK_MASK                 (0x01 << SPC_VD_SYS_CFG_LOCK_SHIFT)
#define SPC_VD_SYS_CFG_LOCK(x)                   (((x) << SPC_VD_SYS_CFG_LOCK_SHIFT) & SPC_VD_SYS_CFG_LOCK_MASK)

/* IO Voltage Detect Configuration (VD_IO_CFG) */

#define SPC_VD_IO_CFG_LVDRE_SHIFT                (0)
#define SPC_VD_IO_CFG_LVDRE_MASK                 (0x01 << SPC_VD_IO_CFG_LVDRE_SHIFT)
#define SPC_VD_IO_CFG_LVDRE(x)                   (((x) << SPC_VD_IO_CFG_LVDRE_SHIFT) & SPC_VD_IO_CFG_LVDRE_MASK)

#define SPC_VD_IO_CFG_LVDIE_SHIFT                (1)
#define SPC_VD_IO_CFG_LVDIE_MASK                 (0x01 << SPC_VD_IO_CFG_LVDIE_SHIFT)
#define SPC_VD_IO_CFG_LVDIE(x)                   (((x) << SPC_VD_IO_CFG_LVDIE_SHIFT) & SPC_VD_IO_CFG_LVDIE_MASK)

#define SPC_VD_IO_CFG_HVDRE_SHIFT                (2)
#define SPC_VD_IO_CFG_HVDRE_MASK                 (0x01 << SPC_VD_IO_CFG_HVDRE_SHIFT)
#define SPC_VD_IO_CFG_HVDRE(x)                   (((x) << SPC_VD_IO_CFG_HVDRE_SHIFT) & SPC_VD_IO_CFG_HVDRE_MASK)

#define SPC_VD_IO_CFG_HVDIE_SHIFT                (3)
#define SPC_VD_IO_CFG_HVDIE_MASK                 (0x01 << SPC_VD_IO_CFG_HVDIE_SHIFT)
#define SPC_VD_IO_CFG_HVDIE(x)                   (((x) << SPC_VD_IO_CFG_HVDIE_SHIFT) & SPC_VD_IO_CFG_HVDIE_MASK)

#define SPC_VD_IO_CFG_LVSEL_SHIFT                (8)
#define SPC_VD_IO_CFG_LVSEL_MASK                 (0x01 << SPC_VD_IO_CFG_LVSEL_SHIFT)
#define SPC_VD_IO_CFG_LVSEL(x)                   (((x) << SPC_VD_IO_CFG_LVSEL_SHIFT) & SPC_VD_IO_CFG_LVSEL_MASK)

#define SPC_VD_IO_CFG_LOCK_SHIFT                 (16)
#define SPC_VD_IO_CFG_LOCK_MASK                  (0x01 << SPC_VD_IO_CFG_LOCK_SHIFT)
#define SPC_VD_IO_CFG_LOCK(x)                    (((x) << SPC_VD_IO_CFG_LOCK_SHIFT) & SPC_VD_IO_CFG_LOCK_MASK)

/* External Voltage Domain Configuration (EVD_CFG) */

#define SPC_EVD_CFG_EVDISO_SHIFT                 (0)
#define SPC_EVD_CFG_EVDISO_MASK                  (0x3f << SPC_EVD_CFG_EVDISO_SHIFT)
#define SPC_EVD_CFG_EVDISO(x)                    (((x) << SPC_EVD_CFG_EVDISO_SHIFT) & SPC_EVD_CFG_EVDISO_MASK)

#define SPC_EVD_CFG_EVDLPISO_SHIFT               (8)
#define SPC_EVD_CFG_EVDLPISO_MASK                (0x3f << SPC_EVD_CFG_EVDLPISO_SHIFT)
#define SPC_EVD_CFG_EVDLPISO(x)                  (((x) << SPC_EVD_CFG_EVDLPISO_SHIFT) & SPC_EVD_CFG_EVDLPISO_MASK)

#define SPC_EVD_CFG_EVDSTAT_SHIFT                (16)
#define SPC_EVD_CFG_EVDSTAT_MASK                 (0x3f << SPC_EVD_CFG_EVDSTAT_SHIFT)
#define SPC_EVD_CFG_EVDSTAT(x)                   (((x) << SPC_EVD_CFG_EVDSTAT_SHIFT) & SPC_EVD_CFG_EVDSTAT_MASK)

/* Glitch Detect Status Control (GLITCH_DETECT_SC) */

#define SPC_GLITCH_DETECT_SC_CNT_SELECT_SHIFT    (0)
#define SPC_GLITCH_DETECT_SC_CNT_SELECT_MASK     (0x03 << SPC_GLITCH_DETECT_SC_CNT_SELECT_SHIFT)
#define SPC_GLITCH_DETECT_SC_CNT_SELECT(x)       (((x) << SPC_GLITCH_DETECT_SC_CNT_SELECT_SHIFT) & SPC_GLITCH_DETECT_SC_CNT_SELECT_MASK)

#define SPC_GLITCH_DETECT_SC_TIMEOUT_SHIFT       (2)
#define SPC_GLITCH_DETECT_SC_TIMEOUT_MASK        (0x0f << SPC_GLITCH_DETECT_SC_TIMEOUT_SHIFT)
#define SPC_GLITCH_DETECT_SC_TIMEOUT(x)          (((x) << SPC_GLITCH_DETECT_SC_TIMEOUT_SHIFT) & SPC_GLITCH_DETECT_SC_TIMEOUT_MASK)

#define SPC_GLITCH_DETECT_SC_RE_SHIFT            (6)
#define SPC_GLITCH_DETECT_SC_RE_MASK             (0x01 << SPC_GLITCH_DETECT_SC_RE_SHIFT)
#define SPC_GLITCH_DETECT_SC_RE(x)               (((x) << SPC_GLITCH_DETECT_SC_RE_SHIFT) & SPC_GLITCH_DETECT_SC_RE_MASK)

#define SPC_GLITCH_DETECT_SC_IE_SHIFT            (7)
#define SPC_GLITCH_DETECT_SC_IE_MASK             (0x01 << SPC_GLITCH_DETECT_SC_IE_SHIFT)
#define SPC_GLITCH_DETECT_SC_IE(x)               (((x) << SPC_GLITCH_DETECT_SC_IE_SHIFT) & SPC_GLITCH_DETECT_SC_IE_MASK)

#define SPC_GLITCH_DETECT_SC_GLITCH_DETECT_FLAG_SHIFT (8)
#define SPC_GLITCH_DETECT_SC_GLITCH_DETECT_FLAG_MASK  (0x0f << SPC_GLITCH_DETECT_SC_GLITCH_DETECT_FLAG_SHIFT)
#define SPC_GLITCH_DETECT_SC_GLITCH_DETECT_FLAG(x)    (((x) << SPC_GLITCH_DETECT_SC_GLITCH_DETECT_FLAG_SHIFT) & SPC_GLITCH_DETECT_SC_GLITCH_DETECT_FLAG_MASK)

#define SPC_GLITCH_DETECT_SC_LOCK_SHIFT          (16)
#define SPC_GLITCH_DETECT_SC_LOCK_MASK           (0x01 << SPC_GLITCH_DETECT_SC_LOCK_SHIFT)
#define SPC_GLITCH_DETECT_SC_LOCK(x)             (((x) << SPC_GLITCH_DETECT_SC_LOCK_SHIFT) & SPC_GLITCH_DETECT_SC_LOCK_MASK)

/* LDO_CORE Configuration (CORELDO_CFG) */

#define SPC_CORELDO_CFG_DPDOWN_PULLDOWN_DISABLE_SHIFT (16)
#define SPC_CORELDO_CFG_DPDOWN_PULLDOWN_DISABLE_MASK  (0x01)
#define SPC_CORELDO_CFG_DPDOWN_PULLDOWN_DISABLE(x)    (((x) << SPC_CORELDO_CFG_DPDOWN_PULLDOWN_DISABLE_SHIFT) & SPC_CORELDO_CFG_DPDOWN_PULLDOWN_DISABLE_MASK)

/* LDO_SYS Configuration (SYSLDO_CFG) */

#define SPC_SYSLDO_CFG_ISINKEN_SHIFT             (0)
#define SPC_SYSLDO_CFG_ISINKEN_MASK              (0x01 << SPC_SYSLDO_CFG_ISINKEN_SHIFT)
#define SPC_SYSLDO_CFG_ISINKEN(x)                (((x) << SPC_SYSLDO_CFG_ISINKEN_SHIFT) & SPC_SYSLDO_CFG_ISINKEN_MASK)

/* DCDC Configuration (DCDC_CFG) */

#define SPC_DCDC_CFG_FREQ_CNTRL_ON_SHIFT         (0)
#define SPC_DCDC_CFG_FREQ_CNTRL_ON_MASK          (0x01 << SPC_DCDC_CFG_FREQ_CNTRL_ON_SHIFT)
#define SPC_DCDC_CFG_FREQ_CNTRL_ON(x)            (((x) << SPC_DCDC_CFG_FREQ_CNTRL_ON_SHIFT) & SPC_DCDC_CFG_FREQ_CNTRL_ON_MASK)

#define SPC_DCDC_CFG_FREQ_CNTRL_SHIFT            (8)
#define SPC_DCDC_CFG_FREQ_CNTRL_MASK             (0x3f << SPC_DCDC_CFG_FREQ_CNTRL_SHIFT)
#define SPC_DCDC_CFG_FREQ_CNTRL(x)               (((x) << SPC_DCDC_CFG_FREQ_CNTRL_SHIFT) & SPC_DCDC_CFG_FREQ_CNTRL_MASK)

#define SPC_DCDC_CFG_BLEED_EN_SHIFT              (19)
#define SPC_DCDC_CFG_BLEED_EN_MASK               (0x01 <> SPC_DCDC_CFG_BLEED_EN_SHIFT)
#define SPC_DCDC_CFG_BLEED_EN(x)                 (((x) << SPC_DCDC_CFG_BLEED_EN_SHIFT) & SPC_DCDC_CFG_BLEED_EN_MASK)

/* DCDC Burst Configuration (DCDC_BURST_CFG) */

#define SPC_DCDC_BURST_CFG_BURST_REQ_SHIFT       (0)
#define SPC_DCDC_BURST_CFG_BURST_REQ_MASK        (0x01 << SPC_DCDC_BURST_CFG_BURST_REQ_SHIFT)
#define SPC_DCDC_BURST_CFG_BURST_REQ(x)          (((x) << SPC_DCDC_BURST_CFG_BURST_REQ_SHIFT) & SPC_DCDC_BURST_CFG_BURST_REQ_MASK)

#define SPC_DCDC_BURST_CFG_EXT_BURST_EN_SHIFT    (1)
#define SPC_DCDC_BURST_CFG_EXT_BURST_EN_MASK     (0x01 << SPC_DCDC_BURST_CFG_EXT_BURST_EN_SHIFT)
#define SPC_DCDC_BURST_CFG_EXT_BURST_EN(x)       (((x) << SPC_DCDC_BURST_CFG_EXT_BURST_EN_SHIFT) & SPC_DCDC_BURST_CFG_EXT_BURST_EN_MASK)

#define SPC_DCDC_BURST_CFG_BURST_ACK_SHIFT       (3)
#define SPC_DCDC_BURST_CFG_BURST_ACK_MASK        (0x01 << SPC_DCDC_BURST_CFG_BURST_ACK_SHIFT)
#define SPC_DCDC_BURST_CFG_BURST_ACK(x)          (((x) << SPC_DCDC_BURST_CFG_BURST_ACK_SHIFT) & SPC_DCDC_BURST_CFG_BURST_ACK_MASK)

#define SPC_DCDC_BURST_CFG_PULSE_REFRESH_CNT_SHIFT (16)
#define SPC_DCDC_BURST_CFG_PULSE_REFRESH_CNT_MASK  (0xffff << SPC_DCDC_BURST_CFG_PULSE_REFRESH_CNT_SHIFT)
#define SPC_DCDC_BURST_CFG_PULSE_REFRESH_CNT(x)    (((x) << SPC_DCDC_BURST_CFG_PULSE_REFRESH_CNT_SHIFT) & SPC_DCDC_BURST_CFG_PULSE_REFRESH_CNT_MASK)

#endif /* ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_SPC_H */

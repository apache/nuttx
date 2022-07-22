/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_fs26.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_FS26_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_FS26_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MCU TO FS26 **************************************************************/

#define FS26_M_FS           (1 << 31)  /* Bit 31: Main or Fail-safe register selection (M/FS) */
#define FS26_REG_ADDR_SHIFT (25)       /* Bits 25-31: Register Address + M/FS */
#define FS26_REG_ADDR_MASK  (0x7F << FS26_REG_ADDR_SHIFT)
#define FS26_REG_ADDR(n)    (((n) << FS26_REG_ADDR_SHIFT) & FS26_REG_ADDR_MASK)
#define FS26_RW             (1 << 24)  /* Bit 24: Read/Write (reading Bit 24 = 0) */

/* FS26 General device status ***********************************************/

#define FS26_M_AVAL (1 << 31)  /* Bit 31: Main State machine availability (M_AVAL) */
#define FS26_FS_EN  (1 << 30)  /* Bit 30: Fail Safe State machine status (FS_EN) */
#define FS26_FS_G   (1 << 29)  /* Bit 29: Interrupt notification from the Fail-Safe domain (FS_G) */
#define FS26_COM_G  (1 << 28)  /* Bit 28: Interrupt notification from the M_COM_FLG register (COM_G) */
#define FS26_WIO_G  (1 << 27)  /* Bit 27: Interrupt notification from the M_WIO_FLG register (WIO_G) */
#define FS26_VSUP_G (1 << 26)  /* Bit 26: Interrupt notification from the M_VSUP_FLG register (VSUP_G) */
#define FS26_REG_G  (1 << 25)  /* Bit 25: Interrupt notification from the M_REG_FLG register (REG_G) */
#define FS26_TSD_G  (1 << 24)  /* Bit 24: Interrupt notification from the M_TSD_FLG register (TSD_G) */

/* FS26 Data encoding********************************************************/

#define FS26_DATA_LSB_SHIFT (8)   /* Bits 8-15: DATA_LSB */
#define FS26_DATA_LSB_MASK  (0xFF << FS26_DATA_LSB_SHIFT)
#define FS26_DATA_LSB(n)    (((n) << FS26_DATA_LSB_SHIFT) & FS26_DATA_LSB_MASK)
#define FS26_DATA_MSB_SHIFT (16)  /* Bits 16-23: DATA_MSB */
#define FS26_DATA_MSB_MASK  (0xFF << FS26_DATA_MSB_SHIFT)
#define FS26_DATA_MSB(n)    (((n) << FS26_DATA_MSB_SHIFT) & FS26_DATA_MSB_MASK)
#define FS26_DATA_SHIFT     (8)   /* Bits 8-23: DATA_MSB */
#define FS26_DATA_MASK      (0xFFFF << FS26_DATA_SHIFT)
#define FS26_SET_DATA(n)    (((n) << FS26_DATA_SHIFT) & FS26_DATA_MASK)
#define FS26_GET_DATA(n)    (((n) & FS26_DATA_MASK) >> FS26_DATA_SHIFT)
#define FS26_CRC_SHIFT      (0)   /* Bits 0-7: CRC */
#define FS26_CRC_MASK       (0xFF << FS26_CRC_SHIFT)
#define FS26_CRC(n)         (((n) << FS26_CRC_SHIFT) & FS26_CRC_MASK)

/* FS26 SPI register map */

#define FS26_M_DEVICE_ID (0x0)  /* */
#define FS26_M_PROGID    (0x1)  /* */
#define FS26_M_STATUS    (0x2)  /* */
#define FS26_M_TSD_FLG   (0x3)  /* */
#define FS26_M_TSD_MSK   (0x4)  /* */
#define FS26_M_REG_FLG   (0x5)  /* */
#define FS26_M_REG_MSK   (0x6)  /* */
#define FS26_M_VSUP_FLG  (0x7)  /* */
#define FS26_M_VSUP_MSK  (0x8)  /* */
#define FS26_M_WIO_FLG   (0x9)  /* */
#define FS26_M_WIO_MSK   (0xA)  /* */
#define FS26_M_COM_FLG   (0xB)  /* */
#define FS26_M_COM_MSK   (0xC)  /* */
#define FS26_M_SYS_CFG   (0xD)  /* */
#define FS26_M_TSD_CFG   (0xE)  /* */
#define FS26_M_REG_CFG   (0xF)  /* */
#define FS26_M_WIO_CFG   (0x10) /* */
#define FS26_M_REG_CTRL1 (0x11) /* */
#define FS26_M_REG_CTRL2 (0x12) /* */
#define FS26_M_AMUX_CTRL (0x13) /* */
#define FS26_M_LDT_CFG1  (0x14) /* */
#define FS26_M_LDT_CFG2  (0x15) /* */
#define FS26_M_LDT_CFG3  (0x16) /* */
#define FS26_M_LDT_CTRL  (0x17) /* */
#define FS26_M_MEMORY0   (0x18) /* */
#define FS26_M_MEMORY1   (0x19) /* */

/* FS26 Fail safe register map */
#define FS26_FS_GRL_FLAGS                 (0x40) /* */
#define FS26_FS_I_OVUV_SAFE_REACTION1     (0x41) /* */
#define FS26_FS_I_NOT_OVUV_SAFE_REACTION1 (0x42) /* */
#define FS26_FS_I_OVUV_SAFE_REACTION2     (0x43) /* */
#define FS26_FS_I_NOT_OVUV_SAFE_REACTION2 (0x44) /* */
#define FS26_FS_I_WD_CFG                  (0x45) /* */
#define FS26_FS_I_NOT_WD_CFG              (0x46) /* */
#define FS26_FS_I_SAFE_INPUTS             (0x47) /* */
#define FS26_FS_I_NOT_SAFE_INPUTS         (0x48) /* */
#define FS26_FS_I_FSSM                    (0x49) /* */
#define FS26_FS_I_NOT_FSSM                (0x4A) /* */
#define FS26_FS_WDW_DURATION              (0x4B) /* */
#define FS26_FS_NOT_WDW_DURATION          (0x4C) /* */
#define FS26_FS_WD_ANSWER                 (0x4D) /* */
#define FS26_FS_WD_TOKEN                  (0x4E) /* */
#define FS26_FS_ABIST_ON_DEMAND           (0x4F) /* */
#define FS26_FS_OVUV_REG_STATUS           (0x50) /* */
#define FS26_FS_RELEASE_FS0B_FS1B         (0x51) /* */
#define FS26_FS_SAFE_IOS_1                (0x52) /* */
#define FS26_FS_SAFE_IOS_2                (0x53) /* */
#define FS26_FS_DIAG_SAFETY1              (0x54) /* */
#define FS26_FS_DIAG_SAFETY2              (0x55) /* */
#define FS26_FS_INTB_MASK                 (0x56) /* */
#define FS26_FS_STATES                    (0x57) /* */
#define FS26_FS_LP_REQ                    (0x58) /* */
#define FS26_FS_LDT_LPSEL                 (0x59) /* */

/* FS_I_OVUV_SAFE_REACTION1 register */

#define VMON_PRE_OV_FS_REACTION_SHIFT             (14) /* Reaction on RSTB or FAIL SAFE outputs in case of OV detection on VMON_PRE */
#define VMON_PRE_OV_FS_REACTION_MASK              (0x3 << VMON_PRE_OV_FS_REACTION_SHIFT)
#  define VMON_PRE_OV_FS_REACTION_NO_EFFECT (0x0 << VMON_PRE_OV_FS_REACTION_SHIFT)
#  define VMON_PRE_OV_FS_REACTION_FS0B      (0x1 << VMON_PRE_OV_FS_REACTION_SHIFT)
#  define VMON_PRE_OV_FS_REACTION_RSTB_FS0B (0x2 << VMON_PRE_OV_FS_REACTION_SHIFT)

#define VMON_PRE_UV_FS_REACTION_SHIFT             (12) /* Reaction on RSTB or FAIL SAFE outputs in case of UV detection on VMON_PRE */
#define VMON_PRE_UV_FS_REACTION_MASK              (0x3 << VMON_PRE_UV_FS_REACTION_SHIFT)
#  define VMON_PRE_UV_FS_REACTION_NO_EFFECT (0x0 << VMON_PRE_UV_FS_REACTION_SHIFT)
#  define VMON_PRE_UV_FS_REACTION_FS0B      (0x1 << VMON_PRE_UV_FS_REACTION_SHIFT)
#  define VMON_PRE_UV_FS_REACTION_RSTB_FS0B (0x2 << VMON_PRE_UV_FS_REACTION_SHIFT)

#define VMON_CORE_OV_FS_REACTION_SHIFT             (10) /* Reaction on RSTB or FAIL SAFE outputs in case of OV detection on VMON_CORE */
#define VMON_CORE_OV_FS_REACTION_MASK              (0x3 << VMON_CORE_OV_FS_REACTION_SHIFT)
#  define VMON_CORE_OV_FS_REACTION_NO_EFFECT (0x0 << VMON_CORE_OV_FS_REACTION_SHIFT)
#  define VMON_CORE_OV_FS_REACTION_FS0B      (0x1 << VMON_CORE_OV_FS_REACTION_SHIFT)
#  define VMON_CORE_OV_FS_REACTION_RSTB_FS0B (0x2 << VMON_CORE_OV_FS_REACTION_SHIFT)

#define VMON_CORE_UV_FS_REACTION_SHIFT             (8) /* Reaction on RSTB or FAIL SAFE outputs in case of UV detection on VMON_CORE */
#define VMON_CORE_UV_FS_REACTION_MASK              (0x3 << VMON_CORE_UV_FS_REACTION_SHIFT)
#  define VMON_CORE_UV_FS_REACTION_NO_EFFECT (0x0 << VMON_CORE_UV_FS_REACTION_SHIFT)
#  define VMON_CORE_UV_FS_REACTION_FS0B      (0x1 << VMON_CORE_UV_FS_REACTION_SHIFT)
#  define VMON_CORE_UV_FS_REACTION_RSTB_FS0B (0x2 << VMON_CORE_UV_FS_REACTION_SHIFT)

#define VMON_LDO1_OV_FS_REACTION_SHIFT             (6) /* Reaction on RSTB or FAIL SAFE outputs in case of OV detection on VMON_LDO1 */
#define VMON_LDO1_OV_FS_REACTION_MASK              (0x3 << VMON_LDO1_OV_FS_REACTION_SHIFT)
#  define VMON_LDO1_OV_FS_REACTION_NO_EFFECT (0x0 << VMON_LDO1_OV_FS_REACTION_SHIFT)
#  define VMON_LDO1_OV_FS_REACTION_FS0B      (0x1 << VMON_LDO1_OV_FS_REACTION_SHIFT)
#  define VMON_LDO1_OV_FS_REACTION_RSTB_FS0B (0x2 << VMON_LDO1_OV_FS_REACTION_SHIFT)

#define VMON_LDO1_UV_FS_REACTION_SHIFT             (4) /* Reaction on RSTB or FAIL SAFE outputs in case of UV detection on VMON_LDO1 */
#define VMON_LDO1_UV_FS_REACTION_MASK              (0x3 << VMON_LDO1_UV_FS_REACTION_SHIFT)
#  define VMON_LDO1_UV_FS_REACTION_NO_EFFECT (0x0 << VMON_LDO1_UV_FS_REACTION_SHIFT)
#  define VMON_LDO1_UV_FS_REACTION_FS0B      (0x1 << VMON_LDO1_UV_FS_REACTION_SHIFT)
#  define VMON_LDO1_UV_FS_REACTION_RSTB_FS0B (0x2 << VMON_LDO1_UV_FS_REACTION_SHIFT)

#define VMON_LDO2_OV_FS_REACTION_SHIFT             (2) /* Reaction on RSTB or FAIL SAFE outputs in case of OV detection on VMON_LDO2 */
#define VMON_LDO2_OV_FS_REACTION_MASK              (0x3 << VMON_LDO2_OV_FS_REACTION_SHIFT)
#  define VMON_LDO2_OV_FS_REACTION_NO_EFFECT (0x0 << VMON_LDO2_OV_FS_REACTION_SHIFT)
#  define VMON_LDO2_OV_FS_REACTION_FS0B      (0x1 << VMON_LDO2_OV_FS_REACTION_SHIFT)
#  define VMON_LDO2_OV_FS_REACTION_RSTB_FS0B (0x2 << VMON_LDO2_OV_FS_REACTION_SHIFT)

#define VMON_LDO2_UV_FS_REACTION_SHIFT             (0) /* Reaction on RSTB or FAIL SAFE outputs in case of UV detection on VMON_LDO2 */
#define VMON_LDO2_UV_FS_REACTION_MASK              (0x3 << VMON_LDO2_UV_FS_REACTION_SHIFT)
#  define VMON_LDO2_UV_FS_REACTION_NO_EFFECT (0x0 << VMON_LDO2_UV_FS_REACTION_SHIFT)
#  define VMON_LDO2_UV_FS_REACTION_FS0B      (0x1 << VMON_LDO2_UV_FS_REACTION_SHIFT)
#  define VMON_LDO2_UV_FS_REACTION_RSTB_FS0B (0x2 << VMON_LDO2_UV_FS_REACTION_SHIFT)

/* FS_I_OVUV_SAFE_REACTION2 register */

#define VMON_EXT_OV_FS_REACTION_SHIFT             (14) /* Reaction on RSTB or FAIL SAFE outputs in case of OV detection on VMON_EXT */
#define VMON_EXT_OV_FS_REACTION_MASK              (0x3 << VMON_EXT_OV_FS_REACTION_SHIFT)
#  define VMON_EXT_OV_FS_REACTION_NO_EFFECT (0x0 << VMON_EXT_OV_FS_REACTION_SHIFT)
#  define VMON_EXT_OV_FS_REACTION_FS0B      (0x1 << VMON_EXT_OV_FS_REACTION_SHIFT)
#  define VMON_EXT_OV_FS_REACTION_RSTB_FS0B (0x2 << VMON_EXT_OV_FS_REACTION_SHIFT)

#define VMON_EXT_UV_FS_REACTION_SHIFT             (12) /* Reaction on RSTB or FAIL SAFE outputs in case of UV detection on VMON_EXT */
#define VMON_EXT_UV_FS_REACTION_MASK              (0x3 << VMON_EXT_UV_FS_REACTION_SHIFT)
#  define VMON_EXT_UV_FS_REACTION_NO_EFFECT (0x0 << VMON_EXT_UV_FS_REACTION_SHIFT)
#  define VMON_EXT_UV_FS_REACTION_FS0B      (0x1 << VMON_EXT_UV_FS_REACTION_SHIFT)
#  define VMON_EXT_UV_FS_REACTION_RSTB_FS0B (0x2 << VMON_EXT_UV_FS_REACTION_SHIFT)

#define VMON_REF_OV_FS_REACTION_SHIFT             (10) /* Reaction on RSTB or FAIL SAFE outputs in case of OV detection on VMON_REF */
#define VMON_REF_OV_FS_REACTION_MASK              (0x3 << VMON_REF_OV_FS_REACTION_SHIFT)
#  define VMON_REF_OV_FS_REACTION_NO_EFFECT (0x0 << VMON_REF_OV_FS_REACTION_SHIFT)
#  define VMON_REF_OV_FS_REACTION_FS0B      (0x1 << VMON_REF_OV_FS_REACTION_SHIFT)
#  define VMON_REF_OV_FS_REACTION_RSTB_FS0B (0x2 << VMON_REF_OV_FS_REACTION_SHIFT)

#define VMON_REF_UV_FS_REACTION_SHIFT             (8) /* Reaction on RSTB or FAIL SAFE outputs in case of UV detection on VMON_REF */
#define VMON_REF_UV_FS_REACTION_MASK              (0x3 << VMON_REF_UV_FS_REACTION_SHIFT)
#  define VMON_REF_UV_FS_REACTION_NO_EFFECT (0x0 << VMON_REF_UV_FS_REACTION_SHIFT)
#  define VMON_REF_UV_FS_REACTION_FS0B      (0x1 << VMON_REF_UV_FS_REACTION_SHIFT)
#  define VMON_REF_UV_FS_REACTION_RSTB_FS0B (0x2 << VMON_REF_UV_FS_REACTION_SHIFT)

#define VMON_TRK2_OV_FS_REACTION_SHIFT             (6) /* Reaction on RSTB or FAIL SAFE outputs in case of OV detection on VMON_TRK2 */
#define VMON_TRK2_OV_FS_REACTION_MASK              (0x3 << VMON_TRK2_OV_FS_REACTION_SHIFT)
#  define VMON_TRK2_OV_FS_REACTION_NO_EFFECT (0x0 << VMON_TRK2_OV_FS_REACTION_SHIFT)
#  define VMON_TRK2_OV_FS_REACTION_FS0B      (0x1 << VMON_TRK2_OV_FS_REACTION_SHIFT)
#  define VMON_TRK2_OV_FS_REACTION_RSTB_FS0B (0x2 << VMON_TRK2_OV_FS_REACTION_SHIFT)

#define VMON_TRK2_UV_FS_REACTION_SHIFT             (4) /* Reaction on RSTB or FAIL SAFE outputs in case of UV detection on VMON_TRK2 */
#define VMON_TRK2_UV_FS_REACTION_MASK              (0x3 << VMON_TRK2_UV_FS_REACTION_SHIFT)
#  define VMON_TRK2_UV_FS_REACTION_NO_EFFECT (0x0 << VMON_TRK2_UV_FS_REACTION_SHIFT)
#  define VMON_TRK2_UV_FS_REACTION_FS0B      (0x1 << VMON_TRK2_UV_FS_REACTION_SHIFT)
#  define VMON_TRK2_UV_FS_REACTION_RSTB_FS0B (0x2 << VMON_TRK2_UV_FS_REACTION_SHIFT)

#define VMON_TRK1_OV_FS_REACTION_SHIFT             (2) /* Reaction on RSTB or FAIL SAFE outputs in case of OV detection on VMON_TRK1 */
#define VMON_TRK1_OV_FS_REACTION_MASK              (0x3 << VMON_TRK1_OV_FS_REACTION_SHIFT)
#  define VMON_TRK1_OV_FS_REACTION_NO_EFFECT (0x0 << VMON_TRK1_OV_FS_REACTION_SHIFT)
#  define VMON_TRK1_OV_FS_REACTION_FS0B      (0x1 << VMON_TRK1_OV_FS_REACTION_SHIFT)
#  define VMON_TRK1_OV_FS_REACTION_RSTB_FS0B (0x2 << VMON_TRK1_OV_FS_REACTION_SHIFT)

#define VMON_TRK1_UV_FS_REACTION_SHIFT             (0) /* Reaction on RSTB or FAIL SAFE outputs in case of UV detection on VMON_TRK1 */
#define VMON_TRK1_UV_FS_REACTION_MASK              (0x3 << VMON_TRK1_UV_FS_REACTION_SHIFT)
#  define VMON_TRK1_UV_FS_REACTION_NO_EFFECT (0x0 << VMON_TRK1_UV_FS_REACTION_SHIFT)
#  define VMON_TRK1_UV_FS_REACTION_FS0B      (0x1 << VMON_TRK1_UV_FS_REACTION_SHIFT)
#  define VMON_TRK1_UV_FS_REACTION_RSTB_FS0B (0x2 << VMON_TRK1_UV_FS_REACTION_SHIFT)

/* FS26_FS_I_WD_CFG register */

#define WD_ERR_LIMIT_SHIFT             (14) /* Watchdog error counter limit  */
#define WD_ERR_LIMIT_MASK              (0x3 << WD_ERR_LIMIT_SHIFT)
#  define WD_ERR_LIMIT_8 (0x0 << WD_ERR_LIMIT_SHIFT)
#  define WD_ERR_LIMIT_6 (0x1 << WD_ERR_LIMIT_SHIFT)
#  define WD_ERR_LIMIT_4 (0x2 << WD_ERR_LIMIT_SHIFT)
#  define WD_ERR_LIMIT_2 (0x3 << WD_ERR_LIMIT_SHIFT)

#define WD_RFR_LIMIT_SHIFT             (11) /* Watchdog refresh counter limit  */
#define WD_RFR_LIMIT_MASK              (0x3 << WD_RFR_LIMIT_SHIFT)
#  define WD_RFR_LIMIT_6 (0x0 << WD_RFR_LIMIT_SHIFT)
#  define WD_RFR_LIMIT_4 (0x1 << WD_RFR_LIMIT_SHIFT)
#  define WD_RFR_LIMIT_2 (0x2 << WD_RFR_LIMIT_SHIFT)
#  define WD_RFR_LIMIT_1 (0x3 << WD_RFR_LIMIT_SHIFT)

#define WD_FS_REACTION_SHIFT             (8) /* Reaction on RSTB or FAIL SAFE output in case of BAD Watchdog (data or timing)  */
#define WD_FS_REACTION_MASK              (0x3 << WD_FS_REACTION_SHIFT)
#  define WD_FS_REACTION_NO_ACTION (0x0 << WD_FS_REACTION_SHIFT)
#  define WD_FS_REACTION_FS0B      (0x1 << WD_FS_REACTION_SHIFT)
#  define WD_FS_REACTION_RSTB_FS0B (0x2 << WD_FS_REACTION_SHIFT)

#define WD_RFR_CNT_SHIFT             (8) /* Reflect the value of the Watchdog Refresh Counter */
#define WD_RFR_CNT_MASK              (0x7 << WD_RFR_CNT_SHIFT)
#define WD_RFR_CNT(n)                (n & (0x7 << WD_RFR_CNT_SHIFT))

#define WD_ERR_CNT_SHIFT             (0) /* Reflect the value of the Watchdog Error Counter */
#define WD_ERR_CNT_MASK              (0xF << WD_ERR_CNT_SHIFT)
#define WD_ERR_CNT(n)                ((n & (0x7 << WD_RFR_CNT_SHIFT)) > 11) ? (11) : ((n & (0x7 << WD_RFR_CNT_SHIFT)))

/* FS26_FS_I_SAFE_INPUTS register */

#define FCCU_CFG_SHIFT                    (13) /* FCCU Monitoring Configuration */
#define FCCU_CFG_MASK                     (0x7 << FCCU_CFG_SHIFT)
#  define FCCU_CFG_NO_MONITORING          (0x0 << FCCU_CFG_SHIFT)
#  define FCCU_CFG_FCCU1_FCCU2_PAIR       (0x1 << FCCU_CFG_SHIFT)
#  define FCCU_CFG_FCCU1_FCCU2_SINGLE     (0x2 << FCCU_CFG_SHIFT)
#  define FCCU_CFG_FCCU1_ONLY             (0x3 << FCCU_CFG_SHIFT)
#  define FCCU_CFG_FCCU2_ONLY             (0x4 << FCCU_CFG_SHIFT)
#  define FCCU_CFG_FCCU1_FCCU2_PWM        (0x5 << FCCU_CFG_SHIFT)
#  define FCCU_CFG_FCCU1_PWM_FCCU2_SINGLE (0x6 << FCCU_CFG_SHIFT)
#  define FCCU_CFG_FCCU2_PWM_FCCU1_SINGLE (0x7 << FCCU_CFG_SHIFT)

#define FCCU12_FLT_POL_SHIFT                    (12) /* FCCU12 Fault Polarity */
#define FCCU12_FLT_POL_MASK                     (0x1 << FCCU12_FLT_POL_SHIFT)
#define FCCU12_FLT_POL_FCCU1_0_FCCU2_1_IS_FAULT (0x0 << FCCU12_FLT_POL_SHIFT)
#define FCCU12_FLT_POL_FCCU1_1_FCCU2_0_IS_FAULT (0x1 << FCCU12_FLT_POL_SHIFT)

#define FCCU1_FLT_POL_SHIFT                    (11) /* FCCU1 Fault Polarity */
#define FCCU1_FLT_POL_MASK                     (0x1 << FCCU1_FLT_POL_SHIFT)
#define FCCU1_FLT_POL_LOW  (0x0 << FCCU1_FLT_POL_SHIFT)
#define FCCU1_FLT_POL_HIGH (0x1 << FCCU1_FLT_POL_SHIFT)

#define FCCU2_FLT_POL_SHIFT                    (10) /* FCCU2 Fault Polarity */
#define FCCU2_FLT_POL_MASK                     (0x1 << FCCU2_FLT_POL_SHIFT)
#define FCCU2_FLT_POL_LOW  (0x0 << FCCU2_FLT_POL_SHIFT)
#define FCCU2_FLT_POL_HIGH (0x1 << FCCU2_FLT_POL_SHIFT)

#define FCCU12_FS_REACTION_SHIFT               (9) /* Reaction on RSTB or FAIL SAFE output in case of FAULT DETECTION ON FCCU12 */
#define FCCU12_FS_REACTION_MASK                (0x1 << FCCU12_FS_REACTION_SHIFT)
#define FCCU12_FS_REACTION                     FCCU12_FS_REACTION_MASK

#define FCCU1_FS_REACTION_SHIFT               (8) /* Reaction on RSTB or FAIL SAFE output in case of FAULT DETECTION ON FCCU1 */
#define FCCU1_FS_REACTION_MASK                (0x1 << FCCU1_FS_REACTION_SHIFT)
#define FCCU1_FS_REACTION                     FCCU1_FS_REACTION_MASK

#define FCCU2_FS_REACTION_SHIFT               (7) /* Reaction on RSTB or FAIL SAFE output in case of FAULT DETECTION ON FCCU2 */
#define FCCU2_FS_REACTION_MASK                (0x1 << FCCU2_FS_REACTION_SHIFT)
#define FCCU2_FS_REACTION                     FCCU2_FS_REACTION_MASK

#define ERRMON_FLT_POLARITY_SHIFT                    (5) /* ERRORMON Fault Polarity */
#define ERRMON_FLT_POLARITY_MASK                     (0x1 << ERRMON_FLT_POLARITY_SHIFT)
#define ERRMON_FLT_POLARITY_LOW  (0x0 << ERRMON_FLT_POLARITY_SHIFT)
#define ERRMON_FLT_POLARITY_HIGH (0x1 << ERRMON_FLT_POLARITY_SHIFT)

#define ERRMON_ACK_TIME_SHIFT             (3) /* Acknowledge timing following a fault detection on ERRMON */
#define ERRMON_ACK_TIME_MASK              (0x3 << ERRMON_ACK_TIME_SHIFT)
#  define ERRMON_ACK_TIME_1MS  (0x0 << ERRMON_ACK_TIME_SHIFT)
#  define ERRMON_ACK_TIME_8MS  (0x1 << ERRMON_ACK_TIME_SHIFT)
#  define ERRMON_ACK_TIME_16MS (0x2 << ERRMON_ACK_TIME_SHIFT)
#  define ERRMON_ACK_TIME_32MS (0x3 << ERRMON_ACK_TIME_SHIFT)

#define ERRMON_FS_REACTION_SHIFT               (2) /* Reaction on RSTB or Fail Safe output in case of fault detection on ERRMON */
#define ERRMON_FS_REACTION_MASK                (0x1 << FCCU2_FS_REACTION_SHIFT)
#define ERRMON_FS_REACTION                     FCCU2_FS_REACTION_MASK

#define FCCU12_FILT_SHIFT             (0) /* FCCU pin filtering time settings */
#define FCCU12_FILT_MASK              (0x3 << FCCU12_FILT_SHIFT)
#  define FCCU12_FILT_3US  (0x0 << FCCU12_FILT_SHIFT)
#  define FCCU12_FILT_6US  (0x1 << FCCU12_FILT_SHIFT)
#  define FCCU12_FILT_10US (0x2 << FCCU12_FILT_SHIFT)
#  define FCCU12_FILT_20US (0x3 << FCCU12_FILT_SHIFT)

/* FS26_FS_I_FSSM register */

#define FLT_ERR_CNT_LIMIT_SHIFT             (14) /* Configure the maximum level of the fault counter */
#define FLT_ERR_CNT_LIMIT_MASK              (0x3 << FLT_ERR_CNT_LIMIT_SHIFT)
#  define FLT_ERR_CNT_LIMIT_2  (0x0 << FLT_ERR_CNT_LIMIT_SHIFT)
#  define FLT_ERR_CNT_LIMIT_6  (0x1 << FLT_ERR_CNT_LIMIT_SHIFT)
#  define FLT_ERR_CNT_LIMIT_8  (0x2 << FLT_ERR_CNT_LIMIT_SHIFT)
#  define FLT_ERR_CNT_LIMIT_12 (0x3 << FLT_ERR_CNT_LIMIT_SHIFT)

#define FLT_ERR_REACTION_SHIFT             (8) /* Configure the RSTB and FS0B behavior when fault error counter ≥ intermediate value  */
#define FLT_ERR_REACTION_MASK              (0x3 << FLT_ERR_REACTION_SHIFT)
#  define FLT_ERR_REACTION_NO_EFFECT (0x0 << FLT_ERR_REACTION_SHIFT)
#  define FLT_ERR_REACTION_FS0B      (0x1 << FLT_ERR_REACTION_SHIFT)
#  define FLT_ERR_REACTION_RSTB_FS0B (0x2 << FLT_ERR_REACTION_SHIFT)

#define RSTB_DUR_SHIFT               (9) /* Reset duration configuration */
#define RSTB_DUR_MASK                (0x1 << RSTB_DUR_SHIFT)
#define RSTB_DUR_1MS                 RSTB_DUR_MASK
#define RSTB_DUR_10MS                (0)

#define BACKUP_SAFETY_PATH_FS0B_SHIFT               (7) /* Assert RSTB in case a short to high is detected on FS0B */
#define BACKUP_SAFETY_PATH_FS0B_MASK                (0x1 << BACKUP_SAFETY_PATH_FS0B_SHIFT)
#define BACKUP_SAFETY_PATH_FS0B                     BACKUP_SAFETY_PATH_FS0B_MASK

#define BACKUP_SAFETY_PATH_FS1B_SHIFT               (6) /* Assert RSTB in case a short to high is detected on FS1B */
#define BACKUP_SAFETY_PATH_FS1B_MASK                (0x1 << BACKUP_SAFETY_PATH_FS1B_SHIFT)
#define BACKUP_SAFETY_PATH_FS1B                     BACKUP_SAFETY_PATH_FS1B_MASK

#define CLK_MON_DIS_SHIFT               (5) /* Disable CLK Monitoring */
#define CLK_MON_DIS_MASK                (0x1 << CLK_MON_DIS_SHIFT)
#define CLK_MON_DIS                     CLK_MON_DIS_MASK

#define DIS8S_SHIFT               (4) /* Disable 8s RSTB timer */
#define DIS8S_MASK                (0x1 << DIS8S_SHIFT)
#define DIS8S                     DIS8S_MASK

#define FLT_ERR_CNT_SHIFT             (0) /* Reflect the value of the Watchdog Error Counter */
#define FLT_ERR_CNT_MASK              (0xF << FLT_ERR_CNT_SHIFT)
#define FLT_ERR_CNT(n)                ((n & (0x7 << FLT_ERR_CNT_SHIFT)) > 12) ? (12) : ((n & (0x7 << FLT_ERR_CNT_SHIFT)))

/* FS26_FS_WDW_DURATION register */

#define WDW_PERIOD_SHIFT               (12) /* Watchdog window period */
#define WDW_PERIOD_MASK                (0xF << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_DISABLE           (0x0 << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_1MS               (0x1 << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_2MS               (0x2 << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_3MS               (0x3 << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_4MS               (0x4 << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_6MS               (0x5 << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_8MS               (0x6 << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_12MS              (0x7 << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_16MS              (0x8 << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_24MS              (0x9 << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_32MS              (0xA << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_64MS              (0xB << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_128MS             (0xC << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_256MS             (0xD << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_512MS             (0xE << WDW_PERIOD_SHIFT)
#  define WDW_PERIOD_1024MS            (0xF << WDW_PERIOD_SHIFT)

#define WDW_DC_SHIFT                   (6) /* Watchdog window duty cycle */
#define WDW_DC_MASK                    (0x7 << WDW_DC_SHIFT)
#  define WDW_DC_31_68                 (0x0 << WDW_PERIOD_SHIFT)
#  define WDW_DC_37_62                 (0x1 << WDW_PERIOD_SHIFT)
#  define WDW_DC_50_50                 (0x2 << WDW_PERIOD_SHIFT)
#  define WDW_DC_62_37                 (0x3 << WDW_PERIOD_SHIFT)
#  define WDW_DC_68_31                 (0x4 << WDW_PERIOD_SHIFT)

#define WDW_RECOVERY_SHIFT               (0) /* Watchdog window period */
#define WDW_RECOVERY_MASK                (0xF << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_DISABLE           (0x0 << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_1MS               (0x1 << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_2MS               (0x2 << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_3MS               (0x3 << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_4MS               (0x4 << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_6MS               (0x5 << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_8MS               (0x6 << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_12MS              (0x7 << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_16MS              (0x8 << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_24MS              (0x9 << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_32MS              (0xA << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_64MS              (0xB << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_128MS             (0xC << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_256MS             (0xD << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_512MS             (0xE << WDW_RECOVERY_SHIFT)
#  define WDW_RECOVERY_1024MS            (0xF << WDW_RECOVERY_SHIFT)

/* FS26_FS_DIAG_SAFETY1 register */

#define BAD_WD_DATA_SHIFT               (10) /* Bad WD refresh, Error in the data */
#define BAD_WD_DATA_MASK                (0x1 << BAD_WD_DATA_SHIFT)
#define BAD_WD_DATA                     BAD_WD_DATA_MASK

#define BAD_WD_TIMING_SHIFT               (9) /* Bad WD refresh, Error in the timing */
#define BAD_WD_TIMING_MASK                (0x1 << BAD_WD_TIMING_SHIFT)
#define BAD_WD_TIMING                     BAD_WD_TIMING_MASK

#define ABIST1_PASS_SHIFT               (8) /* ABIST 1 pass */
#define ABIST1_PASS_MASK                (0x1 << ABIST1_PASS_SHIFT)
#define ABIST1_PASS                     ABIST1_PASS_MASK

#define ABIST2_PASS_SHIFT               (7) /* ABIST 2 pass */
#define ABIST2_PASS_MASK                (0x1 << ABIST2_PASS_SHIFT)
#define ABIST2_PASS                     ABIST2_PASS_MASK

#define ABIST2_DONE_SHIFT               (6) /* ABIST 2 done */
#define ABIST2_DONE_MASK                (0x1 << ABIST2_DONE_SHIFT)
#define ABIST2_DONE                     ABIST2_DONE_MASK

#define SPI_FS_CLK_SHIFT               (5) /* SPI CLK error */
#define SPI_FS_CLK_MASK                (0x1 << SPI_FS_CLK_SHIFT)
#define SPI_FS_CLK                     SPI_FS_CLK_MASK

#define SPI_FS_REQ_SHIFT               (4) /* SPI invalid read/write error */
#define SPI_FS_REQ_MASK                (0x1 << SPI_FS_REQ_SHIFT)
#define SPI_FS_REQ                     SPI_FS_REQ_MASK

#define SPI_FS_CRC_SHIFT               (3) /* SPI CRC error */
#define SPI_FS_CRC_MASK                (0x1 << SPI_FS_CRC_SHIFT)
#define SPI_FS_CRC                     SPI_FS_CRC_MASK

#define FS_OSC_DRIFT_SHIFT               (2) /* FS OSC drift */
#define FS_OSC_DRIFT_MASK                (0x1 << FS_OSC_DRIFT_SHIFT)
#define FS_OSC_DRIFT                     FS_OSC_DRIFT_MASK

#define LBIST_STATUS_SHIFT               (0) /* LBIST STATUS */
#define LBIST_STATUS_MASK                (0x3 << LBIST_STATUS_SHIFT)
#define LBIST_STATUS                     LBIST_STATUS_MASK
#  define LBIST_STATUS_FAIL              (0x0 << LBIST_STATUS_SHIFT)
#  define LBIST_STATUS_BYPASSED          (0x1 << LBIST_STATUS_SHIFT)
#  define LBIST_STATUS_FAIL2             (0x2 << LBIST_STATUS_SHIFT)
#  define LBIST_STATUS_OK                (0x3 << LBIST_STATUS_SHIFT)

/* FS26_FS_STATES register */

#define EXIT_DBG_MODE_SHIFT               (14) /* Leave debug mode */
#define EXIT_DBG_MODE_MASK                (0x1 << EXIT_DBG_MODE_SHIFT)
#define EXIT_DBG_MODE                     EXIT_DBG_MODE_MASK

#define DBG_MODE_SHIFT               (13) /* debug mode */
#define DBG_MODE_MASK                (0x1 << DBG_MODE_SHIFT)
#define DBG_MODE                     DBG_MODE_MASK

#define OTP_CORRUPT_SHIFT               (12) /* OTP crc error */
#define OTP_CORRUPT_MASK                (0x1 << OTP_CORRUPT_SHIFT)
#define OTP_CORRUPT                     OTP_CORRUPT_MASK

#define REG_CORRUPT_SHIFT               (11) /* INIT register error */
#define REG_CORRUPT_MASK                (0x1 << REG_CORRUPT_SHIFT)
#define REG_CORRUPT                     REG_CORRUPT_MASK

#define FS_STATES_SHIFT               (0) /* LBIST STATUS */
#define FS_STATES_MASK                (0x1F << FS_STATES_SHIFT)
#define FS_STATES                     FS_STATES_MASK
#  define FS_STATES_DEBUG_ENTRY       (0x4 << FS_STATES_SHIFT)
#  define FS_STATES_ENABLE_MON        (0x6 << FS_STATES_SHIFT)
#  define FS_STATES_RSTB_RELEASE      (0x8 << FS_STATES_SHIFT)
#  define FS_STATES_INIT_FS           (0x9 << FS_STATES_SHIFT)
#  define FS_STATES_SAFETY_OUT_NOT    (0xA << FS_STATES_SHIFT)
#  define FS_STATES_NORMAL            (0xB << FS_STATES_SHIFT)

/* FS26_FS_GRL_FLAGS register */

#define FS_COM_G_SHIFT               (15) /* Report an issue in the communication (SPI) */
#define FS_COM_G_MASK                (0x1 << FS_COM_G_SHIFT)
#define FS_COM_G                     FS_COM_G_MASK

#define FS_WD_G_SHIFT                (14) /* Report an issue on the Watchdog Refresh */
#define FS_WD_G_MASK                 (0x1 << FS_WD_G_SHIFT)
#define FS_WD_G                      FS_WD_G_MASK

#define FS_IO_G_SHIFT                (13) /* Report an issue in one of the Fail Safe IOs */
#define FS_IO_G_MASK                 (0x1 << FS_IO_G_SHIFT)
#define FS_IO_G                      FS_IO_G_MASK

#define FS_REG_OVUV_G_SHIFT          (12) /* Report an issue in one of the voltage monitoring (OV or UV) */
#define FS_REG_OVUV_G_MASK           (0x1 << FS_REG_OVUV_G_SHIFT)
#define FS_REG_OVUV_G                FS_REG_OVUV_G_MASK

#define FS_BIST_G_SHIFT              (11) /* Report an issue on BIST (Logical or Analog) */
#define FS_BIST_G_MASK               (0x1 << FS_BIST_G_SHIFT)
#define FS_BIST_G                    FS_BIST_G_MASK

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_FS26_H */

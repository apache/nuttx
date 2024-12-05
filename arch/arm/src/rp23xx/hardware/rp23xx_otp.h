/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_otp.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_OTP_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_OTP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_OTP_SW_LOCK_OFFSET(n)        (0x000000 + (n) * 4)
#define RP23XX_OTP_SBPI_INSTR_OFFSET        0x00000100
#define RP23XX_OTP_SBPI_WDATA_0_OFFSET      0x00000104
#define RP23XX_OTP_SBPI_WDATA_1_OFFSET      0x00000108
#define RP23XX_OTP_SBPI_WDATA_2_OFFSET      0x0000010c
#define RP23XX_OTP_SBPI_WDATA_3_OFFSET      0x00000110
#define RP23XX_OTP_SBPI_RDATA_0_OFFSET      0x00000114
#define RP23XX_OTP_SBPI_RDATA_1_OFFSET      0x00000118
#define RP23XX_OTP_SBPI_RDATA_2_OFFSET      0x0000011c
#define RP23XX_OTP_SBPI_RDATA_3_OFFSET      0x00000120
#define RP23XX_OTP_SBPI_STATUS_OFFSET       0x00000124
#define RP23XX_OTP_USR_OFFSET               0x00000128
#define RP23XX_OTP_DBG_OFFSET               0x0000012c
#define RP23XX_OTP_BIST_OFFSET              0x00000134
#define RP23XX_OTP_CRT_KEY_W0_OFFSET        0x00000138
#define RP23XX_OTP_CRT_KEY_W1_OFFSET        0x0000013c
#define RP23XX_OTP_CRT_KEY_W2_OFFSET        0x00000140
#define RP23XX_OTP_CRT_KEY_W3_OFFSET        0x00000144
#define RP23XX_OTP_CRITICAL_OFFSET          0x00000148
#define RP23XX_OTP_KEY_VALID_OFFSET         0x0000014c
#define RP23XX_OTP_DEBUGEN_OFFSET           0x00000150
#define RP23XX_OTP_DEBUGEN_LOCK_OFFSET      0x00000154
#define RP23XX_OTP_ARCHSEL_OFFSET           0x00000158
#define RP23XX_OTP_ARCHSEL_STATUS_OFFSET    0x0000015c
#define RP23XX_OTP_BOOTDIS_OFFSET           0x00000160
#define RP23XX_OTP_INTR_OFFSET              0x00000164
#define RP23XX_OTP_INTE_OFFSET              0x00000168
#define RP23XX_OTP_INTF_OFFSET              0x0000016c
#define RP23XX_OTP_INTS_OFFSET              0x00000170

/* Register definitions *****************************************************/

#define RP23XX_OTP_SW_LOCK(n)       (RP23XX_OTP_BASE + RP23XX_OTP_SW_LOCK_OFFSET(n))
#define RP23XX_OTP_SBPI_INSTR       (RP23XX_OTP_BASE + RP23XX_OTP_SBPI_INSTR_OFFSET)
#define RP23XX_OTP_SBPI_WDATA_0     (RP23XX_OTP_BASE + RP23XX_OTP_SBPI_WDATA_0_OFFSET)
#define RP23XX_OTP_SBPI_WDATA_1     (RP23XX_OTP_BASE + RP23XX_OTP_SBPI_WDATA_1_OFFSET)
#define RP23XX_OTP_SBPI_WDATA_2     (RP23XX_OTP_BASE + RP23XX_OTP_SBPI_WDATA_2_OFFSET)
#define RP23XX_OTP_SBPI_WDATA_3     (RP23XX_OTP_BASE + RP23XX_OTP_SBPI_WDATA_3_OFFSET)
#define RP23XX_OTP_SBPI_RDATA_0     (RP23XX_OTP_BASE + RP23XX_OTP_SBPI_RDATA_0_OFFSET)
#define RP23XX_OTP_SBPI_RDATA_1     (RP23XX_OTP_BASE + RP23XX_OTP_SBPI_RDATA_1_OFFSET)
#define RP23XX_OTP_SBPI_RDATA_2     (RP23XX_OTP_BASE + RP23XX_OTP_SBPI_RDATA_2_OFFSET)
#define RP23XX_OTP_SBPI_RDATA_3     (RP23XX_OTP_BASE + RP23XX_OTP_SBPI_RDATA_3_OFFSET)
#define RP23XX_OTP_SBPI_STATUS      (RP23XX_OTP_BASE + RP23XX_OTP_SBPI_STATUS_OFFSET)
#define RP23XX_OTP_USR              (RP23XX_OTP_BASE + RP23XX_OTP_USR_OFFSET)
#define RP23XX_OTP_DBG              (RP23XX_OTP_BASE + RP23XX_OTP_DBG_OFFSET)
#define RP23XX_OTP_BIST             (RP23XX_OTP_BASE + RP23XX_OTP_BIST_OFFSET)
#define RP23XX_OTP_CRT_KEY_W0       (RP23XX_OTP_BASE + RP23XX_OTP_CRT_KEY_W0_OFFSET)
#define RP23XX_OTP_CRT_KEY_W1       (RP23XX_OTP_BASE + RP23XX_OTP_CRT_KEY_W1_OFFSET)
#define RP23XX_OTP_CRT_KEY_W2       (RP23XX_OTP_BASE + RP23XX_OTP_CRT_KEY_W2_OFFSET)
#define RP23XX_OTP_CRT_KEY_W3       (RP23XX_OTP_BASE + RP23XX_OTP_CRT_KEY_W3_OFFSET)
#define RP23XX_OTP_CRITICAL         (RP23XX_OTP_BASE + RP23XX_OTP_CRITICAL_OFFSET)
#define RP23XX_OTP_KEY_VALID        (RP23XX_OTP_BASE + RP23XX_OTP_KEY_VALID_OFFSET)
#define RP23XX_OTP_DEBUGEN          (RP23XX_OTP_BASE + RP23XX_OTP_DEBUGEN_OFFSET)
#define RP23XX_OTP_DEBUGEN_LOCK     (RP23XX_OTP_BASE + RP23XX_OTP_DEBUGEN_LOCK_OFFSET)
#define RP23XX_OTP_ARCHSEL          (RP23XX_OTP_BASE + RP23XX_OTP_ARCHSEL_OFFSET)
#define RP23XX_OTP_ARCHSEL_STATUS   (RP23XX_OTP_BASE + RP23XX_OTP_ARCHSEL_STATUS_OFFSET)
#define RP23XX_OTP_BOOTDIS          (RP23XX_OTP_BASE + RP23XX_OTP_BOOTDIS_OFFSET)
#define RP23XX_OTP_INTR             (RP23XX_OTP_BASE + RP23XX_OTP_INTR_OFFSET)
#define RP23XX_OTP_INTE             (RP23XX_OTP_BASE + RP23XX_OTP_INTE_OFFSET)
#define RP23XX_OTP_INTF             (RP23XX_OTP_BASE + RP23XX_OTP_INTF_OFFSET)
#define RP23XX_OTP_INTS             (RP23XX_OTP_BASE + RP23XX_OTP_INTS_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_OTP_SW_LOCK_MASK                         0x0000000f
#define RP23XX_OTP_SW_LOCK_NSEC_MASK                    0x0000000c
#define RP23XX_OTP_SW_LOCK_SEC_MASK                     0x00000003
#define RP23XX_OTP_SBPI_INSTR_MASK                      0x7fffffff
#define RP23XX_OTP_SBPI_INSTR_EXEC                      (1 << 30)
#define RP23XX_OTP_SBPI_INSTR_IS_WR                     (1 << 29)
#define RP23XX_OTP_SBPI_INSTR_HAS_PAYLOAD               (1 << 28)
#define RP23XX_OTP_SBPI_INSTR_PAYLOAD_SIZE_M1_MASK      0x0f000000
#define RP23XX_OTP_SBPI_INSTR_TARGET_MASK               0x00ff0000
#define RP23XX_OTP_SBPI_INSTR_CMD_MASK                  0x0000ff00
#define RP23XX_OTP_SBPI_INSTR_SHORT_WDATA_MASK          0x000000ff
#define RP23XX_OTP_SBPI_STATUS_MASK                     0x00ff1111
#define RP23XX_OTP_SBPI_STATUS_MISO_MASK                0x00ff0000
#define RP23XX_OTP_SBPI_STATUS_FLAG                     (1 << 12)
#define RP23XX_OTP_SBPI_STATUS_INSTR_MISS               (1 << 8)
#define RP23XX_OTP_SBPI_STATUS_INSTR_DONE               (1 << 4)
#define RP23XX_OTP_SBPI_STATUS_RDATA_VLD                (1 << 0)
#define RP23XX_OTP_USR_MASK                             0x00000011
#define RP23XX_OTP_USR_PD                               (1 << 4)
#define RP23XX_OTP_USR_DCTRL                            (1 << 0)
#define RP23XX_OTP_DBG_MASK                             0x000010ff
#define RP23XX_OTP_DBG_CUSTOMER_RMA_FLAG                (1 << 12)
#define RP23XX_OTP_DBG_PSM_STATE_MASK                   0x000000f0
#define RP23XX_OTP_DBG_ROSC_UP                          (1 << 3)
#define RP23XX_OTP_DBG_ROSC_UP_SEEN                     (1 << 2)
#define RP23XX_OTP_DBG_BOOT_DONE                        (1 << 1)
#define RP23XX_OTP_DBG_PSM_DONE                         (1 << 0)
#define RP23XX_OTP_BIST_MASK                            0x7fff1fff
#define RP23XX_OTP_BIST_CNT_FAIL                        (1 << 30)
#define RP23XX_OTP_BIST_CNT_CLR                         (1 << 29)
#define RP23XX_OTP_BIST_CNT_ENA                         (1 << 28)
#define RP23XX_OTP_BIST_CNT_MAX_MASK                    0x0fff0000
#define RP23XX_OTP_BIST_CNT_MASK                        0x00001fff
#define RP23XX_OTP_CRITICAL_MASK                        0x0003007f
#define RP23XX_OTP_CRITICAL_RISCV_DISABLE               (1 << 17)
#define RP23XX_OTP_CRITICAL_ARM_DISABLE                 (1 << 16)
#define RP23XX_OTP_CRITICAL_GLITCH_DETECTOR_SENS_MASK   0x00000060
#define RP23XX_OTP_CRITICAL_GLITCH_DETECTOR_ENABLE      (1 << 4)
#define RP23XX_OTP_CRITICAL_DEFAULT_ARCHSEL             (1 << 3)
#define RP23XX_OTP_CRITICAL_DEBUG_DISABLE               (1 << 2)
#define RP23XX_OTP_CRITICAL_SECURE_DEBUG_DISABLE        (1 << 1)
#define RP23XX_OTP_CRITICAL_SECURE_BOOT_ENABLE          (1 << 0)
#define RP23XX_OTP_KEY_VALID_MASK                       0x000000ff
#define RP23XX_OTP_DEBUGEN_MASK                         0x0000010f
#define RP23XX_OTP_DEBUGEN_MISC                         (1 << 8)
#define RP23XX_OTP_DEBUGEN_PROC1_SECURE                 (1 << 3)
#define RP23XX_OTP_DEBUGEN_PROC1                        (1 << 2)
#define RP23XX_OTP_DEBUGEN_PROC0_SECURE                 (1 << 1)
#define RP23XX_OTP_DEBUGEN_PROC0                        (1 << 0)
#define RP23XX_OTP_DEBUGEN_LOCK_MASK                    0x0000010f
#define RP23XX_OTP_DEBUGEN_LOCK_MISC                    (1 << 8)
#define RP23XX_OTP_DEBUGEN_LOCK_PROC1_SECURE            (1 << 3)
#define RP23XX_OTP_DEBUGEN_LOCK_PROC1                   (1 << 2)
#define RP23XX_OTP_DEBUGEN_LOCK_PROC0_SECURE            (1 << 1)
#define RP23XX_OTP_DEBUGEN_LOCK_PROC0                   (1 << 0)
#define RP23XX_OTP_ARCHSEL_MASK                         0x00000003
#define RP23XX_OTP_ARCHSEL_CORE1                        (1 << 1)
#define RP23XX_OTP_ARCHSEL_CORE0                        (1 << 0)
#define RP23XX_OTP_ARCHSEL_STATUS_MASK                  0x00000003
#define RP23XX_OTP_ARCHSEL_STATUS_CORE1                 (1 << 1)
#define RP23XX_OTP_ARCHSEL_STATUS_CORE0                 (1 << 0)
#define RP23XX_OTP_BOOTDIS_MASK                         0x00000003
#define RP23XX_OTP_BOOTDIS_NEXT                         (1 << 1)
#define RP23XX_OTP_BOOTDIS_NOW                          (1 << 0)
#define RP23XX_OTP_INTR_MASK                            0x0000001f
#define RP23XX_OTP_INTR_APB_RD_NSEC_FAIL                (1 << 4)
#define RP23XX_OTP_INTR_APB_RD_SEC_FAIL                 (1 << 3)
#define RP23XX_OTP_INTR_APB_DCTRL_FAIL                  (1 << 2)
#define RP23XX_OTP_INTR_SBPI_WR_FAIL                    (1 << 1)
#define RP23XX_OTP_INTR_SBPI_FLAG_N                     (1 << 0)
#define RP23XX_OTP_INTE_MASK                            0x0000001f
#define RP23XX_OTP_INTE_APB_RD_NSEC_FAIL                (1 << 4)
#define RP23XX_OTP_INTE_APB_RD_SEC_FAIL                 (1 << 3)
#define RP23XX_OTP_INTE_APB_DCTRL_FAIL                  (1 << 2)
#define RP23XX_OTP_INTE_SBPI_WR_FAIL                    (1 << 1)
#define RP23XX_OTP_INTE_SBPI_FLAG_N                     (1 << 0)
#define RP23XX_OTP_INTF_MASK                            0x0000001f
#define RP23XX_OTP_INTF_APB_RD_NSEC_FAIL                (1 << 4)
#define RP23XX_OTP_INTF_APB_RD_SEC_FAIL                 (1 << 3)
#define RP23XX_OTP_INTF_APB_DCTRL_FAIL                  (1 << 2)
#define RP23XX_OTP_INTF_SBPI_WR_FAIL                    (1 << 1)
#define RP23XX_OTP_INTF_SBPI_FLAG_N                     (1 << 0)
#define RP23XX_OTP_INTS_MASK                            0x0000001f
#define RP23XX_OTP_INTS_APB_RD_NSEC_FAIL                (1 << 4)
#define RP23XX_OTP_INTS_APB_RD_SEC_FAIL                 (1 << 3)
#define RP23XX_OTP_INTS_APB_DCTRL_FAIL                  (1 << 2)
#define RP23XX_OTP_INTS_SBPI_WR_FAIL                    (1 << 1)
#define RP23XX_OTP_INTS_SBPI_FLAG_N                     (1 << 0)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_OTP_H */

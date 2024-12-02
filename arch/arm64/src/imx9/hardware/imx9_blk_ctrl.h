/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx9_blk_ctrl.h
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

#ifndef __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_BLK_CTRL_H
#define __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_BLK_CTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/imx9_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Wakeupmix block control Register offsets *********************************/

#define IMX9_WAKUPMIX_IPG_DEBUG_CM33_OFFSET   0x4   /* IPG DEBUG mask bit */
#define IMX9_WAKUPMIX_QCH_DIS_OFFSET          0x10  /* QCHANNEL DISABLE */
#define IMX9_WAKUPMIX_DEXSC_ERR_OFFSET        0x1c  /* DEXSC error response configuration */
#define IMX9_WAKUPMIX_MQS_SETTING_OFFSET      0x20  /* MQS Settings for MQS2 */
#define IMX9_WAKUPMIX_SAI_CLK_SEL_OFFSET      0x24  /* SAI2 and SAI3 MCLK1~3 CLK root mux settings */
#define IMX9_WAKUPMIX_GPR_OFFSET              0x28  /* ENET QOS control signals */
#define IMX9_WAKUPMIX_ENET_CLK_SEL_OFFSET     0x2c  /* ENET CLK direction selection */
#define IMX9_WAKUPMIX_VOLT_DETECT_OFFSET      0x34  /* Voltage detectors output */
#define IMX9_WAKUPMIX_I3C2_WAKEUP_OFFSET      0x38  /* I3C2 WAKEUPX CLR */
#define IMX9_WAKUPMIX_IPG_DEBUG_CA55C0_OFFSET 0x3c  /* IPG DEBUG mask bit for CA55 core0 */
#define IMX9_WAKUPMIX_IPG_DEBUG_CA55C1_OFFSET 0x40  /* IPG DEBUG mask bit for CA55 core1 */
#define IMX9_WAKUPMIX_AXI_ATTR_CFG_OFFSET     0x44  /* AXI CACHE OVERRITE BIT */
#define IMX9_WAKUPMIX_I3C2_SDA_IRQ_OFFSET     0x48  /* I3C SDA IRQ Control */

/* Wakeupmix block control registers ****************************************/

#define IMX9_WAKUPMIX_IPG_DEBUG_CM33   (IMX9_BLK_CTRL_WAKEUPMIX1_BASE + IMX9_WAKUPMIX_IPG_DEBUG_CM33_OFFSET)
#define IMX9_WAKUPMIX_QCH_DIS          (IMX9_BLK_CTRL_WAKEUPMIX1_BASE + IMX9_WAKUPMIX_QCH_DIS_OFFSET)
#define IMX9_WAKUPMIX_DEXSC_ERR        (IMX9_BLK_CTRL_WAKEUPMIX1_BASE + IMX9_WAKUPMIX_DEXSC_ERR_OFFSET)
#define IMX9_WAKUPMIX_MQS_SETTING      (IMX9_BLK_CTRL_WAKEUPMIX1_BASE + IMX9_WAKUPMIX_MQS_SETTING_OFFSET)
#define IMX9_WAKUPMIX_SAI_CLK_SEL      (IMX9_BLK_CTRL_WAKEUPMIX1_BASE + IMX9_WAKUPMIX_SAI_CLK_SEL_OFFSET)
#define IMX9_WAKUPMIX_GPR              (IMX9_BLK_CTRL_WAKEUPMIX1_BASE + IMX9_WAKUPMIX_GPR_OFFSET)
#define IMX9_WAKUPMIX_ENET_CLK_SEL     (IMX9_BLK_CTRL_WAKEUPMIX1_BASE + IMX9_WAKUPMIX_ENET_CLK_SEL_OFFSET)
#define IMX9_WAKUPMIX_VOLT_DETECT      (IMX9_BLK_CTRL_WAKEUPMIX1_BASE + IMX9_WAKUPMIX_VOLT_DETECT_OFFSET)
#define IMX9_WAKUPMIX_I3C2_WAKEUP      (IMX9_BLK_CTRL_WAKEUPMIX1_BASE + IMX9_WAKUPMIX_I3C2_WAKEUP_OFFSET)
#define IMX9_WAKUPMIX_IPG_DEBUG_CA55C0 (IMX9_BLK_CTRL_WAKEUPMIX1_BASE + IMX9_WAKUPMIX_IPG_DEBUG_CA55C0_OFFSET)
#define IMX9_WAKUPMIX_IPG_DEBUG_CA55C1 (IMX9_BLK_CTRL_WAKEUPMIX1_BASE + IMX9_WAKUPMIX_IPG_DEBUG_CA55C1_OFFSET)
#define IMX9_WAKUPMIX_AXI_ATTR_CFG     (IMX9_BLK_CTRL_WAKEUPMIX1_BASE + IMX9_WAKUPMIX_AXI_ATTR_CFG_OFFSET)
#define IMX9_WAKUPMIX_I3C2_SDA_IRQ     (IMX9_BLK_CTRL_WAKEUPMIX1_BASE + IMX9_WAKUPMIX_I3C2_SDA_IRQ_OFFSET)

/* Wakeupmix register bit definitions ***************************************/

#define WAKEUPMIX_ENET1_TX_CLK_SEL        ( 1 << 1 ) /* Direction of TX_CLK of ENET */
#define WAKEUPMIX_ENET_QOS_CLK_TX_CLK_SEL ( 1 << 0 ) /* Direction of TX_CLK of ENET */

#endif // __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_BLK_CTRL_H

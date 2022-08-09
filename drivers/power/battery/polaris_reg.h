/****************************************************************************
 * drivers/power/battery/polaris_reg.h
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

#ifndef __POLARIS_REG_H
#define __POLARIS_REG_H

/****************************************************************************

 * Pre-processor Definitions
 ****************************************************************************/

#ifndef BIT
#if defined(_ASMLANGUAGE)
#define BIT(n)  (1 << (n))
#else
/**
 * @brief Unsigned integer with bit position @p n set (signed in
 * assembly language).
 */
#define BIT(n)  (1U << (n))
#endif
#endif

/* WLC_RX_INTR_EN_REG states parsing */

#define WLC_RX_OTP_INT_MASK           BIT(0)  /* Byte0 */
#define WLC_RX_OCP_INT_MASK           BIT(1)  /* Byte0 */
#define WLC_RX_OVP_INT_MASK           BIT(2)  /* Byte0 */
#define WLC_RX_SCP_INT_MASK           BIT(4)  /* Byte0 */
#define WLC_RX_R_TX_INT_MASK          BIT(5)  /* Byte0 */
#define WLC_RX_OUTPUT_ON_INT_MASK     BIT(6)  /* Byte0 */
#define WLC_RX_OUTPUT_OFF_INT_MASK    BIT(7)  /* Byte0 */
#define WLC_RX_SS_TX_INT_MASK         BIT(10) /* Byte1 */
#define WLC_RX_UVP_INT_MASK           BIT(12) /* Byte1 */
#define WLC_RX_PP_DONE_INT_MASK       BIT(24) /* Byte3 */

#define WLC_CHIP_ID_REG               0x0000  /* Length: 2, default: 0x00 */
#define WLC_CHIP_REV_REG              0x0002  /* Length: 1, default: 0x00 */
#define WLC_CUST_ID_REG               0x0003  /* Length: 1, default: 0x00 */
#define WLC_ROM_ID_REG                0x0004  /* Length: 2, default: 0x00 */
#define WLC_NVM_PATCH_ID_REG          0x0006  /* Length: 2, default: 0x00 */
#define WLC_RAM_PATCH_ID_REG          0x0008  /* Length: 2, default: 0x00 */
#define WLC_CFG_ID_REG                0x000A  /* Length: 2, default: 0x00 */
#define WLC_PE_ID_REG                 0x000C  /* Length: 2, default: 0x00 */
#define WLC_OP_MODE_REG               0x000E  /* Length: 1, default: 0x00 */
#define WLC_DEVICE_ID_REG             0x0010  /* Length: 1, default: 0x00 */
#define WLC_SYS_CMD_REG               0x0020  /* Length: 2, default: 0x00 */
#define WLC_NVM_WR_PWD_REG            0x0022  /* Length: 1, default: 0x00 */
#define WLC_AUX_LEN_REG               0x0023  /* Length: 1, default: 0x00 */
#define WLC_AUX_ADDR_REG              0x0024  /* Length: 4, default: 0x00 */
#define WLC_SYS_ERR_LATCH_REG         0x002C  /* Length: 4, default: 0x00 */
#define WLC_RX_INTR_EN_REG            0x0080  /* Length: 4, default: 0x00 */
#define WLC_RX_INTR_CLR_REG           0x0084  /* Length: 4, default: 0x00 */
#define WLC_RX_INTR_LATCH_REG         0x0088  /* Length: 4, default: 0x00 */
#define WLC_RX_STAT_REG               0x008C  /* Length: 4, default: 0x00 */
#define WLC_RX_CMD_REG                0x0090  /* Length: 2, default: 0x00 */
#define WLC_RX_VRECT_REG              0x0092  /* Length: 2, default: 0x00 */
#define WLC_RX_VOUT_REG               0x0094  /* Length: 2, default: 0x00 */
#define WLC_RX_IOUT_REG               0x0096  /* Length: 2, default: 0x00 */
#define WLC_RX_CHIP_TEMP_REG          0x0098  /* Length: 2, default: 0x00 */
#define WLC_RX_OP_FREQ_REG            0x009A  /* Length: 2, default: 0x00 */
#define WLC_RX_NTC_REG                0x009C  /* Length: 2, default: 0x00 */
#define WLC_RX_ADC_IN1_REG            0x009E  /* Length: 2, default: 0x00 */
#define WLC_RX_ADC_IN2_REG            0x00A0  /* Length: 2, default: 0x00 */
#define WLC_RX_CTRL_ERR_REG           0x00A4  /* Length: 2, default: 0x00 */
#define WLC_RX_RCVD_PWR_REG           0x00A6  /* Length: 2, default: 0x00 */
#define WLC_RX_SIGNAL_STRENGTH_REG    0x00A8  /* Length: 1, default: 0x00 */
#define WLC_RX_VOUT_SET_REG           0x00B1  /* Length: 2, default: 0x00 */
#define WLC_RX_VRECT_ADJ_REG          0x00B4  /* Length: 1, default: 0x00 */
#define WLC_RX_ILIM_SET_REG           0x00B5  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_CUR_THRES1_REG     0x00B6  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_CUR_THRES2_REG     0x00B7  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_CUR_THRES3_REG     0x00B8  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_CUR_THRES4_REG     0x00B9  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_CUR_THRES5_REG     0x00BA  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_GAIN0_REG          0x00BB  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_GAIN1_REG          0x00BC  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_GAIN2_REG          0x00BD  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_GAIN3_REG          0x00BE  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_GAIN4_REG          0x00BF  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_GAIN5_REG          0x00C0  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_OFFSET0_REG        0x00C1  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_OFFSET1_REG        0x00C2  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_OFFSET2_REG        0x00C3  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_OFFSET3_REG        0x00C4  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_OFFSET4_REG        0x00C5  /* Length: 1, default: 0x00 */
#define WLC_RX_FOD_OFFSET5_REG        0x00C6  /* Length: 1, default: 0x00 */
#define WLC_RX_RSER_REG               0x00C7  /* Length: 1, default: 0x00 */
#define WLC_RX_LDO_DROP0_REG          0x00C8  /* Length: 1, default: 0x00 */
#define WLC_RX_LDO_DROP1_REG          0x00C9  /* Length: 1, default: 0x00 */
#define WLC_RX_LDO_DROP2_REG          0x00CA  /* Length: 1, default: 0x00 */
#define WLC_RX_LDO_DROP3_REG          0x00CB  /* Length: 1, default: 0x00 */
#define WLC_RX_LDO_CUR_THRES1_REG     0x00CC  /* Length: 1, default: 0x00 */
#define WLC_RX_LDO_CUR_THRES2_REG     0x00CD  /* Length: 1, default: 0x00 */
#define WLC_RX_LDO_CUR_THRES3_REG     0x00CE  /* Length: 1, default: 0x00 */
#define WLC_RX_EPT_MSG_REG            0x00CF  /* Length: 1, default: 0x00 */
#define WLC_RX_DTS_SEND_REG           0x00D8  /* Length: 4, default: 0x00 */
#define WLC_RX_DTS_RCVD_REG           0x00DC  /* Length: 4, default: 0x00 */
#define WLC_AUX_DATA_XX_REG           0x0180  /* Length: 1, default: 0x00 */
#define WLC_XM_PP_STATUS              0x00E0
#define WLC_XM_PP_ADAPTER_STYPE       0x00E1
#define WLC_XM_PP_TX_MODEL            0x00E2
#define WLC_LAST_CE                   0x013C /* Length: 2, default: 0x00 */

#define WLC_RX_VOL_BASE               500 /* mV */
#endif /* __POLARIS_REG_H */


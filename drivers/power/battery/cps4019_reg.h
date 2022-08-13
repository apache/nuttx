/****************************************************************************
 * drivers/power/battery/cps4019_reg.h
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

#ifndef __CPS4019_REG_H
#define __CPS4019_REG_H

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

struct cps4019_chip_info
{
  uint16_t chip_id;
  uint16_t fw_revision_l;
  uint16_t fw_revision_h;
};

#define NVM_SECTOR_SIZE_BYTES           256 //128
#define NVM_PATCH_START_SECTOR_INDEX    0
#define NVM_CFG_START_SECTOR_INDEX      126 //93
#define I2C_CHUNK_SIZE                  256 //128

#define AFTER_SYS_RESET_SLEEP_MS        50

/* FW register address */
#define FWREG_OP_MODE_ADDR              0x000E
#define FWREG_SYS_CMD_ADDR              0x0020
#define FWREG_NVM_SECTOR_INDEX_ADDR     0x0024
#define FWREG_AUX_DATA_00               0x0180
#define FWREG_NVM_PWD_ADDR              0x0022

/* SYSREG registers */
#define HWREG_HW_VER_ADDR               0x2001C002
#define HWREG_HW_SYS_RST_ADDR           0x2001F200

#define WRITE_READ_OPERATION            0x01
#define WRITE_OPERATION                 0x02

#define OPCODE_WRITE                    0xFA

#define MIN_WR_BYTE_LENGTH              5
#define MIN_W_BYTE_LENGTH               4

#define CMD_STR_LEN                     1024

#define OK                              ((int)0x00000000)
#define E_BUS_R                         ((int)0x80000001)
#define E_BUS_W                         ((int)0x80000002)
#define E_BUS_WR                        ((int)0x80000003)
#define E_UNEXPECTED_OP_MODE            ((int)0x80000004)
#define E_NVM_WRITE                     ((int)0x80000005)
#define E_INVALID_INPUT                 ((int)0x80000006)
#define E_MEMORY_ALLOC                  ((int)0x80000007)
#define E_UNEXPECTED_HW_REV             ((int)0x80000008)
#define E_TIMEOUT                       ((int)0x80000009)
#define E_NVM_DATA_MISMATCH             ((int)0x8000000A)

#define CPS_CHIPID_LOW_REG              0x0000

#define CPS_DRV_VERSION                 "1.0.0" /* driver version string format */

struct rx_int_state_s
{
  bool cps_rx_int_otp;
  bool cps_rx_int_ocp;
  bool cps_rx_int_ovp;
  bool cps_rx_int_scp;
  bool cps_rx_int_ss_tx;
  bool cps_rx_int_output_on;
  bool cps_rx_int_output_off;
  bool cps_rx_int_uvp;
  bool cps_rx_pp_done;
};

typedef enum
{
  SYS_MODE_NA = 0,
  SYS_MODE_BACKPOWER = 1,
  SYS_MODE_RX = 2,
}sys_mode_t;

#define CPS_HEALTH_TEMP_MAX  80
#define CPS_HEALTH_TEMP_MIN  10

#define CPS_HEALTH_UNKNOWN   -1
#define CPS_HEALTH_GOOD      0
#define CPS_HEALTH_OVERHEAT  1
#define CPS_HEALTH_OVERCOLD  2

/* HW REG */

#define HWREG_HW_ENABLE_ADDR         0xFFFFFF00
#define HWREG_HW_PASSWORD_ADDR       0x40040070
#define HWREG_HW_HALT_MCU_ADDR       0x40040004
#define HWREG_HW_SRAM_ADDR           0x20000000
#define HWREG_HW_DISABLE_TRIM_ADDR   0x40040010
#define HWREG_HW_EN_REGMAP_ADDR      0x40040004
#define HWREG_HW_FW_SIZE_ADDR        0x200005F4
#define HWREG_HW_CTL_ADDR            0x200005FC
#define HWREG_HW_BL_CHEK_RESULT_ADDR 0x200005F8
#define HWREG_HW_BUFFER0_ADDR        0x20000600
#define HWREG_HW_BUFFER1_ADDR        0x20000700
#define HWREG_HW_RESET_ADDR          0x40040004
#define HWREG_HW_PROGRAM_BUFF_SIZE   64

/* RX INT BIT */

#define CPS_RX_R_TX_INT_MASK          BIT(0)  /* Byte0 */
#define CPS_RX_UVP_INT_MASK           BIT(1)  /* Byte1 */
#define CPS_RX_OTP_INT_MASK           BIT(2)  /* Byte2 */
#define CPS_RX_OVP_INT_MASK           BIT(3)  /* Byte3 */
#define CPS_RX_OCP_INT_MASK           BIT(4)  /* Byte4 */
#define CPS_RX_ID_CFG_FINISH_INT_MASK BIT(5)  /* Byte5 */
#define CPS_RX_VOUT_STATE_INT_MASK    BIT(6)  /* Byte6 */

/* RX CMD BIT */

#define CPS_RX_CMD_SEND_DATA           BIT(0)
#define CPS_RX_CMD_LDO_TOGGLE          BIT(1)
#define CPS_RX_CMD_MCU_RESET           BIT(2)
#define CPS_RX_CMD_SEND_EPT            BIT(3)
#define CPS_RX_CMD_SEND_CSP            BIT(4)
#define CPS_RX_CMD_SEND_CLR_INT        BIT(5)

/* RX CMFET CTL BIT */

#define CPS_RX_CMFET_CMB2_EN           BIT(4)
#define CPS_RX_CMFET_CMB1_EN           BIT(5)
#define CPS_RX_CMFET_CMA2_EN           BIT(6)
#define CPS_RX_CMFET_CMA1_EN           BIT(7)

/* RX reg */

#define CPS_CHIP_ID_L_REG             0x0000  /* Length: 1, default: 0x19 */
#define CPS_CHIP_ID_H_REG             0x0001  /* Length: 1, default: 0x40 */
#define CPS_FW_ID_REG                 0x0002  /* Length: 4, default: 0x00 */
#define CPS_SYS_MODE_REG              0x0006  /* Length: 1, default: 0x00 */
#define CPS_INT_STATUS_REG            0x0007  /* Length: 2, default: 0x00 */
#define CPS_INT_REG                   0x0009  /* Length: 2, default: 0x00 */
#define CPS_INT_ENB_REG               0x000B  /* Length: 2, default: 0x00 */
#define CPS_INT_CLR_REG               0x000D  /* Length: 2, default: 0x00 */
#define CPS_CMD_REG                   0x000F  /* Length: 2, default: 0x00 */
#define CPS_CHG_STATUS_REG            0x0011  /* Length: 1, default: 0x00 */
#define CPS_EPT_REG                   0x0012  /* Length: 1, default: 0x00 */
#define CPS_VOUT_SET_REG              0x0013  /* Length: 2, default: 0x1388 */
#define CPS_VRECT_ADJ_REG             0x0015  /* Length: 1, default: 0x00 */
#define CPS_ILIMIT_SET_REG            0x0016  /* Length: 1, default: 0x16 */
#define CPS_ADC_VOUT_REG              0x0017  /* Length: 2, default: 0x00 */
#define CPS_ADC_VRECT_REG             0x0019  /* Length: 2, default: 0x00 */
#define CPS_ADC_IOUT_REG              0x001B  /* Length: 2, default: 0x00 */
#define CPS_ADC_DIE_TEMP_REG          0x001D  /* Length: 2, default: 0x00 */
#define CPS_RX_OP_FREQ_REG            0x001F  /* Length: 2, default: 0x00 */
#define CPS_RX_PING_FREQ_REG          0x0021  /* Length: 2, default: 0x00 */
#define CPS_RX_TARGET_VRECT_REG       0x0023  /* Length: 2, default: 0x00 */
#define CPS_RX_SEND_DATA_REG          0x0025  /* Length: 9, default: 0x00 */
#define CPS_RX_RCVD_DATA_REG          0X002E  /* Length: 9, default: 0x00 */
#define CPS_RX_CMFET_CTRL_REG         0x0037  /* Length: 1, default: 0x00 */
#define CPS_RX_OVP_REG                0x0038  /* Length: 1, default: 0x00 */
#define CPS_RX_PRE_CLAMP_REG          0x0039  /* Length: 2, default: 0x19 */
#define CPS_RX_VOUT_LOWSET_REG        0x003A  /* Length: 2, default: 0x0DAC */
#define CPS_RX_VOUT_HIGHSET_REG       0x003C  /* Length: 2, default: 0x2328 */
#define CPS_RX_VOUT_UVREF_REG         0x003E  /* Length: 2, default: 0x0DAC */
#define CPS_RX_VOUT_UVDEB_REG         0x0040  /* Length: 1, default: 0x00 */
#define CPS_RX_CEP_REG                0x0041  /* Length: 1, default: 0x00 */
#define CPS_RX_RPP_REG                0x0042  /* Length: 1, default: 0x00 */
#define CPS_RX_HEAVY_LOAD_SET_REG     0x0043  /* Length: 2, default: 0x82 */
#define CPS_RX_LIGHT_LOAD_SET_REG     0x0044  /* Length: 1, default: 0x64 */
#define CPS_RX_FOD_GAIN0_REG          0x0045  /* Length: 1, default: 0x38 */
#define CPS_RX_FOD_OFFSET0_REG        0x0046  /* Length: 1, default: 0x06 */
#define CPS_RX_FOD_GAIN1_REG          0x0047  /* Length: 1, default: 0x38 */
#define CPS_RX_FOD_OFFSET1_REG        0x0048  /* Length: 1, default: 0x06 */
#define CPS_RX_FOD_GAIN2_REG          0x0049  /* Length: 1, default: 0x38 */
#define CPS_RX_FOD_OFFSET2_REG        0x004A  /* Length: 1, default: 0x06 */
#define CPS_RX_FOD_GAIN3_REG          0x004B  /* Length: 1, default: 0x38 */
#define CPS_RX_FOD_OFFSET3_REG        0x004C  /* Length: 1, default: 0x06 */
#define CPS_RX_FOD_GAIN4_REG          0x004D  /* Length: 1, default: 0x38 */
#define CPS_RX_FOD_OFFSET4_REG        0x004E  /* Length: 1, default: 0x06 */
#define CPS_RX_FOD_GAIN5_REG          0x004F  /* Length: 1, default: 0x38 */
#define CPS_RX_FOD_OFFSET5_REG        0x0050  /* Length: 1, default: 0x06 */
#define CPS_RX_FOD_GAIN6_REG          0x0051  /* Length: 1, default: 0x38 */
#define CPS_RX_FOD_OFFSET6_REG        0x0052  /* Length: 1, default: 0x06 */
#define CPS_RX_FOD_GAIN7_REG          0x0053  /* Length: 1, default: 0x38 */
#define CPS_RX_FOD_OFFSET7_REG        0x0054  /* Length: 1, default: 0x06 */
#define CPS_RX_FOD_CUR_THRES0_REG     0x0056  /* Length: 1, default: 0x05 */
#define CPS_RX_FOD_CUR_THRES1_REG     0x0057  /* Length: 1, default: 0x0D */
#define CPS_RX_FOD_CUR_THRES2_REG     0x0058  /* Length: 1, default: 0x14 */
#define CPS_RX_FOD_CUR_THRES3_REG     0x0059  /* Length: 1, default: 0x1E */
#define CPS_RX_FOD_CUR_THRES4_REG     0x005A  /* Length: 1, default: 0x28 */
#define CPS_RX_FOD_CUR_THRES5_REG     0x005B  /* Length: 1, default: 0x32 */
#define CPS_RX_FOD_CUR_THRES6_REG     0x005C  /* Length: 1, default: 0x3C */
#define CPS_RX_PHM_PERIOD_REG         0x005D  /* Length: 1, default: 0x04 */
#define CPS_RX_MLDO_ON_THRESH0_REG    0x005E  /* Length: 1, default: 0x23 */
#define CPS_RX_MLDO_ON_THRESH1_REG    0x005F  /* Length: 1, default: 0x41 */
#define CPS_RX_LDO_DROP_MIN_REG       0x0060  /* Length: 1, default: 0x0C */
#define CPS_RX_LDO_DROP_MAX_REG       0x0061  /* Length: 1, default: 0x32 */
#define CPS_RX_LDO_DROP_MIN_CUR_REG   0x0062  /* Length: 1, default: 0x0D */
#define CPS_RX_LDO_DROP_MIAX_CUR_REG  0x0063  /* Length: 1, default: 0x0A */
#define CPS_NO_MOD_DUMMY_LOAD_REG     0x0064  /* Length: 1, default: 0x03 */
#define CPS_MOD_DUMMY_LOAD_REG        0x0065  /* Length: 1, default: 0x05 */
#define CPS_FUNC_EN_REG               0x0066  /* Length: 1, default: 0x00 */
#define CPS_ANA_STATUS_REG            0x0067  /* Length: 1, default: 0x00 */

#define CHARGER_DETECT_WORK_TIME  10000
#define DETECT_WORK_INIT_TIME     20000

#define DETECT_WORK_NO_EXIST       0
#define DETECT_WORK_EXIST          1
#define BATT_CHARGING_STAT_INIT   -1
#endif /* __CPS4019_REG_H */


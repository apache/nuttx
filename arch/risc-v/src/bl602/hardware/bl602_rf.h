/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_rf.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_RF_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_RF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_RF_REV_OFFSET                  0x000000  /* Silicon revision */
#define BL602_RF_FSM_CTRL_HW_OFFSET          0x000004  /* Digital Control */
#define BL602_RF_FSM_CTRL_SW_OFFSET          0x000008  /* rfsm status reg */
#define BL602_RFCTRL_HW_EN_OFFSET            0x00000c  /* Control logic switch */
#define BL602_RF_TEMP_COMP_OFFSET            0x000010  /* temp_comp */
#define BL602_RFCAL_STATUS_OFFSET            0x000014  /* rfcal_status */
#define BL602_RFCAL_STATUS2_OFFSET           0x000018  /* rfcal_status2 */
#define BL602_RFCAL_CTRLEN_OFFSET            0x00001c  /* Calibration mode register */
#define BL602_RFCAL_STATEEN_OFFSET           0x000020  /* rf calibration state enable in full cal list */
#define BL602_RF_SARADC_RESV_OFFSET          0x000024  /* SARADC Control Registers */
#define BL602_RF_BASE_CTRL1_OFFSET           0x000028  /* ZRF Control register 0 */
#define BL602_RF_BASE_CTRL2_OFFSET           0x00002c  /* ZRF Control register 0 */
#define BL602_RF_PUCR1_OFFSET                0x000030  /* pucr1 */
#define BL602_RF_PUCR1_HW_OFFSET             0x000034  /* read only from hardware logic */
#define BL602_RF_PUCR2_OFFSET                0x000038  /* pucr2 */
#define BL602_RF_PUCR2_HW_OFFSET             0x00003c  /* pucr2_hw */
#define BL602_RF_PPU_CTRL_HW_OFFSET          0x000040  /* ppu_ctrl_hw */
#define BL602_RF_TRX_GAIN1_OFFSET            0x000048  /* gain control1 */
#define BL602_RF_PUD_CTRL_HW_OFFSET          0x000044  /* pud_ctrl_hw */
#define BL602_RF_TEN_DC_OFFSET               0x000050  /* dc test register */
#define BL602_RF_TRX_GAIN_HW_OFFSET          0x00004c  /* trx gain hardware readback */
#define BL602_RF_TEN_AC_OFFSET               0x000058  /* ac test register */
#define BL602_RF_TEN_DIG_OFFSET              0x000054  /* digital test register */
#define BL602_RF_CIP_OFFSET                  0x000060  /* RX normal bias mode registers */
#define BL602_RF_PMIP_MV2AON_OFFSET          0x00005c  /* pmip_mv2aon */
#define BL602_RF_PA2_OFFSET                  0x000068  /* RX normal bias mode registers */
#define BL602_RF_PA1_OFFSET                  0x000064  /* pa1 */
#define BL602_RF_TBB_OFFSET                  0x000070  /* tbb */
#define BL602_RF_TMX_OFFSET                  0x00006c  /* tmx */
#define BL602_RF_RMXGM_OFFSET                0x000078  /* rmxgm */
#define BL602_RF_LNA_OFFSET                  0x000074  /* lna */
#define BL602_RF_RBB2_OFFSET                 0x000080  /* rbb2 */
#define BL602_RF_RBB1_OFFSET                 0x00007c  /* rbb1 */
#define BL602_RF_RBB4_OFFSET                 0x000088  /* rbb4 */
#define BL602_RF_RBB3_OFFSET                 0x000084  /* rbb3 */
#define BL602_RF_ADDA2_OFFSET                0x000090  /* adda2 */
#define BL602_RF_ADDA1_OFFSET                0x00008c  /* adda1 */
#define BL602_RF_VCO2_OFFSET                 0x0000a4  /* vco2 */
#define BL602_RF_VCO1_OFFSET                 0x0000a0  /* vco1 */
#define BL602_RF_VCO4_OFFSET                 0x0000ac  /* vco4 */
#define BL602_RF_VCO3_OFFSET                 0x0000a8  /* vco3 */
#define BL602_RF_LO_OFFSET                   0x0000b4  /* lo */
#define BL602_RF_PFDCP_OFFSET                0x0000b0  /* pfdcp */
#define BL602_RF_LODIST_OFFSET               0x0000bc  /* lodist */
#define BL602_RF_FBDV_OFFSET                 0x0000b8  /* fbdv */
#define BL602_RF_SDM2_OFFSET                 0x0000c4  /* sdm2 */
#define BL602_RF_SDM1_OFFSET                 0x0000c0  /* sdm1 */
#define BL602_RF_RESV_REG_0_OFFSET           0x0000ec  /* rf_resv_reg_0 */
#define BL602_RF_SDM3_OFFSET                 0x0000c8  /* sdm3 */
#define BL602_RF_RESV_REG_2_OFFSET           0x0000f4  /* rf_resv_reg_2 */
#define BL602_RF_RESV_REG_1_OFFSET           0x0000f0  /* rf_resv_reg_1 */
#define BL602_RRF_GAIN_INDEX2_OFFSET         0x0000fc  /* rrf_gain_index2 */
#define BL602_RRF_GAIN_INDEX1_OFFSET         0x0000f8  /* rrf_gain_index1 */
#define BL602_RF_LNA_CTRL_HW_MUX_OFFSET      0x000100  /* lna_ctrl_hw_mux */
#define BL602_RF_RBB_GAIN_INDEX1_OFFSET      0x000104  /* rbb_gain_index1 */
#define BL602_RF_RBB_GAIN_INDEX2_OFFSET      0x000108  /* rbb_gain_index2 */
#define BL602_RF_RBB_GAIN_INDEX3_OFFSET      0x00010c  /* rbb_gain_index3 */
#define BL602_RF_RBB_GAIN_INDEX4_OFFSET      0x000110  /* rbb_gain_index4 */
#define BL602_RF_RBB_GAIN_INDEX5_OFFSET      0x000114  /* rbb_gain_index5 */
#define BL602_RF_TBB_GAIN_INDEX1_OFFSET      0x000118  /* tbb_gain_index1 */
#define BL602_RF_TBB_GAIN_INDEX2_OFFSET      0x00011c  /* tbb_gain_index2 */
#define BL602_RF_TBB_GAIN_INDEX3_OFFSET      0x000120  /* tbb_gain_index3 */
#define BL602_RF_TBB_GAIN_INDEX4_OFFSET      0x000124  /* tbb_gain_index4 */
#define BL602_RF_PA_REG_CTRL_HW1_OFFSET      0x000128  /* pa_reg_ctrl_hw1 */
#define BL602_RF_PA_REG_CTRL_HW2_OFFSET      0x00012c  /* pa_reg_ctrl_hw2 */
#define BL602_RF_PA_REG_WIFI_CTRL_HW_OFFSET  0x000130  /* pa_reg_wifi_ctrl_hw */
#define BL602_RF_ADDA_REG_CTRL_HW_OFFSET     0x000134  /* adda_reg_ctrl_hw */
#define BL602_RF_LO_REG_CTRL_HW1_OFFSET      0x000138  /* lo_reg_ctrl_hw1 */
#define BL602_RF_LO_CAL_CTRL_HW1_OFFSET      0x00013c  /* lo_cal_ctrl_hw1 */
#define BL602_RF_LO_CAL_CTRL_HW2_OFFSET      0x000140  /* lo_cal_ctrl_hw2 */
#define BL602_RF_LO_CAL_CTRL_HW3_OFFSET      0x000144  /* lo_cal_ctrl_hw3 */
#define BL602_RF_LO_CAL_CTRL_HW4_OFFSET      0x000148  /* lo_cal_ctrl_hw4 */
#define BL602_RF_LO_CAL_CTRL_HW5_OFFSET      0x00014c  /* lo_cal_ctrl_hw5 */
#define BL602_RF_LO_CAL_CTRL_HW6_OFFSET      0x000150  /* lo_cal_ctrl_hw6 */
#define BL602_RF_LO_CAL_CTRL_HW7_OFFSET      0x000154  /* lo_cal_ctrl_hw7 */
#define BL602_RF_LO_CAL_CTRL_HW8_OFFSET      0x000158  /* lo_cal_ctrl_hw8 */
#define BL602_RF_LO_CAL_CTRL_HW9_OFFSET      0x00015c  /* lo_cal_ctrl_hw9 */
#define BL602_RF_LO_CAL_CTRL_HW10_OFFSET     0x000160  /* lo_cal_ctrl_hw10 */
#define BL602_RF_LO_CAL_CTRL_HW11_OFFSET     0x000164  /* lo_cal_ctrl_hw11 */
#define BL602_RF_ROSDAC_CTRL_HW1_OFFSET      0x000168  /* rosdac_ctrl_hw1 */
#define BL602_RF_ROSDAC_CTRL_HW2_OFFSET      0x00016c  /* rosdac_ctrl_hw2 */
#define BL602_RF_RXIQ_CTRL_HW1_OFFSET        0x000170  /* rxiq_ctrl_hw1 */
#define BL602_RF_RXIQ_CTRL_HW2_OFFSET        0x000174  /* rxiq_ctrl_hw2 */
#define BL602_RF_RXIQ_CTRL_HW3_OFFSET        0x000178  /* rxiq_ctrl_hw3 */
#define BL602_RF_RXIQ_CTRL_HW4_OFFSET        0x00017c  /* rxiq_ctrl_hw4 */
#define BL602_RF_TOSDAC_CTRL_HW1_OFFSET      0x000180  /* tosdac_ctrl_hw1 */
#define BL602_RF_TOSDAC_CTRL_HW2_OFFSET      0x000184  /* tosdac_ctrl_hw2 */
#define BL602_RF_TOSDAC_CTRL_HW3_OFFSET      0x000188  /* tosdac_ctrl_hw3 */
#define BL602_RF_TOSDAC_CTRL_HW4_OFFSET      0x00018c  /* tosdac_ctrl_hw4 */
#define BL602_RF_TX_IQ_GAIN_HW0_OFFSET       0x000190  /* tx_iq_gain_hw0 */
#define BL602_RF_TX_IQ_GAIN_HW1_OFFSET       0x000194  /* tx_iq_gain_hw1 */
#define BL602_RF_TX_IQ_GAIN_HW2_OFFSET       0x000198  /* tx_iq_gain_hw2 */
#define BL602_RF_TX_IQ_GAIN_HW3_OFFSET       0x00019c  /* tx_iq_gain_hw3 */
#define BL602_RF_TX_IQ_GAIN_HW4_OFFSET       0x0001a0  /* tx_iq_gain_hw4 */
#define BL602_RF_TX_IQ_GAIN_HW5_OFFSET       0x0001a4  /* tx_iq_gain_hw5 */
#define BL602_RF_TX_IQ_GAIN_HW6_OFFSET       0x0001a8  /* tx_iq_gain_hw6 */
#define BL602_RF_TX_IQ_GAIN_HW7_OFFSET       0x0001ac  /* tx_iq_gain_hw7 */
#define BL602_RF_LO_SDM_CTRL_HW1_OFFSET      0x0001b0  /* lo_sdm_ctrl_hw1 */
#define BL602_RF_LO_SDM_CTRL_HW2_OFFSET      0x0001b4  /* lo_sdm_ctrl_hw2 */
#define BL602_RF_LO_SDM_CTRL_HW3_OFFSET      0x0001b8  /* lo_sdm_ctrl_hw3 */
#define BL602_RF_LO_SDM_CTRL_HW4_OFFSET      0x0001bc  /* lo_sdm_ctrl_hw4 */
#define BL602_RF_LO_SDM_CTRL_HW5_OFFSET      0x0001c0  /* lo_sdm_ctrl_hw5 */
#define BL602_RF_LO_SDM_CTRL_HW6_OFFSET      0x0001c4  /* lo_sdm_ctrl_hw6 */
#define BL602_RF_LO_SDM_CTRL_HW7_OFFSET      0x0001c8  /* lo_sdm_ctrl_hw7 */
#define BL602_RF_LO_SDM_CTRL_HW8_OFFSET      0x0001cc  /* lo_sdm_ctrl_hw8 */
#define BL602_RF_RBB_BW_CTRL_HW_OFFSET       0x0001d0  /* rbb_bw_ctrl_hw */
#define BL602_RF_SINGEN_CTRL0_OFFSET         0x00020c  /* singen_ctrl0 */
#define BL602_RF_SINGEN_CTRL1_OFFSET         0x000210  /* singen_ctrl1 */
#define BL602_RF_SINGEN_CTRL2_OFFSET         0x000214  /* singen_ctrl2 */
#define BL602_RF_SINGEN_CTRL3_OFFSET         0x000218  /* singen_ctrl3 */
#define BL602_RF_SINGEN_CTRL4_OFFSET         0x00021c  /* singen_ctrl4 */
#define BL602_RF_RFIF_DFE_CTRL0_OFFSET       0x000220  /* rfif_dfe_ctrl0 */
#define BL602_RF_RFIF_TEST_READ_OFFSET       0x000224  /* rfif_test_read */
#define BL602_RF_RFIF_DIG_CTRL_OFFSET        0x000228  /* rfif_dig_ctrl */
#define BL602_RF_DATA_TEMP_0_OFFSET          0x00022c  /* rf_data_temp_0 */
#define BL602_RF_DATA_TEMP_1_OFFSET          0x000230  /* rf_data_temp_1 */
#define BL602_RF_DATA_TEMP_2_OFFSET          0x000234  /* rf_data_temp_2 */
#define BL602_RF_DATA_TEMP_3_OFFSET          0x000238  /* rf_data_temp_3 */
#define BL602_RF_SRAM_CTRL0_OFFSET           0x00023c  /* rf_sram_ctrl0 */
#define BL602_RF_SRAM_CTRL1_OFFSET           0x000240  /* rf_sram_ctrl1 */
#define BL602_RF_SRAM_CTRL2_OFFSET           0x000244  /* rf_sram_ctrl2 */
#define BL602_RF_SRAM_CTRL3_OFFSET           0x000248  /* rf_sram_ctrl3 */
#define BL602_RF_SRAM_CTRL4_OFFSET           0x00024c  /* rf_sram_ctrl4 */
#define BL602_RF_SRAM_CTRL5_OFFSET           0x000250  /* rf_sram_ctrl5 */
#define BL602_RF_SRAM_CTRL6_OFFSET           0x000254  /* rf_sram_ctrl6 */
#define BL602_RF_ICAL_CTRL0_OFFSET           0x000258  /* rf_ical_ctrl0 */
#define BL602_RF_ICAL_CTRL1_OFFSET           0x00025c  /* rf_ical_ctrl1 */
#define BL602_RF_ICAL_CTRL2_OFFSET           0x000260  /* rf_ical_ctrl2 */
#define BL602_RF_FSM_CTRL0_OFFSET            0x000264  /* rf_fsm_ctrl0 */
#define BL602_RF_FSM_CTRL1_OFFSET            0x000268  /* rf_fsm_ctrl1 */
#define BL602_RF_FSM_CTRL2_OFFSET            0x00026c  /* rf_fsm_ctrl2 */
#define BL602_RF_PKDET_CTRL0_OFFSET          0x000270  /* rf_pkdet_ctrl0 */
#define BL602_RF_DFE_CTRL_0_OFFSET           0x000600  /* dfe_ctrl_0 */
#define BL602_RF_DFE_CTRL_1_OFFSET           0x000604  /* dfe_ctrl_1 */
#define BL602_RF_DFE_CTRL_2_OFFSET           0x000608  /* dfe_ctrl_2 */
#define BL602_RF_DFE_CTRL_3_OFFSET           0x00060c  /* dfe_ctrl_3 */
#define BL602_RF_DFE_CTRL_4_OFFSET           0x000610  /* dfe_ctrl_4 */
#define BL602_RF_DFE_CTRL_5_OFFSET           0x000614  /* dfe_ctrl_5 */
#define BL602_RF_DFE_CTRL_6_OFFSET           0x000618  /* dfe_ctrl_6 */
#define BL602_RF_DFE_CTRL_7_OFFSET           0x00061c  /* dfe_ctrl_7 */
#define BL602_RF_DFE_CTRL_8_OFFSET           0x000620  /* dfe_ctrl_8 */
#define BL602_RF_DFE_CTRL_9_OFFSET           0x000624  /* dfe_ctrl_9 */
#define BL602_RF_DFE_CTRL_10_OFFSET          0x000628  /* dfe_ctrl_10 */
#define BL602_RF_DFE_CTRL_11_OFFSET          0x00062c  /* dfe_ctrl_11 */
#define BL602_RF_DFE_CTRL_12_OFFSET          0x000630  /* dfe_ctrl_12 */
#define BL602_RF_DFE_CTRL_13_OFFSET          0x000634  /* dfe_ctrl_13 */
#define BL602_RF_DFE_CTRL_14_OFFSET          0x000638  /* dfe_ctrl_14 */
#define BL602_RF_DFE_CTRL_15_OFFSET          0x00063c  /* dfe_ctrl_15 */
#define BL602_RF_DFE_CTRL_16_OFFSET          0x000640  /* dfe_ctrl_16 */
#define BL602_RF_DFE_CTRL_17_OFFSET          0x000644  /* dfe_ctrl_17 */
#define BL602_RF_DFE_CTRL_18_OFFSET          0x000648  /* dfe_ctrl_18 */

/* Register definitions *****************************************************/

#define BL602_RF_REV                  (BL602_RF_BASE + BL602_RF_REV_OFFSET)
#define BL602_RF_FSM_CTRL_HW          (BL602_RF_BASE + BL602_RF_FSM_CTRL_HW_OFFSET)
#define BL602_RF_FSM_CTRL_SW          (BL602_RF_BASE + BL602_RF_FSM_CTRL_SW_OFFSET)
#define BL602_RFCTRL_HW_EN            (BL602_RF_BASE + BL602_RFCTRL_HW_EN_OFFSET)
#define BL602_RF_TEMP_COMP            (BL602_RF_BASE + BL602_RF_TEMP_COMP_OFFSET)
#define BL602_RFCAL_STATUS            (BL602_RF_BASE + BL602_RFCAL_STATUS_OFFSET)
#define BL602_RFCAL_STATUS2           (BL602_RF_BASE + BL602_RFCAL_STATUS2_OFFSET)
#define BL602_RFCAL_CTRLEN            (BL602_RF_BASE + BL602_RFCAL_CTRLEN_OFFSET)
#define BL602_RFCAL_STATEEN           (BL602_RF_BASE + BL602_RFCAL_STATEEN_OFFSET)
#define BL602_RF_SARADC_RESV          (BL602_RF_BASE + BL602_RF_SARADC_RESV_OFFSET)
#define BL602_RF_BASE_CTRL1           (BL602_RF_BASE + BL602_RF_BASE_CTRL1_OFFSET)
#define BL602_RF_BASE_CTRL2           (BL602_RF_BASE + BL602_RF_BASE_CTRL2_OFFSET)
#define BL602_RF_PUCR1                (BL602_RF_BASE + BL602_RF_PUCR1_OFFSET)
#define BL602_RF_PUCR1_HW             (BL602_RF_BASE + BL602_RF_PUCR1_HW_OFFSET)
#define BL602_RF_PUCR2                (BL602_RF_BASE + BL602_RF_PUCR2_OFFSET)
#define BL602_RF_PUCR2_HW             (BL602_RF_BASE + BL602_RF_PUCR2_HW_OFFSET)
#define BL602_RF_PPU_CTRL_HW          (BL602_RF_BASE + BL602_RF_PPU_CTRL_HW_OFFSET)
#define BL602_RF_PUD_CTRL_HW          (BL602_RF_BASE + BL602_RF_PUD_CTRL_HW_OFFSET)
#define BL602_RF_TRX_GAIN1            (BL602_RF_BASE + BL602_RF_TRX_GAIN1_OFFSET)
#define BL602_RF_TRX_GAIN_HW          (BL602_RF_BASE + BL602_RF_TRX_GAIN_HW_OFFSET)
#define BL602_RF_TEN_DC               (BL602_RF_BASE + BL602_RF_TEN_DC_OFFSET)
#define BL602_RF_TEN_DIG              (BL602_RF_BASE + BL602_RF_TEN_DIG_OFFSET)
#define BL602_RF_TEN_AC               (BL602_RF_BASE + BL602_RF_TEN_AC_OFFSET)
#define BL602_RF_PMIP_MV2AON          (BL602_RF_BASE + BL602_RF_PMIP_MV2AON_OFFSET)
#define BL602_RF_CIP                  (BL602_RF_BASE + BL602_RF_CIP_OFFSET)
#define BL602_RF_PA1                  (BL602_RF_BASE + BL602_RF_PA1_OFFSET)
#define BL602_RF_PA2                  (BL602_RF_BASE + BL602_RF_PA2_OFFSET)
#define BL602_RF_TMX                  (BL602_RF_BASE + BL602_RF_TMX_OFFSET)
#define BL602_RF_TBB                  (BL602_RF_BASE + BL602_RF_TBB_OFFSET)
#define BL602_RF_LNA                  (BL602_RF_BASE + BL602_RF_LNA_OFFSET)
#define BL602_RF_RMXGM                (BL602_RF_BASE + BL602_RF_RMXGM_OFFSET)
#define BL602_RF_RBB1                 (BL602_RF_BASE + BL602_RF_RBB1_OFFSET)
#define BL602_RF_RBB2                 (BL602_RF_BASE + BL602_RF_RBB2_OFFSET)
#define BL602_RF_RBB3                 (BL602_RF_BASE + BL602_RF_RBB3_OFFSET)
#define BL602_RF_RBB4                 (BL602_RF_BASE + BL602_RF_RBB4_OFFSET)
#define BL602_RF_ADDA1                (BL602_RF_BASE + BL602_RF_ADDA1_OFFSET)
#define BL602_RF_ADDA2                (BL602_RF_BASE + BL602_RF_ADDA2_OFFSET)
#define BL602_RF_VCO1                 (BL602_RF_BASE + BL602_RF_VCO1_OFFSET)
#define BL602_RF_VCO2                 (BL602_RF_BASE + BL602_RF_VCO2_OFFSET)
#define BL602_RF_VCO3                 (BL602_RF_BASE + BL602_RF_VCO3_OFFSET)
#define BL602_RF_VCO4                 (BL602_RF_BASE + BL602_RF_VCO4_OFFSET)
#define BL602_RF_PFDCP                (BL602_RF_BASE + BL602_RF_PFDCP_OFFSET)
#define BL602_RF_LO                   (BL602_RF_BASE + BL602_RF_LO_OFFSET)
#define BL602_RF_FBDV                 (BL602_RF_BASE + BL602_RF_FBDV_OFFSET)
#define BL602_RF_LODIST               (BL602_RF_BASE + BL602_RF_LODIST_OFFSET)
#define BL602_RF_SDM1                 (BL602_RF_BASE + BL602_RF_SDM1_OFFSET)
#define BL602_RF_SDM2                 (BL602_RF_BASE + BL602_RF_SDM2_OFFSET)
#define BL602_RF_SDM3                 (BL602_RF_BASE + BL602_RF_SDM3_OFFSET)
#define BL602_RF_RESV_REG_0           (BL602_RF_BASE + BL602_RF_RESV_REG_0_OFFSET)
#define BL602_RF_RESV_REG_1           (BL602_RF_BASE + BL602_RF_RESV_REG_1_OFFSET)
#define BL602_RF_RESV_REG_2           (BL602_RF_BASE + BL602_RF_RESV_REG_2_OFFSET)
#define BL602_RRF_GAIN_INDEX1         (BL602_RF_BASE + BL602_RRF_GAIN_INDEX1_OFFSET)
#define BL602_RRF_GAIN_INDEX2         (BL602_RF_BASE + BL602_RRF_GAIN_INDEX2_OFFSET)
#define BL602_RF_LNA_CTRL_HW_MUX      (BL602_RF_BASE + BL602_RF_LNA_CTRL_HW_MUX_OFFSET)
#define BL602_RF_RBB_GAIN_INDEX1      (BL602_RF_BASE + BL602_RF_RBB_GAIN_INDEX1_OFFSET)
#define BL602_RF_RBB_GAIN_INDEX2      (BL602_RF_BASE + BL602_RF_RBB_GAIN_INDEX2_OFFSET)
#define BL602_RF_RBB_GAIN_INDEX3      (BL602_RF_BASE + BL602_RF_RBB_GAIN_INDEX3_OFFSET)
#define BL602_RF_RBB_GAIN_INDEX4      (BL602_RF_BASE + BL602_RF_RBB_GAIN_INDEX4_OFFSET)
#define BL602_RF_RBB_GAIN_INDEX5      (BL602_RF_BASE + BL602_RF_RBB_GAIN_INDEX5_OFFSET)
#define BL602_RF_TBB_GAIN_INDEX1      (BL602_RF_BASE + BL602_RF_TBB_GAIN_INDEX1_OFFSET)
#define BL602_RF_TBB_GAIN_INDEX2      (BL602_RF_BASE + BL602_RF_TBB_GAIN_INDEX2_OFFSET)
#define BL602_RF_TBB_GAIN_INDEX3      (BL602_RF_BASE + BL602_RF_TBB_GAIN_INDEX3_OFFSET)
#define BL602_RF_TBB_GAIN_INDEX4      (BL602_RF_BASE + BL602_RF_TBB_GAIN_INDEX4_OFFSET)
#define BL602_RF_PA_REG_CTRL_HW1      (BL602_RF_BASE + BL602_RF_PA_REG_CTRL_HW1_OFFSET)
#define BL602_RF_PA_REG_CTRL_HW2      (BL602_RF_BASE + BL602_RF_PA_REG_CTRL_HW2_OFFSET)
#define BL602_RF_PA_REG_WIFI_CTRL_HW  (BL602_RF_BASE + BL602_RF_PA_REG_WIFI_CTRL_HW_OFFSET)
#define BL602_RF_ADDA_REG_CTRL_HW     (BL602_RF_BASE + BL602_RF_ADDA_REG_CTRL_HW_OFFSET)
#define BL602_RF_LO_REG_CTRL_HW1      (BL602_RF_BASE + BL602_RF_LO_REG_CTRL_HW1_OFFSET)
#define BL602_RF_LO_CAL_CTRL_HW1      (BL602_RF_BASE + BL602_RF_LO_CAL_CTRL_HW1_OFFSET)
#define BL602_RF_LO_CAL_CTRL_HW2      (BL602_RF_BASE + BL602_RF_LO_CAL_CTRL_HW2_OFFSET)
#define BL602_RF_LO_CAL_CTRL_HW3      (BL602_RF_BASE + BL602_RF_LO_CAL_CTRL_HW3_OFFSET)
#define BL602_RF_LO_CAL_CTRL_HW4      (BL602_RF_BASE + BL602_RF_LO_CAL_CTRL_HW4_OFFSET)
#define BL602_RF_LO_CAL_CTRL_HW5      (BL602_RF_BASE + BL602_RF_LO_CAL_CTRL_HW5_OFFSET)
#define BL602_RF_LO_CAL_CTRL_HW6      (BL602_RF_BASE + BL602_RF_LO_CAL_CTRL_HW6_OFFSET)
#define BL602_RF_LO_CAL_CTRL_HW7      (BL602_RF_BASE + BL602_RF_LO_CAL_CTRL_HW7_OFFSET)
#define BL602_RF_LO_CAL_CTRL_HW8      (BL602_RF_BASE + BL602_RF_LO_CAL_CTRL_HW8_OFFSET)
#define BL602_RF_LO_CAL_CTRL_HW9      (BL602_RF_BASE + BL602_RF_LO_CAL_CTRL_HW9_OFFSET)
#define BL602_RF_LO_CAL_CTRL_HW10     (BL602_RF_BASE + BL602_RF_LO_CAL_CTRL_HW10_OFFSET)
#define BL602_RF_LO_CAL_CTRL_HW11     (BL602_RF_BASE + BL602_RF_LO_CAL_CTRL_HW11_OFFSET)
#define BL602_RF_ROSDAC_CTRL_HW1      (BL602_RF_BASE + BL602_RF_ROSDAC_CTRL_HW1_OFFSET)
#define BL602_RF_ROSDAC_CTRL_HW2      (BL602_RF_BASE + BL602_RF_ROSDAC_CTRL_HW2_OFFSET)
#define BL602_RF_RXIQ_CTRL_HW1        (BL602_RF_BASE + BL602_RF_RXIQ_CTRL_HW1_OFFSET)
#define BL602_RF_RXIQ_CTRL_HW2        (BL602_RF_BASE + BL602_RF_RXIQ_CTRL_HW2_OFFSET)
#define BL602_RF_RXIQ_CTRL_HW3        (BL602_RF_BASE + BL602_RF_RXIQ_CTRL_HW3_OFFSET)
#define BL602_RF_RXIQ_CTRL_HW4        (BL602_RF_BASE + BL602_RF_RXIQ_CTRL_HW4_OFFSET)
#define BL602_RF_TOSDAC_CTRL_HW1      (BL602_RF_BASE + BL602_RF_TOSDAC_CTRL_HW1_OFFSET)
#define BL602_RF_TOSDAC_CTRL_HW2      (BL602_RF_BASE + BL602_RF_TOSDAC_CTRL_HW2_OFFSET)
#define BL602_RF_TOSDAC_CTRL_HW3      (BL602_RF_BASE + BL602_RF_TOSDAC_CTRL_HW3_OFFSET)
#define BL602_RF_TOSDAC_CTRL_HW4      (BL602_RF_BASE + BL602_RF_TOSDAC_CTRL_HW4_OFFSET)
#define BL602_RF_TX_IQ_GAIN_HW0       (BL602_RF_BASE + BL602_RF_TX_IQ_GAIN_HW0_OFFSET)
#define BL602_RF_TX_IQ_GAIN_HW1       (BL602_RF_BASE + BL602_RF_TX_IQ_GAIN_HW1_OFFSET)
#define BL602_RF_TX_IQ_GAIN_HW2       (BL602_RF_BASE + BL602_RF_TX_IQ_GAIN_HW2_OFFSET)
#define BL602_RF_TX_IQ_GAIN_HW3       (BL602_RF_BASE + BL602_RF_TX_IQ_GAIN_HW3_OFFSET)
#define BL602_RF_TX_IQ_GAIN_HW4       (BL602_RF_BASE + BL602_RF_TX_IQ_GAIN_HW4_OFFSET)
#define BL602_RF_TX_IQ_GAIN_HW5       (BL602_RF_BASE + BL602_RF_TX_IQ_GAIN_HW5_OFFSET)
#define BL602_RF_TX_IQ_GAIN_HW6       (BL602_RF_BASE + BL602_RF_TX_IQ_GAIN_HW6_OFFSET)
#define BL602_RF_TX_IQ_GAIN_HW7       (BL602_RF_BASE + BL602_RF_TX_IQ_GAIN_HW7_OFFSET)
#define BL602_RF_LO_SDM_CTRL_HW1      (BL602_RF_BASE + BL602_RF_LO_SDM_CTRL_HW1_OFFSET)
#define BL602_RF_LO_SDM_CTRL_HW2      (BL602_RF_BASE + BL602_RF_LO_SDM_CTRL_HW2_OFFSET)
#define BL602_RF_LO_SDM_CTRL_HW3      (BL602_RF_BASE + BL602_RF_LO_SDM_CTRL_HW3_OFFSET)
#define BL602_RF_LO_SDM_CTRL_HW4      (BL602_RF_BASE + BL602_RF_LO_SDM_CTRL_HW4_OFFSET)
#define BL602_RF_LO_SDM_CTRL_HW5      (BL602_RF_BASE + BL602_RF_LO_SDM_CTRL_HW5_OFFSET)
#define BL602_RF_LO_SDM_CTRL_HW6      (BL602_RF_BASE + BL602_RF_LO_SDM_CTRL_HW6_OFFSET)
#define BL602_RF_LO_SDM_CTRL_HW7      (BL602_RF_BASE + BL602_RF_LO_SDM_CTRL_HW7_OFFSET)
#define BL602_RF_LO_SDM_CTRL_HW8      (BL602_RF_BASE + BL602_RF_LO_SDM_CTRL_HW8_OFFSET)
#define BL602_RF_RBB_BW_CTRL_HW       (BL602_RF_BASE + BL602_RF_RBB_BW_CTRL_HW_OFFSET)
#define BL602_RF_SINGEN_CTRL0         (BL602_RF_BASE + BL602_RF_SINGEN_CTRL0_OFFSET)
#define BL602_RF_SINGEN_CTRL1         (BL602_RF_BASE + BL602_RF_SINGEN_CTRL1_OFFSET)
#define BL602_RF_SINGEN_CTRL2         (BL602_RF_BASE + BL602_RF_SINGEN_CTRL2_OFFSET)
#define BL602_RF_SINGEN_CTRL3         (BL602_RF_BASE + BL602_RF_SINGEN_CTRL3_OFFSET)
#define BL602_RF_SINGEN_CTRL4         (BL602_RF_BASE + BL602_RF_SINGEN_CTRL4_OFFSET)
#define BL602_RF_RFIF_DFE_CTRL0       (BL602_RF_BASE + BL602_RF_RFIF_DFE_CTRL0_OFFSET)
#define BL602_RF_RFIF_TEST_READ       (BL602_RF_BASE + BL602_RF_RFIF_TEST_READ_OFFSET)
#define BL602_RF_RFIF_DIG_CTRL        (BL602_RF_BASE + BL602_RF_RFIF_DIG_CTRL_OFFSET)
#define BL602_RF_DATA_TEMP_0          (BL602_RF_BASE + BL602_RF_DATA_TEMP_0_OFFSET)
#define BL602_RF_DATA_TEMP_1          (BL602_RF_BASE + BL602_RF_DATA_TEMP_1_OFFSET)
#define BL602_RF_DATA_TEMP_2          (BL602_RF_BASE + BL602_RF_DATA_TEMP_2_OFFSET)
#define BL602_RF_DATA_TEMP_3          (BL602_RF_BASE + BL602_RF_DATA_TEMP_3_OFFSET)
#define BL602_RF_SRAM_CTRL0           (BL602_RF_BASE + BL602_RF_SRAM_CTRL0_OFFSET)
#define BL602_RF_SRAM_CTRL1           (BL602_RF_BASE + BL602_RF_SRAM_CTRL1_OFFSET)
#define BL602_RF_SRAM_CTRL2           (BL602_RF_BASE + BL602_RF_SRAM_CTRL2_OFFSET)
#define BL602_RF_SRAM_CTRL3           (BL602_RF_BASE + BL602_RF_SRAM_CTRL3_OFFSET)
#define BL602_RF_SRAM_CTRL4           (BL602_RF_BASE + BL602_RF_SRAM_CTRL4_OFFSET)
#define BL602_RF_SRAM_CTRL5           (BL602_RF_BASE + BL602_RF_SRAM_CTRL5_OFFSET)
#define BL602_RF_SRAM_CTRL6           (BL602_RF_BASE + BL602_RF_SRAM_CTRL6_OFFSET)
#define BL602_RF_ICAL_CTRL0           (BL602_RF_BASE + BL602_RF_ICAL_CTRL0_OFFSET)
#define BL602_RF_ICAL_CTRL1           (BL602_RF_BASE + BL602_RF_ICAL_CTRL1_OFFSET)
#define BL602_RF_ICAL_CTRL2           (BL602_RF_BASE + BL602_RF_ICAL_CTRL2_OFFSET)
#define BL602_RF_FSM_CTRL0            (BL602_RF_BASE + BL602_RF_FSM_CTRL0_OFFSET)
#define BL602_RF_FSM_CTRL1            (BL602_RF_BASE + BL602_RF_FSM_CTRL1_OFFSET)
#define BL602_RF_FSM_CTRL2            (BL602_RF_BASE + BL602_RF_FSM_CTRL2_OFFSET)
#define BL602_RF_PKDET_CTRL0          (BL602_RF_BASE + BL602_RF_PKDET_CTRL0_OFFSET)
#define BL602_RF_DFE_CTRL_0           (BL602_RF_BASE + BL602_RF_DFE_CTRL_0_OFFSET)
#define BL602_RF_DFE_CTRL_1           (BL602_RF_BASE + BL602_RF_DFE_CTRL_1_OFFSET)
#define BL602_RF_DFE_CTRL_2           (BL602_RF_BASE + BL602_RF_DFE_CTRL_2_OFFSET)
#define BL602_RF_DFE_CTRL_3           (BL602_RF_BASE + BL602_RF_DFE_CTRL_3_OFFSET)
#define BL602_RF_DFE_CTRL_4           (BL602_RF_BASE + BL602_RF_DFE_CTRL_4_OFFSET)
#define BL602_RF_DFE_CTRL_5           (BL602_RF_BASE + BL602_RF_DFE_CTRL_5_OFFSET)
#define BL602_RF_DFE_CTRL_6           (BL602_RF_BASE + BL602_RF_DFE_CTRL_6_OFFSET)
#define BL602_RF_DFE_CTRL_7           (BL602_RF_BASE + BL602_RF_DFE_CTRL_7_OFFSET)
#define BL602_RF_DFE_CTRL_8           (BL602_RF_BASE + BL602_RF_DFE_CTRL_8_OFFSET)
#define BL602_RF_DFE_CTRL_9           (BL602_RF_BASE + BL602_RF_DFE_CTRL_9_OFFSET)
#define BL602_RF_DFE_CTRL_10          (BL602_RF_BASE + BL602_RF_DFE_CTRL_10_OFFSET)
#define BL602_RF_DFE_CTRL_11          (BL602_RF_BASE + BL602_RF_DFE_CTRL_11_OFFSET)
#define BL602_RF_DFE_CTRL_12          (BL602_RF_BASE + BL602_RF_DFE_CTRL_12_OFFSET)
#define BL602_RF_DFE_CTRL_13          (BL602_RF_BASE + BL602_RF_DFE_CTRL_13_OFFSET)
#define BL602_RF_DFE_CTRL_14          (BL602_RF_BASE + BL602_RF_DFE_CTRL_14_OFFSET)
#define BL602_RF_DFE_CTRL_15          (BL602_RF_BASE + BL602_RF_DFE_CTRL_15_OFFSET)
#define BL602_RF_DFE_CTRL_16          (BL602_RF_BASE + BL602_RF_DFE_CTRL_16_OFFSET)
#define BL602_RF_DFE_CTRL_17          (BL602_RF_BASE + BL602_RF_DFE_CTRL_17_OFFSET)
#define BL602_RF_DFE_CTRL_18          (BL602_RF_BASE + BL602_RF_DFE_CTRL_18_OFFSET)

/* Register bit definitions *************************************************/

#define RF_REV_HW_REV_SHIFT                                (16)
#define RF_REV_HW_REV_MASK                                 (0xff << RF_REV_HW_REV_SHIFT)
#define RF_REV_FW_REV_SHIFT                                (8)
#define RF_REV_FW_REV_MASK                                 (0xff << RF_REV_FW_REV_SHIFT)
#define RF_REV_RF_ID_MASK                                  (0xff)

#define RF_FSM_CTRL_HW_RF_RC_STATE_VALUE_SHIFT             (28)
#define RF_FSM_CTRL_HW_RF_RC_STATE_VALUE_MASK              (0x07 << RF_FSM_CTRL_HW_RF_RC_STATE_VALUE_SHIFT)
#define RF_FSM_CTRL_HW_RF_FSM_ST_INT_SET                   (1 << 24)
#define RF_FSM_CTRL_HW_RF_FSM_ST_INT_CLR                   (1 << 20)
#define RF_FSM_CTRL_HW_RF_FSM_ST_INT                       (1 << 16)
#define RF_FSM_CTRL_HW_RF_FSM_ST_INT_SEL_SHIFT             (12)
#define RF_FSM_CTRL_HW_RF_FSM_ST_INT_SEL_MASK              (0x07 << RF_FSM_CTRL_HW_RF_FSM_ST_INT_SEL_SHIFT)
#define RF_FSM_CTRL_HW_RF_RC_STATE_DBG_EN                  (1 << 11)
#define RF_FSM_CTRL_HW_RF_RC_STATE_DBG_SHIFT               (8)
#define RF_FSM_CTRL_HW_RF_RC_STATE_DBG_MASK                (0x07 << RF_FSM_CTRL_HW_RF_RC_STATE_DBG_SHIFT)
#define RF_FSM_CTRL_HW_RF_FSM_STATE_SHIFT                  (4)
#define RF_FSM_CTRL_HW_RF_FSM_STATE_MASK                   (0x07 << RF_FSM_CTRL_HW_RF_FSM_STATE_SHIFT)
#define RF_FSM_CTRL_HW_RF_FSM_T2R_CAL_MODE_SHIFT           (2)
#define RF_FSM_CTRL_HW_RF_FSM_T2R_CAL_MODE_MASK            (0x03 << RF_FSM_CTRL_HW_RF_FSM_T2R_CAL_MODE_SHIFT)
#define RF_FSM_CTRL_HW_RF_FSM_CTRL_EN                      (1 << 1)

#define RF_FSM_CTRL_SW_LO_UNLOCKED                         (1 << 20)
#define RF_FSM_CTRL_SW_INC_CAL_TIMEOUT                     (1 << 16)
#define RF_FSM_CTRL_SW_FULL_CAL_EN                         (1 << 12)
#define RF_FSM_CTRL_SW_RF_FSM_SW_ST_VLD                    (1 << 8)
#define RF_FSM_CTRL_SW_RF_FSM_SW_ST_MASK                   (0x1f)

#define RFCTRL_HW_EN_ADDA_CTRL_HW                          (1 << 12)
#define RFCTRL_HW_EN_RBB_PKDET_OUT_RSTN_CTRL_HW            (1 << 11)
#define RFCTRL_HW_EN_RBB_PKDET_EN_CTRL_HW                  (1 << 10)
#define RFCTRL_HW_EN_SDM_CTRL_HW                           (1 << 9)
#define RFCTRL_HW_EN_INC_FCAL_CTRL_EN_HW                   (1 << 8)
#define RFCTRL_HW_EN_INC_ACAL_CTRL_EN_HW                   (1 << 7)
#define RFCTRL_HW_EN_LO_CTRL_HW                            (1 << 6)
#define RFCTRL_HW_EN_TRXCAL_CTRL_HW                        (1 << 5)
#define RFCTRL_HW_EN_RBB_BW_CTRL_HW                        (1 << 4)
#define RFCTRL_HW_EN_LNA_CTRL_HW                           (1 << 3)
#define RFCTRL_HW_EN_TX_GAIN_CTRL_HW                       (1 << 2)
#define RFCTRL_HW_EN_RX_GAIN_CTRL_HW                       (1 << 1)
#define RFCTRL_HW_EN_PU_CTRL_HW                            (1 << 0)

#define RF_TEMP_COMP_TEMP_COMP_EN                             (1 << 16)
#define RF_TEMP_COMP_CONST_FCAL_SHIFT                         (8)
#define RF_TEMP_COMP_CONST_FCAL_MASK                          (0xff << RF_TEMP_COMP_CONST_FCAL_SHIFT)
#define RF_TEMP_COMP_CONST_ACAL_MASK                          (0xff)

#define RFCAL_STATUS_DPD_STATUS_SHIFT                      (30)
#define RFCAL_STATUS_DPD_STATUS_MASK                       (0x03 << RFCAL_STATUS_DPD_STATUS_SHIFT)
#define RFCAL_STATUS_TENSCAL_STATUS_SHIFT                  (28)
#define RFCAL_STATUS_TENSCAL_STATUS_MASK                   (0x03 << RFCAL_STATUS_TENSCAL_STATUS_SHIFT)
#define RFCAL_STATUS_PWDET_CAL_STATUS_SHIFT                (26)
#define RFCAL_STATUS_PWDET_CAL_STATUS_MASK                 (0x03 << RFCAL_STATUS_PWDET_CAL_STATUS_SHIFT)
#define RFCAL_STATUS_RIQCAL_STATUS_RESV_SHIFT              (24)
#define RFCAL_STATUS_RIQCAL_STATUS_RESV_MASK               (0x03 << RFCAL_STATUS_RIQCAL_STATUS_RESV_SHIFT)
#define RFCAL_STATUS_TIQCAL_STATUS_RESV_SHIFT              (22)
#define RFCAL_STATUS_TIQCAL_STATUS_RESV_MASK               (0x03 << RFCAL_STATUS_TIQCAL_STATUS_RESV_SHIFT)
#define RFCAL_STATUS_LO_LEAKCAL_STATUS_SHIFT               (20)
#define RFCAL_STATUS_LO_LEAKCAL_STATUS_MASK                (0x03 << RFCAL_STATUS_LO_LEAKCAL_STATUS_SHIFT)
#define RFCAL_STATUS_RCCAL_STATUS_SHIFT                    (18)
#define RFCAL_STATUS_RCCAL_STATUS_MASK                     (0x03 << RFCAL_STATUS_RCCAL_STATUS_SHIFT)
#define RFCAL_STATUS_TOS_STATUS_SHIFT                      (16)
#define RFCAL_STATUS_TOS_STATUS_MASK                       (0x03 << RFCAL_STATUS_TOS_STATUS_SHIFT)
#define RFCAL_STATUS_ROS_STATUS_SHIFT                      (14)
#define RFCAL_STATUS_ROS_STATUS_MASK                       (0x03 << RFCAL_STATUS_ROS_STATUS_SHIFT)
#define RFCAL_STATUS_CLKPLL_CAL_STATUS_SHIFT               (12)
#define RFCAL_STATUS_CLKPLL_CAL_STATUS_MASK                (0x03 << RFCAL_STATUS_CLKPLL_CAL_STATUS_SHIFT)
#define RFCAL_STATUS_INC_ACAL_STATUS_SHIFT                 (10)
#define RFCAL_STATUS_INC_ACAL_STATUS_MASK                  (0x03 << RFCAL_STATUS_INC_ACAL_STATUS_SHIFT)
#define RFCAL_STATUS_INC_FCAL_STATUS_SHIFT                 (8)
#define RFCAL_STATUS_INC_FCAL_STATUS_MASK                  (0x03 << RFCAL_STATUS_INC_FCAL_STATUS_SHIFT)
#define RFCAL_STATUS_ACAL_STATUS_SHIFT                     (6)
#define RFCAL_STATUS_ACAL_STATUS_MASK                      (0x03 << RFCAL_STATUS_ACAL_STATUS_SHIFT)
#define RFCAL_STATUS_FCAL_STATUS_SHIFT                     (4)
#define RFCAL_STATUS_FCAL_STATUS_MASK                      (0x03 << RFCAL_STATUS_FCAL_STATUS_SHIFT)
#define RFCAL_STATUS_ADC_OSCAL_STATUS_SHIFT                (2)
#define RFCAL_STATUS_ADC_OSCAL_STATUS_MASK                 (0x03 << RFCAL_STATUS_ADC_OSCAL_STATUS_SHIFT)
#define RFCAL_STATUS_RCAL_STATUS_MASK                      (0x03)

#define RFCAL_STATUS2_DL_RFCAL_TABLE_STATUS_MASK           (0x03)

#define RFCAL_CTRLEN_DPD_EN                                (1 << 17)
#define RFCAL_CTRLEN_TSENCAL_EN                            (1 << 16)
#define RFCAL_CTRLEN_PWDET_CAL_EN                          (1 << 15)
#define RFCAL_CTRLEN_RIQCAL_EN                             (1 << 14)
#define RFCAL_CTRLEN_TIQCAL_EN                             (1 << 13)
#define RFCAL_CTRLEN_LO_LEAKCAL_EN                         (1 << 12)
#define RFCAL_CTRLEN_RCCAL_EN                              (1 << 11)
#define RFCAL_CTRLEN_TOSCAL_EN                             (1 << 10)
#define RFCAL_CTRLEN_ROSCAL_EN                             (1 << 9)
#define RFCAL_CTRLEN_CLKPLL_CAL_EN                         (1 << 8)
#define RFCAL_CTRLEN_ROSCAL_INC_EN                         (1 << 7)
#define RFCAL_CTRLEN_ACAL_INC_EN                           (1 << 6)
#define RFCAL_CTRLEN_FCAL_INC_EN                           (1 << 5)
#define RFCAL_CTRLEN_ACAL_EN                               (1 << 4)
#define RFCAL_CTRLEN_FCAL_EN                               (1 << 3)
#define RFCAL_CTRLEN_DL_RFCAL_TABLE_EN                     (1 << 2)
#define RFCAL_CTRLEN_ADC_OSCAL_EN                          (1 << 1)
#define RFCAL_CTRLEN_RCAL_EN_RESV                          (1 << 0)

#define RFCAL_STATEEN_RFCAL_LEVEL_SHIFT                    (30)
#define RFCAL_STATEEN_RFCAL_LEVEL_MASK                     (0x03 << RFCAL_STATEEN_RFCAL_LEVEL_SHIFT)
#define RFCAL_STATEEN_DPD_STEN                             (1 << 16)
#define RFCAL_STATEEN_TSENCAL_STEN                         (1 << 15)
#define RFCAL_STATEEN_PWDET_CAL_STEN                       (1 << 14)
#define RFCAL_STATEEN_RIQCAL_STEN                          (1 << 13)
#define RFCAL_STATEEN_TIQCAL_STEN                          (1 << 12)
#define RFCAL_STATEEN_LO_LEAKCAL_STEN                      (1 << 11)
#define RFCAL_STATEEN_RCCAL_STEN                           (1 << 10)
#define RFCAL_STATEEN_TOSCAL_STEN_RESV                     (1 << 9)
#define RFCAL_STATEEN_ROSCAL_STEN                          (1 << 8)
#define RFCAL_STATEEN_CLKPLL_CAL_STEN                      (1 << 7)
#define RFCAL_STATEEN_INC_ACAL_STEN                        (1 << 6)
#define RFCAL_STATEEN_INC_FCAL_STEN                        (1 << 5)
#define RFCAL_STATEEN_ACAL_STEN                            (1 << 4)
#define RFCAL_STATEEN_FCAL_STEN                            (1 << 3)
#define RFCAL_STATEEN_DL_RFCAL_TABLE_STEN                  (1 << 2)
#define RFCAL_STATEEN_ADC_OSCAL_STEN                       (1 << 1)
#define RFCAL_STATEEN_RCAL_STEN_RESV                       (1 << 0)

#define RF_BASE_CTRL1_MBG_TRIM_SHIFT                       (27)
#define RF_BASE_CTRL1_MBG_TRIM_MASK                        (0x03 << RF_BASE_CTRL1_MBG_TRIM_SHIFT)
#define RF_BASE_CTRL1_PUD_PA_DLY_SHIFT                     (14)
#define RF_BASE_CTRL1_PUD_PA_DLY_MASK                      (0x03 << RF_BASE_CTRL1_PUD_PA_DLY_SHIFT)
#define RF_BASE_CTRL1_PUD_IREF_DLY_SHIFT                   (12)
#define RF_BASE_CTRL1_PUD_IREF_DLY_MASK                    (0x03 << RF_BASE_CTRL1_PUD_IREF_DLY_SHIFT)
#define RF_BASE_CTRL1_PUD_VCO_DLY_SHIFT                    (10)
#define RF_BASE_CTRL1_PUD_VCO_DLY_MASK                     (0x03 << RF_BASE_CTRL1_PUD_VCO_DLY_SHIFT)
#define RF_BASE_CTRL1_PPU_LEAD_SHIFT                       (8)
#define RF_BASE_CTRL1_PPU_LEAD_MASK                        (0x03 << RF_BASE_CTRL1_PPU_LEAD_SHIFT)
#define RF_BASE_CTRL1_LO_SDM_RST_DLY_SHIFT                 (2)
#define RF_BASE_CTRL1_LO_SDM_RST_DLY_MASK                  (0x03 << RF_BASE_CTRL1_LO_SDM_RST_DLY_SHIFT)
#define RF_BASE_CTRL1_AUPLL_SDM_RST_DLY_MASK               (0x03)

#define RF_PUCR1_PU_TOSDAC                                    (1 << 31)
#define RF_PUCR1_PU_PWRMX                                     (1 << 30)
#define RF_PUCR1_PU_ROSDAC                                    (1 << 29)
#define RF_PUCR1_PU_PKDET                                     (1 << 28)
#define RF_PUCR1_TRSW_EN                                      (1 << 26)
#define RF_PUCR1_PU_TXBUF                                     (1 << 25)
#define RF_PUCR1_PU_RXBUF                                     (1 << 24)
#define RF_PUCR1_PU_OSMX                                      (1 << 23)
#define RF_PUCR1_PU_PFD                                       (1 << 22)
#define RF_PUCR1_PU_FBDV                                      (1 << 21)
#define RF_PUCR1_PU_VCO                                       (1 << 20)
#define RF_PUCR1_PU_DAC                                       (1 << 19)
#define RF_PUCR1_PU_TBB                                       (1 << 18)
#define RF_PUCR1_PU_TMX                                       (1 << 17)
#define RF_PUCR1_PU_PA                                        (1 << 16)
#define RF_PUCR1_PU_OP_ATEST                                  (1 << 15)
#define RF_PUCR1_PU_ADC                                       (1 << 14)
#define RF_PUCR1_ADC_CLK_EN                                   (1 << 13)
#define RF_PUCR1_PU_ADDA_LDO                                  (1 << 12)
#define RF_PUCR1_PU_RBB                                       (1 << 11)
#define RF_PUCR1_PU_RMX                                       (1 << 10)
#define RF_PUCR1_PU_RMXGM                                     (1 << 9)
#define RF_PUCR1_PU_LNA                                       (1 << 8)
#define RF_PUCR1_PU_SFREG                                     (1 << 0)

#define RF_PUCR1_HW_PU_TOSDAC_HW                              (1 << 31)
#define RF_PUCR1_HW_PU_ROSDAC_HW                              (1 << 29)
#define RF_PUCR1_HW_PU_PKDET_HW                               (1 << 28)
#define RF_PUCR1_HW_TRSW_EN_HW                                (1 << 26)
#define RF_PUCR1_HW_PU_TXBUF_HW                               (1 << 25)
#define RF_PUCR1_HW_PU_RXBUF_HW                               (1 << 24)
#define RF_PUCR1_HW_PU_OSMX_HW                                (1 << 23)
#define RF_PUCR1_HW_PU_PFD_HW                                 (1 << 22)
#define RF_PUCR1_HW_PU_FBDV_HW                                (1 << 21)
#define RF_PUCR1_HW_PU_VCO_HW                                 (1 << 20)
#define RF_PUCR1_HW_PU_DAC_HW                                 (1 << 19)
#define RF_PUCR1_HW_PU_TBB_HW                                 (1 << 18)
#define RF_PUCR1_HW_PU_TMX_HW                                 (1 << 17)
#define RF_PUCR1_HW_PU_PA_HW                                  (1 << 16)
#define RF_PUCR1_HW_PU_ADC_HW                                 (1 << 14)
#define RF_PUCR1_HW_ADC_CLK_EN_HW                             (1 << 13)
#define RF_PUCR1_HW_PU_ADDA_LDO_HW                            (1 << 12)
#define RF_PUCR1_HW_PU_RBB_HW                                 (1 << 11)
#define RF_PUCR1_HW_PU_RMX_HW                                 (1 << 10)
#define RF_PUCR1_HW_PU_RMXGM_HW                               (1 << 9)
#define RF_PUCR1_HW_PU_LNA_HW                                 (1 << 8)
#define RF_PUCR1_HW_PU_SFREG_HW                               (1 << 0)

#define RF_PPU_CTRL_HW_PPU_TXBUF_HW                           (1 << 25)
#define RF_PPU_CTRL_HW_PPU_RXBUF_HW                           (1 << 24)
#define RF_PPU_CTRL_HW_PPU_OSMX_HW                            (1 << 23)
#define RF_PPU_CTRL_HW_PPU_PFD_HW                             (1 << 22)
#define RF_PPU_CTRL_HW_PPU_FBDV_HW                            (1 << 21)
#define RF_PPU_CTRL_HW_PPU_VCO_HW                             (1 << 20)
#define RF_PPU_CTRL_HW_PPU_RBB_HW                             (1 << 11)
#define RF_PPU_CTRL_HW_PPU_RMXGM_HW                           (1 << 9)
#define RF_PPU_CTRL_HW_PPU_LNA_HW                             (1 << 8)

#define RF_PUD_CTRL_HW_PUD_VCO_HW                             (1 << 20)

#define RF_TRX_GAIN1_GC_TBB_BOOST_SHIFT                       (28)
#define RF_TRX_GAIN1_GC_TBB_BOOST_MASK                        (0x03 << RF_TRX_GAIN1_GC_TBB_BOOST_SHIFT)
#define RF_TRX_GAIN1_GC_TBB_SHIFT                             (20)
#define RF_TRX_GAIN1_GC_TBB_MASK                              (0x1f << RF_TRX_GAIN1_GC_TBB_SHIFT)
#define RF_TRX_GAIN1_GC_TMX_SHIFT                             (16)
#define RF_TRX_GAIN1_GC_TMX_MASK                              (0x07 << RF_TRX_GAIN1_GC_TMX_SHIFT)
#define RF_TRX_GAIN1_GC_RBB2_SHIFT                            (12)
#define RF_TRX_GAIN1_GC_RBB2_MASK                             (0x07 << RF_TRX_GAIN1_GC_RBB2_SHIFT)
#define RF_TRX_GAIN1_GC_RBB1_SHIFT                            (8)
#define RF_TRX_GAIN1_GC_RBB1_MASK                             (0x03 << RF_TRX_GAIN1_GC_RBB1_SHIFT)
#define RF_TRX_GAIN1_GC_RMXGM_SHIFT                           (4)
#define RF_TRX_GAIN1_GC_RMXGM_MASK                            (0x03 << RF_TRX_GAIN1_GC_RMXGM_SHIFT)
#define RF_TRX_GAIN1_GC_LNA_MASK                              (0x07)

#define RF_TRX_GAIN_HW_GC_TBB_BOOST_HW_SHIFT                  (28)
#define RF_TRX_GAIN_HW_GC_TBB_BOOST_HW_MASK                   (0x03 << RF_TRX_GAIN_HW_GC_TBB_BOOST_HW_SHIFT)
#define RF_TRX_GAIN_HW_GC_TBB_HW_SHIFT                        (20)
#define RF_TRX_GAIN_HW_GC_TBB_HW_MASK                         (0x1f << RF_TRX_GAIN_HW_GC_TBB_HW_SHIFT)
#define RF_TRX_GAIN_HW_GC_TMX_HW_SHIFT                        (16)
#define RF_TRX_GAIN_HW_GC_TMX_HW_MASK                         (0x07 << RF_TRX_GAIN_HW_GC_TMX_HW_SHIFT)
#define RF_TRX_GAIN_HW_GC_RBB2_HW_SHIFT                       (12)
#define RF_TRX_GAIN_HW_GC_RBB2_HW_MASK                        (0x07 << RF_TRX_GAIN_HW_GC_RBB2_HW_SHIFT)
#define RF_TRX_GAIN_HW_GC_RBB1_HW_SHIFT                       (8)
#define RF_TRX_GAIN_HW_GC_RBB1_HW_MASK                        (0x03 << RF_TRX_GAIN_HW_GC_RBB1_HW_SHIFT)
#define RF_TRX_GAIN_HW_GC_RMXGM_HW_SHIFT                      (4)
#define RF_TRX_GAIN_HW_GC_RMXGM_HW_MASK                       (0x03 << RF_TRX_GAIN_HW_GC_RMXGM_HW_SHIFT)
#define RF_TRX_GAIN_HW_GC_LNA_HW_MASK                         (0x07)

#define RF_TEN_DC_TEN_LODIST                                  (1 << 27)
#define RF_TEN_DC_TEN_LF                                      (1 << 26)
#define RF_TEN_DC_TEN_PFDCP                                   (1 << 25)
#define RF_TEN_DC_TEN_VCO                                     (1 << 24)
#define RF_TEN_DC_TEN_DAC_Q                                   (1 << 22)
#define RF_TEN_DC_TEN_DAC_I                                   (1 << 21)
#define RF_TEN_DC_TEN_ADC                                     (1 << 20)
#define RF_TEN_DC_TEN_TBB                                     (1 << 19)
#define RF_TEN_DC_TEN_ATEST                                   (1 << 18)
#define RF_TEN_DC_TEN_BQ                                      (1 << 17)
#define RF_TEN_DC_TEN_TIA                                     (1 << 16)
#define RF_TEN_DC_TEN_TMX                                     (1 << 15)
#define RF_TEN_DC_TEN_PA                                      (1 << 14)
#define RF_TEN_DC_TEN_RRF_1                                   (1 << 13)
#define RF_TEN_DC_TEN_RRF_0                                   (1 << 12)
#define RF_TEN_DC_TEN_CLKPLL_SFREG                            (1 << 9)
#define RF_TEN_DC_TEN_CLKPLL                                  (1 << 8)
#define RF_TEN_DC_DC_TP_CLKPLL_EN                             (1 << 4)
#define RF_TEN_DC_DC_TP_EN                                    (1 << 3)
#define RF_TEN_DC_TMUX_MASK                                   (0x07)

#define RF_TEN_DIG_RF_DTEST_EN                                (1 << 23)
#define RF_TEN_DIG_DTEST_PULL_DOWN                            (1 << 9)
#define RF_TEN_DIG_DTEN_LO_FREF                               (1 << 8)
#define RF_TEN_DIG_DTEN_LO_FSDM                               (1 << 6)
#define RF_TEN_DIG_DTEN_CLKPLL_FIN                            (1 << 5)
#define RF_TEN_DIG_DTEN_CLKPLL_FREF                           (1 << 4)
#define RF_TEN_DIG_DTEN_CLKPLL_FSDM                           (1 << 3)
#define RF_TEN_DIG_DTEN_CLKPLL_CLK32M                         (1 << 2)
#define RF_TEN_DIG_DTEN_CLKPLL_CLK96M                         (1 << 1)
#define RF_TEN_DIG_DTEN_CLKPLL_POSTDIV_CLK                    (1 << 0)

#define RF_TEN_AC_ATEST_IN_EN_I                               (1 << 23)
#define RF_TEN_AC_ATEST_IN_EN_Q                               (1 << 22)
#define RF_TEN_AC_ATEST_OUT_EN_I                              (1 << 21)
#define RF_TEN_AC_ATEST_OUT_EN_Q                              (1 << 20)
#define RF_TEN_AC_ATEST_GAIN_R5_SHIFT                         (16)
#define RF_TEN_AC_ATEST_GAIN_R5_MASK                          (0x07 << RF_TEN_AC_ATEST_GAIN_R5_SHIFT)
#define RF_TEN_AC_ATEST_GAIN_R6_SHIFT                         (14)
#define RF_TEN_AC_ATEST_GAIN_R6_MASK                          (0x03 << RF_TEN_AC_ATEST_GAIN_R6_SHIFT)
#define RF_TEN_AC_ATEST_GAIN_R7_SHIFT                         (12)
#define RF_TEN_AC_ATEST_GAIN_R7_MASK                          (0x03 << RF_TEN_AC_ATEST_GAIN_R7_SHIFT)
#define RF_TEN_AC_ATEST_GAIN_R8_SHIFT                         (10)
#define RF_TEN_AC_ATEST_GAIN_R8_MASK                          (0x03 << RF_TEN_AC_ATEST_GAIN_R8_SHIFT)
#define RF_TEN_AC_ATEST_GAIN_R9_SHIFT                         (8)
#define RF_TEN_AC_ATEST_GAIN_R9_MASK                          (0x03 << RF_TEN_AC_ATEST_GAIN_R9_SHIFT)
#define RF_TEN_AC_ATEST_IN_EN                                 (1 << 6)
#define RF_TEN_AC_ATEST_IN_TRX_SW                             (1 << 5)
#define RF_TEN_AC_ATEST_DAC_EN                                (1 << 4)
#define RF_TEN_AC_ATEST_OP_CC_MASK                            (0x0f)

#define RF_CIP_VG13_SEL_SHIFT                                 (2)
#define RF_CIP_VG13_SEL_MASK                                  (0x03 << RF_CIP_VG13_SEL_SHIFT)
#define RF_CIP_VG11_SEL_MASK                                  (0x03)

#define RF_PA1_PA_ATT_GC_SHIFT                                (28)
#define RF_PA1_PA_ATT_GC_MASK                                 (0x0f << RF_PA1_PA_ATT_GC_SHIFT)
#define RF_PA1_PA_PWRMX_BM_SHIFT                              (24)
#define RF_PA1_PA_PWRMX_BM_MASK                               (0x07 << RF_PA1_PA_PWRMX_BM_SHIFT)
#define RF_PA1_PA_PWRMX_DAC_PN_SWITCH                         (1 << 22)
#define RF_PA1_PA_PWRMX_OSDAC_SHIFT                           (18)
#define RF_PA1_PA_PWRMX_OSDAC_MASK                            (0x0f << RF_PA1_PA_PWRMX_OSDAC_SHIFT)
#define RF_PA1_PA_LZ_BIAS_EN                                  (1 << 17)
#define RF_PA1_PA_IB_FIX                                      (1 << 16)
#define RF_PA1_PA_HALF_ON                                     (1 << 15)
#define RF_PA1_PA_VBCAS_SHIFT                                 (12)
#define RF_PA1_PA_VBCAS_MASK                                  (0x07 << RF_PA1_PA_VBCAS_SHIFT)
#define RF_PA1_PA_VBCORE_SHIFT                                (8)
#define RF_PA1_PA_VBCORE_MASK                                 (0x0f << RF_PA1_PA_VBCORE_SHIFT)
#define RF_PA1_PA_IET_SHIFT                                   (4)
#define RF_PA1_PA_IET_MASK                                    (0x0f << RF_PA1_PA_IET_SHIFT)
#define RF_PA1_PA_ETB_EN                                      (1 << 3)
#define RF_PA1_PA_IAQ_MASK                                    (0x07)

#define RF_PA2_PA_IB_FIX_HW                                   (1 << 17)
#define RF_PA2_PA_HALF_ON_HW                                  (1 << 16)
#define RF_PA2_PA_VBCAS_HW_SHIFT                              (12)
#define RF_PA2_PA_VBCAS_HW_MASK                               (0x07 << RF_PA2_PA_VBCAS_HW_SHIFT)
#define RF_PA2_PA_VBCORE_HW_SHIFT                             (8)
#define RF_PA2_PA_VBCORE_HW_MASK                              (0x0f << RF_PA2_PA_VBCORE_HW_SHIFT)
#define RF_PA2_PA_IET_HW_SHIFT                                (4)
#define RF_PA2_PA_IET_HW_MASK                                 (0x0f << RF_PA2_PA_IET_HW_SHIFT)
#define RF_PA2_PA_ETB_EN_HW                                   (1 << 3)

#define RF_TMX_TX_TSENSE_EN                                   (1 << 16)
#define RF_TMX_TMX_BM_CAS_BULK_SHIFT                          (12)
#define RF_TMX_TMX_BM_CAS_BULK_MASK                           (0x07 << RF_TMX_TMX_BM_CAS_BULK_SHIFT)
#define RF_TMX_TMX_BM_CAS_SHIFT                               (8)
#define RF_TMX_TMX_BM_CAS_MASK                                (0x07 << RF_TMX_TMX_BM_CAS_SHIFT)
#define RF_TMX_TMX_BM_SW_SHIFT                                (4)
#define RF_TMX_TMX_BM_SW_MASK                                 (0x07 << RF_TMX_TMX_BM_SW_SHIFT)
#define RF_TMX_TMX_CS_MASK                                    (0x07)

#define RF_TBB_TOSDAC_I_SHIFT                                 (24)
#define RF_TBB_TOSDAC_I_MASK                                  (0x3f << RF_TBB_TOSDAC_I_SHIFT)
#define RF_TBB_TOSDAC_Q_SHIFT                                 (16)
#define RF_TBB_TOSDAC_Q_MASK                                  (0x3f << RF_TBB_TOSDAC_Q_SHIFT)
#define RF_TBB_ATEST_OUT_EN                                   (1 << 15)
#define RF_TBB_IQ_BIAS_SHORT                                  (1 << 14)
#define RF_TBB_CFLT_SHIFT                                     (12)
#define RF_TBB_CFLT_MASK                                      (0x03 << RF_TBB_CFLT_SHIFT)
#define RF_TBB_VCM_SHIFT                                      (8)
#define RF_TBB_VCM_MASK                                       (0x03 << RF_TBB_VCM_SHIFT)
#define RF_TBB_BM_CG_SHIFT                                    (4)
#define RF_TBB_BM_CG_MASK                                     (0x03 << RF_TBB_BM_CG_SHIFT)
#define RF_TBB_BM_SF_MASK                                     (0x03)

#define RF_LNA_LG_GSEL_SHIFT                                  (24)
#define RF_LNA_LG_GSEL_MASK                                   (0x07 << RF_LNA_LG_GSEL_SHIFT)
#define RF_LNA_CAP_LG_SHIFT                                   (20)
#define RF_LNA_CAP_LG_MASK                                    (0x03 << RF_LNA_CAP_LG_SHIFT)
#define RF_LNA_RFB_MATCH_SHIFT                                (16)
#define RF_LNA_RFB_MATCH_MASK                                 (0x07 << RF_LNA_RFB_MATCH_SHIFT)
#define RF_LNA_LOAD_CSW_HW_SHIFT                              (12)
#define RF_LNA_LOAD_CSW_HW_MASK                               (0x0f << RF_LNA_LOAD_CSW_HW_SHIFT)
#define RF_LNA_LOAD_CSW_SHIFT                                 (8)
#define RF_LNA_LOAD_CSW_MASK                                  (0x0f << RF_LNA_LOAD_CSW_SHIFT)
#define RF_LNA_BM_HW_SHIFT                                    (4)
#define RF_LNA_BM_HW_MASK                                     (0x0f << RF_LNA_BM_HW_SHIFT)
#define RF_LNA_BM_MASK                                        (0x0f)

#define RF_RMXGM_RMXGM_10M_MODE_EN                            (1 << 8)
#define RF_RMXGM_RMXGM_BM_SHIFT                               (4)
#define RF_RMXGM_RMXGM_BM_MASK                                (0x07 << RF_RMXGM_RMXGM_BM_SHIFT)
#define RF_RMXGM_RMX_BM_MASK                                  (0x07)

#define RF_RBB1_ROSDAC_RANGE                                  (1 << 31)
#define RF_RBB1_ROSDAC_I_HW_SHIFT                             (24)
#define RF_RBB1_ROSDAC_I_HW_MASK                              (0x3f << RF_RBB1_ROSDAC_I_HW_SHIFT)
#define RF_RBB1_ROSDAC_Q_HW_SHIFT                             (16)
#define RF_RBB1_ROSDAC_Q_HW_MASK                              (0x3f << RF_RBB1_ROSDAC_Q_HW_SHIFT)
#define RF_RBB1_ROSDAC_I_SHIFT                                (8)
#define RF_RBB1_ROSDAC_I_MASK                                 (0x3f << RF_RBB1_ROSDAC_I_SHIFT)
#define RF_RBB1_ROSDAC_Q_MASK                                 (0x3f)

#define RF_RBB2_RBB_CAP1_FC_I_SHIFT                           (24)
#define RF_RBB2_RBB_CAP1_FC_I_MASK                            (0x3f << RF_RBB2_RBB_CAP1_FC_I_SHIFT)
#define RF_RBB2_RBB_CAP1_FC_Q_SHIFT                           (16)
#define RF_RBB2_RBB_CAP1_FC_Q_MASK                            (0x3f << RF_RBB2_RBB_CAP1_FC_Q_SHIFT)
#define RF_RBB2_RBB_CAP2_FC_I_SHIFT                           (8)
#define RF_RBB2_RBB_CAP2_FC_I_MASK                            (0x3f << RF_RBB2_RBB_CAP2_FC_I_SHIFT)
#define RF_RBB2_RBB_CAP2_FC_Q_MASK                            (0x3f)

#define RF_RBB3_PWR_DET_EN                                    (1 << 31)
#define RF_RBB3_RXIQCAL_EN                                    (1 << 28)
#define RF_RBB3_RBB_BW_SHIFT                                  (24)
#define RF_RBB3_RBB_BW_MASK                                   (0x03 << RF_RBB3_RBB_BW_SHIFT)
#define RF_RBB3_RBB_TIA_IQBIAS_SHORT                          (1 << 21)
#define RF_RBB3_RBB_BQ_IQBIAS_SHORT                           (1 << 20)
#define RF_RBB3_RBB_VCM_SHIFT                                 (16)
#define RF_RBB3_RBB_VCM_MASK                                  (0x03 << RF_RBB3_RBB_VCM_SHIFT)
#define RF_RBB3_RBB_BM_OP_SHIFT                               (12)
#define RF_RBB3_RBB_BM_OP_MASK                                (0x07 << RF_RBB3_RBB_BM_OP_SHIFT)
#define RF_RBB3_RBB_DEQ_SHIFT                                 (8)
#define RF_RBB3_RBB_DEQ_MASK                                  (0x03 << RF_RBB3_RBB_DEQ_SHIFT)
#define RF_RBB3_RBB_BT_FIF_TUNE_SHIFT                         (5)
#define RF_RBB3_RBB_BT_FIF_TUNE_MASK                          (0x03 << RF_RBB3_RBB_BT_FIF_TUNE_SHIFT)
#define RF_RBB3_RBB_BT_MODE                                   (1 << 4)
#define RF_RBB3_RBB_BT_MODE_HW                                (1 << 0)

#define RF_RBB4_PKDET_OUT_LATCH                               (1 << 24)
#define RF_RBB4_PKDET_OUT_RAW                                 (1 << 20)
#define RF_RBB4_RBB_PKDET_EN_HW                               (1 << 16)
#define RF_RBB4_RBB_PKDET_OUT_RSTN_HW                         (1 << 12)
#define RF_RBB4_RBB_PKDET_EN                                  (1 << 8)
#define RF_RBB4_RBB_PKDET_OUT_RSTN                            (1 << 4)
#define RF_RBB4_RBB_PKDET_VTH_MASK                            (0x0f)

#define RF_ADDA1_ADDA_LDO_DVDD_SEL_HW_SHIFT                   (24)
#define RF_ADDA1_ADDA_LDO_DVDD_SEL_HW_MASK                    (0x07 << RF_ADDA1_ADDA_LDO_DVDD_SEL_HW_SHIFT)
#define RF_ADDA1_ADDA_LDO_DVDD_SEL_SHIFT                      (20)
#define RF_ADDA1_ADDA_LDO_DVDD_SEL_MASK                       (0x07 << RF_ADDA1_ADDA_LDO_DVDD_SEL_SHIFT)
#define RF_ADDA1_ADDA_LDO_BYPS                                (1 << 16)
#define RF_ADDA1_DAC_CLK_SYNC_INV                             (1 << 13)
#define RF_ADDA1_DAC_RCCALSEL                                 (1 << 12)
#define RF_ADDA1_DAC_CLK_SEL_SHIFT                            (8)
#define RF_ADDA1_DAC_CLK_SEL_MASK                             (0x03 << RF_ADDA1_DAC_CLK_SEL_SHIFT)
#define RF_ADDA1_DAC_BIAS_SEL_SHIFT                           (4)
#define RF_ADDA1_DAC_BIAS_SEL_MASK                            (0x03 << RF_ADDA1_DAC_BIAS_SEL_SHIFT)
#define RF_ADDA1_DAC_DVDD_SEL_MASK                            (0x03)

#define RF_ADDA2_ADC_CLK_DIV_SEL                              (1 << 28)
#define RF_ADDA2_ADC_CLK_INV                                  (1 << 24)
#define RF_ADDA2_ADC_CLK_SYNC_INV                             (1 << 20)
#define RF_ADDA2_ADC_GT_RM                                    (1 << 16)
#define RF_ADDA2_ADC_SAR_ASCAL_EN                             (1 << 12)
#define RF_ADDA2_ADC_DVDD_SEL_SHIFT                           (8)
#define RF_ADDA2_ADC_DVDD_SEL_MASK                            (0x03 << RF_ADDA2_ADC_DVDD_SEL_SHIFT)
#define RF_ADDA2_ADC_DLY_CTL_SHIFT                            (4)
#define RF_ADDA2_ADC_DLY_CTL_MASK                             (0x03 << RF_ADDA2_ADC_DLY_CTL_SHIFT)
#define RF_ADDA2_ADC_VREF_SEL_MASK                            (0x03)

#define RF_VCO1_LO_VCO_IDAC_CW_HW_SHIFT                       (24)
#define RF_VCO1_LO_VCO_IDAC_CW_HW_MASK                        (0x1f << RF_VCO1_LO_VCO_IDAC_CW_HW_SHIFT)
#define RF_VCO1_LO_VCO_IDAC_CW_SHIFT                          (16)
#define RF_VCO1_LO_VCO_IDAC_CW_MASK                           (0x1f << RF_VCO1_LO_VCO_IDAC_CW_SHIFT)
#define RF_VCO1_LO_VCO_FREQ_CW_HW_SHIFT                       (8)
#define RF_VCO1_LO_VCO_FREQ_CW_HW_MASK                        (0xff << RF_VCO1_LO_VCO_FREQ_CW_HW_SHIFT)
#define RF_VCO1_LO_VCO_FREQ_CW_MASK                           (0xff)

#define RF_VCO2_ACAL_INC_EN_HW                                (1 << 16)
#define RF_VCO2_ACAL_VCO_UD                                   (1 << 12)
#define RF_VCO2_ACAL_VREF_CW_SHIFT                            (8)
#define RF_VCO2_ACAL_VREF_CW_MASK                             (0x07 << RF_VCO2_ACAL_VREF_CW_SHIFT)
#define RF_VCO2_LO_VCO_SHORT_IDAC_FILTER                      (1 << 6)
#define RF_VCO2_LO_VCO_SHORT_VBIAS_FILTER                     (1 << 5)
#define RF_VCO2_LO_VCO_IDAC_BOOT                              (1 << 4)
#define RF_VCO2_LO_VCO_VBIAS_CW_MASK                          (0x03)

#define RF_VCO3_FCAL_CNT_OP_SHIFT                             (16)
#define RF_VCO3_FCAL_CNT_OP_MASK                              (0xffff << RF_VCO3_FCAL_CNT_OP_SHIFT)
#define RF_VCO3_FCAL_DIV_MASK                                 (0xffff)

#define RF_VCO4_FCAL_INC_VCTRL_UD_SHIFT                       (24)
#define RF_VCO4_FCAL_INC_VCTRL_UD_MASK                        (0x03 << RF_VCO4_FCAL_INC_VCTRL_UD_SHIFT)
#define RF_VCO4_FCAL_CNT_RDY                                  (1 << 20)
#define RF_VCO4_FCAL_INC_LARGE_RANGE                          (1 << 16)
#define RF_VCO4_FCAL_INC_EN_HW                                (1 << 8)
#define RF_VCO4_FCAL_CNT_START                                (1 << 4)

#define RF_PFDCP_LO_PFD_RST_CSD_HW                            (1 << 29)
#define RF_PFDCP_LO_PFD_RST_CSD                               (1 << 28)
#define RF_PFDCP_LO_PFD_RVDD_BOOST                            (1 << 24)
#define RF_PFDCP_LO_CP_HIZ                                    (1 << 20)
#define RF_PFDCP_LO_CP_OPAMP_EN                               (1 << 16)
#define RF_PFDCP_LO_CP_OTA_EN                                 (1 << 12)
#define RF_PFDCP_LO_CP_STARTUP_EN                             (1 << 8)
#define RF_PFDCP_LO_CP_SEL_HW                                 (1 << 4)
#define RF_PFDCP_LO_CP_SEL                                    (1 << 0)

#define RF_LO_SLIPPED_UP                                      (1 << 24)
#define RF_LO_SLIPPED_DN                                      (1 << 20)
#define RF_LO_LF_R4_SHORT                                     (1 << 18)
#define RF_LO_LF_R4_SHIFT                                     (16)
#define RF_LO_LF_R4_MASK                                      (0x03 << RF_LO_LF_R4_SHIFT)
#define RF_LO_LF_CZ_SHIFT                                     (14)
#define RF_LO_LF_CZ_MASK                                      (0x03 << RF_LO_LF_CZ_SHIFT)
#define RF_LO_LF_RZ_SHIFT                                     (12)
#define RF_LO_LF_RZ_MASK                                      (0x03 << RF_LO_LF_RZ_SHIFT)
#define RF_LO_LF_CZ_HW_SHIFT                                  (8)
#define RF_LO_LF_CZ_HW_MASK                                   (0x03 << RF_LO_LF_CZ_HW_SHIFT)
#define RF_LO_LF_R4_HW_SHIFT                                  (4)
#define RF_LO_LF_R4_HW_MASK                                   (0x03 << RF_LO_LF_R4_HW_SHIFT)
#define RF_LO_LF_RZ_HW_MASK                                   (0x03)

#define RF_FBDV_LO_FBDV_RST_HW                                (1 << 20)
#define RF_FBDV_LO_FBDV_RST                                   (1 << 16)
#define RF_FBDV_LO_FBDV_SEL_FB_CLK_SHIFT                      (12)
#define RF_FBDV_LO_FBDV_SEL_FB_CLK_MASK                       (0x03 << RF_FBDV_LO_FBDV_SEL_FB_CLK_SHIFT)
#define RF_FBDV_LO_FBDV_SEL_SAMPLE_CLK_SHIFT                  (8)
#define RF_FBDV_LO_FBDV_SEL_SAMPLE_CLK_MASK                   (0x03 << RF_FBDV_LO_FBDV_SEL_SAMPLE_CLK_SHIFT)
#define RF_FBDV_LO_FBDV_HALFSTEP_EN                           (1 << 4)
#define RF_FBDV_LO_FBDV_HALFSTEP_EN_HW                        (1 << 0)

#define RF_LODIST_RXBUF_STRE                                  (1 << 28)
#define RF_LODIST_TXBUF_STRE                                  (1 << 24)
#define RF_LODIST_LO_OSMX_CAP_SHIFT                           (20)
#define RF_LODIST_LO_OSMX_CAP_MASK                            (0x0f << RF_LODIST_LO_OSMX_CAP_SHIFT)
#define RF_LODIST_LO_OSMX_CAPBANK_BIAS_SHIFT                  (16)
#define RF_LODIST_LO_OSMX_CAPBANK_BIAS_MASK                   (0x03 << RF_LODIST_LO_OSMX_CAPBANK_BIAS_SHIFT)
#define RF_LODIST_LO_OSMX_VBUF_STRE                           (1 << 12)
#define RF_LODIST_LO_OSMX_FIX_CAP                             (1 << 8)
#define RF_LODIST_LO_OSMX_EN_XGM                              (1 << 4)
#define RF_LODIST_LO_OSMX_XGM_BOOST                           (1 << 0)

#define RF_SDM1_LO_SDM_FLAG                                   (1 << 20)
#define RF_SDM1_LO_SDM_RSTB_HW                                (1 << 17)
#define RF_SDM1_LO_SDM_RSTB                                   (1 << 16)
#define RF_SDM1_LO_SDM_BYPASS                                 (1 << 12)
#define RF_SDM1_LO_SDM_DITHER_SEL_SHIFT                       (8)
#define RF_SDM1_LO_SDM_DITHER_SEL_MASK                        (0x03 << RF_SDM1_LO_SDM_DITHER_SEL_SHIFT)
#define RF_SDM1_LO_SDM_BYPASS_HW                              (1 << 4)
#define RF_SDM1_LO_SDM_DITHER_SEL_HW_MASK                     (0x03)

#define RF_SDM2_LO_SDMIN_MASK                                 (0x3fffffff)

#define RF_SDM3_LO_SDMIN_HW_MASK                              (0x3fffffff)

#define RRF_GAIN_INDEX1_GAIN_CTRL5_GC_LNA_SHIFT               (27)
#define RRF_GAIN_INDEX1_GAIN_CTRL5_GC_LNA_MASK                (0x07 << RRF_GAIN_INDEX1_GAIN_CTRL5_GC_LNA_SHIFT)
#define RRF_GAIN_INDEX1_GAIN_CTRL5_GC_RMXGM_SHIFT             (25)
#define RRF_GAIN_INDEX1_GAIN_CTRL5_GC_RMXGM_MASK              (0x03 << RRF_GAIN_INDEX1_GAIN_CTRL5_GC_RMXGM_SHIFT)
#define RRF_GAIN_INDEX1_GAIN_CTRL4_GC_LNA_SHIFT               (22)
#define RRF_GAIN_INDEX1_GAIN_CTRL4_GC_LNA_MASK                (0x07 << RRF_GAIN_INDEX1_GAIN_CTRL4_GC_LNA_SHIFT)
#define RRF_GAIN_INDEX1_GAIN_CTRL4_GC_RMXGM_SHIFT             (20)
#define RRF_GAIN_INDEX1_GAIN_CTRL4_GC_RMXGM_MASK              (0x03 << RRF_GAIN_INDEX1_GAIN_CTRL4_GC_RMXGM_SHIFT)
#define RRF_GAIN_INDEX1_GAIN_CTRL3_GC_LNA_SHIFT               (17)
#define RRF_GAIN_INDEX1_GAIN_CTRL3_GC_LNA_MASK                (0x07 << RRF_GAIN_INDEX1_GAIN_CTRL3_GC_LNA_SHIFT)
#define RRF_GAIN_INDEX1_GAIN_CTRL3_GC_RMXGM_SHIFT             (15)
#define RRF_GAIN_INDEX1_GAIN_CTRL3_GC_RMXGM_MASK              (0x03 << RRF_GAIN_INDEX1_GAIN_CTRL3_GC_RMXGM_SHIFT)
#define RRF_GAIN_INDEX1_GAIN_CTRL2_GC_LNA_SHIFT               (12)
#define RRF_GAIN_INDEX1_GAIN_CTRL2_GC_LNA_MASK                (0x07 << RRF_GAIN_INDEX1_GAIN_CTRL2_GC_LNA_SHIFT)
#define RRF_GAIN_INDEX1_GAIN_CTRL2_GC_RMXGM_SHIFT             (10)
#define RRF_GAIN_INDEX1_GAIN_CTRL2_GC_RMXGM_MASK              (0x03 << RRF_GAIN_INDEX1_GAIN_CTRL2_GC_RMXGM_SHIFT)
#define RRF_GAIN_INDEX1_GAIN_CTRL1_GC_LNA_SHIFT               (7)
#define RRF_GAIN_INDEX1_GAIN_CTRL1_GC_LNA_MASK                (0x07 << RRF_GAIN_INDEX1_GAIN_CTRL1_GC_LNA_SHIFT)
#define RRF_GAIN_INDEX1_GAIN_CTRL1_GC_RMXGM_SHIFT             (5)
#define RRF_GAIN_INDEX1_GAIN_CTRL1_GC_RMXGM_MASK              (0x03 << RRF_GAIN_INDEX1_GAIN_CTRL1_GC_RMXGM_SHIFT)
#define RRF_GAIN_INDEX1_GAIN_CTRL0_GC_LNA_SHIFT               (2)
#define RRF_GAIN_INDEX1_GAIN_CTRL0_GC_LNA_MASK                (0x07 << RRF_GAIN_INDEX1_GAIN_CTRL0_GC_LNA_SHIFT)
#define RRF_GAIN_INDEX1_GAIN_CTRL0_GC_RMXGM_MASK              (0x03)

#define RRF_GAIN_INDEX2_GAIN_CTRL6_GC_LNA_SHIFT               (12)
#define RRF_GAIN_INDEX2_GAIN_CTRL6_GC_LNA_MASK                (0x07 << RRF_GAIN_INDEX2_GAIN_CTRL6_GC_LNA_SHIFT)
#define RRF_GAIN_INDEX2_GAIN_CTRL6_GC_RMXGM_SHIFT             (10)
#define RRF_GAIN_INDEX2_GAIN_CTRL6_GC_RMXGM_MASK              (0x03 << RRF_GAIN_INDEX2_GAIN_CTRL6_GC_RMXGM_SHIFT)
#define RRF_GAIN_INDEX2_GAIN_CTRL7_GC_LNA_SHIFT               (7)
#define RRF_GAIN_INDEX2_GAIN_CTRL7_GC_LNA_MASK                (0x07 << RRF_GAIN_INDEX2_GAIN_CTRL7_GC_LNA_SHIFT)
#define RRF_GAIN_INDEX2_GAIN_CTRL7_GC_RMXGM_SHIFT             (5)
#define RRF_GAIN_INDEX2_GAIN_CTRL7_GC_RMXGM_MASK              (0x03 << RRF_GAIN_INDEX2_GAIN_CTRL7_GC_RMXGM_SHIFT)
#define RRF_GAIN_INDEX2_GAIN_CTRL8_GC_LNA_SHIFT               (2)
#define RRF_GAIN_INDEX2_GAIN_CTRL8_GC_LNA_MASK                (0x07 << RRF_GAIN_INDEX2_GAIN_CTRL8_GC_LNA_SHIFT)
#define RRF_GAIN_INDEX2_GAIN_CTRL8_GC_RMXGM_MASK              (0x03)

#define RF_LNA_CTRL_HW_MUX_LNA_LOAD_CSW_LG_SHIFT              (12)
#define RF_LNA_CTRL_HW_MUX_LNA_LOAD_CSW_LG_MASK               (0x0f << RF_LNA_CTRL_HW_MUX_LNA_LOAD_CSW_LG_SHIFT)
#define RF_LNA_CTRL_HW_MUX_LNA_LOAD_CSW_HG_SHIFT              (8)
#define RF_LNA_CTRL_HW_MUX_LNA_LOAD_CSW_HG_MASK               (0x0f << RF_LNA_CTRL_HW_MUX_LNA_LOAD_CSW_HG_SHIFT)
#define RF_LNA_CTRL_HW_MUX_LNA_BM_LG_SHIFT                    (4)
#define RF_LNA_CTRL_HW_MUX_LNA_BM_LG_MASK                     (0x0f << RF_LNA_CTRL_HW_MUX_LNA_BM_LG_SHIFT)
#define RF_LNA_CTRL_HW_MUX_LNA_BM_HG_MASK                     (0x0f)

#define RF_RBB_GAIN_INDEX1_GAIN_CTRL3_GC_RBB2_SHIFT           (28)
#define RF_RBB_GAIN_INDEX1_GAIN_CTRL3_GC_RBB2_MASK            (0x07 << RF_RBB_GAIN_INDEX1_GAIN_CTRL3_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX1_GAIN_CTRL3_GC_RBB1_SHIFT           (24)
#define RF_RBB_GAIN_INDEX1_GAIN_CTRL3_GC_RBB1_MASK            (0x03 << RF_RBB_GAIN_INDEX1_GAIN_CTRL3_GC_RBB1_SHIFT)
#define RF_RBB_GAIN_INDEX1_GAIN_CTRL2_GC_RBB2_SHIFT           (20)
#define RF_RBB_GAIN_INDEX1_GAIN_CTRL2_GC_RBB2_MASK            (0x07 << RF_RBB_GAIN_INDEX1_GAIN_CTRL2_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX1_GAIN_CTRL2_GC_RBB1_SHIFT           (16)
#define RF_RBB_GAIN_INDEX1_GAIN_CTRL2_GC_RBB1_MASK            (0x03 << RF_RBB_GAIN_INDEX1_GAIN_CTRL2_GC_RBB1_SHIFT)
#define RF_RBB_GAIN_INDEX1_GAIN_CTRL1_GC_RBB2_SHIFT           (12)
#define RF_RBB_GAIN_INDEX1_GAIN_CTRL1_GC_RBB2_MASK            (0x07 << RF_RBB_GAIN_INDEX1_GAIN_CTRL1_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX1_GAIN_CTRL1_GC_RBB1_SHIFT           (8)
#define RF_RBB_GAIN_INDEX1_GAIN_CTRL1_GC_RBB1_MASK            (0x03 << RF_RBB_GAIN_INDEX1_GAIN_CTRL1_GC_RBB1_SHIFT)
#define RF_RBB_GAIN_INDEX1_GAIN_CTRL0_GC_RBB2_SHIFT           (4)
#define RF_RBB_GAIN_INDEX1_GAIN_CTRL0_GC_RBB2_MASK            (0x07 << RF_RBB_GAIN_INDEX1_GAIN_CTRL0_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX1_GAIN_CTRL0_GC_RBB1_MASK            (0x03)

#define RF_RBB_GAIN_INDEX2_GAIN_CTRL7_GC_RBB2_SHIFT           (28)
#define RF_RBB_GAIN_INDEX2_GAIN_CTRL7_GC_RBB2_MASK            (0x07 << RF_RBB_GAIN_INDEX2_GAIN_CTRL7_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX2_GAIN_CTRL7_GC_RBB1_SHIFT           (24)
#define RF_RBB_GAIN_INDEX2_GAIN_CTRL7_GC_RBB1_MASK            (0x03 << RF_RBB_GAIN_INDEX2_GAIN_CTRL7_GC_RBB1_SHIFT)
#define RF_RBB_GAIN_INDEX2_GAIN_CTRL6_GC_RBB2_SHIFT           (20)
#define RF_RBB_GAIN_INDEX2_GAIN_CTRL6_GC_RBB2_MASK            (0x07 << RF_RBB_GAIN_INDEX2_GAIN_CTRL6_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX2_GAIN_CTRL6_GC_RBB1_SHIFT           (16)
#define RF_RBB_GAIN_INDEX2_GAIN_CTRL6_GC_RBB1_MASK            (0x03 << RF_RBB_GAIN_INDEX2_GAIN_CTRL6_GC_RBB1_SHIFT)
#define RF_RBB_GAIN_INDEX2_GAIN_CTRL5_GC_RBB2_SHIFT           (12)
#define RF_RBB_GAIN_INDEX2_GAIN_CTRL5_GC_RBB2_MASK            (0x07 << RF_RBB_GAIN_INDEX2_GAIN_CTRL5_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX2_GAIN_CTRL5_GC_RBB1_SHIFT           (8)
#define RF_RBB_GAIN_INDEX2_GAIN_CTRL5_GC_RBB1_MASK            (0x03 << RF_RBB_GAIN_INDEX2_GAIN_CTRL5_GC_RBB1_SHIFT)
#define RF_RBB_GAIN_INDEX2_GAIN_CTRL4_GC_RBB2_SHIFT           (4)
#define RF_RBB_GAIN_INDEX2_GAIN_CTRL4_GC_RBB2_MASK            (0x07 << RF_RBB_GAIN_INDEX2_GAIN_CTRL4_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX2_GAIN_CTRL4_GC_RBB1_MASK            (0x03)

#define RF_RBB_GAIN_INDEX3_GAIN_CTRL11_GC_RBB2_SHIFT          (28)
#define RF_RBB_GAIN_INDEX3_GAIN_CTRL11_GC_RBB2_MASK           (0x07 << RF_RBB_GAIN_INDEX3_GAIN_CTRL11_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX3_GAIN_CTRL11_GC_RBB1_SHIFT          (24)
#define RF_RBB_GAIN_INDEX3_GAIN_CTRL11_GC_RBB1_MASK           (0x03 << RF_RBB_GAIN_INDEX3_GAIN_CTRL11_GC_RBB1_SHIFT)
#define RF_RBB_GAIN_INDEX3_GAIN_CTRL10_GC_RBB2_SHIFT          (20)
#define RF_RBB_GAIN_INDEX3_GAIN_CTRL10_GC_RBB2_MASK           (0x07 << RF_RBB_GAIN_INDEX3_GAIN_CTRL10_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX3_GAIN_CTRL10_GC_RBB1_SHIFT          (16)
#define RF_RBB_GAIN_INDEX3_GAIN_CTRL10_GC_RBB1_MASK           (0x03 << RF_RBB_GAIN_INDEX3_GAIN_CTRL10_GC_RBB1_SHIFT)
#define RF_RBB_GAIN_INDEX3_GAIN_CTRL9_GC_RBB2_SHIFT           (12)
#define RF_RBB_GAIN_INDEX3_GAIN_CTRL9_GC_RBB2_MASK            (0x07 << RF_RBB_GAIN_INDEX3_GAIN_CTRL9_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX3_GAIN_CTRL9_GC_RBB1_SHIFT           (8)
#define RF_RBB_GAIN_INDEX3_GAIN_CTRL9_GC_RBB1_MASK            (0x03 << RF_RBB_GAIN_INDEX3_GAIN_CTRL9_GC_RBB1_SHIFT)
#define RF_RBB_GAIN_INDEX3_GAIN_CTRL8_GC_RBB2_SHIFT           (4)
#define RF_RBB_GAIN_INDEX3_GAIN_CTRL8_GC_RBB2_MASK            (0x07 << RF_RBB_GAIN_INDEX3_GAIN_CTRL8_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX3_GAIN_CTRL8_GC_RBB1_MASK            (0x03)

#define RF_RBB_GAIN_INDEX4_GAIN_CTRL15_GC_RBB2_SHIFT          (28)
#define RF_RBB_GAIN_INDEX4_GAIN_CTRL15_GC_RBB2_MASK           (0x07 << RF_RBB_GAIN_INDEX4_GAIN_CTRL15_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX4_GAIN_CTRL15_GC_RBB1_SHIFT          (24)
#define RF_RBB_GAIN_INDEX4_GAIN_CTRL15_GC_RBB1_MASK           (0x03 << RF_RBB_GAIN_INDEX4_GAIN_CTRL15_GC_RBB1_SHIFT)
#define RF_RBB_GAIN_INDEX4_GAIN_CTRL14_GC_RBB2_SHIFT          (20)
#define RF_RBB_GAIN_INDEX4_GAIN_CTRL14_GC_RBB2_MASK           (0x07 << RF_RBB_GAIN_INDEX4_GAIN_CTRL14_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX4_GAIN_CTRL14_GC_RBB1_SHIFT          (16)
#define RF_RBB_GAIN_INDEX4_GAIN_CTRL14_GC_RBB1_MASK           (0x03 << RF_RBB_GAIN_INDEX4_GAIN_CTRL14_GC_RBB1_SHIFT)
#define RF_RBB_GAIN_INDEX4_GAIN_CTRL13_GC_RBB2_SHIFT          (12)
#define RF_RBB_GAIN_INDEX4_GAIN_CTRL13_GC_RBB2_MASK           (0x07 << RF_RBB_GAIN_INDEX4_GAIN_CTRL13_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX4_GAIN_CTRL13_GC_RBB1_SHIFT          (8)
#define RF_RBB_GAIN_INDEX4_GAIN_CTRL13_GC_RBB1_MASK           (0x03 << RF_RBB_GAIN_INDEX4_GAIN_CTRL13_GC_RBB1_SHIFT)
#define RF_RBB_GAIN_INDEX4_GAIN_CTRL12_GC_RBB2_SHIFT          (4)
#define RF_RBB_GAIN_INDEX4_GAIN_CTRL12_GC_RBB2_MASK           (0x07 << RF_RBB_GAIN_INDEX4_GAIN_CTRL12_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX4_GAIN_CTRL12_GC_RBB1_MASK           (0x03)

#define RF_RBB_GAIN_INDEX5_GAIN_CTRL16_GC_RBB2_SHIFT          (4)
#define RF_RBB_GAIN_INDEX5_GAIN_CTRL16_GC_RBB2_MASK           (0x07 << RF_RBB_GAIN_INDEX5_GAIN_CTRL16_GC_RBB2_SHIFT)
#define RF_RBB_GAIN_INDEX5_GAIN_CTRL16_GC_RBB1_MASK           (0x03)

#define RF_TBB_GAIN_INDEX1_GAIN_CTRL1_GC_TBB_BOOST_SHIFT      (30)
#define RF_TBB_GAIN_INDEX1_GAIN_CTRL1_GC_TBB_BOOST_MASK       (0x03 << RF_TBB_GAIN_INDEX1_GAIN_CTRL1_GC_TBB_BOOST_SHIFT)
#define RF_TBB_GAIN_INDEX1_GAIN_CTRL1_DAC_BIAS_SEL_SHIFT      (28)
#define RF_TBB_GAIN_INDEX1_GAIN_CTRL1_DAC_BIAS_SEL_MASK       (0x03 << RF_TBB_GAIN_INDEX1_GAIN_CTRL1_DAC_BIAS_SEL_SHIFT)
#define RF_TBB_GAIN_INDEX1_GAIN_CTRL1_GC_TMX_SHIFT            (24)
#define RF_TBB_GAIN_INDEX1_GAIN_CTRL1_GC_TMX_MASK             (0x07 << RF_TBB_GAIN_INDEX1_GAIN_CTRL1_GC_TMX_SHIFT)
#define RF_TBB_GAIN_INDEX1_GAIN_CTRL1_GC_TBB_SHIFT            (16)
#define RF_TBB_GAIN_INDEX1_GAIN_CTRL1_GC_TBB_MASK             (0x1f << RF_TBB_GAIN_INDEX1_GAIN_CTRL1_GC_TBB_SHIFT)
#define RF_TBB_GAIN_INDEX1_GAIN_CTRL0_GC_TBB_BOOST_SHIFT      (14)
#define RF_TBB_GAIN_INDEX1_GAIN_CTRL0_GC_TBB_BOOST_MASK       (0x03 << RF_TBB_GAIN_INDEX1_GAIN_CTRL0_GC_TBB_BOOST_SHIFT)
#define RF_TBB_GAIN_INDEX1_GAIN_CTRL0_DAC_BIAS_SEL_SHIFT      (12)
#define RF_TBB_GAIN_INDEX1_GAIN_CTRL0_DAC_BIAS_SEL_MASK       (0x03 << RF_TBB_GAIN_INDEX1_GAIN_CTRL0_DAC_BIAS_SEL_SHIFT)
#define RF_TBB_GAIN_INDEX1_GAIN_CTRL0_GC_TMX_SHIFT            (8)
#define RF_TBB_GAIN_INDEX1_GAIN_CTRL0_GC_TMX_MASK             (0x07 << RF_TBB_GAIN_INDEX1_GAIN_CTRL0_GC_TMX_SHIFT)
#define RF_TBB_GAIN_INDEX1_GAIN_CTRL0_GC_TBB_MASK             (0x1f)

#define RF_TBB_GAIN_INDEX2_GAIN_CTRL3_GC_TBB_BOOST_SHIFT      (30)
#define RF_TBB_GAIN_INDEX2_GAIN_CTRL3_GC_TBB_BOOST_MASK       (0x03 << RF_TBB_GAIN_INDEX2_GAIN_CTRL3_GC_TBB_BOOST_SHIFT)
#define RF_TBB_GAIN_INDEX2_GAIN_CTRL3_DAC_BIAS_SEL_SHIFT      (28)
#define RF_TBB_GAIN_INDEX2_GAIN_CTRL3_DAC_BIAS_SEL_MASK       (0x03 << RF_TBB_GAIN_INDEX2_GAIN_CTRL3_DAC_BIAS_SEL_SHIFT)
#define RF_TBB_GAIN_INDEX2_GAIN_CTRL3_GC_TMX_SHIFT            (24)
#define RF_TBB_GAIN_INDEX2_GAIN_CTRL3_GC_TMX_MASK             (0x07 << RF_TBB_GAIN_INDEX2_GAIN_CTRL3_GC_TMX_SHIFT)
#define RF_TBB_GAIN_INDEX2_GAIN_CTRL3_GC_TBB_SHIFT            (16)
#define RF_TBB_GAIN_INDEX2_GAIN_CTRL3_GC_TBB_MASK             (0x1f << RF_TBB_GAIN_INDEX2_GAIN_CTRL3_GC_TBB_SHIFT)
#define RF_TBB_GAIN_INDEX2_GAIN_CTRL2_GC_TBB_BOOST_SHIFT      (14)
#define RF_TBB_GAIN_INDEX2_GAIN_CTRL2_GC_TBB_BOOST_MASK       (0x03 << RF_TBB_GAIN_INDEX2_GAIN_CTRL2_GC_TBB_BOOST_SHIFT)
#define RF_TBB_GAIN_INDEX2_GAIN_CTRL2_DAC_BIAS_SEL_SHIFT      (12)
#define RF_TBB_GAIN_INDEX2_GAIN_CTRL2_DAC_BIAS_SEL_MASK       (0x03 << RF_TBB_GAIN_INDEX2_GAIN_CTRL2_DAC_BIAS_SEL_SHIFT)
#define RF_TBB_GAIN_INDEX2_GAIN_CTRL2_GC_TMX_SHIFT            (8)
#define RF_TBB_GAIN_INDEX2_GAIN_CTRL2_GC_TMX_MASK             (0x07 << RF_TBB_GAIN_INDEX2_GAIN_CTRL2_GC_TMX_SHIFT)
#define RF_TBB_GAIN_INDEX2_GAIN_CTRL2_GC_TBB_MASK             (0x1f)

#define RF_TBB_GAIN_INDEX3_GAIN_CTRL5_GC_TBB_BOOST_SHIFT      (30)
#define RF_TBB_GAIN_INDEX3_GAIN_CTRL5_GC_TBB_BOOST_MASK       (0x03 << RF_TBB_GAIN_INDEX3_GAIN_CTRL5_GC_TBB_BOOST_SHIFT)
#define RF_TBB_GAIN_INDEX3_GAIN_CTRL5_DAC_BIAS_SEL_SHIFT      (28)
#define RF_TBB_GAIN_INDEX3_GAIN_CTRL5_DAC_BIAS_SEL_MASK       (0x03 << RF_TBB_GAIN_INDEX3_GAIN_CTRL5_DAC_BIAS_SEL_SHIFT)
#define RF_TBB_GAIN_INDEX3_GAIN_CTRL5_GC_TMX_SHIFT            (24)
#define RF_TBB_GAIN_INDEX3_GAIN_CTRL5_GC_TMX_MASK             (0x07 << RF_TBB_GAIN_INDEX3_GAIN_CTRL5_GC_TMX_SHIFT)
#define RF_TBB_GAIN_INDEX3_GAIN_CTRL5_GC_TBB_SHIFT            (16)
#define RF_TBB_GAIN_INDEX3_GAIN_CTRL5_GC_TBB_MASK             (0x1f << RF_TBB_GAIN_INDEX3_GAIN_CTRL5_GC_TBB_SHIFT)
#define RF_TBB_GAIN_INDEX3_GAIN_CTRL4_GC_TBB_BOOST_SHIFT      (14)
#define RF_TBB_GAIN_INDEX3_GAIN_CTRL4_GC_TBB_BOOST_MASK       (0x03 << RF_TBB_GAIN_INDEX3_GAIN_CTRL4_GC_TBB_BOOST_SHIFT)
#define RF_TBB_GAIN_INDEX3_GAIN_CTRL4_DAC_BIAS_SEL_SHIFT      (12)
#define RF_TBB_GAIN_INDEX3_GAIN_CTRL4_DAC_BIAS_SEL_MASK       (0x03 << RF_TBB_GAIN_INDEX3_GAIN_CTRL4_DAC_BIAS_SEL_SHIFT)
#define RF_TBB_GAIN_INDEX3_GAIN_CTRL4_GC_TMX_SHIFT            (8)
#define RF_TBB_GAIN_INDEX3_GAIN_CTRL4_GC_TMX_MASK             (0x07 << RF_TBB_GAIN_INDEX3_GAIN_CTRL4_GC_TMX_SHIFT)
#define RF_TBB_GAIN_INDEX3_GAIN_CTRL4_GC_TBB_MASK             (0x1f)

#define RF_TBB_GAIN_INDEX4_GAIN_CTRL7_GC_TBB_BOOST_SHIFT      (30)
#define RF_TBB_GAIN_INDEX4_GAIN_CTRL7_GC_TBB_BOOST_MASK       (0x03 << RF_TBB_GAIN_INDEX4_GAIN_CTRL7_GC_TBB_BOOST_SHIFT)
#define RF_TBB_GAIN_INDEX4_GAIN_CTRL7_DAC_BIAS_SEL_SHIFT      (28)
#define RF_TBB_GAIN_INDEX4_GAIN_CTRL7_DAC_BIAS_SEL_MASK       (0x03 << RF_TBB_GAIN_INDEX4_GAIN_CTRL7_DAC_BIAS_SEL_SHIFT)
#define RF_TBB_GAIN_INDEX4_GAIN_CTRL7_GC_TMX_SHIFT            (24)
#define RF_TBB_GAIN_INDEX4_GAIN_CTRL7_GC_TMX_MASK             (0x07 << RF_TBB_GAIN_INDEX4_GAIN_CTRL7_GC_TMX_SHIFT)
#define RF_TBB_GAIN_INDEX4_GAIN_CTRL7_GC_TBB_SHIFT            (16)
#define RF_TBB_GAIN_INDEX4_GAIN_CTRL7_GC_TBB_MASK             (0x1f << RF_TBB_GAIN_INDEX4_GAIN_CTRL7_GC_TBB_SHIFT)
#define RF_TBB_GAIN_INDEX4_GAIN_CTRL6_GC_TBB_BOOST_SHIFT      (14)
#define RF_TBB_GAIN_INDEX4_GAIN_CTRL6_GC_TBB_BOOST_MASK       (0x03 << RF_TBB_GAIN_INDEX4_GAIN_CTRL6_GC_TBB_BOOST_SHIFT)
#define RF_TBB_GAIN_INDEX4_GAIN_CTRL6_DAC_BIAS_SEL_SHIFT      (12)
#define RF_TBB_GAIN_INDEX4_GAIN_CTRL6_DAC_BIAS_SEL_MASK       (0x03 << RF_TBB_GAIN_INDEX4_GAIN_CTRL6_DAC_BIAS_SEL_SHIFT)
#define RF_TBB_GAIN_INDEX4_GAIN_CTRL6_GC_TMX_SHIFT            (8)
#define RF_TBB_GAIN_INDEX4_GAIN_CTRL6_GC_TMX_MASK             (0x07 << RF_TBB_GAIN_INDEX4_GAIN_CTRL6_GC_TMX_SHIFT)
#define RF_TBB_GAIN_INDEX4_GAIN_CTRL6_GC_TBB_MASK             (0x1f)

#define RF_PA_REG_CTRL_HW1_PA_VBCAS_11N_SHIFT                 (20)
#define RF_PA_REG_CTRL_HW1_PA_VBCAS_11N_MASK                  (0x07 << RF_PA_REG_CTRL_HW1_PA_VBCAS_11N_SHIFT)
#define RF_PA_REG_CTRL_HW1_PA_VBCORE_11N_SHIFT                (16)
#define RF_PA_REG_CTRL_HW1_PA_VBCORE_11N_MASK                 (0x0f << RF_PA_REG_CTRL_HW1_PA_VBCORE_11N_SHIFT)
#define RF_PA_REG_CTRL_HW1_PA_IET_11N_SHIFT                   (12)
#define RF_PA_REG_CTRL_HW1_PA_IET_11N_MASK                    (0x0f << RF_PA_REG_CTRL_HW1_PA_IET_11N_SHIFT)

#define RF_PA_REG_CTRL_HW2_PA_VBCAS_11B_SHIFT                 (20)
#define RF_PA_REG_CTRL_HW2_PA_VBCAS_11B_MASK                  (0x07 << RF_PA_REG_CTRL_HW2_PA_VBCAS_11B_SHIFT)
#define RF_PA_REG_CTRL_HW2_PA_VBCORE_11B_SHIFT                (16)
#define RF_PA_REG_CTRL_HW2_PA_VBCORE_11B_MASK                 (0x0f << RF_PA_REG_CTRL_HW2_PA_VBCORE_11B_SHIFT)
#define RF_PA_REG_CTRL_HW2_PA_IET_11B_SHIFT                   (12)
#define RF_PA_REG_CTRL_HW2_PA_IET_11B_MASK                    (0x0f << RF_PA_REG_CTRL_HW2_PA_IET_11B_SHIFT)
#define RF_PA_REG_CTRL_HW2_PA_VBCAS_11G_SHIFT                 (8)
#define RF_PA_REG_CTRL_HW2_PA_VBCAS_11G_MASK                  (0x07 << RF_PA_REG_CTRL_HW2_PA_VBCAS_11G_SHIFT)
#define RF_PA_REG_CTRL_HW2_PA_VBCORE_11G_SHIFT                (4)
#define RF_PA_REG_CTRL_HW2_PA_VBCORE_11G_MASK                 (0x0f << RF_PA_REG_CTRL_HW2_PA_VBCORE_11G_SHIFT)
#define RF_PA_REG_CTRL_HW2_PA_IET_11G_MASK                    (0x0f)

#define RF_PA_REG_WIFI_CTRL_HW_PA_IB_FIX_WIFI                 (1 << 16)
#define RF_PA_REG_WIFI_CTRL_HW_PA_ETB_EN_WIFI                 (1 << 8)
#define RF_PA_REG_WIFI_CTRL_HW_PA_HALF_ON_WIFI                (1 << 0)

#define RF_ADDA_REG_CTRL_HW_ADDA_LDO_DVDD_SEL_TX_SHIFT        (4)
#define RF_ADDA_REG_CTRL_HW_ADDA_LDO_DVDD_SEL_TX_MASK         (0x07 << RF_ADDA_REG_CTRL_HW_ADDA_LDO_DVDD_SEL_TX_SHIFT)
#define RF_ADDA_REG_CTRL_HW_ADDA_LDO_DVDD_SEL_RX_MASK         (0x07)

#define RF_LO_REG_CTRL_HW1_LO_LF_R4_TX_SHIFT                  (24)
#define RF_LO_REG_CTRL_HW1_LO_LF_R4_TX_MASK                   (0x03 << RF_LO_REG_CTRL_HW1_LO_LF_R4_TX_SHIFT)
#define RF_LO_REG_CTRL_HW1_LO_LF_R4_RX_SHIFT                  (20)
#define RF_LO_REG_CTRL_HW1_LO_LF_R4_RX_MASK                   (0x03 << RF_LO_REG_CTRL_HW1_LO_LF_R4_RX_SHIFT)
#define RF_LO_REG_CTRL_HW1_LO_LF_RZ_TX_SHIFT                  (16)
#define RF_LO_REG_CTRL_HW1_LO_LF_RZ_TX_MASK                   (0x03 << RF_LO_REG_CTRL_HW1_LO_LF_RZ_TX_SHIFT)
#define RF_LO_REG_CTRL_HW1_LO_LF_RZ_RX_SHIFT                  (12)
#define RF_LO_REG_CTRL_HW1_LO_LF_RZ_RX_MASK                   (0x03 << RF_LO_REG_CTRL_HW1_LO_LF_RZ_RX_SHIFT)
#define RF_LO_REG_CTRL_HW1_LO_LF_CZ_TX_SHIFT                  (8)
#define RF_LO_REG_CTRL_HW1_LO_LF_CZ_TX_MASK                   (0x03 << RF_LO_REG_CTRL_HW1_LO_LF_CZ_TX_SHIFT)
#define RF_LO_REG_CTRL_HW1_LO_LF_CZ_RX_SHIFT                  (4)
#define RF_LO_REG_CTRL_HW1_LO_LF_CZ_RX_MASK                   (0x03 << RF_LO_REG_CTRL_HW1_LO_LF_CZ_RX_SHIFT)
#define RF_LO_REG_CTRL_HW1_LO_CP_SEL_TX                       (1 << 3)
#define RF_LO_REG_CTRL_HW1_LO_CP_SEL_RX                       (1 << 2)
#define RF_LO_REG_CTRL_HW1_LO_FBDV_HALFSTEP_EN_TX             (1 << 1)
#define RF_LO_REG_CTRL_HW1_LO_FBDV_HALFSTEP_EN_RX             (1 << 0)

#define RF_LO_CAL_CTRL_HW1_LO_VCO_FREQ_CW_2408_SHIFT          (24)
#define RF_LO_CAL_CTRL_HW1_LO_VCO_FREQ_CW_2408_MASK           (0xff << RF_LO_CAL_CTRL_HW1_LO_VCO_FREQ_CW_2408_SHIFT)
#define RF_LO_CAL_CTRL_HW1_LO_VCO_IDAC_CW_2408_SHIFT          (16)
#define RF_LO_CAL_CTRL_HW1_LO_VCO_IDAC_CW_2408_MASK           (0x1f << RF_LO_CAL_CTRL_HW1_LO_VCO_IDAC_CW_2408_SHIFT)
#define RF_LO_CAL_CTRL_HW1_LO_VCO_FREQ_CW_2404_SHIFT          (8)
#define RF_LO_CAL_CTRL_HW1_LO_VCO_FREQ_CW_2404_MASK           (0xff << RF_LO_CAL_CTRL_HW1_LO_VCO_FREQ_CW_2404_SHIFT)
#define RF_LO_CAL_CTRL_HW1_LO_VCO_IDAC_CW_2404_MASK           (0x1f)

#define RF_LO_CAL_CTRL_HW2_LO_VCO_FREQ_CW_2416_SHIFT          (24)
#define RF_LO_CAL_CTRL_HW2_LO_VCO_FREQ_CW_2416_MASK           (0xff << RF_LO_CAL_CTRL_HW2_LO_VCO_FREQ_CW_2416_SHIFT)
#define RF_LO_CAL_CTRL_HW2_LO_VCO_IDAC_CW_2416_SHIFT          (16)
#define RF_LO_CAL_CTRL_HW2_LO_VCO_IDAC_CW_2416_MASK           (0x1f << RF_LO_CAL_CTRL_HW2_LO_VCO_IDAC_CW_2416_SHIFT)
#define RF_LO_CAL_CTRL_HW2_LO_VCO_FREQ_CW_2412_SHIFT          (8)
#define RF_LO_CAL_CTRL_HW2_LO_VCO_FREQ_CW_2412_MASK           (0xff << RF_LO_CAL_CTRL_HW2_LO_VCO_FREQ_CW_2412_SHIFT)
#define RF_LO_CAL_CTRL_HW2_LO_VCO_IDAC_CW_2412_MASK           (0x1f)

#define RF_LO_CAL_CTRL_HW3_LO_VCO_FREQ_CW_2424_SHIFT          (24)
#define RF_LO_CAL_CTRL_HW3_LO_VCO_FREQ_CW_2424_MASK           (0xff << RF_LO_CAL_CTRL_HW3_LO_VCO_FREQ_CW_2424_SHIFT)
#define RF_LO_CAL_CTRL_HW3_LO_VCO_IDAC_CW_2424_SHIFT          (16)
#define RF_LO_CAL_CTRL_HW3_LO_VCO_IDAC_CW_2424_MASK           (0x1f << RF_LO_CAL_CTRL_HW3_LO_VCO_IDAC_CW_2424_SHIFT)
#define RF_LO_CAL_CTRL_HW3_LO_VCO_FREQ_CW_2420_SHIFT          (8)
#define RF_LO_CAL_CTRL_HW3_LO_VCO_FREQ_CW_2420_MASK           (0xff << RF_LO_CAL_CTRL_HW3_LO_VCO_FREQ_CW_2420_SHIFT)
#define RF_LO_CAL_CTRL_HW3_LO_VCO_IDAC_CW_2420_MASK           (0x1f)

#define RF_LO_CAL_CTRL_HW4_LO_VCO_FREQ_CW_2432_SHIFT          (24)
#define RF_LO_CAL_CTRL_HW4_LO_VCO_FREQ_CW_2432_MASK           (0xff << RF_LO_CAL_CTRL_HW4_LO_VCO_FREQ_CW_2432_SHIFT)
#define RF_LO_CAL_CTRL_HW4_LO_VCO_IDAC_CW_2432_SHIFT          (16)
#define RF_LO_CAL_CTRL_HW4_LO_VCO_IDAC_CW_2432_MASK           (0x1f << RF_LO_CAL_CTRL_HW4_LO_VCO_IDAC_CW_2432_SHIFT)
#define RF_LO_CAL_CTRL_HW4_LO_VCO_FREQ_CW_2428_SHIFT          (8)
#define RF_LO_CAL_CTRL_HW4_LO_VCO_FREQ_CW_2428_MASK           (0xff << RF_LO_CAL_CTRL_HW4_LO_VCO_FREQ_CW_2428_SHIFT)
#define RF_LO_CAL_CTRL_HW4_LO_VCO_IDAC_CW_2428_MASK           (0x1f)

#define RF_LO_CAL_CTRL_HW5_LO_VCO_FREQ_CW_2440_SHIFT          (24)
#define RF_LO_CAL_CTRL_HW5_LO_VCO_FREQ_CW_2440_MASK           (0xff << RF_LO_CAL_CTRL_HW5_LO_VCO_FREQ_CW_2440_SHIFT)
#define RF_LO_CAL_CTRL_HW5_LO_VCO_IDAC_CW_2440_SHIFT          (16)
#define RF_LO_CAL_CTRL_HW5_LO_VCO_IDAC_CW_2440_MASK           (0x1f << RF_LO_CAL_CTRL_HW5_LO_VCO_IDAC_CW_2440_SHIFT)
#define RF_LO_CAL_CTRL_HW5_LO_VCO_FREQ_CW_2436_SHIFT          (8)
#define RF_LO_CAL_CTRL_HW5_LO_VCO_FREQ_CW_2436_MASK           (0xff << RF_LO_CAL_CTRL_HW5_LO_VCO_FREQ_CW_2436_SHIFT)
#define RF_LO_CAL_CTRL_HW5_LO_VCO_IDAC_CW_2436_MASK           (0x1f)

#define RF_LO_CAL_CTRL_HW6_LO_VCO_FREQ_CW_2448_SHIFT          (24)
#define RF_LO_CAL_CTRL_HW6_LO_VCO_FREQ_CW_2448_MASK           (0xff << RF_LO_CAL_CTRL_HW6_LO_VCO_FREQ_CW_2448_SHIFT)
#define RF_LO_CAL_CTRL_HW6_LO_VCO_IDAC_CW_2448_SHIFT          (16)
#define RF_LO_CAL_CTRL_HW6_LO_VCO_IDAC_CW_2448_MASK           (0x1f << RF_LO_CAL_CTRL_HW6_LO_VCO_IDAC_CW_2448_SHIFT)
#define RF_LO_CAL_CTRL_HW6_LO_VCO_FREQ_CW_2444_SHIFT          (8)
#define RF_LO_CAL_CTRL_HW6_LO_VCO_FREQ_CW_2444_MASK           (0xff << RF_LO_CAL_CTRL_HW6_LO_VCO_FREQ_CW_2444_SHIFT)
#define RF_LO_CAL_CTRL_HW6_LO_VCO_IDAC_CW_2444_MASK           (0x1f)

#define RF_LO_CAL_CTRL_HW7_LO_VCO_FREQ_CW_2456_SHIFT          (24)
#define RF_LO_CAL_CTRL_HW7_LO_VCO_FREQ_CW_2456_MASK           (0xff << RF_LO_CAL_CTRL_HW7_LO_VCO_FREQ_CW_2456_SHIFT)
#define RF_LO_CAL_CTRL_HW7_LO_VCO_IDAC_CW_2456_SHIFT          (16)
#define RF_LO_CAL_CTRL_HW7_LO_VCO_IDAC_CW_2456_MASK           (0x1f << RF_LO_CAL_CTRL_HW7_LO_VCO_IDAC_CW_2456_SHIFT)
#define RF_LO_CAL_CTRL_HW7_LO_VCO_FREQ_CW_2452_SHIFT          (8)
#define RF_LO_CAL_CTRL_HW7_LO_VCO_FREQ_CW_2452_MASK           (0xff << RF_LO_CAL_CTRL_HW7_LO_VCO_FREQ_CW_2452_SHIFT)
#define RF_LO_CAL_CTRL_HW7_LO_VCO_IDAC_CW_2452_MASK           (0x1f)

#define RF_LO_CAL_CTRL_HW8_LO_VCO_FREQ_CW_2464_SHIFT          (24)
#define RF_LO_CAL_CTRL_HW8_LO_VCO_FREQ_CW_2464_MASK           (0xff << RF_LO_CAL_CTRL_HW8_LO_VCO_FREQ_CW_2464_SHIFT)
#define RF_LO_CAL_CTRL_HW8_LO_VCO_IDAC_CW_2464_SHIFT          (16)
#define RF_LO_CAL_CTRL_HW8_LO_VCO_IDAC_CW_2464_MASK           (0x1f << RF_LO_CAL_CTRL_HW8_LO_VCO_IDAC_CW_2464_SHIFT)
#define RF_LO_CAL_CTRL_HW8_LO_VCO_FREQ_CW_2460_SHIFT          (8)
#define RF_LO_CAL_CTRL_HW8_LO_VCO_FREQ_CW_2460_MASK           (0xff << RF_LO_CAL_CTRL_HW8_LO_VCO_FREQ_CW_2460_SHIFT)
#define RF_LO_CAL_CTRL_HW8_LO_VCO_IDAC_CW_2460_MASK           (0x1f)

#define RF_LO_CAL_CTRL_HW9_LO_VCO_FREQ_CW_2472_SHIFT          (24)
#define RF_LO_CAL_CTRL_HW9_LO_VCO_FREQ_CW_2472_MASK           (0xff << RF_LO_CAL_CTRL_HW9_LO_VCO_FREQ_CW_2472_SHIFT)
#define RF_LO_CAL_CTRL_HW9_LO_VCO_IDAC_CW_2472_SHIFT          (16)
#define RF_LO_CAL_CTRL_HW9_LO_VCO_IDAC_CW_2472_MASK           (0x1f << RF_LO_CAL_CTRL_HW9_LO_VCO_IDAC_CW_2472_SHIFT)
#define RF_LO_CAL_CTRL_HW9_LO_VCO_FREQ_CW_2468_SHIFT          (8)
#define RF_LO_CAL_CTRL_HW9_LO_VCO_FREQ_CW_2468_MASK           (0xff << RF_LO_CAL_CTRL_HW9_LO_VCO_FREQ_CW_2468_SHIFT)
#define RF_LO_CAL_CTRL_HW9_LO_VCO_IDAC_CW_2468_MASK           (0x1f)

#define RF_LO_CAL_CTRL_HW10_LO_VCO_FREQ_CW_2480_SHIFT         (24)
#define RF_LO_CAL_CTRL_HW10_LO_VCO_FREQ_CW_2480_MASK          (0xff << RF_LO_CAL_CTRL_HW10_LO_VCO_FREQ_CW_2480_SHIFT)
#define RF_LO_CAL_CTRL_HW10_LO_VCO_IDAC_CW_2480_SHIFT         (16)
#define RF_LO_CAL_CTRL_HW10_LO_VCO_IDAC_CW_2480_MASK          (0x1f << RF_LO_CAL_CTRL_HW10_LO_VCO_IDAC_CW_2480_SHIFT)
#define RF_LO_CAL_CTRL_HW10_LO_VCO_FREQ_CW_2476_SHIFT         (8)
#define RF_LO_CAL_CTRL_HW10_LO_VCO_FREQ_CW_2476_MASK          (0xff << RF_LO_CAL_CTRL_HW10_LO_VCO_FREQ_CW_2476_SHIFT)
#define RF_LO_CAL_CTRL_HW10_LO_VCO_IDAC_CW_2476_MASK          (0x1f)

#define RF_LO_CAL_CTRL_HW11_LO_VCO_FREQ_CW_2484_SHIFT         (8)
#define RF_LO_CAL_CTRL_HW11_LO_VCO_FREQ_CW_2484_MASK          (0xff << RF_LO_CAL_CTRL_HW11_LO_VCO_FREQ_CW_2484_SHIFT)
#define RF_LO_CAL_CTRL_HW11_LO_VCO_IDAC_CW_2484_MASK          (0x1f)

#define RF_ROSDAC_CTRL_HW1_ROSDAC_Q_GC1_SHIFT                 (24)
#define RF_ROSDAC_CTRL_HW1_ROSDAC_Q_GC1_MASK                  (0x3f << RF_ROSDAC_CTRL_HW1_ROSDAC_Q_GC1_SHIFT)
#define RF_ROSDAC_CTRL_HW1_ROSDAC_I_GC1_SHIFT                 (16)
#define RF_ROSDAC_CTRL_HW1_ROSDAC_I_GC1_MASK                  (0x3f << RF_ROSDAC_CTRL_HW1_ROSDAC_I_GC1_SHIFT)
#define RF_ROSDAC_CTRL_HW1_ROSDAC_Q_GC0_SHIFT                 (8)
#define RF_ROSDAC_CTRL_HW1_ROSDAC_Q_GC0_MASK                  (0x3f << RF_ROSDAC_CTRL_HW1_ROSDAC_Q_GC0_SHIFT)
#define RF_ROSDAC_CTRL_HW1_ROSDAC_I_GC0_MASK                  (0x3f)

#define RF_ROSDAC_CTRL_HW2_ROSDAC_Q_GC3_SHIFT                 (24)
#define RF_ROSDAC_CTRL_HW2_ROSDAC_Q_GC3_MASK                  (0x3f << RF_ROSDAC_CTRL_HW2_ROSDAC_Q_GC3_SHIFT)
#define RF_ROSDAC_CTRL_HW2_ROSDAC_I_GC3_SHIFT                 (16)
#define RF_ROSDAC_CTRL_HW2_ROSDAC_I_GC3_MASK                  (0x3f << RF_ROSDAC_CTRL_HW2_ROSDAC_I_GC3_SHIFT)
#define RF_ROSDAC_CTRL_HW2_ROSDAC_Q_GC2_SHIFT                 (8)
#define RF_ROSDAC_CTRL_HW2_ROSDAC_Q_GC2_MASK                  (0x3f << RF_ROSDAC_CTRL_HW2_ROSDAC_Q_GC2_SHIFT)
#define RF_ROSDAC_CTRL_HW2_ROSDAC_I_GC2_MASK                  (0x3f)

#define RF_RXIQ_CTRL_HW1_RX_IQ_GAIN_COMP_GC0_SHIFT            (16)
#define RF_RXIQ_CTRL_HW1_RX_IQ_GAIN_COMP_GC0_MASK             (0x7ff << RF_RXIQ_CTRL_HW1_RX_IQ_GAIN_COMP_GC0_SHIFT)
#define RF_RXIQ_CTRL_HW1_RX_IQ_PHASE_COMP_GC0_MASK            (0x3ff)

#define RF_RXIQ_CTRL_HW2_RX_IQ_GAIN_COMP_GC1_SHIFT            (16)
#define RF_RXIQ_CTRL_HW2_RX_IQ_GAIN_COMP_GC1_MASK             (0x7ff << RF_RXIQ_CTRL_HW2_RX_IQ_GAIN_COMP_GC1_SHIFT)
#define RF_RXIQ_CTRL_HW2_RX_IQ_PHASE_COMP_GC1_MASK            (0x3ff)

#define RF_RXIQ_CTRL_HW3_RX_IQ_GAIN_COMP_GC2_SHIFT            (16)
#define RF_RXIQ_CTRL_HW3_RX_IQ_GAIN_COMP_GC2_MASK             (0x7ff << RF_RXIQ_CTRL_HW3_RX_IQ_GAIN_COMP_GC2_SHIFT)
#define RF_RXIQ_CTRL_HW3_RX_IQ_PHASE_COMP_GC2_MASK            (0x3ff)

#define RF_RXIQ_CTRL_HW4_RX_IQ_GAIN_COMP_GC3_SHIFT            (16)
#define RF_RXIQ_CTRL_HW4_RX_IQ_GAIN_COMP_GC3_MASK             (0x7ff << RF_RXIQ_CTRL_HW4_RX_IQ_GAIN_COMP_GC3_SHIFT)
#define RF_RXIQ_CTRL_HW4_RX_IQ_PHASE_COMP_GC3_MASK            (0x3ff)

#define RF_TOSDAC_CTRL_HW1_TBB_TOSDAC_Q_GC1_SHIFT             (24)
#define RF_TOSDAC_CTRL_HW1_TBB_TOSDAC_Q_GC1_MASK              (0x3f << RF_TOSDAC_CTRL_HW1_TBB_TOSDAC_Q_GC1_SHIFT)
#define RF_TOSDAC_CTRL_HW1_TBB_TOSDAC_I_GC1_SHIFT             (16)
#define RF_TOSDAC_CTRL_HW1_TBB_TOSDAC_I_GC1_MASK              (0x3f << RF_TOSDAC_CTRL_HW1_TBB_TOSDAC_I_GC1_SHIFT)
#define RF_TOSDAC_CTRL_HW1_TBB_TOSDAC_Q_GC0_SHIFT             (8)
#define RF_TOSDAC_CTRL_HW1_TBB_TOSDAC_Q_GC0_MASK              (0x3f << RF_TOSDAC_CTRL_HW1_TBB_TOSDAC_Q_GC0_SHIFT)
#define RF_TOSDAC_CTRL_HW1_TBB_TOSDAC_I_GC0_MASK              (0x3f)

#define RF_TOSDAC_CTRL_HW2_TBB_TOSDAC_Q_GC3_SHIFT             (24)
#define RF_TOSDAC_CTRL_HW2_TBB_TOSDAC_Q_GC3_MASK              (0x3f << RF_TOSDAC_CTRL_HW2_TBB_TOSDAC_Q_GC3_SHIFT)
#define RF_TOSDAC_CTRL_HW2_TBB_TOSDAC_I_GC3_SHIFT             (16)
#define RF_TOSDAC_CTRL_HW2_TBB_TOSDAC_I_GC3_MASK              (0x3f << RF_TOSDAC_CTRL_HW2_TBB_TOSDAC_I_GC3_SHIFT)
#define RF_TOSDAC_CTRL_HW2_TBB_TOSDAC_Q_GC2_SHIFT             (8)
#define RF_TOSDAC_CTRL_HW2_TBB_TOSDAC_Q_GC2_MASK              (0x3f << RF_TOSDAC_CTRL_HW2_TBB_TOSDAC_Q_GC2_SHIFT)
#define RF_TOSDAC_CTRL_HW2_TBB_TOSDAC_I_GC2_MASK              (0x3f)

#define RF_TOSDAC_CTRL_HW3_TBB_TOSDAC_Q_GC5_SHIFT             (24)
#define RF_TOSDAC_CTRL_HW3_TBB_TOSDAC_Q_GC5_MASK              (0x3f << RF_TOSDAC_CTRL_HW3_TBB_TOSDAC_Q_GC5_SHIFT)
#define RF_TOSDAC_CTRL_HW3_TBB_TOSDAC_I_GC5_SHIFT             (16)
#define RF_TOSDAC_CTRL_HW3_TBB_TOSDAC_I_GC5_MASK              (0x3f << RF_TOSDAC_CTRL_HW3_TBB_TOSDAC_I_GC5_SHIFT)
#define RF_TOSDAC_CTRL_HW3_TBB_TOSDAC_Q_GC4_SHIFT             (8)
#define RF_TOSDAC_CTRL_HW3_TBB_TOSDAC_Q_GC4_MASK              (0x3f << RF_TOSDAC_CTRL_HW3_TBB_TOSDAC_Q_GC4_SHIFT)
#define RF_TOSDAC_CTRL_HW3_TBB_TOSDAC_I_GC4_MASK              (0x3f)

#define RF_TOSDAC_CTRL_HW4_TBB_TOSDAC_Q_GC7_SHIFT             (24)
#define RF_TOSDAC_CTRL_HW4_TBB_TOSDAC_Q_GC7_MASK              (0x3f << RF_TOSDAC_CTRL_HW4_TBB_TOSDAC_Q_GC7_SHIFT)
#define RF_TOSDAC_CTRL_HW4_TBB_TOSDAC_I_GC7_SHIFT             (16)
#define RF_TOSDAC_CTRL_HW4_TBB_TOSDAC_I_GC7_MASK              (0x3f << RF_TOSDAC_CTRL_HW4_TBB_TOSDAC_I_GC7_SHIFT)
#define RF_TOSDAC_CTRL_HW4_TBB_TOSDAC_Q_GC6_SHIFT             (8)
#define RF_TOSDAC_CTRL_HW4_TBB_TOSDAC_Q_GC6_MASK              (0x3f << RF_TOSDAC_CTRL_HW4_TBB_TOSDAC_Q_GC6_SHIFT)
#define RF_TOSDAC_CTRL_HW4_TBB_TOSDAC_I_GC6_MASK              (0x3f)

#define RF_TX_IQ_GAIN_HW0_TX_IQ_GAIN_COMP_GC0_SHIFT           (16)
#define RF_TX_IQ_GAIN_HW0_TX_IQ_GAIN_COMP_GC0_MASK            (0x7ff << RF_TX_IQ_GAIN_HW0_TX_IQ_GAIN_COMP_GC0_SHIFT)
#define RF_TX_IQ_GAIN_HW0_TX_IQ_PHASE_COMP_GC0_MASK           (0x3ff)

#define RF_TX_IQ_GAIN_HW1_TX_IQ_GAIN_COMP_GC1_SHIFT           (16)
#define RF_TX_IQ_GAIN_HW1_TX_IQ_GAIN_COMP_GC1_MASK            (0x7ff << RF_TX_IQ_GAIN_HW1_TX_IQ_GAIN_COMP_GC1_SHIFT)
#define RF_TX_IQ_GAIN_HW1_TX_IQ_PHASE_COMP_GC1_MASK           (0x3ff)

#define RF_TX_IQ_GAIN_HW2_TX_IQ_GAIN_COMP_GC2_SHIFT           (16)
#define RF_TX_IQ_GAIN_HW2_TX_IQ_GAIN_COMP_GC2_MASK            (0x7ff << RF_TX_IQ_GAIN_HW2_TX_IQ_GAIN_COMP_GC2_SHIFT)
#define RF_TX_IQ_GAIN_HW2_TX_IQ_PHASE_COMP_GC2_MASK           (0x3ff)

#define RF_TX_IQ_GAIN_HW3_TX_IQ_GAIN_COMP_GC3_SHIFT           (16)
#define RF_TX_IQ_GAIN_HW3_TX_IQ_GAIN_COMP_GC3_MASK            (0x7ff << RF_TX_IQ_GAIN_HW3_TX_IQ_GAIN_COMP_GC3_SHIFT)
#define RF_TX_IQ_GAIN_HW3_TX_IQ_PHASE_COMP_GC3_MASK           (0x3ff)

#define RF_TX_IQ_GAIN_HW4_TX_IQ_GAIN_COMP_GC4_SHIFT           (16)
#define RF_TX_IQ_GAIN_HW4_TX_IQ_GAIN_COMP_GC4_MASK            (0x7ff << RF_TX_IQ_GAIN_HW4_TX_IQ_GAIN_COMP_GC4_SHIFT)
#define RF_TX_IQ_GAIN_HW4_TX_IQ_PHASE_COMP_GC4_MASK           (0x3ff)

#define RF_TX_IQ_GAIN_HW5_TX_IQ_GAIN_COMP_GC5_SHIFT           (16)
#define RF_TX_IQ_GAIN_HW5_TX_IQ_GAIN_COMP_GC5_MASK            (0x7ff << RF_TX_IQ_GAIN_HW5_TX_IQ_GAIN_COMP_GC5_SHIFT)
#define RF_TX_IQ_GAIN_HW5_TX_IQ_PHASE_COMP_GC5_MASK           (0x3ff)

#define RF_TX_IQ_GAIN_HW6_TX_IQ_GAIN_COMP_GC6_SHIFT           (16)
#define RF_TX_IQ_GAIN_HW6_TX_IQ_GAIN_COMP_GC6_MASK            (0x7ff << RF_TX_IQ_GAIN_HW6_TX_IQ_GAIN_COMP_GC6_SHIFT)
#define RF_TX_IQ_GAIN_HW6_TX_IQ_PHASE_COMP_GC6_MASK           (0x3ff)

#define RF_TX_IQ_GAIN_HW7_TX_IQ_GAIN_COMP_GC7_SHIFT           (16)
#define RF_TX_IQ_GAIN_HW7_TX_IQ_GAIN_COMP_GC7_MASK            (0x7ff << RF_TX_IQ_GAIN_HW7_TX_IQ_GAIN_COMP_GC7_SHIFT)
#define RF_TX_IQ_GAIN_HW7_TX_IQ_PHASE_COMP_GC7_MASK           (0x3ff)

#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2484_SHIFT  (26)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2484_MASK   (0x03 << RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2484_SHIFT)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2472_SHIFT  (24)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2472_MASK   (0x03 << RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2472_SHIFT)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2467_SHIFT  (22)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2467_MASK   (0x03 << RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2467_SHIFT)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2462_SHIFT  (20)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2462_MASK   (0x03 << RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2462_SHIFT)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2457_SHIFT  (18)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2457_MASK   (0x03 << RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2457_SHIFT)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2452_SHIFT  (16)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2452_MASK   (0x03 << RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2452_SHIFT)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2447_SHIFT  (14)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2447_MASK   (0x03 << RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2447_SHIFT)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2442_SHIFT  (12)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2442_MASK   (0x03 << RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2442_SHIFT)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2437_SHIFT  (10)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2437_MASK   (0x03 << RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2437_SHIFT)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2432_SHIFT  (8)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2432_MASK   (0x03 << RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2432_SHIFT)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2427_SHIFT  (6)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2427_MASK   (0x03 << RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2427_SHIFT)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2422_SHIFT  (4)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2422_MASK   (0x03 << RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2422_SHIFT)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2417_SHIFT  (2)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2417_MASK   (0x03 << RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2417_SHIFT)
#define RF_LO_SDM_CTRL_HW1_LO_SDM_DITHER_SEL_WLAN_2412_MASK   (0x03)

#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2432_SHIFT   (30)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2432_MASK    (0x03 << RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2432_SHIFT)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2430_SHIFT   (28)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2430_MASK    (0x03 << RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2430_SHIFT)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2428_SHIFT   (26)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2428_MASK    (0x03 << RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2428_SHIFT)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2426_SHIFT   (24)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2426_MASK    (0x03 << RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2426_SHIFT)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2424_SHIFT   (22)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2424_MASK    (0x03 << RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2424_SHIFT)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2422_SHIFT   (20)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2422_MASK    (0x03 << RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2422_SHIFT)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2420_SHIFT   (18)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2420_MASK    (0x03 << RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2420_SHIFT)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2418_SHIFT   (16)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2418_MASK    (0x03 << RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2418_SHIFT)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2416_SHIFT   (14)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2416_MASK    (0x03 << RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2416_SHIFT)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2414_SHIFT   (12)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2414_MASK    (0x03 << RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2414_SHIFT)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2412_SHIFT   (10)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2412_MASK    (0x03 << RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2412_SHIFT)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2410_SHIFT   (8)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2410_MASK    (0x03 << RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2410_SHIFT)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2408_SHIFT   (6)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2408_MASK    (0x03 << RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2408_SHIFT)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2406_SHIFT   (4)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2406_MASK    (0x03 << RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2406_SHIFT)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2404_SHIFT   (2)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2404_MASK    (0x03 << RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2404_SHIFT)
#define RF_LO_SDM_CTRL_HW2_LO_SDM_DITHER_SEL_BLE_2402_MASK    (0x03)

#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2464_SHIFT   (30)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2464_MASK    (0x03 << RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2464_SHIFT)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2462_SHIFT   (28)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2462_MASK    (0x03 << RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2462_SHIFT)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2460_SHIFT   (26)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2460_MASK    (0x03 << RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2460_SHIFT)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2458_SHIFT   (24)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2458_MASK    (0x03 << RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2458_SHIFT)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2456_SHIFT   (22)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2456_MASK    (0x03 << RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2456_SHIFT)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2454_SHIFT   (20)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2454_MASK    (0x03 << RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2454_SHIFT)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2452_SHIFT   (18)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2452_MASK    (0x03 << RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2452_SHIFT)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2450_SHIFT   (16)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2450_MASK    (0x03 << RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2450_SHIFT)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2448_SHIFT   (14)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2448_MASK    (0x03 << RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2448_SHIFT)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2446_SHIFT   (12)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2446_MASK    (0x03 << RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2446_SHIFT)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2444_SHIFT   (10)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2444_MASK    (0x03 << RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2444_SHIFT)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2442_SHIFT   (8)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2442_MASK    (0x03 << RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2442_SHIFT)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2440_SHIFT   (6)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2440_MASK    (0x03 << RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2440_SHIFT)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2438_SHIFT   (4)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2438_MASK    (0x03 << RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2438_SHIFT)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2436_SHIFT   (2)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2436_MASK    (0x03 << RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2436_SHIFT)
#define RF_LO_SDM_CTRL_HW3_LO_SDM_DITHER_SEL_BLE_2434_MASK    (0x03)

#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_TX_SHIFT     (16)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_TX_MASK      (0x03 << RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_TX_SHIFT)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2480_SHIFT   (14)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2480_MASK    (0x03 << RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2480_SHIFT)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2478_SHIFT   (12)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2478_MASK    (0x03 << RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2478_SHIFT)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2476_SHIFT   (10)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2476_MASK    (0x03 << RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2476_SHIFT)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2474_SHIFT   (8)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2474_MASK    (0x03 << RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2474_SHIFT)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2472_SHIFT   (6)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2472_MASK    (0x03 << RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2472_SHIFT)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2470_SHIFT   (4)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2470_MASK    (0x03 << RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2470_SHIFT)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2468_SHIFT   (2)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2468_MASK    (0x03 << RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2468_SHIFT)
#define RF_LO_SDM_CTRL_HW4_LO_SDM_DITHER_SEL_BLE_2466_MASK    (0x03)

#define RF_LO_SDM_CTRL_HW5_LO_SDM_BYPASS_MODE_SHIFT           (12)
#define RF_LO_SDM_CTRL_HW5_LO_SDM_BYPASS_MODE_MASK            (0x3f << RF_LO_SDM_CTRL_HW5_LO_SDM_BYPASS_MODE_SHIFT)
#define RF_LO_SDM_CTRL_HW5_LO_CENTER_FREQ_MHZ_MASK            (0xfff)

#define RF_LO_SDM_CTRL_HW6_LO_SDMIN_CENTER_MASK               (0x1fffffff)

#define RF_LO_SDM_CTRL_HW7_LO_SDMIN_1M_MASK                   (0xfffff)

#define RF_LO_SDM_CTRL_HW8_LO_SDMIN_IF_MASK                   (0xfffff)

#define RF_RBB_BW_CTRL_HW_RBB_BT_MODE_BLE                     (1 << 0)

#define RF_SINGEN_CTRL0_SINGEN_EN                             (1 << 31)
#define RF_SINGEN_CTRL0_SINGEN_CLKDIV_N_SHIFT                 (29)
#define RF_SINGEN_CTRL0_SINGEN_CLKDIV_N_MASK                  (0x03 << RF_SINGEN_CTRL0_SINGEN_CLKDIV_N_SHIFT)
#define RF_SINGEN_CTRL0_SINGEN_UNSIGN_EN                      (1 << 28)
#define RF_SINGEN_CTRL0_SINGEN_INC_STEP0_SHIFT                (16)
#define RF_SINGEN_CTRL0_SINGEN_INC_STEP0_MASK                 (0x3ff << RF_SINGEN_CTRL0_SINGEN_INC_STEP0_SHIFT)
#define RF_SINGEN_CTRL0_SINGEN_INC_STEP1_MASK                 (0x3ff)

#define RF_SINGEN_CTRL1_SINGEN_MODE_I_SHIFT                   (28)
#define RF_SINGEN_CTRL1_SINGEN_MODE_I_MASK                    (0x0f << RF_SINGEN_CTRL1_SINGEN_MODE_I_SHIFT)
#define RF_SINGEN_CTRL1_SINGEN_CLKDIV_I_SHIFT                 (16)
#define RF_SINGEN_CTRL1_SINGEN_CLKDIV_I_MASK                  (0x3ff << RF_SINGEN_CTRL1_SINGEN_CLKDIV_I_SHIFT)
#define RF_SINGEN_CTRL1_SINGEN_MODE_Q_SHIFT                   (12)
#define RF_SINGEN_CTRL1_SINGEN_MODE_Q_MASK                    (0x0f << RF_SINGEN_CTRL1_SINGEN_MODE_Q_SHIFT)
#define RF_SINGEN_CTRL1_SINGEN_CLKDIV_Q_MASK                  (0x3ff)

#define RF_SINGEN_CTRL2_SINGEN_START_ADDR0_I_SHIFT            (22)
#define RF_SINGEN_CTRL2_SINGEN_START_ADDR0_I_MASK             (0x3ff << RF_SINGEN_CTRL2_SINGEN_START_ADDR0_I_SHIFT)
#define RF_SINGEN_CTRL2_SINGEN_START_ADDR1_I_SHIFT            (12)
#define RF_SINGEN_CTRL2_SINGEN_START_ADDR1_I_MASK             (0x3ff << RF_SINGEN_CTRL2_SINGEN_START_ADDR1_I_SHIFT)
#define RF_SINGEN_CTRL2_SINGEN_GAIN_I_MASK                    (0x7ff)

#define RF_SINGEN_CTRL3_SINGEN_START_ADDR0_Q_SHIFT            (22)
#define RF_SINGEN_CTRL3_SINGEN_START_ADDR0_Q_MASK             (0x3ff << RF_SINGEN_CTRL3_SINGEN_START_ADDR0_Q_SHIFT)
#define RF_SINGEN_CTRL3_SINGEN_START_ADDR1_Q_SHIFT            (12)
#define RF_SINGEN_CTRL3_SINGEN_START_ADDR1_Q_MASK             (0x3ff << RF_SINGEN_CTRL3_SINGEN_START_ADDR1_Q_SHIFT)
#define RF_SINGEN_CTRL3_SINGEN_GAIN_Q_MASK                    (0x7ff)

#define RF_SINGEN_CTRL4_SINGEN_FIX_EN_I                       (1 << 28)
#define RF_SINGEN_CTRL4_SINGEN_FIX_I_SHIFT                    (16)
#define RF_SINGEN_CTRL4_SINGEN_FIX_I_MASK                     (0xfff << RF_SINGEN_CTRL4_SINGEN_FIX_I_SHIFT)
#define RF_SINGEN_CTRL4_SINGEN_FIX_EN_Q                       (1 << 12)
#define RF_SINGEN_CTRL4_SINGEN_FIX_Q_MASK                     (0xfff)

#define RF_RFIF_DFE_CTRL0_TEST_SEL_SHIFT                      (28)
#define RF_RFIF_DFE_CTRL0_TEST_SEL_MASK                       (0x0f << RF_RFIF_DFE_CTRL0_TEST_SEL_SHIFT)
#define RF_RFIF_DFE_CTRL0_BBMODE_4S_EN                        (1 << 27)
#define RF_RFIF_DFE_CTRL0_BBMODE_4S                           (1 << 26)
#define RF_RFIF_DFE_CTRL0_WIFIMODE_4S_EN                      (1 << 25)
#define RF_RFIF_DFE_CTRL0_WIFIMODE_4S_SHIFT                   (23)
#define RF_RFIF_DFE_CTRL0_WIFIMODE_4S_MASK                    (0x03 << RF_RFIF_DFE_CTRL0_WIFIMODE_4S_SHIFT)
#define RF_RFIF_DFE_CTRL0_RF_CH_IND_BLE_4S_EN                 (1 << 22)
#define RF_RFIF_DFE_CTRL0_RF_CH_IND_BLE_4S_SHIFT              (15)
#define RF_RFIF_DFE_CTRL0_RF_CH_IND_BLE_4S_MASK               (0x7f << RF_RFIF_DFE_CTRL0_RF_CH_IND_BLE_4S_SHIFT)
#define RF_RFIF_DFE_CTRL0_PAD_DAC_CLKOUT_INV_EN               (1 << 14)
#define RF_RFIF_DFE_CTRL0_PAD_ADC_CLKOUT_INV_EN               (1 << 13)
#define RF_RFIF_DFE_CTRL0_TX_TEST_SEL_SHIFT                   (11)
#define RF_RFIF_DFE_CTRL0_TX_TEST_SEL_MASK                    (0x03 << RF_RFIF_DFE_CTRL0_TX_TEST_SEL_SHIFT)
#define RF_RFIF_DFE_CTRL0_RX_TEST_SEL_SHIFT                   (9)
#define RF_RFIF_DFE_CTRL0_RX_TEST_SEL_MASK                    (0x03 << RF_RFIF_DFE_CTRL0_RX_TEST_SEL_SHIFT)
#define RF_RFIF_DFE_CTRL0_TX_DFE_EN_4S_EN                     (1 << 8)
#define RF_RFIF_DFE_CTRL0_TX_DFE_EN_4S                        (1 << 7)
#define RF_RFIF_DFE_CTRL0_RX_DFE_EN_4S_EN                     (1 << 6)
#define RF_RFIF_DFE_CTRL0_RX_DFE_EN_4S                        (1 << 5)
#define RF_RFIF_DFE_CTRL0_RFCKG_DAC_AFIFO_INV                 (1 << 4)
#define RF_RFIF_DFE_CTRL0_RFCKG_ADC_CLKOUT_SEL                (1 << 3)
#define RF_RFIF_DFE_CTRL0_RFCKG_ADC_AFIFO_INV                 (1 << 2)
#define RF_RFIF_DFE_CTRL0_RFCKG_TXCLK_4S_ON                   (1 << 1)
#define RF_RFIF_DFE_CTRL0_RFCKG_RXCLK_4S_ON                   (1 << 0)

#define RF_RFIF_DIG_CTRL_RFIF_PPUD_MANAUAL_EN                 (1 << 30)
#define RF_RFIF_DIG_CTRL_RFIF_PPUD_CNT1_SHIFT                 (25)
#define RF_RFIF_DIG_CTRL_RFIF_PPUD_CNT1_MASK                  (0x1f << RF_RFIF_DIG_CTRL_RFIF_PPUD_CNT1_SHIFT)
#define RF_RFIF_DIG_CTRL_RFIF_PPUD_CNT2_SHIFT                 (16)
#define RF_RFIF_DIG_CTRL_RFIF_PPUD_CNT2_MASK                  (0x1ff << RF_RFIF_DIG_CTRL_RFIF_PPUD_CNT2_SHIFT)
#define RF_RFIF_DIG_CTRL_RFIF_INT_LO_UNLOCKED_MASK            (1 << 3)
#define RF_RFIF_DIG_CTRL_RFCKG_RXCLK_DIV2_MODE                (1 << 2)
#define RF_RFIF_DIG_CTRL_TEST_GC_FROM_PAD_EN                  (1 << 1)
#define RF_RFIF_DIG_CTRL_TEST_FROM_PAD_EN                     (1 << 0)

#define RF_SRAM_CTRL0_RF_SRAM_EXT_CLR                         (1 << 19)
#define RF_SRAM_CTRL0_RF_SRAM_SWAP                            (1 << 18)
#define RF_SRAM_CTRL0_RF_SRAM_LINK_MODE_SHIFT                 (16)
#define RF_SRAM_CTRL0_RF_SRAM_LINK_MODE_MASK                  (0x03 << RF_SRAM_CTRL0_RF_SRAM_LINK_MODE_SHIFT)
#define RF_SRAM_CTRL0_RF_SRAM_LINK_DLY_MASK                   (0xffff)

#define RF_SRAM_CTRL1_RF_SRAM_ADC_DONE_CNT_SHIFT              (16)
#define RF_SRAM_CTRL1_RF_SRAM_ADC_DONE_CNT_MASK               (0xffff << RF_SRAM_CTRL1_RF_SRAM_ADC_DONE_CNT_SHIFT)
#define RF_SRAM_CTRL1_RF_SRAM_ADC_STS_CLR                     (1 << 3)
#define RF_SRAM_CTRL1_RF_SRAM_ADC_LOOP_EN                     (1 << 2)
#define RF_SRAM_CTRL1_RF_SRAM_ADC_EN                          (1 << 1)
#define RF_SRAM_CTRL1_RF_SRAM_ADC_DONE                        (1 << 0)

#define RF_SRAM_CTRL2_RF_SRAM_ADC_ADDR_START_SHIFT            (16)
#define RF_SRAM_CTRL2_RF_SRAM_ADC_ADDR_START_MASK             (0xffff << RF_SRAM_CTRL2_RF_SRAM_ADC_ADDR_START_SHIFT)
#define RF_SRAM_CTRL2_RF_SRAM_ADC_ADDR_END_MASK               (0xffff)

#define RF_SRAM_CTRL4_RF_SRAM_DAC_DONE_CNT_SHIFT              (16)
#define RF_SRAM_CTRL4_RF_SRAM_DAC_DONE_CNT_MASK               (0xffff << RF_SRAM_CTRL4_RF_SRAM_DAC_DONE_CNT_SHIFT)
#define RF_SRAM_CTRL4_RF_SRAM_DAC_STS_CLR                     (1 << 3)
#define RF_SRAM_CTRL4_RF_SRAM_DAC_LOOP_EN                     (1 << 2)
#define RF_SRAM_CTRL4_RF_SRAM_DAC_EN                          (1 << 1)
#define RF_SRAM_CTRL4_RF_SRAM_DAC_DONE                        (1 << 0)

#define RF_SRAM_CTRL5_RF_SRAM_DAC_ADDR_START_SHIFT            (16)
#define RF_SRAM_CTRL5_RF_SRAM_DAC_ADDR_START_MASK             (0xffff << RF_SRAM_CTRL5_RF_SRAM_DAC_ADDR_START_SHIFT)
#define RF_SRAM_CTRL5_RF_SRAM_DAC_ADDR_END_MASK               (0xffff)

#define RF_ICAL_CTRL0_RF_ICAL_F_UD_INV_EN                     (1 << 31)
#define RF_ICAL_CTRL0_RF_ICAL_A_UD_INV_EN                     (1 << 30)
#define RF_ICAL_CTRL0_RF_ICAL_F_CNT_N_SHIFT                   (20)
#define RF_ICAL_CTRL0_RF_ICAL_F_CNT_N_MASK                    (0x3ff << RF_ICAL_CTRL0_RF_ICAL_F_CNT_N_SHIFT)
#define RF_ICAL_CTRL0_RF_ICAL_A_CNT_N_SHIFT                   (10)
#define RF_ICAL_CTRL0_RF_ICAL_A_CNT_N_MASK                    (0x3ff << RF_ICAL_CTRL0_RF_ICAL_A_CNT_N_SHIFT)
#define RF_ICAL_CTRL0_RF_ICAL_R_CNT_N_MASK                    (0x3ff)

#define RF_ICAL_CTRL1_RF_ICAL_R_OS_I_SHIFT                    (20)
#define RF_ICAL_CTRL1_RF_ICAL_R_OS_I_MASK                     (0x3ff << RF_ICAL_CTRL1_RF_ICAL_R_OS_I_SHIFT)
#define RF_ICAL_CTRL1_RF_ICAL_R_OS_Q_SHIFT                    (10)
#define RF_ICAL_CTRL1_RF_ICAL_R_OS_Q_MASK                     (0x3ff << RF_ICAL_CTRL1_RF_ICAL_R_OS_Q_SHIFT)
#define RF_ICAL_CTRL1_RF_ICAL_R_AVG_N_MASK                    (0x1f)

#define RF_ICAL_CTRL2_RF_ICAL_PERIOD_N_MASK                   (0xffff)

#define RF_FSM_CTRL0_RF_CH_IND_WIFI_MASK                      (0xfff)

#define RF_FSM_CTRL1_RF_FSM_PU_PA_DLY_N_SHIFT                 (20)
#define RF_FSM_CTRL1_RF_FSM_PU_PA_DLY_N_MASK                  (0x3ff << RF_FSM_CTRL1_RF_FSM_PU_PA_DLY_N_SHIFT)
#define RF_FSM_CTRL1_RF_FSM_LO_RDY_SBCLR                      (1 << 19)
#define RF_FSM_CTRL1_RF_FSM_LO_RDY_4S_1                       (1 << 18)
#define RF_FSM_CTRL1_RF_FSM_LO_RDY_RST                        (1 << 17)
#define RF_FSM_CTRL1_RF_FSM_LO_RDY                            (1 << 16)
#define RF_FSM_CTRL1_RF_FSM_LO_TIME_MASK                      (0xffff)

#define RF_FSM_CTRL2_RF_FSM_DFE_RX_DLY_N_SHIFT                (20)
#define RF_FSM_CTRL2_RF_FSM_DFE_RX_DLY_N_MASK                 (0x3ff << RF_FSM_CTRL2_RF_FSM_DFE_RX_DLY_N_SHIFT)
#define RF_FSM_CTRL2_RF_FSM_DFE_TX_DLY_N_SHIFT                (10)
#define RF_FSM_CTRL2_RF_FSM_DFE_TX_DLY_N_MASK                 (0x3ff << RF_FSM_CTRL2_RF_FSM_DFE_TX_DLY_N_SHIFT)
#define RF_FSM_CTRL2_RF_TRX_BLE_4S_EN                         (1 << 6)
#define RF_FSM_CTRL2_RF_TRX_SW_BLE_4S                         (1 << 5)
#define RF_FSM_CTRL2_RF_TRX_EN_BLE_4S                         (1 << 4)
#define RF_FSM_CTRL2_RF_FSM_ST_DBG_EN                         (1 << 3)
#define RF_FSM_CTRL2_RF_FSM_ST_DBG_MASK                       (0x07)

#define RF_PKDET_CTRL0_PKDET_OUT_MODE                         (1 << 5)
#define RF_PKDET_CTRL0_PKDET_OUT_CNT_EN                       (1 << 4)
#define RF_PKDET_CTRL0_PKDET_OUT_CNT_STS_MASK                 (0x0f)

#define RF_DFE_CTRL_0_TX_DVGA_GAIN_CTRL_HW                    (1 << 31)
#define RF_DFE_CTRL_0_TX_DVGA_GAIN_QDB_SHIFT                  (24)
#define RF_DFE_CTRL_0_TX_DVGA_GAIN_QDB_MASK                   (0x7f << RF_DFE_CTRL_0_TX_DVGA_GAIN_QDB_SHIFT)
#define RF_DFE_CTRL_0_TX_IQC_GAIN_EN                          (1 << 23)
#define RF_DFE_CTRL_0_TX_IQC_GAIN_SHIFT                       (12)
#define RF_DFE_CTRL_0_TX_IQC_GAIN_MASK                        (0x7ff << RF_DFE_CTRL_0_TX_IQC_GAIN_SHIFT)
#define RF_DFE_CTRL_0_TX_IQC_PHASE_EN                         (1 << 10)
#define RF_DFE_CTRL_0_TX_IQC_PHASE_MASK                       (0x3ff)

#define RF_DFE_CTRL_1_TX_DAC_IQ_SWAP                          (1 << 31)
#define RF_DFE_CTRL_1_TX_DAC_DAT_FORMAT                       (1 << 30)
#define RF_DFE_CTRL_1_TX_DAC_OS_Q_SHIFT                       (16)
#define RF_DFE_CTRL_1_TX_DAC_OS_Q_MASK                        (0xfff << RF_DFE_CTRL_1_TX_DAC_OS_Q_SHIFT)
#define RF_DFE_CTRL_1_TX_DAC_OS_I_MASK                        (0xfff)

#define RF_DFE_CTRL_2_RX_ADC_IQ_SWAP                          (1 << 31)
#define RF_DFE_CTRL_2_RX_ADC_DAT_FORMAT                       (1 << 30)
#define RF_DFE_CTRL_2_RX_ADC_LOW_POW_EN                       (1 << 29)
#define RF_DFE_CTRL_2_RX_ADC_DCE_FLT_EN                       (1 << 28)
#define RF_DFE_CTRL_2_RX_ADC_OS_Q_SHIFT                       (16)
#define RF_DFE_CTRL_2_RX_ADC_OS_Q_MASK                        (0x3ff << RF_DFE_CTRL_2_RX_ADC_OS_Q_SHIFT)
#define RF_DFE_CTRL_2_RX_ADC_OS_I_MASK                        (0x3ff)

#define RF_DFE_CTRL_3_RX_ADC_4S_Q_EN                          (1 << 26)
#define RF_DFE_CTRL_3_RX_ADC_4S_Q_VAL_SHIFT                   (16)
#define RF_DFE_CTRL_3_RX_ADC_4S_Q_VAL_MASK                    (0x3ff << RF_DFE_CTRL_3_RX_ADC_4S_Q_VAL_SHIFT)
#define RF_DFE_CTRL_3_RX_ADC_4S_I_EN                          (1 << 10)
#define RF_DFE_CTRL_3_RX_ADC_4S_I_VAL_MASK                    (0x3ff)

#define RF_DFE_CTRL_4_RX_PF_I_EN                              (1 << 31)
#define RF_DFE_CTRL_4_RX_PF_Q_EN                              (1 << 30)
#define RF_DFE_CTRL_4_RX_PF_TH1_SHIFT                         (16)
#define RF_DFE_CTRL_4_RX_PF_TH1_MASK                          (0x3ff << RF_DFE_CTRL_4_RX_PF_TH1_SHIFT)
#define RF_DFE_CTRL_4_RX_PF_TH2_MASK                          (0x3ff)

#define RF_DFE_CTRL_5_RX_IQC_GAIN_EN                          (1 << 23)
#define RF_DFE_CTRL_5_RX_IQC_GAIN_SHIFT                       (12)
#define RF_DFE_CTRL_5_RX_IQC_GAIN_MASK                        (0x7ff << RF_DFE_CTRL_5_RX_IQC_GAIN_SHIFT)
#define RF_DFE_CTRL_5_RX_IQC_PHASE_EN                         (1 << 10)
#define RF_DFE_CTRL_5_RX_IQC_PHASE_MASK                       (0x3ff)

#define RF_DFE_CTRL_6_RX_PM_IN_SEL_SHIFT                      (30)
#define RF_DFE_CTRL_6_RX_PM_IN_SEL_MASK                       (0x03 << RF_DFE_CTRL_6_RX_PM_IN_SEL_SHIFT)
#define RF_DFE_CTRL_6_RX_PM_EN                                (1 << 29)
#define RF_DFE_CTRL_6_RX_PM_DONE                              (1 << 28)
#define RF_DFE_CTRL_6_RX_PM_FREQSHIFT_EN                      (1 << 20)
#define RF_DFE_CTRL_6_RX_PM_FREQSHIFT_CW_MASK                 (0xfffff)

#define RF_DFE_CTRL_7_RX_PM_ACC_LEN_SHIFT                     (16)
#define RF_DFE_CTRL_7_RX_PM_ACC_LEN_MASK                      (0xffff << RF_DFE_CTRL_7_RX_PM_ACC_LEN_SHIFT)
#define RF_DFE_CTRL_7_RX_PM_START_OFS_MASK                    (0xffff)

#define RF_DFE_CTRL_8_RX_PM_IQACC_I_MASK                      (0x1ffffff)

#define RF_DFE_CTRL_9_RX_PM_IQACC_Q_MASK                      (0x1ffffff)

#define RF_DFE_CTRL_10_DFE_DAC_RAW_Q_SHIFT                    (16)
#define RF_DFE_CTRL_10_DFE_DAC_RAW_Q_MASK                     (0x7ff << RF_DFE_CTRL_10_DFE_DAC_RAW_Q_SHIFT)
#define RF_DFE_CTRL_10_DFE_DAC_RAW_I_MASK                     (0x7ff)

#define RF_DFE_CTRL_11_DFE_ADC_RAW_Q_SHIFT                    (16)
#define RF_DFE_CTRL_11_DFE_ADC_RAW_Q_MASK                     (0x3ff << RF_DFE_CTRL_11_DFE_ADC_RAW_Q_SHIFT)
#define RF_DFE_CTRL_11_DFE_ADC_RAW_I_MASK                     (0x3ff)

#define RF_DFE_CTRL_12_TX_DVGA_GAIN_QDB_GC3_SHIFT             (24)
#define RF_DFE_CTRL_12_TX_DVGA_GAIN_QDB_GC3_MASK              (0x7f << RF_DFE_CTRL_12_TX_DVGA_GAIN_QDB_GC3_SHIFT)
#define RF_DFE_CTRL_12_TX_DVGA_GAIN_QDB_GC2_SHIFT             (16)
#define RF_DFE_CTRL_12_TX_DVGA_GAIN_QDB_GC2_MASK              (0x7f << RF_DFE_CTRL_12_TX_DVGA_GAIN_QDB_GC2_SHIFT)
#define RF_DFE_CTRL_12_TX_DVGA_GAIN_QDB_GC1_SHIFT             (8)
#define RF_DFE_CTRL_12_TX_DVGA_GAIN_QDB_GC1_MASK              (0x7f << RF_DFE_CTRL_12_TX_DVGA_GAIN_QDB_GC1_SHIFT)
#define RF_DFE_CTRL_12_TX_DVGA_GAIN_QDB_GC0_MASK              (0x7f)

#define RF_DFE_CTRL_13_TX_DVGA_GAIN_QDB_GC7_SHIFT             (24)
#define RF_DFE_CTRL_13_TX_DVGA_GAIN_QDB_GC7_MASK              (0x7f << RF_DFE_CTRL_13_TX_DVGA_GAIN_QDB_GC7_SHIFT)
#define RF_DFE_CTRL_13_TX_DVGA_GAIN_QDB_GC6_SHIFT             (16)
#define RF_DFE_CTRL_13_TX_DVGA_GAIN_QDB_GC6_MASK              (0x7f << RF_DFE_CTRL_13_TX_DVGA_GAIN_QDB_GC6_SHIFT)
#define RF_DFE_CTRL_13_TX_DVGA_GAIN_QDB_GC5_SHIFT             (8)
#define RF_DFE_CTRL_13_TX_DVGA_GAIN_QDB_GC5_MASK              (0x7f << RF_DFE_CTRL_13_TX_DVGA_GAIN_QDB_GC5_SHIFT)
#define RF_DFE_CTRL_13_TX_DVGA_GAIN_QDB_GC4_MASK              (0x7f)

#define RF_DFE_CTRL_14_TX_DVGA_GAIN_QDB_GC11_SHIFT            (24)
#define RF_DFE_CTRL_14_TX_DVGA_GAIN_QDB_GC11_MASK             (0x7f << RF_DFE_CTRL_14_TX_DVGA_GAIN_QDB_GC11_SHIFT)
#define RF_DFE_CTRL_14_TX_DVGA_GAIN_QDB_GC10_SHIFT            (16)
#define RF_DFE_CTRL_14_TX_DVGA_GAIN_QDB_GC10_MASK             (0x7f << RF_DFE_CTRL_14_TX_DVGA_GAIN_QDB_GC10_SHIFT)
#define RF_DFE_CTRL_14_TX_DVGA_GAIN_QDB_GC9_SHIFT             (8)
#define RF_DFE_CTRL_14_TX_DVGA_GAIN_QDB_GC9_MASK              (0x7f << RF_DFE_CTRL_14_TX_DVGA_GAIN_QDB_GC9_SHIFT)
#define RF_DFE_CTRL_14_TX_DVGA_GAIN_QDB_GC8_MASK              (0x7f)

#define RF_DFE_CTRL_15_TX_DVGA_GAIN_QDB_GC15_SHIFT            (24)
#define RF_DFE_CTRL_15_TX_DVGA_GAIN_QDB_GC15_MASK             (0x7f << RF_DFE_CTRL_15_TX_DVGA_GAIN_QDB_GC15_SHIFT)
#define RF_DFE_CTRL_15_TX_DVGA_GAIN_QDB_GC14_SHIFT            (16)
#define RF_DFE_CTRL_15_TX_DVGA_GAIN_QDB_GC14_MASK             (0x7f << RF_DFE_CTRL_15_TX_DVGA_GAIN_QDB_GC14_SHIFT)
#define RF_DFE_CTRL_15_TX_DVGA_GAIN_QDB_GC13_SHIFT            (8)
#define RF_DFE_CTRL_15_TX_DVGA_GAIN_QDB_GC13_MASK             (0x7f << RF_DFE_CTRL_15_TX_DVGA_GAIN_QDB_GC13_SHIFT)
#define RF_DFE_CTRL_15_TX_DVGA_GAIN_QDB_GC12_MASK             (0x7f)

#define RF_DFE_CTRL_16_RF_TBB_IND_GC7_SHIFT                   (28)
#define RF_DFE_CTRL_16_RF_TBB_IND_GC7_MASK                    (0x07 << RF_DFE_CTRL_16_RF_TBB_IND_GC7_SHIFT)
#define RF_DFE_CTRL_16_RF_TBB_IND_GC6_SHIFT                   (24)
#define RF_DFE_CTRL_16_RF_TBB_IND_GC6_MASK                    (0x07 << RF_DFE_CTRL_16_RF_TBB_IND_GC6_SHIFT)
#define RF_DFE_CTRL_16_RF_TBB_IND_GC5_SHIFT                   (20)
#define RF_DFE_CTRL_16_RF_TBB_IND_GC5_MASK                    (0x07 << RF_DFE_CTRL_16_RF_TBB_IND_GC5_SHIFT)
#define RF_DFE_CTRL_16_RF_TBB_IND_GC4_SHIFT                   (16)
#define RF_DFE_CTRL_16_RF_TBB_IND_GC4_MASK                    (0x07 << RF_DFE_CTRL_16_RF_TBB_IND_GC4_SHIFT)
#define RF_DFE_CTRL_16_RF_TBB_IND_GC3_SHIFT                   (12)
#define RF_DFE_CTRL_16_RF_TBB_IND_GC3_MASK                    (0x07 << RF_DFE_CTRL_16_RF_TBB_IND_GC3_SHIFT)
#define RF_DFE_CTRL_16_RF_TBB_IND_GC2_SHIFT                   (8)
#define RF_DFE_CTRL_16_RF_TBB_IND_GC2_MASK                    (0x07 << RF_DFE_CTRL_16_RF_TBB_IND_GC2_SHIFT)
#define RF_DFE_CTRL_16_RF_TBB_IND_GC1_SHIFT                   (4)
#define RF_DFE_CTRL_16_RF_TBB_IND_GC1_MASK                    (0x07 << RF_DFE_CTRL_16_RF_TBB_IND_GC1_SHIFT)
#define RF_DFE_CTRL_16_RF_TBB_IND_GC0_MASK                    (0x07)

#define RF_DFE_CTRL_17_RF_TBB_IND_GC15_SHIFT                  (28)
#define RF_DFE_CTRL_17_RF_TBB_IND_GC15_MASK                   (0x07 << RF_DFE_CTRL_17_RF_TBB_IND_GC15_SHIFT)
#define RF_DFE_CTRL_17_RF_TBB_IND_GC14_SHIFT                  (24)
#define RF_DFE_CTRL_17_RF_TBB_IND_GC14_MASK                   (0x07 << RF_DFE_CTRL_17_RF_TBB_IND_GC14_SHIFT)
#define RF_DFE_CTRL_17_RF_TBB_IND_GC13_SHIFT                  (20)
#define RF_DFE_CTRL_17_RF_TBB_IND_GC13_MASK                   (0x07 << RF_DFE_CTRL_17_RF_TBB_IND_GC13_SHIFT)
#define RF_DFE_CTRL_17_RF_TBB_IND_GC12_SHIFT                  (16)
#define RF_DFE_CTRL_17_RF_TBB_IND_GC12_MASK                   (0x07 << RF_DFE_CTRL_17_RF_TBB_IND_GC12_SHIFT)
#define RF_DFE_CTRL_17_RF_TBB_IND_GC11_SHIFT                  (12)
#define RF_DFE_CTRL_17_RF_TBB_IND_GC11_MASK                   (0x07 << RF_DFE_CTRL_17_RF_TBB_IND_GC11_SHIFT)
#define RF_DFE_CTRL_17_RF_TBB_IND_GC10_SHIFT                  (8)
#define RF_DFE_CTRL_17_RF_TBB_IND_GC10_MASK                   (0x07 << RF_DFE_CTRL_17_RF_TBB_IND_GC10_SHIFT)
#define RF_DFE_CTRL_17_RF_TBB_IND_GC9_SHIFT                   (4)
#define RF_DFE_CTRL_17_RF_TBB_IND_GC9_MASK                    (0x07 << RF_DFE_CTRL_17_RF_TBB_IND_GC9_SHIFT)
#define RF_DFE_CTRL_17_RF_TBB_IND_GC8_MASK                    (0x07)

#define RF_DFE_CTRL_18_TX_DVGA_GAIN_QDB_BLE_GC2_SHIFT         (16)
#define RF_DFE_CTRL_18_TX_DVGA_GAIN_QDB_BLE_GC2_MASK          (0x7f << RF_DFE_CTRL_18_TX_DVGA_GAIN_QDB_BLE_GC2_SHIFT)
#define RF_DFE_CTRL_18_TX_DVGA_GAIN_QDB_BLE_GC1_SHIFT         (8)
#define RF_DFE_CTRL_18_TX_DVGA_GAIN_QDB_BLE_GC1_MASK          (0x7f << RF_DFE_CTRL_18_TX_DVGA_GAIN_QDB_BLE_GC1_SHIFT)
#define RF_DFE_CTRL_18_TX_DVGA_GAIN_QDB_BLE_GC0_MASK          (0x7f)

#endif /* __ARCH_RISCV_SRC_HARDWARE_GLB_H */

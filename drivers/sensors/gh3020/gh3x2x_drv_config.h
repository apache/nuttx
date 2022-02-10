/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv_config.h
 *
 * @brief   gh3x2x driver configurations
 *
 * @version ref gh3x2x_drv_version.h
 *
 */

#ifndef _GH3X2X_DRV_CONFIG_H_
#define _GH3X2X_DRV_CONFIG_H_
#include "gh3x2x_drv.h"
#include "gh3020_bridge.h"

/* started bitmap */

#define  GH3X2X_STARTED_BITMAP_ALL_OFF              (0x0000)    /**< fnction started bitmap, all off bit */
#define  GH3X2X_STARTED_BITMAP_HBA                  (0x0001)    /**< fnction started bitmap, hba bit */
#define  GH3X2X_STARTED_BITMAP_SPO2                 (0x0002)    /**< fnction started bitmap, spo2 bit */
#define  GH3X2X_STARTED_BITMAP_ECG                  (0x0004)    /**< fnction started bitmap, ecg bit */
#define  GH3X2X_STARTED_BITMAP_ADT                  (0x0008)    /**< fnction started bitmap, adt bit */
#define  GH3X2X_STARTED_BITMAP_HRV                  (0x0010)    /**< fnction started bitmap, hrv bit */
#define  GH3X2X_STARTED_BITMAP_CUSTOMIZE            (0x0080)    /**< fnction started bitmap, customize bit */


#define  GH3X2X_REG_IS_VIRTUAL0X0_BIT               (0x0000)
#define  GH3X2X_REG_IS_VIRTUAL0X1_BIT               (0x1000)
#define  GH3X2X_REG_IS_VIRTUAL0X2_BIT               (0x2000)
#define  GH3X2X_REG_IS_VIRTUAL0X3_BIT               (0x3000)
#define  GH3X2X_REG_IS_VIRTUAL0X4_BIT               (0x4000)
#define  GH3X2X_REG_IS_VIRTUAL0X5_BIT               (0x5000)
#define  GH3X2X_REG_IS_VIRTUAL0X6_BIT               (0x6000)
#define  GH3X2X_REG_IS_VIRTUAL0X7_BIT               (0x7000)
#define  GH3X2X_REG_IS_VIRTUAL0X8_BIT               (0x8000)
#define  GH3X2X_REG_IS_VIRTUAL0X9_BIT               (0x9000)
#define  GH3X2X_REG_IS_VIRTUAL0XA_BIT               (0xA000)
#define  GH3X2X_REG_IS_VIRTUAL0XB_BIT               (0xB000)
#define  GH3X2X_REG_IS_VIRTUAL0XC_BIT               (0xC000)
#define  GH3X2X_REG_IS_VIRTUAL0XE_BIT               (0xE000)
#define  GH3X2X_REG_IS_VIRTUAL0XF_BIT               (0xF000)


//vitual addr version
#define GH3X2X_VIRTUAL_REG_VERSION_STRING "Gh3x2x_Virtual_Reg_v3.3"
//top
#define GH3X2X_TOP_INFO_ADDR 0x1000
#define GH3X2X_SLOT_LEDDRV_ADDR 0x10E0
#define GH3X2X_DUMP_CFG_ADDR 0x1100
#define GH3X2X_FIFO_CTRL_ADDR 0x1120
#define GH3X2X_G_SENSOR_CFG_ADDR 0x1140
#define GH3X2X_SOFT_AGC_PARA_ADDR 0x1160
#define GH3X2X_CAP_CFG_ADDR       0x1180
#define GH3X2X_TEMP_CFG_ADDR      0x11A0

#define GH3X2X_CHNL_MAP_ADDR 0x2000
#define GH3X2X_FS_PARA_ADDR 0x2880
#define GH3X2X_ADT_DRV_CFG_ADDR 0x3000
#define GH3X2X_ADT_ALG_CFG_ADDR 0x30C0
#define GH3X2X_HR_DRV_CFG_ADDR 0x3300
#define GH3X2X_HR_ALG_CFG_ADDR 0x33C0
#define GH3X2X_HRV_DRV_CFG_ADDR 0x3600
#define GH3X2X_HRV_ALG_CFG_ADDR 0x36C0
#define GH3X2X_HSM_DRV_CFG_ADDR 0x3900
#define GH3X2X_HSM_ALG_CFG_ADDR 0x39C0
#define GH3X2X_FPBP_DRV_CFG_ADDR 0x3C00
#define GH3X2X_FPBP_ALG_CFG_ADDR 0x3CC0
#define GH3X2X_PWA_DRV_CFG_ADDR 0x3F00
#define GH3X2X_PWA_ALG_CFG_ADDR 0x3FC0
#define GH3X2X_SPO2_DRV_CFG_ADDR 0x4200
#define GH3X2X_SPO2_ALG_CFG_ADDR 0x42C0
#define GH3X2X_ECG_DRV_CFG_ADDR 0x4500
#define GH3X2X_ECG_ALG_CFG_ADDR 0x45C0
#define GH3X2X_PWTT_DRV_CFG_ADDR 0x4800
#define GH3X2X_PWTT_ALG_CFG_ADDR 0x48C0
#define GH3X2X_SOFTADTGREEN_DRV_CFG_ADDR 0x4B00
#define GH3X2X_SOFTADTGREEN_ALG_CFG_ADDR 0x4BC0
#define GH3X2X_BT_DRV_CFG_ADDR 0x4E00
#define GH3X2X_BT_ALG_CFG_ADDR 0x4EC0
#define GH3X2X_RESP_DRV_CFG_ADDR 0x5100
#define GH3X2X_RESP_ALG_CFG_ADDR 0x51C0
#define GH3X2X_AF_DRV_CFG_ADDR 0x5400
#define GH3X2X_AF_ALG_CFG_ADDR 0x54C0
#define GH3X2X_TEST1_DRV_CFG_ADDR 0x5700
#define GH3X2X_TEST1_ALG_CFG_ADDR 0x57C0
#define GH3X2X_TEST2_DRV_CFG_ADDR 0x5A00
#define GH3X2X_TEST2_ALG_CFG_ADDR 0x5AC0
#define GH3X2X_SOFTADTIR_DRV_CFG_ADDR 0x5D00
#define GH3X2X_SOFTADTIR_ALG_CFG_ADDR 0x5DC0
#define GH3X2X_FINISH_FLAG_ADDR 0xFF00
//top_info
#define GH3X2X_TOP_INFO_OFFSET 0x00DF
#define GH3X2X_CFG_VER_ADDR 0x1000
#define GH3X2X_CFG_TOOL_VER_ADDR 0x1002
#define GH3X2X_PROJECT_ID_ADDR 0x1004
#define GH3X2X_TIMESTAMP_L_ADDR 0x1006
#define GH3X2X_TIMESTAMP_H_ADDR 0x1008
#define GH3X2X_REINIT_PARAM_ADDR 0x100A
//slot_leddrv
#define GH3X2X_SLOT_LEDDRV_OFFSET 0x001F
#define GH3X2X_SLOT0_1_LEDDRV_ADDR 0x10E0
#define GH3X2X_SLOT2_3_LEDDRV_ADDR 0x10E2
#define GH3X2X_SLOT4_5_LEDDRV_ADDR 0x10E4
#define GH3X2X_SLOT6_7_LEDDRV_ADDR 0x10E6
//dump_cfg
#define GH3X2X_DUMP_CFG_OFFSET 0x001F
#define GH3X2X_PPG_DUMP_MODE_ADDR 0x1100
#define GH3X2X_BG_LEVEL_SET_ADDR 0x1102
//fifo_ctrl
#define GH3X2X_FIFO_CTRL_OFFSET 0x001F
#define GH3X2X_NORMAL_FIFO_WATER_LINE_ADDR 0x1120
#define GH3X2X_ADT_FIFO_WATER_LINE_ADDR 0x1122
#define GH3X2X_FIFO_PACKAGE_SEND_MODE_ADDR 0x1124
//g_sensor_cfg
#define GH3X2X_G_SENSOR_CFG_OFFSET 0x001F
#define GH3X2X_GSENSOR_VENDOR_ID_ADDR 0x1140
#define GH3X2X_GSENSOR_SAMPLE_RATE_ADDR 0x1142
#define GH3X2X_GSENSOR_CTRL_ADDR 0x1144
//soft_agc_para
#define GH3X2X_SOFT_AGC_PARA_OFFSET 0x001F
#define GH3X2X_AGC_BG_CANCEL_ADJUST_SLOT_EN_ADDR 0x1160
#define GH3X2X_AGC_AMB_SLOT_CTRL_ADDR 0x1162
#define GH3X2X_AGC_GAIN_LIMIT_ADDR 0x1164
#define GH3X2X_AGC_TRIG_THD_H_LSB_16_ADDR 0x1166
#define GH3X2X_AGC_TRIG_THD_H_MSB_16_ADDR 0x1168
#define GH3X2X_AGC_TRIG_THD_L_LSB_16_ADDR 0x116A
#define GH3X2X_AGC_TRIG_THD_L_MSB_16_ADDR 0x116C
#define GH3X2X_AGC_RESTRAIN_THD_H_LSB_16_ADDR 0x116E
#define GH3X2X_AGC_RESTRAIN_THD_H_MSB_16_ADDR 0x1170
#define GH3X2X_AGC_RESTRAIN_THD_L_LSB_16_ADDR 0x1172
#define GH3X2X_AGC_RESTRAIN_THD_L_MSB_16_ADDR 0x1174
//cap_cfg
#define GH3X2X_CAP_CFG_OFFSET 0x001F
#define GH3X2X_CAP_CTRL_ADDR  0x1180
//Temp_cfg
#define GH3X2X_TEMP_CFG_OFFSET 0x001F
#define GH3X2X_TEMP_CTRL_ADDR  0x11A0


//chnl_map
#define GH3X2X_CHNL_MAP_OFFSET 0x087F
#define GH3X2X_ADT_CHMAP_CNT_ADDR 0x2000
#define GH3X2X_ADT_CHMAP_CH0_1_ADDR 0x2002
#define GH3X2X_HR_CHMAP_CNT_ADDR 0x2022
#define GH3X2X_HR_CHMAP_CH0_1_ADDR 0x2024
#define GH3X2X_HRV_CHMAP_CNT_ADDR 0x2044
#define GH3X2X_HRV_CHMAP_CH0_1_ADDR 0x2046
#define GH3X2X_HSM_CHMAP_CNT_ADDR 0x2066
#define GH3X2X_HSM_CHMAP_CH0_1_ADDR 0x2068
#define GH3X2X_FPBP_CHMAP_CNT_ADDR 0x2088
#define GH3X2X_FPBP_CHMAP_CH0_1_ADDR 0x208A
#define GH3X2X_PWA_CHMAP_CNT_ADDR 0x20AA
#define GH3X2X_PWA_CHMAP_CH0_1_ADDR 0x20AC
#define GH3X2X_SPO2_CHMAP_CNT_ADDR 0x20CC
#define GH3X2X_SPO2_CHMAP_CH0_1_ADDR 0x20CE
#define GH3X2X_ECG_CHMAP_CNT_ADDR 0x20EE
#define GH3X2X_ECG_CHMAP_CH0_1_ADDR 0x20F0
#define GH3X2X_PWTT_CHMAP_CNT_ADDR 0x2110
#define GH3X2X_PWTT_CHMAP_CH0_1_ADDR 0x2112
#define GH3X2X_SOFT_ADT_GREEN_CHMAP_CNT_ADDR 0x2132
#define GH3X2X_SOFT_ADT_GREEN_CHMAP_CH0_1_ADDR 0x2134
#define GH3X2X_BT_CHMAP_CNT_ADDR 0x2154
#define GH3X2X_BT_CHMAP_CH0_1_ADDR 0x2156
#define GH3X2X_RESP_CHMAP_CNT_ADDR 0x2176
#define GH3X2X_RESP_CHMAP_CH0_1_ADDR 0x2178
#define GH3X2X_AF_CHMAP_CNT_ADDR 0x2198
#define GH3X2X_AF_CHMAP_CH0_1_ADDR 0x219A
#define GH3X2X_TEST1_CHMAP_CNT_ADDR 0x21BA
#define GH3X2X_TEST1_CHMAP_CH0_1_ADDR 0x21BC
#define GH3X2X_TEST2_CHMAP_CNT_ADDR 0x21DC
#define GH3X2X_TEST2_CHMAP_CH0_1_ADDR 0x21DE
#define GH3X2X_SOFT_ADT_IR_CHMAP_CNT_ADDR 0x21FE
#define GH3X2X_SOFT_ADT_IR_CHMAP_CH0_1_ADDR 0x2200
//fs_para
#define GH3X2X_FS_PARA_OFFSET 0x007F
#define GH3X2X_ADT_FS_ADDR 0x2880
#define GH3X2X_HR_FS_ADDR 0x2882
#define GH3X2X_HRV_FS_ADDR 0x2884
#define GH3X2X_HSM_FS_ADDR 0x2886
#define GH3X2X_FPBP_FS_ADDR 0x2888
#define GH3X2X_PWA_FS_ADDR 0x288A
#define GH3X2X_SPO2_FS_ADDR 0x288C
#define GH3X2X_ECG_FS_ADDR 0x288E
#define GH3X2X_PWTT_FS_ADDR 0x2890
#define GH3X2X_SOFT_ADT_GREEN_FS_ADDR 0x2892
#define GH3X2X_BT_FS_ADDR 0x2894
#define GH3X2X_RESP_FS_ADDR 0x2896
#define GH3X2X_AF_FS_ADDR 0x2898
#define GH3X2X_TEST1_FS_ADDR 0x289A
#define GH3X2X_TEST2_FS_ADDR 0x289C
#define GH3X2X_SOFT_ADT_IR_FS_ADDR 0x289E
//adt_drv_cfg
#define GH3X2X_ADT_DRV_CFG_OFFSET 0x00C7
#define GH3X2X_ADT_CONFIG0_ADDR 0x3000
#define GH3X2X_ADT_CONFIG1_ADDR 0x3002
//adt_alg_cfg
#define GH3X2X_ADT_ALG_CFG_OFFSET 0x023F
//hr_drv_cfg
#define GH3X2X_HR_DRV_CFG_OFFSET 0x00C7
//hr_alg_cfg
#define GH3X2X_HR_ALG_CFG_OFFSET 0x023F
#define GH3X2X_HR_SCENARIO_CFG_L_ADDR 0x33C0
#define GH3X2X_HR_SCENARIO_CFG_H_ADDR 0x33C2
#define GH3X2X_HR_OUTPUT_TIME_L_ADDR 0x33C4
#define GH3X2X_HR_OUTPUT_TIME_H_ADDR 0x33C6
#define GH3X2X_HR_FS_L_ADDR 0x33C8
#define GH3X2X_HR_FS_H_ADDR 0x33CA
#define GH3X2X_HR_ALGO_CHNL_NUM_ADDR 0x35C0
#define GH3X2X_ALGO_CHNL_MAP_HR_GREEN_CHNL0_1_ADDR 0x35C4
#define GH3X2X_ALGO_CHNL_MAP_HR_GREEN_CHNL2_3_ADDR 0x35C6
#define GH3X2X_ALGO_CHNL_MAP_HR_IR_CHNL0_1_ADDR 0x35CC
#define GH3X2X_ALGO_CHNL_MAP_HR_IR_CHNL2_3_ADDR 0x35CE
#define GH3X2X_ALGO_CHNL_MAP_HR_RED_CHNL0_1_ADDR 0x35D4
#define GH3X2X_ALGO_CHNL_MAP_HR_RED_CHNL2_3_ADDR 0x35D6
//hrv_drv_cfg
#define GH3X2X_HRV_DRV_CFG_OFFSET 0x00C7
//hrv_alg_cfg
#define GH3X2X_HRV_ALG_CFG_OFFSET 0x023F
#define GH3X2X_HRV_INTERPOLATION_ENABLE_L_ADDR 0x36C0
#define GH3X2X_HRV_INTERPOLATION_ENABLE_H_ADDR 0x36C2
#define GH3X2X_HRV_ACC_THR_MAX_L_ADDR 0x36C4
#define GH3X2X_HRV_ACC_THR_MAX_H_ADDR 0x36C6
#define GH3X2X_HRV_ACC_THR_MIN_L_ADDR 0x36C8
#define GH3X2X_HRV_ACC_THR_MIN_H_ADDR 0x36CA
#define GH3X2X_HRV_ACC_THR_NUM_L_ADDR 0x36CC
#define GH3X2X_HRV_ACC_THR_NUM_H_ADDR 0x36D0
#define GH3X2X_HRV_ACC_THR_SCALE_L_ADDR 0x36D2
#define GH3X2X_HRV_ACC_THR_SCALE_H_ADDR 0x36D4
//hsm_drv_cfg
#define GH3X2X_HSM_DRV_CFG_OFFSET 0x00C7
//hsm_alg_cfg
#define GH3X2X_HSM_ALG_CFG_OFFSET 0x023F
//fpbp_drv_cfg
#define GH3X2X_FPBP_DRV_CFG_OFFSET 0x00C7
//fpbp_alg_cfg
#define GH3X2X_FPBP_ALG_CFG_OFFSET 0x023F
#define GH3X2X_FPBP_CALC_MODE_L_ADDR 0x3CC0
#define GH3X2X_FPBP_CALC_MODE_H_ADDR 0x3CC2
#define GH3X2X_FPBP_CALI_WAVE_REPLEVEL_THRD_L_ADDR 0x3CC4
#define GH3X2X_FPBP_CALI_WAVE_REPLEVEL_THRD_H_ADDR 0x3CC6
#define GH3X2X_FPBP_CALI_WAVE_STBLEVEL_THRD_L_ADDR 0x3CC8
#define GH3X2X_FPBP_CALI_WAVE_STBLEVEL_THRD_H_ADDR 0x3CCA
#define GH3X2X_FPBP_CALI_CONFSCORE_THRD_L_ADDR 0x3CCC
#define GH3X2X_FPBP_CALI_CONFSCORE_THRD_H_ADDR 0x3CCE
#define GH3X2X_FPBP_CALC_WAVE_REPLEVEL_THRD_L_ADDR 0x3CD0
#define GH3X2X_FPBP_CALC_WAVE_REPLEVEL_THRD_H_ADDR 0x3CD2
#define GH3X2X_FPBP_CALC_WAVE_STBLEVEL_THRD_L_ADDR 0x3CD4
#define GH3X2X_FPBP_CALC_WAVE_STBLEVEL_THRD_H_ADDR 0x3CD6
#define GH3X2X_FPBP_CALC_CONFSCORE_THRD_L_ADDR 0x3CD8
#define GH3X2X_FPBP_CALC_CONFSCORE_THRD_H_ADDR 0x3CDA
#define GH3X2X_FPBP_RESULT_CHECK_FLAG_L_ADDR 0x3CDC
#define GH3X2X_FPBP_RESULT_CHECK_FLAG_H_ADDR 0x3CDE
#define GH3X2X_FPBP_PROGRAM_SELECT_L_ADDR 0x3CE0
#define GH3X2X_FPBP_PROGRAM_SELECT_H_ADDR 0x3CE2
#define GH3X2X_FPBP_PRESSURE_OFFSET_L_ADDR 0x3CE4
#define GH3X2X_FPBP_PRESSURE_OFFSET_H_ADDR 0x3CE6
#define GH3X2X_FPBP_PRESSURE_FACTOR_L_ADDR 0x3CE8
#define GH3X2X_FPBP_PRESSURE_FACTOR_H_ADDR 0x3CEA
#define GH3X2X_FPBP_PRESSURE_AREA_L_ADDR 0x3CEC
#define GH3X2X_FPBP_PRESSURE_AREA_H_ADDR 0x3CEE
#define GH3X2X_FPBP_ALGO_CHNL_NUM_ADDR 0x3EC0
#define GH3X2X_ALGO_CHNL_MAP_FPBP_ECG_CHNL_ADDR 0x3EC2
#define GH3X2X_ALGO_CHNL_MAP_FPBP_GREEN_CHNL0_1_ADDR 0x3EC4
#define GH3X2X_ALGO_CHNL_MAP_FPBP_GREEN_CHNL2_3_ADDR 0x3EC6
#define GH3X2X_ALGO_CHNL_MAP_FPBP_IR_CHNL0_1_ADDR 0x3ECC
#define GH3X2X_ALGO_CHNL_MAP_FPBP_IR_CHNL2_3_ADDR 0x3ECE
#define GH3X2X_ALGO_CHNL_MAP_FPBP_RED_CHNL0_1_ADDR 0x3ED4
#define GH3X2X_ALGO_CHNL_MAP_FPBP_RED_CHNL2_3_ADDR 0x3ED6
//pwa_drv_cfg
#define GH3X2X_PWA_DRV_CFG_OFFSET 0x00C7
//pwa_alg_cfg
#define GH3X2X_PWA_ALG_CFG_OFFSET 0x023F
#define GH3X2X_PWA_CALC_MODE_L_ADDR 0x3FC0
#define GH3X2X_PWA_CALC_MODE_H_ADDR 0x3FC2
#define GH3X2X_PWA_CALI_WAVE_REPLEVEL_THRD_L_ADDR 0x3FC4
#define GH3X2X_PWA_CALI_WAVE_REPLEVEL_THRD_H_ADDR 0x3FC6
#define GH3X2X_PWA_CALI_WAVE_STBLEVEL_THRD_L_ADDR 0x3FC8
#define GH3X2X_PWA_CALI_WAVE_STBLEVEL_THRD_H_ADDR 0x3FCA
#define GH3X2X_PWA_CALI_CONFSCORE_THRD_L_ADDR 0x3FCC
#define GH3X2X_PWA_CALI_CONFSCORE_THRD_H_ADDR 0x3FCE
#define GH3X2X_PWA_CALC_WAVE_REPLEVEL_THRD_L_ADDR 0x3FD0
#define GH3X2X_PWA_CALC_WAVE_REPLEVEL_THRD_H_ADDR 0x3FD2
#define GH3X2X_PWA_CALC_WAVE_STBLEVEL_THRD_L_ADDR 0x3FD4
#define GH3X2X_PWA_CALC_WAVE_STBLEVEL_THRD_H_ADDR 0x3FD6
#define GH3X2X_PWA_CALC_CONFSCORE_THRD_L_ADDR 0x3FD8
#define GH3X2X_PWA_CALC_CONFSCORE_THRD_H_ADDR 0x3FDA
#define GH3X2X_PWA_RESULT_CHECK_FLAG_L_ADDR 0x3FDC
#define GH3X2X_PWA_RESULT_CHECK_FLAG_H_ADDR 0x3FDE
#define GH3X2X_PWA_PROGRAM_SELECT_L_ADDR 0x3FE0
#define GH3X2X_PWA_PROGRAM_SELECT_H_ADDR 0x3FE2
#define GH3X2X_PWA_PRESSURE_OFFSET_L_ADDR 0x3FE4
#define GH3X2X_PWA_PRESSURE_OFFSET_H_ADDR 0x3FE6
#define GH3X2X_PWA_PRESSURE_FACTOR_L_ADDR 0x3FE8
#define GH3X2X_PWA_PRESSURE_FACTOR_H_ADDR 0x3FEA
#define GH3X2X_PWA_PRESSURE_AREA_L_ADDR 0x3FEC
#define GH3X2X_PWA_PRESSURE_AREA_H_ADDR 0x3FEE
#define GH3X2X_PWA_ALGO_CHNL_NUM_ADDR 0x41C0
#define GH3X2X_ALGO_CHNL_MAP_PWA_ECG_CHNL_ADDR 0x41C2
#define GH3X2X_ALGO_CHNL_MAP_PWA_GREEN_CHNL0_1_ADDR 0x41C4
#define GH3X2X_ALGO_CHNL_MAP_PWA_GREEN_CHNL2_3_ADDR 0x41C6
#define GH3X2X_ALGO_CHNL_MAP_PWA_IR_CHNL0_1_ADDR 0x41CC
#define GH3X2X_ALGO_CHNL_MAP_PWA_IR_CHNL2_3_ADDR 0x41CE
#define GH3X2X_ALGO_CHNL_MAP_PWA_RED_CHNL0_1_ADDR 0x41D4
#define GH3X2X_ALGO_CHNL_MAP_PWA_RED_CHNL2_3_ADDR 0x41D6
//spo2_drv_cfg
#define GH3X2X_SPO2_DRV_CFG_OFFSET 0x00C7
//spo2_alg_cfg
#define GH3X2X_SPO2_ALG_CFG_OFFSET 0x023F
#define GH3X2X_SPO2_CH0_CORRECTION_FACTOR0_L_ADDR 0x42C0
#define GH3X2X_SPO2_CH0_CORRECTION_FACTOR0_H_ADDR 0x42C2
#define GH3X2X_SPO2_CH0_CORRECTION_FACTOR1_L_ADDR 0x42C4
#define GH3X2X_SPO2_CH0_CORRECTION_FACTOR1_H_ADDR 0x42C6
#define GH3X2X_SPO2_CH0_CORRECTION_FACTOR2_L_ADDR 0x42C8
#define GH3X2X_SPO2_CH0_CORRECTION_FACTOR2_H_ADDR 0x42CA
#define GH3X2X_SPO2_CH0_CORRECTION_FACTOR3_L_ADDR 0x42CC
#define GH3X2X_SPO2_CH0_CORRECTION_FACTOR3_H_ADDR 0x42CE
#define GH3X2X_SPO2_CH0_CORRECTION_FACTOR4_L_ADDR 0x42D0
#define GH3X2X_SPO2_CH0_CORRECTION_FACTOR4_H_ADDR 0x42D2
#define GH3X2X_SPO2_CH1_CORRECTION_FACTOR0_L_ADDR 0x42D4
#define GH3X2X_SPO2_CH1_CORRECTION_FACTOR0_H_ADDR 0x42D6
#define GH3X2X_SPO2_CH1_CORRECTION_FACTOR1_L_ADDR 0x42D8
#define GH3X2X_SPO2_CH1_CORRECTION_FACTOR1_H_ADDR 0x42DA
#define GH3X2X_SPO2_CH1_CORRECTION_FACTOR2_L_ADDR 0x42DC
#define GH3X2X_SPO2_CH1_CORRECTION_FACTOR2_H_ADDR 0x42DE
#define GH3X2X_SPO2_CH1_CORRECTION_FACTOR3_L_ADDR 0x42E0
#define GH3X2X_SPO2_CH1_CORRECTION_FACTOR3_H_ADDR 0x42E2
#define GH3X2X_SPO2_CH1_CORRECTION_FACTOR4_L_ADDR 0x42E4
#define GH3X2X_SPO2_CH1_CORRECTION_FACTOR4_H_ADDR 0x42E6
#define GH3X2X_SPO2_CH2_CORRECTION_FACTOR0_L_ADDR 0x42E8
#define GH3X2X_SPO2_CH2_CORRECTION_FACTOR0_H_ADDR 0x42EA
#define GH3X2X_SPO2_CH2_CORRECTION_FACTOR1_L_ADDR 0x42EC
#define GH3X2X_SPO2_CH2_CORRECTION_FACTOR1_H_ADDR 0x42EE
#define GH3X2X_SPO2_CH2_CORRECTION_FACTOR2_L_ADDR 0x42F0
#define GH3X2X_SPO2_CH2_CORRECTION_FACTOR2_H_ADDR 0x42F2
#define GH3X2X_SPO2_CH2_CORRECTION_FACTOR3_L_ADDR 0x42F4
#define GH3X2X_SPO2_CH2_CORRECTION_FACTOR3_H_ADDR 0x42F6
#define GH3X2X_SPO2_CH2_CORRECTION_FACTOR4_L_ADDR 0x42F8
#define GH3X2X_SPO2_CH2_CORRECTION_FACTOR4_H_ADDR 0x42FA
#define GH3X2X_SPO2_CH3_CORRECTION_FACTOR0_L_ADDR 0x42FC
#define GH3X2X_SPO2_CH3_CORRECTION_FACTOR0_H_ADDR 0x42FE
#define GH3X2X_SPO2_CH3_CORRECTION_FACTOR1_L_ADDR 0x4300
#define GH3X2X_SPO2_CH3_CORRECTION_FACTOR1_H_ADDR 0x4302
#define GH3X2X_SPO2_CH3_CORRECTION_FACTOR2_L_ADDR 0x4304
#define GH3X2X_SPO2_CH3_CORRECTION_FACTOR2_H_ADDR 0x4306
#define GH3X2X_SPO2_CH3_CORRECTION_FACTOR3_L_ADDR 0x4308
#define GH3X2X_SPO2_CH3_CORRECTION_FACTOR3_H_ADDR 0x430A
#define GH3X2X_SPO2_CH3_CORRECTION_FACTOR4_L_ADDR 0x430C
#define GH3X2X_SPO2_CH3_CORRECTION_FACTOR4_H_ADDR 0x430E
#define GH3X2X_SPO2_ALGO_CHNL_NUM_ADDR 0x44C0
#define GH3X2X_ALGO_CHNL_MAP_SPO2_GREEN_CHNL0_1_ADDR 0x44C4
#define GH3X2X_ALGO_CHNL_MAP_SPO2_GREEN_CHNL2_3_ADDR 0x44C6
#define GH3X2X_ALGO_CHNL_MAP_SPO2_IR_CHNL0_1_ADDR 0x44CC
#define GH3X2X_ALGO_CHNL_MAP_SPO2_IR_CHNL2_3_ADDR 0x44CE
#define GH3X2X_ALGO_CHNL_MAP_SPO2_RED_CHNL0_1_ADDR 0x44D4
#define GH3X2X_ALGO_CHNL_MAP_SPO2_RED_CHNL2_3_ADDR 0x44D6
//ecg_drv_cfg
#define GH3X2X_ECG_DRV_CFG_OFFSET 0x00C7
#define GH3X2X_ECG_SETTING0_ADDR 0x4500
#define GH3X2X_ECG_SETTING1_ADDR 0x4502
#define GH3X2X_ECG_SETTING2_ADDR 0x4504
#define GH3X2X_ECG_SETTING3_ADDR 0x4506
//ecg_alg_cfg
#define GH3X2X_ECG_ALG_CFG_OFFSET 0x023F
//pwtt_drv_cfg
#define GH3X2X_PWTT_DRV_CFG_OFFSET 0x00C7
//pwtt_alg_cfg
#define GH3X2X_PWTT_ALG_CFG_OFFSET 0x023F
//softadtgreen_drv_cfg
#define GH3X2X_SOFTADTGREEN_DRV_CFG_OFFSET 0x00C7
//softadtgreen_alg_cfg
#define GH3X2X_SOFTADTGREEN_ALG_CFG_OFFSET 0x023F
#define GH3X2X_SOFTADT_GREEN_ALGO_CHNL_NUM_ADDR 0x4DC0
#define GH3X2X_ALGO_CHNL_MAP_SOFTADTGREEN_GREEN_CHNL0_1_ADDR 0x4DC4
#define GH3X2X_ALGO_CHNL_MAP_SOFTADTGREEN_GREEN_CHNL2_3_ADDR 0x4DC6
#define GH3X2X_ALGO_CHNL_MAP_SOFTADTGREEN_GREEN_BG_CHNL0_1_ADDR 0x4DCC
#define GH3X2X_ALGO_CHNL_MAP_SOFTADTGREEN_BG_CHNL2_3_ADDR 0x4DCE
#define GH3X2X_ALGO_CHNL_MAP_SOFTADTGREEN_ADT_CHNL0_1_ADDR 0x4DD4
#define GH3X2X_ALGO_CHNL_MAP_SOFTADTGREEN_ADT_CHNL2_3_ADDR 0x4DD6
//bt_drv_cfg
#define GH3X2X_BT_DRV_CFG_OFFSET 0x00C7
//bt_alg_cfg
#define GH3X2X_BT_ALG_CFG_OFFSET 0x023F
#define GH3X2X_BT_NTC0_CALIRESISTANCE1_L_ADDR 0x4EC0
#define GH3X2X_BT_NTC0_CALIRESISTANCE1_H_ADDR 0x4EC2
#define GH3X2X_BT_NTC0_CALIRESISTANCE2_L_ADDR 0x4EC4
#define GH3X2X_BT_NTC0_CALIRESISTANCE2_H_ADDR 0x4EC6
#define GH3X2X_BT_NTC0_CALIRESISTANCE3_L_ADDR 0x4EC8
#define GH3X2X_BT_NTC0_CALIRESISTANCE3_H_ADDR 0x4ECA
#define GH3X2X_BT_NTC0_BASERESISTANCE_L_ADDR 0x4ECC
#define GH3X2X_BT_NTC0_BASERESISTANCE_H_ADDR 0x4ECE
#define GH3X2X_BT_NTC0_CALITEMPERATURE1_L_ADDR 0x4ED0
#define GH3X2X_BT_NTC0_CALITEMPERATURE1_H_ADDR 0x4ED2
#define GH3X2X_BT_NTC0_CALITEMPERATURE2_L_ADDR 0x4ED4
#define GH3X2X_BT_NTC0_CALITEMPERATURE2_H_ADDR 0x4ED6
#define GH3X2X_BT_NTC0_CALITEMPERATURE3_L_ADDR 0x4ED8
#define GH3X2X_BT_NTC0_CALITEMPERATURE3_H_ADDR 0x4EDA
#define GH3X2X_BT_NTC1_CALIRESISTANCE1_L_ADDR 0x4EDC
#define GH3X2X_BT_NTC1_CALIRESISTANCE1_H_ADDR 0x4EDE
#define GH3X2X_BT_NTC1_CALIRESISTANCE2_L_ADDR 0x4EE0
#define GH3X2X_BT_NTC1_CALIRESISTANCE2_H_ADDR 0x4EE2
#define GH3X2X_BT_NTC1_CALIRESISTANCE3_L_ADDR 0x4EE4
#define GH3X2X_BT_NTC1_CALIRESISTANCE3_H_ADDR 0x4EE6
#define GH3X2X_BT_NTC1_BASERESISTANCE_L_ADDR 0x4EE8
#define GH3X2X_BT_NTC1_BASERESISTANCE_H_ADDR 0x4EEA
#define GH3X2X_BT_NTC1_CALITEMPERATURE1_L_ADDR 0x4EEC
#define GH3X2X_BT_NTC1_CALITEMPERATURE1_H_ADDR 0x4EEE
#define GH3X2X_BT_NTC1_CALITEMPERATURE2_L_ADDR 0x4EF0
#define GH3X2X_BT_NTC1_CALITEMPERATURE2_H_ADDR 0x4EF2
#define GH3X2X_BT_NTC1_CALITEMPERATURE3_L_ADDR 0x4EF4
#define GH3X2X_BT_NTC1_CALITEMPERATURE3_H_ADDR 0x4EF6
//resp_drv_cfg
#define GH3X2X_RESP_DRV_CFG_OFFSET 0x00C7
//resp_alg_cfg
#define GH3X2X_RESP_ALG_CFG_OFFSET 0x023F
//af_drv_cfg
#define GH3X2X_AF_DRV_CFG_OFFSET 0x00C7
//af_alg_cfg
#define GH3X2X_AF_ALG_CFG_OFFSET 0x023F
#define GH3X2X_AF_WAIT_TIME_L_ADDR 0x54C0
#define GH3X2X_AF_WAIT_TIME_H_ADDR 0x54C2
//test1_drv_cfg
#define GH3X2X_TEST1_DRV_CFG_OFFSET 0x00C7
//test1_alg_cfg
#define GH3X2X_TEST1_ALG_CFG_OFFSET 0x023F
//test2_drv_cfg
#define GH3X2X_TEST2_DRV_CFG_OFFSET 0x00C7
//test2_alg_cfg
#define GH3X2X_TEST2_ALG_CFG_OFFSET 0x023F
//softadtir_drv_cfg
#define GH3X2X_SOFTADTIR_DRV_CFG_OFFSET 0x00C7
//softadtir_alg_cfg
#define GH3X2X_SOFTADTIR_ALG_CFG_OFFSET 0x023F
#define GH3X2X_SOFTADT_ALGO_IR_CHNL_NUM_ADDR 0x5FC0
#define GH3X2X_ALGO_CHNL_MAP_SOFTADTIR_IR_CHNL0_1_ADDR 0x5FC4
#define GH3X2X_ALGO_CHNL_MAP_SOFTADTIR_IR_CHNL2_3_ADDR 0x5FC6
#define GH3X2X_ALGO_CHNL_MAP_SOFTADTIR_BG_CHNL0_1_ADDR 0x5FCC
#define GH3X2X_ALGO_CHNL_MAP_SOFTADTIR_BG_CHNL2_3_ADDR 0x5FCE
#define GH3X2X_ALGO_CHNL_MAP_SOFTADTIR_ADT_CHNL0_1_ADDR 0x5FD4
#define GH3X2X_ALGO_CHNL_MAP_SOFTADTIR_ADT_CHNL2_3_ADDR 0x5FD6
//finish_flag
#define GH3X2X_FINISH_FLAG_OFFSET 0x00FF
#define GH3X2X_CHECKSUM_ADDR 0xFFFE
#define GH3X2X_END_FLAG_ADDR 0xFFFF



#define GH3X2X_CHNLMAP_OFFSET 0X0022
#define GH3X2X_VREG_ALGO_PARA_OFFSET 0X00C0
#define GH3X2X_VREG_FUNCTION_OFFSET 0X0300
#define GH3X2X_VREG_FUNCTION_ALGO_OFFSET 0X00C0
#define GH3X2X_VREG_ALGO_CHNL_MAP_OFFSET 0X0200



/* control config */

#define GH3X2X_DUMP_MODE_EN     (0)

#define GH3X2X_ELECTRO_RRECER_OBSERVE_EN     (0)

#define GH3X2X_AGC_BG_CANCEL_ADJ_EN (0)

#define GH3X2X_AGC_2X2_GAIN_ADJ_EN (0)

#define GH3X2X_SUPPORT_READ_BACK_VIRTUAL_REG (0)







/// wakeup and sleep every control api
#define   GH3X2X_CTRL_API_INSERT_WAKEUP_AND_DSLP_ENABLED        (0)

/// instructions chip inited by reg val, using reg chip_sw_backup[1:0]
#define   GH3X2X_INSTRUCTIONS_CHIP_INIED_ENABLED                (1)

/// wait power pulse mode off effective
#define   GH3X2X_PMU_WAIT_PULSE_OFF_ENABLED                     (0)

/// fifo power cut before sleep
#define   GH3X2X_PMU_FIFO_POWER_CTRL_ENABLED                    (1)

/// init communicate confirm max cnt
#define   GH3X2X_COMMUNICATE_CONFIRM_MAX_CNT                    (3)

/// virtual reg support config software
#define   GH3X2X_VIRTUAL_REG_SW_CONFIG_SUPPORT                  (1)

/// channel map max channel support to unpack rawdata
#define   GH3X2X_CHANNEL_MAP_MAX_CH                             (32)

/// gsensor data num
#define   GH3X2X_GSENSOR_MAX_SIZE                               (3)

/// wear on force switch with adt logic sel
#define   GH3X2X_WEAR_ON_FORCE_SWITCH_WITH_LOGIC_SEL            (1)

/// ex control slot enable bit form virtual reg config
#define   GH3X2X_EX_SLOT_EN_FROM_CONFIG                         (0)

/// ex control channel map cnt follow algorithm
#define   GH3X2X_EX_CHANNEL_CNT_FOLLOW_ALGORITHMS               (0)


/* algorithm config */

/// ecg algorithm default sampling frequency
#define   GH3X2X_ALGORITHM_ADT_DEFAULT_FS                       (5)

/// ram for algorithms bytes align
#define   GH3X2X_ALGORITHMS_MEMORY_ALIGN                        (4)

/// ram for algorithms bytes to align size
#define   GH3X2X_ALGORITHMS_MEMORY_BYTES_2_ALGIN_SIZE(x)        ((x) / GH3X2X_ALGORITHMS_MEMORY_ALIGN)

/// encrypt array start index
#define   GH3X2X_ENCRYPT_ARRAY_START_INDEX                      (6)

/// virtual reg support config algo param
#define   GH3X2X_VIRTUAL_REG_FUNCTION_CONFIG_SUPPORT            (1)

/// hba algorithm support
#define   GH3X2X_ALGORITHM_HBA_SUPPORT                          (1)

/// hba algorithm default sampling frequency
#define   GH3X2X_ALGORITHM_HBA_DEFAULT_FS                       (25)

/// hrv algorithm support
#define   GH3X2X_ALGORITHM_HRV_SUPPORT                          (1)

/// hrv algorithm default sampling frequency
#define   GH3X2X_ALGORITHM_HRV_DEFAULT_FS                       (100)

/// spo2 algorithm support
#define   GH3X2X_ALGORITHM_SPO2_SUPPORT                         (1)

/// spo2 algorithm default sampling frequency
#define   GH3X2X_ALGORITHM_SPO2_DEFAULT_FS                      (100)

/// ecg algorithm support
#define   GH3X2X_ALGORITHM_ECG_SUPPORT                          (0)
#define   GH3X2X_800HZ_LEAD_OFF_SUPPORT_ECG                     (1)

/// ecg algorithm default sampling frequency
#define   GH3X2X_ALGORITHM_ECG_DEFAULT_FS                       (500)

#ifdef ANDROID_REE_PLATFORM
/// hsm algorithm support
#define   GH3X2X_ALGORITHM_HSM_SUPPORT                          (0)
/// bt algorithm support
#define   GH3X2X_ALGORITHM_BT_SUPPORT                           (0)
/// resp algorithm support
#define   GH3X2X_ALGORITHM_RESP_SUPPORT                         (0)
/// af algorithm support
#define   GH3X2X_ALGORITHM_AF_SUPPORT                           (0)
/// nadt algorithm support
#define   GH3X2X_ALGORITHM_NADT_SUPPORT                         (0)
/// fpbp algorithm support
#define   GH3X2X_ALGORITHM_BP_SUPPORT                           (1)
#else
/// hsm algorithm support
#define   GH3X2X_ALGORITHM_HSM_SUPPORT                          (1)
/// bt algorithm support
#define   GH3X2X_ALGORITHM_BT_SUPPORT                           (1)
/// resp algorithm support
#define   GH3X2X_ALGORITHM_RESP_SUPPORT                         (1)
/// af algorithm support
#define   GH3X2X_ALGORITHM_AF_SUPPORT                           (1)
/// nadt algorithm support
#define   GH3X2X_ALGORITHM_NADT_SUPPORT                         (1)
/// fpbp algorithm support
#define   GH3X2X_ALGORITHM_BP_SUPPORT                           (1)
#endif
#if (GH3X2X_ALGORITHM_HBA_SUPPORT)

/// hba algorithm support string
#define   GH3X2X_ALGO_FNC_SUPPORT_HBA                          "_hba"

#else

/// hba algorithm support string
#define   GH3X2X_ALGO_FNC_SUPPORT_HBA

#endif

#if (GH3X2X_ALGORITHM_SPO2_SUPPORT)

/// hba algorithm support string
#define   GH3X2X_ALGO_FNC_SUPPORT_SPO2                          "_spo2"

#else

/// hba algorithm support string
#define   GH3X2X_ALGO_FNC_SUPPORT_SPO2

#endif

#if (GH3X2X_ALGORITHM_ECG_SUPPORT)

/// ecg algorithm support string
#define   GH3X2X_ALGO_FNC_SUPPORT_ECG                          "_ecg"

#else

/// ecg algorithm support string
#define   GH3X2X_ALGO_FNC_SUPPORT_ECG

#endif

/// macro of algorithm function support string
#define   GH3X2X_ALGO_FNC_SUPPORT_STRING                        "sup"\
                                                                GH3X2X_ALGO_FNC_SUPPORT_HBA\
                                                                GH3X2X_ALGO_FNC_SUPPORT_SPO2\
                                                                GH3X2X_ALGO_FNC_SUPPORT_ECG


/* protocol config */

/// universal protocol packet min len support
#define  GH3X2X_UPROTOCOL_PACKET_LEN_MIN                        (20)



/// cmd support config: fifo thr cmd support
#define  GH3X2X_UPROTOCOL_FIFO_THR_CFG_CMD_SUP                  (1)

/// cmd support config: chip ctrl cmd support wakeup & sleep
#define  GH3X2X_UPROTOCOL_CHIP_CTRL_CMD_WAKEUP_SLEEP_SUP        (1)

char *GH3X2X_GetVirtualRegVersion(void);

/**
 * @fn     uint16_t GH3X2X_ReadSwConfigWithVirtualReg(uint16_t usVirtualRegAddr)
 *
 * @brief  Read software config with virtual reg
 *
 * @attention   Virtual reg addr has del control bits, so reg addr is [0:11] valid.
 *
 * @param[in]   usVirtualRegAddr        virtual reg addr
 * @param[out]  None
 *
 * @return  virtual reg val
 */
uint16_t GH3X2X_ReadSwConfigWithVirtualReg(uint16_t usVirtualRegAddr);

/**
 * @fn     int8_t GH3X2X_WriteSwConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue)
 *
 * @brief  Write software param config with virtual reg
 *
 * @attention   Virtual reg addr has del control bits, so reg addr is [0:11] valid.
 *
 * @param[in]   usVirtualRegAddr        virtual reg addr
 * @param[in]   usVirtualRegValue       virtual reg value
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_WriteSwConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue);

/**
 * @fn     uint16_t GH3X2X_ReadChnlMapConfigWithVirtualReg(uint16_t usVirtualRegAddr)
 *
 * @brief  Read software config with virtual reg
 *
 * @attention   Virtual reg addr has del control bits, so reg addr is [0:11] valid.
 *
 * @param[in]   usVirtualRegAddr        virtual reg addr
 * @param[out]  None
 *
 * @return  virtual reg val
 */
uint16_t GH3X2X_ReadChnlMapConfigWithVirtualReg(uint16_t usVirtualRegAddr);

/**
 * @fn     int8_t GH3X2X_WriteChnlMapConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue)
 *
 * @brief  Write software param config with virtual reg
 *
 * @attention   Virtual reg addr has del control bits, so reg addr is [0:11] valid.
 *
 * @param[in]   usVirtualRegAddr        virtual reg addr
 * @param[in]   usVirtualRegValue       virtual reg value
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_WriteChnlMapConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue);

/**
 * @fn     int8_t GH3X2X_WriteVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue)
 *
 * @brief  Write virtual reg val, for software param config
 *
 * @attention   Virtual reg addr has del control bits, so reg addr is [0:11] valid.
 *
 * @param[in]   usVirtualRegAddr        virtual reg addr
 * @param[in]   usVirtualRegValue       virtual reg value
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_WriteVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue);

/**
 * @fn     uint16_t GH3X2X_ReadVirtualReg(uint16_t usVirtualRegAddr)
 *
 * @brief  Read virtual reg val, for software param config
 *
 * @attention   Virtual reg addr has del control bits, so reg addr is [0:11] valid.
 *
 * @param[in]   usVirtualRegAddr        virtual reg addr
 * @param[out]  None
 *
 * @return  virtual reg val
 */
uint16_t GH3X2X_ReadVirtualReg(uint16_t usVirtualRegAddr);

extern void GH3X2X_WriteHrAlgConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue);
extern void GH3X2X_WriteHrvAlgConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue);
extern void GH3X2X_WriteSpo2AlgConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue);
extern void GH3X2X_WriteBtAlgConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue);
extern void GH3X2X_WriteAfAlgConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue);
extern void GH3X2X_WriteBpAlgConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue);
extern void GH3X2X_SetCurrentConfigFlag(uint8_t value);

#endif /* _GH3X2X_DRV_CONFIG_H_ */

/********END OF FILE********* Copyright (c) 2003 - 2021, Goodix Co., Ltd. ********/

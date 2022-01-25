/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv_registers.h
 *
 * @brief   gh3x2x driver registers define
 *
 * @version ref gh3x2x_drv_version.h
 *
 */

#ifndef _GH3X2X_DRV_REGISTERS_H_
#define _GH3X2X_DRV_REGISTERS_H_

/* register mapping definition */

// top ctrl block, start addr: 0x0000
#define   GH3X2X_CARDIFF_CTRL_REG_ADDR                  (0x0000)    /**< start ctrl */
#define   GH3X2X_SYSCLK_CTRL_REG_ADDR                   (0x0002)
#define   GH3X2X_SYS_SAMPLE_RATE_CTRL_REG_ADDR          (0x0004)
#define   GH3X2X_DATA_CTRL0_REG_ADDR                    (0x0006)
#define   GH3X2X_DATA_CTRL1_REG_ADDR                    (0x0008)
#define   GH3X2X_FIFO_WATERLINE_REG_ADDR                (0x000A)    /**< fifo watermark thr */
#define   GH3X2X_WKUP_TMR_REG_ADDR                      (0x000E)
#define   GH3X2X_ADC0_DATA_L_REG_ADDR                   (0x0010)
#define   GH3X2X_ADC0_DATA_H_REG_ADDR                   (0x0012)
#define   GH3X2X_ADC1_DATA_L_REG_ADDR                   (0x0014)
#define   GH3X2X_ADC1_DATA_H_REG_ADDR                   (0x0016)
#define   GH3X2X_ADC2_DATA_L_REG_ADDR                   (0x0018)
#define   GH3X2X_ADC2_DATA_H_REG_ADDR                   (0x001A)
#define   GH3X2X_ADC3_DATA_L_REG_ADDR                   (0x001C)
#define   GH3X2X_ADC3_DATA_H_REG_ADDR                   (0x001E)
#define   GH3X2X_PAD_CTRL0_REG_ADDR                     (0x0020)
#define   GH3X2X_PAD_CTRL1_REG_ADDR                     (0x0022)
#define   GH3X2X_PAD_CTRL2_REG_ADDR                     (0x0024)
#define   GH3X2X_PAD_CTRL3_REG_ADDR                     (0x0026)
#define   GH3X2X_PAD_CTRL4_REG_ADDR                     (0x0028)
#define   GH3X2X_PAD_CTRL5_REG_ADDR                     (0x002A)
#define   GH3X2X_PRODUCT_L_REG_ADDR                     (0x0030)    /**< product id l */
#define   GH3X2X_PRODUCT_H_REG_ADDR                     (0x0032)    /**< product id h */
#define   GH3X2X_CHIP_ID_REG_ADDR                       (0x0034)    /**< chip id */
#define   GH3X2X_CHIP_READY_CODE_REG_ADDR               (0x0036)
#define   GH3X2X_DYN_CKGT_CTRL_REG_ADDR                 (0x0038)
#define   GH3X2X_ACCESS_READY_CODE_ADDR                 (0x0050)
#define   GH3X2X_CHIP_ECG_BACKDOOR_REG_ADDR             (0x0070)
#define   GH3X2X_INSTRUCTIONS_CHIP_INIED_REG_ADDR       (0x0072)    /**< reg addr 0x72:rg_chip_sw_backup. */
#define   GH3X2X_CHIP_ECO_RESERVE_REG_ADDR              (0x0074)

// pmu block, start addr: 0x0080
#define   GH3X2X_PMU_CTRL0_REG_ADDR                     (0x0080)
#define   GH3X2X_PMU_CTRL1_REG_ADDR                     (0x0082)
#define   GH3X2X_PMU_CTRL2_REG_ADDR                     (0x0084)
#define   GH3X2X_PMU_CTRL3_REG_ADDR                     (0x0086)
#define   GH3X2X_PMU_CTRL4_REG_ADDR                     (0x0088)    /**< pmu fifo power ctrl */
#define   GH3X2X_PMU_CTRL5_REG_ADDR                     (0x008A)
#define   GH3X2X_PMU_CTRL6_REG_ADDR                     (0x008C)
#define   GH3X2X_PMU_CTRL7_REG_ADDR                     (0x008E)
#define   GH3X2X_PMU_CTRL8_REG_ADDR                     (0x0090)

// timeslot block, start addr: 0x0100
#define   GH3X2X_SLOT_INDEX_CTRL0_REG_ADDR              (0x0100)
#define   GH3X2X_SLOT_INDEX_CTRL1_REG_ADDR              (0x0102)
#define   GH3X2X_SLOT_INDEX_CTRL2_REG_ADDR              (0x0104)
#define   GH3X2X_SLOT_INDEX_CTRL3_REG_ADDR              (0x0106)
#define   GH3X2X_SLOT_ENABLE_CFG_REG_ADDR               (0x0108)    /**< slot enable config */
#define   GH3X2X_SLOT0_CTRL_0_REG_ADDR                  (0x010A)    /**< slot 0 */
#define   GH3X2X_SLOT0_CTRL_1_REG_ADDR                  (0x010C)
#define   GH3X2X_SLOT0_CTRL_2_REG_ADDR                  (0x010E)
#define   GH3X2X_SLOT0_CTRL_3_REG_ADDR                  (0x0110)
#define   GH3X2X_SLOT0_CTRL_4_REG_ADDR                  (0x0112)
#define   GH3X2X_SLOT0_CTRL_5_REG_ADDR                  (0x0114)
#define   GH3X2X_SLOT0_CTRL_6_REG_ADDR                  (0x0116)
#define   GH3X2X_SLOT0_CTRL_7_REG_ADDR                  (0x0118)
#define   GH3X2X_SLOT0_CTRL_8_REG_ADDR                  (0x011A)
#define   GH3X2X_SLOT0_CTRL_9_REG_ADDR                  (0x011C)
#define   GH3X2X_SLOT0_CTRL_10_REG_ADDR                 (0x011E)
#define   GH3X2X_SLOT0_CTRL_11_REG_ADDR                 (0x0120)
#define   GH3X2X_SLOT0_CTRL_12_REG_ADDR                 (0x0122)
#define   GH3X2X_SLOT0_CTRL_13_REG_ADDR                 (0x0124)
#define   GH3X2X_SLOT1_CTRL_0_REG_ADDR                  (0x0126)    /**< slot 1 */
#define   GH3X2X_SLOT1_CTRL_1_REG_ADDR                  (0x0128)
#define   GH3X2X_SLOT1_CTRL_2_REG_ADDR                  (0x012A)
#define   GH3X2X_SLOT1_CTRL_3_REG_ADDR                  (0x012C)
#define   GH3X2X_SLOT1_CTRL_4_REG_ADDR                  (0x012E)
#define   GH3X2X_SLOT1_CTRL_5_REG_ADDR                  (0x0130)
#define   GH3X2X_SLOT1_CTRL_6_REG_ADDR                  (0x0132)
#define   GH3X2X_SLOT1_CTRL_7_REG_ADDR                  (0x0134)
#define   GH3X2X_SLOT1_CTRL_8_REG_ADDR                  (0x0136)
#define   GH3X2X_SLOT1_CTRL_9_REG_ADDR                  (0x0138)
#define   GH3X2X_SLOT1_CTRL_10_REG_ADDR                 (0x013A)
#define   GH3X2X_SLOT1_CTRL_11_REG_ADDR                 (0x013C)
#define   GH3X2X_SLOT1_CTRL_12_REG_ADDR                 (0x013E)
#define   GH3X2X_SLOT1_CTRL_13_REG_ADDR                 (0x0140)
#define   GH3X2X_SLOT2_CTRL_0_REG_ADDR                  (0x0142)    /**< slot 2 */
#define   GH3X2X_SLOT2_CTRL_1_REG_ADDR                  (0x0144)
#define   GH3X2X_SLOT2_CTRL_2_REG_ADDR                  (0x0146)
#define   GH3X2X_SLOT2_CTRL_3_REG_ADDR                  (0x0148)
#define   GH3X2X_SLOT2_CTRL_4_REG_ADDR                  (0x014A)
#define   GH3X2X_SLOT2_CTRL_5_REG_ADDR                  (0x014C)
#define   GH3X2X_SLOT2_CTRL_6_REG_ADDR                  (0x014E)
#define   GH3X2X_SLOT2_CTRL_7_REG_ADDR                  (0x0150)
#define   GH3X2X_SLOT2_CTRL_8_REG_ADDR                  (0x0152)
#define   GH3X2X_SLOT2_CTRL_9_REG_ADDR                  (0x0154)
#define   GH3X2X_SLOT2_CTRL_10_REG_ADDR                 (0x0156)
#define   GH3X2X_SLOT2_CTRL_11_REG_ADDR                 (0x0158)
#define   GH3X2X_SLOT2_CTRL_12_REG_ADDR                 (0x015A)
#define   GH3X2X_SLOT2_CTRL_13_REG_ADDR                 (0x015C)
#define   GH3X2X_SLOT3_CTRL_0_REG_ADDR                  (0x015E)    /**< slot 3 */
#define   GH3X2X_SLOT3_CTRL_1_REG_ADDR                  (0x0160)
#define   GH3X2X_SLOT3_CTRL_2_REG_ADDR                  (0x0162)
#define   GH3X2X_SLOT3_CTRL_3_REG_ADDR                  (0x0164)
#define   GH3X2X_SLOT3_CTRL_4_REG_ADDR                  (0x0166)
#define   GH3X2X_SLOT3_CTRL_5_REG_ADDR                  (0x0168)
#define   GH3X2X_SLOT3_CTRL_6_REG_ADDR                  (0x016A)
#define   GH3X2X_SLOT3_CTRL_7_REG_ADDR                  (0x016C)
#define   GH3X2X_SLOT3_CTRL_8_REG_ADDR                  (0x016E)
#define   GH3X2X_SLOT3_CTRL_9_REG_ADDR                  (0x0170)
#define   GH3X2X_SLOT3_CTRL_10_REG_ADDR                 (0x0172)
#define   GH3X2X_SLOT3_CTRL_11_REG_ADDR                 (0x0174)
#define   GH3X2X_SLOT3_CTRL_12_REG_ADDR                 (0x0176)
#define   GH3X2X_SLOT3_CTRL_13_REG_ADDR                 (0x0178)
#define   GH3X2X_SLOT4_CTRL_0_REG_ADDR                  (0x017A)    /**< slot 4 */
#define   GH3X2X_SLOT4_CTRL_1_REG_ADDR                  (0x017C)
#define   GH3X2X_SLOT4_CTRL_2_REG_ADDR                  (0x017E)
#define   GH3X2X_SLOT4_CTRL_3_REG_ADDR                  (0x0180)
#define   GH3X2X_SLOT4_CTRL_4_REG_ADDR                  (0x0182)
#define   GH3X2X_SLOT4_CTRL_5_REG_ADDR                  (0x0184)
#define   GH3X2X_SLOT4_CTRL_6_REG_ADDR                  (0x0186)
#define   GH3X2X_SLOT4_CTRL_7_REG_ADDR                  (0x0188)
#define   GH3X2X_SLOT4_CTRL_8_REG_ADDR                  (0x018A)
#define   GH3X2X_SLOT4_CTRL_9_REG_ADDR                  (0x018C)
#define   GH3X2X_SLOT4_CTRL_10_REG_ADDR                 (0x018E)
#define   GH3X2X_SLOT4_CTRL_11_REG_ADDR                 (0x0190)
#define   GH3X2X_SLOT4_CTRL_12_REG_ADDR                 (0x0192)
#define   GH3X2X_SLOT4_CTRL_13_REG_ADDR                 (0x0194)
#define   GH3X2X_SLOT5_CTRL_0_REG_ADDR                  (0x0196)    /**< slot 5 */
#define   GH3X2X_SLOT5_CTRL_1_REG_ADDR                  (0x0198)
#define   GH3X2X_SLOT5_CTRL_2_REG_ADDR                  (0x019A)
#define   GH3X2X_SLOT5_CTRL_3_REG_ADDR                  (0x019C)
#define   GH3X2X_SLOT5_CTRL_4_REG_ADDR                  (0x019E)
#define   GH3X2X_SLOT5_CTRL_5_REG_ADDR                  (0x01A0)
#define   GH3X2X_SLOT5_CTRL_6_REG_ADDR                  (0x01A2)
#define   GH3X2X_SLOT5_CTRL_7_REG_ADDR                  (0x01A4)
#define   GH3X2X_SLOT5_CTRL_8_REG_ADDR                  (0x01A6)
#define   GH3X2X_SLOT5_CTRL_9_REG_ADDR                  (0x01A8)
#define   GH3X2X_SLOT5_CTRL_10_REG_ADDR                 (0x01AA)
#define   GH3X2X_SLOT5_CTRL_11_REG_ADDR                 (0x01AC)
#define   GH3X2X_SLOT5_CTRL_12_REG_ADDR                 (0x01AE)
#define   GH3X2X_SLOT5_CTRL_13_REG_ADDR                 (0x01B0)
#define   GH3X2X_SLOT6_CTRL_0_REG_ADDR                  (0x01B2)    /**< slot 6 */
#define   GH3X2X_SLOT6_CTRL_1_REG_ADDR                  (0x01B4)
#define   GH3X2X_SLOT6_CTRL_2_REG_ADDR                  (0x01B6)
#define   GH3X2X_SLOT6_CTRL_3_REG_ADDR                  (0x01B8)
#define   GH3X2X_SLOT6_CTRL_4_REG_ADDR                  (0x01BA)
#define   GH3X2X_SLOT6_CTRL_5_REG_ADDR                  (0x01BC)
#define   GH3X2X_SLOT6_CTRL_6_REG_ADDR                  (0x01BE)
#define   GH3X2X_SLOT6_CTRL_7_REG_ADDR                  (0x01C0)
#define   GH3X2X_SLOT6_CTRL_8_REG_ADDR                  (0x01C2)
#define   GH3X2X_SLOT6_CTRL_9_REG_ADDR                  (0x01C4)
#define   GH3X2X_SLOT6_CTRL_10_REG_ADDR                 (0x01C6)
#define   GH3X2X_SLOT6_CTRL_11_REG_ADDR                 (0x01C8)
#define   GH3X2X_SLOT6_CTRL_12_REG_ADDR                 (0x01CA)
#define   GH3X2X_SLOT6_CTRL_13_REG_ADDR                 (0x01CC)
#define   GH3X2X_SLOT7_CTRL_0_REG_ADDR                  (0x01CE)    /**< slot 7 */
#define   GH3X2X_SLOT7_CTRL_1_REG_ADDR                  (0x01D0)
#define   GH3X2X_SLOT7_CTRL_2_REG_ADDR                  (0x01D2)
#define   GH3X2X_SLOT7_CTRL_3_REG_ADDR                  (0x01D4)
#define   GH3X2X_SLOT7_CTRL_4_REG_ADDR                  (0x01D6)
#define   GH3X2X_SLOT7_CTRL_5_REG_ADDR                  (0x01D8)
#define   GH3X2X_SLOT7_CTRL_6_REG_ADDR                  (0x01DA)
#define   GH3X2X_SLOT7_CTRL_7_REG_ADDR                  (0x01DC)
#define   GH3X2X_SLOT7_CTRL_8_REG_ADDR                  (0x01DE)
#define   GH3X2X_SLOT7_CTRL_9_REG_ADDR                  (0x01E0)
#define   GH3X2X_SLOT7_CTRL_10_REG_ADDR                 (0x01E2)
#define   GH3X2X_SLOT7_CTRL_11_REG_ADDR                 (0x01E4)
#define   GH3X2X_SLOT7_CTRL_12_REG_ADDR                 (0x01E6)
#define   GH3X2X_SLOT7_CTRL_13_REG_ADDR                 (0x01E8)
#define   GH3X2X_ECG_CTRL_REG_ADDR                      (0x01EA)
#define   GH3X2X_RG_SLOT_TMR0_REG_ADDR                  (0x01EC)
#define   GH3X2X_RG_SLOT_TMR1_REG_ADDR                  (0x01EE)
#define   GH3X2X_RG_SLOT_TMR2_REG_ADDR                  (0x01F0)
#define   GH3X2X_RG_SLOT_TMR3_REG_ADDR                  (0x01F2)
#define   GH3X2X_RG_SLOT_TMR4_REG_ADDR                  (0x01F4)
#define   GH3X2X_RG_SLOT_TMR5_REG_ADDR                  (0x01F6)
#define   GH3X2X_RG_SLOT_TMR6_REG_ADDR                  (0x01F8)
#define   GH3X2X_RG_SLOT_TMR7_REG_ADDR                  (0x01FA)

// afe_top block, start addr: 0x0200
#define   GH3X2X_AFE_REG0_REG_ADDR                      (0x0200)
#define   GH3X2X_AFE_REG1_REG_ADDR                      (0x0202)
#define   GH3X2X_AFE_REG2_REG_ADDR                      (0x0204)
#define   GH3X2X_AFE_REG3_REG_ADDR                      (0x0206)
#define   GH3X2X_AFE_REG4_REG_ADDR                      (0x0208)
#define   GH3X2X_AFE_REG5_REG_ADDR                      (0x020A)
#define   GH3X2X_AFE_REG6_REG_ADDR                      (0x020C)
#define   GH3X2X_AFE_REG7_REG_ADDR                      (0x020E)
#define   GH3X2X_AFE_REG8_REG_ADDR                      (0x0210)
#define   GH3X2X_AFE_REG9_REG_ADDR                      (0x0212)
#define   GH3X2X_AFE_REG10_REG_ADDR                     (0x0214)

// led_agc block, start addr: 0x0280
#define   GH3X2X_LED_AGC_CTRL0_REG_ADDR                 (0x0280)
#define   GH3X2X_LED_AGC_CTRL1_REG_ADDR                 (0x0282)
#define   GH3X2X_LED_AGC_CTRL2_REG_ADDR                 (0x0284)
#define   GH3X2X_LED_AGC_CTRL3_REG_ADDR                 (0x0286)
#define   GH3X2X_LED_AGC_CTRL4_REG_ADDR                 (0x0288)
#define   GH3X2X_LED_AGC_CTRL5_REG_ADDR                 (0x028A)
#define   GH3X2X_LED_AGC_CTRL6_REG_ADDR                 (0x028C)
#define   GH3X2X_LED_AGC_CTRL7_REG_ADDR                 (0x028E)
#define   GH3X2X_LED_AGC_CTRL8_REG_ADDR                 (0x0290)
#define   GH3X2X_LED_AGC_CTRL9_REG_ADDR                 (0x0292)

// decimation block, start addr: 0x0300
#define   GH3X2X_DCMT_CTRL0_REG_ADDR                    (0x0300)
#define   GH3X2X_DCMT_CTRL1_REG_ADDR                    (0x0302)
#define   GH3X2X_DCMT_CTRL2_REG_ADDR                    (0x0304)
#define   GH3X2X_DCMT_CTRL3_REG_ADDR                    (0x0306)
#define   GH3X2X_DCMT_CTRL4_REG_ADDR                    (0x0308)
#define   GH3X2X_DCMT_CTRL5_REG_ADDR                    (0x030A)
#define   GH3X2X_DCMT_CTRL6_REG_ADDR                    (0x030C)
#define   GH3X2X_DCMT_CTRL7_REG_ADDR                    (0x0310)
#define   GH3X2X_DCMT_CTRL8_REG_ADDR                    (0x0312)

// spi_i2c block, start addr: 0x0380
#define   GH3X2X_SPI_CTRL0_REG_ADDR                     (0x0380)

// adt block, start addr: 0x0400
#define   GH3X2X_ADT_LEADON_CR_REG_ADDR                 (0x0400)
#define   GH3X2X_ADT_LEADON_TR_REG_ADDR                 (0x0402)
#define   GH3X2X_ADT_LEADON_PR_REG_ADDR                 (0x0404)
#define   GH3X2X_ADT_LEADON_PR1_REG_ADDR                (0x0406)
#define   GH3X2X_ADT_WEARON_CR_REG_ADDR                 (0x0408)
#define   GH3X2X_ADT_WEARON1_THU_L_REG_ADDR             (0x0410)
#define   GH3X2X_ADT_WEARON1_THU_H_REG_ADDR             (0x0412)
#define   GH3X2X_ADT_WEARON1_THD_L_REG_ADDR             (0x0414)
#define   GH3X2X_ADT_WEARON1_THD_H_REG_ADDR             (0x0416)
#define   GH3X2X_ADT_WEARON2_THU_L_REG_ADDR             (0x0418)
#define   GH3X2X_ADT_WEARON2_THU_H_REG_ADDR             (0x041A)
#define   GH3X2X_ADT_WEARON2_THD_L_REG_ADDR             (0x041C)
#define   GH3X2X_ADT_WEARON2_THD_H_REG_ADDR             (0x041E)
#define   GH3X2X_ADT_LEADON_CR2_REG_ADDR                (0x0420)
#define   GH3X2X_ADT_WEARON_PR_REG_ADDR                 (0x0422)
#define   GH3X2X_ADT_WEARON_CONFIRM_TIME_REG_ADDR       (0x0424)
#define   GH3X2X_ADT_WEARON_LOGIC_SEL_REG_ADDR          (0x0426)
#define   GH3X2X_ADT_WEAROFF_LOGIC_SEL_REG_ADDR         (0x0428)
#define   GH3X2X_ADT_LEADON_STR_REG_ADDR                (0x042A)

// ecg_recovery block, start addr: 0x0480
#define   GH3X2X_ECG_RECOVERY_CR_REG_ADDR               (0x0480)
#define   GH3X2X_ECG_RECOVERY_DMCR_REG_ADDR             (0x0482)

// int_ctrl block, start addr: 0x0500
#define   GH3X2X_INT_CR_REG_ADDR                        (0x0500)
#define   GH3X2X_INT_CR2_REG_ADDR                       (0x0502)
#define   GH3X2X_INT_PWR_REG_ADDR                       (0x0504)
#define   GH3X2X_INT_CTR_REG_ADDR                       (0x0506)
#define   GH3X2X_INT_STR_REG_ADDR                       (0x0508)
//#define   GH3X2X_INT_FIFO_UR_REG_ADDR                   (0x050A)     /**< fifo use cnt */
#define   GH3X2X_INT_STR2_REG_ADDR                      (0x050C)

// osc_cal block, start addr: 0x0580
#define   GH3X2X_OSC_CR_REG_ADDR                        (0x0580)
#define   GH3X2X_OSC_THR_REG_ADDR                       (0x0582)
#define   GH3X2X_OSC13M_TUNE_REG_ADDR                   (0x0584)
#define   GH3X2X_OSC32K_TUNE_REG_ADDR                   (0x0586)
#define   GH3X2X_OSC32K_TEMP_REG_ADDR                   (0x0588)
#define   GH3X2X_OSC_FREQ_ERR_UR_REG_ADDR               (0x058A)
#define   GH3X2X_OSC_FLAG_REG_ADDR                      (0x058C)

// skin_color block, start addr: 0x0600
#define   GH3X2X_SKIN_BLR_REG_ADDR                      (0x0600)
#define   GH3X2X_SKIN_BHR_REG_ADDR                      (0x0602)
#define   GH3X2X_SKIN_YLR_REG_ADDR                      (0x0604)
#define   GH3X2X_SKIN_YHR_REG_ADDR                      (0x0606)
#define   GH3X2X_SKIN_FLR_REG_ADDR                      (0x0608)
#define   GH3X2X_SKIN_FHR_REG_ADDR                      (0x060A)
#define   GH3X2X_SKIN_CR_REG_ADDR                       (0x060C)
#define   GH3X2X_SKIN_STR_REG_ADDR                      (0x060E)

// sys_afe block, start addr: 0x0680
#define   GH3X2X_PPG_TIA_AD_REG_REG_ADDR                (0x0680)
#define   GH3X2X_PPG_DAC_AD_REG0_REG_ADDR               (0x0682)
#define   GH3X2X_PPG_DAC_AD_REG1_REG_ADDR               (0x0684)
#define   GH3X2X_PPG_DAC_AD_REG2_REG_ADDR               (0x0686)
#define   GH3X2X_PPG_DAC_AD_REG3_REG_ADDR               (0x0688)
#define   GH3X2X_PPG_ADC_AD_REG_REG_ADDR                (0x0690)
#define   GH3X2X_LED_DRV_AD_REG_REG_ADDR                (0x0692)
#define   GH3X2X_ECG_IA_AD_REG_REG_ADDR                 (0x0694)
#define   GH3X2X_ECG_IA_AD_REG1_REG_ADDR                (0x0696)
#define   GH3X2X_ECG_IA_AD_REG2_REG_ADDR                (0x0698)
#define   GH3X2X_ECG_IA_AD_REG3_REG_ADDR                (0x069A)
#define   GH3X2X_ECG_IA_AD_REG4_REG_ADDR                (0x069C)
#define   GH3X2X_ECG_IA_AD_REG5_REG_ADDR                (0x06A0)
#define   GH3X2X_ECG_IA_AD_REG6_REG_ADDR                (0x06A2)
#define   GH3X2X_SYS_AFE_REG0_REG_ADDR                  (0x06B0)
#define   GH3X2X_SYS_AD_REG_REG_ADDR                    (0x06B2)

// efuse_ctrl block, start addr: 0x0700
#define   GH3X2X_EFUSE_CTRL_CMD_REG_ADDR                (0x0700)
#define   GH3X2X_EFUSE_CTRL_PASSWORD_REG_ADDR           (0x0702)
#define   GH3X2X_EFUSE_CTRL_TIMING_CFG_0_REG_ADDR       (0x0704)
#define   GH3X2X_EFUSE_CTRL_TIMING_CFG_1_REG_ADDR       (0x0706)
#define   GH3X2X_EFUSE_CTRL_WDATA_0_REG_ADDR            (0x0708)
#define   GH3X2X_EFUSE_CTRL_WDATA_1_REG_ADDR            (0x070A)
#define   GH3X2X_EFUSE_CTRL_WDATA_2_REG_ADDR            (0x070C)
#define   GH3X2X_EFUSE_CTRL_WDATA_3_REG_ADDR            (0x070E)
#define   GH3X2X_EFUSE_CTRL_RDATA_0_REG_ADDR            (0x0710)
#define   GH3X2X_EFUSE_CTRL_RDATA_1_REG_ADDR            (0x0712)
#define   GH3X2X_EFUSE_CTRL_RDATA_2_REG_ADDR            (0x0714)
#define   GH3X2X_EFUSE_CTRL_RDATA_3_REG_ADDR            (0x0716)
#define   GH3X2X_EFUSE_CTRL_STATUS_REG_ADDR             (0x0718)
#define   GH3X2X_EFUSE_CTRL_REG_MODE_REG_ADDR           (0x071A)
#define   GH3X2X_EFUSE_CTRL_EFUSE0_AUTOLOAD_0_ADDR      (0x071C)
#define   GH3X2X_EFUSE_CTRL_EFUSE0_AUTOLOAD_1_ADDR      (0x071E)
#define   GH3X2X_EFUSE_CTRL_EFUSE0_AUTOLOAD_2_ADDR      (0x0720)
#define   GH3X2X_EFUSE_CTRL_EFUSE0_AUTOLOAD_3_ADDR      (0x0722)
#define   GH3X2X_EFUSE_CTRL_EFUSE1_AUTOLOAD_0_ADDR      (0x0724)
#define   GH3X2X_EFUSE_CTRL_EFUSE1_AUTOLOAD_1_ADDR      (0x0726)
#define   GH3X2X_EFUSE_CTRL_EFUSE1_AUTOLOAD_2_ADDR      (0x0728)
#define   GH3X2X_EFUSE_CTRL_EFUSE1_AUTOLOAD_3_ADDR      (0x072A)
#define   GH3X2X_EFUSE_CTRL_EFUSE2_AUTOLOAD_0_ADDR      (0x072C)
#define   GH3X2X_EFUSE_CTRL_EFUSE2_AUTOLOAD_1_ADDR      (0x072E)
#define   GH3X2X_EFUSE_CTRL_EFUSE2_AUTOLOAD_2_ADDR      (0x0730)
#define   GH3X2X_EFUSE_CTRL_EFUSE2_AUTOLOAD_3_ADDR      (0x0732)

// special reg addr
#define   GH3X2X_FIFO_REG_ADDR                      (0xAAAA)    /**< fifo reg addr */
#define   GH3X2X_I2C_CMD_ADDR                       (0xDDDD)    /**< i2c cmd reg addr */


#define   GH3X2X_REG_ADDR_SIZE                      (0x0002)    /**< reg addr size */
#define   GH3X2X_REG_ADDR_EVEN_FIXED                (0xFFFE)    /**< reg addr even fixed */
#define   GH3X2X_REG_ADDR_MAX                       (0xFFFF)    /**< reg addr max */

/* Register bit definition */ 

#define   GH3X2X_CARDIFF_CTRL_START_BIT             (0x0001u)   /**< start bit msk */

#define   GH3X2X_CHIP_ID_REG_VAL                    (0x0011)    /**< chip id value  */
#define   GH3X2X_PRODUCT_L_REG_VAL                  (0x0201)    /**< product l value */
#define   GH3X2X_PRODUCT_H_REG_VAL                  (0x0301)    /**< product l value */
#define   GH3X2X_CHIP_READY_REG_VAL                 (0xAA55)    /**< chip ready value */

#define   GH3X2X_INT_STR_MSK_ALL_BIT                (0x7FFFu)   /**< irq status mask */
#define   GH3X2X_INT_CTRL_MODE_MSK_BIT              (0x0003u)   /**< int ctrl mode bit msk */

#define   GH3X2X_PMU_CTRL4_FIFO_DISABLE_MASK        (0x0003u)   /**< fifo module disable mask */

#define   GH3X2X_INSTRUCTIONS_CHIP_INIED_REG_VAL    (0x0011)    /**< reg rg_chip_sw_backup bit4&bit0 set 1(0x0011) */

#define   GH3X2X_SLOT_CTRL_OFFSET                   (0x001C)    /**< slot ctrl reg offset */

#define   GH3X2X_ADT_WEAR_STATUS_MASK_BIT           (0x0010u)   /**< wear status bit mask */
#define   GH3X2X_ADT_WEAR_DET_EN_MASK_BIT           (0x0001u)   /**< wear detect enable bit mask */
#define   GH3X2X_ADT_WEAR_CR_WEAR_ON_VAL            (0x0A00u)   /**< wear detect ctrl reg wear on val! cardful ! */
#define   GH3X2X_ADT_WEAR_CR_WEAR_OFF_VAL           (0x0500u)   /**< wear detect ctrl reg wear off val! cardful ! */

#define   GH3X2X_SKIN_COLOR_STATUS_MASK_BIT         (0x0003u)   /**< skin color status bit mask */

#define   GH3X2X_SLOT_LED_CURRENT_CLEAR_MASK        (0x00FFu)   /**< slot led current clear mask */

/* utils reg macro */

#define   GH3X2X_SLOT_NUM_MAX                       (8)         /**< slot num max */

#define   GH3X2X_SLOT_LED_DRV_NUM_MAX               (2)         /**< slot led driver max */

#define   GH3X2X_SLOT_TIA_GAIN_NUM_MAX              (4)         /**< slot tia gain num max */

#define   GH3X2X_SLOT_TIA_GAIN_VAL_MAX              (13)        /**< slot tia gain val max */

#define   GH3X2X_SLOT_TIA_GAIN_BITS_SIZE            (4)         /**< slot tia gain val bits size */

#define   GH3X2X_SLOT_TIA_GAIN_BITS_MARK            (0x000F)    /**< slot tia gain val bits mark */

#define   GH3X2X_SLOT_KDC_THR_VAL_MAX               (8)         /**< kdc thr val max */

#define   GH3X2X_SLOT_KDC_THR_BITS_SIZE             (4)         /**< kdc thr bits size */

#define   GH3X2X_SLOT_KDC_THR_BITS_MARK             (0x0007)    /**< kdc thr bits mark */

#define   GH3X2X_SLOT_KDC_THR_ADC_INDEX_MAX         (3)         /**< kdc thr adc index max */


/* reg bit field index */

#define   GH3X2X_ECG_FAST_RECV_EN_LSB               (10)        /**< ecg fast recv en lsb @GH3X2X_ECG_CTRL_REG */
#define   GH3X2X_ECG_FAST_RECV_EN_MSB               (10)        /**< ecg fast recv en msb @GH3X2X_ECG_CTRL_REG */

#define   GH3X2X_ECG_LEAD_DET_EN_LSB                (0)         /**< ecg lead det en lsb @GH3X2X_ADT_LEADON_CR_REG */
#define   GH3X2X_ECG_LEAD_DET_EN_MSB                (0)         /**< ecg lead det en msb @GH3X2X_ADT_LEADON_CR_REG */

#define   GH3X2X_LEAD_ON_AUTO_SWITCH_LSB            (8)         /**< lead on auto switch lsb @GH3X2X_ECG_CTRL_REG */
#define   GH3X2X_LEAD_ON_AUTO_SWITCH_MSB            (8)         /**< lead on auto switch msb @GH3X2X_ECG_CTRL_REG */

#define   GH3X2X_LEAD_OFF_AUTO_SWITCH_LSB           (9)         /**< lead off auto switch lsb @GH3X2X_ECG_CTRL_REG */
#define   GH3X2X_LEAD_OFF_AUTO_SWITCH_MSB           (9)         /**< lead off auto switch msb @GH3X2X_ECG_CTRL_REG */


#define  GH3X2X_WEAR_ON_EVENT_EN_LSB                (10)        /**< wear on event en lsb*/
#define  GH3X2X_WEAR_ON_EVENT_EN_MSB                (10)        /**< wear on event en msb*/

#define  GH3X2X_WEAR_OFF_EVENT_EN_LSB               (11)        /**< wear off event en lsb*/
#define  GH3X2X_WEAR_OFF_EVENT_EN_MSB               (11)        /**< wear off event en msb*/


#define  GH3X2X_DRV0_FULL_SCAL_CURRENT_LSB          (0)
#define  GH3X2X_DRV0_FULL_SCAL_CURRENT_MSB          (1)

#define  GH3X2X_DRV1_FULL_SCAL_CURRENT_LSB          (4)
#define  GH3X2X_DRV1_FULL_SCAL_CURRENT_MSB          (5)


#define  GH3X2X_SLOT_AGC_EN_LSB                     (8)
#define  GH3X2X_SLOT_AGC_EN_MSB                     (9)

#define  GH3X2X_SLOT_BG_LEVEL_LSB                   (4)
#define  GH3X2X_SLOT_BG_LEVEL_MSB                   (5)

#define  GH3X2X_SLOT_BG_CANCEL_LSB                  (10)
#define  GH3X2X_SLOT_BG_CANCEL_MSB                  (10)

#define  GH3X2X_SLOT_ADC_INT_TIME_LSB               (0)
#define  GH3X2X_SLOT_ADC_INT_TIME_MSB               (3)

#define  GH3X2X_SLOT_SR_LSB                         (8)
#define  GH3X2X_SLOT_SR_MSB                         (15)






#endif /* _GH3X2X_DRV_REGISTERS_H_ */

/********END OF FILE********* Copyright (c) 2003 - 2021, Goodix Co., Ltd. ********/

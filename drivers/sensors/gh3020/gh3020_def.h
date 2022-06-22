/****************************************************************************
 * drivers/sensors/gh3020/gh3020_def.h
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

#ifndef __DRIVERS_SENSORS_GH3020_DEF_H
#define __DRIVERS_SENSORS_GH3020_DEF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/sensor.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Part1. Register mapping and values */

/* Part1.1 Register addresses */

/* Top ctrl block, start addr: 0x0000 */

#define GH3020_REG_CARDIFF_CTRL            0x0000  /* System ctrl */
#define GH3020_REG_SYSCLK_CTRL             0x0002  /* System clock ctrl */
#define GH3020_REG_SYS_SAMPLE_RATE_CTRL    0x0004  /* System sample rate ctrl */
#define GH3020_REG_DATA_CTRL0              0x0006  /* Data ctrl0 */
#define GH3020_REG_DATA_CTRL1              0x0008  /* Data ctrl1 */
#define GH3020_REG_FIFO_WATERLINE          0x000a  /* FIFO watermark thres */
#define GH3020_REG_WKUP_TMR                0x000e  /* Wakeup timer */
#define GH3020_REG_ADC0_DATA_L             0x0010  /* ADC0 data low byte */
#define GH3020_REG_ADC0_DATA_H             0x0012  /* ADC0 data high byte */
#define GH3020_REG_ADC1_DATA_L             0x0014  /* ADC1 data low byte */
#define GH3020_REG_ADC1_DATA_H             0x0016  /* ADC1 data high byte */
#define GH3020_REG_ADC2_DATA_L             0x0018  /* ADC2 data low byte */
#define GH3020_REG_ADC2_DATA_H             0x001a  /* ADC2 data high byte */
#define GH3020_REG_ADC3_DATA_L             0x001c  /* ADC3 data low byte */
#define GH3020_REG_ADC3_DATA_H             0x001e  /* ADC3 data high byte */
#define GH3020_REG_PAD_CTRL0               0x0020  /* PAD ctrl0 */
#define GH3020_REG_PAD_CTRL1               0x0022  /* PAD ctrl1 */
#define GH3020_REG_PAD_CTRL2               0x0024  /* PAD ctrl2 */
#define GH3020_REG_PAD_CTRL3               0x0026  /* PAD ctrl3 */
#define GH3020_REG_PAD_CTRL4               0x0028  /* PAD ctrl4 */
#define GH3020_REG_PAD_CTRL5               0x002a  /* PAD ctrl5 */
#define GH3020_REG_PRODUCT_L               0x0030  /* Product ID low byte */
#define GH3020_REG_PRODUCT_H               0x0032  /* Product ID high byte */
#define GH3020_REG_CHIP_ID                 0x0034  /* Chip ID */
#define GH3020_REG_CHIP_READY_CODE         0x0036  /* Chip ready code. Read only */
#define GH3020_REG_DYN_CKGT_CTRL           0x0038  /* DYN CKGT ctrl */
#define GH3020_REG_ACCESS_READY_CODE       0x0050  /* Access ready */
#define GH3020_REG_CHIP_ECG_BACKDOOR       0x0070  /* Chip ECG backdoor */
#define GH3020_REG_INSTRUCTIONS_CHIP_INIED 0x0072  /* rg_chip_sw_backup */
#define GH3020_REG_CHIP_ECO_RESERVE        0x0074  /* Chip ECO reserve */

/* PMU block, start addr: 0x0080 */

#define GH3020_REG_PMU_CTRL0               0x0080  /* PMU ctrl0 */
#define GH3020_REG_PMU_CTRL1               0x0082  /* PMU ctrl1 */
#define GH3020_REG_PMU_CTRL2               0x0084  /* PMU ctrl2 */
#define GH3020_REG_PMU_CTRL3               0x0086  /* PMU ctrl3 */
#define GH3020_REG_PMU_CTRL4               0x0088  /* PMU FIFO power ctrl */
#define GH3020_REG_PMU_CTRL5               0x008a  /* PMU ctrl5 */
#define GH3020_REG_PMU_CTRL6               0x008c  /* PMU ctrl6 */
#define GH3020_REG_PMU_CTRL7               0x008e  /* PMU ctrl7 */
#define GH3020_REG_PMU_CTRL8               0x0090  /* PMU ctrl8 */

/* timeslot block, start addr: 0x0100 */

#define GH3020_REG_SLOT_INDEX_CTRL0        0x0100  /* Slot index ctrl0 */
#define GH3020_REG_SLOT_INDEX_CTRL1        0x0102  /* Slot index ctrl1 */
#define GH3020_REG_SLOT_INDEX_CTRL2        0x0104  /* Slot index ctrl2 */
#define GH3020_REG_SLOT_INDEX_CTRL3        0x0106  /* Slot index ctrl3 */
#define GH3020_REG_SLOT_ENABLE_CFG         0x0108  /* Slot enable config */
#define GH3020_REG_SLOT0_CTRL_0            0x010a  /* Slot0 ctrl0 */
#define GH3020_REG_SLOT0_CTRL_1            0x010c  /* Slot0 ctrl1 */
#define GH3020_REG_SLOT0_CTRL_2            0x010e  /* Slot0 ctrl2 */
#define GH3020_REG_SLOT0_CTRL_3            0x0110  /* Slot0 ctrl3 */
#define GH3020_REG_SLOT0_CTRL_4            0x0112  /* Slot0 ctrl4 */
#define GH3020_REG_SLOT0_CTRL_5            0x0114  /* Slot0 ctrl5 */
#define GH3020_REG_SLOT0_CTRL_6            0x0116  /* Slot0 ctrl6 */
#define GH3020_REG_SLOT0_CTRL_7            0x0118  /* Slot0 ctrl6 */
#define GH3020_REG_SLOT0_CTRL_8            0x011a  /* Slot0 ctrl7 */
#define GH3020_REG_SLOT0_CTRL_9            0x011c  /* Slot0 ctrl8 */
#define GH3020_REG_SLOT0_CTRL_10           0x011e  /* Slot0 ctrl9 */
#define GH3020_REG_SLOT0_CTRL_11           0x0120  /* Slot0 ctrl10 */
#define GH3020_REG_SLOT0_CTRL_12           0x0122  /* Slot0 ctrl11 */
#define GH3020_REG_SLOT0_CTRL_13           0x0124  /* Slot0 ctrl12 */
#define GH3020_REG_SLOT1_CTRL_0            0x0126  /* Slot1 ctrl0 */
#define GH3020_REG_SLOT1_CTRL_1            0x0128  /* Slot1 ctrl1 */
#define GH3020_REG_SLOT1_CTRL_2            0x012a  /* Slot1 ctrl2 */
#define GH3020_REG_SLOT1_CTRL_3            0x012c  /* Slot1 ctrl3 */
#define GH3020_REG_SLOT1_CTRL_4            0x012e  /* Slot1 ctrl4 */
#define GH3020_REG_SLOT1_CTRL_5            0x0130  /* Slot1 ctrl5 */
#define GH3020_REG_SLOT1_CTRL_6            0x0132  /* Slot1 ctrl6 */
#define GH3020_REG_SLOT1_CTRL_7            0x0134  /* Slot1 ctrl7 */
#define GH3020_REG_SLOT1_CTRL_8            0x0136  /* Slot1 ctrl8 */
#define GH3020_REG_SLOT1_CTRL_9            0x0138  /* Slot1 ctrl9 */
#define GH3020_REG_SLOT1_CTRL_10           0x013a  /* Slot1 ctrl10 */
#define GH3020_REG_SLOT1_CTRL_11           0x013c  /* Slot1 ctrl11 */
#define GH3020_REG_SLOT1_CTRL_12           0x013e  /* Slot1 ctrl12 */
#define GH3020_REG_SLOT1_CTRL_13           0x0140  /* Slot1 ctrl13 */
#define GH3020_REG_SLOT2_CTRL_0            0x0142  /* Slot2 ctrl0 */
#define GH3020_REG_SLOT2_CTRL_1            0x0144  /* Slot2 ctrl1 */
#define GH3020_REG_SLOT2_CTRL_2            0x0146  /* Slot2 ctrl2 */
#define GH3020_REG_SLOT2_CTRL_3            0x0148  /* Slot2 ctrl3 */
#define GH3020_REG_SLOT2_CTRL_4            0x014a  /* Slot2 ctrl4 */
#define GH3020_REG_SLOT2_CTRL_5            0x014c  /* Slot2 ctrl5 */
#define GH3020_REG_SLOT2_CTRL_6            0x014e  /* Slot2 ctrl6 */
#define GH3020_REG_SLOT2_CTRL_7            0x0150  /* Slot2 ctrl7 */
#define GH3020_REG_SLOT2_CTRL_8            0x0152  /* Slot2 ctrl8 */
#define GH3020_REG_SLOT2_CTRL_9            0x0154  /* Slot2 ctrl9 */
#define GH3020_REG_SLOT2_CTRL_10           0x0156  /* Slot2 ctrl10 */
#define GH3020_REG_SLOT2_CTRL_11           0x0158  /* Slot2 ctrl11 */
#define GH3020_REG_SLOT2_CTRL_12           0x015a  /* Slot2 ctrl12 */
#define GH3020_REG_SLOT2_CTRL_13           0x015c  /* Slot2 ctrl13 */
#define GH3020_REG_SLOT3_CTRL_0            0x015e  /* Slot3 ctrl0 */
#define GH3020_REG_SLOT3_CTRL_1            0x0160  /* Slot3 ctrl1 */
#define GH3020_REG_SLOT3_CTRL_2            0x0162  /* Slot3 ctrl2 */
#define GH3020_REG_SLOT3_CTRL_3            0x0164  /* Slot3 ctrl3 */
#define GH3020_REG_SLOT3_CTRL_4            0x0166  /* Slot3 ctrl4 */
#define GH3020_REG_SLOT3_CTRL_5            0x0168  /* Slot3 ctrl5 */
#define GH3020_REG_SLOT3_CTRL_6            0x016a  /* Slot3 ctrl6 */
#define GH3020_REG_SLOT3_CTRL_7            0x016c  /* Slot3 ctrl7 */
#define GH3020_REG_SLOT3_CTRL_8            0x016e  /* Slot3 ctrl8 */
#define GH3020_REG_SLOT3_CTRL_9            0x0170  /* Slot3 ctrl9 */
#define GH3020_REG_SLOT3_CTRL_10           0x0172  /* Slot3 ctrl10 */
#define GH3020_REG_SLOT3_CTRL_11           0x0174  /* Slot3 ctrl11 */
#define GH3020_REG_SLOT3_CTRL_12           0x0176  /* Slot3 ctrl12 */
#define GH3020_REG_SLOT3_CTRL_13           0x0178  /* Slot3 ctrl13 */
#define GH3020_REG_SLOT4_CTRL_0            0x017a  /* Slot4 ctrl0 */
#define GH3020_REG_SLOT4_CTRL_1            0x017c  /* Slot4 ctrl1 */
#define GH3020_REG_SLOT4_CTRL_2            0x017e  /* Slot4 ctrl2 */
#define GH3020_REG_SLOT4_CTRL_3            0x0180  /* Slot4 ctrl3 */
#define GH3020_REG_SLOT4_CTRL_4            0x0182  /* Slot4 ctrl4 */
#define GH3020_REG_SLOT4_CTRL_5            0x0184  /* Slot4 ctrl5 */
#define GH3020_REG_SLOT4_CTRL_6            0x0186  /* Slot4 ctrl6 */
#define GH3020_REG_SLOT4_CTRL_7            0x0188  /* Slot4 ctrl7 */
#define GH3020_REG_SLOT4_CTRL_8            0x018a  /* Slot4 ctrl8 */
#define GH3020_REG_SLOT4_CTRL_9            0x018c  /* Slot4 ctrl9 */
#define GH3020_REG_SLOT4_CTRL_10           0x018e  /* Slot4 ctrl10 */
#define GH3020_REG_SLOT4_CTRL_11           0x0190  /* Slot4 ctrl11 */
#define GH3020_REG_SLOT4_CTRL_12           0x0192  /* Slot4 ctrl12 */
#define GH3020_REG_SLOT4_CTRL_13           0x0194  /* Slot4 ctrl13 */
#define GH3020_REG_SLOT5_CTRL_0            0x0196  /* Slot5 ctrl0 */
#define GH3020_REG_SLOT5_CTRL_1            0x0198  /* Slot5 ctrl1 */
#define GH3020_REG_SLOT5_CTRL_2            0x019a  /* Slot5 ctrl2 */
#define GH3020_REG_SLOT5_CTRL_3            0x019c  /* Slot5 ctrl3 */
#define GH3020_REG_SLOT5_CTRL_4            0x019e  /* Slot5 ctrl4 */
#define GH3020_REG_SLOT5_CTRL_5            0x01a0  /* Slot5 ctrl5 */
#define GH3020_REG_SLOT5_CTRL_6            0x01a2  /* Slot5 ctrl6 */
#define GH3020_REG_SLOT5_CTRL_7            0x01a4  /* Slot5 ctrl7 */
#define GH3020_REG_SLOT5_CTRL_8            0x01a6  /* Slot5 ctrl8 */
#define GH3020_REG_SLOT5_CTRL_9            0x01a8  /* Slot5 ctrl9 */
#define GH3020_REG_SLOT5_CTRL_10           0x01aa  /* Slot5 ctrl10 */
#define GH3020_REG_SLOT5_CTRL_11           0x01ac  /* Slot5 ctrl11 */
#define GH3020_REG_SLOT5_CTRL_12           0x01ae  /* Slot5 ctrl12 */
#define GH3020_REG_SLOT5_CTRL_13           0x01b0  /* Slot5 ctrl13 */
#define GH3020_REG_SLOT6_CTRL_0            0x01b2  /* Slot6 ctrl0 */
#define GH3020_REG_SLOT6_CTRL_1            0x01b4  /* Slot6 ctrl1 */
#define GH3020_REG_SLOT6_CTRL_2            0x01b6  /* Slot6 ctrl2 */
#define GH3020_REG_SLOT6_CTRL_3            0x01b8  /* Slot6 ctrl3 */
#define GH3020_REG_SLOT6_CTRL_4            0x01ba  /* Slot6 ctrl4 */
#define GH3020_REG_SLOT6_CTRL_5            0x01bc  /* Slot6 ctrl5 */
#define GH3020_REG_SLOT6_CTRL_6            0x01be  /* Slot6 ctrl6 */
#define GH3020_REG_SLOT6_CTRL_7            0x01c0  /* Slot6 ctrl7 */
#define GH3020_REG_SLOT6_CTRL_8            0x01c2  /* Slot6 ctrl8 */
#define GH3020_REG_SLOT6_CTRL_9            0x01c4  /* Slot6 ctrl9 */
#define GH3020_REG_SLOT6_CTRL_10           0x01c6  /* Slot6 ctrl10 */
#define GH3020_REG_SLOT6_CTRL_11           0x01c8  /* Slot6 ctrl11 */
#define GH3020_REG_SLOT6_CTRL_12           0x01ca  /* Slot6 ctrl12 */
#define GH3020_REG_SLOT6_CTRL_13           0x01cc  /* Slot6 ctrl13 */
#define GH3020_REG_SLOT7_CTRL_0            0x01ce  /* Slot7 ctrl0 */
#define GH3020_REG_SLOT7_CTRL_1            0x01d0  /* Slot7 ctrl1 */
#define GH3020_REG_SLOT7_CTRL_2            0x01d2  /* Slot7 ctrl2 */
#define GH3020_REG_SLOT7_CTRL_3            0x01d4  /* Slot7 ctrl3 */
#define GH3020_REG_SLOT7_CTRL_4            0x01d6  /* Slot7 ctrl4 */
#define GH3020_REG_SLOT7_CTRL_5            0x01d8  /* Slot7 ctrl5 */
#define GH3020_REG_SLOT7_CTRL_6            0x01da  /* Slot7 ctrl6 */
#define GH3020_REG_SLOT7_CTRL_7            0x01dc  /* Slot7 ctrl7 */
#define GH3020_REG_SLOT7_CTRL_8            0x01de  /* Slot7 ctrl8 */
#define GH3020_REG_SLOT7_CTRL_9            0x01e0  /* Slot7 ctrl9 */
#define GH3020_REG_SLOT7_CTRL_10           0x01e2  /* Slot7 ctrl10 */
#define GH3020_REG_SLOT7_CTRL_11           0x01e4  /* Slot7 ctrl11 */
#define GH3020_REG_SLOT7_CTRL_12           0x01e6  /* Slot7 ctrl12 */
#define GH3020_REG_SLOT7_CTRL_13           0x01e8  /* Slot7 ctrl13 */
#define GH3020_REG_ECG_CTRL                0x01ea  /* ECG ctrl */
#define GH3020_REG_SLOT_TMR0               0x01ec  /* Slot0 running timer */
#define GH3020_REG_SLOT_TMR1               0x01ee  /* Slot1 running timer */
#define GH3020_REG_SLOT_TMR2               0x01f0  /* Slot2 running timer */
#define GH3020_REG_SLOT_TMR3               0x01f2  /* Slot3 running timer */
#define GH3020_REG_SLOT_TMR4               0x01f4  /* Slot4 running timer */
#define GH3020_REG_SLOT_TMR5               0x01f6  /* Slot5 running timer */
#define GH3020_REG_SLOT_TMR6               0x01f8  /* Slot6 running timer */
#define GH3020_REG_SLOT_TMR7               0x01fa  /* Slot7 running timer */

/* AFE top block, start addr: 0x0200 */

#define GH3020_REG_AFE0                    0x0200  /* AFE0 */
#define GH3020_REG_AFE1                    0x0202  /* AFE1 */
#define GH3020_REG_AFE2                    0x0204  /* AFE2 */
#define GH3020_REG_AFE3                    0x0206  /* AFE3 */
#define GH3020_REG_AFE4                    0x0208  /* AFE4 */
#define GH3020_REG_AFE5                    0x020a  /* AFE5 */
#define GH3020_REG_AFE6                    0x020c  /* AFE6 */
#define GH3020_REG_AFE7                    0x020e  /* AFE7 */
#define GH3020_REG_AFE8                    0x0210  /* AFE8 */
#define GH3020_REG_AFE9                    0x0212  /* AFE9 */
#define GH3020_REG_AFE10                   0x0214  /* AFE10 */

/* LED AGC block, start addr: 0x0280 */

#define GH3020_REG_LED_AGC_CTRL0           0x0280  /* LED AGC ctrl0 */
#define GH3020_REG_LED_AGC_CTRL1           0x0282  /* LED AGC ctrl1 */
#define GH3020_REG_LED_AGC_CTRL2           0x0284  /* LED AGC ctrl2 */
#define GH3020_REG_LED_AGC_CTRL3           0x0286  /* LED AGC ctrl3 */
#define GH3020_REG_LED_AGC_CTRL4           0x0288  /* LED AGC ctrl4 */
#define GH3020_REG_LED_AGC_CTRL5           0x028a  /* LED AGC ctrl5 */
#define GH3020_REG_LED_AGC_CTRL6           0x028c  /* LED AGC ctrl6 */
#define GH3020_REG_LED_AGC_CTRL7           0x028e  /* LED AGC ctrl7 */
#define GH3020_REG_LED_AGC_CTRL8           0x0290  /* LED AGC ctrl8 */
#define GH3020_REG_LED_AGC_CTRL9           0x0292  /* LED AGC ctrl9 */

/* Decimation block, start addr: 0x0300 */

#define GH3020_REG_DCMT_CTRL0              0x0300  /* Decimation ctrl0 */
#define GH3020_REG_DCMT_CTRL1              0x0302  /* Decimation ctrl1 */
#define GH3020_REG_DCMT_CTRL2              0x0304  /* Decimation ctrl2 */
#define GH3020_REG_DCMT_CTRL3              0x0306  /* Decimation ctrl3 */
#define GH3020_REG_DCMT_CTRL4              0x0308  /* Decimation ctrl4 */
#define GH3020_REG_DCMT_CTRL5              0x030a  /* Decimation ctrl5 */
#define GH3020_REG_DCMT_CTRL6              0x030c  /* Decimation ctrl6 */
#define GH3020_REG_DCMT_CTRL7              0x0310  /* Decimation ctrl7 */
#define GH3020_REG_DCMT_CTRL8              0x0312  /* Decimation ctrl8 */

/* SPI and I2C block, start addr: 0x0380 */

#define GH3020_REG_SPI_CTRL0               0x0380  /* SPI & I2C ID */

/* ADT block, start addr: 0x0400 */

#define GH3020_REG_ADT_LEADON_CR           0x0400  /* ADT lead-on ctrl */
#define GH3020_REG_ADT_LEADON_TR           0x0402  /* ADT lead-on timer */
#define GH3020_REG_ADT_LEADON_PR           0x0404  /* ADT lead-on PR */
#define GH3020_REG_ADT_LEADON_PR1          0x0406  /* ADT lead-on PR1 */
#define GH3020_REG_ADT_WEARON_CR           0x0408  /* ADT wear-on ctrl */
#define GH3020_REG_ADT_WEARON1_THU_L       0x0410  /* ADT wear-on1 upper thres low byte */
#define GH3020_REG_ADT_WEARON1_THU_H       0x0412  /* ADT wear-on1 upper thres high byte */
#define GH3020_REG_ADT_WEARON1_THD_L       0x0414  /* ADT wear-on1 lower thres low byte */
#define GH3020_REG_ADT_WEARON1_THD_H       0x0416  /* ADT wear-on1 lower thres high byte */
#define GH3020_REG_ADT_WEARON2_THU_L       0x0418  /* ADT wear-on2 upper thres low byte */
#define GH3020_REG_ADT_WEARON2_THU_H       0x041a  /* ADT wear-on2 upper thres high byte */
#define GH3020_REG_ADT_WEARON2_THD_L       0x041c  /* ADT wear-on2 lower thres low byte */
#define GH3020_REG_ADT_WEARON2_THD_H       0x041e  /* ADT wear-on2 lower thres high byte */
#define GH3020_REG_ADT_LEADON_CR2          0x0420  /* ADT lead-on ctrl2 */
#define GH3020_REG_ADT_WEARON_PR           0x0422  /* ADT wear-on PR */
#define GH3020_REG_ADT_WEARON_CONFIRM_TIME 0x0424  /* ADT wear-on confirm time */
#define GH3020_REG_ADT_WEARON_LOGIC_SEL    0x0426  /* ADT wear-on logic select */
#define GH3020_REG_ADT_WEAROFF_LOGIC_SEL   0x0428  /* ADT wear-off logic select */
#define GH3020_REG_ADT_LEADON_STR          0x042a  /* ADT lead-on STR */

/* Interrupt ctrl block, start addr: 0x0500 */

#define GH3020_REG_INT_CR                  0x0500  /* Interrupt control */
#define GH3020_REG_INT_CR2                 0x0502  /* Interrupt control2 */
#define GH3020_REG_INT_PWR                 0x0504  /* Interrupt pulse width */
#define GH3020_REG_INT_CTR                 0x0506  /* Interrupt cold time */
#define GH3020_REG_INT_STR                 0x0508  /* Interrupt event */
#define GH3020_REG_INT_STR2                0x050c  /* Interrupt event2 */

/* Oscillator calibration block, start addr: 0x0580 */

#define GH3020_REG_OSC_CR                  0x0580  /* Oscillator ctrl */
#define GH3020_REG_OSC_THR                 0x0582  /* Oscillator THR */
#define GH3020_REG_OSC13M_TUNE             0x0584  /* Oscillator 13MHz tune */
#define GH3020_REG_OSC32K_TUNE             0x0586  /* Oscillator 32kHz tune */
#define GH3020_REG_OSC32K_TEMP             0x0588  /* Oscillator 32kHz temperature */
#define GH3020_REG_OSC_FREQ_ERR_UR         0x058a  /* Oscillator freq error */
#define GH3020_REG_OSC_FLAG                0x058c  /* Oscillator flag */

/* Skin color block, start addr: 0x0600 */

#define GH3020_REG_SKIN_BLR                0x0600  /* Skin BLR */
#define GH3020_REG_SKIN_BHR                0x0602  /* Skin BHR */
#define GH3020_REG_SKIN_YLR                0x0604  /* Skin YLR */
#define GH3020_REG_SKIN_YHR                0x0606  /* Skin YHR */
#define GH3020_REG_SKIN_FLR                0x0608  /* Skin FLR */
#define GH3020_REG_SKIN_FHR                0x060a  /* Skin FHR */
#define GH3020_REG_SKIN_CR                 0x060c  /* Skin ctrl */
#define GH3020_REG_SKIN_STR                0x060e  /* Skin STR */

/* sys_afe block, start addr: 0x0680 */

#define GH3020_REG_PPG_TIA_AD_REG          0x0680  /* PPG TIA */
#define GH3020_REG_PPG_DAC_AD_REG0         0x0682  /* PPG DAC 0 */
#define GH3020_REG_PPG_DAC_AD_REG1         0x0684  /* PPG DAC 1 */
#define GH3020_REG_PPG_DAC_AD_REG2         0x0686  /* PPG DAC 2 */
#define GH3020_REG_PPG_DAC_AD_REG3         0x0688  /* PPG DAC 3 */
#define GH3020_REG_PPG_ADC_AD_REG          0x0690  /* PPG ADC */
#define GH3020_REG_LED_DRV_AD_REG          0x0692  /* LED driver */
#define GH3020_REG_ECG_IA_AD_REG           0x0694  /* ECG IA */
#define GH3020_REG_ECG_IA_AD_REG1          0x0696  /* ECG IA 1 */
#define GH3020_REG_ECG_IA_AD_REG2          0x0698  /* ECG IA 2 */
#define GH3020_REG_ECG_IA_AD_REG3          0x069a  /* ECG IA 3 */
#define GH3020_REG_ECG_IA_AD_REG4          0x069c  /* ECG IA 4 */
#define GH3020_REG_ECG_IA_AD_REG5          0x06a0  /* ECG IA 5 */
#define GH3020_REG_ECG_IA_AD_REG6          0x06a2  /* ECG IA 6 */
#define GH3020_REG_SYS_AFE_REG0            0x06b0  /* System AFE */
#define GH3020_REG_SYS_AD_REG              0x06b2  /* System */

/* EFUSE ctrl block, start addr: 0x0700 */

#define GH3020_REG_EFUSE_CTRL_CMD          0x0700  /* EFUSE ctrl cmd */
#define GH3020_REG_EFUSE_CTRL_PASSWORD     0x0702  /* EFUSE ctrl password */
#define GH3020_REG_EFUSE_CTRL_TIMING_CFG_0 0x0704  /* EFUSE ctrl timing config0 */
#define GH3020_REG_EFUSE_CTRL_TIMING_CFG_1 0x0706  /* EFUSE ctrl timing config2 */
#define GH3020_REG_EFUSE_CTRL_WDATA_0      0x0708  /* EFUSE ctrl write data 0 */
#define GH3020_REG_EFUSE_CTRL_WDATA_1      0x070a  /* EFUSE ctrl write data 1 */
#define GH3020_REG_EFUSE_CTRL_WDATA_2      0x070c  /* EFUSE ctrl write data 2 */
#define GH3020_REG_EFUSE_CTRL_WDATA_3      0x070e  /* EFUSE ctrl write data 3 */
#define GH3020_REG_EFUSE_CTRL_RDATA_0      0x0710  /* EFUSE ctrl read data 0 */
#define GH3020_REG_EFUSE_CTRL_RDATA_1      0x0712  /* EFUSE ctrl read data 1 */
#define GH3020_REG_EFUSE_CTRL_RDATA_2      0x0714  /* EFUSE ctrl read data 2 */
#define GH3020_REG_EFUSE_CTRL_RDATA_3      0x0716  /* EFUSE ctrl read data 3 */
#define GH3020_REG_EFUSE_CTRL_STATUS       0x0718  /* EFUSE ctrl status */
#define GH3020_REG_EFUSE_CTRL_REG_MODE     0x071a  /* EFUSE ctrl register mode */
#define GH3020_REG_EFUSE0_AUTOLOAD_0       0x071c  /* EFUSE0 autoload 0 */
#define GH3020_REG_EFUSE0_AUTOLOAD_1       0x071e  /* EFUSE0 autoload 1 */
#define GH3020_REG_EFUSE0_AUTOLOAD_2       0x0720  /* EFUSE0 autoload 2 */
#define GH3020_REG_EFUSE0_AUTOLOAD_3       0x0722  /* EFUSE0 autoload 3 */
#define GH3020_REG_EFUSE1_AUTOLOAD_0       0x0724  /* EFUSE1 autoload 0 */
#define GH3020_REG_EFUSE1_AUTOLOAD_1       0x0726  /* EFUSE1 autoload 1 */
#define GH3020_REG_EFUSE1_AUTOLOAD_2       0x0728  /* EFUSE1 autoload 2 */
#define GH3020_REG_EFUSE1_AUTOLOAD_3       0x072a  /* EFUSE1 autoload 3 */
#define GH3020_REG_EFUSE2_AUTOLOAD_0       0x072c  /* EFUSE2 autoload 0 */
#define GH3020_REG_EFUSE2_AUTOLOAD_1       0x072e  /* EFUSE2 autoload 1 */
#define GH3020_REG_EFUSE2_AUTOLOAD_2       0x0730  /* EFUSE2 autoload 2 */
#define GH3020_REG_EFUSE2_AUTOLOAD_3       0x0732  /* EFUSE2 autoload 3 */

/* Speical register address */

#define GH3020_REG_FIFO                    0xaaaa  /* FIFO register */

/* Part1.2 Registers' constant values */

#define GH3020_REGVAL_CHIP_READY           0xaa55  /* Chip ready value */
#define GH3020_REGVAL_INSTRUCT_CHIP_INIED  0x0011  /* rg_chip_sw_backup set 0x0011 */
#define GH3020_REGVAL_ADT_WEAR_CR_WEAR_ON  0x0a00u /* wear detect ctrl reg wear on */
#define GH3020_REGVAL_ADT_WEAR_CR_WEAR_OFF 0x0500u /* wear detect ctrl reg wear off */

/* Part1.3 Register bit definitions and masks */

/* Masks */

#define GH3020_MSK_CARDIFF_CTRL_START      0x0001u /* Start bit msk */
#define GH3020_MSK_INT_STR_ALL_BIT         0x7fffu /* IRQ status mask */
#define GH3020_MSK_INT_CTRL_MODE_BIT       0x0003u /* Intrpt ctrl mode bit */
#define GH3020_MSK_PMU_CTRL4_FIFO_DISABLE  0x0003u /* FIFO module disable */
#define GH3020_MSK_ADT_WEAR_STATUS         0x0010u /* Wear status */
#define GH3020_MSK_ADT_WEAR_DET_EN         0x0001u /* Wear detect enable */
#define GH3020_MSK_SKIN_COLOR_STATUS       0x0003u /* Skin color status */
#define GH3020_MSK_SLOT_LED_CURRENT_CLEAR  0x00ffu /* Slot LED current clear */
#define GH3020_MSK_SLOT_TIA_GAIN           0x000fu /* Slot TIA gain value */
#define GH3020_MSK_SLOT_KDC_THR            0x0007u /* kdc thr bits mark */
#define GH3020_MSK_EFUSE_CTRL_CMD_START    0x0010u /* Start EFUSE R/W */
#define GH3020_MSK_EFUSE_CTRL_CMD_EN       0x0020u /* Enable EFUSE clock */
#define GH3020_MSK_EFUSE_CTRL_STATUS_DONE  0x0001u /* EFUSE read finished */
#define GH3020_MSK_EFUSE_CTRL_RDATA1_LED   0X00ffu /* LED driver in EFUSE1 */

/* Bit offset */

#define GH3020_OFFSET_SLOT_CTRL            0x001c  /* slot ctrl reg offset */
#define GH3020_OFFSET_EFUSE1_TIA           8       /* TIA offset in EFUSE1 */

/* Bit filed index */

#define GH3020_WEAR_ON_EVENT_EN_LSB        10      /* Wear on event en LSB */
#define GH3020_WEAR_ON_EVENT_EN_MSB        10      /* Wear on event en MSB */
#define GH3020_WEAR_OFF_EVENT_EN_LSB       11      /* Wear off event en LSB */
#define GH3020_WEAR_OFF_EVENT_EN_MSB       11      /* Wear off event en MSB */
#define GH3020_DRV0_FULL_SCAL_CURRENT_LSB  0       /* LED driver0 full scale current LSB */
#define GH3020_DRV0_FULL_SCAL_CURRENT_MSB  1       /* LED driver0 full scale current MSB */
#define GH3020_DRV1_FULL_SCAL_CURRENT_LSB  4       /* LED driver1 full scale current LSB */
#define GH3020_DRV1_FULL_SCAL_CURRENT_MSB  5       /* LED driver1 full scale current MSB */
#define GH3020_SLOT_AGC_EN_LSB             8       /* AGC enable LSB */
#define GH3020_SLOT_AGC_EN_MSB             9       /* AGC enable MSB */
#define GH3020_SLOT_BG_LEVEL_LSB           4       /* Background level LSB */
#define GH3020_SLOT_BG_LEVEL_MSB           5       /* Background level MSB */
#define GH3020_SLOT_BG_CANCEL_LSB          10      /* Background cancel LSB */
#define GH3020_SLOT_BG_CANCEL_MSB          10      /* Background cancel MSB */
#define GH3020_SLOT_ADC_INT_TIME_LSB       0       /* ADC integrate time LSB*/
#define GH3020_SLOT_ADC_INT_TIME_MSB       3       /* ADC integrate time MSB*/
#define GH3020_SLOT_SR_LSB                 8       /* Sample rate LSB*/
#define GH3020_SLOT_SR_MSB                 15      /* Sample rate MSB*/

/* Part1.4 Utils reg definitions */

#define GH3020_REG_ADDR_SIZE               0x0002  /* Reg addr size */
#define GH3020_REG_ADDR_EVEN_FIXED         0xfffe  /* Reg addr even fixed */
#define GH3020_REG_ADDR_MAX                0xffff  /* Reg addr max */
#define GH3020_SLOT_NUM_MAX                8       /* Slot num max */
#define GH3020_SLOT_LED_DRV_NUM_MAX        2       /* Slot LED driver max */
#define GH3020_SLOT_TIA_GAIN_NUM_MAX       4       /* Slot TIA gain num max */
#define GH3020_SLOT_TIA_GAIN_VAL_MAX       13      /* Slot TIA gain val max */
#define GH3020_SLOT_TIA_GAIN_BITS_SIZE     4       /* Slot TIA gain val bits size */
#define GH3020_SLOT_KDC_THR_VAL_MAX        8       /* kdc thr val max */
#define GH3020_SLOT_KDC_THR_BITS_SIZE      4       /* kdc thr bits size */
#define GH3020_SLOT_KDC_THR_ADC_INDEX_MAX  3       /* kdc thr adc index max */

/* Part2. SPI interface */

#define GH3020_SPI_CMD_WRITE               0xf0    /* Write command */
#define GH3020_SPI_CMD_READ                0xf1    /* Read command */
#define GH3020_SPI_CMD_SLEEP               0xc4    /* Sleep command */
#define GH3020_SPI_CMD_RESUME              0xc3    /* Resume command */
#define GH3020_SPI_CMD_RESET               0xc2    /* Resume command */
#define GH3020_SPI_WR_BUF_LEN              7       /* Write reg buffer len */
#define GH3020_SPI_WR_LEN_H                0x00    /* Write reg len high byte */
#define GH3020_SPI_WR_LEN_L                0x02    /* Write reg len low byte */
#define GH3020_SPI_RD_BUF_LEN              3       /* Read reg buffer len */
#define GH3020_SPI_BYTES_PER_REG           2       /* How many bytes in a reg */
#define GH3020_SPI_LOCALBUF_SIZE           100     /* Size of local exchange buf */

/* Part3. Enumerations and constants */

/* LED driver scale N for EFUSE calibration */

#define GH3020_LED_DRV_N_FOR_EFUSE_25MA    8       /* The parameter used in 25mA LED driver EFUSE calibration */
#define GH3020_LED_DRV_N_FOR_EFUSE_50MA    4       /* The parameter used in 50mA LED driver EFUSE calibration */
#define GH3020_LED_DRV_N_FOR_EFUSE_100MA   2       /* The parameter used in 100mA LED driver EFUSE calibration */
#define GH3020_LED_DRV_N_FOR_EFUSE_200MA   1       /* The parameter used in 200mA LED driver EFUSE calibration */

/* Factest mode integration time */

#define GH3020_FACTEST_INT_TIME_10_US      0       /* Integration time = 10 us */
#define GH3020_FACTEST_INT_TIME_20_US      1       /* Integration time = 20 us */
#define GH3020_FACTEST_INT_TIME_30_US      2       /* Integration time = 30 us */
#define GH3020_FACTEST_INT_TIME_39_US      3       /* Integration time = 39 us */
#define GH3020_FACTEST_INT_TIME_79_US      4       /* Integration time = 79 us */
#define GH3020_FACTEST_INT_TIME_158_US     5       /* Integration time = 158 us */
#define GH3020_FACTEST_INT_TIME_316_US     6       /* Integration time = 316 us */

/* Factest mode TIA gain */

#define GH3020_FACTEST_TIA_GAIN_10_K       0       /* TIA gain = 10 KOhm */
#define GH3020_FACTEST_TIA_GAIN_25_K       1       /* TIA gain = 25 KOhm */
#define GH3020_FACTEST_TIA_GAIN_50_K       2       /* TIA gain = 50 KOhm */
#define GH3020_FACTEST_TIA_GAIN_75_K       3       /* TIA gain = 75 KOhm */
#define GH3020_FACTEST_TIA_GAIN_100_K      4       /* TIA gain = 100 KOhm */
#define GH3020_FACTEST_TIA_GAIN_250_K      5       /* TIA gain = 250 KOhm */
#define GH3020_FACTEST_TIA_GAIN_500_K      6       /* TIA gain = 500 KOhm */
#define GH3020_FACTEST_TIA_GAIN_750_K      7       /* TIA gain = 750 KOhm */
#define GH3020_FACTEST_TIA_GAIN_1000_K     8       /* TIA gain = 1000 KOhm */
#define GH3020_FACTEST_TIA_GAIN_1250_K     9       /* TIA gain = 1250 KOhm */
#define GH3020_FACTEST_TIA_GAIN_1500_K     10      /* TIA gain = 1500 KOhm */
#define GH3020_FACTEST_TIA_GAIN_1750_K     11      /* TIA gain = 1750 KOhm */
#define GH3020_FACTEST_TIA_GAIN_2000_K     12      /* TIA gain = 2000 KOhm */

/* GH3020 constants */

#define GH3020_FACTEST_CH_MAX              32        /* Maximum factest channels */
#define GH3020_CMD_DELAY_US                750       /* Delay time(us) for a cmd */
#define GH3020_SINGLE_ADC_NUM              1         /* Some measurements use 1 ADC channel */
#define GH3020_QUAD_ADC_NUM                4         /* Some measurements use 4 ADC channel */
#define GH3020_TOTAL_ADC_NUM               4         /* Totally 4 ADC channels */
#define GH3020_BYTES_PER_DATA              4         /* Each PD data has 4bytes */
#define GH3020_LEDDRV_CURRENT_MA_MAX       200       /* Each LED driver output 200mA max. */
#define GH3020_ADC_BASELINE                0x800000  /* ADC baseline */
#define GH3020_LED_DRV0_Y0                 0.1f      /* LED driver0 Y0 = 0.1 Ohm */
#define GH3020_LED_DRV1_Y1                 0.01f     /* LED driver0 Y1 = 0.01 Ohm */
#define GH3020_LED_DRV_VOLTAGE_UV          225000.0f /* LED driver source voltage = 225000 uV */
#define GH3020_LED_DRV_VOLTAGE_MV          225.0f    /* LED driver source voltage = 225 mV */

/* Part5 Macros */

/* Set bits and change the source's value */

#define GH3020_VAL_SET_BIT(x, m)     ((x) = (x) | (m))

/* Clear bits and change the source's value */

#define GH3020_VAL_CLEAR_BIT(x, m)   ((x) = (x) & (~(m)))

/* Get the masked bit field and change the source's value */

#define GH3020_VAL_GET_BIT(x, m)     ((x) = (x) & (m))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Register struct */

struct gh3020_reg_s
{
  uint16_t regaddr;                       /* Register address */
  uint16_t regval;                        /* Register value */
};

/* Initial registers configuration */

struct gh3020_initcfg_s
{
  FAR const struct gh3020_reg_s *pregarr; /* Pointer of registers array */
  uint16_t                      arrlen;   /* Registers array length */
};

/* Factest mode parameters */

struct gh3020_factestmode_param_s
{
  uint32_t channelmode;                   /* Channel mode */

  /* ADC TIA gain for each channel */

  uint8_t  tia_gain[GH3020_FACTEST_CH_MAX];

  /* LED driver 0 current(mA, 0~200) for each channel. */

  uint8_t  led_drv0_current[GH3020_FACTEST_CH_MAX];

  /* LED driver 1 current(mA, 0~200). Drv0 and 1 are in parallel. */

  uint8_t  led_drv1_current[GH3020_FACTEST_CH_MAX];
  uint16_t sample_rate;                   /* Sample rate (5Hz~1000Hz) */
  uint8_t  int_time;                      /* ADC integation time */
  bool     tia_gain_change_en;            /* If need to change TIA gain */
  bool     led_current_change_en;         /* If need to change LED current */
  bool     sample_rate_change_en;         /* If need to change sample rate */
  bool     int_time_change_en;            /* If need to change ADC int time */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_SENSORS_AD5940_DEF_H */

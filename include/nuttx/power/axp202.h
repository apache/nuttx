/****************************************************************************
 * include/nuttx/power/axp202.h
 * msa301 Driver declaration
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

#ifndef __INCLUDE_NUTTX_POWER_BATTERY_AXP202_H
#define __INCLUDE_NUTTX_POWER_BATTERY_AXP202_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AXP202_SLAVE_ADDRESS 0x35
#define AXP202_CHIP_ID                          (0x41)

#define AXP202_STATUS                           (0x00)
#define AXP202_MODE_CHGSTATUS                   (0x01)
#define AXP202_OTG_STATUS                       (0x02)
#define AXP202_IC_TYPE                          (0x03)
#define AXP202_DATA_BUFFER1                     (0x04)
#define AXP202_DATA_BUFFER2                     (0x05)
#define AXP202_DATA_BUFFER3                     (0x06)
#define AXP202_DATA_BUFFER4                     (0x07)
#define AXP202_DATA_BUFFER5                     (0x08)
#define AXP202_DATA_BUFFER6                     (0x09)
#define AXP202_DATA_BUFFER7                     (0x0A)
#define AXP202_DATA_BUFFER8                     (0x0B)
#define AXP202_DATA_BUFFER9                     (0x0C)
#define AXP202_DATA_BUFFERA                     (0x0D)
#define AXP202_DATA_BUFFERB                     (0x0E)
#define AXP202_DATA_BUFFERC                     (0x0F)
#define AXP202_LDO234_DC23_CTL                  (0x12)
#define AXP202_DC2OUT_VOL                       (0x23)
#define AXP202_LDO3_DC2_DVM                     (0x25)
#define AXP202_DC3OUT_VOL                       (0x27)
#define AXP202_LDO24OUT_VOL                     (0x28)
#define AXP202_LDO3OUT_VOL                      (0x29)
#define AXP202_IPS_SET                          (0x30)
#define AXP202_VOFF_SET                         (0x31)
#define AXP202_OFF_CTL                          (0x32)
#define AXP202_CHARGE1                          (0x33)
#define AXP202_CHARGE2                          (0x34)
#define AXP202_BACKUP_CHG                       (0x35)
#define AXP202_POK_SET                          (0x36)
#define AXP202_DCDC_FREQSET                     (0x37)
#define AXP202_VLTF_CHGSET                      (0x38)
#define AXP202_VHTF_CHGSET                      (0x39)
#define AXP202_APS_WARNING1                     (0x3A)
#define AXP202_APS_WARNING2                     (0x3B)
#define AXP202_TLTF_DISCHGSET                   (0x3C)
#define AXP202_THTF_DISCHGSET                   (0x3D)
#define AXP202_DCDC_MODESET                     (0x80)
#define AXP202_ADC_EN1                          (0x82)
#define AXP202_ADC_EN2                          (0x83)
#define AXP202_ADC_SPEED                        (0x84)
#define AXP202_ADC_INPUTRANGE                   (0x85)
#define AXP202_ADC_IRQ_RETFSET                  (0x86)
#define AXP202_ADC_IRQ_FETFSET                  (0x87)
#define AXP202_TIMER_CTL                        (0x8A)
#define AXP202_VBUS_DET_SRP                     (0x8B)
#define AXP202_HOTOVER_CTL                      (0x8F)
#define AXP202_GPIO0_CTL                        (0x90)
#define AXP202_GPIO0_VOL                        (0x91)
#define AXP202_GPIO1_CTL                        (0x92)
#define AXP202_GPIO2_CTL                        (0x93)
#define AXP202_GPIO012_SIGNAL                   (0x94)
#define AXP202_GPIO3_CTL                        (0x95)
#define AXP202_INTEN1                           (0x40)
#define AXP202_INTEN2                           (0x41)
#define AXP202_INTEN3                           (0x42)
#define AXP202_INTEN4                           (0x43)
#define AXP202_INTEN5                           (0x44)
#define AXP202_INTSTS1                          (0x48)
#define AXP202_INTSTS2                          (0x49)
#define AXP202_INTSTS3                          (0x4A)
#define AXP202_INTSTS4                          (0x4B)
#define AXP202_INTSTS5                          (0x4C)

/* axp 192/202 adc data register */

#define AXP202_BAT_AVERVOL_H8                   (0x78)
#define AXP202_BAT_AVERVOL_L4                   (0x79)
#define AXP202_BAT_AVERCHGCUR_H8                (0x7A)
#define AXP202_BAT_AVERCHGCUR_L4                (0x7B)
#define AXP202_BAT_AVERCHGCUR_L5                (0x7B)
#define AXP202_ACIN_VOL_H8                      (0x56)
#define AXP202_ACIN_VOL_L4                      (0x57)
#define AXP202_ACIN_CUR_H8                      (0x58)
#define AXP202_ACIN_CUR_L4                      (0x59)
#define AXP202_VBUS_VOL_H8                      (0x5A)
#define AXP202_VBUS_VOL_L4                      (0x5B)
#define AXP202_VBUS_CUR_H8                      (0x5C)
#define AXP202_VBUS_CUR_L4                      (0x5D)
#define AXP202_INTERNAL_TEMP_H8                 (0x5E)
#define AXP202_INTERNAL_TEMP_L4                 (0x5F)
#define AXP202_TS_IN_H8                         (0x62)
#define AXP202_TS_IN_L4                         (0x63)
#define AXP202_GPIO0_VOL_ADC_H8                 (0x64)
#define AXP202_GPIO0_VOL_ADC_L4                 (0x65)
#define AXP202_GPIO1_VOL_ADC_H8                 (0x66)
#define AXP202_GPIO1_VOL_ADC_L4                 (0x67)

#define AXP202_BAT_AVERDISCHGCUR_H8             (0x7C)
#define AXP202_BAT_AVERDISCHGCUR_L5             (0x7D)
#define AXP202_APS_AVERVOL_H8                   (0x7E)
#define AXP202_APS_AVERVOL_L4                   (0x7F)
#define AXP202_INT_BAT_CHGCUR_H8                (0xA0)
#define AXP202_INT_BAT_CHGCUR_L4                (0xA1)
#define AXP202_EXT_BAT_CHGCUR_H8                (0xA2)
#define AXP202_EXT_BAT_CHGCUR_L4                (0xA3)
#define AXP202_INT_BAT_DISCHGCUR_H8             (0xA4)
#define AXP202_INT_BAT_DISCHGCUR_L4             (0xA5)
#define AXP202_EXT_BAT_DISCHGCUR_H8             (0xA6)
#define AXP202_EXT_BAT_DISCHGCUR_L4             (0xA7)
#define AXP202_BAT_CHGCOULOMB3                  (0xB0)
#define AXP202_BAT_CHGCOULOMB2                  (0xB1)
#define AXP202_BAT_CHGCOULOMB1                  (0xB2)
#define AXP202_BAT_CHGCOULOMB0                  (0xB3)
#define AXP202_BAT_DISCHGCOULOMB3               (0xB4)
#define AXP202_BAT_DISCHGCOULOMB2               (0xB5)
#define AXP202_BAT_DISCHGCOULOMB1               (0xB6)
#define AXP202_BAT_DISCHGCOULOMB0               (0xB7)
#define AXP202_COULOMB_CTL                      (0xB8)
#define AXP202_BAT_POWERH8                      (0x70)
#define AXP202_BAT_POWERM8                      (0x71)
#define AXP202_BAT_POWERL8                      (0x72)

#define AXP202_VREF_TEM_CTRL                    (0xF3)
#define AXP202_BATT_PERCENTAGE                  (0xB9)

#define AXP202_BATT_VOLTAGE_STEP                (1.1F)
#define AXP202_BATT_DISCHARGE_CUR_STEP          (0.5F)
#define AXP202_BATT_CHARGE_CUR_STEP             (0.5F)
#define AXP202_ACIN_VOLTAGE_STEP                (1.7F)
#define AXP202_ACIN_CUR_STEP                    (0.625F)
#define AXP202_VBUS_VOLTAGE_STEP                (1.7F)
#define AXP202_VBUS_CUR_STEP                    (0.375F)
#define AXP202_INTERNAL_TEMP_STEP               (0.1F)
#define AXP202_APS_VOLTAGE_STEP                 (1.4F)
#define AXP202_TS_PIN_OUT_STEP                  (0.8F)
#define AXP202_GPIO0_STEP                       (0.5F)
#define AXP202_GPIO1_STEP                       (0.5F)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_AXP202)

#ifdef __cplusplus
extern "C"
{
#endif

struct i2c_master_s;
FAR struct battery_charger_dev_s *axp202_initialize(
                                     FAR struct i2c_master_s *i2c,
                                     uint8_t addr,
                                     uint32_t frequency
                                     );

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_I2C_AXP202 */
#endif


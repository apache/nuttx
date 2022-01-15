/****************************************************************************
 * include/nuttx/power/bq769x0.h
 * Lower half driver for 769x0 battery monitor
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

#ifndef __INCLUDE_NUTTX_POWER_BQ769X0_H
#define __INCLUDE_NUTTX_POWER_BQ769X0_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Chip variant definitions */

#define CHIP_BQ76920                    0
#define CHIP_BQ76930                    1
#define CHIP_BQ76940                    2

/* Auxiliary Definitions */

#define BQ769X0_BASE_GAIN               365 /* uV/LSB */
#define BQ769X0_MAX_CELLS               15  /* No more than 15 cells per chip*/
#define BQ769X0_BAL_REG_COUNT           3   /* CELBAL1, CELBAL2, CELBAL3 */
#define BQ769X0_BAL_BITS_PER_REG        5
#define BQ76920_MIN_CELL_COUNT          3
#define BQ76920_MAX_CELL_COUNT          5
#define BQ76930_MIN_CELL_COUNT          6
#define BQ76930_MAX_CELL_COUNT          10
#define BQ76940_MIN_CELL_COUNT          9
#define BQ76940_MAX_CELL_COUNT          15
#define BQ76920_TEMP_COUNT              1
#define BQ76930_TEMP_COUNT              2
#define BQ76940_TEMP_COUNT              3
#define BQ769X0_CC_CFG_DEFAULT_VAL      0x19 /* Per Datasheet */
#define BQ769X0_CELLBAL_MASK            0x1f /* 5 LSBs in each register */
#define BQ769X0_CC_TIME                 250  /* milliseconds per CC sample */
#define BQ769X0_CC_POLL_INTERVAL        50   /* milliseconds (arbitrary) */
#define BQ769X0_CC_SCALE                8440 /* nanovolts / LSB */

/* BQ769x0 Register Definitions *********************************************/

/* System Control & Configuration Registers */

#define BQ769X0_REG_SYS_STAT            0x00
#define BQ769X0_REG_CELLBAL1            0x01
#define BQ769X0_REG_CELLBAL2            0x02 /* Only on BQ76930 and BQ76940 */
#define BQ769X0_REG_CELLBAL3            0x03 /* Only on BQ76940 */
#define BQ769X0_REG_SYS_CTRL1           0x04
#define BQ769X0_REG_SYS_CTRL2           0x05
#define BQ769X0_REG_PROTECT1            0x06
#define BQ769X0_REG_PROTECT2            0x07
#define BQ769X0_REG_PROTECT3            0x08
#define BQ769X0_REG_OV_TRIP             0x09
#define BQ769X0_REG_UV_TRIP             0x0a
#define BQ769X0_REG_CC_CFG              0x0b

/* Cell Voltage Registers
 * BQ76920 can use VC1 - VC5
 * BQ76930 can use VC1 - VC10
 * BQ76940 can use VC1 - VC15
 */

#define BQ769X0_REG_VC1_HI              0x0c
#define BQ769X0_REG_VC1_LO              0x0d
#define BQ769X0_REG_VC2_HI              0x0e
#define BQ769X0_REG_VC2_LO              0x0f
#define BQ769X0_REG_VC3_HI              0x10
#define BQ769X0_REG_VC3_LO              0x11
#define BQ769X0_REG_VC4_HI              0x12
#define BQ769X0_REG_VC4_LO              0x13
#define BQ769X0_REG_VC5_HI              0x14
#define BQ769X0_REG_VC5_LO              0x15
#define BQ769X0_REG_VC6_HI              0x16
#define BQ769X0_REG_VC6_LO              0x17
#define BQ769X0_REG_VC7_HI              0x18
#define BQ769X0_REG_VC7_LO              0x19
#define BQ769X0_REG_VC8_HI              0x1a
#define BQ769X0_REG_VC8_LO              0x1b
#define BQ769X0_REG_VC9_HI              0x1c
#define BQ769X0_REG_VC9_LO              0x1d
#define BQ769X0_REG_VC10_HI             0x1e
#define BQ769X0_REG_VC10_LO             0x1f
#define BQ769X0_REG_VC11_HI             0x20
#define BQ769X0_REG_VC11_LO             0x21
#define BQ769X0_REG_VC12_HI             0x22
#define BQ769X0_REG_VC12_LO             0x23
#define BQ769X0_REG_VC13_HI             0x24
#define BQ769X0_REG_VC13_LO             0x25
#define BQ769X0_REG_VC14_HI             0x26
#define BQ769X0_REG_VC14_LO             0x27
#define BQ769X0_REG_VC15_HI             0x28
#define BQ769X0_REG_VC15_LO             0x29

/* System Voltage, Temperature, and Current Registers */

#define BQ769X0_REG_BAT_HI              0x2a
#define BQ769X0_REG_BAT_LO              0x2b
#define BQ769X0_REG_TS1_HI              0x2c
#define BQ769X0_REG_TS1_LO              0x2d
#define BQ769X0_REG_TS2_HI              0x2e
#define BQ769X0_REG_TS2_LO              0x2f
#define BQ769X0_REG_TS3_HI              0x30
#define BQ769X0_REG_TS3_LO              0x31
#define BQ769X0_REG_CC_HI               0x32
#define BQ769X0_REG_CC_LO               0x33

/* ADC Calibration Registers */

#define BQ769X0_REG_ADCGAIN1            0x50
#define BQ769X0_REG_ADCOFFSET           0x51
#define BQ769X0_REG_ADCGAIN2            0x59

/* REG_SYS_STAT */

#define BQ769X0_CC_READY                (1 << 7) /* Updated Coulomb counter reading is available */
                                                 /* Bit 6: Reserved */
#define BQ769X0_DEVICE_XREADY           (1 << 5) /* Internal chip fault detected */
#define BQ769X0_OVRD_ALERT              (1 << 4) /* External pull-up detected on ALERT pin */
#define BQ769X0_UV                      (1 << 3) /* Undervoltage fault */
#define BQ769X0_OV                      (1 << 2) /* Overvoltage fault */
#define BQ769X0_SCD                     (1 << 1) /* Short circuit in discharge fault */
#define BQ769X0_OCD                     (1 << 0) /* Over current in discharge fault */
#define BQ769X0_FAULT_MASK              (0x3f)   /* Bottom 6 bits */

/* REG_SYS_CTRL1 */

#define BQ769X0_LOAD_PRESENT            (1 << 7) /* CHG pin is higher than VLOAD_DETECT, external load is present */
                                                 /* Bit 6: Reserved */
                                                 /* Bit 5: Reserved */
#define BQ769X0_ADC_EN                  (1 << 4) /* Enable voltage, temperature measurement and OV protection */
#define BQ769X0_TEMP_SEL                (1 << 3) /* 0 = use onboard temp sensors, 1 = use external temp sensors */
                                                 /* Bit 2: Reserved */
#define BQ769X0_SHUT_A                  (1 << 1) /* Shutdown command, bit A */
#define BQ769X0_SHUT_B                  (1 << 0) /* Shutdown command, bit B */

#define BQ769X0_SYS_CTRL1_WRITE_MASK    (0x1b)   /* Bits 0, 1, 3, and 4 are writeable */
#define BQ769X0_SYS_CTRL1_SHUTDOWN_MASK (0x03)   /* Bits 0 and 1 for shutdown */

/* REG_SYS_CTRL2 */

#define BQ769X0_DELAY_DIS               (1 << 7) /* 1 = Disable protection delays for testing */
#define BQ769X0_CC_EN                   (1 << 6) /* 1 = Enable continuous Coulomb counter readings */
#define BQ769X0_CC_ONESHOT              (1 << 5) /* Trigger a single 250ms Coulomb counter read */
                                                 /* Bit 4: Reserved */
                                                 /* Bit 3: Reserved */
                                                 /* Bit 2: Reserved */
#define BQ769X0_DSG_ON                  (1 << 1) /* DSG (Discharge FET) control */
#define BQ769X0_CHG_ON                  (1 << 0) /* CHG (Charge FET) control */

#define BQ769X0_SYS_CTRL2_WRITE_MASK    (0xe3)   /* Bits 0, 1, 5, 6, and 7 are writeable */
#define BQ769X0_SYS_CTRL2_CHGDSG_MASK   (0x03)   /* Bits 0 and 1 for charge/discharge */

/* REG_PROTECT1 */

#define BQ769X0_RSNS                    (1 << 7) /* Selects low or high input range for OCD/SCD */
                                                 /* Bit 6: Reserved */
                                                 /* Bit 5: Reserved */
#define BQ769X0_SCD_DELAY_SHIFT         (3)      /* Short circuit in discharge delay timer */
#define BQ769X0_SCD_DELAY_MASK          (0x03 << BQ769X0_SCD_DELAY_SHIFT)
#  define BQ769X0_SCD_DELAY_70US        (0 << BQ769X0_SCD_DELAY_SHIFT)
#  define BQ769X0_SCD_DELAY_100US       (1 << BQ769X0_SCD_DELAY_SHIFT)
#  define BQ769X0_SCD_DELAY_200US       (2 << BQ769X0_SCD_DELAY_SHIFT)
#  define BQ769X0_SCD_DELAY_400US       (3 << BQ769X0_SCD_DELAY_SHIFT)
#define BQ769X0_SCD_THRESH_SHIFT        (0)      /* Short circuit in discharge threshold value */
#define BQ769X0_SCD_THRESH_MASK         (0x07 << BQ769X0_SCD_THRESH_SHIFT)

/* These defines apply when RSNS is 0 */

#  define BQ769X0_SCD_THRESH_0_22MV     (0 << BQ769X0_SCD_THRESH_SHIFT)
#  define BQ769X0_SCD_THRESH_0_33MV     (1 << BQ769X0_SCD_THRESH_SHIFT)
#  define BQ769X0_SCD_THRESH_0_44MV     (2 << BQ769X0_SCD_THRESH_SHIFT)
#  define BQ769X0_SCD_THRESH_0_56MV     (3 << BQ769X0_SCD_THRESH_SHIFT)
#  define BQ769X0_SCD_THRESH_0_67MV     (4 << BQ769X0_SCD_THRESH_SHIFT)
#  define BQ769X0_SCD_THRESH_0_78MV     (5 << BQ769X0_SCD_THRESH_SHIFT)
#  define BQ769X0_SCD_THRESH_0_89MV     (6 << BQ769X0_SCD_THRESH_SHIFT)
#  define BQ769X0_SCD_THRESH_0_100MV    (7 << BQ769X0_SCD_THRESH_SHIFT)

/* These defines apply when RSNS is 1 */

#  define BQ769X0_SCD_THRESH_1_44MV     (0 << BQ769X0_SCD_THRESH_SHIFT)
#  define BQ769X0_SCD_THRESH_1_67MV     (1 << BQ769X0_SCD_THRESH_SHIFT)
#  define BQ769X0_SCD_THRESH_1_89MV     (2 << BQ769X0_SCD_THRESH_SHIFT)
#  define BQ769X0_SCD_THRESH_1_111MV    (3 << BQ769X0_SCD_THRESH_SHIFT)
#  define BQ769X0_SCD_THRESH_1_133MV    (4 << BQ769X0_SCD_THRESH_SHIFT)
#  define BQ769X0_SCD_THRESH_1_155MV    (5 << BQ769X0_SCD_THRESH_SHIFT)
#  define BQ769X0_SCD_THRESH_1_178MV    (6 << BQ769X0_SCD_THRESH_SHIFT)
#  define BQ769X0_SCD_THRESH_1_200MV    (7 << BQ769X0_SCD_THRESH_SHIFT)

/* REG_PROTECT2 */

                                            /* Bit 7: Reserved */
#define BQ769X0_OCD_DELAY_SHIFT         (4) /* Over current in discharge delay timer */
#define BQ769X0_OCD_DELAY_MASK          (0x07 << BQ769X0_OCD_DELAY_SHIFT)
#  define BQ769X0_OCD_DELAY_8MS         (0 << BQ769X0_OCD_DELAY_SHIFT)
#  define BQ769X0_OCD_DELAY_20MS        (1 << BQ769X0_OCD_DELAY_SHIFT)
#  define BQ769X0_OCD_DELAY_40MS        (2 << BQ769X0_OCD_DELAY_SHIFT)
#  define BQ769X0_OCD_DELAY_80MS        (3 << BQ769X0_OCD_DELAY_SHIFT)
#  define BQ769X0_OCD_DELAY_160MS       (4 << BQ769X0_OCD_DELAY_SHIFT)
#  define BQ769X0_OCD_DELAY_320MS       (5 << BQ769X0_OCD_DELAY_SHIFT)
#  define BQ769X0_OCD_DELAY_640MS       (6 << BQ769X0_OCD_DELAY_SHIFT)
#  define BQ769X0_OCD_DELAY_1280MS      (7 << BQ769X0_OCD_DELAY_SHIFT)
#define BQ769X0_OCD_THRESH_SHIFT        (0) /* Over current in discharge threshold value */
#define BQ769X0_OCD_THRESH_MASK         (0x0f << BQ769X0_OCD_THRESH_SHIFT)

/* These defines apply when RSNS is 0 */

#  define BQ769X0_OCD_THRESH_0_8MV      (0 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_0_11MV     (1 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_0_14MV     (2 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_0_17MV     (3 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_0_19MV     (4 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_0_22MV     (5 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_0_25MV     (6 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_0_28MV     (7 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_0_31MV     (8 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_0_33MV     (9 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_0_36MV     (10 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_0_39MV     (11 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_0_42MV     (12 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_0_44MV     (13 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_0_47MV     (14 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_0_50MV     (15 << BQ769X0_OCD_THRESH_SHIFT)

/* These defines apply when RSNS is 1 */

#  define BQ769X0_OCD_THRESH_1_17MV     (0 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_1_22MV     (1 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_1_28MV     (2 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_1_33MV     (3 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_1_39MV     (4 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_1_44MV     (5 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_1_50MV     (6 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_1_56MV     (7 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_1_61MV     (8 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_1_67MV     (9 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_1_72MV     (10 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_1_78MV     (11 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_1_83MV     (12 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_1_89MV     (13 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_1_94MV     (14 << BQ769X0_OCD_THRESH_SHIFT)
#  define BQ769X0_OCD_THRESH_1_100MV    (15 << BQ769X0_OCD_THRESH_SHIFT)

/* REG_PROTECT3 */

#define BQ769X0_UV_DELAY_SHIFT          (6)
#define BQ769X0_UV_DELAY_MASK           (0x03 << BQ769X0_UV_DELAY_SHIFT)
#  define BQ769X0_UV_DELAY_1S           (0 << BQ769X0_UV_DELAY_SHIFT)
#  define BQ769X0_UV_DELAY_4S           (1 << BQ769X0_UV_DELAY_SHIFT)
#  define BQ769X0_UV_DELAY_8S           (2 << BQ769X0_UV_DELAY_SHIFT)
#  define BQ769X0_UV_DELAY_16S          (3 << BQ769X0_UV_DELAY_SHIFT)
#define BQ769X0_OV_DELAY_SHIFT          (4)
#define BQ769X0_OV_DELAY_MASK           (0x03 << BQ769X0_OV_DELAY_SHIFT)
#  define BQ769X0_OV_DELAY_1S           (0 << BQ769X0_OV_DELAY_SHIFT)
#  define BQ769X0_OV_DELAY_2S           (1 << BQ769X0_OV_DELAY_SHIFT)
#  define BQ769X0_OV_DELAY_4S           (2 << BQ769X0_OV_DELAY_SHIFT)
#  define BQ769X0_OV_DELAY_8S           (3 << BQ769X0_OV_DELAY_SHIFT)

/* REG_ADCGAIN1 */

#define BQ769X0_ADCGAIN1_SHIFT          (2)
#define BQ769X0_ADCGAIN1_MASK            (0x03 << BQ769X0_ADCGAIN1_SHIFT)

/* REG_ADCGAIN2 */

#define BQ769X0_ADCGAIN2_SHIFT          (5)
#define BQ769X0_ADCGAIN2_MASK           (0x07 << BQ769X0_ADCGAIN2_SHIFT)

#endif /* __INCLUDE_NUTTX_POWER_BQ769X0_H */

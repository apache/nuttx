/****************************************************************************
 * include/nuttx/lcd/ili9325.h
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

#ifndef __INCLUDE_NUTTX_LCD_ILI9325_H
#define __INCLUDE_NUTTX_LCD_ILI9325_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ILI9325 ID code */

#define ILI9325_DEVICE_CODE                    0x9325

/* ILI9325 LCD Register Addresses *******************************************/

#define ILI9325_DEVICE_CODE_REG                0x00 /* Driver Code Register */
#define ILI9325_START_OSC_CTRL                 0x00 /* Start Oscillator Control */
#define ILI9325_DRIVER_OUTPUT_CTRL1            0x01 /* Start Oscillator Control */
#define ILI9325_LCD_DRIVING_CTRL               0x02 /* LCD Driving Control */
#define ILI9325_ENTRY_MODE                     0x03 /* Entry Mode */
#define ILI9325_RESIZE_CTRL                    0x04 /* Resize Control */
#define ILI9325_DISP_CTRL1                     0x07 /* Display Control 1 */
#define ILI9325_DISP_CTRL2                     0x08 /* Display Control 2 */
#define ILI9325_DISP_CTRL3                     0x09 /* Display Control 3 */
#define ILI9325_DISP_CTRL4                     0x0a /* Display Control 4 */
#define ILI9325_RGB_DISP_INTERFACE_CTRL1       0x0c /* RGB Display Interface Control 1 */
#define ILI9325_FRAME_MAKER_SHIFT              0x0d /* Frame Maker Position */
#define ILI9325_RGB_DISP_INTERFACE_CTRL2       0x0f /* RGB Display Interface Control 2 */
#define ILI9325_POWER_CTRL1                    0x10 /* Power Control 1 */
#define ILI9325_POWER_CTRL2                    0x11 /* Power Control 2 */
#define ILI9325_POWER_CTRL3                    0x12 /* Power Control 3 */
#define ILI9325_POWER_CTRL4                    0x13 /* Power Control 4 */
#define ILI9325_HORIZONTAL_GRAM_ADDR_SET       0x20 /* Horizontal GRAM Address Set */
#define ILI9325_VERTICAL_GRAM_ADDR_SET         0x21 /* Vertical  GRAM Address Set */
#define ILI9325_GRAM_DATA_REG                  0x22 /* GRAM Data Register */
#define ILI9325_POWER_CTRL7                    0x29 /* Power Control 7 */
#define ILI9325_FRAME_RATE_AND_COLOR_CTRL      0x2b /* Frame Rate and Color Control */
#define ILI9325_GAMMA_CTRL1                    0x30 /* Gamma Control 1 */
#define ILI9325_GAMMA_CTRL2                    0x31 /* Gamma Control 2 */
#define ILI9325_GAMMA_CTRL3                    0x32 /* Gamma Control 3 */
#define ILI9325_GAMMA_CTRL4                    0x35 /* Gamma Control 4 */
#define ILI9325_GAMMA_CTRL5                    0x36 /* Gamma Control 5 */
#define ILI9325_GAMMA_CTRL6                    0x37 /* Gamma Control 6 */
#define ILI9325_GAMMA_CTRL7                    0x38 /* Gamma Control 7 */
#define ILI9325_GAMMA_CTRL8                    0x39 /* Gamma Control 8 */
#define ILI9325_GAMMA_CTRL9                    0x3c /* Gamma Control 9 */
#define ILI9325_GAMMA_CTRL10                   0x3d /* Gamma Control 10 */
#define ILI9325_HORIZONTAL_ADDR_START          0x50 /* Horizontal Address Start Position */
#define ILI9325_HORIZONTAL_ADDR_END            0x51 /* Horizontal Address End Position */
#define ILI9325_VERTICAL_ADDR_START            0x52 /* Vertical Address Start Position */
#define ILI9325_VERTICAL_ADDR_END              0x53 /* Vertical Address End Position */
#define ILI9325_DRIVER_OUTPUT_CTRL2            0x60 /* Driver Output Control 2 */
#define ILI9325_BASE_IMG_DISP_CTRL             0x61 /* Base Image Display Control */
#define ILI9325_VERTICAL_SCROLL_CTRL           0x6a /* Vertical Scroll Control */
#define ILI9325_PARTIAL_IMG1_DISP_SHIFT        0x80 /* Partial Image 1 Display Position */
#define ILI9325_PARTIAL_IMG1_AREA_START_LINE   0x81 /* Partial Image 1 Area (Start Line) */
#define ILI9325_PARTIAL_IMG1_AREA_END_LINE     0x82 /* Partial Image 1 Area (End Line) */
#define ILI9325_PARTIAL_IMG2_DISP_SHIFT        0x83 /* Partial Image 2 Display Position */
#define ILI9325_PARTIAL_IMG2_AREA_START_LINE   0x84 /* Partial Image 2 Area (Start Line) */
#define ILI9325_PARTIAL_IMG2_AREA_END_LINE     0x85 /* Partial Image 2 Area (End Line) */
#define ILI9325_PANEL_INTERFACE_CTRL1          0x90 /* Panel Interface Control 1 */
#define ILI9325_PANEL_INTERFACE_CTRL2          0x92 /* Panel Interface Control 2 */
#define ILI9325_PANEL_INTERFACE_CTRL4          0x95 /* Panel Interface Control 4 */
#define ILI9325_OTP_VCM_PROG_CTRL              0xa1 /* OTP VCM Programming Control */
#define ILI9325_OTP_VCM_STATUS_AND_ENABLE      0xa2 /* OTP VCM Status and Enable */
#define ILI9325_OTP_PROG_ID_KEY                0xa5 /* OTP Programming ID Key */

/* ILI9325 LCD Register Bit Definitions *************************************/

/* ILI9325_START_OSC_CTRL, Start Oscillator Control, Offset: 0x00)  */

#define ILI9325_START_OSC_CTRL_EN              (1 << 0)

/* ILI9325_DRIVER_OUTPUT_CTRL1, Start Oscillator Control, Offset: 0x01 */

#define ILI9325_DRIVER_OUTPUT_CTRL1_SS         (1 << 8)
#define ILI9325_DRIVER_OUTPUT_CTRL1_SM         (1 << 10)

/* ILI9325_LCD_DRIVING_CTRL, LCD Driving Control, Offset: 0x02 */

#define ILI9325_LCD_DRIVING_CTRL_EOR           (1 << 8)
#define ILI9325_LCD_DRIVING_CTRL_BC0           (1 << 9)
#define ILI9325_LCD_DRIVING_CTRL_BIT10         (1 << 10)

/* ILI9325_ENTRY_MODE, Entry Mode, Offset: 0x03 */

#define ILI9325_ENTRY_MODE_AM                  (1 << 3)
#define ILI9325_ENTRY_MODE_ID_SHIFT            4
#define ILI9325_ENTRY_MODE_ID_MASK             (3 << ILI9325_ENTRY_MODE_ID_SHIFT)
#  define ILI9325_ENTRY_MODE_ID(n)             ((uint16_t)(n) << ILI9325_ENTRY_MODE_ID_SHIFT)
#define ILI9325_ENTRY_MODE_ORG                 (1 << 7)
#define ILI9325_ENTRY_MODE_HWM                 (1 << 9)
#define ILI9325_ENTRY_MODE_BGR                 (1 << 12)
#define ILI9325_ENTRY_MODE_DFM                 (1 << 14)
#define ILI9325_ENTRY_MODE_TRI                 (1 << 15)

/* ILI9325_RESIZE_CTRL, Resize Control, Offset: 0x04 */

#define ILI9325_RESIZE_CTRL_RSZ_SHIFT          0
#define ILI9325_RESIZE_CTRL_RSZ_MASK           (3 << ILI9325_RESIZE_CTRL_RSZ_SHIFT)
#  define ILI9325_RESIZE_CTRL_RSZ(n)           ((uint16_t)(n) << ILI9325_RESIZE_CTRL_RSZ_SHIFT)
#define ILI9325_RESIZE_CTRL_RCH_SHIFT          4
#define ILI9325_RESIZE_CTRL_RCH_MASK           (3 << ILI9325_RESIZE_CTRL_RCH_SHIFT)
#  define ILI9325_RESIZE_CTRL_RCH(n)           ((uint16_t)(n) << ILI9325_RESIZE_CTRL_RCH_SHIFT)
#define ILI9325_RESIZE_CTRL_RCV_SHIFT          8
#define ILI9325_RESIZE_CTRL_RCV_MASK           (3 << ILI9325_RESIZE_CTRL_RCV_SHIFT)
#  define ILI9325_RESIZE_CTRL_RCV(n)           ((uint16_t)(n) << ILI9325_RESIZE_CTRL_RCV_SHIFT)

/* ILI9325_DISP_CTRL1, Display Control 1, Offset: 0x07 */

#define ILI9325_DISP_CTRL1_D_SHIFT             0
#define ILI9325_DISP_CTRL1_D_MASK              (3 << ILI9325_DISP_CTRL1_D_SHIFT)
#  define ILI9325_DISP_CTRL1_D(n)              ((uint16_t)(n) << ILI9325_DISP_CTRL1_D_SHIFT)
#define ILI9325_DISP_CTRL1_CL                  (1 << 3)
#define ILI9325_DISP_CTRL1_DTE                 (1 << 4)
#define ILI9325_DISP_CTRL1_GON                 (1 << 5)
#define ILI9325_DISP_CTRL1_BASEE               (1 << 8)
#define ILI9325_DISP_CTRL1_PTDE_SHIFT          12
#define ILI9325_DISP_CTRL1_PTDE_MASK           (3 << ILI9325_DISP_CTRL1_PTDE_SHIFT)
#  define ILI9325_DISP_CTRL1_PTDE(n)           ((uint16_t)(n) << ILI9325_DISP_CTRL1_PTDE_SHIFT)

/* ILI9325_DISP_CTRL2, Display Control 2, Offset: 0x08 */

#define ILI9325_DISP_CTRL2_BP_SHIFT            0
#define ILI9325_DISP_CTRL2_BP_MASK             (0xf << ILI9325_DISP_CTRL2_BP_SHIFT)
#  define ILI9325_DISP_CTRL2_BP(n)             ((uint16_t)(n) << ILI9325_DISP_CTRL2_BP_SHIFT)
#define ILI9325_DISP_CTRL2_FP_SHIFT             8
#define ILI9325_DISP_CTRL2_FP_MASK             (0xf << ILI9325_DISP_CTRL2_FP_SHIFT)
#  define ILI9325_DISP_CTRL2_FP(n)             ((uint16_t)(n) << ILI9325_DISP_CTRL2_FP_SHIFT)

/* ILI9325_DISP_CTRL3, Display Control 3, Offset: 0x09 */

#define ILI9325_DISP_CTRL3_ISC_SHIFT           0
#define ILI9325_DISP_CTRL3_ISC_MASK            (0xf << ILI9325_DISP_CTRL3_ISC_SHIFT)
#  define ILI9325_DISP_CTRL3_ISC(n)            ((uint16_t)(n) << ILI9325_DISP_CTRL3_ISC_SHIFT)
#define ILI9325_DISP_CTRL3_PTG_SHIFT           4
#define ILI9325_DISP_CTRL3_PTG_MASK            (3 << ILI9325_DISP_CTRL3_PTG_SHIFT)
#  define ILI9325_DISP_CTRL3_PTG(n)            ((uint16_t)(n) << ILI9325_DISP_CTRL3_PTG_SHIFT)
#define ILI9325_DISP_CTRL3_PTS_SHIFT           8
#define ILI9325_DISP_CTRL3_PTS_MASK            (7 << ILI9325_DISP_CTRL3_PTS_SHIFT)
#  define ILI9325_DISP_CTRL3_PTS(n)            ((uint16_t)(n) << ILI9325_DISP_CTRL3_PTS_SHIFT)

/* ILI9325_DISP_CTRL4, Display Control 4, Offset: 0x0a */

#define ILI9325_DISP_CTRL4_FMI_SHIFT           0
#define ILI9325_DISP_CTRL4_FMI_MASK            (7 << ILI9325_DISP_CTRL4_FMI_SHIFT)
#  define ILI9325_DISP_CTRL4_FMI(n)            ((uint16_t)(n) << ILI9325_DISP_CTRL4_FMI_SHIFT)
#define ILI9325_DISP_CTRL4_FMARKOE             (1 << 3)

/* ILI9325_RGB_DISP_INTERFACE_CTRL1,
 * RGB Display Interface Control 1, Offset: 0x0c
 */

#define ILI9325_RGB_DISP_INTERFACE_CTRL1_RIM_SHIFT 0
#define ILI9325_RGB_DISP_INTERFACE_CTRL1_RIM_MASK  (3 << ILI9325_RGB_DISP_INTERFACE_CTRL1_RIM_SHIFT)
#  define ILI9325_RGB_DISP_INTERFACE_CTRL1_RIM(n)  ((uint16_t)(n) << ILI9325_RGB_DISP_INTERFACE_CTRL1_RIM_SHIFT)
#define ILI9325_RGB_DISP_INTERFACE_CTRL1_DM0       (1 << 4)
#define ILI9325_RGB_DISP_INTERFACE_CTRL1_DM1       (1 << 5)
#define ILI9325_RGB_DISP_INTERFACE_CTRL1_DM_SHIFT  4
#define ILI9325_RGB_DISP_INTERFACE_CTRL1_DM_MASK   (3 << ILI9325_RGB_DISP_INTERFACE_CTRL1_DM_SHIFT)
#  define ILI9325_RGB_DISP_INTERFACE_CTRL1_DM(n)   ((uint16_t)(n) << ILI9325_RGB_DISP_INTERFACE_CTRL1_DM_SHIFT)
#define ILI9325_RGB_DISP_INTERFACE_CTRL1_RM        (1 << 8)
#define ILI9325_RGB_DISP_INTERFACE_CTRL1_ENC_SHIFT 12
#define ILI9325_RGB_DISP_INTERFACE_CTRL1_ENC_MASK  (7 << ILI9325_RGB_DISP_INTERFACE_CTRL1_ENC_SHIFT)
#  define ILI9325_RGB_DISP_INTERFACE_CTRL1_ENC(n)  ((uint16_t)(n) <ILI9325_RGB_DISP_INTERFACE_CTRL1_ENC_SHIFT)

/* ILI9325_FRAME_MAKER_SHIFT, Frame Maker Position, Offset: 0x0d */

#define ILI9325_FRAME_MAKER_SHIFT_FMP_SHIFT    0
#define ILI9325_FRAME_MAKER_SHIFT_FMP_MASK     (0x1ff << ILI9325_FRAME_MAKER_SHIFT_FMP_SHIFT)
#  define ILI9325_FRAME_MAKER_SHIFT_FMP(n)     ((uint16_t)(n) << ILI9325_FRAME_MAKER_SHIFT_FMP_SHIFT)

/* ILI9325_RGB_DISP_INTERFACE_CTRL2,
 * RGB Display Interface Control 2, Offset: 0x0f
 */

#define ILI9325_RGB_DISP_INTERFACE_CTRL2_EPL   (1 << 0)
#define ILI9325_RGB_DISP_INTERFACE_CTRL2_DPL   (1 << 1)
#define ILI9325_RGB_DISP_INTERFACE_CTRL2_HSPL  (1 << 3)
#define ILI9325_RGB_DISP_INTERFACE_CTRL2_VSPL  (1 << 4)

/* ILI9325_POWER_CTRL1, Power Control 1, Offset: 0x10 */

#define ILI9325_POWER_CTRL1_STB                (1 << 0)
#define ILI9325_POWER_CTRL1_SLP                (1 << 1)
#define ILI9325_POWER_CTRL1_DSTB               (1 << 2)
#define ILI9325_POWER_CTRL1_AP_SHIFT           4
#define ILI9325_POWER_CTRL1_AP_MASK            (7 << ILI9325_POWER_CTRL1_AP_SHIFT)
#  define ILI9325_POWER_CTRL1_AP(n)            ((uint16_t)(n) << ILI9325_POWER_CTRL1_AP_SHIFT)
#define ILI9325_POWER_CTRL1_APE                (1 << 7)
#define ILI9325_POWER_CTRL1_BT_SHIFT           8
#define ILI9325_POWER_CTRL1_BT_MASK            (7 << ILI9325_POWER_CTRL1_BT_SHIFT)
#  define ILI9325_POWER_CTRL1_BT(n)            ((uint16_t)(n) << ILI9325_POWER_CTRL1_BT_SHIFT)
#define ILI9325_POWER_CTRL1_SAP                (1 << 12)

/* ILI9325_POWER_CTRL2, Power Control 2, Offset: 0x11 */

#define ILI9325_POWER_CTRL2_VC_SHIFT           0
#define ILI9325_POWER_CTRL2_VC_MASK            (7 << ILI9325_POWER_CTRL2_VC_SHIFT)
#  define ILI9325_POWER_CTRL2_VC(n)            ((uint16_t)(n) << ILI9325_POWER_CTRL2_VC_SHIFT)
#define ILI9325_POWER_CTRL2_DC0_SHIFT          4
#define ILI9325_POWER_CTRL2_DC0_MASK           (7 << ILI9325_POWER_CTRL2_DC0_SHIFT)
#  define ILI9325_POWER_CTRL2_DC0(n)           ((uint16_t)(n) << ILI9325_POWER_CTRL2_DC0_SHIFT)
#define ILI9325_POWER_CTRL2_DC1_SHIFT          8
#define ILI9325_POWER_CTRL2_DC1_MASK           (7 << ILI9325_POWER_CTRL2_DC1_SHIFT)
#  define ILI9325_POWER_CTRL2_DC1(n)           ((uint16_t)(n) << ILI9325_POWER_CTRL2_DC1_SHIFT)

/* ILI9325_POWER_CTRL3, Power Control 3, Offset: 0x12 */

#define ILI9325_POWER_CTRL3_VRH_SHIFT          0
#define ILI9325_POWER_CTRL3_VRH_MASK           (0xf << ILI9325_POWER_CTRL3_VRH_SHIFT)
#  define ILI9325_POWER_CTRL3_VRH(n)           ((uint16_t)(n) << ILI9325_POWER_CTRL3_VRH_SHIFT)
#define ILI9325_POWER_CTRL3_PON                (1 << 4)
#define ILI9325_POWER_CTRL3_VCIRE              (1 << 7)

/* ILI9325_POWER_CTRL4, Power Control 4, Offset: 0x13 */

#define ILI9325_POWER_CTRL4_VDV_SHIFT          8
#define ILI9325_POWER_CTRL4_VDV_MASK           (0x1f << ILI9325_POWER_CTRL4_VDV_SHIFT)
#  define ILI9325_POWER_CTRL4_VDV(n)           ((uint16_t)(n) << ILI9325_POWER_CTRL4_VDV_SHIFT)

/* ILI9325_HORIZONTAL_GRAM_ADDR_SET,
 * Horizontal GRAM Address Set, Offset: 0x20
 */

#define ILI9325_HORIZONTAL_GRAM_ADDR_SET_AD_SHIFT 0
#define ILI9325_HORIZONTAL_GRAM_ADDR_SET_AD_MASK  (0xff << ILI9325_HORIZONTAL_GRAM_ADDR_SET_AD_SHIFT)
#  define ILI9325_HORIZONTAL_GRAM_ADDR_SET_AD(n)  ((uint16_t)(n) << ILI9325_HORIZONTAL_GRAM_ADDR_SET_AD_SHIFT)

/* ILI9325_VERTICAL_GRAM_ADDR_SET,
 * Vertical  GRAM Address Set, Offset: 0x21
 */

#define ILI9325_VERTICAL_GRAM_ADDR_SET_AD_SHIFT 0
#define ILI9325_VERTICAL_GRAM_ADDR_SET_AD_MASK  (0xff << ILI9325_VERTICAL_GRAM_ADDR_SET_AD_SHIFT)
#  define ILI9325_VERTICAL_GRAM_ADDR_SET_AD(n)  ((uint16_t)(n) << ILI9325_VERTICAL_GRAM_ADDR_SET_AD_SHIFT)

/* ILI9325_POWER_CTRL7, Power Control 7, Offset: 0x29 */

#define ILI9325_POWER_CTRL7_VCM_SHIFT          0
#define ILI9325_POWER_CTRL7_VCM_MASK           (0x3f << ILI9325_POWER_CTRL7_VCM_SHIFT)
#define ILI9325_POWER_CTRL7_VCM(n)             ((uint16_t)(n) << ILI9325_POWER_CTRL7_VCM_SHIFT)

/* ILI9325_FRAME_RATE_AND_COLOR_CTRL,
 * Frame Rate and Color Control, Offset: 0x2b
 */

#define ILI9325_FRAME_RATE_AND_COLOR_CTRL_FRS_SHIFT 0
#define ILI9325_FRAME_RATE_AND_COLOR_CTRL_FRS_MASK  (0xf << ILI9325_FRAME_RATE_AND_COLOR_CTRL_FRS_SHIFT)
#  define ILI9325_FRAME_RATE_AND_COLOR_CTRL_FRS(n)  ((uint16_t)(n) << ILI9325_FRAME_RATE_AND_COLOR_CTRL_FRS_SHIFT)

/* ILI9325_GAMMA_CTRL1, Gamma Control 1, Offset: 0x30 */

#define ILI9325_GAMMA_CTRL1_KP0_SHIFT          0
#define ILI9325_GAMMA_CTRL1_KP0_MASK           (7 << ILI9325_GAMMA_CTRL1_KP0_SHIFT)
#  define ILI9325_GAMMA_CTRL1_KP0(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL1_KP0_SHIFT)
#define ILI9325_GAMMA_CTRL1_KP1_SHIFT          8
#define ILI9325_GAMMA_CTRL1_KP1_MASK           (7 << ILI9325_GAMMA_CTRL1_KP1_SHIFT)
#  define ILI9325_GAMMA_CTRL1_KP1(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL1_KP1_SHIFT)

/* ILI9325_GAMMA_CTRL2, Gamma Control 2, Offset: 0x31 */

#define ILI9325_GAMMA_CTRL2_KP2_SHIFT          0
#define ILI9325_GAMMA_CTRL2_KP2_MASK           (7 << ILI9325_GAMMA_CTRL2_KP2_SHIFT)
#  define ILI9325_GAMMA_CTRL2_KP2(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL2_KP2_SHIFT)
#define ILI9325_GAMMA_CTRL2_KP3_SHIFT          8
#define ILI9325_GAMMA_CTRL2_KP3_MASK           (7 << ILI9325_GAMMA_CTRL2_KP3_SHIFT)
#  define ILI9325_GAMMA_CTRL2_KP3(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL2_KP3_SHIFT)

/* ILI9325_GAMMA_CTRL3, Gamma Control 3, Offset: 0x32 */

#define ILI9325_GAMMA_CTRL3_KP4_SHIFT          0
#define ILI9325_GAMMA_CTRL3_KP4_MASK           (7 << ILI9325_GAMMA_CTRL3_KP4_SHIFT)
#  define ILI9325_GAMMA_CTRL3_KP4(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL3_KP4_SHIFT)
#define ILI9325_GAMMA_CTRL3_KP5_SHIFT          8
#define ILI9325_GAMMA_CTRL3_KP5_MASK           (7 << ILI9325_GAMMA_CTRL3_KP5_SHIFT)
#  define ILI9325_GAMMA_CTRL3_KP5(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL3_KP5_SHIFT)

/* ILI9325_GAMMA_CTRL4, Gamma Control 4, Offset: 0x35 */

#define ILI9325_GAMMA_CTRL4_RP0_SHIFT          0
#define ILI9325_GAMMA_CTRL4_RP0_MASK           (7 << ILI9325_GAMMA_CTRL4_RP0_SHIFT)
#  define ILI9325_GAMMA_CTRL4_RP0(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL4_RP0_SHIFT)
#define ILI9325_GAMMA_CTRL4_RP1_SHIFT          8
#define ILI9325_GAMMA_CTRL4_RP1_MASK           (7 << ILI9325_GAMMA_CTRL4_RP1_SHIFT)
#  define ILI9325_GAMMA_CTRL4_RP1(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL4_RP1_SHIFT)

/* ILI9325_GAMMA_CTRL5, Gamma Control 5, Offset: 0x36 */

#define ILI9325_GAMMA_CTRL5_VRP0_SHIFT         0
#define ILI9325_GAMMA_CTRL5_VRP0_MASK          (0xf << ILI9325_GAMMA_CTRL5_VRP0_SHIFT)
#  define ILI9325_GAMMA_CTRL5_VRP0(n)          ((uint16_t)(n) << ILI9325_GAMMA_CTRL5_VRP0_SHIFT)
#define ILI9325_GAMMA_CTRL5_VRP1_SHIFT         8
#define ILI9325_GAMMA_CTRL5_VRP1_MASK          (0x1f << ILI9325_GAMMA_CTRL5_VRP1_SHIFT)
#  define ILI9325_GAMMA_CTRL5_VRP1(n)          ((uint16_t)(n) << ILI9325_GAMMA_CTRL5_VRP1_SHIFT)

/* ILI9325_GAMMA_CTRL6, Gamma Control 6, Offset: 0x37 */

#define ILI9325_GAMMA_CTRL6_KN0_SHIFT          0
#define ILI9325_GAMMA_CTRL6_KN0_MASK           (7 << ILI9325_GAMMA_CTRL6_KN0_SHIFT)
#  define ILI9325_GAMMA_CTRL6_KN0(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL6_KN0_SHIFT)
#define ILI9325_GAMMA_CTRL6_KN1_SHIFT          8
#define ILI9325_GAMMA_CTRL6_KN1_MASK           (7 << ILI9325_GAMMA_CTRL6_KN1_SHIFT)
#  define ILI9325_GAMMA_CTRL6_KN1(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL6_KN1_SHIFT)

/* ILI9325_GAMMA_CTRL7, Gamma Control 7, Offset: 0x38 */

#define ILI9325_GAMMA_CTRL7_KN2_SHIFT          0
#define ILI9325_GAMMA_CTRL7_KN2_MASK           (7 << ILI9325_GAMMA_CTRL7_KN2_SHIFT)
#  define ILI9325_GAMMA_CTRL7_KN2(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL7_KN2_SHIFT)
#define ILI9325_GAMMA_CTRL7_KN3_SHIFT          8
#define ILI9325_GAMMA_CTRL7_KN3_MASK           (7 << ILI9325_GAMMA_CTRL7_KN3_SHIFT)
#  define ILI9325_GAMMA_CTRL7_KN3(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL7_KN3_SHIFT)

/* ILI9325_GAMMA_CTRL8, Gamma Control 8, Offset: 0x39 */

#define ILI9325_GAMMA_CTRL8_KN4_SHIFT          0
#define ILI9325_GAMMA_CTRL8_KN4_MASK           (7 << ILI9325_GAMMA_CTRL8_KN4_SHIFT)
#  define ILI9325_GAMMA_CTRL8_KN4(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL8_KN4_SHIFT)
#define ILI9325_GAMMA_CTRL8_KN5_SHIFT          8
#define ILI9325_GAMMA_CTRL8_KN5_MASK           (7 << ILI9325_GAMMA_CTRL8_KN5_SHIFT)
#  define ILI9325_GAMMA_CTRL8_KN5(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL8_KN5_SHIFT)

/* ILI9325_GAMMA_CTRL9, Gamma Control 9, Offset: 0x3c */

#define ILI9325_GAMMA_CTRL9_RN0_SHIFT          0
#define ILI9325_GAMMA_CTRL9_RN0_MASK           (7 << ILI9325_GAMMA_CTRL9_RN0_SHIFT)
#  define ILI9325_GAMMA_CTRL9_RN0(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL9_RN0_SHIFT)
#define ILI9325_GAMMA_CTRL9_RN1_SHIFT          8
#define ILI9325_GAMMA_CTRL9_RN1_MASK           (7 << ILI9325_GAMMA_CTRL9_RN1_SHIFT)
#  define ILI9325_GAMMA_CTRL9_RN1(n)           ((uint16_t)(n) << ILI9325_GAMMA_CTRL9_RN1_SHIFT)

/* ILI9325_GAMMA_CTRL10, Gamma Control 10, Offset: 0x3d */

#define ILI9325_GAMMA_CTRL10_VRN0_SHIFT        0
#define ILI9325_GAMMA_CTRL10_VRN0_MASK         (0xf << ILI9325_GAMMA_CTRL10_VRN0_SHIFT)
#  define ILI9325_GAMMA_CTRL10_VRN0(n)         ((uint16_t)(n) << ILI9325_GAMMA_CTRL10_VRN0_SHIFT)
#define ILI9325_GAMMA_CTRL10_VRN1_SHIFT        8
#define ILI9325_GAMMA_CTRL10_VRN1_MASK         (0x1f << ILI9325_GAMMA_CTRL10_VRN1_SHIFT)
#  define ILI9325_GAMMA_CTRL10_VRN1(n)         ((uint16_t)(n) << ILI9325_GAMMA_CTRL10_VRN1_SHIFT)

/* ILI9325_HORIZONTAL_ADDR_START,
 * Horizontal Address Start Position, Offset: 0x50
 */

#define ILI9325_HORIZONTAL_ADDR_START_HSA_SHIFT 0
#define ILI9325_HORIZONTAL_ADDR_START_HSA_MASK  (0xff << ILI9325_HORIZONTAL_ADDR_START_HSA_SHIFT)
#define ILI9325_HORIZONTAL_ADDR_START_HSA(n)    ((uint16_t)(n) << ILI9325_HORIZONTAL_ADDR_START_HSA_SHIFT)

/* ILI9325_HORIZONTAL_ADDR_END,
 * Horizontal Address End Position, Offset: 0x51
 */

#define ILI9325_HORIZONTAL_ADDR_END_HEA_SHIFT  0
#define ILI9325_HORIZONTAL_ADDR_END_HEA_MASK   (0xff << ILI9325_HORIZONTAL_ADDR_END_HEA_SHIFT)
#  define ILI9325_HORIZONTAL_ADDR_END_HEA(n)   ((uint16_t)(n) << ILI9325_HORIZONTAL_ADDR_END_HEA_SHIFT)

/* ILI9325_VERTICAL_ADDR_START,
 * Vertical Address Start Position, Offset: 0x52
 */

#define ILI9325_VERTICAL_ADDR_START_VSA_SHIFT  0
#define ILI9325_VERTICAL_ADDR_START_VSA_MASK   (0x1ff << ILI9325_VERTICAL_ADDR_START_VSA_SHIFT)
#  define ILI9325_VERTICAL_ADDR_START_VSA(n)   ((uint16_t)(n) << ILI9325_VERTICAL_ADDR_START_VSA_SHIFT)

/* ILI9325_VERTICAL_ADDR_END,
 * Vertical Address End Position, Offset: 0x53
 */

#define ILI9325_VERTICAL_ADDR_END_VEA_SHIFT    0
#define ILI9325_VERTICAL_ADDR_END_VEA_MASK     (0x1ff << ILI9325_VERTICAL_ADDR_END_VEA_SHIFT)
#  define ILI9325_VERTICAL_ADDR_END_VEA(n)     ((uint16_t)(n) << ILI9325_VERTICAL_ADDR_END_VEA_SHIFT)

/* ILI9325_DRIVER_OUTPUT_CTRL2,
 * Driver Output Control 2, Offset: 0x60
 */

#define ILI9325_DRIVER_OUTPUT_CTRL2_SCN_SHIFT  0
#define ILI9325_DRIVER_OUTPUT_CTRL2_SCN_MASK   (0x3f << ILI9325_DRIVER_OUTPUT_CTRL2_SCN_SHIFT)
#  define ILI9325_DRIVER_OUTPUT_CTRL2_SCN(n)   ((uint16_t)(n) << ILI9325_DRIVER_OUTPUT_CTRL2_SCN_SHIFT)
#define ILI9325_DRIVER_OUTPUT_CTRL2_NL_SHIFT   8
#define ILI9325_DRIVER_OUTPUT_CTRL2_NL_MASK    (0x3f << ILI9325_DRIVER_OUTPUT_CTRL2_NL_SHIFT)
#  define ILI9325_DRIVER_OUTPUT_CTRL2_NL(n)    ((uint16_t)(n) << ILI9325_DRIVER_OUTPUT_CTRL2_NL_SHIFT)
#define ILI9325_DRIVER_OUTPUT_CTRL2_GS         (1 << 15)

/* ILI9325_BASE_IMG_DISP_CTRL,
 * Base Image Display Control, Offset: 0x61
 */

#define ILI9325_BASE_IMG_DISP_CTRL_REV         (1 << 0)
#define ILI9325_BASE_IMG_DISP_CTRL_VLE         (1 << 1)
#define ILI9325_BASE_IMG_DISP_CTRL_NDL         (1 << 2)

/* ILI9325_VERTICAL_SCROLL_CTRL,
 * Vertical Scroll Control, Offset: 0x6a
 */

#define ILI9325_VERTICAL_SCROLL_CTRL_VL_SHIFT  0
#define ILI9325_VERTICAL_SCROLL_CTRL_VL_MASK   (0x1ff << ILI9325_VERTICAL_SCROLL_CTRL_VL_SHIFT)
#  define ILI9325_VERTICAL_SCROLL_CTRL_VL(n)   ((uint16_t)(n) << ILI9325_VERTICAL_SCROLL_CTRL_VL_SHIFT)

/* ILI9325_PARTIAL_IMG1_DISP_SHIFT,
 * Partial Image 1 Display Position, Offset: 0x80
 */

#define ILI9325_PARTIAL_IMG1_DISP_SHIFT_PTDP0_SHIFT 0
#define ILI9325_PARTIAL_IMG1_DISP_SHIFT_PTDP0_MASK  (0x1ff << ILI9325_PARTIAL_IMG1_DISP_SHIFT_PTDP0_SHIFT)
#define ILI9325_PARTIAL_IMG1_DISP_SHIFT_PTDP0(n)    ((uint16_t)(n) << ILI9325_PARTIAL_IMG1_DISP_SHIFT_PTDP0_SHIFT)

/* ILI9325_PARTIAL_IMG1_AREA_START_LINE,
 * Partial Image 1 Area (Start Line), Offset: 0x81
 */

#define ILI9325_PARTIAL_IMG1_AREA_START_LINE_PTSA0_SHIFT 0
#define ILI9325_PARTIAL_IMG1_AREA_START_LINE_PTSA0_MASK  (0x1ff << ILI9325_PARTIAL_IMG1_AREA_START_LINE_PTSA0_SHIFT)
#  define ILI9325_PARTIAL_IMG1_AREA_START_LINE_PTSA0(n)  ((uint16_t)(n) << ILI9325_PARTIAL_IMG1_AREA_START_LINE_PTSA0_SHIFT)

/* ILI9325_PARTIAL_IMG1_AREA_END_LINE,
 * Partial Image 1 Area (End Line), Offset: 0x82
 */

#define ILI9325_PARTIAL_IMG1_AREA_END_LINE_PTEA0_SHIFT 0
#define ILI9325_PARTIAL_IMG1_AREA_END_LINE_PTEA0_MASK  (0x1ff << ILI9325_PARTIAL_IMG1_AREA_END_LINE_PTEA0_SHIFT)
#  define ILI9325_PARTIAL_IMG1_AREA_END_LINE_PTEA0(n)  ((uint16_t)(n) << ILI9325_PARTIAL_IMG1_AREA_END_LINE_PTEA0_SHIFT)

/* ILI9325_PARTIAL_IMG2_DISP_SHIFT,
 * Partial Image 2 Display Position, Offset: 0x83
 */

#define ILI9325_PARTIAL_IMG2_DISP_SHIFT_PTDP1_SHIFT 0
#define ILI9325_PARTIAL_IMG2_DISP_SHIFT_PTDP1_MASK  (0x1ff << ILI9325_PARTIAL_IMG2_DISP_SHIFT_PTDP1_SHIFT)
#  define ILI9325_PARTIAL_IMG2_DISP_SHIFT_PTDP1(n)  ((uint16_t)(n) << ILI9325_PARTIAL_IMG2_DISP_SHIFT_PTDP1_SHIFT)

/* ILI9325_PARTIAL_IMG2_AREA_START_LINE,
 * Partial Image 2 Area (Start Line), Offset: 0x84
 */

#define ILI9325_PARTIAL_IMG2_AREA_START_LINE_PTSA1_SHIFT 0
#define ILI9325_PARTIAL_IMG2_AREA_START_LINE_PTSA1_MASK  (0x1ff << ILI9325_PARTIAL_IMG2_AREA_START_LINE_PTSA1_SHIFT)
#  define ILI9325_PARTIAL_IMG2_AREA_START_LINE_PTSA1(n)  ((uint16_t)(n) << ILI9325_PARTIAL_IMG2_AREA_START_LINE_PTSA1_SHIFT)

/* ILI9325_PARTIAL_IMG2_AREA_END_LINE,
 * Partial Image 2 Area (End Line), Offset: 0x85
 */

#define ILI9325_PARTIAL_IMG2_AREA_END_LINE_PTEA1_SHIFT 0
#define ILI9325_PARTIAL_IMG2_AREA_END_LINE_PTEA1_MASK  (0x1ff << ILI9325_PARTIAL_IMG2_AREA_END_LINE_PTEA1_SHIFT)
#  define ILI9325_PARTIAL_IMG2_AREA_END_LINE_PTEA1(n)  ((uint16_t)(n) << ILI9325_PARTIAL_IMG2_AREA_END_LINE_PTEA1_SHIFT)

/* ILI9325_PANEL_INTERFACE_CTRL1,
 * Panel Interface Control 1, Offset: 0x90
 */

#define ILI9325_PANEL_INTERFACE_CTRL1_RTNI_SHIFT 0
#define ILI9325_PANEL_INTERFACE_CTRL1_RTNI_MASK  (0x1f << ILI9325_PANEL_INTERFACE_CTRL1_RTNI_SHIFT)
#  define ILI9325_PANEL_INTERFACE_CTRL1_RTNI(n)  ((uint16_t)(n) << ILI9325_PANEL_INTERFACE_CTRL1_RTNI_SHIFT)
#define ILI9325_PANEL_INTERFACE_CTRL1_DIVI_SHIFT 8
#define ILI9325_PANEL_INTERFACE_CTRL1_DIVI_MASK  (3 << ILI9325_PANEL_INTERFACE_CTRL1_DIVI_SHIFT)
#  define ILI9325_PANEL_INTERFACE_CTRL1_DIVI(n)  ((uint16_t)(n) << ILI9325_PANEL_INTERFACE_CTRL1_DIVI_SHIFT)

/* ILI9325_PANEL_INTERFACE_CTRL2,
 * Panel Interface Control 2, Offset: 0x92
 */

#define ILI9325_PANEL_INTERFACE_CTRL2_NOWI_SHIFT 8
#define ILI9325_PANEL_INTERFACE_CTRL2_NOWI_MASK  (7 << ILI9325_PANEL_INTERFACE_CTRL2_NOWI_SHIFT)
#  define ILI9325_PANEL_INTERFACE_CTRL2_NOWI(n)  ((uint16_t)(n) << ILI9325_PANEL_INTERFACE_CTRL2_NOWI_SHIFT)

/* ILI9325_PANEL_INTERFACE_CTRL4,
 * Panel Interface Control 4, Offset: 0x95
 */

#define ILI9325_PANEL_INTERFACE_CTRL4_RTNE_SHIFT 0
#define ILI9325_PANEL_INTERFACE_CTRL4_RTNE_MASK  (0x3f << ILI9325_PANEL_INTERFACE_CTRL4_RTNE_SHIFT)
#  define ILI9325_PANEL_INTERFACE_CTRL4_RTNE(n)  ((uint16_t)(n) << ILI9325_PANEL_INTERFACE_CTRL4_RTNE_SHIFT)
#define ILI9325_PANEL_INTERFACE_CTRL4_DIVE_SHIFT 8
#define ILI9325_PANEL_INTERFACE_CTRL4_DIVE_MASK  (3 << ILI9325_PANEL_INTERFACE_CTRL4_DIVE_SHIFT)
#  define ILI9325_PANEL_INTERFACE_CTRL4_DIVE(n)  ((uint16_t)(n) << ILI9325_PANEL_INTERFACE_CTRL4_DIVE_SHIFT)

/* ILI9325_OTP_VCM_PROG_CTRL,
 * OTP VCM Programming Control, Offset: 0xa1
 */

#define ILI9325_OTP_VCM_PROG_CTRL_VCM_OTP_SHIFT 0
#define ILI9325_OTP_VCM_PROG_CTRL_VCM_OTP_MASK  (0x3f << ILI9325_OTP_VCM_PROG_CTRL_VCM_OTP_SHIFT)
#  define ILI9325_OTP_VCM_PROG_CTRL_VCM_OTP(n)  ((uint16_t)(n) << ILI9325_OTP_VCM_PROG_CTRL_VCM_OTP_SHIFT)
#define ILI9325_OTP_VCM_PROG_CTRL_OTP_PGM_EN    (1 << 11)

/* ILI9325_OTP_VCM_STATUS_AND_ENABLE,
 * OTP VCM Status and Enable, Offset: 0xa2
 */

#define ILI9325_OTP_VCM_STATUS_AND_ENABLE_VCM_EN        (1 << 0)
#define ILI9325_OTP_VCM_STATUS_AND_ENABLE_VCM_D_SHIFT   8
#define ILI9325_OTP_VCM_STATUS_AND_ENABLE_VCM_D_MASK    (0x3f << ILI9325_OTP_VCM_STATUS_AND_ENABLE_VCM_D_SHIFT)
#  define ILI9325_OTP_VCM_STATUS_AND_ENABLE_VCM_D(n)    ((uint16_t)(n) << ILI9325_OTP_VCM_STATUS_AND_ENABLE_VCM_D_SHIFT)
#define ILI9325_OTP_VCM_STATUS_AND_ENABLE_PGM_CNT_SHIFT 14
#define ILI9325_OTP_VCM_STATUS_AND_ENABLE_PGM_CNT_MASK  (3 << ILI9325_OTP_VCM_STATUS_AND_ENABLE_PGM_CNT_SHIFT)
#  define ILI9325_OTP_VCM_STATUS_AND_ENABLE_PGM_CNT(n)  ((uint16_t)(n) << IILI9325_OTP_VCM_STATUS_AND_ENABLE_PGM_CNT_SHIFT)

/* ILI9325_OTP_PROG_ID_KEY,
 * OTP Programming ID Key, Offset: 0xa5
 */

#define ILI9325_OTP_PROG_ID_KEY_KEY_SHIFT      0
#define ILI9325_OTP_PROG_ID_KEY_KEY_MASK      (0xffffu << ILI9325_OTP_PROG_ID_KEY_KEY_SHIFT)
#  define ILI9325_OTP_PROG_ID_KEY_KEY(n)      ((uint16_t)(n) << ILI9325_OTP_PROG_ID_KEY_KEY_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_ILI9325_H */

/****************************************************************************
 * include/nuttx/lcd/ili9225.h
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

#ifndef __INCLUDE_NUTTX_LCD_ILI9225_H
#define __INCLUDE_NUTTX_LCD_ILI9225_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ILI9225 ID code */

#define ILI9225_DEVICE_CODE                    0x9225

/* ILI9225 LCD Register Addresses *******************************************/

#define ILI9225_DEVICE_CODE_REG                0x00 /* Driver Code Register */
#define ILI9225_START_OSC_CTRL                 0x00 /* Start Oscillator Control */
#define ILI9225_DRIVER_OUTPUT_CTRL             0x01 /* Start Oscillator Control */
#define ILI9225_LCD_DRIVING_CTRL               0x02 /* LCD Driving Control */
#define ILI9225_ENTRY_MODE                     0x03 /* Entry Mode */
#define ILI9225_DISP_CTRL1                     0x07 /* Display Control 1 */
#define ILI9225_DISP_CTRL2                     0x08 /* Display Control 2 */
#define ILI9225_FRAME_CYCLE_CTRL               0x0b /* Frame Cyce Control */
#define ILI9225_RGB_DISP_INT_CTRL1             0x0c /* RGB Display Interface Control 1 */
#define ILI9225_OSC_CTRL                       0x0f /* Oscillator Control */
#define ILI9225_POWER_CTRL1                    0x10 /* Power Control 1 */
#define ILI9225_POWER_CTRL2                    0x11 /* Power Control 2 */
#define ILI9225_POWER_CTRL3                    0x12 /* Power Control 3 */
#define ILI9225_POWER_CTRL4                    0x13 /* Power Control 4 */
#define ILI9225_POWER_CTRL5                    0x14 /* Power Control 5 */
#define ILI9225_VCI_REC                        0x15 /* VCI Recycling */
#define ILI9225_HORIZONTAL_GRAM_ADDR_SET       0x20 /* Horizontal GRAM Address Set */
#define ILI9225_VERTICAL_GRAM_ADDR_SET         0x21 /* Vertical  GRAM Address Set */
#define ILI9225_GRAM_DATA_REG                  0x22 /* GRAM Data Register */
#define ILI9225_SOFT_RESET                     0x28 /* Software Reset */
#define ILI9225_GATE_SCAN_CTRL                 0x30 /* Gate Scan Control */
#define ILI9225_VER_SCROLL_CTRL1               0x31 /* Vertical Scroll Control 1 */
#define ILI9225_VER_SCROLL_CTRL2               0x32 /* Vertical Scroll Control 2 */
#define ILI9225_VER_SCROLL_CTRL3               0x33 /* Vertical Scroll Control 3 */
#define ILI9225_PART_SCR_DRIV_POS1             0x34 /* Partial Screen Driving Position 1 */
#define ILI9225_PART_SCR_DRIV_POS2             0x35 /* Partial Screen Driving Position 2 */
#define ILI9225_HORIZONTAL_ADDR_END            0x36 /* Horizontal Address Start Position */
#define ILI9225_HORIZONTAL_ADDR_START          0x37 /* Horizontal Address End Position */
#define ILI9225_VERTICAL_ADDR_END              0x38 /* Vertical Address Start Position */
#define ILI9225_VERTICAL_ADDR_START            0x39 /* Vertical Address End Position */
#define ILI9225_GAMMA_CTRL1                    0x50 /* Gamma Control 1 */
#define ILI9225_GAMMA_CTRL2                    0x51 /* Gamma Control 2 */
#define ILI9225_GAMMA_CTRL3                    0x52 /* Gamma Control 3 */
#define ILI9225_GAMMA_CTRL4                    0x53 /* Gamma Control 4 */
#define ILI9225_GAMMA_CTRL5                    0x54 /* Gamma Control 5 */
#define ILI9225_GAMMA_CTRL6                    0x55 /* Gamma Control 6 */
#define ILI9225_GAMMA_CTRL7                    0x56 /* Gamma Control 7 */
#define ILI9225_GAMMA_CTRL8                    0x57 /* Gamma Control 8 */
#define ILI9225_GAMMA_CTRL9                    0x58 /* Gamma Control 9 */
#define ILI9225_GAMMA_CTRL10                   0x59 /* Gamma Control 10 */
#define ILI9225_NV_MEM_DATA                    0x60 /* NV Memory Data Programming */
#define ILI9225_NV_MEM_CTRL                    0x61 /* NV Memory Control */
#define ILI9225_NV_MEM_STS                     0x62 /* NV Memory Status */
#define ILI9225_NV_MEM_PROTECTION              0x63 /* NV Memory Protection Key */
#define ILI9225_ID_CODE                        0x65 /* ID Code */

/* ILI9225 LCD Register Bit Definitions *************************************/

/* ILI9225_START_OSC_CTRL, Start Oscillator Control, Offset: 0x00)  */

#define ILI9225_START_OSC_CTRL_EN              (1 << 0)

/* ILI9225_DRIVER_OUTPUT_CTRL, Start Oscillator Control, Offset: 0x01 */

#define ILI9225_DRIVER_OUTPUT_CTRL_NL_SHIFT     (0)
#define ILI9225_DRIVER_OUTPUT_CTRL_NL_MASK      (0x1f << ILI9225_DRIVER_OUTPUT_CTRL_NL_SHIFT)
#define ILI9225_DRIVER_OUTPUT_CTRL_NL(n)        (((uint16_t)(n) << ILI9225_DRIVER_OUTPUT_CTRL_NL_SHIFT) & ILI9225_DRIVER_OUTPUT_CTRL_NL_MASK)
#define ILI9225_DRIVER_OUTPUT_CTRL_SS           (1 << 8)     /* Select shift direction of outputs from source */
#define ILI9225_DRIVER_OUTPUT_CTRL_GS           (1 << 9)     /* Select shift direction of outputs from gate */
#define ILI9225_DRIVER_OUTPUT_CTRL_SM           (1 << 10)    /* Set the scan order by the gate driver */
#define ILI9225_DRIVER_OUTPUT_CTRL_EPL          (1 << 12)    /* Inverts the polarity of signals from ENABLE pin */
#define ILI9225_DRIVER_OUTPUT_CTRL_DPL          (1 << 13)    /* Inverts the polarity of signals from DOTCLK pin */
#define ILI9225_DRIVER_OUTPUT_CTRL_HSPL         (1 << 14)    /* Inverts the polarity of signals from HSYNC pin */
#define ILI9225_DRIVER_OUTPUT_CTRL_VSPL         (1 << 15)    /* Inverts the polarity of signals from VSYNC pin */

/* ILI9225_LCD_DRIVING_CTRL, LCD Driving Control, Offset: 0x02 */

#define ILI9225_LCD_DRIVING_CTRL_FLD           (1 << 0)
#define ILI9225_LCD_DRIVING_CTRL_INV0          (1 << 8)
#define ILI9225_LCD_DRIVING_CTRL_INV1          (1 << 9)

/* ILI9225_ENTRY_MODE, Entry Mode, Offset: 0x03 */

#define ILI9225_ENTRY_MODE_AM                  (1 << 3)     /* Control the GRAM update direction */
#define ILI9225_ENTRY_MODE_ID_SHIFT            (4)          /* Control the address counter */
#define ILI9225_ENTRY_MODE_ID_MASK             (3 << ILI9225_ENTRY_MODE_ID_SHIFT)
#define ILI9225_ENTRY_MODE_ID(n)               (((uint16_t)(n) << ILI9225_ENTRY_MODE_ID_SHIFT) & ILI9225_ENTRY_MODE_ID_MASK)
#define ILI9225_ENTRY_MODE_MDT_SHIFT           (8)
#define ILI9225_ENTRY_MODE_MDT_MASK            (3 << ILI9225_ENTRY_MODE_MDT_SHIFT)
#define ILI9225_ENTRY_MODE_MDT                 (((uint16_t)(n) << ILI9225_ENTRY_MODE_MDT_SHIFT) & ILI9225_ENTRY_MODE_MDT_MASK)
#define ILI9225_ENTRY_MODE_BGR                 (1 << 12)    /* Swap the R and B order of written data */

/* ILI9225_DISP_CTRL1, Display Control 1, Offset: 0x07 */

#define ILI9225_DISP_CTRL1_D_SHIFT             (0)       /* Turn on/off the display panel */
#define ILI9225_DISP_CTRL1_D_MASK              (3 << ILI9225_DISP_CTRL1_D_SHIFT)
#define ILI9225_DISP_CTRL1_D(n)                (((uint16_t)(n) << ILI9225_DISP_CTRL1_D_SHIFT) & ILI9225_DISP_CTRL1_D_MASK)
#define ILI9225_DISP_CTRL1_REV                 (1 << 2)  /* Invert grayscale levels */
#define ILI9225_DISP_CTRL1_CL                  (1 << 3)  /* Select 8-color display mode */
#define ILI9225_DISP_CTRL1_GON                 (1 << 4)  /* Set the output leve of gate driver */
#define ILI9225_DISP_CTRL1_TEMON               (1 << 12) /* Enable the frame flag output signal */

/* ILI9225_DISP_CTRL2, Display Control 2, Offset: 0x08 */

#define ILI9225_DISP_CTRL2_BP_SHIFT            (0)  /* Numbers of lines for Back Porch */
#define ILI9225_DISP_CTRL2_BP_MASK             (0xf << ILI9225_DISP_CTRL2_BP_SHIFT)
#define ILI9225_DISP_CTRL2_BP(n)               (((uint16_t)(n) << ILI9225_DISP_CTRL2_BP_SHIFT) & ILI9225_DISP_CTRL2_BP_MASK)
#define ILI9225_DISP_CTRL2_FP_SHIFT            (8)  /* Numbers of lines for Front Porch */
#define ILI9225_DISP_CTRL2_FP_MASK             (0xf << ILI9225_DISP_CTRL2_FP_SHIFT)
#define ILI9225_DISP_CTRL2_FP(n)               (((uint16_t)(n) << ILI9225_DISP_CTRL2_FP_SHIFT) & ILI9225_DISP_CTRL2_FP_MASK)

/* ILI9225_FRAME_CYCLE_CONTROL, Frame Cycle Control, Offset: 0x0b */

#define ILI9225_FRAME_CYCLE_CTRL_RTN_SHIFT     (0)  /* Set the clock cycle number */
#define ILI9225_FRAME_CYCLE_CTRL_RTN_MASK      (0xf << ILI9225_FRAME_CYCLE_CTRL_RTN_SHIFT)
#define ILI9225_FRAME_CYCLE_CTRL_RTN(n)        (((uint16_t)(n) << ILI9225_FRAME_CYCLE_CTRL_RTN_SHIFT) & ILI9225_FRAME_CYCLE_CTRL_RTN_MASK)
#define ILI9225_FRAME_CYCLE_CTRL_STD_SHIFT     (8)  /* Set delay amount from gate edge */
#define ILI9225_FRAME_CYCLE_CTRL_STD_MASK      (0xf << ILI9225_FRAME_CYCLE_CTRL_STD_SHIFT)
#define ILI9225_FRAME_CYCLE_CTRL_STD(n)        (((uint16_t)(n) << ILI9225_FRAME_CYCLE_CTRL_STD_SHIFT) & ILI9225_FRAME_CYCLE_CTRL_STD_MASK)
#define ILI9225_FRAME_CYCLE_CTRL_NO_SHIFT      (12) /* Set amount of non-everlay for the gate output */
#define ILI9225_FRAME_CYCLE_CTRL_NO_MASK       (0xf << ILI9225_FRAME_CYCLE_CTRL_NO_SHIFT)
#define ILI9225_FRAME_CYCLE_CTRL_NO(n)         (((uint16_t)(n) << ILI9225_FRAME_CYCLE_CTRL_NO_SHIFT) & ILI9225_FRAME_CYCLE_CTRL_NO_MASK)

/* ILI9225_RGB_DISP_INT_CTRL1,
 * RGB Display Interface Control 1, Offset: 0x0c
 */

#define ILI9225_RGB_DISP_INT_CTRL1_RIM_SHIFT   (0)      /* Selest the data bus width */
#define ILI9225_RGB_DISP_INT_CTRL1_RIM_MASK    (3 << ILI9225_RGB_DISP_INT_CTRL1_RIM_SHIFT)
#define ILI9225_RGB_DISP_INT_CTRL1_RIM(n)      (((uint16_t)(n) << ILI9225_RGB_DISP_INT_CTRL1_RIM_SHIFT) & ILI9225_RGB_DISP_INT_CTRL1_RIM_MASK)
#define ILI9225_RGB_DISP_INT_CTRL1_DM          (1 << 4) /* Select RGB interface */
#define ILI9225_RGB_DISP_INT_CTRL1_RM          (1 << 8) /* Select RGB interface acces to GRAM */

/* ILI9225_OSC_CTRL, Frame Maker Position, Offset: 0x0f */

#define ILI9225_OSC_CTRL_EN                    (1 << 0) /* Oscillator on */
#define ILI9225_OSC_CTRL_FOSC_SHIFT            (8)      /* Select OSC frequency */
#define ILI9225_OSC_CTRL_FOSC_MASK             (0xf << ILI9225_OSC_CTRL_FOSC_SHIFT)
#define ILI9225_OSC_CTRL_FOSC(n)               (((uint16_t)(n) << ILI9225_OSC_CTRL_FOSC_SHIFT) & ILI9225_OSC_CTRL_FOSC_MASK)

/* ILI9225_POWER_CTRL1, Power Control 1, Offset: 0x10 */

#define ILI9225_POWER_CTRL1_STB                (1 << 0) /* Enter sleep mode */
#define ILI9225_POWER_CTRL1_DSTB               (1 << 1) /* Enter deep standby mode */
#define ILI9225_POWER_CTRL1_SAP_SHIFT          (8)      /* Set the driving capability of source driver */
#define ILI9225_POWER_CTRL1_SAP_MASK           (0xf << ILI9225_POWER_CTRL1_SAP_SHIFT)
#define ILI9225_POWER_CTRL1_SAP(n)             (((uint16_t)(n) << ILI9225_POWER_CTRL1_SAP_SHIFT) & ILI9225_POWER_CTRL1_SAP_MASK)

/* ILI9225_POWER_CTRL2, Power Control 2, Offset: 0x11 */

#define ILI9225_POWER_CTRL2_VC_SHIFT           (0)       /* Set the VCI1 voltage */
#define ILI9225_POWER_CTRL2_VC_MASK            (0xf << ILI9225_POWER_CTRL2_VC_SHIFT)
#define ILI9225_POWER_CTRL2_VC(n)              (((uint16_t)(n) << ILI9225_POWER_CTRL2_VC_SHIFT) & ILI9225_POWER_CTRL2_VC_MASK)
#define ILI9225_POWER_CTRL2_VC_EN              (1 << 3)  /* Enable VCI1 generation amp */
#define ILI9225_POWER_CTRL2_AON                (1 << 4)  /* Operation starting bit for amp */
#define ILI9225_POWER_CTRL2_PON                (1 << 8)  /* Operation starting bit for booster circuit 1 */
#define ILI9225_POWER_CTRL2_PON1               (1 << 9)  /* Operation starting bit for booster circuit 2 (VGH) */
#define ILI9225_POWER_CTRL2_PON2               (1 << 10) /* Operation starting bit for booster circuit 2 (VGL) */
#define ILI9225_POWER_CTRL2_PON3               (1 << 11) /* Operation starting bit for booster circout 3 (VCL) */
#define ILI9225_POWER_CTRL2_APON               (1 << 12) /* Automatic boosting operatio starting bit */

/* ILI9225_POWER_CTRL3, Power Control 3, Offset: 0x12 */

#define ILI9225_POWER_CTRL3_DC3_SHIFT          (0)  /* Operating frequency in circuit 3 */
#define ILI9225_POWER_CTRL3_DC3_MASK           (0x7 << ILI9225_POWER_CTRL3_DC3_SHIFT)
#define ILI9225_POWER_CTRL3_DC3(n)             (((uint16_t)(n) << ILI9225_POWER_CTRL3_DC3_SHIFT) & ILI9225_POWER_CTRL3_DC3_MASK)
#define ILI9225_POWER_CTRL3_DC2_SHIFT          (4)  /* Operating frequency in circuit 2 */
#define ILI9225_POWER_CTRL3_DC2_MASK           (0x7 << ILI9225_POWER_CTRL3_DC2_SHIFT)
#define ILI9225_POWER_CTRL3_DC2(n)             (((uint16_t)(n) << ILI9225_POWER_CTRL3_DC2_SHIFT) & ILI9225_POWER_CTRL3_DC2_MASK)
#define ILI9225_POWER_CTRL3_DC1_SHIFT          (8)  /* Operating frequency in circuit 1 */
#define ILI9225_POWER_CTRL3_DC1_MASK           (0x7 << ILI9225_POWER_CTRL3_DC1_SHIFT)
#define ILI9225_POWER_CTRL3_DC1(n)             (((uint16_t)(n) << ILI9225_POWER_CTRL3_DC1_SHIFT) & ILI9225_POWER_CTRL3_DC1_MASK)
#define ILI9225_POWER_CTRL3_BT_SHIFT           (12) /* Output factor of step-up circuit */
#define ILI9225_POWER_CTRL3_BT_MASK            (0x7 << ILI9225_POWER_CTRL3_BT_SHIFT)
#define ILI9225_POWER_CTRL3_BT(n)              (((uint16_t)(n) << ILI9225_POWER_CTRL3_BT_SHIFT) & ILI9225_POWER_CTRL3_BT_MASK)

/* ILI9225_POWER_CTRL4, Power Control 4, Offset: 0x13 */

#define ILI9225_POWER_CTRL4_GVD_SHIFT          (0)  /* Amplifying factor for the GVGG voltage */
#define ILI9225_POWER_CTRL4_GVD_MASK           (0x7f << ILI9225_POWER_CTRL4_GVD_SHIFT)
#define ILI9225_POWER_CTRL4_GVD(n)             (((uint16_t)(n) << ILI9225_POWER_CTRL4_GVD_SHIFT) & ILI9225_POWER_CTRL4_GVD_MASK)

/* ILI9225_POWER_CTRL5, Power Control 5, Offset: 0x14 */

#define ILI9225_POWER_CTRL5_VML_SHIFT          (0)  /* Alternating amplitudes of VCOM at the VCOM drive */
#define ILI9225_POWER_CTRL5_VML_MASK           (0x7f << ILI9225_POWER_CTRL5_VML_SHIFT)
#define ILI9225_POWER_CTRL5_VML(n)             (((uint16_t)(n) << ILI9225_POWER_CTRL5_VML_SHIFT) & ILI9225_POWER_CTRL5_VML_MASK)
#define ILI9225_POWER_CTRL5_VCM_SHIFT          (8)  /* Set the VCOMG voltage */
#define ILI9225_POWER_CTRL5_VCM_MASK           (0x7f << ILI9225_POWER_CTRL5_VCM_SHIFT)
#define ILI9225_POWER_CTRL5_VCM(n)             (((uint16_t)(n) << ILI9225_POWER_CTRL5_VCM_SHIFT) & ILI9225_POWER_CTRL5_VCM_MASK)
#define ILI9225_POWER_CTRL5_VCOMG              (1 << 15)   /* Fix VCOM signal at AVSS */

/* ILI9225_VCI_REC, VCI Recycling, Offset: 0x15 */

#define ILI9225_VCI_REC_VCIR_SHIFT             (4)  /* Set CVI recycling period */
#define ILI9225_VCI_REC_VCIR_MASK              (0x7 << ILI9225_VCI_REC_VCIR_SHIFT)
#define ILI9225_VCI_REC_VCIR(n)                (((uint16_t)(n) << ILI9225_VCI_REC_VCIR_MASK) & ILI9225_VCI_REC_VCIR_SHIFT)

/* ILI9225_HORIZONTAL_GRAM_ADDR_SET,
 * Horizontal GRAM Address Set, Offset: 0x20
 */

#define ILI9225_HORIZONTAL_GRAM_ADDR_SET_AD_SHIFT (0)   /* Set the initial value of address counter */
#define ILI9225_HORIZONTAL_GRAM_ADDR_SET_AD_MASK  (0xff << ILI9225_HORIZONTAL_GRAM_ADDR_SET_AD_SHIFT)
#define ILI9225_HORIZONTAL_GRAM_ADDR_SET_AD(n)    (((uint16_t)(n) << ILI9225_HORIZONTAL_GRAM_ADDR_SET_AD_SHIFT) & ILI9225_HORIZONTAL_GRAM_ADDR_SET_AD_MASK)

/* ILI9225_VERTICAL_GRAM_ADDR_SET,
 * Vertical  GRAM Address Set, Offset: 0x21
 */

#define ILI9225_VERTICAL_GRAM_ADDR_SET_AD_SHIFT   (0)  /* Set the initial value of address counter */
#define ILI9225_VERTICAL_GRAM_ADDR_SET_AD_MASK    (0xff << ILI9225_VERTICAL_GRAM_ADDR_SET_AD_SHIFT)
#define ILI9225_VERTICAL_GRAM_ADDR_SET_AD(n)      (((uint16_t)(n) << ILI9225_VERTICAL_GRAM_ADDR_SET_AD_SHIFT) & ILI9225_VERTICAL_GRAM_ADDR_SET_AD_MASK)

/* ILI9225_GRAM_DATA_REG, Read/Write Data from/to GRAM, Offset: 0x22 */

#define ILI9225_GRAM_DATA_REG_WD_SHIFT        (0)
#define ILI9225_GRAM_DATA_REG_WD_MASK         (0xffff << ILI9225_GRAM_DATA_REGWD_SHIFT)
#define ILI9225_GRAM_DATA_REG_WD(n)           (((uint16_t)(n) << ILI9225_GRAM_DATA_REG_WD_SHIFT) & ILI9225_GRAM_DATA_REG_WD_MASK)

#define ILI9225_GRAM_DATA_REG_RD_SHIFT        (0)
#define ILI9225_GRAM_DATA_REG_RD_MASK         (0xffff << ILI9225_GRAM_DATA_REG_RD_SHIFT)
#define ILI9225_GRAM_DATA_REG_RD(n)           (((uint16_t)(n) << ILI9225_GRAM_DATA_REG_RD_SHIFT) & ILI9225_GRAM_DATA_REG_RD_MASK)

/* ILI9225_SOFT_RESET, Software Reset, Offset: 0x28 */

#define ILI9225_SOFT_RESET_CODE_SHIFT         (0) /* Soft reset parameter is 0x00CE */
#define ILI9225_SOFT_RESET_CODE_MASK          (0xffff << ILI9225_SOFT_RESET_CODE_SHIFT)
#define ILI9225_SOFT_RESET_CODE(n)            (((uint16_t)(n) << ILI9225_SOFT_RESET_CODE_SHIFT) & ILI9225_SOFT_RESET_CODE_MASK)

/* ILI9225_GATE_SCAN_CTRL, Gate Scan Control, Offset: 0x30 */

#define ILI9225_GATE_SCAN_CTRL_SCN_SHIFT      (0) /* Specify the gate line from which the draver starts scan */
#define ILI9225_GATE_SCAN_CTRL_SCN_MASK       (0x1f << ILI9225_GATE_SCAN_CTRL_SCN_SHIFT)
#define ILI9225_GATE_SCAN_CTRL_SCN(n)         (((uint16_t)(n) << ILI9225_GATE_SCAN_CTRL_SCN_SHIFT) & ILI9225_GATE_SCAN_CTRL_SCN_MASK)

/* ILI9225_VER_SCROLL_CTRL1, Vertical Scroll Control 1, Offset: 0x31 */

#define ILI9225_VER_SCROLL_CTRL1_SEA_SHIFT    (0) /* Specify scroll start address */
#define ILI9225_VER_SCROLL_CTRL1_SEA_MASK     (0xff << ILI9225_VER_SCROLL_CTRL1_SEA_SHIFT)
#define ILI9225_VER_SCROLL_CTRL1_SEA(n)       (((uint16_t)(n) << ILI9225_VER_SCROLL_CTRL1_SEA_SHIFT) & ILI9225_VER_SCROLL_CTRL1_SEA_MASK)

/* ILI9225_VER_SCROLL_CTRL2, Vertical Scroll Control 2, Offset: 0x32 */

#define ILI9225_VER_SCROLL_CTRL2_SSA_SHIFT    (0) /* Specify scroll end address */
#define ILI9225_VER_SCROLL_CTRL2_SSA_MASK     (0xff << ILI9225_VER_SCROLL_CTRL2_SSA_SHIFT)
#define ILI9225_VER_SCROLL_CTRL2_SSA(n)       (((uint16_t)(n) << ILI9225_VER_SCROLL_CTRL2_SSA_SHIFT) & ILI9225_VER_SCROLL_CTRL2_SSA_MASK)

/* ILI9225_VER_SCROLL_CTRL3, Vertical Scroll Control 3, Offset: 0x33 */

#define ILI9225_VER_SCROLL_CTRL3_SST_SHIFT    (0) /* Specify scroll start and step */
#define ILI9225_VER_SCROLL_CTRL3_SST_MASK     (0xff << ILI9225_VER_SCROLL_CTRL3_SST_SHIFT)
#define ILI9225_VER_SCROLL_CTRL3_SST(n)       (((uint16_t)(n) << ILI9225_VER_SCROLL_CTRL3_SST_SHIFT) & ILI9225_VER_SCROLL_CTRL3_SST_MASK)

/* ILI9225_PART_SCR_DRIV_POS1, Partial Screen Driving Position 1,
 * Offset: 0x34
 */

#define ILI9225_PART_SCR_DRIV_POS1_SE_SHIFT   (0)   /* Specify the driving end position for the screen */
#define ILI9225_PART_SCR_DRIV_POS1_SE_MASK    (0xff << ILI9225_PART_SCR_DRIV_POS1_SE_SHIFT)
#define ILI9225_PART_SCR_DRIV_POS1_SE(n)      (((uint16_t)(n) << ILI9225_PART_SCR_DRIV_POS1_SE_SHIFT) & ILI9225_PART_SCR_DRIV_POS1_SE_MASK)

/* ILI9225_PART_SCR_DRIV_POS2, Partial Screen Driving Position 2,
 * Offset: 0x35
 */

#define ILI9225_PART_SCR_DRIV_POS2_SS_SHIFT   (0)   /* Specify the driving start position for the screen */
#define ILI9225_PART_SCR_DRIV_POS2_SS_MASK    (0xff << ILI9225_PART_SCR_DRIV_POS2_SS_SHIFT)
#define ILI9225_PART_SCR_DRIV_POS2_SS(n)      (((uint16_t)(n) << ILI9225_PART_SCR_DRIV_POS2_SS_SHIFT) & ILI9225_PART_SCR_DRIV_POS2_SS_MASK)

/* ILI9225_HORIZONTAL_ADDR_END,
 * Horizontal Address End Position, Offset: 0x36
 */

#define ILI9225_HORIZONTAL_ADDR_END_HEA_SHIFT  (0)  /* End of the window address area in horizontal direction */
#define ILI9225_HORIZONTAL_ADDR_END_HEA_MASK   (0xff << ILI9225_HORIZONTAL_ADDR_END_HEA_SHIFT)
#define ILI9225_HORIZONTAL_ADDR_END_HEA(n)     (((uint16_t)(n) << ILI9225_HORIZONTAL_ADDR_END_HEA_SHIFT) & ILI9225_HORIZONTAL_ADDR_END_HEA_MASK)

/* ILI9225_HORIZONTAL_ADDR_START,
 * Horizontal Address Start Position, Offset: 0x37
 */

#define ILI9225_HORIZONTAL_ADDR_START_HSA_SHIFT (0)  /* Start of the window address area in horizontal direction */
#define ILI9225_HORIZONTAL_ADDR_START_HSA_MASK  (0xff << ILI9225_HORIZONTAL_ADDR_START_HSA_SHIFT)
#define ILI9225_HORIZONTAL_ADDR_START_HSA(n)    (((uint16_t)(n) << ILI9225_HORIZONTAL_ADDR_START_HSA_SHIFT) & ILI9225_HORIZONTAL_ADDR_START_HSA_MASK)

/* ILI9225_VERTICAL_ADDR_END,
 * Vertical Address End Position, Offset: 0x38
 */

#define ILI9225_VERTICAL_ADDR_END_VEA_SHIFT    (0)  /* End of the window address area in vertical direction */
#define ILI9225_VERTICAL_ADDR_END_VEA_MASK     (0xff << ILI9225_VERTICAL_ADDR_END_VEA_SHIFT)
#define ILI9225_VERTICAL_ADDR_END_VEA(n)       (((uint16_t)(n) << ILI9225_VERTICAL_ADDR_END_VEA_SHIFT) & ILI9225_VERTICAL_ADDR_END_VEA_MASK)

/* ILI9225_VERTICAL_ADDR_START,
 * Vertical Address Start Position, Offset: 0x39
 */

#define ILI9225_VERTICAL_ADDR_START_VSA_SHIFT  (0)  /* Start of the window address area in vertical direction */
#define ILI9225_VERTICAL_ADDR_START_VSA_MASK   (0xff << ILI9225_VERTICAL_ADDR_START_VSA_SHIFT)
#define ILI9225_VERTICAL_ADDR_START_VSA(n)     (((uint16_t)(n) << ILI9225_VERTICAL_ADDR_START_VSA_SHIFT) & ILI9225_VERTICAL_ADDR_START_VSA_MASK)

/* ILI9225_GAMMA_CTRL1, Gamma Control 1, Offset: 0x50 */

#define ILI9225_GAMMA_CTRL1_KP0_SHIFT          (0)
#define ILI9225_GAMMA_CTRL1_KP0_MASK           (0xf << ILI9225_GAMMA_CTRL1_KP0_SHIFT)
#define ILI9225_GAMMA_CTRL1_KP0(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL1_KP0_SHIFT) & ILI9225_GAMMA_CTRL1_KP0_MASK)
#define ILI9225_GAMMA_CTRL1_KP1_SHIFT          (8)
#define ILI9225_GAMMA_CTRL1_KP1_MASK           (0xf << ILI9225_GAMMA_CTRL1_KP1_SHIFT)
#define ILI9225_GAMMA_CTRL1_KP1(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL1_KP1_SHIFT) & ILI9225_GAMMA_CTRL1_KP1_MASK)

/* ILI9225_GAMMA_CTRL2, Gamma Control 2, Offset: 0x51 */

#define ILI9225_GAMMA_CTRL2_KP2_SHIFT          (0)
#define ILI9225_GAMMA_CTRL2_KP2_MASK           (0xf << ILI9225_GAMMA_CTRL2_KP2_SHIFT)
#define ILI9225_GAMMA_CTRL2_KP2(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL2_KP2_SHIFT) & ILI9225_GAMMA_CTRL2_KP2_MASK)
#define ILI9225_GAMMA_CTRL2_KP3_SHIFT          (8)
#define ILI9225_GAMMA_CTRL2_KP3_MASK           (0xf << ILI9225_GAMMA_CTRL2_KP3_SHIFT)
#define ILI9225_GAMMA_CTRL2_KP3(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL2_KP3_SHIFT) & ILI9225_GAMMA_CTRL2_KP3_MASK)

/* ILI9225_GAMMA_CTRL3, Gamma Control 3, Offset: 0x52 */

#define ILI9225_GAMMA_CTRL3_KP4_SHIFT          (0)
#define ILI9225_GAMMA_CTRL3_KP4_MASK           (0xf << ILI9225_GAMMA_CTRL3_KP4_SHIFT)
#define ILI9225_GAMMA_CTRL3_KP4(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL3_KP4_SHIFT) & ILI9225_GAMMA_CTRL3_KP4_MASK)
#define ILI9225_GAMMA_CTRL3_KP5_SHIFT          (8)
#define ILI9225_GAMMA_CTRL3_KP5_MASK           (0xf << ILI9225_GAMMA_CTRL3_KP5_SHIFT)
#define ILI9225_GAMMA_CTRL3_KP5(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL3_KP5_SHIFT) & ILI9225_GAMMA_CTRL3_KP5_MASK)

/* ILI9225_GAMMA_CTRL4, Gamma Control 4, Offset: 0x53 */

#define ILI9225_GAMMA_CTRL4_RP0_SHIFT          (0)
#define ILI9225_GAMMA_CTRL4_RP0_MASK           (0xf << ILI9225_GAMMA_CTRL4_RP0_SHIFT)
#define ILI9225_GAMMA_CTRL4_RP0(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL4_RP0_SHIFT) & ILI9225_GAMMA_CTRL4_RP0_MASK)
#define ILI9225_GAMMA_CTRL4_RP1_SHIFT          (8)
#define ILI9225_GAMMA_CTRL4_RP1_MASK           (0xf << ILI9225_GAMMA_CTRL4_RP1_SHIFT)
#define ILI9225_GAMMA_CTRL4_RP1(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL4_RP1_SHIFT) & ILI9225_GAMMA_CTR4_RP1_MASK)

/* ILI9225_GAMMA_CTRL5, Gamma Control 5, Offset: 0x54 */

#define ILI9225_GAMMA_CTRL5_KN0_SHIFT          (0)
#define ILI9225_GAMMA_CTRL5_KN0_MASK           (0xf << ILI9225_GAMMA_CTRL5_KN0_SHIFT)
#define ILI9225_GAMMA_CTRL5_KN0(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL5_KN0_SHIFT) & ILI9225_GAMMA_CTRL5_KN0_MASK)
#define ILI9225_GAMMA_CTRL5_KN1_SHIFT          (8)
#define ILI9225_GAMMA_CTRL5_KN1_MASK           (0xf << ILI9225_GAMMA_CTRL5_KN1_SHIFT)
#define ILI9225_GAMMA_CTRL5_KN1(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL5_KN1_SHIFT) & ILI9225_GAMMA_CTRL5_KN1_MASK)

/* ILI9225_GAMMA_CTRL6, Gamma Control 6, Offset: 0x55 */

#define ILI9225_GAMMA_CTRL6_KN2_SHIFT          (0)
#define ILI9225_GAMMA_CTRL6_KN2_MASK           (0xf << ILI9225_GAMMA_CTRL6_KN2_SHIFT)
#define ILI9225_GAMMA_CTRL6_KN2(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL6_KN2_SHIFT) & ILI9225_GAMMA_CTRL6_KN2_MASK)
#define ILI9225_GAMMA_CTRL6_KN3_SHIFT          (8)
#define ILI9225_GAMMA_CTRL6_KN3_MASK           (0xf << ILI9225_GAMMA_CTRL6_KN3_SHIFT)
#define ILI9225_GAMMA_CTRL6_KN3(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL6_KN3_SHIFT) & ILI9225_GAMMA_CTRL6_KN3_MASK)

/* ILI9225_GAMMA_CTRL7, Gamma Control 7, Offset: 0x56 */

#define ILI9225_GAMMA_CTRL7_KN4_SHIFT          (0)
#define ILI9225_GAMMA_CTRL7_KN4_MASK           (0xf << ILI9225_GAMMA_CTRL7_KN4_SHIFT)
#define ILI9225_GAMMA_CTRL7_KN4(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL7_KN4_SHIFT) & ILI9225_GAMMA_CTRL7_KN4_MASK)
#define ILI9225_GAMMA_CTRL7_KN5_SHIFT          (8)
#define ILI9225_GAMMA_CTRL7_KN5_MASK           (0xf << ILI9225_GAMMA_CTRL7_KN5_SHIFT)
#define ILI9225_GAMMA_CTRL7_KN5(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL7_KN5_SHIFT) & ILI9225_GAMMA_CTRL7_KN5_MASK)

/* ILI9225_GAMMA_CTRL8, Gamma Control 8, Offset: 0x57 */

#define ILI9225_GAMMA_CTRL8_RN0_SHIFT          (0)
#define ILI9225_GAMMA_CTRL8_RN0_MASK           (0xf << ILI9225_GAMMA_CTRL8_RN0_SHIFT)
#define ILI9225_GAMMA_CTRL8_RN0(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL8_RN0_SHIFT) & ILI9225_GAMMA_CTRL8_RN0_MASK)
#define ILI9225_GAMMA_CTRL8_RN1_SHIFT          (8)
#define ILI9225_GAMMA_CTRL8_RN1_MASK           (0xf << ILI9225_GAMMA_CTRL8_RN1_SHIFT)
#define ILI9225_GAMMA_CTRL8_RN1(n)             (((uint16_t)(n) << ILI9225_GAMMA_CTRL8_RN1_SHIFT) & ILI9225_GAMMA_CTRL8_RN1_MASK)

/* ILI9225_GAMMA_CTR9, Gamma Control 9, Offset: 0x58 */

#define ILI9225_GAMMA_CTRL9_VRP0_SHIFT         (0)
#define ILI9225_GAMMA_CTRL9_VRP0_MASK          (0x1f << ILI9225_GAMMA_CTRL9_VRP0_SHIFT)
#define ILI9225_GAMMA_CTRL9_VRP0(n)            (((uint16_t)(n) << ILI9225_GAMMA_CTRL9_VRP0_SHIFT) & ILI9225_GAMMA_CTRL9_VRP0_MASK)
#define ILI9225_GAMMA_CTRL9_VRP1_SHIFT         (8)
#define ILI9225_GAMMA_CTRL9_VRP1_MASK          (0x1f << ILI9225_GAMMA_CTRL9_VRP1_SHIFT)
#define ILI9225_GAMMA_CTRL9_VRP1(n)            (((uint16_t)(n) << ILI9225_GAMMA_CTRL9_VRP1_SHIFT) & ILI9225_GAMMA_CTRL9_VRP1_MASK)

/* ILI9225_GAMMA_CTR10, Gamma Control 10, Offset: 0x59 */

#define ILI9225_GAMMA_CTRL10_VRN0_SHIFT        (0)
#define ILI9225_GAMMA_CTRL10_VRN0_MASK         (0x1f << ILI9225_GAMMA_CTRL10_VRN0_SHIFT)
#define ILI9225_GAMMA_CTRL10_VRN0(n)           (((uint16_t)(n) << ILI9225_GAMMA_CTRL10_VRN0_SHIFT) & ILI9225_GAMMA_CTRL10_VRN0_MASK)
#define ILI9225_GAMMA_CTRL10_VRN1_SHIFT        (8)
#define ILI9225_GAMMA_CTRL10_VRN1_MASK         (0x1f << ILI9225_GAMMA_CTRL10_VRN1_SHIFT)
#define ILI9225_GAMMA_CTRL10_VRN1(n)           (((uint16_t)(n) << ILI9225_GAMMA_CTRL10_VRN1_SHIFT) & ILI9225_GAMMA_CTRL10_VRN1_MASK)

/* ILI9225_NV_MEM_DATA, NV Memory Data Programming, Offset: 0x60 */

#define ILI9225_NV_MEM_DATA_VM_SHIFT           (0)
#define ILI9225_NV_MEM_DATA_VM_MASK            (0xff << ILI9225_NV_MEM_DATA_VM_SHIFT)
#define ILI9225_NV_MEM_DATA_VM(n)              (((uint16_t)(n) << ILI9225_NV_MEM_DATA_VM_SHIFT) & ILI9225_NV_MEM_DATA_VM_MASK)

/* ILI9225_NV_MEM_CTRL, NV Memory Data Control, Offset: 0x61 */

#define ILI9225_NV_MEM_CTRL_VCM_PGM_EN         (1 << 0)
#define ILI9225_NV_MEM_CTRL_ID_PGM_EN          (1 << 1)
#define ILI9225_NV_MEM_CTRL_VCM_SEL            (1 << 8)

/* ILI9225_NV_MEM_STS, NV Memory Data Status, Offset: 0x62 */

#define ILI9225_NV_MEM_STS_VCM_SHIFT           (0)
#define ILI9225_NV_MEM_STS_VCM_MASK            (0x7f << ILI9225_NV_MEM_STS_VCM_SHIFT)
#define ILI9225_NV_MEM_DATA_VM(n)              (((uint16_t)(n) << ILI9225_NV_MEM_STS_VCM_SHIFT) & ILI9225_NV_MEM_STS_VCM_MASK)
#define ILI9225_NV_MEM_STS_PGM_CNT_SHIFT       (0)
#define ILI9225_NV_MEM_STS_PGM_CNT_MASK        (0x3 << ILI9225_NV_MEM_STS_PGM_CNT_SHIFT)
#define ILI9225_NV_MEM_DATA_PGM_CNT(n)         (((uint16_t)(n) << ILI9225_NV_MEM_STS_PGM_CNT_SHIFT) & ILI9225_NV_MEM_STS_PGM_CNT_MASK)

/* ILI9225_NV_MEM_PROTECTION, NV Memory Protection Key, Offset: 0x63 */

#define ILI9225_NV_MEM_PROTECTION_KEY_SHIFT    (0)
#define ILI9225_NV_MEM_PROTECTION_KEY_MASK     (0xffff << ILI9225_NV_MEM_PROTECTION_KEY_SHIFT)
#define ILI9225_NV_MEM_PROTECTION_KEY(n)       (((uint16_t)(n) << ILI9225_NV_MEM_PROTECTION_KEY_SHIFT) & ILI9225_NV_MEM_PROTECTION_KEY_MASK)

/* ILI9225_ID_CODE, ID Code, Offset: 0x65 */

#define ILI9225_ID_CODE_ID_SHIFT               (0)
#define ILI9225_ID_CODE_ID_MASK                (0xffff << ILI9225_ID_CODE_ID_SHIFT)
#define ILI9225_ID_CODE_ID(n)                  (((uint16_t)(n) << ILI9225_ID_CODE_ID_SHIFT) & ILI9225_ID_CODE_ID_MASK)

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

/****************************************************************************
 * Name:  ili9225_initialize
 *
 * Description:
 *   Initialize the LCD video driver internal structure. Also initialize the
 *   lcd hardware if not done.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD driver object
 *   for the specified LCD driver. NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *ili9225_lcdinitialize(FAR struct spi_dev_s *spi);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_ILI9225_H */

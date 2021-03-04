/****************************************************************************
 * drivers/lcd/tda19988.h
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

/* Definitions for the TDA19988.
 * The TDA19988 is a very low power and very small  size
 * High-Definition Multimedia Interface (HDMI) 1.4a transmitter
 */

#ifndef __DRIVERS_LCD_TDA19988_H
#define __DRIVERS_LCD_TDA19988_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CEC Registers ************************************************************/

/* The device has two I2C interfaces CEC (0x34) and HDMI (0x70). */

#define CEC_FRO_IM_CLK_CTRL_REG                  0xfb
#  define CEC_FRO_IM_CLK_CTRL_GHOST_DIS          (1 << 7)
#  define CEC_FRO_IM_CLK_CTRL_IMCLK_SEL          (1 << 1)

#define CEC_STATUS_REG                           0xfe
#  define CEC_STATUS_CONNECTED                   (1 << 1)

#define CEC_ENAMODS_REG                          0xff
#  define CEC_ENAMODS_HDMI                       (1 << 1)
#  define CEC_ENAMODS_RXSENS                     (1 << 2)
#  define CEC_ENABLE_ALL                         0x87

/* HDMI Memory Pages ********************************************************/

/* HDMI Memory is accessed via page and address.
 * The page must first be selected, then only the address is sent in order
 * accessing memory locations within the selected page.
 */

#define HDMI_CTRL_PAGE                           0x00  /* General control page */
#define HDMI_PLL_PAGE                            0x02  /* PLL settings page */
#define HDMI_EDID_PAGE                           0x09  /* EDID control page */
#define HDMI_INFO_PAGE                           0x10  /* Information frames and packets page */
#define HDMI_AUDIO_PAGE                          0x11  /* Audio settings and content info packets page */
#define HDMI_HDCPOTP_PAGE                        0x12  /* HDCP (TDA19988AHN and TDA19988AET only) and OTP */
#define HDMI_GAMUT_PAGE                          0x13  /* Gamut-related metadata packets page */

/* The page select register does not lie within the above pages.
 * The value of 0xff is used for this access.
 */

#define HDMI_NO_PAGE                             0xff

/* Page-related macros */

#define MKREG(page, addr)                        (((page) << 8) | (addr))
#define REGPAGE(reg)                             (((reg) >> 8) & 0xff)
#define REGADDR(reg)                             ((reg) & 0xff)

/* General Control Page Registers and Bit Definitions */

#define HDMI_CTRL_REV_LO_REG                     MKREG(HDMI_CTRL_PAGE, 0x00)

#define HDMI_CTRL_MAIN_CNTRL0_REG                MKREG(HDMI_CTRL_PAGE, 0x01)
#  define HDMI_CTRL_MAIN_CNTRL0_SR               (1 << 0)

#define HDMI_CTRL_REV_HI_REG                     MKREG(HDMI_CTRL_PAGE, 0x02)
#  define HDMI_CTRL_REV_TDA9989N2                0x0101  /* Masking out bits 4-5 */
#  define HDMI_CTRL_REV_TDA19989                 0x0201
#  define HDMI_CTRL_REV_TDA19989N2               0x0202
#  define HDMI_CTRL_REV_TDA19988                 0x0301

#define HDMI_CTRL_RESET_REG                      MKREG(HDMI_CTRL_PAGE, 0x0a)
#  define HDMI_CTRL_RESET_AUDIO                  (1 << 0)
#  define HDMI_CTRL_RESET_I2C                    (1 << 1)

#define HDMI_CTRL_DDC_CTRL_REG                   MKREG(HDMI_CTRL_PAGE, 0x0b)
#  define HDMI_CTRL_DDC_EN                       0x00

#define HDMI_CTRL_DDC_CLK_REG                    MKREG(HDMI_CTRL_PAGE, 0x0c)
#  define HDMI_CTRL_DDC_CLK_EN                   (1 << 0)

#define HDMI_CTRL_INTR_CTRL_REG                  MKREG(HDMI_CTRL_PAGE, 0x0f)
#  define HDMI_CTRL_INTR_EN_GLO                  (1 << 2)

#define HDMI_CTRL_INT_REG                        MKREG(HDMI_CTRL_PAGE, 0x11)
#  define HDMI_CTRL_INT_EDID                     (1 << 1)

#define HDMI_CTRL_VIPCTRL_0_REG                  MKREG(HDMI_CTRL_PAGE, 0x20)
#define HDMI_CTRL_VIPCTRL_1_REG                  MKREG(HDMI_CTRL_PAGE, 0x21)
#define HDMI_CTRL_VIPCTRL_2_REG                  MKREG(HDMI_CTRL_PAGE, 0x22)

#define HDMI_CTRL_VIPCTRL_3_REG                  MKREG(HDMI_CTRL_PAGE, 0x23)
#  define HDMI_CTRL_VIPCTRL_3_SYNC_HS            (2 << 4)
#  define HDMI_CTRL_VIPCTRL_3_V_TGL              (1 << 2)
#  define HDMI_CTRL_VIPCTRL_3_H_TGL              (1 << 1)

#define HDMI_CTRL_VIPCTRL_4_REG                  MKREG(HDMI_CTRL_PAGE, 0x24)
#  define HDMI_CTRL_VIPCTRL_4_BLANKIT_NDE        (0 << 2)
#  define HDMI_CTRL_VIPCTRL_4_BLANKIT_HS_VS      (1 << 2)
#  define HDMI_CTRL_VIPCTRL_4_BLANKIT_NHS_VS     (2 << 2)
#  define HDMI_CTRL_VIPCTRL_4_BLANKIT_HE_VE      (3 << 2)
#  define HDMI_CTRL_VIPCTRL_4_BLC_NONE           (0 << 0)
#  define HDMI_CTRL_VIPCTRL_4_BLC_RGB444         (1 << 0)
#  define HDMI_CTRL_VIPCTRL_4_BLC_YUV444         (2 << 0)
#  define HDMI_CTRL_VIPCTRL_4_BLC_YUV422         (3 << 0)

#define HDMI_CTRL_VIPCTRL_5_REG                  MKREG(HDMI_CTRL_PAGE, 0x25)
#  define HDMI_CTRL_VIPCTRL_5_SP_CNT(n)          (((n) & 3) << 1)

#define HDMI_CTRL_MUX_VP_VIP_OUT_REG             MKREG(HDMI_CTRL_PAGE, 0x27)

#define HDMI_CTRL_MATCTRL_REG                    MKREG(HDMI_CTRL_PAGE, 0x80)
#  define HDMI_CTRL_MAT_CONTRL_MAT_BP            (1 << 2)

#define HDMI_CTRL_MUX_VIDFORMAT_REG              MKREG(HDMI_CTRL_PAGE, 0xa0)
#define HDMI_CTRL_MUX_REFPIX_MSB_REG             MKREG(HDMI_CTRL_PAGE, 0xa1)
#define HDMI_CTRL_MUX_REFPIX_LSB_REG             MKREG(HDMI_CTRL_PAGE, 0xa2)
#define HDMI_CTRL_MUX_REFLINE_MSB_REG            MKREG(HDMI_CTRL_PAGE, 0xa3)
#define HDMI_CTRL_MUX_REFLINE_LSB_REG            MKREG(HDMI_CTRL_PAGE, 0xa4)
#define HDMI_CTRL_MUX_NPIX_MSB_REG               MKREG(HDMI_CTRL_PAGE, 0xa5)
#define HDMI_CTRL_MUX_NPIX_LSB_REG               MKREG(HDMI_CTRL_PAGE, 0xa6)
#define HDMI_CTRL_MUX_NLINE_MSB_REG              MKREG(HDMI_CTRL_PAGE, 0xa7)
#define HDMI_CTRL_MUX_NLINE_LSB_REG              MKREG(HDMI_CTRL_PAGE, 0xa8)
#define HDMI_CTRL_MUX_VS_LINE_STRT_1_MSB_REG     MKREG(HDMI_CTRL_PAGE, 0xa9)
#define HDMI_CTRL_MUX_VS_LINE_STRT_1_LSB_REG     MKREG(HDMI_CTRL_PAGE, 0xaa)
#define HDMI_CTRL_MUX_VS_PIX_STRT_1_MSB_REG      MKREG(HDMI_CTRL_PAGE, 0xab)
#define HDMI_CTRL_VS_PIX_STRT_1_LSB_REG          MKREG(HDMI_CTRL_PAGE, 0xac)
#define HDMI_CTRL_VS_LINE_END_1_MSB_REG          MKREG(HDMI_CTRL_PAGE, 0xad)
#define HDMI_CTRL_VS_LINE_END_1_LSB_REG          MKREG(HDMI_CTRL_PAGE, 0xae)
#define HDMI_CTRL_VS_PIX_END_1_MSB_REG           MKREG(HDMI_CTRL_PAGE, 0xaf)
#define HDMI_CTRL_VS_PIX_END_1_LSB_REG           MKREG(HDMI_CTRL_PAGE, 0xb0)
#define HDMI_CTRL_VS_LINE_STRT_2_MSB_REG         MKREG(HDMI_CTRL_PAGE, 0xb1)
#define HDMI_CTRL_VS_LINE_STRT_2_LSB_REG         MKREG(HDMI_CTRL_PAGE, 0xb2)
#define HDMI_CTRL_VS_PIX_STRT_2_MSB_REG          MKREG(HDMI_CTRL_PAGE, 0xb3)
#define HDMI_CTRL_VS_PIX_STRT_2_LSB_REG          MKREG(HDMI_CTRL_PAGE, 0xb4)
#define HDMI_CTRL_VS_LINE_END_2_MSB_REG          MKREG(HDMI_CTRL_PAGE, 0xb5)
#define HDMI_CTRL_VS_LINE_END_2_LSB_REG          MKREG(HDMI_CTRL_PAGE, 0xb6)
#define HDMI_CTRL_VS_PIX_END_2_MSB_REG           MKREG(HDMI_CTRL_PAGE, 0xb7)
#define HDMI_CTRL_VS_PIX_END_2_LSB_REG           MKREG(HDMI_CTRL_PAGE, 0xb8)
#define HDMI_CTRL_HS_PIX_START_MSB_REG           MKREG(HDMI_CTRL_PAGE, 0xb9)
#define HDMI_CTRL_HS_PIX_START_LSB_REG           MKREG(HDMI_CTRL_PAGE, 0xba)
#define HDMI_CTRL_HS_PIX_STOP_MSB_REG            MKREG(HDMI_CTRL_PAGE, 0xbb)
#define HDMI_CTRL_HS_PIX_STOP_LSB_REG            MKREG(HDMI_CTRL_PAGE, 0xbc)
#define HDMI_CTRL_VWIN_START_1_MSB_REG           MKREG(HDMI_CTRL_PAGE, 0xbd)
#define HDMI_CTRL_VWIN_START_1_LSB_REG           MKREG(HDMI_CTRL_PAGE, 0xbe)
#define HDMI_CTRL_VWIN_END_1_MSB_REG             MKREG(HDMI_CTRL_PAGE, 0xbf)
#define HDMI_CTRL_VWIN_END_1_LSB_REG             MKREG(HDMI_CTRL_PAGE, 0xc0)
#define HDMI_CTRL_VWIN_START_2_MSB_REG           MKREG(HDMI_CTRL_PAGE, 0xc1)
#define HDMI_CTRL_VWIN_START_2_LSB_REG           MKREG(HDMI_CTRL_PAGE, 0xc2)
#define HDMI_CTRL_VWIN_END_2_MSB_REG             MKREG(HDMI_CTRL_PAGE, 0xc3)
#define HDMI_CTRL_VWIN_END_2_LSB_REG             MKREG(HDMI_CTRL_PAGE, 0xc4)
#define HDMI_CTRL_DE_START_MSB_REG               MKREG(HDMI_CTRL_PAGE, 0xc5)
#define HDMI_CTRL_DE_START_LSB_REG               MKREG(HDMI_CTRL_PAGE, 0xc6)
#define HDMI_CTRL_DE_STOP_MSB_REG                MKREG(HDMI_CTRL_PAGE, 0xc7)
#define HDMI_CTRL_DE_STOP_LSB_REG                MKREG(HDMI_CTRL_PAGE, 0xc8)

#define HDMI_CTRL_TBG_CNTRL_0_REG                MKREG(HDMI_CTRL_PAGE, 0xca)
#  define HDMI_CTRL_TBG_CNTRL_0_SYNC_MTHD        (1 << 6)
#  define HDMI_CTRL_TBG_CNTRL_0_SYNC_ONCE        (1 << 7)

#define HDMI_CTRL_TBG_CNTRL_1_REG                MKREG(HDMI_CTRL_PAGE, 0xcb)
#  define HDMI_CTRL_TBG_CNTRL_1_H_TGL            (1 << 0)
#  define HDMI_CTRL_TBG_CNTRL_1_V_TGL            (1 << 1)
#  define HDMI_CTRL_TBG_CNTRL_1_TGL_EN           (1 << 2)
#  define HDMI_CTRL_TBG_CNTRL_1_DWIN_DIS         (1 << 6)

#define HDMI_CTRL_HVF_CNTRL_0_REG                MKREG(HDMI_CTRL_PAGE, 0xe4)
#  define HDMI_CTRL_HVF_CNTRL_0_INTPOL_BYPASS    (0 << 0)
#  define HDMI_CTRL_HVF_CNTRL_0_PREFIL_NONE      (0 << 2)

#define HDMI_CTRL_HVF_CNTRL_1_REG                MKREG(HDMI_CTRL_PAGE, 0xe5)
#  define HDMI_CTRL_HVF_CNTRL_1_VQR(x)           (((x) & 3) << 2)
#  define HDMI_CTRL_HVF_CNTRL_1_VQR_FULL         HDMI_CTRL_HVF_CNTRL_1_VQR(0)

#define HDMI_CTRL_ENABLE_SPACE_REG               MKREG(HDMI_CTRL_PAGE, 0xd6)
#define HDMI_CTRL_RPT_CNTRL_REG                  MKREG(HDMI_CTRL_PAGE, 0xf0)

/* PLL Page Registers */

#define HDMI_PLL_SERIAL_1_REG                    MKREG(HDMI_PLL_PAGE, 0x00)
#  define HDMI_PLL_SERIAL_1_SRL_MAN_IP           (1 << 6)

#define HDMI_PLL_SERIAL_2_REG                    MKREG(HDMI_PLL_PAGE, 0x01)
#  define HDMI_PLL_SERIAL_2_SRL_NOSC(x)          (((x) & 0x3) << 0)
#  define HDMI_PLL_SERIAL_2_SRL_PR(x)            (((x) & 0xf) << 4)

#define HDMI_PLL_SERIAL_3_REG                    MKREG(HDMI_PLL_PAGE, 0x02)
#  define HDMI_PLL_SERIAL_3_SRL_CCIR             (1 << 0)
#  define HDMI_PLL_SERIAL_3_SRL_DE               (1 << 2)
#  define HDMI_PLL_SERIAL_3_SRL_PXIN_SEL         (1 << 4)

#define HDMI_PLL_SERIALIZER_REG                  MKREG(HDMI_PLL_PAGE, 0x03)
#define HDMI_PLL_BUFFER_OUT_REG                  MKREG(HDMI_PLL_PAGE, 0x04)
#define HDMI_PLL_SCG1_REG                        MKREG(HDMI_PLL_PAGE, 0x05)
#define HDMI_PLL_SCG2_REG                        MKREG(HDMI_PLL_PAGE, 0x06)
#define HDMI_PLL_SCGN1_REG                       MKREG(HDMI_PLL_PAGE, 0x07)
#define HDMI_PLL_SCGN2_REG                       MKREG(HDMI_PLL_PAGE, 0x08)
#define HDMI_PLL_SCGR1_REG                       MKREG(HDMI_PLL_PAGE, 0x09)
#define HDMI_PLL_SCGR2_REG                       MKREG(HDMI_PLL_PAGE, 0x0a)

#define HDMI_PLL_SEL_CLK_REG                     MKREG(HDMI_PLL_PAGE, 0x11)
#  define HDMI_PLL_SEL_CLK_ENA_SC_CLK            (1 << 3)
#  define HDMI_PLL_SEL_CLK_SEL_VRF_CLK(x)        (((x) & 3) << 1)
#  define HDMI_PLL_SEL_CLK_SEL_CLK1              (1 << 0)

#define HDMI_PLL_ANA_GENERAL_REG                 MKREG(HDMI_PLL_PAGE, 0x12)

/* EDID Control Page Registers and Bit Definitions */

#define HDMI_EDID_DATA_REG                       MKREG(HDMI_EDID_PAGE, 0x00)

#define HDMI_EDID_REQ_REG                        MKREG(HDMI_EDID_PAGE, 0xfa)
#  define HDMI_EDID_REQ_READ                     (1 << 0)

#define HDMI_EDID_DEV_ADDR_REG                   MKREG(HDMI_EDID_PAGE, 0xfb)
#  define HDMI_EDID_DEV_ADDR                     0xa0

#define HDMI_EDID_OFFSET_REG                     MKREG(HDMI_EDID_PAGE, 0xfc)
#  define HDMI_EDID_OFFSET                       0x00

#define HDMI_EDID_SEGM_ADDR_REG                  MKREG(HDMI_EDID_PAGE, 0xfd)
#define HDMI_EDID_SEGM_ADDR                      0x00

#define HDMI_EDID_DDC_SEGM_REG                   MKREG(HDMI_EDID_PAGE, 0xfe)
#  define HDMI_EDID_SEG_ADDR                     0x00

/* HDCP (TDA19988AHN and TDA19988AET only) and OTP Page Registers and Bit
 * Definitions.
 */

#define HDMI_HDCPOTP_TX3_REG                     MKREG(HDMI_HDCPOTP_PAGE, 0x9a)

#define HDMI_HDCPOTP_TX4_REG                     MKREG(HDMI_HDCPOTP_PAGE, 0x9b)
#  define HDMI_HDCPOTP_TX4_PDRAM                 (1 << 1)

#define HDMI_HDCPOTP_TX33_REG                    MKREG(HDMI_HDCPOTP_PAGE, 0x9b)
#  define HDMI_HDCPOTP_TX33_HDMI                 (1 << 1)

/* Information Frames and Packets Page Registers and Bit Definitions */

#define HDMI_INFO_VSP                            MKREG(HDMI_INFO_PAGE, 0x20)
#define HDMI_INFO_AVI                            MKREG(HDMI_INFO_PAGE, 0x40)
#define HDMI_INFO_SPD                            MKREG(HDMI_INFO_PAGE, 0x60)
#define HDMI_INFO_AUD                            MKREG(HDMI_INFO_PAGE, 0x80)
#define HDMI_INFO_MPS                            MKREG(HDMI_INFO_PAGE, 0xa0)

/* Audio settings and content info packets page Registers and Bit
 * Definitions
 */

#define HDMI_AUDIO_ENC_CTRL_REG                  MKREG(HDMI_AUDIO_PAGE, 0x0d)
#  define HDMI_AUDIO_ENC_CNTRL_DVI_MODE          (0 << 2)
#  define HDMI_AUDIO_ENC_CNTRL_HDMI_MODE         (1 << 2)

#define HDMI_AUDIO_DIP_IF_FLAGS_REG              MKREG(HDMI_AUDIO_PAGE, 0x0f)
#  define HDMI_AUDIO_DIP_IF_FLAGS_IF1            (1 << 1)
#  define HDMI_AUDIO_DIP_IF_FLAGS_IF2            (1 << 2) /* AVI IF on page 10h */
#  define HDMI_AUDIO_DIP_IF_FLAGS_IF3            (1 << 3)
#  define HDMI_AUDIO_DIP_IF_FLAGS_IF4            (1 << 4)
#  define HDMI_AUDIO_DIP_IF_FLAGS_IF5            (1 << 5)

/* Page Select Register (no page) */

#define HDMI_PAGE_SELECT_REG                     MKREG(HDMI_NO_PAGE, 0xff)

#endif /* __DRIVERS_LCD_TDA19988_H */

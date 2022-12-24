/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone_lcd.c
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

/* Reference:
 *
 * "Understanding PinePhone's Display (MIPI DSI)"
 * https://lupyuen.github.io/articles/dsi
 *
 * "NuttX RTOS for PinePhone: Display Driver in Zig"
 * https://lupyuen.github.io/articles/dsi2
 *
 * "Rendering PinePhone's Display (DE and TCON0)"
 * https://lupyuen.github.io/articles/de
 *
 * "NuttX RTOS for PinePhone: Render Graphics in Zig"
 * https://lupyuen.github.io/articles/de2
 *
 * "A64 Page" refers to Allwinner A64 User Manual
 * https://lupyuen.github.io/images/Allwinner_A64_User_Manual_V1.1.pdf
 *
 * "ST7703 Page" refers to Sitronix ST7703 Data Sheet
 * https://lupyuen.github.io/images/ST7703_DS_v01_20160128.pdf
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/video/fb.h>
#include "chip.h"
#include "arm64_internal.h"
#include "a64_mipi_dsi.h"
#include "a64_pio.h"
#include "pinephone_lcd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Period of LCD Backlight PWM (Number of cycles of PWM Clock) */

#define BACKLIGHT_PWM_PERIOD 1199

/* LCD Panel Reset connected on PD23 */

#define LCD_RESET (PIO_OUTPUT | PIO_PULL_NONE | PIO_DRIVE_MEDLOW | \
                   PIO_INT_NONE | PIO_OUTPUT_SET | PIO_PORT_PIOD | \
                   PIO_PIN23)

/* LCD Backlight PWM connected on PL10 */

#define LCD_PWM   (PIO_PWM | PIO_PULL_NONE | PIO_DRIVE_MEDLOW | \
                   PIO_INT_NONE | PIO_OUTPUT_SET | PIO_PORT_PIOL | \
                   PIO_PIN10)

/* LCD Backlight Enable connected on PH10 */

#define LCD_BL_EN (PIO_OUTPUT | PIO_PULL_NONE | PIO_DRIVE_MEDLOW | \
                   PIO_INT_NONE | PIO_OUTPUT_SET | PIO_PORT_PIOH | \
                   PIO_PIN10)

/* A64 R_PWM Registers and Bit Definitions **********************************/

/* R_PWM Control Register (Undocumented)
 * Assume same as PWM Control Register (A64 Page 194)
 */

#define R_PWM_CTRL_REG            (A64_RPWM_ADDR + 0)
#define PWM_CH0_PRESCAL(n)        ((n) << 0)
#define PWM_CH0_EN                (1 << 4)
#define SCLK_CH0_GATING           (1 << 6)

/* R_PWM Channel 0 Period Register (Undocumented)
 * Assume same as PWM Channel 0 Period Register (A64 Page 195)
 */

#define R_PWM_CH0_PERIOD          (A64_RPWM_ADDR + 4)
#define PWM_CH0_ENTIRE_ACT_CYS(n) ((n) << 0)
#define PWM_CH0_ENTIRE_CYS(n)     ((n) << 16)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Initialization Command for Sitronix ST7703 LCD Controller ****************/

struct pinephone_cmd_s
{
  const uint8_t *cmd;  /* ST7703 Command and Parameters */
  uint8_t len;         /* Length of Command and Parameters */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Initialization Commands for Sitronix ST7703 LCD Controller ***************/

/* Command #1: SETEXTC (ST7703 Page 131)
 * Enable USER Command
 */

static const uint8_t g_pinephone_setextc[] =
{
  0xb9,  /* SETEXTC (ST7703 Page 131): Enable USER Command */
  0xf1,  /* Enable User command */
  0x12,  /* (Continued) */
  0x83   /* (Continued) */
};

/* Command #2: SETMIPI (ST7703 Page 144)
 * Set MIPI related register
 */

static const uint8_t g_pinephone_setmipi[] =
{
  0xba,  /* SETMIPI (ST7703 Page 144): Set MIPI related register */
  0x33,  /* Virtual Channel = 0 (VC_Main = 0);
          * Number of Lanes = 4 (Lane_Number = 3) */
  0x81,  /* LDO = 1.7 V (DSI_LDO_SEL = 4);
          * Terminal Resistance = 90 Ohm (RTERM = 1) */
  0x05,  /* MIPI Low High Speed driving ability = x6 (IHSRX = 5) */
  0xf9,  /* TXCLK speed in DSI LP mode = fDSICLK / 16 (Tx_clk_sel = 2) */
  0x0e,  /* Min HFP number in DSI mode = 14 (HFP_OSC = 14) */
  0x0e,  /* Min HBP number in DSI mode = 14 (HBP_OSC = 14) */
  0x20,  /* Undocumented */
  0x00,  /* Undocumented */
  0x00,  /* Undocumented */
  0x00,  /* Undocumented */
  0x00,  /* Undocumented */
  0x00,  /* Undocumented */
  0x00,  /* Undocumented */
  0x00,  /* Undocumented */
  0x44,  /* Undocumented */
  0x25,  /* Undocumented */
  0x00,  /* Undocumented */
  0x91,  /* Undocumented */
  0x0a,  /* Undocumented */
  0x00,  /* Undocumented */
  0x00,  /* Undocumented */
  0x02,  /* Undocumented */
  0x4f,  /* Undocumented */
  0x11,  /* Undocumented */
  0x00,  /* Undocumented */
  0x00,  /* Undocumented */
  0x37   /* Undocumented */
};

/* Command #3: SETPOWER_EXT (ST7703 Page 142)
 * Set display related register
 */

static const uint8_t g_pinephone_setpower_ext[] =
{
  0xb8,  /* SETPOWER_EXT: Set display related register */
  0x25,  /* External power IC or PFM: VSP = FL1002, VSN = FL1002 (PCCS = 2);
          * VCSW1 / VCSW2 Frequency for Pumping VSP / VSN = 1/4 Hsync
          * (ECP_DC_DIV = 5) */
  0x22,  /* VCSW1/VCSW2 soft start time = 15 ms (DT = 2) ; Pumping ratio of
          * VSP / VSN with VCI = x2 (XDK_ECP = 1) */
  0x20,  /* PFM operation frequency FoscD = Fosc/1 (PFM_DC_DIV = 0) */
  0x03   /* Enable power IC pumping frequency synchronization =
          * Synchronize with external Hsync (ECP_SYNC_EN = 1);
          * Enable VGH/VGL pumping frequency synchronization =
          * Synchronize with external Hsync (VGX_SYNC_EN = 1) */
};

/* Command #4: SETRGBIF (ST7703 Page 134)
 * Control RGB I/F porch timing for internal use
 */

static const uint8_t g_pinephone_setrgbif[] =
{
  0xb3,  /* SETRGBIF: Control RGB I/F porch timing for internal use */
  0x10,  /* Vertical back porch HS number in Blank Frame Period =
          * Hsync number 16 (VBP_RGB_GEN = 16) */
  0x10,  /* Vertical front porch HS number in Blank Frame Period =
          * Hsync number 16 (VFP_RGB_GEN = 16) */
  0x05,  /* HBP OSC number in Blank Frame Period = OSC number 5
          * (DE_BP_RGB_GEN = 5) */
  0x05,  /* HFP OSC number in Blank Frame Period = OSC number 5
          * (DE_FP_RGB_GEN = 5) */
  0x03,  /* Undocumented */
  0xff,  /* Undocumented */
  0x00,  /* Undocumented */
  0x00,  /* Undocumented */
  0x00,  /* Undocumented */
  0x00   /* Undocumented */
};

/* Command #5: SETSCR (ST7703 Page 147)
 * Set related setting of Source driving
 */

static const uint8_t g_pinephone_setscr[] =
{
  0xc0,  /* SETSCR: Set related setting of Source driving */
  0x73,  /* Source OP Amp driving period for positive polarity in
          * Normal Mode: Source OP Period = 115*4/Fosc (N_POPON = 115) */
  0x73,  /* Source OP Amp driving period for negative polarity in
          * Normal Mode: Source OP Period = 115*4/Fosc (N_NOPON = 115) */
  0x50,  /* Source OP Amp driving period for positive polarity in
          * Idle mode: Source OP Period   = 80*4/Fosc (I_POPON = 80) */
  0x50,  /* Source OP Amp dirivng period for negative polarity in
          * Idle Mode: Source OP Period   = 80*4/Fosc (I_NOPON = 80) */
  0x00,  /* (SCR Bits 24-31 = 0x00) */
  0xc0,  /* (SCR Bits 16-23 = 0xC0) */
  0x08,  /* Gamma bias current fine tune: Current xIbias = 4
          * (SCR Bits 9-13 = 4) ; (SCR Bits 8-15 = 0x08) */
  0x70,  /* Source and Gamma bias current core tune: Ibias = 1
          * (SCR Bits 0-3 = 0) ; Source bias current fine tune:
          * Current xIbias = 7 (SCR Bits 4-8 = 7) ;
          * (SCR Bits 0-7 = 0x70) */
  0x00   /* Undocumented */
};

/* Command #6: SETVDC (ST7703 Page 146)
 * Control NVDDD/VDDD Voltage
 */

static const uint8_t g_pinephone_setvdc[] =
{
  0xbc,  /* SETVDC: Control NVDDD/VDDD Voltage */
  0x4e   /* NVDDD voltage = -1.8 V (NVDDD_SEL = 4);
          * VDDD voltage = 1.9 V (VDDD_SEL = 6) */
};

/* Command #7: SETPANEL (ST7703 Page 154)
 * Set display related register
 */

static const uint8_t g_pinephone_setpanel[] =
{
  0xcc,  /* SETPANEL: Set display related register */
  0x0b   /* Enable reverse the source scan direction (SS_PANEL = 1);
          * Normal vertical scan direction (GS_PANEL = 0);
          * Normally black panel (REV_PANEL = 1);
          * S1:S2:S3 = B:G:R (BGR_PANEL = 1) */
};

/* Command #8: SETCYC (ST7703 Page 135)
 * Control display inversion type
 */

static const uint8_t g_pinephone_setcyc[] =
{
  0xb4,  /* SETCYC: Control display inversion type */
  0x80   /* Extra source for Zig-Zag Inversion = S2401 (ZINV_S2401_EN = 1);
          * Row source data dislocates = Even row (ZINV_G_EVEN_EN = 0);
          * Disable Zig-Zag Inversion (ZINV_EN = 0);
          * Enable Zig-Zag1 Inversion (ZINV2_EN = 0);
          * Normal mode inversion type = Column inversion (N_NW = 0) */
};

/* Command #9: SETDISP (ST7703 Page 132)
 * Control the display resolution
 */

static const uint8_t g_pinephone_setdisp[] =
{
  0xb2,  /* SETDISP: Control the display resolution */
  0xf0,  /* Gate number of vertical direction = 480 + (240*4) (NL = 240) */
  0x12,  /* (RES_V_LSB = 0);
          * Non-display area source output control:
          *   Source output = VSSD (BLK_CON = 1);
          * Channel number of source direction = 720RGB (RESO_SEL = 2) */
  0xf0   /* Source voltage during Blanking Time when accessing Sleep-Out /
          * Sleep-In command = GND (WHITE_GND_EN = 1);
          * Blank timing control when access sleep out command:
          *   Blank Frame Period = 7 Frames (WHITE_FRAME_SEL = 7);
          * Source output refresh control:
          *   Refresh Period = 0 Frames (ISC = 0) */
};

/* Command #10: SETEQ (ST7703 Page 159)
 * Set EQ related register
 */

static const uint8_t g_pinephone_seteq[] =
{
  0xe3,  /* SETEQ: Set EQ related register */
  0x00,  /* Temporal spacing between HSYNC and PEQGND = 0*4/Fosc
          *   (PNOEQ = 0) */
  0x00,  /* Temporal spacing between HSYNC and NEQGND = 0*4/Fosc
          *   (NNOEQ = 0) */
  0x0b,  /* Source EQ GND period when Source up to positive voltage =
          *   11*4/Fosc (PEQGND = 11) */
  0x0b,  /* Source EQ GND period when Source down to negative voltage =
          *   11*4/Fosc (NEQGND = 11) */
  0x10,  /* Source EQ VCI period when Source up to positive voltage =
          *   16*4/Fosc (PEQVCI = 16) */
  0x10,  /* Source EQ VCI period when Source down to negative voltage =
          *   16*4/Fosc (NEQVCI = 16) */
  0x00,  /* Temporal period of PEQVCI1 = 0*4/Fosc (PEQVCI1 = 0) */
  0x00,  /* Temporal period of NEQVCI1 = 0*4/Fosc (NEQVCI1 = 0) */
  0x00,  /* (Reserved) */
  0x00,  /* (Reserved) */
  0xff,  /* (Undocumented) */
  0x00,  /* (Reserved) */
  0xc0,  /* White pattern to protect GOA glass (ESD_DET_DATA_WHITE = 1);
          * Enable ESD detection function to protect GOA glass
          * (ESD_WHITE_EN = 1) */
  0x10   /* No Need VSYNC (additional frame) after Sleep-In command
          * to display sleep-in blanking frame then into Sleep-In State
          *   (SLPIN_OPTION = 1);
          * Enable video function detection (VEDIO_NO_CHECK_EN = 0);
          * Disable ESD white pattern scanning voltage pull ground
          *   (ESD_WHITE_GND_EN = 0);
          * ESD detection function period = 0 Frames
          *   (ESD_DET_TIME_SEL = 0) */
};

/* Command #11: Undocumented */

static const uint8_t g_pinephone_c6[] =
{
  0xc6,  /* Undocumented */
  0x01,  /* Undocumented */
  0x00,  /* Undocumented */
  0xff,  /* Undocumented */
  0xff,  /* Undocumented */
  0x00   /* Undocumented */
};

/* Command #12: SETPOWER (ST7703 Page 149)
 * Set related setting of power
 */

static const uint8_t g_pinephone_setpower[] =
{
  0xc1,  /* SETPOWER: Set related setting of power */
  0x74,  /* VGH Voltage Adjustment = 17 V (VBTHS = 7);
          * VGL Voltage Adjustment = -11 V (VBTLS = 4) */
  0x00,  /* Enable VGH feedback voltage detection.
          *   Output voltage = VBTHS (FBOFF_VGH = 0);
          * Enable VGL feedback voltage detection.
          *   Output voltage = VBTLS (FBOFF_VGL = 0) */
  0x32,  /* VSPROUT Voltage = (VRH[5:0] x 0.05 + 3.3) x (VREF/4.8)
          *   if VREF [4]=0 (VRP = 50) */
  0x32,  /* VSNROUT Voltage = (VRH[5:0] x 0.05 + 3.3) x (VREF/5.6)
          *   if VREF [4]=1 (VRN = 50) */
  0x77,  /* Undocumented */
  0xf1,  /* Enable VGL voltage Detect Function =
          *   VGL voltage Abnormal (VGL_DET_EN = 1);
          * Enable VGH voltage Detect Function =
          *   VGH voltage Abnormal (VGH_DET_EN = 1);
          * Enlarge VGL Voltage at "FBOFF_VGL=1" =
          *   "VGL=-15V" (VGL_TURBO = 1);
          * Enlarge VGH Voltage at "FBOFF_VGH=1" =
          *   "VGH=20V" (VGH_TURBO = 1); (APS = 1) */
  0xff,  /* Left side VGH stage 1 pumping frequency =
          *   1.5 MHz (VGH1_L_DIV = 15);
          * Left side VGL stage 1 pumping frequency =
          *   1.5 MHz (VGL1_L_DIV = 15) */
  0xff,  /* Right side VGH stage 1 pumping frequency =
          *   1.5 MHz (VGH1_R_DIV = 15);
          * Right side VGL stage 1 pumping frequency =
          *   1.5 MHz (VGL1_R_DIV = 15) */
  0xcc,  /* Left side VGH stage 2 pumping frequency =
          *   2.6 MHz (VGH2_L_DIV = 12);
          * Left side VGL stage 2 pumping frequency =
          *   2.6 MHz (VGL2_L_DIV = 12) */
  0xcc,  /* Right side VGH stage 2 pumping frequency =
          *   2.6 MHz (VGH2_R_DIV = 12);
          * Right side VGL stage 2 pumping frequency =
          *   2.6 MHz (VGL2_R_DIV = 12) */
  0x77,  /* Left side VGH stage 3 pumping frequency =
          *   4.5 MHz (VGH3_L_DIV = 7);
          * Left side VGL stage 3 pumping frequency =
          *   4.5 MHz (VGL3_L_DIV = 7) */
  0x77   /* Right side VGH stage 3 pumping frequency =
          *   4.5 MHz (VGH3_R_DIV = 7);
          * Right side VGL stage 3 pumping frequency =
          *   4.5 MHz (VGL3_R_DIV = 7) */
};

/* Command #13: SETBGP (ST7703 Page 136)
 * Internal reference voltage setting
 */

static const uint8_t g_pinephone_setbgp[] =
{
  0xb5,  /* SETBGP: Internal reference voltage setting */
  0x07,  /* VREF Voltage: 4.2 V (VREF_SEL = 7) */
  0x07   /* NVREF Voltage: 4.2 V (NVREF_SEL = 7) */
};

/* Command #14: SETVCOM (ST7703 Page 137)
 * Set VCOM Voltage
 */

static const uint8_t g_pinephone_setvcom[] =
{
  0xb6,  /* SETVCOM: Set VCOM Voltage */
  0x2c,  /* VCOMDC voltage at "GS_PANEL=0" = -0.67 V (VCOMDC_F = 0x2C) */
  0x2c   /* VCOMDC voltage at "GS_PANEL=1" = -0.67 V (VCOMDC_B = 0x2C) */
};

/* Command #15: Undocumented */

static const uint8_t g_pinephone_bf[] =
{
  0xbf,  /* Undocumented */
  0x02,  /* Undocumented */
  0x11,  /* Undocumented */
  0x00   /* Undocumented */
};

/* Command #16: SETGIP1 (ST7703 Page 163)
 * Set forward GIP timing
 */

static const uint8_t g_pinephone_setgip1[] =
{
  0xe9,  /* SETGIP1: Set forward GIP timing */
  0x82,  /* SHR0, SHR1, CHR, CHR2 refer to Internal DE (REF_EN = 1);
          *   (PANEL_SEL = 2) */
  0x10,  /* Starting position of GIP STV group 0 = 4102 HSYNC
          *   (SHR0 Bits 8-12 = 0x10) */
  0x06,  /*   (SHR0 Bits 0-7 = 0x06) */
  0x05,  /* Starting position of GIP STV group 1 = 1442 HSYNC
          *   (SHR1 Bits 8-12 = 0x05) */
  0xa2,  /* (SHR1 Bits 0-7 = 0xA2) */
  0x0a,  /* Distance of STV rising edge and HYSNC = 10*2 Fosc
          *   (SPON  Bits 0-7 = 0x0A) */
  0xa5,  /* Distance of STV falling edge and HYSNC = 165*2 Fosc
          *   (SPOFF Bits 0-7 = 0xA5) */
  0x12,  /* STV0_1 distance with STV0_0 = 1 HSYNC (SHR0_1 = 1);
          * STV0_2 distance with STV0_0 = 2 HSYNC (SHR0_2 = 2) */
  0x31,  /* STV0_3 distance with STV0_0 = 3 HSYNC (SHR0_3 = 3);
          * STV1_1 distance with STV1_0 = 1 HSYNC (SHR1_1 = 1) */
  0x23,  /* STV1_2 distance with STV1_0 = 2 HSYNC (SHR1_2 = 2);
          * STV1_3 distance with STV1_0 = 3 HSYNC (SHR1_3 = 3) */
  0x37,  /* STV signal high pulse width = 3 HSYNC (SHP = 3);
          * Total number of STV signal = 7 (SCP = 7) */
  0x83,  /* Starting position of GIP CKV group 0 (CKV0_0) = 131 HSYNC
          *   (CHR = 0x83) */
  0x04,  /* Distance of CKV rising edge and HYSNC  = 4*2   Fosc
          *   (CON  Bits 0-7 = 0x04) */
  0xbc,  /* Distance of CKV falling edge and HYSNC = 188*2 Fosc
          *   (COFF Bits 0-7 = 0xBC) */
  0x27,  /* CKV signal high pulse width = 2 HSYNC (CHP = 2);
          * Total period cycle of CKV signal = 7 HSYNC (CCP = 7) */
  0x38,  /* Extra gate counter at blanking area: Gate number = 56
          *   (USER_GIP_GATE = 0x38) */
  0x0c,  /* Left side GIP output pad signal = ? (CGTS_L Bits 16-21 = 0x0C) */
  0x00,  /* (CGTS_L Bits  8-15 = 0x00) */
  0x03,  /* (CGTS_L Bits  0-7  = 0x03) */
  0x00,  /* Normal polarity of Left side GIP output pad signal
          *   (CGTS_INV_L Bits 16-21 = 0x00) */
  0x00,  /* (CGTS_INV_L Bits  8-15 = 0x00) */
  0x00,  /* (CGTS_INV_L Bits  0-7  = 0x00) */
  0x0c,  /* Right side GIP output pad signal = ?
          *   (CGTS_R Bits 16-21 = 0x0C) */
  0x00,  /* (CGTS_R Bits  8-15 = 0x00) */
  0x03,  /* (CGTS_R Bits  0-7  = 0x03) */
  0x00,  /* Normal polarity of Right side GIP output pad signal
          *   (CGTS_INV_R Bits 16-21 = 0x00) */
  0x00,  /* (CGTS_INV_R Bits  8-15 = 0x00) */
  0x00,  /* (CGTS_INV_R Bits  0-7  = 0x00) */
  0x75,  /* Left side GIP output pad signal = ? (COS1_L = 7);
          * Left side GIP output pad signal = ? (COS2_L = 5) */
  0x75,  /* Left side GIP output pad signal = ? (COS3_L = 7); (COS4_L = 5) */
  0x31,  /* Left side GIP output pad signal = ? (COS5_L = 3); (COS6_L = 1) */
  0x88,  /* Reserved (Parameter 32) */
  0x88,  /* Reserved (Parameter 33) */
  0x88,  /* Reserved (Parameter 34) */
  0x88,  /* Reserved (Parameter 35) */
  0x88,  /* Reserved (Parameter 36) */
  0x88,  /* Left side GIP output pad signal  = ? (COS17_L = 8);
          * Left side GIP output pad signal  = ? (COS18_L = 8) */
  0x13,  /* Left side GIP output pad signal  = ? (COS19_L = 1);
          * Left side GIP output pad signal  = ? (COS20_L = 3) */
  0x88,  /* Left side GIP output pad signal  = ? (COS21_L = 8);
          * Left side GIP output pad signal  = ? (COS22_L = 8) */
  0x64,  /* Right side GIP output pad signal = ? (COS1_R  = 6);
          * Right side GIP output pad signal = ? (COS2_R  = 4) */
  0x64,  /* Right side GIP output pad signal = ? (COS3_R  = 6);
          * Right side GIP output pad signal = ? (COS4_R  = 4) */
  0x20,  /* Right side GIP output pad signal = ? (COS5_R  = 2);
          * Right side GIP output pad signal = ? (COS6_R  = 0) */
  0x88,  /* Reserved (Parameter 43) */
  0x88,  /* Reserved (Parameter 44) */
  0x88,  /* Reserved (Parameter 45) */
  0x88,  /* Reserved (Parameter 46) */
  0x88,  /* Reserved (Parameter 47) */
  0x88,  /* Right side GIP output pad signal = ? (COS17_R = 8);
          * Right side GIP output pad signal = ? (COS18_R = 8) */
  0x02,  /* Right side GIP output pad signal = ? (COS19_R = 0);
          * Right side GIP output pad signal = ? (COS20_R = 2) */
  0x88,  /* Right side GIP output pad signal = ? (COS21_R = 8);
          * Right side GIP output pad signal = ? (COS22_R = 8) */
  0x00,  /* (TCON_OPT = 0x00) */
  0x00,  /* (GIP_OPT Bits 16-22 = 0x00) */
  0x00,  /* (GIP_OPT Bits  8-15 = 0x00) */
  0x00,  /* (GIP_OPT Bits  0-7  = 0x00) */
  0x00,  /* Starting position of GIP CKV group 1 (CKV1_0) = 0 HSYNC
          *   (CHR2 = 0x00) */
  0x00,  /* Distance of CKV1 rising edge and HYSNC  = 0*2 Fosc
          *   (CON2  Bits 0-7 = 0x00) */
  0x00,  /* Distance of CKV1 falling edge and HYSNC = 0*2 Fosc
          *   (COFF2 Bits 0-7 = 0x00) */
  0x00,  /* CKV1 signal high pulse width = 0 HSYNC (CHP2 = 0);
          *   Total period cycle of CKV1 signal = 0 HSYNC (CCP2 = 0) */
  0x00,  /* (CKS Bits 16-21 = 0x00) */
  0x00,  /* (CKS Bits  8-15 = 0x00) */
  0x00,  /* (CKS Bits  0-7  = 0x00) */
  0x00,  /* (COFF  Bits 8-9 = 0); (CON  Bits 8-9 = 0);
          * (SPOFF Bits 8-9 = 0); (SPON Bits 8-9 = 0) */
  0x00   /* (COFF2 Bits 8-9 = 0); (CON2 Bits 8-9 = 0) */
};

/* Command #17: SETGIP2 (ST7703 Page 170)
 * Set backward GIP timing
 */

static const uint8_t g_pinephone_setgip2[] =
{
  0xea,  /* SETGIP2: Set backward GIP timing */
  0x02,  /* YS2 Signal Mode = INYS1/INYS2 (YS2_SEL = 0);
          * YS2 Signal Mode = INYS1/INYS2 (YS1_SEL = 0);
          * Don't reverse YS2 signal (YS2_XOR = 0);
          * Don't reverse YS1 signal (YS1_XOR = 0);
          * Enable YS signal function (YS_FLAG_EN = 1);
          * Disable ALL ON function (ALL_ON_EN = 0) */
  0x21,  /* (GATE = 0x21) */
  0x00,  /* (CK_ALL_ON_EN = 0); (STV_ALL_ON_EN = 0);
          * Timing of YS1 and YS2 signal = ? (CK_ALL_ON_WIDTH1 = 0) */
  0x00,  /* Timing of YS1 and YS2 signal = ? (CK_ALL_ON_WIDTH2 = 0) */
  0x00,  /* Timing of YS1 and YS2 signal = ? (CK_ALL_ON_WIDTH3 = 0) */
  0x00,  /* (YS_FLAG_PERIOD = 0) */
  0x00,  /* (YS2_SEL_2 = 0); (YS1_SEL_2 = 0); (YS2_XOR_2 = 0);
          * (YS_FLAG_EN_2 = 0); (ALL_ON_EN_2 = 0) */
  0x00,  /* Distance of GIP ALL On rising edge and DE = ?
          *   (USER_GIP_GATE1_2 = 0) */
  0x00,  /* (CK_ALL_ON_EN_2 = 0); (STV_ALL_ON_EN_2 = 0);
          * (CK_ALL_ON_WIDTH1_2 = 0) */
  0x00,  /* (CK_ALL_ON_WIDTH2_2 = 0) */
  0x00,  /* (CK_ALL_ON_WIDTH3_2 = 0) */
  0x00,  /* (YS_FLAG_PERIOD_2 = 0) */
  0x02,  /* (COS1_L_GS = 0); (COS2_L_GS = 2) */
  0x46,  /* (COS3_L_GS = 4); (COS4_L_GS = 6) */
  0x02,  /* (COS5_L_GS = 0); (COS6_L_GS = 2) */
  0x88,  /* Reserved (Parameter 16) */
  0x88,  /* Reserved (Parameter 17) */
  0x88,  /* Reserved (Parameter 18) */
  0x88,  /* Reserved (Parameter 19) */
  0x88,  /* Reserved (Parameter 20) */
  0x88,  /* (COS17_L_GS = 8); (COS18_L_GS = 8) */
  0x64,  /* (COS19_L_GS = 6); (COS20_L_GS = 4) */
  0x88,  /* (COS21_L_GS = 8); (COS22_L_GS = 8) */
  0x13,  /* (COS1_R_GS  = 1); (COS2_R_GS  = 3) */
  0x57,  /* (COS3_R_GS  = 5); (COS4_R_GS  = 7) */
  0x13,  /* (COS5_R_GS  = 1); (COS6_R_GS  = 3) */
  0x88,  /* Reserved (Parameter 27) */
  0x88,  /* Reserved (Parameter 28) */
  0x88,  /* Reserved (Parameter 29) */
  0x88,  /* Reserved (Parameter 30) */
  0x88,  /* Reserved (Parameter 31) */
  0x88,  /* (COS17_R_GS = 8); (COS18_R_GS = 8) */
  0x75,  /* (COS19_R_GS = 7); (COS20_R_GS = 5) */
  0x88,  /* (COS21_R_GS = 8); (COS22_R_GS = 8) */
  0x23,  /* GIP output EQ signal: P_EQ = Yes, N_EQ = No (EQOPT = 2);
          * GIP output EQ signal level: P_EQ = GND, N_EQ = GND
          *   (EQ_SEL = 3) */
  0x14,  /* Distance of EQ rising edge and HYSNC = 20 Fosc
          *   (EQ_DELAY = 0x14) */
  0x00,  /* Distance of EQ rising edge and HYSNC = 0 HSYNC
          *   (EQ_DELAY_HSYNC = 0) */
  0x00,  /* (HSYNC_TO_CL1_CNT10 Bits 8-9 = 0) */
  0x02,  /* GIP reference HSYNC between external HSYNC = 2 Fosc
          * (HSYNC_TO_CL1_CNT10 Bits 0-7 = 2) */
  0x00,  /* Undocumented (Parameter 40) */
  0x00,  /* Undocumented (Parameter 41) */
  0x00,  /* Undocumented (Parameter 42) */
  0x00,  /* Undocumented (Parameter 43) */
  0x00,  /* Undocumented (Parameter 44) */
  0x00,  /* Undocumented (Parameter 45) */
  0x00,  /* Undocumented (Parameter 46) */
  0x00,  /* Undocumented (Parameter 47) */
  0x00,  /* Undocumented (Parameter 48) */
  0x00,  /* Undocumented (Parameter 49) */
  0x00,  /* Undocumented (Parameter 50) */
  0x00,  /* Undocumented (Parameter 51) */
  0x00,  /* Undocumented (Parameter 52) */
  0x00,  /* Undocumented (Parameter 53) */
  0x00,  /* Undocumented (Parameter 54) */
  0x03,  /* Undocumented (Parameter 55) */
  0x0a,  /* Undocumented (Parameter 56) */
  0xa5,  /* Undocumented (Parameter 57) */
  0x00,  /* Undocumented (Parameter 58) */
  0x00,  /* Undocumented (Parameter 59) */
  0x00,  /* Undocumented (Parameter 60) */
  0x00   /* Undocumented (Parameter 61) */
};

/* Command #18: SETGAMMA (ST7703 Page 158)
 * Set the gray scale voltage to adjust the gamma characteristics of the
 * TFT panel
 */

static const uint8_t g_pinephone_setgamma[] =
{
  0xe0,  /* SETGAMMA: Set the gray scale voltage to adjust the gamma */
  0x00,  /* (PVR0 = 0x00) */
  0x09,  /* (PVR1 = 0x09) */
  0x0d,  /* (PVR2 = 0x0D) */
  0x23,  /* (PVR3 = 0x23) */
  0x27,  /* (PVR4 = 0x27) */
  0x3c,  /* (PVR5 = 0x3C) */
  0x41,  /* (PPR0 = 0x41) */
  0x35,  /* (PPR1 = 0x35) */
  0x07,  /* (PPK0 = 0x07) */
  0x0d,  /* (PPK1 = 0x0D) */
  0x0e,  /* (PPK2 = 0x0E) */
  0x12,  /* (PPK3 = 0x12) */
  0x13,  /* (PPK4 = 0x13) */
  0x10,  /* (PPK5 = 0x10) */
  0x12,  /* (PPK6 = 0x12) */
  0x12,  /* (PPK7 = 0x12) */
  0x18,  /* (PPK8 = 0x18) */
  0x00,  /* (NVR0 = 0x00) */
  0x09,  /* (NVR1 = 0x09) */
  0x0d,  /* (NVR2 = 0x0D) */
  0x23,  /* (NVR3 = 0x23) */
  0x27,  /* (NVR4 = 0x27) */
  0x3c,  /* (NVR5 = 0x3C) */
  0x41,  /* (NPR0 = 0x41) */
  0x35,  /* (NPR1 = 0x35) */
  0x07,  /* (NPK0 = 0x07) */
  0x0d,  /* (NPK1 = 0x0D) */
  0x0e,  /* (NPK2 = 0x0E) */
  0x12,  /* (NPK3 = 0x12) */
  0x13,  /* (NPK4 = 0x13) */
  0x10,  /* (NPK5 = 0x10) */
  0x12,  /* (NPK6 = 0x12) */
  0x12,  /* (NPK7 = 0x12) */
  0x18   /* (NPK8 = 0x18) */
};

/* Command #19: SLPOUT (ST7703 Page 89)
 * Turn off sleep mode (MIPI_DCS_EXIT_SLEEP_MODE)
 */

static const uint8_t g_pinephone_slpout[] =
{
  0x11  /* SLPOUT: Turn off sleep mode */
};

/* Wait 120 milliseconds */

/* Command #20: Display On (ST7703 Page 97)
 * Recover from DISPLAY OFF mode (MIPI_DCS_SET_DISPLAY_ON)
 */

static const uint8_t g_pinephone_displayon[] =
{
  0x29  /* Display On: Recover from DISPLAY OFF mode */
};

/* Consolidate Initialization Commands **************************************/

/* 20 Initialization Commands to be sent to ST7703 LCD Controller */

static const struct pinephone_cmd_s g_pinephone_commands[] =
{
  {
    g_pinephone_setextc,   /* Command #1: SETEXTC (ST7703 Page 131) */
    sizeof(g_pinephone_setextc)
  },
  {
    g_pinephone_setmipi,   /* Command #2: SETMIPI (ST7703 Page 144) */
    sizeof(g_pinephone_setmipi)
  },
  {
    g_pinephone_setpower_ext, /* Command #3: SETPOWER_EXT (ST7703 Pg 142) */
    sizeof(g_pinephone_setpower_ext)
  },
  {
    g_pinephone_setrgbif,  /* Command #4: SETRGBIF (ST7703 Page 134) */
    sizeof(g_pinephone_setrgbif)
  },
  {
    g_pinephone_setscr,    /* Command #5: SETSCR (ST7703 Page 147) */
    sizeof(g_pinephone_setscr)
  },
  {
    g_pinephone_setvdc,    /* Command #6: SETVDC (ST7703 Page 146) */
    sizeof(g_pinephone_setvdc)
  },
  {
    g_pinephone_setpanel,  /* Command #7: SETPANEL (ST7703 Page 154) */
    sizeof(g_pinephone_setpanel)
  },
  {
    g_pinephone_setcyc,    /* Command #8: SETCYC (ST7703 Page 135) */
    sizeof(g_pinephone_setcyc)
  },
  {
    g_pinephone_setdisp,   /* Command #9: SETDISP (ST7703 Page 132) */
    sizeof(g_pinephone_setdisp)
  },
  {
    g_pinephone_seteq,     /* Command #10: SETEQ (ST7703 Page 159) */
    sizeof(g_pinephone_seteq)
  },
  {
    g_pinephone_c6,        /* Command #11: Undocumented */
    sizeof(g_pinephone_c6)
  },
  {
    g_pinephone_setpower,  /* Command #12: SETPOWER (ST7703 Page 149) */
    sizeof(g_pinephone_setpower)
  },
  {
    g_pinephone_setbgp,    /* Command #13: SETBGP (ST7703 Page 136) */
    sizeof(g_pinephone_setbgp)
  },
  {
    g_pinephone_setvcom,   /* Command #14: SETVCOM (ST7703 Page 137) */
    sizeof(g_pinephone_setvcom)
  },
  {
    g_pinephone_bf,        /* Command #15: Undocumented */
    sizeof(g_pinephone_bf)
  },
  {
    g_pinephone_setgip1,   /* Command #16: SETGIP1 (ST7703 Page 163) */
    sizeof(g_pinephone_setgip1)
  },
  {
    g_pinephone_setgip2,   /* Command #17: SETGIP2 (ST7703 Page 170) */
    sizeof(g_pinephone_setgip2)
  },
  {
    g_pinephone_setgamma,  /* Command #18: SETGAMMA (ST7703 Page 158) */
    sizeof(g_pinephone_setgamma)
  },
  {
    g_pinephone_slpout,    /* Command #19: SLPOUT (ST7703 Page 89) */
    sizeof(g_pinephone_slpout)
  },
  {
    NULL,                  /* Wait 120 milliseconds */
    0
  },
  {
    g_pinephone_displayon, /* Command #20: Display On (ST7703 Page 97) */
    sizeof(g_pinephone_displayon)
  }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: write_dcs
 *
 * Description:
 *   Write the DCS Command to MIPI DSI Bus.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int write_dcs(const uint8_t *buf, size_t len)
{
  int ret = -1;

  ginfo("len=%ld\n", len);
  ginfodumpbuffer("buf", buf, len);
  DEBUGASSERT(len > 0);

  /* Do DCS Short Write or Long Write depending on command length */

  switch (len)
    {
      /* DCS Short Write (without parameter) */

      case 1:
        ret = a64_mipi_dsi_write(A64_MIPI_DSI_VIRTUAL_CHANNEL,
          MIPI_DSI_DCS_SHORT_WRITE,
          buf, len);
        break;

      /* DCS Short Write (with parameter) */

      case 2:
        ret = a64_mipi_dsi_write(A64_MIPI_DSI_VIRTUAL_CHANNEL,
          MIPI_DSI_DCS_SHORT_WRITE_PARAM,
          buf, len);
        break;

      /* DCS Long Write */

      default:
        ret = a64_mipi_dsi_write(A64_MIPI_DSI_VIRTUAL_CHANNEL,
          MIPI_DSI_DCS_LONG_WRITE,
          buf, len);
        break;
    }

  if (ret < 0)
    {
      gerr("MIPI DSI Write failed: %d\n", ret);
      return ret;
    }

  DEBUGASSERT(ret == len);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinephone_lcd_backlight_enable
 *
 * Description:
 *   Turn on the Backlight in Xingbangda XBD599 LCD Panel.
 *
 * Input Parameters:
 *   percent - Percent Brightness
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int pinephone_lcd_backlight_enable(uint32_t percent)
{
  int ret;
  uint32_t period;
  uint32_t ctrl;

  /* Configure PL10 for PWM */

  ginfo("Configure PL10 for PWM\n");
  ret = a64_pio_config(LCD_PWM);
  if (ret < 0)
    {
      gerr("Configure PL10 failed: %d\n", ret);
      return ret;
    }

  /* R_PWM Control Register (Undocumented)
   * Assume same as PWM Control Register (A64 Page 194)
   * Set SCLK_CH0_GATING (Bit 6) to 0 (Mask)
   */

  ginfo("Disable R_PWM\n");
  modreg32(0, SCLK_CH0_GATING, R_PWM_CTRL_REG);

  /* R_PWM Channel 0 Period Register (Undocumented)
   * Assume same as PWM Channel 0 Period Register (A64 Page 195)
   * Set PWM_CH0_ENTIRE_CYS (Bits 16 to 31) to PWM Period
   * Set PWM_CH0_ENTIRE_ACT_CYS (Bits 0 to 15) to PWM Period * Percent / 100
   */

  ginfo("Configure R_PWM Period\n");
  period = PWM_CH0_ENTIRE_CYS(BACKLIGHT_PWM_PERIOD) |
           PWM_CH0_ENTIRE_ACT_CYS(BACKLIGHT_PWM_PERIOD * percent / 100);
  putreg32(period, R_PWM_CH0_PERIOD);

  /* R_PWM Control Register (Undocumented)
   * Assume same as PWM Control Register (A64 Page 194)
   * Set SCLK_CH0_GATING (Bit 6) to 1 (Pass)
   * Set PWM_CH0_EN (Bit 4) to 1 (Enable)
   * Set PWM_CH0_PRESCAL (Bits 0 to 3) to 0b1111 (Prescaler 1)
   */

  ginfo("Enable R_PWM\n");
  ctrl = SCLK_CH0_GATING | PWM_CH0_EN | PWM_CH0_PRESCAL(0b1111);
  putreg32(ctrl, R_PWM_CTRL_REG);

  /* Configure PH10 for Output */

  ginfo("Configure PH10 for Output\n");
  ret = a64_pio_config(LCD_BL_EN);
  if (ret < 0)
    {
      gerr("Configure PH10 failed: %d\n", ret);
      return ret;
    }

  /* Set PH10 to High */

  ginfo("Set PH10 to High\n");
  a64_pio_write(LCD_BL_EN, true);

  return OK;
}

/****************************************************************************
 * Name: pinephone_lcd_panel_reset
 *
 * Description:
 *   Reset the Xingbangda XBD599 LCD Panel.
 *
 * Input Parameters:
 *   val - True if Reset should be set to High; False if Reset should be
 *         set to Low
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int pinephone_lcd_panel_reset(bool val)
{
  int ret;

  /* Reset LCD Panel at PD23 (Active Low), configure PD23 for Output */

  ginfo("Configure PD23 for Output\n");
  ret = a64_pio_config(LCD_RESET);
  if (ret < 0)
    {
      gerr("Configure PD23 failed: %d\n", ret);
      return ret;
    }

  /* Set PD23 to High or Low */

  ginfo("Set PD23 to %d\n", val);
  a64_pio_write(LCD_RESET, val);

  return OK;
}

/****************************************************************************
 * Name: pinephone_lcd_panel_init
 *
 * Description:
 *   Initialize the Sitronix ST7703 LCD Controller in Xingbangda XBD599
 *   LCD Panel.  Send 20 Initialization Commands to ST7703 over MIPI DSI.
 *   Assumes that the LCD Panel has been powered on (via AXP803 PMIC),
 *   MIPI DSI and D-PHY have been enabled on the SoC, and LCD Panel has
 *   been Reset (to Low then High).  After the LCD Panel has been
 *   initialized, we may start MIPI DSI (in HSC and HSD modes) and
 *   Display Engine.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int pinephone_lcd_panel_init(void)
{
  int i;
  int ret;
  const int cmd_len = sizeof(g_pinephone_commands) /
                      sizeof(g_pinephone_commands[0]);

  /* For every ST7703 Initialization Command */

  ginfo("Init ST7703 LCD Controller\n");
  for (i = 0; i < cmd_len; i++)
    {
      /* Get the ST7703 command and length */

      const uint8_t *cmd = g_pinephone_commands[i].cmd;
      const uint8_t len = g_pinephone_commands[i].len;

      /* If command is null, wait 120 milliseconds */

      if (cmd == NULL)
        {
          up_mdelay(120);
          continue;
        }

      /* Send the command to ST7703 over MIPI DSI */

      ret = write_dcs(cmd, len);
      if (ret < 0)
        {
          gerr("Write DCS failed: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

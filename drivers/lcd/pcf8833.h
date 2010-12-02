/**************************************************************************************
 * drivers/lcd/pcf8833.h
 * Definitions for the Phillips PCF8833 LCD controller
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * References: "Data Sheet, PCF8833 STN RGB 132x132x3 driver," Phillips, 2003 Feb 14.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************************/

#ifndef __DRIVERS_LCD_PCF8833_H
#define __DRIVERS_LCD_PCF8833_H

/**************************************************************************************
 * Included Files
 **************************************************************************************/

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* LCD Commands */

#define PCF8833_NOP       0x00 /* No operation; Data: none */
#define PCF8833_SWRESET   0x01 /* Software reset ; Data: none */
#define PCF8833_BSTROFF   0x02 /* Booster voltage off; Data: none */
#define PCF8833_BSTRON    0x03 /* Booster voltage on; Data: none */
#define PCF8833_RDDIDIF   0x04 /* Read display identification; Data: none */
#define PCF8833_RDDST     0x09 /* Read display status; Data: none */
#define PCF8833_SLEEPIN   0x10 /* Sleep_IN; Data: none */
#define PCF8833_SLEEPOUT  0x11 /* Sleep_OUT; Data: none */
#define PCF8833_PTLON     0x12 /* Partial mode on; Data: none */
#define PCF8833_NORON     0x13 /* Normal Display mode on; Data: none */
#define PCF8833_INVOFF    0x20 /* Display inversion off; Data: none */
#define PCF8833_INVON     0x21 /* Display inversion on; Data: none */
#define PCF8833_DALO      0x22 /* All pixel off; Data: none */
#define PCF8833_DAL       0x23 /* All pixel on; Data: none */
#define PCF8833_SETCON    0x25 /* Set contrast; Data: (1) contrast */
#define PCF8833_DISPOFF   0x28 /* Display off; Data: none */
#define PCF8833_DISPON    0x29 /* Display on; Data: none */
#define PCF8833_CASET     0x2a /* Column address set; Data: (1) X start (2) X end  */
#define PCF8833_PASET     0x2b /* Page address set Data: (1) Y start (2) Y end */
#define PCF8833_RAMWR     0x2c /* Memory write; Data: (1) write data */
#define PCF8833_RGBSET    0x2d /* Colour set; Data: (1-8) red tones, (9-16) green tones, (17-20) blue tones */
#define PCF8833_PTLAR     0x30 /* Partial area; Data: (1) start address (2) end address */
#define PCF8833_VSCRDEF   0x33 /* Vertical scroll definition; Data: (1) top fixed, (2) scrol area, (3) bottom fixed */
#define PCF8833_TEOFF     0x34 /* Tearing line off; Data: none */
#define PCF8833_TEON      0x35 /* Tearing line on;  Data: (1) don't care */
#define PCF8833_MADCTL    0x36 /* Memory data access control; Data: (1) access control settings */
#define PCF8833_SEP       0x37 /* Set Scroll Entry Point; Data: (1) scroll entry point */
#define PCF8833_IDMOFF    0x38 /* Idle mode off; Data: none */
#define PCF8833_IDMON     0x39 /* Idle mode on; Data: none */
#define PCF8833_COLMOD    0x3a /* Interface pixel format; Data: (1) color interface format */
#define PCF8833_SETVOP    0xb0 /* Set VOP; Data: (1) VOP5-8 (2) VOP0-4 */
#define PCF8833_BRS       0xb4 /* Bottom Row Swap; Data: none */
#define PCF8833_TRS       0xb6 /* Top Row Swap; Data: none */
#define PCF8833_FINV      0xb9 /* Super Frame INVersion; Data: none */
#define PCF8833_DOR       0xba /* Data ORder; Data: none */
#define PCF8833_TCDFE     0xbd /* Enable/disable DF temp comp; Data: none */
#define PCF8833_TCVOPE    0xbf /* Enable or disable VOP temp comp; Data: none */
#define PCF8833_EC        0xc0 /* Internal or external oscillator; Data: none */
#define PCF8833_SETMUL    0xc2 /* Set multiplication factor; Data: (1) Multiplication factor */
#define PCF8833_TCVOPAB   0xc3 /* Set TCVOP slopes A and B; Data: (1) SLB and SLA */
#define PCF8833_TCVOPCD   0xc4 /* Set TCVOP slopes C and D; Data: (1) SLD and SLC */
#define PCF8833_TCDF      0xc5 /* Set divider frequency; Data: Divider factor in region (1) A (2) B (3) C (4) D */
#define PCF8833_DF8COLOR  0xc6 /* Set divider frequency 8-colour mode; Data: (1) DF80-6 */
#define PCF8833_SETBS     0xc7 /* Set bias system; Data: (1) Bias systems */
#define PCF8833_RDTEMP    0xc8 /* Temperature read back; Data: none */
#define PCF8833_NLI       0xc9 /* N-Line Inversion; Data: (1) NLI time slots invervsion */
#define PCF8833_RDID1     0xda /* Read ID1; Data: none */
#define PCF8833_RDID2     0xdb /* Read ID2; Data: none */
#define PCF8833_RDID3     0xdc /* Read ID3; Data: none */
#define PCF8833_SFD       0xef /* Select factory defaults; Data: none */
#define PCF8833_ECM       0xf0 /* Enter Calibration mode; Data: (1) Calibration control settings */
#define PCF8833_OTPSHTIN  0xf1 /* Shift data in OTP shift registers; Data: Any number of bytes */

/* PCF8833 status register bit definitions */

#define PCF8833_ST_RGB_ORD        (1 <<  2)
#define PCF8833_ST_LINE_ADDR_ORD  (1 <<  3)
#define PCF8833_ST_ADDR_MODE      (1 <<  4)
#define PCF8833_ST_X_ADDR_ODR     (1 <<  5)
#define PCF8833_ST_Y_ADDR_ODR     (1 <<  6)
#define PCF8833_ST_BOOSTER_ON     (1 <<  7)
#define PCF8833_ST_NORM_MODE      (1 <<  8)
#define PCF8833_ST_SLEEP_MODE     (1 <<  9)
#define PCF8833_ST_PARTIAL_MODE   (1 << 10)
#define PCF8833_ST_IDLE_MODE      (1 << 11)
#define PCF8833_ST_PIXEL_FORM     (7 << 12)
#define PCF8833_ST_TEARING_MODE   (1 << 17)
#define PCF8833_ST_DISPLAY_MODE   (1 << 18)
#define PCF8833_ST_ALL_PIXELS_OFF (1 << 19)
#define PCF8833_ST_ALL_PIXELS_ON  (1 << 20)
#define PCF8833_ST_INV_MODE       (1 << 21)
#define PCF8833_ST_V_SCROLL_MODE  (1 << 23)

#endif /* __DRIVERS_LCD_PCF8833_H */
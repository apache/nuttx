/**************************************************************************************
 * drivers/lcd/s1d15g10.h
 * Definitions for the Epson S1D15G0 LCD controller
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * References: S1D15G0D08B000, Seiko Epson Corportation, 2002.
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

#ifndef __DRIVERS_LCD_S1D15G10_H
#define __DRIVERS_LCD_S1D15G10_H

/**************************************************************************************
 * Included Files
 **************************************************************************************/

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* Epson S1D15G10 Command Set */

#define S1D15G10_DISON       0xaf /* Display on; Data: none */
#define S1D15G10_DISOFF      0xae /* Display off; Data: none */
#define S1D15G10_DISNOR      0xa6 /* Normal display; Data: none */
#define S1D15G10_DISINV      0xa7 /* Inverse display; Data: none */
#define S1D15G10_COMSCN      0xbb /* Common scan direction; Data: (1) common scan direction */
#define S1D15G10_DISCTL      0xca /* Display control; Data: Data: (1) CL div, F1/2 pat, (2) duty, (3) FR inverse (4) dispersion */
#define S1D15G10_SLPIN       0x95 /* Sleep in; Data: none */
#define S1D15G10_SLPOUT      0x94 /* Sleep out; Data: none */
#define S1D15G10_PASET       0x75 /* Page address set; Data: (1) start page, (2) end page */
#define S1D15G10_CASET       0x15 /* Column address set; Data: (1) start addr, (2) end addr */
#define S1D15G10_DATCTL      0xbc /* Data scan direction, etc.; Data: (1) inverse, scan dir (2) RGB, (3) gray-scale */
#define S1D15G10_RGBSET8     0xce /* 256-color position set; Data: (1-8) red tones, (9-16) green tones, (17-20) blue tones */
#define S1D15G10_RAMWR       0x5c /* Writing to memory; Data: (1) write data */
#define S1D15G10_RAMRD       0x5d /* Reading from memory; Data: (1) read data */
#define S1D15G10_PTLIN       0xa8 /* Partial display in; Data: (1) start addr, (2) end addr */
#define S1D15G10_PTLOUT      0xa9 /* Partial display out; Data: none */
#define S1D15G10_RMWIN       0xe0 /* Read and modify write; Data: none */
#define S1D15G10_RMWOUT      0xee /* End; Data: none */
#define S1D15G10_ASCSET      0xaa /* Area scroll set; Data: (1) top addr, (2) bottom addr, (3) Num blocks, (4) scroll mode */
#define S1D15G10_SCSTART     0xab /* Scroll start set; Data: (1) start block addr */
#define S1D15G10_OSCON       0xd1 /* Internal oscillation on; Data: none */
#define S1D15G10_OSCOFF      0xd2 /* Internal oscillation off; Data: none */
#define S1D15G10_PWRCTR      0x20 /* Power control; Data: (1) LCD drive power */
#define S1D15G10_VOLCTR      0x81 /* Electronic volume control; Data: (1) volume value, (2) resistance ratio */
#define S1D15G10_VOLUP       0xd6 /* Increment electronic control by 1; Data: none */
#define S1D15G10_VOLDOWN     0xd7 /* Decrement electronic control by 1; Data: none */
#define S1D15G10_TMPGRD      0x82 /* Temperature gradient set; Data: (1-14) temperature gradient */
#define S1D15G10_EPCTIN      0xcd /* Control EEPROM; Data: (1) read/write */
#define S1D15G10_EPCOUT      0xcc /* Cancel EEPROM control; Data: none */
#define S1D15G10_EPMWR       0xfc /* Write into EEPROM; Data: none */
#define S1D15G10_EPMRD       0xfd /* Read from EEPROM; Data: none */
#define S1D15G10_EPSRRD1     0x7c /* Read register 1; Data: none */
#define S1D15G10_EPSRRD2     0x7d /* Read regiser 2; Data: none */
#define S1D15G10_NOP         0x25 /* NOP intruction (0x45?); Data: none */
#define S1D15G10_STREAD      0x20 /* Status read; Data: none */

/* Status register bit definions (after reset or NOP) */

#define S1D15G10_SR_PARTIAL  (1 << 0)  /* Bit 0: Partial display */
#define S1D15G10_SR_NORMAL   (1 << 1)  /* Bit 1: Normal (vs. inverse) display */
#define S1D15G10_SR_EEPROM   (1 << 2)  /* Bit 2: EEPROM access */
#define S1D15G10_SR_DISPON   (1 << 3)  /* Bit 3: Display on */
#define S1D15G10_SR_COLSCAN  (1 << 4)  /* Bit 4: Column (vs. page) scan direction */
#define S1D15G10_SR_RMW      (1 << 5)  /* Bit 5: Read modify write */
#define S1D15G10_SR_SCROLL   (3 << 6)  /* Bits 6-7: Area scroll mode */

/* Status register bit definions (after EPSRRD1) */

#define S1D15G10_SR_VOLUME   0x3f      /* Bits 0-5: Electronic volume control values */

/* Status register bit definions (after EPSRRD2) */

#define S1D15G10_SR_VOLUME   0x07      /* Bits 0-2: Built-in resistance ratio */

#endif /* __DRIVERS_LCD_S1D15G10_H */
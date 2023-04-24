/****************************************************************************
 * drivers/lcd/st7789.h
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

#ifndef __DRIVERS_LCD_ST7789_H
#define __DRIVERS_LCD_ST7789_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System Function Command Table 1 */

#define ST7789_NOP        0x00   /* No Operation */
#define ST7789_SWRESET    0x01   /* Software Reset */
#define ST7789_RDDID      0x04   /* Read Display ID */
#define ST7789_RDDST      0x09   /* Read Display Status */
#define ST7789_RDDPM      0x0a   /* Read Display Power */
#define ST7789_RDDMADCTL  0x0b   /* Read Display MADCTL */
#define ST7789_RDDCOLMOD  0x0c   /* Read Display Pixel Format */
#define ST7789_RDDIM      0x0d   /* Read Display Image Mode */
#define ST7789_RDDSM      0x0e   /* Read Display Signal Mode */
#define ST7789_RDDSDR     0x0f   /* Read Display Self-Diagnostic Result */
#define ST7789_SLPIN      0x10   /* Sleep In & Booster Off */
#define ST7789_SLPOUT     0x11   /* Sleep Out & Booster On */
#define ST7789_PTLON      0x12   /* Partial Mode On */
#define ST7789_NORON      0x13   /* Partial Mode Off */
#define ST7789_INVOFF     0x20   /* Display Inversion Off */
#define ST7789_INVON      0x21   /* Display Inversion On */
#define ST7789_GAMSET     0x26   /* Gamma Set */
#define ST7789_DISPOFF    0x28   /* Display Off */
#define ST7789_DISPON     0x29   /* Display On */
#define ST7789_CASET      0x2a   /* Column Address Set */
#define ST7789_RASET      0x2b   /* Row Address Set */
#define ST7789_RAMWR      0x2c   /* Memory Write */
#define ST7789_RAMRD      0x2e   /* Memory Read */
#define ST7789_PTLAR      0x30   /* Partial Area */
#define ST7789_VSCRDEF    0x33   /* Vertical Scrolling Definition */
#define ST7789_TEOFF      0x34   /* Tearing Effect Line Off */
#define ST7789_TEON       0x35   /* Tearing Effect Line On */
#define ST7789_MADCTL     0x36   /* Memory Data Access Control */
#define ST7789_VSCRSADD   0x37   /* Vertical Scrolling Start Address */
#define ST7789_IDMOFF     0x38   /* Idle Mode Off */
#define ST7789_IDMON      0x39   /* Idle Mode On */
#define ST7789_COLMOD     0x3a   /* Interface Pixel Format */
#define ST7789_RAMWRC     0x3c   /* Memory Write Continue */
#define ST7789_RAMRDC     0x3e   /* Memory Read Continue */
#define ST7789_TESCAN     0x44   /* Set Tear Scanline */
#define ST7789_RDTESCAN   0x45   /* Get Scanline */
#define ST7789_WRDISBV    0x51   /* Write Display Brightness */
#define ST7789_RDDISBV    0x52   /* Read Display Brightness Value */
#define ST7789_WRCTRLD    0x53   /* Write CTRL Display */
#define ST7789_RDCTRLD    0x54   /* Read CTRL Value Display */
#define ST7789_WRCACE     0x55   /* Write Content Adaptive Brightness Control and Color Enhancement */
#define ST7789_RDCABC     0x56   /* Read Content Adaptive Brightness Control */
#define ST7789_WRCABCMB   0x5e   /* Write CABC Minimum Brightness */
#define ST7789_RDCABCMB   0x5f   /* Read CABC Minimum Brightness */
#define ST7789_RDABCSDR   0x68   /* Read Automatic Brightness Control Self-Diagnostic Result */
#define ST7789_RDID1      0xda   /* Read ID1 */
#define ST7789_RDID2      0xdb   /* Read ID2 */
#define ST7789_RDID3      0xdc   /* Read ID3 */

/* System Function Command Table 2 */

#define ST7789_RAMCTRL    0xb0   /* RAM Control */
#define ST7789_RGBCTRL    0xb1   /* RGB Control */
#define ST7789_PORCTRL    0xb2   /* Porch Control */
#define ST7789_FRCTRL1    0xb3   /* Frame Rate Control */
#define ST7789_PARCTRL    0xb5   /* Partial Control */
#define ST7789_GCTRL      0xb7   /* Gate Control */
#define ST7789_GTADJ      0xb8   /* Gate on Timing Adjustment */
#define ST7789_DGMEN      0xba   /* Digital Gamma Enable */
#define ST7789_VCOMS      0xbb   /* VCOMS Setting */
#define ST7789_POWSAVE    0xbc   /* Power Saving Mode */
#define ST7789_DLPOFFSAVE 0xbd   /* Display Off Power Save */
#define ST7789_LCMCTRL    0xc0   /* LCM Control */
#define ST7789_IDSET      0xc1   /* ID Setting */
#define ST7789_VDVVRHEN   0xc2   /* VDV and VRH Command Enable */
#define ST7789_VRHS       0xc3   /* VRH Set */
#define ST7789_VDVSET     0xc4   /* VDV Set */
#define ST7789_VCMOFSET   0xc5   /* VCOMS Offset Set */
#define ST7789_FRCTR2     0xc6   /* FR Control 2 */
#define ST7789_CABCCTRL   0xc7   /* CABC Control */
#define ST7789_REGSEL1    0xc8   /* Register Value Selection 1 */
#define ST7789_REGSEL2    0xca   /* Register Value Selection 2 */
#define ST7789_PWMFRSEL   0xcc   /* PWM Frequency Selection */
#define ST7789_PWCTRL1    0xd0   /* Power Control 1 */
#define ST7789_VAPVANEN   0xd2   /* Enable VAP/VAN Signal Output */
#define ST7789_CMD2EN     0xdf   /* Command 2 Enable */
#define ST7789_PVGAMCTRL  0xe0   /* Positive Voltage Gamma Control */
#define ST7789_NVGAMCTRL  0xe1   /* Negative Voltage Gamma Control */
#define ST7789_DGMLUTR    0xe2   /* Digital Gamma Look-Up Table for Red */
#define ST7789_DGMLUTB    0xe3   /* Digital Gamma Look-Up Table for Blue */
#define ST7789_GATECTRL   0xe4   /* Gate Control */
#define ST7789_SPI2EN     0xe7   /* SPI2 Enable */
#define ST7789_PWCTRL2    0xe8   /* Power Control 2 */
#define ST7789_EQCTRL     0xe9   /* Equalize Time Control */
#define ST7789_PROMCTRL   0xec   /* Program Control */
#define ST7789_PROMEN     0xfa   /* Program Mode Enable */
#define ST7789_NVMSET     0xfc   /* NVM Setting */
#define ST7789_PROMACT    0xfe   /* Program Action */

#endif /* __DRIVERS_LCD_ST7789_H */

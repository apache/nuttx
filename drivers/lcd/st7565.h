/****************************************************************************
 * drivers/lcd/st7565.h
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

/* Definitions for the ST7565 128x64 Dot Matrix LCD
 * Driver with C
 *
 * References:
 * SSD1305.pdf, "Solomon Systech SSD1305 136x64 Dot Matrix OLED/PLED
 * Segment/Common Driver with Controller," Solomon Systech Limited,
 * http://www.solomon-systech.com, May, 2008.
 */

#ifndef __DRIVERS_LCD_ST7565_H
#define __DRIVERS_LCD_ST7565_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Fundamental Commands *****************************************************/

#define ST7565_DISPOFF             0xae /* 0xae: Display OFF (sleep mode) */
#define ST7565_DISPON              0xaf /* 0xaf: Display ON in normal mode */

#define ST7565_SETSTARTLINE        0x40 /* 0x40-7f: Set display start line */
#  define ST7565_STARTLINE_MASK    0x3f

#define ST7565_SETPAGESTART        0xb0 /* 0xb0-b7: Set page start address */
#  define ST7565_PAGESTART_MASK    0x0f

#define ST7565_SETCOLL             0x00 /* 0x00-0x0f: Set lower column address */
#  define ST7565_COLL_MASK         0x0f
#define ST7565_SETCOLH             0x10 /* 0x10-0x1f: Set higher column address */
#  define ST7565_COLH_MASK         0x0f

#define ST7565_ADCNORMAL           0xa0 /* 0xa6: ADC Normal */
#define ST7565_ADCINVERSE          0xa1 /* 0xa7: ADC Inverse */

#define ST7565_DISPNORMAL          0xa6 /* 0xa6: Normal display */
#define ST7565_DISPINVERSE         0xa7 /* 0xa7: Inverse display */

#define ST7565_DISPRAM             0xa4 /* 0xa4: Resume to RAM content display */
#define ST7565_DISPENTIRE          0xa5 /* 0xa5: Entire display ON */

#define ST7565_BIAS_1_9            0xa2 /* 0xa2: Select BIAS setting 1/9 */
#define ST7565_BIAS_1_7            0xa3 /* 0xa3: Select BIAS setting 1/7 */

#define ST7565_ENTER_RMWMODE       0xe0 /* 0xe0: Enter the Read Modify Write mode */
#define ST7565_EXIT_RMWMODE        0xee /* 0xee: Leave the Read Modify Write mode */
#define ST7565_EXIT_SOFTRST        0xe2 /* 0xe2: Software RESET */

#define ST7565_SETCOMNORMAL        0xc0 /* 0xc0: Set COM output direction, normal mode */
#define ST7565_SETCOMREVERSE       0xc8 /* 0xc8: Set COM output direction, reverse mode */

#define ST7565_POWERCTRL_EXT       0x28 /* 0x28: Only the external power supply is used */
#define ST7565_POWERCTRL_VF        0x29 /* 0x29: Only the V/F circuit is used */
#define ST7565_POWERCTRL_VR        0x2b /* 0x2b: Only the voltage regulator circuit and the
                                         * voltage follower circuit are used
                                         */
#define ST7565_POWERCTRL_B         0x2c /* 0x2c: Booster=ON */
#define ST7565_POWERCTRL_BR        0x2e /* 0x2e: Booster=ON V/R=ON */
#define ST7565_POWERCTRL_INT       0x2f /* 0x2f: Only the internal power supply is used */

/* Regulation Resistior ratio V0 = (1+Rb/Ra)*Vev */

#define ST7565_REG_RES_3_0         0x20 /* (Ra/Rb = 3.0) */
#define ST7565_REG_RES_3_5         0x21 /* (Ra/Rb = 5.5) */
#define ST7565_REG_RES_4_0         0x22 /* (Ra/Rb = 4.0) */
#define ST7565_REG_RES_4_5         0x23 /* (Ra/Rb = 4.5) */
#define ST7565_REG_RES_5_0         0x24 /* (Ra/Rb = 5.0) */
#define ST7565_REG_RES_5_5         0x25 /* (Ra/Rb = 5.5) */
#define ST7565_REG_RES_6_0         0x26 /* (Ra/Rb = 6.0) */
#define ST7565_REG_RES_6_5         0x27 /* (Ra/Rb = 6.5) */

/* Set Electronic volume mode */

#define ST7565_SETEVMODE           0x81 /* 0x81: Set contrast control */

/* Set Ev value register
 * ratio Vev = (1-a/162)*Vreg, Vreg = 2.1V
 */

#define ST7565_SETEVREG(x)        (0x0 | ((x) & 0x3F))

/* Set booster ratio */

#define ST7565_MODE_NORMAL         0xac /* Set Normal Mode */
#define ST7565_MODE_SLEEP          0xad /* Set Sleep Mode */

/* Set booster ratio */

#define ST7565_SETBOOSTER          0xf8
#define ST7565_SETBOOSTER4X        0x00 /* x2, x3, x4 */
#define ST7565_SETBOOSTER5X        0x01 /* x5 */
#define ST7565_SETBOOSTER6X        0x02 /* x6 */

#define ST7565_NOP                 0xe3 /* 0xe3: NOP Command for no operation */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#endif /* __DRIVERS_LCD_ST7565_H */

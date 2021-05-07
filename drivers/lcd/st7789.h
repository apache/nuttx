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

#define ST7789_NOP       0x00   /* No Operation */
#define ST7789_SWRESET   0x01   /* Software Reset */
#define ST7789_RDDID     0x04   /* Read Display ID */
#define ST7789_RDDST     0x09   /* Read Display Status */
#define ST7789_RDDPM     0x0a   /* Read Display Power */
#define ST7789_SLPIN     0x10   /* Sleep In & Booster Off */
#define ST7789_SLPOUT    0x11   /* Sleep Out & Booster On */
#define ST7789_PTLON     0x12   /* Partial Mode On */
#define ST7789_NORON     0x13   /* Partial Mode Off */
#define ST7789_INVOFF    0x20   /* Display Inversion Off */
#define ST7789_INVON     0x21   /* Display Inversion On */
#define ST7789_DISPOFF   0x28   /* Display Off */
#define ST7789_DISPON    0x29   /* Display On */
#define ST7789_CASET     0x2a   /* Column Address Set */
#define ST7789_RASET     0x2b   /* Row Address Set */
#define ST7789_RAMWR     0x2c   /* Memory Write */
#define ST7789_RAMRD     0x2e   /* Memory Read */
#define ST7789_MADCTL    0x36   /* Memory Data Access Control */
#define ST7789_IDMOFF    0x38   /* Idle Mode Off */
#define ST7789_IDMON     0x39   /* Idle Mode On */
#define ST7789_COLMOD    0x3a   /* Interface Pixel Format */

#endif /* __DRIVERS_LCD_ST7789_H */

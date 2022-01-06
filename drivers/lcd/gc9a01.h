/****************************************************************************
 * drivers/lcd/gc9a01.h
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

#ifndef __DRIVERS_LCD_GC9A01_H
#define __DRIVERS_LCD_GC9A01_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GC9A01_NOP       0x00   /* No Operation */
#define GC9A01_SWRESET   0x01   /* Software Reset */
#define GC9A01_RDDID     0x04   /* Read Display ID */
#define GC9A01_RDDST     0x09   /* Read Display Status */

#define GC9A01_SLPIN     0x10   /* Sleep In & Booster Off */
#define GC9A01_SLPOUT    0x11   /* Sleep Out & Booster On */
#define GC9A01_PTLON     0x12   /* Partial Mode On */
#define GC9A01_NORON     0x13   /* Partial Mode Off */

#define GC9A01_INVOFF    0x20   /* Display Inversion Off */
#define GC9A01_INVON     0x21   /* Display Inversion On */
#define GC9A01_DISPOFF   0x28   /* Display Off */
#define GC9A01_DISPON    0x29   /* Display On */
#define GC9A01_CASET     0x2a   /* Column Address Set */
#define GC9A01_RASET     0x2b   /* Row Address Set */
#define GC9A01_RAMWR     0x2c   /* Memory Write */
#define GC9A01_RAMRD     0x2e   /* Memory Read */

#define GC9A01_PTLAR     0x30   /* Partial Area */
#define GC9A01_VSCRDEF   0x33   /* Vertical Scrolling Definition */
#define GC9A01_TEON      0x35   /* Tering Effect Line On */
#define GC9A01_MADCTL    0x36   /* Memory Data Access Control */

#define GC9A01_MADCTL_MY  (1<<7)  /* Page Address Order */
#define GC9A01_MADCTL_MX  (1<<6)  /* Column Address Order */
#define GC9A01_MADCTL_MV  (1<<5)  /* Page/Column Order */
#define GC9A01_MADCTL_ML  (1<<4)  /* Line Address Order */
#define GC9A01_MADCTL_BGR (1<<3)  /* Set Panel Order BGR */
#define GC9A01_MADCTL_MH  (1<<2)  /* Display Data Latch Order */

#define GC9A01_VSCSAD    0x37   /* Vertical Scrolling Start Address */
#define GC9A01_IDMOFF    0x38   /* Idle Mode Off */
#define GC9A01_IDMON     0x39   /* Idle Mode On */
#define GC9A01_COLMOD    0x3a   /* Interface Pixel Format */

#define GC9A01_ENIREG1   0xFE   /* Enable internal register 1 */
#define GC9A01_ENIREG2   0xEF   /* Enable internal register 2 */

#endif /* __DRIVERS_LCD_GC9A01_H */

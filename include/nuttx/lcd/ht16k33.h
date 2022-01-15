/****************************************************************************
 * include/nuttx/lcd/ht16k33.h
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

#ifndef __INCLUDE_NUTTX_LCD_HT16K33_H
#define __INCLUDE_NUTTX_LCD_HT16K33_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/* Configuration
 * CONFIG_I2C - Enables support for SPI drivers
 * CONFIG_LCD_HT16K33 - Enables support for the HT16K33 driver
 */

#if defined(CONFIG_I2C) && defined(CONFIG_LCD_HT16K33)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HT16K33 Address */

#define HT16K33_I2C_ADDR               0x70

/* Default contrast */

#define DEFAULT_CONTRAST               0xf

#define HT16K33_CONTRAST_MIN           0
#define HT16K33_CONTRAST_MAX           15

/* HT16K33 register addresses */

#define HT16K33_DISP_DATA_ADDR         0x00 /* Display Data Address Pointer: 0x00-0x0f */
#define HT16K33_SYSTEM_SETUP           0x20 /* System Setup: bit 0 = System Oscilator */
#define HT16K33_KEY_DATA_ADDR          0x40 /* Key Data Address Pointer: 0x00-0x07 */
#define HT16K33_INT_FLAG_ADDR          0x60 /* INT Flag Address */
#define HT16K33_DISPLAY_SETUP          0x80 /* DISPLAY Setup */
#define HT16K33_ROW_INT_SET            0xa0 /* ROW/INT set */
#define HT16K33_DIMMING_SET            0xe0 /* Dimming set */
#define HT16K33_TEST_MODE              0xd9 /* Holtek Test Mode */

/* Bits and flags definitions */

#define SYSTEM_SETUP_OSC_ON            (1 << 0) /* Turn ON the Oscillator */
#define DISPLAY_SETUP_DISP_ON          (1 << 0) /* Display ON */
#define DISPLAY_SETUP_BLINK_SHIFT      1
#define DISPLAY_SETUP_BLINK_MASK       (3 << DISPLAY_SETUP_BLINK_SHIFT)
#  define DISPLAY_SETUP_BLINK_OFF      (0 << DISPLAY_SETUP_BLINK_SHIFT)
#  define DISPLAY_SETUP_BLINK_2HZ      (1 << DISPLAY_SETUP_BLINK_SHIFT)
#  define DISPLAY_SETUP_BLINK_1HZ      (2 << DISPLAY_SETUP_BLINK_SHIFT)
#  define DISPLAY_SETUP_BLINK_0P5HZ    (3 << DISPLAY_SETUP_BLINK_SHIFT)
#define DIMMING_SHIFT                  0
#define DIMMING_MASK                   (0xf << DIMMING_SHIFT)
#define DIMMING_DUTY(N)                ((N - 1) << DIMMING_SHIFT)

/* This 14-segment mapping comes from Dave Madison's LED-Segment-ASCII
 * released under MIT license:
 * https://github.com/dmadison/LED-Segment-ASCII
 */

static const uint16_t asciito14seg[96] =
{
  0x0000, /* (space) */
  0x4006, /* ! */
  0x0202, /* " */
  0x12ce, /* # */
  0x12ed, /* $ */
  0x3fe4, /* % */
  0x2359, /* & */
  0x0200, /* ' */
  0x2400, /* ( */
  0x0900, /* ) */
  0x3fc0, /* * */
  0x12c0, /* + */
  0x0800, /* , */
  0x00c0, /* - */
  0x4000, /* . */
  0x0c00, /* / */
  0x0c3f, /* 0 */
  0x0406, /* 1 */
  0x00db, /* 2 */
  0x008f, /* 3 */
  0x00e6, /* 4 */
  0x2069, /* 5 */
  0x00fd, /* 6 */
  0x0007, /* 7 */
  0x00ff, /* 8 */
  0x00ef, /* 9 */
  0x1200, /* : */
  0x0a00, /* ; */
  0x2440, /* < */
  0x00c8, /* = */
  0x0980, /* > */
  0x5083, /* ? */
  0x02bb, /* @ */
  0x00f7, /* A */
  0x128f, /* B */
  0x0039, /* C */
  0x120f, /* D */
  0x0079, /* E */
  0x0071, /* F */
  0x00bd, /* G */
  0x00f6, /* H */
  0x1209, /* I */
  0x001e, /* J */
  0x2470, /* K */
  0x0038, /* L */
  0x0536, /* M */
  0x2136, /* N */
  0x003f, /* O */
  0x00f3, /* P */
  0x203f, /* Q */
  0x20f3, /* R */
  0x00ed, /* S */
  0x1201, /* T */
  0x003e, /* U */
  0x0c30, /* V */
  0x2836, /* W */
  0x2d00, /* X */
  0x00ee, /* Y */
  0x0c09, /* Z */
  0x0039, /* [ */
  0x2100, /* \ */
  0x000f, /* ] */
  0x2800, /* ^ */
  0x0008, /* _ */
  0x0100, /* ` */
  0x1058, /* a */
  0x2078, /* b */
  0x00d8, /* c */
  0x088e, /* d */
  0x0858, /* e */
  0x14c0, /* f */
  0x048e, /* g */
  0x1070, /* h */
  0x1000, /* i */
  0x0a10, /* j */
  0x3600, /* k */
  0x0030, /* l */
  0x10d4, /* m */
  0x1050, /* n */
  0x00dc, /* o */
  0x0170, /* p */
  0x0486, /* q */
  0x0050, /* r */
  0x2088, /* s */
  0x0078, /* t */
  0x001c, /* u */
  0x0810, /* v */
  0x2814, /* w */
  0x2d00, /* x */
  0x028e, /* y */
  0x0848, /* z */
  0x0949, /* { */
  0x1200, /* | */
  0x2489, /* } */
  0x0cc0, /* ~ */
  0x0000, /* (del) */
};

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ht16k33_register
 *
 * Description:
 *   Initialize the HT16K33 device as a LEDS interface.
 *
 * Input Parameters:
 *   spi   - An instance of the SPI interface to use to communicate
 *           with the HT16K33.
 *   devno - Device number to identify current display.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

struct i2c_master_s;  /* Forward reference */

int ht16k33_register(int devno, FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_HT16K33 */
#endif /* __INCLUDE_NUTTX_LCD_HT16K33_H */

/****************************************************************************
 * include/nuttx/lcd/slcd_ioctl.h
 * IOCTL commands for segment LCDs
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

#ifndef __INCLUDE_NUTTX_LCD_SLCD_IOCTL_H
#define __INCLUDE_NUTTX_LCD_SLCD_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL commands that may be supported by some SLCD drivers */

/* SLCDIOC_GETATTRIBUTES:  Get the attributes of the SLCD
 *
 * argument:  Pointer to struct slcd_attributes_s in which values will be
 *            returned
 */

#define SLCDIOC_GETATTRIBUTES  _SLCDIOC(0x0001)

/* SLCDIOC_CURPOS:  Get the SLCD cursor positioni (rows x characters)
 *
 * argument:  Pointer to struct slcd_curpos_s in which values will be
 *            returned
 */

#define SLCDIOC_CURPOS _SLCDIOC(0x0002)

/* SLCDIOC_SETBAR: Set bars on a bar display
 *
 * argument:  32-bit bitset, with each bit corresponding to one bar.
 */

#define SLCDIOC_SETBAR  _SLCDIOC(0x0003)

/* SLCDIOC_GETCONTRAST: Get the current contrast setting
 *
 * argument:  Pointer type int that will receive the current contrast
 *            setting
 */

#define SLCDIOC_GETCONTRAST _SLCDIOC(0x0004)

/* SLCDIOC_SETCONTRAST: Set the contrast to a new value
 *
 * argument:  The new contrast value
 */

#define SLCDIOC_SETCONTRAST _SLCDIOC(0x0005)

/* SLCDIOC_GETBRIGHTNESS: Get the current brightness setting
 *
 * argument:  Pointer type int that will receive the current brightness
 *            setting
 */

#define SLCDIOC_GETBRIGHTNESS _SLCDIOC(0x0006)

/* SLCDIOC_SETBRIGHTNESS: Set the brightness to a new value
 *
 * argument:  The new brightness value
 */

#define SLCDIOC_SETBRIGHTNESS _SLCDIOC(0x0007)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Used with the SLCDIOC_GETATTRIBUTES ioctl call */

struct slcd_attributes_s
{
  uint16_t nrows;           /* Number of the rows on the SLCD */
  uint16_t ncolumns;        /* Number of characters in one row on the SLCD */
  uint8_t  nbars;           /* Number of bars supported by the SLCD */
  uint8_t  maxcontrast;     /* Maximum contrast value */
  uint8_t  maxbrightness;   /* Maximum brightness value */
};

/* Used with the SLCDIOC_CURPOS ioctl call */

struct slcd_curpos_s
{
  uint16_t row;             /* Current row (zero-based) */
  uint16_t column;          /* Current column (zero-based) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_SLCD_IOCTL_H */

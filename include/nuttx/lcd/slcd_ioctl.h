/************************************************************************************
 * include/nuttx/input/slcd_ioctl.h
 * IOCTL commands for segment LCDs
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#ifndef __INCLUDE_NUTTX_INPUT_SLCD_IOCTL_H
#define __INCLUDE_NUTTX_INPUT_SLCD_IOCTL_H

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

#endif /* __INCLUDE_NUTTX_INPUT_SLCD_IOCTL_H */


/************************************************************************************
 * include/nuttx/input/mouse.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

/* The mouse driver exports a standard character driver interface. By
 * convention, the mouse driver is registered as an input device at
 * /dev/mouseN where N uniquely identifies the driver instance.
 *
 * This header file documents the generic interface that all NuttX
 * mouse devices must conform.  It adds standards and conventions on
 * top of the standard character driver interface.
 */

#ifndef __INCLUDE_NUTTX_INPUT_MOUSE_H
#define __INCLUDE_NUTTX_INPUT_MOUSE_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* These definitions provide the meaning of all of the bits that may be
 * reported in the struct mouse_report_s buttons.
 */

#define MOUSE_BUTTON_1       (1 << 0) /* True: Left mouse button pressed */
#define MOUSE_BUTTON_2       (1 << 1) /* True: Right mouse button pressed */
#define MOUSE_BUTTON_3       (1 << 2) /* True: Middle mouse button pressed */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* This structure contains information about the current mouse button states and
 * mouse position.  Positional units are device specific and determined by mouse
 * configuration settings.
 */

struct mouse_report_s
{
  uint8_t  buttons;  /* See TOUCH_* definitions above */
                     /* Possibly padded with 1 byte here */
  int16_t  x;        /* X coordinate of the mouse position */
  int16_t  y;        /* Y coordinate of the mouse position */
#ifdef CONFIG_MOUSE_WHEEL
  int16_t  wheel;    /* Mouse wheel position */
#endif
};

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_INPUT_MOUSE_H */

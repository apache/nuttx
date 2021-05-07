/************************************************************************************
 * include/nuttx/input/mouse.h
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
#ifdef CONFIG_INPUT_MOUSE_WHEEL
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

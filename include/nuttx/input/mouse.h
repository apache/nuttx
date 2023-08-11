/****************************************************************************
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
 ****************************************************************************/

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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These definitions provide the meaning of all of the bits that may be
 * reported in the struct mouse_report_s buttons.
 */

#define MOUSE_BUTTON_1       (1 << 0) /* True: Left mouse button pressed */
#define MOUSE_BUTTON_2       (1 << 1) /* True: Right mouse button pressed */
#define MOUSE_BUTTON_3       (1 << 2) /* True: Middle mouse button pressed */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure contains information about the current mouse button
 * states and mouse position.  Positional units are device specific
 * and determined by mouseconfiguration settings.
 */

struct mouse_report_s
{
  uint8_t buttons;  /* See MOUSE_* definitions above */
                    /* Possibly padded with 1 byte here */
  int16_t x;        /* X coordinate of the mouse position */
  int16_t y;        /* Y coordinate of the mouse position */
#ifdef CONFIG_INPUT_MOUSE_WHEEL
  int16_t wheel;    /* Mouse wheel position */
#endif
};

/* This structure is for mouse lower half driver */

struct mouse_lowerhalf_s
{
  FAR void *priv;  /* Save the upper half pointer */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mouse_event
 *
 * Description:
 *   The lower half driver pushes mouse events through this interface,
 *   provided by mouse upper half.
 *
 * Arguments:
 *   priv    - Upper half driver handle.
 *   sample  - pointer to data of mouse point event.
 ****************************************************************************/

void mouse_event(FAR void *priv, FAR const struct mouse_report_s *sample);

/****************************************************************************
 * Name: mouse_register
 *
 * Description:
 *   This function registers a mouse device, the upper half binds
 *   with hardware device through the lower half instance.
 *
 * Arguments:
 *   lower     - A pointer of lower half instance.
 *   path      - The path of mouse device. such as "/dev/mouse0"
 *   nums      - Number of the mouse points structure.
 *
 * Return:
 *   OK if the driver was successfully registered; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int mouse_register(FAR struct mouse_lowerhalf_s *lower,
                   FAR const char *path, uint8_t nums);

/****************************************************************************
 * Name: mouse_unregister
 *
 * Description:
 *   This function is used to mouse driver to unregister and
 *   release the occupied resources.
 *
 * Arguments:
 *   lower     - A pointer to an insatnce of mouse lower half driver.
 *   path      - The path of mouse device. such as "/dev/mouse0"
 ****************************************************************************/

void mouse_unregister(FAR struct mouse_lowerhalf_s *lower,
                      FAR const char *path);

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_INPUT_MOUSE_H */

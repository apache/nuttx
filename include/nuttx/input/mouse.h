/****************************************************************************
 * include/nuttx/input/mouse.h
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These definitions provide the meaning of all of the bits that may be
 * reported in the struct mouse_report_s buttons.
 */

#define MOUSE_BUTTON_1       (1 << 0) /* True: Left mouse button pressed */
#define MOUSE_BUTTON_2       (1 << 1) /* True: Right mouse button pressed */
#define MOUSE_BUTTON_3       (1 << 2) /* True: Middle mouse button pressed */

/* IOCTL Commands ***********************************************************/

/* Common mouse IOCTL commands */

#define MSIOC_VENDOR         _MSIOC(0x0001)  /* Vendor-specific commands */

#define MSC_FIRST            0x0001          /* First common command */
#define MSC_NCMDS            1               /* One common commands */

/* Vendor-specific command structure
 *
 * This structure is used to pass vendor-specific commands to the mouse
 * driver.  The vendor-specific command is identified by the 'cmd' field
 * and the length of the data is specified by the 'len' field.  The
 * data follows the structure in a contiguous block of memory.
 *
 * The vendor-specific command is defined by the vendor and is not
 * standardized.  The data format and meaning is defined by the vendor.
 *
 * The usage is as follows :
 *
 *   struct mse_vendor_data_s
 *   {
 *     uint16_t cmd;
 *     uint16_t len;
 *     uint16_t data;
 *     ... ... ...
 *   };
 *
 *   struct mse_vendor_data_s cmd_data;
 *   cmd_data.cmd = VENDOR_CMD_ID;
 *   cmd_data.data = 12;
 *
 *   struct mouse_vendor_cmd_s *ioctl;
 *   ioctl = malloc(sizeof(*ioctl) + sizeof(struct mse_vendor_data_s));
 *   ioctl->len = sizeof(struct mse_vendor_data_s);
 *   memcpy(ioctl->data, &cmd_data, sizeof(struct mse_vendor_data_s));
 *
 *   ioctl(file, MSIOC_VENDOR, ioctl);
 *
 */

struct mouse_vendor_cmd_s
{
  size_t len;
  char data[1];
};

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
  int16_t wheel;    /* Mouse wheel position */
};

/* This structure is for mouse lower half driver */

struct mouse_lowerhalf_s
{
  FAR void *priv;  /* Save the upper half pointer */

  /**************************************************************************
   * Name: control
   *
   * Description:
   *   Users can use this interface to implement custom IOCTL.
   *
   * Arguments:
   *   lower   - The instance of lower half of mouse device.
   *   cmd     - User defined specific command.
   *   arg     - Argument of the specific command.
   *
   * Return Value:
   *   Zero(OK) on success; a negated errno value on failure.
   *   -ENOTTY - The command is not supported.
   **************************************************************************/

  CODE int (*control)(FAR struct mouse_lowerhalf_s *lower,
                      int cmd, unsigned long arg);

  /**************************************************************************
   * Name: open
   *
   * Description:
   *   This function pointer is used to open a connection to the mouse driver
   *   instance. It initializes the mouse and prepares it for subsequent
   *   interactions with the user. This function typically sets up the state
   *   of the driver and allocates any necessary resources.
   *
   * Input Parameters:
   *   lower  - A pointer to the instance of the lower half mouse driver.
   *   filep  - A pointer to the file structure representing the user.
   *
   * Returned Value:
   *   It returns zero (OK) on success; a negative errno value on failure.
   **************************************************************************/

  CODE int (*open)(FAR struct mouse_lowerhalf_s *lower);

  /**************************************************************************
   * Name: close
   *
   * Description:
   *   This function pointer is used to close the connection to the mouse
   *   driver instance. It performs any necessary cleanup operations, such as
   *   releasing resources and resetting the state of the mouse driver,
   *   before ending theinteraction with the user.
   *
   * Input Parameters:
   *   lower  - A pointer to the instance of the lower half mouse driver.
   *   filep  - A pointer to the file structure representing the user closing
   *            the mouse connection.
   *
   * Returned Value:
   *   Returns zero (OK) on success; a negative errno value on failure.
   **************************************************************************/

  CODE int (*close)(FAR struct mouse_lowerhalf_s *lower);
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

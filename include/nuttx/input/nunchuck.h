/****************************************************************************
 * include/nuttx/input/nunchuck.h
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

/* This header file provides definition for Nintendo Wii Nunchuck joystick
 * interface.  The Nunchuck joystick  provides X/Y  positional data as
 * integer values. The analog positional data may also be accompanied by
 * discrete button data.
 *
 * The nunchuck joystick driver exports a standard character driver
 * interface. By convention, the nunchuck joystick is registered as an input
 * device at /dev/nunchuckN where N uniquely identifies the driver instance.
 */

#ifndef __INCLUDE_NUTTX_INPUT_NUNCHUCK_H
#define __INCLUDE_NUTTX_INPUT_NUNCHUCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/input/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NUNCHUCK_ADDR              0x52    /* Nunchuck at address 0x52 */
#define NUNCHUCK_I2C_FREQ          100000  /* 100Khz */

/* Joystick Interface *******************************************************/

/* These definitions provide the meaning of all of the bits that may be
 * reported in the nunchuck_buttonset_t bitset.
 */

#define NUNCHUCK_BUTTON(n)         ((n)-1)              /* Bit n-1: Button n, n=1..2 */
#define NUNCHUCK_BUTTON_Z          (0)                  /* Bit 0: Button Z */
#define NUNCHUCK_BUTTON_C          (1)                  /* Bit 1: Button C */
#define NUNCHUCK_NBUTTONS          (2)                  /* Total number of buttons */

#define NUNCHUCK_BUTTON_Z_BIT      (1 << NUNCHUCK_BUTTON_Z) /* 1:Button C pressed */
#define NUNCHUCK_BUTTON_C_BIT      (1 << NUNCHUCK_BUTTON_C) /* 1:Button Z pressed */
#define NUNCHUCK_BUTTONS_ALL       0x3                      /* Set of all buttons */

/* Typical usage */

#define NUNCHUCK_BUTTON_SELECT     NUNCHUCK_BUTTON_C
#define NUNCHUCK_BUTTON_FIRE       NUNCHUCK_BUTTON_Z
#define NUNCHUCK_BUTTON_JUMP       0

#define NUNCHUCK_BUTTON_SELECT_BIT NUNCHUCK_BUTTON_C_BIT
#define NUNCHUCK_BUTTON_FIRE_BIT   NUNCHUCK_BUTTON_Z_BIT
#define NUNCHUCK_BUTTON_JUMP_BIT   0

/* IOCTL commands
 *
 * Nunchuck joystick drivers do not support the character driver write() or
 * seek() methods.  The remaining driver methods behave as follows:
 *
 * 1) The read() method will always return a single value of size
 *    struct nunchuck_sample_s represent the current joystick positional and
 *    the state of all joystick buttons. read() never blocks.  X an Y
 *    position data is raw converted data.
 *     Zeroing and scaling must be performed by the application.
 * 2) The ioctl() method supports the commands documented below:
 */

/* Command:     NUNCHUCKIOC_SUPPORTED
 * Description: Report the set of button events supported by the hardware;
 * Argument:    A pointer to writeable integer value in which to return the
 *              set of supported buttons.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define NUNCHUCKIOC_SUPPORTED  _NUNCHUCKIOC(0x0001)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This type is a bit set that contains the state of all analog joystick
 * buttons.
 */

typedef uint8_t nunchuck_buttonset_t;

/* This structure is returned by read() and provides the sample state of the
 * nunchuck joystick.
 *
 * NOTE: that this structure is equivalent to the struct mouse_report_s
 * structure (with no wheel) defined in include/nuttx/input/mouse.h and can
 * be used interchangeably in certain contexts.
 */

struct nunchuck_sample_s
{
  /* Compatible with analog joystick */

  nunchuck_buttonset_t nck_buttons; /* State of all buttons */
                                    /* Possibly padded with 1 byte here */
  int16_t js_x;                     /* X/horizontal position */
  int16_t js_y;                     /* Y/vertical position */

  /* Specific of the Nunchuck */

  int8_t  acc_x;                    /* Accelerometer X */
  int8_t  acc_y;                    /* Accelerometer Y */
  int8_t  acc_z;                    /* Accelerometer Z */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

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

/****************************************************************************
 * Name: nunchuck_register
 *
 * Description:
 *   Register the Nunchuck character driver as the specified device.
 *
 * Input Parameters:
 *   devname - The name of the Nunchuck joystick device to be registered.
 *     This should be a string of the form "/dev/nunchuckN" where N is the
 *     minor device number.
 *   i2c - An instance of the platform-specific I2C connected to Nunchuck.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

struct i2c_master_s;
int nunchuck_register(FAR const char *devname, FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_INPUT_NUNCHUCK_H */

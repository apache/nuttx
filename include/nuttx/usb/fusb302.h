/****************************************************************************
 * include/nuttx/usb/fusb302.h
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

#ifndef __INCLUDE_NUTTX_USB_FUSB302_H
#define __INCLUDE_NUTTX_USB_FUSB302_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default FUSB302 I2C ID */

#define FUSB302_I2C_ADDR      0x22

/* IOCTL Commands ***********************************************************/

#define USBCIOC_READ_DEVID    _USBCIOC(0x0001)  /* Arg: uint8_t* pointer */
#define USBCIOC_SETUP         _USBCIOC(0x0002)  /* Arg: uint8_t* pointer */
#define USBCIOC_READ_STATUS   _USBCIOC(0x0005)  /* Arg: uint8_t* pointer*/
#define USBCIOC_RESET         _USBCIOC(0x0007)  /* Arg: None */
#define USBCIOC_SET_POLL_MODE _USBCIOC(0x0008)  /* Arg: uint8_t value */

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum fusb302_mode_e
{
  MODE_NONE = 0,
  MODE_SRC_POLL_AUTO,
  MODE_SNK_POLL_AUTO,
  MODE_DRP_POLL_AUTO,

  /* Revisit - FUSB302 supports other modes not implemented at this time */

#if 0
  MODE_SRC_POLL_MAN,
  MODE_DRP_POLL_MAN,
  MODE_SNK_POLL_MAN,
#endif
};

enum fusb_connect_status_e
{
  NOTHING_CONNECTED = 0,
  TOGGLING,
  SRC_DEVICE_CONNECTED,
  SNK_DEVICE_CONNECTED,
  AUDIO_ACCESSORY_CONNECTED,
  SNK_DETACH_DETECTED,
  SRC_DETACH_DETECTED,
  BC_LEVEL_CHANGE_REQUSTED,
  UNKNOWN_CONNECTED,
  CONNECT_ERROR,
};

struct fusb302_config_s
{
  /* Device characterization */

  int irq;

  CODE int  (*irq_attach)(FAR struct fusb302_config_s *state, xcpt_t isr,
                          FAR void *arg);
  CODE void (*irq_enable)(FAR struct fusb302_config_s *state, bool enable);
  CODE void (*irq_clear)(FAR struct fusb302_config_s *state);

  FAR struct i2c_master_s *i2c;
  uint8_t i2c_addr;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: fusb302_register
 *
 * Description:
 *   Register the FUSB302 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/usbc0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             FUSB302
 *   addr    - The I2C address of the FUSB302.
 *             The I2C address of the FUSB302 is 0x22.
 *   config  - Pointer to FUSB302 configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int fusb302_register(FAR const char *devpath,
                     FAR struct fusb302_config_s *config);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_FUSB302_H */

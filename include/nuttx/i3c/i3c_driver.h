/****************************************************************************
 * include/nuttx/i3c/i3c_driver.h
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

#ifndef _INCLUDE_NUTTX_I3C_I3C_DRIVER_H
#define _INCLUDE_NUTTX_I3C_I3C_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/i3c/device.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I3C Character Driver IOCTL Commands **************************************/

/* The I3C driver is intended to support application testing of the I3C bus.
 * The I3C driver simply wraps an instance of struct i3c_driver_s and then
 * provides the following IOCTL commands to access each method of the I3c
 * interface.
 */

/* Command:      I3CIOC_PRIV_XFERS
 * Description:  Perform an I3C transfer
 * Argument:     A reference to an instance of struct i3c_transfer_s.
 * Dependencies:
 */

#define I3CIOC_PRIV_XFERS      _I3CIOC(0x0001)

/* Command:      I3CIOC_EN_IBI
 * Description:  Perform an I3C bus IBI enabled operation
 *
 * Argument:     None
 * Dependencies:
 */

#define I3CIOC_EN_IBI         _I3CIOC(0x0002)

/* Command:      I3CIOC_DIS_IBI
 * Description:  Perform an I3C bus IBI disabled operation
 *
 * Argument:     None
 * Dependencies:
 */

#define I3CIOC_DIS_IBI         _I3CIOC(0x0003)

/* Command:      I3CIOC_REQ_IBI
 * Description:  Perform an I3C bus IBI requested operation
 *
 * Argument:     None
 * Dependencies:
 */

#define I3CIOC_REQ_IBI         _I3CIOC(0x0004)

/* Command:      I3CIOC_FREE_IBI
 * Description:  Perform an I3C bus IBI free operation
 *
 * Argument:     None
 * Dependencies:
 */

#define I3CIOC_FREE_IBI         _I3CIOC(0x0005)

/* Command:      I3CIOC_GET_DEVINFO
 * Description:  Get an I3C bus dev infomation
 *
 * Argument:     None
 * Dependencies:
 */

#define I3CIOC_GET_DEVINFO      _I3CIOC(0x0006)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i3c_master_s;
struct i3c_priv_xfer;
struct i3c_ibi_setup;
struct i3c_master_controller;

/* This structure is used to communicate with the I3C character driver in
 * order to perform IOCTL transfers.
 */

struct i3c_transfer_s
{
  /* manufID and partID is used to attach  to i3c_dev_desc, it's */

  uint16_t manufid;
  uint16_t partid;

  /* Number of messages in the array. */

  size_t nxfers;

  /* Array of I3C messages for the transfer */

  FAR struct i3c_priv_xfer *xfers;

  FAR struct i3c_ibi_setup *req;

  /* I3C device information */

  FAR struct i3c_device_info *info;
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: i3c_register
 *
 * Description:
 *   Create and register the I3C character driver.
 *
 *   The I3C character driver is a simple character driver that supports I3C
 *   transfers.  The intent of this driver is to support I3C testing.  It is
 *   not suitable for use in any real driver application.
 *
 * Input Parameters:
 *   master - An instance of the lower half I3C core driver.
 *   bus    - The I3C bus number.  This will be used as the I3C device minor
 *   number.The I3C character device will be registered as /dev/i3cN
 *   where N is the minor number.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I3C_DRIVER
int i3c_register(FAR struct i3c_master_controller *master, int bus);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* _INCLUDE_NUTTX_I3C_I3C_DRIVER_H */

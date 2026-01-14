/****************************************************************************
 * include/nuttx/usb/stusb4500.h
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

/* Definitions, names and concept inspiered by
 * https://github.com/ardnew/STUSB4500/blob/master/LICENSE
 */

#ifndef __INCLUDE_NUTTX_USB_STUSB4500_H
#define __INCLUDE_NUTTX_USB_STUSB4500_H

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

/* IOCTL Commands ***********************************************************/

#define USBCIOC_READ_DEVID      _USBCIOC(0x0001)  /* Arg: uint8_t* pointer */
#define USBCIOC_READ_PD_STATUS  _USBCIOC(0x0002)  /* Arg: uint8_t* pointer */
#define USBCIOC_SET_PWR         _USBCIOC(0x0003)  /* Arg: uint8_t* pointer */
#define USBCIOC_READ_PWR        _USBCIOC(0x0004)  /* Arg: uint8_t* pointer */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef union
{
  uint8_t bytes[4];
  struct
  {
    uint32_t operational_current        : 10;
    uint32_t voltage                    : 10;
    uint8_t  reserved_22_20             :  3;
    uint8_t  fast_role_req_cur          :  2;  /* must be set to 0 in 2.0 */
    uint8_t  dual_role_data             :  1;
    uint8_t  usb_communications_capable :  1;
    uint8_t  unconstrained_power        :  1;
    uint8_t  higher_capability          :  1;
    uint8_t  dual_role_power            :  1;
    uint8_t  fixed_supply               :  2;
  } fix;
  struct
  {
    uint32_t operating_current : 10;
    uint32_t min_voltage       : 10;
    uint32_t max_voltage       : 10;
    uint8_t  variablesupply    :  2;
  } var;
  struct
  {
    uint32_t operating_power : 10;
    uint32_t min_voltage     : 10;
    uint32_t max_voltage     : 10;
    uint8_t  battery         :  2;
  } bat;
}
USB_PD_SNK_PDO_TYPEDEF end_packed_struct;

typedef union
{
  uint8_t bytes[4];
  struct
  {
    uint32_t maxcurrent        : 10; /* bits 9..0 */
    uint32_t operatingcurrent  : 10;
    uint8_t  reserved_22_20    :  3;
    uint8_t  unchunkedmess_sup :  1;
    uint8_t  usbsuspend        :  1;
    uint8_t  usbcomcap         :  1;
    uint8_t  capamismatch      :  1;
    uint8_t  giveback          :  1;
    uint8_t  object_pos        :  3; /* bits 30..28 (3-bit) */
    uint8_t  reserved_31       :  1; /* bits 31 */
  } b;
}
STUSB_GEN1S_RDO_REG_STATUS_REGTYPEDEF end_packed_struct;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stusb4500_register
 *
 * Description:
 *   Register the STUSB4500 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g.,
 *             "/dev/stusb4500_0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             STUSB4500
 *   addr    - The I2C address of the STUSB4500.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stusb4500_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_FUSB303_H */

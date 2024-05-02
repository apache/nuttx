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
    uint32_t Operational_Current        : 10;
    uint32_t Voltage                    : 10;
    uint8_t  Reserved_22_20             :  3;
    uint8_t  Fast_Role_Req_cur          :  2;  /* must be set to 0 in 2.0 */
    uint8_t  Dual_Role_Data             :  1;
    uint8_t  USB_Communications_Capable :  1;
    uint8_t  Unconstrained_Power        :  1;
    uint8_t  Higher_Capability          :  1;
    uint8_t  Dual_Role_Power            :  1;
    uint8_t  Fixed_Supply               :  2;
  } fix;
  struct
  {
    uint32_t Operating_Current : 10;
    uint32_t Min_Voltage       : 10;
    uint32_t Max_Voltage       : 10;
    uint8_t  VariableSupply    :  2;
  } var;
  struct
  {
    uint32_t Operating_Power : 10;
    uint32_t Min_Voltage     : 10;
    uint32_t Max_Voltage     : 10;
    uint8_t  Battery         :  2;
  } bat;

} USB_PD_SNK_PDO_TypeDef __attribute__((__packed__));

typedef union
{
  uint8_t bytes[4];
  struct
  {
    uint32_t MaxCurrent        : 10; // Bits 9..0
    uint32_t OperatingCurrent  : 10;
    uint8_t  reserved_22_20    :  3;
    uint8_t  UnchunkedMess_sup :  1;
    uint8_t  UsbSuspend        :  1;
    uint8_t  UsbComCap         :  1;
    uint8_t  CapaMismatch      :  1;
    uint8_t  GiveBack          :  1;
    uint8_t  Object_Pos        :  3; // Bits 30..28 (3-bit)
    uint8_t  reserved_31       :  1; // Bits 31

  } b;
} STUSB_GEN1S_RDO_REG_STATUS_RegTypeDef __attribute__((__packed__));

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
 *   devpath - The full path to the driver to register. E.g., "/dev/stusb4500_0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             STUSB4500
 *   addr    - The I2C address of the STUSB4500.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stusb4500_register(FAR const char* devpath, FAR struct i2c_master_s* i2c,
                       uint8_t addr);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_FUSB303_H */

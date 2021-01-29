/****************************************************************************
 * include/nuttx/power/battery_ioctl.h
 * NuttX Battery IOCTLs definition
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

#ifndef __INCLUDE_NUTTX_POWER_BATTERY_IOCTL_H
#define __INCLUDE_NUTTX_POWER_BATTERY_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All battery-related IOCTL commands must be defined in this header file
 * in order to assure that every IOCTL command is unique and will not be
 * aliased.
 */

#define BATIOC_STATE         _BATIOC(0x0001)
#define BATIOC_HEALTH        _BATIOC(0x0002)
#define BATIOC_ONLINE        _BATIOC(0x0003)
#define BATIOC_VOLTAGE       _BATIOC(0x0004)
#define BATIOC_CURRENT       _BATIOC(0x0005)
#define BATIOC_INPUT_CURRENT _BATIOC(0x0006)
#define BATIOC_CAPACITY      _BATIOC(0x0007)
#define BATIOC_OPERATE       _BATIOC(0x0008)
#define BATIOC_CELLVOLTAGE   _BATIOC(0x0009)
#define BATIOC_TEMPERATURE   _BATIOC(0x000A)
#define BATIOC_BALANCE       _BATIOC(0x000B)
#define BATIOC_SHUTDOWN      _BATIOC(0x000C)
#define BATIOC_SETLIMITS     _BATIOC(0x000D)
#define BATIOC_CHGDSG        _BATIOC(0x000E)
#define BATIOC_CLEARFAULTS   _BATIOC(0x000F)
#define BATIOC_COULOMBS      _BATIOC(0x0010)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct batio_operate_msg_s
{
  uint8_t operate_type; /* Really enum batio_operate_e */
  union
  {
    uint32_t u32;
    uint8_t  u8[8];
  };
};

#if defined(CONFIG_I2C_BQ2429X)
enum batio_operate_e
{
  BATIO_OPRTN_NOP = 0,
  BATIO_OPRTN_BOOST,
  BATIO_OPRTN_CHARGE,
  BATIO_OPRTN_EN_TERM,
  BATIO_OPRTN_HIZ,
  BATIO_OPRTN_SYSOFF,
  BATIO_OPRTN_SYSON,
  BATIO_OPRTN_RESET,
  BATIO_OPRTN_WDOG,
  BATIO_OPRTN_END
};
#endif

#endif /* __INCLUDE_NUTTX_POWER_BATTERY_IOCTL_H */

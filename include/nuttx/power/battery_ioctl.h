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
#define BATIOC_CHIPID        _BATIOC(0x0011)
#define BATIOC_GET_VOLTAGE   _BATIOC(0x0012)
#define BATIOC_VOLTAGE_INFO  _BATIOC(0x0013)
#define BATIOC_GET_PROTOCOL  _BATIOC(0x0014)

/* Special input values for BATIOC_INPUT_CURRENT that may optionally
 * be supported by lower-half driver:
 */

#define BATTERY_INPUT_CURRENT_EXT_LIM   (-1) /* External input current limit */

/* The change mask definition used to set the mask. */

#define BATTERY_STATE_CHANGED           (1U << 0)
#define BATTERY_HEALTH_CHANGED          (1U << 1)
#define BATTERY_ONLINE_CHANGED          (1U << 2)
#define BATTERY_VOLTAGE_CHANGED         (1U << 3)
#define BATTERY_CURRENT_CHANGED         (1U << 4)
#define BATTERY_CAPACITY_CHANGED        (1U << 5)
#define BATTERY_CELLVOLTAGE_CHANGED     (1U << 6)
#define BATTERY_TEMPERATURE_CHANGED     (1U << 7)
#define BATTERY_COULOMBS_CHANGED        (1U << 8)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Battery status */

enum battery_status_e
{
  BATTERY_UNKNOWN = 0, /* Battery state is not known */
  BATTERY_FAULT,       /* Charger reported a fault, get health for more info */
  BATTERY_IDLE,        /* Not full, not charging, not discharging */
  BATTERY_FULL,        /* Full, not discharging */
  BATTERY_CHARGING,    /* Not full, charging */
  BATTERY_DISCHARGING  /* Probably not full, discharging */
};

/* Battery Health status */

enum battery_health_e
{
  BATTERY_HEALTH_UNKNOWN = 0,   /* Battery health state is not known */
  BATTERY_HEALTH_GOOD,          /* Battery is in good condiction */
  BATTERY_HEALTH_DEAD,          /* Battery is dead, nothing we can do */
  BATTERY_HEALTH_OVERHEAT,      /* Battery is over recommended temperature */
  BATTERY_HEALTH_OVERVOLTAGE,   /* Battery voltage is over recommended level */
  BATTERY_HEALTH_UNDERVOLTAGE,  /* Battery monitor reported an unspecified failure */
  BATTERY_HEALTH_OVERCURRENT,   /* Battery monitor reported an overcurrent event */
  BATTERY_HEALTH_SHORT_CIRCUIT, /* Battery monitor reported a short circuit event */
  BATTERY_HEALTH_UNSPEC_FAIL,   /* Battery charger reported an unspected failure */
  BATTERY_HEALTH_COLD,          /* Battery is under recommended temperature */
  BATTERY_HEALTH_WD_TMR_EXP,    /* Battery WatchDog Timer Expired */
  BATTERY_HEALTH_SAFE_TMR_EXP,  /* Battery Safety Timer Expired */
  BATTERY_HEALTH_DISCONNECTED   /* Battery is not connected */
};

/* battery charge protocol type */

enum battery_protocol_e
{
  BATTERY_PROTOCOL_QC3P0 = 1 << 0,      /* Battery charge protocol of adapter is QC 3.0 */
  BATTERY_PROTOCOL_TX_XIAOMI = 1 << 1,  /* Battery charge protocol of TX is xiaomi standard */
};

/* Battery operation message */

struct batio_operate_msg_s
{
  uint8_t operate_type; /* Really enum batio_operate_e */
  union
  {
    uint32_t u32;
    uint8_t  u8[8];
  };
};

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
  BATIO_OPRTN_SHIPMODE,
  BATIO_OPRTN_CUTOFF_CURRENT,
  BATIO_OPRTN_END
};

#endif /* __INCLUDE_NUTTX_POWER_BATTERY_IOCTL_H */

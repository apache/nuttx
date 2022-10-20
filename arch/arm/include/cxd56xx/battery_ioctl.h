/****************************************************************************
 * arch/arm/include/cxd56xx/battery_ioctl.h
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_BATTERY_IOCTL_H
#define __ARCH_ARM_INCLUDE_CXD56XX_BATTERY_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* ioctl commands */

#define BATIOC_GET_CHGVOLTAGE    _BATIOC(0x0020)
#define BATIOC_GET_CHGCURRENT    _BATIOC(0x0021)
#define BATIOC_GET_RECHARGEVOL   _BATIOC(0x0022)
#define BATIOC_SET_RECHARGEVOL   _BATIOC(0x0023)
#define BATIOC_GET_COMPCURRENT   _BATIOC(0x0024)
#define BATIOC_SET_COMPCURRENT   _BATIOC(0x0025)
#define BATIOC_GET_TEMPTABLE     _BATIOC(0x0026)
#define BATIOC_SET_TEMPTABLE     _BATIOC(0x0027)
#define BATIOC_GET_CURRENT       _BATIOC(0x0028)

#define BATIOC_MONITOR_ENABLE    _BATIOC(0x0030)
#define BATIOC_MONITOR_STATUS    _BATIOC(0x0031)
#define BATIOC_MONITOR_SET       _BATIOC(0x0032)
#define BATIOC_MONITOR_GET       _BATIOC(0x0033)

#define BATIOC_DEBUG             _BATIOC(0x00db)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct battery_temp_table_s
{
  int T60; /* 60 degree C */
  int T45; /* 45 degree C */
  int T10; /* 10 degree C */
  int T00; /*  0 degree C */
};

struct bat_monitor_enable_s
{
  int on;
  int interval;
  int threshold_volt;
  int threshold_current;
};

struct bat_monitor_status_s
{
  int run;
  int index;
  int latest;
  int totalwatt;
  int totaltime;
};

struct bat_monitor_set_s
{
  int clearbuf;
  int clearsum;
};

struct bat_monitor_rec_s
{
  uint16_t index;
  uint16_t timestamp;
  uint16_t voltage;
  int16_t  current;
};

struct bat_monitor_log_s
{
  struct bat_monitor_rec_s *rec;
  int index;
  int size;
};

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_BATTERY_IOCTL_H */

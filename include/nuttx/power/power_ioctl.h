/****************************************************************************
 * include/nuttx/power/power_ioctl.h
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

#ifndef __INCLUDE_NUTTX_POWER_POWER_IOCTL_H
#define __INCLUDE_NUTTX_POWER_POWER_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All power-related IOCTL commands must be defined in this header file
 * in order to assure that every IOCTL command is unique and will not be
 * aliased.
 */

/* The power driver sub-system uses the standard character driver framework.
 * However, since the driver is a devices control interface rather than a
 * data transfer interface, the majority of the functionality is implemented
 * in driver ioctl calls.  Standard ioctl commands are listed below:
 */

#define PWRIOC_START          _PWRIOC(1)
#define PWRIOC_STOP           _PWRIOC(2)
#define PWRIOC_SET_MODE       _PWRIOC(3)
#define PWRIOC_SET_LIMITS     _PWRIOC(4)
#define PWRIOC_GET_STATE      _PWRIOC(5)
#define PWRIOC_SET_STATE      _PWRIOC(6)
#define PWRIOC_GET_FAULT      _PWRIOC(7)
#define PWRIOC_SET_FAULT      _PWRIOC(8)
#define PWRIOC_CLEAN_FAULT    _PWRIOC(9)
#define PWRIOC_SET_PARAMS     _PWRIOC(10)

#define PWR_FIRST          0x0001          /* First common command */
#define PWR_NCMDS          10              /* Number of common commands */

/* User defined ioctl commands are also supported. These will be forwarded
 * by the upper-half driver to the lower-half driver via the ioctl()
 * method of the lower-half interface.  However, the lower-half driver
 * must reserve a block of commands as follows in order prevent IOCTL
 * command numbers from overlapping.
 */

/* See include/nuttx/power/husb238.h */

#define PWR_HUSB238_FIRST  (PWR_FIRST + PWR_NCMDS)
#define PWS_HUSB238_NCMDS  6

#endif /* __INCLUDE_NUTTX_POWER_POWER_IOCTL_H */

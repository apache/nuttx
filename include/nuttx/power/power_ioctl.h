/****************************************************************************
 * include/nuttx/power/power_ioctl.h
 * NuttX Power-Related IOCTLs definitions
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

#endif /* __INCLUDE_NUTTX_POWER_POWER_IOCTL_H */

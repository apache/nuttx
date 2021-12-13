/****************************************************************************
 * include/nuttx/motor/motor_ioctl.h
 * NuttX Motor-Related IOCTLs definitions
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

#ifndef __INCLUDE_NUTTX_MOTOR_MOTOR_IOCTL_H
#define __INCLUDE_NUTTX_MOTOR_MOTOR_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All foc-related IOCTL commands must be defined in this header file
 * in order to assure that every IOCTL command is unique and will not be
 * aliased.
 */

#define MTRIOC_START          _MTRIOC(1)
#define MTRIOC_STOP           _MTRIOC(2)
#define MTRIOC_GET_STATE      _MTRIOC(3)
#define MTRIOC_CLEAR_FAULT    _MTRIOC(4)
#define MTRIOC_SET_PARAMS     _MTRIOC(5)
#define MTRIOC_SET_CONFIG     _MTRIOC(6)
#define MTRIOC_GET_INFO       _MTRIOC(7)
#define MTRIOC_SET_MODE       _MTRIOC(8)
#define MTRIOC_SET_LIMITS     _MTRIOC(9)
#define MTRIOC_SET_FAULT      _MTRIOC(10)
#define MTRIOC_GET_FAULT      _MTRIOC(11)
#define MTRIOC_CALIBRATE      _MTRIOC(12)
#define MTRIOC_SELFTEST       _MTRIOC(13)
#define MTRIOC_SET_CALIBDATA  _MTRIOC(14)

#endif /* __INCLUDE_NUTTX_MOTOR_MOTOR_IOCTL_H */

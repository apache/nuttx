/****************************************************************************
 * include/nuttx/motor/stepper_ioctl.h
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

#ifndef __INCLUDE_NUTTX_MOTOR_STEPPER_IOCTL_H
#define __INCLUDE_NUTTX_MOTOR_STEPPER_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STEPIOC_IDLE            _STEPIOC(1)
#define STEPIOC_CLEAR_FAULT     _STEPIOC(2)
#define STEPIOC_MICROSTEPPING   _STEPIOC(3)
#define STEPIOC_SET_CURRENT_POS _STEPIOC(4)

#endif /* __INCLUDE_NUTTX_MOTOR_STEPPER_IOCTL_H */

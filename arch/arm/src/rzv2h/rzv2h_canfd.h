/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_canfd.h
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

#ifndef __ARCH_ARM_SRC_RZV2H_RZV2H_CANFD_H
#define __ARCH_ARM_SRC_RZV2H_RZV2H_CANFD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/can/can.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_RZV2H_CANFD_SOCKET
/****************************************************************************
 * Name: rzv2h_canfd_sock_initialize
 *
 * Description:
 *   Register one CANFD channel as the SocketCAN network interface
 *   can<channel>. The hardware is set up later, when the interface is
 *   brought up.
 *
 * Input Parameters:
 *   channel - CANFD channel number (0-5)
 *
 * Returned Value:
 *   OK on success, a negative errno on failure
 *
 ****************************************************************************/

int rzv2h_canfd_sock_initialize(int channel);
#else
/****************************************************************************
 * Name: rzv2h_canfd_initialize
 *
 * Description:
 *   Return the NuttX CAN device of one CANFD channel. The hardware is set
 *   up later, in rzv2h_canfd_setup(), when the device is first opened.
 *
 * Input Parameters:
 *   channel - CANFD channel number (0-5)
 *
 * Returned Value:
 *   CAN device instance on success, NULL on failure
 *
 ****************************************************************************/

struct can_dev_s *rzv2h_canfd_initialize(int channel);
#endif

#endif /* __ARCH_ARM_SRC_RZV2H_RZV2H_CANFD_H */

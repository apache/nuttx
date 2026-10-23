/****************************************************************************
 * drivers/wireless/ieee80211/morse/mmport.h
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

/* Port definitions required by the Morse Micro mmosal.h header */

#ifndef __DRIVERS_WIRELESS_IEEE80211_MORSE_MMPORT_H
#define __DRIVERS_WIRELESS_IEEE80211_MORSE_MMPORT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MMPORT_BREAKPOINT()  __asm volatile ("bkpt 0\n\t")
#define MMPORT_GET_LR()      return_address(0)
#define MMPORT_GET_PC(_a)    __asm volatile ("mov %0, pc" : "=r" (_a))
#define MMPORT_MEM_SYNC()    __sync_synchronize()

#endif /* __DRIVERS_WIRELESS_IEEE80211_MORSE_MMPORT_H */

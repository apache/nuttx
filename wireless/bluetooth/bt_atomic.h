/****************************************************************************
 * wireless/bluetooth/bt_atomic.h
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

#ifndef __WIRELESS_BLUETOOTH_BT_ATOMIC_H
#define __WIRELESS_BLUETOOTH_BT_ATOMIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define bt_atomic_set(ptr, value)        atomic_set(ptr, value);
#define bt_atomic_get(ptr)               atomic_read(ptr)
#define bt_atomic_testbit(ptr, bitno)    ((atomic_read(ptr) & (1 << (bitno))) != 0)
#define bt_atomic_incr(ptr)              atomic_fetch_add(ptr, 1)
#define bt_atomic_decr(ptr)              atomic_fetch_sub(ptr, 1)
#define bt_atomic_setbit(ptr, bitno)     atomic_fetch_or(ptr, (1 << (bitno)))
#define bt_atomic_clrbit(ptr, bitno)     atomic_fetch_and(ptr, ~(1 << (bitno)))
#define bt_atomic_testsetbit(ptr, bitno) ((atomic_fetch_or(ptr, (1 << (bitno))) & (1 << (bitno))) != 0)
#define bt_atomic_testclrbit(ptr, bitno) ((atomic_fetch_and(ptr, ~(1 << (bitno))) & (1 << (bitno))) != 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef atomic_t bt_atomic_t;

#endif /* __WIRELESS_BLUETOOTH_BT_ATOMIC_H */

/****************************************************************************
 * wireless/bluetooth/bt_atomic.h
 * Linux like atomic operations
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

#define bt_atomic_set(ptr, value)     (*(ptr) = (value))
#define bt_atomic_get(ptr)            (*(ptr))
#define bt_atomic_testbit(ptr, bitno) ((*(ptr) & (1 << (bitno))) != 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint8_t bt_atomic_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

bt_atomic_t bt_atomic_incr(FAR bt_atomic_t *ptr);
bt_atomic_t bt_atomic_decr(FAR bt_atomic_t *ptr);
bt_atomic_t bt_atomic_setbit(FAR bt_atomic_t *ptr, bt_atomic_t bitno);
bt_atomic_t bt_atomic_clrbit(FAR bt_atomic_t *ptr, bt_atomic_t bitno);

bool bt_atomic_testsetbit(FAR bt_atomic_t *ptr, bt_atomic_t bitno);
bool bt_atomic_testclrbit(FAR bt_atomic_t *ptr, bt_atomic_t bitno);

#endif /* __WIRELESS_BLUETOOTH_BT_ATOMIC_H */

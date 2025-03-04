/****************************************************************************
 * drivers/serial/uart_bth5.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

#include <debug.h>
#include <string.h>

#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/wireless/bluetooth/bt_slip.h>

#include <nuttx/serial/uart_bth4.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int uart_bth5_register(FAR const char *path, FAR struct bt_driver_s *drv)
{
  return uart_bth4_register(path, bt_slip_register(drv));
}

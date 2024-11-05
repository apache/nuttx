/****************************************************************************
 * drivers/wireless/bluetooth/bt_uart_generic.c
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

/* Generic UART based Bluetooth driver */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "bt_uart.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btuart_create
 *
 *   Create the UART-based bluetooth device.
 *
 * Input Parameters:
 *   lower - an instance of the lower half driver interface
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int btuart_create(FAR const struct btuart_lowerhalf_s *lower,
                  FAR struct bt_driver_s **driver)
{
  FAR struct btuart_upperhalf_s *upper;

  wlinfo("lower %p\n", lower);

  if (lower == NULL)
    {
      wlerr("ERROR: btuart lower half is NULL\n");
      return -ENODEV;
    }

  /* Allocate a new instance of the upper half driver state structure */

  upper = (FAR struct btuart_upperhalf_s *)
    kmm_zalloc(sizeof(struct btuart_upperhalf_s));

  if (upper == NULL)
    {
      wlerr("ERROR: Failed to allocate upper-half state\n");
      return -ENOMEM;
    }

  /* Initialize the upper half driver state */

  upper->dev.head_reserve = H4_HEADER_SIZE;
  upper->dev.open         = btuart_open;
  upper->dev.send         = btuart_send;
  upper->dev.close        = btuart_close;
  upper->dev.ioctl        = btuart_ioctl;
  upper->lower            = lower;

  *driver = &upper->dev;
  return OK;
}

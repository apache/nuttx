/****************************************************************************
 * drivers/vhost/vhost-i2c.h
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

#ifndef __DRIVERS_VHOST_VHOST_I2C_H
#define __DRIVERS_VHOST_VHOST_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_DRIVERS_VHOST_I2C

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_register_i2c_driver
 *
 * Description:
 *   Register the vhost (device-side) I2C driver with the vhost core so it
 *   can be bound to a matching VirtIO I2C vdev.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int vhost_register_i2c_driver(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_VHOST_I2C */

#endif /* __DRIVERS_VHOST_VHOST_I2C_H */

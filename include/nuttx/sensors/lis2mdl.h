/****************************************************************************
 * include/nuttx/sensors/lis2mdl.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
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

#include <nuttx/sensors/ioctl.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s; /* Forward reference */

typedef int (*lis2mdl_attach)(xcpt_t, FAR void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lis2mdl_register
 *
 * Description:
 *   Register the LIS2MDL device as a UORB sensor.
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the LIS2MDL.
 *   addr    - The I2C address of the LIS2MDL. Should always be 0x1e.
 *   devno   - The device number to use for the topic (i.e. /dev/mag0)
 *   attach  - A function which is called by this driver to attach the
 *             LIS2MDL interrupt handler to an IRQ. Pass NULL to operate
 *             in polling mode. This function should return 0 on success
 *             and a negated error code otherwise.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lis2mdl_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr,
                     lis2mdl_attach attach);

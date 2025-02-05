/****************************************************************************
 * include/nuttx/sensors/lsm6dso32.h
 *
 * Contributed by Carleton University InSpace
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

#ifndef __INCLUDE_NUTTX_SENSORS_LSM6DSO32_H
#define __INCLUDE_NUTTX_SENSORS_LSM6DSO32_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* LSM6DSO32 interrupt pins */

enum lsm6dso32_int_e
{
  LSM6DSO32_INT1 = 0, /* Interrupt pin 1 */
  LSM6DSO32_INT2 = 1, /* Interrupt pin 2 */
};

typedef int (*lsm6dso32_attach)(xcpt_t handler, FAR void *arg);

/* Configuration for the LSM6DSO32 driver. */

struct lsm6dso32_config_s
{
  enum lsm6dso32_int_e gy_int; /* Interrupt pin to use for gyro */
  enum lsm6dso32_int_e xl_int; /* Interrupt pin to use for accel */
  lsm6dso32_attach gy_attach;  /* Attach gyro interrupt (NULL for kthread) */
  lsm6dso32_attach xl_attach;  /* Attach accel interrupt (NULL for kthread) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6dso32_register
 *
 * Description:
 *   Register the LSM6DSO32 character device as a UORB sensor with accel and
 *   gyro topics. If used with interrupts and device registration fails, it
 *   is the caller's responsibility to detach the interrupt handler.
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the LSM6DSO32
 *   addr    - The I2C address of the LSM6DSO32.
 *   devno   - The device number for the UORB topics registered (i.e.
 *             sensor_accel<n>)
 *   config  - Configuration setup for interrupt-driven or polling driven
 *             data fetching. Leave `*_attach` function NULL to use kthread
 *             polling instead of interrupt handling.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm6dso32_register(FAR struct i2c_master_s *i2c, uint8_t addr,
                       uint8_t devno, struct lsm6dso32_config_s *config);

#endif // __INCLUDE_NUTTX_SENSORS_LSM6DSO32_H

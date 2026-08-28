/****************************************************************************
 * include/nuttx/sensors/lsm6ds3trc.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LSM6DS3TRC_H
#define __INCLUDE_NUTTX_SENSORS_LSM6DS3TRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* LSM6DS3TR-C interrupt pins */

enum lsm6ds3trc_int_e
{
  LSM6DS3TRC_INT1 = 0, /* Interrupt pin 1 */
  LSM6DS3TRC_INT2 = 1, /* Interrupt pin 2 */
};

typedef int (*lsm6ds3trc_attach)(xcpt_t handler, FAR void *arg);

/* Configuration for the LSM6DS3TR-C driver.
 *
 * The chip can OR both the accelerometer and gyroscope data-ready flags
 * onto a single INT pin (each has its own enable bit in that pin's
 * INT<n>_CTRL register), so one interrupt line is enough for both
 * sub-sensors -- there's no need for the two independent INT1/INT2 paths
 * a chip with a single combined DRDY bit would require two pins for.
 * Mirrors mpu6050_config_s: one shared handler, one shared HPWORK worker
 * that bursts accel+gyro+temp from one contiguous I2C read and pushes
 * whichever topic(s) are currently subscribed.
 *
 * Leave `attach` NULL to use kthread polling instead (each sub-sensor
 * polls independently at its own rate, same as always).
 */

struct lsm6ds3trc_config_s
{
  enum lsm6ds3trc_int_e int_pin; /* INT pin both DRDY_XL and DRDY_G use */
  lsm6ds3trc_attach attach;      /* Attach the interrupt (NULL for kthread) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6ds3trc_register
 *
 * Description:
 *   Register the LSM6DS3TR-C as a UORB sensor with accel and gyro topics.
 *   If used with interrupts and device registration fails, it is the
 *   caller's responsibility to detach the interrupt handler.
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the LSM6DS3TR-C
 *   addr    - The I2C address of the LSM6DS3TR-C (0x6a with SA0 low, 0x6b
 *             with SA0 high).
 *   devno   - The device number for the UORB topics registered (i.e.
 *             sensor_accel<n>)
 *   config  - Configuration setup for interrupt-driven or polling driven
 *             data fetching. Leave `*_attach` function NULL to use kthread
 *             polling instead of interrupt handling. Must not be NULL.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm6ds3trc_register(FAR struct i2c_master_s *i2c, uint8_t addr,
                        uint8_t devno,
                        FAR struct lsm6ds3trc_config_s *config);

#endif /* __INCLUDE_NUTTX_SENSORS_LSM6DS3TRC_H */

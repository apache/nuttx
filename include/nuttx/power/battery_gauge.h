/****************************************************************************
 * include/nuttx/power/battery_gauge.h
 * NuttX Battery Fuel Gauge Interfaces
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

#ifndef __INCLUDE_NUTTX_POWER_BATTERY_GAUGE_H
#define __INCLUDE_NUTTX_POWER_BATTERY_GAUGE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>

#include <stdbool.h>
#include <fixedmath.h>

#ifdef CONFIG_BATTERY_GAUGE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

/* CONFIG_BATTERY_GAUGE - Upper half battery fuel gauge driver support
 *
 * Specific, lower-half drivers will have other configuration requirements
 * such as:
 *
 *   CONFIG_I2C - I2C support *may* be needed
 *   CONFIG_I2C_MAX1704X - The MAX1704x driver must be explicitly selected.
 */

/* IOCTL Commands */

/* The upper-half battery fuel gauge driver provides a character driver
 * "wrapper"  * around the lower-half battery driver that does all of the
 * real work.
 * Since there is no real data transfer to/or from a battery, all of the
 * driver interaction is through IOCTIL commands.  The IOCTL commands
 * supported by the upper-half driver simply provide calls into the
 * lower half as summarized below:
 *
 * BATIOC_STATE - Return the current state of the battery (see
 *   enum battery_gauge_status_e).
 *   Input value:  A pointer to type int.
 * BATIOC_ONLINE - Return 1 if the battery is online; 0 if offline.
 *   Input value:  A pointer to type bool.
 * BATIOC_VOLTAGE - Return the current battery voltage.  The returned value
 *   is a fixed precision number in units of volts.
 *   Input value:  A pointer to type b16_t.
 * BATIOC_CAPACITY - Return the current battery capacity or State of Charge
 *   (SoC).  The returned value is a fixed precision percentage of the
 *   batteries full capacity.
 *   Input value:  A pointer to type b16_t.
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Battery status */

enum battery_gauge_status_e
{
  BATTERY_UNKNOWN = 0, /* Battery state is not known */
  BATTERY_IDLE,        /* Not full, not charging, not discharging */
  BATTERY_FULL,        /* Full, not discharging */
  BATTERY_CHARGING,    /* Not full, charging */
  BATTERY_DISCHARGING  /* Probably not full, discharging */
};

/* This structure defines the lower half battery interface */

struct battery_gauge_dev_s;
struct battery_gauge_operations_s
{
  /* Return the current battery state (see enum battery_gauge_status_e) */

  int (*state)(struct battery_gauge_dev_s *dev, int *status);

  /* Return true if the batter is online */

  int (*online)(struct battery_gauge_dev_s *dev, bool *status);

  /* Current battery voltage */

  int (*voltage)(struct battery_gauge_dev_s *dev, b16_t *value);

  /* Battery capacity */

  int (*capacity)(struct battery_gauge_dev_s *dev, b16_t *value);
};

/* This structure defines the battery driver state structure */

struct battery_gauge_dev_s
{
  /* Fields required by the upper-half driver */

  FAR const struct battery_gauge_operations_s *ops; /* Battery operations */
  sem_t batsem;                                     /* Enforce mutually exclusive access */

  /* Data fields specific to the lower-half driver may follow */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: battery_gauge_register
 *
 * Description:
 *   Register a lower half battery driver with the common, upper-half
 *   battery driver.
 *
 * Input Parameters:
 *   devpath - The location in the pseudo-filesystem to create the driver.
 *     Recommended standard is "/dev/bat0", "/dev/bat1", etc.
 *   dev - An instance of the battery state structure .
 *
 * Returned Value:
 *    Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int battery_gauge_register(FAR const char *devpath,
                           FAR struct battery_gauge_dev_s *dev);

/****************************************************************************
 * Name: bq27426_initialize
 *
 * Description:
 *   Initialize the bq2742 battery driver and return an instance of the
 *   lower_half interface that may be used with battery_gauge_register();
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY_GAUGE - Upper half battery fuel gauge driver support
 *   CONFIG_I2C - I2C support
 *   CONFIG_BQ27426 - And the driver must be explicitly selected.
 *
 * Input Parameters:
 *   i2c - An instance of the I2C interface to use to communicate with the
 *         bq27426
 *   addr - The I2C address of the bq27426 (Better be 0x55).
 *   frequency - The I2C frequency
 *
 * Returned Value:
 *   A pointer to the initializeed battery driver instance.  A NULL pointer
 *   is returned on a failure to initialize the bq27426 lower half.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_BQ27426)
struct i2c_master_s; /* Forward reference */

FAR struct battery_gauge_dev_s *bq27426_initialize(
                                                FAR struct i2c_master_s *i2c,
                                                uint8_t addr,
                                                uint32_t frequency);
#endif

/****************************************************************************
 * Name: max1704x_initialize
 *
 * Description:
 *   Initialize the MAX1704x battery driver and return an instance of the
 *   lower_half interface that may be used with battery_gauge_register();
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY_GAUGE - Upper half battery fuel gauge driver support
 *   CONFIG_I2C - I2C support
 *   CONFIG_I2C_MAX1704X - And the driver must be explicitly selected.
 *   CONFIG_I2C_MAX17040 or CONFIG_I2C_MAX17041 - The driver must know which
 *     chip is on the board in order to scale the voltage correctly.
 *
 * Input Parameters:
 *   i2c - An instance of the I2C interface to use to communicate with
 *         the MAX1704x.
 *   addr - The I2C address of the MAX1704x (Better be 0x36).
 *   frequency - The I2C frequency
 *
 * Returned Value:
 *   A pointer to the initializeed battery driver instance.  A NULL pointer
 *   is returned on a failure to initialize the MAX1704x lower half.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_MAX1704X)
struct i2c_master_s; /* Forward reference */

FAR struct battery_gauge_dev_s *max1704x_initialize(
                                                FAR struct i2c_master_s *i2c,
                                                uint8_t addr,
                                                uint32_t frequency);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_BATTERY_GAUGE */
#endif /* __INCLUDE_NUTTX_POWER_BATTERY_GAUGE_H */

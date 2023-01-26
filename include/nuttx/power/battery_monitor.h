/****************************************************************************
 * include/nuttx/power/battery_monitor.h
 * NuttX Battery battery manager & monitor interfaces
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

#ifndef __INCLUDE_NUTTX_POWER_BATTERY_MONITOR_H
#define __INCLUDE_NUTTX_POWER_BATTERY_MONITOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/list.h>

#include <stdbool.h>
#include <stdint.h>
#include <fixedmath.h>

#include <nuttx/mutex.h>

#ifdef CONFIG_BATTERY_MONITOR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_BATTERY_MONITOR - Upper half battery monitor driver support
 *
 * Specific, lower-half drivers will have other configuration requirements
 * such as:
 *
 *   CONFIG_I2C - I2C support *may* be needed
 *   CONFIG_I2C_BQ769X0 - The BQ769X0 driver must be explicitly selected.
 */

/* IOCTL Commands ***********************************************************/

/* The upper-half battery monitor driver provides a character driver
 * "wrapper" around the lower-half battery monitor driver that does all of
 * the real work. Since there is no real data transfer to/or from a battery,
 * all of the driver interaction is through IOCTL commands.  The IOCTL
 * commands supported by the upper-half driver simply provide calls into the
 * lower half as summarized below:
 *
 * BATIOC_STATE - Return the current state of the battery (see
 *   enum battery_status_e).
 *   Input value:  A pointer to type int.
 * BATIOC_HEALTH - Return the current health of the battery (see
 *   enum battery_health_e).
 *   Input value:  A pointer to type int.
 * BATIOC_ONLINE - Return 1 if the battery is online; 0 if offline.
 *   Input value:  A pointer to type bool.
 * BATIOC_VOLTAGE - Return the current battery pack voltage in microvolts.
 *   Input value:  A pointer to type int.
 * BATIOC_CELLVOLTAGE - Return the voltage of all cells in microvolts.
 *   Input value:  A pointer to type battery_monitor_voltage_s.
 * BATIOC_CURRENT - Return the current battery pack current in microamps.
 *   The returned data includes duration over which measurement was done, if
 *   provided by the hardware.
 *   Input value:  A pointer to type battery_monitor_current_s.
 * BATIOC_SOC - Return the state-of-charge of the battery, in percent.
 *   Input value:  A pointer to type b16_t.
 * BATIOC_COULOMBS - Return the value of the monitor's coulomb counter,
 *   if provided by the hardware
 *   Input value:  A pointer to type int.
 * BATIOC_TEMPERATURE - Return the value of any temperature sensors attached
 *   to the monitor, in microvolts.
 *   Input value:  A pointer to type battery_monitor_temperature_s.
 * BATIOC_BALANCE - Set the monitor's battery balancing switches.
 *   Input value:  A pointer to type battery_monitor_balance_s.
 * BATIOC_SHUTDOWN - Put the device into low-power shutdown mode.
 *   Input value:  None.
 * BATIOC_SETLIMITS - Set the device's safety limits for voltage, current,
 *   etc.
 *   Input value:  A pointer to type battery_monitor_limits_s.
 * BATIOC_CHGDSG - Set the device's charge and discharge switches.
 *   Input value:  A pointer to type battery_monitor_switches_s.
 * BATIOC_CLEARFAULTS - Clear the device's most recent fault.
 *   Input value:  None.
 * BATIOC_OPERATE - Perform miscellaneous, device-specific charger operation.
 *   Input value:  An uintptr_t that can hold a pointer to struct
 *   batio_operate_msg_s.
 * BATIOC_CHIPID -Get the chip id.
 *   Input value:  A pointer to type unsigned int.
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct battery_monitor_voltage_s
{
  /* Before call, contains the desired number of cells to read.
   * After call, contains the actual number of cells read.
   */

  int cell_count;

  /* Pointer to array where cell voltages should be stored.
   * MUST contain at least cell_count elements.
   * Cell voltages are stored in microvolts (uV)
   * Cell voltages in this array should be ordered according to the
   * physical layout of cells in the system.  Driver should rearrange
   * voltage values as necessary to present the user with a contiguous
   * list of cell voltages in the expected order.
   */

  FAR uint32_t *cell_voltages;
};

struct battery_monitor_temperature_s
{
  /* Before call, contains the desired number of temperature sensors to read.
   * After call, contains the actual number of temperature sensors read.
   */

  int sensor_count;

  /* Pointer to array where temperature values should be stored.
   * MUST contain at least sensor_count elements.
   * Temperature values are stored in microvolts (uV)
   * It is up to the application to convert these to
   * actual temperature values, as the system could have
   * any number of different types of sensors hooked up,
   * each of which have a different conversion between voltage
   * and current.
   */

  FAR uint32_t *temperatures;
};

struct battery_monitor_balance_s
{
  int balance_count;

  /* Pointer to array where balance switch values should be stored.
   * MUST contain at least balance_count elements.
   * Balance switch is turned on if true, off if false
   * Array indices for balance switches should correspond to those
   * for cell voltage.  The driver must rearrange the balance values as
   * necessary to make this happen.
   */

  FAR bool *balance;
};

struct battery_monitor_limits_s
{
  /* The driver may overwrite any value in this structure to indicate the
   * actual value that was set if the exact requested value was not
   * available.
   *
   * All voltage limits are per-cell.
   */

  uint32_t overvoltage_limit;   /* Overvoltage trip threshold, in uV */
  uint32_t undervoltage_limit;  /* Undervoltage trip threshold, in uV */
  uint32_t overcurrent_limit;   /* Overcurrent trip threshold, in mA */
  uint32_t shortcircuit_limit;  /* Short circuit current limit, in mA */
  uint32_t overvoltage_delay;   /* Delay before overvoltage trips, in uS */
  uint32_t undervoltage_delay;  /* Delay before undervoltage trips, in uS */
  uint32_t overcurrent_delay;   /* Delay before overcurrent trips, in uS */
  uint32_t shortcircuit_delay;  /* Delay before short circuit trips, in uS */
};

struct battery_monitor_switches_s
{
  /* Sets the state of the CHARGE switch, which allows the
   * battery to accept current.
   */

  bool charge;

  /* Sets the state of the DISCHARGE switch, which allows the
   * battery to supply current.
   */

  bool discharge;
};

struct battery_monitor_current_s
{
  /* The value of current measured by the monitor IC, in uA */

  int32_t current;

  /* The time over which the above current was measured,
   * in uS.  The application may request a specific time interval
   * by filling in this field when calling the BATIOC_CURRENT function,
   * and the driver may or may not choose to honor it.  The driver
   * will always overwrite this field with the actual measurement time.
   * A value of 0 means that instantaneous current should be measured.
   */

  uint32_t time;
};

  /* This structure defines the lower half battery interface */

struct battery_monitor_dev_s;

struct battery_monitor_operations_s
{
  /* Return the current battery state (see enum battery_status_e) */

  CODE int (*state)(FAR struct battery_monitor_dev_s *dev, FAR int *status);

  /* Return the current battery health (see enum battery_health_e) */

  CODE int (*health)(FAR struct battery_monitor_dev_s *dev, FAR int *health);

  /* Return true if the battery is online */

  CODE int (*online)(FAR struct battery_monitor_dev_s *dev,
                     FAR bool *status);

  /* Get the battery pack voltage */

  CODE int (*voltage)(FAR struct battery_monitor_dev_s *dev, FAR int *value);

  /* Get the battery cell voltages */

  CODE int (*cell_voltage)(FAR struct battery_monitor_dev_s *dev,
                           FAR struct battery_monitor_voltage_s *cellv);

  /* Get the battery pack current */

  CODE int (*current)(FAR struct battery_monitor_dev_s *dev,
                      FAR struct battery_monitor_current_s *current);

  /* Get the battery pack state of charge */

  CODE int (*soc)(FAR struct battery_monitor_dev_s *dev, FAR b16_t *value);

  /* Get the battery pack Coulomb count value */

  CODE int (*coulombs)(FAR struct battery_monitor_dev_s *dev,
                       FAR int *value);

  /* Read battery pack temperature sensor(s) */

  CODE int (*temperature)(FAR struct battery_monitor_dev_s *dev,
                          FAR struct battery_monitor_temperature_s *temps);

  /* Set balance switches on battery cells */

  CODE int (*balance)(FAR struct battery_monitor_dev_s *dev,
                      FAR struct battery_monitor_balance_s *bal);

  /* Put monitor device into low-power shutdown mode */

  CODE int (*shutdown)(FAR struct battery_monitor_dev_s *dev,
                       uintptr_t param);

  /* Configure safety limits for the device */

  CODE int (*setlimits)(FAR struct battery_monitor_dev_s *dev,
                        FAR struct battery_monitor_limits_s *limits);

  /* Set the state of charge/discharge switches to allow battery to
   * source/sink power
   */

  CODE int (*chgdsg)(FAR struct battery_monitor_dev_s *dev,
                     FAR struct battery_monitor_switches_s *sw);

  /* Clear battery monitor faults */

  CODE int (*clearfaults)(FAR struct battery_monitor_dev_s *dev,
                          uintptr_t param);

  /* Do device specific operation */

  CODE int (*operate)(FAR struct battery_monitor_dev_s *dev,
                      uintptr_t param);

  /* Get chip id */

  CODE int (*chipid)(FAR struct battery_charger_dev_s *dev,
                     FAR unsigned int *value);
};

/* This structure defines the battery driver state structure */

struct battery_monitor_dev_s
{
  /* Fields required by the upper-half driver */

  FAR const struct battery_monitor_operations_s *ops; /* Battery operations */
  mutex_t batlock;                                    /* Enforce mutually exclusive access */

  struct list_node flist;

  uint32_t mask;  /* record drive support features */

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
 * Name: battery_monitor_changed
 ****************************************************************************/

int battery_monitor_changed(FAR struct battery_monitor_dev_s *dev,
                            uint32_t mask);

/****************************************************************************
 * Name: battery_monitor_register
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

int battery_monitor_register(FAR const char *devpath,
                             FAR struct battery_monitor_dev_s *dev);

/****************************************************************************
 * Name: BQ769X0_initialize
 *
 * Description:
 *   Initialize the BQ769X0 battery driver and return an instance of the
 *   lower-half interface that may be used with battery_monitor_register().
 *
 * This is for:
 *   BQ7692000XXX
 *   BQ7693000XXX
 *   bq7694000XXX
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY_MONITOR - Upper half battery monitor driver support
 *   CONFIG_I2C - I2C support
 *   CONFIG_I2C_BQ769X0 - And the driver must be explicitly selected.
 *
 * Input Parameters:
 *   i2c       - An instance of the I2C interface to use to communicate with
 *               the BQ769X0
 *   addr      - The I2C address of the BQ769X0 (Can be 0x08 or 0x18).
 *   frequency - The I2C frequency
 *   crc       - True if the device has CRC enabled (see TI datasheet)
 *   cellcount - The number of battery cells attached to the BQ769X0.  The
 *               mapping of the cells changes based on count - see datasheet.
 *   chip      - The chip type (either CHIP_BQ76920, CHIP_BQ76930, or
 *               CHIP_BQ76940).  This is used to map cell numbers when the
 *               full capacity of the chip is not used.  See the TI datasheet
 *               for cell wiring information.
 *   sense_r   - The value of the current sense resistor, in micro ohms.
 *               This value is used to calculate reported current, and when
 *               setting overcurrent thresholds.
 * Returned Value:
 *   A pointer to the initialized battery driver instance.  A NULL pointer
 *   is returned on a failure to initialize the BQ769X0 lower half.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_BQ769X0)

struct i2c_master_s;
FAR struct battery_monitor_dev_s *
bq769x0_initialize(FAR struct i2c_master_s *i2c,
                   uint8_t addr, uint32_t frequency, bool crc,
                   uint8_t cellcount, uint8_t chip, uint32_t sense_r);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_BATTERY_MONITOR */
#endif /* __INCLUDE_NUTTX_POWER_BATTERY_H */

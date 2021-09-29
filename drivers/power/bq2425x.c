/****************************************************************************
 * drivers/power/bq2425x.c
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

/* Lower half driver for BQ2425x battery charger
 *
 * The BQ24250/BQ24251 are Li-Ion Battery Charger with Power-Path Management.
 * It can be configured to Input Current Limit up to 2A.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include "bq2425x.h"

/* This driver requires:
 *
 * CONFIG_BATTERY_CHARGER - Upper half battery driver support
 * CONFIG_I2C - I2C support
 * CONFIG_I2C_BQ2425X - And the driver must be explicitly selected.
 */

#if defined(CONFIG_BATTERY_CHARGER) && defined(CONFIG_I2C) && \
    defined(CONFIG_I2C_BQ2425X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_BQ2425X
#  define baterr  _err
#  define batreg  _err
#else
#  define baterr  _none
#  define batreg  _none
#endif

/****************************************************************************
 * Private
 ****************************************************************************/

struct bq2425x_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_charger_dev_s dev; /* Battery charger device */

  /* Data fields specific to the lower half BQ2425x driver follow */

  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  uint32_t frequency;           /* I2C frequency */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C support */

static int bq2425x_getreg8(FAR struct bq2425x_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval);
static int bq2425x_putreg8(FAR struct bq2425x_dev_s *priv, uint8_t regaddr,
                           uint8_t regval);

static inline int bq2425x_getreport(FAR struct bq2425x_dev_s *priv,
                                    uint8_t *report);
static inline int bq2425x_reset(FAR struct bq2425x_dev_s *priv);
static inline int bq2425x_watchdog(FAR struct bq2425x_dev_s *priv,
                                   bool enable);
static inline int bq2425x_powersupply(FAR struct bq2425x_dev_s *priv,
                                      int current);
static inline int bq2425x_setvolt(FAR struct bq2425x_dev_s *priv,
                                  int volts);
static inline int bq2425x_setcurr(FAR struct bq2425x_dev_s *priv,
                                  int current);

/* Battery driver lower half methods */

static int bq2425x_state(FAR struct battery_charger_dev_s *dev,
                         FAR int *status);
static int bq2425x_health(FAR struct battery_charger_dev_s *dev,
                          FAR int *health);
static int bq2425x_online(FAR struct battery_charger_dev_s *dev,
                          FAR bool *status);
static int bq2425x_voltage(FAR struct battery_charger_dev_s *dev,
                           int value);
static int bq2425x_current(FAR struct battery_charger_dev_s *dev,
                           int value);
static int bq2425x_input_current(FAR struct battery_charger_dev_s *dev,
                                 int value);
static int bq2425x_operate(FAR struct battery_charger_dev_s *dev,
                           uintptr_t param);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_charger_operations_s g_bq2425xops =
{
  bq2425x_state,
  bq2425x_health,
  bq2425x_online,
  bq2425x_voltage,
  bq2425x_current,
  bq2425x_input_current,
  bq2425x_operate
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bq2425x_getreg8
 *
 * Description:
 *   Read a 8-bit value from a BQ2425x register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK
 *   REPEATED-START <I2C read address> ACK Data0 ACK Data1 NO-ACK STOP
 *
 ****************************************************************************/

static int bq2425x_getreg8(FAR struct bq2425x_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval)
{
  struct i2c_config_s config;
  uint8_t val;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      baterr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 8-bits from the register */

  ret = i2c_read(priv->i2c, &config, &val, 1);
  if (ret < 0)
    {
      baterr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* Copy 8-bit value to be returned */

  *regval = val;
  return OK;
}

/****************************************************************************
 * Name: bq2425x_putreg8
 *
 * Description:
 *   Write a 8-bit value to a BQ2425x register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK Data0 ACK Data1 ACK STOP
 *
 ****************************************************************************/

static int bq2425x_putreg8(FAR struct bq2425x_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  batreg("addr: %02x regval: %08x\n", regaddr, regval);

  /* Set up a 3 byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Write the register address followed by the data (no RESTART) */

  return i2c_write(priv->i2c, &config, buffer, 2);
}

/****************************************************************************
 * Name: bq2425x_getreport
 *
 * Description:
 *   Read the BQ2425X Register #1 (status and fault)
 *
 ****************************************************************************/

static inline int bq2425x_getreport(FAR struct bq2425x_dev_s *priv,
                                    FAR uint8_t *report)
{
  uint8_t regval = 0;
  int ret;

  ret = bq2425x_getreg8(priv, BQ2425X_REG_1, &regval);
  if (ret == OK)
    {
      *report = regval;
    }

  return ret;
}

/****************************************************************************
 * Name: bq2425x_reset
 *
 * Description:
 *   Reset the BQ2425x
 *
 ****************************************************************************/

static inline int bq2425x_reset(FAR struct bq2425x_dev_s *priv)
{
  int ret;
  uint8_t regval;

  /* Read current register value */

  ret = bq2425x_getreg8(priv, BQ2425X_REG_2, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2425X! Error = %d\n", ret);
      return ret;
    }

  /* Send reset command */

  regval |= BQ2425X_RESET;
  ret = bq2425x_putreg8(priv, BQ2425X_REG_2, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2425X! Error = %d\n", ret);
      return ret;
    }

  /* Wait a little bit to clear registers */

  nxsig_usleep(500);

  /* There is a BUG in BQ2425X the RESET bit is always read as 1 */

  regval &= ~(BQ2425X_RESET);
  ret = bq2425x_putreg8(priv, BQ2425X_REG_2, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2425X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2425x_watchdog
 *
 * Description:
 *   Enable/Disable the BQ2425x watchdog
 *
 ****************************************************************************/

static inline int bq2425x_watchdog(FAR struct bq2425x_dev_s *priv,
                                   bool enable)
{
  int ret;
  uint8_t regval;

  ret = bq2425x_getreg8(priv, BQ2425X_REG_1, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2425X! Error = %d\n", ret);
      return ret;
    }

  if (enable)
    {
      regval |= BQ2425X_WD_EN;
    }
  else
    {
      regval &= ~(BQ2425X_WD_EN);
    }

  ret = bq2425x_putreg8(priv, BQ2425X_REG_1, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2425X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2425x_state
 *
 * Description:
 *   Return the current battery state
 *
 ****************************************************************************/

static int bq2425x_state(FAR struct battery_charger_dev_s *dev,
                         FAR int *status)
{
  FAR struct bq2425x_dev_s *priv = (FAR struct bq2425x_dev_s *)dev;
  uint8_t regval = 0;
  int ret;

  ret = bq2425x_getreport(priv, &regval);
  if (ret < 0)
    {
      *status = BATTERY_UNKNOWN;
      return ret;
    }

  regval &= BQ2425X_STAT_MASK;

  /* Is there some fault in the battery? */

  if (regval == BQ2425X_STAT_FAULT)
    {
     *status = BATTERY_FAULT;
    }

  /* Is the charging done? */

  else if (regval == BQ2425X_STAT_CHG_DONE)
    {
      *status = BATTERY_FULL;
    }

  /* Is the charging in progress? */

  else if (regval == BQ2425X_STAT_CHG_PROGRESS)
    {
      *status = BATTERY_CHARGING;
    }

  /* Is the charging ready? */

  else if (regval == BQ2425X_STAT_READY)
    {
      *status = BATTERY_IDLE;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2425x_health
 *
 * Description:
 *   Return the current battery health state
 *
 * Note: if more than one fault happened the user needs to call this ioctl
 * again to read a new fault, repeat until receive a BATTERY_HEALTH_GOOD.
 *
 ****************************************************************************/

static int bq2425x_health(FAR struct battery_charger_dev_s *dev,
                          FAR int *health)
{
  FAR struct bq2425x_dev_s *priv = (FAR struct bq2425x_dev_s *)dev;
  uint8_t regval = 0;
  int ret;

  ret = bq2425x_getreport(priv, &regval);
  if (ret < 0)
    {
      *health = BATTERY_HEALTH_UNKNOWN;
      return ret;
    }

  regval &= BQ2425X_FAULT_MASK;

  switch (regval)
  {
  case BQ2425X_FAULT_NORMAL:
    *health = BATTERY_HEALTH_GOOD;
    break;

  case BQ2425X_FAULT_SLEEP:
  case BQ2425X_FAULT_INP_OVP:
  case BQ2425X_FAULT_INP_UVLO:
  case BQ2425X_FAULT_ISET_SHORT:
  case BQ2425X_FAULT_INP_LDO_LOW:
    *health = BATTERY_HEALTH_UNSPEC_FAIL;
    break;

  case BQ2425X_FAULT_BAT_OVP:
    *health = BATTERY_HEALTH_OVERVOLTAGE;
    break;

  case BQ2425X_FAULT_BAT_TEMP:
  case BQ2425X_FAULT_THERM_SHUT:
    *health = BATTERY_HEALTH_OVERHEAT;
    break;

  case BQ2425X_FAULT_TIMER:
    *health = BATTERY_HEALTH_SAFE_TMR_EXP;
    break;

  case BQ2425X_FAULT_NO_BATTERY:
    *health = BATTERY_HEALTH_DISCONNECTED;
    break;

  default:
    *health = BATTERY_HEALTH_UNKNOWN;
    break;
  }

  return OK;
}

/****************************************************************************
 * Name: bq2425x_online
 *
 * Description:
 *   Return true if the battery is online
 *
 ****************************************************************************/

static int bq2425x_online(FAR struct battery_charger_dev_s *dev,
                          FAR bool *status)
{
  /* There is no concept of online/offline in this driver */

  *status = true;
  return OK;
}

/****************************************************************************
 * Name: bq2425x_powersupply
 *
 * Description:
 *   Set the Power Supply Current Limit.
 *
 ****************************************************************************/

static inline int bq2425x_powersupply(FAR struct bq2425x_dev_s *priv,
                                      int current)
{
  uint8_t regval;
  int idx;
  int ret;

  switch (current)
  {
  case BATTERY_INPUT_CURRENT_EXT_LIM:
    idx = BQ2425X_INP_CURR_EXT_ILIM;
    break;

  case 100:
    idx = BQ2425X_INP_CURR_LIM_100MA;
    break;

  case 150:
    idx = BQ2425X_INP_CURR_LIM_150MA;
    break;

  case 500:
    idx = BQ2425X_INP_CURR_LIM_500MA;
    break;

  case 900:
    idx = BQ2425X_INP_CURR_LIM_900MA;
    break;

  case 1500:
    idx = BQ2425X_INP_CURR_LIM_1500MA;
    break;

  case 2000:
    idx = BQ2425X_INP_CURR_LIM_2000MA;
    break;

  default:
    baterr("ERROR: Current not supported, setting default to 100mA!\n");
    idx = BQ2425X_INP_CURR_LIM_100MA;
    break;
  }

  /* Read current register */

  ret = bq2425x_getreg8(priv, BQ2425X_REG_2, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2425X! Error = %d\n", ret);
      return ret;
    }

  /* Clear previous current and set new value */

  regval &= ~(BQ2425X_INP_CURR_LIM_MASK);
  regval |= idx;

  /* Also clear Reset bit to avoid the resetting BUG */

  regval &= ~(BQ2425X_RESET);

  ret = bq2425x_putreg8(priv, BQ2425X_REG_2, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2425X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2425x_setvolt
 *
 * Description:
 *   Set the voltage level to charge the battery. Voltage value in mV.
 *
 ****************************************************************************/

static inline int bq2425x_setvolt(FAR struct bq2425x_dev_s *priv, int volts)
{
  uint8_t regval;
  int idx;
  int ret;

  /* Verify if voltage is in the acceptable range */

  if (volts < BQ2425X_VOLT_MIN || volts > BQ2425X_VOLT_MAX)
    {
      baterr("ERROR: Voltage %d mV is out of range.\n", volts);
      return -EINVAL;
    }

  ret = bq2425x_getreg8(priv, BQ2425X_REG_3, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2425X! Error = %d\n", ret);
      return ret;
    }

  /* Voltage starts at 3500mV and increases in steps of 20mV */

  idx = volts - BQ2425X_VOLT_MIN;
  idx = idx / 20;

  /* Clear previous voltage */

  regval &= ~(BQ2425X_BAT_VOLT_MASK);
  regval |= (idx << BQ2425X_BAT_VOLT_SHIFT);

  ret = bq2425x_putreg8(priv, BQ2425X_REG_3, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2425X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2425x_setcurr
 *
 * Description:
 *   Set the current to charge the battery. Current value in mA.
 *
 ****************************************************************************/

static inline int bq2425x_setcurr(FAR struct bq2425x_dev_s *priv,
                                  int current)
{
  uint8_t regval;
  int idx;
  int ret;

  /* Verify if current is in the acceptable range */

  if (current < BQ2425X_CURR_MIN || current > BQ2425X_CURR_MAX)
    {
      baterr("ERROR: Current %d mA is out of range.\n", current);
      return -EINVAL;
    }

  ret = bq2425x_getreg8(priv, BQ2425X_REG_4, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2425X! Error = %d\n", ret);
      return ret;
    }

  /* Current starts at 500mV and increases in steps of 50mA */

  idx = current - BQ2425X_CURR_MIN;
  idx = idx / 50;

  /* Clear previous current and set new value */

  regval &= ~(BQ2425X_CHG_CURRENT_MASK);
  regval |= (idx << BQ2425X_CHG_CURRENT_SHIFT);

  ret = bq2425x_putreg8(priv, BQ2425X_REG_4, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2425X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2425x_voltage
 *
 * Description:
 *   Set battery charger voltage
 *
 ****************************************************************************/

static int bq2425x_voltage(FAR struct battery_charger_dev_s *dev, int value)
{
  FAR struct bq2425x_dev_s *priv = (FAR struct bq2425x_dev_s *)dev;
  int ret;

  /* Set voltage to battery charger */

  ret = bq2425x_setvolt(priv, value);
  if (ret < 0)
    {
      baterr("ERROR: Error setting voltage to BQ2425X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2425x_current
 *
 * Description:
 *   Set the battery charger current rate for charging
 *
 ****************************************************************************/

static int bq2425x_current(FAR struct battery_charger_dev_s *dev, int value)
{
  FAR struct bq2425x_dev_s *priv = (FAR struct bq2425x_dev_s *)dev;
  int ret;

  /* Set current to battery charger */

  ret = bq2425x_setcurr(priv, value);
  if (ret < 0)
    {
      baterr("ERROR: Error setting current to BQ2425X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2425x_input_current
 *
 * Description:
 *   Set the power-supply input current limit
 *
 ****************************************************************************/

static int bq2425x_input_current(FAR struct battery_charger_dev_s *dev,
                                 int value)
{
  FAR struct bq2425x_dev_s *priv = (FAR struct bq2425x_dev_s *)dev;
  int ret;

  ret = bq2425x_powersupply(priv, value);
  if (ret < 0)
    {
      baterr("ERROR: Failed to set BQ2425x power supply: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2425x_operate
 *
 * Description:
 *   Do miscellaneous battery ioctl()
 *
 ****************************************************************************/

static int bq2425x_operate(FAR struct battery_charger_dev_s *dev,
                           uintptr_t param)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bq2425x_initialize
 *
 * Description:
 *   Initialize the BQ2425x battery driver and return an instance of the
 *   lower_half interface that may be used with battery_charger_register();
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY_CHARGER - Upper half battery driver support
 *   CONFIG_I2C - I2C support
 *   CONFIG_I2C_BQ2425X - And the driver must be explicitly selected.
 *
 * Input Parameters:
 *   i2c       - An instance of the I2C interface to use to communicate with
 *               the BQ2425x
 *   addr      - The I2C address of the BQ2425x (Better be 0x6a)
 *   frequency - The I2C frequency
 *   current   - The input current our power-supply can offer to charger
 *
 * Returned Value:
 *   A pointer to the initialized lower-half driver instance.  A NULL pointer
 *   is returned on a failure to initialize the BQ2425x lower half.
 *
 ****************************************************************************/

FAR struct battery_charger_dev_s *
  bq2425x_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                     uint32_t frequency, int current)
{
  FAR struct bq2425x_dev_s *priv;
  int ret;

  /* Initialize the BQ2425x device structure */

  priv = kmm_zalloc(sizeof(struct bq2425x_dev_s));
  if (priv)
    {
      /* Initialize the BQ2425x device structure */

      priv->dev.ops   = &g_bq2425xops;
      priv->i2c       = i2c;
      priv->addr      = addr;
      priv->frequency = frequency;

      /* Reset the BQ2425x */

      ret = bq2425x_reset(priv);
      if (ret < 0)
        {
          baterr("ERROR: Failed to reset the BQ2425x: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Disable watchdog otherwise BQ2425x returns to StandAlone mode */

      ret = bq2425x_watchdog(priv, false);
      if (ret < 0)
        {
          baterr("ERROR: Failed to disable BQ2425x watchdog: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Define the current that our power supply can offer to the charger. */

      ret = bq2425x_powersupply(priv, current);
      if (ret < 0)
        {
          baterr("ERROR: Failed to set BQ2425x power supply: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }
    }

  return (FAR struct battery_charger_dev_s *)priv;
}

#endif /* CONFIG_BATTERY_CHARGER && CONFIG_I2C && CONFIG_I2C_BQ2425X */

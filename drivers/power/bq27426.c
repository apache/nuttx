/****************************************************************************
 * drivers/power/bq27426.c
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

/* Lower half driver for BQ27426 battery fuel gauge */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_gauge.h>
#include <nuttx/power/bq27426.h>

/* This driver requires:
 *
 * CONFIG_BATTERY - Upper half battery driver support
 * CONFIG_I2C - I2C support
 * CONFIG_BQ27426 - And the driver must be explicitly selected.
 */

#if defined(CONFIG_BATTERY_GAUGE) && defined(CONFIG_I2C) && \
    defined(CONFIG_BQ27426)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_BQ27426
#  define baterr  _err
#  define batdbg  _info
#  define batinfo _info
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define baterr(x...)
#    define batdbg(x...)
#    define batinfo(x...)
#  else
#    define baterr (void)
#    define batdbg (void)
#    define batinfo(void)
#  endif
#endif

/****************************************************************************
 * Private
 ****************************************************************************/

/* Parameters for the capacity() function,
 * to specify which capacity to read in mAh
 */

typedef enum
{
  REMAIN,      /* Remaining Capacity (DEFAULT) */
  FULL,        /* Full Capacity */
  AVAIL,       /* Available Capacity */
  AVAIL_FULL,  /* Full Available Capacity */
  REMAIN_F,    /* Remaining Capacity Filtered */
  REMAIN_UF,   /* Remaining Capacity Unfiltered */
  FULL_F,      /* Full Capacity Filtered */
  FULL_UF      /* Full Capacity Unfiltered */
} capacity_measure;

/****************************************************************************
 * Private
 ****************************************************************************/

struct bq27426_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_gauge_dev_s dev; /* Battery gauge device */

  /* Data fields specific to the lower half bq27426 driver follow */

  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  uint32_t frequency;           /* I2C frequency */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C support */

static int bq27426_getreg16(FAR struct bq27426_dev_s *priv, uint8_t regaddr,
                             FAR uint16_t *regval);
static int bq27426_putreg16(FAR struct bq27426_dev_s *priv, uint8_t regaddr,
                             uint16_t regval);
static inline int bq27426_getvoltage(FAR struct bq27426_dev_s *priv,
                                    b16_t *voltage);
static inline int bq27426_getsoc(FAR struct bq27426_dev_s *priv,
                                 b16_t *soc);
static inline int bq27426_getcapacity(FAR struct bq27426_dev_s *priv,
                                      uint8_t capacity,
                                      b16_t *value);
static inline int bq27426_get_device_id(FAR struct bq27426_dev_s *priv,
                                    uint16_t *id);

/* Battery driver lower half methods */

static int bq27426_state(struct battery_gauge_dev_s *dev, int *status);
static int bq27426_online(struct battery_gauge_dev_s *dev, bool *status);
static int bq27426_voltage(struct battery_gauge_dev_s *dev, b16_t *value);
static int bq27426_capacity_full(struct battery_gauge_dev_s *dev,
                                 b16_t *capacity_full);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_gauge_operations_s g_bq27426ops =
{
  bq27426_state,
  bq27426_online,
  bq27426_voltage,
  bq27426_capacity_full,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bq27426_getreg16
 *
 * Description:
 *   Read a 16-bit value from a bq27426 register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK
 *   REPEATED-START <I2C read address> ACK Data0 ACK Data1 NO-ACK STOP
 *
 ****************************************************************************/

static int bq27426_getreg16(FAR struct bq27426_dev_s *priv, uint8_t regaddr,
                             FAR uint16_t *regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret;

  /* Set up the configuration and perform the write-read operation */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      baterr("i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 16-bits from the register */

  ret = i2c_read(priv->i2c, &config, buffer, 2);
  if (ret < 0)
    {
      baterr("i2c_read failed: %d\n", ret);
      return ret;
    }

    *regval = ((uint16_t) buffer[1] << 8) | buffer[0];

  return OK;
}

/****************************************************************************
 * Name: bq27426_putreg16
 *
 * Description:
 *   Write a 16-bit value to a bq27426 register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK Data0 ACK Data1 ACK STOP
 *
 ****************************************************************************/

static int bq27426_putreg16(FAR struct bq27426_dev_s *priv, uint8_t regaddr,
                             uint16_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[3];

  baterr("addr: %02x regval: %04x\n", regaddr, regval);

  /* Set up a 3 byte message to send */

  buffer[0] = regaddr;
  buffer[1] = (uint8_t)(regval & 0x00ff);
  buffer[2] = (uint8_t)(regval >> 8);

  /* Set up the configuration and perform the write-read operation */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address followed by the data (no RESTART) */

  return i2c_write(priv->i2c, &config, buffer, 3);
}

/****************************************************************************
 * Name: bq27426_get_device_id
 *
 * Description:
 *   Read the ID register
 *
 ****************************************************************************/

static inline int bq27426_get_device_id(FAR struct bq27426_dev_s *priv,
                                    uint16_t *id)
{
  uint16_t regval = 0;
  int ret;

  ret = bq27426_putreg16(priv, BQ27426_COMMAND_CONTROL,
                         BQ27426_CONTROL_DEVICE_TYPE);
  if (ret == OK)
    {
      ret = bq27426_getreg16(priv, BQ27426_COMMAND_CONTROL, &regval);
     *id = regval;
    }

  return ret;
}

/****************************************************************************
 * Name: bq27426_getvoltage
 *
 * Description:
 *   Read the VCELL register and scale the returned value
 *
 ****************************************************************************/

static inline int bq27426_getvoltage(FAR struct bq27426_dev_s *priv,
                                    b16_t *voltage)
{
  uint16_t regval = 0;
  int ret;

  ret = bq27426_getreg16(priv, BQ27426_COMMAND_VOLTAGE, &regval);
  if (ret == OK)
    {
     *voltage = regval;
    }

  return ret;
}

/****************************************************************************
 * Name: bq27426_getsoc
 *
 * Description:
 *   Read the SOC register and scale the returned value
 *
 ****************************************************************************/

static inline int bq27426_getsoc(FAR struct bq27426_dev_s *priv, b16_t *soc)
{
  uint16_t regval = 0;
  int ret;

  ret = bq27426_getreg16(priv, BQ27426_COMMAND_SOC, &regval);
  if (ret == OK)
    *soc = regval;

  return ret;
}

/****************************************************************************
 * Name: bq27426_getcapacity
 *
 * Description:
 *   Read the SOC register and scale the returned value
 *
 ****************************************************************************/

static inline int bq27426_getcapacity(FAR struct bq27426_dev_s *priv,
                                      uint8_t capacity, b16_t *value)
{
  uint16_t regval = 0;
  return bq27426_getreg16(priv, capacity, &regval);
}

/****************************************************************************
 * Name: bq27426_state
 *
 * Description:
 *   Return the battery state
 *
 ****************************************************************************/

static int bq27426_state(struct battery_gauge_dev_s *dev, int *status)
{
  FAR struct bq27426_dev_s *priv = (FAR struct bq27426_dev_s *)dev;
  b16_t soc = 0;
  int ret;

  /* Only a few of the possible battery states are supported by this driver:
   *
   *  BATTERY_UNKNOWN - Returned on error conditions
   *  BATTERY_IDLE - This is what will usually be reported
   *  BATTERY_FULL - This will be reported if the SoC is greater than 95%
   *  BATTERY_CHARGING and BATTERY_DISCHARGING - I don't think this hardware
   *    knows anything about current (charging or dischargin).
   *
   */

  ret = bq27426_getsoc(priv, &soc);
  if (ret < 0)
    {
      *status = BATTERY_UNKNOWN;
      return ret;
    }

    *status = soc;

  return OK;
}

/****************************************************************************
 * Name: bq27426_voltage
 *
 * Description:
 *   Current battery voltage
 *
 ****************************************************************************/

static int bq27426_voltage(struct battery_gauge_dev_s *dev, b16_t *value)
{
  FAR struct bq27426_dev_s *priv = (FAR struct bq27426_dev_s *)dev;
  return bq27426_getvoltage(priv, value);
}

/****************************************************************************
 * Name: bq27426_capacity_full
 *
 * Description:
 *   Battery capacity (mAh)
 *
 ****************************************************************************/

static int bq27426_capacity_full(struct battery_gauge_dev_s *dev,
                                 b16_t *value)
{
  FAR struct bq27426_dev_s *priv = (FAR struct bq27426_dev_s *)dev;
  return bq27426_getcapacity(priv, BQ27426_COMMAND_FULL_CAP_FIL, value);
}

/****************************************************************************
 * Name: bq27426_online
 *
 * Description:
 *   Return true if the batter is online
 *
 ****************************************************************************/

static int bq27426_online(struct battery_gauge_dev_s *dev, bool *status)
{
  /* There is no concept of online/offline in this driver */

  *status = true;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bq27426_initialize
 *
 * Description:
 *   Initialize the bq27426 battery driver and return an instance of the
 *   lower_half interface that may be used with battery_register();
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY - Upper half battery driver support
 *   CONFIG_I2C - I2C support
 *   CONFIG_BQ27426 - And the driver must be explicitly selected.
 *
 * Input Parameters:
 *   i2c - An instance of the I2C interface to use to communicate with the bq
 *   addr - The I2C address of the bq27426 (Better be 0x55).
 *   frequency - The I2C frequency
 *
 * Returned Value:
 *   A pointer to the initializeed lower-half driver instance. A NULL
 *   pointer is returned on a failure to initialize the bq27426 lower half.
 *
 ****************************************************************************/

FAR struct battery_gauge_dev_s *bq27426_initialize(
                                                FAR struct i2c_master_s *i2c,
                                                uint8_t addr,
                                                uint32_t frequency)
{
  FAR struct bq27426_dev_s *priv;

  /* Initialize the bq27426 device structure */

  priv = (FAR struct bq27426_dev_s *)kmm_zalloc(sizeof(
                                                      struct bq27426_dev_s));
  if (priv)
    {
      /* Initialize the bq27426 device structure */

      priv->dev.ops   = &g_bq27426ops;
      priv->i2c       = i2c;
      priv->addr      = addr;
      priv->frequency = frequency;
    }

  return (FAR struct battery_gauge_dev_s *)priv;
}

#endif /* CONFIG_BATTERY && CONFIG_I2C && CONFIG_BQ27426 */

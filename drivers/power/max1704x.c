/****************************************************************************
 * drivers/power/max1704x.c
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

/* Lower half driver for MAX1704x battery fuel gauge */

/* "MAX17040/MAX17041 are ultra-compact, low-cost, host-side fuel-gauge
 * systems for lithium-ion (Li+) batteries in handheld and portable
 * equipment. MAX17040 is configured to operate with a single lithium cell
 * and the MAX17041 is configured for a dual-cell 2S pack.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_gauge.h>

/* This driver requires:
 *
 * CONFIG_BATTERY - Upper half battery gauge driver support
 * CONFIG_I2C - I2C support
 * CONFIG_I2C_MAX1704X - And the driver must be explicitly selected.
 */

#if defined(CONFIG_BATTERY) && defined(CONFIG_I2C) && defined(CONFIG_I2C_MAX1704X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/*   CONFIG_I2C_MAX17040 or CONFIG_I2C_MAX17041 - The driver must know which
 *     chip is on the board in order to scale the voltage correctly.
 */

#if !defined(CONFIG_I2C_MAX17040) && !defined(CONFIG_I2C_MAX17041)
#  warning "Assuming CONFIG_I2C_MAX17040"
#  define CONFIG_I2C_MAX17040 1
#endif

/* MAX1704x Register Definitions ********************************************/

/* "All host interaction with MAX17040/MAX17041 is handled by writing to
 *  and reading from register locations. MAX17040/MAX17041 have six 16-bit
 *  registers: SOC, VCELL, MODE, VERSION, RCOMP, and COMMAND. Register reads
 *  and writes are only valid if all 16 bits are transferred..."
 */

/* "VCELL Register.  Battery voltage is measured at the CELL pin input with
 *  respect to GND over a 0 to 5.00V range for MAX17040 and 0 to 10.00V
 *  for the MAX17041 with resolutions of 1.25mV and 2.50mV, respectively..."
 */

#define MAX1407X_VCELL_ADDR      0x02  /* Bits 4-15: Bits 0-11 of the battery voltage */

/* VCELL conversion macros */

#define MAX14700_VCELL_CONV 82   /* 0.00125 v * 65536 */
#define MAX14070_VCELL(v)       ((b16_t)(v) * MAX14700_VCELL_CONV)

#define MAX14701_VCELL_CONV 163  /* 0.0025 v * 65536 */
#define MAX14071_VCELL(v)       ((b16_t)(v) * MAX14701_VCELL_CONV)

#ifdef CONFIG_I2C_MAX17040
#  define MAX1407X_VCELL(v)     MAX14070_VCELL(v)
#else
#  define MAX1407X_VCELL(v)     MAX14071_VCELL(v)
#endif

/* "SOC Register. The SOC register is a read-only register that displays the
 *  state of charge of the cell as calculated by the ModelGauge algorithm.
 *  The result is displayed as a percentage of the cell�s full capacity...
 *
 * "...Units of % can be directly determined by observing only the high byte
 *  of the SOC register. The low byte provides additional resolution in units
 *  1/256%.
 */

#define MAX1407X_SOC_ADDR        0x04  /* Bits 0-15: Full SOC */

/* SoC conversion macros */

#define MAX1407X_SOC(s)         ((b16_t)(s) << 8)
#define MAX17040_SOC_FULL       itob16(95) /* We say full if Soc >= 95% */

/* "MODE Register.The MODE register allows the host processor to send special
 *  commands to the IC."
 */

#define MAX1407X_MODE_ADDR       0x06  /* Bits 0-15: 16-bit MODE */

/* Supported modes */

#define MAX1407X_MODE_QUICKSTART 0x4000

/* "The VERSION register is a read-only register that contains a value
 *  indicating the production version of MAX17040/MAX17041."
 */

#define MAX1407X_VERSION_ADDR    0x08  /* Bits 0-15: 16-bit VERSION */

/* "RCOMP Register. RCOMP is a 16-bit value used to compensate the ModelGauge
 *  algorithm. RCOMP can be adjusted to optimize performance for different
 *  lithium chemistries or different operating temperatures... The factory-
 *  default value for RCOMP is 9700h."
 */

#define MAX1407X_RCOMP_ADDR      0x0c  /* Bits 0-15: 16-bit RCOMP */

/* "COMMAND Register.  The COMMAND register allows the host processor to send
 *  special commands to the IC..."
 */

#define MAX1407X_COMMAND_ADDR    0xfe  /* Bits 0-7:  16-bit COMMAND */

/* Supported copmmands */

#define MAX1407X_COMMAND_POR     0x5400

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_MAX1704X
#  define baterr  _err
#else
#  define baterr  _none
#endif

/****************************************************************************
 * Private
 ****************************************************************************/

struct max1704x_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  FAR const struct battery_gauge_operations_s *ops; /* Battery operations */
  sem_t batsem;                                     /* Enforce mutually exclusive access */

  /* Data fields specific to the lower half MAX1704x driver follow */

  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  uint32_t frequency;           /* I2C frequency */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C support */

static int max1704x_getreg16(FAR struct max1704x_dev_s *priv,
                             uint8_t regaddr, FAR uint16_t *regval);
static int max1704x_putreg16(FAR struct max1704x_dev_s *priv,
                             uint8_t regaddr, uint16_t regval);

static inline int max1704x_getvcell(FAR struct max1704x_dev_s *priv,
                                    b16_t *vcell);
static inline int max1704x_getsoc(FAR struct max1704x_dev_s *priv,
                                  b16_t *soc);
static inline int max1704x_setquikstart(FAR struct max1704x_dev_s *priv);
static inline int max1704x_getversion(FAR struct max1704x_dev_s *priv,
                                      uint16_t *version);
static inline int max1704x_reset(FAR struct max1704x_dev_s *priv);

/* Battery driver lower half methods */

static int max1704x_state(struct battery_gauge_dev_s *dev, int *status);
static int max1704x_online(struct battery_gauge_dev_s *dev, bool *status);
static int max1704x_voltage(struct battery_gauge_dev_s *dev, b16_t *value);
static int max1704x_capacity(struct battery_gauge_dev_s *dev, b16_t *value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_gauge_operations_s g_max1704xops =
{
  max1704x_state,
  max1704x_online,
  max1704x_voltage,
  max1704x_capacity
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max1704x_getreg16
 *
 * Description:
 *   Read a 16-bit value from a MAX1704x register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK
 *   REPEATED-START <I2C read address> ACK Data0 ACK Data1 NO-ACK STOP
 *
 ****************************************************************************/

static int max1704x_getreg16(FAR struct max1704x_dev_s *priv,
                             uint8_t regaddr, FAR uint16_t *regval)
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

  /* Return the 16-bit value */

  return (uint16_t)buffer[0] << 8 | (uint16_t)buffer[1];
}

/****************************************************************************
 * Name: max1704x_putreg16
 *
 * Description:
 *   Write a 16-bit value to a MAX1704x register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK Data0 ACK Data1 ACK STOP
 *
 ****************************************************************************/

static int max1704x_putreg16(FAR struct max1704x_dev_s *priv,
                             uint8_t regaddr, uint16_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[3];

  baterr("addr: %02x regval: %08x\n", regaddr, regval);

  /* Set up a 3 byte message to send */

  buffer[0] = regaddr;
  buffer[1] = (uint8_t)(regval >> 8);
  buffer[2] = (uint8_t)(regval & 0xff);

  /* Set up the configuration and perform the write-read operation */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address followed by the data (no RESTART) */

  return i2c_write(priv->i2c, &config, buffer, 3);
}

/****************************************************************************
 * Name: max1704x_getvcell
 *
 * Description:
 *   Read the VCELL register and scale the returned value
 *
 ****************************************************************************/

static inline int max1704x_getvcell(FAR struct max1704x_dev_s *priv,
                                    b16_t *vcell)
{
  uint16_t regval = 0;
  int ret;

  ret = max1704x_getreg16(priv, MAX1407X_VCELL_ADDR, &regval);
  if (ret == OK)
    {
      *vcell = MAX1407X_VCELL(regval);
    }

  return ret;
}

/****************************************************************************
 * Name: max1704x_getsoc
 *
 * Description:
 *   Read the SOC register and scale the returned value
 *
 ****************************************************************************/

static inline int max1704x_getsoc(FAR struct max1704x_dev_s *priv,
                                  b16_t *soc)
{
  uint16_t regval = 0;
  int ret;

  ret = max1704x_getreg16(priv, MAX1407X_VCELL_ADDR, &regval);
  if (ret == OK)
    {
      *soc = MAX1407X_SOC(regval);
    }

  return ret;
}

/****************************************************************************
 * Name: max1704x_setquikstart
 *
 * Description:
 *   Set Quickstart mode
 *
 ****************************************************************************/

static inline int max1704x_setquikstart(FAR struct max1704x_dev_s *priv)
{
  return max1704x_putreg16(priv, MAX1407X_MODE_ADDR,
                           MAX1407X_MODE_QUICKSTART);
}

/****************************************************************************
 * Name: max1704x_getversion
 *
 * Description:
 *   Read the SOC register and scale the returned value
 *
 ****************************************************************************/

static inline int max1704x_getversion(FAR struct max1704x_dev_s *priv,
                                      uint16_t *version)
{
  return max1704x_getreg16(priv, MAX1407X_VCELL_ADDR, version);
}

/****************************************************************************
 * Name: max1704x_setrcomp
 *
 * Description:
 *   Set Quickstart mode
 *
 ****************************************************************************/

static inline int max1704x_setrcomp(FAR struct max1704x_dev_s *priv,
                                    uint16_t rcomp)
{
  return max1704x_putreg16(priv, MAX1407X_RCOMP_ADDR, rcomp);
}

/****************************************************************************
 * Name: max1704x_reset
 *
 * Description:
 *   Reset MAX1704x
 *
 ****************************************************************************/

static inline int max1704x_reset(FAR struct max1704x_dev_s *priv)
{
  return max1704x_putreg16(priv, MAX1407X_COMMAND_ADDR,
                           MAX1407X_COMMAND_POR);
}

/****************************************************************************
 * Name: max1704x_state
 *
 * Description:
 *   Return the current battery state
 *
 ****************************************************************************/

static int max1704x_state(struct battery_gauge_dev_s *dev, int *status)
{
  FAR struct max1704x_dev_s *priv = (FAR struct max1704x_dev_s *)dev;
  b16_t soc = 0;
  int ret;

  /* Only a few of the possible battery states are supported by this driver:
   *
   *  BATTERY_UNKNOWN - Returned on error conditions
   *  BATTERY_IDLE - This is what will usually be reported
   *  BATTERY_FULL - This will be reported if the SoC is greater than 95%
   *  BATTERY_CHARGING and BATTERY_DISCHARGING - I don't think this hardware
   *    knows anything about current (charging or dischargin).
   */

  ret = max1704x_getsoc(priv, &soc);
  if (ret < 0)
    {
      *status = BATTERY_UNKNOWN;
      return ret;
    }

  /* Is the battery fully charged? */

  if (soc > MAX17040_SOC_FULL)
    {
      *status = BATTERY_FULL;
    }
  else
    {
      *status = BATTERY_IDLE;
    }

  return OK;
}

/****************************************************************************
 * Name: max1704x_online
 *
 * Description:
 *   Return true if the batter is online
 *
 ****************************************************************************/

static int max1704x_online(struct battery_gauge_dev_s *dev, bool *status)
{
  /* There is no concept of online/offline in this driver */

  *status = true;
  return OK;
}

/****************************************************************************
 * Name: max1704x_voltage
 *
 * Description:
 *   Current battery voltage
 *
 ****************************************************************************/

static int max1704x_voltage(struct battery_gauge_dev_s *dev, b16_t *value)
{
  FAR struct max1704x_dev_s *priv = (FAR struct max1704x_dev_s *)dev;
  return max1704x_getvcell(priv, value);
}

/****************************************************************************
 * Name: max1704x_capacity
 *
 * Description:
 *   Battery capacity
 *
 ****************************************************************************/

static int max1704x_capacity(struct battery_gauge_dev_s *dev, b16_t *value)
{
  FAR struct max1704x_dev_s *priv = (FAR struct max1704x_dev_s *)dev;
  return max1704x_getsoc(priv, value);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max1704x_initialize
 *
 * Description:
 *   Initialize MAX1704x battery driver and return an instance of the
 *   lower_half interface that may be used with battery_register();
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY - Upper half battery driver support
 *   CONFIG_I2C - I2C support
 *   CONFIG_I2C_MAX1704X - And the driver must be explicitly selected.
 *   CONFIG_I2C_MAX17040 or CONFIG_I2C_MAX17041 - The driver must know which
 *     chip is on the board in order to scale the voltage correctly.
 *
 * Input Parameters:
 *   i2c - An instance of the I2C interface to communicate with MAX1704x
 *   addr - The I2C address of MAX1704x (Better be 0x36).
 *   frequency - The I2C frequency
 *
 * Returned Value:
 *   A pointer to the initializeed lower-half driver instance. A NULL pointer
 *   is returned on a failure to initialize MAX1704x lower half.
 *
 ****************************************************************************/

FAR struct battery_gauge_dev_s *
max1704x_initialize(FAR struct i2c_master_s *i2c,
                    uint8_t addr, uint32_t frequency)
{
  FAR struct max1704x_dev_s *priv;
#if 0
  int ret;
#endif

  /* Initialize MAX1704x device structure */

  priv = kmm_zalloc(sizeof(struct max1704x_dev_s));
  if (priv)
    {
      /* Initialize MAX1704x device structure */

      nxsem_init(&priv->batsem, 0, 1);
      priv->ops       = &g_max1704xops;
      priv->i2c       = i2c;
      priv->addr      = addr;
      priv->frequency = frequency;

      /* Reset MAX1704x (mostly just to make sure that we can talk to it) */

#if 0
      ret = max1704x_reset(priv);
      if (ret < 0)
        {
          baterr("Failed to reset MAX1704x: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }
#endif
    }

  return (FAR struct battery_gauge_dev_s *)priv;
}

#endif /* CONFIG_BATTERY && CONFIG_I2C && CONFIG_I2C_MAX1704X */

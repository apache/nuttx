/****************************************************************************
 * drivers/power/battery/axp202.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>
#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/power/axp202.h>

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_AXP202)

/****************************************************************************
 * Private type
 ****************************************************************************/

struct axp202_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_charger_dev_s dev; /* Battery operations */

  /* Data fields specific to the lower half axp202 driver follow */

  FAR struct i2c_master_s *i2c;       /* I2C interface */
  uint8_t                  addr;      /* I2C address */
  uint32_t                 frequency; /* I2C frequency */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C support */

static int axp202_getreg8(FAR struct axp202_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *regval);
static int axp202_putreg8(FAR struct axp202_dev_s *priv, uint8_t regaddr,
                          uint8_t regval);

/* Battery driver lower half methods */

static int axp202_state(FAR struct battery_charger_dev_s *dev,
                        FAR int *                         status);
static int axp202_health(FAR struct battery_charger_dev_s *dev,
                         FAR int *                         health);
static int axp202_online(FAR struct battery_charger_dev_s *dev,
                         FAR bool *                        status);
static int axp202_voltage(FAR struct battery_charger_dev_s *dev, int value);
static int axp202_current(FAR struct battery_charger_dev_s *dev, int value);
static int axp202_input_current(FAR struct battery_charger_dev_s *dev,
                                int                               value);
static int axp202_operate(FAR struct battery_charger_dev_s *dev,
                          uintptr_t                         param);

static int axp202_set_ldo2_valtage(FAR struct axp202_dev_s *dev,
                                   int                      voltage);
static int axp202_set_ldo3_valtage(FAR struct axp202_dev_s *dev,
                                   int                      voltage);
static int axp202_set_ldo4_valtage(FAR struct axp202_dev_s *dev,
                                   int                      voltage);
static int axp202_set_dc2_valtage(FAR struct axp202_dev_s *dev,
                                  int                      voltage);
static int axp202_set_dc2_valtage(FAR struct axp202_dev_s *dev,
                                  int                      voltage);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_charger_operations_s g_axp202ops =
{
  axp202_state,
  axp202_health,
  axp202_online,
  axp202_voltage,
  axp202_current,
  axp202_input_current,
  axp202_operate
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int axp202_getreg8(FAR struct axp202_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *regval)
{
  struct i2c_config_s config;
  int                 ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(regval != NULL);

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 8 bits from the register */

  ret = i2c_read(priv->i2c, &config, regval, sizeof(*regval));
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  sninfo("addr: %02x value: %02x\n", regaddr, *regval);

  return OK;
}

static int axp202_putreg8(FAR struct axp202_dev_s *priv, uint8_t regaddr,
                          uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t             buffer[2];
  int                 ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Set up a 2-byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address followed by the data (no RESTART) */

  ret = i2c_write(priv->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  sninfo("addr: %02x value: %02x\n", regaddr, regval);

  return OK;
}

static int axp202_state(struct battery_charger_dev_s *dev, int *status)
{
  FAR struct axp202_dev_s *priv = (FAR struct axp202_dev_s *)dev;
  uint8_t                  val  = 0;
  int                      ret  = 0;

  /* Only a few of the possible battery states are supported by this driver:
   *
   *  BATTERY_UNKNOWN - Returned on error conditions
   *  BATTERY_IDLE - This is what will usually be reported
   *  BATTERY_FULL - This will be reported if the SoC is greater than 95%
   *  BATTERY_CHARGING and BATTERY_DISCHARGING - I don't think this hardware
   *    knows anything about current (charging or dischargin).
   *
   */

  ret = axp202_getreg8(priv, AXP202_MODE_CHGSTATUS, &val);

  if (ret != OK)
    {
      *status = BATTERY_UNKNOWN;
      return ret;
    }

  if (val & (1 << 6))
    {
      *status = BATTERY_CHARGING;
    }
  else
    {
      *status = BATTERY_DISCHARGING;
    }

  return OK;
}

/****************************************************************************
 * Name: axp202_health
 *
 * Description:
 *   Return the current battery health state
 *
 * Note: if more than one fault happened the user needs to call this ioctl
 * again to read a new fault, repeat until receive a BATTERY_HEALTH_GOOD.
 *
 ****************************************************************************/

static int axp202_health(FAR struct battery_charger_dev_s *dev,
                         FAR int *                         health)
{
  FAR struct axp202_dev_s *priv = (FAR struct axp202_dev_s *)dev;
  uint8_t                  val  = 0;
  int                      ret  = 0;

  /*  Only a few of the possible states are supported by this driver:
   *   BATTERY_HEALTH_UNKNOWN       - health state is not known
   *   BATTERY_HEALTH_GOOD          - is in good condiction
   *   BATTERY_HEALTH_DEAD          - is dead, nothing we can do
   *   BATTERY_HEALTH_OVERHEAT      - is over recommended temperature
   *   BATTERY_HEALTH_OVERVOLTAGE   - voltage is over recommended level
   *   BATTERY_HEALTH_UNSPEC_FAIL   - charger reported an unspected failure
   *   BATTERY_HEALTH_COLD          - is under recommended temperature
   *   BATTERY_HEALTH_WD_TMR_EXP    - WatchDog Timer Expired
   *   BATTERY_HEALTH_SAFE_TMR_EXP  - Safety Timer Expired
   *   BATTERY_HEALTH_DISCONNECTED  - is not connected
   */

  ret = axp202_getreg8(priv, AXP202_MODE_CHGSTATUS, &val);
  if (ret < 0)
    {
      *health = BATTERY_HEALTH_UNKNOWN;
      return ret;
    }

  if (val & (1 << 7))
    {
      *health = BATTERY_HEALTH_OVERHEAT;
    }
  else if (!(val & (1 << 5)))
    {
      *health = BATTERY_HEALTH_DISCONNECTED;
    }
  else
    {
      *health = BATTERY_HEALTH_GOOD;
    }

  return OK;
}

/****************************************************************************
 * Name: axp202_online
 *
 * Description:
 *   Return true if the batter is online
 *
 ****************************************************************************/

static int axp202_online(struct battery_charger_dev_s *dev, bool *status)
{
  FAR struct axp202_dev_s *priv = (FAR struct axp202_dev_s *)dev;
  uint8_t                  val  = 0;
  int                      ret  = 0;

  ret = axp202_getreg8(priv, AXP202_MODE_CHGSTATUS, &val);

  if (ret != OK)
    {
      return ret;
    }

  if (val & (1 << 5))
    {
      *status = true;
    }
  else
    {
      *status = false;
    }

  return OK;
}

/****************************************************************************
 * Name: axp202_voltage
 *
 * Description:
 *   Set battery charger voltage
 *
 ****************************************************************************/

static int axp202_voltage(FAR struct battery_charger_dev_s *dev, int value)
{
  FAR struct axp202_dev_s *priv = (FAR struct axp202_dev_s *)dev;
  int                      ret;
  uint8_t                  reg = 0;

  /* Set voltage to battery charger */

  ret = axp202_getreg8(priv, AXP202_CHARGE1, &reg);

  if (ret != OK)
    {
      return ret;
    }

  reg &= ~(3 << 5);

  switch (value)
  {
  case 4100:
    reg |= (0 << 5);
    break;

  case 4150:
    reg |= (1 << 5);
    break;

  case 4200:
    reg |= (2 << 5);
    break;

  case 4360:
    reg |= (3 << 5);
    break;

  default:
    return -EINVAL;
  }

  ret = axp202_putreg8(priv, AXP202_CHARGE1, reg);
  if (ret != OK)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: axp202_current
 *
 * Description:
 *   Set the battery charger current rate for charging
 *
 ****************************************************************************/

static int axp202_current(FAR struct battery_charger_dev_s *dev, int value)
{
  FAR struct axp202_dev_s *priv = (FAR struct axp202_dev_s *)dev;
  int                      ret;
  uint8_t                  reg = 0;

  /* Set current to battery charger */

  ret = axp202_getreg8(priv, AXP202_CHARGE1, &reg);

  if (ret != OK)
    {
      return ret;
    }

  /**
   * Charge current setting
   * Icharge= [300+(Bit3-0)*100] mA
   */

  value = (value - 300) / 100;
  value &= 0x0f;

  reg &= 0xf0;
  reg |= value;

  ret = axp202_putreg8(priv, AXP202_CHARGE1, reg);
  if (ret != OK)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: axp202_input_current
 *
 * Description:
 *   Set the power-supply input current limit
 *
 ****************************************************************************/

static int axp202_input_current(FAR struct battery_charger_dev_s *dev,
                                int                               value)
{
  FAR struct axp202_dev_s *priv = (FAR struct axp202_dev_s *)dev;
  int                      ret;
  uint8_t                  reg = 0;

  ret = axp202_getreg8(priv, AXP202_IPS_SET, &reg);

  if (ret != OK)
    {
      return ret;
    }

  reg &= ~(3 << 0);

  switch (value)
  {
  case BATTERY_INPUT_CURRENT_EXT_LIM:
    reg |= (3 << 0);
    break;

  case 900:
    reg |= (0 << 0);
    break;

  case 500:
    reg |= (1 << 0);
    break;

  case 100:
    reg |= (2 << 0);
    break;

  default:
    return -EINVAL;
  }

  ret = axp202_putreg8(priv, AXP202_IPS_SET, reg);
  if (ret != OK)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: axp202_operate
 *
 * Description:
 *   Do miscellaneous battery ioctl()
 *
 ****************************************************************************/

static int axp202_operate(FAR struct battery_charger_dev_s *dev,
                          uintptr_t                         param)
{
  return -ENOSYS;
}

static int axp202_set_ldo2_valtage(FAR struct axp202_dev_s *priv,
                                   int                      voltage)
{
  uint8_t reg = 0;

  axp202_getreg8(priv, AXP202_LDO234_DC23_CTL, &reg);

  if (voltage == 0)
    {
      reg &= ~(1 << 2);
    }
  else
    {
      reg |= (1 << 2);
    }

  axp202_putreg8(priv, AXP202_LDO234_DC23_CTL, reg);

  if (voltage != 0)
    {
      axp202_getreg8(priv, AXP202_LDO24OUT_VOL, &reg);
      reg &= ~(0xf << 4);
      reg |= ((((voltage - 1800) / 100) & 0x0f) << 4);
      axp202_putreg8(priv, AXP202_LDO24OUT_VOL, reg);
    }

  return OK;
}

static int axp202_set_ldo3_valtage(FAR struct axp202_dev_s *priv,
                                   int                      voltage)
{
  uint8_t reg = 0;

  axp202_getreg8(priv, AXP202_LDO234_DC23_CTL, &reg);

  if (voltage == 0)
    {
      reg &= ~(1 << 6);
    }
  else
    {
      reg |= (1 << 6);
    }

  axp202_putreg8(priv, AXP202_LDO234_DC23_CTL, reg);

  if (voltage != 0)
    {
      axp202_getreg8(priv, AXP202_LDO3OUT_VOL, &reg);
      reg &= ~(0xf << 4);
      reg |= ((((voltage - 700) / 25) & 0x7f) << 0);
      axp202_putreg8(priv, AXP202_LDO3OUT_VOL, reg);
    }

  return OK;
}

static int axp202_set_ldo4_valtage(FAR struct axp202_dev_s *priv,
                                   int                      voltage)
{
  uint8_t reg = 0;

  axp202_getreg8(priv, AXP202_LDO234_DC23_CTL, &reg);

  if (voltage == 0)
    {
      reg &= ~(1 << 3);
    }
  else
    {
      reg |= (1 << 3);
    }

  axp202_putreg8(priv, AXP202_LDO234_DC23_CTL, reg);

  if (voltage != 0)
    {
      axp202_getreg8(priv, AXP202_LDO24OUT_VOL, &reg);
      reg &= ~(0xf << 0);

      switch (voltage)
      {
      case 1250:
        reg |= 0;  break;
      case 1300:
        reg |= 1;  break;
      case 1400:
        reg |= 2;  break;
      case 1500:
        reg |= 3;  break;
      case 1600:
        reg |= 4;  break;
      case 1700:
        reg |= 5;  break;
      case 1800:
        reg |= 6;  break;
      case 1900:
        reg |= 7;  break;
      case 2000:
        reg |= 8;  break;
      case 2500:
        reg |= 9;  break;
      case 2700:
        reg |= 10; break;
      case 2800:
        reg |= 11; break;
      case 3000:
        reg |= 12; break;
      case 3100:
        reg |= 13; break;
      case 3200:
        reg |= 14; break;
      case 3300:
        reg |= 15; break;
      default:
        break;
      }

      axp202_putreg8(priv, AXP202_LDO24OUT_VOL, reg);
    }

  return OK;
}

static int axp202_set_dc2_valtage(FAR struct axp202_dev_s *priv,
                                  int                      voltage)
{
  uint8_t reg = 0;

  axp202_getreg8(priv, AXP202_LDO234_DC23_CTL, &reg);

  if (voltage == 0)
    {
      reg &= ~(1 << 4);
    }
  else
    {
      reg |= (1 << 4);
    }

  axp202_putreg8(priv, AXP202_LDO234_DC23_CTL, reg);

  if (voltage != 0)
    {
      axp202_getreg8(priv, AXP202_DC2OUT_VOL, &reg);
      reg &= ~(0xf << 4);
      reg |= ((((voltage - 700) / 25) & 0x3f) << 0);
      axp202_putreg8(priv, AXP202_DC2OUT_VOL, reg);
    }

  return OK;
}

static int axp202_set_dc3_valtage(FAR struct axp202_dev_s *priv,
                                  int                      voltage)
{
  uint8_t reg = 0;

  axp202_getreg8(priv, AXP202_LDO234_DC23_CTL, &reg);

  if (voltage == 0)
    {
      reg &= ~(1 << 1);
    }
  else
    {
      reg |= (1 << 1);
    }

  axp202_putreg8(priv, AXP202_LDO234_DC23_CTL, reg);

  if (voltage != 0)
    {
      axp202_getreg8(priv, AXP202_DC3OUT_VOL, &reg);
      reg &= ~(0xf << 4);
      reg |= ((((voltage - 700) / 25) & 0x7f) << 0);
      axp202_putreg8(priv, AXP202_DC3OUT_VOL, reg);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct battery_charger_dev_s *
axp202_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                  uint32_t frequency)
{
  FAR struct axp202_dev_s *priv;
  uint8_t                  chipid = 0;

  /* Initialize the axp202 device structure */

  priv = (FAR struct axp202_dev_s *)kmm_zalloc(sizeof(struct axp202_dev_s));

  if (priv)
    {
      /* Initialize the axp202 device structure */

      priv->dev.ops   = &g_axp202ops;
      priv->i2c       = i2c;
      priv->addr      = addr;
      priv->frequency = frequency;
    }

  axp202_getreg8(priv, AXP202_IC_TYPE, &chipid);

  if (AXP202_CHIP_ID != chipid)
    {
      kmm_free(priv);
      return NULL;
    }

  /* Initialize ldo2 output voltage */

  axp202_set_ldo2_valtage(priv, CONFIG_AXP202_LDO2_VOLTAGE);

  /* Initialize ldo3 output voltage */

  axp202_set_ldo3_valtage(priv, CONFIG_AXP202_LDO3_VOLTAGE);

  /* Initialize ldo4 output voltage */

  axp202_set_ldo4_valtage(priv, CONFIG_AXP202_LDO4_VOLTAGE);

  /* Initialize dc2 output voltage */

  axp202_set_dc2_valtage(priv, CONFIG_AXP202_DC2_VOLTAGE);

  /* Initialize dc3 output voltage */

  axp202_set_dc3_valtage(priv, CONFIG_AXP202_DC3_VOLTAGE);

  return (FAR struct battery_charger_dev_s *)priv;
}

#endif /* CONFIG_I2C && CONFIG_I2C_AXP202 */

/****************************************************************************
 * drivers/power/battery/gd30bc2416.c
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

/* Lower half driver for the GigaDevice GD30BC2416 single-cell switching
 * battery charger with boost converter.
 *
 * The charge current is set by the ISET resistor of the board; the I2C
 * interface can only scale it (100/75/50/25 %).  The charger has a 12-bit
 * ADC that measures the battery voltage, which is reported with
 * BATIOC_GET_VOLTAGE.
 *
 * The charger returns to its reset configuration when the USB power is
 * removed, and clears ADC_EN on some faults.  The driver keeps the
 * configuration that was requested and writes it again when it is lost:
 * on the IRQ of the charger and before each status read.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/power/gd30bc2416.h>

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_GD30BC2416)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Conversion time of the charger ADC: 25 ADC clocks at 2 MHz, rounded up
 * generously.
 */

#define GD30BC2416_ADC_DELAY_US      1000

/* Below this voltage no battery is considered to be connected */

#define GD30BC2416_VBAT_PRESENT_MV   2500

/* The fields of the configuration that the driver maintains */

#define GD30BC2416_CTRL_KEEP         (GD30BC2416_CTRL_ADC_EN | \
                                      GD30BC2416_CTRL_BST_EN | \
                                      GD30BC2416_CTRL_CH_EN)

/* The events reported to the upper half on an IRQ */

#define GD30BC2416_IRQ_EVENTS        (BATTERY_STATE_CHANGED | \
                                      BATTERY_HEALTH_CHANGED | \
                                      BATTERY_ONLINE_CHANGED | \
                                      BATTERY_VOLTAGE_CHANGED)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gd30bc2416_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_charger_dev_s dev; /* Battery charger device */

  /* Data fields specific to the lower half GD30BC2416 driver follow */

  FAR struct i2c_master_s *i2c;     /* I2C interface */

  /* Board configuration */

  FAR const struct gd30bc2416_config_s *config;

  uint16_t ctrl;                    /* Requested CTRL_KEEP bits */
  uint16_t cfg2;                    /* Requested CFG2 ICCCH field */
  uint16_t cfg3;                    /* Requested CFG3 VCCCHT field */
#ifdef CONFIG_SCHED_HPWORK
  struct work_s work;               /* IRQ bottom half */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C support */

static int gd30bc2416_getreg16(FAR struct gd30bc2416_dev_s *priv,
                               uint8_t regaddr, FAR uint16_t *regval);
static int gd30bc2416_putreg16(FAR struct gd30bc2416_dev_s *priv,
                               uint8_t regaddr, uint16_t regval);
static int gd30bc2416_modifyreg16(FAR struct gd30bc2416_dev_s *priv,
                                  uint8_t regaddr, uint16_t clearbits,
                                  uint16_t setbits);
static int gd30bc2416_restore(FAR struct gd30bc2416_dev_s *priv);
static int gd30bc2416_getvbat(FAR struct gd30bc2416_dev_s *priv,
                              FAR int *millivolts);

/* Battery driver lower half methods */

static int gd30bc2416_state(FAR struct battery_charger_dev_s *dev,
                            FAR int *status);
static int gd30bc2416_health(FAR struct battery_charger_dev_s *dev,
                             FAR int *health);
static int gd30bc2416_online(FAR struct battery_charger_dev_s *dev,
                             FAR bool *status);
static int gd30bc2416_voltage(FAR struct battery_charger_dev_s *dev,
                              int value);
static int gd30bc2416_current(FAR struct battery_charger_dev_s *dev,
                              int value);
static int gd30bc2416_input_current(FAR struct battery_charger_dev_s *dev,
                                    int value);
static int gd30bc2416_operate(FAR struct battery_charger_dev_s *dev,
                              uintptr_t param);
static int gd30bc2416_chipid(FAR struct battery_charger_dev_s *dev,
                             FAR unsigned int *value);
static int gd30bc2416_get_voltage(FAR struct battery_charger_dev_s *dev,
                                  FAR int *value);
static int gd30bc2416_voltage_info(FAR struct battery_charger_dev_s *dev,
                                   FAR int *value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_charger_operations_s g_gd30bc2416ops =
{
  gd30bc2416_state,
  gd30bc2416_health,
  gd30bc2416_online,
  gd30bc2416_voltage,
  gd30bc2416_current,
  gd30bc2416_input_current,
  gd30bc2416_operate,
  gd30bc2416_chipid,
  gd30bc2416_get_voltage,
  gd30bc2416_voltage_info,
  NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd30bc2416_getreg16
 *
 * Description:
 *   Read a 16-bit register.  The register address is part of the slave
 *   address, so this is a single read message.
 *
 ****************************************************************************/

static int gd30bc2416_getreg16(FAR struct gd30bc2416_dev_s *priv,
                               uint8_t regaddr, FAR uint16_t *regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret;

  config.frequency = priv->config->frequency;
  config.address   = GD30BC2416_I2C_ADDR(regaddr);
  config.addrlen   = 7;

  ret = i2c_read(priv->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      baterr("ERROR: i2c_read of register 0x%02x failed: %d\n",
             regaddr, ret);
      return ret;
    }

  *regval = ((uint16_t)buffer[0] << 8) | buffer[1];
  batinfo("REG 0x%02x: 0x%04x\n", regaddr, *regval);
  return OK;
}

/****************************************************************************
 * Name: gd30bc2416_putreg16
 *
 * Description:
 *   Write a 16-bit register (high byte first).
 *
 ****************************************************************************/

static int gd30bc2416_putreg16(FAR struct gd30bc2416_dev_s *priv,
                               uint8_t regaddr, uint16_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret;

  config.frequency = priv->config->frequency;
  config.address   = GD30BC2416_I2C_ADDR(regaddr);
  config.addrlen   = 7;

  buffer[0] = regval >> 8;
  buffer[1] = regval & 0xff;

  batinfo("REG 0x%02x <- 0x%04x\n", regaddr, regval);

  ret = i2c_write(priv->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      baterr("ERROR: i2c_write of register 0x%02x failed: %d\n",
             regaddr, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: gd30bc2416_modifyreg16
 *
 * Description:
 *   Read-modify-write a register.  Only the fields that differ are
 *   written.  The command bits of the control register read back as zero,
 *   but they are masked anyway so that a stale value can never trigger a
 *   reset.
 *
 ****************************************************************************/

static int gd30bc2416_modifyreg16(FAR struct gd30bc2416_dev_s *priv,
                                  uint8_t regaddr, uint16_t clearbits,
                                  uint16_t setbits)
{
  uint16_t oldval;
  uint16_t newval;
  int ret;

  ret = gd30bc2416_getreg16(priv, regaddr, &oldval);
  if (ret < 0)
    {
      return ret;
    }

  if (regaddr == GD30BC2416_REG_CTRL)
    {
      oldval &= ~GD30BC2416_CTRL_CMD_MASK;
    }

  newval = (oldval & ~clearbits) | setbits;
  if (newval == oldval)
    {
      return OK;
    }

  return gd30bc2416_putreg16(priv, regaddr, newval);
}

/****************************************************************************
 * Name: gd30bc2416_restore
 *
 * Description:
 *   Write the requested configuration again where the charger has lost it
 *   (USB power removed, fault).
 *
 ****************************************************************************/

static int gd30bc2416_restore(FAR struct gd30bc2416_dev_s *priv)
{
  int ret;

  ret = gd30bc2416_modifyreg16(priv, GD30BC2416_REG_CFG2,
                               GD30BC2416_CFG2_ICCCH_MASK, priv->cfg2);
  if (ret < 0)
    {
      return ret;
    }

  ret = gd30bc2416_modifyreg16(priv, GD30BC2416_REG_CFG3,
                               GD30BC2416_CFG3_VCCCHT_MASK, priv->cfg3);
  if (ret < 0)
    {
      return ret;
    }

  /* The control register last, so that the charge only starts with the
   * charge current and voltage in place.
   */

  return gd30bc2416_modifyreg16(priv, GD30BC2416_REG_CTRL,
                                GD30BC2416_CTRL_KEEP, priv->ctrl);
}

/****************************************************************************
 * Name: gd30bc2416_getvbat
 *
 * Description:
 *   Convert the battery voltage with the charger ADC.
 *
 ****************************************************************************/

static int gd30bc2416_getvbat(FAR struct gd30bc2416_dev_s *priv,
                              FAR int *millivolts)
{
  uint16_t regval;
  int ret;

  /* The ADC may have been switched off by the charger */

  ret = gd30bc2416_restore(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Select the VBAT channel, which also starts a new conversion */

  ret = gd30bc2416_putreg16(priv, GD30BC2416_REG_ADC,
                            GD30BC2416_ADC_CH_VBAT);
  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(GD30BC2416_ADC_DELAY_US);

  ret = gd30bc2416_getreg16(priv, GD30BC2416_REG_ADC, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval = (regval & GD30BC2416_ADC_DATA_MASK) >> GD30BC2416_ADC_DATA_SHIFT;
  *millivolts = GD30BC2416_ADC_VBAT_MV(regval);
  return OK;
}

#ifdef CONFIG_SCHED_HPWORK
/****************************************************************************
 * Name: gd30bc2416_worker
 *
 * Description:
 *   Bottom half of the IRQ: restore the configuration and notify the
 *   upper half.
 *
 ****************************************************************************/

static void gd30bc2416_worker(FAR void *arg)
{
  FAR struct gd30bc2416_dev_s *priv = arg;
  uint16_t fault = 0;
  uint16_t status1 = 0;
  int ret;

  ret = nxmutex_lock(&priv->dev.batlock);
  if (ret < 0)
    {
      return;
    }

  gd30bc2416_getreg16(priv, GD30BC2416_REG_FAULT, &fault);
  gd30bc2416_getreg16(priv, GD30BC2416_REG_STATUS1, &status1);
  batinfo("IRQ: FAULT 0x%04x STATUS1 0x%04x\n", fault, status1);

  ret = gd30bc2416_restore(priv);
  nxmutex_unlock(&priv->dev.batlock);

  if (ret < 0)
    {
      baterr("ERROR: Failed to restore the configuration: %d\n", ret);
    }

  battery_charger_changed(&priv->dev, GD30BC2416_IRQ_EVENTS);
}

/****************************************************************************
 * Name: gd30bc2416_interrupt
 ****************************************************************************/

static int gd30bc2416_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct gd30bc2416_dev_s *priv = arg;

  /* The pulse gives two edges: one bottom half is enough */

  if (work_available(&priv->work))
    {
      work_queue(HPWORK, &priv->work, gd30bc2416_worker, priv, 0);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: gd30bc2416_state
 *
 * Description:
 *   Return the current battery state
 *
 ****************************************************************************/

static int gd30bc2416_state(FAR struct battery_charger_dev_s *dev,
                            FAR int *status)
{
  FAR struct gd30bc2416_dev_s *priv = (FAR struct gd30bc2416_dev_s *)dev;
  uint16_t fault;
  uint16_t status1;
  uint16_t status2;
  int ret;

  *status = BATTERY_UNKNOWN;

  ret = gd30bc2416_restore(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = gd30bc2416_getreg16(priv, GD30BC2416_REG_FAULT, &fault);
  if (ret < 0)
    {
      return ret;
    }

  ret = gd30bc2416_getreg16(priv, GD30BC2416_REG_STATUS1, &status1);
  if (ret < 0)
    {
      return ret;
    }

  ret = gd30bc2416_getreg16(priv, GD30BC2416_REG_STATUS2, &status2);
  if (ret < 0)
    {
      return ret;
    }

  if ((fault & ~GD30BC2416_FAULT_I2C_ACK) != 0)
    {
      *status = BATTERY_FAULT;
    }
  else if ((status1 & GD30BC2416_STATUS1_CH_END) != 0)
    {
      *status = BATTERY_FULL;
    }
  else if ((status1 & GD30BC2416_STATUS1_USB_PLUG) != 0)
    {
      *status = (status2 & GD30BC2416_STATUS2_CH_EN) != 0 ?
                BATTERY_CHARGING : BATTERY_IDLE;
    }
  else
    {
      *status = BATTERY_DISCHARGING;
    }

  return OK;
}

/****************************************************************************
 * Name: gd30bc2416_health
 *
 * Description:
 *   Return the current battery health state
 *
 ****************************************************************************/

static int gd30bc2416_health(FAR struct battery_charger_dev_s *dev,
                             FAR int *health)
{
  FAR struct gd30bc2416_dev_s *priv = (FAR struct gd30bc2416_dev_s *)dev;
  uint16_t fault;
  int ret;

  ret = gd30bc2416_getreg16(priv, GD30BC2416_REG_FAULT, &fault);
  if (ret < 0)
    {
      *health = BATTERY_HEALTH_UNKNOWN;
      return ret;
    }

  if ((fault & (GD30BC2416_FAULT_TEMP | GD30BC2416_FAULT_NTC_HOT)) != 0)
    {
      *health = BATTERY_HEALTH_OVERHEAT;
    }
  else if ((fault & GD30BC2416_FAULT_NTC_COLD) != 0)
    {
      *health = BATTERY_HEALTH_COLD;
    }
  else if ((fault & (GD30BC2416_FAULT_VBAT_OV |
                     GD30BC2416_FAULT_VUSB_OV)) != 0)
    {
      *health = BATTERY_HEALTH_OVERVOLTAGE;
    }
  else if ((fault & GD30BC2416_FAULT_VBAT_UV) != 0)
    {
      *health = BATTERY_HEALTH_UNDERVOLTAGE;
    }
  else if ((fault & GD30BC2416_FAULT_VUSB_OC) != 0)
    {
      *health = BATTERY_HEALTH_OVERCURRENT;
    }
  else if ((fault & GD30BC2416_FAULT_WDG) != 0)
    {
      *health = BATTERY_HEALTH_WD_TMR_EXP;
    }
  else if ((fault & (GD30BC2416_FAULT_CH_TO |
                     GD30BC2416_FAULT_PCH_TO)) != 0)
    {
      *health = BATTERY_HEALTH_SAFE_TMR_EXP;
    }
  else if ((fault & (GD30BC2416_FAULT_VSYS_OV |
                     GD30BC2416_FAULT_VSYS_UV)) != 0)
    {
      *health = BATTERY_HEALTH_UNSPEC_FAIL;
    }
  else
    {
      *health = BATTERY_HEALTH_GOOD;
    }

  return OK;
}

/****************************************************************************
 * Name: gd30bc2416_online
 *
 * Description:
 *   Return true if a battery is connected.  The charger has no battery
 *   detection, so this is based on the battery voltage.
 *
 ****************************************************************************/

static int gd30bc2416_online(FAR struct battery_charger_dev_s *dev,
                             FAR bool *status)
{
  FAR struct gd30bc2416_dev_s *priv = (FAR struct gd30bc2416_dev_s *)dev;
  int millivolts;
  int ret;

  ret = gd30bc2416_getvbat(priv, &millivolts);
  if (ret < 0)
    {
      *status = false;
      return ret;
    }

  *status = millivolts >= GD30BC2416_VBAT_PRESENT_MV;
  return OK;
}

/****************************************************************************
 * Name: gd30bc2416_voltage
 *
 * Description:
 *   Set the charge termination voltage, in mV
 *
 ****************************************************************************/

static int gd30bc2416_voltage(FAR struct battery_charger_dev_s *dev,
                              int value)
{
  FAR struct gd30bc2416_dev_s *priv = (FAR struct gd30bc2416_dev_s *)dev;

  switch (value)
    {
      case 4100:
        priv->cfg3 = GD30BC2416_CFG3_VCCCHT_4100;
        break;

      case 4200:
        priv->cfg3 = GD30BC2416_CFG3_VCCCHT_4200;
        break;

      case 4300:
        priv->cfg3 = GD30BC2416_CFG3_VCCCHT_4300;
        break;

      case 4350:
        priv->cfg3 = GD30BC2416_CFG3_VCCCHT_4350;
        break;

      case 4400:
        priv->cfg3 = GD30BC2416_CFG3_VCCCHT_4400;
        break;

      default:
        baterr("ERROR: Unsupported charge voltage: %d mV\n", value);
        return -EINVAL;
    }

  return gd30bc2416_restore(priv);
}

/****************************************************************************
 * Name: gd30bc2416_current
 *
 * Description:
 *   Set the constant current charge current, in mA.  The value is rounded
 *   down to the nearest step (100/75/50/25 % of the ISET current).
 *
 ****************************************************************************/

static int gd30bc2416_current(FAR struct battery_charger_dev_s *dev,
                              int value)
{
  FAR struct gd30bc2416_dev_s *priv = (FAR struct gd30bc2416_dev_s *)dev;
  int percent;

  if (value <= 0)
    {
      return -EINVAL;
    }

  percent = (value * 100) / priv->config->current;

  if (percent >= 100)
    {
      priv->cfg2 = GD30BC2416_CFG2_ICCCH_100;
    }
  else if (percent >= 75)
    {
      priv->cfg2 = GD30BC2416_CFG2_ICCCH_75;
    }
  else if (percent >= 50)
    {
      priv->cfg2 = GD30BC2416_CFG2_ICCCH_50;
    }
  else
    {
      priv->cfg2 = GD30BC2416_CFG2_ICCCH_25;
    }

  return gd30bc2416_restore(priv);
}

/****************************************************************************
 * Name: gd30bc2416_input_current
 *
 * Description:
 *   The input current limit is relative to the charge current and cannot
 *   be set in mA.
 *
 ****************************************************************************/

static int gd30bc2416_input_current(FAR struct battery_charger_dev_s *dev,
                                    int value)
{
  return -ENOTSUP;
}

/****************************************************************************
 * Name: gd30bc2416_operate
 *
 * Description:
 *   Do a device specific operation:
 *
 *   BATIO_OPRTN_CHARGE   - Enable (u32 != 0) or disable the charge
 *   BATIO_OPRTN_BOOST    - Enable (u32 != 0) or disable the 5 V boost
 *   BATIO_OPRTN_SHIPMODE - Enter the sleep mode
 *   BATIO_OPRTN_RESET    - Reset the charger, except its I2C interface,
 *                          and write the configuration again
 *   BATIO_OPRTN_WDOG     - Enable (u32 != 0) or disable the watchdog
 *
 ****************************************************************************/

static int gd30bc2416_operate(FAR struct battery_charger_dev_s *dev,
                              uintptr_t param)
{
  FAR struct gd30bc2416_dev_s *priv = (FAR struct gd30bc2416_dev_s *)dev;
  FAR struct batio_operate_msg_s *msg =
    (FAR struct batio_operate_msg_s *)param;
  bool enable = msg->u32 != 0;
  int ret;

  switch (msg->operate_type)
    {
      case BATIO_OPRTN_CHARGE:
        if (enable)
          {
            priv->ctrl |= GD30BC2416_CTRL_CH_EN;
          }
        else
          {
            priv->ctrl &= ~GD30BC2416_CTRL_CH_EN;
          }

        ret = gd30bc2416_restore(priv);
        break;

      case BATIO_OPRTN_BOOST:
        if (enable)
          {
            priv->ctrl |= GD30BC2416_CTRL_BST_EN;
          }
        else
          {
            priv->ctrl &= ~GD30BC2416_CTRL_BST_EN;
          }

        ret = gd30bc2416_restore(priv);
        break;

      case BATIO_OPRTN_SHIPMODE:
        ret = gd30bc2416_modifyreg16(priv, GD30BC2416_REG_CTRL, 0,
                                     GD30BC2416_CTRL_SLP_EN);
        break;

      case BATIO_OPRTN_RESET:
        ret = gd30bc2416_putreg16(priv, GD30BC2416_REG_CTRL,
                                  GD30BC2416_CTRL_RST_OTHS);
        if (ret >= 0)
          {
            ret = gd30bc2416_restore(priv);
          }
        break;

      case BATIO_OPRTN_WDOG:
        ret = gd30bc2416_modifyreg16(priv, GD30BC2416_REG_WDG,
                                     enable ? 0 : GD30BC2416_WDG_EN,
                                     enable ? GD30BC2416_WDG_EN : 0);
        break;

      default:
        baterr("ERROR: Unsupported operation: %d\n", msg->operate_type);
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: gd30bc2416_chipid
 *
 * Description:
 *   The GD30BC2416 has no identification register.
 *
 ****************************************************************************/

static int gd30bc2416_chipid(FAR struct battery_charger_dev_s *dev,
                             FAR unsigned int *value)
{
  return -ENOTSUP;
}

/****************************************************************************
 * Name: gd30bc2416_get_voltage
 *
 * Description:
 *   Return the battery voltage, in mV
 *
 ****************************************************************************/

static int gd30bc2416_get_voltage(FAR struct battery_charger_dev_s *dev,
                                  FAR int *value)
{
  FAR struct gd30bc2416_dev_s *priv = (FAR struct gd30bc2416_dev_s *)dev;

  return gd30bc2416_getvbat(priv, value);
}

/****************************************************************************
 * Name: gd30bc2416_voltage_info
 ****************************************************************************/

static int gd30bc2416_voltage_info(FAR struct battery_charger_dev_s *dev,
                                   FAR int *value)
{
  return -ENOTSUP;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd30bc2416_initialize
 *
 * Description:
 *   Initialize the GD30BC2416 battery charger driver and return an
 *   instance of the lower-half interface that may be used with
 *   battery_charger_register().
 *
 ****************************************************************************/

FAR struct battery_charger_dev_s *
gd30bc2416_initialize(FAR struct i2c_master_s *i2c,
                      FAR const struct gd30bc2416_config_s *config)
{
  FAR struct gd30bc2416_dev_s *priv;
  uint16_t regval;
  int ret;

  DEBUGASSERT(i2c != NULL && config != NULL && config->current > 0);

  priv = kmm_zalloc(sizeof(struct gd30bc2416_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->dev.ops = &g_gd30bc2416ops;
  priv->i2c     = i2c;
  priv->config  = config;

  /* There is no identification register: check that the watchdog
   * register, whose reserved bits read as zero, answers.
   */

  ret = gd30bc2416_getreg16(priv, GD30BC2416_REG_WDG, &regval);
  if (ret < 0 || (regval & 0xfc00) != 0)
    {
      baterr("ERROR: No GD30BC2416 found: %d 0x%04x\n", ret, regval);
      kmm_free(priv);
      return NULL;
    }

  /* Start from the reset configuration of the charger (registers survive
   * a reset of the MCU), with the ADC enabled for the battery voltage.
   */

  priv->ctrl = GD30BC2416_CTRL_ADC_EN;
#ifdef CONFIG_GD30BC2416_CHARGE
  priv->ctrl |= GD30BC2416_CTRL_CH_EN;
#endif
  priv->cfg2 = GD30BC2416_CFG2_ICCCH_100;
  priv->cfg3 = GD30BC2416_CFG3_VCCCHT_4200;

  ret = gd30bc2416_restore(priv);
  if (ret < 0)
    {
      kmm_free(priv);
      return NULL;
    }

#ifdef CONFIG_SCHED_HPWORK
  if (config->irq_attach != NULL)
    {
      ret = config->irq_attach(config, gd30bc2416_interrupt, priv);
      if (ret < 0)
        {
          baterr("ERROR: Failed to attach the IRQ: %d\n", ret);
        }
    }
#endif

  return &priv->dev;
}

#endif /* CONFIG_I2C && CONFIG_I2C_GD30BC2416 */

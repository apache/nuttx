/****************************************************************************
 * drivers/power/battery/axp2101.c
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

/* Lower halves of the battery charger and battery gauge drivers for the
 * X-Powers AXP2101 PMIC.  The power rails are left as programmed by the
 * OTP of the chip.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/power/axp2101.h>

#ifdef CONFIG_AXP2101_GAUGE
#  include <nuttx/power/battery_gauge.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers */

#define AXP2101_STATUS1         0x00  /* PMU status 1 */
#define AXP2101_STATUS2         0x01  /* PMU status 2 */
#define AXP2101_CHIPID          0x03  /* Chip ID */
#define AXP2101_COMMON_CFG      0x10  /* PMU common configuration */
#define AXP2101_ICL             0x16  /* Input current limit */
#define AXP2101_VOFF            0x24  /* Power-off voltage */
#define AXP2101_MODULE_EN       0x18  /* Charger, gauge, watchdog enable */
#define AXP2101_ADC_EN          0x30  /* ADC channel enable */
#define AXP2101_VBAT_H          0x34  /* Battery voltage, bits 13:8 */
#define AXP2101_VBAT_L          0x35  /* Battery voltage, bits 7:0 */
#define AXP2101_VBUS_H          0x38  /* VBUS voltage, bits 13:8 */
#define AXP2101_VBUS_L          0x39  /* VBUS voltage, bits 7:0 */
#define AXP2101_TDIE_H          0x3c  /* Die temperature, bits 13:8 */
#define AXP2101_TDIE_L          0x3d  /* Die temperature, bits 7:0 */
#define AXP2101_IRQ_STATUS0     0x48  /* IRQ status 0 */
#define AXP2101_IRQ_STATUS2     0x4a  /* IRQ status 2 */
#define AXP2101_ICC             0x62  /* Constant charge current */
#define AXP2101_ITERM           0x63  /* Termination current and control */
#define AXP2101_CV              0x64  /* Charge voltage */
#define AXP2101_SOC             0xa4  /* Battery percentage */

/* AXP2101_STATUS1 */

#define STATUS1_VBUS_GOOD       (1 << 5)
#define STATUS1_BAT_PRESENT     (1 << 3)

/* AXP2101_STATUS2 */

#define STATUS2_DIR_SHIFT       5
#define STATUS2_DIR_MASK        (3 << STATUS2_DIR_SHIFT)
#define STATUS2_DIR_STANDBY     (0 << STATUS2_DIR_SHIFT)
#define STATUS2_DIR_CHARGE      (1 << STATUS2_DIR_SHIFT)
#define STATUS2_DIR_DISCHARGE   (2 << STATUS2_DIR_SHIFT)
#define STATUS2_CHG_MASK        0x07
#define STATUS2_CHG_CV          0x03  /* 0 to 3: trickle, pre, CC, CV */
#define STATUS2_CHG_DONE        0x04

/* AXP2101_COMMON_CFG */

#define COMMON_CFG_PWROFF       (1 << 0)

/* AXP2101_MODULE_EN */

#define MODULE_EN_GAUGE         (1 << 3)

/* AXP2101_ADC_EN */

#define ADC_EN_VBAT             (1 << 0)
#define ADC_EN_VBUS             (1 << 2)
#define ADC_EN_VSYS             (1 << 3)
#define ADC_EN_TDIE             (1 << 4)

/* AXP2101_IRQ_STATUS0 */

#define IRQ0_BAT_OT_CHG         (1 << 3)
#define IRQ0_BAT_UT_CHG         (1 << 2)
#define IRQ0_BAT_OT_WORK        (1 << 1)
#define IRQ0_BAT_UT_WORK        (1 << 0)

/* AXP2101_IRQ_STATUS2 */

#define IRQ2_SAFETY_TIMER       (1 << 1)
#define IRQ2_BAT_OVP            (1 << 0)

/* The fault flags used for the battery health */

#define IRQ0_HEALTH             (IRQ0_BAT_OT_CHG | IRQ0_BAT_UT_CHG | \
                                 IRQ0_BAT_OT_WORK | IRQ0_BAT_UT_WORK)
#define IRQ2_HEALTH             (IRQ2_SAFETY_TIMER | IRQ2_BAT_OVP)

/* AXP2101_ITERM */

#define ITERM_EN                (1 << 4)

/* Field masks */

#define ICL_MASK                0x07
#define VOFF_MASK               0x07
#define ICC_MASK                0x1f
#define CV_MASK                 0x07

/* ADC results are 14 bits, 1 mV per LSB for the voltage channels */

#define ADC_H_MASK              0x3f

#if CONFIG_AXP2101_VOFF_VOLTAGE != 0 && \
    (CONFIG_AXP2101_VOFF_VOLTAGE < 2600 || \
     CONFIG_AXP2101_VOFF_VOLTAGE % 100 != 0)
#  error "CONFIG_AXP2101_VOFF_VOLTAGE must be 0 or 2600..3300 in 100 mV steps"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct axp2101_bus_s
{
  FAR struct i2c_master_s *i2c;       /* I2C interface */
  uint8_t                  addr;      /* I2C address */
  uint32_t                 frequency; /* I2C frequency */
};

struct axp2101_dev_s
{
  struct battery_charger_dev_s dev;   /* Must be first */
  struct axp2101_bus_s         bus;
};

#ifdef CONFIG_AXP2101_GAUGE
struct axp2101_gauge_s
{
  struct battery_gauge_dev_s   dev;   /* Must be first */
  struct axp2101_bus_s         bus;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int axp2101_state(FAR struct battery_charger_dev_s *dev,
                         FAR int *status);
static int axp2101_health(FAR struct battery_charger_dev_s *dev,
                          FAR int *health);
static int axp2101_online(FAR struct battery_charger_dev_s *dev,
                          FAR bool *status);
static int axp2101_voltage(FAR struct battery_charger_dev_s *dev,
                           int value);
static int axp2101_current(FAR struct battery_charger_dev_s *dev,
                           int value);
static int axp2101_input_current(FAR struct battery_charger_dev_s *dev,
                                 int value);
static int axp2101_operate(FAR struct battery_charger_dev_s *dev,
                           uintptr_t param);
static int axp2101_chipid(FAR struct battery_charger_dev_s *dev,
                          FAR unsigned int *value);
static int axp2101_get_voltage(FAR struct battery_charger_dev_s *dev,
                               FAR int *value);
static int axp2101_voltage_info(FAR struct battery_charger_dev_s *dev,
                                FAR int *value);

#ifdef CONFIG_AXP2101_GAUGE
static int axp2101_gauge_state(FAR struct battery_gauge_dev_s *dev,
                               FAR int *status);
static int axp2101_gauge_online(FAR struct battery_gauge_dev_s *dev,
                                FAR bool *status);
static int axp2101_gauge_voltage(FAR struct battery_gauge_dev_s *dev,
                                 FAR int *value);
static int axp2101_gauge_capacity(FAR struct battery_gauge_dev_s *dev,
                                  FAR int *value);
static int axp2101_gauge_temp(FAR struct battery_gauge_dev_s *dev,
                              FAR int *value);
static int axp2101_gauge_chipid(FAR struct battery_gauge_dev_s *dev,
                                FAR unsigned int *value);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_charger_operations_s g_axp2101ops =
{
  axp2101_state,
  axp2101_health,
  axp2101_online,
  axp2101_voltage,
  axp2101_current,
  axp2101_input_current,
  axp2101_operate,
  axp2101_chipid,
  axp2101_get_voltage,
  axp2101_voltage_info,
  NULL                    /* get_protocol */
};

#ifdef CONFIG_AXP2101_GAUGE
static const struct battery_gauge_operations_s g_axp2101_gaugeops =
{
  axp2101_gauge_state,
  axp2101_gauge_online,
  axp2101_gauge_voltage,
  axp2101_gauge_capacity,
  NULL,                   /* current: no current ADC */
  axp2101_gauge_temp,
  axp2101_gauge_chipid,
  NULL                    /* operate */
};
#endif

/* Input current limit steps in mA, index is the register value */

static const uint16_t g_axp2101_icl[] =
{
  100, 500, 900, 1000, 1500, 2000
};

/* Charge voltage steps in mV, index is the register value (0 is 5.0 V and
 * is never selected).
 */

static const uint16_t g_axp2101_cv[] =
{
  0, 4000, 4100, 4200, 4350, 4400
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int axp2101_getreg(FAR struct axp2101_bus_s *bus, uint8_t regaddr,
                          FAR uint8_t *regval)
{
  struct i2c_config_s config;
  int ret;

  config.frequency = bus->frequency;
  config.address   = bus->addr;
  config.addrlen   = 7;

  ret = i2c_writeread(bus->i2c, &config, &regaddr, 1, regval, 1);
  if (ret < 0)
    {
      baterr("ERROR: read of reg 0x%02x failed: %d\n", regaddr, ret);
      return ret;
    }

  return OK;
}

static int axp2101_putreg(FAR struct axp2101_bus_s *bus, uint8_t regaddr,
                          uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret;

  config.frequency = bus->frequency;
  config.address   = bus->addr;
  config.addrlen   = 7;

  buffer[0] = regaddr;
  buffer[1] = regval;

  ret = i2c_write(bus->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      baterr("ERROR: write of reg 0x%02x failed: %d\n", regaddr, ret);
      return ret;
    }

  return OK;
}

static int axp2101_modifyreg(FAR struct axp2101_bus_s *bus, uint8_t regaddr,
                             uint8_t clearbits, uint8_t setbits)
{
  uint8_t regval;
  int ret;

  ret = axp2101_getreg(bus, regaddr, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~clearbits;
  regval |= setbits;

  return axp2101_putreg(bus, regaddr, regval);
}

/* Read a 14-bit ADC result.  The high byte must be read first. */

static int axp2101_getadc(FAR struct axp2101_bus_s *bus, uint8_t regh,
                          FAR int *value)
{
  uint8_t high;
  uint8_t low;
  int ret;

  ret = axp2101_getreg(bus, regh, &high);
  if (ret < 0)
    {
      return ret;
    }

  ret = axp2101_getreg(bus, regh + 1, &low);
  if (ret < 0)
    {
      return ret;
    }

  *value = ((high & ADC_H_MASK) << 8) | low;
  return OK;
}

/* Check the chip ID and enable the ADC channels used by the driver. */

static int axp2101_probe(FAR struct axp2101_bus_s *bus)
{
  uint8_t chipid;
  int ret;

  ret = axp2101_getreg(bus, AXP2101_CHIPID, &chipid);
  if (ret < 0)
    {
      return ret;
    }

  if (chipid != AXP2101_CHIP_ID)
    {
      baterr("ERROR: unexpected chip ID 0x%02x\n", chipid);
      return -ENODEV;
    }

  ret = axp2101_modifyreg(bus, AXP2101_ADC_EN, 0,
                          ADC_EN_VBAT | ADC_EN_VBUS | ADC_EN_VSYS |
                          ADC_EN_TDIE);

#if CONFIG_AXP2101_VOFF_VOLTAGE > 0
  /* Power off below this battery voltage, to protect the battery from deep
   * discharge (2.6 V + 0.1 V per step).
   */

  if (ret >= 0)
    {
      ret = axp2101_modifyreg(bus, AXP2101_VOFF, VOFF_MASK,
                              (CONFIG_AXP2101_VOFF_VOLTAGE - 2600) / 100);
    }
#endif

  return ret;
}

static int axp2101_bat_state(FAR struct axp2101_bus_s *bus,
                             FAR int *status)
{
  uint8_t status1;
  uint8_t status2;
  uint8_t chg;
  int ret;

  *status = BATTERY_UNKNOWN;

  ret = axp2101_getreg(bus, AXP2101_STATUS1, &status1);
  if (ret < 0)
    {
      return ret;
    }

  ret = axp2101_getreg(bus, AXP2101_STATUS2, &status2);
  if (ret < 0)
    {
      return ret;
    }

  chg = status2 & STATUS2_CHG_MASK;

  if ((status1 & STATUS1_BAT_PRESENT) == 0)
    {
      /* No battery: BATIOC_HEALTH reports it as disconnected */

      *status = BATTERY_IDLE;
    }
  else if (chg == STATUS2_CHG_DONE)
    {
      *status = BATTERY_FULL;
    }
  else if ((status2 & STATUS2_DIR_MASK) == STATUS2_DIR_CHARGE ||
           chg <= STATUS2_CHG_CV)
    {
      *status = BATTERY_CHARGING;
    }
  else if ((status2 & STATUS2_DIR_MASK) == STATUS2_DIR_DISCHARGE)
    {
      *status = BATTERY_DISCHARGING;
    }
  else
    {
      *status = BATTERY_IDLE;
    }

  return OK;
}

/****************************************************************************
 * Name: axp2101_state
 *
 * Description:
 *   Return the battery state (enum battery_status_e).
 *
 ****************************************************************************/

static int axp2101_state(FAR struct battery_charger_dev_s *dev,
                         FAR int *status)
{
  FAR struct axp2101_dev_s *priv = (FAR struct axp2101_dev_s *)dev;

  return axp2101_bat_state(&priv->bus, status);
}

/****************************************************************************
 * Name: axp2101_health
 *
 * Description:
 *   Return the battery health (enum battery_health_e).  The fault flags
 *   are IRQ status bits, which the chip sets only for the IRQs enabled in
 *   REG 40h-42h.  The driver does not change these enables.  The flags
 *   are latched until cleared, so each fault event is reported once and
 *   then cleared.
 *
 ****************************************************************************/

static int axp2101_health(FAR struct battery_charger_dev_s *dev,
                          FAR int *health)
{
  FAR struct axp2101_dev_s *priv = (FAR struct axp2101_dev_s *)dev;
  uint8_t status1;
  uint8_t irq0;
  uint8_t irq2;
  int ret;

  *health = BATTERY_HEALTH_UNKNOWN;

  ret = axp2101_getreg(&priv->bus, AXP2101_STATUS1, &status1);
  if (ret < 0)
    {
      return ret;
    }

  ret = axp2101_getreg(&priv->bus, AXP2101_IRQ_STATUS0, &irq0);
  if (ret < 0)
    {
      return ret;
    }

  ret = axp2101_getreg(&priv->bus, AXP2101_IRQ_STATUS2, &irq2);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear the latched fault flags that were seen (write 1 to clear) */

  irq0 &= IRQ0_HEALTH;
  irq2 &= IRQ2_HEALTH;

  if (irq0 != 0)
    {
      ret = axp2101_putreg(&priv->bus, AXP2101_IRQ_STATUS0, irq0);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (irq2 != 0)
    {
      ret = axp2101_putreg(&priv->bus, AXP2101_IRQ_STATUS2, irq2);
      if (ret < 0)
        {
          return ret;
        }
    }

  if ((status1 & STATUS1_BAT_PRESENT) == 0)
    {
      *health = BATTERY_HEALTH_DISCONNECTED;
    }
  else if (irq2 & IRQ2_BAT_OVP)
    {
      *health = BATTERY_HEALTH_OVERVOLTAGE;
    }
  else if (irq0 & (IRQ0_BAT_OT_CHG | IRQ0_BAT_OT_WORK))
    {
      *health = BATTERY_HEALTH_OVERHEAT;
    }
  else if (irq0 & (IRQ0_BAT_UT_CHG | IRQ0_BAT_UT_WORK))
    {
      *health = BATTERY_HEALTH_COLD;
    }
  else if (irq2 & IRQ2_SAFETY_TIMER)
    {
      *health = BATTERY_HEALTH_SAFE_TMR_EXP;
    }
  else
    {
      *health = BATTERY_HEALTH_GOOD;
    }

  return OK;
}

/****************************************************************************
 * Name: axp2101_online
 *
 * Description:
 *   Return true if a good VBUS input is present.
 *
 ****************************************************************************/

static int axp2101_online(FAR struct battery_charger_dev_s *dev,
                          FAR bool *status)
{
  FAR struct axp2101_dev_s *priv = (FAR struct axp2101_dev_s *)dev;
  uint8_t regval;
  int ret;

  ret = axp2101_getreg(&priv->bus, AXP2101_STATUS1, &regval);
  if (ret < 0)
    {
      return ret;
    }

  *status = (regval & STATUS1_VBUS_GOOD) != 0;
  return OK;
}

/****************************************************************************
 * Name: axp2101_voltage
 *
 * Description:
 *   Set the charge voltage in mV: 4000, 4100, 4200, 4350 or 4400.
 *
 ****************************************************************************/

static int axp2101_voltage(FAR struct battery_charger_dev_s *dev,
                           int value)
{
  FAR struct axp2101_dev_s *priv = (FAR struct axp2101_dev_s *)dev;
  int i;

  for (i = 1; i < nitems(g_axp2101_cv); i++)
    {
      if (g_axp2101_cv[i] == value)
        {
          return axp2101_modifyreg(&priv->bus, AXP2101_CV, CV_MASK, i);
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: axp2101_current
 *
 * Description:
 *   Set the constant charge current in mA, from 0 to 1000.  The value is
 *   rounded down to a step of 25 mA up to 200 mA and of 100 mA above.
 *
 ****************************************************************************/

static int axp2101_current(FAR struct battery_charger_dev_s *dev,
                           int value)
{
  FAR struct axp2101_dev_s *priv = (FAR struct axp2101_dev_s *)dev;
  uint8_t regval;

  if (value < 0 || value > 1000)
    {
      return -EINVAL;
    }

  if (value <= 200)
    {
      regval = value / 25;
    }
  else
    {
      regval = 8 + (value - 200) / 100;
    }

  return axp2101_modifyreg(&priv->bus, AXP2101_ICC, ICC_MASK, regval);
}

/****************************************************************************
 * Name: axp2101_input_current
 *
 * Description:
 *   Set the VBUS input current limit in mA, rounded down to one of 100,
 *   500, 900, 1000, 1500 or 2000.
 *
 ****************************************************************************/

static int axp2101_input_current(FAR struct battery_charger_dev_s *dev,
                                 int value)
{
  FAR struct axp2101_dev_s *priv = (FAR struct axp2101_dev_s *)dev;
  int i;

  if (value < g_axp2101_icl[0])
    {
      return -EINVAL;
    }

  for (i = nitems(g_axp2101_icl) - 1; i > 0; i--)
    {
      if (value >= g_axp2101_icl[i])
        {
          break;
        }
    }

  return axp2101_modifyreg(&priv->bus, AXP2101_ICL, ICL_MASK, i);
}

/****************************************************************************
 * Name: axp2101_operate
 *
 * Description:
 *   Supported operations:
 *     BATIO_OPRTN_EN_TERM - u32 != 0 enables the charge termination
 *     BATIO_OPRTN_SYSOFF  - Turn off the PMIC outputs
 *
 ****************************************************************************/

static int axp2101_operate(FAR struct battery_charger_dev_s *dev,
                           uintptr_t param)
{
  FAR struct axp2101_dev_s *priv = (FAR struct axp2101_dev_s *)dev;
  FAR struct batio_operate_msg_s *msg =
    (FAR struct batio_operate_msg_s *)param;

  switch (msg->operate_type)
    {
      case BATIO_OPRTN_EN_TERM:
        return axp2101_modifyreg(&priv->bus, AXP2101_ITERM, ITERM_EN,
                                 msg->u32 != 0 ? ITERM_EN : 0);

      case BATIO_OPRTN_SYSOFF:
        return axp2101_poweroff(dev);

      default:
        return -ENOSYS;
    }
}

/****************************************************************************
 * Name: axp2101_chipid
 ****************************************************************************/

static int axp2101_chipid(FAR struct battery_charger_dev_s *dev,
                          FAR unsigned int *value)
{
  FAR struct axp2101_dev_s *priv = (FAR struct axp2101_dev_s *)dev;
  uint8_t regval;
  int ret;

  ret = axp2101_getreg(&priv->bus, AXP2101_CHIPID, &regval);
  if (ret < 0)
    {
      return ret;
    }

  *value = regval;
  return OK;
}

/****************************************************************************
 * Name: axp2101_get_voltage
 *
 * Description:
 *   Return the measured battery voltage in mV.
 *
 ****************************************************************************/

static int axp2101_get_voltage(FAR struct battery_charger_dev_s *dev,
                               FAR int *value)
{
  FAR struct axp2101_dev_s *priv = (FAR struct axp2101_dev_s *)dev;

  return axp2101_getadc(&priv->bus, AXP2101_VBAT_H, value);
}

/****************************************************************************
 * Name: axp2101_voltage_info
 *
 * Description:
 *   Return the measured VBUS input voltage in mV.
 *
 ****************************************************************************/

static int axp2101_voltage_info(FAR struct battery_charger_dev_s *dev,
                                FAR int *value)
{
  FAR struct axp2101_dev_s *priv = (FAR struct axp2101_dev_s *)dev;

  return axp2101_getadc(&priv->bus, AXP2101_VBUS_H, value);
}

#ifdef CONFIG_AXP2101_GAUGE

static int axp2101_bat_present(FAR struct axp2101_bus_s *bus,
                               FAR bool *present)
{
  uint8_t regval;
  int ret;

  ret = axp2101_getreg(bus, AXP2101_STATUS1, &regval);
  if (ret < 0)
    {
      return ret;
    }

  *present = (regval & STATUS1_BAT_PRESENT) != 0;
  return OK;
}

/****************************************************************************
 * Name: axp2101_gauge_state
 ****************************************************************************/

static int axp2101_gauge_state(FAR struct battery_gauge_dev_s *dev,
                               FAR int *status)
{
  FAR struct axp2101_gauge_s *priv = (FAR struct axp2101_gauge_s *)dev;

  return axp2101_bat_state(&priv->bus, status);
}

/****************************************************************************
 * Name: axp2101_gauge_online
 *
 * Description:
 *   Return true if a battery is present.
 *
 ****************************************************************************/

static int axp2101_gauge_online(FAR struct battery_gauge_dev_s *dev,
                                FAR bool *status)
{
  FAR struct axp2101_gauge_s *priv = (FAR struct axp2101_gauge_s *)dev;

  return axp2101_bat_present(&priv->bus, status);
}

/****************************************************************************
 * Name: axp2101_gauge_voltage
 *
 * Description:
 *   Return the battery voltage in mV, 0 without battery.
 *
 ****************************************************************************/

static int axp2101_gauge_voltage(FAR struct battery_gauge_dev_s *dev,
                                 FAR int *value)
{
  FAR struct axp2101_gauge_s *priv = (FAR struct axp2101_gauge_s *)dev;
  bool present;
  int ret;

  ret = axp2101_bat_present(&priv->bus, &present);
  if (ret < 0)
    {
      return ret;
    }

  if (!present)
    {
      *value = 0;
      return OK;
    }

  return axp2101_getadc(&priv->bus, AXP2101_VBAT_H, value);
}

/****************************************************************************
 * Name: axp2101_gauge_capacity
 *
 * Description:
 *   Return the state of charge in percent, 0 without battery.
 *
 ****************************************************************************/

static int axp2101_gauge_capacity(FAR struct battery_gauge_dev_s *dev,
                                  FAR int *value)
{
  FAR struct axp2101_gauge_s *priv = (FAR struct axp2101_gauge_s *)dev;
  uint8_t regval;
  bool present;
  int ret;

  ret = axp2101_bat_present(&priv->bus, &present);
  if (ret < 0)
    {
      return ret;
    }

  if (!present)
    {
      *value = 0;
      return OK;
    }

  ret = axp2101_getreg(&priv->bus, AXP2101_SOC, &regval);
  if (ret < 0)
    {
      return ret;
    }

  *value = regval > 100 ? 100 : regval;
  return OK;
}

/****************************************************************************
 * Name: axp2101_gauge_temp
 *
 * Description:
 *   Return the die temperature of the PMIC in 0.1 degrees Celsius.  The
 *   chip has no battery temperature sensor of its own.
 *
 ****************************************************************************/

static int axp2101_gauge_temp(FAR struct battery_gauge_dev_s *dev,
                              FAR int *value)
{
  FAR struct axp2101_gauge_s *priv = (FAR struct axp2101_gauge_s *)dev;
  int raw;
  int ret;

  ret = axp2101_getadc(&priv->bus, AXP2101_TDIE_H, &raw);
  if (ret < 0)
    {
      return ret;
    }

  /* T = 22 + (7274 - raw) / 20 degrees Celsius */

  *value = 220 + (7274 - raw) / 2;
  return OK;
}

/****************************************************************************
 * Name: axp2101_gauge_chipid
 ****************************************************************************/

static int axp2101_gauge_chipid(FAR struct battery_gauge_dev_s *dev,
                                FAR unsigned int *value)
{
  FAR struct axp2101_gauge_s *priv = (FAR struct axp2101_gauge_s *)dev;
  uint8_t regval;
  int ret;

  ret = axp2101_getreg(&priv->bus, AXP2101_CHIPID, &regval);
  if (ret < 0)
    {
      return ret;
    }

  *value = regval;
  return OK;
}

#endif /* CONFIG_AXP2101_GAUGE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: axp2101_initialize
 *
 * Description:
 *   Initialize the AXP2101 battery charger lower half.
 *
 ****************************************************************************/

FAR struct battery_charger_dev_s *
axp2101_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                   uint32_t frequency)
{
  FAR struct axp2101_dev_s *priv;

  DEBUGASSERT(i2c != NULL);

  priv = kmm_zalloc(sizeof(struct axp2101_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->dev.ops       = &g_axp2101ops;
  priv->bus.i2c       = i2c;
  priv->bus.addr      = addr;
  priv->bus.frequency = frequency;

  if (axp2101_probe(&priv->bus) < 0)
    {
      kmm_free(priv);
      return NULL;
    }

  return &priv->dev;
}

/****************************************************************************
 * Name: axp2101_poweroff
 *
 * Description:
 *   Turn off all the outputs of the AXP2101.
 *
 ****************************************************************************/

int axp2101_poweroff(FAR struct battery_charger_dev_s *dev)
{
  FAR struct axp2101_dev_s *priv = (FAR struct axp2101_dev_s *)dev;

  DEBUGASSERT(priv != NULL);

  return axp2101_modifyreg(&priv->bus, AXP2101_COMMON_CFG, 0,
                           COMMON_CFG_PWROFF);
}

#ifdef CONFIG_AXP2101_GAUGE

/****************************************************************************
 * Name: axp2101_gauge_initialize
 *
 * Description:
 *   Initialize the AXP2101 fuel gauge lower half.
 *
 ****************************************************************************/

FAR struct battery_gauge_dev_s *
axp2101_gauge_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                         uint32_t frequency)
{
  FAR struct axp2101_gauge_s *priv;

  DEBUGASSERT(i2c != NULL);

  priv = kmm_zalloc(sizeof(struct axp2101_gauge_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->dev.ops       = &g_axp2101_gaugeops;
  priv->bus.i2c       = i2c;
  priv->bus.addr      = addr;
  priv->bus.frequency = frequency;

  /* The gauge is enabled by default, make sure that it still is */

  if (axp2101_probe(&priv->bus) < 0 ||
      axp2101_modifyreg(&priv->bus, AXP2101_MODULE_EN, 0,
                        MODULE_EN_GAUGE) < 0)
    {
      kmm_free(priv);
      return NULL;
    }

  return &priv->dev;
}

#endif /* CONFIG_AXP2101_GAUGE */

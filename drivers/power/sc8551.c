/****************************************************************************
 * drivers/power/sc8551.c
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
 * The SC8551 are Li-Ion Battery management
 *
 * This driver requires:
 *
 * CONFIG_BATTERY_CHARGER - Upper half battery driver support
 * CONFIG_I2C - I2C support
 * CONFIG_I2C_SC8551 - And the driver must be explicitly selected.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>

#include "sc8551_reg.h"
#include "sc8551.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NOT_PMIC
#ifdef CONFIG_DEBUG_STWLC38
#  define baterr  _err
#  define batdbg  _err
#  define batinfo _err
#else
#  define baterr  _none
#  define batdbg  _none
#  define batinfo _none
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sc8551_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_charger_dev_s dev; /* Battery charger device */

  /* Data fields specific to the lower half SC8551 driver follow */

  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  uint32_t frequency;           /* I2C frequency */

  /* status fields specific to the current status */

  struct sc8551_stat_s stat;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C support */

static int sc8551_getreg8(FAR struct sc8551_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *val, int num);
static int sc8551_putreg8(FAR struct sc8551_dev_s *priv, uint8_t regaddr,
                          uint8_t val);

static int sc8551_reset(FAR struct sc8551_dev_s *priv);
static int sc8551_watchdog(FAR struct sc8551_dev_s *priv, bool enable);
static int sc8551_sysoff(FAR struct sc8551_dev_s *priv);
static int sc8551_en_term(FAR struct sc8551_dev_s *priv, bool state);
static int sc8551_en_hiz(FAR struct sc8551_dev_s *priv, bool state);
static int sc8551_en_stat(FAR struct sc8551_dev_s *priv, bool state);
static int sc8551_setboost_otg_config(FAR struct sc8551_dev_s *priv,
                                      bool state);
static int sc8551_powersupply(FAR struct sc8551_dev_s *priv, int current);
static int sc8551_setvolt(FAR struct sc8551_dev_s *priv, int volts);
static inline int sc8551_setcurr(FAR struct sc8551_dev_s *priv,
                                 int req_current);

/* Battery driver lower half methods */

static int sc8551_state(FAR struct battery_charger_dev_s *dev,
                        FAR int *status);
static int sc8551_health(FAR struct battery_charger_dev_s *dev,
                         FAR int *health);
static int sc8551_online(FAR struct battery_charger_dev_s *dev,
                         FAR bool *status);
static int sc8551_voltage(FAR struct battery_charger_dev_s *dev,
                          int value);
static int sc8551_current(FAR struct battery_charger_dev_s *dev,
                          int value);
static int sc8551_input_current(FAR struct battery_charger_dev_s *dev,
                                int value);
static int sc8551_operate(FAR struct battery_charger_dev_s *dev,
                          uintptr_t param);

#ifdef CONFIG_DEBUG_SC8551
static int sc8551_dump_regs(FAR struct sc8551_dev_s *priv);
#  define sc8551_dump_regs(priv) sc8551_dump_regs(priv)
#else
#  define sc8551_dump_regs(priv)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Configuration fields specific to the default parameter
 ****************************************************************************/

const struct sc8551_cfg_s g_sc8551_cfg =
{
  .bat_ovp_disable = false,
  .bat_ocp_disable = false,
  .bat_ovp_alm_disable = true,
  .bat_ocp_alm_disable = true,
  .bat_ovp_th = 4500,
  .bat_ovp_alm_th = 4350,
  .bat_ocp_th = 8500,
  .bat_ocp_alm_th = 8000,
  .bus_ovp_alm_disable = true,
  .bus_ocp_disable = false,
  .bus_ocp_alm_disable = true,
  .bus_ovp_th = 11000,
  .bus_ovp_alm_th = 10500,
  .bus_ocp_th = 4000,
  .bus_ocp_alm_th = 3800,
  .bat_ucp_alm_disable = false,
  .bat_ucp_alm_th = 2600,
  .ac_ovp_th = 300,
  .bat_therm_disable = true,
  .bus_therm_disable = true,
  .die_therm_disable = false,
  .bat_therm_th = 21,    /* in % */
  .bus_therm_th = 21,    /* in % */
  .die_therm_th = 125,   /* in degC */
  .sense_r_mohm = 2,
};

static const struct battery_charger_operations_s g_sc8551ops =
{
  sc8551_state,
  sc8551_health,
  sc8551_online,
  sc8551_voltage,
  sc8551_current,
  sc8551_input_current,
  sc8551_operate,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sc8551_getreg8
 *
 * Description:
 *   Read a 8-bit value from a SC8551 register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK
 *   REPEATED-START <I2C read address> ACK Data0 ACK Data1 NO-ACK STOP
 *
 ****************************************************************************/

static int sc8551_getreg8(FAR struct sc8551_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *val, int num_char)
{
  struct i2c_config_s config;
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

  /* Restart and read 8-bit values from the register */

  ret = i2c_read(priv->i2c, &config, val, num_char);
  if (ret < 0)
    {
      baterr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: sc8551_putreg8
 *
 * Description:
 *   Write a 8-bit value to a SC8551 register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK Data0 ACK Data1 ACK STOP
 *
 ****************************************************************************/

static int sc8551_putreg8(FAR struct sc8551_dev_s *priv, uint8_t regaddr,
                          uint8_t val)
{
  struct i2c_config_s config;
  uint8_t buffer[2];

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  batwarn("addr: %02x val: %08x\n", regaddr, val);

  /* Set up a message to send */

  buffer[0] = regaddr;
  buffer[1] = val;

  /* Write the register address followed by the data (no RESTART) */

  return i2c_write(priv->i2c, &config, buffer, 2);
}

#ifdef CONFIG_DEBUG_SC8551
static void sc8551_show_registers(FAR struct sc8551_dev_s * priv)
{
  int i;
  uint8_t value;
  for (i = 0; i < SC8551_REG_MAX; i++)
    {
      sc8551_getreg8(priv, i, &value, 1);
      batinfo("[ SC8551] Reg[%02x] = 0x%02x\r\n", i, value);
    }
}

static int sc8551_dump_regs(FAR struct sc8551_dev_s * priv)
{
  int ret;
  uint8_t value = 0;

  ret  = sc8551_getreg8(priv, SC8551_REG_00, &value, 1);
  batinfo("REG#00: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_01, &value, 1);
  batinfo("REG#01: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_02, &value, 1);
  batinfo("REG#02: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_03, &value, 1);
  batinfo("REG#03: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_04, &value, 1);
  batinfo("REG#04: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_05, &value, 1);
  batinfo("REG#05: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_06, &value, 1);
  batinfo("REG#06: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_07, &value, 1);
  batinfo("REG#07: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_08, &value, 1);
  batinfo("REG#08: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_09, &value, 1);
  batinfo("REG#09: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_0A, &value, 1);
  batinfo("REG#0A: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_0B, &value, 1);
  batinfo("REG#0B: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_0C, &value, 1);
  batinfo("REG#0C: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_0D, &value, 1);
  batinfo("REG#0D: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_0E, &value, 1);
  batinfo("REG#0E: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_0F, &value, 1);
  batinfo("REG#0F: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_10, &value, 1);
  batinfo("REG#10: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_11, &value, 1);
  batinfo("REG#11: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_12, &value, 1);
  batinfo("REG#12: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_13, &value, 1);
  batinfo("REG#13: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_14, &value, 1);
  batinfo("REG#14: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_15, &value, 1);
  batinfo("REG#15: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_16, &value, 1);
  batinfo("REG#16: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_17, &value, 1);
  batinfo("REG#17: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_18, &value, 1);
  batinfo("REG#18: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_19, &value, 1);
  batinfo("REG#19: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_1A, &value, 1);
  batinfo("REG#1A: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_1B, &value, 1);
  batinfo("REG#1B: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_1C, &value, 1);
  batinfo("REG#1C: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_1D, &value, 1);
  batinfo("REG#1D: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_1E, &value, 1);
  batinfo("REG#1E: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_1F, &value, 1);
  batinfo("REG#1F: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_20, &value, 1);
  batinfo("REG#20: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_21, &value, 1);
  batinfo("REG#21: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_22, &value, 1);
  batinfo("REG#22: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_23, &value, 1);
  batinfo("REG#23: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_24, &value, 1);
  batinfo("REG#24: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_25, &value, 1);
  batinfo("REG#25: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_26, &value, 1);
  batinfo("REG#26: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_27, &value, 1);
  batinfo("REG#27: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_28, &value, 1);
  batinfo("REG#28: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_29, &value, 1);
  batinfo("REG#29: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_2A, &value, 1);
  batinfo("REG#2A: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_2B, &value, 1);
  batinfo("REG#2B: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_2C, &value, 1);
  batinfo("REG#2C: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_2D, &value, 1);
  batinfo("REG#2D: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_2E, &value, 1);
  batinfo("REG#2E: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_2F, &value, 1);
  batinfo("REG#2F: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_30, &value, 1);
  batinfo("REG#30: 0x%08X\n", value);
  ret  = sc8551_getreg8(priv, SC8551_REG_31, &value, 1);
  batinfo("REG#31: 0x%08X\n", value);

  /* Not reading fault register. */

  ret  = sc8551_getreg8(priv, SC8551_REG_34, &value, 1);
  batinfo("REG#34: 0x%08X\n", value);

  return ret;
}
#endif

/****************************************************************************
 * Name: sc8551_detect_device
 *
 * Description:
 *   detect the SC8551
 *
 ****************************************************************************/

static int sc8551_detect_device(FAR struct sc8551_dev_s *priv)
{
  uint8_t val;
  int ret;

  ret = sc8551_getreg8(priv, SC8551_REG_13, &val, 1);
  if (ret < 0)
    {
      return 1;
    }

  priv->stat.part_no = (val & SC8551_DEV_ID_MASK);
  priv->stat.part_no >>= SC8551_DEV_ID_SHIFT;

  return 0;
}

static int sc8551_set_ss_timeout(FAR struct sc8551_dev_s *priv, int timeout)
{
  int ret;
  uint8_t val;
  uint8_t val_r;

  switch (timeout)
    {
      case 0:
        val = SC8551_SS_TIMEOUT_DISABLE;
      break;
      case 12:
        val = SC8551_SS_TIMEOUT_12P5MS;
      break;
      case 25:
        val = SC8551_SS_TIMEOUT_25MS;
      break;
      case 50:
        val = SC8551_SS_TIMEOUT_50MS;
      break;
      case 100:
        val = SC8551_SS_TIMEOUT_100MS;
      break;
      case 400:
        val = SC8551_SS_TIMEOUT_400MS;
      break;
      case 1500:
        val = SC8551_SS_TIMEOUT_1500MS;
      break;
      case 100000:
        val = SC8551_SS_TIMEOUT_100000MS;
      break;
      default:
        val = SC8551_SS_TIMEOUT_DISABLE;
      break;
    }

  val <<= SC8551_SS_TIMEOUT_SET_SHIFT;
  val &= SC8551_SS_TIMEOUT_SET_MASK;

  ret = sc8551_getreg8(priv, SC8551_REG_2B, &val_r, 1);
  val |= val_r;

  ret = sc8551_putreg8(priv, SC8551_REG_2B, val);

  return ret;
}

/**/

static int sc8551_set_sense_resistor(FAR struct sc8551_dev_s *priv,
                                         int r_mohm)
{
  int ret;
  uint8_t val;
  uint8_t val_r;

  if (r_mohm == 2)
    val = SC8551_SET_IBAT_SNS_RES_2MHM;
  else if (r_mohm == 5)
    val = SC8551_SET_IBAT_SNS_RES_5MHM;
  else
    {
      return -1;
    }

  val <<= SC8551_SET_IBAT_SNS_RES_SHIFT;
  val &= SC8551_SET_IBAT_SNS_RES_MASK;

  ret = sc8551_getreg8(priv, SC8551_REG_2B, &val_r, 1);
  val |= val_r;

  ret = sc8551_putreg8(priv, SC8551_REG_2B, val);

  return ret;
}

static int sc8551_update_bits(FAR struct sc8551_dev_s *priv,
                                  uint8_t reg,
                                  uint8_t mask,
                                  uint8_t val)
{
  int ret;
  uint8_t val_r;

  ret = sc8551_getreg8(priv, reg, &val_r, 1);
  if (ret < 0)
    {
      baterr("[ SC8551] Failed: reg=%02X\r\n", reg);
      return -1;
    }

  val &= mask;
  val |= val_r;

  ret = sc8551_putreg8(priv, reg, val);
  if (ret < 0)
    {
      baterr("[ SC8551] Failed: reg=%02X\r\n", reg);
      return -1;
    }

  return 0;
}

static int sc8551_enable_batovp(FAR struct sc8551_dev_s *priv, bool enable)
{
  int ret;
  uint8_t val;

  if (enable)
    val = SC8551_BAT_OVP_ENABLE;
  else
    val = SC8551_BAT_OVP_DISABLE;

  val <<= SC8551_BAT_OVP_DIS_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_00,
                           SC8551_BAT_OVP_DIS_MASK, val);
  return ret;
}

static int sc8551_set_batovp_th(FAR struct sc8551_dev_s *priv, int threshold)
{
  int ret;
  uint8_t val;

  if (threshold < SC8551_BAT_OVP_BASE)
    threshold = SC8551_BAT_OVP_BASE;

  val = (threshold - SC8551_BAT_OVP_BASE) / SC8551_BAT_OVP_LSB;

  val <<= SC8551_BAT_OVP_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_00,
                    SC8551_BAT_OVP_MASK, val);
  return ret;
}

static int sc8551_enable_batovp_alarm(FAR struct sc8551_dev_s *priv,
                                                         bool enable)
{
  int ret;
  uint8_t val;

  if (enable)
    val = SC8551_BAT_OVP_ALM_ENABLE;
  else
    val = SC8551_BAT_OVP_ALM_DISABLE;

  val <<= SC8551_BAT_OVP_ALM_DIS_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_01,
                  SC8551_BAT_OVP_ALM_DIS_MASK, val);
  return ret;
}

static int sc8551_set_batovp_alarm_th(FAR struct sc8551_dev_s *priv,
                                                       int threshold)
{
  int ret;
  uint8_t val;

  if (threshold < SC8551_BAT_OVP_ALM_BASE)
    threshold = SC8551_BAT_OVP_ALM_BASE;

  val = (threshold - SC8551_BAT_OVP_ALM_BASE) / SC8551_BAT_OVP_ALM_LSB;

  val <<= SC8551_BAT_OVP_ALM_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_01,
                           SC8551_BAT_OVP_ALM_MASK, val);
  return ret;
}

static int sc8551_enable_batocp(FAR struct sc8551_dev_s *priv, bool enable)
{
  int ret;
  uint8_t val;

  if (enable)
    val = SC8551_BAT_OCP_ENABLE;
  else
    val = SC8551_BAT_OCP_DISABLE;

  val <<= SC8551_BAT_OCP_DIS_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_02,
                    SC8551_BAT_OCP_DIS_MASK, val);
  return ret;
}

static int sc8551_set_batocp_th(FAR struct sc8551_dev_s *priv, int threshold)
{
  int ret;
  uint8_t val;

  if (threshold < SC8551_BAT_OCP_BASE)
    threshold = SC8551_BAT_OCP_BASE;

  val = (threshold - SC8551_BAT_OCP_BASE) / SC8551_BAT_OCP_LSB;

  val <<= SC8551_BAT_OCP_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_02,
                  SC8551_BAT_OCP_MASK, val);
  return ret;
}

static int sc8551_enable_batocp_alarm(FAR struct sc8551_dev_s *priv,
                                                         bool enable)
{
  int ret;
  uint8_t val;

  if (enable)
    val = SC8551_BAT_OCP_ALM_ENABLE;
  else
    val = SC8551_BAT_OCP_ALM_DISABLE;

  val <<= SC8551_BAT_OCP_ALM_DIS_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_03,
                    SC8551_BAT_OCP_ALM_DIS_MASK, val);
  return ret;
}

static int sc8551_set_batocp_alarm_th(FAR struct sc8551_dev_s *priv,
                                                       int threshold)
{
  int ret;
  uint8_t val;

  if (threshold < SC8551_BAT_OCP_ALM_BASE)
      threshold = SC8551_BAT_OCP_ALM_BASE;

  val = (threshold - SC8551_BAT_OCP_ALM_BASE) / SC8551_BAT_OCP_ALM_LSB;

  val <<= SC8551_BAT_OCP_ALM_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_03,
                  SC8551_BAT_OCP_ALM_MASK, val);
  return ret;
}

static int sc8551_set_busovp_th(FAR struct sc8551_dev_s *priv, int threshold)
{
  int ret;
  uint8_t val;

  if (threshold < SC8551_BUS_OVP_BASE)
    threshold = SC8551_BUS_OVP_BASE;

  val = (threshold - SC8551_BUS_OVP_BASE) / SC8551_BUS_OVP_LSB;

  val <<= SC8551_BUS_OVP_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_06,
                SC8551_BUS_OVP_MASK, val);
  return ret;
}

static int sc8551_enable_busovp_alarm(FAR struct sc8551_dev_s *priv,
                                                         bool enable)
{
  int ret;
  uint8_t val;

  if (enable)
    val = SC8551_BUS_OVP_ALM_ENABLE;
  else
    val = SC8551_BUS_OVP_ALM_DISABLE;

  val <<= SC8551_BUS_OVP_ALM_DIS_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_07,
              SC8551_BUS_OVP_ALM_DIS_MASK, val);
  return ret;
}

static int sc8551_set_busovp_alarm_th(FAR struct sc8551_dev_s *priv,
                                                       int threshold)
{
  int ret;
  uint8_t val;

  if (threshold < SC8551_BUS_OVP_ALM_BASE)
    threshold = SC8551_BUS_OVP_ALM_BASE;

  val = (threshold - SC8551_BUS_OVP_ALM_BASE)
                     / SC8551_BUS_OVP_ALM_LSB;

  val <<= SC8551_BUS_OVP_ALM_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_07,
                SC8551_BUS_OVP_ALM_MASK, val);
  return ret;
}

static int sc8551_enable_busocp(FAR struct sc8551_dev_s *priv,
                                                   bool enable)
{
  int ret;
  uint8_t val;

  if (enable)
    val = SC8551_BUS_OCP_ENABLE;
  else
    val = SC8551_BUS_OCP_DISABLE;

  val <<= SC8551_BUS_OCP_DIS_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_08,
                   SC8551_BUS_OCP_DIS_MASK, val);
  return ret;
}

static int sc8551_set_busocp_th(FAR struct sc8551_dev_s *priv, int threshold)
{
  int ret;
  uint8_t val;

  if (threshold < SC8551_BUS_OCP_BASE)
    threshold = SC8551_BUS_OCP_BASE;

  val = (threshold - SC8551_BUS_OCP_BASE) / SC8551_BUS_OCP_LSB;

  val <<= SC8551_BUS_OCP_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_08,
                    SC8551_BUS_OCP_MASK, val);
  return ret;
}

static int sc8551_enable_busocp_alarm(FAR struct sc8551_dev_s *priv,
                                                         bool enable)
{
  int ret;
  uint8_t val;

  if (enable)
    val = SC8551_BUS_OCP_ALM_ENABLE;
  else
    val = SC8551_BUS_OCP_ALM_DISABLE;

  val <<= SC8551_BUS_OCP_ALM_DIS_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_09,
               SC8551_BUS_OCP_ALM_DIS_MASK, val);
  return ret;
}

static int sc8551_set_busocp_alarm_th(FAR struct sc8551_dev_s *priv,
                                                       int threshold)
{
  int ret;
  uint8_t val;

  if (threshold < SC8551_BUS_OCP_ALM_BASE)
    threshold = SC8551_BUS_OCP_ALM_BASE;

  val = (threshold - SC8551_BUS_OCP_ALM_BASE) / SC8551_BUS_OCP_ALM_LSB;

  val <<= SC8551_BUS_OCP_ALM_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_09,
                  SC8551_BUS_OCP_ALM_MASK, val);
  return ret;
}

static int sc8551_enable_batucp_alarm(FAR struct sc8551_dev_s *priv,
                                                         bool enable)
{
  int ret;
  uint8_t val;

  if (enable)
    val = SC8551_BAT_UCP_ALM_ENABLE;
  else
    val = SC8551_BAT_UCP_ALM_DISABLE;

  val <<= SC8551_BAT_UCP_ALM_DIS_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_04,
               SC8551_BAT_UCP_ALM_DIS_MASK, val);
  return ret;
}

static int sc8551_set_batucp_alarm_th(FAR struct sc8551_dev_s *priv,
                                                       int threshold)
{
  int ret;
  uint8_t val;

  if (threshold < SC8551_BAT_UCP_ALM_BASE)
    threshold = SC8551_BAT_UCP_ALM_BASE;

  val = (threshold - SC8551_BAT_UCP_ALM_BASE) / SC8551_BAT_UCP_ALM_LSB;

  val <<= SC8551_BAT_UCP_ALM_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_04,
                  SC8551_BAT_UCP_ALM_MASK, val);
  return ret;
}

static int sc8551_set_acovp_th(FAR struct sc8551_dev_s *priv,
                                                int threshold)
{
  int ret;
  uint8_t val;

  if (threshold < SC8551_AC_OVP_BASE)
    threshold = SC8551_AC_OVP_BASE;

  if (threshold == SC8551_AC_OVP_6P5V)
    val = 0x07;
  else
    val = (threshold - SC8551_AC_OVP_BASE) /  SC8551_AC_OVP_LSB;

  val <<= SC8551_AC_OVP_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_05,
                        SC8551_AC_OVP_MASK, val);

  return ret;
}

static int sc8551_set_vdrop_th(FAR struct sc8551_dev_s *priv, int threshold)
{
  int ret;
  uint8_t val;

  if (threshold == 300)
    val = SC8551_VDROP_THRESHOLD_300MV;
  else
    val = SC8551_VDROP_THRESHOLD_400MV;

  val <<= SC8551_VDROP_THRESHOLD_SET_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_05,
           SC8551_VDROP_THRESHOLD_SET_MASK, val);

  return ret;
}

static int sc8551_set_vdrop_deglitch(FAR struct sc8551_dev_s *priv, int us)
{
  int ret;
  uint8_t val;

  if (us == 8)
    val = SC8551_VDROP_DEGLITCH_8US;
  else
    val = SC8551_VDROP_DEGLITCH_5MS;

  val <<= SC8551_VDROP_DEGLITCH_SET_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_05,
            SC8551_VDROP_DEGLITCH_SET_MASK, val);
  return ret;
}

static int sc8551_enable_bat_therm(FAR struct sc8551_dev_s *priv,
                                   bool enable)
{
  int ret;
  uint8_t val;

  if (enable)
    val = SC8551_TSBAT_ENABLE;
  else
    val = SC8551_TSBAT_DISABLE;

  val <<= SC8551_TSBAT_DIS_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_0C,
                     SC8551_TSBAT_DIS_MASK, val);
  return ret;
}

static int sc8551_init_protection(FAR struct sc8551_dev_s *priv)
{
  int ret;

  ret = sc8551_enable_batovp(priv, (bool)!g_sc8551_cfg.bat_ovp_disable);
  batinfo("[ SC8551] %s bat ovp %s\r\n",
          g_sc8551_cfg.bat_ovp_disable ? "disable" : "enable",
          !ret ? "successfullly" : "failed");

  ret = sc8551_enable_batocp(priv, (bool)!g_sc8551_cfg.bat_ocp_disable);
  batinfo("[ SC8551] %s bat ocp %s\r\n",
          g_sc8551_cfg.bat_ocp_disable ? "disable" : "enable",
          !ret ? "successfullly" : "failed");

  ret = sc8551_enable_batovp_alarm(priv,
                                   (bool)!g_sc8551_cfg.bat_ovp_alm_disable);
  batinfo("[ SC8551] %s bat ovp alarm %s\r\n",
          g_sc8551_cfg.bat_ovp_alm_disable ? "disable" : "enable",
          !ret ? "successfullly" : "failed");

  ret = sc8551_enable_batocp_alarm(priv,
                                   (bool)!g_sc8551_cfg.bat_ocp_alm_disable);
  batinfo("[ SC8551] %s bat ocp alarm %s\r\n",
          g_sc8551_cfg.bat_ocp_alm_disable ? "disable" : "enable",
          !ret ? "successfullly" : "failed");

  ret = sc8551_enable_batucp_alarm(priv,
                                   (bool)!g_sc8551_cfg.bat_ucp_alm_disable);
  batinfo("[ SC8551] %s bat ocp alarm %s\r\n",
          g_sc8551_cfg.bat_ucp_alm_disable ? "disable" : "enable",
          !ret ? "successfullly" : "failed");

  ret = sc8551_enable_busovp_alarm(priv,
                                   (bool)!g_sc8551_cfg.bus_ovp_alm_disable);
  batinfo("[ SC8551] %s bus ovp alarm %s\r\n",
          g_sc8551_cfg.bus_ovp_alm_disable ? "disable" : "enable",
          !ret ? "successfullly" : "failed");

  ret = sc8551_enable_busocp(priv, (bool)!g_sc8551_cfg.bus_ocp_disable);
  batinfo("[ SC8551] %s bus ocp %s\r\n",
          g_sc8551_cfg.bus_ocp_disable ? "disable" : "enable",
          !ret ? "successfullly" : "failed");

  ret = sc8551_enable_busocp_alarm(priv,
                                   (bool)!g_sc8551_cfg.bus_ocp_alm_disable);
  batinfo("[ SC8551] %s bus ocp alarm %s\r\n",
          g_sc8551_cfg.bus_ocp_alm_disable ? "disable" : "enable",
          !ret ? "successfullly" : "failed");

  ret = sc8551_enable_bat_therm(priv, (bool)!g_sc8551_cfg.bat_therm_disable);
  batinfo("[ SC8551] %s bat therm %s\r\n",
          g_sc8551_cfg.bat_therm_disable ? "disable" : "enable",
          !ret ? "successfullly" : "failed");

  ret = sc8551_enable_bus_therm(priv, (bool)!g_sc8551_cfg.bus_therm_disable);
  batinfo("[ SC8551] %s bus therm %s\r\n",
          g_sc8551_cfg.bus_therm_disable ? "disable" : "enable",
          !ret ? "successfullly" : "failed");

  ret = sc8551_set_batovp_th(priv, g_sc8551_cfg.bat_ovp_th);
  batinfo("[ SC8551] set bat ovp th %d %s\r\n",
          g_sc8551_cfg.bat_ovp_th,
          !ret ? "successfully" : "failed");

  ret = sc8551_set_batovp_alarm_th(priv, g_sc8551_cfg.bat_ovp_alm_th);
  batinfo("[ SC8551] set bat ovp alarm threshold %d %s\r\n",
          g_sc8551_cfg.bat_ovp_alm_th,
          !ret ? "successfully" : "failed");

  ret = sc8551_set_batocp_th(priv, g_sc8551_cfg.bat_ocp_th);
  batinfo("[ SC8551] set bat ocp threshold %d %s\r\n",
          g_sc8551_cfg.bat_ocp_th,
          !ret ? "successfully" : "failed");

  ret = sc8551_set_batocp_alarm_th(priv, g_sc8551_cfg.bat_ocp_alm_th);
  batinfo("[ SC8551] set bat ocp alarm threshold %d %s\r\n",
          g_sc8551_cfg.bat_ocp_alm_th,
          !ret ? "successfully" : "failed");

  ret = sc8551_set_busovp_th(priv, g_sc8551_cfg.bus_ovp_th);
  batinfo("[ SC8551] set bus ovp threshold %d %s\r\n",
          g_sc8551_cfg.bus_ovp_th,
          !ret ? "successfully" : "failed");

  ret = sc8551_set_busovp_alarm_th(priv, g_sc8551_cfg.bus_ovp_alm_th);
  batinfo("[ SC8551] set bus ovp alarm threshold %d %s\r\n",
          g_sc8551_cfg.bus_ovp_alm_th,
          !ret ? "successfully" : "failed");

  ret = sc8551_set_busocp_th(priv, g_sc8551_cfg.bus_ocp_th);
  batinfo("[ SC8551] set bus ocp threshold %d %s\r\n",
          g_sc8551_cfg.bus_ocp_th,
          !ret ? "successfully" : "failed");

  ret = sc8551_set_busocp_alarm_th(priv, g_sc8551_cfg.bus_ocp_alm_th);
  batinfo("[ SC8551] set bus ocp alarm th %d %s\r\n",
          g_sc8551_cfg.bus_ocp_alm_th,
          !ret ? "successfully" : "failed");

  ret = sc8551_set_batucp_alarm_th(priv, g_sc8551_cfg.bat_ucp_alm_th);
  batinfo("[ SC8551] set bat ucp threshold %d %s\r\n",
          g_sc8551_cfg.bat_ucp_alm_th,
          !ret ? "successfully" : "failed");

  ret = sc8551_set_bat_therm_th(priv, g_sc8551_cfg.bat_therm_th);
  batinfo("[ SC8551] set die therm threshold %d %s\r\n",
          g_sc8551_cfg.bat_therm_th,
          !ret ? "successfully" : "failed");

  ret = sc8551_set_bus_therm_th(priv, g_sc8551_cfg.bus_therm_th);
  batinfo("[ SC8551] set bus therm threshold %d %s\r\n",
          g_sc8551_cfg.bus_therm_th,
          !ret ? "successfully" : "failed");

  ret = sc8551_set_die_therm_th(priv, g_sc8551_cfg.die_therm_th);
  batinfo("[ SC8551] set die therm threshold %d %s\r\n",
          g_sc8551_cfg.die_therm_th,
          !ret ? "successfully" : "failed");

  ret = sc8551_set_acovp_th(priv, g_sc8551_cfg.ac_ovp_th);
  batinfo("[ SC8551] set ac ovp threshold %d %s\r\n",
          g_sc8551_cfg.ac_ovp_th,
          !ret ? "successfully" : "failed");

  return 0;
}

static int sc8551_set_adc_scanrate(FAR struct sc8551_dev_s *priv,
                                   bool oneshot)
{
  int ret;
  uint8_t val;

  if (oneshot)
    val = SC8551_ADC_RATE_ONESHOT;
  else
    val = SC8551_ADC_RATE_CONTINOUS;

  val <<= SC8551_ADC_RATE_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_14,
                           SC8551_ADC_EN_MASK, val);
  return ret;
}

static int sc8551_set_adc_scan(FAR struct sc8551_dev_s *priv,
                               int channel, bool enable)
{
  int ret;
  uint8_t reg;
  uint8_t mask;
  uint8_t shift;
  uint8_t val;

  if (channel > ADC_MAX_NUM)
    {
      return -1;
    }

  if (channel == ADC_IBUS)
    {
      reg = SC8551_REG_14;
      shift = SC8551_IBUS_ADC_DIS_SHIFT;
      mask = SC8551_IBUS_ADC_DIS_MASK;
    }
  else
    {
    reg = SC8551_REG_15;
    shift = 8 - channel;
    mask = 1 << shift;
    }

  if (enable)
    val = 0 << shift;
  else
    val = 1 << shift;

  ret = sc8551_update_bits(priv, reg, mask, val);

  return ret;
}

static int sc8551_init_adc(FAR struct sc8551_dev_s *priv)
{
  sc8551_set_adc_scanrate(priv, false);
  sc8551_set_adc_scan(priv, ADC_IBUS, true);
  sc8551_set_adc_scan(priv, ADC_VBUS, true);
  sc8551_set_adc_scan(priv, ADC_VOUT, true);
  sc8551_set_adc_scan(priv, ADC_VBAT, true);
  sc8551_set_adc_scan(priv, ADC_IBAT, true);
  sc8551_set_adc_scan(priv, ADC_TBUS, true);
  sc8551_set_adc_scan(priv, ADC_TBAT, true);
  sc8551_set_adc_scan(priv, ADC_TDIE, true);
  sc8551_set_adc_scan(priv, ADC_VAC, true);

  sc8551_enable_adc(true);

  return 0;
}

static int  sc8551_set_alarm_int_mask(FAR struct sc8551_dev_s *priv,
                                      uint8_t val)
{
  return sc8551_update_bits(priv, SC8551_REG_0F, val, val);
}

/****************************************************************************
 * TODO:be careful ts bus and ts bat alarm bit mask is in
 * fault mask register, so you need call
 * sc8551_set_fault_int_mask for tsbus and tsbat alarm
 ****************************************************************************/

static int sc8551_init_int_src(FAR struct sc8551_dev_s *priv)
{
  int ret;

  ret = sc8551_set_alarm_int_mask(priv, ADC_DONE | BAT_OVP_ALARM);   /* | BAT_UCP_ALARM */
  if (ret)
    {
      baterr("[ SC8551] failed to set alarm mask:%d\r\n", ret);
      return ret;
    }

  return ret;
}

static int sc8551_set_ibat_reg_th(FAR struct sc8551_dev_s *priv, int th_ma)
{
  int ret;
  uint8_t val;

  if (th_ma == 200)
      val = SC8551_IBAT_REG_200MA;
  else if (th_ma == 300)
      val = SC8551_IBAT_REG_300MA;
  else if (th_ma == 400)
      val = SC8551_IBAT_REG_400MA;
  else if (th_ma == 500)
      val = SC8551_IBAT_REG_500MA;
  else
      val = SC8551_IBAT_REG_500MA;

  val <<= SC8551_IBAT_REG_SHIFT;
  ret = sc8551_update_bits(priv, SC8551_REG_2C,
                           SC8551_IBAT_REG_MASK, val);

  return ret;
}

static int sc8551_set_vbat_reg_th(FAR struct sc8551_dev_s *priv, int th_mv)
{
  int ret;
  uint8_t val;

  switch (th_mv)
  {
    case 50:
      val = SC8551_VBAT_REG_50MV;
      break;
    case 100:
      val = SC8551_VBAT_REG_100MV;
      break;
    case 150:
      val = SC8551_VBAT_REG_150MV;
      break;
    default:
      val = SC8551_VBAT_REG_200MV;
      break;
  }

  val <<= SC8551_VBAT_REG_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_2C,
                           SC8551_VBAT_REG_MASK, val);

  return ret;
}

static int sc8551_enable_regulation(FAR struct sc8551_dev_s *priv,
                                    bool enable)
{
  int ret;
  uint8_t val;

  if (enable)
    val = SC8551_EN_REGULATION_ENABLE;
  else
    val = SC8551_EN_REGULATION_DISABLE;

  val <<= SC8551_EN_REGULATION_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_2B,
                           SC8551_EN_REGULATION_MASK, val);

  return ret;
}

static int sc8551_init_regulation(FAR struct sc8551_dev_s *priv)
{
  sc8551_set_ibat_reg_th(priv, 300);
  sc8551_set_vbat_reg_th(priv, 100);

  sc8551_set_vdrop_deglitch(priv, 5000);
  sc8551_set_vdrop_th(priv, 400);

  sc8551_enable_regulation(priv, false);    /* test add */

  sc8551_update_bits(priv, SC8551_REG_2E, 0x08, 0x08);
  sc8551_update_bits(priv, SC8551_REG_34, 0x01, 0x01);

  return 0;
}

/****************************************************************************
 * Name: sc8551_reset
 *
 * Description:
 *   Reset the SC8551
 *
 ****************************************************************************/

static int sc8551_reset(FAR struct sc8551_dev_s *priv)
{
  int ret;
  uint8_t val;

  /* Read current register value */

  ret = sc8551_getreg8(priv, SC8551_REG_0B, &val, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from SC8551! Error = %d\n", ret);
      return ret;
    }

  /* Send reset command */

  val |= SC8551_REG_RST_ENABLE;
  ret = sc8551_putreg8(priv, SC8551_REG_0B, val);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to SC8551! Error = %d\n", ret);
      return ret;
    }

  /* Wait a little bit to clear registers */

  nxsig_usleep(500);

#ifndef NOT_PMIC
  /* Inherit from BQ2429, unknown useness for SC8551 */

  /* Maybe there is a BUG in SC8551 */

  val &= ~(SC8551_REG_RST_MASK);
  ret = sc8551_putreg8(priv, SC8551_REG_01, val);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to SC8551! Error = %d\n", ret);
      return ret;
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: sc8551_watchdog
 *
 * Description:
 *   Enable/Disable the SC8551 watchdog
 *
 ****************************************************************************/

static int sc8551_watchdog(FAR struct sc8551_dev_s *priv, bool enable)
{
  int ret;
  uint8_t val;

  ret = sc8551_getreg8(priv, SC8551_REG_0B, &val, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from SC8551! Error = %d\n", ret);
      return ret;
    }

  if (enable)
    {
      /* Hw Default is 40Sec so use that for time being */

      val &= ~(SC8551_WATCHDOG_MASK);
      val |= SC8551_WATCHDOG_30S;
    }
  else
    {
      /* 0 is disable */

      val &= ~(SC8551_WATCHDOG_DIS_MASK);
    }

  ret = sc8551_putreg8(priv, SC8551_REG_0B, val);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to SC8551! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: sc8551_sysoff
 *
 * Description:
 *   Turn the internal battery FET off in order to reduce the leakage from
 *   the BAT pin. Note that this disconnects the battery from the system.
 *
 ****************************************************************************/

static int sc8551_sysoff(FAR struct sc8551_dev_s *priv)
{
  #ifndef NOT_PMIC /* The chip does not support the function */
  int ret;
  uint8_t value = 0;

  ret = sc8551_getreg8(priv, SC8551_REG_07, &value, 1);
  batwarn("REG7 read value: 0x%08X\n", value);
  value |= SC8551R7_BATFET_DISABLE;
  ret |= sc8551_putreg8(priv, SC8551_REG_07, value);

  return ret;
  #endif

  batwarn(" The chip does not support the function of sc8551_sysoff ");

  return ERROR;
}

/****************************************************************************
 * Name: sc8551_syson
 *
 * Description:
 *   Turn the internal battery FET on.
 *
 ****************************************************************************/

static int sc8551_syson(FAR struct sc8551_dev_s *priv)
{
  #ifndef NOT_PMIC /* The chip does not support the function */
  int ret;
  uint8_t value = 0;

  ret = sc8551_getreg8(priv, SC8551_REG_07, &value, 1);
  batwarn("REG7 read value: 0x%08X\n", value);
  value &= ~SC8551R7_BATFET_DISABLE;
  ret |= sc8551_putreg8(priv, SC8551_REG_07, value);

  return ret;
  #endif

  batwarn(" The chip does not support the function of sc8551_syson ");

  return ERROR;
}

/****************************************************************************
 * Name: sc8551_en_term
 *
 * Description:
 *   Enable charger termination.  When termination is disabled, there are no
 *   indications of the charger terminating (i.e. STAT pin or registers).
 *
 ****************************************************************************/

static int sc8551_en_term(FAR struct sc8551_dev_s *priv, bool state)
{
  #ifndef NOT_PMIC  /* The chip does not support the function */
  uint8_t val;
  int ret;

  ret = sc8551_getreg8(priv, SC8551_REG_05, &val, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from SC8551 REG5! Error = %d\n", ret);
      return ret;
    }

  batwarn("en_term: REG05 %02X EN_TERM=%d\n",
         val, !!(val & SC8551R5_EN_TERM));

  /* Clear previous and set new value */

  if (state)
    {
      val |= SC8551R5_EN_TERM;
    }
  else
    {
      val &= ~SC8551R5_EN_TERM;
    }

  ret = sc8551_putreg8(priv, SC8551_REG_05, val);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to SC8551R5! Error = %d\n", ret);
      return ret;
    }

  return OK;
  #endif

  batwarn("The chip does not support the function of sc8551_en_term");

  return ERROR;
}

/****************************************************************************
 * Name: sc8551_en_hiz
 *
 * Description:
 *   Enable high-impedance mode. Sets the charger IC into low power standby
 *   mode.
 *
 ****************************************************************************/

static int sc8551_en_hiz(FAR struct sc8551_dev_s *priv, bool state)
{
  #ifndef NOT_PMIC /* The chip does not support the function */
  uint8_t val;
  int ret;

  ret = sc8551_getreg8(priv, SC8551_REG_00, &val, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from SC8551 REG0! Error = %d\n", ret);
      return ret;
    }

  batwarn("en_hiz: REG00 %02X EN_HIZ=%d\n",
         val, !!(val & SC8551R1_EN_HIZ));

  /* Clear previous and set new value */

  if (state)
    {
      val |= SC8551R1_EN_HIZ;
    }
  else
    {
      val &= ~SC8551R1_EN_HIZ;
    }

  ret = sc8551_putreg8(priv, SC8551_REG_00, val);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to SC8551R0! Error = %d\n", ret);
      return ret;
    }

  return OK;
  #endif

  batwarn("The chip does not support the function of sc8551_en_hiz ");

  return ERROR;
}

/****************************************************************************
 * Name: sc8551_en_stat
 *
 * Description:
 *   Enable interrupts.
 *
 ****************************************************************************/

static int sc8551_en_stat(FAR struct sc8551_dev_s *priv, bool state)
{
  #ifndef NOT_PMIC /* The chip does not support the function */
  uint8_t val;
  int ret;

  ret = sc8551_getreg8(priv, SC8551_REG_07, &val, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from SC8551 REG7! Error = %d\n", ret);
      return ret;
    }

  batwarn("int stat: REG07 %02X INT_MASK1=%d INT_MASK0=%d\n", val,
         !!(val & SC8551R7_INT_MASK1), !!(val & SC8551R7_INT_MASK0));

  /* We always set or clear both interrupts together. */

  if (state)
    {
      val |= (SC8551R7_INT_MASK0 | SC8551R7_INT_MASK1);
    }
  else
    {
      val &= ~(SC8551R7_INT_MASK0 | SC8551R7_INT_MASK1);
    }

  ret = sc8551_putreg8(priv, SC8551_REG_07, val);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to SC8551R7! Error = %d\n", ret);
      return ret;
    }

  return OK;
  #endif

  batwarn("The chip does not support the function of sc8551_en_stat ");

  return ERROR;
}

/****************************************************************************
 * Name: sc8551_setboost_otg_config
 *
 * Description:
 *   Set the device boost mode.
 *
 ****************************************************************************/

static int sc8551_setboost_otg_config(FAR struct sc8551_dev_s *priv,
                                      bool state)
{
  #ifndef NOT_PMIC /* The chip does not support the function */
  uint8_t val;
  int ret;

  ret = sc8551_getreg8(priv, SC8551_REG_01, &val, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from SC8551! Error = %d\n", ret);
      return ret;
    }

  /* Clear previous current and set new value */

  if (state)
    {
      /* Set to Boost disable Charge */

      val = SC8551R1_OTG_CONFIG | (val & ~SC8551R1_CHG_CONFIG);
    }
  else
    {
      /* Set to Charge disable Boost */

      val = SC8551R1_CHG_CONFIG | (val & ~SC8551R1_OTG_CONFIG);
    }

  ret = sc8551_putreg8(priv, SC8551_REG_01, val);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to SC8551! Error = %d\n", ret);
      return ret;
    }

#define BST_CONFIG_MASK (SC8551R1_CHG_CONFIG | SC8551R1_OTG_CONFIG)
  batwarn("otg_config: REG01 %02X Boost=%d\n", val,
      ((SC8551R1_OTG_CONFIG == (val & BST_CONFIG_MASK)) ? 1 : 0));

  return OK;
  #endif

  batwarn("The chip does not support the sc8551_setboost_otg_config ");

  return ERROR;
}

/****************************************************************************
 * Name: sc8551_state
 *
 * Description:
 *   Return the current battery management state
 *
 ****************************************************************************/

static int sc8551_state(FAR struct battery_charger_dev_s *dev,
                        FAR int *status)
{
  FAR struct sc8551_dev_s *priv = (FAR struct sc8551_dev_s *)dev;
  uint8_t val;
  bool isfault = false;
  int ret;

  ret = sc8551_getreg8(priv, SC8551_REG_0D, &val, 1);
  if (ret < 0)
    {
      *status = BATTERY_UNKNOWN;
      return ret;
    }

  if (ret & VBUS_INSERT)
    {
      priv->stat.vbus_present = true;
      *status = BATTERY_CHARGING;
    }
  else
    {
      priv->stat.vbus_present = false;
      *status = BATTERY_IDLE;
    }

  return OK;
}

/****************************************************************************
 * Name: sc8551_health
 *
 * Description:
 *   Return the current battery health state
 *
 * Note: if more than one fault happened the user needs to call this ioctl
 * again to read a new fault, repeat until receive a BATTERY_HEALTH_GOOD.
 *
 ****************************************************************************/

static int sc8551_health(FAR struct battery_charger_dev_s *dev,
                         FAR int *health)
{
  batwarn("The chip does not support the function of sc8551_health ");
  return ERROR;
}

/****************************************************************************
 * Name: sc8551_online
 *
 * Description:
 *   Return true if the battery is online
 *
 ****************************************************************************/

static int sc8551_online(FAR struct battery_charger_dev_s *dev,
                         FAR bool *status)
{
  batwarn("The chip does not support the function of sc8551_online ");
  return ERROR;
}

/****************************************************************************
 * Name: sc8551_powersupply
 *
 * Description:
 *   Set the Power Supply Current Limit.
 *
 ****************************************************************************/

static int sc8551_powersupply(FAR struct sc8551_dev_s *priv, int current)
{
  batwarn("The chip does not support the function of sc8551_powersupply ");
  return ERROR;
}

/****************************************************************************
 * Name: sc8551_setvolt
 *
 * Description:
 *   Set the voltage level to charge the battery. Voltage value in mV.
 *
 ****************************************************************************/

static int sc8551_setvolt(FAR struct sc8551_dev_s *priv, int req_volts)
{
  batwarn("The chip does not support the function of sc8551_powersupply ");
  return ERROR;
}

/****************************************************************************
 * Name: sc8551_setcurr
 *
 * Description:
 *   Set the current to charge the battery. Current value in mA.
 *
 ****************************************************************************/

static inline int sc8551_setcurr(FAR struct sc8551_dev_s *priv,
                                 int req_current)
{
  batwarn("The chip does not support the function of sc8551_setcurr ");
  return ERROR;
}

/****************************************************************************
 * Name: sc8551_voltage
 *
 * Description:
 *   Set battery charger voltage
 *
 ****************************************************************************/

static int sc8551_voltage(FAR struct battery_charger_dev_s *dev, int value)
{
  batwarn("The chip does not support the function of sc8551_voltage ");
  return ERROR;
}

/****************************************************************************
 * Name: sc8551_current
 *
 * Description:
 *   Set the battery charger current rate for charging
 ****************************************************************************/

static int sc8551_current(FAR struct battery_charger_dev_s *dev, int value)
{
  batwarn("The chip does not support the function of sc8551_current ");
  return ERROR;
}

/****************************************************************************
 * Name: sc8551_input_current
 *
 * Description:
 *   Set the power-supply input current limit
 *
 ****************************************************************************/

static int sc8551_input_current(FAR struct battery_charger_dev_s *dev,
                                int value)
{
  batwarn("The chip does not support the function of sc8551_input_current ");
  return ERROR;
}

/****************************************************************************
 * Name: sc8551_operate
 ****************************************************************************/

static int sc8551_operate(FAR struct battery_charger_dev_s *dev,
                          uintptr_t param)
{
  FAR struct sc8551_dev_s *priv = (FAR struct sc8551_dev_s *)dev;
  FAR struct batio_operate_msg_s *msg =
    (FAR struct batio_operate_msg_s *) param;
  int op;
  int value;
  int ret = OK;

  sc8551_dump_regs(priv);

  op = msg->operate_type;
  value = (int)msg->u32;
  switch (op)
    {
      case BATIO_OPRTN_BOOST:
        ret = sc8551_setboost_otg_config(priv, true);
        break;

      case BATIO_OPRTN_CHARGE:
        ret = sc8551_setboost_otg_config(priv, false);
        break;

      case BATIO_OPRTN_EN_TERM:
        ret = sc8551_en_term(priv, (bool)value);
        break;

      case BATIO_OPRTN_HIZ:
        ret = sc8551_en_hiz(priv, (bool)value);

        /* Also need to set to 100mA USB host if the battery above Vbatgd? */

        break;

      case BATIO_OPRTN_SYSOFF:
        ret = sc8551_sysoff(priv);
        break;

      case BATIO_OPRTN_SYSON:
        ret = sc8551_syson(priv);
        break;

      case BATIO_OPRTN_RESET:
        ret = sc8551_reset(priv);
        break;

      case BATIO_OPRTN_WDOG:
        ret = sc8551_watchdog(priv, (bool)value);
        break;

      default:
        baterr("Unsupported opt: 0x%X\n", op);
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: sc8551_initialize
 *
 * Description:
 *   Initialize the SC8551 battery driver and return an instance of the
 *   lower-half interface that may be used with battery_charger_register().
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY_CHARGER - Upper half battery driver support
 *   CONFIG_I2C - I2C support
 *   CONFIG_I2C_SC8551 - And the driver must be explicitly selected.
 *
 * Input Parameters:
 *   i2c       - An instance of the I2C interface to use to communicate with
 *               the SC8551
 *   addr      - The I2C address of the SC8551 (Better be 0x6B).
 *   frequency - The I2C frequency
 *   current   - The input current our power-supply can offer to charger
 *
 * Returned Value:
 *   A pointer to the initialized lower-half driver instance.  A NULL pointer
 *   is returned on a failure to initialize the SC8551 lower half.
 *
 ****************************************************************************/

FAR struct battery_charger_dev_s *
  sc8551_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                    uint32_t frequency, int current)
{
  FAR struct sc8551_dev_s *priv;
  int ret;

  /* Initialize the SC8551 device structure */

  priv = kmm_zalloc(sizeof(struct sc8551_dev_s));
  if (priv)
    {
      /* Initialize the SC8551 device structure */

      priv->dev.ops   = &g_sc8551ops;
      priv->i2c       = i2c;
      priv->addr      = addr;
      priv->frequency = frequency;

      /* Reset the SC8551 */

      ret = sc8551_reset(priv);
      if (ret < 0)
        {
          baterr("ERROR: Failed to reset the SC8551: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Disable watchdog otherwise SC8551 returns to StandAlone mode */

      ret = sc8551_watchdog(priv, false);
      if (ret < 0)
        {
          baterr("ERROR: Failed to disable SC8551 watchdog: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Define the current that our power supply can offer to the charger. */

      ret = sc8551_powersupply(priv, current);
      if (ret < 0)
        {
          baterr("ERROR: Failed to set SC8551 power supply: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Disable all interrupts. */

      ret = sc8551_en_stat(priv, false);
      if (ret < 0)
        {
          baterr("ERROR: Failed to disable SC8551 interrupts: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Detect the SC8551 and set part_no */

      ret = sc8551_detect_device(priv);
      if (ret < 0)
        {
          baterr("ERROR: Failed to detect_device: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* disable watchdog */

      ret = sc8551_watchdog(priv, false);
      if (ret < 0)
        {
          baterr("ERROR: Failed to disable watchdog: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Adjust timeout to rise to threshold */

      ret = sc8551_set_ss_timeout(priv, 100000);
      if (ret < 0)
        {
          baterr("ERROR: Failed to Adjust timeout to \
            rise to threshold: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      ret = sc8551_set_sense_resistor(priv, g_sc8551_cfg.sense_r_mohm);
      if (ret < 0)
        {
          baterr("ERROR: Failed to set sense resistor: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      ret = sc8551_init_protection(priv);
      if (ret < 0)
        {
          baterr("ERROR: Failed to init protection: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      ret = sc8551_init_adc(priv);
      if (ret < 0)
        {
          baterr("ERROR: Failed to init adc: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      ret = sc8551_init_int_src(priv);
      if (ret < 0)
        {
          baterr("ERROR: Failed to init src: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      ret = sc8551_init_regulation(priv);
      if (ret < 0)
        {
          baterr("ERROR: Failed to init regulation: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }
    }

  return (FAR struct battery_charger_dev_s *)priv;
}

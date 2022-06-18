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
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/wqueue.h>

#include "sc8551_reg.h"
#include "sc8551.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NOT_PMIC

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sc8551_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_charger_dev_s dev; /* Battery charger device */

  /* Data fields specific to the lower half SC8551 driver follow */

  uint8_t addr;                         /* I2C address */
  uint32_t frequency;                   /* I2C frequency */
  uint32_t pin;                         /* Interrupt pin */
  uint32_t current;                     /* pump current */
  FAR struct i2c_master_s *i2c;         /* I2C interface */
  FAR struct ioexpander_dev_s *ioedev;  /* Ioexpander device */
  struct work_s work;                   /* Interrupt handler worker */

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
static int sc8551_get_temp(FAR struct sc8551_dev_s *priv);
static int sc8551_watchdog(FAR struct sc8551_dev_s *priv, bool enable);
static int sc8551_en_adc(FAR struct sc8551_dev_s *priv, bool state);
static int sc8551_powersupply(FAR struct sc8551_dev_s *priv, int current);

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
static int sc8551_chipid(FAR struct battery_charger_dev_s *dev,
                         unsigned int *value);
static int sc8551_get_voltage(FAR struct battery_charger_dev_s *dev,
                                 int *value);

/* Charger pump interrupt functions */

static int sc8551_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                     ioe_pinset_t pinset, FAR void *arg);
static void sc8551_worker(FAR void *arg);

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
  .bat_ocp_th = 2500,
  .bat_ocp_alm_th = 2000,
  .bus_ovp_alm_disable = true,
  .bus_ocp_disable = false,
  .bus_ocp_alm_disable = true,
  .bus_ovp_th = 12350,
  .bus_ovp_alm_th = 10500,
  .bus_ocp_th = 1250,
  .bus_ocp_alm_th = 3800,
  .bat_ucp_alm_disable = false,
  .bat_ucp_alm_th = 2600,
  .ac_ovp_th = 130,
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
  sc8551_chipid,
  sc8551_get_voltage,
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
  FAR struct i2c_master_s *dev = priv->i2c;
  struct i2c_msg_s msg[2];
  int err;
  int retries = 0;

  msg[0].frequency = priv->frequency;
  msg[0].addr = priv->addr;
  msg[0].buffer = &regaddr;
  msg[0].length = 1;
  msg[0].flags = I2C_M_NOSTOP;

  msg[1].frequency = priv->frequency;
  msg[1].addr = priv->addr;
  msg[1].buffer = val;
  msg[1].length = num_char;
  msg[1].flags = I2C_M_READ;

  for (retries = 0; retries < SC_IIC_RETRY_NUM; retries++)
    {
      err = I2C_TRANSFER(dev, msg, 2);
      if (err >= 0)
        {
          break;
        }
      else
        {
          nxsig_usleep(1);
          baterr("ERROR: i2c transfer failed! err: %d retries:%d\n",
                  err, retries);
        }
    }

  return (err >= 0) ? OK : err;
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

  batinfo("addr: %02x val: %08x\n", regaddr, val);

  /* Set up a message to send */

  buffer[0] = regaddr;
  buffer[1] = val;

  /* Write the register address followed by the data (no RESTART) */

  return i2c_write(priv->i2c, &config, buffer, 2);
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

  val_r &= ~mask;
  val_r |= val & mask;

  ret = sc8551_putreg8(priv, reg, val_r);
  if (ret < 0)
    {
      baterr("[ SC8551] Failed: reg=%02X\r\n", reg);
      return -1;
    }

  return 0;
}

static uint16_t sc8551_get_vbus(FAR struct sc8551_dev_s * priv,
                                uint16_t *vbus)
{
  uint8_t vbus_reg[2];
  int ret;

  ret  = sc8551_getreg8(priv, SC8551_REG_18, &vbus_reg[0], 1);
  ret  = sc8551_getreg8(priv, SC8551_REG_19, &vbus_reg[1], 1);
  if (ret < 0)
    {
      baterr("ERROR: Failed to get vbus voltage of sc8551: %d\n", ret);
      return ERROR;
    }

  *vbus = (((vbus_reg[0] & SC8551_IBUS_POL_H_MASK) << 8)
         | (vbus_reg[1] & SC8551_VBUS_POL_L_MASK))
         * SC8551_VBUS_ADC_LSB;

  return ret;
}

static int sc8551_get_vout(FAR struct sc8551_dev_s * priv, uint16_t *vout)
{
  uint8_t vout_reg[2];
  int ret;

  ret = sc8551_getreg8(priv, SC8551_REG_1C, &vout_reg[0], 1);
  ret = sc8551_getreg8(priv, SC8551_REG_1D, &vout_reg[1], 1);
  if (ret < 0)
    {
      baterr("ERROR: Failed to get output voltage of sc8551: %d\n", ret);
      return ERROR;
    }

  *vout = (((vout_reg[0] & SC8551_VOUT_POL_H_MASK) << 8
           | (vout_reg[1] & SC8551_VOUT_POL_L_MASK)))
           * SC8551_VOUT_ADC_LSB;

  return ret;
}

static int sc8551_get_flt_stat(FAR struct sc8551_dev_s * priv,
                                bool *vbat_ovp,
                                bool *ibat_ocp,
                                bool *vbus_ovp,
                                bool *ibus_ocp)
{
  uint8_t flt_reg;
  int ret;

  ret  = sc8551_getreg8(priv, SC8551_REG_11, &flt_reg, 1);
  if (ret < 0)
    {
      baterr("ERROR: Failed to get flt_stat of sc8551: %d\n", ret);
      return ERROR;
    }

  if (flt_reg & SC8551_BAT_OVP_FLT_FLAG_MASK) *vbat_ovp = true;
  if (flt_reg & SC8551_BAT_OCP_FLT_FLAG_MASK) *ibat_ocp = true;
  if (flt_reg & SC8551_BUS_OVP_FLT_FLAG_MASK) *vbus_ovp = true;
  if (flt_reg & SC8551_BUS_OCP_FLT_FLAG_MASK) *ibus_ocp = true;

  return ret;
}

static int sc8551_get_ibus_ucp(FAR struct sc8551_dev_s * priv,
                               bool *ibus_ucp)
{
  uint8_t ibus_reg;
  int ret;

  ret  = sc8551_getreg8(priv, SC8551_REG_08, &ibus_reg, 1);
  if (ret < 0)
    {
      baterr("ERROR: Failed to get flt_stat of sc8551: %d\n", ret);
      return ERROR;
    }

  if (ibus_reg & SC8551_IBUS_UCP_FALL_FLAG_MASK) *ibus_ucp = true;

  return ret;
}

static int sc8551_get_int_stat(FAR struct sc8551_dev_s * priv,
                               bool *adapter_insert,
                               bool *vbat_insert,
                               bool *adc_done)
{
  uint8_t int_reg;
  int ret;

  ret = sc8551_getreg8(priv, SC8551_REG_0D, &int_reg, 1);
  if (ret < 0)
    {
      baterr("ERROR: Failed to get flt_stat of sc8551: %d\n", ret);
      return ERROR;
    }

  if (int_reg & VBUS_INSERT) *adapter_insert = true;
  if (int_reg & VBAT_INSERT) *vbat_insert = true;
  if (int_reg & ADC_DONE) *adc_done = true;

  return ret;
}

static int sc8551_get_converse_stat(FAR struct sc8551_dev_s * priv,
                                    bool *vbus_errorlo_stat,
                                    bool *vbus_errorhi_stat,
                                    bool *cp_switching_stat)
{
  uint8_t conv_reg;
  int ret;

  ret = sc8551_getreg8(priv, SC8551_REG_0A, &conv_reg, 1);
  if (ret < 0)
    {
      baterr("ERROR: Failed to get flt_stat of sc8551: %d\n", ret);
      return ERROR;
    }

  if (conv_reg & SC8551_VBUS_ERRORLO_STAT_MASK) *vbus_errorlo_stat = true;
  if (conv_reg & SC8551_VBUS_ERRORHI_STAT_MASK) *vbus_errorhi_stat = true;
  if (conv_reg & SC8551_CONV_SWITCHING_STAT_MASK) *cp_switching_stat = true;

  return ret;
}

static int sc8551_get_charge_en_stat(FAR struct sc8551_dev_s * priv,
                                      bool *charge_en_stat)
{
  uint8_t chg_ctl;
  int ret;

  ret = sc8551_getreg8(priv, SC8551_REG_0C, &chg_ctl, 1);
  if (ret < 0)
    {
      baterr("ERROR: Failed to get charge_en_stat of sc8551: %d\n", ret);
      return ERROR;
    }

  if (chg_ctl & SC8551_CHG_EN_MASK) *charge_en_stat = true;

  return ret;
}

static int sc8551_get_key_state(FAR struct sc8551_dev_s * priv,
                            FAR struct sc8551_key_state_s *sc8551_key_state)
{
  int ret;

  ret = sc8551_get_vbus(priv,
                       &(sc8551_key_state->vbus));
  ret = sc8551_get_vout(priv,
                       &(sc8551_key_state->vout));
  ret = sc8551_get_flt_stat(priv,
                           &(sc8551_key_state->vbus_ovp),
                           &(sc8551_key_state->ibus_ocp),
                           &(sc8551_key_state->vbat_ovp),
                           &(sc8551_key_state->ibat_ocp));
  ret = sc8551_get_ibus_ucp(priv,
                           &(sc8551_key_state->ibus_ucp));
  ret = sc8551_get_int_stat(priv,
                           &(sc8551_key_state->adapter_insert),
                           &(sc8551_key_state->vbat_insert),
                           &(sc8551_key_state->adc_done));
  ret = sc8551_get_converse_stat(priv,
                                 &(sc8551_key_state->vbus_errorlo_stat),
                                 &(sc8551_key_state->vbus_errorhi_stat),
                                 &(sc8551_key_state->cp_switching_stat));
  ret = sc8551_get_charge_en_stat(priv,
                                 &(sc8551_key_state->charge_en_stat));
  if (ret < 0)
    {
      baterr("ERROR: !!! Failed to get key_state of sc8551: %d\n", ret);
      return ERROR;
    }

#ifdef CONFIG_DEBUG_SC8551
  batinfo("INFO: sc8551_key_state: \n");
  batinfo("  vbus = %d\n", sc8551_key_state->vbus);
  batinfo("  vout = %d\n", sc8551_key_state->vout);
  batinfo("  vbat_ovp = %s\n",
          sc8551_key_state->vbat_ovp ? "true" : "false");
  batinfo("  ibat_ocp = %s\n",
          sc8551_key_state->ibat_ocp ? "true" : "false");
  batinfo("  vbus_ovp = %s\n",
          sc8551_key_state->vbus_ovp ? "true" : "false");
  batinfo("  ibus_ocp = %s\n",
          sc8551_key_state->ibus_ocp ? "true" : "false");
  batinfo("  ibus_ucp = %s\n",
          sc8551_key_state->ibus_ucp ? "true" : "false");
  batinfo("  adapter_insert = %s\n",
          sc8551_key_state->adapter_insert ? "true" : "false");
  batinfo("  vbat_insert = %s\n",
          sc8551_key_state->vbat_insert ? "true" : "false");
  batinfo("  adc_done = %s\n",
          sc8551_key_state->adc_done ? "true" : "false");
  batinfo("  vbus_errorlo_stat = %s\n",
          sc8551_key_state->vbus_errorlo_stat ? "true" : "false");
  batinfo("  vbus_errorhi_stat = %s\n",
          sc8551_key_state->vbus_errorhi_stat ? "true" : "false");
  batinfo("  cp_switching_stat = %s\n",
          sc8551_key_state->cp_switching_stat ? "true" : "false");
  batinfo("  charge_en_stat = %s\n",
          sc8551_key_state->charge_en_stat ? "true" : "false");

#endif

  return ret;
}

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
      return ret;
    }

  priv->stat.part_no = (val & SC8551_DEV_ID_MASK);
  priv->stat.part_no >>= SC8551_DEV_ID_SHIFT;

  return 0;
}

static int sc8551_set_ss_timeout(FAR struct sc8551_dev_s *priv, int timeout)
{
  int ret;
  uint8_t val;

  switch (timeout)
    {
      case 0:
        val = SC8551_SS_TIMEOUT_DISABLE;
      break;
      case 40:
        val = SC8551_SS_TIMEOUT_40MS;
      break;
      case 80:
        val = SC8551_SS_TIMEOUT_80MS;
      break;
      case 320:
        val = SC8551_SS_TIMEOUT_320MS;
      break;
      case 1280:
        val = SC8551_SS_TIMEOUT_1S280MS;
      break;
      case 5120:
        val = SC8551_SS_TIMEOUT_5S120MS;
      break;
      case 20480:
        val = SC8551_SS_TIMEOUT_20S480MS;
      break;
      case 81920:
        val = SC8551_SS_TIMEOUT_81S920MS;
      break;
      default:
        val = SC8551_SS_TIMEOUT_DISABLE;
      break;
    }

  val <<= SC8551_SS_TIMEOUT_SET_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_2B,
                      SC8551_SS_TIMEOUT_SET_MASK, val);
  return ret;
}

static int sc8551_set_sense_resistor(FAR struct sc8551_dev_s *priv,
                                         int r_mohm)
{
  int ret;
  uint8_t val;

  if (r_mohm == 2)
    val = SC8551_SET_IBAT_SNS_RES_2MHM;
  else if (r_mohm == 5)
    val = SC8551_SET_IBAT_SNS_RES_5MHM;
  else
    {
      return -1;
    }

  val <<= SC8551_SET_IBAT_SNS_RES_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_2B,
                    SC8551_SET_IBAT_SNS_RES_MASK, val);

  return ret;
}

static int sc8551_enable_ibus_ucp(FAR struct sc8551_dev_s *priv, bool enable)
{
  int ret;
  uint8_t val;

  if (enable)
    {
      val = SC8551_IBUS_UCP_ENABLE;
    }
  else
    {
      val = SC8551_IBUS_UCP_DISABLE;
    }

  val <<= SC8551_IBUS_UCP_SHIFT;
  ret = sc8551_update_bits(priv, SC8551_REG_31,
                           SC8551_IBUS_UCP_MASK, val);

  return ret;
}

static int sc8551_enable_charge(FAR struct sc8551_dev_s *priv, bool enable)
{
  int ret;
  uint8_t val;

  if (enable)
    {
      val = SC8551_CHG_ENABLE;
    }
  else
    {
      val = SC8551_CHG_DISABLE;
    }

  val <<= SC8551_CHG_EN_SHIFT;
  ret = sc8551_update_bits(priv, SC8551_REG_0C,
                           SC8551_CHG_EN_MASK, val);

  return ret;
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

/****************************************************************************
 *the input threshold is the raw value that would write to register directly.
 ****************************************************************************/

static int sc8551_set_bat_therm_th(FAR struct sc8551_dev_s *priv,
                                   uint8_t threshold)
{
  int ret;

  ret = sc8551_putreg8(priv, SC8551_REG_29, threshold);

  return ret;
}

static int sc8551_enable_bus_therm(FAR struct sc8551_dev_s *priv,
                                   bool enable)
{
  int ret;
  uint8_t val;

  if (enable)
    val = SC8551_TSBUS_ENABLE;
  else
    val = SC8551_TSBUS_DISABLE;

  val <<= SC8551_TSBUS_DIS_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_0C,
                  SC8551_TSBUS_DIS_MASK, val);

  return ret;
}

/****************************************************************************
 *the input threshold is the raw value that would write to register directly.
 ****************************************************************************/

static int sc8551_set_bus_therm_th(FAR struct sc8551_dev_s *priv,
                                   uint8_t threshold)
{
  int ret;

  ret = sc8551_putreg8(priv, SC8551_REG_28, threshold);

  return ret;
}

/****************************************************************************
 * please be noted that the unit here is degC
 ****************************************************************************/

static int sc8551_set_die_therm_th(FAR struct sc8551_dev_s *priv,
                                   uint8_t threshold)
{
  int ret;
  uint8_t val;

  /* BE careful, LSB is here is 1/LSB, so we use multiply here */

  val = (threshold - SC8551_TDIE_ALM_BASE) * 10 / SC8551_TDIE_ALM_LSB;
  val <<= SC8551_TDIE_ALM_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_2A,
                  SC8551_TDIE_ALM_MASK, val);

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

static int sc8551_enable_adc(FAR struct sc8551_dev_s *priv, bool enable)
{
  int ret;
  uint8_t val;

  if (enable)
    val = SC8551_ADC_ENABLE;
  else
    val = SC8551_ADC_DISABLE;

  val <<= SC8551_ADC_EN_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_14,
                    SC8551_ADC_EN_MASK, val);

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

  sc8551_enable_adc(priv, false);

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
      batinfo("[ SC8551] failed to set alarm mask:%d\r\n", ret);
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
 * Name: sc8551_get_temp
 *
 * Description:
 *   Get the temperature of sc8551
 *
 ****************************************************************************/

static int sc8551_get_temp(FAR struct sc8551_dev_s *priv)
{
  int ret;
  int temp = 0;
  uint8_t value;

  ret = sc8551_getreg8(priv, SC8551_REG_27, &value, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from SC8551_REG_27! Error = %d\n", ret);
      return ret;
    }

  if (value == 0xff)
    temp = value >> 1;
  else
    temp = (value +1) >> 1;

  return temp;
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

  val = SC8551_REG_RST_ENABLE;
  val <<= SC8551_REG_RST_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_0B,
                           SC8551_REG_RST_MASK, val);

  /* Wait a little bit to clear registers */

  nxsig_usleep(500);

  return ret;
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

  if (enable)
    {
      val = SC8551_WATCHDOG_ENABLE;
    }
  else
    {
      val = SC8551_WATCHDOG_DISABLE;
    }

  val <<= SC8551_WATCHDOG_DIS_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_0B,
                      SC8551_WATCHDOG_DIS_MASK, val);

  return ret;
}

/****************************************************************************
 * Name: sc8551_en_adc
 *
 * Description:
 *   Enable charger adc.  When adc is disabled, there are no charger current.
 *
 ****************************************************************************/

static int sc8551_en_adc(FAR struct sc8551_dev_s *priv, bool enable)
{
  uint8_t val;
  int ret;

  if (enable)
    val = SC8551_ADC_ENABLE;
  else
    val = SC8551_ADC_DISABLE;

  val <<= SC8551_ADC_EN_SHIFT;

  ret = sc8551_update_bits(priv, SC8551_REG_14,
                           SC8551_ADC_EN_MASK, val);
  return ret;
}

/****************************************************************************
 * Name: sc8551_interrupt_handler
 *
 * Description:
 *   Handle the pump interrupt.
 *
 * Input Parameters:
 *   dev     - ioexpander device.
 *   pinset  - Interrupt pin.
 *   arg     - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sc8551_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                     ioe_pinset_t pinset, FAR void *arg)
{
  /* This function should be called upon a rising edge on the sc8551 new
   * data interrupt pin since it signals that new data has been measured.
   */

  FAR struct sc8551_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  work_queue(LPWORK, &priv->work, sc8551_worker, priv, 0);

  return OK;
}

/****************************************************************************
 * Name: sc8551_worker
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data. We should not do
 *   this in a interrupt since it might take too long. Also we cannot lock
 *   the I2C bus from within an interrupt.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   none.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sc8551_readpump(FAR struct sc8551_dev_s *priv)
{
  /* to-do */

  return OK;
}

static void sc8551_worker(FAR void *arg)
{
  int ret;
  FAR struct sc8551_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  /* Read out the latest pump data */

  ret = sc8551_readpump(priv);

  if (ret == 0)
    {
      /* push data to upper half driver */

      baterr("SUCCESS: sc8551_readpump!");
    }

  return;
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
  FAR struct sc8551_key_state_s sc8551_key_state;
  int ret;

  memset(&sc8551_key_state, 0, sizeof(sc8551_key_state));
  ret = sc8551_get_key_state(priv, &sc8551_key_state);
  if (ret < 0)
    {
      baterr("ERROR: Error sc8551_get_key_state! Error = %d\n", ret);
      *status = BATTERY_UNKNOWN;
      return ret;
    }

  *status = 0;
  if (sc8551_key_state.vbat_ovp) *status |= VBAT_OVP_MASK;
  if (sc8551_key_state.ibat_ocp) *status |= IBAT_OCP_MASK;
  if (sc8551_key_state.vbus_ovp) *status |= VBUS_OVP_MASK;
  if (sc8551_key_state.ibus_ocp) *status |= IBUS_OCP_MASK;
  if (sc8551_key_state.ibus_ucp) *status |= IBUS_UCP_MASK;
  if (sc8551_key_state.adapter_insert) *status |= ADAPTER_INSERT_MASK;
  if (sc8551_key_state.vbat_insert) *status |= VBAT_INSERT_MASK;
  if (sc8551_key_state.adc_done) *status |= ADC_DONE_MASK;
  if (sc8551_key_state.vbus_errorlo_stat) *status |= VBUS_ERRORLO_STAT_MASK;
  if (sc8551_key_state.vbus_errorhi_stat) *status |= VBUS_ERRORHI_STAT_MASK;
  if (sc8551_key_state.cp_switching_stat) *status |= CP_SWITCHING_STAT_MASK;
  if (sc8551_key_state.charge_en_stat) *status |= CHG_EN_STAT_MASK;

  batinfo("ATTENION INFO: status is 0x%4x \n", *status);

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
  FAR struct sc8551_dev_s *priv = (FAR struct sc8551_dev_s *)dev;
  int temp;

  temp = sc8551_get_temp(priv);
  if (temp < 0)
    *health = SC8551_HEALTH_UNKNOWN;
  else if (temp > SC8551_HEALTH_TEMP_MAX)
    *health = SC8551_HEALTH_OVERHEAT;
  else if (temp < SC8551_HEALTH_TEMP_MIN)
    *health = SC8551_HEALTH_OVERCOLD;
  else
    *health = SC8551_HEALTH_GOOD;

  return OK;
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
  *status = true;
  return OK;
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
  int ret;

  ret = sc8551_enable_busocp(priv, SC8551_BUS_OCP_ENABLE);
  ret = sc8551_set_busocp_th(priv, current);

  if (ret < 0)
    {
      baterr("ERROR: Error setting sc8551 bus OCP ! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: sc8551_voltage
 *
 * Description:
 *   Set battery charger voltage
 *
 ****************************************************************************/

static int  sc8551_voltage(FAR struct battery_charger_dev_s *dev, int value)
{
  batinfo("The chip does not support the function of sc8551_voltage ");
  return OK;
}

/****************************************************************************
 * Name: sc8551_current
 *
 * Description:
 *   Set the battery charger current rate for charging
 ****************************************************************************/

static int sc8551_current(FAR struct battery_charger_dev_s *dev, int value)
{
  FAR struct sc8551_dev_s *priv = (FAR struct sc8551_dev_s *)dev;
  int ret;

  ret = sc8551_set_batocp_th(priv, value);
  if (ret < 0)
    {
      baterr("ERROR: Error Set the charger current! Error = %d\n", ret);
      return ret;
    }

  return OK;
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
  batinfo("The chip does not support the function of sc8551_input_current ");
  return OK;
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

  op = msg->operate_type;
  value = (int)msg->u32;
  switch (op)
    {
      case BATIO_OPRTN_EN_TERM:
        ret = sc8551_en_adc(priv, (bool)value);
        break;

      case BATIO_OPRTN_SYSOFF:
        ret = sc8551_enable_charge(priv, false);
        break;

      case BATIO_OPRTN_CHARGE:
        ret = sc8551_enable_ibus_ucp(priv, (bool)value);
        break;

      case BATIO_OPRTN_SYSON:
        ret = sc8551_enable_charge(priv, true);
        break;

      case BATIO_OPRTN_RESET:
        ret = sc8551_reset(priv);
        break;

      case BATIO_OPRTN_WDOG:
        ret = sc8551_watchdog(priv, (bool)value);
        break;

      default:
        batinfo("Unsupported opt: 0x%X\n", op);
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: sc8551_chipid
 *
 * Description:
 *       Get chip id
 *
 ****************************************************************************/

static int sc8551_chipid(FAR struct battery_charger_dev_s *dev,
                         unsigned int *value)
{
  FAR struct sc8551_dev_s *priv = (FAR struct sc8551_dev_s *)dev;
  int ret;

  ret = sc8551_getreg8(priv, SC8551_REG_13, (uint8_t *)value, 1);
  if (ret < 0)
    {
      baterr("ERROR: Failed to Get sc8551 chipid: %d\n", ret);
      return ERROR;
    }

  batinfo("the chipid of sc8551 is: %d\n", *value);
  return OK;
}

/****************************************************************************
 * Name: sc8551_get_voltage
 *
 * Description:
 *   Get the actual output voltage for charging
 *
 ****************************************************************************/

static int sc8551_get_voltage(FAR struct battery_charger_dev_s *dev,
                                 int *value)
{
  FAR struct sc8551_dev_s *priv = (FAR struct sc8551_dev_s *)dev;
  int ret;
  uint8_t part_info;
  uint8_t val[2];

  ret = sc8551_getreg8(priv, SC8551_REG_13, &part_info, 1);

  ret = sc8551_getreg8(priv, SC8551_REG_1C, &val[0], 1);
  ret = sc8551_getreg8(priv, SC8551_REG_1D, &val[1], 1);
  if (ret < 0)
    {
      baterr("ERROR: Failed to get output voltage of sc8551: %d\n", ret);
      return ERROR;
    }

  if (part_info == SC8551_DEV_ID)
    {
      *value = ((val[0] & 0x0f) << 8 | val[1]) * 5 / 4;
      batinfo("The the actual output voltage of sc8551 is %d mv\n", *value);
    }

  else if (part_info == NU2105_DEV_ID)
    {
      *value = (val[0] & 0x0f) << 8 | val[1];
      batinfo("The the actual output voltage of nu2105 is %d mv\n", *value);
    }

  else
    {
      batinfo("The pump is unknow one");
    }

  return OK;
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
  sc8551_initialize(FAR struct i2c_master_s *i2c,
                    uint32_t pin,
                    uint8_t addr,
                    uint32_t frequency,
                    int current,
                    FAR struct ioexpander_dev_s *dev)
{
  FAR struct sc8551_dev_s *priv;
  FAR struct sc8551_key_state_s sc8551_key_state;
  int ret;
  void *ioepattach;

  /* Initialize the SC8551 device structure */

  priv = kmm_zalloc(sizeof(struct sc8551_dev_s));
  if (priv)
    {
      /* Initialize the SC8551 device structure */

      priv->dev.ops   = &g_sc8551ops;
      priv->i2c       = i2c;
      priv->pin       = pin;
      priv->addr      = addr;
      priv->current   = current;
      priv->frequency = frequency;
      priv->ioedev    = dev;

      /* Interrupt register */

      ret = IOEXP_SETDIRECTION(priv->ioedev, priv->pin,
                               IOEXPANDER_DIRECTION_IN_PULLUP);
      if (ret < 0)
        {
          baterr("Failed to set direction: %d\n", ret);
        }

      ioepattach = IOEP_ATTACH(priv->ioedev, priv->pin,
                              sc8551_interrupt_handler, priv);
      if (ioepattach == NULL)
        {
          baterr("Failed to attach sc8551_interrupt_handler");
          ret = -EIO;
        }

      ret = IOEXP_SETOPTION(priv->ioedev, priv->pin,
                        IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_DISABLE);
      if (ret < 0)
        {
          baterr("Failed to set option: %d\n", ret);
          IOEP_DETACH(priv->ioedev, sc8551_interrupt_handler);
        }

      /* Reset the SC8551 */

      ret = sc8551_reset(priv);
      if (ret < 0)
        {
          baterr("ERROR: Failed to reset the SC8551: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Disable watchdog otherwise SC8551 returns to StandAlone mode */

      ret = sc8551_watchdog(priv, true);
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

      ret = sc8551_set_ss_timeout(priv, 5120);
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

  /* Get key states of sc8551 */

  memset(&sc8551_key_state, 0, sizeof(sc8551_key_state));
  ret = sc8551_get_key_state(priv, &sc8551_key_state);
  if (ret < 0)
    {
      baterr("ERROR: Failed to sc8551_get_key_state: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  return (FAR struct battery_charger_dev_s *)priv;
}

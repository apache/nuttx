/****************************************************************************
 * drivers/power/battery/bq25618.c
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

/* Lower half driver for BQ25618 battery charger
 *
 * The BQ25618 are Li-Ion Battery Charger with Power-Path Management.
 * It can be configured to Input Current Limit up to 1.5A.
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
#include "bq25618_reg.h"

/* This driver requires:
 *
 * CONFIG_BATTERY_CHARGER - Upper half battery driver support
 * CONFIG_I2C - I2C support
 * CONFIG_I2C_BQ25618 - And the driver must be explicitly selected.
 */

#if defined(CONFIG_BATTERY_CHARGER) && defined(CONFIG_I2C) && \
    defined(CONFIG_I2C_BQ25618)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARRAY_SIZE(a)  (sizeof(a)/sizeof(a[0]))

/****************************************************************************
 * Private
 ****************************************************************************/

struct bq25618_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_charger_dev_s dev; /* Battery charger device */

  /* Data fields specific to the lower half BQ25618 driver follow */

  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  uint32_t frequency;           /* I2C frequency */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C support */

static int bq25618_getreg8(FAR struct bq25618_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval);
static int bq25618_putreg8(FAR struct bq25618_dev_s *priv, uint8_t regaddr,
                           uint8_t regval);

static inline int bq25618_getreport(FAR struct bq25618_dev_s *priv,
                                    uint8_t *report, uint8_t regaddr);
static inline int bq25618_reset(FAR struct bq25618_dev_s *priv);
static inline int bq25618_watchdog(FAR struct bq25618_dev_s *priv,
                                   bool enable);
static inline int bq25618_powersupply(FAR struct bq25618_dev_s *priv,
                                      int current);
static inline int bq25618_setvolt(FAR struct bq25618_dev_s *priv,
                                  int volts);
static inline int bq25618_setcurr(FAR struct bq25618_dev_s *priv,
                                  int current);

/* Battery driver lower half methods */

static int bq25618_state(FAR struct battery_charger_dev_s *dev,
                         FAR int *status);
static int bq25618_health(FAR struct battery_charger_dev_s *dev,
                          FAR int *health);
static int bq25618_online(FAR struct battery_charger_dev_s *dev,
                          FAR bool *status);
static int bq25618_voltage(FAR struct battery_charger_dev_s *dev,
                           int value);
static int bq25618_current(FAR struct battery_charger_dev_s *dev,
                           int value);
static int bq25618_input_current(FAR struct battery_charger_dev_s *dev,
                                 int value);
static int bq25618_operate(FAR struct battery_charger_dev_s *dev,
                           uintptr_t param);

/****************************************************************************
 * Private Data
 ****************************************************************************/

FAR struct bq25618_ic_state
{
    uint8_t vbus_stat;
    uint8_t chrg_stat;
    bool online;

    uint8_t wdt_fault;
    uint8_t bat_fault;
    uint8_t chrg_fault;
    uint8_t ntc_fault;
};

static const int bq25618_vbatreg_values[] =
{
    3504, 3600, 3696, 3800, 3904, 4000, 4100, 4150,
    4200
};

static const int bq25618_ichg_values[] =
{
    1290, 1360, 1430, 1500
};

static const struct battery_charger_operations_s g_bq25618ops =
{
  bq25618_state,
  bq25618_health,
  bq25618_online,
  bq25618_voltage,
  bq25618_current,
  bq25618_input_current,
  bq25618_operate,
};

#ifdef CONFIG_DEBUG_BQ25618
static int bq25618_dump_regs(FAR struct bq25618_dev_s *priv);
#  define bq25618_dump_regs(priv) bq25618_dump_regs(priv)
#else
#  define bq25618_dump_regs(priv)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bq25618_getreg8
 *
 * Description:
 *   Read a 8-bit value from a BQ25618 register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK
 *   REPEATED-START <I2C read address> ACK Data0 ACK Data1 NO-ACK STOP
 *
 ****************************************************************************/

static int bq25618_getreg8(FAR struct bq25618_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval)
{
  FAR struct i2c_master_s *dev = priv->i2c;
  struct i2c_msg_s msg[2];
  int err;

  msg[0].frequency = priv->frequency;
  msg[0].addr = priv->addr;
  msg[0].buffer = &regaddr;
  msg[0].length = 1;
  msg[0].flags = I2C_M_NOSTOP;

  msg[1].frequency = priv->frequency;
  msg[1].addr = priv->addr;
  msg[1].buffer = regval;
  msg[1].length = 1;
  msg[1].flags = I2C_M_READ;

  if ((err = I2C_TRANSFER(dev, msg, 2)) < OK)
    {
      baterr("ERROR: i2c transfer failed! err: %d\n", err);
      return err;
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_putreg8
 *
 * Description:
 *   Write a 8-bit value to a BQ25618 register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK Data0 ACK Data1 ACK STOP
 *
 ****************************************************************************/

static int bq25618_putreg8(FAR struct bq25618_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  batinfo("addr: %02x regval: %08x\n", regaddr, regval);

  /* Set up a 3 byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Write the register address followed by the data (no RESTART) */

  return i2c_write(priv->i2c, &config, buffer, 2);
}

/****************************************************************************
 * Name: bq25618_array_parse
 *
 * Description:
 *   return The position of the element in the array
 *
 ****************************************************************************/

static int bq25618_array_parse(int array_size, int val, const int array[])
{
  int i = 0;

  if (val < array[i])
      return i - 1;

  if (val >= array[array_size - 1])
      return array_size - 1;

  for (i = 1; i < array_size; i++)
    {
      if (val == array[i])
          return i;

      if (val > array[i - 1] && val < array[i])
        {
          if (val < array[i])
              return i - 1;
          else
              return i;
        }
    }

  return -EINVAL;
}

#ifdef CONFIG_DEBUG_BQ25618
static int (bq25618_dump_regs) (FAR struct bq25618_dev_s * priv)
{
  int ret;
  uint8_t value = 0;

  ret  = bq25618_getreg8(priv, BQ25618_INPUT_CURRENT_LIMIT, &value);
  batinfo("REG#0: 0x%08X\n", value);
  ret |= bq25618_getreg8(priv, BQ25618_CHARGER_CONTROL_0, &value);
  batinfo("REG#1: 0x%08X\n", value);
  ret |= bq25618_getreg8(priv, BQ25618_CHARGE_CURRENT_LIMIT, &value);
  batinfo("REG#2: 0x%08X\n", value);
  ret |= bq25618_getreg8(priv, BQ25618_PRECHG_AND_TERM_CURR_LIM, &value);
  batinfo("REG#3: 0x%08X\n", value);
  ret |= bq25618_getreg8(priv, BQ25618_BATTERY_VOLTAGE_LIMIT, &value);
  batinfo("REG#4: 0x%08X\n", value);
  ret |= bq25618_getreg8(priv, BQ25618_CHARGER_CONTROL_1, &value);
  batinfo("REG#5: 0x%08X\n", value);
  ret |= bq25618_getreg8(priv, BQ25618_CHARGER_CONTROL_2, &value);
  batinfo("REG#6: 0x%08X\n", value);
  ret |= bq25618_getreg8(priv, BQ25618_CHARGER_CONTROL_3, &value);
  batinfo("REG#7: 0x%08X\n", value);
  ret |= bq25618_getreg8(priv, BQ25618_CHARGER_STATUS_0, &value);
  batinfo("REG#8: 0x%08X\n", value);
  ret |= bq25618_getreg8(priv, BQ25618_CHARGER_STATUS_1, &value);
  batinfo("REG#9: 0x%08X\n", value);
  ret |= bq25618_getreg8(priv, BQ25618_CHARGER_STATUS_2, &value);
  batinfo("REG#A: 0x%08X\n", value);
  ret |= bq25618_getreg8(priv, BQ25618_PART_INFORMATION, &value);
  batinfo("REG#B: 0x%08X\n", value);
  ret |= bq25618_getreg8(priv, BQ25618_CHARGER_CONTROL_4, &value);
  batinfo("REG#C: 0x%08X\n", value);

  return ret;
}
#endif

/****************************************************************************
 * Name: bq25618_getreport
 *
 * Description:
 *   Read the BQ25618 Register #1 (status and fault)
 *
 ****************************************************************************/

static inline int bq25618_getreport(FAR struct bq25618_dev_s *priv,
                                    FAR uint8_t *report, uint8_t regaddr)
{
  uint8_t regval = 0;
  int ret;

  ret = bq25618_getreg8(priv, regaddr, &regval);
  if (ret == OK)
    {
      *report = regval;
    }

  return ret;
}

/****************************************************************************
 * Name: bq25618_get_ichg_curr
 *
 * Description:
 *   Return the bq25618 charge current
 *
 ****************************************************************************/

int bq25618_get_ichg_curr(FAR struct bq25618_dev_s *priv)
{
  int ret;
  uint8_t regval;
  uint8_t ichg_reg_code;

  ret = bq25618_getreg8(priv, BQ25618_CHARGE_CURRENT_LIMIT, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return ret;
    }

  ichg_reg_code = regval & BQ25618_CHG_CURRENT_MASK;

  if (ichg_reg_code < BQ25618_ICHG_THRESH)
      return ichg_reg_code * BQ25618_ICHG_STEP_UA;

  return bq25618_ichg_values[ichg_reg_code - BQ25618_ICHG_THRESH];
}

/****************************************************************************
 * Name: bq25618_get_input_curr_lim
 *
 * Description:
 *   Return the bq25618 input current limit
 *
 ****************************************************************************/

int bq25618_get_input_curr_lim(FAR struct bq25618_dev_s *priv)
{
  int ret;
  uint8_t regval;
  uint8_t iindpm_reg_code;

  ret = bq25618_getreg8(priv, BQ25618_INPUT_CURRENT_LIMIT, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return ret;
    }

  iindpm_reg_code = regval & BQ25618_INP_CURR_LIM_MASK;

  return (iindpm_reg_code * BQ25618_IINDPM_STEP_UA) +
                                          BQ25618_IINDPM_OFFSET_UA;
}

/****************************************************************************
 * Name: bq25618_get_chrg_volt
 *
 * Description:
 *   Return the bq25618 charge volt
 *
 ****************************************************************************/

int bq25618_get_chrg_volt(FAR struct bq25618_dev_s *priv)
{
  int ret;
  uint8_t regval;
  uint8_t vbatreg_reg_code;

  ret = bq25618_getreg8(priv, BQ25618_BATTERY_VOLTAGE_LIMIT, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return ret;
    }

  vbatreg_reg_code = (regval & BQ25618_VBATREG_MASK) >>
                                            BQ25618_VBATREG_BIT_SHIFT;

  if (vbatreg_reg_code > BQ25618_VBATREG_THRESH)
      return ((vbatreg_reg_code - BQ25618_VBATREG_THRESH) *
                  BQ25618_VBATREG_STEP_UV) + BQ25618_VBATREG_THRESH_UV;

  return bq25618_vbatreg_values[vbatreg_reg_code];
}

/****************************************************************************
 * Name: bq25618_get_iterm_curr
 *
 * Description:
 *   Return the bq25618 iterm current
 *
 ****************************************************************************/

int bq25618_get_iterm_curr(FAR struct bq25618_dev_s *priv)
{
  int ret;
  uint8_t regval;
  uint8_t iterm_reg_code;

  ret = bq25618_getreg8(priv, BQ25618_PRECHG_AND_TERM_CURR_LIM, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return ret;
    }

  iterm_reg_code = regval & BQ25618_ITERM_CURR_MASK;

  return (iterm_reg_code * BQ25618_ITERM_STEP_UA) +
                                        BQ25618_ITERM_CURR_MIN;
}

/****************************************************************************
 * Name: bq25618_get_prechrg_curr
 *
 * Description:
 *   Return the bq25618 pre charge current
 *
 ****************************************************************************/

int bq25618_get_prechrg_curr(FAR struct bq25618_dev_s *priv)
{
  int ret;
  uint8_t regval;
  uint8_t iprechg_reg_code;

  ret = bq25618_getreg8(priv, BQ25618_PRECHG_AND_TERM_CURR_LIM, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return ret;
    }

  iprechg_reg_code = (regval & BQ25618_PRE_CURR_MASK) >>
                                       BQ25618_PRE_CURR_SHIFT;

  return (iprechg_reg_code * BQ25618_PRE_CURR_STEP_UA) +
                                       BQ25618_PRE_CURR_MIN;
}

/****************************************************************************
 * Name: bq25618_get_input_volt_lim
 *
 * Description:
 *   Return the bq25618 input volt limit
 *
 ****************************************************************************/

int bq25618_get_input_volt_lim(FAR struct bq25618_dev_s *priv)
{
  int ret;
  uint8_t regval;
  uint8_t vindpm_reg_code;

  ret = bq25618_getreg8(priv, BQ25618_CHARGER_CONTROL_2, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return ret;
    }

  vindpm_reg_code = regval & BQ25618_VINDPM_MASK;

  return (vindpm_reg_code * BQ25618_VINDPM_STEP_UV) +
                                     BQ25618_VINDPM_OFFSET_UV;
}

/****************************************************************************
 * Name: bq25618_control_shipmode
 *
 * Description:
 *   Return the control state
 *
 ****************************************************************************/

int bq25618_control_shipmode(FAR struct bq25618_dev_s *priv, bool enable)
{
  int ret;
  int idx;
  uint8_t regval;

  ret = bq25618_getreg8(priv, BQ25618_CHARGER_CONTROL_3, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
    }

  /* true:enter ship mode; false:exit ship mode */

  idx = enable;
  regval &= ~(BQ25618_SHIPMODE_MASK);
  regval |= (idx << BQ25618_SHIPMODE_SHIFT);

  ret = bq25618_putreg8(priv, BQ25618_CHARGER_CONTROL_3, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ25618! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_control_charge
 *
 * Description:
 *   Return the control enable and disable charge state
 *
 ****************************************************************************/

int bq25618_control_charge(FAR struct bq25618_dev_s *priv, bool enable)
{
  int ret;
  int idx;
  uint8_t regval;

  ret = bq25618_getreg8(priv, BQ25618_CHARGER_CONTROL_0, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
    }

  /* true:enable charge; false:disable charge */

  idx = enable;
  regval &= ~(BQ25618_CONTROL_CHARGE_MASK);
  regval |= (idx << BQ25618_CONTROL_CHARGE_SHIFT);

  ret = bq25618_putreg8(priv, BQ25618_CHARGER_CONTROL_0, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ25618! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_get_state
 *
 * Description:
 *   Return the bq25618 ic state
 *
 ****************************************************************************/

static int bq25618_get_state(FAR struct bq25618_dev_s *priv,
                             FAR struct bq25618_ic_state *state)
{
  uint8_t charge_status_0;
  uint8_t charge_status_1;
  int ret;

  ret = bq25618_getreg8(priv, BQ25618_CHARGER_STATUS_0, &charge_status_0);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return -1;
    }

  ret = bq25618_getreg8(priv, BQ25618_CHARGER_STATUS_1, &charge_status_1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return -1;
    }

  state->vbus_stat = charge_status_0 & BQ25618_VBUS_STAT_MASK;
  state->chrg_stat = charge_status_0 & BQ25618_CHRG_STAT_MASK;
  state->online = charge_status_0 & BQ25618_PG_STAT_MASK;

  state->wdt_fault = charge_status_1 & BQ25618_WDT_FAULT_MASK;
  state->bat_fault = charge_status_1 & BQ25618_BAT_FAULT_MASK;
  state->chrg_fault = charge_status_1 & BQ25618_CHRG_FAULT_MASK;
  state->ntc_fault = charge_status_1 & BQ25618_NTC_FAULT_MASK;

  return OK;
}

/****************************************************************************
 * Name: bq25618_detect_device
 *
 * Description:
 *   Return the bq25618 device info
 *
 ****************************************************************************/

static int bq25618_detect_device(FAR struct bq25618_dev_s *priv)
{
  uint8_t val;
  uint8_t part_no;
  int ret;

  ret = bq25618_getreg8(priv, BQ25618_PART_INFORMATION, &val);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return -1;
    }

  part_no = (val & BQ25618_DEV_ID_MASK);
  part_no >>= BQ25618_DEV_ID_SHIFT;
  if (part_no == 0x05)
    {
      baterr("detect bq25618 ID success\n");
    }
  else
    {
      baterr("detect bq25618 ID fail\n");
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_set_vindpm
 *
 * Description:
 *   Set the voltage level to charger. Voltage value in mV.
 *
 ****************************************************************************/

static inline int bq25618_set_vindpm(FAR struct bq25618_dev_s *priv,
                                     int value)
{
  uint8_t regval;
  int idx;
  int ret;

  ret = bq25618_getreg8(priv, BQ25618_CHARGER_CONTROL_2, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return ret;
    }

  idx = (value - BQ25618_VINDPM_OFFSET_UV) /
                         BQ25618_VINDPM_STEP_UV;

  /* Clear previous voltage */

  regval &= ~(BQ25618_VINDPM_MASK);
  regval |= (idx << BQ25618_VINDPM_SHIFT);

  ret = bq25618_putreg8(priv, BQ25618_CHARGER_CONTROL_2, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ25618! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_set_prechrg_curr
 *
 * Description:
 *   Set battery pre charge current
 *
 ****************************************************************************/

static inline int bq25618_set_prechrg_curr(FAR struct bq25618_dev_s *priv,
                                           int current)
{
  uint8_t regval;
  int idx;
  int ret;

  ret = bq25618_getreg8(priv, BQ25618_PRECHG_AND_TERM_CURR_LIM, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return ret;
    }

  current = current - BQ25618_PRE_CURR_MIN;
  idx = current / 20;

  regval &= ~(BQ25618_PRE_CURR_MASK);
  regval |= (idx << BQ25618_PRE_CURR_SHIFT);

  ret = bq25618_putreg8(priv, BQ25618_PRECHG_AND_TERM_CURR_LIM, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ25618! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_set_iterm_curr
 *
 * Description:
 *   Set battery iterm charge current
 *
 ****************************************************************************/

static inline int bq25618_set_iterm_curr(FAR struct bq25618_dev_s *priv,
                                         int current)
{
  uint8_t regval;
  int idx;
  int ret;

  ret = bq25618_getreg8(priv, BQ25618_PRECHG_AND_TERM_CURR_LIM, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return ret;
    }

  current = current - BQ25618_ITERM_CURR_MIN;
  idx = current / 20;

  regval &= ~(BQ25618_ITERM_CURR_MASK);
  regval |= (idx << BQ25618_ITERM_CURR_SHIFT);

  ret = bq25618_putreg8(priv, BQ25618_PRECHG_AND_TERM_CURR_LIM, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ25618! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_reset
 *
 * Description:
 *   Reset the BQ25618
 *
 ****************************************************************************/

static inline int bq25618_reset(FAR struct bq25618_dev_s *priv)
{
  int ret;
  uint8_t regval;

  /* Read current register value */

  ret = bq25618_getreg8(priv, BQ25618_PART_INFORMATION, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return ret;
    }

  /* Send reset command */

  regval |= BQ25618_RESET;
  ret = bq25618_putreg8(priv, BQ25618_PART_INFORMATION, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ25618! Error = %d\n", ret);
      return ret;
    }

  /* Wait a little bit to clear registers */

  nxsig_usleep(500);

  /* There is a BUG in BQ25618 the RESET bit is always read as 1 */

  regval &= ~(BQ25618_RESET);
  ret = bq25618_putreg8(priv, BQ25618_PART_INFORMATION, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ25618! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_watchdog
 *
 * Description:
 *   Enable/Disable the BQ25618 watchdog
 *
 ****************************************************************************/

static inline int bq25618_watchdog(FAR struct bq25618_dev_s *priv,
                                   bool enable)
{
  int ret;
  uint8_t regval;

  ret = bq25618_getreg8(priv, BQ25618_CHARGER_CONTROL_1, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return ret;
    }

  if (enable)
    {
      regval |= BQ25618_WD_EN;
    }
  else
    {
      regval &= ~(BQ25618_WD_EN);
    }

  ret = bq25618_putreg8(priv, BQ25618_CHARGER_CONTROL_1, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ25618! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_state
 *
 * Description:
 *   Return the current battery state
 *
 ****************************************************************************/

static int bq25618_state(FAR struct battery_charger_dev_s *dev,
                         FAR int *status)
{
  FAR struct bq25618_dev_s *priv = (FAR struct bq25618_dev_s *)dev;
  FAR struct bq25618_ic_state state;
  int ret;

  ret = bq25618_get_state(priv, &state);
  if (ret < 0)
    {
      *status = BATTERY_UNKNOWN;
      return ret;
    }

  if (state.vbus_stat == BQ25618_VBUS_STAT_NO_INPUT ||
               state.vbus_stat == BQ25618_VBUS_STAT_USB_OTG)
    {
      *status = BATTERY_DISCHARGING;
    }

  else if (state.chrg_stat == BQ25618_CHRG_STAT_NOT_CHRGING)
    {
      *status = BATTERY_IDLE;
    }

  else if (state.chrg_stat == BQ25618_CHRG_STAT_CHRG_TERM)
    {
      *status = BATTERY_FULL;
    }

  else
      *status = BATTERY_CHARGING;

  return OK;
}

/****************************************************************************
 * Name: bq25618_health
 *
 * Description:
 *   Return the current battery health state
 *
 * Note: if more than one fault happened the user needs to call this ioctl
 * again to read a new fault, repeat until receive a BATTERY_HEALTH_GOOD.
 *
 ****************************************************************************/

static int bq25618_health(FAR struct battery_charger_dev_s *dev,
                          FAR int *health)
{
  FAR struct bq25618_dev_s *priv = (FAR struct bq25618_dev_s *)dev;
  FAR struct bq25618_ic_state state;
  int ret;

  ret = bq25618_get_state(priv, &state);
  if (ret < 0)
    {
      *health = BATTERY_HEALTH_UNKNOWN;
      return ret;
    }

  if (state.wdt_fault)
    {
      *health = BATTERY_HEALTH_WD_TMR_EXP;
    }

  else if (state.bat_fault)
    {
      *health = BATTERY_HEALTH_OVERVOLTAGE;
    }

  else
    {
      switch (state.chrg_fault)
        {
          case BQ25618_CHRG_FAULT_INPUT:
              *health =  BATTERY_HEALTH_UNSPEC_FAIL;
              break;
          case BQ25618_CHRG_FAULT_THERM:
              *health =  BATTERY_HEALTH_OVERHEAT;
              break;
          case BQ25618_CHRG_FAULT_CST_EXPIRE:
              *health =  BATTERY_HEALTH_SAFE_TMR_EXP;
              break;
          default:
              break;
        }

      switch (state.ntc_fault)
        {
          case BQ25618_NTC_FAULT_WARM:
          case BQ25618_NTC_FAULT_HOT:
              *health =  BATTERY_HEALTH_OVERHEAT;
              break;
          case BQ25618_NTC_FAULT_COOL:
          case BQ25618_NTC_FAULT_COLD:
              *health =  BATTERY_HEALTH_COLD;
              break;
          default:
              *health =  BATTERY_HEALTH_GOOD;
              break;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_online
 *
 * Description:
 *   Return true if the battery is online
 *
 ****************************************************************************/

static int bq25618_online(FAR struct battery_charger_dev_s *dev,
                          FAR bool *status)
{
  /* There is no concept of online/offline in this driver */

  *status = true;
  return OK;
}

/****************************************************************************
 * Name: bq25618_powersupply
 *
 * Description:
 *   Set the Power Supply Current Limit.
 *
 ****************************************************************************/

static inline int bq25618_powersupply(FAR struct bq25618_dev_s *priv,
                                      int current)
{
  uint8_t regval;
  int idx;
  int ret;

  switch (current)
  {
  case 100:
    idx = BQ25618_INP_CURR_LIM_100MA;
    break;

  case 200:
    idx = BQ25618_INP_CURR_LIM_200MA;
    break;

  case 500:
    idx = BQ25618_INP_CURR_LIM_500MA;
    break;

  case 800:
    idx = BQ25618_INP_CURR_LIM_800MA;
    break;

  case 1600:
    idx = BQ25618_INP_CURR_LIM_1600MA;
    break;

  case 2400:
    idx = BQ25618_INP_CURR_LIM_2400MA;
    break;

  default:
    baterr("ERROR: Current not supported, setting default to 100mA!\n");
    idx = BQ25618_INP_CURR_LIM_100MA;
    break;
  }

  /* Read current register */

  ret = bq25618_getreg8(priv, BQ25618_INPUT_CURRENT_LIMIT, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return ret;
    }

  /* Clear previous current and set new value */

  regval &= ~(BQ25618_INP_CURR_LIM_MASK);
  regval |= idx;

  ret = bq25618_putreg8(priv, BQ25618_INPUT_CURRENT_LIMIT, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ25618! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_setvolt
 *
 * Description:
 *   Set the voltage level to charge the battery. Voltage value in mV.
 *
 ****************************************************************************/

static inline int bq25618_setvolt(FAR struct bq25618_dev_s *priv, int volts)
{
  uint8_t regval;
  int idx;
  int ret;

  /* Verify if voltage is in the acceptable range */

  if (volts < BQ25618_VOLT_MIN || volts > BQ25618_VOLT_MAX)
    {
      baterr("ERROR: Voltage %d mV is out of range.\n", volts);
      return -EINVAL;
    }

  ret = bq25618_getreg8(priv, BQ25618_BATTERY_VOLTAGE_LIMIT, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return ret;
    }

  switch (volts)
  {
     case 4100:
        idx = BQ25618_BATTERY_VOLT_ILIM_4V1;
        break;
     case 4200:
        idx = BQ25618_BATTERY_VOLT_ILIM_4V2;
        break;
     case 4300:
        idx = BQ25618_BATTERY_VOLT_ILIM_4V3;
        break;
     case 4400:
        idx = BQ25618_BATTERY_VOLT_ILIM_4V4;
        break;
     case 4450:
        idx = BQ25618_BATTERY_VOLT_ILIM_4V45;
        break;
     case 4500:
        idx = BQ25618_BATTERY_VOLT_ILIM_4V5;
        break;
    default:
        baterr("ERROR:volt not supported, setting default to 4.2V!\n");
        idx = BQ25618_BATTERY_VOLT_ILIM_4V2;
        break;
  }

  /* Clear previous voltage */

  regval &= ~(BQ25618_VBATREG_MASK);
  regval |= idx;

  ret = bq25618_putreg8(priv, BQ25618_BATTERY_VOLTAGE_LIMIT, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ25618! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_setcurr
 *
 * Description:
 *   Set the current to charge the battery. Current value in mA.
 *
 ****************************************************************************/

static inline int bq25618_setcurr(FAR struct bq25618_dev_s *priv,
                                  int current)
{
  uint8_t regval;
  int idx;
  int ret;

  /* Verify if current is in the acceptable range */

  int array_size = ARRAY_SIZE(bq25618_ichg_values);

  if (current < BQ25618_CURR_MIN || current > BQ25618_CURR_MAX)
    {
      baterr("ERROR: Current %d mA is out of range.\n", current);
      return -EINVAL;
    }

  ret = bq25618_getreg8(priv, BQ25618_CHARGE_CURRENT_LIMIT, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ25618! Error = %d\n", ret);
      return ret;
    }

  if (current <= BQ25618_ICHG_THRESH_UA)
    idx = current / BQ25618_ICHG_STEP_UA;
  else
    idx = bq25618_array_parse(array_size, current,
      bq25618_ichg_values) + BQ25618_ICHG_THRESH;

  /* Clear previous current and set new value */

  regval &= ~(BQ25618_CHG_CURRENT_MASK);
  regval |= (idx << BQ25618_CHG_CURRENT_SHIFT);

  ret = bq25618_putreg8(priv, BQ25618_CHARGE_CURRENT_LIMIT, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ25618! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_voltage
 *
 * Description:
 *   Set battery charger voltage
 *
 ****************************************************************************/

static int bq25618_voltage(FAR struct battery_charger_dev_s *dev, int value)
{
  FAR struct bq25618_dev_s *priv = (FAR struct bq25618_dev_s *)dev;
  int ret;

  /* Set voltage to battery charger */

  ret = bq25618_setvolt(priv, value);
  if (ret < 0)
    {
      baterr("ERROR: Error setting voltage to BQ25618! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_current
 *
 * Description:
 *   Set the battery charger current rate for charging
 *
 ****************************************************************************/

static int bq25618_current(FAR struct battery_charger_dev_s *dev, int value)
{
  FAR struct bq25618_dev_s *priv = (FAR struct bq25618_dev_s *)dev;
  int ret;

  /* Set current to battery charger */

  ret = bq25618_setcurr(priv, value);
  if (ret < 0)
    {
      baterr("ERROR: Error setting current to BQ25618! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_input_current
 *
 * Description:
 *   Set the power-supply input current limit
 *
 ****************************************************************************/

static int bq25618_input_current(FAR struct battery_charger_dev_s *dev,
                                 int value)
{
  FAR struct bq25618_dev_s *priv = (FAR struct bq25618_dev_s *)dev;
  int ret;

  ret = bq25618_powersupply(priv, value);
  if (ret < 0)
    {
      baterr("ERROR: Failed to set BQ25618 power supply: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq25618_operate
 *
 * Description:
 *   Do miscellaneous battery ioctl()
 *
 ****************************************************************************/

static int bq25618_operate(FAR struct battery_charger_dev_s *dev,
                           uintptr_t param)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bq25618_init
 *
 * Description:
 *   init bq25618
 *
 ****************************************************************************/

static int bq25618_init(FAR struct bq25618_dev_s *priv, int current)
{
  int ret;

  /* set charger vin 4.5V */

  ret = bq25618_set_vindpm(priv, BQ25618_VINDPM_DEFAULT_UV);
  if (ret < 0)
    {
      baterr("ERROR: Failed to set BQ25618 set vindpm: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* 0x17:set input current 2400ma */

  ret = bq25618_powersupply(priv, current);
  if (ret < 0)
    {
      baterr("ERROR: Failed to set BQ25618 power supply: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* set charge current 340ma */

  ret = bq25618_setcurr(priv, BQ25618_ICHG_DEFAULT_UA);
  if (ret < 0)
    {
      baterr("ERROR: Failed to set BQ25618 current: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* set precurr 40ma */

  ret = bq25618_set_prechrg_curr(priv, BQ25618_PRE_CURR_DEFAULT_UA);
  if (ret < 0)
    {
      baterr("ERROR: Failed to set pre current: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* set itermcurr 60ma */

  ret = bq25618_set_iterm_curr(priv, BQ25618_ITERM_DEFAULT_UA);
  if (ret < 0)
    {
      baterr("ERROR: Failed to set iterm current: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* set vbat 4.4V */

  ret = bq25618_setvolt(priv, BQ25618_VBAT_DEFAULT_UV);
  if (ret < 0)
    {
      baterr("ERROR: Failed to set BAT vol: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  bq25618_dump_regs(priv);
  baterr("bq25618 init success\n");

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bq25618_initialize
 *
 * Description:
 *   Initialize the BQ25618 battery driver and return an instance of the
 *   lower_half interface that may be used with battery_charger_register();
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY_CHARGER - Upper half battery driver support
 *   CONFIG_I2C - I2C support
 *   CONFIG_I2C_BQ25618 - And the driver must be explicitly selected.
 *
 * Input Parameters:
 *   i2c       - An instance of the I2C interface to use to communicate with
 *               the BQ25618
 *   addr      - The I2C address of the BQ25618 (Better be 0x6a)
 *   frequency - The I2C frequency
 *   current   - The input current our power-supply can offer to charger
 *
 * Returned Value:
 *   A pointer to the initialized lower-half driver instance.  A NULL pointer
 *   is returned on a failure to initialize the BQ25618 lower half.
 *
 ****************************************************************************/

FAR struct battery_charger_dev_s *
bq25618_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                   uint32_t frequency, int current)
{
  FAR struct bq25618_dev_s *priv;
  int ret;

  /* Initialize the BQ25618 device structure */

  priv = kmm_zalloc(sizeof(struct bq25618_dev_s));
  if (priv)
    {
      /* Initialize the BQ25618 device structure */

      priv->dev.ops   = &g_bq25618ops;
      priv->i2c       = i2c;
      priv->addr      = addr;
      priv->frequency = frequency;

      /* Reset the BQ25618 */

      ret = bq25618_reset(priv);
      if (ret < 0)
        {
          baterr("ERROR: Failed to reset the BQ25618: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Disable watchdog otherwise BQ25618 returns to StandAlone mode */

      ret = bq25618_watchdog(priv, false);
      if (ret < 0)
        {
          baterr("ERROR: Failed to disable BQ25618 watchdog: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      ret = bq25618_detect_device(priv);
      if (ret < 0)
        {
          baterr("ERROR: Failed to detect_device: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      ret = bq25618_init(priv, current);
      if (ret < 0)
        {
          baterr("ERROR: Failed to init BQ25618:%d\n", ret);
          kmm_free(priv);
          return NULL;
        }
    }

  return (FAR struct battery_charger_dev_s *)priv;
}

#endif /* CONFIG_BATTERY_CHARGER && CONFIG_I2C && CONFIG_I2C_BQ25618 */

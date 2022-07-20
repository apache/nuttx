/****************************************************************************
 * drivers/power/da9168.c
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

/* Lower half driver for DA9168 battery charger
 *
 * The DA9168 are Li-Ion Battery Charger with Power-Path Management.
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
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/wqueue.h>
#include "da9168.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Perform range check and exit with failure if out of range */

#define DA9168_IN_RANGE_OR_FAIL(val, min, max)  \
do {  \
     if ((val < min) || (val > max))  \
       {  \
         baterr( "'" #val "'" " out of range (%d - %d)\n", min, max);  \
         return ERROR;  \
       }  \
   } while(0)

/****************************************************************************
 * Convert a real world value to the register field representation.
 * This will round down to nearest step value if given choice
 *  isn't in step granularity.
 ****************************************************************************/

#define __DA9168_VAL_TO_FIELD(val, step, min, field, offset)  \
        (((((val - min) / step) + offset) << field##_SHIFT) & field##_MASK)

#define DA9168_VAL_TO_FIELD_OFFSET(val, step, min, field)  \
        __DA9168_VAL_TO_FIELD(val, step, min, field, field##_OFFSET)

#define DA9168_VAL_TO_FIELD(val, step, min, field)  \
        __DA9168_VAL_TO_FIELD(val, step, min, field, 0)

/* Convert field selector value to correct location in register */

#define DA9168_SEL_TO_FIELD(sel, field)  \
        ((sel << field##_SHIFT) & field##_MASK)

/****************************************************************************
 * Private
 ****************************************************************************/

struct da9168_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_charger_dev_s dev;  /* Battery charger device */

  /* Data fields specific to the lower half DA9168 driver follow */

  FAR struct i2c_master_s *i2c;      /* I2C interface */
  FAR struct ioexpander_dev_s *ioe;  /* Ioexpander device. */
  uint8_t addr;                      /* I2C address */
  uint32_t frequency;                /* I2C frequency */
  int pin;                           /* Interrupt pin */
  struct work_s work;                /* Interrupt handler worker */
  bool vindpm_int_flag;              /* vindpm interrupt flag */
  bool vbus_uv_int_flag;             /* vbus uv interrupt flag */
  uint8_t vindpm_int_count;          /* vindpm interrupt count */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C support */

static int da9168_getreg8(FAR struct da9168_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *regval);
static int da9168_putreg8(FAR struct da9168_dev_s *priv, uint8_t regaddr,
                          uint8_t regval);
static int da9168_reg_update_bits(FAR struct da9168_dev_s *priv,
                                  uint8_t regaddr,
                                  uint8_t mask, uint8_t regval_bits);
static inline int da9168_get_devid(FAR struct da9168_dev_s *priv,
                                   uint8_t *id);
static inline int da9168_getreport(FAR struct da9168_dev_s *priv,
                                   uint8_t *report, uint8_t regaddr);
static void da9168_get_state(FAR struct da9168_dev_s *priv,
                             FAR struct da9168_ic_state_s *state);
static inline int da9168_reset(FAR struct da9168_dev_s *priv);
static inline int da9168_watchdog(FAR struct da9168_dev_s *priv,
                                  bool enable);
static int da9168_control_charge(FAR struct da9168_dev_s *priv, bool enable);
static inline int da9168_set_chg_range(FAR struct da9168_dev_s *priv,
                                       bool chg_range_sel);
static int da9168_control_hiz(FAR struct da9168_dev_s *priv, bool enable);
static inline int da9168_powersupply(FAR struct da9168_dev_s *priv,
                                     int current);
static inline int da9168_setvolt(FAR struct da9168_dev_s *priv,
                                 int volts);
static inline int da9168_set_vbus_ovsel(FAR struct da9168_dev_s *priv,
                                        int value);
static inline int da9168_setcurr(FAR struct da9168_dev_s *priv,
                                 int current);
static inline int da9168_set_pre_curr(FAR struct da9168_dev_s *priv,
                                      int pre_current, bool range_pre_sel);
static inline int da9168_set_iterm_curr(FAR struct da9168_dev_s *priv,
                                    int term_current, bool range_term_sel);
static inline int da9168_set_recharge_level(FAR struct da9168_dev_s *priv,
                                            bool  rchg_voltage_sel);
static inline int da9168_enable_interrput(FAR struct da9168_dev_s *priv,
                                          uint8_t regaddr, uint8_t mask,
                                          uint8_t regval_bits,
                                          bool int_mask_sel);
static int da9168_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                    ioe_pinset_t pinset, FAR void *arg);
static int da9168_config_shipmode(FAR struct da9168_dev_s *priv,
                       enum da9168_ship_mode_entry_delay_sel entry_delay_sel,
                       enum da9168_ship_mode_exit_deb_sel exit_deb_sel);
static int da9168_enable_shipmode(FAR struct da9168_dev_s *priv);
static int da9168_control_rev_vbus(FAR struct da9168_dev_s *priv,
                                   bool enable);
static int da9168_control_boost_en(FAR struct da9168_dev_s *priv,
                                   bool enable);
static inline int da9168_control_pre_charge_safe_timer(
                  FAR struct da9168_dev_s *priv, uint8_t value);

/* Battery driver lower half methods */

static int da9168_state(FAR struct battery_charger_dev_s *dev,
                        FAR int *status);
static int da9168_health(FAR struct battery_charger_dev_s *dev,
                         FAR int *health);
static int da9168_online(FAR struct battery_charger_dev_s *dev,
                         FAR bool *status);
static int da9168_voltage(FAR struct battery_charger_dev_s *dev,
                          int value);
static int da9168_current(FAR struct battery_charger_dev_s *dev,
                          int value);
static int da9168_input_current(FAR struct battery_charger_dev_s *dev,
                                int value);
static int da9168_operate(FAR struct battery_charger_dev_s *dev,
                          uintptr_t param);
static int da9168_get_protocol(FAR struct battery_charger_dev_s *dev,
                           int *value);
static int da9168_chipid(FAR struct battery_charger_dev_s *dev,
                           unsigned int *value);
static int da9168_get_voltage(FAR struct battery_charger_dev_s *dev,
                                int *value);
static int da9168_voltage_info(FAR struct battery_charger_dev_s *dev,
                                int *value);
static void da9168_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_charger_operations_s g_da9168ops =
{
  da9168_state,
  da9168_health,
  da9168_online,
  da9168_voltage,
  da9168_current,
  da9168_input_current,
  da9168_operate,
  da9168_chipid,
  da9168_get_voltage,
  da9168_voltage_info,
  da9168_get_protocol,
};

#ifdef CONFIG_DEBUG_DA9168
static int da9168_dump_regs(FAR struct da9168_dev_s *priv);
#  define da9168_dump_regs(priv) da9168_dump_regs(priv)
#else
#  define da9168_dump_regs(priv)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: da9168_getreg8
 *
 * Description:
 *   Read a 8-bit value from a DA9168 register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK
 *   REPEATED-START <I2C read address> ACK Data0 ACK Data1 NO-ACK STOP
 *
 ****************************************************************************/

static int da9168_getreg8(FAR struct da9168_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *regval)
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
  msg[1].buffer = regval;
  msg[1].length = 1;
  msg[1].flags = I2C_M_READ;

  for (retries = 0; retries < DA_IIC_RETRY_NUM; retries++)
    {
      err = I2C_TRANSFER(dev, msg, 2);
      if (err >= 0)
        {
          break;
        }
      else
        {
          nxsig_usleep(1);
          baterr("i2c transf err:%d retry:%d\n", err, retries);
        }
    }

  return (err >= 0) ? OK : err;
}

/****************************************************************************
 * Name: da9168_putreg8
 *
 * Description:
 *   Write a 8-bit value to a DA9168 register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK Data0 ACK Data1 ACK STOP
 *
 ****************************************************************************/

static int da9168_putreg8(FAR struct da9168_dev_s *priv, uint8_t regaddr,
                          uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Set up a 3 byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Write the register address followed by the data (no RESTART) */

  return i2c_write(priv->i2c, &config, buffer, 2);
}

#ifdef CONFIG_DEBUG_DA9168

static int da9168_dump_regs (FAR struct da9168_dev_s * priv)
{
  int ret;
  uint8_t value ;

  ret = da9168_getreg8(priv, DA9168_PMC_STATUS_00, &value);
  batinfo("REG#0: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_STATUS_01, &value);
  batinfo("REG#1: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_STATUS_02, &value);
  batinfo("REG#2: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_STATUS_03, &value);
  batinfo("REG#3: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_STATUS_04, &value);
  batinfo("REG#4: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_EVENT_00, &value);
  batinfo("REG#5: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_EVENT_01, &value);
  batinfo("REG#6: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_EVENT_02, &value);
  batinfo("REG#7: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_EVENT_03, &value);
  batinfo("REG#8: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_EVENT_04, &value);
  batinfo("REG#9: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_MASK_00, &value);
  batinfo("REG#A: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_MASK_01, &value);
  batinfo("REG#B: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_MASK_02, &value);
  batinfo("REG#C: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_MASK_03, &value);
  batinfo("REG#D: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_MASK_04, &value);
  batinfo("REG#E: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_SYS_00, &value);
  batinfo("REG#F: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_SYS_01, &value);
  batinfo("REG#10: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_SYS_02, &value);
  batinfo("REG#11: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_SYS_03, &value);
  batinfo("REG#12: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_SYS_04, &value);
  batinfo("REG#13: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_SYS_05, &value);
  batinfo("REG#14: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_SYS_06, &value);
  batinfo("REG#15: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_CHG_00, &value);
  batinfo("REG#16: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_CHG_01, &value);
  batinfo("REG#17: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_CHG_02, &value);
  batinfo("REG#18: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_CHG_03, &value);
  batinfo("REG#19: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_CHG_04, &value);
  batinfo("REG#1a: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_CHG_05, &value);
  batinfo("REG#1b: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_CHG_06, &value);
  batinfo("REG#1c: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_LDO_00, &value);
  batinfo("REG#1d: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_LDO_01, &value);
  batinfo("REG#1e: 0x%08X\n", value);
  ret |= da9168_getreg8(priv, DA9168_PMC_LDO_02, &value);
  batinfo("REG#1f: 0x%08X\n", value);

  return ret;
}

#endif

/****************************************************************************
 * Name: da9168_reg_update_bits
 *
 * Description:
 *   update da9168 resgister value.
 *
 ****************************************************************************/

static int da9168_reg_update_bits(FAR struct da9168_dev_s *priv,
                                  uint8_t regaddr, uint8_t mask,
                                  uint8_t regval_bits)
{
  uint8_t regval;
  int ret ;

  ret = da9168_getreg8(priv, regaddr, &regval);
  if (ret < 0)
    {
      baterr("get %x reg err:%d\n", regaddr, ret);
      return ret;
    }

  regval &= ~mask;
  regval |= regval_bits & mask;
  ret = da9168_putreg8(priv, regaddr, regval);
  if (ret < 0)
    {
      baterr("set %x reg err:%d\n", regaddr, ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_get_devid
 *
 * Description:
 *   Read the ID register.
 *
 * Input Parameters:
 *   priv    - Device struct
 *   id      - Chip id
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static inline int da9168_get_devid(FAR struct da9168_dev_s *priv,
                                   FAR uint8_t *id)
{
  uint8_t regval ;
  int ret;

  ret = da9168_getreg8(priv, DA9168_OTP_DEVICE_ID, &regval);
  if (ret == OK)
    {
     *id = regval;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_getreport
 *
 * Description:
 *   Read the DA9168 Register #1 (status and fault)
 *
 ****************************************************************************/

static inline int da9168_getreport(FAR struct da9168_dev_s *priv,
                                   FAR uint8_t *report, uint8_t regaddr)
{
  uint8_t regval;
  int ret;

  ret = da9168_getreg8(priv, regaddr, &regval);
  if (ret >= 0)
    {
      *report = regval;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_enable_interrput
 *
 * Description:
 *   Da9168 enable interrupt event.
 *
 ****************************************************************************/

static inline int da9168_enable_interrput(FAR struct da9168_dev_s *priv,
                                          uint8_t regaddr, uint8_t mask,
                                          uint8_t regval_bits,
                                          bool int_mask_sel)
{
  int ret;

  ret = da9168_reg_update_bits(priv, regaddr, mask,
                               int_mask_sel << regval_bits);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_enable_shipmode
 *
 * Description:
 *   Make da9168 into shipmode
 *
 ****************************************************************************/

int da9168_enable_shipmode(FAR struct da9168_dev_s *priv)
{
  int ret;
  uint8_t regval;

  regval = DA9168_SHIP_MODE_MASK;
  ret = da9168_reg_update_bits(priv, DA9168_PMC_SYS_06,
                               DA9168_SHIP_MODE_MASK, regval);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: da9168_config_shipmode
 *
 * Description:
 *   Config da9168 into shipmode delay time and exit shipmode dealy time.
 *
 ****************************************************************************/

int da9168_config_shipmode(
                       FAR struct da9168_dev_s *priv,
                       enum da9168_ship_mode_entry_delay_sel entry_delay_sel,
                       enum da9168_ship_mode_exit_deb_sel exit_deb_sel)
{
  int ret;
  uint8_t regval;

  if ((entry_delay_sel < DA9168_SHIP_MODE_ENTRY_DELAY_2S) ||
      (entry_delay_sel > DA9168_SHIP_MODE_ENTRY_DELAY_10S))
    {
      baterr("shipmode delay err\n");
      return -EINVAL;
    }

  if ((exit_deb_sel < DA9168_SHIP_MODE_EXIT_DEB_20MS) ||
      (exit_deb_sel > DA9168_SHIP_MODE_EXIT_DEB_2S))
    {
      baterr("shipmode deb err\n");
      return -EINVAL;
    }

  regval = DA9168_SEL_TO_FIELD(entry_delay_sel, DA9168_SHIP_DLY);
  regval |= DA9168_SEL_TO_FIELD(exit_deb_sel, DA9168_RIN_N_SHIP_EXIT_TMR);

  ret = da9168_reg_update_bits(priv, DA9168_PMC_SYS_06,
                               DA9168_SHIP_DLY_MASK |
                               DA9168_RIN_N_SHIP_EXIT_TMR_MASK, regval);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: da9168_control_charge
 *
 * Description:
 *   Return the control enable and disable charge state
 *
 ****************************************************************************/

static int da9168_control_charge(FAR struct da9168_dev_s *priv, bool enable)
{
  int ret;

  ret = da9168_reg_update_bits(priv, DA9168_PMC_CHG_00, DA9168_CHG_EN_MASK,
                               (enable) ? DA9168_CHG_EN_MASK : 0);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_control_hiz
 *
 * Description:
 *   Control DA9168 into hiz mode or not
 *
 ****************************************************************************/

static int da9168_control_hiz(FAR struct da9168_dev_s *priv, bool enable)
{
  int ret;

  ret = da9168_reg_update_bits(priv, DA9168_PMC_SYS_06, DA9168_HIZ_MODE_MASK,
                               enable ? DA9168_HIZ_MODE_MASK : 0);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_control_rev_vbus
 *
 * Description:
 *   Enable DA9168 rev vbus
 *
 ****************************************************************************/

static int da9168_control_rev_vbus(FAR struct da9168_dev_s *priv,
                                   bool enable)
{
  int ret;

  ret = da9168_reg_update_bits(priv, DA9168_PMC_SYS_04,
        DA9168_REV_VBUS_EN_MASK, enable ? DA9168_REV_VBUS_EN_MASK : 0);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_control_boost_en
 *
 * Description:
 *   Enable DA9168 boost
 *
 ****************************************************************************/

static int da9168_control_boost_en(FAR struct da9168_dev_s *priv,
                                   bool enable)
{
  int ret;

  ret = da9168_reg_update_bits(priv, DA9168_PMC_SYS_04, DA9168_BOOST_EN_MASK,
                               enable ? DA9168_BOOST_EN_MASK : 0);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_chg_get_bat_fault
 *
 * Description:
 *   Get da9168 bat fault
 *
 ****************************************************************************/

enum da9168_chg_faults_e da9168_chg_get_bat_fault(
                                               FAR struct da9168_dev_s *priv)
{
  uint8_t status01;
  uint8_t status02;
  uint8_t status00;
  int ret;

  ret = da9168_getreg8(priv, DA9168_PMC_STATUS_01, &status01);
  if (ret < 0)
    {
      return DA9168_CHG_FAULT_UNKNOWN;
    }

  ret = da9168_getreg8(priv, DA9168_PMC_STATUS_02, &status02);
  if (ret < 0)
    {
      return DA9168_CHG_FAULT_UNKNOWN;
    }

  ret = da9168_getreg8(priv, DA9168_PMC_STATUS_00, &status00);
  if (ret < 0)
    {
      return DA9168_CHG_FAULT_UNKNOWN;
    }

  if (status00 & DA9168_S_VBAT_OC_MASK)
    {
      ret = DA9168_CHG_FAULT_VBAT_OC;
    }
  else if (status01 & DA9168_S_VBAT_OV_MASK)
    {
      ret = DA9168_CHG_FAULT_VBAT_OV;
    }
  else if (status01 & DA9168_S_VBAT_UV_MASK)
    {
      ret = DA9168_CHG_FAULT_VBAT_UV;
    }
  else if (status02 & DA9168_S_TS_HOT_MASK)
    {
      ret = DA9168_CHG_FAULT_TBAT_HOT;
    }
  else if (status02 & DA9168_S_TS_COLD_MASK)
    {
      ret = DA9168_CHG_FAULT_TBAT_COLD;
    }
  else if (status02 & DA9168_S_TS_WARM_MASK)
    {
      ret = DA9168_CHG_FAULT_TBAT_WARM;
    }
  else if (status02 & DA9168_S_TS_COOL_MASK)
    {
      ret = DA9168_CHG_FAULT_TBAT_COOL;
    }
  else
    {
      ret = DA9168_CHG_FAULT_NONE;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_chg_get_vsys_fault
 *
 * Description:
 *   Get da9168 vsys fault
 *
 ****************************************************************************/

enum da9168_chg_faults_e da9168_chg_get_vsys_fault(
                                            FAR struct da9168_dev_s *priv)
{
  uint8_t status01;
  uint8_t status04;
  int ret;

  ret = da9168_getreg8(priv, DA9168_PMC_STATUS_01, &status01);
  if (ret < 0)
    {
      return DA9168_CHG_FAULT_UNKNOWN;
    }

  ret = da9168_getreg8(priv, DA9168_PMC_STATUS_04, &status04);
  if (ret < 0)
    {
      return DA9168_CHG_FAULT_UNKNOWN;
    }

  if (status04 & DA9168_S_VSYS_SHUTDOWN_MASK)
    {
      ret = DA9168_CHG_FAULT_VSYS_SHUTDOWN;
    }
  else if (status01 & DA9168_S_VSYS_OV_MASK)
    {
      ret = DA9168_CHG_FAULT_VSYS_OV;
    }
  else if (status01 & DA9168_S_VSYS_UV_MASK)
    {
      ret = DA9168_CHG_FAULT_VSYS_UV;
    }
  else
    {
      ret = DA9168_CHG_FAULT_NONE;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_chg_get_timer_fault
 *
 * Description:
 *   Get da9168 timer fault
 *
 ****************************************************************************/

enum da9168_chg_faults_e da9168_chg_get_timer_fault(
                                               FAR struct da9168_dev_s *priv)
{
  uint8_t status02;
  uint8_t status03;
  int ret;

  ret = da9168_getreg8(priv, DA9168_PMC_STATUS_01, &status02);
  if (ret < 0)
    {
      return DA9168_CHG_FAULT_UNKNOWN;
    }

  ret = da9168_getreg8(priv, DA9168_PMC_STATUS_02, &status03);
  if (ret < 0)
    {
      return DA9168_CHG_FAULT_UNKNOWN;
    }

  if (status02 & DA9168_S_WD_TIMER_MASK)
    {
      ret = DA9168_CHG_FAULT_WD_TIMER;
    }
  else if (status03 & DA9168_S_CHG_TIMER_MASK)
    {
      ret = DA9168_CHG_FAULT_CHG_TIMER;
    }
  else
    {
      ret = DA9168_CHG_FAULT_NONE;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_chg_get_vbus_fault
 *
 * Description:
 *   Get da9168 vbus fault
 *
 ****************************************************************************/

enum da9168_chg_faults_e da9168_chg_get_vbus_fault(
                                              FAR struct da9168_dev_s *priv)
{
  uint8_t status00;
  uint8_t status01;
  int ret;

  ret = da9168_getreg8(priv, DA9168_PMC_STATUS_00, &status00);
  if (ret < 0)
    {
      return DA9168_CHG_FAULT_UNKNOWN;
    }

  ret = da9168_getreg8(priv, DA9168_PMC_STATUS_01, &status01);
  if (ret < 0)
    {
      return DA9168_CHG_FAULT_UNKNOWN;
    }

  if (status01 & DA9168_S_VBUS_OV_MASK)
    {
      ret = DA9168_CHG_FAULT_VBUS_OV;
    }
  else if (status01 & DA9168_S_VBUS_UV_MASK)
    {
      ret = DA9168_CHG_FAULT_VBUS_UV;
    }
  else if (status00 & DA9168_S_VBUS_VINDPM_MASK)
    {
      ret = DA9168_CHG_FAULT_VBUS_DPM;
    }
  else if (status00 & DA9168_S_VBUS_IINDPM_MASK)
    {
      ret = DA9168_CHG_FAULT_IBUS_DPM;
    }
  else
    {
      ret = DA9168_CHG_FAULT_NONE;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_chg_get_vbus_fault
 *
 * Description:
 *   Get da9168 vbus fault
 *
 ****************************************************************************/

enum da9168_chg_phases_e da9168_chg_get_phase(FAR struct da9168_dev_s *priv)
{
  uint8_t status03;
  int ret;

  ret = da9168_getreg8(priv, DA9168_PMC_STATUS_03, &status03);
  if (ret < 0)
    {
      return DA9168_CHG_PHASE_UNKNOWN;
    }

  if (status03 & DA9168_S_CHG_TRICKLE_MASK)
    {
      ret = DA9168_CHG_PHASE_TRICKLE;
    }
  else if (status03 & DA9168_S_CHG_PRE_MASK)
    {
      ret = DA9168_CHG_PHASE_PRE;
    }
  else if (status03 & DA9168_S_CHG_CC_MASK)
    {
      ret = DA9168_CHG_PHASE_CC;
    }
  else if (status03 & DA9168_S_CHG_CV_MASK)
    {
      ret = DA9168_CHG_PHASE_CV;
    }
  else if (status03 & DA9168_S_CHG_DONE_MASK)
    {
      ret = DA9168_CHG_PHASE_FULL;
    }
  else
    {
      /* If it's not one of the states above then we're not charging */

      ret = DA9168_CHG_PHASE_OFF;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_get_state
 *
 * Description:
 *   Return the da9168 ic state
 *
 ****************************************************************************/

static void da9168_get_state(FAR struct da9168_dev_s *priv,
                             FAR struct da9168_ic_state_s *state)
{
  state->vsys_fault = da9168_chg_get_vsys_fault(priv);
  state->bat_fault = da9168_chg_get_bat_fault(priv);
  state->timer_fault = da9168_chg_get_timer_fault(priv);
  state->vbus_fault = da9168_chg_get_vbus_fault(priv);
  state->chg_stat = da9168_chg_get_phase(priv);
}

/****************************************************************************
 * Name: da9168_detect_device
 *
 * Description:
 *   Return the da9168 device info
 *
 ****************************************************************************/

static int da9168_detect_device(FAR struct da9168_dev_s *priv)
{
  uint8_t regval;
  int ret;

  ret = da9168_getreg8(priv, DA9168_OTP_DEVICE_ID, &regval);
  if (ret < 0)
    {
      baterr("reading otp devid err:%d\n", ret);
      return ret;
    }

  if (regval != DA9168_DEV_ID)
    {
      baterr("detect da9168 ID fail\n");
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_reset
 *
 * Description:
 *   Reset the da9168
 *
 ****************************************************************************/

static inline int da9168_reset(FAR struct da9168_dev_s *priv)
{
  int ret;
  uint8_t regval;

  /* Read current register value */

  ret = da9168_getreg8(priv, DA9168_PMC_SYS_03, &regval);
  if (ret < 0)
    {
      return ret;
    }

  /* Send reset command */

  regval |= DA9168_RST_REG_MASK;
  ret = da9168_putreg8(priv, DA9168_PMC_SYS_03, regval);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_watchdog
 *
 * Description:
 *   Enable/Disable the DA9168 watchdog
 *
 ****************************************************************************/

static inline int da9168_watchdog(FAR struct da9168_dev_s *priv,
                                  bool enable)
{
  int ret;
  uint8_t regval;

  ret = da9168_getreg8(priv, DA9168_PMC_SYS_03, &regval);
  if (ret < 0)
    {
      return ret;
    }

  if (enable)
    {
      regval |= DA9168_WD_EN_MASK;
    }
  else
    {
      regval &= ~(DA9168_WD_EN_MASK);
    }

  ret = da9168_putreg8(priv, DA9168_PMC_SYS_03, regval);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_state
 *
 * Description:
 *   Return the current battery state
 *
 ****************************************************************************/

static int da9168_state(FAR struct battery_charger_dev_s *dev,
                        FAR int *status)
{
  FAR struct da9168_dev_s *priv = (FAR struct da9168_dev_s *)dev;
  FAR struct da9168_ic_state_s state;

  da9168_get_state(priv, &state);
  if (state.chg_stat == DA9168_CHG_PHASE_OFF)
    {
      *status = BATTERY_IDLE;
    }
  else if (state.chg_stat ==  DA9168_CHG_PHASE_FULL)
    {
      *status = BATTERY_FULL;
    }
  else if (state.chg_stat ==  DA9168_CHG_PHASE_TRICKLE ||
           state.chg_stat ==  DA9168_CHG_PHASE_PRE ||
           state.chg_stat ==  DA9168_CHG_PHASE_CC ||
           state.chg_stat ==  DA9168_CHG_PHASE_CV)
    {
       *status = BATTERY_CHARGING;
    }
  else if (state.vsys_fault || state.vbus_fault || state.bat_fault ||
      state.timer_fault)
    {
      *status = BATTERY_FAULT;
    }
  else
    {
      *status = BATTERY_UNKNOWN;
    }

  return OK;
}

/****************************************************************************
 * Name: da9168_health
 *
 * Description:
 *   Return the current battery health state
 *
 * Note: if more than one fault happened the user needs to call this ioctl
 * again to read a new fault, repeat until receive a BATTERY_HEALTH_GOOD.
 *
 ****************************************************************************/

static int da9168_health(FAR struct battery_charger_dev_s *dev,
                         FAR int *health)
{
  FAR struct da9168_dev_s *priv = (FAR struct da9168_dev_s *)dev;
  FAR struct da9168_ic_state_s state;

  da9168_get_state(priv, &state);
  if (state.timer_fault)
    {
      switch (state.timer_fault)
        {
          case DA9168_CHG_FAULT_CHG_TIMER:
              *health =  BATTERY_HEALTH_SAFE_TMR_EXP;
              break;
          case DA9168_CHG_FAULT_WD_TIMER:
              *health = BATTERY_HEALTH_WD_TMR_EXP;
              break;
          default:
              break;
        }
    }
  else if (state.bat_fault)
    {
       switch (state.bat_fault)
        {
          case DA9168_CHG_FAULT_VBAT_OC:
              *health =  BATTERY_HEALTH_OVERCURRENT;
              break;
          case DA9168_CHG_FAULT_VBAT_OV:
              *health = BATTERY_HEALTH_UNDERVOLTAGE;
              break;
          case DA9168_CHG_FAULT_VBAT_UV:
              *health = BATTERY_HEALTH_OVERVOLTAGE;
              break;
          case DA9168_CHG_FAULT_TBAT_HOT:
              *health = BATTERY_HEALTH_UNDERVOLTAGE;
              break;
          case DA9168_CHG_FAULT_TBAT_COLD:
              *health = BATTERY_HEALTH_COLD;
              break;
          default:
              break;
        }
    }
  else if(state.vsys_fault || state.vbus_fault)
    {
      *health = BATTERY_HEALTH_UNSPEC_FAIL;
    }
  else
    {
      *health = BATTERY_HEALTH_UNKNOWN;
    }

  return OK;
}

/****************************************************************************
 * Name: da9168_worker
 *
 * Description:
 *   Handle the da9168 vindpm interrput.
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

static void da9168_worker(FAR void *arg)
{
  int ret;
  uint8_t regval;

  FAR struct da9168_dev_s *priv = arg;
  ret = da9168_getreg8(priv, DA9168_PMC_EVENT_00, &regval);
  if (ret < 0)
    {
      baterr("get event _00 reg err\n");
    }
  else
    {
      if ((regval&DA9168_E_VBUS_VINDPM_MASK) >> DA9168_E_VBUS_VINDPM_SHIFT)
        {
          priv->vindpm_int_count++;
          priv->vindpm_int_flag = true;
        }
    }

  ret = da9168_getreg8(priv, DA9168_PMC_EVENT_01, &regval);
  if (ret < 0)
    {
      baterr("get event _01 reg err\n");
    }
  else
    {
      if ((regval & DA9168_E_VBUS_UV_MASK) >> DA9168_E_VBUS_UV_SHIFT)
        {
          priv->vbus_uv_int_flag = true;
        }
    }
}

/****************************************************************************
 * Name: da9168_interrupt_handler
 *
 * Description:
 *   Handle the charger interrupt.
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

static int da9168_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                    ioe_pinset_t pinset, FAR void *arg)
{
  FAR struct da9168_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  work_queue(LPWORK, &priv->work, da9168_worker, priv, 0);

  return OK;
}

/****************************************************************************
 * Name: da9168_set_vindpm
 *
 * Description:
 *   Set the voltage level to charger. Voltage value in mV.
 *
 ****************************************************************************/

static inline int da9168_set_vindpm(FAR struct da9168_dev_s *priv, int value)
{
  uint8_t regval;
  int ret;

  DA9168_IN_RANGE_OR_FAIL(value, DA9168_PCFG_VBUS_DPM_MIN_MV,
                          DA9168_PCFG_VBUS_DPM_MAX_MV);

  regval = DA9168_VAL_TO_FIELD_OFFSET(value, DA9168_PCFG_VBUS_STEP_MV,
                                      DA9168_PCFG_VBUS_DPM_MIN_MV,
                                      DA9168_VINDPM);
  ret = da9168_reg_update_bits(priv, DA9168_PMC_SYS_00, DA9168_VINDPM_MASK,
                               regval);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_control_pre_charge_safe_timer
 *
 * Description:
 *   Set the pre_charge safe timer time
 *
 ****************************************************************************/

static inline int da9168_control_pre_charge_safe_timer(
                  FAR struct da9168_dev_s *priv, uint8_t value)
{
  int ret;

  ret = da9168_reg_update_bits(priv, DA9168_PMC_CHG_01,
        DA9168_CHG_TMR_PRE_MASK, (value << DA9168_CHG_TMR_PRE_SHIFT));
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_set_vbus_ovsel
 *
 * Description:
 *   Set the vbus overvoltage setting. Voltage value in mV.
 *
 ****************************************************************************/

static inline int da9168_set_vbus_ovsel(FAR struct da9168_dev_s *priv,
                                        int value)
{
  uint8_t regval;
  int ret;

  regval = DA9168_SEL_TO_FIELD(value, DA9168_VBUS_OVSEL);
  ret = da9168_reg_update_bits(priv, DA9168_PMC_SYS_06,
                                     DA9168_VBUS_OVSEL_MASK, regval);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_setvolt
 *
 * Description:
 *   Set the voltage level to charge the battery. Voltage value in mV.
 *
 ****************************************************************************/

static inline int da9168_setvolt(FAR struct da9168_dev_s *priv, int volts)
{
  uint8_t regval;
  int ret;

  /* Verify if voltage is in the acceptable range */

  DA9168_IN_RANGE_OR_FAIL(volts, DA9168_PCFG_CHG_VOLT_CV_MIN_MV,
                          DA9168_PCFG_CHG_VOLT_CV_MAX_MV);

  regval = DA9168_VAL_TO_FIELD_OFFSET(volts, DA9168_PCFG_CHG_VOLT_CV_STEP_MV,
                                      DA9168_PCFG_CHG_VOLT_CV_MIN_MV,
                                      DA9168_CHG_VBATREG);
  ret = da9168_putreg8(priv, DA9168_PMC_CHG_04, regval);
  if (ret < 0)
    {
      baterr("write pmc_chg_04 err:%d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_set_recharge_level
 *
 * Description:
 *   Set charger recharge threshold offset(mv)
 *
 ****************************************************************************/

static inline int da9168_set_recharge_level(FAR struct da9168_dev_s *priv,
                                            bool rchg_voltage_sel)
{
  int ret;

  ret = da9168_reg_update_bits(priv, DA9168_PMC_CHG_00,
                               DA9168_CHG_VRCHG_MASK,
                               (rchg_voltage_sel << DA9168_CHG_VRCHG_SHIFT));
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_set_chg_range
 *
 * Description:
 *   Set charging current range setting.
 *
 ****************************************************************************/

static inline int da9168_set_chg_range(FAR struct da9168_dev_s *priv,
                                       bool chg_range_sel)
{
  int ret;

  ret = da9168_reg_update_bits(priv, DA9168_PMC_CHG_03,
                               DA9168_CHG_RANGE_MASK,
                               (chg_range_sel ? DA9168_CHG_RANGE_MASK : 0));
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_powersupply
 *
 * Description:
 *   Set the Input Current Limit.
 *
 ****************************************************************************/

static inline int da9168_powersupply(FAR struct da9168_dev_s *priv,
                                     int current)
{
  int ret;
  uint8_t regval;

  if (current > DA9168_IBUS_LIM_MAX_MA)
    {
      current = DA9168_IBUS_LIM_MAX_MA;
    }

  regval = DA9168_VAL_TO_FIELD(current, DA9168_IBUS_LIM_STEP_MA,
                               DA9168_IBUS_LIM_MIN_MA, DA9168_IINDPM);

  ret = da9168_reg_update_bits(priv, DA9168_PMC_SYS_01, DA9168_IINDPM_MASK,
                               regval);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_set_pre_curr
 *
 * Description:
 *   Set battery pre charge current
 *
 ****************************************************************************/

static inline int da9168_set_pre_curr(FAR struct da9168_dev_s *priv,
                                      int pre_current, bool range_pre_sel)
{
  uint8_t regval;
  int ret;

  /* Range checking  pre_charging current */

  if (range_pre_sel == DA9168_PCFG_CHG_RANGE_STEPS_5MA)
    {
      if (pre_current >= DA9168_PCFG_CHG_CURR_PRE_LOW_OFFSET_MIN_MA)
        {
          DA9168_IN_RANGE_OR_FAIL(pre_current,
                         DA9168_PCFG_CHG_CURR_PRE_LOW_OFFSET_MIN_MA,
                         DA9168_PCFG_CHG_CURR_PRE_LOW_OFFSET_MAX_MA);
        }
      else
        {
          DA9168_IN_RANGE_OR_FAIL(pre_current,
                                  DA9168_PCFG_CHG_CURR_PRE_LOW_MIN_MA,
                                  DA9168_PCFG_CHG_CURR_PRE_LOW_MAX_MA);
        }
    }
  else
    {
      if (pre_current >= DA9168_PCFG_CHG_CURR_PRE_HIGH_OFFSET_MIN_MA)
        {
          DA9168_IN_RANGE_OR_FAIL(pre_current,
                         DA9168_PCFG_CHG_CURR_PRE_HIGH_OFFSET_MIN_MA,
                         DA9168_PCFG_CHG_CURR_PRE_HIGH_OFFSET_MAX_MA);
        }
      else
        {
          DA9168_IN_RANGE_OR_FAIL(pre_current,
                                  DA9168_PCFG_CHG_CURR_PRE_HIGH_MIN_MA,
                                  DA9168_PCFG_CHG_CURR_PRE_HIGH_MAX_MA);
        }
    }

  ret = da9168_getreg8(priv, DA9168_PMC_CHG_02, &regval);
  if (ret < 0)
    {
      baterr("read pmc_chg_02 err:%d\n", ret);
      return ret;
    }

  regval &= ~(DA9168_CHG_RANGE_PRE_MASK | DA9168_CHG_IPRE_MASK);
  regval |= DA9168_SEL_TO_FIELD(range_pre_sel, DA9168_CHG_RANGE_PRE);
  if (range_pre_sel == DA9168_PCFG_CHG_RANGE_STEPS_5MA)
    {
      if (pre_current >= DA9168_PCFG_CHG_CURR_PRE_LOW_OFFSET_MIN_MA)
        {
          regval |= DA9168_CHG_IPRE_MSB_MASK;
          regval |= DA9168_VAL_TO_FIELD(pre_current,
                                 DA9168_PCFG_CHG_CURR_LOW_STEP_MA,
                                 DA9168_PCFG_CHG_CURR_PRE_LOW_OFFSET_MIN_MA,
                                 DA9168_CHG_IPRE);
        }
      else
        {
          regval |= DA9168_VAL_TO_FIELD_OFFSET(pre_current,
                                        DA9168_PCFG_CHG_CURR_LOW_STEP_MA,
                                        DA9168_PCFG_CHG_CURR_PRE_LOW_MIN_MA,
                                        DA9168_CHG_IPRE);
        }
    }
  else
    {
      if (pre_current >= DA9168_PCFG_CHG_CURR_PRE_HIGH_OFFSET_MIN_MA)
        {
          regval |= DA9168_CHG_IPRE_MSB_MASK;
          regval |= DA9168_VAL_TO_FIELD(pre_current,
                               DA9168_PCFG_CHG_CURR_HIGH_STEP_MA,
                               DA9168_PCFG_CHG_CURR_PRE_HIGH_OFFSET_MIN_MA,
                               DA9168_CHG_IPRE);
        }
      else
        {
          regval |= DA9168_VAL_TO_FIELD_OFFSET(pre_current,
                                        DA9168_PCFG_CHG_CURR_HIGH_STEP_MA,
                                        DA9168_PCFG_CHG_CURR_PRE_HIGH_MIN_MA,
                                        DA9168_CHG_IPRE);
        }
    }

  ret = da9168_putreg8(priv, DA9168_PMC_CHG_02, regval);
  if (ret < 0)
    {
      baterr("write pmc_chg_02 err:%d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_set_iterm_curr
 *
 * Description:
 *   Set battery iterm charge current
 *
 ****************************************************************************/

static inline int da9168_set_iterm_curr(FAR struct da9168_dev_s *priv,
                                      int term_current, bool range_term_sel)
{
  uint8_t regval;
  int ret;

  if (range_term_sel == DA9168_PCFG_CHG_RANGE_STEPS_5MA)
    {
      DA9168_IN_RANGE_OR_FAIL(term_current,
                              DA9168_PCFG_CHG_CURR_TERM_LOW_MIN_MA,
                              DA9168_PCFG_CHG_CURR_TERM_LOW_MAX_MA);
    }
  else
    {
      DA9168_IN_RANGE_OR_FAIL(term_current,
                              DA9168_PCFG_CHG_CURR_TERM_HIGH_MIN_MA,
                              DA9168_PCFG_CHG_CURR_TERM_HIGH_MAX_MA);
    }

  ret = da9168_getreg8(priv, DA9168_PMC_CHG_02, &regval);
  if (ret < 0)
    {
      baterr("read pmc_chg_02 err:%d\n", ret);
      return ret;
    }

  regval &= ~(DA9168_CHG_ITERM_MASK | DA9168_CHG_RANGE_TERM_MASK);

  /* Charge termination current */

  regval |= DA9168_SEL_TO_FIELD(range_term_sel, DA9168_CHG_RANGE_TERM);
  if (range_term_sel == DA9168_PCFG_CHG_RANGE_STEPS_5MA)
    {
      regval |= DA9168_VAL_TO_FIELD_OFFSET(term_current,
                                    DA9168_PCFG_CHG_CURR_LOW_STEP_MA,
                                    DA9168_PCFG_CHG_CURR_TERM_LOW_MIN_MA,
                                    DA9168_CHG_ITERM);
    }
  else
    {
      regval |= DA9168_VAL_TO_FIELD_OFFSET(term_current,
                                    DA9168_PCFG_CHG_CURR_HIGH_STEP_MA,
                                    DA9168_PCFG_CHG_CURR_TERM_HIGH_MIN_MA,
                                    DA9168_CHG_ITERM);
    }

  ret = da9168_putreg8(priv, DA9168_PMC_CHG_02, regval);
  if (ret < 0)
    {
      baterr("write pmc_chg_02 err:%d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_setcurr
 *
 * Description:
 *   Set the current to charge the battery. Current value in mA.
 *
 ****************************************************************************/

static inline int da9168_setcurr(FAR struct da9168_dev_s *priv,
                                 int current)
{
  uint8_t regval;
  int ret;
  bool range_cc_sel;

  ret = da9168_getreg8(priv, DA9168_PMC_CHG_03, &regval);
  if (ret < 0)
    {
      baterr("read pmc_chg_03 err:%d\n", ret);
      return ret;
    }
  else
    {
      if ((regval & 0x80) == 0x80)
        {
          range_cc_sel  = true;
        }
      else
        {
          range_cc_sel = false;
        }
    }

  if (range_cc_sel == DA9168_PCFG_CHG_RANGE_STEPS_5MA)
    {
      DA9168_IN_RANGE_OR_FAIL(current,
                              DA9168_PCFG_CHG_CURR_CC_LOW_MIN_MA,
                              DA9168_PCFG_CHG_CURR_CC_LOW_MAX_MA);
    }
  else
    {
      DA9168_IN_RANGE_OR_FAIL(current,
                              DA9168_PCFG_CHG_CURR_CC_HIGH_MIN_MA,
                              DA9168_PCFG_CHG_CURR_CC_HIGH_MAX_MA);
    }

  regval = 0;
  regval = DA9168_SEL_TO_FIELD(range_cc_sel, DA9168_CHG_RANGE);
  if (range_cc_sel == DA9168_PCFG_CHG_RANGE_STEPS_5MA)
    {
      regval |= DA9168_VAL_TO_FIELD_OFFSET(current,
                                         DA9168_PCFG_CHG_CURR_LOW_STEP_MA,
                                         DA9168_PCFG_CHG_CURR_CC_LOW_MIN_MA,
                                         DA9168_CHG_ICHG);
    }
  else
    {
      regval |= DA9168_VAL_TO_FIELD_OFFSET(current,
                                         DA9168_PCFG_CHG_CURR_HIGH_STEP_MA,
                                         DA9168_PCFG_CHG_CURR_CC_HIGH_MIN_MA,
                                         DA9168_CHG_ICHG);
    }

  ret = da9168_putreg8(priv, DA9168_PMC_CHG_03, regval);
  if (ret < 0)
    {
      baterr("write pmc_chg_03 err:%d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_get_protocol
 *
 * Description:
 *   Return true if the battery is online
 *
 ****************************************************************************/

static int da9168_get_protocol(FAR struct battery_charger_dev_s *dev,
                                                                int *value)
{
  int temp_val = 0;
  FAR struct da9168_dev_s *priv = (FAR struct da9168_dev_s *)dev;

  if (priv->vindpm_int_flag || priv->vbus_uv_int_flag)
    {
      temp_val |= DA9168_VINDPM_INT_FLAG;
      temp_val |= (priv->vindpm_int_count << 1);
      temp_val |= DA9168_VBUS_UV_INT_FLAG;
      priv->vindpm_int_flag = false;
      priv->vindpm_int_count = 0;
      priv->vbus_uv_int_flag = false;
    }
  else
    {
      temp_val = 0;
    }

  *value =  temp_val;
  return 0;
}

/****************************************************************************
 * Name: da9168_online
 *
 * Description:
 *   Return true if the battery is online
 *
 ****************************************************************************/

static int da9168_online(FAR struct battery_charger_dev_s *dev,
                          FAR bool *status)
{
  /* There is no concept of online/offline in this driver */

  *status = true;
  return OK;
}

static int da9168_chipid(FAR struct battery_charger_dev_s *dev,
                           unsigned int *value)
{
  return OK;
}

static int da9168_get_voltage(FAR struct battery_charger_dev_s *dev,
                                int *value)
{
  return OK;
}

static int da9168_voltage_info(FAR struct battery_charger_dev_s *dev,
                                int *value)
{
  return OK;
}

/****************************************************************************
 * Name: da9168_voltage
 *
 * Description:
 *   Set battery charger voltage
 *
 ****************************************************************************/

static int da9168_voltage(FAR struct battery_charger_dev_s *dev, int value)
{
  FAR struct da9168_dev_s *priv = (FAR struct da9168_dev_s *)dev;
  int ret;

  /* Set voltage to battery charger */

  ret = da9168_setvolt(priv, value);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: da9168_current
 *
 * Description:
 *   Set the battery charger current rate for charging
 *
 ****************************************************************************/

static int da9168_current(FAR struct battery_charger_dev_s *dev, int value)
{
  FAR struct da9168_dev_s *priv = (FAR struct da9168_dev_s *)dev;
  int ret;

  /* Set current to battery charger */

  ret = da9168_setcurr(priv, value);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: da9168_input_current
 *
 * Description:
 *   Set the input current limit
 *
 ****************************************************************************/

static int da9168_input_current(FAR struct battery_charger_dev_s *dev,
                                 int value)
{
  FAR struct da9168_dev_s *priv = (FAR struct da9168_dev_s *)dev;
  int ret;

  ret = da9168_powersupply(priv, value);
  if (ret < 0)
    {
      baterr("da9168 power supply err:%d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: da9168_operate
 *
 * Description:
 *   Do miscellaneous battery ioctl()
 *
 ****************************************************************************/

static int da9168_operate(FAR struct battery_charger_dev_s *dev,
                           uintptr_t param)
{
  FAR struct da9168_dev_s *priv = (FAR struct da9168_dev_s *)dev;
  FAR struct batio_operate_msg_s *msg =
                              (FAR struct batio_operate_msg_s *)param;
  int ret = OK;
  int value;

  value =  msg->u32;
  switch (msg->operate_type)
    {
      case BATIO_OPRTN_SHIPMODE:
        {
          ret = da9168_enable_shipmode(priv);
          if (ret < 0)
            {
              baterr("da9168 en shipmode err:%d\n", ret);
            }
        }

        break;

      case BATIO_OPRTN_CHARGE:
        {
          ret = da9168_control_charge(priv, value);
          if (ret < 0)
            {
              baterr("da9168 en charge err:%d\n", ret);
             }
        }

        break;

      case BATIO_OPRTN_BOOST:
        {
          batinfo("BATIO_OPRTN_BOOST\n");
          ret = da9168_control_rev_vbus(priv, value);
          ret |= da9168_control_boost_en(priv, value);
          if (ret < 0)
            {
              baterr("da9168 en boost:%d\n", ret);
            }
        }

      break;

      default:
        baterr("Unsupported otp:%d\n", msg->operate_type);
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: da9168_init
 *
 * Description:
 *   init da9168
 *
 ****************************************************************************/

static int da9168_init(FAR struct da9168_dev_s *priv, int current)
{
  int ret;

  /* set charging current range */

  ret = da9168_set_chg_range(priv, true);
  if (ret < 0)
    {
      baterr("da9168 set charge range err:%d\n", ret);
      return ret;
    }

  /* set charger vindpm 4700mV */

  ret = da9168_set_vindpm(priv, 4700);
  if (ret < 0)
    {
      baterr("da9168 set vindpm err:%d\n", ret);
      return ret;
    }

  /* set vbus overvoltage value 10500mV */

  ret =  da9168_set_vbus_ovsel(priv, 0x03);
  if (ret < 0)
    {
      baterr("da9168 set vbus ovsel er:%d\n", ret);
      return ret;
    }

  /* default set input current 2400ma */

  ret = da9168_powersupply(priv, current);
  if (ret < 0)
    {
      baterr("da9168 set input cur err:%d\n", ret);
      return ret;
    }

  /* set charge current 140ma */

  ret = da9168_setcurr(priv, DA9168_ICHG_DEFAULT_UA);
  if (ret < 0)
    {
      baterr("da9168 set charge cur err:%d\n", ret);
      return ret;
    }

  /* set precurr 20ma */

  ret = da9168_set_pre_curr(priv, DA9168_PRE_CURR_DEFAULT_UA, TRUE);
  if (ret < 0)
    {
      baterr("da9168 set pre cur err:%d\n", ret);
      return ret;
    }

  /* set iterm current 40ma */

  ret = da9168_set_iterm_curr(priv, 40, TRUE);
  if (ret < 0)
    {
      baterr("da9168 set iterm cur err:%d\n", ret);
      return ret;
    }

  /* set vbat 4440mV */

  ret = da9168_setvolt(priv, 4440);
  if (ret < 0)
    {
      baterr("da9168 set bat vol err:%d\n", ret);
      return ret;
    }

  /* set recharge threshold, 100mv */

  ret = da9168_set_recharge_level(priv, false);
  if (ret < 0)
    {
      baterr("da9168 set recharge level err:%d\n", ret);
      return ret;
    }

  /* enable vin_dpm interrput event */

  ret = da9168_enable_interrput(priv, DA9168_PMC_MASK_00,
                                DA9168_M_VBUS_VINDPM_MASK,
                                DA9168_M_VBUS_VINDPM_SHIFT, false);
  if (ret < 0)
    {
      baterr("da9168 en interrput err:%d\n", ret);
      return ret;
    }

  /* enable vus_uv interrput event */

  ret = da9168_enable_interrput(priv, DA9168_PMC_MASK_01,
                                DA9168_M_VBUS_UV_MASK,
                                DA9168_M_VBUS_UV_SHIFT, false);
  if (ret < 0)
    {
      baterr("da9168 en interrput err:%d\n", ret);
      return ret;
    }

  /* config shipmode parameter */

  da9168_config_shipmode(priv, DA9168_SHIP_MODE_ENTRY_DELAY_2S,
                         DA9168_SHIP_MODE_EXIT_DEB_2S);
  if (ret < 0)
    {
      baterr("da9168 shipmode param err:%d\n", ret);
      return ret;
    }

  /* enable rev vbus */

  ret = da9168_control_rev_vbus(priv, true);
  if (ret < 0)
    {
      baterr("da9168 control rev vubs err:%d\n", ret);
      return ret;
    }

  /* enable boost */

  ret = da9168_control_boost_en(priv, true);
  if (ret < 0)
    {
      baterr("da9168 control boost err:%d\n", ret);
      return ret;
    }

  /* set pre_charge safe timer 120 min */

  ret = da9168_control_pre_charge_safe_timer(priv, 0x03);
  if (ret < 0)
    {
      baterr("set pre charge safe timer err: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: da9168_initialize
 *
 * Description:
 *   Initialize the DA9168 battery driver and return an instance of the
 *   lower-half interface that may be used with battery_charger_register().
 *
 * Input Parameters:
 *   i2c       - An instance of the I2C interface to use to communicate with
 *               the DA9168
 *   dev    -  An instance of the ioexpander_dev_s.
 *   addr      - The I2C address of the DA9168 (Better be 0x68).
 *   frequency - The I2C frequency
 *   current   - The input current our power-supply can offer to charger
 *   int_pin   - The interrput pin
 *
 * Returned Value:
 *   A pointer to the initialized battery driver instance.  A NULL pointer
 *   is returned on a failure to initialize the DA9168 lower half.
 *
 ****************************************************************************/

FAR struct battery_charger_dev_s *
  da9168_initialize(FAR struct i2c_master_s *i2c,
                    FAR struct ioexpander_dev_s *dev, uint8_t addr,
                    uint32_t frequency, int current, int int_pin)
{
  FAR struct da9168_dev_s *priv;
  int *ioephanle;
  int ret;

  priv = kmm_zalloc(sizeof(struct da9168_dev_s));
  if (!priv)
    {
      baterr("allocate instance err\n");
      goto err;
    }

  /* Initialize the DA9168 device structure */

  priv->dev.ops   = &g_da9168ops;
  priv->i2c       = i2c;
  priv->addr      = addr;
  priv->frequency = frequency;
  priv->ioe       = dev;
  priv->pin       = int_pin;
  priv->vindpm_int_flag = false;
  priv->vbus_uv_int_flag = false;
  priv->vindpm_int_count = 0;

  /* Reset the DA9168 */

  ret = da9168_reset(priv);
  if (ret < 0)
    {
      baterr("da9168 reset err:%d\n", ret);
      goto err;
    }

  /* Detect deviceID */

  ret = da9168_detect_device(priv);
  if (ret < 0)
    {
      baterr("da9168 deviceid err:%d\n", ret);
      goto err;
    }

  /* Disable watchdog */

  ret = da9168_watchdog(priv, false);
  if (ret < 0)
    {
      baterr("da9168 watchdog err:%d\n", ret);
      goto err;
    }

  /* enable charge */

  ret = da9168_control_charge(priv, true);
  if (ret < 0)
    {
      baterr("da9168 control charge err:%d\n", ret);
      goto err;
    }

  /* enable hiz */

  ret = da9168_control_hiz(priv, false);
  if (ret < 0)
    {
      baterr("da9168 control hiz err:%d\n", ret);
      goto err;
    }

  /* Interrupt pin */

  ret = IOEXP_SETDIRECTION(priv->ioe, priv->pin,
                           IOEXPANDER_DIRECTION_IN);
  if (ret < 0)
    {
      baterr("set direction err:%d\n", ret);
      goto err;
    }

  ioephanle = IOEP_ATTACH(priv->ioe, priv->pin,
                          da9168_interrupt_handler, priv);
  if (!ioephanle)
    {
      baterr("attach err:%d\n", ret);
      ret = -EIO;
      goto err;
    }

  ret = IOEXP_SETOPTION(priv->ioe, priv->pin,
           IOEXPANDER_OPTION_INTCFG, (void *)IOEXPANDER_VAL_FALLING);
  if (ret < 0)
    {
      baterr("set option err:%d\n", ret);
      IOEP_DETACH(priv->ioe, da9168_interrupt_handler);
      goto err;
    }

  ret = da9168_init(priv, current);
  if (ret < 0)
    {
      baterr("da9168 init err:%d\n", ret);
      goto err;
    }

  return (FAR struct battery_charger_dev_s *)priv;

err:
  kmm_free(priv);
  return NULL;
}

/****************************************************************************
 * drivers/power/bq2429x.c
 * Lower half driver for BQ2429x battery charger
 *
 *   Copyright (C) 2017 Neil Hancock. All rights reserved.
 *
 *   Copyright (C) 2017 Haltian Ltd. All rights reserved.
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* The BQ24296M/BQ24296 are Li-Ion Battery management
 * with Power-Path Management
 * and USB OTG +5V boost.
 *
 * BQ charger with OTG boost and not DSBGA listed below
 * BQ24190/24192 compared as similar
 * BQ24259
 * BQ24295
 * BQ24195 Boost2A/PMID  Chg4A5 SolarPwr
 * BQ24195L Boost1A/PMID Chg2A5 SolarPwr
 * BQ24296
 * BQ24296M  Achg=0.1-3A, Vout=4.55 to 5.0V Current Limit 1.5A
 * BQ24297
 * BQ24298
 * BQ25601 I2C - to 13.5 Vin
 * BQ25606 No I2C control
 * BQ25890H
 * BQ25892
 * BQ25895
 * BQ25896 I2C 14V
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
#include <nuttx/power/bq2429x.h>

/* This driver requires:
 *
 * CONFIG_BATTERY_CHARGER - Upper half battery driver support
 * CONFIG_I2C - I2C support
 * CONFIG_I2C_BQ2429X - And the driver must be explicitly selected.
 */

#if defined(CONFIG_BATTERY_CHARGER) && defined(CONFIG_I2C) && \
    defined(CONFIG_I2C_BQ2429X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_BQ2429X
#  define baterr  _err
#  define batdbg  _info
#  define batinfo _info
#else
#  define baterr  _none
#  define batdbg  _none
#  define batinfo _none
#endif

/****************************************************************************
 * Private
 ****************************************************************************/

struct bq2429x_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  FAR const struct battery_charger_operations_s *ops; /* Battery operations */
  sem_t batsem;                                       /* Enforce mutually exclusive access */

  /* Data fields specific to the lower half BQ2429X driver follow */

  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  uint32_t frequency;           /* I2C frequency */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C support */

static int bq2429x_getreg8(FAR struct bq2429x_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *val, int num);
static int bq2429x_putreg8(FAR struct bq2429x_dev_s *priv, uint8_t regaddr,
                           uint8_t regval);

static int bq2429x_reset(FAR struct bq2429x_dev_s *priv);
static int bq2429x_watchdog(FAR struct bq2429x_dev_s *priv, bool enable);
static int bq2429x_sysoff(FAR struct bq2429x_dev_s *priv);
static int bq2429x_en_term(FAR struct bq2429x_dev_s *priv, bool state);
static int bq2429x_en_hiz(FAR struct bq2429x_dev_s *priv, bool state);
static int bq2429x_en_stat(FAR struct bq2429x_dev_s *priv, bool state);
static int bq2429x_setboost_otg_config(FAR struct bq2429x_dev_s *priv,
                                       bool state);
static int bq2429x_powersupply(FAR struct bq2429x_dev_s *priv, int current);
static int bq2429x_setvolt(FAR struct bq2429x_dev_s *priv, int volts);
static inline int bq2429x_setcurr(FAR struct bq2429x_dev_s *priv,
                                  int req_current);

/* Battery driver lower half methods */

static int bq2429x_state(FAR struct battery_charger_dev_s *dev,
                         FAR int *status);
static int bq2429x_health(FAR struct battery_charger_dev_s *dev,
                          FAR int *health);
static int bq2429x_online(FAR struct battery_charger_dev_s *dev,
                          FAR bool *status);
static int bq2429x_voltage(FAR struct battery_charger_dev_s *dev,
                           int value);
static int bq2429x_current(FAR struct battery_charger_dev_s *dev,
                           int value);
static int bq2429x_input_current(FAR struct battery_charger_dev_s *dev,
                                 int value);
static int bq2429x_operate(FAR struct battery_charger_dev_s *dev,
                           uintptr_t param);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_charger_operations_s g_bq2429xops =
{
  bq2429x_state,
  bq2429x_health,
  bq2429x_online,
  bq2429x_voltage,
  bq2429x_current,
  bq2429x_input_current,
  bq2429x_operate,
};

#ifdef CONFIG_DEBUG_BQ2429X
static int bq2429x_dump_regs(FAR struct bq2429x_dev_s *priv);
#  define bq2429x_dump_regs(priv) bq2429x_dump_regs(priv)
#else
#  define bq2429x_dump_regs(priv)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bq2429x_getreg8
 *
 * Description:
 *   Read a 8-bit value from a BQ2429x register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK
 *   REPEATED-START <I2C read address> ACK Data0 ACK Data1 NO-ACK STOP
 *
 ****************************************************************************/

static int bq2429x_getreg8(FAR struct bq2429x_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval, int num_char)
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

  ret = i2c_read(priv->i2c, &config, regval, num_char);
  if (ret < 0)
    {
      baterr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2429x_putreg8
 *
 * Description:
 *   Write a 8-bit value to a BQ2429x register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK Data0 ACK Data1 ACK STOP
 *
 ****************************************************************************/

static int bq2429x_putreg8(FAR struct bq2429x_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  batdbg("addr: %02x regval: %08x\n", regaddr, regval);

  /* Set up a message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Write the register address followed by the data (no RESTART) */

  return i2c_write(priv->i2c, &config, buffer, 2);
}

#ifdef CONFIG_DEBUG_BQ2429X
static int (bq2429x_dump_regs) (FAR struct bq2429x_dev_s * priv)
{
  int ret;
  uint8_t value = 0;

  ret  = bq2429x_getreg8(priv, BQ2429X_REG00, &value, 1);
  batdbg("REG#0: 0x%08X\n", value);
  ret |= bq2429x_getreg8(priv, BQ2429X_REG01, &value, 1);
  batdbg("REG#1: 0x%08X\n", value);
  ret |= bq2429x_getreg8(priv, BQ2429X_REG02, &value, 1);
  batdbg("REG#2: 0x%08X\n", value);
  ret |= bq2429x_getreg8(priv, BQ2429X_REG03, &value, 1);
  batdbg("REG#3: 0x%08X\n", value);
  ret |= bq2429x_getreg8(priv, BQ2429X_REG04, &value, 1);
  batdbg("REG#4: 0x%08X\n", value);
  ret |= bq2429x_getreg8(priv, BQ2429X_REG05, &value, 1);
  batdbg("REG#5: 0x%08X\n", value);
  ret |= bq2429x_getreg8(priv, BQ2429X_REG06, &value, 1);
  batdbg("REG#6: 0x%08X\n", value);
  ret |= bq2429x_getreg8(priv, BQ2429X_REG07, &value, 1);
  batdbg("REG#7: 0x%08X\n", value);
  ret |= bq2429x_getreg8(priv, BQ2429X_REG08, &value, 1);
  batdbg("REG#8: 0x%08X\n", value);

  /* Not reading fault register. */

  ret |= bq2429x_getreg8(priv, BQ2429X_REG0A, &value, 1);
  batdbg("REG#10: 0x%08X\n", value);

  return ret;
}
#endif

/****************************************************************************
 * Name: bq2429x_reset
 *
 * Description:
 *   Reset the BQ2429x
 *
 ****************************************************************************/

static int bq2429x_reset(FAR struct bq2429x_dev_s *priv)
{
  int ret;
  uint8_t regval;

  /* Read current register value */

  ret = bq2429x_getreg8(priv, BQ2429X_REG01, &regval, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2429X! Error = %d\n", ret);
      return ret;
    }

  /* Send reset command */

  regval |= BQ2429XR1_REG_RESET;
  ret = bq2429x_putreg8(priv, BQ2429X_REG01, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2429X! Error = %d\n", ret);
      return ret;
    }

  /* Wait a little bit to clear registers */

  nxsig_usleep(500);

#if 0
  /* There is a BUG in BQ2429X the RESET bit is always read as 1 */

  regval &= ~(BQ2429X_RESET);
  ret = bq2429x_putreg8(priv, BQ2429X_REG01, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2429X! Error = %d\n", ret);
      return ret;
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: bq2429x_watchdog
 *
 * Description:
 *   Enable/Disable the BQ2429x watchdog
 *
 ****************************************************************************/

static int bq2429x_watchdog(FAR struct bq2429x_dev_s *priv, bool enable)
{
  int ret;
  uint8_t regval;

  ret = bq2429x_getreg8(priv, BQ2429X_REG05, &regval, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2429X! Error = %d\n", ret);
      return ret;
    }

  if (enable)
    {
      /* Hw Default is 40Sec so use that for time being */

      regval &= ~(BQ2429XR5_WATCHDOG_MASK);
      regval |= BQ2429XR5_WATCHDOG_040SEC;
    }
  else
    {
      /* 0 is disable */

      regval &= ~(BQ2429XR5_WATCHDOG_MASK);
    }

  ret = bq2429x_putreg8(priv, BQ2429X_REG05, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2429X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2429x_sysoff
 *
 * Description:
 *   Turn the internal battery FET off in order to reduce the leakage from
 *   the BAT pin. Note that this disconnects the battery from the system.
 *
 ****************************************************************************/

static int bq2429x_sysoff(FAR struct bq2429x_dev_s *priv)
{
  int ret;
  uint8_t value = 0;

  ret = bq2429x_getreg8(priv, BQ2429X_REG07, &value, 1);
  batdbg("REG7 read value: 0x%08X\n", value);
  value |= BQ2429XR7_BATFET_DISABLE;
  ret |= bq2429x_putreg8(priv, BQ2429X_REG07, value);

  return ret;
}

/****************************************************************************
 * Name: bq2429x_syson
 *
 * Description:
 *   Turn the internal battery FET on.
 *
 ****************************************************************************/

static int bq2429x_syson(FAR struct bq2429x_dev_s *priv)
{
  int ret;
  uint8_t value = 0;

  ret = bq2429x_getreg8(priv, BQ2429X_REG07, &value, 1);
  batdbg("REG7 read value: 0x%08X\n", value);
  value &= ~BQ2429XR7_BATFET_DISABLE;
  ret |= bq2429x_putreg8(priv, BQ2429X_REG07, value);

  return ret;
}

/****************************************************************************
 * Name: bq2429x_en_term
 *
 * Description:
 *   Enable charger termination.  When termination is disabled, there are no
 *   indications of the charger terminating (i.e. STAT pin or registers).
 *
 ****************************************************************************/

static int bq2429x_en_term(FAR struct bq2429x_dev_s *priv, bool state)
{
  uint8_t regval;
  int ret;

  ret = bq2429x_getreg8(priv, BQ2429X_REG05, &regval, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2429X REG5! Error = %d\n", ret);
      return ret;
    }

  batdbg("en_term: REG05 %02X EN_TERM=%d\n",
         regval, !!(regval & BQ2429XR5_EN_TERM));

  /* Clear previous and set new value */

  if (state)
    {
      regval |= BQ2429XR5_EN_TERM;
    }
  else
    {
      regval &= ~BQ2429XR5_EN_TERM;
    }

  ret = bq2429x_putreg8(priv, BQ2429X_REG05, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2429XR5! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2429x_en_hiz
 *
 * Description:
 *   Enable high-impedance mode. Sets the charger IC into low power standby
 *   mode.
 *
 ****************************************************************************/

static int bq2429x_en_hiz(FAR struct bq2429x_dev_s *priv, bool state)
{
  uint8_t regval;
  int ret;

  ret = bq2429x_getreg8(priv, BQ2429X_REG00, &regval, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2429X REG0! Error = %d\n", ret);
      return ret;
    }

  batdbg("en_hiz: REG00 %02X EN_HIZ=%d\n",
         regval, !!(regval & BQ2429XR1_EN_HIZ));

  /* Clear previous and set new value */

  if (state)
    {
      regval |= BQ2429XR1_EN_HIZ;
    }
  else
    {
      regval &= ~BQ2429XR1_EN_HIZ;
    }

  ret = bq2429x_putreg8(priv, BQ2429X_REG00, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2429XR0! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2429x_en_stat
 *
 * Description:
 *   Enable interrupts.
 *
 ****************************************************************************/

static int bq2429x_en_stat(FAR struct bq2429x_dev_s *priv, bool state)
{
  uint8_t regval;
  int ret;

  ret = bq2429x_getreg8(priv, BQ2429X_REG07, &regval, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2429X REG7! Error = %d\n", ret);
      return ret;
    }

  batdbg("int stat: REG07 %02X INT_MASK1=%d INT_MASK0=%d\n", regval,
         !!(regval & BQ2429XR7_INT_MASK1), !!(regval & BQ2429XR7_INT_MASK0));

  /* We always set or clear both interrupts together. */

  if (state)
    {
      regval |= (BQ2429XR7_INT_MASK0 | BQ2429XR7_INT_MASK1);
    }
  else
    {
      regval &= ~(BQ2429XR7_INT_MASK0 | BQ2429XR7_INT_MASK1);
    }

  ret = bq2429x_putreg8(priv, BQ2429X_REG07, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2429XR7! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2429x_setboost_otg_config
 *
 * Description:
 *   Set the device boost mode.
 *
 ****************************************************************************/

static int bq2429x_setboost_otg_config(FAR struct bq2429x_dev_s *priv,
                                       bool state)
{
  uint8_t regval;
  int ret;

  ret = bq2429x_getreg8(priv, BQ2429X_REG01, &regval, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2429X! Error = %d\n", ret);
      return ret;
    }

  /* Clear previous current and set new value */

  if (state)
    {
      /* Set to Boost disable Charge */

      regval = BQ2429XR1_OTG_CONFIG | (regval & ~BQ2429XR1_CHG_CONFIG);
    }
  else
    {
      /* Set to Charge disable Boost */

      regval = BQ2429XR1_CHG_CONFIG | (regval & ~BQ2429XR1_OTG_CONFIG);
    }

  ret = bq2429x_putreg8(priv, BQ2429X_REG01, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2429X! Error = %d\n", ret);
      return ret;
    }

#define BST_CONFIG_MASK (BQ2429XR1_CHG_CONFIG | BQ2429XR1_OTG_CONFIG)
  batdbg("otg_config: REG01 %02X Boost=%d\n", regval,
      ((BQ2429XR1_OTG_CONFIG == (regval & BST_CONFIG_MASK)) ? 1 : 0));

  return OK;
}

/****************************************************************************
 * Name: bq2429x_state
 *
 * Description:
 *   Return the current battery management state
 *
 ****************************************************************************/

static int bq2429x_state(FAR struct battery_charger_dev_s *dev,
                         FAR int *status)
{
  FAR struct bq2429x_dev_s *priv = (FAR struct bq2429x_dev_s *)dev;
  uint8_t regval;
  bool isfault = false;
  int ret;

  ret = bq2429x_getreg8(priv, BQ2429X_REG08, &regval, 1);
  if (ret < 0)
    {
      *status = BATTERY_UNKNOWN;
      return ret;
    }

  if (regval & BQ2429XR8_DPM_STAT)
    {
      isfault = true;
      batinfo("DPM detected!\n");
    }

  if (!(regval & BQ2429XR8_PG_STAT))
    {
      isfault = true;
      batinfo("Power is not good!\n");
    }

  if (regval & BQ2429XR8_THERM_STAT)
    {
      isfault = true;
      batinfo("Thermal regulation!\n");
    }

  if (regval & BQ2429XR8_VSYS_STAT)
    {
      isfault = true;
      batinfo("VSYSMIN regulation! Battery is too low!\n");
    }

  regval &= BQ2429XR8_CHRG_STAT_MASK;

  /* TODO: should we check REG09 faults here as well? */

  if (isfault)
    {
      *status = BATTERY_FAULT;
    }

  /* Is the charging done? */

  else if (regval == BQ2429XR8_CHRG_STAT_DONE)
    {
      *status = BATTERY_FULL;
    }

  /* Is the charging in progress? */

  else if (regval == BQ2429XR8_CHRG_STAT_PRECHG ||
           regval == BQ2429XR8_CHRG_STAT_FASTCHG)
    {
      *status = BATTERY_CHARGING;
    }

  /* Is the charging ready? */

  else if (regval == BQ2429XR8_CHRG_STAT_NONE)
    {
      *status = BATTERY_IDLE;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2429x_health
 *
 * Description:
 *   Return the current battery health state
 *
 * Note: if more than one fault happened the user needs to call this ioctl
 * again to read a new fault, repeat until receive a BATTERY_HEALTH_GOOD.
 *
 ****************************************************************************/

static int bq2429x_health(FAR struct battery_charger_dev_s *dev,
                          FAR int *health)
{
  FAR struct bq2429x_dev_s *priv = (FAR struct bq2429x_dev_s *)dev;
  uint8_t regval;
  int ret;

  ret = bq2429x_getreg8(priv, BQ2429X_REG09, &regval, 1);
  if (ret < 0)
    {
      *health = BATTERY_HEALTH_UNKNOWN;
      return ret;
    }

  switch (regval & BQ2429XR9_CHRG_FAULT_MASK)
    {
      case BQ2429XR9_CHRG_FAULT_TIMER:
        *health = BATTERY_HEALTH_SAFE_TMR_EXP;
        batinfo("battery safety timer expiration!\n");
        return OK;

      case BQ2429XR9_CHRG_FAULT_INPUT:
        *health = BATTERY_HEALTH_DISCONNECTED;
        batinfo("input disconnect/fault!\n");
        return OK;

      case BQ2429XR9_CHRG_FAULT_THERMAL:
        *health = BATTERY_HEALTH_OVERHEAT;
        batinfo("thermal shutdown!\n");
        return OK;

      case BQ2429XR9_CHRG_FAULT_NORMAL:
        *health = BATTERY_HEALTH_GOOD;
        break; /* No return, check for other faults */

      default:
        DEBUGASSERT(false);
        *health = BATTERY_HEALTH_UNKNOWN;
        return OK;
    }

  if (regval & BQ2429XR9_BAT_FAULT)
    {
      *health = BATTERY_HEALTH_OVERVOLTAGE;
      batinfo("battery OVP!\n");
    }
  else if (regval & BQ2429XR9_NTC_FAULT2_HOT)
    {
      *health = BATTERY_HEALTH_OVERHEAT;
      batinfo("NTC hot!\n");
    }
  else if (regval & BQ2429XR9_NTC_FAULT1_COLD)
    {
      *health = BATTERY_HEALTH_COLD;
      batinfo("NTC cold!\n");
    }
  else if (regval & BQ2429XR9_WATCHDOG_FAULT)
    {
      *health = BATTERY_HEALTH_WD_TMR_EXP;
      batinfo("watchdog expiration!\n");
    }
  else if (regval & BQ2429XR9_OTG_FAULT)
    {
      *health = BATTERY_HEALTH_UNSPEC_FAIL;
      batinfo("VBUS overload or OVP!\n");
    }

  return OK;
}

/****************************************************************************
 * Name: bq2429x_online
 *
 * Description:
 *   Return true if the battery is online
 *
 ****************************************************************************/

static int bq2429x_online(FAR struct battery_charger_dev_s *dev,
                          FAR bool *status)
{
  FAR struct bq2429x_dev_s *priv = (FAR struct bq2429x_dev_s *)dev;
  uint8_t regval;
  int ret;

  ret = bq2429x_getreg8(priv, BQ2429X_REG00, &regval, 1);
  if (ret < 0)
    {
      *status = false;
      return ret;
    }

  if (regval & BQ2429XR1_EN_HIZ)
    {
      /* Device is HIGH IMPEDANCE battery offline */

      *status = false;
    }
  else
    {
      *status = true;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2429x_powersupply
 *
 * Description:
 *   Set the Power Supply Current Limit.
 *
 ****************************************************************************/

static int bq2429x_powersupply(FAR struct bq2429x_dev_s *priv, int current)
{
  uint8_t regval;
  uint8_t idx;
  int ret;

  switch (current)
  {
  case 100:
    idx = BQ2429XR0_INLIM_0100MA;
    break;

  case 150:
    idx = BQ2429XR0_INLIM_0150MA;
    break;

  case 500:
    idx = BQ2429XR0_INLIM_0500MA;
    break;

  case 900:
    idx = BQ2429XR0_INLIM_0900MA;
    break;

  case 1000:
    idx = BQ2429XR0_INLIM_1000MA;
    break;

  case 1500:
    idx = BQ2429XR0_INLIM_1500MA;
    break;

  case 2000:
    idx = BQ2429XR0_INLIM_2000MA;
    break;

  case 3000:
    idx = BQ2429XR0_INLIM_3000MA;
    break;

  default:
    baterr("ERROR: Current not supported, setting default to 100mA.!\n");
    idx = BQ2429XR0_INLIM_0100MA;
    break;
  }

  /* Read current register */

  ret = bq2429x_getreg8(priv, BQ2429X_REG00, &regval, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2429X! Error = %d\n", ret);
      return ret;
    }

  /* Clear previous current and set new value */

  regval &= ~(BQ2429XR0_INLIM_MASK);
  regval |= idx;

  ret = bq2429x_putreg8(priv, BQ2429X_REG00, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2429X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2429x_setvolt
 *
 * Description:
 *   Set the voltage level to charge the battery. Voltage value in mV.
 *
 ****************************************************************************/

static int bq2429x_setvolt(FAR struct bq2429x_dev_s *priv, int req_volts)
{
  uint8_t regval;
  int idx;
  int ret;

  /* Verify if voltage is in the acceptable range */

  if (req_volts < BQ2429X_VOLTCHG_MIN || req_volts > BQ2429X_VOLTCHG_MAX)
    {
      baterr("ERROR: Voltage %d mV is out of range.\n", req_volts);
      return -EINVAL;
    }

  ret = bq2429x_getreg8(priv, BQ2429X_REG04, &regval, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2429X! Error = %d\n", ret);
      return ret;
    }

  /* Charge Voltage starts at _MIN and increases in steps of 16mV */

  idx = req_volts - BQ2429X_VOLTCHG_MIN;
  idx = idx / 16;

  /* Clear previous voltage */

  regval &= ~(BQ2429XR4_VREG_MASK);
  regval |= (idx << BQ2429XR4_VREG_SHIFT);

  ret = bq2429x_putreg8(priv, BQ2429X_REG04, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2429X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2429x_setcurr
 *
 * Description:
 *   Set the current to charge the battery. Current value in mA.
 *
 ****************************************************************************/

static inline int bq2429x_setcurr(FAR struct bq2429x_dev_s *priv,
                                  int req_current)
{
  uint8_t regval;
  bool force_20pct = false;
  int idx;
  int ret;

  /* If requested current is below the minimum for fast charge,
   * configure for trickle charging. Trickle charging uses 20%
   * of current programmed to ICHG bits, so we multiply by five.
   */

  if (req_current < BQ2429X_CURRCHG_MIN)
    {
      force_20pct = true;
      req_current *= 5;
    }

  /* Verify if current is in the acceptable range. */

  if (req_current < BQ2429X_CURRCHG_MIN || req_current > BQ2429X_CURRCHG_MAX)
    {
      baterr("ERROR: Current %d mA is out of range.\n",
             force_20pct ? req_current / 5 : req_current);
      return -EINVAL;
    }

  /* According to the "Termination when REG02[0] = 1" section of
   * the bq24296M datasheet, the trickle charge could be less than
   * the termination current so it is recommended to turn off the
   * termination function.
   *
   * This means that the user will have to manually turn off the
   * charging when in 20% mode. Otherwise there could be battery
   * damage if we continuously trickle charge full battery until
   * safety timer finally stops it after 10 hours (timer is running
   * in half speed in 20% mode.)
   */

  ret = bq2429x_en_term(priv, !force_20pct);
  if (ret < 0)
    {
      return ret;
    }

  /* Read previous current and charge type. */

  ret = bq2429x_getreg8(priv, BQ2429X_REG02, &regval, 1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2429X! Error = %d\n", ret);
      return ret;
    }

  /* Current starts at _MIN mA and increases in steps of 64mA. */

  idx = req_current - BQ2429X_CURRCHG_MIN;
  idx = idx / 64;

  /* Clear previous current and charge type and set new values. */

  regval &= ~(BQ2429XR2_ICHG_MASK);
  regval |= (idx << BQ2429XR2_ICHG_SHIFT);

  if (force_20pct)
    {
      regval |= BQ2429XR2_FORCE_20PCT;
    }
  else
    {
      regval &= ~BQ2429XR2_FORCE_20PCT;
    }

  ret = bq2429x_putreg8(priv, BQ2429X_REG02, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2429X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2429x_voltage
 *
 * Description:
 *   Set battery charger voltage
 *
 ****************************************************************************/

static int bq2429x_voltage(FAR struct battery_charger_dev_s *dev, int value)
{
  FAR struct bq2429x_dev_s *priv = (FAR struct bq2429x_dev_s *)dev;
  int ret;

  /* Set voltage to battery charger */

  ret = bq2429x_setvolt(priv, value);
  if (ret < 0)
    {
      baterr("ERROR: Error setting voltage to BQ2429X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2429x_current
 *
 * Description:
 *   Set the battery charger current rate for charging
 ****************************************************************************/

static int bq2429x_current(FAR struct battery_charger_dev_s *dev, int value)
{
  FAR struct bq2429x_dev_s *priv = (FAR struct bq2429x_dev_s *)dev;
  int ret;

  /* Set current to battery charger */

  ret = bq2429x_setcurr(priv, value);
  if (ret < 0)
    {
      baterr("ERROR: Error setting current to BQ2429X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2429x_input_current
 *
 * Description:
 *   Set the power-supply input current limit
 *
 ****************************************************************************/

static int bq2429x_input_current(FAR struct battery_charger_dev_s *dev,
                                 int value)
{
  FAR struct bq2429x_dev_s *priv = (FAR struct bq2429x_dev_s *)dev;
  int ret;

  ret = bq2429x_powersupply(priv, value);
  if (ret < 0)
    {
      baterr("ERROR: Failed to set BQ2429x power supply input limit: %d\n",
             ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2429x_operate
 *
 * Description:
 *   Do miscellaneous battery operation. There are numerous options that are
 *   configurable on the bq2429x that go beyond what the NuttX battery
 *   charger API provide access to. This operate() function allows changing
 *   some of them.
 *
 *   REG00 EN_HIZ
 *   REG01[1] BOOST_LIM 1A/1.5A Default:1.5A
 *   REG01 CHG_CONFIG
 *   REG02[1] BCOLD
 *   REG02[1] FORCE_20PCT
 *   REG05[1] EN_TERM Charging Termination Enable
 *   REG05[2] WATCHDOG I2C Watchdog Setting 5-20Hrs default 12Hrs
 *   REG05[1] EN_TIMER Charging safety timer
 *   REG05[2] CHG_TIMER Fast Charge Timer Setting
 *   TREG[2] Thermal Regulation Threshold
 *   REG07[1] DPDM_EN Force DPDM detection when VBUS poower is present
 *   REG07[1] TMR2X_EN Safety Timer Slowed during DPM or thermal regulation
 *   REG07[1] BATFET_Disable Turnon off BATFET (Q4)
 *   REG07[1] INT_MASK1 - Allow INT on CHRG_FAULT Default: 1 Allow
 *   REG07[1] INT_MASK0 - Allow INT on BAT_FAULT  Default: 1 Allow
 *
 *   Set by other battery charger methods:
 *   REG00[3] InputCurrent Limit 100mA 3000mA with PSEL
 *   REG02[1] ICHG Fast Charge Current Limit,  512-3008mA Default 2048mA
 *   REG03[4] IPRECHG Pre-charge current Limit 128-2048mA Default: 128mA
 *   REG03[3] ITERM Termination Current Limit  128-1024mA Default: 256mA
 *
 *   System output voltage related:
 *   REG00[4] VINDPM 3.88-5.08V Default:4.36V
 *   REG01[3] Min Sys Voltage Range3.0-3.7V
 *   REG04[6] Charge Voltage Limit 3504-4400mV Default: 4208mV
 *   REG04[1] Battery Recharge Threshold 100/300mV Default 100mV
 *   REG06[4] BOOSTV - Boost Voltage 4550-5510mV Default 4998mV
 *   REG06[2] BHOT Boost Mode Temperature Monitor
 *
 ****************************************************************************/

static int bq2429x_operate(FAR struct battery_charger_dev_s *dev,
                           uintptr_t param)
{
  FAR struct bq2429x_dev_s *priv = (FAR struct bq2429x_dev_s *)dev;
  FAR struct batio_operate_msg_s *msg =
    (FAR struct batio_operate_msg_s *)param;
  int op;
  int value;
  int ret = OK;

  bq2429x_dump_regs(priv);

  op = msg->operate_type;
  value = (int)msg->u32;
  switch (op)
    {
      case BATIO_OPRTN_BOOST:
        ret = bq2429x_setboost_otg_config(priv, true);
        break;

      case BATIO_OPRTN_CHARGE:
        ret = bq2429x_setboost_otg_config(priv, false);
        break;

      case BATIO_OPRTN_EN_TERM:
        ret = bq2429x_en_term(priv, (bool)value);
        break;

      case BATIO_OPRTN_HIZ:
        ret = bq2429x_en_hiz(priv, (bool)value);

        /* Also need to set to 100mA USB host if the battery above Vbatgd? */

        break;

      case BATIO_OPRTN_SYSOFF:
        ret = bq2429x_sysoff(priv);
        break;

      case BATIO_OPRTN_SYSON:
        ret = bq2429x_syson(priv);
        break;

      case BATIO_OPRTN_RESET:
        ret = bq2429x_reset(priv);
        break;

      case BATIO_OPRTN_WDOG:
        ret = bq2429x_watchdog(priv, (bool)value);
        break;

      default:
        baterr("Unsupported opt: 0x%X\n", op);
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: bq2429x_initialize
 *
 * Description:
 *   Initialize the BQ2429x battery driver and return an instance of the
 *   lower-half interface that may be used with battery_charger_register().
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY_CHARGER - Upper half battery driver support
 *   CONFIG_I2C - I2C support
 *   CONFIG_I2C_BQ2429X - And the driver must be explicitly selected.
 *
 * Input Parameters:
 *   i2c       - An instance of the I2C interface to use to communicate with
 *               the BQ2429x
 *   addr      - The I2C address of the BQ2429x (Better be 0x6B).
 *   frequency - The I2C frequency
 *   current   - The input current our power-supply can offer to charger
 *
 * Returned Value:
 *   A pointer to the initialized lower-half driver instance.  A NULL pointer
 *   is returned on a failure to initialize the BQ2429x lower half.
 *
 ****************************************************************************/

FAR struct battery_charger_dev_s *
  bq2429x_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                     uint32_t frequency, int current)
{
  FAR struct bq2429x_dev_s *priv;
  int ret;

  /* Initialize the BQ2429x device structure */

  priv = kmm_zalloc(sizeof(struct bq2429x_dev_s));
  if (priv)
    {
      /* Initialize the BQ2429x device structure */

      nxsem_init(&priv->batsem, 0, 1);
      priv->ops       = &g_bq2429xops;
      priv->i2c       = i2c;
      priv->addr      = addr;
      priv->frequency = frequency;

      /* Reset the BQ2429x */

      ret = bq2429x_reset(priv);
      if (ret < 0)
        {
          baterr("ERROR: Failed to reset the BQ2429x: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Disable watchdog otherwise BQ2429x returns to StandAlone mode */

      ret = bq2429x_watchdog(priv, false);
      if (ret < 0)
        {
          baterr("ERROR: Failed to disable BQ2429x watchdog: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Define the current that our power supply can offer to the charger. */

      ret = bq2429x_powersupply(priv, current);
      if (ret < 0)
        {
          baterr("ERROR: Failed to set BQ2429x power supply: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Disable all interrupts. */

      ret = bq2429x_en_stat(priv, false);
      if (ret < 0)
        {
          baterr("ERROR: Failed to disable BQ2429x interrupts: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }
    }

  return (FAR struct battery_charger_dev_s *)priv;
}

#endif /* CONFIG_BATTERY_CHARGER && CONFIG_I2C && CONFIG_I2C_BQ2429X */

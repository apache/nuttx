/****************************************************************************
 * drivers/power/supply/act8945a.c
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
#include <string.h>
#include <debug.h>
#include <poll.h>
#include <fcntl.h>
#include <assert.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/regulator.h>
#include <nuttx/power/act8945a.h>

#if defined(CONFIG_I2C) && defined(CONFIG_REGULATOR_ACT8945A)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define ACT8945A_BUS_SPEED                        300000
#define ACT8945A_SLAVE_ADDRESS                    0x5b

#ifdef CONFIG_DEBUG_POWER_ERROR
#  define act8945a_err(x, ...)        _err(x, ##__VA_ARGS__)
#else
#  define act8945a_err(x, ...)        uerr(x, ##__VA_ARGS__)
#endif
#ifdef CONFIG_DEBUG_POWER_WARN
#  define act8945a_warn(x, ...)       _warn(x, ##__VA_ARGS__)
#else
#  define act8945a_warn(x, ...)       uwarn(x, ##__VA_ARGS__)
#endif
#ifdef CONFIG_DEBUG_POWER_INFO
#  define act8945a_info(x, ...)       _info(x, ##__VA_ARGS__)
#else
#  define act8945a_info(x, ...)       uinfo(x, ##__VA_ARGS__)
#endif

#define ACT8945A_NUM_VOLTAGES                     64

#define ACT8945A_SYS0                             (0x00)
#define ACT8945A_SYS1                             (0x01)

#define ACT8945A_DCDC1_VSET0                      (0x20)
#define ACT8945A_DCDC1_VSET1                      (0x21)
#define ACT8945A_DCDC1_CONTROL                    (0x22)
#define ACT8945A_DCDC2_VSET0                      (0x30)
#define ACT8945A_DCDC2_VSET1                      (0x31)
#define ACT8945A_DCDC2_CONTROL                    (0x32)
#define ACT8945A_DCDC3_VSET0                      (0x40)
#define ACT8945A_DCDC3_VSET1                      (0x41)
#define ACT8945A_DCDC3_CONTROL                    (0x42)
#define ACT8945A_LDO1_VSET                        (0x50)
#define ACT8945A_LDO1_CONTROL                     (0x51)
#define ACT8945A_LDO2_VSET                        (0x54)
#define ACT8945A_LDO2_CONTROL                     (0x55)
#define ACT8945A_LDO3_VSET                        (0x60)
#define ACT8945A_LDO3_CONTROL                     (0x61)
#define ACT8945A_LDO4_VSET                        (0x64)
#define ACT8945A_LDO4_CONTROL                     (0x65)

#define ACT8945A_SYSTRST_MASK                     (0x80)
#define ACT8945A_SYSMODE_MASK                     (0x40)
#define ACT8945A_SYSLEV_MASK                      (0x0f)

#define ACT8945A_VSET_MASK                        (0x3f)
#define ACT8945A_EN_MASK                          (0x80)

#define ACT8945A_SCRATCH_MASK                     (0x0f)
#define SCRATCH_TEST_VAL                          (0x05)

#define ACT8945A_PULLDOWN_MASK                    (0x40)

#ifdef CONFIG_ACT8945A_TRST_64
#  define ACT8945A_TRST                           (0x80) /* 64ms*/
#else
#  define ACT8945A_TRST                           (0)    /* 260ms */
#endif

#ifdef CONFIG_ACT8945A_SYSLEV_MODE_INTERRUPT
#  define ACT8945A_SYSLEV_MODE                    (0x40) /* Interrupt */
#else
#  define ACT8945A_SYSLEV_MODE                    (0)    /* Shutdown */
#endif

#if defined(CONFIG_ACT8945A_SYSLEV_2300)
#  define ACT8945A_SYSLEV 0
#elif defined(CONFIG_ACT8945A_SYSLEV_2400)
#  define ACT8945A_SYSLEV 1
#elif defined(CONFIG_ACT8945A_SYSLEV_2500)
#  define ACT8945A_SYSLEV 2
#elif defined(CONFIG_ACT8945A_SYSLEV_2600)
#  define ACT8945A_SYSLEV 3
#elif defined(CONFIG_ACT8945A_SYSLEV_2700)
#  define ACT8945A_SYSLEV 4
#elif defined(CONFIG_ACT8945A_SYSLEV_2800)
#  define ACT8945A_SYSLEV 5
#elif defined(CONFIG_ACT8945A_SYSLEV_2900)
#  define ACT8945A_SYSLEV 6
#elif defined(CONFIG_ACT8945A_SYSLEV_3000)
#  define ACT8945A_SYSLEV 7
#elif defined(CONFIG_ACT8945A_SYSLEV_3100)
#  define ACT8945A_SYSLEV 8
#elif defined(CONFIG_ACT8945A_SYSLEV_3200)
#  define ACT8945A_SYSLEV 9
#elif defined(CONFIG_ACT8945A_SYSLEV_3300)
#  define ACT8945A_SYSLEV 10
#elif defined(CONFIG_ACT8945A_SYSLEV_3400)
#  define ACT8945A_SYSLEV 11
#elif defined(CONFIG_ACT8945A_SYSLEV_3500)
#  define ACT8945A_SYSLEV 12
#elif defined(CONFIG_ACT8945A_SYSLEV_3600)
#  define ACT8945A_SYSLEV 13
#elif defined(CONFIG_ACT8945A_SYSLEV_3700)
#  define ACT8945A_SYSLEV 14
#elif defined(CONFIG_ACT8945A_SYSLEV_3800)
#  define ACT8945A_SYSLEV 15
#else
#  error No act8945a sys level detect threshold defined
#endif

#ifdef CONFIG_ACT8945A_DCDC1_BOOT_ON
#  define ACT8945A_DCDC1_BOOT_ON 1
#else
#  define ACT8945A_DCDC1_BOOT_ON 0
#endif

#ifdef CONFIG_ACT8945A_DCDC2_BOOT_ON
#  define ACT8945A_DCDC2_BOOT_ON 1
#else
#  define ACT8945A_DCDC2_BOOT_ON 0
#endif

#ifdef CONFIG_ACT8945A_DCDC3_BOOT_ON
#  define ACT8945A_DCDC3_BOOT_ON 1
#else
#  define ACT8945A_DCDC3_BOOT_ON 0
#endif

#ifdef CONFIG_ACT8945A_LDO1_BOOT_ON
#  define ACT8945A_LDO1_BOOT_ON 1
#else
#  define ACT8945A_LDO1_BOOT_ON 0
#endif

#ifdef CONFIG_ACT8945A_LDO2_BOOT_ON
#  define ACT8945A_LDO2_BOOT_ON 1
#else
#  define ACT8945A_LDO2_BOOT_ON 0
#endif

#ifdef CONFIG_ACT8945A_LDO3_BOOT_ON
#  define ACT8945A_LDO3_BOOT_ON 1
#else
#  define ACT8945A_LDO3_BOOT_ON 0
#endif

#ifdef CONFIG_ACT8945A_LDO4_BOOT_ON
#  define ACT8945A_LDO4_BOOT_ON 1
#else
#  define ACT8945A_LDO4_BOOT_ON 0
#endif

#ifdef CONFIG_ACT8945A_DCDC1_APPLY_UV
#  define ACT8945A_DCDC1_APPLY_UV 1
#else
#  define ACT8945A_DCDC1_APPLY_UV 0
#endif

#ifdef CONFIG_ACT8945A_DCDC2_APPLY_UV
#  define ACT8945A_DCDC2_APPLY_UV 1
#else
#  define ACT8945A_DCDC2_APPLY_UV 0
#endif

#ifdef CONFIG_ACT8945A_DCDC3_APPLY_UV
#  define ACT8945A_DCDC3_APPLY_UV 1
#else
#  define ACT8945A_DCDC3_APPLY_UV 0
#endif

#ifdef CONFIG_ACT8945A_LDO1_APPLY_UV
#  define ACT8945A_LDO1_APPLY_UV 1
#else
#  define ACT8945A_LDO1_APPLY_UV 0
#endif

#ifdef CONFIG_ACT8945A_LDO2_APPLY_UV
#  define ACT8945A_LDO2_APPLY_UV 1
#else
#  define ACT8945A_LDO2_APPLY_UV 0
#endif

#ifdef CONFIG_ACT8945A_LDO3_APPLY_UV
#  define ACT8945A_LDO3_APPLY_UV 1
#else
#  define ACT8945A_LDO3_APPLY_UV 0
#endif

#ifdef CONFIG_ACT8945A_LDO4_APPLY_UV
#  define ACT8945A_LDO4_APPLY_UV 1
#else
#  define ACT8945A_LDO4_APPLY_UV 0
#endif

#define ACT8945A_DCDC1_PULLDOWN 0
#define ACT8945A_DCDC2_PULLDOWN 0
#define ACT8945A_DCDC3_PULLDOWN 0

#ifdef CONFIG_ACT8945A_LDO1_PULLDOWN
#  define ACT8945A_LDO1_PULLDOWN 1
#else
#  define ACT8945A_LDO1_PULLDOWN 0
#endif

#ifdef CONFIG_ACT8945A_LDO2_PULLDOWN
#  define ACT8945A_LDO2_PULLDOWN 1
#else
#  define ACT8945A_LDO2_PULLDOWN 0
#endif

#ifdef CONFIG_ACT8945A_LDO3_PULLDOWN
#  define ACT8945A_LDO3_PULLDOWN 1
#else
#  define ACT8945A_LDO3_PULLDOWN 0
#endif

#ifdef CONFIG_ACT8945A_LDO4_PULLDOWN
#  define ACT8945A_LDO4_PULLDOWN 1
#else
#  define ACT8945A_LDO4_PULLDOWN 0
#endif

/* Ramp times are fixed for this regulator */

#define ETIME_0   0
#define ETIME_400 400
#define ETIME_800 800

#define ACT8945A_REG(_id, _vsel, _etime)              \
  [ACT8945A_##_id] =                                  \
  {                                                   \
    .n_voltages    = ACT8945A_NUM_VOLTAGES,           \
    .vsel_reg      = ACT8945A_##_id##_##_vsel,        \
    .vsel_mask     = ACT8945A_VSET_MASK,              \
    .name          = CONFIG_ACT8945A_##_id##_NAME,    \
    .id            = ACT8945A_##_id,                  \
    .enable_reg    = ACT8945A_##_id##_CONTROL,        \
    .enable_mask   = ACT8945A_EN_MASK,                \
    .enable_time   = (_etime),                        \
    .ramp_delay    = 400,                             \
    .uv_step       = 0,                               \
    .min_uv        = CONFIG_ACT8945A_##_id##_MIN_UV,  \
    .max_uv        = CONFIG_ACT8945A_##_id##_MAX_UV,  \
    .pulldown      = ACT8945A_##_id##_PULLDOWN,       \
    .pulldown_reg  = ACT8945A_##_id##_CONTROL,        \
    .pulldown_mask = ACT8945A_PULLDOWN_MASK,          \
    .apply_uv      = ACT8945A_##_id##_APPLY_UV,       \
    .boot_on       = ACT8945A_##_id##_BOOT_ON,        \
  }
/****************************************************************************
 * Private type
 ****************************************************************************/

struct regulator_act8945a_priv
{
  FAR struct       i2c_master_s     *i2c;
  uint8_t                           i2c_addr;
  int                               i2c_freq;
  FAR const struct regulator_desc_s *regulators;
  FAR struct       regulator_dev_s  *rdev;
};

struct act8945a_dev_s
{
  /* Common part of the regulator driver visible to the upper-half driver */

  FAR struct regulator_desc_s  *regulator_desc_s;

  /* Data fields specific to the lower half act8945a driver follow */

  FAR struct i2c_master_s      *i2c;      /* I2C interface */
  uint8_t                      addr;      /* I2C address */
  uint32_t                     frequency; /* I2C frequency */
};

enum
{
  ACT8945A_DCDC1 = 0,
  ACT8945A_DCDC2,
  ACT8945A_DCDC3,
  ACT8945A_LDO1,
  ACT8945A_LDO2,
  ACT8945A_LDO3,
  ACT8945A_LDO4,
  ACT8945A_MAX,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

/* regulator lower half methods */

static int act8945a_list_voltage(struct regulator_dev_s *rdev,
                                 unsigned int selector);
static int act8945a_set_voltage_sel(FAR struct regulator_dev_s *rdev,
                                    unsigned int selector);
static int act8945a_get_voltage_sel(FAR struct regulator_dev_s *rdev);
static int act8945a_enable(struct regulator_dev_s *rdev);
static int act8945a_is_enabled(struct regulator_dev_s *rdev);
static int act8945a_disable(struct regulator_dev_s *rdev);
static int act8945a_write_scratch(struct regulator_act8945a_priv *priv,
                                  uint8_t value);
static int act8945a_read_scratch(struct regulator_act8945a_priv *priv,
                                 uint8_t * scratch);
static int act8945a_enable_pulldown(struct regulator_dev_s *rdev);
static int act8945a_disable_pulldown(struct regulator_dev_s *rdev);
static int act89845a_set_sysmode(struct regulator_act8945a_priv *priv,
                                 uint8_t mode);
static int act89845a_set_trstmode(struct regulator_act8945a_priv *priv,
                                  uint8_t timer);
static int act89845a_set_syslev(struct regulator_act8945a_priv *priv,
                                  uint8_t level);

/* I2C support */

static int act8945a_getreg(FAR struct regulator_act8945a_priv *priv,
                           uint8_t regaddr, uint8_t *regval);
static int act8945a_putreg(FAR struct regulator_act8945a_priv *priv,
                           uint8_t regaddr, uint8_t regval);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const int mv_list[] =
{
  600,   625,  650,  675,  700,  725,  750,  775,
  800,   825,  850,  875,  900,  925,  950,  975,
  1000, 1025, 1050, 1075, 1100, 1125, 1150, 1175,
  1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550,
  1600, 1650, 1700, 1750, 1800, 1850, 1900, 1950,
  2000, 2050, 2100, 2150, 2200, 2250, 2300, 2350,
  2400, 2500, 2600, 2700, 2800, 2900, 3000, 3100,
  3200, 3300, 3400, 3500, 3600, 3700, 3800, 3900,
};

static const struct regulator_desc_s g_act8945a_regulators[] =
{
  ACT8945A_REG(DCDC1, VSET0, ETIME_0),
  ACT8945A_REG(DCDC2, VSET0, ETIME_400),
  ACT8945A_REG(DCDC3, VSET0, ETIME_0),
  ACT8945A_REG(LDO1, VSET, ETIME_800),
  ACT8945A_REG(LDO2, VSET, ETIME_0),
  ACT8945A_REG(LDO3, VSET, ETIME_0),
  ACT8945A_REG(LDO4, VSET, ETIME_0),
};

static const struct regulator_desc_s g_alt_act8945a_regulators[] =
{
  ACT8945A_REG(DCDC1, VSET1, ETIME_0),
  ACT8945A_REG(DCDC2, VSET1, 400000),
  ACT8945A_REG(DCDC3, VSET1, ETIME_0),
  ACT8945A_REG(LDO1, VSET, 800000),
  ACT8945A_REG(LDO2, VSET, ETIME_0),
  ACT8945A_REG(LDO3, VSET, ETIME_0),
  ACT8945A_REG(LDO4, VSET, ETIME_0),
};

/* Regulator operations */

static const struct regulator_ops_s g_act8945a_ops =
{
  act8945a_list_voltage,     /* list_voltage     */
  NULL,                      /* set_voltage      */
  act8945a_set_voltage_sel,  /* set_voltage_sel  */
  NULL,                      /* get_voltage      */
  act8945a_get_voltage_sel,  /* get_voltage_sel  */
  act8945a_enable,           /* enable           */
  act8945a_is_enabled,       /* is_enabled       */
  act8945a_disable,          /* disable          */
  act8945a_enable_pulldown,  /* enable pulldown if reg. disabled */
  act8945a_disable_pulldown, /* disable pulldown if reg. disabled */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: act8945a_getreg
 *
 * Description:
 *   Reads a single register from the ACT8945A regulator
 *
 * Input Parameters:
 *
 *   priv    - private act8945a device structure
 *   regaddr - the act8945a register address
 *   regval  - location to return the act8945a register value
 *
 * Returned value:
 *
 *   Success, or fail reason
 *
 ****************************************************************************/

static int act8945a_getreg(FAR struct regulator_act8945a_priv *priv,
                           uint8_t regaddr, uint8_t *regval)
{
  struct i2c_config_s config;
  int                 ret;
  irqstate_t          flags;

  /* Sanity check */

  DEBUGASSERT(priv   != NULL);
  DEBUGASSERT(regval != NULL);

  /* Set up the I2C configuration */

  config.frequency = priv->i2c_freq;
  config.address   = priv->i2c_addr;
  config.addrlen   = 7;

  /* Write the register address */

  flags = spin_lock_irqsave(NULL);
  ret = i2c_write(priv->i2c, &config, &regaddr, sizeof(regaddr));
  spin_unlock_irqrestore(NULL, flags);
  if (ret < 0)
    {
      act8945a_err("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 8 bits from the register */

  flags = spin_lock_irqsave(NULL);
  ret = i2c_read(priv->i2c, &config, regval, sizeof(*regval));
  spin_unlock_irqrestore(NULL, flags);
  if (ret < 0)
    {
      act8945a_err("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  act8945a_info("INFO: addr: %02x value: %02x\n", regaddr, *regval);

  return OK;
}

/****************************************************************************
 * Name: act8945a_putreg
 *
 * Description:
 *   Writes a single byte to one of the ACT8945A registers.
 *
 * Input Parameters:
 *
 *   priv    - private act8945a device structure
 *   regaddr - the act8945a register address
 *   regval  - value to write to the act8945a register
 *
 * Returned value:
 *
 *   Success, or fail reason
 *
 ****************************************************************************/

static int act8945a_putreg(FAR struct regulator_act8945a_priv *priv,
                           uint8_t regaddr, uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t             buffer[2];
  int                 ret;
  irqstate_t          flags;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Set up a 2-byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Set up the I2C configuration */

  config.frequency = priv->i2c_freq;
  config.address   = priv->i2c_addr;
  config.addrlen   = 7;

  /* Write the register address followed by the data (no RESTART) */

  flags = spin_lock_irqsave(NULL);
  ret = i2c_write(priv->i2c, &config, buffer, sizeof(buffer));
  spin_unlock_irqrestore(NULL, flags);

  if (ret < 0)
    {
      act8945a_err("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  act8945a_info("INFO:addr: %02x value: %02x\n", regaddr, regval);

  return OK;
}

/****************************************************************************
 * Name: act8945a_list_voltage
 *
 * Description:
 *
 * Input Parameters:
 *
 *   rdev     - The regulator device pointer.
 *   selector - The selector for the voltage look-up
 *
 * Returned Value:
 *
 *   The voltage for the selector, or error code.
 *
 ****************************************************************************/

static int act8945a_list_voltage(FAR struct regulator_dev_s *rdev,
                                 unsigned int selector)
{
  if (selector < ACT8945A_NUM_VOLTAGES)
    {
      return (mv_list[selector]);
    }
  else
    {
      return -EINVAL;
    }
}

/****************************************************************************
 * Name: act8945a_enable_pulldown
 *
 * Description:
 *
 * Input Parameters:
 *
 *   rdev     - The regulator device pointer.
 *
 * Returned Value
 *
 *   Success or error code.
 *
 ****************************************************************************/

static int act8945a_enable_pulldown(struct regulator_dev_s *rdev)
{
  FAR struct regulator_act8945a_priv *priv = rdev->priv;
  FAR const struct regulator_desc_s *regulator = rdev->desc;

  uint8_t regval;

  /* for ease of code commonality we skip any calls for DCDC1-3 */

  if (regulator->id < ACT8945A_LDO1)
    {
      return OK;
    }

  if (act8945a_getreg(priv, regulator->enable_reg, &regval) == 0)
    {
      regval |= regulator->pulldown_mask;
      if (act8945a_putreg(priv, regulator->pulldown_reg, regval) == 0)
        {
          pwrinfo("INFO: Output discharge control enabled %s\n",
                   regulator->name);
          return OK;
        }
    }

  return -EIO;
}

/****************************************************************************
 * Name: act8945a_disable_pulldown
 *
 * Description:
 *
 * Input Parameters:
 *
 *   rdev     - The regulator device pointer.
 *
 * Returned Value:
 *
 *   Success or error code.
 *
 ****************************************************************************/

static int act8945a_disable_pulldown(struct regulator_dev_s *rdev)
{
  FAR struct regulator_act8945a_priv *priv = rdev->priv;
  FAR const struct regulator_desc_s *regulator = rdev->desc;

  uint8_t regval;

  /* for ease of code commonality we skip any calls for DCDC1-3 */

  if (regulator->id < ACT8945A_LDO1)
    {
      return OK;
    }

  if (act8945a_getreg(priv, regulator->pulldown_reg, &regval) == 0)
    {
      regval &= ~regulator->pulldown_mask;
      if (act8945a_putreg(priv, regulator->enable_reg, regval) == 0)
        {
          pwrinfo("INFO: Output discharge control disabled %s\n",
                   regulator->name);
          return OK;
        }
    }

  return -EIO;
}

/****************************************************************************
 * Name: act8945a_write_scratch
 *
 * Description:
 *
 * Input Parameters:
 *
 *   priv     - private act8945a device structure
 *   value    - The value to write to the 4 scratch bits
 *
 * Returned Value:
 *
 *   Success or error code.
 *
 ****************************************************************************/

static int act8945a_write_scratch(struct regulator_act8945a_priv *priv,
                                  uint8_t value)
{
  uint8_t regval;

  if (act8945a_getreg(priv, ACT8945A_SYS1, &regval) == 0)
    {
      regval &= ~ACT8945A_SCRATCH_MASK;
      regval |= value & ACT8945A_SCRATCH_MASK;
      if (act8945a_putreg(priv, ACT8945A_SYS1, regval) == 0)
        {
          return OK;
        }
    }

  return -EIO;
}

/****************************************************************************
 * Name: act8945a_read_scratch
 *
 * Description:
 *
 * Input Parameters:
 *
 *   priv    - private act8945a device structure
 *   scratch - pointer to the scratch value read
 *
 * Returned Value:
 *
 *   The scratch value read, and success or error code.
 *
 ****************************************************************************/

static int act8945a_read_scratch(struct regulator_act8945a_priv *priv,
                                 uint8_t *scratch)
{
  if (act8945a_getreg(priv, ACT8945A_SYS1, scratch) == 0)
    {
      return OK;
    }
  else
    {
      return -EIO;
    }
}

/****************************************************************************
 * Name: act89845a_set_sysmode
 *
 * Description:
 *
 * Input Parameters:
 *
 *   priv     - private act8945a device structure
 *   mode     - The system mode to set
 *
 * Returned Value:
 *
 *   Success or error code.
 *
 ****************************************************************************/

static int act89845a_set_sysmode(struct regulator_act8945a_priv *priv,
                                 uint8_t mode)
{
  uint8_t regval;

  if (act8945a_getreg(priv, ACT8945A_SYS0, &regval) == 0)
    {
      regval &= ~ACT8945A_SYSMODE_MASK;
      regval |= mode & ACT8945A_SYSMODE_MASK;
      if (act8945a_putreg(priv, ACT8945A_SYS0, regval) == 0)
        {
          pwrinfo ("INFO: sysmode set to %d\n", mode >> 6);
          return OK;
        }
    }

  return -EIO;
}

/****************************************************************************
 * Name: act89845a_set_trstmode
 *
 * Description:
 *
 * Input Parameters:
 *
 *   priv     - private act8945a device structure
 *   mode     - The system reset timer setting to set
 *
 * Returned Value:
 *
 *   Success or error code.
 *
 ****************************************************************************/

static int act89845a_set_trstmode(struct regulator_act8945a_priv *priv,
                                  uint8_t timer)
{
  uint8_t regval;

  if (act8945a_getreg(priv, ACT8945A_SYS0, &regval) == 0)
    {
      regval &= ~ACT8945A_SYSTRST_MASK;
      regval |= timer & ACT8945A_SYSTRST_MASK;
      if (act8945a_putreg(priv, ACT8945A_SYS0, regval) == 0)
        {
          pwrinfo ("INFO: sysmode set to %d\n", timer >> 7);
          return OK;
        }
    }

  return -EIO;
}

/****************************************************************************
 * Name: act89845a_set_syslev
 *
 * Description:
 *
 * Input Parameters:
 *
 *   priv     - private act8945a device structure
 *   level    - The system voltage level detect threshold to set
 *
 * Returned Value:
 *
 *   Success or error code.
 *
 ****************************************************************************/

static int act89845a_set_syslev(struct regulator_act8945a_priv *priv,
                                uint8_t level)
{
  uint8_t regval;

  if (act8945a_getreg(priv, ACT8945A_SYS0, &regval) == 0)
    {
      regval &= ~ACT8945A_SYSLEV_MASK;
      regval |= level & ACT8945A_SYSLEV_MASK;
      if (act8945a_putreg(priv, ACT8945A_SYS0, regval) == 0)
        {
          pwrinfo ("INFO: syslevel set to %d\n", level);
          return OK;
        }
    }

  return -EIO;
}

/****************************************************************************
 * Name: act8945a_set_voltage_sel
 *
 * Description:
 *
 * Input Parameters:
 *
 *   rdev     - The regulator device pointer.
 *   selector - The selector to use for the regulator
 *
 * Returned Value:
 *
 *   Success or error code.
 *
 ****************************************************************************/

static int act8945a_set_voltage_sel(struct regulator_dev_s *rdev,
                                    unsigned selector)
{
  FAR struct regulator_act8945a_priv *priv = rdev->priv;
  FAR const struct regulator_desc_s *regulator = rdev->desc;

  if (selector >= ACT8945A_NUM_VOLTAGES)
    {
      return -EINVAL;
    }
  else
    {
      if (act8945a_putreg(priv, regulator->vsel_reg,
                           selector & regulator->vsel_mask) == 0)
        {
          pwrinfo("INFO: reg %s set to %d mV using selector %d \n",
                   regulator->name, mv_list[selector], selector);
          return OK;
        }
      else
        {
          return -EIO;
        }
    }
}

/****************************************************************************
 * Name: act8945a_get_voltage_sel
 *
 * Description:
 *
 * Input Parameters:
 *
 *   rdev     - The regulator device pointer.
 *
 * Returned Value:
 *
 *   The current selector from the device, or error code.
 *
 ****************************************************************************/

static int act8945a_get_voltage_sel(struct regulator_dev_s *rdev)
{
  FAR struct regulator_act8945a_priv *priv = rdev->priv;
  FAR const struct regulator_desc_s *regulator = rdev->desc;

  uint8_t regval;

  if (act8945a_getreg(priv, regulator->vsel_reg, &regval) == 0)
    {
      return regval;
    }
  else
    {
      return -EIO;
    }
}

/****************************************************************************
 * Name: act8945a_enable
 *
 * Description:
 *
 * Input Parameters:
 *
 *   rdev     - The regulator device pointer.
 *
 * Returned Value:
 *
 *   Success, or error code.
 *
 ****************************************************************************/

static int act8945a_enable(struct regulator_dev_s *rdev)
{
  FAR struct regulator_act8945a_priv *priv = rdev->priv;
  FAR const struct regulator_desc_s *regulator = rdev->desc;

  uint8_t regval;

  if (act8945a_getreg(priv, regulator->vsel_reg, &regval) == 0)
    {
      regval |= regulator->enable_mask;
      if (act8945a_putreg(priv, regulator->enable_reg, regval) == 0)
        {
          pwrinfo("INFO: reg %s enabled\n", regulator->name);
          return OK;
        }
    }

  return -EIO;
}

/****************************************************************************
 * Name: act8945a_is_enabled
 *
 * Description:
 *
 * Input Parameters:
 *
 *   rdev    - The regulator device pointer.
 *
 * Returned Value:
 *
 *   0      - disabled
 *   1      - enabled or disabled
 *   -value - error code.
 *
 ****************************************************************************/

static int act8945a_is_enabled(struct regulator_dev_s *rdev)
{
  FAR struct regulator_act8945a_priv *priv = rdev->priv;
  FAR const struct regulator_desc_s *regulator = rdev->desc;

  uint8_t regval;

  if (act8945a_getreg(priv, regulator->enable_reg, &regval) == 0)
    {
      return (regval & regulator->enable_mask);
    }
  else
    {
      return -ENODEV;
    }
}

/****************************************************************************
 * Name: act8945a_disable
 *
 * Description:
 *
 * Input Parameters:
 *
 *   rdev     - The regulator device pointer.
 *
 * Returned Value:
 *
 *   Success, or error code.
 *
 ****************************************************************************/

static int act8945a_disable(struct regulator_dev_s *rdev)
{
  FAR struct regulator_act8945a_priv *priv = rdev->priv;
  FAR const struct regulator_desc_s *regulator = rdev->desc;

  uint8_t regval;

  if (act8945a_getreg(priv, regulator->vsel_reg, &regval) == 0)
    {
      regval &= ~regulator->enable_mask;
      if (act8945a_putreg(priv, regulator->enable_reg, regval) == 0)
        {
          pwrinfo("INFO: reg %s disabled\n", regulator->name);
          return OK;
        }
    }

  return -EIO;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int act8945a_initialize(FAR struct i2c_master_s *i2c, unsigned int vsel)
{
  FAR struct regulator_act8945a_priv *priv;

  uint8_t scratch;
  int regnum;
  bool reg_failed = false;

  DEBUGASSERT(vsel == 0 || vsel == 1);

  if (!i2c)
    {
      return -ENODEV;
    }

  priv = kmm_zalloc(sizeof(struct regulator_act8945a_priv));

  if (!priv)
    {
      return -ENOMEM;
    }

  priv->i2c      = i2c;
  priv->i2c_addr = ACT8945A_SLAVE_ADDRESS;
  priv->i2c_freq = ACT8945A_BUS_SPEED;

  /* Early test to see if we can read the ACT8945A.
   *
   * We do this by writing a data pattern into the 4 scratch bits of SYS1
   * register and making sure we can read it back OK.
   */

  if (!act8945a_write_scratch(priv, SCRATCH_TEST_VAL))
    {
      if (!act8945a_read_scratch(priv, &scratch))
        {
          if (scratch != SCRATCH_TEST_VAL)
            {
              goto error;
            }
        }
    }
  else
    {
      goto error;
    }

  /* Initialise with Kconfig values, using correct struct depending on VSEL.
   * - Some hardware may set VSEL via software.
   * - Some hardware may have it hard coded and may or may not
   * - allow the pin to be read back.
   * - The value must be determined by board specific logic.
   */

  if (vsel == 0)
    {
      priv->regulators = g_act8945a_regulators;
    }
  else
    {
      priv->regulators = g_alt_act8945a_regulators;
    }

  for (regnum = ACT8945A_DCDC1; regnum < ACT8945A_NUM_REGS; regnum++)
    {
      priv->rdev = regulator_register(&priv->regulators[regnum],
                                      &g_act8945a_ops, priv);
      if (priv->rdev == NULL)
        {
          reg_failed = true;
          pwrerr("ERROR: failed to register act8945a regulator %s\n",
                 priv->regulators[regnum].name);
        }
    }

  if (reg_failed)
    {
      goto error;
    }

  act89845a_set_sysmode(priv, ACT8945A_SYSLEV_MODE);
  act89845a_set_trstmode(priv, ACT8945A_TRST);
  act89845a_set_syslev(priv, ACT8945A_SYSLEV);

  return OK;

error:
  kmm_free(priv);
  return -ENODEV;
}

#endif /* CONFIG_I2C && CONFIG_REGULATOR_ACT8945A */

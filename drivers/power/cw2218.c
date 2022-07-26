/****************************************************************************
 * drivers/power/cw2218.c
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

/* Lower half driver for CW2218 battery fuel gauge */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <nuttx/kthread.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>
#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/power/battery_gauge.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/signal.h>
#include "cw2218.h"
#include <nuttx/wqueue.h>

/* This driver requires:
 *
 * CONFIG_BATTERY - Upper half battery driver support
 * CONFIG_I2C - I2C support
 * CONFIG_CW2218 - And the driver must be explicitly selected.
 */

#if defined(CONFIG_BATTERY_GAUGE) && defined(CONFIG_I2C) && \
    defined(CONFIG_CW2218)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define CW2218_REG_TEMPMAX_DEFAULT_VALUE  0xFF /* Maximum temperature threshold register default value */
#define CW2218_REG_TEMPMIN_DEFAULT_VALUE  0x14 /* Minimum temperature threshold register default value */
#define CW2218_TEMP_ERROR_DEFAULT_VALUE   -10  /* default temperature return when error */

/****************************************************************************
 * Private
 ****************************************************************************/

struct cw2218_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_gauge_dev_s dev; /* Battery gauge device */

  /* Data fields specific to the lower half cw2218 driver follow */

  FAR struct i2c_master_s *i2c;                      /* I2C interface */
  FAR struct ioexpander_dev_s *ioe;                  /* Ioexpander device. */
  uint8_t addr;                                      /* I2C address */
  uint32_t frequency;                                /* I2C frequency */
  int pin;                                           /* Interrupt pin */
  struct work_s work;                                /* Work queue for reading data. */
  struct work_s soc_full_work;                       /* Work queue for reading soc data. */
  int last_cap;                                      /* battery cap change */
  int last_batt_temp;                                /* battery temp change */
  bool gauge_init_status;                            /* gauge init status */
  int full_work_exit;                                /* ui full work */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C support */

static int cw2218_getreg8(FAR struct cw2218_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *regval, int num_char);
static int cw2218_putreg8(FAR struct cw2218_dev_s *priv, uint8_t regaddr,
                          uint8_t regval);
static inline int cw2218_getvoltage(FAR struct cw2218_dev_s *priv,
                                    b16_t *voltage);
static inline int cw2218_getsoc(FAR struct cw2218_dev_s *priv,
                                b16_t *soc, ub8_t *soc_h);
static inline int cw2218_getcurrent(FAR struct cw2218_dev_s *priv,
                                    b16_t *current);
static inline int cw2218_gettemp(FAR struct cw2218_dev_s *priv,
                                 b8_t *temp);
static inline int cw2218_get_chipid(FAR struct cw2218_dev_s *priv,
                                    unsigned int *id);

static int cw2218_active(FAR struct cw2218_dev_s *priv);
static int cw2218_sleep(FAR struct cw2218_dev_s *priv);
static int cw2218_write_profile(FAR struct cw2218_dev_s *priv,
                                unsigned char const buf[]);
static int cw2218_get_state(FAR struct cw2218_dev_s *priv);
static int cw2218_config_start_ic(FAR struct cw2218_dev_s *priv);
static int cw2218_init(FAR struct cw2218_dev_s *priv);

/* Battery driver lower half methods */

static int cw2218_state(struct battery_gauge_dev_s *dev, int *status);
static int cw2218_online(struct battery_gauge_dev_s *dev, bool *status);
static int cw2218_voltage(struct battery_gauge_dev_s *dev, b16_t *voltage);
static int cw2218_current(struct battery_gauge_dev_s *dev, b16_t *current);
static int cw2218_temp(struct battery_gauge_dev_s *dev, b8_t *temp);
static int cw2218_capacity(struct battery_gauge_dev_s *dev, b16_t *capacity);
static int cw2218_chipid(struct battery_gauge_dev_s *dev,
                         unsigned int *chipid);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static void cw2218_interrupt_worker(FAR void *arg);
static void soc_change_worker(FAR void *arg);
static void cw2218_start_soc_full_work(FAR struct cw2218_dev_s *priv);

static const struct battery_gauge_operations_s g_cw2218ops =
{
  cw2218_state,
  cw2218_online,
  cw2218_voltage,
  cw2218_capacity,
  cw2218_current,
  cw2218_temp,
  cw2218_chipid,
};

static const unsigned char g_config_profile_info[SIZE_OF_PROFILE] =
{
  0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xA9, 0xC3, 0xC3, 0xCC, 0xC2, 0xC3, 0x8F, 0x4F,
  0x2C, 0xF8, 0xD2, 0x99, 0x7F, 0x6A, 0x57, 0x49,
  0x3F, 0x35, 0x28, 0x57, 0x61, 0xDE, 0x5A, 0xC6,
  0xC2, 0xC7, 0xCF, 0xD3, 0xD3, 0xD2, 0xCF, 0xCC,
  0xC9, 0xCE, 0xD4, 0xB5, 0xA1, 0x94, 0x8C, 0x85,
  0x81, 0x83, 0x8E, 0x95, 0xA6, 0x93, 0x5B, 0xA3,
  0x20, 0x00, 0xAB, 0x10, 0x02, 0x81, 0x9A, 0x00,
  0x00, 0x00, 0x64, 0x33, 0x90, 0xB3, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE9,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cw2218_getreg8
 *
 * Description:
 *    Read a 8-bit value from a cw2218 register pair.
 *
 * Input Parameters:
 *   priv    - Device struct
 *   regaddr - Register address
 *   regval  - Register value
 *   num_char - length of regval
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int cw2218_getreg8(FAR struct cw2218_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *regval, int num_char)
{
  int err;
  int retries = 0;
  struct i2c_msg_s msg[2];
  FAR struct i2c_master_s *dev = priv->i2c;

  msg[0].addr = priv->addr;
  msg[0].buffer = &regaddr;
  msg[0].length = 1;
  msg[0].flags = I2C_M_NOSTOP;
  msg[0].frequency = priv->frequency;

  msg[1].addr = priv->addr;
  msg[1].buffer = regval;
  msg[1].length = num_char;
  msg[1].flags = I2C_M_READ;
  msg[1].frequency = priv->frequency;

  for (retries = 0; retries < CW_IIC_RETRY_NUM; retries++)
    {
      err = I2C_TRANSFER(dev, msg, 2);
      if (err >= 0)
        {
          break;
        }
      else
        {
          nxsig_usleep(1);
          baterr("i2c_write failed:%d retries:%d\n", err, retries);
        }
    }

  return (err >= 0) ? OK : err;
}

/****************************************************************************
 * Name: cw2218_putreg8
 *
 * Description:
 *    Write a 8-bit value to a cw2218 register pair.
 *
 * Input Parameters:
 *   priv    - Device struct
 *   regaddr - Register address
 *   regval  - Register value
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int cw2218_putreg8(FAR struct cw2218_dev_s *priv, uint8_t regaddr,
                          uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Set up a message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Write the register address followed by the data (no RESTART) */

  return i2c_write(priv->i2c, &config, buffer, 2);
}

/****************************************************************************
 * Name: cw2218_get_chipid
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

static inline int cw2218_get_chipid(FAR struct cw2218_dev_s *priv,
                                    unsigned int *id)
{
  uint8_t regval = 0;
  int ret;

  ret = cw2218_getreg8(priv, CW2218_COMMAND_VERSION, &regval, 1);
  if (ret == OK)
    {
     *id = regval;
    }

  return ret;
}

/****************************************************************************
 * Name: cw2218_getvoltage
 *
 * Description:
 *   Read the VCELL register and scale the returned value.
 *
 * Input Parameters:
 *   priv    - Device struct
 *   voltage - Voltage value
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static inline int cw2218_getvoltage(FAR struct cw2218_dev_s *priv,
                                    b16_t *voltage)
{
  uint8_t buffer[2];
  int ret;

  ret = cw2218_getreg8(priv, CW2218_COMMAND_VCELL, buffer, 2);
  if (ret == OK)
    {
      *voltage = ((buffer[0] << 8) | buffer[1]) * CW2218_VOL_MAGIC_PART1 /
         CW2218_VOL_MAGIC_PART2;
    }

  return ret;
}

/****************************************************************************
 * Name: cw2218_getsoc
 *
 * Description:
 *   Read the SOC register and scale the returned value.
 *
 * Input Parameters:
 *   priv    - Device struct
 *   soc     - Soc value
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static inline int cw2218_getsoc(FAR struct cw2218_dev_s *priv,  b16_t *soc,
                                ub8_t *soc_h)
{
  uint8_t buffer[2];
  int ui_soc;
  int ui_100 = CW2218_UI_FULL;
  int ret;

  ret = cw2218_getreg8(priv, CW2218_COMMAND_SOC, buffer, 2);
  if (ret == OK)
    {
      ui_soc = ((buffer[0] * CW2218_SOC_MAGIC_BASE + buffer[1]) *
        CW2218_SOC_MAGIC_100) / (ui_100 * CW2218_SOC_MAGIC_BASE);

      if (ui_soc > CW2218_SOC_MAGIC_100)
        {
          batinfo("CW2018 : UI_SOC = %d larger 100!\n", ui_soc);
          ui_soc = CW2218_SOC_MAGIC_100;
        }

      *soc = ui_soc;
      *soc_h = buffer[0];
    }

  return ret;
}

/****************************************************************************
 * Name: get_complement_code
 *
 * Description:
 *   Get complement code function, unsigned short must be U16.
 *
 * Input Parameters:
 *   raw_code    - Register raw value.
 *
 * Returned Value:
 *   Return the current value with on sign big.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static long get_complement_code(unsigned short raw_code)
{
  long complement_code;
  int dir;

  if (0 != (raw_code & COMPLEMENT_CODE_U16))
    {
      dir = -1;
      raw_code = (~raw_code) + 1;
    }
  else
    {
      dir = 1;
    }

  complement_code = (long)raw_code * dir;
  return complement_code;
}

/****************************************************************************
 * Name: cw2218_getcurrent
 *
 * Description:
 *   Read the CURRENT register and scale the returned value.
 *
 * Input Parameters:
 *   priv    - Device struct
 *   current - Current value
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static inline int cw2218_getcurrent(FAR struct cw2218_dev_s *priv,
                                    b16_t *current)
{
  uint8_t buffer[2];
  ub16_t regval;
  b16_t cw_current;
  int ret;

  ret = cw2218_getreg8(priv, CW2218_COMMAND_CURRENT, buffer, 2);
  if (ret == OK)
    {
      regval = ((buffer[0] << 8) | buffer[1]);
      cw_current = get_complement_code(regval);
      cw_current = cw_current * CW2218_CUR_MAGIC_PART1 /
        CW2218_CUR_MAGIC_PART2 / CW2218_RSENSE / CW2218_CUR_MAGIC_PART3;
      *current = cw_current;
    }

  return ret;
}

/****************************************************************************
 * Name: cw2218_gettemp
 *
 * Description:
 *   Read the TEMP register and scale the returned value.
 *
 * Input Parameters:
 *   priv    - Device struct
 *   temp    - Temperature value
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static inline int cw2218_gettemp(FAR struct cw2218_dev_s *priv, b8_t *temp)
{
  uint8_t regval = 0;
  b8_t cw_temp;
  int ret = -1;

  ret = cw2218_getreg8(priv, CW2218_COMMAND_TEMP, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }
  else
    {
      if ((regval >= CW2218_REG_TEMPMIN_DEFAULT_VALUE) &&
         (regval <= CW2218_REG_TEMPMAX_DEFAULT_VALUE))
        {
          cw_temp = (int)regval * CW2218_TEMP_MAGIC_PART1 /
            CW2218_TEMP_MAGIC_PART2 - CW2218_TEMP_MAGIC_PART3;
          *temp = cw_temp;
        }
      else /* error occurred */
        {
          baterr("get abnormal value:%d\n", regval);
          *temp = CW2218_TEMP_ERROR_DEFAULT_VALUE;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: cw2218_active
 *
 * Description:
 *   Active the cw2218 after startup.
 *
 * Input Parameters:
 *   priv    - Device struct
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int cw2218_active(FAR struct cw2218_dev_s *priv)
{
  uint8_t reg_val = CONFIG_MODE_RESTART;
  int ret;

  ret = cw2218_putreg8(priv, CW2218_COMMAND_CONFIG, reg_val);

  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(CW2218_SLEEP_20MS);     /* Here delay must >= 20 ms */
  reg_val = CONFIG_MODE_ACTIVE;
  ret = cw2218_putreg8(priv, CW2218_COMMAND_CONFIG, reg_val);
  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(CW2218_SLEEP_10MS);
  return OK;
}

/****************************************************************************
 * Name: cw2218_sleep
 *
 * Description:
 *   Make cw2218 enter sleep mode
 *
 * Input Parameters:
 *   priv    - Device struct
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int cw2218_sleep(FAR struct cw2218_dev_s *priv)
{
  uint8_t reg_val = CONFIG_MODE_RESTART;
  int ret;

  ret = cw2218_putreg8(priv, CW2218_COMMAND_CONFIG, reg_val);

  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(CW2218_SLEEP_20MS); /* Here delay must >= 20 ms */
  reg_val = CONFIG_MODE_SLEEP;
  ret = cw2218_putreg8(priv, CW2218_COMMAND_CONFIG, reg_val);
  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(CW2218_SLEEP_10MS);
  return OK;
}

/****************************************************************************
 * Name: cw2218_write_profile
 *
 * Description:
 *   Write battery profile to BATINFO reg
 *
 * Input Parameters:
 *   priv    - Device struct
 *   buf[]   - Array of  unsigned char const type
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int cw2218_write_profile(FAR struct cw2218_dev_s *priv,
                                unsigned char const buf[])
{
  int ret;
  int i;

  for (i = 0; i < SIZE_OF_PROFILE; i++)
    {
      ret = cw2218_putreg8(priv, CW2218_COMMAND_BATINFO + i, buf[i]);

      if (ret < 0)
        {
          baterr("wirte profile err:%d\n", ret);
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: cw2218_get_state
 *
 * Description:
 *   Get the cw2218 running stat
 *    Determine whether the profile needs to be updated.
 *
 * Input Parameters:
 *   priv    - Device struct
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int cw2218_get_state(FAR struct cw2218_dev_s *priv)
{
  unsigned char reg_val;
  int i;
  int ret;

  ret = cw2218_getreg8(priv, CW2218_COMMAND_CONFIG, &reg_val, 1);
  if (ret < 0)
    {
      return ret;
    }

  if (reg_val != CONFIG_MODE_ACTIVE)
    {
      return CW2218_NOT_ACTIVE;
    }

  ret = cw2218_getreg8(priv, CW2218_COMMAND_SOC_ALERT, &reg_val, 1);
  if (ret < 0)
    {
      return ret;
    }

  if (0x00 == (reg_val & CONFIG_UPDATE_FLG))
    {
      return CW2218_PROFILE_NOT_READY;
    }

  for (i = 0; i < SIZE_OF_PROFILE; i++)
    {
      ret = cw2218_getreg8(priv, (CW2218_COMMAND_BATINFO + i), &reg_val, 1);
      if (ret < 0)
        {
          return ret;
        }

      if (g_config_profile_info[i] != reg_val)
        {
          break;
        }
    }

  if (i != SIZE_OF_PROFILE)
    {
      return CW2218_PROFILE_NEED_UPDATE;
    }

  return OK;
}

/****************************************************************************
 * Name: cw2218_config_start_ic
 *
 * Description:
 *   CW2218 update profile function, often called during
 * initialization.
 *
 * Input Parameters:
 *   priv    - Device struct
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int cw2218_config_start_ic(FAR struct cw2218_dev_s *priv)
{
  ub8_t soc_h;
  int count = 0;
  int i;
  int ret = 0;
  b16_t voltage;
  b16_t soc;

  ret = cw2218_sleep(priv);
  if (ret < 0)
    {
      baterr("sleep err:%d\n", ret);
      return ret;
    }

  /* update new battery info */

  ret = cw2218_write_profile(priv, g_config_profile_info);
  if (ret < 0)
    {
      return ret;
    }

  ret = cw2218_active(priv);
  if (ret < 0)
    {
      baterr("active err:%d\n", ret);
      return ret;
    }

  while (1)
    {
      nxsig_usleep(CW2218_SLEEP_100MS);
      ret = cw2218_getvoltage(priv, &voltage);
      if (ret < 0)
        {
          baterr("get voltage err:%d\n", ret);
          return ret;
        }

      ret = cw2218_getsoc(priv, &soc, &soc_h);
      if (ret < 0)
        {
          baterr("get soc err:%d\n", ret);
          return ret;
        }

      if ((voltage != 0) && (soc_h == 0xff))
        {
          break;
        }

      count++;
      if (count >= CW2218_SLEEP_COUNTS)
        {
          ret = cw2218_sleep(priv);
          if (ret < 0)
            {
              baterr("sleep err:%d\n", ret);
              return ret;
            }

          return -CW2218_NOT_ACTIVE;
        }
    }

  for (i = 0; i < CW2218_SLEEP_COUNTS_SOC; i++)
    {
      nxsig_usleep(CW2218_SLEEP_100MS);
      ret = cw2218_getsoc(priv, &soc, &soc_h);
      if (ret < 0)
        {
          baterr("get soc err:%d\n", ret);
          return ret;
        }

      if (soc_h <= CW2218_SOC_MAGIC_100)
        {
          break;
        }
    }

  if (i >= CW2218_SLEEP_COUNTS_SOC)
    {
      cw2218_sleep(priv);
      if (ret < 0)
        {
          baterr("sleep err:%d\n", ret);
          return ret;
        }

      return -CW2218_NOT_ACTIVE;
    }

  return ret;
}

static int cw2218_config_interrupt(FAR struct cw2218_dev_s *priv)
{
  unsigned char reg_val;
  int ret;

  /* enable all interruptes */

  reg_val = 0x70;
  ret = cw2218_putreg8(priv, CW2218_COMMAND_INT_CONFIG, reg_val);
  if (ret < 0)
    {
      return ret;
    }

  reg_val = CONFIG_UPDATE_FLG | GPIO_SOC_IRQ_VALUE;
  ret = cw2218_putreg8(priv, CW2218_COMMAND_SOC_ALERT, reg_val);
  if (ret < 0)
    {
      return ret;
    }

  reg_val = TEMP_MAX_INT_VALUE;
  ret = cw2218_putreg8(priv, CW2218_COMMAND_TEMP_MAX, reg_val);
  if (ret < 0)
    {
      return ret;
    }

  reg_val = TEMP_MIN_INT_VALUE;
  ret = cw2218_putreg8(priv, CW2218_COMMAND_TEMP_MIN, reg_val);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: cw2218_worker
 *
 * Description:
 *   Polling the battery gauge data, according to 5s Frequency
 *
 * Input Parameters
 *   priv    - Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void cw2218_worker(FAR void *arg)
{
  FAR struct cw2218_dev_s *priv = arg;
  int ret;
  int capacity;
  b16_t cap;
#ifdef CONFIG_FACTEST_CHARGE
  b16_t current;
#endif
  b8_t batt_temp;

  ret = cw2218_capacity((struct battery_gauge_dev_s *)priv, &cap);
  if (ret < 0)
    {
      baterr("work get cap err:%d\n", ret);
    }

  capacity = cap;
  if (capacity != priv->last_cap)
    {
      battery_gauge_changed(&priv->dev, BATTERY_CAPACITY_CHANGED);
    }

  ret = cw2218_gettemp(priv, &batt_temp);
  if (ret < 0)
    {
      baterr("work get temp err:%d\n", ret);
    }

  if (priv->last_batt_temp != batt_temp)
    {
      priv->last_batt_temp = batt_temp;
      battery_gauge_changed(&priv->dev, BATTERY_TEMPERATURE_CHANGED);
    }

#ifdef CONFIG_FACTEST_CHARGE
  ret =  cw2218_getcurrent(priv, &current);
  if (ret < 0)
    {
      baterr("work get current err:%d\n", ret);
    }
  else
    {
      battery_gauge_changed(&priv->dev, BATTERY_CURRENT_CHANGED);
    }
#endif
}

/****************************************************************************
 * Name: cw2218_init
 *
 * Description:
 *   CW2218 init function, Often called during initialization.
 *
 * Input Parameters:
 *   priv    - Device struct
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int cw2218_init(FAR struct cw2218_dev_s *priv)
{
  unsigned int id = 0;
  int ret;

  ret = cw2218_get_chipid(priv, &id);
  if (ret < 0)
    {
      baterr("read id err:%d\n", ret);
      return ret;
    }

  if (id != CW2218_DEVICE_ID)
    {
      baterr("device id mismatch\n");
      return ERROR;
    }

  ret = cw2218_get_state(priv);
  if (ret < 0)
    {
      baterr("get state err:%d\n", ret);
      return ret;
    }

  if ((ret == CW2218_NOT_ACTIVE) || (ret == CW2218_PROFILE_NOT_READY) \
      || (ret == CW2218_PROFILE_NEED_UPDATE))
    {
      ret = cw2218_config_start_ic(priv);
      if (ret < 0)
        {
          baterr("config start ic err:%d\n", ret);
          return ret;
        }
    }

  ret = cw2218_config_interrupt(priv);
  if (ret < 0)
    {
      baterr("get config interrupt err:%d\n", ret);
      return ret;
    }

  cw2218_start_soc_full_work(priv);

  work_queue(HPWORK, &priv->work, cw2218_worker, priv, 0);

  return OK;
}

/****************************************************************************
 * Name: cw2218_state
 *
 * Description:
 *   Return the battery state.
 *
 * Input Parameters:
 *   priv    - Device struct
 *   status  - Status
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int cw2218_state(struct battery_gauge_dev_s *dev, int *status)
{
  return OK;
}

/****************************************************************************
 * Name: cw2218_capacity
 *
 * Description:
 *   Current battery capacity.
 *
 * Input Parameters:
 *   priv    - Device struct
 *   capacity  - Soc value
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int cw2218_capacity(struct battery_gauge_dev_s *dev,  b16_t *capacity)
{
  FAR struct cw2218_dev_s *priv = (FAR struct cw2218_dev_s *)dev;
  ub8_t soc_h;
  return cw2218_getsoc(priv, capacity, &soc_h);
}

/****************************************************************************
 * Name: cw2218_voltage
 *
 * Description:
 *   Current battery voltage(mV).
 *
 * Input Parameters:
 *   priv    - Device struct
 *   voltage  - Voltage value to be obtained
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int cw2218_voltage(struct battery_gauge_dev_s *dev, b16_t *voltage)
{
  FAR struct cw2218_dev_s *priv = (FAR struct cw2218_dev_s *)dev;
  return cw2218_getvoltage(priv, voltage);
}

/****************************************************************************
 * Name: cw2218_current
 *
 * Description:
 *   Battery current (mA).
 *
 * Input Parameters:
 *   priv    - Device struct
 *   current  - Current value to be obtained
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int cw2218_current(struct battery_gauge_dev_s *dev,
                          b16_t *current)
{
  FAR struct cw2218_dev_s *priv = (FAR struct cw2218_dev_s *)dev;
  return cw2218_getcurrent(priv,  current);
}

/****************************************************************************
 * Name: cw2218_temp
 *
 * Description:
 *   Battery temperature(0.1 摄氏度)
 *
 * Input Parameters:
 *   priv    - Device struct
 *   temp  - temp value to be obtained
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int cw2218_temp(struct battery_gauge_dev_s *dev, b8_t *temp)
{
  FAR struct cw2218_dev_s *priv = (FAR struct cw2218_dev_s *)dev;
  return cw2218_gettemp(priv,  temp);
}

/****************************************************************************
 * Name: cw2218_chipid
 *
 * Description:
 *   Battery gauge cw2218 chip id.
 *
 * Input Parameters:
 *   priv    - Device struct
 *   chipid  - id
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int cw2218_chipid(struct battery_gauge_dev_s *dev,
                         unsigned int *chipid)
{
  FAR struct cw2218_dev_s *priv = (FAR struct cw2218_dev_s *)dev;
  return cw2218_get_chipid(priv,  chipid);
}

/****************************************************************************
 * Name: cw2218_online
 *
 * Description:
 *   Return true if the batter is online
 *
 * Input Parameters:
 *   priv    - Device struct
 *   temp  - temp value to be obtained
 *
 * Returned Value:
 *    Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int cw2218_online(struct battery_gauge_dev_s *dev, bool *status)
{
  /* There is no concept of online/offline in this driver */

  FAR struct cw2218_dev_s *priv = (FAR struct cw2218_dev_s *)dev;

  *status = priv->gauge_init_status;

  return OK;
}

/****************************************************************************
 * Name: cw2218_interrupt_handler
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

static int cw2218_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                    ioe_pinset_t pinset, FAR void *arg)
{
  FAR struct cw2218_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  work_queue(LPWORK, &priv->work, cw2218_interrupt_worker, priv, 0);

  return OK;
}

static void cw2218_start_soc_full_work(FAR struct cw2218_dev_s *priv)
{
  ub8_t soc_h;
  b16_t soc;
  int ret = 0;

  ret = cw2218_getsoc(priv, &soc, &soc_h);
  if (ret < 0)
    {
      baterr("get soc err:%d\n", ret);
    }

  if (soc >= 99 && !priv->full_work_exit)
    {
      priv->full_work_exit = FULL_WORK_EXIT;
      work_queue(LPWORK, &priv->soc_full_work, soc_change_worker, priv,
                 CW2218_UI_FULL_START_TIME / USEC_PER_TICK);
    }
}

static void soc_change_worker(FAR void *arg)
{
  FAR struct cw2218_dev_s *priv = arg;
  int ret;
  int capacity;
  b16_t cap;

  ret = cw2218_capacity((struct battery_gauge_dev_s *)priv, &cap);
  if (ret < 0)
    {
      baterr("work get cap err:%d\n", ret);
    }

  capacity = cap;
  if (capacity != priv->last_cap)
    {
      priv->last_cap = capacity;
      battery_gauge_changed(&priv->dev, BATTERY_CAPACITY_CHANGED);
    }

  if (capacity <= 98)
    {
      priv->full_work_exit = FULL_WORK_NO_EXIT;
      work_cancel(LPWORK, &priv->soc_full_work);
    }
  else
    {
      work_queue(LPWORK, &priv->soc_full_work, soc_change_worker, priv,
                 CW2218_UI_FULL_TIME / USEC_PER_TICK);
    }
}

/****************************************************************************
 * Name: cw2218_interrupt_worker
 *
 * Description:
 *   Polling the battery gauge data, when interrupt occur
 *
 * Input Parameters
 *   priv    - Device struct
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void cw2218_interrupt_worker(FAR void *arg)
{
  FAR struct cw2218_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  battery_gauge_changed(&priv->dev, BATTERY_CAPACITY_CHANGED);
  battery_gauge_changed(&priv->dev, BATTERY_TEMPERATURE_CHANGED);

  cw2218_start_soc_full_work(priv);
}

static int cw2218_init_interrupt(FAR struct cw2218_dev_s *priv)
{
  int ret;
  int *ioephanle;

  /* Interrupt pin */

  ret = IOEXP_SETDIRECTION(priv->ioe, priv->pin,
                           IOEXPANDER_DIRECTION_IN);
  if (ret < 0)
    {
      baterr("err to set dir:%d\n", ret);
    }

  ioephanle = IOEP_ATTACH(priv->ioe, priv->pin,
                          cw2218_interrupt_handler, priv);
  if (!ioephanle)
    {
      baterr("err attach cw2218_interrupt\n");
      ret = -EIO;
    }

  ret = IOEXP_SETOPTION(priv->ioe, priv->pin,
                        IOEXPANDER_OPTION_INTCFG,
                        (FAR void *)IOEXPANDER_VAL_FALLING);
  if (ret < 0)
    {
      baterr("err set option:%d\n", ret);
      IOEP_DETACH(priv->ioe, cw2218_interrupt_handler);
    }

  return ret;
}

/****************************************************************************
 * Name: gauge_init_thread
 *
 * Description:
 * The battery gauge init detect, according to 5s Frequency
 *
 * Input Parameters
 *   priv    - Device struct
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int gauge_init_thread(int argc, char** argv)
{
  FAR struct cw2218_dev_s *priv = (FAR struct cw2218_dev_s *)
             ((uintptr_t)strtoul(argv[1], NULL, 0));
  uint8_t regval = 0;
  int ret;

  while (1)
    {
      nxsig_usleep(CW2218_BATTERY_INIT_5S);
      ret = cw2218_init(priv);
      if (ret < 0)
        {
          baterr("gauge init thread runing\n");
          priv->gauge_init_status = false;
        }
      else
        {
          ret = cw2218_getreg8(priv, CW2218_COMMAND_TEMP, &regval, 1);
          if (ret < 0)
            {
              return ret;
            }

          if (regval != 0)
            {
              baterr("gauge init success\n");
              priv->gauge_init_status = true;
              break;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cw2218_initialize
 *
 * Description:
 *   Initialize the cw2218 battery driver and return an instance of the
 *   lower_half interface that may be used with battery_register();
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY - Upper half battery driver support
 *   CONFIG_I2C - I2C support
 *   CONFIG_CW2218 - And the driver must be explicitly selected.
 *
 * Input Parameters:
 *   i2c - An instance of the I2C interface to use to communicate with the bq
 *   addr - The I2C address of the cw2218 (Better be 0x64).
 *   frequency - The I2C frequency
 *
 * Returned Value:
 *   A pointer to the initializeed lower-half driver instance. A NULL
 *   pointer is returned on a failure to initialize the cw2218 lower half.
 *
 ****************************************************************************/

FAR struct battery_gauge_dev_s *cw2218_initialize(
                                FAR struct i2c_master_s *i2c,
                                FAR struct ioexpander_dev_s *dev,
                                uint8_t addr,
                                uint32_t frequency,
                                int int_pin)
{
  FAR struct cw2218_dev_s *priv;
  FAR char *argv[2];
  char arg1[32];
  int ret;

  /* Initialize the cw2218 device structure */

  priv = (FAR struct cw2218_dev_s *)kmm_zalloc(sizeof(struct cw2218_dev_s));
  if (!priv)
    {
      baterr("err alloc instance\n");
      goto err;
    }

  priv->dev.ops   = &g_cw2218ops;
  priv->i2c       = i2c;
  priv->addr      = addr;
  priv->frequency = frequency;
  priv->ioe       = dev;
  priv->pin       = int_pin;
  priv->last_cap  = -1;
  priv->last_batt_temp  = -1;
  priv->gauge_init_status = true;
  priv->full_work_exit = FULL_WORK_NO_EXIT;

  ret = cw2218_init_interrupt(priv);
  if (ret < 0)
    {
      baterr("err init interrupt:%d\n", ret);
    }

  ret = cw2218_init(priv);
  if (ret < 0)
    {
      snprintf(arg1, 32, "%p", priv);
      argv[0] = arg1;
      argv[1] = NULL;
      priv->gauge_init_status = false;
      ret = kthread_create("battery_init_thread",
                SCHED_PRIORITY_DEFAULT, CONFIG_DEFAULT_TASK_STACKSIZE,
                gauge_init_thread, argv);
      if (ret < 0)
        {
          baterr("err create gauge init thread\n");
          priv->gauge_init_status = false;
          goto err;
        }
    }

  return (FAR struct battery_gauge_dev_s *)priv;

err:
  kmm_free(priv);
  return NULL;
}

#endif /* CONFIG_BATTERY && CONFIG_I2C && CONFIG_CW2218 */

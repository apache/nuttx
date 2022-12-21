/****************************************************************************
 * drivers/leds/lp503x.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/leds/lp503x.h>

#if defined(CONFIG_I2C) && defined(CONFIG_LP503X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_LP503X
#  define lp503x_err(x, ...)        _err(x, ##__VA_ARGS__)
#  define lp503x_info(x, ...)       _info(x, ##__VA_ARGS__)
#else
#  define lp503x_err(x, ...)        uerr(x, ##__VA_ARGS__)
#  define lp503x_info(x, ...)       uinfo(x, ##__VA_ARGS__)
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

enum lp503x_state
{
  LP503X_STATE_UNINIT = 0,
  LP503X_STATE_RESET,
  LP503X_STATE_CONFIGURED,
};

struct lp503x_dev_s
{
  struct  i2c_master_s  *i2c;
  uint8_t               i2c_addr;
  int                   i2c_freq;
  int                   count;

  /* device configuration/setup data */

  struct lp503x_config_s *lp503x_config;

  /* current state of the lp503x device */

  enum lp503x_state state;
};

/* A set of default config parameters as set in LED driver Kconfig */

struct lp503x_config_s config_default =
{
#ifdef CONFIG_LP503X_LOG_MODE
  .enable_log_mode          = 1,
#else
  .enable_log_mode          = 0,
#endif
#ifdef CONFIG_LP503X_POWER_SAVE
  .enable_power_save        = 1,
#else  
  .enable_power_save        = 0,
#endif
#ifdef CONFIG_LP503X_DITHER_MODE
  .enable_pwm_dithering     = 1,
#else
  .enable_pwm_dithering     = 0,
#endif
#ifdef CONFIG_LP503X_MAX_CURRENT
  .set_max_current_35ma     = 1,
#else
  .set_max_current_35ma     = 0,
#endif
#ifdef CONFIG_LP503X_GLOBAL_SHUTDOWN
  .enable_all_led_shutdown  = 1,
#else
  .enable_all_led_shutdown  = 0,
#endif

  /* all leds will default to independent control, not bank control */

  .led_mode[0]  = LP503X_LED_BANK_MODE_DISABLED,
  .led_mode[1]  = LP503X_LED_BANK_MODE_DISABLED,
  .led_mode[2]  = LP503X_LED_BANK_MODE_DISABLED,
  .led_mode[3]  = LP503X_LED_BANK_MODE_DISABLED,
  .led_mode[4]  = LP503X_LED_BANK_MODE_DISABLED,
  .led_mode[5]  = LP503X_LED_BANK_MODE_DISABLED,
  .led_mode[6]  = LP503X_LED_BANK_MODE_DISABLED,
  .led_mode[7]  = LP503X_LED_BANK_MODE_DISABLED,
  .led_mode[8]  = LP503X_LED_BANK_MODE_DISABLED,
  .led_mode[9]  = LP503X_LED_BANK_MODE_DISABLED,
  .led_mode[10] = LP503X_LED_BANK_MODE_DISABLED,
  .led_mode[11] = LP503X_LED_BANK_MODE_DISABLED,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lp503x_i2c_write_reg(struct lp503x_dev_s *priv,
                                uint8_t const reg_addr,
                                uint8_t const reg_val);
static int lp503x_i2c_read_reg(struct lp503x_dev_s *priv,
                               uint8_t const reg_addr,
                               uint8_t *regval);
static int lp503x_open(struct file *filep);
static int lp503x_close(struct file *filep);
static int lp503x_ioctl(struct file *filep, int cmd,
                        unsigned long arg);
#ifdef CONFIG_DEBUG_LP503X
static int lp503x_dump_registers(struct lp503x_dev_s *priv,
                                 const char *msg);
#else
#  define lp503x_dump_registers(priv, msg);
#endif 
static int lp503x_reset(struct lp503x_dev_s *priv);
static int lp503x_enable(struct lp503x_dev_s *priv, bool enable);
static int lp503x_set_rgbbrightness(struct lp503x_dev_s *priv, int led,
                                    int brightness);
static int lp503x_set_outcolour(struct lp503x_dev_s *priv, int led,
                                int colour);
static int lp503x_set_config(struct lp503x_dev_s *priv);
static int lp503x_set_bank_mode(struct lp503x_dev_s *priv);
static int lp503x_set_bank_colour(struct lp503x_dev_s *priv, char bank,
                                  int brightness);
static int lp503x_set_bank_brightness(struct lp503x_dev_s *priv,
                                      int brightness);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_lp503x_fileops =
{
  lp503x_open,               /* open */
  lp503x_close,              /* close */
  NULL,                      /* read */
  NULL,                      /* write */
  NULL,                      /* seek */
  lp503x_ioctl,              /* ioctl */
  NULL                       /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                     /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lp503x_dumpregs
 *
 * Description:
 *   Dump the contents of all lp503x registers
 *
 * Input Parameters:
 *   priv - A reference to the lp503x peripheral state
 *   msg  - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_LP503X
static int lp503x_dump_registers(struct lp503x_dev_s *priv,
                                 const char *msg)
{
  uint8_t val1;
  uint8_t val2;
  uint8_t val3;
  uint8_t val4;
  int ret;
  lp503x_info("lp503x Registers: %s\n", msg);
  ret = lp503x_i2c_read_reg(priv, LP503X_DEVICE_CONFIG0,   &val1);
  ret = lp503x_i2c_read_reg(priv, LP503X_DEVICE_CONFIG1,   &val2);
  ret = lp503x_i2c_read_reg(priv, LP503X_LED_CONFIG0,      &val3);
  ret = lp503x_i2c_read_reg(priv, LP503X_LED_CONFIG1,      &val4);
  lp503x_info
    ("Dev Config0:\t%02x Dev Conf1:\t%02x \
      LED Conf0:\t%02x: LED Conf1: \t %02x\n",
      val1, val2, val3, val4);

  ret = lp503x_i2c_read_reg(priv, LP503X_BANK_BRIGHTNESS,  &val1);
  ret = lp503x_i2c_read_reg(priv, LP503X_BANK_A_COLOUR,    &val2);
  ret = lp503x_i2c_read_reg(priv, LP503X_BANK_B_COLOUR,    &val3);
  ret = lp503x_i2c_read_reg(priv, LP503X_BANK_C_COLOUR,    &val4);
  lp503x_info
    ("Bank Bright:\t%02x BankA Col:\t%02x \
      BankB Col:\t%02x: BankC Col:\t %02x\n",
      val1, val2, val3, val4);

  ret = lp503x_i2c_read_reg(priv, LP503X_LED0_BRIGHTNESS,  &val1);
  ret = lp503x_i2c_read_reg(priv, LP503X_LED1_BRIGHTNESS,  &val2);
  ret = lp503x_i2c_read_reg(priv, LP503X_LED2_BRIGHTNESS,  &val3);
  ret = lp503x_i2c_read_reg(priv, LP503X_LED3_BRIGHTNESS,  &val4);
  lp503x_info
    ("LED0 Bright:\t%02x LED1 Col:\t%02x \
      LED2 Bright:\t%02x: LED3 Bright: %02x\n",
      val1, val2, val3, val4);

  ret = lp503x_i2c_read_reg(priv, LP503X_LED4_BRIGHTNESS,  &val1);
  ret = lp503x_i2c_read_reg(priv, LP503X_LED5_BRIGHTNESS,  &val2);
  ret = lp503x_i2c_read_reg(priv, LP503X_LED6_BRIGHTNESS,  &val3);
  ret = lp503x_i2c_read_reg(priv, LP503X_LED7_BRIGHTNESS,  &val4);
  lp503x_info
    ("LED4 Bright:\t%02x LED5 Bright:\t%02x \
      LED6 Bright:\t%02x: LED7 Bright: %02x\n",
      val1, val2, val3, val4);

  ret = lp503x_i2c_read_reg(priv, LP503X_LED8_BRIGHTNESS,  &val1);
  ret = lp503x_i2c_read_reg(priv, LP503X_LED9_BRIGHTNESS,  &val2);
  ret = lp503x_i2c_read_reg(priv, LP503X_LED10_BRIGHTNESS, &val3);
  ret = lp503x_i2c_read_reg(priv, LP503X_LED11_BRIGHTNESS, &val4);
  lp503x_info
    ("LED8 Bright:\t%02x LED9 Bright:\t%02x \
      LED10 Bright:\t%02x: LED11 Bright:%02x\n",
      val1, val2, val3, val4);

  ret = lp503x_i2c_read_reg(priv, LP503X_OUT0_COLOUR,      &val1);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT1_COLOUR,      &val2);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT2_COLOUR,      &val3);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT3_COLOUR,      &val4);
  lp503x_info
    ("Out0 Col:\t%02x Out1 Col:\t%02x \
      Out2 Col:\t\t%02x  Out3 Col:\t %02x\n",
      val1, val2, val3, val4);

  ret = lp503x_i2c_read_reg(priv, LP503X_OUT4_COLOUR,      &val1);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT5_COLOUR,      &val2);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT6_COLOUR,      &val3);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT7_COLOUR,      &val4);
  lp503x_info
    ("Out4 Col:\t%02x Out5 Col:\t%02x \
      Out6 Col:\t\t%02x  Out7 Col:\t %02x\n",
      val1, val2, val3, val4);

  ret = lp503x_i2c_read_reg(priv, LP503X_OUT8_COLOUR,      &val1);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT9_COLOUR,      &val2);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT10_COLOUR,     &val3);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT11_COLOUR,     &val4);
  lp503x_info
     ("Out8 Col:\t%02x Out9 Col:\t%02x \
       Out10 Col:\t%02x  Out11 Col:\t %02x\n",
       val1, val2, val3, val4);

  ret = lp503x_i2c_read_reg(priv, LP503X_OUT12_COLOUR,     &val1);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT13_COLOUR,     &val2);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT14_COLOUR,     &val3);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT15_COLOUR,     &val4);
  lp503x_info
    ("Out12 Col:\t%02x Out13 Col:\t%02x \
      Out14 Col:\t%02x  Out15 Col:\t %02x\n",
      val1, val2, val3, val4);

  ret = lp503x_i2c_read_reg(priv, LP503X_OUT16_COLOUR,     &val1);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT17_COLOUR,     &val2);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT18_COLOUR,     &val3);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT19_COLOUR,     &val4);
  lp503x_info
    ("Out16 Col:\t%02x Out17 Col:\t%02x \
      Out18 Col:\t%02x  Out19 Col:\t %02x\n",
      val1, val2, val3, val4);

  ret = lp503x_i2c_read_reg(priv, LP503X_OUT20_COLOUR,     &val1);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT21_COLOUR,     &val2);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT22_COLOUR,     &val3);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT23_COLOUR,     &val4);
  lp503x_info
    ("Out20 Col:\t%02x Out21 Col:\t%02x \
      Out22 Col:\t%02x  Out23 Col:\t %02x\n",
      val1, val2, val3, val4);

  ret = lp503x_i2c_read_reg(priv, LP503X_OUT24_COLOUR,     &val1);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT25_COLOUR,     &val2);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT26_COLOUR,     &val3);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT27_COLOUR,     &val4);
  lp503x_info
    ("Out24 Col:\t%02x Out25 Col:\t%02x \
      Out26 Col:\t%02x  Out27 Col:\t %02x\n",
      val1, val2, val3, val4);

  ret = lp503x_i2c_read_reg(priv, LP503X_OUT28_COLOUR,     &val1);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT29_COLOUR,     &val2);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT30_COLOUR,     &val3);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT31_COLOUR,     &val4);
  lp503x_info
    ("Out28 Col:\t%02x Out29 Col:\t%02x \
      Out30 Col:\t%02x  Out31 Col:\t %02x\n",
      val1, val2, val3, val4);

  ret = lp503x_i2c_read_reg(priv, LP503X_OUT32_COLOUR,     &val1);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT33_COLOUR,     &val2);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT34_COLOUR,     &val3);
  ret = lp503x_i2c_read_reg(priv, LP503X_OUT35_COLOUR,     &val4);
  lp503x_info
    ("Out28 Col:\t%02x Out29 Col:\t%02x \
      Out30 Col:\t%02x  Out31 Col:\t %02x\n",
      val1, val2, val3, val4);

  return ret;
}

#endif
/****************************************************************************
 * Name: lp503x_i2c_write_reg
 *
 * Description:
 *   Write a single byte to one of the LP503X configuration registers.
 *
 ****************************************************************************/

static int lp503x_i2c_write_reg(struct lp503x_dev_s *priv,
                                uint8_t const reg_addr,
                                uint8_t const reg_val)
{
  struct i2c_config_s config;
  int ret;

  /* assemble the 2 byte message comprised of reg_addr and reg_val */

  uint8_t const BUFFER_SIZE = 2;
  uint8_t buffer[BUFFER_SIZE];

  buffer[0] = reg_addr;
  buffer[1] = reg_val;

  /* Setup up the I2C configuration */

  config.frequency = priv->i2c_freq;
  config.address   = priv->i2c_addr;
  config.addrlen   = 7;

  /* Write the register address followed by the data (no RESTART) */

  ledinfo("i2c addr: 0x%02X reg addr: 0x%02X value: 0x%02X\n",
          priv->i2c_addr, buffer[0], buffer[1]);

  ret = i2c_write(priv->i2c, &config, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      lederr("ERROR: i2c_write returned error code %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: lp503x_i2c_read_reg
 *
 * Description:
 *   Read a single byte from one of the LP503X configuration registers.
 *
 ****************************************************************************/

static int lp503x_i2c_read_reg(struct lp503x_dev_s *priv,
                               uint8_t const reg_addr,
                               uint8_t *regval)
{
  struct i2c_config_s config;
  int ret;

  /* Setup up the I2C configuration */

  config.frequency = priv->i2c_freq;
  config.address   = priv->i2c_addr;
  config.addrlen   = 7;

  /* Write the register address followed by the data (no RESTART) */

  ret = i2c_write(priv->i2c, &config, &reg_addr, 1);
  ret = i2c_read(priv->i2c, &config, regval, 1);

  return ret;
}

/****************************************************************************
 * Name: lp503x_open
 *
 * Description:
 *   This function is called whenever a LP503X device is opened.
 *
 ****************************************************************************/

static int lp503x_open(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct lp503x_dev_s *priv = inode->i_private;
  int ret;

  ledinfo("INFO: Opening, resetting and enabling the LP503X for business\n");

  /* reset and enable the device */

  /* means the device was possibly never regsitered? */

  if (priv->state == LP503X_STATE_UNINIT)
    {
      return -ENODEV;
    }
  else if (priv->state == LP503X_STATE_RESET)
    {
      ret = lp503x_enable(priv, true);

      if (ret != 0)
        {
          lederr("ERROR: unable to enable lp503x\n");
          return -EIO;
        }
      else
        {
          /* use device defaults */

          priv->lp503x_config = &config_default;
        }

      ret = lp503x_set_config(priv);
      if (ret != 0)
        {
          lederr("ERROR: Unable to set device config: %d\n", ret);
          return -EIO;
        }

      priv->state = LP503X_STATE_CONFIGURED;
    }

  lp503x_dump_registers(priv, "File Open");

  return ret;
}

/****************************************************************************
 * Name: lp503x_close
 *
 * Description:
 *   This function is called whenever a LP503X device is closed.
 *
 ****************************************************************************/

static int lp503x_close(struct file *filep)
{
  int ret;

  struct inode *inode = filep->f_inode;
  struct lp503x_dev_s *priv = inode->i_private;

  ret = lp503x_enable(priv, false);
  if (ret < 0)
    {
      lederr("ERROR: Could not disable LP503X\n");
    }

  return ret;
}

/****************************************************************************
 * Name: lp503x_reset
 *
 * Description:
 *   Resets all registers to default values
 *
 ****************************************************************************/

static int lp503x_reset(struct lp503x_dev_s *priv)
{
  int ret;

  ret = lp503x_i2c_write_reg(priv, LP503X_RESET, LP503X_RESET_ALL_REGISTERS);
  if (ret != 0)
    {
      return -EIO;
    }
  else
    {
      priv->state = LP503X_STATE_RESET;
      return OK;
    }
}

/****************************************************************************
 * Name: lp503x_enable
 *
 * Description:
 *   enables or disables the entire device
 *
 ****************************************************************************/

static int lp503x_enable(struct lp503x_dev_s *priv, bool enable)
{
  int ret;

  if (enable)
    {
      ret = lp503x_i2c_write_reg(priv, LP503X_DEVICE_CONFIG0,
                                       LP503X_CHIP_ENABLE);
      ledinfo("INFO: LP503x enabled\n");
    }
  else
    {
      ret = lp503x_i2c_write_reg(priv, LP503X_DEVICE_CONFIG0,
                                       LP503X_CHIP_DISABLE);
      ledinfo("INFO: LP503x disabled\n");
    }

  return ret;
}

/****************************************************************************
 * Name: lp503x_set_config
 *
 * Description:
 *   configures basic operation modes of the device
 *
 ****************************************************************************/

static int lp503x_set_config(struct lp503x_dev_s *priv)
{
  int ret;
  uint8_t regval;
  struct lp503x_config_s *config;

  config = priv->lp503x_config;

  ret = lp503x_i2c_read_reg(priv, LP503X_DEVICE_CONFIG1, &regval);
  if (config->enable_log_mode)
    {
      regval |= LP503X_CONFIG1_LOG_SCALE;
    }
  else
    {
      regval &= ~LP503X_CONFIG1_LOG_SCALE;
    }

  if (config->enable_power_save)
    {
      regval |= LP503X_CONFIG1_PWRSAVE;
    }
  else
    {
      regval &= ~LP503X_CONFIG1_PWRSAVE;
    }

  if (config->enable_pwm_dithering)
    {
      regval |= LP503X_CONFIG1_DITHERING;
    }
  else
    {
      regval &= ~LP503X_CONFIG1_DITHERING;
    }

  if (config->set_max_current_35ma)
    {
      regval |= LP503X_CONFIG1_PWRSAVE;
    }
  else
    {
      regval &= ~LP503X_CONFIG1_PWRSAVE;
    }

  if (config->enable_all_led_shutdown)
    {
      regval |= LP503X_CONFIG1_GLOBAL_OFF;
    }
  else
    {
      regval &= ~LP503X_CONFIG1_GLOBAL_OFF;
    }

  ret = lp503x_i2c_write_reg(priv, LP503X_DEVICE_CONFIG1, regval);

  return ret;
}

/****************************************************************************
 * Name: lp503x_set_bank_brightness
 *
 * Description:
 *   sets banks to the  required brightness
 *
 ****************************************************************************/

static int lp503x_set_bank_brightness(struct lp503x_dev_s *priv,
                                      int brightness)
{
  if (brightness > MAX_BRIGHTNESS)
    {
      return -EINVAL;
    }
  else
    {
      return lp503x_i2c_write_reg(priv, LP503X_BANK_BRIGHTNESS, brightness);
    }
}

/****************************************************************************
 * Name: lp503x_set_bank_colour
 *
 * Description:
 *   sets bank A, B or C led to required coloiur (mix)
 *
 ****************************************************************************/

static int lp503x_set_bank_colour(struct lp503x_dev_s *priv, char bank,
                                  int brightness)
{
  if (brightness > MAX_BRIGHTNESS)
    {
      return -EINVAL;
    }
  else
    {
      if (bank == 'A')
        {
          return lp503x_i2c_write_reg(priv, LP503X_BANK_A_COLOUR,
                                      brightness);
        }
      else if (bank == 'B')
        {
          return lp503x_i2c_write_reg(priv, LP503X_BANK_B_COLOUR,
                                      brightness);
        }
      else if (bank == 'C')
        {
          return lp503x_i2c_write_reg(priv, LP503X_BANK_C_COLOUR,
                                      brightness);
        }
      else
        {
          return -EINVAL;
        }
    }
}

/****************************************************************************
 * Name: lp503x_set_bank_mode
 *
 * Description:
 *   enables or disables bank mode for selected LED
 *
 ****************************************************************************/

static int lp503x_set_bank_mode(struct lp503x_dev_s *priv)
{
  int ret;
  int count;
  int regval;

  struct lp503x_config_s *config;

  config = priv->lp503x_config;

  regval = 0;
  for (count = 0; count < 8; count++)
    {
      if (config->led_mode[count] == LP503X_LED_BANK_MODE_ENABLED)
        {
          regval |= (LP503X_LED0_BANK_ENABLE << count);
        }
      else
        {
          regval &= ~(LP503X_LED0_BANK_ENABLE << count);
        }
    }

  ret = lp503x_i2c_write_reg(priv, LP503X_LED_CONFIG0, regval);

  for (count = 8; count < 12; count++)
    {
      if (config->led_mode[count] == LP503X_LED_BANK_MODE_ENABLED)
        {
          regval |= (LP503X_LED0_BANK_ENABLE << (count - 8));
        }
      else
        {
          regval &= ~(LP503X_LED0_BANK_ENABLE << (count - 8));
        }
    }

  ret = lp503x_i2c_write_reg(priv, LP503X_LED_CONFIG1, regval);

  return ret;
}

/****************************************************************************
 * Name: lp503x_set_rgbled_colour
 *
 * Description:
 *   sets RGB led to chosen html colour
 *
 ****************************************************************************/

static int lp503x_set_rgbled_colour(struct lp503x_dev_s *priv,
                                    int led, int colour)
{
  int ret;
  int regaddr;

  if ((led > MAX_RGB_LEDS) || (colour > MAX_RGB_COLOUR))
    {
      ret = -EINVAL;
    }
  else
    {
      regaddr = LP503X_OUT0_COLOUR + (3*led);

      ret = lp503x_i2c_write_reg(priv, regaddr++, (colour >> 16) & 0xff);
      ret = lp503x_i2c_write_reg(priv, regaddr++, (colour >> 8)  & 0xff);
      ret = lp503x_i2c_write_reg(priv, regaddr,   (colour >> 0)  & 0xff);
      ledinfo("INFO: RGB LED %d set to RGB colour %06x\n", led, colour);
    }

  return ret;
}

/****************************************************************************
 * Name: lp503x_set_outcolour
 *
 * Description:
 *   Sets OUT brightness ("colour" of individual LED outputs
 *
 ****************************************************************************/

static int lp503x_set_outcolour(struct lp503x_dev_s *priv, int led,
                                int brightness)
{
  int ret;
  if ((led > MAX_LEDS) || (brightness > MAX_BRIGHTNESS))
    {
      ret = -EINVAL;
    }
  else
    {
      ret = lp503x_i2c_write_reg(priv, LP503X_OUT0_COLOUR + led, brightness);
      ledinfo("INFO: individual LED %d set to brightness %d\n", led,
              brightness);
    }

  return ret;
}

/****************************************************************************
 * Name: lp503x_set_rgbbrightness
 *
 * Description:
 *   Sets brightness of all RGB LED
 *
 ****************************************************************************/

static int lp503x_set_rgbbrightness(struct lp503x_dev_s *priv, int led,
                                    int brightness)
{
  int ret;

  if ((led > MAX_RGB_LEDS) || (brightness > MAX_BRIGHTNESS))
    {
      ret = -EINVAL;
    }
  else
    {
      ret = lp503x_i2c_write_reg(priv, LP503X_LED0_BRIGHTNESS + led,
                                 brightness);
      ledinfo("INFO: LED %d set to brightness %d\n", led, brightness);
    }

  return ret;
}

/****************************************************************************
 * Name: lp503x_ioctl
 *
 * Description:
 *   This function is called whenever an ioctl call to a LP503X is
 *   performed.
 *
 ****************************************************************************/

static int lp503x_ioctl(struct file *filep, int cmd,
                        unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct lp503x_dev_s *priv = inode->i_private;
  struct lp503x_config_s *config;
  int ret;
  const struct ioctl_arg_s *lp503x_ioctl_args = (struct ioctl_arg_s *)arg;

  config = priv->lp503x_config;

  ret = OK;

  ledinfo("cmd: %d arg: %ld\n", cmd, arg);

  switch (cmd)
    {
      case PWMIOC_ENABLE: /* arg is true or false */
        config->enable_all_led_shutdown = lp503x_ioctl_args->param;
        ret = lp503x_set_config(priv);
        break;

      case PWMIOC_RESET:  /* no args */
        lp503x_reset(priv);
        break;

      case PWMIOC_ENABLE_LED_BANK_MODE: /* led(0..11), mode required */
        ledinfo("INFO: setting LED %d mode to %" PRIx32 "\n",
                lp503x_ioctl_args->lednum,
                lp503x_ioctl_args->param);
        config->led_mode[lp503x_ioctl_args->lednum] =
                         lp503x_ioctl_args->param;
        lp503x_set_bank_mode(priv);
        break;

      case PWMIOC_SET_BANK_MIX_COLOUR:/* bank(A/B/C),level(0-255)   */
        ledinfo("INFO: setting bank %c to brightness %" PRIx32 "\n",
                lp503x_ioctl_args->lednum, lp503x_ioctl_args->param);
        ret = lp503x_set_bank_colour(priv, lp503x_ioctl_args->lednum,
                                     lp503x_ioctl_args->param);
        break;

      case PWMIOC_SET_BANK_BRIGHTNESS:
        ledinfo("INFO: setting bank brightness to %" PRIx32 "\n",
                lp503x_ioctl_args->param);
        lp503x_set_bank_brightness(priv, lp503x_ioctl_args->param);
        break;

      case PWMIOC_CONFIG: /* config is struct within priv */
        ledinfo("INFO: setting device config to:\n");
        ledinfo("\tlog mode          = %d\n",  config->enable_log_mode);
        ledinfo("\tpower save        = %d\n",  config->enable_power_save);
        ledinfo("\tpwm dithering     = %d\n",  config->enable_pwm_dithering);
        ledinfo("\tmax current       = %s\n", (config->set_max_current_35ma)
                                        ? "30mA" : "25.5mA");
        ledinfo("\tall leds shutdown = %d\n",
                config->enable_all_led_shutdown);

        ret = lp503x_set_config(priv);
        break;

      case PWMIOC_SET_LED_COLOUR: /* led(0..35), Colour(0..255) */
        ledinfo("INFO: set LED %d to colour/brightness %" PRIx32 "\n",
                lp503x_ioctl_args->lednum, lp503x_ioctl_args->param);
        ret = lp503x_set_outcolour(priv, lp503x_ioctl_args->lednum,
                                   lp503x_ioctl_args->param);
        break;

      case PWMIOC_SET_RGB_BRIGHTNESS: /* led(0..11), level(0..255)  */
        ledinfo("INFO: requested brightness level %d for led %" PRIx32 "\n",
                lp503x_ioctl_args->lednum, lp503x_ioctl_args->param);
        ret = lp503x_set_rgbbrightness(priv, lp503x_ioctl_args->lednum,
                                       lp503x_ioctl_args->param);
        break;

      case PWMIOC_SET_RGB_COLOUR: /* led(0..11)                 */
        ledinfo("requested led %d to be RGB colour = %" PRIx32 "\n",
                lp503x_ioctl_args->lednum, lp503x_ioctl_args->param);
        ret = lp503x_set_rgbled_colour(priv, lp503x_ioctl_args->lednum,
                                       lp503x_ioctl_args->param);
        break;

      default: /* The used ioctl command was invalid */
        lederr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lp503x_register
 *
 * Description:
 *   Register the LP503X device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/leddrv0".
 *   i2c     - An instance of the I2C interface to use to communicate
 *             with the LM92.
 *   lp503x_i2c_addr
 *           - The I2C address of the LP503X.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lp503x_register(const char *devpath, struct i2c_master_s *i2c,
                    uint8_t const lp503x_i2c_addr, int const i2c_frequency)
{
  int ret;

  /* Sanity check */

  DEBUGASSERT(devpath != NULL && i2c != NULL);

  /* Initialize the LP503X device structure */

  struct lp503x_dev_s *priv =
         (struct lp503x_dev_s *)kmm_malloc(sizeof(struct lp503x_dev_s));

  if (priv == NULL)
    {
      lederr("ERROR: Failed to allocate instance of lp503x_dev_s\n");
      return -ENOMEM;
    }

  priv->i2c           = i2c;
  priv->i2c_addr      = lp503x_i2c_addr;
  priv->i2c_freq      = i2c_frequency;
  priv->state         = LP503X_STATE_UNINIT;

  /* Register the character driver */

  ret = register_driver(devpath, &g_lp503x_fileops, 0222, priv);
  if (ret != OK)
    {
      lederr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }
  else
    {
      ret = lp503x_reset(priv);
      if (ret != OK)
        {
          lederr("ERROR: failed to reset lp503x device\n");
          return ret;
        }

      priv->state = LP503X_STATE_RESET;
    }

  return OK;
}

#endif /* CONFIG_I2C && CONFIG_I2C_LP503X */

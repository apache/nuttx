/****************************************************************************
 * drivers/leds/ktd2052.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/leds/ktd2052.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#if defined(CONFIG_I2C) && defined(CONFIG_KTD2052)

/* Register Addresses */
#define KTD2052_ID_REG          0x00
#define KTD2052_MONITOR_REG     0x01
#define KTD2052_CONTROL_REG     0x02
#define KTD2052_IRED1_REG       0x03
#define KTD2052_IGRN1_REG       0x04
#define KTD2052_IBLU1_REG       0x05
#define KTD2052_IRED2_REG       0x06
#define KTD2052_IGRN2_REG       0x07
#define KTD2052_IBLU2_REG       0x08
#define KTD2052_IRED3_REG       0x09
#define KTD2052_IGRN3_REG       0x0a
#define KTD2052_IBLU3_REG       0x0b
#define KTD2052_IRED4_REG       0x0c
#define KTD2052_IGRN4_REG       0x0d
#define KTD2052_IBLU4_REG       0x0e
#define KTD2052_PG_CNTL_REG     0x0f
#define KTD2052_PG_FADE_REG     0x10
#define KTD2052_PG_RGB1_REG     0x11
#define KTD2052_PG_RGB2_REG     0x12
#define KTD2052_PG_RGB3_REG     0x13
#define KTD2052_PG_RGB4_REG     0x14
#define KTD2052_PG_WD_REG       0x15

/* Control Register Definitions */
#define CONTROL_BE_EN           (0x01 << 5)

/* Pattern Generator Control Register Bits */
#define PG_MODE_OFF             (0x00 << 6)
#define PG_MODE_4SLOTS          (0x01 << 6)
#define PG_MODE_6SLOTS          (0x02 << 6)
#define PG_MODE_8SLOTS          (0x03 << 6)

/* Private Data Structure */

struct ktd2052_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t addr;
  uint8_t frequency;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ktd2052_write_regs(FAR struct ktd2052_dev_s *priv,
                              uint8_t reg_addr,
                              const uint8_t *reg_vals,
                              uint8_t count);
static int ktd2052_read_reg(FAR struct ktd2052_dev_s *priv, uint8_t reg_addr,
                            uint8_t *reg_val);
static int ktd2052_write_watchdog(FAR struct ktd2052_dev_s *priv,
                                  uint8_t value);

static int ktd2052_open(FAR struct file *filep);
static int ktd2052_close(FAR struct file *filep);
static ssize_t ktd2052_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t ktd2052_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int ktd2052_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Driver Operations Structure
 ****************************************************************************/

static const struct file_operations g_ktd2052_fops =
{
  ktd2052_open,    /* open */
  ktd2052_close,   /* close */
  ktd2052_read,    /* read */
  ktd2052_write,   /* write */
  NULL,            /* seek */
  ktd2052_ioctl,   /* ioctl */
  NULL             /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ktd2052_write_regs
 *
 * Description:
 *   Write one or more registers to device
 *
 * Input Parameters:
 *   priv     - Driver instance
 *   reg_addr - Starting device register address to write
 *   reg_val  - Pointer to register value(s)
 *   count    - Number of registers to write
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ktd2052_write_regs(FAR struct ktd2052_dev_s *priv,
                              uint8_t reg_addr,
                              const uint8_t *reg_vals,
                              uint8_t count)
{
  struct i2c_msg_s msg[1];
  uint8_t buf[count + 1];
  int ret;

  buf[0] = reg_addr;
  memcpy(&buf[1], reg_vals, count);

  msg[0].frequency = priv->frequency;
  msg[0].addr = priv->addr;
  msg[0].flags = 0;
  msg[0].buffer = buf;
  msg[0].length = count + 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: ktd2052_read_reg
 *
 * Description:
 *   Read a single register from device
 *
 * Input Parameters:
 *   priv     - Driver instance
 *   reg_addr - Device register address to read
 *   reg_val  - Pointer to location where register value will be stored
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ktd2052_read_reg(FAR struct ktd2052_dev_s *priv, uint8_t reg_addr,
                            uint8_t *reg_val)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->frequency;
  msg[0].addr = priv->addr;
  msg[0].flags = 0;
  msg[0].buffer = &reg_addr;
  msg[0].length = 1;

  msg[1].frequency = priv->frequency;
  msg[1].addr = priv->addr;
  msg[1].flags = I2C_M_READ;
  msg[1].buffer = reg_val;
  msg[1].length = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: ktd2052_write_watchdog
 *
 * Description:
 *   Set watchdog timer value
 *
 * Input Parameters:
 *   priv    - Driver instance
 *   value   - watchdog value in number of cycles
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ktd2052_write_watchdog(FAR struct ktd2052_dev_s *priv,
                                  uint8_t value)
{
  int ret;

  /* Watchdog register requires two writes */

  ret = ktd2052_write_regs(priv, KTD2052_PG_WD_REG, &value, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = ktd2052_write_regs(priv, KTD2052_PG_WD_REG, &value, 1);
  return ret;
}

/****************************************************************************
 * Name: ktd2052_initialize
 *
 * Description:
 *   Initialize the KTD2052 device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to communicate with KTD2052
 *   addr    - The I2C address of the KTD2052 (default: 0x74)
 *   freq    - The I2C frequency to use for the KTD2052
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static FAR
struct ktd2052_dev_s *ktd2052_initialize(FAR struct i2c_master_s *i2c,
                                         uint8_t addr, uint32_t freq)
{
  FAR struct ktd2052_dev_s *priv;
  uint8_t reg_val;
  int ret;

  /* Allocate device structure */

  priv = (FAR struct ktd2052_dev_s *)kmm_malloc(
                                              sizeof(struct ktd2052_dev_s));
  if (!priv)
    {
      return NULL;
    }

  /* Initialize structure */

  priv->i2c = i2c;
  priv->addr = addr;
  priv->frequency = freq;

  /* Verify device by reading ID register */

  ret = ktd2052_read_reg(priv, KTD2052_ID_REG, &reg_val);
  if (ret < 0)
    {
      kmm_free(priv);
      return NULL;
    }

  /* Verify vendor ID (bits 7:5 should be 101) */

  if ((reg_val & 0xe0) != 0xa0)
    {
      kmm_free(priv);
      return NULL;
    }

  return priv;
}

/****************************************************************************
 * Name: ktd2052_set_rgb
 *
 * Description:
 *   Set RGB LED color and brightness for a specific module
 *
 * Input Parameters:
 *   priv    - Driver instance
 *   module  - RGB module number (1-4)
 *   rgb     - array of uint8_t; red, green, and blue LED current
 *             (0-192 for 0-24mA)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ktd2052_set_rgb(FAR struct ktd2052_dev_s *priv, uint8_t module,
                           const uint8_t *rgb)
{
  int ret;
  uint8_t base_reg;

  if (module < 1 || module > 4)
    {
      return -EINVAL;
    }

  /* Calculate base register for the module */

  base_reg = KTD2052_IRED1_REG + ((module - 1) * 3);

  /* Write RGB values */

  ret = ktd2052_write_regs(priv, base_reg, rgb, 3);

  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ktd2052_set_mode
 *
 * Description:
 *   Set the operating mode of the device
 *
 * Input Parameters:
 *   priv           - Driver instance
 *   mode           - Operating mode (0=off, 1=night mode, 2=normal mode)
 *   bright_extend  - Enable BrightExtend feature
 *   temp_limit     - Temperature limit (0=135C, 1=120C, 2=105C, 3=90C)
 *   fade_rate      - Fade rate (0-7, slower with increasing value)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ktd2052_set_mode(FAR struct ktd2052_dev_s *priv, uint8_t mode,
                            bool bright_extend, uint8_t temp_limit,
                            uint8_t fade_rate)
{
  uint8_t reg_val = 0;

  /* Validate parameters */

  if (mode > 2 || temp_limit > 3 || fade_rate > 7)
    {
      return -EINVAL;
    }

  /* Build control register value */

  reg_val = (mode << 6) |
            (bright_extend ? CONTROL_BE_EN : 0) |
            (temp_limit << 3) |
            fade_rate;

  return ktd2052_write_regs(priv, KTD2052_CONTROL_REG, &reg_val, 1);
}

/****************************************************************************
 * Name: ktd2052_setup_pattern
 *
 * Description:
 *   Configure the pattern generator
 *
 * Input Parameters:
 *   priv       - Driver instance
 *   slots      - Number of pattern slots (4, 6, or 8)
 *   duration   - Duration for each slot (0-7, longer with increasing value)
 *   fade_rate1 - Secondary fade rate (0-7)
 *   watchdog   - Number of pattern cycles (0-255, 255=infinite)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ktd2052_setup_pattern(FAR struct ktd2052_dev_s *priv,
                                 uint8_t slots,
                                 uint8_t duration,
                                 uint8_t fade_rate1,
                                 uint8_t watchdog)
{
  uint8_t mode;
  uint8_t reg_val;
  int ret;

  /* Convert slots to mode bits */

  switch (slots)
    {
      case 4:
        mode = PG_MODE_4SLOTS;
        break;
      case 6:
        mode = PG_MODE_6SLOTS;
        break;
      case 8:
        mode = PG_MODE_8SLOTS;
        break;
      default:
        return -EINVAL;
        break;
    }

  /* Validate other parameters */

  if (duration > 7 || fade_rate1 > 7)
    {
      return -EINVAL;
    }

  reg_val = mode | (duration << 3) | fade_rate1;

  /* Configure pattern generator control register */

  ret = ktd2052_write_regs(priv, KTD2052_PG_CNTL_REG,
                           &reg_val, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Set watchdog value */

  return ktd2052_write_watchdog(priv, watchdog);
}

/****************************************************************************
 * Name: ktd2052_set_pattern_slots
 *
 * Description:
 *   Configure which RGB modules are active in each pattern slot
 *
 * Input Parameters:
 *   priv     - Driver instance
 *   module   - RGB module number (1-4)
 *   slots    - Bit pattern for active slots (LSB = slot 0)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ktd2052_set_pattern_slots(FAR struct ktd2052_dev_s *priv,
                                     uint8_t module,
                                     uint8_t slots)
{
  if (module < 1 || module > 4)
    {
      return -EINVAL;
    }

  return ktd2052_write_regs(priv,
                            KTD2052_PG_RGB1_REG + (module - 1),
                            &slots, 1);
}

/****************************************************************************
 * Name: ktd2052_open
 ****************************************************************************/

static int ktd2052_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ktd2052_dev_s *priv = inode->i_private;

  /* Verify the device still exists (I2C bus check) */

  uint8_t reg_val;
  int ret = ktd2052_read_reg(priv, KTD2052_ID_REG, &reg_val);
  if (ret < 0)
    {
      return ret;
    }

  /* Set default device configuration */

  ret = ktd2052_set_mode(priv, KTD2052_MODE_NORMAL, true,
                         KTD2052_TEMP_LIMIT_90, 0);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ktd2052_close
 ****************************************************************************/

static int ktd2052_close(FAR struct file *filep)
{
  /* Nothing to do
   * Revisit: should device be put into shutdown mode here?
   */

  return OK;
}

/****************************************************************************
 * Name: ktd2052_read
 *
 * Description:
 *   Read will return the device's status monitor register, as well as the
 *   control register if sufficient buffer space is provided.
 ****************************************************************************/

static ssize_t ktd2052_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ktd2052_dev_s *priv = inode->i_private;
  ssize_t nread = 0;
  int ret;

  if (buflen >= 1)
    {
      /* Read status register */

      ret = ktd2052_read_reg(priv, KTD2052_MONITOR_REG,
                             (uint8_t *)&buffer[0]);
      if (ret < 0)
        {
          return ret;
        }

      nread += 1;
      if (buflen >= 2)
        {
          /* Read control register */

          ret = ktd2052_read_reg(priv, KTD2052_CONTROL_REG,
                                 (uint8_t *)&buffer[1]);
          if (ret < 0)
            {
              return ret;
            }

          nread += 1;
        }
    }

  return nread;
}

/****************************************************************************
 * Name: ktd2052_write
 *
 * Description:
 *   Write will write directly to the RGB values for LED modules 1-4. Partial
 *   writes are allowed.  It expects 1-12 bytes in the format:
 *   [red] [green] [blue] [red] [green] [blue] etc
 ****************************************************************************/

static ssize_t ktd2052_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ktd2052_dev_s *priv = inode->i_private;
  int ret;

  if (buflen <= 0 || buflen > 12)
    {
      return -EINVAL;
    }

  /* Set the RGB values */

  ret = ktd2052_write_regs(priv, KTD2052_IRED1_REG,
                           (const uint8_t *)&buffer[0], buflen);
  if (ret < 0)
    {
      return ret;
    }

  return buflen;
}

/****************************************************************************
 * Name: ktd2052_ioctl
 ****************************************************************************/

static int ktd2052_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ktd2052_dev_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      case KTDIOSETRGB:
        {
          /* Set RGB value of a single module.  Expects pointer to 4-byte
           * uint8_t array (module number and r, g, b values)
           */

          FAR uint8_t *mrgb = (FAR uint8_t *)arg;
          if (mrgb == NULL)
            {
              ret = -EINVAL;
              break;
            }

          ret = ktd2052_set_rgb(priv, mrgb[0], &mrgb[1]);
        }
        break;

      case KTDIOSETMODE:
        {
          /* Set operating mode */

          FAR struct ktd2052_mode_s *mode = (FAR struct ktd2052_mode_s *)arg;
          if (mode == NULL)
            {
              ret = -EINVAL;
              break;
            }

          ret = ktd2052_set_mode(priv, mode->mode, mode->bright_extend,
                                 mode->temp_limit, mode->fade_rate);
        }
        break;

      case KTDIOSETPATTERN:
        {
          /* Set pattern configuration */

          FAR struct ktd2052_pattern_s *pattern =
            (FAR struct ktd2052_pattern_s *)arg;
          if (pattern == NULL)
            {
              ret = -EINVAL;
              break;
            }

          ret = ktd2052_setup_pattern(priv, pattern->slots,
                                      pattern->duration,
                                      pattern->fade_rate1,
                                      pattern->watchdog);
        }
        break;

      case KTDIOSETSLOTS:
        {
          /* Set pattern slots for a module */

          FAR struct ktd2052_slots_s *slots =
                                          (FAR struct ktd2052_slots_s *)arg;
          if (slots == NULL)
            {
              ret = -EINVAL;
              break;
            }

          ret = ktd2052_set_pattern_slots(priv, slots->module, slots->slots);
        }
        break;

      case KTDIOGETMONITOR:
        {
          /* Get monitor register status */

          FAR uint8_t *status = (FAR uint8_t *)arg;
          if (status == NULL)
            {
              ret = -EINVAL;
              break;
            }

          ret = ktd2052_read_reg(priv, KTD2052_MONITOR_REG, status);
        }
        break;

      case KTDIOSETWDOG:
        {
          /* Update watchdog register */

          uint8_t wd = (uint8_t)arg;
          ret = ktd2052_write_watchdog(priv, wd);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ktd2052_register
 *
 * Description:
 *   Register the KTD2052 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/leds0"
 *   i2c     - An instance of the I2C interface to communicate with KTD2052
 *   addr    - The I2C address of the KTD2052
 *   freq    - The I2C frequency to use for the KTD2052
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ktd2052_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr, uint32_t freq)
{
  FAR struct ktd2052_dev_s *priv;
  int ret;

  /* Initialize the device structure */

  priv = ktd2052_initialize(i2c, addr, freq);
  if (priv == NULL)
    {
      return -ENODEV;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_ktd2052_fops, 0666, priv);
  if (ret < 0)
    {
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_KTD2052 */

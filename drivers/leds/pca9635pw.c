/****************************************************************************
 * drivers/leds/pca9635pw.c
 *
 *   Copyright (C) 2015 DS-Automotion GmbH. All rights reserved.
 *   Author: Alexander Entinger <a.entinger@ds-automotion.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/leds/pca9635pw.h>

#if defined(CONFIG_I2C) && defined(CONFIG_PCA9635PW)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct pca9635pw_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t i2c_addr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pca9635pw_i2c_write_byte(FAR struct pca9635pw_dev_s *priv,
                                    uint8_t const reg_addr,
                                    uint8_t const reg_val);
static int pca9635pw_set_led_mode(FAR struct pca9635pw_dev_s *priv,
                                  uint8_t const led_out_x_mode);

static int pca9635pw_open(FAR struct file *filep);
static int pca9635pw_close(FAR struct file *filep);
static int pca9635pw_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pca9635pw_fileops =
{
  pca9635pw_open,               /* open */
  pca9635pw_close,              /* close */
  0,                            /* read */
  0,                            /* write */
  0,                            /* seek */
  pca9635pw_ioctl,              /* ioctl */
  0                             /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pca9635pw_i2c_write_byte
 *
 * Description:
 *   Write a single byte to one of the PCA9635PW configuration registers.
 *
 ****************************************************************************/

static int pca9635pw_i2c_write_byte(FAR struct pca9635pw_dev_s *priv,
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

  config.frequency = I2C_BUS_FREQ_HZ;
  config.address   = priv->i2c_addr;
  config.addrlen   = 7;

  /* Write the register address followed by the data (no RESTART) */

  lcdinfo("i2c addr: 0x%02X reg addr: 0x%02X value: 0x%02X\n",
          priv->i2c_addr, buffer[0], buffer[1]);

  ret = i2c_write(priv->i2c, &config, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      lcderr("ERROR: i2c_write returned error code %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: pca9635pw_set_led_mode
 *
 * Description:
 *   Set the led output mode (see PCA9635PW_LED_OUT_x register value
 *   definitions)
 *
 ****************************************************************************/

static int pca9635pw_set_led_mode(FAR struct pca9635pw_dev_s *priv,
                                  uint8_t const led_out_x_mode)
{
  uint8_t current_ledout_reg = PCA9635PW_LED_OUT_0;

  for (; current_ledout_reg <= PCA9635PW_LED_OUT_3; current_ledout_reg++)
    {
      int const ret = pca9635pw_i2c_write_byte(priv, current_ledout_reg,
                                               led_out_x_mode);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: pca9635pw_open
 *
 * Description:
 *   This function is called whenever a PCA9635PW device is opened.
 *
 ****************************************************************************/

static int pca9635pw_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct pca9635pw_dev_s *priv = inode->i_private;
  int ret = -1;

  /* Wake up the PCA9635PW (sleep bit PCA9635PW_MODE_1_SLEEP is set to zero
   * and  enable register auto increment - this way we can write to multiple
   * consecutive I2C registers without having to always write first the
   * address and then the data byte.
   */

  uint8_t const PCA9635PW_MODE_1_INITIAL_VALUE = PCA9635PW_MODE_1_AI2;

  ret = pca9635pw_i2c_write_byte(priv, PCA9635PW_MODE_1,
                                 PCA9635PW_MODE_1_INITIAL_VALUE);
  if (ret < 0)
    {
      lcderr("ERROR: Could not set initial config for PCA9635PW_MODE_1\n");
      return ret;
    }

  /* Configure the PCA9635PW output drivers for totem-pole structure since
   * the output of the PCA9635PW are coupled with the Gates of MOSFET's
   * which then drive the LED's.  Since we have this kind of schematic
   * structure we also need to invert the output.
   */

  uint8_t const PCA9635PW_MODE_2_INITIAL_VALUE =
    PCA9635PW_MODE_2_INVRT | PCA9635PW_MODE_2_OUTDRV;

  ret = pca9635pw_i2c_write_byte(priv, PCA9635PW_MODE_2,
                                 PCA9635PW_MODE_2_INITIAL_VALUE);
  if (ret < 0)
    {
      lcderr("ERROR: Could not set initial config for PCA9635PW_MODE_2\n");
      return ret;
    }

  /* A delay of 500 us is necessary since this is the maximum time which the
   * oscillator of the PCA9635PW needs to be up and running once sleep mode
   * was left.
   */

  nxsig_usleep(500);

  /* Turn all led drivers to mode 2 in which the led brightness is controlled
   * by the individual pwm registers.
   */

  ret = pca9635pw_set_led_mode(priv, PCA9635PW_LED_OUT_X_MODE_2);
  if (ret < 0)
    {
      lcderr("ERROR: Could not set led driver outputs to MODE2"
             " (LED's brightness are controlled by pwm registers)\n");
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: pca9635pw_close
 *
 * Description:
 *   This function is called whenever a PCA9635PW device is closed.
 *
 ****************************************************************************/

static int pca9635pw_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct pca9635pw_dev_s *priv = inode->i_private;
  int ret = -1;

  /* Turn all led drivers off */

  ret = pca9635pw_set_led_mode(priv, PCA9635PW_LED_OUT_X_MODE_0);
  if (ret < 0)
    {
      lcderr("ERROR: Could not set led driver outputs to MODE0"
             " (LED's are off)\n");
      return ret;
    }

  /* Send the PCA9635PW back to sleep mode */

  uint8_t const PCA9635PW_MODE_1_FINAL_VALUE = PCA9635PW_MODE_1_SLEEP;

  ret = pca9635pw_i2c_write_byte(priv, PCA9635PW_MODE_1,
                                PCA9635PW_MODE_1_FINAL_VALUE);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: pca9635pw_close
 *
 * Description:
 *   This function is called whenever an ioctl call to a PCA9635PW is
 *   performed.
 *
 ****************************************************************************/

static int pca9635pw_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct pca9635pw_dev_s *priv = inode->i_private;
  int ret = OK;

  lcdinfo("cmd: %d arg: %ld\n", cmd, arg);

  switch (cmd)
    {
      /* Set the brightness of an individual LED.
       * Arg: pca9635pw_brightness_s pointer.
       */

    case PWMIOC_SETLED_BRIGHTNESS:
      {
        /* Retrieve the information handed over as argument for this ioctl */

        FAR const struct pca9635pw_brightness_s *ptr =
          (FAR const struct pca9635pw_brightness_s *)((uintptr_t)arg);

        DEBUGASSERT(ptr != NULL);

        /* Set the brighntess of the led */

        ret = pca9635pw_i2c_write_byte(priv, ptr->led, ptr->brightness);
      }
      break;

      /* The used ioctl command was invalid */

    default:
      {
        lcderr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
      }
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pca9635pw_register
 *
 * Description:
 *   Register the PCA9635PW device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/leddrv0".
 *   i2c     - An instance of the I2C interface to use to communicate
 *             with the LM92.
 *   pca9635pw_i2c_addr
 *           - The I2C address of the PCA9635PW.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pca9635pw_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t const pca9635pw_i2c_addr)
{
  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the PCA9635PW device structure */

  FAR struct pca9635pw_dev_s *priv =
    (FAR struct pca9635pw_dev_s *)kmm_malloc(sizeof(struct pca9635pw_dev_s));

  if (priv == NULL)
    {
      lcderr("ERROR: Failed to allocate instance of pca9635pw_dev_s\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->i2c_addr = pca9635pw_i2c_addr;

  /* Register the character driver */

  int const ret = register_driver(devpath, &g_pca9635pw_fileops, 0666, priv);
  if (ret != OK)
    {
      lcderr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_I2C && CONFIG_I2C_PCA9635PW */

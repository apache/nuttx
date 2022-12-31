/****************************************************************************
 * drivers/leds/rgbled.c
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
#include <stdlib.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/leds/rgbled.h>
#include <nuttx/mutex.h>

#include <arch/irq.h>

#ifdef CONFIG_RGBLED

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct rgbled_upperhalf_s
{
  uint8_t           crefs;    /* The number of times the device has been opened */
  volatile bool     started;  /* True: pulsed output is being generated */
  mutex_t           lock;     /* Supports mutual exclusion */
  FAR struct pwm_lowerhalf_s *devledr;
  FAR struct pwm_lowerhalf_s *devledg;
  FAR struct pwm_lowerhalf_s *devledb;
#ifdef CONFIG_PWM_MULTICHAN
  int chanr;                  /* Red PWM channel */
  int chang;                  /* Green PWM channel */
  int chanb;                  /* Blue PWM channel */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     rgbled_open(FAR struct file *filep);
static int     rgbled_close(FAR struct file *filep);
static ssize_t rgbled_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t rgbled_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_rgbledops =
{
  rgbled_open,  /* open */
  rgbled_close, /* close */
  rgbled_read,  /* read */
  rgbled_write, /* write */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rgbled_open
 *
 * Description:
 *   This function is called whenever the PWM device is opened.
 *
 ****************************************************************************/

static int rgbled_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct rgbled_upperhalf_s *upper = inode->i_private;
  uint8_t                     tmp;
  int                         ret;

  lcdinfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      lcderr("ERROR: nxsem_wait failed: %d\n", ret);
      goto errout;
    }

  /* Increment the count of references to the device.  If this the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = upper->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_lock;
    }

  /* Save the new open count on success */

  upper->crefs = tmp;
  ret = OK;

errout_with_lock:
  nxmutex_unlock(&upper->lock);

errout:
  return ret;
}

/****************************************************************************
 * Name: rgbled_close
 *
 * Description:
 *   This function is called when the PWM device is closed.
 *
 ****************************************************************************/

static int rgbled_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct rgbled_upperhalf_s *upper = inode->i_private;
  int                         ret;

  lcdinfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      lcderr("ERROR: nxsem_wait failed: %d\n", ret);
      goto errout;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (upper->crefs > 1)
    {
      upper->crefs--;
    }

  nxmutex_unlock(&upper->lock);
  ret = OK;

errout:
  return ret;
}

/****************************************************************************
 * Name: rgbled_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t rgbled_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/****************************************************************************
 * Name: rgbled_lightness
 *
 * Description:
 *   Convert an 8-bit color level to a 16-bit PWM command, using a
 *   piecewise linear approximation of the CIE 1931 lightness formula.
 *
 ****************************************************************************/

#ifdef CONFIG_RGBLED_LIGHTNESS_CORRECTION
static unsigned short rgbled_lightness(unsigned char color_level)
{
  unsigned int lut_index;
  unsigned short pwm_cmd = 0;

  static const unsigned char lut_color_in[9] =
    {
      0x00, 0x20, 0x40, 0x60, 0x80, 0xa0, 0xc0, 0xe0, 0xff
    };

  static const unsigned short lut_pwm_out[9] =
    {
      0x0000, 0x03d1, 0x0b62, 0x1952, 0x2f93,
      0x5015, 0x7ccb, 0xb7a7, 0xffff
    };

  for (lut_index = 0; lut_index < sizeof(lut_color_in); ++lut_index)
    {
      if (lut_color_in[lut_index] >= color_level)
        {
          break;
        }
    }

  if (lut_index < sizeof(lut_color_in))
    {
      if (lut_color_in[lut_index] == color_level)
        {
          pwm_cmd = lut_pwm_out[lut_index];
        }
      else
        {
          pwm_cmd =  (unsigned short)(lut_pwm_out[lut_index - 1] +
            (int)(lut_pwm_out[lut_index] - lut_pwm_out[lut_index - 1]) *
            (int)(color_level - lut_color_in[lut_index - 1]) /
            (int)(lut_color_in[lut_index] - lut_color_in[lut_index - 1]));
        }
    }

  return pwm_cmd;
}
#endif /* CONFIG_RGBLED_LIGHTNESS_CORRECTION */

/****************************************************************************
 * Name: rgbled_write
 *
 * Description:
 *   A write method which parses an HTML-style RGB string like "#FF8833"
 *   into color values, and sends them to the lower-half PWM drivers.
 *
 ****************************************************************************/

static ssize_t rgbled_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rgbled_upperhalf_s *upper = inode->i_private;
  FAR struct pwm_lowerhalf_s *ledr = upper->devledr;
  FAR struct pwm_lowerhalf_s *ledg = upper->devledg;
  FAR struct pwm_lowerhalf_s *ledb = upper->devledb;
  struct pwm_info_s pwm;
  unsigned int red;
  unsigned int green;
  unsigned int blue;
  char color[3];
#ifdef CONFIG_PWM_MULTICHAN
  int i;
#endif

  /* We need to receive a string #RRGGBB = 7 bytes */

  if (buffer == NULL || buflen < 7)
    {
      /* Well... nothing to do */

      return -EINVAL;
    }

  /* Check if it is a color format */

  if (buffer[0] != '#')
    {
      /* The color code needs to start with # */

      return -EINVAL;
    }

  /* Move buffer to next character */

  buffer++;

  color[0] = buffer[0];
  color[1] = buffer[1];
  color[2] = '\0';

  red = strtol(color, NULL, 16);

  color[0] = buffer[2];
  color[1] = buffer[3];
  color[2] = '\0';

  green = strtol(color, NULL, 16);

  color[0] = buffer[4];
  color[1] = buffer[5];
  color[2] = '\0';

  blue = strtol(color, NULL, 16);

  /* Sane check */

  if (red > 255)
    {
      red = 255;
    }

  if (green > 255)
    {
      green = 255;
    }

  if (blue > 255)
    {
      blue = 255;
    }

  /* Convert 8bit to 16bits */

#ifdef CONFIG_RGBLED_LIGHTNESS_CORRECTION
  red   = rgbled_lightness((unsigned char)red);
  green = rgbled_lightness((unsigned char)green);
  blue  = rgbled_lightness((unsigned char)blue);
#else
  red   = (red   << 8) | red;
  green = (green << 8) | green;
  blue  = (blue  << 8) | blue;
#endif

#ifdef CONFIG_RGBLED_INVERT
  red   ^= 0xffff;
  green ^= 0xffff;
  blue  ^= 0xffff;
#endif

#ifdef CONFIG_PWM_MULTICHAN
  memset(&pwm, 0, sizeof(struct pwm_info_s));
  pwm.frequency = 100;

  i = 0;
  pwm.channels[i].duty = red;
  pwm.channels[i++].channel = upper->chanr;

  /* If the green pwm source is on the same timer as the red,
   * set that up now too.
   */

  if (ledr == ledg)
    {
      pwm.channels[i].duty = green;
      pwm.channels[i++].channel = upper->chang;
    }

  /* If the blue pwm source is on the same timer as the red,
   * set that up now too.
   */

  if (ledr == ledb)
    {
      pwm.channels[i].duty = blue;
      pwm.channels[i++].channel = upper->chanb;
    }

  ledr->ops->start(ledr, &pwm);

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      pwm.channels[i].channel = 0;
    }

  /* If the green timer is not the same as the red timer, update it
   * separately.
   */

  if (ledg != ledr)
    {
      i = 0;
      pwm.channels[i].duty = green;
      pwm.channels[i++].channel = upper->chang;

      /* If the blue pwm source is on the same timer as the green,
       * set that up now too.
       */

      if (ledg == ledb)
        {
          pwm.channels[i].duty = blue;
          pwm.channels[i++].channel = upper->chanb;
        }

      ledg->ops->start(ledg, &pwm);

      for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
        {
          pwm.channels[i].channel = 0;
        }
    }

  /* If the blue timer is not the same as the red or green timer, update it
   * separately.
   */

  if (ledb != ledr && ledb != ledg)
    {
      pwm.channels[0].duty = green;
      pwm.channels[0].channel = upper->chanb;

      ledb->ops->start(ledb, &pwm);
    }
#else
  pwm.frequency = 100;

  pwm.duty = red;
  ledr->ops->start(ledr, &pwm);

  pwm.duty = green;
  ledg->ops->start(ledg, &pwm);

  pwm.duty = blue;
  ledb->ops->start(ledb, &pwm);
#endif

  return buflen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rgbled_register
 *
 * Description:
 *   This function binds three instances of a "lower half" PWM driver with
 *   the "upper half" RGB LED device and registers that device so that can
 *   be used by application code.
 *
 *
 * Input Parameters:
 *   path - The full path to the driver to be registers in the NuttX pseudo-
 *     filesystem.  The recommended convention is to name all PWM drivers
 *     as "/dev/rgdbled0", "/dev/rgbled1", etc.  where the driver path
 *     differs only in the "minor" number at the end of the device name.
 *   ledr, ledg, and ledb - A pointer to an instance of lower half PWM
 *     drivers for the red, green, and blue LEDs, respectively.  These
 *     instances will be bound to the RGB LED driver and must persists as
 *     long as that driver persists.
 *   chanr, chang, chanb -Red/Green/Blue PWM channels (only if
 *     CONFIG_PWM_MULTICHAN is defined)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rgbled_register(FAR const char *path, FAR struct pwm_lowerhalf_s *ledr,
                                          FAR struct pwm_lowerhalf_s *ledg,
                                          FAR struct pwm_lowerhalf_s *ledb
#ifdef CONFIG_PWM_MULTICHAN
                                        , int chanr, int chang, int chanb
#endif
                                          )
{
  FAR struct rgbled_upperhalf_s *upper;

  /* Allocate the upper-half data structure */

  upper = (FAR struct rgbled_upperhalf_s *)
    kmm_zalloc(sizeof(struct rgbled_upperhalf_s));

  if (!upper)
    {
      lcderr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the PWM device structure (it was already zeroed by
   * kmm_zalloc())
   */

  nxmutex_init(&upper->lock);
  upper->devledr = ledr;
  upper->devledg = ledg;
  upper->devledb = ledb;

#ifdef CONFIG_PWM_MULTICHAN
  upper->chanr = chanr;
  upper->chang = chang;
  upper->chanb = chanb;
#endif

  /* Register the PWM device */

  lcdinfo("Registering %s\n", path);
  return register_driver(path, &g_rgbledops, 0666, upper);
}

#endif /* CONFIG_RGBLED */

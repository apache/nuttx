/****************************************************************************
 * drivers/rgbled.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
 * Compilation Switches
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
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/pwm.h>
#include <nuttx/leds/rgbled.h>

#include <arch/irq.h>

#ifdef CONFIG_RGBLED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing PWM */

#ifdef CONFIG_DEBUG_RGBLED
#  define pwmdbg    dbg
#  define pwmvdbg   vdbg
#  define pwmlldbg  lldbg
#  define pwmllvdbg llvdbg
#else
#  define pwmdbg(x...)
#  define pwmvdbg(x...)
#  define pwmlldbg(x...)
#  define pwmllvdbg(x...)
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct rgbled_upperhalf_s
{
  uint8_t           crefs;    /* The number of times the device has been opened */
  volatile bool     started;  /* True: pulsed output is being generated */
  sem_t             exclsem;  /* Supports mutual exclusion */
  struct pwm_info_s ledr;     /* Pulsed output for LED R*/
  struct pwm_info_s ledg;     /* Pulsed output for LED G*/
  struct pwm_info_s ledb;     /* Pulsed output for LED B*/
  struct pwm_lowerhalf_s *devledr;
  struct pwm_lowerhalf_s *devledg;
  struct pwm_lowerhalf_s *devledb;
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
  0,            /* seek */
  0             /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0           /* poll */
#endif
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

  pwmvdbg("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      ret = -get_errno();
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
      goto errout_with_sem;
    }

  /* Save the new open count on success */

  upper->crefs = tmp;
  ret = OK;

errout_with_sem:
  sem_post(&upper->exclsem);

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

  pwmvdbg("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      ret = -get_errno();
      goto errout;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (upper->crefs > 1)
    {
      upper->crefs--;
    }

  sem_post(&upper->exclsem);
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
 * Name: rgbled_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
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

  unsigned int red;
  unsigned int green;
  unsigned int blue;
  char color[3];

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

  red   <<= 8;
  green <<= 8;
  blue  <<= 8;

  /* Setup LED R */

  upper->ledr.frequency = 100;
  upper->ledr.duty = red;

  ledr->ops->start(ledr, &upper->ledr);

  /* Setup LED G */

  upper->ledg.frequency = 100;
  upper->ledg.duty = green;

  ledg->ops->start(ledg, &upper->ledg);

  /* Setup LED B */

  upper->ledb.frequency = 100;
  upper->ledb.duty = blue;

  ledb->ops->start(ledb, &upper->ledb);

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
 * Input parameters:
 *   path - The full path to the driver to be registers in the NuttX pseudo-
 *     filesystem.  The recommended convention is to name all PWM drivers
 *     as "/dev/rgdbled0", "/dev/rgbled1", etc.  where the driver path
 *     differs only in the "minor" number at the end of the device name.
 *   ledr, ledg, and ledb - A pointer to an instance of lower half PWM
 *     drivers for the red, green, and blue LEDs, respectively.  These
 *     instances will be bound to the RGB LED driver and must persists as
 *     long as that driver persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rgbled_register(FAR const char *path, FAR struct pwm_lowerhalf_s *ledr,
                                          FAR struct pwm_lowerhalf_s *ledg,
                                          FAR struct pwm_lowerhalf_s *ledb)
{
  FAR struct rgbled_upperhalf_s *upper;

  /* Allocate the upper-half data structure */

  upper = (FAR struct rgbled_upperhalf_s *)
    kmm_zalloc(sizeof(struct rgbled_upperhalf_s));

  if (!upper)
    {
      pwmdbg("Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the PWM device structure (it was already zeroed by
   * kmm_zalloc())
   */

  sem_init(&upper->exclsem, 0, 1);
  upper->devledr = ledr;
  upper->devledg = ledg;
  upper->devledb = ledb;

  /* Register the PWM device */

  pwmvdbg("Registering %s\n", path);
  return register_driver(path, &g_rgbledops, 0666, upper);
}

#endif /* CONFIG_RGBLED */

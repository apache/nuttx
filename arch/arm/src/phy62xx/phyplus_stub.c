/****************************************************************************
 * arch/arm/src/phy62xx/phyplus_stub.c
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spinlock.h>
#include <nuttx/ioexpander/gpio.h>
#include "phyplus_stub.h"
#include "phyplus_gpio.h"
#include "gpio.h"
#include "phyplus_tim.h"
#include "timer.h"
#include "mcu_phy_bumbee.h"
#include "phyplus_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CMD_LEN   128

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t phyplus_stub_read(
                struct file *filep, char *buffer, size_t buflen);
static ssize_t phyplus_stub_write(
                struct file *filep, const char *buffer,
                size_t buflen);
static off_t   phyplus_stub_seek(
                struct file *filep, off_t offset, int whence);
static int     phyplus_stub_ioctl(
                struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_stub_drvrops =
{
  NULL,                /* open */
  NULL,                /* close */
  phyplus_stub_read,   /* read */
  phyplus_stub_write,  /* write */
  phyplus_stub_seek,   /* seek */
  phyplus_stub_ioctl,  /* ioctl */
  NULL                 /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL               /* unlink */
#endif
};

/* static struct gpio_dev_s stub_dev; */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

  /* fprintf(stderr, "USAGE: %s [<reg_gpio|unreg_gpio>] [-idx <1-23>]"
   * "[-mode <input|output|interrupt>] [-trig <raise|fall>]"
   * "[-val <high|low>] [-pull <float|strong_up|up|down>]", progname);
   */

static int phyplus_parse_gpio_param(char *buff,
                struct phyplus_gpio_param_s *gpio_param)
{
  char *ptr;
  int temp_val;

  /* setup default params.. */

  gpio_param->pin_idx = 0;
  gpio_param->mode = PHYPLUS_GPIO_INPUT;
  gpio_param->trig_mode = POL_FALLING;
  gpio_param->default_val = 0;
  gpio_param->pin_pull = GPIO_FLOATING;
  ptr = strtok(buff, ",");
  if (ptr != NULL)
    {
      if (0 == strncmp("idx", ptr, 3))
        {
          ptr = ptr + 4;
          temp_val = atoi(ptr);
          if ((temp_val < 0) || (temp_val >= GPIO_NUM))
            {
              fprintf(stderr, "parse gpio_idx failure");
              return -1;
            }
          else
            {
              gpio_param->pin_idx = temp_val;
            }
        }
      else if(0 == strncmp("mode", ptr, 4))
        {
          ptr = ptr + 5;
          if (0 == strncmp(ptr, "input", 5))
            {
              gpio_param->mode = PHYPLUS_GPIO_INPUT;
            }
          else if (0 == strncmp(ptr, "output", 6))
            {
              gpio_param->mode = PHYPLUS_GPIO_OUTPUT;
            }
          else if (0 == strncmp(ptr, "interrupt", 9))
            {
              gpio_param->mode = PHYPLUS_GPIO_INTERRUPT;
            }
          else
            {
              fprintf(stderr, "parse gpio_mode failure");
              return -1;
            }
        }
      else if (0 == strncmp("trig", ptr, 4))
        {
          ptr = ptr + 5;
          if (0 == strncmp(ptr, "raise", 5))
            {
              gpio_param->trig_mode = POL_RISING;
            }
          else if (0 == strncmp(ptr, "fall", 4))
            {
              gpio_param->trig_mode = POL_FALLING;
            }
          else
            {
              fprintf(stderr, "parse gpio_trig failure");
              return -1;
            }
        }
      else if (0 == strncmp("val", ptr, 3))
        {
          ptr = ptr + 4;
          temp_val = atoi(ptr);
          if ((temp_val < 0) || (temp_val > 2))
            {
              fprintf(stderr, "parse gpio_default_val failure");
              return -1;
            }
          else
            {
              gpio_param->pin_idx = temp_val;
            }
        }
      else if (strncmp(ptr, "pull", 4) == 0)
        {
          ptr = ptr + 5;
          if (0 == strncmp(ptr, "float", 5))
            {
              gpio_param->pin_pull = GPIO_FLOATING;
            }
          else if (0 == strncmp(ptr, "strong_up", 9))
            {
              gpio_param->pin_pull = GPIO_PULL_UP_S;
            }
          else if (0 == strncmp(ptr, "up", 2))
            {
              gpio_param->pin_pull = GPIO_PULL_UP;
            }
          else if (0 == strncmp(ptr, "down", 4))
            {
              gpio_param->pin_pull = GPIO_PULL_DOWN;
            }
          else
            {
              fprintf(stderr, "parse gpio_pull failure");
              return -1;
            }
        }
    }
  else
    {
      fprintf(stderr, "parse first param failure");
      return -1;
    }

  while ((ptr = strtok(NULL, ",")))
    {
      if (0 == strncmp("idx", ptr, 3))
        {
          ptr = ptr + 4;
          temp_val = atoi(ptr);
          if ((temp_val < 0) || (temp_val >= GPIO_NUM))
            {
              fprintf(stderr, "parse gpio_idx failure");
              return -1;
            }
          else
            {
              gpio_param->pin_idx = temp_val;
            }
        }
      else if (0 == strncmp("mode", ptr, 4))
        {
          ptr = ptr + 5;
          if (0 == strncmp(ptr, "input", 5))
            {
              gpio_param->mode = PHYPLUS_GPIO_INPUT;
            }
          else if (0 == strncmp(ptr, "output", 6))
            {
              gpio_param->mode = PHYPLUS_GPIO_OUTPUT;
            }
          else if (0 == strncmp(ptr, "interrupt", 9))
            {
              gpio_param->mode = PHYPLUS_GPIO_INTERRUPT;
            }
          else
            {
              fprintf(stderr, "parse gpio_mode failure");
              return -1;
            }
        }
      else if (0 == strncmp("trig", ptr, 4))
        {
          ptr = ptr + 5;
          if (0 == strncmp(ptr, "raise", 5))
            {
              gpio_param->trig_mode = POL_RISING;
            }
          else if (0 == strncmp(ptr, "fall", 4))
            {
              gpio_param->trig_mode = POL_FALLING;
            }
          else
            {
              fprintf(stderr, "parse gpio_trig failure");
              return -1;
            }
        }
      else if (0 == strncmp("val", ptr, 3))
        {
          ptr = ptr + 4;
          temp_val = atoi(ptr);
          if ((temp_val < 0) || (temp_val > 2))
            {
              fprintf(stderr, "parse gpio_default_val failure");
              return -1;
            }
          else
            {
              gpio_param->pin_idx = temp_val;
            }
        }
      else if (strncmp(ptr, "pull", 4) == 0)
        {
          ptr = ptr + 5;
          if (0 == strncmp(ptr, "float", 5))
            {
              gpio_param->pin_pull = GPIO_FLOATING;
            }
          else if (0 == strncmp(ptr, "strong_up", 9))
            {
              gpio_param->pin_pull = GPIO_PULL_UP_S;
            }
          else if (0 == strncmp(ptr, "up", 2))
            {
              gpio_param->pin_pull = GPIO_PULL_UP;
            }
          else if (0 == strncmp(ptr, "down", 4))
            {
              gpio_param->pin_pull = GPIO_PULL_DOWN;
            }
          else
            {
              fprintf(stderr, "parse gpio_pull failure");
              return -1;
            }
        }
    }

  return 0;
}

/* fprintf(stderr, "USAGE: %s [<reg_timer|unreg_timer>] [idx= <1-6>]" */

/* "[mode=<freerun|count>] [val=<0-2^23>]" */

/* " [en=<true|false>] [mask=<true|false>]", progname); */

#if 0
static int phyplus_parse_timer_param(
               char *buff, struct AP_TIM_TYPEDEF *timer_param)
{
  char *ptr;
  int temp_val;

  /* setup default params.. */

  timer_param->idx = 0;
  timer_param->enable = 0;
  timer_param->mask = 0;
  timer_param->mode = 0;       /* 0:freerun, 1:count */
  timer_param->value = 0;

  ptr = strtok(buff, ",");
  if (ptr != NULL)
    {
      if (0 == strncmp("idx", ptr, 3))
        {
          ptr = ptr + 4;
          temp_val = atoi(ptr);
          if ((temp_val < 1) || (temp_val > 6))
            {
              fprintf(stderr, "parse timer_idx failure");
              return -1;
            }
          else
            {
              timer_param->idx = temp_val;
            }
        }
      else if (0 == strncmp("mode", ptr, 4))
        {
          ptr = ptr + 5;
          if (0 == strncmp(ptr, "freerun", 7))
            {
              timer_param->mode = 0;
            }
          else if (0 == strncmp(ptr, "count", 5))
            {
              timer_param->mode = 1;
            }
          else
            {
              fprintf(stderr, "parse timer_mode failure");
              return -1;
            }
        }
      else if (0 == strncmp("enable", ptr, 6))
        {
          ptr = ptr + 7;
          if (0 == strncmp(ptr, "true", 4))
            {
              timer_param->mode = 1;
            }
          else if (0 == strncmp(ptr, "false", 5))
            {
              timer_param->mode = 0;
            }
          else
            {
              fprintf(stderr, "parse enable failure");
              return -1;
            }
        }
      else if (0 == strncmp("mask", ptr, 4))
        {
          ptr = ptr + 5;
          if (0 == strncmp(ptr, "true", 4))
            {
              timer_param->mask = 1;
            }
          else if (0 == strncmp(ptr, "false", 5))
            {
              timer_param->mask = 0;
            }
          else
            {
              fprintf(stderr, "parse mask failure");
              return -1;
            }
        }
      else if (0 == strncmp("value", ptr, 3))
        {
          ptr = ptr + 4;
          temp_val = atoi(ptr);
          timer_param->idx = temp_val;
        }
    }

  while (ptr = strtok(NULL, ","))
    {
      if (ptr != NULL)
        {
          if (0 == strncmp("idx", ptr, 3))
            {
              ptr = ptr + 4;
              temp_val = atoi(ptr);
              if ((temp_val < 1) || (temp_val > 6))
                {
                  fprintf(stderr, "parse timer_idx failure");
                  return -1;
                }
              else
                {
                  timer_param->idx = temp_val;
                }
            }
          else if (0 == strncmp("mode", ptr, 4))
            {
              ptr = ptr + 5;
              if (0 == strncmp(ptr, "freerun", 7))
                {
                  timer_param->mode = 0;
                }
              else if (0 == strncmp(ptr, "count", 5))
                {
                  timer_param->mode = 1;
                }
              else
                {
                  fprintf(stderr, "parse timer_mode failure");
                  return -1;
                }
            }
          else if (0 == strncmp("enable", ptr, 6))
            {
              ptr = ptr + 7;
              if (0 == strncmp(ptr, "true", 4))
                {
                  timer_param->mode = 1;
                }
              else if (0 == strncmp(ptr, "false", 5))
                {
                  timer_param->mode = 0;
                }
              else
                {
                  fprintf(stderr, "parse enable failure");
                  return -1;
                }
            }
          else if (0 == strncmp("mask", ptr, 4))
            {
              ptr = ptr + 5;
              if (0 == strncmp(ptr, "true", 4))
                {
                  timer_param->mask = 1;
                }
              else if (0 == strncmp(ptr, "false", 5))
                {
                  timer_param->mask = 0;
                }
              else
                {
                  fprintf(stderr, "parse mask failure");
                  return -1;
                }
            }
          else if (0 == strncmp("value", ptr, 3))
            {
              ptr = ptr + 4;
              temp_val = atoi(ptr);
              timer_param->idx = temp_val;
            }
        }
    }

  return 0;
}
#endif

static int phyplus_parse_params_and_action(char *buff)
{
  char *p = buff;
  int ret = 0;
  struct phyplus_gpio_param_s gpio_param;
#if 0
  struct AP_TIM_TYPEDEF timer_param;
#endif
  if (0 == strncmp(buff, "reg_gpio", 8))
    {
      p += 9;
      ret = phyplus_parse_gpio_param(p, &gpio_param);
      if (0 != ret)
        {
          fprintf(stderr, "parse gpio param failure");
          return -1;
        }

      ret = phyplus_gpio_register(&gpio_param);
      if (0 != ret)
        {
          fprintf(stderr, "gpio register failure");
          return -1;
        }
    }
  else if (0 == strncmp(buff, "unreg_gpio", 10))
    {
      p += 11;
      ret = phyplus_parse_gpio_param(p, &gpio_param);
      if (0 != ret)
        {
          fprintf(stderr, "parse gpio param failure");
          return -1;
        }

      ret = phyplus_gpio_register(&gpio_param);
      if (0 != ret)
        {
          fprintf(stderr, "gpio unregister failure");
          return -1;
        }
    }
#if 0  
  else if (0 == strncmp(buff, "reg_timer", 9))
    {
      p += 10;
      ret = phyplus_parse_timer_param(p, &timer_param);
      if (0 != ret)
        {
          fprintf(stderr, "parse timer param failure");
          return -1;
        }

      ret = phyplus_timer_register(&gpio_param);
      if (0 != ret)
        {
          fprintf(stderr, "timer register failure");
          return -1;
        }
    }
  else if (0 == strncmp(buff, "unreg_timer", 11))
    {
      p += 12;
      ret = phyplus_parse_timer_param(p, &timer_param);
      if (0 != ret)
        {
          fprintf(stderr, "parse timer param failure");
          return -1;
        }

      ret = phyplus_timer_register(&timer_param);
      if (0 != ret)
        {
          fprintf(stderr, "timer unregister failure");
          return -1;
        }
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: phyplus_gpio_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t phyplus_stub_read(
                struct file *filep, char *buffer, size_t buflen)
{
  buffer[0] = 'T';
  buffer[1] = 'E';
  buffer[2] = 'S';
  buffer[3] = 'T';
  buffer[4] = '\0';
  filep->f_pos  = 0;

  /* buffer[0]    += '0';
   *  filep->f_pos  = 1;
   */

  return 1;
}

/****************************************************************************
 * Name: phyplus_stub_write
 *
 * Description:
 *   Standard character driver write method.
 *
 *   REVISIT:  The read() method obeys the semantics of a file position and
 *   requires re-opening the driver or seeking to address 0.  The write()
 *   method does not.  This is an inconsistency.
 *
 ****************************************************************************/

static char phyplus_cmd[CMD_LEN];

static ssize_t phyplus_stub_write(struct file *filep,
                                  const char *buffer, size_t buflen)
{
  struct inode *inode;

  /* struct gpio_dev_s *dev; */

  int ret = 0;
  static int cmd_pos = 0;
  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);

  /* dev = inode->i_private; */

  /* Verify that a buffer containing data was provided */

  DEBUGASSERT(buffer != NULL);

  if (1 == buflen)
    {
      if ('#' == buffer[0])
        {
          phyplus_cmd[cmd_pos] = '\0';
          ret = phyplus_parse_params_and_action(phyplus_cmd);
          memset(phyplus_cmd, 0x0, CMD_LEN);
          if (ret != 0)
            {
              fprintf(stderr, "phyplus action failed");
            }
        }
      else
        {
          phyplus_cmd[cmd_pos] = buffer[0];
          cmd_pos++;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: phyplus_stub_seek
 *
 * Description:
 *   Reset read flag on seek to 0
 *
 *   REVISIT:  Seeking address zero is required to return addition GPIO
 *   values from read().  This, however, is an un-natural use of a file
 *   position since there is no file position associated with a GPIO.  It
 *   also makes the read() method difficult to use programmatically.
 *
 ****************************************************************************/

static off_t phyplus_stub_seek(struct file *filep, off_t offset,
                               int whence)
{
  /* Only SEEK_SET is supported, return ENOSYS for other valid options */

  if (whence == SEEK_CUR || whence == SEEK_END)
    {
      return -ENOSYS;
    }

  /* Only Offset zero makes sense,  POSIX permits setting the file position
   * beyond the end of the file, but that makes little sense here.
   */

  if (whence == SEEK_SET && offset == 0)
    {
      filep->f_pos = 0;
      return 0;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: phyplus_stub_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int phyplus_stub_ioctl(struct file *filep, int cmd,
                              unsigned long arg)
{
    struct inode *inode;
    int ret = 0;

      /* irqstate_t flags;
       * pid_t pid;
       * int i;
       * int j = 0;
       */

    DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
    inode = filep->f_inode;
    DEBUGASSERT(inode->i_private != NULL);

    struct phyplus_gpio_param_s *phyplus_gpio =
            (struct phyplus_gpio_param_s *)arg;
#if 0
    struct phyplus_timer_param_s *phyplus_timer =
            (struct phyplus_timer_param_s *)arg;
#endif

    switch (cmd)
    {
        case PHYPLUS_GPIO_REGISTER:
            ret = phyplus_gpio_register(phyplus_gpio);
            break;

        case PHYPLUS_GPIO_UNREGISTER:
            ret = phyplus_gpio_unregister(phyplus_gpio);
            break;
        case PHYPLUS_TIMER_REGISTER:

        /* ret = phyplus_timer_register(phyplus_timer); */

            break;
        case PHYPLUS_TIMER_UNREGISTER:

        /* ret = phyplus_timer_unregister(phyplus_timer); */

            break;

        /* Unrecognized command */

      default:
        ret = -ENOTTY;
        break;
      }

  return ret;
}

/****************************************************************************
 * Name: phyplus_stub_register
 *
 * Description:
 *   Register phyplus stub device driver.
 *
 *   - Input pin types will be registered at /dev/gpinN
 *
 *   Where N is the provided minor number in the range of 0-99.
 *
 ****************************************************************************/

int phyplus_stub_register(void)
{
  char devname[16];
  snprintf(devname, 16, "/dev/phyplus");
  return register_driver(devname, &g_stub_drvrops, 0666, NULL);
}

/****************************************************************************
 * Name: phyplus_stub_unregister
 *
 * Description:
 *   Unregister phyplus stub device driver.
 *
 *   - Input pin types will be registered at /dev/gpinN
 *   - Output pin types will be registered at /dev/gpoutN
 *   - Interrupt pin types will be registered at /dev/gpintN
 *
 *   Where N is the provided minor number in the range of 0-99.
 *
 *
 ****************************************************************************/

void phyplus_stub_unregister(void)
{
  char devname[16];
  snprintf(devname, 16, "/dev/phyplus");
  unregister_driver(devname);
}

int phyplus_stub_init(void)
{
  int ret = 0;
  phyplus_stub_register();

  return ret;
}

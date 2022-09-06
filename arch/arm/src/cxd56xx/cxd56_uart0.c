/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_uart0.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <debug.h>
#include <errno.h>

#include "arm_internal.h"
#include "chip.h"
#include "cxd56_pinconfig.h"

#ifdef CONFIG_CXD56_UART0

#include <arch/chip/uart0.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CXD56_UART0_BAUD
#  define CONFIG_CXD56_UART0_BAUD 921600
#endif
#ifndef CONFIG_CXD56_UART0_BITS
#  define CONFIG_CXD56_UART0_BITS 8
#endif
#ifndef CONFIG_CXD56_UART0_PARITY
#  define CONFIG_CXD56_UART0_PARITY   0
#endif
#ifndef CONFIG_CXD56_UART0_2STOP
#  define CONFIG_CXD56_UART0_2STOP     0
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int uart0_open(struct file *filep);
static int uart0_close(struct file *filep);
static ssize_t uart0_read(struct file *filep,
                          char *buffer, size_t len);
static ssize_t uart0_write(struct file *filep,
                           const char *buffer, size_t len);

/****************************************************************************
 * FarAPI prototypes
 ****************************************************************************/

int fw_pd_uartinit(int ch);
int fw_pd_uartuninit(int ch);
int fw_pd_uartconfiguration(int ch, int baudrate, int databits,
                         int parity, int stopbit, int flowctrl);
int fw_pd_uartenable(int ch);
int fw_pd_uartdisable(int ch);
int fw_pd_uartreceive(int ch, void *buf, int size, int leave);
int fw_pd_uartsend(int ch, void *buf, int size, int leave);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_uart0fops =
{
  .open  = uart0_open,
  .close = uart0_close,
  .read  = uart0_read,
  .write = uart0_write
};

static mutex_t g_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart0_open
 ****************************************************************************/

static int uart0_open(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  int flowctl;
  int bits;
  int stop;
  int ret;

  if (inode->i_crefs > 1)
    {
      return OK;
    }

  ret = fw_pd_uartinit(0);
  if (ret < 0)
    {
      return -EFAULT;
    }

  /* 0 = 5bit, 1 = 6bit, 2 = 7bit, 3 = 8bit */

  bits = CONFIG_CXD56_UART0_BITS - 5;

  /* 1 = 1 stop, 2 = 2 stop bit */

  stop = CONFIG_CXD56_UART0_2STOP + 1;

  /* Enable UART0 pin configuration */

#ifdef CONFIG_CXD56_UART0_FLOWCONTROL
  flowctl = 1;
  CXD56_PIN_CONFIGS(PINCONFS_SPI2_UART0);
#else
  flowctl = 0;
  CXD56_PIN_CONFIGS(PINCONFS_SPI2A_UART0);
#endif

  ret = fw_pd_uartconfiguration(0, CONFIG_CXD56_UART0_BAUD,
                             bits,
                             CONFIG_CXD56_UART0_PARITY,
                             stop, flowctl);
  if (ret < 0)
    {
      fw_pd_uartuninit(0);
      return -EINVAL;
    }

  ret = fw_pd_uartenable(0);
  if (ret < 0)
    {
      fw_pd_uartuninit(0);
      return -EFAULT;
    }

  return OK;
}

/****************************************************************************
 * Name: uart0_close
 ****************************************************************************/

static int uart0_close(struct file *filep)
{
  struct inode *inode = filep->f_inode;

  if (inode->i_crefs == 1)
    {
      fw_pd_uartdisable(0);
      fw_pd_uartuninit(0);

      /* Disable UART0 pin by changing Hi-Z GPIO */

#ifdef CONFIG_CXD56_UART0_FLOWCONTROL
      CXD56_PIN_CONFIGS(PINCONFS_SPI2_GPIO);
#else
      CXD56_PIN_CONFIGS(PINCONFS_SPI2A_GPIO);
#endif
    }

  return 0;
}

/****************************************************************************
 * Name: uart0_read
 ****************************************************************************/

static ssize_t uart0_read(struct file *filep,
                          char *buffer, size_t len)
{
  int ret;

  nxmutex_lock(&g_lock);

  ret = fw_pd_uartreceive(0, buffer, len,
                          ((filep->f_oflags & O_NONBLOCK) != 0));

  nxmutex_unlock(&g_lock);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: uart0_write
 ****************************************************************************/

static ssize_t uart0_write(struct file *filep,
                           const char *buffer, size_t len)
{
  int ret;

  nxmutex_lock(&g_lock);

  ret = fw_pd_uartsend(0, (void *)buffer, len,
                       ((filep->f_oflags & O_NONBLOCK) != 0));

  nxmutex_unlock(&g_lock);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: cxd56_uart0initialize
 ****************************************************************************/

int cxd56_uart0initialize(const char *devname)
{
  int ret;

  ret = register_driver(devname, &g_uart0fops, 0666, NULL);
  if (ret != 0)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_uart0uninitialize
 ****************************************************************************/

void cxd56_uart0uninitialize(const char *devname)
{
  unregister_driver(devname);
}

#endif /* CONFIG_CXD56_UART0 */

/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_uart0.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>

#include <queue.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <debug.h>
#include <errno.h>

#include "arm_arch.h"
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

static int uart0_open(FAR struct file *filep);
static int uart0_close(FAR struct file *filep);
static ssize_t uart0_read(FAR struct file *filep,
                          FAR char *buffer, size_t len);
static ssize_t uart0_write(FAR struct file *filep,
                           FAR const char *buffer, size_t len);
static int uart0_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int uart0_semtake(sem_t *id);
static void uart0_semgive(sem_t *id);

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
  .write = uart0_write,
  .seek  = 0,
  .ioctl = uart0_ioctl,
};

static sem_t g_lock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart0_semtake
 ****************************************************************************/

static int uart0_semtake(sem_t *id)
{
  return nxsem_wait_uninterruptible(id);
}

/****************************************************************************
 * Name: uart0_semgive
 ****************************************************************************/

static void uart0_semgive(sem_t *id)
{
  nxsem_post(id);
}

/****************************************************************************
 * Name: uart0_open
 ****************************************************************************/

static int uart0_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
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
      set_errno(EFAULT);
      return ERROR;
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
      set_errno(EINVAL);
      return ERROR;
    }

  ret = fw_pd_uartenable(0);
  if (ret < 0)
    {
      fw_pd_uartuninit(0);
      set_errno(EFAULT);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: uart0_close
 ****************************************************************************/

static int uart0_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;

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

static ssize_t uart0_read(FAR struct file *filep,
                          FAR char *buffer, size_t len)
{
  int ret;

  uart0_semtake(&g_lock);

  ret = fw_pd_uartreceive(0, buffer, len,
                          ((filep->f_oflags & O_NONBLOCK) != 0));

  uart0_semgive(&g_lock);

  if (ret < 0)
    {
      set_errno(-ret);
      ret = 0; /* Receive no data */
    }

  return (ssize_t)ret;
}

/****************************************************************************
 * Name: uart0_write
 ****************************************************************************/

static ssize_t uart0_write(FAR struct file *filep,
                           FAR const char *buffer, size_t len)
{
  int ret;

  uart0_semtake(&g_lock);

  ret = fw_pd_uartsend(0, (FAR void *)buffer, len,
                       ((filep->f_oflags & O_NONBLOCK) != 0));

  uart0_semgive(&g_lock);

  if (ret < 0)
    {
      set_errno(-ret);
      ret = 0;
    }

  return (ssize_t)ret;
}

/****************************************************************************
 * Name: uart0_ioctl
 ****************************************************************************/

static int uart0_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: cxd56_uart0initialize
 ****************************************************************************/

int cxd56_uart0initialize(FAR const char *devname)
{
  int ret;

  nxsem_init(&g_lock, 0, 1);

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

void cxd56_uart0uninitialize(FAR const char *devname)
{
  unregister_driver(devname);
  nxsem_destroy(&g_lock);
}

#endif /* CONFIG_CXD56_UART0 */

/****************************************************************************
 * drivers/syslog/syslog_console.c
 *
 *   Copyright (C) 2015 Pierre-noel Bouteville. All rights reserved.
 *   Author: Pierre-noel Bouteville <pnb990@gmail.com>
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

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/syslog/syslog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The architecture must provide syslog_putc for this driver */

#if defined(CONFIG_SYSLOG)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t syslog_console_read(FAR struct file *filep, FAR char *buffer, 
                                   size_t buflen);
static ssize_t syslog_console_write(FAR struct file *filep,
                                    FAR const char *buffer, size_t buflen);
static int     syslog_console_ioctl(FAR struct file *filep, int cmd, 
                                    unsigned long arg);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static const struct file_operations g_consoleops =
{
  0,                    /* open */
  0,                    /* close */
  syslog_console_read,  /* read */
  syslog_console_write, /* write */
  0,                    /* seek */
  syslog_console_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0                   /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_console_ioctl
 ****************************************************************************/

static int syslog_console_ioctl(FAR struct file *filep, int cmd,
                                unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: syslog_console_read
 ****************************************************************************/

static ssize_t syslog_console_read(FAR struct file *filep, FAR char *buffer, 
                                   size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: syslog_console_write
 ****************************************************************************/

static ssize_t syslog_console_write(FAR struct file *filep,
                                    FAR const char *buffer, size_t buflen)
{
  ssize_t ret = buflen;

  for (; buflen; buflen--)
    {
      syslog_putc(*buffer++);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_console_init
****************************************************************************/

void syslog_console_init(void)
{
  (void)register_driver("/dev/console", &g_consoleops, 0666, NULL);
}
#endif /* CONFIG_SYSLOG */

/****************************************************************************
 * drivers/serial/ptmx.c
 *
 *   Copyright (C) 2016-2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/serial/pty.h>

#include "pty.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_PTY_MAXPTY
#  define CONFIG_PTY_MAXPTY 32
#endif

#if CONFIG_PTY_MAXPTY > 256
#  define CONFIG_PTY_MAXPTY 256
#endif

#define PTY_MAX   ((CONFIG_PTY_MAXPTY + 31) & ~31)
#define INDEX_MAX (PTY_MAX >> 5)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* PTMX device state */

struct ptmx_dev_s
{
  uint8_t px_next;                  /* Next minor number to allocate */
  sem_t px_exclsem;                 /* Supports mutual exclusion */
  uint32_t px_alloctab[INDEX_MAX];  /* Set of allocated PTYs */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     ptmx_open(FAR struct file *filep);
static ssize_t ptmx_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t ptmx_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ptmx_fops =
{
  ptmx_open,     /* open */
  NULL,          /* close */
  ptmx_read,     /* read */
  ptmx_write,    /* write */
  NULL,          /* seek */
  NULL,          /* ioctl */
  NULL           /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

static struct ptmx_dev_s g_ptmx;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ptmx_semtake and ptmx_semgive
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static void ptmx_semtake(void)
{
  nxsem_wait_uninterruptible(&g_ptmx.px_exclsem);
}

#define ptmx_semgive() nxsem_post(&g_ptmx.px_exclsem)

/****************************************************************************
 * Name: ptmx_minor_allocate
 *
 * Description:
 *   Allocate a new unique PTY minor number.
 *
 * Assumptions:
 *   Caller hold the px_exclsem
 *
 ****************************************************************************/

static int ptmx_minor_allocate(void)
{
  uint8_t startaddr = g_ptmx.px_next;
  uint8_t minor;
  int index;
  int bitno;

  /* Loop until we find a valid device address */

  for (; ; )
    {
      /* Try the next device address */

      minor = g_ptmx.px_next;
      if (g_ptmx.px_next >= PTY_MAX)
        {
          g_ptmx.px_next = 0;
        }
      else
        {
          g_ptmx.px_next++;
        }

      /* Is this address already allocated? */

      index = minor >> 5;
      bitno = minor & 31;
      if ((g_ptmx.px_alloctab[index] & (1 << bitno)) == 0)
        {
          /* No... allocate it now */

          g_ptmx.px_alloctab[index] |= (1 << bitno);
          return (int)minor;
        }

      /* This address has already been allocated.  The following logic will
       * prevent (unexpected) infinite loops.
       */

      if (startaddr == minor)
        {
          /* We are back where we started... the are no free device address */

          return -ENOMEM;
        }
    }
}

/****************************************************************************
 * Name: ptmx_open
 ****************************************************************************/

static int ptmx_open(FAR struct file *filep)
{
  char devname[16];
  int minor;
  int fd;
  int ret;

  /* Get exclusive access */

  ptmx_semtake();

  /* Allocate a PTY minor */

  minor = ptmx_minor_allocate();
  if (minor < 0)
    {
      ret = minor;
      goto errout_with_sem;
    }

  /* Create the master slave pair.  This should create:
   *
   *   Slave device:  /dev/pts/N
   *   Master device: /dev/ptyN
   *
   * Where N=minor
   */

  ret = pty_register(minor);
  if (ret < 0)
    {
      goto errout_with_minor;
    }

  /* Open the master device:  /dev/ptyN, where N=minor */

  snprintf(devname, 16, "/dev/pty%d", minor);
  fd = nx_open(devname, O_RDWR);
  DEBUGASSERT(fd >= 0);  /* nx_open() should never fail */

  /* No unlink the master.  This will remove it from the VFS namespace,
   * the driver will still persist because of the open count on the
   * driver.
   */

  ret = unlink(devname);
  DEBUGASSERT(ret >= 0);  /* unlink() should never fail */
  UNUSED(ret);

  /* Return the encoded, master file descriptor */

  ptmx_semgive();
  DEBUGASSERT((unsigned)fd <= OPEN_MAXFD);
  return (int)OPEN_SETFD(fd);

errout_with_minor:
  ptmx_minor_free(minor);

errout_with_sem:
  ptmx_semgive();
  return ret;
}

/****************************************************************************
 * Name: ptmx_read
 ****************************************************************************/

static ssize_t ptmx_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  return 0; /* Return EOF */
}

/****************************************************************************
 * Name: ptmx_write
 ****************************************************************************/

static ssize_t ptmx_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
  return len; /* Say that everything was written */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ptmx_register
 *
 * Input Parameters:
 *   None
 *
 * Description:
 *   Register the master pseudo-terminal multiplexor device at /dev/ptmx
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int ptmx_register(void)
{
  /* Initialize driver state */

  nxsem_init(&g_ptmx.px_exclsem, 0, 1);

  /* Register the PTMX driver */

  return register_driver("/dev/ptmx", &g_ptmx_fops, 0666, NULL);
}

/****************************************************************************
 * Name: ptmx_minor_free
 *
 * Description:
 *   De-allocate a PTY minor number.
 *
 * Assumptions:
 *   Caller hold the px_exclsem
 *
 ****************************************************************************/

void ptmx_minor_free(uint8_t minor)
{
  int index;
  int bitno;

  /* Free the address by clearing the associated bit in the px_alloctab[]; */

  index = minor >> 5;
  bitno = minor & 31;

  DEBUGASSERT((g_ptmx.px_alloctab[index] |= (1 << bitno)) != 0);
  g_ptmx.px_alloctab[index] &= ~(1 << bitno);

  /* Reset the next pointer if the one just released has a lower value */

  if (minor < g_ptmx.px_next)
    {
      g_ptmx.px_next = minor;
    }
}

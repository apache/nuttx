/****************************************************************************
 * drivers/serial/ptmx.c
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

  ret = nxsem_wait(&g_ptmx.px_exclsem);
  if (ret < 0)
    {
      return ret;
    }

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

  nxsem_post(&g_ptmx.px_exclsem);
  DEBUGASSERT((unsigned)fd <= OPEN_MAXFD);
  return (int)OPEN_SETFD(fd);

errout_with_minor:
  ptmx_minor_free(minor);

errout_with_sem:
  nxsem_post(&g_ptmx.px_exclsem);
  return ret;
}

/****************************************************************************
 * Name: ptmx_read
 ****************************************************************************/

static ssize_t ptmx_read(FAR struct file *filep,
                         FAR char *buffer, size_t len)
{
  return 0; /* Return EOF */
}

/****************************************************************************
 * Name: ptmx_write
 ****************************************************************************/

static ssize_t ptmx_write(FAR struct file *filep,
                          FAR const char *buffer, size_t len)
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

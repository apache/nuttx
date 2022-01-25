/****************************************************************************
 * drivers/modem/u-blox.c
 *
 *   Copyright (C) 2016 Vladimir Komendantskiy. All rights reserved.
 *   Author: Vladimir Komendantskiy <vladimir@moixaenergy.com>
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
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/modem/u-blox.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing the modem driver */

#ifdef CONFIG_MODEM_U_BLOX_DEBUG
#  define m_err     _err
#  define m_info    _info
#else
#  define m_err(x...)
#  define m_info(x...)
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* The type of upper half driver state. */

struct ubxmdm_upper
{
  FAR char * path;     /* Registration path */

  /* The contained lower-half driver. */

  FAR struct ubxmdm_lower * lower;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t ubxmdm_read (FAR struct file * filep,
                            FAR char * buffer,
                            size_t buflen);
static ssize_t ubxmdm_write(FAR struct file * filep,
                            FAR const char * buffer,
                            size_t buflen);
static int     ubxmdm_ioctl(FAR struct file * filep,
                            int cmd,
                            unsigned long arg);

static int     ubxmdm_poll (FAR struct file * filep,
                            FAR struct pollfd * fds,
                            bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations ubxmdm_fops =
{
  NULL,         /* open */
  NULL,         /* close */
  ubxmdm_read,  /* read */
  ubxmdm_write, /* write */
  NULL,         /* seek */
  ubxmdm_ioctl, /* ioctl */
  ubxmdm_poll   /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t ubxmdm_read(FAR struct file * filep,
                           FAR char * buffer,
                           size_t len)
{
  return 0; /* Return EOF */
}

static ssize_t ubxmdm_write(FAR struct file * filep,
                            FAR const char * buffer,
                            size_t len)
{
  return len; /* Say that everything was written */
}

static int ubxmdm_ioctl(FAR struct file * filep,
                        int cmd,
                        unsigned long arg)
{
  FAR struct inode *         inode = filep->f_inode;
  FAR struct ubxmdm_upper *  upper;
  FAR struct ubxmdm_lower *  lower;
  int                        ret;
  FAR struct ubxmdm_status * status;

  m_info("cmd: %d arg: %ld\n", cmd, arg);
  upper = inode->i_private;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  switch (cmd)
    {
      /* cmd:         UBXMDM_IOC_START
       * Description:
       * arg:         Ignored
       */

    case MODEM_IOC_POWERON:
      if (lower->ops->poweron)
        {
          ret = lower->ops->poweron(lower);
        }
      else
        {
          ret = -ENOSYS;
        }

      break;

      /* cmd:         UBXMDM_IOC_STOP
       * Description:
       * arg:         Ignored
       */

    case MODEM_IOC_POWEROFF:
      if (lower->ops->poweroff)
        {
          ret = lower->ops->poweroff(lower);
        }
      else
        {
          ret = -ENOSYS;
        }

      break;

      /* cmd:         UBXMDM_IOC_RESET
       * Description:
       * arg:         Ignored
       */

    case MODEM_IOC_RESET:
      if (lower->ops->reset)
        {
          ret = lower->ops->reset(lower);
        }
      else
        {
          ret = -ENOSYS;
        }

      break;

      /* cmd:         UBXMDM_IOC_GETSTATUS
       * Description:
       * arg:         Writeable pointer to struct ubxmdm_status.
       */

    case MODEM_IOC_GETSTATUS:
      if (lower->ops->getstatus)
        {
          status = (FAR struct ubxmdm_status *) ((uintptr_t) arg);
          if (status)
            {
              ret = lower->ops->getstatus(lower, status);
            }
          else
            {
              ret = -EINVAL;
            }
        }
      else
        {
          ret = -ENOSYS;
        }

      break;

      /* Unrecognized IOCTL commands are forwarded to the lower-half IOCTL
       * handler, if defined.
       */

    default:
      m_info("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);

      if (lower->ops->ioctl)
        {
          ret = lower->ops->ioctl(lower, cmd, arg);
        }
      else
        {
          ret = -ENOTTY;
        }

      break;
    }

  return ret;
}

static int ubxmdm_poll(FAR struct file * filep,
                       FAR struct pollfd * fds,
                       bool setup)
{
  if (setup)
    {
      fds->revents |= (fds->events & (POLLIN | POLLOUT));
      if (fds->revents != 0)
        {
          nxsem_post(fds->sem);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void * ubxmdm_register(FAR const char * path,
                          FAR struct ubxmdm_lower * lower)
{
  FAR struct ubxmdm_upper *upper;
  int ret;

  DEBUGASSERT(path && lower);

  upper = (FAR struct ubxmdm_upper *)
    kmm_zalloc(sizeof(struct ubxmdm_upper));
  if (!upper)
    {
      m_err("ERROR: Upper half allocation failed\n");
      goto errout;
    }

  upper->lower = lower;
  upper->path = strdup(path);
  if (!upper->path)
    {
      m_err("ERROR: Path allocation failed\n");
      goto errout_with_upper;
    }

  ret = register_driver(path, &ubxmdm_fops, 0666, upper);
  if (ret < 0)
    {
      m_err("ERROR: register_driver failed: %d\n", ret);
      goto errout_with_path;
    }

  return (FAR void *) upper;

errout_with_path:
  kmm_free(upper->path);

errout_with_upper:
  kmm_free(upper);

errout:
  return NULL;
}

void ubxmdm_unregister(FAR void *handle)
{
  FAR struct ubxmdm_upper *upper;
  FAR struct ubxmdm_lower *lower;

  upper = (FAR struct ubxmdm_upper *) handle;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  m_info("Unregistering: %s\n", upper->path);

  DEBUGASSERT(lower->ops->poweroff);
  lower->ops->poweroff(lower);

  unregister_driver(upper->path);

  kmm_free(upper->path);
  kmm_free(upper);
}

/****************************************************************************
 * drivers/modem/altair/altmdm.c
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

#include <string.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/modem/altmdm.h>
#include "altmdm_dev.h"
#include "altmdm_spi.h"
#include "altmdm_pm.h"
#include "altmdm_sys.h"

#if defined(CONFIG_MODEM_ALTMDM)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods. */

static int altmdm_open(FAR struct file *filep);
static int altmdm_close(FAR struct file *filep);
static ssize_t altmdm_read(FAR struct file *filep, FAR char *buffer,
                           size_t len);
static ssize_t altmdm_write(FAR struct file *filep, FAR const char *buffer,
                            size_t len);
static int altmdm_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface. */

static const struct file_operations g_altmdmfops =
{
  altmdm_open,                  /* open */
  altmdm_close,                 /* close */
  altmdm_read,                  /* read */
  altmdm_write,                 /* write */
  0,                            /* seek */
  altmdm_ioctl,                 /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: is_poweron
 *
 * Description:
 *   Check ALTMDM power on.
 *
 ****************************************************************************/

static int is_poweron(FAR struct altmdm_dev_s *priv)
{
  int poweron;

  altmdm_sys_lock(&priv->lock);

  poweron = priv->poweron;

  altmdm_sys_unlock(&priv->lock);

  return poweron;
}

/****************************************************************************
 * Name: altmdm_initialize
 *
 * Description:
 *   Initialize ALTMDM driver.
 *
 ****************************************************************************/

static int altmdm_initialize(FAR struct altmdm_dev_s *priv)
{
  int ret;

  ret = altmdm_sys_initlock(&priv->lock);
  if (ret == ERROR)
    {
      m_err("altmdm_sys_initlock() failed:%d\n", ret);
    }

  priv->poweron = 0;

  /* Initialize ALTMDM SPI driver. */

  ret = altmdm_spi_init(priv);

  return ret;
}

/****************************************************************************
 * Name: altmdm_uninitialize
 *
 * Description:
 *   Uninitialize ALTMDM driver.
 *
 ****************************************************************************/

static int altmdm_uninitialize(FAR struct altmdm_dev_s *priv)
{
  int ret;

  /* Uninitialize ALTMDM SPI driver */

  altmdm_spi_uninit(priv);

  ret = altmdm_sys_deletelock(&priv->lock);
  if (ret == ERROR)
    {
      m_err("altmdm_sys_deletelock() failed:%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: altmdm_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int altmdm_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: altmdm_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int altmdm_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: altmdm_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t altmdm_read(FAR struct file *filep,
                           FAR char *buffer,
                           size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct altmdm_dev_s *priv = inode->i_private;
  ssize_t rsize = -EPERM;

  rsize = altmdm_spi_read(priv, buffer, len);

  return rsize;
}

/****************************************************************************
 * Name: altmdm_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t altmdm_write(FAR struct file *filep, FAR const char *buffer,
                            size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct altmdm_dev_s *priv = inode->i_private;
  ssize_t wsize = -EPERM;

  if (is_poweron(priv))
    {
      wsize = altmdm_spi_write(priv, buffer, len);
    }

  return wsize;
}

/****************************************************************************
 * Name: altmdm_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int altmdm_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct altmdm_dev_s *priv = inode->i_private;
  int ret = -EPERM;

  switch (cmd)
    {
    case MODEM_IOC_POWERON:    /* Power on ALTMDM. */
      {
        altmdm_sys_lock(&priv->lock);

        if (!priv->poweron)
          {
            altmdm_pm_poweron(priv);

            priv->poweron = 1;

            ret = altmdm_spi_enable(priv);
          }
        else
          {
            ret = -EBUSY;
          }

        altmdm_sys_unlock(&priv->lock);
      }
      break;

    case MODEM_IOC_POWEROFF:   /* Power off ALTMDM. */
      {
        altmdm_sys_lock(&priv->lock);

        if (priv->poweron)
          {
            ret = altmdm_spi_disable(priv);

            altmdm_pm_poweroff(priv);

            priv->poweron = 0;
          }
        else
          {
            ret = -EBUSY;
          }

        altmdm_sys_unlock(&priv->lock);
      }
      break;

    case MODEM_IOC_READABORT:  /* Abort the read process. */
      {
        ret = altmdm_spi_readabort(priv);
      }
      break;

    case MODEM_IOC_SLEEP:      /* Make ALTMDM sleep. */
      {
        if (is_poweron(priv))
          {
            ret = altmdm_spi_sleepmodem(priv);
          }
      }
      break;

    case MODEM_IOC_PM_REGISTERCB:      /* Register callback function. */
      {
        ret = altmdm_pm_registercb(MODEM_PM_CB_TYPE_NORMAL,
                                   (altmdm_pm_cbfunc_t) arg);
      }
      break;

    case MODEM_IOC_PM_DEREGISTERCB:    /* Deregister callback function. */
      {
        ret = altmdm_pm_deregistercb(MODEM_PM_CB_TYPE_NORMAL);
      }
      break;

    case MODEM_IOC_PM_ERR_REGISTERCB:  /* Register error callback function. */
      {
        ret = altmdm_pm_registercb(MODEM_PM_CB_TYPE_ERROR,
                                   (altmdm_pm_cbfunc_t) arg);
      }
      break;

    case MODEM_IOC_PM_ERR_DEREGISTERCB:        /* Deregister error callback
                                                  function.
                                                */
      {
        ret = altmdm_pm_deregistercb(MODEM_PM_CB_TYPE_ERROR);
      }
      break;

    case MODEM_IOC_PM_GETSTATE:        /* Get ALTMDM power management state. */
      {
        *(uint32_t *) arg = altmdm_pm_getstate();
        ret = 0;
      }
      break;

    case MODEM_IOC_PM_INITWAKELOCK:    /* Initialize wakelock resource. */
      {
        ret = altmdm_pm_initwakelock((struct altmdm_pm_wakelock_s *)arg);
      }
      break;

    case MODEM_IOC_PM_ACQUIREWAKELOCK: /* Acquire wakelock. */
      {
        ret = altmdm_pm_acquirewakelock((struct altmdm_pm_wakelock_s *)arg);
      }
      break;

    case MODEM_IOC_PM_RELEASEWAKELOCK: /* Release wakelock. */
      {
        ret = altmdm_pm_releasewakelock((struct altmdm_pm_wakelock_s *)arg);
      }
      break;

    case MODEM_IOC_PM_GETNUMOFWAKELOCK:        /* Get number of wakelocks. */
      {
        ret = altmdm_pm_getnumofwakelock((struct altmdm_pm_wakelock_s *)arg);
      }
      break;

    case MODEM_IOC_PM_GETWAKELOCKSTATE:        /* Get wakelock state. */
      {
        ret = altmdm_pm_getwakelockstate();
      }
      break;

    default:
      m_err("Unrecognized cmd: 0x%08x\n", cmd);
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altmdm_register
 *
 * Description:
 *   Register the ALTMDM character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/altmdm".
 *   dev     - An instance of the SPI interface to use to communicate with
 *             ALTMDM.
 *
 * Returned Value:
 *   Not NULL on success; NULL on failure.
 *
 ****************************************************************************/

FAR void *altmdm_register(FAR const char *devpath, FAR struct spi_dev_s *dev,
                          FAR const struct altmdm_lower_s *lower)
{
  FAR struct altmdm_dev_s *priv;
  int ret;
  int size = sizeof(struct altmdm_dev_s);

  priv = (FAR struct altmdm_dev_s *)kmm_malloc(size);
  if (!priv)
    {
      m_err("Failed to allocate instance.\n");
      return NULL;
    }

  priv->spi = dev;
  priv->path = strdup(devpath);
  priv->lower = lower;

  ret = altmdm_initialize(priv);
  if (ret < 0)
    {
      m_err("Failed to initialize ALTMDM driver.\n");
      kmm_free(priv);
      return NULL;
    }

  ret = register_driver(devpath, &g_altmdmfops, 0666, priv);
  if (ret < 0)
    {
      m_err("Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  return (FAR void *)priv;
}

/****************************************************************************
 * Name: altmdm_unregister
 *
 * Description:
 *   Unregister the ALTMDM character device.
 *
 * Input Parameters:
 *   handle - The pointer that getting from altmdm_register.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void altmdm_unregister(FAR void *handle)
{
  FAR struct altmdm_dev_s *priv;

  if (handle)
    {
      priv = (FAR struct altmdm_dev_s *)handle;

      altmdm_uninitialize(priv);

      unregister_driver(priv->path);

      kmm_free(priv->path);
      kmm_free(priv);
    }
}

#endif

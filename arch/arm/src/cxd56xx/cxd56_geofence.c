/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_geofence.c
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
#include <fixedmath.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>

#include <arch/chip/geofence.h>
#include "cxd56_gnss_api.h"
#include "cxd56_cpu1signal.h"
#include "cxd56_gnss.h"

#if defined(CONFIG_CXD56_GEOFENCE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_GEOFENCE_NPOLLWAITERS
#  define CONFIG_GEOFENCE_NPOLLWAITERS    4
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cxd56_geofence_dev_s
{
  mutex_t        devlock;
  struct pollfd *fds[CONFIG_GEOFENCE_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* file operation functions */

static ssize_t cxd56_geofence_read(struct file *filep,
                                   char *buffer,
                                   size_t len);
static int cxd56_geofence_ioctl(struct file *filep,
                                int cmd,
                                unsigned long arg);
static int cxd56_geofence_poll(struct file *filep,
                               struct pollfd *fds,
                               bool setup);

/* ioctl command functions */

static int cxd56_geofence_start(unsigned long arg);
static int cxd56_geofence_stop(unsigned long arg);
static int cxd56_geofence_add_region(unsigned long arg);
static int cxd56_geofence_modify_region(unsigned long arg);
static int cxd56_geofence_delete_region(unsigned long arg);
static int cxd56_geofence_delete_all_region(unsigned long arg);
static int cxd56_geofence_get_region_data(unsigned long arg);
static int cxd56_geofence_get_used_id(unsigned long arg);
static int cxd56_geofence_get_all_status(unsigned long arg);
static int cxd56_geofence_set_mode(unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_geofencefops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  cxd56_geofence_read,  /* read */
  NULL,                 /* write */
  NULL,                 /* seek */
  cxd56_geofence_ioctl, /* ioctl */
  NULL,                 /* truncate */
  NULL,                 /* mmap */
  cxd56_geofence_poll   /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                /* unlink */
#endif
};

/* ioctl command list */

static int (*g_cmdlist[CXD56_GEOFENCE_IOCTL_MAX])(unsigned long) =
{
  NULL,              /* CXD56_GEOFENCE_IOCTL_INVAL = 0 */
  cxd56_geofence_start,
  cxd56_geofence_stop,
  cxd56_geofence_add_region,
  cxd56_geofence_modify_region,
  cxd56_geofence_delete_region,
  cxd56_geofence_delete_all_region,
  cxd56_geofence_get_region_data,
  cxd56_geofence_get_used_id,
  cxd56_geofence_get_all_status,
  cxd56_geofence_set_mode,

  /* max                CXD56_GEOFENCE_IOCTL_MAX */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_geofence_start
 *
 * Description:
 *   Process CXD56_GEOFENCE_IOCTL_START command.
 *   Start GEOFENCE Detect
 *
 * Input Parameters:
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_geofence_start(unsigned long arg)
{
  return fw_gd_registergeofence();
}

/****************************************************************************
 * Name: cxd56_geofence_stop
 *
 * Description:
 *   Process CXD56_GEOFENCE_IOCTL_STOP command.
 *   Stop GEOFENCE Detect
 *
 * Input Parameters:
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_geofence_stop(unsigned long arg)
{
  return fw_gd_releasegeofence();
}

/****************************************************************************
 * Name: cxd56_geofence_add_region
 *
 * Description:
 *   Process CXD56_GEOFENCE_IOCTL_ADD command.
 *   Add region
 *
 * Input Parameters:
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_geofence_add_region(unsigned long arg)
{
  int                             ret;
  struct cxd56_geofence_region_s *reg_data;

  if (!arg)
    {
      return -EINVAL;
    }

  reg_data = (struct cxd56_geofence_region_s *)arg;

  ret = fw_gd_geoaddregion(reg_data->id,
                        reg_data->latitude,
                        reg_data->longitude,
                        reg_data->radius);

  return ret;
}

/****************************************************************************
 * Name: cxd56_geofence_modify_region
 *
 * Description:
 *   Process CXD56_GEOFENCE_IOCTL_MODIFY command.
 *   Modify region
 *
 * Input Parameters:
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_geofence_modify_region(unsigned long arg)
{
  int                             ret;
  struct cxd56_geofence_region_s *reg_data;

  if (!arg)
    {
      return -EINVAL;
    }

  reg_data = (struct cxd56_geofence_region_s *)arg;

  ret = fw_gd_geomodifyregion(reg_data->id, reg_data->latitude,
                           reg_data->longitude, reg_data->radius);

  return ret;
}

/****************************************************************************
 * Name: cxd56_geofence_delete_region
 *
 * Description:
 *   Process CXD56_GEOFENCE_IOCTL_DELETE command.
 *   Delete region
 *
 * Input Parameters:
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_geofence_delete_region(unsigned long arg)
{
  int     ret;
  uint8_t id;

  if (UINT8_MAX < arg)
    {
      return -EINVAL;
    }

  id = (uint8_t)arg;
  ret = fw_gd_geodeleteregione(id);

  return ret;
}

/****************************************************************************
 * Name: cxd56_geofence_delete_all_region
 *
 * Description:
 *   Process CXD56_GEOFENCE_IOCTL_ALL_DELETE command.
 *   All delete region
 *
 * Input Parameters:
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_geofence_delete_all_region(unsigned long arg)
{
  int ret;

  ret = fw_gd_geodeleteallregion();

  return ret;
}

/****************************************************************************
 * Name: cxd56_geofence_get_region_data
 *
 * Description:
 *   Process CXD56_GEOFENCE_IOCTL_GET_REGION_DATA command.
 *   Get used region ID
 *
 * Input Parameters:
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_geofence_get_region_data(unsigned long arg)
{
  int                             ret;
  struct cxd56_geofence_region_s *reg_data;

  if (!arg)
    {
      return -EINVAL;
    }

  reg_data = (struct cxd56_geofence_region_s *)arg;

  ret = fw_gd_geogetregiondata(reg_data->id, &reg_data->latitude,
                            &reg_data->longitude, &reg_data->radius);

  return ret;
}

/****************************************************************************
 * Name: cxd56_geofence_get_used_id
 *
 * Description:
 *   Process CXD56_GEOFENCE_IOCTL_GET_USED_ID command.
 *   Get used region ID
 *
 * Input Parameters:
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_geofence_get_used_id(unsigned long arg)
{
  if (!arg)
    {
      return -EINVAL;
    }

  *(uint32_t *)arg = fw_gd_geogetusedregionid();

  return 0;
}

/****************************************************************************
 * Name: cxd56_geofence_get_all_status
 *
 * Description:
 *   Process CXD56_GEOFENCE_IOCTL_GET_ALL_STATUS command.
 *   Get All transition status
 *
 * Input Parameters:
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_geofence_get_all_status(unsigned long arg)
{
  int ret;

  ret = fw_gd_geosetallrgionnotifyrequest();

  return ret;
}

/****************************************************************************
 * Name: cxd56_geofence_set_mode
 *
 * Description:
 *   Process CXD56_GEOFENCE_IOCTL_SET_MODE command.
 *   Set geofence operation mode
 *
 * Input Parameters:
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_geofence_set_mode(unsigned long arg)
{
  int                           ret;
  struct cxd56_geofence_mode_s *mode;

  if (!arg)
    {
      return -EINVAL;
    }

  mode = (struct cxd56_geofence_mode_s *)arg;

  ret = fw_gd_geosetopmode(mode->deadzone, mode->dwell_detecttime);

  return ret;
}

/****************************************************************************
 * Name: cxd56_geofence_sighandler
 *
 * Description:
 *   Common signal handler from GNSS CPU.
 *
 * Input Parameters:
 *   data     - Received data from GNSS CPU
 *   userdata - User data, this is the device information specified by the
 *              second argument of the function cxd56_cpu1siginit.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void cxd56_geofence_sighandler(uint32_t data, void *userdata)
{
  struct cxd56_geofence_dev_s *priv =
    (struct cxd56_geofence_dev_s *)userdata;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return;
    }

  poll_notify(priv->fds, CONFIG_GEOFENCE_NPOLLWAITERS, POLLIN);

  nxmutex_unlock(&priv->devlock);
}

/****************************************************************************
 * Name: cxd56_geofence_initialize
 *
 * Description:
 *   initialize GEOFENCE device
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_geofence_initialize(struct cxd56_geofence_dev_s *dev)
{
  int32_t ret = 0;

  return ret;
}

/****************************************************************************
 * Name: cxd56_geofence_read
 *
 * Description:
 *   Standard character driver read method.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   buffer - Buffer to write
 *   buflen - The write length of the buffer
 *
 * Returned Value:
 *   Always returns -ENOENT error.
 *
 ****************************************************************************/

static ssize_t cxd56_geofence_read(struct file *filep, char *buffer,
                                   size_t len)
{
  /* Check argument */

  if (!buffer)
    {
      return -EINVAL;
    }

  if (len == 0)
    {
      return 0;
    }

  /* fw_gd_readbuffer returns copied data size or negative error code */

  return fw_gd_readbuffer(CXD56_CPU1_DEV_GEOFENCE, 0, buffer, len);
}

/****************************************************************************
 * Name: cxd56_geofence_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   fds   - Array of file descriptor
 *   setup - 1 if start poll, 0 if stop poll
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_geofence_ioctl(struct file *filep, int cmd,
                                unsigned long arg)
{
  if (cmd <= CXD56_GEOFENCE_IOCTL_INVAL || cmd >= CXD56_GEOFENCE_IOCTL_MAX)
    {
      return -EINVAL;
    }

  return g_cmdlist[cmd](arg);
}

/****************************************************************************
 * Name: cxd56_geofence_poll
 *
 * Description:
 *   Standard character driver poll method.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   fds   - array of file descriptor
 *   setup - 1 if start poll, 0 if stop poll
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_geofence_poll(struct file *filep,
                               struct pollfd *fds,
                               bool setup)
{
  struct inode                *inode;
  struct cxd56_geofence_dev_s *priv;
  int                          ret = OK;
  int                          i;

  inode = filep->f_inode;
  priv  = (struct cxd56_geofence_dev_s *)inode->i_private;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      for (i = 0; i < CONFIG_GEOFENCE_NPOLLWAITERS; i++)
        {
          /* Find an unused slot */

          if (priv->fds[i] == NULL)
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              fw_gd_setnotifymask(CXD56_CPU1_DEV_GEOFENCE, FALSE);
              break;
            }
        }

      /* No space in priv fds array for poll handling */

      if (i >= CONFIG_GEOFENCE_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret       = -EBUSY;
          goto errout;
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

errout:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: cxd56_geofence_register
 *
 * Description:
 *   Register the GEOFENCE character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/geofence"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_geofence_register(const char *devpath)
{
  struct cxd56_geofence_dev_s *priv;
  int                          ret;

  priv = (struct cxd56_geofence_dev_s *)kmm_zalloc(
    sizeof(struct cxd56_geofence_dev_s));
  if (!priv)
    {
      gnsserr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  nxmutex_init(&priv->devlock);

  ret = cxd56_geofence_initialize(priv);
  if (ret < 0)
    {
      gnsserr("Failed to initialize geofence device!\n");
      goto err0;
    }

  ret = register_driver(devpath, &g_geofencefops, 0666, priv);
  if (ret < 0)
    {
      gnsserr("Failed to register driver: %d\n", ret);
      goto err0;
    }

  ret = cxd56_cpu1siginit(CXD56_CPU1_DEV_GEOFENCE, priv);
  if (ret < 0)
    {
      gnsserr("Failed to initialize ICC for GPS CPU: %d\n", ret);
      goto err1;
    }

  cxd56_cpu1sigregisterhandler(CXD56_CPU1_DEV_GEOFENCE,
                               cxd56_geofence_sighandler);

  gnssinfo("GEOFENCE driver loaded successfully!\n");
  return ret;

err1:
  unregister_driver(devpath);

err0:
  nxmutex_destroy(&priv->devlock);
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: cxd56_geofenceinitialize
 *
 * Description:
 *   Initialize GEOFENCE device
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/geofence"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_geofenceinitialize(const char *devpath)
{
  int ret;

  gnssinfo("Initializing GEOFENCE..\n");

  ret = cxd56_geofence_register(devpath);
  if (ret < 0)
    {
      gnsserr("Error registering GEOFENCE\n");
    }

  return ret;
}

#endif

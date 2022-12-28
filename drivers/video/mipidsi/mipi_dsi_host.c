/****************************************************************************
 * drivers/video/mipidsi/mipi_dsi_host.c
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

#include <debug.h>

#include <nuttx/kmalloc.h>

#include "mipi_dsi.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mipi_dsi_hosts_s
{
  int count;
  struct mipi_dsi_host *hosts[0];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct mipi_dsi_hosts_s *g_hosts;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mipi_dsi_host_exist
 ****************************************************************************/

static bool mipi_dsi_host_exist(int bus)
{
  int i = 0;

  while (g_hosts != NULL && i < g_hosts->count)
    {
      if (g_hosts->hosts[i]->bus == bus)
        {
          return true;
        }

      i++;
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mipi_dsi_host_register
 *
 * Description:
 *   Register mipi dsi host, if defined CONFIG_MIPI_DSI_DRIVER, will create
 *   character device at /dev.
 *
 * Input Parameters:
 *   host - An instance of the dsi host
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int mipi_dsi_host_register(FAR struct mipi_dsi_host *host)
{
  DEBUGASSERT(host != NULL && host->ops != NULL);

  if (mipi_dsi_host_exist(host->bus))
    {
      return -EINVAL;
    }

  if (g_hosts == NULL)
    {
      g_hosts = kmm_zalloc(sizeof(struct mipi_dsi_hosts_s) +
                           sizeof(FAR struct mipi_dsi_host *));
    }
  else
    {
      g_hosts = kmm_realloc(g_hosts, sizeof(struct mipi_dsi_hosts_s) +
                                     sizeof(FAR struct mipi_dsi_host *) *
                                     (g_hosts->count + 1));
    }

  if (g_hosts == NULL)
    {
      return -ENOMEM;
    }

  g_hosts->hosts[g_hosts->count] = host;
  g_hosts->count++;

#ifdef CONFIG_MIPI_DSI_DRIVER
  return mipi_dsi_host_driver_register(host);
#else
  return OK;
#endif
}

/****************************************************************************
 * Name: mipi_dsi_host_get
 *
 * Description:
 *   Find host in list by bus number. Lcd driver can get host by this
 *   interface to register dsi device.
 *
 * Input Parameters:
 *   bus - The dsi host bus number.
 *
 * Returned Value:
 *   struct mipi_dsi_host pointer if the host was successfully registered;
 *   NULL pointer is returned on any failure.
 *
 ****************************************************************************/

FAR struct mipi_dsi_host *mipi_dsi_host_get(int bus)
{
  int i = 0;

  while (g_hosts != NULL && i < g_hosts->count)
    {
      if (g_hosts->hosts[i]->bus == bus)
        {
          return g_hosts->hosts[i];
        }

      i++;
    }

  return NULL;
}

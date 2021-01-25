/****************************************************************************
 * drivers/modem/altair/altmdm_dev.h
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

#ifndef __DRIVERS_MODEM_ALTAIR_ALTMDM_DEV_H
#define __DRIVERS_MODEM_ALTAIR_ALTMDM_DEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/modem/altmdm.h>

#include "altmdm_spi.h"
#include "altmdm_sys.h"

#if defined(CONFIG_MODEM_ALTMDM)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct altmdm_dev_s
{
  FAR char *path;             /* Registration path */
  FAR struct spi_dev_s *spi;
  struct altmdm_spi_dev_s spidev;
  struct altmdm_sys_lock_s lock;
  int poweron;
  FAR const struct altmdm_lower_s *lower;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altmdm_spi_init
 *
 * Description:
 *   Initialize ALTMDM driver.
 *
 ****************************************************************************/

int altmdm_spi_init(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_spi_uninit
 *
 * Description:
 *   Uninitialize ALTMDM driver.
 *
 ****************************************************************************/

int altmdm_spi_uninit(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_spi_enable
 *
 * Description:
 *   Enable ALTMDM SPI driver.
 *
 ****************************************************************************/

int altmdm_spi_enable(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_spi_disable
 *
 * Description:
 *   Disable ALTMDM SPI driver.
 *
 ****************************************************************************/

int altmdm_spi_disable(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_spi_read
 *
 * Description:
 *   ALTMDM SPI driver read method.
 *
 ****************************************************************************/

ssize_t altmdm_spi_read(FAR struct altmdm_dev_s *priv,
                        FAR const char *buffer, size_t readlen);

/****************************************************************************
 * Name: altmdm_spi_write
 *
 * Description:
 *   ALTMDM SPI driver write method.
 *
 ****************************************************************************/

ssize_t altmdm_spi_write(FAR struct altmdm_dev_s *priv,
                         FAR const char *buffer, size_t witelen);

/****************************************************************************
 * Name: altmdm_spi_readabort
 *
 * Description:
 *   Abort the read process.
 *
 ****************************************************************************/

int altmdm_spi_readabort(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_spi_sleepmodem
 *
 * Description:
 *   Make ALTMDM sleep.
 *
 ****************************************************************************/

int altmdm_spi_sleepmodem(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_spi_setreceiverready
 *
 * Description:
 *   Set receiver ready notification.
 *
 ****************************************************************************/

int altmdm_spi_setreceiverready(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_spi_isreceiverready
 *
 * Description:
 *   Check already notified or not by altmdm_spi_setreceiverready.
 *
 ****************************************************************************/

int altmdm_spi_isreceiverready(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_spi_clearreceiverready
 *
 * Description:
 *   Clear receiver ready notification.
 *
 ****************************************************************************/

int altmdm_spi_clearreceiverready(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_spi_gpioreadyisr
 *
 * Description:
 *   Interrupt handler for SLAVE_REQUEST GPIO line.
 *
 ****************************************************************************/

int altmdm_spi_gpioreadyisr(int irq, FAR void *context, FAR void *arg);

#endif
#endif /* __DRIVERS_MODEM_ALTAIR_ALTMDM_DEV_H */

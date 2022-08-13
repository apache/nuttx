/****************************************************************************
 * drivers/ipcc/ipcc_register.c
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
#include <nuttx/ipcc.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/circbuf.h>

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/types.h>

#include "ipcc_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device naming ************************************************************/

#define DEVNAME_FMT     "/dev/ipcc%d"
#define DEVNAME_FMTLEN  (9 + 3 + 1)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations ipcc_fops =
{
#ifdef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .unlink = NULL,
#else /* CONFIG_DISABLE_PSEUDOFS_OPERATIONS */
  .unlink = ipcc_unlink,
#endif /* CONFIG_DISABLE_PSEUDOFS_OPERATIONS */
  .open   = ipcc_open,
  .close  = ipcc_close,
  .poll   = ipcc_poll,
  .read   = ipcc_read,
  .write  = ipcc_write,
  .ioctl  = NULL,
  .seek   = NULL
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipcc_cleanup
 *
 * Description:
 *   Cleans up resources allocated by ipcc_register()
 *
 * Input Parameters:
 *   priv - ipcc driver instance to clean up
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   This function should be called only when ipcc_register is run with
 *   success and all resources in driver instance are properly allocated.
 *
 ****************************************************************************/

void ipcc_cleanup(FAR struct ipcc_driver_s *priv)
{
#ifdef CONFIG_IPCC_BUFFERED
  circbuf_uninit(&priv->ipcc->rxbuf);
  circbuf_uninit(&priv->ipcc->txbuf);
#endif
  nxsem_destroy(&priv->rxsem);
  nxsem_destroy(&priv->txsem);
  priv->ipcc->ops.cleanup(priv->ipcc);
  nxsem_destroy(&priv->exclsem);
  kmm_free(priv);
}

/****************************************************************************
 * Name: ipcc_register
 *
 * Description:
 *   Create and register the IPCC character driver.
 *
 *   IPCC is a simple character driver that supports inter processor
 *   communication.
 *
 * Input Parameters:
 *   ipcc - An instance of the lower half IPCC driver
 *   chan - IPCC channel. This will be used ad IPCC minor number.
 *     IPPC will be registered as /dev/ipccN where N is the minor number.
 *   buflen - Length of tx and rx buffers, 0 for unbuffered communication.
 *
 * Returned Value:
 *   OK if the driver was successfully registered, or negated errno on
 *   failure.
 *
 * Assumptions/Limitations:
 *   ipcc is already allocated and initialized by architecture code.
 *
 ****************************************************************************/

#ifdef CONFIG_IPCC_BUFFERED
int ipcc_register(FAR struct ipcc_lower_s *ipcc, size_t rxbuflen,
                  size_t txbuflen)
#else
int ipcc_register(FAR struct ipcc_lower_s *ipcc)
#endif
{
  FAR struct ipcc_driver_s *priv;
  char devname[DEVNAME_FMTLEN];
  int ret;

  /* Allocate a IPCC character device structure */

  if ((priv = kmm_zalloc(sizeof(*priv))) == NULL)
    {
      return -ENOMEM;
    }

  /* Link upper and lower driver together */

  priv->ipcc = ipcc;
  ipcc->upper = priv;

#ifdef CONFIG_IPCC_BUFFERED
  /* allocate buffers for reading and writing data to IPCC memory */

  if (rxbuflen)
    {
      if ((ret = circbuf_init(&priv->ipcc->rxbuf, NULL, rxbuflen)))
        {
          goto error;
        }
    }

  if (txbuflen)
    {
      if ((ret = circbuf_init(&priv->ipcc->txbuf, NULL, txbuflen)))
        {
          goto error;
        }
    }
#endif /* CONFIG_IPCC_BUFFERED */

  /* Create the character device name */

  snprintf(devname, DEVNAME_FMTLEN, DEVNAME_FMT, ipcc->chan);
  if ((ret = register_driver(devname, &ipcc_fops, 0666, priv)))
    {
      goto error;
    }

  /* nxsem_init can't really fail us if we provide it with valid params */

  nxsem_init(&priv->exclsem, 0, 1);
  nxsem_init(&priv->rxsem, 0, 0);
  nxsem_init(&priv->txsem, 0, 1);

  return OK;

error:
  ipcc_cleanup(priv);

  return ret;
}

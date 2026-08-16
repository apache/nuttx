/****************************************************************************
 * drivers/usbhost/usbhost_xhci_pci.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Finding an xHCI controller on a PCI bus.
 *
 * The controller itself is described by its specification and driven by
 * usbhost_xhci.c, which is the same code wherever the part is fitted.  This
 * file is only the part that is true of PCI and of nothing else: which
 * device identifiers to answer to, how to switch a device on and find its
 * register window, and how its interrupt is arranged.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pci/pci.h>
#include <nuttx/usb/usbhost.h>

#include "usbhost_xhci.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* What this file needs to remember about a controller, which is only the
 * things the PCI layer will ask for again when the device goes away.
 */

struct pci_xhci_s
{
  FAR struct pci_device_s         *dev;   /* The device we were given */
  FAR struct usbhost_connection_s *conn;  /* What the controller gave back */
  int                              irq;   /* Allocated MSI-X vector */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pci_xhci_irq_attach(FAR void *arg, xcpt_t handler,
                               FAR void *priv);
static void pci_xhci_irq_detach(FAR void *arg);
static int pci_xhci_probe(FAR struct pci_device_s *dev);
static void pci_xhci_remove(FAR struct pci_device_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct xhci_bus_ops_s g_pci_xhci_ops =
{
  .irq_attach = pci_xhci_irq_attach,
  .irq_detach = pci_xhci_irq_detach,
};

/* PCI device table */

static const struct pci_device_id_s g_pci_xhci_id_table[] =
{
  /* QEMU xHCI */

  {
    PCI_DEVICE(0x1b36, 0x000d),
    .driver_data = 0
  },

  /* Intel Alder Lake USB 3.2 xHCI controller */

  {
    PCI_DEVICE(0x8086, 0x51ed),
    .driver_data = 0
  },

  /* Intel Alder Lake-S USB 3.2 Gen 2x2 xHCI controller */

  {
    PCI_DEVICE(0x8086, 0x7ae0),
    .driver_data = 0
  },
  { }
};

/* PCI driver */

static struct pci_driver_s g_pci_xhci_drv =
{
  .id_table = g_pci_xhci_id_table,
  .probe    = pci_xhci_probe,
  .remove   = pci_xhci_remove,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_xhci_irq_attach
 *
 * Description:
 *   Give the controller an interrupt.  On PCI that means asking for a
 *   message rather than finding a wire, so the vector is allocated here and
 *   only then attached.
 *
 ****************************************************************************/

static int pci_xhci_irq_attach(FAR void *arg, xcpt_t handler, FAR void *priv)
{
  FAR struct pci_xhci_s *pcix = arg;
  int                    ret;

  ret = pci_alloc_irq(pcix->dev, &pcix->irq, 1);
  if (ret != 1)
    {
      pcierr("Failed to allocate MSI %d\n", ret);
      return ret;
    }

  irq_attach(pcix->irq, handler, priv);

  ret = pci_connect_irq(pcix->dev, &pcix->irq, 1);
  if (ret != OK)
    {
      pcierr("Failed to connect MSI %d\n", ret);
      pci_release_irq(pcix->dev, &pcix->irq, 1);

      return -ENOTSUP;
    }

  up_enable_irq(pcix->irq);

  return OK;
}

/****************************************************************************
 * Name: pci_xhci_irq_detach
 ****************************************************************************/

static void pci_xhci_irq_detach(FAR void *arg)
{
  FAR struct pci_xhci_s *pcix = arg;

  up_disable_irq(pcix->irq);
  irq_detach(pcix->irq);
  pci_release_irq(pcix->dev, &pcix->irq, 1);
}

/****************************************************************************
 * Name: pci_xhci_probe
 *
 * Description:
 *   Switch on a PCI xHCI controller and hand it to the controller driver.
 *
 ****************************************************************************/

static int pci_xhci_probe(FAR struct pci_device_s *dev)
{
  FAR struct pci_xhci_s *pcix;
  uintptr_t              base;
  int                    ret;

  pcix = kmm_zalloc(sizeof(struct pci_xhci_s));
  if (pcix == NULL)
    {
      return -ENOMEM;
    }

  pcix->dev = dev;
  dev->priv = pcix;

  /* The controller has to be able to reach memory on its own, and its
   * registers have to be answerable, before either is used.
   */

  pci_set_master(dev);
  pciinfo("Enabled bus mastering\n");
  pci_enable_device(dev);
  pciinfo("Enabled memory resources\n");

  base = (uintptr_t)pci_map_bar(dev, 0);
  if (base == 0)
    {
      pcierr("Not found BAR 0!\n");
      ret = -EIO;
      goto errout;
    }

  pcix->conn = xhci_initialize("usb", base, &g_pci_xhci_ops, pcix);
  if (pcix->conn == NULL)
    {
      pcierr("xhci_initialize failed\n");
      ret = -EIO;
      goto errout;
    }

  return OK;

errout:
  pci_clear_master(dev);
  pci_disable_device(dev);
  kmm_free(pcix);

  return ret;
}

/****************************************************************************
 * Name: pci_xhci_remove
 ****************************************************************************/

static void pci_xhci_remove(FAR struct pci_device_s *dev)
{
  FAR struct pci_xhci_s *pcix = dev->priv;

  if (pcix == NULL)
    {
      return;
    }

  xhci_uninitialize(pcix->conn);

  pci_clear_master(dev);
  pci_disable_device(dev);

  dev->priv = NULL;
  kmm_free(pcix);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_xhci_init
 *
 * Description:
 *   Initialize the USB host xHCI as PCI device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int pci_xhci_init(void)
{
  return pci_register_driver(&g_pci_xhci_drv);
}

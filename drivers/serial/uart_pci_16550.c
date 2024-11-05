/*****************************************************************************
 * drivers/serial/uart_pci_16550.c
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
 *****************************************************************************/

/* Serial driver for 16550 UART PCI */

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/param.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <debug.h>

#include <nuttx/spinlock.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/pci/pci.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/uart_pci_16550.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#define PCI_U16550_DEV_PATH0 "/dev/ttyS0"
#define PCI_U16550_DEV_PATH1 "/dev/ttyS1"
#define PCI_U16550_DEV_PATH2 "/dev/ttyS2"
#define PCI_U16550_DEV_PATH3 "/dev/ttyS3"

/* UART PCI console support */

#if defined(CONFIG_16550_PCI_UART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV g_pci_u16550_dev0
#elif defined(CONFIG_16550_PCI_UART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV g_pci_u16550_dev1
#elif defined(CONFIG_16550_PCI_UART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV g_pci_u16550_dev2
#elif defined(CONFIG_16550_PCI_UART3_SERIAL_CONSOLE)
#  define CONSOLE_DEV g_pci_u16550_dev3
#endif

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/* Extend default PCI devie type */

struct pci_u16550_type_s
{
  uint8_t  ports;               /* Number of ports */
  uint8_t  regincr;             /* Address increment */
  uint8_t  portincr;            /* Port address increment */
};

/* Extend default UART 16550 strucutre */

struct pci_u16550_priv_s
{
  /* Common UART 16550 data must be first */

  struct u16550_s          common;
  FAR struct pci_device_s *pcidev;
  uint16_t                 vendor;
  uint16_t                 device;
  uint8_t                  port;
  FAR const char          *path;
};

/*****************************************************************************
 * Private Functions Definitions
 *****************************************************************************/

static uart_datawidth_t pci_u16550_getreg_mem(FAR struct u16550_s *priv,
                                              unsigned int offset);
static void pci_u16550_putreg_mem(FAR struct u16550_s *priv,
                                  unsigned int offset,
                                  uart_datawidth_t value);
static uart_datawidth_t pci_u16550_getreg_io(FAR struct u16550_s *priv,
                                              unsigned int offset);
static void pci_u16550_putreg_io(FAR struct u16550_s *priv,
                                  unsigned int offset,
                                  uart_datawidth_t value);
static int pci_u16550_ioctl(FAR struct u16550_s *priv, int cmd,
                            unsigned long arg);

static FAR struct dma_chan_s *pci_u16550_dmachan(FAR struct u16550_s *priv,
                                                 unsigned int ident);
static int pci_u16550_interrupt(int irq, FAR void *context, FAR void *arg);
static int pci_u16550_initialize(FAR struct pci_u16550_priv_s       *priv,
                                 FAR const struct pci_u16550_type_s *type,
                                 uintptr_t                           base,
                                 FAR struct pci_device_s            *dev,
                                 bool                                mmio);
static int pci_u16550_register(FAR uart_dev_t *dev);
static int pci_u16550_probe(FAR struct pci_device_s *dev);

/*****************************************************************************
 * Private Data
 *****************************************************************************/

#ifdef CONFIG_16550_PCI_UART_QEMU

static const struct pci_u16550_type_s g_pci_u16550_qemu_x1 =
{
  .ports    = 1,
  .regincr  = 1,
  .portincr = 0,
};

static const struct pci_u16550_type_s g_pci_u16550_qemu_x2 =
{
  .ports    = 2,
  .regincr  = 1,
  .portincr = 8,
};

static const struct pci_u16550_type_s g_pci_u16550_qemu_x4 =
{
  .ports    = 4,
  .regincr  = 1,
  .portincr = 8,
};
#endif  /* CONFIG_16550_PCI_UART_QEMU */

#ifdef CONFIG_16550_PCI_UART_AX99100
static const struct pci_u16550_type_s g_pci_u16550_ax99100_x2 =
{
  .ports    = 2,
  .regincr  = 1,
  .portincr = 8,
};
#endif  /* CONFIG_16550_PCI_UART_AX99100 */

static const struct pci_device_id_s g_pci_u16550_id_table[] =
{
#ifdef CONFIG_16550_PCI_UART_QEMU
  {
    PCI_DEVICE(0x1b36, 0x0002),
    .driver_data = (uintptr_t)&g_pci_u16550_qemu_x1
  },
  {
    PCI_DEVICE(0x1b36, 0x0003),
    .driver_data = (uintptr_t)&g_pci_u16550_qemu_x2
  },
  {
    PCI_DEVICE(0x1b36, 0x0004),
    .driver_data = (uintptr_t)&g_pci_u16550_qemu_x4
  },
#endif
#ifdef CONFIG_16550_PCI_UART_AX99100
  {
    PCI_DEVICE(0x125b, 0x9100),
    .driver_data = (uintptr_t)&g_pci_u16550_ax99100_x2
  },
#endif
  { }
};

static struct pci_driver_s g_pci_u16550_drv =
{
  .id_table = g_pci_u16550_id_table,
  .probe    = pci_u16550_probe,
};

/* UART 16550 ops for MMIO operations */

static const struct u16550_ops_s g_pci_u16550_mem_ops =
{
  .isr     = pci_u16550_interrupt,
  .getreg  = pci_u16550_getreg_mem,
  .putreg  = pci_u16550_putreg_mem,
  .ioctl   = pci_u16550_ioctl,
  .dmachan = pci_u16550_dmachan,
};

/* UART 16550 ops for IO operations */

static const struct u16550_ops_s g_pci_u16550_io_ops =
{
  .isr     = pci_u16550_interrupt,
  .getreg  = pci_u16550_getreg_io,
  .putreg  = pci_u16550_putreg_io,
  .ioctl   = pci_u16550_ioctl,
  .dmachan = pci_u16550_dmachan,
};

/* I/O buffers */

#ifdef CONFIG_16550_PCI_UART0
static char g_pci_u16550_rxbuffer0[CONFIG_16550_PCI_UART0_RXBUFSIZE];
static char g_pci_u16550_txbuffer0[CONFIG_16550_PCI_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_16550_PCI_UART1
static char g_pci_u16550_rxbuffer1[CONFIG_16550_PCI_UART1_RXBUFSIZE];
static char g_pci_u16550_txbuffer1[CONFIG_16550_PCI_UART1_TXBUFSIZE];
#endif

#ifdef CONFIG_16550_PCI_UART2
static char g_pci_u16550_rxbuffer2[CONFIG_16550_PCI_UART2_RXBUFSIZE];
static char g_pci_u16550_txbuffer2[CONFIG_16550_PCI_UART2_TXBUFSIZE];
#endif

#ifdef CONFIG_16550_PCI_UART3
static char g_pci_u16550_rxbuffer3[CONFIG_16550_PCI_UART3_RXBUFSIZE];
static char g_pci_u16550_txbuffer3[CONFIG_16550_PCI_UART3_TXBUFSIZE];
#endif

/* This describes the state of the 16550 UART0 PCI port. */

#ifdef CONFIG_16550_PCI_UART0
static struct pci_u16550_priv_s g_pci_u16550_priv0 =
{
  /* UART 16550 common data */

  .common =
  {
    .baud      = CONFIG_16550_PCI_UART0_BAUD,
    .uartclk   = CONFIG_16550_PCI_UART0_CLOCK,
    .parity    = CONFIG_16550_PCI_UART0_PARITY,
    .bits      = CONFIG_16550_PCI_UART0_BITS,
    .stopbits2 = CONFIG_16550_PCI_UART0_2STOP,
#if defined(CONFIG_16550_PCI_UART0_IFLOWCONTROL) || \
    defined(CONFIG_16550_PCI_UART0_OFLOWCONTROL)
    .flow      = true,
#endif
    .rxtrigger = 2,
  },

  /* PCI specific data */

  .vendor      = CONFIG_16550_PCI_UART0_VENDOR,
  .device      = CONFIG_16550_PCI_UART0_DEVICE,
  .port        = CONFIG_16550_PCI_UART0_PORT,
  .path        = PCI_U16550_DEV_PATH0
};

static uart_dev_t g_pci_u16550_dev0 =
{
  .recv     =
  {
    .size   = CONFIG_16550_PCI_UART0_RXBUFSIZE,
    .buffer = g_pci_u16550_rxbuffer0,
  },
  .xmit     =
  {
    .size   = CONFIG_16550_PCI_UART0_TXBUFSIZE,
    .buffer = g_pci_u16550_txbuffer0,
  },
  .priv     = &g_pci_u16550_priv0.common,
#ifdef CONFIG_16550_PCI_UART0_SERIAL_CONSOLE
  .isconsole = true,
#endif
};
#endif

/* This describes the state of the 16550 UART1 PCI port. */

#ifdef CONFIG_16550_PCI_UART1
static struct pci_u16550_priv_s g_pci_u16550_priv1 =
{
  /* UART 16550 common data */

  .common =
  {
    .baud      = CONFIG_16550_PCI_UART1_BAUD,
    .uartclk   = CONFIG_16550_PCI_UART1_CLOCK,
    .parity    = CONFIG_16550_PCI_UART1_PARITY,
    .bits      = CONFIG_16550_PCI_UART1_BITS,
    .stopbits2 = CONFIG_16550_PCI_UART1_2STOP,
#if defined(CONFIG_16550_PCI_UART1_IFLOWCONTROL) || \
    defined(CONFIG_16550_PCI_UART1_OFLOWCONTROL)
    .flow      = true,
#endif
    .rxtrigger = 2,
  },

  /* PCI specific data */

  .vendor      = CONFIG_16550_PCI_UART1_VENDOR,
  .device      = CONFIG_16550_PCI_UART1_DEVICE,
  .port        = CONFIG_16550_PCI_UART1_PORT,
  .path        = PCI_U16550_DEV_PATH1
};

static uart_dev_t g_pci_u16550_dev1 =
{
  .recv     =
  {
    .size   = CONFIG_16550_PCI_UART1_RXBUFSIZE,
    .buffer = g_pci_u16550_rxbuffer1,
  },
  .xmit     =
  {
    .size   = CONFIG_16550_PCI_UART1_TXBUFSIZE,
    .buffer = g_pci_u16550_txbuffer1,
  },
  .priv     = &g_pci_u16550_priv1.common,
#ifdef CONFIG_16550_PCI_UART1_SERIAL_CONSOLE
  .isconsole = true,
#endif
};
#endif

/* This describes the state of the 16550 UART2 PCI port. */

#ifdef CONFIG_16550_PCI_UART2
static struct pci_u16550_priv_s g_pci_u16550_priv2 =
{
  /* UART 16550 common data */

  .common =
  {
    .baud      = CONFIG_16550_PCI_UART2_BAUD,
    .uartclk   = CONFIG_16550_PCI_UART2_CLOCK,
    .parity    = CONFIG_16550_PCI_UART2_PARITY,
    .bits      = CONFIG_16550_PCI_UART2_BITS,
    .stopbits2 = CONFIG_16550_PCI_UART2_2STOP,
#if defined(CONFIG_16550_PCI_UART2_IFLOWCONTROL) || \
    defined(CONFIG_16550_PCI_UART2_OFLOWCONTROL)
    .flow      = true,
#endif
    .rxtrigger = 2,
  },

  /* PCI specific data */

  .vendor      = CONFIG_16550_PCI_UART2_VENDOR,
  .device      = CONFIG_16550_PCI_UART2_DEVICE,
  .port        = CONFIG_16550_PCI_UART2_PORT,
  .path        = PCI_U16550_DEV_PATH2
};

static uart_dev_t g_pci_u16550_dev2 =
{
  .recv     =
  {
    .size   = CONFIG_16550_PCI_UART2_RXBUFSIZE,
    .buffer = g_pci_u16550_rxbuffer2,
  },
  .xmit     =
  {
    .size   = CONFIG_16550_PCI_UART2_TXBUFSIZE,
    .buffer = g_pci_u16550_txbuffer2,
  },
  .priv     = &g_pci_u16550_priv2.common,
#ifdef CONFIG_16550_PCI_UART2_SERIAL_CONSOLE
  .isconsole = true,
#endif
};
#endif

#ifdef CONFIG_16550_PCI_UART3
static struct pci_u16550_priv_s g_pci_u16550_priv3 =
{
  /* UART 16550 common data */

  .common =
  {
    .baud      = CONFIG_16550_PCI_UART3_BAUD,
    .uartclk   = CONFIG_16550_PCI_UART3_CLOCK,
    .parity    = CONFIG_16550_PCI_UART3_PARITY,
    .bits      = CONFIG_16550_PCI_UART3_BITS,
    .stopbits2 = CONFIG_16550_PCI_UART3_2STOP,
#if defined(CONFIG_16550_PCI_UART3_IFLOWCONTROL) || \
    defined(CONFIG_16550_PCI_UART3_OFLOWCONTROL)
    .flow      = true,
#endif
    .rxtrigger = 2,
  },

  /* PCI specific data */

  .vendor      = CONFIG_16550_PCI_UART3_VENDOR,
  .device      = CONFIG_16550_PCI_UART3_DEVICE,
  .port        = CONFIG_16550_PCI_UART3_PORT,
  .path        = PCI_U16550_DEV_PATH3
};

static uart_dev_t g_pci_u16550_dev3 =
{
  .recv     =
  {
    .size   = CONFIG_16550_PCI_UART3_RXBUFSIZE,
    .buffer = g_pci_u16550_rxbuffer3,
  },
  .xmit     =
  {
    .size   = CONFIG_16550_PCI_UART3_TXBUFSIZE,
    .buffer = g_pci_u16550_txbuffer3,
  },
  .priv     = &g_pci_u16550_priv3.common,
#ifdef CONFIG_16550_PCI_UART3_SERIAL_CONSOLE
  .isconsole = true,
#endif
};
#endif

/* PCI devices */

static uart_dev_t *const g_pci_u16550_dev[] =
{
#ifdef CONFIG_16550_PCI_UART0
  &g_pci_u16550_dev0,
#endif
#ifdef CONFIG_16550_PCI_UART1
  &g_pci_u16550_dev1,
#endif
#ifdef CONFIG_16550_PCI_UART2
  &g_pci_u16550_dev2,
#endif
#ifdef CONFIG_16550_PCI_UART3
  &g_pci_u16550_dev3,
#endif
};

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: pci_u16550_getreg_mem
 *****************************************************************************/

static uart_datawidth_t pci_u16550_getreg_mem(FAR struct u16550_s *priv,
                                              unsigned int offset)
{
  uintptr_t addr = priv->uartbase + offset;

  return *((FAR volatile uart_datawidth_t *)addr);
}

/*****************************************************************************
 * Name: pci_u16550_putreg_mem
 *****************************************************************************/

static void pci_u16550_putreg_mem(FAR struct u16550_s *priv,
                                  unsigned int offset,
                                  uart_datawidth_t value)
{
  uintptr_t addr = priv->uartbase + offset;

  *((FAR volatile uart_datawidth_t *)addr) = value;
}

/*****************************************************************************
 * Name: pci_u16550_getreg_io
 *****************************************************************************/

static uart_datawidth_t pci_u16550_getreg_io(FAR struct u16550_s *priv,
                                             unsigned int offset)
{
  FAR struct pci_u16550_priv_s *p    = (FAR struct pci_u16550_priv_s *)priv;
  uintptr_t                     addr = priv->uartbase + offset;
  uint8_t                       ret  = 0;

  pci_read_io_byte(p->pcidev, addr, &ret);
  return ret;
}

/*****************************************************************************
 * Name: pci_u16550_putreg_io
 *****************************************************************************/

static void pci_u16550_putreg_io(FAR struct u16550_s *priv,
                                 unsigned int offset,
                                 uart_datawidth_t value)
{
  FAR struct pci_u16550_priv_s *p    = (FAR struct pci_u16550_priv_s *)priv;
  uintptr_t                     addr = priv->uartbase + offset;

  pci_write_io_byte(p->pcidev, addr, value);
}

/*****************************************************************************
 * Name: pci_u16550_ioctl
 *****************************************************************************/

static int pci_u16550_ioctl(FAR struct u16550_s *priv, int cmd,
                            unsigned long arg)
{
  return -ENOTTY;
}

/*****************************************************************************
 * Name: pci_u16550_dmachan
 *****************************************************************************/

static FAR struct dma_chan_s *pci_u16550_dmachan(FAR struct u16550_s *priv,
                                                 unsigned int ident)
{
  return NULL;
}

/*****************************************************************************
 * Name: pci_u16550_interrupt
 *
 * Description:
 *   Handle PCI interrupt.
 *
 *****************************************************************************/

static int pci_u16550_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct uart_dev_s *dev = (FAR struct uart_dev_s *)arg;

  DEBUGASSERT(dev != NULL);
  u16550_interrupt(0, NULL, dev);

  return OK;
}

/*****************************************************************************
 * Name: pci_u16550_initialize
 *
 * Description:
 *   Initialize UART 16550 PCI device.
 *
 *****************************************************************************/

static int pci_u16550_initialize(FAR struct pci_u16550_priv_s       *priv,
                                 FAR const struct pci_u16550_type_s *type,
                                 uintptr_t                           base,
                                 FAR struct pci_device_s            *dev,
                                 bool                                mmio)
{
  int ret = 0;
  int offset;

  /* Configure UART PCI */

  priv->common.uartbase = base;

  if (mmio)
    {
      priv->common.ops = &g_pci_u16550_mem_ops;
    }
  else
    {
      priv->common.ops = &g_pci_u16550_io_ops;
    }

  priv->common.regincr = type->regincr;
  priv->pcidev         = dev;

  /* Make sure that all interrupts are disabled otherwise spurious MSI
   * interrupt can happen just after we connect MSI.
   */

  offset = (priv->common.regincr * sizeof(uart_datawidth_t) *
            UART_IER_OFFSET);
  priv->common.ops->putreg(&priv->common, offset, 0);

  /* Allocate and connect MSI if supported */

  ret = pci_alloc_irq(dev, &priv->common.irq, 1);
  if (ret != 1)
    {
      pcierr("Failed to allocate MSI %d\n", ret);
      goto legacy_irq;
    }

  ret = pci_connect_irq(dev, &priv->common.irq, 1);
  if (ret == OK)
    {
      return OK;
    }

  pci_release_irq(dev, &priv->common.irq, 1);

legacy_irq:

  /* Get legacy IRQ if MSI not supported */

  priv->common.irq = pci_get_irq(dev);

  return OK;
}

/*****************************************************************************
 * Name: pci_u16550_register
 *
 * Description:
 *   Register UART 16550 PCI device.
 *
 *****************************************************************************/

static int pci_u16550_register(FAR uart_dev_t *dev)
{
  FAR struct pci_u16550_priv_s *priv   =
    (FAR struct pci_u16550_priv_s *)dev->priv;
  int                           ret = OK;

  /* Bind with 16550 common driver */

  ret = u16550_bind(dev);
  if (ret < 0)
    {
      /* No associated device found */

      return ret;
    }

  DEBUGASSERT(dev->ops);

  /* Register driver */

  pciinfo("Register %s", priv->path);

  ret = uart_register(priv->path, dev);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_16550_PCI_CONSOLE
  /* Register console */

  if (dev->isconsole)
    {
      ret = uart_register("/dev/console", dev);
    }
#endif

  return ret;
}

/*****************************************************************************
 * Name: pci_u16550_probe
 *
 * Description:
 *   Initialize device.
 *
 *****************************************************************************/

static int pci_u16550_probe(FAR struct pci_device_s *dev)
{
  FAR const struct pci_u16550_type_s *type = NULL;
  FAR uart_dev_t                     *udev = NULL;
  FAR struct pci_u16550_priv_s       *priv = NULL;
  uintptr_t                           base = 0;
  size_t                              i;
  uint8_t                             port;
  bool                                mmio = false;
  int                                 ret;

  /* Get type data associated with this PCI device card */

  type = (FAR const struct pci_u16550_type_s *)dev->id->driver_data;

  /* Not found private data */

  if (type == NULL)
    {
      return -ENODEV;
    }

  pci_set_master(dev);
  pciinfo("Enabled bus mastering\n");
  pci_enable_device(dev);
  pciinfo("Enabled memory resources\n");

  /* Hardcode BAR 0 for now */

  if (pci_resource_flags(dev, 0) == PCI_RESOURCE_IO)
    {
      base = pci_resource_start(dev, 0);
    }
  else
    {
      /* If the BAR is MMIO then it must be mapped */

      base = (uintptr_t)pci_map_bar(dev, 0);
      mmio = true;
    }

  for (port = 0; port < type->ports; port++)
    {
      /* Get port address */

      base += type->portincr * port;

      /* Take the instance that matches the configuration */

      udev = NULL;
      for (i = 0; i < nitems(g_pci_u16550_dev); i++)
        {
          udev = g_pci_u16550_dev[i];
          priv = (FAR struct pci_u16550_priv_s *)udev->priv;

          if (priv->vendor == dev->vendor &&
              priv->device == dev->device &&
              priv->port == port)
            {
              break;
            }
        }

      /* Not found */

      if (udev == NULL)
        {
          return -ENODEV;
        }

      /* Device already registered */

      if (udev->ops != NULL)
        {
          return -EBUSY;
        }

      /* Initialize device */

      ret = pci_u16550_initialize(priv, type, base, dev, mmio);
      if (ret < 0)
        {
          return ret;
        }

      /* Register UART device */

      ret = pci_u16550_register(udev);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/*****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes.
 *
 *****************************************************************************/

#ifdef CONFIG_16550_PCI_CONSOLE
void up_putc(int ch)
{
  irqstate_t flags;

  /* Console not initialized yet */

  if (CONSOLE_DEV.ops == NULL)
    {
      return;
    }

  /* All interrupts must be disabled to prevent re-entrancy and to prevent
   * interrupts from firing in the serial driver code.
   */

  flags = spin_lock_irqsave(NULL);
  u16550_putc(CONSOLE_DEV.priv, ch);
  spin_unlock_irqrestore(NULL, flags);
}
#endif

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: pci_u16550_init
 *
 * Description:
 *   Register a pci driver
 *
 *****************************************************************************/

int pci_u16550_init(void)
{
  return pci_register_driver(&g_pci_u16550_drv);
}

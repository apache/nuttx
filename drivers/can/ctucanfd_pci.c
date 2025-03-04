/*****************************************************************************
 * drivers/can/ctucanfd_pci.c
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

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pci/pci.h>

#include <nuttx/can/can.h>

#ifdef CONFIG_CAN_CTUCANFD_SOCKET
#  include <nuttx/wqueue.h>
#  include <nuttx/net/netdev.h>
#  include <nuttx/net/can.h>
#endif

#include "ctucanfd.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* PCI BARs */

#define CTUCANFD_BAR0         0
#define CTUCANFD_CTUCAN_BAR1  1

/* Registers per channel */

#define CTUCANFD_CTUCAN_REGS  0x4000

/* ID register in BAR0 */

#define CTUCANFD_BAR0_ID      0

/* Interrupts */

#define CTUCANFD_INT_ERR       (CTUCANFD_INT_DOI | CTUCANFD_INT_ALI | \
                                CTUCANFD_INT_BEI)

/* Supported TX buffers */

#define CTUCANFD_TXBUF_CNT    4

/* SocketCAN specific */

#ifdef CONFIG_CAN_CTUCANFD_SOCKET

#  define CTUCANFD_POOL_SIZE            1

/* Work queue support is required. */

#  if !defined(CONFIG_SCHED_WORKQUEUE)
#    error Work queue support is required
#  endif

#  define CTUCANFD_CANWORK              LPWORK

#endif /* CONFIG_CAN_CTUCANFD_SOCKET */

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/* CTUCANFD channel private data */

struct ctucanfd_can_s
{
#ifdef CONFIG_CAN_CTUCANFD_CHARDEV
  struct can_dev_s   dev;
#endif

#ifdef CONFIG_CAN_CTUCANFD_SOCKET
  struct net_driver_s dev;      /* Interface understood by the network */
  struct work_s       pollwork; /* For deferring poll work to the work wq */

  /* TX/RX pool */

#  ifdef CONFIG_NET_CAN_CANFD
  struct canfd_frame tx_pool[CTUCANFD_POOL_SIZE];
  struct canfd_frame rx_pool[CTUCANFD_POOL_SIZE];
#  else
  struct can_frame tx_pool[CTUCANFD_POOL_SIZE];
  struct can_frame rx_pool[CTUCANFD_POOL_SIZE];
#  endif
#endif

  FAR struct pci_device_s *pcidev;
  uint64_t                 base;
  uint8_t                  txbufcnt;
};

/* CTUCANFD private data */

struct ctucanfd_driver_s
{
  FAR struct ctucanfd_can_s *devs;
  uint8_t                    count;
  uintptr_t                  bar0_base;
  uintptr_t                  canfd_base;

  /* PCI data */

  FAR struct pci_device_s   *pcidev;
  int                        irq;
  uintptr_t                  base;

#ifdef CONFIG_CAN_CTUCANFD_SOCKET
  /* For deferring interrupt work to the wq */

  struct work_s              irqwork;
#endif
};

/*****************************************************************************
 * Private Functions Definitions
 *****************************************************************************/

/* Helpers */

static uint32_t ctucanfd_getreg(FAR struct ctucanfd_can_s *priv,
                                unsigned int offset);
static void ctucanfd_putreg(FAR struct ctucanfd_can_s *priv,
                            unsigned int offset,
                            uint32_t value);

/* Common methods */

static void ctucanfd_reset(FAR struct ctucanfd_can_s *priv);
static void ctucanfd_shutdown(FAR struct ctucanfd_can_s *priv);
static void ctucanfd_setup(FAR struct ctucanfd_can_s *priv);
static void ctucanfd_rxint(FAR struct ctucanfd_can_s *priv, bool enable);
static void ctucanfd_txint(FAR struct ctucanfd_can_s *priv, bool enable);
static bool ctucanfd_txready(FAR struct ctucanfd_can_s *priv);

#ifdef CONFIG_CAN_CTUCANFD_CHARDEV
/* CAN character device methods */

static void ctucanfd_chrdev_reset(FAR struct can_dev_s *dev);
static int  ctucanfd_chrdev_setup(FAR struct can_dev_s *dev);
static void ctucanfd_chrdev_shutdown(FAR struct can_dev_s *dev);
static void ctucanfd_chrdev_rxint(FAR struct can_dev_s *dev, bool enable);
static void ctucanfd_chrdev_txint(FAR struct can_dev_s *dev, bool enable);
static int  ctucanfd_chrdev_ioctl(FAR struct can_dev_s *dev, int cmd,
                                  unsigned long arg);
static int  ctucanfd_chrdev_remoterequest(FAR struct can_dev_s *dev,
                                          uint16_t id);
static int  ctucanfd_chrdev_send(FAR struct can_dev_s *dev,
                                 struct can_msg_s *msg);
static bool ctucanfd_chrdev_txready(FAR struct can_dev_s *dev);
static bool ctucanfd_chrdev_txempty(FAR struct can_dev_s *dev);
static void ctucanfd_chardev_receive(FAR struct ctucanfd_can_s *priv);
static void ctucanfd_chardev_interrupt(FAR struct ctucanfd_driver_s *priv);
#endif

#ifdef CONFIG_CAN_CTUCANFD_SOCKET
/* SocketCAN methods */

static int  ctucanfd_sock_ifup(FAR struct net_driver_s *dev);
static int  ctucanfd_sock_ifdown(FAR struct net_driver_s *dev);

static void ctucanfd_sock_txavail_work(FAR void *arg);
static int  ctucanfd_sock_txavail(FAR struct net_driver_s *dev);

#  ifdef CONFIG_NETDEV_IOCTL
static int  ctucanfd_sock_ioctl(FAR struct net_driver_s *dev, int cmd,
                                unsigned long arg);
#  endif

static int ctucanfd_sock_txpoll(FAR struct net_driver_s *dev);
static int ctucanfd_sock_send(FAR struct ctucanfd_can_s *priv);
static void ctucanfd_sock_receive(FAR struct ctucanfd_can_s *priv);
static void ctucanfd_sock_interrupt_work(FAR void *arg);
#endif

/* Interrupts */

static int ctucanfd_interrupt(int irq, FAR void *context, FAR void *arg);

/* PCI */

static void ctucanfd_init(FAR struct ctucanfd_driver_s *priv);
static uint8_t ctucanfd_ctucanfd_probe(FAR struct ctucanfd_driver_s *priv);
static int ctucanfd_probe(FAR struct pci_device_s *dev);

/*****************************************************************************
 * Private Data
 *****************************************************************************/

#ifdef CONFIG_CAN_CTUCANFD_CHARDEV
static const struct can_ops_s g_ctucanfd_can_ops =
{
  .co_reset         = ctucanfd_chrdev_reset,
  .co_setup         = ctucanfd_chrdev_setup,
  .co_shutdown      = ctucanfd_chrdev_shutdown,
  .co_rxint         = ctucanfd_chrdev_rxint,
  .co_txint         = ctucanfd_chrdev_txint,
  .co_ioctl         = ctucanfd_chrdev_ioctl,
  .co_remoterequest = ctucanfd_chrdev_remoterequest,
  .co_send          = ctucanfd_chrdev_send,
  .co_txready       = ctucanfd_chrdev_txready,
  .co_txempty       = ctucanfd_chrdev_txempty,
};
#endif

static const struct pci_device_id_s g_ctucanfd_id_table[] =
{
  {
    PCI_DEVICE(0x1760, 0xff00),
    .driver_data = 0
  },
  { }
};

static struct pci_driver_s g_ctucanfd_drv =
{
  .id_table = g_ctucanfd_id_table,
  .probe    = ctucanfd_probe,
};

#ifdef CONFIG_CAN_CTUCANFD_CHARDEV
static uint8_t g_ctucanfd_count = 0;
#endif

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: ctucanfd_getreg
 *****************************************************************************/

static uint32_t ctucanfd_getreg(FAR struct ctucanfd_can_s *priv,
                                unsigned int offset)
{
  uintptr_t addr = priv->base + offset;
  return *((FAR volatile uint32_t *)addr);
}

/*****************************************************************************
 * Name: ctucanfd_putreg
 *****************************************************************************/

static void ctucanfd_putreg(FAR struct ctucanfd_can_s *priv,
                            unsigned int offset,
                            uint32_t value)
{
  uintptr_t addr = priv->base + offset;
  *((FAR volatile uint32_t *)addr) = value;
}

/*****************************************************************************
 * Name: ctucanfd_reset
 *****************************************************************************/

static void ctucanfd_reset(FAR struct ctucanfd_can_s *priv)
{
  /* Soft reset */

  ctucanfd_putreg(priv, CTUCANFD_SET_MODE, CTUCANFD_MODE_RST);
}

/*****************************************************************************
 * Name: ctucanfd_shutdown
 *****************************************************************************/

static void ctucanfd_shutdown(FAR struct ctucanfd_can_s *priv)
{
  ctucanfd_putreg(priv, CTUCANFD_SET_MODE, 0);
}

/*****************************************************************************
 * Name: ctucanfd_setup
 *****************************************************************************/

static void ctucanfd_setup(FAR struct ctucanfd_can_s *priv)
{
  uint32_t regval = 0;
  int      i;

  /* REVISIT: missing bus timings configuration.
   *
   * This driver was verified on QEMU with virtual host CAN network,
   * which doesn't need bus timings.
   * For real hardware, these registers must be properly configured !
   */

  ctucanfd_putreg(priv, CTUCANFD_BTR, 0);
  ctucanfd_putreg(priv, CTUCANFD_BTRFD, 0);

  /* Enable interrupts */

#if defined(CONFIG_CAN_ERRORS) || defined(CONFIG_NET_CAN_ERRORS)
  ctucanfd_putreg(priv, CTUCANFD_INTENSET, CTUCANFD_INT_ERR);
#else
  ctucanfd_putreg(priv, CTUCANFD_INTENCLR, CTUCANFD_INT_ERR);
#endif

  /* Configure TX priority */

  for (i = 0; i < priv->txbufcnt; i++)
    {
      regval |= 1 << (CTUCANFD_TXPRIO_SHIFT * i);
    }

  ctucanfd_putreg(priv, CTUCANFD_TXPRIO, regval);

  /* Set MODE register */

  regval = ctucanfd_getreg(priv, CTUCANFD_SET_MODE);

  /* Enable CTU CAN FD */

  regval |= CTUCANFD_SET_ENA << CTUCANFD_SET_SHFIT;

  /* RX buffer automatic mode */

  regval |= CTUCANFD_MODE_RXBAM;

  /* Write SETTINGS and MODE */

  ctucanfd_putreg(priv, CTUCANFD_SET_MODE, regval);
}

/*****************************************************************************
 * Name: ctucanfd_rxint
 *****************************************************************************/

static void ctucanfd_rxint(FAR struct ctucanfd_can_s *priv, bool enable)
{
  if (enable)
    {
      ctucanfd_putreg(priv, CTUCANFD_INTENSET, CTUCANFD_INT_RBNEI);
    }
  else
    {
      ctucanfd_putreg(priv, CTUCANFD_INTENCLR, CTUCANFD_INT_RBNEI);
    }
}

/*****************************************************************************
 * Name: ctucanfd_txint
 *****************************************************************************/

static void ctucanfd_txint(FAR struct ctucanfd_can_s *priv, bool enable)
{
  if (enable)
    {
      ctucanfd_putreg(priv, CTUCANFD_INTENSET, CTUCANFD_INT_TXI);
    }
  else
    {
      ctucanfd_putreg(priv, CTUCANFD_INTENCLR, CTUCANFD_INT_TXI);
    }
}

/*****************************************************************************
 * Name: ctucanfd_txready
 *****************************************************************************/

static bool ctucanfd_txready(FAR struct ctucanfd_can_s *priv)
{
  uint32_t regval;
  int      i;

  regval = ctucanfd_getreg(priv, CTUCANFD_TXSTAT);

  for (i = 0; i < priv->txbufcnt; i++)
    {
      if (CTUCANFD_TXSTAT_GET(regval, i) == CTUCANFD_TXSTAT_ETY)
        {
          return true;
        }
    }

  return false;
}

#ifdef CONFIG_CAN_CTUCANFD_CHARDEV
/*****************************************************************************
 * Name: ctucanfd_chrdev_reset
 *
 * Description:
 *   Reset the CAN device.  Called early to initialize the hardware. This
 *   function is called, before ctucanfd_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *  None
 *
 *****************************************************************************/

static void ctucanfd_chrdev_reset(FAR struct can_dev_s *dev)
{
  FAR struct ctucanfd_can_s *priv = (FAR struct ctucanfd_can_s *)dev;

  ctucanfd_reset(priv);
}

/*****************************************************************************
 * Name: ctucanfd_chrdev_setup
 *
 * Description:
 *   Configure the CAN. This method is called the first time that the CAN
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching CAN interrupts.
 *   All CAN interrupts are disabled upon return.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 *****************************************************************************/

static int ctucanfd_chrdev_setup(FAR struct can_dev_s *dev)
{
  FAR struct ctucanfd_can_s *priv = (FAR struct ctucanfd_can_s *)dev;

  ctucanfd_setup(priv);

  return OK;
}

/*****************************************************************************
 * Name: ctucanfd_chrdev_shutdown
 *
 * Description:
 *   Disable the CAN.  This method is called when the CAN device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

static void ctucanfd_chrdev_shutdown(FAR struct can_dev_s *dev)
{
  FAR struct ctucanfd_can_s *priv = (FAR struct ctucanfd_can_s *)dev;

  ctucanfd_shutdown(priv);
}

/*****************************************************************************
 * Name: ctucanfd_chrdev_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

static void ctucanfd_chrdev_rxint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct ctucanfd_can_s *priv = (FAR struct ctucanfd_can_s *)dev;

  ctucanfd_rxint(priv, enable);
}

/*****************************************************************************
 * Name: ctucanfd_chrdev_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

static void ctucanfd_chrdev_txint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct ctucanfd_can_s *priv = (FAR struct ctucanfd_can_s *)dev;

  ctucanfd_txint(priv, enable);
}

/*****************************************************************************
 * Name: ctucanfd_chrdev_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 *****************************************************************************/

static int ctucanfd_chrdev_ioctl(FAR struct can_dev_s *dev, int cmd,
                                 unsigned long arg)
{
  return -ENOTTY;
}

/*****************************************************************************
 * Name: ctucanfd_chrdev_remoterequest
 *
 * Description:
 *   Send a remote request
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 *****************************************************************************/

static int ctucanfd_chrdev_remoterequest(FAR struct can_dev_s *dev,
                                         uint16_t id)
{
  return -ENOTSUP;
}

/*****************************************************************************
 * Name: ctucanfd_chrdev_send
 *
 * Description:
 *    Send one can message.
 *
 *    One CAN-message consists of a maximum of 10 bytes.  A message is
 *    composed of at least the first 2 bytes (when there are no data bytes).
 *
 *    Byte 0:      Bits 0-7: Bits 3-10 of the 11-bit CAN identifier
 *    Byte 1:      Bits 5-7: Bits 0-2 of the 11-bit CAN identifier
 *                 Bit 4:    Remote Transmission Request (RTR)
 *                 Bits 0-3: Data Length Code (DLC)
 *    Bytes 2-10: CAN data
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 *****************************************************************************/

static int ctucanfd_chrdev_send(FAR struct can_dev_s *dev,
                                FAR struct can_msg_s *msg)
{
  FAR struct ctucanfd_can_s  *priv = (FAR struct ctucanfd_can_s *)dev;
  union ctucanfd_frame_fmt_u  fmt;
  union ctucanfd_frame_id_u   id;
  uint32_t                    regval;
  uint32_t                    offset = 0;
  int                         txidx;
  int                         i;
  uint8_t                     bytes;

  /* Get TX empty buffer */

  regval = ctucanfd_getreg(priv, CTUCANFD_TXSTAT);

  for (i = 0; i < priv->txbufcnt; i++)
    {
      if (CTUCANFD_TXSTAT_GET(regval, i) == CTUCANFD_TXSTAT_ETY)
        {
          offset = CTUCANFD_TXT1 + CTUCANFD_TXT_SIZE * i;
          txidx = i;
          break;
        }
    }

  if (offset == 0)
    {
      canerr("ERROR: TX buffer not available\n");
      return -EBUSY;
    }

  /* Reset data */

  fmt.u32 = 0;
  id.u32  = 0;

  /* Set up the DLC */

  fmt.s.dlc = msg->cm_hdr.ch_dlc;

  /* Set RTR bit */

  fmt.s.rtr = msg->cm_hdr.ch_rtr;

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      fmt.s.ide   = 1;
      id.s.id_ext = msg->cm_hdr.ch_id;
    }
  else
#endif
    {
      fmt.s.ide = 0;
      id.s.id   = msg->cm_hdr.ch_id;
    }

#ifdef CONFIG_CAN_FD

  /* Set CAN FD specific flags */

  fmt.s.brs     = msg->cm_hdr.ch_brs;
  fmt.s.esi_rsv = msg->cm_hdr.ch_esi;
  fmt.s.fdf = msg->cm_hdr.ch_edl;
#endif

  /* Write frame */

  ctucanfd_putreg(priv, offset + CTUCANFD_TXBUF_FMT, fmt.u32);
  ctucanfd_putreg(priv, offset + CTUCANFD_TXBUF_ID, id.u32);
  ctucanfd_putreg(priv, offset + CTUCANFD_TXBUF_TSL, 0);
  ctucanfd_putreg(priv, offset + CTUCANFD_TXBUF_TSU, 0);

  bytes = can_dlc2bytes(msg->cm_hdr.ch_dlc);
  for (i = 0; i < bytes; i++)
    {
      ctucanfd_putreg(priv, offset + CTUCANFD_TXBUF_DATA + i,
                      msg->cm_data[i]);
    }

  /* Set TX ready */

  regval = CTUCANFD_TXCMD_TXCR + (1 << (CTUCANFD_TXCMD_TXB_SHIFT + txidx));
  ctucanfd_putreg(priv, CTUCANFD_TXINFOCMD, regval);

  return OK;
}

/*****************************************************************************
 * Name: ctucanfd_chrdev_txready
 *
 * Description:
 *   Return true if the CAN hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if the CAN hardware is ready to accept another TX message.
 *
 *****************************************************************************/

static bool ctucanfd_chrdev_txready(FAR struct can_dev_s *dev)
{
  FAR struct ctucanfd_can_s *priv = (FAR struct ctucanfd_can_s *)dev;

  return ctucanfd_txready(priv);
}

/*****************************************************************************
 * Name: ctucanfd_chrdev_txempty
 *
 * Description:
 *   Return true if all message have been sent.  If for example, the CAN
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if there are no pending TX transfers in the CAN hardware.
 *
 *****************************************************************************/

static bool ctucanfd_chrdev_txempty(FAR struct can_dev_s *dev)
{
  FAR struct ctucanfd_can_s *priv   = (FAR struct ctucanfd_can_s *)dev;
  uint32_t                   regval = 0;
  bool                       ret    = true;
  int                        i;

  regval = ctucanfd_getreg(priv, CTUCANFD_TXSTAT);

  for (i = 0; i < priv->txbufcnt; i++)
    {
      if (CTUCANFD_TXSTAT_GET(regval, i) != CTUCANFD_TXSTAT_ETY)
        {
          ret = false;
          break;
        }
    }

  return ret;
}

/*****************************************************************************
 * Name: ctucanfd_chardev_receive
 *
 * Description:
 *   Receive CAN frame
 *
 *****************************************************************************/

static void ctucanfd_chardev_receive(FAR struct ctucanfd_can_s *priv)
{
  FAR struct can_dev_s    *dev    = (FAR struct can_dev_s *)priv;
  FAR uint32_t            *ptr    = NULL;
  struct ctucanfd_frame_s  frame;
  struct can_hdr_s         hdr;
  int                      i      = 0;
  int                      ret    = 0;
  uint16_t                 frc    = 0;
  uint32_t                 regval = 0;

  /* Get frame count */

  regval = ctucanfd_getreg(priv, CTUCANFD_RXSETSTAT);
  frc = (regval & CTUCANFD_RXSTAT_RXFRC_MASK) >> CTUCANFD_RXSTAT_RXFRC_SHIFT;

  /* Read frames */

  while (frc-- > 0)
    {
      ptr = (FAR uint32_t *)&frame;

      for (i = 0; i < sizeof(struct ctucanfd_frame_s) / 4; i++)
        {
          /* RX buffer in automatic mode */

          ptr[i] = ctucanfd_getreg(priv, CTUCANFD_RXDATA);
        }

      /* Get the DLC */

      hdr.ch_dlc = frame.fmt.dlc;

      /* Get RTR bit */

      hdr.ch_rtr = frame.fmt.rtr;

#ifdef CONFIG_CAN_EXTID
      /* Get the CAN identifier. */

      hdr.ch_extid = frame.fmt.ide;

      if (hdr.ch_extid)
        {
          hdr.ch_id = frame.id.id_ext;
        }
      else
        {
          hdr.ch_id = frame.id.id;
        }
#else
      if (frame.fmt.ide)
        {
          canerr("ERROR: Received message with extended"
                 " identifier.  Dropped\n");

          continue;
        }

      hdr.ch_id = frame.id.id;
#endif

      /* Clear the error indication and unused bits */

#ifdef CONFIG_CAN_ERRORS
      hdr.ch_error = 0;
#endif
      hdr.ch_tcf   = 0;

#ifdef CONFIG_CAN_FD
      hdr.ch_esi = frame.fmt.esi_rsv;
      hdr.ch_edl = frame.fmt.fdf;
      hdr.ch_brs = frame.fmt.brs;
#else
      if (frame.fmt.fdf)
        {
          /* Drop any FD CAN messages if not supported */

          canerr("ERROR: Received CAN FD message.  Dropped\n");

          return;
        }
#endif

      /* Provide the data to the upper half driver */

      ret = can_receive(dev, &hdr, (FAR uint8_t *)&frame.data);
      if (ret < 0)
        {
          canerr("ERROR: can_receive failed %d\n", ret);
        }
    }
}

#ifdef CONFIG_CAN_ERRORS
/*****************************************************************************
 * Name: ctucanfd_chardev_error
 *****************************************************************************/

static void ctucanfd_chardev_error(FAR struct ctucanfd_can_s *priv,
                                   uint32_t stat)
{
  struct can_hdr_s hdr;
  uint16_t         errbits = 0;
  uint8_t          data[CAN_ERROR_DLC];
  int              ret;

  memset(data, 0, sizeof(data));

  /* Data overrun interrupt */

  if (stat & CTUCANFD_INT_DOI)
    {
      data[1] |= CAN_ERROR1_RXOVERFLOW;
      errbits |= CAN_ERROR_CONTROLLER;

      /* Clear data overrun */

      ctucanfd_putreg(priv, CTUCANFD_CMD,
                      CTUCANFD_CMD_RRB | CTUCANFD_CMD_CDO);
    }

  /* Arbitration lost interrupt */

  if (stat & CTUCANFD_INT_ALI)
    {
      errbits |= CAN_ERROR_LOSTARB;
      data[0]  = CAN_ERROR0_UNSPEC;
    }

  /* BUS error interrupt */

  if (stat & CTUCANFD_INT_BEI)
    {
      errbits |= CAN_ERROR_BUSERROR;
    }

  /* Report a CAN error */

  if (errbits != 0)
    {
      canerr("ERROR: errbits = %08" PRIx16 "\n", errbits);

      /* Format the CAN header for the error report. */

      hdr.ch_id    = errbits;
      hdr.ch_dlc   = CAN_ERROR_DLC;
      hdr.ch_rtr   = 0;
      hdr.ch_error = 1;
#ifdef CONFIG_CAN_EXTID
      hdr.ch_extid = 0;
#endif
      hdr.ch_tcf   = 0;

      /* And provide the error report to the upper half logic */

      ret = can_receive(&priv->dev, &hdr, data);
      if (ret < 0)
        {
          canerr("ERROR: can_receive failed: %d\n", ret);
        }
    }
}
#endif

/*****************************************************************************
 * Name: ctucanfd_chardev_interrupt
 *****************************************************************************/

static void ctucanfd_chardev_interrupt(FAR struct ctucanfd_driver_s *priv)
{
  uint32_t stat   = 0;
  uint32_t regval = 0;
  int      i      = 0;
  int      txidx  = 0;

  for (i = 0; i < priv->count; i++)
    {
      stat = ctucanfd_getreg(&priv->devs[i], CTUCANFD_INTSTAT);
      if (stat == 0)
        {
          continue;
        }

      /* Frame received */

      if (stat & CTUCANFD_INT_RXI)
        {
          ctucanfd_chardev_receive(&priv->devs[i]);
        }

      /* Frame transmitted */

      if (stat & CTUCANFD_INT_TXI)
        {
          regval = ctucanfd_getreg(&priv->devs[i], CTUCANFD_TXSTAT);

          for (txidx = 0; txidx < priv->devs[i].txbufcnt; txidx++)
            {
              if (CTUCANFD_TXSTAT_GET(regval, txidx) ==
                  CTUCANFD_TXSTAT_TOK)
                {
                  can_txdone(&priv->devs[i].dev);

                  /* Mark buffer as empty */

                  regval = (CTUCANFD_TXCMD_TXCE +
                            (1 << (CTUCANFD_TXCMD_TXB_SHIFT + txidx)));
                  ctucanfd_putreg(&priv->devs[i], CTUCANFD_TXINFOCMD, regval);
                }
            }
        }

#ifdef CONFIG_CAN_ERRORS
      /* Handle errors */

      if (stat & CTUCANFD_INT_ERR)
        {
          ctucanfd_chardev_error(&priv->devs[i], stat);
        }
#endif

      /* Clear interrupts */

      ctucanfd_putreg(&priv->devs[i], CTUCANFD_INTSTAT, stat);

      /* Re-enable RX interrupts */

      ctucanfd_rxint(&priv->devs[i], true);
    }
}
#endif  /* CONFIG_CAN_CTUCANFD_CHARDEV */

#ifdef CONFIG_CAN_CTUCANFD_SOCKET
/*****************************************************************************
 * Name: ctucanfd_sock_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

static int ctucanfd_sock_ifup(FAR struct net_driver_s *dev)
{
  FAR struct ctucanfd_can_s *priv =
    (FAR struct ctucanfd_can_s *)dev->d_private;

  priv->dev.d_buf = (FAR uint8_t *)priv->tx_pool;

  ctucanfd_reset(priv);
  ctucanfd_setup(priv);

  ctucanfd_txint(priv, true);
  ctucanfd_rxint(priv, true);

  return OK;
}

/*****************************************************************************
 * Name: ctucanfd_sock_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

static int ctucanfd_sock_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct ctucanfd_can_s *priv =
    (FAR struct ctucanfd_can_s *)dev->d_private;

  ctucanfd_txint(priv, false);
  ctucanfd_rxint(priv, false);

  /* Shutdown */

  ctucanfd_shutdown(priv);

  return OK;
}

/*****************************************************************************
 * Name: ctucanfd_sock_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 *****************************************************************************/

static void ctucanfd_sock_txavail_work(FAR void *arg)
{
  FAR struct ctucanfd_can_s *priv = (FAR struct ctucanfd_can_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (IFF_IS_UP(priv->dev.d_flags))
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      if (ctucanfd_txready(priv))
        {
          /* No, there is space for another transfer.  Poll the network for
           * new XMIT data.
           */

          devif_poll(&priv->dev, ctucanfd_sock_txpoll);
        }
    }

  net_unlock();
}

/*****************************************************************************
 * Name: ctucanfd_sock_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 *****************************************************************************/

static int ctucanfd_sock_txavail(FAR struct net_driver_s *dev)
{
  FAR struct ctucanfd_can_s *priv =
    (FAR struct ctucanfd_can_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      ctucanfd_sock_txavail_work(priv);
    }

  return OK;
}

#  ifdef CONFIG_NETDEV_IOCTL
/*****************************************************************************
 * Name: ctucanfd_sock_ioctl
 *
 * Description:
 *   PHY ioctl command handler
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   cmd  - ioctl command
 *   arg  - Argument accompanying the command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *
 *****************************************************************************/

static int  ctucanfd_sock_ioctl(FAR struct net_driver_s *dev, int cmd,
                                unsigned long arg)
{
  return -ENOTTY;
}

#  endif  /* CONFIG_NETDEV_IOCTL */

/*****************************************************************************
 * Name: ctucanfd_sock_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 *****************************************************************************/

static int ctucanfd_sock_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct ctucanfd_can_s *priv =
    (FAR struct ctucanfd_can_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Send the packet */

      ctucanfd_sock_send(priv);

      /* Check if there is room in the device to hold another packet. If
       * not, return a non-zero value to terminate the poll.
       */

      if (!ctucanfd_txready(priv))
        {
          return -EBUSY;
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/*****************************************************************************
 * Name: ctucanfd_sock_send
 *****************************************************************************/

static int ctucanfd_sock_send(FAR struct ctucanfd_can_s *priv)
{
  union ctucanfd_frame_fmt_u fmt;
  union ctucanfd_frame_id_u  id;
  uint32_t                   regval;
  uint32_t                   offset;
  int                        txidx;
  int                        i;

  /* Get TX empty buffer */

  regval = ctucanfd_getreg(priv, CTUCANFD_TXSTAT);

  offset = 0;
  for (i = 0; i < priv->txbufcnt; i++)
    {
      if (CTUCANFD_TXSTAT_GET(regval, i) == CTUCANFD_TXSTAT_ETY)
        {
          offset = CTUCANFD_TXT1 + CTUCANFD_TXT_SIZE * i;
          txidx = i;
          break;
        }
    }

  if (offset == 0)
    {
      canerr("ERROR: TX buffer not available\n");
      return -EBUSY;
    }

  /* Reset data */

  fmt.u32 = 0;
  id.u32  = 0;

  /* CAN 2.0 or CAN FD */

  if (priv->dev.d_len == sizeof(struct can_frame))
    {
      FAR struct can_frame *frame = frame =
        (FAR struct can_frame *)priv->dev.d_buf;

      /* Set up the DLC */

      fmt.s.dlc = frame->can_dlc;

      /* Set RTR bit */

      fmt.s.rtr = (frame->can_id & CAN_RTR_FLAG);

#ifdef CONFIG_NET_CAN_EXTID
      if (frame->can_id & CAN_EFF_FLAG)
        {
          fmt.s.ide   = 1;
          id.s.id_ext = (frame->can_id & 0x1ffffff);
        }
      else
#endif
        {
          fmt.s.ide = 0;
          id.s.id   = (frame->can_id & 0x07ff);
        }

      /* Write frame */

      ctucanfd_putreg(priv, offset + CTUCANFD_TXBUF_FMT, fmt.u32);
      ctucanfd_putreg(priv, offset + CTUCANFD_TXBUF_ID, id.u32);
      ctucanfd_putreg(priv, offset + CTUCANFD_TXBUF_TSL, 0);
      ctucanfd_putreg(priv, offset + CTUCANFD_TXBUF_TSU, 0);

      for (i = 0; i < frame->can_dlc; i++)
        {
          ctucanfd_putreg(priv, offset + CTUCANFD_TXBUF_DATA + i,
                          frame->data[i]);
        }
    }
#ifdef CONFIG_NET_CAN_CANFD
  else /* CAN FD frame */
    {
      FAR struct canfd_frame *frame =
        (FAR struct canfd_frame *)priv->dev.d_buf;

      /* CAN FD frame */

      fmt.s.fdf = 1;

      /* Set up the DLC */

      fmt.s.dlc = can_bytes2dlc(frame->len);

      /* Set flags */

      fmt.s.rtr     = (frame->can_id & CAN_RTR_FLAG);
      fmt.s.brs     = (frame->flags & CANFD_BRS);
      fmt.s.esi_rsv = (frame->flags & CANFD_ESI);

#ifdef CONFIG_NET_CAN_EXTID
      if (frame->can_id & CAN_EFF_FLAG)
        {
          fmt.s.ide   = 1;
          id.s.id_ext = (frame->can_id & 0x1ffffff);
        }
      else
#endif
        {
          fmt.s.ide = 0;
          id.s.id   = (frame->can_id & 0x07ff);
        }

      /* Write frame */

      ctucanfd_putreg(priv, offset + CTUCANFD_TXBUF_FMT, fmt.u32);
      ctucanfd_putreg(priv, offset + CTUCANFD_TXBUF_ID, id.u32);
      ctucanfd_putreg(priv, offset + CTUCANFD_TXBUF_TSL, 0);
      ctucanfd_putreg(priv, offset + CTUCANFD_TXBUF_TSU, 0);

      for (i = 0; i < frame->len; i++)
        {
          ctucanfd_putreg(priv, offset + CTUCANFD_TXBUF_DATA + i,
                          frame->data[i]);
        }
    }
#endif

  /* Set TX ready */

  regval = CTUCANFD_TXCMD_TXCR + (1 << (CTUCANFD_TXCMD_TXB_SHIFT + txidx));
  ctucanfd_putreg(priv, CTUCANFD_TXINFOCMD, regval);

  return OK;
}

/*****************************************************************************
 * Name: ctucanfd_sock_receive
 *****************************************************************************/

static void ctucanfd_sock_receive(FAR struct ctucanfd_can_s *priv)
{
  struct ctucanfd_frame_s  rxframe;
  FAR uint32_t            *ptr    = NULL;
  int                      i      = 0;
  uint16_t                 frc    = 0;
  uint32_t                 regval = 0;
  uint8_t                  bytes;

  /* Get frame count */

  regval = ctucanfd_getreg(priv, CTUCANFD_RXSETSTAT);
  frc = (regval & CTUCANFD_RXSTAT_RXFRC_MASK) >> CTUCANFD_RXSTAT_RXFRC_SHIFT;

  /* Read frames */

  while (frc-- > 0)
    {
      ptr = (FAR uint32_t *)&rxframe;

      for (i = 0; i < sizeof(struct ctucanfd_frame_s) / 4; i++)
        {
          /* RX buffer in automatic mode */

          ptr[i] = ctucanfd_getreg(priv, CTUCANFD_RXDATA);
        }

      /* CAN 2.0 or CAN FD */

#ifdef CONFIG_NET_CAN_CANFD
      if (rxframe.fmt.fdf)
        {
          struct canfd_frame *frame = (struct canfd_frame *)priv->rx_pool;

          /* Get the DLC */

          frame->len = can_dlc2bytes(rxframe.fmt.dlc);

#ifdef CONFIG_NET_CAN_EXTID
          /* Get the CAN identifier. */

          if (rxframe.fmt.ide)
            {
              frame->can_id = rxframe.id.id_ext;
              frame->can_id |= CAN_EFF_FLAG;
            }
          else
            {
              frame->can_id = rxframe.id.id;
            }
#else
          if (rxframe.fmt.ide)
            {
              canerr("ERROR: Received message with extended"
                     " identifier.  Dropped\n");
              continue;
            }

          frame->can_id = rxframe.id.id;
#endif

          /* Extract the RTR bit */

          if (rxframe.fmt.rtr)
            {
              frame->can_id |= CAN_RTR_FLAG;
            }

          /* Get CANFD flags */

          frame->flags = 0;

          if (rxframe.fmt.esi_rsv)
            {
              frame->flags |= CANFD_ESI;
            }

          if (rxframe.fmt.brs)
            {
              frame->flags |= CANFD_BRS;
            }

          /* Get data */

          bytes = can_dlc2bytes(rxframe.fmt.dlc);
          for (i = 0; i < bytes; i++)
            {
              frame->data[i] = rxframe.data[i];
            }

          /* Copy the buffer pointer to priv->dev..  Set amount of data
           * in priv->dev.d_len
           */

          priv->dev.d_len = sizeof(struct canfd_frame);
          priv->dev.d_buf = (FAR uint8_t *)frame;

          /* Send to socket interface */

          NETDEV_RXPACKETS(&priv->dev);

          can_input(&priv->dev);

          /* Point the packet buffer back to the next Tx buffer that will be
           * used during the next write.  If the write queue is full, then
           * this will point at an active buffer, which must not be written
           * to.  This is OK because devif_poll won't be called unless the
           * queue is not full.
           */

          priv->dev.d_buf = (FAR uint8_t *)priv->tx_pool;
        }
      else
#endif
        {
          FAR struct can_frame *frame  =
            (FAR struct can_frame *)priv->rx_pool;

          /* Get the DLC */

          frame->can_dlc = rxframe.fmt.dlc;

#ifdef CONFIG_NET_CAN_EXTID
          /* Get the CAN identifier. */

          if (rxframe.fmt.ide)
            {
              frame->can_id = rxframe.id.id_ext;
              frame->can_id |= CAN_EFF_FLAG;
            }
          else
            {
              frame->can_id = rxframe.id.id;
            }
#else
          if (rxframe.fmt.ide)
            {
              canerr("ERROR: Received message with extended"
                     " identifier.  Dropped\n");
              continue;
            }

          frame->can_id = rxframe.id.id;
#endif

          /* Extract the RTR bit */

          if (rxframe.fmt.rtr)
            {
              frame->can_id |= CAN_RTR_FLAG;
            }

          /* Get data */

          for (i = 0; i < rxframe.fmt.dlc; i++)
            {
              frame->data[i] = rxframe.data[i];
            }

          /* Copy the buffer pointer to priv->dev..  Set amount of data
           * in priv->dev.d_len
           */

          priv->dev.d_len = sizeof(struct can_frame);
          priv->dev.d_buf = (FAR uint8_t *)frame;

          /* Send to socket interface */

          NETDEV_RXPACKETS(&priv->dev);

          can_input(&priv->dev);

          /* Point the packet buffer back to the next Tx buffer that will be
           * used during the next write.  If the write queue is full, then
           * this will point at an active buffer, which must not be written
           * to.  This is OK because devif_poll won't be called unless the
           * queue is not full.
           */

          priv->dev.d_buf = (FAR uint8_t *)priv->tx_pool;
        }
    }
}

#ifdef CONFIG_NET_CAN_ERRORS
/*****************************************************************************
 * Name: ctucanfd_sock_error
 *****************************************************************************/

static void ctucanfd_sock_error(FAR struct ctucanfd_can_s *priv,
                                uint32_t stat)
{
  FAR struct can_frame *frame   = NULL;
  uint16_t              errbits = 0;

  /* Data overrun interrupt */

  if (stat & CTUCANFD_INT_DOI)
    {
      frame->data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
      errbits        |= CAN_ERR_CRTL;

      /* Clear data overrun */

      ctucanfd_putreg(priv, CTUCANFD_CMD,
                      (CTUCANFD_CMD_RRB | CTUCANFD_CMD_CDO));
    }

  /* Arbitration lost interrupt */

  if (stat & CTUCANFD_INT_ALI)
    {
      errbits        |= CAN_ERR_LOSTARB;
      frame->data[0]  = CAN_ERR_LOSTARB_UNSPEC;
    }

  /* BUS error interrupt */

  if (stat & CTUCANFD_INT_BEI)
    {
      errbits |= CAN_ERR_BUSERROR;
    }

  if (errbits != 0)
    {
      frame = (FAR struct can_frame *)priv->rx_pool;

      canerr("ERROR: errbits = %08" PRIx16 "\n", errbits);

      /* Copy frame */

      frame->can_id  = errbits;
      frame->can_dlc = CAN_ERR_DLC;

      net_lock();

      /* Copy the buffer pointer to priv->dev..  Set amount of data
       * in priv->dev.d_len
       */

      priv->dev.d_len = sizeof(struct can_frame);
      priv->dev.d_buf = (FAR uint8_t *)frame;

      /* Send to socket interface */

      NETDEV_ERRORS(&priv->dev);

      can_input(&priv->dev);

      /* Point the packet buffer back to the next Tx buffer that will be
       * used during the next write.  If the write queue is full, then
       * this will point at an active buffer, which must not be written
       * to.  This is OK because devif_poll won't be called unless the
       * queue is not full.
       */

      priv->dev.d_buf = (FAR uint8_t *)priv->tx_pool;
      net_unlock();
    }
}
#endif

/*****************************************************************************
 * Name: ctucanfd_sock_interrupt_work
 *****************************************************************************/

static void ctucanfd_sock_interrupt_work(FAR void *arg)
{
  FAR struct ctucanfd_driver_s *priv   = arg;
  uint32_t                      stat   = 0;
  uint32_t                      regval = 0;
  int                           i      = 0;
  int                           txidx  = 0;

  for (i = 0; i < priv->count; i++)
    {
      stat = ctucanfd_getreg(&priv->devs[i], CTUCANFD_INTSTAT);
      if (stat == 0)
        {
          continue;
        }

      /* RX buffer not empty interrupt */

      if (stat & CTUCANFD_INT_RBNEI)
        {
          ctucanfd_sock_receive(&priv->devs[i]);
        }

      /* Transmit interrupt */

      if (stat & CTUCANFD_INT_TXI)
        {
          regval = ctucanfd_getreg(&priv->devs[i], CTUCANFD_TXSTAT);

          for (txidx = 0; txidx < priv->devs[i].txbufcnt; txidx++)
            {
              if (CTUCANFD_TXSTAT_GET(regval, txidx) ==
                  CTUCANFD_TXSTAT_TOK)
                {
                  NETDEV_TXDONE(&priv->devs[i].dev);

                  /* There should be space for a new TX in any event.
                   * Poll the network for new XMIT data.
                   */

                  net_lock();
                  devif_poll(&priv->devs[i].dev, ctucanfd_sock_txpoll);
                  net_unlock();

                  /* Mark buffer as empty */

                  regval = (CTUCANFD_TXCMD_TXCE +
                            (1 << (CTUCANFD_TXCMD_TXB_SHIFT + txidx)));
                  ctucanfd_putreg(&priv->devs[i], CTUCANFD_TXINFOCMD, regval);
                }
            }
        }

#ifdef CONFIG_NET_CAN_ERRORS
      /* Handle errors */

      if (stat & CTUCANFD_INT_ERR)
        {
          ctucanfd_sock_error(&priv->devs[i], stat);
        }
#endif

      /* Clear interrupts */

      ctucanfd_putreg(&priv->devs[i], CTUCANFD_INTSTAT, stat);

      /* Re-enable RX interrupts */

      ctucanfd_rxint(&priv->devs[i], true);
    }
}
#endif    /* CONFIG_CAN_CTUCANFD_SOCKET */

/*****************************************************************************
 * Name: ctucanfd_interrupt
 *
 * Description:
 *   Initialize CTUCANFD device
 *
 *****************************************************************************/

static int ctucanfd_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct ctucanfd_driver_s *priv   = (FAR struct ctucanfd_driver_s *)arg;
  uint32_t                      regval = 0;
  int                           i      = 0;

  DEBUGASSERT(priv != NULL);

  for (i = 0; i < priv->count; i++)
    {
      regval |= ctucanfd_getreg(&priv->devs[i], CTUCANFD_INTSTAT);

      /* Break now if interrupt is pending */

      if (regval != 0)
        {
          /* Disable RX interrupts until we handle all interrups */

          ctucanfd_rxint(&priv->devs[i], false);

          break;
        }
    }

  /* Check for pending interrupts for this card */

  if (regval == 0)
    {
      return OK;
    }

#ifdef CONFIG_CAN_CTUCANFD_CHARDEV
  /* Handle character device interrupt */

  ctucanfd_chardev_interrupt(priv);
#endif

#ifdef CONFIG_CAN_CTUCANFD_SOCKET
  /* Derefer SocketCAN interrupt to work queue */

  work_queue(CTUCANFD_CANWORK, &priv->irqwork,
             ctucanfd_sock_interrupt_work, priv, 0);
#endif

  return OK;
}

/*****************************************************************************
 * Name: ctucanfd_init
 *
 * Description:
 *   Initialize CTUCANFD device
 *
 *****************************************************************************/

static void ctucanfd_init(FAR struct ctucanfd_driver_s *priv)
{
  /* REVISIT: Only legacy IRQ supported in QEMU driver */

  priv->irq = pci_get_irq(priv->pcidev);
  irq_attach(priv->irq, ctucanfd_interrupt, priv);

  /* REVISIT: Enable card interrupts (not implemented in QEMU) */

  /* Enable interrupts */

  up_enable_irq(priv->irq);
}

/*****************************************************************************
 * Name: ctucanfd_ctucanfd_proble
 *
 * Description:
 *   Proble CTUCANFD devices on board and return the number of vailalbe chips.
 *
 *****************************************************************************/

static uint8_t ctucanfd_ctucanfd_probe(FAR struct ctucanfd_driver_s *priv)
{
  uint32_t offset;
  uint8_t  regval;

  offset = priv->bar0_base + CTUCANFD_BAR0_ID;
  pci_read_io_byte(priv->pcidev, offset, &regval);

  /* REVISIT: what is the maximum number of channels ? */

  return regval;
}

/*****************************************************************************
 * Name: ctucanfd_probe
 *
 * Description:
 *   Probe device
 *
 *****************************************************************************/

static int ctucanfd_probe(FAR struct pci_device_s *dev)
{
  FAR struct ctucanfd_driver_s *priv = NULL;
  uint8_t                       i    = 0;
  int                           ret;
#ifdef CONFIG_CAN_CTUCANFD_CHARDEV
  uint8_t                       count;
  char                          devpath[PATH_MAX];
#endif

  /* Allocate the interface structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      ret = -ENOMEM;
      return ret;
    }

  /* Initialzie PCI dev */

  priv->pcidev = dev;

  pci_set_master(dev);
  pciinfo("Enabled bus mastering\n");
  pci_enable_device(dev);
  pciinfo("Enabled memory resources\n");

  /* Control register BAR */

  priv->bar0_base = (uintptr_t)pci_map_bar(dev, CTUCANFD_BAR0);
  if (!priv->bar0_base)
    {
      pcierr("Not found BAR0\n");
      ret = -ENODEV;
      goto errout;
    }

  /* MEM access only supported */

  if (pci_resource_flags(dev, CTUCANFD_BAR0) != PCI_RESOURCE_MEM)
    {
      ret = -ENOTSUP;
      goto errout;
    }

  priv->canfd_base = (uintptr_t)pci_map_bar(dev, CTUCANFD_CTUCAN_BAR1);
  if (!priv->canfd_base)
    {
      pcierr("Not found CANFD bar\n");
      ret = -ENODEV;
      goto errout;
    }

  /* MEM access only supported */

  if (pci_resource_flags(dev, CTUCANFD_CTUCAN_BAR1) != PCI_RESOURCE_MEM)
    {
      ret = -ENOTSUP;
      goto errout;
    }

  /* Get number of CTUCANFD chips */

  priv->count = ctucanfd_ctucanfd_probe(priv);

  pciinfo("detected %d CTUCANFD channels\n", priv->count);

  /* Allocate CTUCANFD devices */

  priv->devs = kmm_zalloc(sizeof(struct ctucanfd_can_s) * priv->count);
  if (priv->devs == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* Common initialziation for all channels */

  ctucanfd_init(priv);

  /* Handle all CTUCANFD devices */

  for (i = 0; i < priv->count; i++)
    {
      /* Common initialization */

      priv->devs[i].base     = priv->canfd_base + (CTUCANFD_CTUCAN_REGS * i);
      priv->devs[i].pcidev   = dev;
      priv->devs[i].txbufcnt = CTUCANFD_TXBUF_CNT;

#ifdef CONFIG_CAN_CTUCANFD_CHARDEV
      count = g_ctucanfd_count++;

      /* Get devpath for this CTUCANFD device */

      snprintf(devpath, PATH_MAX, "/dev/can%d", count);

      /* Initialize CTUCANFD channel */

      priv->devs[i].dev.cd_ops  = &g_ctucanfd_can_ops;
      priv->devs[i].dev.cd_priv = &priv->devs[i];

      /* Register CAN device */

      ret = can_register(devpath, &priv->devs[i].dev);
      if (ret < 0)
        {
          pcierr("ERROR: failed to register count=%d, %d\n", i, ret);
          goto errout;
        }
#endif

#ifdef CONFIG_CAN_CTUCANFD_SOCKET
      /* Initialize the driver structure */

      priv->devs[i].dev.d_ifup    = ctucanfd_sock_ifup;
      priv->devs[i].dev.d_ifdown  = ctucanfd_sock_ifdown;
      priv->devs[i].dev.d_txavail = ctucanfd_sock_txavail;
#  ifdef CONFIG_NETDEV_IOCTL
      priv->devs[i].dev.d_ioctl   = ctucanfd_sock_ioctl;
#  endif
      priv->devs[i].dev.d_private = &priv->devs[i];

      /* Put the interface in the down state.  This usually amounts to
       * resetting the device and/or calling ctucanfd_sock_ifdown().
       */

      ctucanfd_sock_ifdown(&priv->devs[i].dev);

      /* Register the device with the OS so that socket IOCTLs can be
       * performed
       */

      ret = netdev_register(&priv->devs[i].dev, NET_LL_CAN);
      if (ret < 0)
        {
          pcierr("ERROR: failed to register count=%d, %d\n", i, ret);
          goto errout;
        }
#endif
    }

  return OK;

errout:
  for (i = 0; i < priv->count; i++)
    {
      if (priv->devs[i].pcidev)
        {
#ifdef CONFIG_CAN_CTUCANFD_SOCKET
          netdev_unregister(&priv->devs[i].dev);
#endif

#ifdef CONFIG_CAN_CTUCANFD_CHARDEV
          unregister_driver(devpath);
#endif
        }
    }

  kmm_free(priv->devs);
  kmm_free(priv);

  return ret;
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: pci_ctucanfd_init
 *
 * Description:
 *   Register a pci driver
 *
 *****************************************************************************/

int pci_ctucanfd_init(void)
{
  return pci_register_driver(&g_ctucanfd_drv);
}

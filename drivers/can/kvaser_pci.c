/*****************************************************************************
 * drivers/can/kvaser_pci.c
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

#ifdef CONFIG_CAN_KVASER_CHARDEV
#  include <nuttx/can/can.h>
#endif

#ifdef CONFIG_CAN_KVASER_SOCKET
#  include <nuttx/net/netdev_lowerhalf.h>
#  include <nuttx/net/can.h>
#endif

#include "sja1000.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* Kvaser card constants */

#define KVASER_SJA_MAX                 4
#define KVASER_XTAL_FREQ               16000000

/* PCI BARs */

#define KVASER_S5920_BAR               0
#define KVASER_SJA_BAR                 1
#define KVASER_XILINX_BAR              2

/* Kvaser registers */

#define KVASER_SJA_REGS                0x20

#define KVASER_S5920_INTCSR            0x38
#define KVASER_S5920_INTCSR_INT_ASSERT (1 << 23) /* Interrupt asserted bit */
#define KVASER_S5920_INTCSR_INT_ADDON  (1 << 13) /* Add-on interrupt pin enable */

/* Interrupts */

#define KVASER_INT_ERR                 (SJA1000_OVERRUN_INT_ENA |      \
                                        SJA1000_ERR_PASSIVE_INT_ENA |  \
                                        SJA1000_ARB_LOST_INT_ENA |     \
                                        SJA1000_BUS_ERR_INT_ENA)

#define KVASER_INT_ERR_ST              (SJA1000_OVERRUN_INT_ST |      \
                                        SJA1000_ERR_PASSIVE_INT_ST |  \
                                        SJA1000_ARB_LOST_INT_ST |     \
                                        SJA1000_BUS_ERR_INT_ST)

/* SocketCAN specific */

#ifdef CONFIG_CAN_KVASER_SOCKET
#  define KVASER_TX_QUOTA              1
#  define KVASER_RX_QUOTA              1

#  if CONFIG_IOB_NBUFFERS < (KVASER_RX_QUOTA + KVASER_TX_QUOTA)
#    error CONFIG_IOB_NBUFFERS must be > (KVASER_RX_QUOTA + KVASER_TX_QUOTA)
#  endif

#endif /* CONFIG_CAN_KVASER_SOCKET */

#ifndef CONFIG_NET_CAN_ERRORS
#  define KVASER_SOCK_RXINT            (SJA1000_RX_INT_ST)
#else
#  define KVASER_SOCK_RXINT            (SJA1000_RX_INT_ST | KVASER_INT_ERR_ST)
#endif

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/* SJA1000 channel private data */

struct kvaser_sja_s
{
#ifdef CONFIG_CAN_KVASER_CHARDEV
  struct can_dev_s    dev;
#endif

#ifdef CONFIG_CAN_KVASER_SOCKET
  /* This holds the information visible to the NuttX network */

  struct netdev_lowerhalf_s dev;
#endif

  /* PCI data */

  FAR struct pci_device_s *pcidev;
  uint64_t                 base;
};

/* KVASER private data */

struct kvaser_driver_s
{
  FAR struct kvaser_sja_s *sja;
  uint8_t                  count;

  /* PCI data */

  FAR struct pci_device_s *pcidev;
  int                      irq;
  uintptr_t                sja_base;
  uintptr_t                s5920_base;
  uintptr_t                xilinx_base;

#ifdef CONFIG_CAN_KVASER_SOCKET
  /* For deferring interrupt work to the wq */

  struct work_s            irqwork;
#endif
};

/*****************************************************************************
 * Private Functions Definitions
 *****************************************************************************/

/* Helpers */

static uint8_t kvaser_getreg_sja(FAR struct kvaser_sja_s *priv,
                                 unsigned int offset);
static void kvaser_putreg_sja(FAR struct kvaser_sja_s *priv,
                              unsigned int offset,
                              uint8_t value);

static uint32_t kvaser_getreg_s5920(FAR struct kvaser_driver_s *priv,
                                   unsigned int offset);
static void kvaser_putreg_s5920(FAR struct kvaser_driver_s *priv,
                                unsigned int offset,
                                uint32_t value);

/* Common methods */

static void kvaser_reset(FAR struct kvaser_sja_s *priv);
static void kvaser_sleep(FAR struct kvaser_sja_s *priv);
static void kvaser_setup(FAR struct kvaser_sja_s *priv);
static void kvaser_rxint(FAR struct kvaser_sja_s *priv, bool enable);
static void kvaser_txint(FAR struct kvaser_sja_s *priv, bool enable);
static bool kvaser_txready(FAR struct kvaser_sja_s *priv);

#ifdef CONFIG_CAN_KVASER_CHARDEV
/* CAN character device methods */

static void kvaser_chrdev_reset(FAR struct can_dev_s *dev);
static int  kvaser_chrdev_setup(FAR struct can_dev_s *dev);
static void kvaser_chrdev_shutdown(FAR struct can_dev_s *dev);
static void kvaser_chrdev_rxint(FAR struct can_dev_s *dev, bool enable);
static void kvaser_chrdev_txint(FAR struct can_dev_s *dev, bool enable);
static int  kvaser_chrdev_ioctl(FAR struct can_dev_s *dev, int cmd,
                                unsigned long arg);
static int  kvaser_chrdev_remoterequest(FAR struct can_dev_s *dev,
                                        uint16_t id);
static int  kvaser_chrdev_send(FAR struct can_dev_s *dev,
                               FAR struct can_msg_s *msg);
static bool kvaser_chrdev_txready(FAR struct can_dev_s *dev);
static bool kvaser_chrdev_txempty(FAR struct can_dev_s *dev);
static void kvaser_chardev_receive(FAR struct kvaser_sja_s *priv);
static void kvaser_chardev_interrupt(FAR struct kvaser_driver_s *priv);
#endif

#ifdef CONFIG_CAN_KVASER_SOCKET
/* SocketCAN methods */

static int  kvaser_sock_ifup(FAR struct netdev_lowerhalf_s *dev);
static int  kvaser_sock_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int  kvaser_sock_transmit(FAR struct netdev_lowerhalf_s *dev,
                                 FAR netpkt_t *pkt);
static FAR netpkt_t *kvaser_sock_recv(FAR struct netdev_lowerhalf_s *dev);

#  ifdef CONFIG_NETDEV_IOCTL
static int  kvaser_sock_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
                              unsigned long arg);
#  endif

#  ifdef CONFIG_NET_CAN_ERRORS
static FAR netpkt_t *kvaser_sock_error(FAR struct netdev_lowerhalf_s *dev);
#  endif

static void kvaser_sock_interrupt(FAR struct kvaser_driver_s *priv);
#endif

/* Interrupts */

static int kvaser_interrupt(int irq, FAR void *context, FAR void *arg);

/* PCI */

static void kvaser_init(FAR struct kvaser_driver_s *priv);
static uint8_t kvaser_count_sja(FAR struct kvaser_driver_s *priv);
static int kvaser_probe(FAR struct pci_device_s *dev);

/*****************************************************************************
 * Private Data
 *****************************************************************************/

#ifdef CONFIG_CAN_KVASER_CHARDEV
static const struct can_ops_s g_kvaser_can_ops =
{
  .co_reset         = kvaser_chrdev_reset,
  .co_setup         = kvaser_chrdev_setup,
  .co_shutdown      = kvaser_chrdev_shutdown,
  .co_rxint         = kvaser_chrdev_rxint,
  .co_txint         = kvaser_chrdev_txint,
  .co_ioctl         = kvaser_chrdev_ioctl,
  .co_remoterequest = kvaser_chrdev_remoterequest,
  .co_send          = kvaser_chrdev_send,
  .co_txready       = kvaser_chrdev_txready,
  .co_txempty       = kvaser_chrdev_txempty,
};
#endif

#ifdef CONFIG_CAN_KVASER_SOCKET
static const struct netdev_ops_s g_kvaser_net_ops =
{
  .ifup     = kvaser_sock_ifup,
  .ifdown   = kvaser_sock_ifdown,
  .transmit = kvaser_sock_transmit,
  .receive  = kvaser_sock_recv,
#ifdef CONFIG_NETDEV_IOCTL
  .ioctl    = kvaser_sock_ioctl,
#endif
};
#endif

static const struct pci_device_id_s g_kvaser_id_table[] =
{
  {
    PCI_DEVICE(0x10e8, 0x8406),
    .driver_data = 0
  },
  { }
};

static struct pci_driver_s g_kvaser_drv =
{
  .id_table = g_kvaser_id_table,
  .probe    = kvaser_probe,
};

#ifdef CONFIG_CAN_KVASER_CHARDEV
static uint8_t g_kvaser_count = 0;
#endif

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: kvaser_getreg_sja
 *****************************************************************************/

static uint8_t kvaser_getreg_sja(FAR struct kvaser_sja_s *priv,
                                 unsigned int offset)
{
  uintptr_t addr = priv->base + offset;
  uint8_t   ret  = 0;

  pci_read_io_byte(priv->pcidev, addr, &ret);
  return ret;
}

/*****************************************************************************
 * Name: kvaser_putreg_sja
 *****************************************************************************/

static void kvaser_putreg_sja(FAR struct kvaser_sja_s *priv,
                              unsigned int offset,
                              uint8_t value)
{
  uintptr_t addr = priv->base + offset;
  pci_write_io_byte(priv->pcidev, addr, value);
}

/*****************************************************************************
 * Name: kvaser_getreg_s5920
 *****************************************************************************/

static uint32_t kvaser_getreg_s5920(FAR struct kvaser_driver_s *priv,
                                    unsigned int offset)
{
  uintptr_t addr = priv->s5920_base + offset;
  uint32_t  ret  = 0;

  pci_read_io_dword(priv->pcidev, addr, &ret);
  return ret;
}

/*****************************************************************************
 * Name: kvaser_putreg_s5920
 *****************************************************************************/

static void kvaser_putreg_s5920(FAR struct kvaser_driver_s *priv,
                                unsigned int offset,
                                uint32_t value)
{
  uintptr_t addr = priv->s5920_base + offset;
  pci_write_io_dword(priv->pcidev, addr, value);
}

/*****************************************************************************
 * Name: kvaser_reset
 *****************************************************************************/

static void kvaser_reset(FAR struct kvaser_sja_s *priv)
{
  uint8_t regval = 0;

  /* Reset mode */

  kvaser_putreg_sja(priv, SJA1000_MODE_REG, SJA1000_RESET_MODE);

  /* Enter PeliCAN mode */

  regval = kvaser_getreg_sja(priv, SJA1000_CLOCK_DIVIDER_REG);
  regval |= SJA1000_EXT_MODE;
  kvaser_putreg_sja(priv, SJA1000_CLOCK_DIVIDER_REG, regval);

  /* Clear counters */

  kvaser_putreg_sja(priv, SJA1000_RX_ERR_CNT_REG, 0);
  kvaser_putreg_sja(priv, SJA1000_TX_ERR_CNT_REG, 0);
  kvaser_putreg_sja(priv, SJA1000_RX_MESSAGE_CNT_REG, 0);

  /* Accept all */

  kvaser_putreg_sja(priv, SJA1000_DATA_0_REG, 0);
  kvaser_putreg_sja(priv, SJA1000_DATA_1_REG, 0);
  kvaser_putreg_sja(priv, SJA1000_DATA_2_REG, 0);
  kvaser_putreg_sja(priv, SJA1000_DATA_3_REG, 0);

  kvaser_putreg_sja(priv, SJA1000_DATA_4_REG, 0xff);
  kvaser_putreg_sja(priv, SJA1000_DATA_5_REG, 0xff);
  kvaser_putreg_sja(priv, SJA1000_DATA_6_REG, 0xff);
  kvaser_putreg_sja(priv, SJA1000_DATA_7_REG, 0xff);
}

/*****************************************************************************
 * Name: kvaser_sleep
 *****************************************************************************/

static void kvaser_sleep(FAR struct kvaser_sja_s *priv)
{
  /* Sleep mode */

  kvaser_putreg_sja(priv, SJA1000_MODE_REG, SJA1000_SLEEP_MODE);
}

/*****************************************************************************
 * Name: kvaser_setup
 *****************************************************************************/

static void kvaser_setup(FAR struct kvaser_sja_s *priv)
{
  /* REVISIT: missing bus timings configuration and output control.
   *
   * This driver was verified on QEMU with virtual host CAN network,
   * which doesn't need bus timings and output control registers set.
   * For real hardware, these registers must be properly configured !
   */

  kvaser_putreg_sja(priv, SJA1000_BUS_TIMING_0_REG, 0);
  kvaser_putreg_sja(priv, SJA1000_BUS_TIMING_1_REG, 0);
  kvaser_putreg_sja(priv, SJA1000_OUTCTRL_REG, 0);

  /* Enable interrupts */

#if defined(CONFIG_CAN_ERRORS) || defined(CONFIG_NET_CAN_ERRORS)
  kvaser_putreg_sja(priv, SJA1000_INT_ENA_REG, KVASER_INT_ERR);
#else
  kvaser_putreg_sja(priv, SJA1000_INT_ENA_REG, 0);
#endif

  /* Exit from reset mode and set `Dual filter mode`.
   *
   * NOTE: `Single filter mode` is broken in older versions of QEMU:
   *        https://gitlab.com/qemu-project/qemu/-/issues/2028
   */

  kvaser_putreg_sja(priv, SJA1000_MODE_REG, 0);
}

/*****************************************************************************
 * Name: kvaser_rxint
 *****************************************************************************/

static void kvaser_rxint(FAR struct kvaser_sja_s *priv, bool enable)
{
  uint8_t regval = 0;

  regval = kvaser_getreg_sja(priv, SJA1000_INT_ENA_REG);
  if (enable)
    {
      regval |= SJA1000_RX_INT_ST;
    }
  else
    {
      regval &= ~SJA1000_RX_INT_ST;
    }

  kvaser_putreg_sja(priv, SJA1000_INT_ENA_REG, regval);
}

/*****************************************************************************
 * Name: kvaser_txint
 *****************************************************************************/

static void kvaser_txint(FAR struct kvaser_sja_s *priv, bool enable)
{
  uint8_t regval = 0;

  regval = kvaser_getreg_sja(priv, SJA1000_INT_ENA_REG);
  if (enable)
    {
      regval |= SJA1000_TX_INT_ST;
    }
  else
    {
      regval &= ~SJA1000_TX_INT_ST;
    }

  kvaser_putreg_sja(priv, SJA1000_INT_ENA_REG, regval);
}

/*****************************************************************************
 * Name: kvaser_txready
 *****************************************************************************/

static bool kvaser_txready(FAR struct kvaser_sja_s *priv)
{
  return kvaser_getreg_sja(priv, SJA1000_STATUS_REG) & SJA1000_TX_BUF_ST;
}

#ifdef CONFIG_CAN_KVASER_CHARDEV
/*****************************************************************************
 * Name: kvaser_chrdev_reset
 *
 * Description:
 *   Reset the CAN device.  Called early to initialize the hardware. This
 *   function is called, before kvaser_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *  None
 *
 *****************************************************************************/

static void kvaser_chrdev_reset(FAR struct can_dev_s *dev)
{
  FAR struct kvaser_sja_s *priv = (FAR struct kvaser_sja_s *)dev;

  kvaser_reset(priv);
}

/*****************************************************************************
 * Name: kvaser_chrdev_setup
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

static int kvaser_chrdev_setup(FAR struct can_dev_s *dev)
{
  FAR struct kvaser_sja_s *priv = (FAR struct kvaser_sja_s *)dev;

  kvaser_setup(priv);

  return OK;
}

/*****************************************************************************
 * Name: kvaser_chrdev_shutdown
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

static void kvaser_chrdev_shutdown(FAR struct can_dev_s *dev)
{
  FAR struct kvaser_sja_s *priv = (FAR struct kvaser_sja_s *)dev;

  kvaser_sleep(priv);
}

/*****************************************************************************
 * Name: kvaser_chrdev_rxint
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

static void kvaser_chrdev_rxint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct kvaser_sja_s *priv = (FAR struct kvaser_sja_s *)dev;

  kvaser_rxint(priv, enable);
}

/*****************************************************************************
 * Name: kvaser_chrdev_txint
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

static void kvaser_chrdev_txint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct kvaser_sja_s *priv = (FAR struct kvaser_sja_s *)dev;

  kvaser_txint(priv, enable);
}

/*****************************************************************************
 * Name: kvaser_chrdev_ioctl
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

static int kvaser_chrdev_ioctl(FAR struct can_dev_s *dev, int cmd,
                               unsigned long arg)
{
  return -ENOTTY;
}

/*****************************************************************************
 * Name: kvaser_chrdev_remoterequest
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

static int kvaser_chrdev_remoterequest(FAR struct can_dev_s *dev,
                                       uint16_t id)
{
  return -ENOTSUP;
}

/*****************************************************************************
 * Name: kvaser_chrdev_send
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

static int kvaser_chrdev_send(FAR struct can_dev_s *dev,
                              FAR struct can_msg_s *msg)
{
  FAR struct kvaser_sja_s *priv   = (FAR struct kvaser_sja_s *)dev;
  uint8_t                  regval = 0;
  uint8_t                  dreg   = 0;
  uint8_t                  fi     = 0;
  uint8_t                  bytes  = 0;
  int                      i      = 0;

  /* Check TX buffer status */

  if (!kvaser_txready(priv))
    {
      canerr("ERROR: TX buffer not available\n");
      return -EBUSY;
    }

  /* Get data bytes for this frame */

  bytes = can_dlc2bytes(msg->cm_hdr.ch_dlc);

#ifdef CONFIG_CAN_FD
  /* Drop CAN FD frames */

  if (bytes > 8 || msg->cm_hdr.ch_edl ||
      msg->cm_hdr.ch_brs || msg->cm_hdr.ch_esi)
    {
      canerr("ERROR: CAN FD frames not supported\n");
      return -ENOTSUP;
    }
#endif

  /* Set up the DLC */

  fi = msg->cm_hdr.ch_dlc;

#ifdef CONFIG_CAN_USE_RTR
  fi |= (msg->cm_hdr.ch_rtr ? SJA1000_FI_RTR : 0);
#endif

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      fi |= SJA1000_FI_FF;
      kvaser_putreg_sja(priv, SJA1000_DATA_0_REG, fi);

      /* Write ID */

      regval = (msg->cm_hdr.ch_id & 0x1fe00000) >> 21;
      kvaser_putreg_sja(priv, SJA1000_DATA_1_REG, regval);
      regval = (msg->cm_hdr.ch_id & 0x001fe000) >> 13;
      kvaser_putreg_sja(priv, SJA1000_DATA_2_REG, regval);
      regval = (msg->cm_hdr.ch_id & 0x00001fe0) >> 5;
      kvaser_putreg_sja(priv, SJA1000_DATA_3_REG, regval);
      regval = (msg->cm_hdr.ch_id & 0x0000001f) << 3;
      kvaser_putreg_sja(priv, SJA1000_DATA_4_REG, regval);

      dreg = SJA1000_DATA_5_REG;
    }
  else
#endif
    {
      kvaser_putreg_sja(priv, SJA1000_DATA_0_REG, fi);

      /* Write ID */

      regval = (msg->cm_hdr.ch_id & 0x07f8) >> 3;
      kvaser_putreg_sja(priv, SJA1000_DATA_1_REG, regval);
      regval = (msg->cm_hdr.ch_id & 0x0007) << 5;
      kvaser_putreg_sja(priv, SJA1000_DATA_2_REG, regval);

      dreg = SJA1000_DATA_3_REG;
    }

  /* Set up the data fields */

  for (i = 0; i < bytes; i++)
    {
      kvaser_putreg_sja(priv, dreg + i, msg->cm_data[i]);
    }

  /* Transmit */

  regval = SJA1000_TX_REQ;
#ifdef CONFIG_CAN_LOOPBACK
  regval |= SJA1000_SELF_RX_REQ;
#endif

  kvaser_putreg_sja(priv, SJA1000_CMD_REG, regval);

  return OK;
}

/*****************************************************************************
 * Name: kvaser_chrdev_txready
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

static bool kvaser_chrdev_txready(FAR struct can_dev_s *dev)
{
  FAR struct kvaser_sja_s *priv = (FAR struct kvaser_sja_s *)dev;

  return kvaser_txready(priv);
}

/*****************************************************************************
 * Name: kvaser_chrdev_txempty
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

static bool kvaser_chrdev_txempty(FAR struct can_dev_s *dev)
{
  FAR struct kvaser_sja_s *priv = (FAR struct kvaser_sja_s *)dev;

  return kvaser_txready(priv);
}

/*****************************************************************************
 * Name: kvaser_chardev_receive
 *
 * Description:
 *   Receive CAN frame
 *
 *****************************************************************************/

static void kvaser_chardev_receive(FAR struct kvaser_sja_s *priv)
{
  FAR struct can_dev_s *dev = (FAR struct can_dev_s *)priv;
  struct can_hdr_s      hdr;
  uint8_t               data[CAN_MAXDATALEN];
  uint8_t               dreg;
  uint8_t               regval;
  uint8_t               i;
  int                   ret;

  /* Wait until data in RXFIFO */

  while (kvaser_getreg_sja(priv, SJA1000_STATUS_REG) & SJA1000_RX_BUF_ST)
    {
      regval = kvaser_getreg_sja(priv, SJA1000_DATA_0_REG);

      /* Get the DLC */

      hdr.ch_dlc = (regval & SJA1000_FI_DLC_MASK);

      /* Get RTR bit */

      hdr.ch_rtr = (regval & SJA1000_FI_RTR ? 1 : 0);

      if (!(regval & SJA1000_FI_FF))
        {
#ifdef CONFIG_CAN_EXTID
          hdr.ch_extid = 0;
#endif

          hdr.ch_id = ((kvaser_getreg_sja(priv, SJA1000_DATA_1_REG) << 3) |
                       (kvaser_getreg_sja(priv, SJA1000_DATA_2_REG) >> 5));

          dreg = SJA1000_DATA_3_REG;
        }
      else
        {
#ifdef CONFIG_CAN_EXTID
          hdr.ch_extid = 1;

          hdr.ch_id = ((kvaser_getreg_sja(priv, SJA1000_DATA_1_REG) << 21) |
                       (kvaser_getreg_sja(priv, SJA1000_DATA_2_REG) << 13) |
                       (kvaser_getreg_sja(priv, SJA1000_DATA_3_REG) << 5)  |
                       (kvaser_getreg_sja(priv, SJA1000_DATA_4_REG) >> 3));

          dreg = SJA1000_DATA_5_REG;
#else
          canerr("ERROR: Received message with extended"
                 " identifier.  Dropped\n");

          /* Release RX buffer */

          kvaser_putreg_sja(priv, SJA1000_CMD_REG, SJA1000_RELEASE_BUF);

          continue;
#endif
        }

      /* Clear the error indication and unused bits */

#ifdef CONFIG_CAN_ERRORS
      hdr.ch_error = 0;
#endif
      hdr.ch_tcf   = 0;

      /* Get data */

      for (i = 0; i < can_dlc2bytes(hdr.ch_dlc); i++)
        {
          data[i] = kvaser_getreg_sja(priv, dreg + i);
        }

      /* Release RX buffer */

      kvaser_putreg_sja(priv, SJA1000_CMD_REG, SJA1000_RELEASE_BUF);

      /* Provide the data to the upper half driver */

      ret = can_receive(dev, &hdr, data);
      if (ret < 0)
        {
          canerr("ERROR: can_receive failed %d\n", ret);
        }
    }
}

#ifdef CONFIG_CAN_ERRORS
/*****************************************************************************
 * Name: kvaser_chardev_error
 *****************************************************************************/

static void kvaser_chardev_error(FAR struct kvaser_sja_s *priv, uint8_t st)
{
  struct can_hdr_s hdr;
  uint16_t         errbits = 0;
  uint8_t          data[CAN_ERROR_DLC];
  uint8_t          regval;
  int              ret;

  memset(data, 0, sizeof(data));

  /* Data overrun interrupt */

  if (st & SJA1000_OVERRUN_INT_ST)
    {
      data[1] |= CAN_ERROR1_RXOVERFLOW;
      errbits |= CAN_ERROR_CONTROLLER;

      /* Clear data overrun */

      kvaser_putreg_sja(priv, SJA1000_CMD_REG, SJA1000_CLR_OVERRUN);
    }

  /* Error passive interrupt */

  if (st & SJA1000_ERR_PASSIVE_INT_ST)
    {
      data[1] |= (CAN_ERROR1_RXPASSIVE | CAN_ERROR1_TXPASSIVE);
      errbits |= CAN_ERROR_CONTROLLER;
    }

  /* Arbitration lost interrupt */

  if (st & SJA1000_ARB_LOST_INT_ST)
    {
      errbits |= CAN_ERROR_LOSTARB;
      regval = (kvaser_getreg_sja(priv, SJA1000_ARB_LOST_CAP_REG) &
                SJA1000_ARB_LOST_CAP_M);
      data[0] = CAN_ERROR0_BIT(regval);
    }

  /* BUS error interrupt */

  if (st & SJA1000_BUS_ERR_INT_ST)
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
 * Name: kvaser_chardev_interrupt
 *****************************************************************************/

static void kvaser_chardev_interrupt(FAR struct kvaser_driver_s *priv)
{
  uint8_t st = 0;
  int     i  = 0;
  int     passes;

  for (passes = 0; passes < CONFIG_CAN_KVASER_IRQ_PASSES; passes++)
    {
      for (i = 0; i < priv->count; i++)
        {
          st = kvaser_getreg_sja(&priv->sja[i], SJA1000_INT_RAW_REG);
          if (st == 0)
            {
              continue;
            }

          /* Handle RX frames */

          kvaser_chardev_receive(&priv->sja[i]);

          /* Transmit interrupt */

          if (st & SJA1000_TX_INT_ST)
            {
              /* Tell the upper half that the transfer is finished. */

              can_txdone(&priv->sja[i].dev);
            }

#ifdef CONFIG_CAN_ERRORS
          /* Handle errors */

          kvaser_chardev_error(&priv->sja[i], st);
#endif
        }
    }
}
#endif  /* CONFIG_CAN_KVASER_CHARDEV */

#ifdef CONFIG_CAN_KVASER_SOCKET
/*****************************************************************************
 * Name: kvaser_sock_ifup
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

static int kvaser_sock_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct kvaser_sja_s *priv = (FAR struct kvaser_sja_s *)dev;

  kvaser_reset(priv);
  kvaser_setup(priv);

  kvaser_txint(priv, true);
  kvaser_rxint(priv, true);

  return OK;
}

/*****************************************************************************
 * Name: kvaser_sock_ifdown
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

static int kvaser_sock_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct kvaser_sja_s *priv = (FAR struct kvaser_sja_s *)dev;

  kvaser_txint(priv, false);
  kvaser_rxint(priv, false);

  /* Sleep mode */

  kvaser_sleep(priv);

  return OK;
}

/*****************************************************************************
 * Name: kvaser_sock_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   Return OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 *****************************************************************************/

static int kvaser_sock_transmit(FAR struct netdev_lowerhalf_s *dev,
                                FAR netpkt_t *pkt)
{
  FAR struct kvaser_sja_s *priv   = (FAR struct kvaser_sja_s *)dev;
  FAR struct can_frame    *frame  = NULL;
  uint8_t                  regval = 0;
  uint8_t                  dreg   = 0;
  uint8_t                  fi     = 0;
  int                      i      = 0;

  /* Check TX buffer status */

  if (!kvaser_txready(priv))
    {
      canerr("ERROR: TX buffer not available\n");
      return -EBUSY;
    }

#ifdef CONFIG_CAN_FD
  /* Drop CAN FD frames */

  if (netpkt_getdatalen(dev, pkt) != sizeof(struct can_frame))
    {
      canerr("ERROR: CAN FD frames not supported\n");
      return -ENOTSUP;
    }
#endif

  /* Get frame data */

  frame = (FAR struct can_frame *)netpkt_getdata(dev, pkt);

  /* Set up the DLC */

  fi = frame->can_dlc;

  /* Set RTR bit */

  fi |= (frame->can_id & CAN_RTR_FLAG ? SJA1000_FI_RTR : 0);

#ifdef CONFIG_NET_CAN_EXTID
  if (frame->can_id & CAN_EFF_FLAG)
    {
      fi |= SJA1000_FI_FF;
      kvaser_putreg_sja(priv, SJA1000_DATA_0_REG, fi);

      /* Write ID */

      regval = (frame->can_id & 0x1fe00000) >> 21;
      kvaser_putreg_sja(priv, SJA1000_DATA_1_REG, regval);
      regval = (frame->can_id & 0x001fe000) >> 13;
      kvaser_putreg_sja(priv, SJA1000_DATA_2_REG, regval);
      regval = (frame->can_id & 0x00001fe0) >> 5;
      kvaser_putreg_sja(priv, SJA1000_DATA_3_REG, regval);
      regval = (frame->can_id & 0x0000001f) << 3;
      kvaser_putreg_sja(priv, SJA1000_DATA_4_REG, regval);

      dreg = SJA1000_DATA_5_REG;
    }
  else
#endif
    {
      kvaser_putreg_sja(priv, SJA1000_DATA_0_REG, fi);

      /* Write ID */

      regval = (frame->can_id & 0x07f8) >> 3;
      kvaser_putreg_sja(priv, SJA1000_DATA_1_REG, regval);
      regval = (frame->can_id & 0x0007) << 5;
      kvaser_putreg_sja(priv, SJA1000_DATA_2_REG, regval);

      dreg = SJA1000_DATA_3_REG;
    }

  /* Set up the data fields */

  if (frame->can_dlc > 8)
    {
      frame->can_dlc = 8;
    }

  for (i = 0; i < frame->can_dlc; i++)
    {
      kvaser_putreg_sja(priv, dreg + i, frame->data[i]);
    }

  /* Transmit */

  regval = SJA1000_TX_REQ;
  kvaser_putreg_sja(priv, SJA1000_CMD_REG, regval);

  /* All is done - free packet */

  netpkt_free(dev, pkt, NETPKT_TX);

  return OK;
}

/*****************************************************************************
 * Name: kvaser_sock_recv
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   A pointer to received packet
 *
 * Assumptions:
 *   The network is locked.
 *
 *****************************************************************************/

static FAR netpkt_t *kvaser_sock_recv(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct kvaser_sja_s *priv  = (FAR struct kvaser_sja_s *)dev;
  FAR struct can_frame    *frame = NULL;
  FAR netpkt_t            *pkt   = NULL;
  uint8_t                  dreg  = 0;
  int                      i     = 0;
  uint8_t                  regval;
  uint8_t                  st;

  /* Read frame if RXFIFO not empty */

  st = kvaser_getreg_sja(priv, SJA1000_STATUS_REG);
  if ((st & SJA1000_RX_BUF_ST) == 0)
    {
#ifdef CONFIG_NET_CAN_ERRORS
      /* Check for errors */

      return kvaser_sock_error(dev);
#else
      return NULL;
#endif
    }

  /* Allocate packet */

  pkt = netpkt_alloc(dev, NETPKT_RX);
  if (!pkt)
    {
      canerr("alloc pkt_new failed\n");
      return NULL;
    }

  netpkt_setdatalen(dev, pkt, sizeof(struct can_frame));
  frame = (FAR struct can_frame *)netpkt_getdata(dev, pkt);

  regval = kvaser_getreg_sja(priv, SJA1000_DATA_0_REG);

  /* Get the DLC */

  frame->can_dlc = (regval & SJA1000_FI_DLC_MASK);

  /* Get the CAN identifier. */

  if (!(regval & SJA1000_FI_FF))
    {
      frame->can_id =
        ((kvaser_getreg_sja(priv, SJA1000_DATA_1_REG) << 3) |
         (kvaser_getreg_sja(priv, SJA1000_DATA_2_REG) >> 5));

      dreg = SJA1000_DATA_3_REG;
    }
  else
    {
#ifdef CONFIG_NET_CAN_EXTID
      frame->can_id =
        ((kvaser_getreg_sja(priv, SJA1000_DATA_1_REG) << 21) |
         (kvaser_getreg_sja(priv, SJA1000_DATA_2_REG) << 13) |
         (kvaser_getreg_sja(priv, SJA1000_DATA_3_REG) << 5)  |
         (kvaser_getreg_sja(priv, SJA1000_DATA_4_REG) >> 3));
      frame->can_id |= CAN_EFF_FLAG;

      dreg = SJA1000_DATA_5_REG;
#else
      canerr("ERROR: Received message with extended"
             " identifier.  Dropped\n");

      /* Release RX buffer */

      kvaser_putreg_sja(priv, SJA1000_CMD_REG, SJA1000_RELEASE_BUF);

      netpkt_free(dev, pkt);
      return NULL;
#endif
    }

  /* Extract the RTR bit */

  if (regval & SJA1000_FI_RTR)
    {
      frame->can_id |= CAN_RTR_FLAG;
    }

  /* Get data */

  for (i = 0; i < can_dlc2bytes(frame->can_dlc); i++)
    {
      frame->data[i] = kvaser_getreg_sja(priv, dreg + i);
    }

  /* Release RX buffer */

  kvaser_putreg_sja(priv, SJA1000_CMD_REG, SJA1000_RELEASE_BUF);

  return pkt;
}

#  ifdef CONFIG_NETDEV_IOCTL
/*****************************************************************************
 * Name: kvaser_sock_ioctl
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

static int kvaser_sock_ioctl(FAR struct netdev_lowerhalf_s, int cmd,
                             unsigned long arg)
{
  return -ENOTTY;
}

#  endif  /* CONFIG_NETDEV_IOCTL */

#ifdef CONFIG_NET_CAN_ERRORS
/*****************************************************************************
 * Name: kvaser_sock_error
 *****************************************************************************/

static FAR netpkt_t *kvaser_sock_error(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct kvaser_sja_s *priv    = (FAR struct kvaser_sja_s *)dev;
  FAR struct can_frame    *frame   = NULL;
  FAR netpkt_t            *pkt     = NULL;
  uint8_t                  regval  = 0;
  uint8_t                  st      = 0;
  uint16_t                 errbits = 0;

  /* Check for no errors */

  st = kvaser_getreg_sja(priv, SJA1000_INT_RAW_REG);
  if ((st & KVASER_INT_ERR_ST) == 0)
    {
      return NULL;
    }

  /* Allocate packet */

  pkt = netpkt_alloc(dev, NETPKT_RX);
  if (!pkt)
    {
      canerr("alloc pkt_new failed\n");
      return NULL;
    }

  netpkt_setdatalen(dev, pkt, sizeof(struct can_frame));
  frame = (FAR struct can_frame *)netpkt_getdata(dev, pkt);

  /* Data overrun interrupt */

  if (st & SJA1000_OVERRUN_INT_ST)
    {
      frame->data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
      errbits        |= CAN_ERR_CRTL;

      /* Clear data overrun */

      kvaser_putreg_sja(priv, SJA1000_CMD_REG, SJA1000_CLR_OVERRUN);
    }

  /* Error passive interrupt */

  if (st & SJA1000_ERR_PASSIVE_INT_ST)
    {
      frame->data[1] |= (CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE);
      errbits        |= CAN_ERR_CRTL;
    }

  /* Arbitration lost interrupt */

  if (st & SJA1000_ARB_LOST_INT_ST)
    {
      errbits |= CAN_ERR_LOSTARB;
      regval = (kvaser_getreg_sja(priv, SJA1000_ARB_LOST_CAP_REG) &
                SJA1000_ARB_LOST_CAP_M);
      frame->data[0] = CAN_ERR_LOSTARB_BIT(regval);
    }

  /* BUS error interrupt */

  if (st & SJA1000_BUS_ERR_INT_ST)
    {
      errbits |= CAN_ERR_BUSERROR;
    }

  canerr("ERROR: errbits = %08" PRIx16 "\n", errbits);

  /* Copy frame */

  frame->can_id  = errbits;
  frame->can_dlc = CAN_ERR_DLC;

  return pkt;
}
#endif

/*****************************************************************************
 * Name: kvaser_sock_interrupt
 *****************************************************************************/

static void kvaser_sock_interrupt(FAR struct kvaser_driver_s *priv)
{
  uint8_t st = 0;
  uint8_t i  = 0;
  int     passes;

  for (passes = 0; passes < CONFIG_CAN_KVASER_IRQ_PASSES; passes++)
    {
      for (i = 0; i < priv->count; i++)
        {
          st = kvaser_getreg_sja(&priv->sja[i], SJA1000_INT_RAW_REG);
          if (st == 0)
            {
              continue;
            }

          /* RX ready */

          if (st & KVASER_SOCK_RXINT)
            {
              netdev_lower_rxready(&priv->sja[i].dev);
            }

          /* Transmit interrupt */

          if (st & SJA1000_TX_INT_ST)
            {
              netdev_lower_txdone(&priv->sja[i].dev);
            }
        }
    }
}
#endif /* CONFIG_CAN_KVASER_SOCKET */

/*****************************************************************************
 * Name: kvaser_interrupt
 *
 * Description:
 *   Initialize SJA1000 device
 *
 *****************************************************************************/

static int kvaser_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct kvaser_driver_s *priv = (FAR struct kvaser_driver_s *)arg;

  DEBUGASSERT(priv != NULL);

  if (kvaser_getreg_s5920(priv, KVASER_S5920_INTCSR) &
      KVASER_S5920_INTCSR_INT_ASSERT)
    {
#ifdef CONFIG_CAN_KVASER_CHARDEV
      /* Handle character device interrupt */

      kvaser_chardev_interrupt(priv);
#endif

#ifdef CONFIG_CAN_KVASER_SOCKET
      /* Handle SocketCAN interrupt */

      kvaser_sock_interrupt(priv);
#endif
    }

  return OK;
}

/*****************************************************************************
 * Name: kvaser_init
 *
 * Description:
 *   Initialize SJA1000 device
 *
 *****************************************************************************/

static void kvaser_init(FAR struct kvaser_driver_s *priv)
{
  uint32_t regval = 0;

  /* Only legacy IRQ supported by kvaser */

  priv->irq = pci_get_irq(priv->pcidev);
  irq_attach(priv->irq, kvaser_interrupt, priv);

  /* Enable card interrupts */

  regval = kvaser_getreg_s5920(priv, KVASER_S5920_INTCSR);
  regval |= KVASER_S5920_INTCSR_INT_ADDON;
  kvaser_putreg_s5920(priv, KVASER_S5920_INTCSR, regval);

  /* Enable interrupts */

  up_enable_irq(priv->irq);
}

/*****************************************************************************
 * Name: kvaser_count_sja
 *
 * Description:
 *   Probe SJA1000 devices on board and return the number of available chips.
 *
 *****************************************************************************/

static uint8_t kvaser_count_sja(FAR struct kvaser_driver_s *priv)
{
  uint32_t offset;
  uint8_t  regval;
  uint8_t  i;

  /* Reset chip and check if reset bit has changed */

  for (i = 0; i < KVASER_SJA_MAX; i++)
    {
      offset = priv->sja_base + SJA1000_MODE_REG + (i * KVASER_SJA_REGS);

      pci_write_io_byte(priv->pcidev, offset, SJA1000_RESET_MODE);
      pci_read_io_byte(priv->pcidev, offset, &regval);

      if (regval != SJA1000_RESET_MODE)
        {
          break;
        }
    }

  return i;
}

/*****************************************************************************
 * Name: kvaser_probe
 *
 * Description:
 *   Probe device
 *
 *****************************************************************************/

static int kvaser_probe(FAR struct pci_device_s *dev)
{
#ifdef CONFIG_CAN_KVASER_SOCKET
  FAR struct netdev_lowerhalf_s *netdev = NULL;
#endif
  FAR struct kvaser_driver_s    *priv   = NULL;
  uint8_t                        i      = 0;
  int                            ret;
#ifdef CONFIG_CAN_KVASER_CHARDEV
  uint8_t                        count;
  char                           devpath[PATH_MAX];
#endif

  /* Allocate the interface structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  /* Initialzie PCI dev */

  priv->pcidev = dev;

  pci_set_master(dev);
  pciinfo("Enabled bus mastering\n");
  pci_enable_device(dev);
  pciinfo("Enabled memory resources\n");

  /* Kvaser cards support only IO access */

  if (pci_resource_flags(dev, KVASER_SJA_BAR) != PCI_RESOURCE_IO)
    {
      ret = -ENOTSUP;
      goto errout;
    }

  /* Get S5920 base */

  priv->s5920_base = (uintptr_t)pci_map_bar(dev, KVASER_S5920_BAR);
  if (!priv->s5920_base)
    {
      pcierr("Not found S5920 bar\n");
      ret = -ENXIO;
      goto errout;
    }

  /* Get SJA1000 base */

  priv->sja_base = (uintptr_t)pci_map_bar(dev, KVASER_SJA_BAR);
  if (!priv->sja_base)
    {
      pcierr("Not found SJA bar\n");
      ret = -ENXIO;
      goto errout;
    }

  /* Get XILINX base */

  priv->xilinx_base = (uintptr_t)pci_map_bar(dev, KVASER_XILINX_BAR);
  if (!priv->xilinx_base)
    {
      pcierr("Not found XILINX bar\n");
      ret = -ENXIO;
      goto errout;
    }

  /* Get number of SJA1000 chips */

  priv->count = kvaser_count_sja(priv);

  pciinfo("detected %d SJA1000 deviced\n", priv->count);

  /* Allocate SJA1000 devices */

  priv->sja = kmm_zalloc(sizeof(struct kvaser_sja_s) * priv->count);
  if (priv->sja == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* Common initialziation for all channels */

  kvaser_init(priv);

  /* Handle all SJA1000 devices */

  for (i = 0; i < priv->count; i++)
    {
      /* Common initialization */

      priv->sja[i].base   = priv->sja_base + (i * KVASER_SJA_REGS);
      priv->sja[i].pcidev = dev;

#ifdef CONFIG_CAN_KVASER_CHARDEV
      count = g_kvaser_count++;

      /* Get devpath for this SJA1000 device */

      snprintf(devpath, PATH_MAX, "/dev/can%d", count);

      /* Initialize SJA1000 channel */

      priv->sja[i].dev.cd_ops  = &g_kvaser_can_ops;
      priv->sja[i].dev.cd_priv = &priv->sja[i];

      /* Register CAN device */

      ret = can_register(devpath, &priv->sja[i].dev);
      if (ret < 0)
        {
          pcierr("ERROR: failed to register count=%d, %d\n", i, ret);
          goto errout;
        }
#endif

#ifdef CONFIG_CAN_KVASER_SOCKET
      netdev = &priv->sja[i].dev;

      /* Register the network device */

      netdev->quota[NETPKT_TX] = KVASER_TX_QUOTA;
      netdev->quota[NETPKT_RX] = KVASER_RX_QUOTA;
      netdev->ops = &g_kvaser_net_ops;

      /* Put the interface in the down state.  This usually amounts to
       * resetting the device and/or calling kvaser_sock_ifdown().
       */

      kvaser_sock_ifdown(&priv->sja[i].dev);

      /* Register the network interface. */

      ret = netdev_lower_register(netdev, NET_LL_CAN);
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
      if (priv->sja[i].pcidev)
        {
#ifdef CONFIG_CAN_KVASER_SOCKET
          netdev = &priv->sja[i].dev;
          netdev_lower_unregister(netdev);
#endif

#ifdef CONFIG_CAN_KVASER_CHARDEV
          snprintf(devpath, PATH_MAX, "/dev/can%d", i);
          unregister_driver(devpath);
#endif
        }
    }

  kmm_free(priv->sja);
  kmm_free(priv);

  return ret;
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: pci_kvaser_init
 *
 * Description:
 *   Register a pci driver
 *
 *****************************************************************************/

int pci_kvaser_init(void)
{
  return pci_register_driver(&g_kvaser_drv);
}

/****************************************************************************
 * arch/arm/src/stm32h5/stm32_usbdrdhost.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/usb/usb.h>

#include <arch/barriers.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_devaddr.h>
#include <nuttx/usb/usbhost_trace.h>

#include <nuttx/irq.h>

#include "chip.h"
#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "hardware/stm32_pinmap.h"
#include "hardware/stm32_usbfs.h"
#include "hardware/stm32h5xxx_rcc.h"
#include "hardware/stm32h5xxx_pwr.h"
#include "stm32_usbdrdhost.h"

#if defined(CONFIG_USBHOST) && defined(CONFIG_STM32H5_USBFS_HOST)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

#ifndef CONFIG_STM32H5_USBDRD_NCHANNELS
#  define CONFIG_STM32H5_USBDRD_NCHANNELS 8
#endif

#ifndef CONFIG_STM32H5_USBDRD_DESCSIZE
#  define CONFIG_STM32H5_USBDRD_DESCSIZE 128
#endif

#ifndef CONFIG_STM32H5_USBDRD_TRANSFER_TIMEOUT
#  define CONFIG_STM32H5_USBDRD_TRANSFER_TIMEOUT 5000
#endif

/* Hardware definitions */

#define STM32H5_NHOST_CHANNELS    CONFIG_STM32H5_USBDRD_NCHANNELS
#define STM32H5_EP0_MAX_PACKET_SIZE 64
#define STM32H5_RETRY_COUNT       3   /* Control transfer retries */

/* PMA Buffer allocation (fixed-size bitmap allocator) */

#define STM32H5_PMA_BUFFER_SIZE        64          /* Fixed buffer size (bytes) */
#define STM32H5_PMA_NBUFFERS           30          /* Total allocatable buffers */
#define STM32H5_PMA_BUFFER_ALLSET      0x3fffffff  /* All 30 buffers available */
#define STM32H5_PMA_BUFFER_BIT(bn)     (1U << (bn))
#define STM32H5_PMA_BUFNO2ADDR(bn)     (USB_DRD_PMA_START_ADDR + ((bn) * STM32H5_PMA_BUFFER_SIZE))
#define STM32H5_PMA_BUFFER_NONE        0xFF   /* Invalid buffer number */

/* Delays */

#define STM32H5_DATANAK_DELAY     SEC2TICK(5)
#define STM32H5_RESET_DELAY       100 /* ms */

/* USB DRD base addresses */

#define STM32H5_USBDRD_BASE       STM32_USB_FS_BASE
#define STM32H5_USBDRD_PMA_BASE   STM32_USB_FS_RAM_BASE

/* Register access helpers */

#define stm32_getreg(addr)      getreg32(addr)
#define stm32_putreg(addr, val) putreg32(val, addr)
#define stm32_modifyreg(addr, clearbits, setbits) modifyreg32(addr, clearbits, setbits)

/* Channel register access */

#define STM32H5_USB_CHEP(n)       (STM32H5_USBDRD_BASE + ((n) << 2))

/* Host channel data PID values */

#define HC_PID_DATA0                   (0)
#define HC_PID_DATA1                   (1)
#define HC_PID_DATA2                   (2)
#define HC_PID_SETUP                   (3)

/* Host channel direction */

#define CH_OUT_DIR                     (0)
#define CH_IN_DIR                      (1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* USB host state machine states */

enum stm32_smstate_e
{
  SMSTATE_DETACHED = 0,  /* Not attached to a device */
  SMSTATE_ATTACHED,      /* Attached to a device */
  SMSTATE_ENUM,          /* Attached, enumerating */
  SMSTATE_CLASS_BOUND,   /* Enumeration complete, class bound */
};

/* Channel halt reason */

enum stm32_chreason_e
{
  CHREASON_IDLE = 0,     /* Inactive */
  CHREASON_FREED,        /* Channel no longer in use */
  CHREASON_XFRC,         /* Transfer complete */
  CHREASON_NAK,          /* NAK received */
  CHREASON_STALL,        /* Endpoint stalled */
  CHREASON_TXERR,        /* Transfer error */
  CHREASON_DTERR,        /* Data toggle error */
  CHREASON_CANCELLED     /* Transfer cancelled */
};

/* Host channel state */

struct stm32_chan_s
{
  sem_t             waitsem;   /* Channel wait semaphore */
  volatile uint8_t  result;    /* Transfer result */
  volatile uint8_t  chreason;  /* Halt reason */
  uint8_t           chidx;     /* Channel index */
  uint8_t           epno;      /* Device endpoint number */
  uint8_t           eptype;    /* Endpoint type */
  uint8_t           funcaddr;  /* Device function address */
  uint8_t           speed;     /* Device speed */
  uint8_t           interval;  /* Polling interval */
  uint8_t           pid;       /* Data PID */
  bool              inuse;     /* Channel in use */
  volatile bool     indata1;   /* IN data toggle */
  volatile bool     outdata1;  /* OUT data toggle */
  bool              in;        /* IN endpoint */
  volatile bool     waiter;    /* Thread waiting */
  uint16_t          maxpacket; /* Max packet size */
  uint16_t          buflen;    /* Buffer length */
  volatile uint16_t xfrd;      /* Bytes transferred */
  uint16_t          pmaaddr;   /* PMA buffer address */
  uint8_t           pmabufno;  /* PMA buffer number (0xFF = none) */
  uint8_t          *buffer;    /* Transfer buffer */
  usbhost_asynch_t  callback;  /* Async callback */
  void             *arg;       /* Callback argument */
};

/* Control endpoint info */

struct stm32_ctrlinfo_s
{
  uint8_t           inndx;     /* EP0 IN channel index */
  uint8_t           outndx;    /* EP0 OUT channel index */
};

/* USB host driver state */

struct stm32_usbhost_s
{
  /* NuttX driver interface - MUST be first */

  struct usbhost_driver_s drvr;

  /* Root hub port description */

  struct usbhost_roothubport_s rhport;

  /* Driver status */

  volatile uint8_t  smstate;   /* State machine state */
  uint8_t           chidx;     /* Channel waiting for TX */
  volatile bool     connected; /* Device connected */
  volatile bool     change;    /* Connection change */
  volatile bool     pscwait;   /* Waiting for port event */
  mutex_t           lock;      /* Access mutex */
  sem_t             pscsem;    /* Port status change sem */
  struct stm32_ctrlinfo_s ep0; /* EP0 description */

#ifdef CONFIG_USBHOST_HUB
  /* Used to pass external hub port events */

  volatile struct usbhost_hubport_s *hport;
#endif

  struct usbhost_devaddr_s devgen;  /* Address generation data */

  /* Host channels */

  struct stm32_chan_s chan[STM32H5_NHOST_CHANNELS];

  /* PMA allocation */

  uint32_t          pma_bufavail;  /* Bitmap of available PMA buffers */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Data conversion helpers */

static inline uint16_t stm32_getle16(const uint8_t *val);

/* PMA buffer management */

static void stm32_pma_write(const uint8_t *buffer, uint16_t pmaaddr,
                               uint16_t nbytes);
static void stm32_pma_read(uint8_t *buffer, uint16_t pmaaddr,
                              uint16_t nbytes);
static int stm32_pma_alloc_buffer(struct stm32_usbhost_s *priv);
static void stm32_pma_free_buffer(struct stm32_usbhost_s *priv,
                                   uint8_t bufno);

/* Channel management */

static int stm32_chan_alloc(struct stm32_usbhost_s *priv);
static inline void stm32_chan_free(struct stm32_usbhost_s *priv,
                                      int chidx);
static inline void stm32_chan_freeall(struct stm32_usbhost_s *priv);
static void stm32_chan_configure(struct stm32_usbhost_s *priv,
                                    int chidx);
static int stm32_chan_waitsetup(struct stm32_usbhost_s *priv,
                                   struct stm32_chan_s *chan);
static int stm32_chan_wait(struct stm32_usbhost_s *priv,
                              struct stm32_chan_s *chan,
                              int timeout_ms);
static void stm32_chan_wakeup(struct stm32_usbhost_s *priv,
                                 struct stm32_chan_s *chan);
static void stm32_set_chep_rx_status(struct stm32_usbhost_s *priv,
                                     int chidx, uint32_t status);
static void stm32_set_chep_tx_status(struct stm32_usbhost_s *priv,
                                     int chidx, uint32_t status);

/* Control endpoint helpers */

static int stm32_ctrlchan_alloc(struct stm32_usbhost_s *priv,
                                   uint8_t epno, uint8_t funcaddr,
                                   uint8_t speed,
                                   struct stm32_ctrlinfo_s *ctrlep);
static int stm32_ctrlep_alloc(struct stm32_usbhost_s *priv,
                                 const struct usbhost_epdesc_s *epdesc,
                                 usbhost_ep_t *ep);
static int stm32_xfrep_alloc(struct stm32_usbhost_s *priv,
                                const struct usbhost_epdesc_s *epdesc,
                                usbhost_ep_t *ep);

/* Transfer functions */

static void stm32_transfer_start(struct stm32_usbhost_s *priv,
                                    int chidx);
static int stm32_ctrl_sendsetup(struct stm32_usbhost_s *priv,
                                   struct stm32_ctrlinfo_s *ep0,
                                   const struct usb_ctrlreq_s *req);
static int stm32_ctrl_senddata(struct stm32_usbhost_s *priv,
                                  struct stm32_ctrlinfo_s *ep0,
                                  uint8_t *buffer, unsigned int buflen);
static int stm32_ctrl_recvdata(struct stm32_usbhost_s *priv,
                                  struct stm32_ctrlinfo_s *ep0,
                                  uint8_t *buffer, unsigned int buflen);
static ssize_t stm32_in_transfer(struct stm32_usbhost_s *priv,
                                    int chidx, uint8_t *buffer,
                                    size_t buflen);
static ssize_t stm32_out_transfer(struct stm32_usbhost_s *priv,
                                     int chidx, uint8_t *buffer,
                                     size_t buflen);

/* Interrupt handling */

static void stm32_hc_in_irq(struct stm32_usbhost_s *priv, int chidx);
static void stm32_hc_out_irq(struct stm32_usbhost_s *priv, int chidx);
static int stm32_usbdrd_interrupt(int irq, void *context, void *arg);
static void stm32_gint_connected(struct stm32_usbhost_s *priv);
static void stm32_gint_disconnected(struct stm32_usbhost_s *priv);

/* USB host driver interface */

static int stm32_wait(struct usbhost_connection_s *conn,
                         struct usbhost_hubport_s **hport);
static int stm32_rh_enumerate(struct stm32_usbhost_s *priv,
                                 struct usbhost_connection_s *conn,
                                 struct usbhost_hubport_s *hport);
static int stm32_enumerate(struct usbhost_connection_s *conn,
                              struct usbhost_hubport_s *hport);

static int stm32_ep0configure(struct usbhost_driver_s *drvr,
                                 usbhost_ep_t ep0, uint8_t funcaddr,
                                 uint8_t speed, uint16_t maxpacketsize);
static int stm32_epalloc(struct usbhost_driver_s *drvr,
                            const struct usbhost_epdesc_s *epdesc,
                            usbhost_ep_t *ep);
static int stm32_epfree(struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int stm32_alloc(struct usbhost_driver_s *drvr,
                          uint8_t **buffer, size_t *maxlen);
static int stm32_free(struct usbhost_driver_s *drvr, uint8_t *buffer);
static int stm32_ioalloc(struct usbhost_driver_s *drvr,
                            uint8_t **buffer, size_t buflen);
static int stm32_iofree(struct usbhost_driver_s *drvr, uint8_t *buffer);
static int stm32_ctrlin(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                           const struct usb_ctrlreq_s *req, uint8_t *buffer);
static int stm32_ctrlout(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                            const struct usb_ctrlreq_s *req,
                            const uint8_t *buffer);
static ssize_t stm32_transfer(struct usbhost_driver_s *drvr,
                                 usbhost_ep_t ep, uint8_t *buffer,
                                 size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static int stm32_asynch(struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                           uint8_t *buffer, size_t buflen,
                           usbhost_asynch_t callback, void *arg);
#endif
static int stm32_cancel(struct usbhost_driver_s *drvr, usbhost_ep_t ep);
#ifdef CONFIG_USBHOST_HUB
static int stm32_connect(struct usbhost_driver_s *drvr,
                            struct usbhost_hubport_s *hport, bool connected);
#endif
static void stm32_disconnect(struct usbhost_driver_s *drvr,
                                struct usbhost_hubport_s *hport);

/* Initialization */

static void stm32_portreset(struct stm32_usbhost_s *priv);
static void stm32_host_initialize(struct stm32_usbhost_s *priv);
static void stm32_sw_initialize(struct stm32_usbhost_s *priv);
static int stm32_hw_initialize(struct stm32_usbhost_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Single USB host instance */

static struct stm32_usbhost_s g_usbhost =
{
  .lock = NXMUTEX_INITIALIZER,
  .pscsem = SEM_INITIALIZER(0),
};

/* Connection/enumeration interface */

static struct usbhost_connection_s g_usbconn =
{
  .wait      = stm32_wait,
  .enumerate = stm32_enumerate,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_getle16
 *
 * Description:
 *   Get a 16-bit value from a little-endian byte stream
 *
 ****************************************************************************/

static inline uint16_t stm32_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: stm32_pma_write
 *
 * Description:
 *   Write data to the Packet Memory Area (PMA)
 *
 ****************************************************************************/

static void stm32_pma_write(const uint8_t *buffer, uint16_t pmaaddr,
                               uint16_t nbytes)
{
  volatile uint32_t *pdwval;
  uint32_t count;
  uint32_t remaining;

  pdwval = (volatile uint32_t *)(STM32H5_USBDRD_PMA_BASE + pmaaddr);
  count = nbytes >> 2;          /* Number of 32-bit words */
  remaining = nbytes & 0x03;    /* Remaining bytes */

  /* Write full 32-bit words */

  while (count > 0)
    {
      *pdwval++ = (uint32_t)buffer[0] |
                  ((uint32_t)buffer[1] << 8) |
                  ((uint32_t)buffer[2] << 16) |
                  ((uint32_t)buffer[3] << 24);
      buffer += 4;
      count--;
    }

  /* Write remaining bytes */

  if (remaining > 0)
    {
      uint32_t val = 0;
      for (uint32_t i = 0; i < remaining; i++)
        {
          val |= ((uint32_t)buffer[i] << (8 * i));
        }

      *pdwval = val;
    }
}

/****************************************************************************
 * Name: stm32_pma_read
 *
 * Description:
 *   Read data from the Packet Memory Area (PMA)
 *
 ****************************************************************************/

static void stm32_pma_read(uint8_t *buffer, uint16_t pmaaddr,
                              uint16_t nbytes)
{
  volatile uint32_t *pdwval;
  uint32_t count;
  uint32_t remaining;
  uint32_t val;

  /* Memory barrier to ensure USB peripheral writes are visible to CPU */

  UP_DSB();

  pdwval = (volatile uint32_t *)(STM32H5_USBDRD_PMA_BASE + pmaaddr);
  count = nbytes >> 2;
  remaining = nbytes & 0x03;

  /* Read full 32-bit words */

  while (count > 0)
    {
      *(uint32_t *)buffer = *pdwval++;
      buffer += 4;
      count--;
    }

  /* Read remaining bytes */

  if (remaining > 0)
    {
      val = *pdwval;
      for (uint32_t i = 0; i < remaining; i++)
        {
          *buffer++ = (uint8_t)((val >> (8 * i)) & 0xff);
        }
    }
}

/****************************************************************************
 * Name: stm32_pma_alloc_buffer
 *
 * Description:
 *   Allocate a PMA buffer using bitmap allocation. Returns buffer number
 *   on success, negative error code on failure.
 *
 ****************************************************************************/

static int stm32_pma_alloc_buffer(struct stm32_usbhost_s *priv)
{
  irqstate_t flags;
  int bufno = -ENOMEM;
  int bufndx;

  flags = enter_critical_section();

  for (bufndx = 0; bufndx < STM32H5_PMA_NBUFFERS; bufndx++)
    {
      uint32_t bit = STM32H5_PMA_BUFFER_BIT(bufndx);
      if ((priv->pma_bufavail & bit) != 0)
        {
          priv->pma_bufavail &= ~bit;  /* Mark allocated */
          bufno = bufndx;
          break;
        }
    }

  leave_critical_section(flags);

  if (bufno >= 0)
    {
      uinfo("PMA buffer allocated: bufno=%d addr=0x%04x\n",
            bufno, STM32H5_PMA_BUFNO2ADDR(bufno));
    }
  else
    {
      uerr("ERROR: PMA buffer allocation failed, all %d buffers in use\n",
           STM32H5_PMA_NBUFFERS);
    }

  return bufno;
}

/****************************************************************************
 * Name: stm32_pma_free_buffer
 *
 * Description:
 *   Free a PMA buffer by buffer number.
 *
 ****************************************************************************/

static void stm32_pma_free_buffer(struct stm32_usbhost_s *priv,
                                   uint8_t bufno)
{
  irqstate_t flags;

  DEBUGASSERT(bufno < STM32H5_PMA_NBUFFERS);

  flags = enter_critical_section();
  priv->pma_bufavail |= STM32H5_PMA_BUFFER_BIT(bufno);  /* Mark available */
  leave_critical_section(flags);

  uinfo("PMA buffer freed: bufno=%d addr=0x%04x\n",
        bufno, STM32H5_PMA_BUFNO2ADDR(bufno));
}

/****************************************************************************
 * Name: stm32_chan_alloc
 *
 * Description:
 *   Allocate a host channel
 *
 ****************************************************************************/

static int stm32_chan_alloc(struct stm32_usbhost_s *priv)
{
  int chidx;

  for (chidx = 0; chidx < STM32H5_NHOST_CHANNELS; chidx++)
    {
      if (!priv->chan[chidx].inuse)
        {
          int bufno = stm32_pma_alloc_buffer(priv);
          if (bufno < 0)
            {
              return -ENOMEM;
            }

          priv->chan[chidx].inuse = true;
          priv->chan[chidx].pmabufno = (uint8_t)bufno;
          priv->chan[chidx].pmaaddr = STM32H5_PMA_BUFNO2ADDR(bufno);
          uinfo("Channel allocated: chidx=%d\n", chidx);
          return chidx;
        }
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: stm32_chan_free
 *
 * Description:
 *   Free a host channel
 *
 ****************************************************************************/

static inline void stm32_chan_free(struct stm32_usbhost_s *priv,
                                      int chidx)
{
  struct stm32_chan_s *chan;

  DEBUGASSERT((unsigned)chidx < STM32H5_NHOST_CHANNELS);

  chan = &priv->chan[chidx];

  /* Free PMA buffer if allocated */

  if (chan->pmabufno != STM32H5_PMA_BUFFER_NONE)
    {
      stm32_set_chep_rx_status(priv, chidx, USB_CHEP_RX_STRX_DIS);
      stm32_set_chep_tx_status(priv, chidx, USB_CHEP_TX_STTX_DIS);
      stm32_pma_free_buffer(priv, chan->pmabufno);
      chan->pmabufno = STM32H5_PMA_BUFFER_NONE;
      chan->pmaaddr = 0;
      uinfo("Channel freed: chidx=%d\n", chidx);
    }

  chan->inuse = false;
}

/****************************************************************************
 * Name: stm32_set_chep_tx_status
 *
 * Description:
 *   Sets the status of a channel's TX endpoint by toggling the
 *   appropriate DTOG bits.
 *
 ****************************************************************************/

static void stm32_set_chep_tx_status(struct stm32_usbhost_s *priv,
                                     int chidx, uint32_t status)
{
  uint32_t regval;

  /* Status changes work by toggling the DTOG bits */

  regval = stm32_getreg(STM32H5_USB_CHEP(priv->chan[chidx].chidx))
                        & USB_CHEP_TX_DTOGMASK;
  if (status & USB_CHEP_TX_DTOG1)
    {
      regval ^= USB_CHEP_TX_DTOG1;
    }

  if (status & USB_CHEP_TX_DTOG2)
    {
      regval ^= USB_CHEP_TX_DTOG2;
    }

  stm32_putreg(STM32H5_USB_CHEP(priv->chan[chidx].chidx),
               regval | USB_CHEP_VTRX | USB_CHEP_VTTX);
}

/****************************************************************************
 * Name: stm32_set_chep_rx_status
 *
 * Description:
 *   Sets the status of a channel's RX endpoint
 *   by toggling the appropriate DTOG bits.
 *
 ****************************************************************************/

static void stm32_set_chep_rx_status(struct stm32_usbhost_s *priv,
                                     int chidx, uint32_t status)
{
  uint32_t regval;

  /* Status changes work by toggling the DTOG bits */

  regval = stm32_getreg(STM32H5_USB_CHEP(priv->chan[chidx].chidx))
                        & USB_CHEP_RX_DTOGMASK;
  if (status & USB_CHEP_RX_DTOG1)
    {
      regval ^= USB_CHEP_RX_DTOG1;
    }

  if (status & USB_CHEP_RX_DTOG2)
    {
      regval ^= USB_CHEP_RX_DTOG2;
    }

  stm32_putreg(STM32H5_USB_CHEP(priv->chan[chidx].chidx),
               regval | USB_CHEP_VTRX | USB_CHEP_VTTX);
}

/****************************************************************************
 * Name: stm32_chan_freeall
 *
 * Description:
 *   Free all channels.
 *
 ****************************************************************************/

static inline void stm32_chan_freeall(struct stm32_usbhost_s *priv)
{
  int chidx;

  /* Free all host channels */

  for (chidx = 0; chidx < STM32H5_NHOST_CHANNELS; chidx++)
    {
      if (priv->chan[chidx].inuse)
        {
          stm32_chan_free(priv, chidx);
        }
    }
}

/****************************************************************************
 * Name: stm32_chan_configure
 *
 * Description:
 *   Configure a host channel for a transfer
 *
 ****************************************************************************/

static void stm32_chan_configure(struct stm32_usbhost_s *priv,
                                    int chidx)
{
  struct stm32_chan_s *chan = &priv->chan[chidx];
  uint32_t regval;
  uint32_t eptype;

  /* Get endpoint type for register */

  switch (chan->eptype)
    {
      case USB_EP_ATTR_XFER_CONTROL:
        eptype = USB_CHEP_UTYPE_CTRL;
        break;
      case USB_EP_ATTR_XFER_BULK:
        eptype = USB_CHEP_UTYPE_BULK;
        break;
      case USB_EP_ATTR_XFER_INT:
        eptype = USB_CHEP_UTYPE_INTR;
        break;
      case USB_EP_ATTR_XFER_ISOC:
        eptype = USB_CHEP_UTYPE_ISOC;
        break;
      default:
        eptype = USB_CHEP_UTYPE_BULK;
        break;
    }

  /* Read current register value and mask toggleable bits */

  regval = stm32_getreg(STM32H5_USB_CHEP(chidx)) & USB_CH_T_MASK;

  /* Set endpoint type */

  regval |= eptype;

  /* Clear and set device address and endpoint number */

  regval &= ~(USB_CHEP_DEVADDR_MASK | USB_CHEP_ADDR_MASK |
              USB_CHEP_LSEP | USB_CHEP_NAK |
              USB_CHEP_KIND | USB_CHEP_ERRTX | USB_CHEP_ERRRX);

  regval |= ((uint32_t)chan->funcaddr << USB_CHEP_DEVADDR_SHIFT);
  regval |= ((uint32_t)chan->epno & 0x0f);

  /* Set low-speed endpoint flag if needed */

  if (chan->speed == USB_SPEED_LOW)
    {
      regval |= USB_CHEP_LSEP;
    }

  /* Write the channel register with VT bits preserved */

  stm32_putreg(STM32H5_USB_CHEP(chidx),
                 regval | USB_CHEP_VTRX | USB_CHEP_VTTX);
}

/****************************************************************************
 * Name: stm32_chan_waitsetup
 *
 * Description:
 *   Set up to wait for transfer completion
 *
 ****************************************************************************/

static int stm32_chan_waitsetup(struct stm32_usbhost_s *priv,
                                   struct stm32_chan_s *chan)
{
  irqstate_t flags = enter_critical_section();

  /* Is the device still connected? */

  if (!priv->connected)
    {
      leave_critical_section(flags);
      return -ENODEV;
    }

  chan->waiter = true;
  chan->callback = NULL;
  chan->result = EBUSY;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: stm32_chan_wait
 *
 * Description:
 *   Wait for transfer completion
 *
 ****************************************************************************/

static int stm32_chan_wait(struct stm32_usbhost_s *priv,
                              struct stm32_chan_s *chan,
                              int timeout_ms)
{
  irqstate_t flags;
  int ret;

  struct timespec abstime;
  clock_gettime(CLOCK_MONOTONIC, &abstime);
  abstime.tv_sec += timeout_ms / 1000;
  abstime.tv_nsec += (timeout_ms % 1000) * 1000000;
  if (abstime.tv_nsec >= 1000000000)
    {
      abstime.tv_sec += 1;
      abstime.tv_nsec -= 1000000000;
    }

  flags = enter_critical_section();

  while (chan->waiter)
    {
      ret = nxsem_clockwait(&chan->waitsem, CLOCK_MONOTONIC, &abstime);
      if (ret < 0)
        {
          /* Cancel the transfer if still active */

          if (chan->in)
            {
              stm32_set_chep_rx_status(priv, chan->chidx,
                                       USB_CHEP_RX_STRX_STALL);
            }
          else
            {
              stm32_set_chep_tx_status(priv, chan->chidx,
                                       USB_CHEP_TX_STTX_STALL);
            }

          leave_critical_section(flags);
          return ret;
        }
    }

  ret = -(int)chan->result;
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: stm32_chan_wakeup
 *
 * Description:
 *   Wake up a waiting thread
 *
 ****************************************************************************/

static void stm32_chan_wakeup(struct stm32_usbhost_s *priv,
                                 struct stm32_chan_s *chan)
{
  if (chan->waiter)
    {
      chan->waiter = false;
      nxsem_post(&chan->waitsem);
    }

#ifdef CONFIG_USBHOST_ASYNCH
  else if (chan->callback)
    {
      usbhost_asynch_t callback = chan->callback;
      void *arg = chan->arg;
      int nbytes = chan->xfrd;
      int result = chan->result;

      chan->callback = NULL;
      chan->arg = NULL;

      if (result != OK)
        {
          nbytes = -(int)result;
        }

      callback(arg, nbytes);
    }
  else
    {
      uerr("ERROR: No waiter or callback for channel %d\n", chan->chidx);
    }
#endif
}

/****************************************************************************
 * Name: stm32_ctrlchan_alloc
 *
 * Description:
 *   Allocate channels for control endpoint (IN and OUT)
 *
 ****************************************************************************/

static int stm32_ctrlchan_alloc(struct stm32_usbhost_s *priv,
                                   uint8_t epno, uint8_t funcaddr,
                                   uint8_t speed,
                                   struct stm32_ctrlinfo_s *ctrlep)
{
  struct stm32_chan_s *chan;
  int inndx;
  int outndx;

  /* Allocate IN channel */

  inndx = stm32_chan_alloc(priv);
  if (inndx < 0)
    {
      return -ENOMEM;
    }

  /* Allocate OUT channel */

  outndx = stm32_chan_alloc(priv);
  if (outndx < 0)
    {
      stm32_chan_free(priv, inndx);
      return -ENOMEM;
    }

  /* Configure IN channel */

  chan = &priv->chan[inndx];
  chan->epno = epno;
  chan->in = true;
  chan->eptype = USB_EP_ATTR_XFER_CONTROL;
  chan->funcaddr = funcaddr;
  chan->speed = speed;
  chan->maxpacket = STM32H5_EP0_MAX_PACKET_SIZE;
  chan->indata1 = false;
  chan->outdata1 = false;

  /* Configure OUT channel */

  chan = &priv->chan[outndx];
  chan->epno = epno;
  chan->in = false;
  chan->eptype = USB_EP_ATTR_XFER_CONTROL;
  chan->funcaddr = funcaddr;
  chan->speed = speed;
  chan->maxpacket = STM32H5_EP0_MAX_PACKET_SIZE;
  chan->indata1 = false;
  chan->outdata1 = false;

  /* Configure channels */

  stm32_chan_configure(priv, inndx);
  stm32_chan_configure(priv, outndx);

  ctrlep->inndx = inndx;
  ctrlep->outndx = outndx;

  return OK;
}

/****************************************************************************
 * Name: stm32_ctrlep_alloc
 *
 * Description:
 *   Allocate a control endpoint
 *
 ****************************************************************************/

static int stm32_ctrlep_alloc(struct stm32_usbhost_s *priv,
                                 const struct usbhost_epdesc_s *epdesc,
                                 usbhost_ep_t *ep)
{
  struct stm32_ctrlinfo_s *ctrlep;
  int ret;

  ctrlep = kmm_malloc(sizeof(struct stm32_ctrlinfo_s));
  if (!ctrlep)
    {
      return -ENOMEM;
    }

  ret = stm32_ctrlchan_alloc(priv, 0, epdesc->hport->funcaddr,
                                epdesc->hport->speed, ctrlep);
  if (ret < 0)
    {
      kmm_free(ctrlep);
      return ret;
    }

  *ep = (usbhost_ep_t)ctrlep;
  return OK;
}

/****************************************************************************
 * Name: stm32_xfrep_alloc
 *
 * Description:
 *   Allocate a bulk/interrupt endpoint
 *
 ****************************************************************************/

static int stm32_xfrep_alloc(struct stm32_usbhost_s *priv,
                                const struct usbhost_epdesc_s *epdesc,
                                usbhost_ep_t *ep)
{
  struct stm32_chan_s *chan;
  int chidx;

  chidx = stm32_chan_alloc(priv);
  if (chidx < 0)
    {
      return -ENOMEM;
    }

  chan = &priv->chan[chidx];
  chan->epno = epdesc->addr & USB_EPNO_MASK;
  chan->in = epdesc->in;
  chan->eptype = epdesc->xfrtype;
  chan->funcaddr = epdesc->hport->funcaddr;
  chan->speed = epdesc->hport->speed;
  chan->interval = epdesc->interval;
  chan->maxpacket = epdesc->mxpacketsize;
  chan->indata1 = false;
  chan->outdata1 = false;

  /* Clamp all endpoints to max 64 bytes. To support larger packets,
   * additional PMA logic is required
   */

  if (chan->maxpacket > 64)
    {
      chan->maxpacket = 64;
    }

  stm32_chan_configure(priv, chidx);

  *ep = (usbhost_ep_t)chidx;
  return OK;
}

/****************************************************************************
 * Name: stm32_transfer_start
 *
 * Description:
 *   Start a USB transfer
 *
 ****************************************************************************/

static void stm32_transfer_start(struct stm32_usbhost_s *priv,
                                    int chidx)
{
  struct stm32_chan_s *chan = &priv->chan[chidx];
  uint32_t regval;
  uint16_t len;
  uint32_t bdval;
  volatile uint32_t *pbd;

  len = chan->buflen > chan->maxpacket ? chan->maxpacket : chan->buflen;

  if (chan->in)
    {
      /* Set up RX buffer in PMA */

      bdval = chan->pmaaddr;  /* chan->pmaaddr is already 4 byte aligned */

      if (len > 62)
        {
          /* BL_SIZE = 1, NUM_BLOCK = (len / 32) - 1 */

          uint32_t nblocks = ((len + 31) / 32);
          bdval |= USB_PMA_RXBD_BLSIZE |
                   ((nblocks - 1) << USB_PMA_RXBD_NUM_BLOCK_SHIFT);
        }
      else
        {
          /* BL_SIZE = 0, NUM_BLOCK = len / 2 */

          uint32_t nblocks = (len + 1) / 2;
          bdval |= (nblocks << USB_PMA_RXBD_NUM_BLOCK_SHIFT);
        }

      pbd = (volatile uint32_t *)(STM32H5_USBDRD_PMA_BASE +
                                  USB_PMA_RXBD_OFFSET(chidx));
      *pbd = bdval;

      /* Memory barrier to ensure BD write completes
       * before enabling transfer
       */

      UP_DSB();

      /* Clear data toggle if starting new transfer */

      regval = stm32_getreg(STM32H5_USB_CHEP(chidx));
      if ((regval & USB_CHEP_DTOG_RX) != 0)
        {
          if (!chan->indata1)
            {
              /* Need DATA0, but DTOG shows DATA1 - toggle it */

              regval = (regval & USB_CHEP_REG_MASK) |
                       USB_CHEP_VTRX | USB_CHEP_VTTX | USB_CHEP_DTOG_RX;
              stm32_putreg(STM32H5_USB_CHEP(chidx), regval);
            }
        }
      else
        {
          if (chan->indata1)
            {
              /* Need DATA1, but DTOG shows DATA0 - toggle it */

              regval = (regval & USB_CHEP_REG_MASK) |
                       USB_CHEP_VTRX | USB_CHEP_VTTX | USB_CHEP_DTOG_RX;
              stm32_putreg(STM32H5_USB_CHEP(chidx), regval);
            }
        }

      /* Enable RX */

      stm32_set_chep_rx_status(priv, chan->chidx, USB_CHEP_RX_STRX_VALID);
    }
  else
    {
      /* OUT transfer - write data to PMA and set TX count */

      if (len > 0)
        {
          stm32_pma_write(chan->buffer, chan->pmaaddr, len);
        }

      /* Update TX count in buffer descriptor */

      bdval = chan->pmaaddr;  /* chan->pmaaddr is already 4 byte aligned */
      bdval |= ((uint32_t)len << USB_PMA_TXBD_COUNT_SHIFT);

      pbd = (volatile uint32_t *)(STM32H5_USBDRD_PMA_BASE +
                                  USB_PMA_TXBD_OFFSET(chidx));
      *pbd = bdval;

      /* Handle SETUP token for control transfers */

      if (chan->pid == HC_PID_SETUP)
        {
          regval = stm32_getreg(STM32H5_USB_CHEP(chidx)) & USB_CHEP_REG_MASK;
          stm32_putreg(STM32H5_USB_CHEP(chidx),
                         regval | USB_CHEP_SETUP |
                         USB_CHEP_VTRX | USB_CHEP_VTTX);
        }

      /* Sync data toggle (only needed at transfer start,
       * hardware auto-toggles after successful transmit)
       */

      regval = stm32_getreg(STM32H5_USB_CHEP(chidx));
      if ((regval & USB_CHEP_DTOG_TX) != 0)
        {
          if (!chan->outdata1)
            {
              /* Need DATA0, toggle it */

              regval = (regval & USB_CHEP_REG_MASK) |
                       USB_CHEP_VTRX | USB_CHEP_VTTX | USB_CHEP_DTOG_TX;
              stm32_putreg(STM32H5_USB_CHEP(chidx), regval);
            }
        }
      else
        {
          if (chan->outdata1)
            {
              /* Need DATA1, toggle it */

              regval = (regval & USB_CHEP_REG_MASK) |
                       USB_CHEP_VTRX | USB_CHEP_VTTX | USB_CHEP_DTOG_TX;
              stm32_putreg(STM32H5_USB_CHEP(chidx), regval);
            }
        }

      /* Enable TX */

      stm32_set_chep_tx_status(priv, chan->chidx, USB_CHEP_TX_STTX_VALID);
    }
}

/****************************************************************************
 * Name: stm32_ctrl_sendsetup
 *
 * Description:
 *   Send a SETUP packet
 *
 ****************************************************************************/

static int stm32_ctrl_sendsetup(struct stm32_usbhost_s *priv,
                                   struct stm32_ctrlinfo_s *ep0,
                                   const struct usb_ctrlreq_s *req)
{
  struct stm32_chan_s *chan;
  int ret;

  chan = &priv->chan[ep0->outndx];

  /* Set up the transfer */

  chan->buffer = (uint8_t *)req;
  chan->buflen = USB_SIZEOF_CTRLREQ;
  chan->pid = HC_PID_SETUP;
  chan->outdata1 = false;

  ret = stm32_chan_waitsetup(priv, chan);
  if (ret < 0)
    {
      return ret;
    }

  stm32_transfer_start(priv, ep0->outndx);
  return stm32_chan_wait(priv, chan, CONFIG_STM32H5_USBDRD_TRANSFER_TIMEOUT);
}

/****************************************************************************
 * Name: stm32_ctrl_senddata
 *
 * Description:
 *   Send control data
 *
 ****************************************************************************/

static int stm32_ctrl_senddata(struct stm32_usbhost_s *priv,
                                  struct stm32_ctrlinfo_s *ep0,
                                  uint8_t *buffer, unsigned int buflen)
{
  struct stm32_chan_s *chan;
  int ret;

  chan = &priv->chan[ep0->outndx];

  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->pid = HC_PID_DATA1;
  chan->outdata1 = true;

  ret = stm32_chan_waitsetup(priv, chan);
  if (ret < 0)
    {
      return ret;
    }

  stm32_transfer_start(priv, ep0->outndx);
  return stm32_chan_wait(priv, chan, CONFIG_STM32H5_USBDRD_TRANSFER_TIMEOUT);
}

/****************************************************************************
 * Name: stm32_ctrl_recvdata
 *
 * Description:
 *   Receive control data
 *
 ****************************************************************************/

static int stm32_ctrl_recvdata(struct stm32_usbhost_s *priv,
                                  struct stm32_ctrlinfo_s *ep0,
                                  uint8_t *buffer, unsigned int buflen)
{
  struct stm32_chan_s *chan;
  int ret;

  chan = &priv->chan[ep0->inndx];

  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->pid = HC_PID_DATA1;
  chan->indata1 = true;

  ret = stm32_chan_waitsetup(priv, chan);
  if (ret < 0)
    {
      return ret;
    }

  stm32_transfer_start(priv, ep0->inndx);
  return stm32_chan_wait(priv, chan, CONFIG_STM32H5_USBDRD_TRANSFER_TIMEOUT);
}

/****************************************************************************
 * Name: stm32_in_transfer
 *
 * Description:
 *   Perform an IN transfer
 *
 ****************************************************************************/

static ssize_t stm32_in_transfer(struct stm32_usbhost_s *priv,
                                    int chidx, uint8_t *buffer,
                                    size_t buflen)
{
  struct stm32_chan_s *chan = &priv->chan[chidx];
  int ret;

  /* Set up for ENTIRE transfer (IRQ will handle multi-packet) */

  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd = 0;

  ret = stm32_chan_waitsetup(priv, chan);
  if (ret < 0)
    {
      return ret;
    }

  stm32_transfer_start(priv, chidx);

  ret = stm32_chan_wait(priv, chan, CONFIG_STM32H5_USBDRD_TRANSFER_TIMEOUT);
  if (ret < 0)
    {
      return ret;
    }

  return chan->xfrd;
}

/****************************************************************************
 * Name: stm32_out_transfer
 *
 * Description:
 *   Perform an OUT transfer
 *
 ****************************************************************************/

static ssize_t stm32_out_transfer(struct stm32_usbhost_s *priv,
                                     int chidx, uint8_t *buffer,
                                     size_t buflen)
{
  struct stm32_chan_s *chan = &priv->chan[chidx];
  int ret;

  /* Set up for ENTIRE transfer (IRQ will handle multi-packet) */

  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd = 0;

  ret = stm32_chan_waitsetup(priv, chan);
  if (ret < 0)
    {
      return ret;
    }

  stm32_transfer_start(priv, chidx);

  ret = stm32_chan_wait(priv, chan, CONFIG_STM32H5_USBDRD_TRANSFER_TIMEOUT);
  if (ret < 0)
    {
      return ret;
    }

  return chan->xfrd;
}

/****************************************************************************
 * Name: stm32_gint_connected
 *
 * Description:
 *   Handle device connection
 *
 ****************************************************************************/

static void stm32_gint_connected(struct stm32_usbhost_s *priv)
{
  /* Were we previously disconnected? */

  if (!priv->connected)
    {
      /* Yes.. then now we are connected */

      uinfo("USB device connected\n");
      priv->connected = true;
      priv->change    = true;
      DEBUGASSERT(priv->smstate == SMSTATE_DETACHED);

      /* Notify any waiters */

      priv->smstate = SMSTATE_ATTACHED;
      if (priv->pscwait)
        {
          nxsem_post(&priv->pscsem);
          priv->pscwait = false;
        }
    }
}

/****************************************************************************
 * Name: stm32_gint_disconnected
 *
 * Description:
 *   Handle device disconnection
 *
 ****************************************************************************/

static void stm32_gint_disconnected(struct stm32_usbhost_s *priv)
{
  /* Were we previously connected? */

  if (priv->connected)
    {
      /* Yes.. then we no longer connected */

      uinfo("USB device disconnected\n");

      /* Are we bound to a class driver? */

      if (priv->rhport.hport.devclass)
        {
          /* Yes.. Disconnect the class driver */

          CLASS_DISCONNECTED(priv->rhport.hport.devclass);
          priv->rhport.hport.devclass = NULL;
        }

      /* Re-Initialize Host for new Enumeration */

      priv->smstate   = SMSTATE_DETACHED;
      priv->connected = false;
      priv->change    = true;
      stm32_chan_freeall(priv);

      priv->rhport.hport.speed = USB_SPEED_FULL;
      priv->rhport.hport.funcaddr = 0;

      /* Notify any waiters that there is a change in the connection state */

      if (priv->pscwait)
        {
          nxsem_post(&priv->pscsem);
          priv->pscwait = false;
        }
    }
}

/****************************************************************************
 * Name: stm32_hc_in_irq
 *
 * Description:
 *   Handle IN channel interrupt (device-to-host transfers).
 *   Based on ST HAL HCD_HC_IN_IRQHandler pattern.
 *
 ****************************************************************************/

static void stm32_hc_in_irq(struct stm32_usbhost_s *priv, int chidx)
{
  struct stm32_chan_s *chan = &priv->chan[chidx];
  uint32_t chepval = stm32_getreg(STM32H5_USB_CHEP(chidx));
  uint32_t rx_status = chepval & USB_CHEP_RX_STRX_MASK;
  bool wakeup = false;

  if ((chepval & USB_CHEP_ERRRX) != 0)
    {
      chepval = stm32_getreg(STM32H5_USB_CHEP(chidx));
      uerr("ERRRX chidx=%d chepval=0x%08x rx_status=%d nak=%d\n",
           chidx, (unsigned int)chepval,
           (int)((chepval & USB_CHEP_RX_STRX_MASK) >>
           USB_CHEP_RX_STRX_SHIFT),
           (int)((chepval & USB_CHEP_NAK) != 0));

      /* Clear ERRRX (write 0) and VTRX (write 0) to acknowledge the CTR
       * interrupt. Preserve VTTX by writing 1.
       */

      chepval = (chepval & (0xffff7fff & USB_CHEP_REG_MASK) &
                 ~USB_CHEP_ERRRX) | USB_CHEP_VTTX;
      stm32_putreg(STM32H5_USB_CHEP(chidx), chepval);

      chan->result = EIO;
      chan->chreason = CHREASON_TXERR;
      stm32_chan_wakeup(priv, chan);
      return;
    }

  /* Check RX status for ACK/NAK/STALL
   * After successful transfer, status becomes USB_CHEP_RX_STRX_DIS (0x0)
   * meaning "ACK received, channel disabled".
   */

  if (rx_status == USB_CHEP_RX_STRX_DIS)
    {
      /* ACK - successful transfer, read data from PMA */

      volatile uint32_t *pbd;
      uint16_t count;
      bool transfer_complete;

      pbd = (volatile uint32_t *)(STM32H5_USBDRD_PMA_BASE +
                                  USB_PMA_RXBD_OFFSET(chidx));
      count = (*pbd >> USB_PMA_RXBD_COUNT_SHIFT) & 0x3ff;

      /* Clip to remaining buffer space */

      if (count > chan->buflen)
        {
          count = chan->buflen;
        }

      /* Read data from PMA using the address from the BD - this ensures we
       * read from wherever the hardware actually wrote the data.
       */

      if (count > 0 && chan->buffer != NULL)
        {
          stm32_pma_read(chan->buffer, chan->pmaaddr, count);
        }

      /* Accumulate bytes, advance buffer, reduce remaining */

      chan->xfrd += count;
      chan->buffer += count;
      chan->buflen -= count;

      /* Toggle DATA0/DATA1 for this packet */

      chan->indata1 ^= true;

      /* Check if transfer is complete:
       * - All requested bytes received (buflen == 0)
       * - Short packet received (count < maxpacket)
       * - Zero-length packet received (count == 0)
       */

      transfer_complete = (chan->buflen == 0) ||
                          (count < chan->maxpacket) ||
                          (count == 0);

      if (!transfer_complete && (chan->waiter || chan->callback))
        {
          /* Clear VTRX by writing 0 to it
           * (toggle bit, write 1 keeps, write 0 clears)
           */

          chepval = stm32_getreg(STM32H5_USB_CHEP(chidx));
          chepval = (chepval &
                    (0xffff7fff & USB_CHEP_REG_MASK)) | USB_CHEP_VTTX;
          stm32_putreg(STM32H5_USB_CHEP(chidx), chepval);

          /* More data expected - reactivate channel for next packet */

          stm32_transfer_start(priv, chidx);
          return;
        }
      else
        {
          /* Transfer complete - set success result */

          chan->result = OK;
          chan->chreason = CHREASON_XFRC;
          wakeup = true;
        }
    }
  else if (rx_status == USB_CHEP_RX_STRX_NAK)
    {
      /* NAK is normal flow control
       * If we have a waiter, just let hardware keep retrying indefinitely.
       * We will timeout in stm32_chan_wait
       * and cancel if the device never answers
       */

      if (!chan->waiter && !chan->callback)
        {
          /* No waiters on this. Cancel the transaction */

          stm32_set_chep_rx_status(priv, chan->chidx,
                                   USB_CHEP_RX_STRX_STALL);
        }
    }
  else if (rx_status == USB_CHEP_RX_STRX_STALL)
    {
      /* STALL - endpoint halted, disable channel */

      chan->result = EPERM;
      chan->chreason = CHREASON_STALL;

      /* Disable the channel */

      stm32_set_chep_rx_status(priv, chan->chidx, USB_CHEP_RX_STRX_DIS);
      wakeup = true;
    }

  /* Clear VTRX by writing 0 to it
   * (toggle bit, write 1 keeps, write 0 clears)
   */

  chepval = stm32_getreg(STM32H5_USB_CHEP(chidx));
  chepval = (chepval & (0xffff7fff & USB_CHEP_REG_MASK)) | USB_CHEP_VTTX;
  stm32_putreg(STM32H5_USB_CHEP(chidx), chepval);

  if (wakeup)
    {
      stm32_chan_wakeup(priv, chan);
    }
}

/****************************************************************************
 * Name: stm32_hc_out_irq
 *
 * Description:
 *   Handle OUT channel interrupt (host-to-device transfers including SETUP).
 *   Based on ST HAL HCD_HC_OUT_IRQHandler pattern.
 *
 ****************************************************************************/

static void stm32_hc_out_irq(struct stm32_usbhost_s *priv, int chidx)
{
  struct stm32_chan_s *chan = &priv->chan[chidx];
  uint32_t chepval = stm32_getreg(STM32H5_USB_CHEP(chidx));
  uint32_t tx_status = chepval & USB_CHEP_TX_STTX_MASK;
  bool wakeup = false;

  if ((chepval & USB_CHEP_ERRTX) != 0)
    {
      /* Clear error bit: write 0 to ERRTX to clear,
       * write 1 to VTRX/VTTX to preserve
       */

      chepval = stm32_getreg(STM32H5_USB_CHEP(chidx));
      chepval = (chepval & USB_CHEP_REG_MASK & ~USB_CHEP_ERRTX) |
                USB_CHEP_VTRX | USB_CHEP_VTTX;
      stm32_putreg(STM32H5_USB_CHEP(chidx), chepval);

      chan->result = EIO;
      chan->chreason = CHREASON_TXERR;
      stm32_chan_wakeup(priv, chan);
      return;
    }

  /* Check TX status for ACK/NAK/STALL
   * After successful transfer, status becomes USB_CHEP_TX_STTX_DIS (0x0)
   * meaning "ACK received, channel disabled".
   */

  if (tx_status == USB_CHEP_TX_STTX_DIS)
    {
      /* ACK - successful transfer of one packet */

      uint16_t sent_count;

      /* Calculate how much was sent (capped at maxpacket) */

      sent_count = (chan->buflen > chan->maxpacket) ?
                    chan->maxpacket : chan->buflen;

      /* Accumulate bytes, advance buffer, reduce remaining */

      chan->xfrd += sent_count;
      chan->buffer += sent_count;
      chan->buflen -= sent_count;

      /* Toggle DATA0/DATA1 */

      chan->outdata1 ^= true;

      /* Check if more data to send */

      if (chan->buflen > 0 && (chan->waiter || chan->callback))
        {
          /* More data to send.
           * prepare and restart channel without waking thread
           */

          stm32_transfer_start(priv, chidx);
        }
      else
        {
          /* All data sent - mark complete */

          chan->result = OK;
          chan->chreason = CHREASON_XFRC;
          wakeup = true;
        }
    }
  else if (((chepval & USB_CHEP_NAK) == USB_CHEP_NAK) ||
           (tx_status == USB_CHEP_TX_STTX_NAK))
    {
      /* NAK is normal flow control
       * If we have a waiter, just let hardware keep retrying indefinitely.
       * We will timeout in stm32_chan_wait and cancel
       * if the device never answers
       */

      /* Clear NAK if it was set */

      if ((chepval & USB_CHEP_NAK) != 0)
        {
          chepval = stm32_getreg(STM32H5_USB_CHEP(chidx));
          chepval = (chepval & USB_CHEP_REG_MASK & ~USB_CHEP_NAK) |
                    USB_CHEP_VTRX | USB_CHEP_VTTX;
          stm32_putreg(STM32H5_USB_CHEP(chidx), chepval);
        }

      if (!chan->waiter && !chan->callback)
        {
          /* No waiters on this. Cancel the transaction */

          stm32_set_chep_tx_status(priv, chan->chidx,
                                   USB_CHEP_TX_STTX_STALL);
        }
    }
  else if (tx_status == USB_CHEP_TX_STTX_STALL)
    {
      /* STALL - endpoint halted, disable channel */

      chan->result = EPERM;
      chan->chreason = CHREASON_STALL;
      wakeup = true;

      /* Disable the channel */

      stm32_set_chep_tx_status(priv, chan->chidx, USB_CHEP_TX_STTX_DIS);
    }

  /* Clear VTTX by writing 0 to it */

  chepval = stm32_getreg(STM32H5_USB_CHEP(chidx));
  chepval = (chepval & (0xffffff7f & USB_CHEP_REG_MASK)) | USB_CHEP_VTRX;
  stm32_putreg(STM32H5_USB_CHEP(chidx), chepval);

  if (wakeup)
    {
      stm32_chan_wakeup(priv, chan);
    }
}

/****************************************************************************
 * Name: stm32_usbdrd_interrupt
 *
 * Description:
 *   USB DRD interrupt handler.
 *   Dispatches channel transfer interrupts to IN/OUT handlers based on
 *   direction, following ST HAL pattern.
 *
 ****************************************************************************/

static int stm32_usbdrd_interrupt(int irq, void *context, void *arg)
{
  struct stm32_usbhost_s *priv = &g_usbhost;
  uint32_t istr;

  istr = stm32_getreg(STM32_USB_ISTR);

  /* Device connection/Disconnection. The STM32H5 has both USB_ISTR_DDISC
   * and USB_ISTR_RESET to detect a connected device. Use USB_ISTR_RESET
   * since it is signaled after 22 cycles (debounced)
   */

  if ((istr & USB_ISTR_RESET) != 0)
    {
      if ((istr & USB_ISTR_DCON_STAT) != 0)
        {
          stm32_gint_connected(priv);
        }
      else
        {
          stm32_gint_disconnected(priv);
        }

      stm32_putreg(STM32_USB_ISTR, ~USB_ISTR_RESET);
    }

  /* Device connection */

  if ((istr & USB_ISTR_DDISC) != 0)
    {
      /* Not used. USB_ISTR_RESET used instead */

      stm32_putreg(STM32_USB_ISTR, ~USB_ISTR_DDISC);
    }

  /* Correct transfer - dispatch to IN or OUT handler based on direction */

  if ((istr & USB_ISTR_CTR) != 0)
    {
      int chidx = istr & USB_ISTR_EPID_MASK;
      bool is_in = (istr & USB_ISTR_DIR) != 0;

      if (is_in)
        {
          stm32_hc_in_irq(priv, chidx);
        }
      else
        {
          /* STM32H562xx/563xx/573xx Errata:
           * During OUT transfers, the correct transfer interrupt (CTR) is
           * triggered a little before the last USB SRAM accesses have
           * completed. If the software responds quickly to the interrupt,
           * the full buffer contents may not be correct.
           * Software should ensure that a small delay is included before
           * accessing the SRAM contents. This delay should be 800 ns in
           * Full Speed mode and 6.4 μs in Low Speed mode.
           */

          up_udelay(1);

          stm32_hc_out_irq(priv, chidx);
        }
    }

  /* Error */

  if ((istr & USB_ISTR_ERR) != 0)
    {
      uerr("ERROR: USB error\n");
      stm32_putreg(STM32_USB_ISTR, ~USB_ISTR_ERR);
    }

  /* PMA overrun */

  if ((istr & USB_ISTR_PMAOVRN) != 0)
    {
      uerr("ERROR: PMA overrun\n");
      stm32_putreg(STM32_USB_ISTR, ~USB_ISTR_PMAOVRN);
    }

  /* Other interrupts - not used, just clear */

  if ((istr & USB_ISTR_WKUP) != 0)
    {
      stm32_putreg(STM32_USB_ISTR, ~USB_ISTR_WKUP);
    }

  if ((istr & USB_ISTR_SUSP) != 0)
    {
      stm32_putreg(STM32_USB_ISTR, ~USB_ISTR_SUSP);
    }

  if ((istr & USB_ISTR_SOF) != 0)
    {
      stm32_putreg(STM32_USB_ISTR, ~USB_ISTR_SOF);
    }

  if ((istr & USB_ISTR_ESOF) != 0)
    {
      stm32_putreg(STM32_USB_ISTR, ~USB_ISTR_ESOF);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_wait
 *
 * Description:
 *   Wait for device connection/disconnection
 *
 ****************************************************************************/

static int stm32_wait(struct usbhost_connection_s *conn,
                         struct usbhost_hubport_s **hport)
{
  struct stm32_usbhost_s *priv = &g_usbhost;
  struct usbhost_hubport_s *connport;
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();

  for (; ; )
    {
      if (priv->change)
        {
          priv->change = false;

          connport = &priv->rhport.hport;
          connport->connected = priv->connected;

          *hport = connport;

          leave_critical_section(flags);

          uinfo("RHport Connected: %s\n",
                connport->connected ? "YES" : "NO");
          return OK;
        }

#ifdef CONFIG_USBHOST_HUB
      /* Is a device connected to an external hub? */

      if (priv->hport)
        {
          /* Yes.. return the external hub port */

          connport = (struct usbhost_hubport_s *)priv->hport;
          priv->hport = NULL;

          *hport = connport;
          leave_critical_section(flags);

          uinfo("Hub port Connected: %s\n",
                connport->connected ? "YES" : "NO");
          return OK;
        }
#endif

      priv->pscwait = true;
      ret = nxsem_wait(&priv->pscsem);
      if (ret < 0)
        {
          leave_critical_section(flags);
          return ret;
        }
    }
}

/****************************************************************************
 * Name: stm32_rh_enumerate
 *
 * Description:
 *   Enumerate the root hub
 *
 ****************************************************************************/

static int stm32_rh_enumerate(struct stm32_usbhost_s *priv,
                                 struct usbhost_connection_s *conn,
                                 struct usbhost_hubport_s *hport)
{
  uint32_t istr;
  int ret;

  DEBUGASSERT(hport != NULL && hport->port == 0);

  /* Are we connected? */

  if (!priv->connected)
    {
      return -ENODEV;
    }

  /* USB 2.0 spec says at least 50ms delay before port reset */

  nxsched_usleep(50 * 1000);

  /* Reset the port */

  stm32_portreset(priv);

  /* Check for low-speed device */

  istr = stm32_getreg(STM32_USB_ISTR);
  if ((istr & USB_ISTR_LS_DCONN) != 0)
    {
      hport->speed = USB_SPEED_LOW;
    }
  else
    {
      hport->speed = USB_SPEED_FULL;
    }

  /* Allocate control endpoint */

  ret = stm32_ctrlchan_alloc(priv, 0, 0, hport->speed, &priv->ep0);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate EP0: %d\n", ret);
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  uinfo("Enumerated device at %s speed\n",
        hport->speed == USB_SPEED_LOW ? "low" : "full");

  return ret;
}

/****************************************************************************
 * Name: stm32_enumerate
 *
 * Description:
 *   Enumerate a connected device
 *
 ****************************************************************************/

static int stm32_enumerate(struct usbhost_connection_s *conn,
                              struct usbhost_hubport_s *hport)
{
  struct stm32_usbhost_s *priv = &g_usbhost;
  int ret;

  DEBUGASSERT(hport != NULL);

  /* If this is the root hub port */

#ifdef CONFIG_USBHOST_HUB
  if (ROOTHUB(hport))
#endif
    {
      ret = stm32_rh_enumerate(priv, conn, hport);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Perform standard enumeration */

  ret = usbhost_enumerate(hport, &hport->devclass);

  /* The enumeration may fail either because of some HCD interfaces failure
   * or because the device class is not supported.  In either case, we just
   * need to perform the disconnection operation and make ready for a new
   * enumeration.
   */

  if (ret < 0)
    {
      /* Return to the disconnected state */

      uerr("ERROR: Enumeration failed: %d\n", ret);
      stm32_gint_disconnected(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_ep0configure
 *
 * Description:
 *   Configure endpoint 0
 *
 ****************************************************************************/

static int stm32_ep0configure(struct usbhost_driver_s *drvr,
                                 usbhost_ep_t ep0, uint8_t funcaddr,
                                 uint8_t speed, uint16_t maxpacketsize)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  struct stm32_ctrlinfo_s *ep0info = (struct stm32_ctrlinfo_s *)ep0;
  struct stm32_chan_s *chan;
  int ret;

  DEBUGASSERT(drvr != NULL && ep0info != NULL);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure IN channel */

  chan = &priv->chan[ep0info->inndx];
  chan->funcaddr = funcaddr;
  chan->speed = speed;
  chan->maxpacket = maxpacketsize;
  stm32_chan_configure(priv, ep0info->inndx);

  /* Configure OUT channel */

  chan = &priv->chan[ep0info->outndx];
  chan->funcaddr = funcaddr;
  chan->speed = speed;
  chan->maxpacket = maxpacketsize;
  stm32_chan_configure(priv, ep0info->outndx);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: stm32_epalloc
 *
 * Description:
 *   Allocate an endpoint
 *
 ****************************************************************************/

static int stm32_epalloc(struct usbhost_driver_s *drvr,
                            const struct usbhost_epdesc_s *epdesc,
                            usbhost_ep_t *ep)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  int ret;

  DEBUGASSERT(drvr != NULL && epdesc != NULL && ep != NULL);
  DEBUGASSERT(priv->connected);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (epdesc->xfrtype == USB_EP_ATTR_XFER_CONTROL)
    {
      ret = stm32_ctrlep_alloc(priv, epdesc, ep);
    }
  else
    {
      ret = stm32_xfrep_alloc(priv, epdesc, ep);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: stm32_epfree
 *
 * Description:
 *   Free an endpoint
 *
 ****************************************************************************/

static int stm32_epfree(struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  int ret;

  DEBUGASSERT(drvr != NULL);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* A single channel is represent by an index in the range of 0 to
   * STM32H5_NHOST_CHANNELS.  Otherwise, the ep must be a pointer to
   * an allocated control endpoint structure.
   */

  if ((uintptr_t)ep < STM32H5_NHOST_CHANNELS)
    {
      /* Halt the channel and mark the channel available */

      stm32_chan_free(priv, (int)ep);
    }
  else
    {
      /* Halt both control channel and mark the channels available */

      struct stm32_ctrlinfo_s *ctrlep =
        (struct stm32_ctrlinfo_s *)ep;

      stm32_chan_free(priv, ctrlep->inndx);
      stm32_chan_free(priv, ctrlep->outndx);

      /* And free the control endpoint container */

      kmm_free(ctrlep);
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: stm32_alloc
 *
 * Description:
 *   Allocate a request/descriptor buffer
 *
 ****************************************************************************/

static int stm32_alloc(struct usbhost_driver_s *drvr,
                          uint8_t **buffer, size_t *maxlen)
{
  uint8_t *alloc;

  DEBUGASSERT(drvr && buffer && maxlen);

  alloc = kmm_malloc(CONFIG_STM32H5_USBDRD_DESCSIZE);
  if (!alloc)
    {
      return -ENOMEM;
    }

  *buffer = alloc;
  *maxlen = CONFIG_STM32H5_USBDRD_DESCSIZE;
  return OK;
}

/****************************************************************************
 * Name: stm32_free
 *
 * Description:
 *   Free a request/descriptor buffer
 *
 ****************************************************************************/

static int stm32_free(struct usbhost_driver_s *drvr, uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);
  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: stm32_ioalloc
 *
 * Description:
 *   Allocate an I/O buffer
 *
 ****************************************************************************/

static int stm32_ioalloc(struct usbhost_driver_s *drvr,
                            uint8_t **buffer, size_t buflen)
{
  uint8_t *alloc;

  DEBUGASSERT(drvr && buffer && buflen > 0);

  alloc = kmm_malloc(buflen);
  if (!alloc)
    {
      return -ENOMEM;
    }

  *buffer = alloc;
  return OK;
}

/****************************************************************************
 * Name: stm32_iofree
 *
 * Description:
 *   Free an I/O buffer
 *
 ****************************************************************************/

static int stm32_iofree(struct usbhost_driver_s *drvr, uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);
  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: stm32_ctrlin
 *
 * Description:
 *   Control IN transfer
 *
 ****************************************************************************/

static int stm32_ctrlin(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                           const struct usb_ctrlreq_s *req, uint8_t *buffer)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  struct stm32_ctrlinfo_s *ep0info = (struct stm32_ctrlinfo_s *)ep0;
  uint16_t buflen;
  int retries;
  int ret;

  DEBUGASSERT(drvr != NULL && ep0info != NULL && req != NULL);

  buflen = stm32_getle16(req->len);

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        req->type, req->req,
        stm32_getle16(req->value),
        stm32_getle16(req->index),
        buflen);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  for (retries = 0; retries < STM32H5_RETRY_COUNT; retries++)
    {
      /* Send SETUP */

      ret = stm32_ctrl_sendsetup(priv, ep0info, req);
      if (ret < 0)
        {
          uerr("ERROR: SETUP failed: %d\n", ret);
          continue;
        }

      /* Receive data */

      if (buflen > 0)
        {
          ret = stm32_ctrl_recvdata(priv, ep0info, buffer, buflen);
          if (ret < 0)
            {
              uerr("ERROR: Data IN failed: %d\n", ret);
              continue;
            }
        }

      /* Send status OUT */

      ret = stm32_ctrl_senddata(priv, ep0info, NULL, 0);
      if (ret < 0)
        {
          uerr("ERROR: Status OUT failed: %d\n", ret);
          continue;
        }

      /* Success */

      nxmutex_unlock(&priv->lock);
      return OK;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: stm32_ctrlout
 *
 * Description:
 *   Control OUT transfer
 *
 ****************************************************************************/

static int stm32_ctrlout(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                            const struct usb_ctrlreq_s *req,
                            const uint8_t *buffer)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  struct stm32_ctrlinfo_s *ep0info = (struct stm32_ctrlinfo_s *)ep0;
  uint16_t buflen;
  int retries;
  int ret;

  DEBUGASSERT(drvr != NULL && ep0info != NULL && req != NULL);

  buflen = stm32_getle16(req->len);

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        req->type, req->req,
        stm32_getle16(req->value),
        stm32_getle16(req->index),
        buflen);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  for (retries = 0; retries < STM32H5_RETRY_COUNT; retries++)
    {
      /* Send SETUP */

      ret = stm32_ctrl_sendsetup(priv, ep0info, req);
      if (ret < 0)
        {
          uerr("ERROR: SETUP failed: %d\n", ret);
          continue;
        }

      /* Send data */

      if (buflen > 0)
        {
          ret = stm32_ctrl_senddata(priv, ep0info,
                                       (uint8_t *)buffer, buflen);
          if (ret < 0)
            {
              uerr("ERROR: Data OUT failed: %d\n", ret);
              continue;
            }
        }

      /* Receive status IN */

      ret = stm32_ctrl_recvdata(priv, ep0info, NULL, 0);
      if (ret < 0)
        {
          uerr("ERROR: Status IN failed: %d\n", ret);
          continue;
        }

      nxmutex_unlock(&priv->lock);
      return OK;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: stm32_transfer
 *
 * Description:
 *   Bulk/interrupt transfer
 *
 ****************************************************************************/

static ssize_t stm32_transfer(struct usbhost_driver_s *drvr,
                                 usbhost_ep_t ep, uint8_t *buffer,
                                 size_t buflen)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  unsigned int chidx = (unsigned int)ep;
  ssize_t nbytes;
  int ret;

  DEBUGASSERT(priv && buffer && chidx < STM32H5_NHOST_CHANNELS);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->chan[chidx].in)
    {
      nbytes = stm32_in_transfer(priv, chidx, buffer, buflen);
    }
  else
    {
      nbytes = stm32_out_transfer(priv, chidx, buffer, buflen);
      if (nbytes > 0 && (buflen % priv->chan[chidx].maxpacket == 0))
        {
          /* For OUT transfers that are a multiple of maxpacket,
           * we need to send a zero-length packet to complete the transfer.
           */

          int zlp_ret = stm32_out_transfer(priv, chidx, NULL, 0);
          if (zlp_ret < 0)
            {
              uerr("ERROR: Failed to send ZLP: %d\n", zlp_ret);
              nbytes = zlp_ret;
            }
        }
    }

  nxmutex_unlock(&priv->lock);
  return nbytes;
}

/****************************************************************************
 * Name: stm32_asynch
 *
 * Description:
 *   Async transfer
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int stm32_asynch(struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                           uint8_t *buffer, size_t buflen,
                           usbhost_asynch_t callback, void *arg)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  unsigned int chidx = (unsigned int)ep;
  struct stm32_chan_s *chan;
  int ret;

  DEBUGASSERT(priv && buffer && chidx < STM32H5_NHOST_CHANNELS);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  chan = &priv->chan[chidx];
  chan->waiter = false;
  chan->callback = callback;
  chan->arg = arg;
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd = 0;

  stm32_transfer_start(priv, chidx);

  nxmutex_unlock(&priv->lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_cancel
 *
 * Description:
 *   Cancel a pending transfer
 *
 ****************************************************************************/

static int stm32_cancel(struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  unsigned int chidx = (unsigned int)ep;
  struct stm32_chan_s *chan;
  irqstate_t flags;

  DEBUGASSERT(priv && chidx < STM32H5_NHOST_CHANNELS);

  chan = &priv->chan[chidx];

  flags = enter_critical_section();

  if (chan->in)
    {
      stm32_set_chep_rx_status(priv, chidx, USB_CHEP_RX_STRX_DIS);
    }
  else
    {
      stm32_set_chep_tx_status(priv, chidx, USB_CHEP_TX_STTX_DIS);
    }

  chan->result = ECANCELED;
  chan->chreason = CHREASON_CANCELLED;
  stm32_chan_wakeup(priv, chan);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: stm32_connect
 *
 * Description:
 *   Hub port connection notification
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_HUB
static int stm32_connect(struct usbhost_driver_s *drvr,
                            struct usbhost_hubport_s *hport, bool connected)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL && hport != NULL);

  /* Set the connected/disconnected flag */

  hport->connected = connected;
  uinfo("Hub port %d connected: %s\n",
        hport->port, connected ? "YES" : "NO");

  /* Report the connection event */

  flags = enter_critical_section();
  priv->hport = hport;
  if (priv->pscwait)
    {
      priv->pscwait = false;
      nxsem_post(&priv->pscsem);
    }

  leave_critical_section(flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_disconnect
 *
 * Description:
 *   Device disconnection notification
 *
 ****************************************************************************/

static void stm32_disconnect(struct usbhost_driver_s *drvr,
                                struct usbhost_hubport_s *hport)
{
  DEBUGASSERT(hport != NULL);
  hport->devclass = NULL;
}

/****************************************************************************
 * Name: stm32_portreset
 *
 * Description:
 *   Reset the USB port
 *
 ****************************************************************************/

static void stm32_portreset(struct stm32_usbhost_s *priv)
{
  uint32_t regval;

  /* Force USB reset */

  regval = stm32_getreg(STM32_USB_CNTR);
  regval |= USB_CNTR_FRES;
  stm32_putreg(STM32_USB_CNTR, regval);

  /* Wait for reset */

  nxsched_usleep(STM32H5_RESET_DELAY * 1000);

  /* Release reset */

  regval &= ~USB_CNTR_FRES;
  stm32_putreg(STM32_USB_CNTR, regval);

  /* Wait for device to be ready */

  nxsched_usleep(30 * 1000);
}

/****************************************************************************
 * Name: stm32_host_initialize
 *
 * Description:
 *   Initialize the USB host hardware
 *
 ****************************************************************************/

static void stm32_host_initialize(struct stm32_usbhost_s *priv)
{
  uint32_t regval;

  UNUSED(priv);

  /* Clear all pending interrupts */

  stm32_putreg(STM32_USB_ISTR, 0);

  /* Disable all interrupts initially (preserve HOST bit) */

  regval = stm32_getreg(STM32_USB_CNTR);
  regval &= ~(USB_CNTR_CTRM | USB_CNTR_PMAOVRN | USB_CNTR_ERRM |
              USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_RESETM |
              USB_CNTR_SOFM | USB_CNTR_ESOFM | USB_CNTR_L1REQ |
              USB_CNTR_DDISCM | USB_CNTR_THR512M);
  stm32_putreg(STM32_USB_CNTR, regval);

  /* Clear pending interrupts again */

  stm32_putreg(STM32_USB_ISTR, 0);

  /* Set pull-down on D+ for device detection (host mode) */

  regval = stm32_getreg(STM32_USB_BCDR);
  regval |= USB_BCDR_DPPD;
  stm32_putreg(STM32_USB_BCDR, regval);

  /* Enable host mode interrupts:
   * - CTRM: Correct transfer
   * - PMAOVRN: PMA overrun - not used
   * - ERRM: Error
   * - WKUPM: Wakeup - not used
   * - SUSPM: Suspend - not used
   * - RESETM/DCON: Device connection (in host mode, RESETM serves as DCON)
   * - SOFM: Start of frame - not used
   * - ESOFM: Expected start of frame - not used
   * - L1REQ: LPM L1 request - not used
   */

  regval = stm32_getreg(STM32_USB_CNTR);
  regval |= (USB_CNTR_CTRM | USB_CNTR_ERRM | USB_CNTR_RESETM);
  stm32_putreg(STM32_USB_CNTR, regval);
}

/****************************************************************************
 * Name: stm32_sw_initialize
 *
 * Description:
 *   Initialize driver software state
 *
 ****************************************************************************/

static void stm32_sw_initialize(struct stm32_usbhost_s *priv)
{
  struct usbhost_driver_s *drvr;
  struct usbhost_hubport_s *hport;
  int i;

  /* Initialize driver interface */

  drvr = &priv->drvr;
  drvr->ep0configure = stm32_ep0configure;
  drvr->epalloc = stm32_epalloc;
  drvr->epfree = stm32_epfree;
  drvr->alloc = stm32_alloc;
  drvr->free = stm32_free;
  drvr->ioalloc = stm32_ioalloc;
  drvr->iofree = stm32_iofree;
  drvr->ctrlin = stm32_ctrlin;
  drvr->ctrlout = stm32_ctrlout;
  drvr->transfer = stm32_transfer;
#ifdef CONFIG_USBHOST_ASYNCH
  drvr->asynch = stm32_asynch;
#endif
  drvr->cancel = stm32_cancel;
#ifdef CONFIG_USBHOST_HUB
  drvr->connect = stm32_connect;
#endif
  drvr->disconnect = stm32_disconnect;

  /* Initialize root hub port */

  hport = &priv->rhport.hport;
  hport->drvr = drvr;
#ifdef CONFIG_USBHOST_HUB
  hport->parent = NULL;
#endif
  hport->ep0 = &priv->ep0;
  hport->port = 0;
  hport->speed = USB_SPEED_FULL;

  /* Initialize function address generation logic */

  usbhost_devaddr_initialize(&priv->devgen);
  priv->rhport.pdevgen = &priv->devgen;

  /* Initialize state */

  priv->smstate = SMSTATE_DETACHED;
  priv->connected = false;
  priv->change = false;
  priv->pscwait = false;

  /* Initialize PMA allocation - all buffers available */

  priv->pma_bufavail = STM32H5_PMA_BUFFER_ALLSET;

  /* Initialize channels */

  for (i = 0; i < STM32H5_NHOST_CHANNELS; i++)
    {
      priv->chan[i].chidx = i;
      priv->chan[i].inuse = false;
      priv->chan[i].pmabufno = STM32H5_PMA_BUFFER_NONE;
      nxsem_init(&priv->chan[i].waitsem, 0, 0);
    }
}

/****************************************************************************
 * Name: stm32_hw_initialize
 *
 * Description:
 *   Initialize the USB hardware
 *
 ****************************************************************************/

static int stm32_hw_initialize(struct stm32_usbhost_s *priv)
{
  uint32_t regval;
  int ret;

  /* Enable VDDUSB supply - required for USB PHY operation */

  putreg32(PWR_USBSCR_USB33DEN, STM32_PWR_USBSCR);

  /* Wait for VDDUSB to be ready */

  while ((getreg32(STM32_PWR_VMSR) & PWR_VMSR_USB33RDY) == 0)
    {
    }

  /* Enable and validate USB supply */

  putreg32(PWR_USBSCR_USB33DEN | PWR_USBSCR_USB33SV, STM32_PWR_USBSCR);

  /* Configure USB GPIO pins (PA11=D-, PA12=D+) */

  stm32_configgpio(GPIO_USB_DM);
  stm32_configgpio(GPIO_USB_DP);

  /* Enable USB clock via RCC */

  regval = getreg32(STM32_RCC_APB2ENR);
  regval |= RCC_APB2ENR_USBEN;
  putreg32(regval, STM32_RCC_APB2ENR);

  /* Wait for USB clock to stabilize */

  nxsched_usleep(2000);

  /* Perform USB core reset sequence:
   * 1. Disable host mode
   * 2. Set USBRST to reset the core
   */

  regval = stm32_getreg(STM32_USB_CNTR);
  regval &= ~USB_CNTR_HOST;
  regval |= USB_CNTR_FRES;  /* Force USB reset */
  stm32_putreg(STM32_USB_CNTR, regval);

  /* Clear all pending interrupts */

  stm32_putreg(STM32_USB_ISTR, 0);

  /* Exit power-down mode and release reset:
   * Clear PDWN to enable the analog transceiver
   * Clear FRES to release the USB reset
   */

  regval = stm32_getreg(STM32_USB_CNTR);
  regval &= ~(USB_CNTR_PDWN | USB_CNTR_FRES);
  stm32_putreg(STM32_USB_CNTR, regval);

  /* Wait for analog circuits to stabilize (tSTARTUP) */

  nxsched_usleep(1000);

  /* Set host mode */

  regval = stm32_getreg(STM32_USB_CNTR);
  regval |= USB_CNTR_HOST;
  stm32_putreg(STM32_USB_CNTR, regval);

  /* Initialize host mode (interrupts, pull-downs, etc.) */

  stm32_host_initialize(priv);

  /* Attach interrupt handler */

  ret = irq_attach(STM32_IRQ_USB_FS, stm32_usbdrd_interrupt, priv);
  if (ret < 0)
    {
      uerr("ERROR: Failed to attach IRQ: %d\n", ret);
      return ret;
    }

  /* Enable interrupt */

  up_enable_irq(STM32_IRQ_USB_FS);

  /* Enable VBUS drive */

  stm32h5_usbhost_vbusdrive(0, true);

  uinfo("USB Host initialized\n");

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32h5_usbhost_initialize
 *
 * Description:
 *   Initialize USB host controller
 *
 ****************************************************************************/

struct usbhost_connection_s *stm32h5_usbhost_initialize(void)
{
  struct stm32_usbhost_s *priv = &g_usbhost;
  int ret;

  uinfo("Initializing USB DRD host controller\n");

  /* Initialize software state */

  stm32_sw_initialize(priv);

  /* Initialize hardware */

  ret = stm32_hw_initialize(priv);
  if (ret < 0)
    {
      uerr("ERROR: Hardware initialization failed: %d\n", ret);
      return NULL;
    }

  return &g_usbconn;
}

/****************************************************************************
 * Name: stm32_usbhost_vbusdrive
 *
 * Description:
 *   Control VBUS power
 *   This is a weak function that should be overridden by board-specific code
 *
 ****************************************************************************/

__attribute__((weak))
void stm32_usbhost_vbusdrive(int port, bool enable)
{
  /* Default implementation - do nothing.
   * Board-specific code should override this to control VBUS power.
   */

  uinfo("VBUS drive port=%d enable=%d (default - no-op)\n", port, enable);
}

#endif /* CONFIG_USBHOST && CONFIG_STM32H5_USBFS_HOST */

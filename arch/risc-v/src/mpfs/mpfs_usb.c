/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_usb.c
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
#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_devaddr.h>
#include <nuttx/usb/usbhost_trace.h>

#include <nuttx/spinlock.h>

#include <arch/board/board.h>

#include "hardware/mpfs_usb.h"
#include "riscv_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USB trace error codes */

#define MPFS_TRACEERR_ALLOCFAIL            0x0001
#define MPFS_TRACEERR_BADCLEARFEATURE      0x0002
#define MPFS_TRACEERR_BADDEVGETSTATUS      0x0003
#define MPFS_TRACEERR_BADEPGETSTATUS       0x0004
#define MPFS_TRACEERR_BADEPNO              0x0006
#define MPFS_TRACEERR_BADEPTYPE            0x0007
#define MPFS_TRACEERR_BADGETCONFIG         0x0008
#define MPFS_TRACEERR_BADGETSTATUS         0x0009
#define MPFS_TRACEERR_BADSETADDRESS        0x000a
#define MPFS_TRACEERR_BADSETCONFIG         0x000b
#define MPFS_TRACEERR_BADSETFEATURE        0x000c
#define MPFS_TRACEERR_BINDFAILED           0x000d
#define MPFS_TRACEERR_DISPATCHSTALL        0x000e
#define MPFS_TRACEERR_EP0SETUPSTALLED      0x000f
#define MPFS_TRACEERR_EPOUTNULLPACKET      0x0010
#define MPFS_TRACEERR_INVALIDCTRLREQ       0x0011
#define MPFS_TRACEERR_IRQREGISTRATION      0x0012
#define MPFS_TRACEERR_TXCOMPERR            0x0013
#define MPFS_TRACEERR_INVALID_EP0_STATE    0x0014
#define MPFS_TRACEERR_EP0SETUPOUTSIZE      0x0015
#define MPFS_TRACEERR_EPOUTQEMPTY          0x0016
#define MPFS_TRACEERR_EP0PREMATURETERM     0x0017

/* USB trace interrupt codes */

#define MPFS_TRACEINTID_INTERRUPT          0x0001
#define MPFS_TRACEINTID_EP_TX_IRQ          0x0002
#define MPFS_TRACEINTID_EP_RX_IRQ          0x0003
#define MPFS_TRACEINTID_EP_RX_CSR          0x0004
#define MPFS_TRACEINTID_EP_RX_COUNT        0x0005
#define MPFS_TRACEINTID_EP_TX_CSR          0x0006
#define MPFS_TRACEINTID_EP0_CSR0           0x0007
#define MPFS_TRACEINTID_EP0_COUNT0         0x0008
#define MPFS_TRACEINTID_EP0SETUPSETADDRESS 0x0009
#define MPFS_TRACEINTID_GETSTATUS          0x000a
#define MPFS_TRACEINTID_DEVGETSTATUS       0x000b
#define MPFS_TRACEINTID_IFGETSTATUS        0x000c
#define MPFS_TRACEINTID_CLEARFEATURE       0x000d
#define MPFS_TRACEINTID_SETFEATURE         0x000e
#define MPFS_TRACEINTID_GETCONFIG          0x000f
#define MPFS_TRACEINTID_SETCONFIG          0x0010
#define MPFS_TRACEINTID_GETSETIF           0x0011
#define MPFS_TRACEINTID_SYNCHFRAME         0x0012
#define MPFS_TRACEINTID_DISPATCH           0x0013
#define MPFS_TRACEINTID_EP0_STALLSENT      0x0014
#define MPFS_TRACEINTID_DATA_END           0x0015

/* USB PMP configuration registers */

#define MPFS_PMPCFG_USB_0    (MPFS_MPUCFG_BASE + 0x600)
#define MPFS_PMPCFG_USB_1    (MPFS_MPUCFG_BASE + 0x608)
#define MPFS_PMPCFG_USB_2    (MPFS_MPUCFG_BASE + 0x610)
#define MPFS_PMPCFG_USB_3    (MPFS_MPUCFG_BASE + 0x618)

/* Reset and clock control registers */

#define MPFS_SYSREG_SOFT_RESET_CR     (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SOFT_RESET_CR_OFFSET)
#define MPFS_SYSREG_SUBBLK_CLOCK_CR   (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET)

#ifdef CONFIG_ENDIAN_BIG
#  define LSB 1
#  define MSB 0
#else
#  define LSB 0
#  define MSB 1
#endif

#define MPFS_NUM_USB_PKT      1
#define MPFS_MIN_EP_FIFO_SIZE 8
#define MPFS_USB_REG_MAX      0x2000

/* Request queue operations *************************************************/

#define mpfs_rqempty(q)      ((q)->head == NULL)
#define mpfs_rqpeek(q)       ((q)->head)

#define MPFS_EPSET_ALL             (0xff)    /* All endpoints */
#define MPFS_EPSET_NOTEP0          (0xfe)    /* All endpoints except EP0 */
#define MPFS_EP_BIT(ep)            (1 << (ep))
#define MPFS_MAX_MULTIPACKET_SIZE  (0x3fff)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum mpfs_epstate_e
{
  USB_EPSTATE_DISABLED = 0, /* Endpoint is disabled */
  USB_EPSTATE_STALLED,      /* Endpoint is stalled */
  USB_EPSTATE_IDLE,         /* Endpoint is idle */
  USB_EPSTATE_SENDING,      /* Endpoint is sending data */
  USB_EPSTATE_RXSTOPPED,    /* EP is stopped waiting for a read request */
  USB_EPSTATE_EP0DATAOUT,   /* Endpoint 0 is receiving SETUP OUT data */
  USB_EPSTATE_EP0STATUSIN,  /* Endpoint 0 is sending SETUP status */
  USB_EPSTATE_EP0ADDRESS    /* Address change is pending completion */
};

/* Device states */

enum mpfs_devstate_e
{
  USB_DEVSTATE_SUSPENDED = 0, /* The device is currently suspended */
  USB_DEVSTATE_POWERED,       /* Host is powered through the USB cable */
  USB_DEVSTATE_DEFAULT,       /* Device has been reset */
  USB_DEVSTATE_ADDRESSED,     /* The device has an address on the bus */
  USB_DEVSTATE_CONFIGURED     /* A valid configuration has been selected. */
};

/* The result of EP0 SETUP */

enum mpfs_ep0setup_e
{
  USB_EP0SETUP_SUCCESS = 0,   /* The SETUP was handled without incident */
  USB_EP0SETUP_DISPATCHED,    /* The SETUP was forwarded */
  USB_EP0SETUP_ADDRESS,       /* A new device address is pending */
  USB_EP0SETUP_STALL          /* An error occurred */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int    mpfs_ep_configure(struct usbdev_ep_s *ep,
                                const struct usb_epdesc_s *desc, bool last);
static int    mpfs_ep_disable(struct usbdev_ep_s *ep);
static struct usbdev_req_s *mpfs_ep_allocreq(struct usbdev_ep_s *ep);
#ifdef CONFIG_USBDEV_DMA
static void  *mpfs_ep_allocbuffer(struct usbdev_ep_s *ep, uint16_t nbytes);
static void   mpfs_ep_freebuffer(struct usbdev_ep_s *ep, void *buf);
#endif
static void   mpfs_ep_freereq(struct usbdev_ep_s *ep,
                              struct usbdev_req_s *);
static int    mpfs_ep_submit(struct usbdev_ep_s *ep,
                             struct usbdev_req_s *req);
static int    mpfs_ep_cancel(struct usbdev_ep_s *ep,
                             struct usbdev_req_s *req);
static int    mpfs_ep_stallresume(struct usbdev_ep_s *ep, bool resume);

/* USB device controller operations */

static struct usbdev_ep_s *mpfs_allocep(struct usbdev_s *dev, uint8_t epno,
                                        bool in, uint8_t eptype);
static void   mpfs_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep);
static int    mpfs_getframe(struct usbdev_s *dev);
static int    mpfs_wakeup(struct usbdev_s *dev);
static int    mpfs_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int    mpfs_pullup(struct usbdev_s *dev,  bool enable);

/* Interrupt level processing */

static void   mpfs_ep0_ctrlread(struct mpfs_usbdev_s *priv);
static void   mpfs_ep0_wrstatus(struct mpfs_usbdev_s *priv,
                                const uint8_t *buffer, size_t buflen);
static void   mpfs_ep0_dispatch(struct mpfs_usbdev_s *priv);
static void   mpfs_setdevaddr(struct mpfs_usbdev_s *priv, uint8_t value);
static void   mpfs_ep0_setup(struct mpfs_usbdev_s *priv);
static int    mpfs_usb_interrupt(int irq, void *context, void *arg);

static void   mpfs_epset_reset(struct mpfs_usbdev_s *priv, uint16_t epset);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mpfs_usbdev_s g_usbd;

static const struct usbdev_epops_s g_epops =
{
  .configure     = mpfs_ep_configure,
  .disable       = mpfs_ep_disable,
  .allocreq      = mpfs_ep_allocreq,
  .freereq       = mpfs_ep_freereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer   = mpfs_ep_allocbuffer,
  .freebuffer    = mpfs_ep_freebuffer,
#endif
  .submit        = mpfs_ep_submit,
  .cancel        = mpfs_ep_cancel,
  .stall         = mpfs_ep_stallresume,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep       = mpfs_allocep,
  .freeep        = mpfs_freeep,
  .getframe      = mpfs_getframe,
  .wakeup        = mpfs_wakeup,
  .selfpowered   = mpfs_selfpowered,
  .pullup        = mpfs_pullup,
};

/* This describes the endpoint 0 */

static const struct usb_epdesc_s g_ep0desc =
{
  .len           = USB_SIZEOF_EPDESC,
  .type          = USB_DESC_TYPE_ENDPOINT,
  .addr          = EP0,
  .attr          = USB_EP_ATTR_XFER_CONTROL,
  .mxpacketsize  =
  {
    64, 0
  },
  .interval      = 0
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_modifyreg16
 *
 * Description:
 *   Atomically modify the specified bits in the memory mapped register.
 *   This also checks the addr range is valid.
 *
 * Input Parameters:
 *   addr      - Address to access
 *   clearbits - Bits to clear
 *   setbits   - Bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_modifyreg16(uintptr_t addr, uint16_t clearbits,
                             uint16_t setbits)
{
  irqstate_t flags;
  uint16_t   regval;

  DEBUGASSERT((addr >= MPFS_USB_BASE) && addr < (MPFS_USB_BASE +
               MPFS_USB_REG_MAX));

  flags   = spin_lock_irqsave(NULL);
  regval  = getreg16(addr);
  regval &= ~clearbits;
  regval |= setbits;
  putreg16(regval, addr);
  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Register Operations
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_putreg32
 *
 * Description:
 *   Set the contents of a 32-bit register.  This helper wrapper checks
 *   the range before accessing the address.
 *
 * Input Parameters:
 *   regval     - Value to store
 *   regaddr    - Address where the regval is stored
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void mpfs_putreg32(uint32_t regval, uintptr_t regaddr)
{
  DEBUGASSERT((regaddr >= MPFS_USB_BASE) && regaddr < (MPFS_USB_BASE +
               MPFS_USB_REG_MAX));

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: mpfs_putreg16
 *
 * Description:
 *   Set the contents of a 16-bit register.  This helper wrapper checks
 *   the range before accessing the address.
 *
 * Input Parameters:
 *   regval     - Value to store
 *   regaddr    - Address where the regval is stored
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void mpfs_putreg16(uint16_t regval, uintptr_t regaddr)
{
  DEBUGASSERT((regaddr >= MPFS_USB_BASE) && regaddr < (MPFS_USB_BASE +
               MPFS_USB_REG_MAX));

  putreg16(regval, regaddr);
}

/****************************************************************************
 * Name: mpfs_putreg8
 *
 * Description:
 *   Set the contents of an 8-bit register.  This helper wrapper checks
 *   the range before accessing the address.
 *
 * Input Parameters:
 *   regval     - Value to store
 *   regaddr    - Address where the regval is stored
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void mpfs_putreg8(uint8_t regval, uintptr_t regaddr)
{
  DEBUGASSERT((regaddr >= MPFS_USB_BASE) && regaddr < (MPFS_USB_BASE +
               MPFS_USB_REG_MAX));

  putreg8(regval, regaddr);
}

/****************************************************************************
 * Name: mpfs_enableclk
 *
 * Description:
 *  Enables the USB clock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_enableclk(void)
{
  modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR, 0, SYSREG_SUBBLK_CLOCK_CR_USB);
}

/****************************************************************************
 * Name: mpfs_disableclk
 *
 * Description:
 *   Disables the USB clock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_disableclk(void)
{
  modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR, SYSREG_SUBBLK_CLOCK_CR_USB, 0);
}

/****************************************************************************
 * Request Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_req_dequeue
 *
 * Description:
 *   Removes an entry from the linked list.
 *
 * Input Parameters:
 *   queue    - Head of the linked list
 *
 * Returned Value:
 *   The entry or NULL is none in the list
 *
 ****************************************************************************/

static struct mpfs_req_s *mpfs_req_dequeue(struct mpfs_rqhead_s *queue)
{
  struct mpfs_req_s *ret = queue->head;

  if (ret != NULL)
    {
      queue->head = ret->flink;
      if (queue->head == NULL)
        {
          queue->tail = NULL;
        }

      ret->flink = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: mpfs_req_enqueue
 *
 * Description:
 *   Add an entry to the linked list.
 *
 * Input Parameters:
 *   queue    - Head of the linked list
 *   req      - The entry to be added
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_req_enqueue(struct mpfs_rqhead_s *queue,
                             struct mpfs_req_s *req)
{
  req->flink = NULL;

  if (queue->head == NULL)
    {
      queue->head = req;
      queue->tail = req;
    }
  else
    {
      queue->tail->flink = req;
      queue->tail        = req;
    }
}

/****************************************************************************
 * Name: mpfs_req_complete
 *
 * Description:
 *   Add an entry to the linked list.
 *
 * Input Parameters:
 *   privep    - Endpoint private data
 *   result    - Status of the completion process
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_req_complete(struct mpfs_ep_s *privep, int16_t result)
{
  struct mpfs_req_s *privreq;
  irqstate_t flags;

  /* Remove the completed request at the head of the endpoint request list */

  flags = enter_critical_section();
  privreq = mpfs_req_dequeue(&privep->reqq);
  leave_critical_section(flags);

  if (privreq)
    {
      /* Save the result in the request structure */

      privreq->req.result = result;

      /* Callback to the request completion handler */

      privreq->flink = NULL;
      privreq->req.callback(&privep->ep, &privreq->req);

      /* Mark the endpoint ready for the next transmission */

      privep->epstate = USB_EPSTATE_IDLE;
      privep->zlpsent = false;
    }
}

/****************************************************************************
 * Name: mpfs_req_cancel
 *
 * Description:
 *   Cancel all entries of an endpoint queue
 *
 * Input Parameters:
 *   privep    - Endpoint private data
 *   result    - Status of the completion process
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_req_cancel(struct mpfs_ep_s *privep, int16_t result)
{
  /* Complete every queued request with the specified status */

  while (!mpfs_rqempty(&privep->reqq))
    {
      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)),
               (mpfs_rqpeek(&privep->reqq))->req.xfrd);
      mpfs_req_complete(privep, result);
    }
}

/****************************************************************************
 * Name: mpfs_write_tx_fifo
 *
 * Description:
 *   Write data into the TX fifo buffer.
 *
 * Input Parameters:
 *   in_data    - Pointer to the data to be sent
 *   length     - Amount of bytes to write
 *   epno       - Endpoint number
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_write_tx_fifo(const void *in_data, uint32_t length,
                               uint8_t epno)
{
  uint32_t i;
  uint32_t *temp;
  uint8_t  *temp_8bit;
  uint16_t tx_csr;
  uint16_t words = length / 4;
  uint16_t bytes = length - words * 4;
  uint16_t offset;

  temp      = (uint32_t *)in_data;
  temp_8bit = (uint8_t *)in_data;

  /* Poll mode: wait for fifo empty first */

  do
    {
      tx_csr = getreg16(MPFS_USB_ENDPOINT(epno) +
                        MPFS_USB_ENDPOINT_TX_CSR_OFFSET);
    }
  while (tx_csr & TXCSRL_REG_EPN_TX_FIFO_NE_MASK);

  /* Send 32-bit words first */

  for (i = 0; i < words; i++)
    {
      mpfs_putreg32((uint32_t)temp[i], MPFS_USB_FIFO(epno));
    }

  offset = words << 2;

  /* Send the remaining bytes */

  for (i = offset; i < (offset + bytes); i++)
    {
      mpfs_putreg8((uint8_t)temp_8bit[i], MPFS_USB_FIFO(epno));
    }
}

/****************************************************************************
 * Name: mpfs_req_wrsetup
 *
 * Description:
 *   Setup the next queued write request.
 *
 * Input Parameters:
 *   priv       - Private USB device abstraction
 *   privep     - Private endpoint abstraction
 *   privreq    - The actual write request
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_req_wrsetup(struct mpfs_usbdev_s *priv,
                             struct mpfs_ep_s *privep,
                             struct mpfs_req_s *privreq)
{
  const uint8_t *buf;
  uint32_t packetsize;
  uint8_t epno;
  int nbytes;

  epno = USB_EPNO(privep->ep.eplog);

  mpfs_putreg8(epno, MPFS_USB_INDEX);

  /* Get the number of bytes remaining to be sent. */

  DEBUGASSERT(privreq->req.xfrd < privreq->req.len);
  nbytes = privreq->req.len - privreq->req.xfrd;

  usbtrace(TRACE_WRITE(USB_EPNO(privep->ep.eplog)), nbytes);

  /* Either send the maxpacketsize(multi) or all of the remaining data in
   * the request.
   */

  if (nbytes >= MPFS_MAX_MULTIPACKET_SIZE)
    {
      nbytes = MPFS_MAX_MULTIPACKET_SIZE;
    }

  /* This is the new number of bytes "in-flight" */

  privreq->inflight = nbytes;

  /* The new buffer pointer is the start of the buffer plus the number of
   * bytes successfully transferred plus the number of bytes previously
   * "in-flight".
   */

  buf = privreq->req.buf + privreq->req.xfrd;

  /* Setup TX transfer using ep configured maxpacket size */

  priv->eplist[epno].descb[1]->addr = (uintptr_t)buf;
  packetsize = priv->eplist[epno].descb[1]->pktsize;

  /* Set automatic ZLP sending if requested on req */

  if (privreq->req.flags & USBDEV_REQFLAGS_NULLPKT)
    {
      /* Handle this properly when DMA supported */
    }

  priv->eplist[epno].descb[1]->pktsize = packetsize;

  /* Indicate that we are in the sending state */

  privep->epstate = USB_EPSTATE_SENDING;

  if (nbytes > packetsize)
    {
      mpfs_write_tx_fifo(buf, packetsize, epno);

      if (epno == EP0)
        {
          mpfs_modifyreg16(MPFS_USB_INDEXED_CSR_EP0_CSR0, 0,
                           CSR0L_DEV_TX_PKT_RDY_MASK);
        }
      else
        {
          mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                           MPFS_USB_ENDPOINT_TX_CSR_OFFSET,
                           TXCSRL_REG_EPN_UNDERRUN_MASK,
                           TXCSRL_REG_EPN_TX_PKT_RDY_MASK);
        }

      privreq->inflight = packetsize;
      return;
    }
  else
    {
      mpfs_write_tx_fifo(buf, nbytes, epno);
    }

  privreq->req.xfrd += nbytes;

  /* With poll mode (no DMA), we're done sending */

  privep->epstate = USB_EPSTATE_IDLE;

  if (epno == EP0)
    {
      mpfs_putreg16(CSR0L_DEV_TX_PKT_RDY_MASK | CSR0L_DEV_DATA_END_MASK,
                    MPFS_USB_INDEXED_CSR_EP0_CSR0);
    }
  else
    {
      mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                       MPFS_USB_ENDPOINT_TX_CSR_OFFSET,
                       TXCSRL_REG_EPN_UNDERRUN_MASK,
                       TXCSRL_REG_EPN_TX_PKT_RDY_MASK);
    }
}

/****************************************************************************
 * Name: mpfs_ep_stall
 *
 * Description:
 *   Stall the endpoint.
 *
 * Input Parameters:
 *   privep     - Private endpoint abstraction
 *
 * Returned Value:
 *   OK always
 *
 ****************************************************************************/

static int mpfs_ep_stall(struct mpfs_ep_s *privep)
{
  irqstate_t flags;
  uint8_t epno;

  DEBUGASSERT(privep->dev);

  epno = USB_EPNO(privep->ep.eplog);

  mpfs_putreg8(epno, MPFS_USB_INDEX);

  /* Check that endpoint is enabled and not already in Halt state */

  flags = enter_critical_section();
  if ((privep->epstate != USB_EPSTATE_DISABLED) &&
      (privep->epstate != USB_EPSTATE_STALLED))
    {
      epno = USB_EPNO(privep->ep.eplog);

      usbtrace(TRACE_EPSTALL, epno);

      /* If this is an IN endpoint (or endpoint 0), then cancel any
       * write requests in progress.
       */

      if (epno == EP0 || USB_ISEPIN(privep->ep.eplog))
        {
          mpfs_req_cancel(privep, -EPERM);
        }

      /* Put endpoint into stalled state */

      privep->epstate = USB_EPSTATE_STALLED;
      privep->stalled = true;
      privep->pending = false;

      if (epno == EP0)
        {
          mpfs_putreg16(CSR0L_DEV_SEND_STALL_MASK |
                        CSR0L_DEV_SERVICED_RX_PKT_RDY_MASK,
                        MPFS_USB_INDEXED_CSR_EP0_CSR0);
        }
      else if (USB_ISEPIN(privep->ep.eplog))
        {
          mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                           MPFS_USB_ENDPOINT_TX_CSR_OFFSET,
                           0,
                           TXCSRL_REG_EPN_SEND_STALL_MASK);
        }
      else
        {
          mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                           MPFS_USB_ENDPOINT_RX_CSR_OFFSET,
                           0,
                           RXCSRL_REG_EPN_SEND_STALL_MASK |
                           RXCSRL_REG_EPN_RX_PKT_RDY_MASK);
        }
    }

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: mpfs_req_write
 *
 * Description:
 *   Write all available entries on the queue.
 *
 * Input Parameters:
 *   priv       - Private USB device abstraction
 *   privep     - Private endpoint abstraction
 *
 * Returned Value:
 *   OK on success, a negated error code in case of failure
 *
 ****************************************************************************/

static int mpfs_req_write(struct mpfs_usbdev_s *priv,
                          struct mpfs_ep_s *privep)
{
  struct mpfs_req_s *privreq;
  uint8_t epno;
  int bytesleft;

  epno = USB_EPNO(privep->ep.eplog);

  mpfs_putreg8(epno, MPFS_USB_INDEX);

  /* We get here when an IN endpoint interrupt occurs.  So now we know that
   * there is no TX transfer in progress (epstate should be IDLE).
   */

  DEBUGASSERT(privep->epstate == USB_EPSTATE_IDLE);

  while (privep->epstate == USB_EPSTATE_IDLE)
    {
      /* Check the request from the head of the endpoint request queue */

      privreq = mpfs_rqpeek(&privep->reqq);
      if (privreq == NULL)
        {
          /* Was there a pending endpoint stall? */

          if (privep->pending)
            {
              /* Yes... stall the endpoint now */

              mpfs_ep_stall(privep);
            }

          return -ENOENT;
        }

      uinfo("epno=%d req=%p: len=%d xfrd=%d inflight=%d\n",
            epno, privreq, privreq->req.len, privreq->req.xfrd,
            privreq->inflight);

      /* Handle any bytes in flight. */

      privreq->req.xfrd += privreq->inflight;
      privreq->inflight  = 0;

      /* Get the number of bytes left to be sent in the packet */

      bytesleft = privreq->req.len - privreq->req.xfrd;
      if (bytesleft > 0)
        {
          /* Perform the write operation. */

          mpfs_req_wrsetup(priv, privep, privreq);
        }
      else if ((privreq->req.len == 0) && !privep->zlpsent)
        {
          /* If we get here, we requested to send the zero length packet now.
           */

          DEBUGASSERT(epno == EP0);

          privep->zlpsent   = true;
          privreq->inflight = 0;

          /* Setup 0 length TX transfer */

          priv->eplist[EP0].descb[1]->addr = (uintptr_t)&priv->ep0out[0];
        }

      if (privep->epstate == USB_EPSTATE_IDLE)
        {
          /* Return the write request to the class driver.  Set the txbusy
           * bit to prevent being called recursively from any new submission
           * generated by returning the write request.
           */

          DEBUGASSERT(privreq->req.len == privreq->req.xfrd);

          privep->txbusy = true;
          usbtrace(TRACE_COMPLETE(epno), privreq->req.xfrd);
          mpfs_req_complete(privep, OK);
          privep->txbusy = false;

          return OK;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: mpfs_read_rx_fifo
 *
 * Description:
 *   Read data from the RX fifo.
 *
 * Input Parameters:
 *   out_data    - Address to read the data
 *   length      - Amount of data to read
 *   epno        - The endpoint number
 *
 * Returned Value:
 *   Amount of data read
 *
 ****************************************************************************/

uint16_t mpfs_read_rx_fifo(uint8_t *out_data, uint32_t length, uint8_t epno)
{
  uint16_t count;
  uint32_t i;
  uint32_t *temp;
  uint8_t  *temp_8bit;
  uint16_t words;
  uint16_t bytes;
  uint16_t offset;

  if (epno == EP0)
    {
      /* Already read the count0 register, this is correct */

      count = length;
    }
  else
    {
      count = getreg16(MPFS_USB_INDEXED_CSR_RX_COUNT);
    }

  words = count / 4;
  bytes = count - words * 4;

  temp      = (uint32_t *)out_data;
  temp_8bit = out_data;

  if (count == 0)
    {
      /* Nothing to do */

      return  0;
    }

  /* 32-bit writes first */

  for (i = 0; i < words; i++)
    {
      temp[i] = getreg32(MPFS_USB_FIFO(epno));
    }

  offset = words << 2;

  /* ...and the remaining bytes last */

  for (i = offset; i < (offset + bytes); i++)
    {
      temp_8bit[i] = getreg8(MPFS_USB_FIFO(epno));
    }

  return count;
}

/****************************************************************************
 * Name: mpfs_req_read
 *
 * Description:
 *   Issue a read according to the request.
 *
 * Input Parameters:
 *   priv      - USB device abstraction
 *   privep    - Endpoint abstraction
 *   recvsize  - Size of the read
 *
 * Returned Value:
 *   OK on success, an error code in case of trouble
 *
 ****************************************************************************/

static int mpfs_req_read(struct mpfs_usbdev_s *priv,
                         struct mpfs_ep_s *privep, uint16_t recvsize)
{
  struct mpfs_req_s *privreq;
  uint16_t reg;
  int epno;

  DEBUGASSERT(priv && privep);

  /* Check the request from the head of the endpoint request queue */

  epno = USB_EPNO(privep->ep.eplog);

  reg = getreg16(MPFS_USB_ENDPOINT(epno) + MPFS_USB_ENDPOINT_RX_CSR_OFFSET);

  uint16_t count = getreg16(MPFS_USB_ENDPOINT(epno) +
                            MPFS_USB_ENDPOINT_RX_COUNT_OFFSET);

  usbtrace(TRACE_READ(USB_EPNO(epno)), count);

  do
    {
      /* Peek at the next read request in the request queue */

      privreq = mpfs_rqpeek(&privep->reqq);
      if (privreq == NULL)
        {
          /* When no read requests are pending no EP descriptors are set to
           * ready.
           */

          privep->epstate = USB_EPSTATE_RXSTOPPED;
          return OK;
        }

      uinfo("EP%d: req.len=%d xfrd=%d recvsize=%d\n",
            epno, privreq->req.len, privreq->req.xfrd, recvsize);

      /* Ignore any attempt to receive a zero length packet */

      if (privreq->req.len == 0)
        {
          usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_EPOUTNULLPACKET), 0);
          if (epno == EP0)
            {
              mpfs_putreg16(CSR0L_DEV_SERVICED_RX_PKT_RDY_MASK |
                            CSR0L_DEV_DATA_END_MASK,
                            MPFS_USB_INDEXED_CSR_EP0_CSR0);
            }
          else
            {
              mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                               MPFS_USB_ENDPOINT_RX_CSR_OFFSET,
                               RXCSRL_REG_EPN_RX_PKT_RDY_MASK, 0);
            }

          mpfs_req_complete(privep, OK);
          privreq = NULL;
        }

      if ((privreq->inflight > 0) && (count != 0) &&
          (reg & RXCSRL_REG_EPN_RX_PKT_RDY_MASK) != 0)
        {
          /* Update the total number of bytes transferred */

          mpfs_putreg8(epno, MPFS_USB_INDEX);
          privep->rxactive   = true;
          privreq->req.xfrd += mpfs_read_rx_fifo(privreq->req.buf,
                                                 recvsize,
                                                 epno);

          privreq->inflight  = 0;

          usbtrace(TRACE_COMPLETE(epno), privreq->req.xfrd);

          mpfs_req_complete(privep, OK);

          if (epno == EP0)
            {
              mpfs_putreg16(CSR0L_DEV_SERVICED_RX_PKT_RDY_MASK |
                            CSR0L_DEV_DATA_END_MASK,
                            MPFS_USB_INDEXED_CSR_EP0_CSR0);
            }
          else
            {
              mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                               MPFS_USB_ENDPOINT_RX_CSR_OFFSET,
                               RXCSRL_REG_EPN_RX_PKT_RDY_MASK, 0);
            }

          /* Need to set recvsize to zero.  When calling mpfs_req_complete()
           * class driver could call submit() again and we have new request
           * ready on next while() loop.
           */

          privep->rxactive = false;
          recvsize = 0;
          privreq = NULL;
        }
    }
  while (privreq == NULL);

  /* Activate new read request from queue */

  privep->rxactive  = true;
  privreq->req.xfrd = 0;
  privreq->inflight = privreq->req.len;
  priv->eplist[epno].descb[0]->addr = (uintptr_t)privreq->req.buf;

  return OK;
}

/****************************************************************************
 * Name: mpfs_ep_set_fifo_size
 *
 * Description:
 *   Sets the fifo size for the endpoint.
 *
 * Input Parameters:
 *   epno      - Endpoint number
 *   in        - Device to host (TX) fifo if set, RX fifo if unset
 *   fifo_size - Desired fifo size
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ep_set_fifo_size(uint8_t epno, uint8_t in,
                                  uint16_t fifo_size)
{
  uint16_t temp;
  uint8_t i = 0;

  DEBUGASSERT(epno > EP0);
  DEBUGASSERT(fifo_size > 8);

  DEBUGASSERT(fifo_size == 8 || fifo_size == 16 || fifo_size == 32 ||
              fifo_size == 64 || fifo_size == 128 || fifo_size == 512 ||
              fifo_size == 1024 || fifo_size == 2048 || fifo_size == 4096);

  mpfs_putreg8(epno, MPFS_USB_INDEX);

  temp = fifo_size / MPFS_MIN_EP_FIFO_SIZE;

  while ((temp & 0x01) == 0)
    {
      temp >>= 1;
      i++;
    }

  /* in: Device to host */

  if (in)
    {
      i |= (1 << 4); /* Double buffering */

      mpfs_putreg8(i, MPFS_USB_TX_FIFO_SIZE);
    }
  else
    {
      mpfs_putreg8(i, MPFS_USB_RX_FIFO_SIZE);
    }
}

/****************************************************************************
 * Name: mpfs_ep_configure_internal
 *
 * Description:
 *   This is the internal implementation of the endpoint configuration logic
 *   and implements the endpoint configuration method of the usbdev_ep_s
 *   interface.  As an internal interface, it will be used to configure
 *   endpoint 0 which is not available to the class implementation.
 *
 * Input Parameters:
 *   privep    - Endpoint abstraction
 *   desc      - USB descriptor
 *
 * Returned Value:
 *   OK always
 *
 ****************************************************************************/

static int mpfs_ep_configure_internal(struct mpfs_ep_s *privep,
                                      const struct usb_epdesc_s *desc)
{
  uint16_t maxpacket;
  uint8_t epno;
  uint8_t eptype;
  bool dirin;

  DEBUGASSERT(privep && privep->dev && desc);

  uinfo("len: 0x%02x type: 0x%02x addr: 0x%02x attr: 0x%02x "
        "maxpacketsize: 0x%02x%02x interval: 0x%02x\n",
        desc->len, desc->type, desc->addr, desc->attr,
        desc->mxpacketsize[1],  desc->mxpacketsize[0],
        desc->interval);

  /* Decode the endpoint descriptor */

  epno      = USB_EPNO(desc->addr);
  dirin     = (desc->addr & USB_DIR_MASK) == USB_REQ_DIR_IN;
  eptype    = (desc->attr & USB_EP_ATTR_XFERTYPE_MASK);
  maxpacket = GETUINT16(desc->mxpacketsize);

  /* update endpoint descriptors to correct size */

  privep->descb[0]->pktsize = maxpacket;
  privep->descb[1]->pktsize = maxpacket;

  /* Initialize the endpoint structure */

  privep->ep.eplog     = desc->addr;  /* Includes direction */
  privep->ep.maxpacket = maxpacket;
  privep->epstate      = USB_EPSTATE_IDLE;

  /* get current config IN and OUT */

  mpfs_putreg8(epno, MPFS_USB_INDEX);

  /* Re-configure and enable the endpoint */

  if (dirin && epno)
    {
      /* Clear previous config first */

      mpfs_putreg16(0, MPFS_USB_ENDPOINT(epno) +
                    MPFS_USB_ENDPOINT_TX_CSR_OFFSET);

      mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                       MPFS_USB_ENDPOINT_TX_CSR_OFFSET,
                       TXCSRL_REG_EPN_UNDERRUN_MASK |
                       TXCSRL_REG_EPN_SEND_STALL_MASK |
                       TXCSRL_REG_EPN_STALL_SENT_MASK,
                        0);

      mpfs_ep_set_fifo_size(epno, 0, maxpacket);

      mpfs_putreg16(desc->addr, MPFS_USB_TX_FIFO_ADDR);

      /* Disable double buffering */

      mpfs_modifyreg16(MPFS_USB_TX_DPBUF_DIS, 0, (1 << epno));

      /* Sequence for setting the max packet size */

      mpfs_putreg16(0, MPFS_USB_ENDPOINT(epno) +
                    MPFS_USB_ENDPOINT_TX_MAX_P_OFFSET);
      mpfs_putreg16(MPFS_NUM_USB_PKT - 1, MPFS_USB_ENDPOINT(epno) +
                    MPFS_USB_ENDPOINT_TX_MAX_P_OFFSET);
      mpfs_putreg16((MPFS_NUM_USB_PKT - 1) << TX_MAX_P_REG_NUM_USB_PKT_SHIFT,
             MPFS_USB_ENDPOINT(epno) + MPFS_USB_ENDPOINT_TX_MAX_P_OFFSET);

      mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                       MPFS_USB_ENDPOINT_TX_MAX_P_OFFSET, 0, maxpacket);

      /* Clear data toggle (by setting the bit, not clearing */

      mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                       MPFS_USB_ENDPOINT_TX_CSR_OFFSET,
                       0,
                       TXCSRL_REG_EPN_CLR_DATA_TOG_MASK);
    }
  else if (epno)
    {
      /* Clear previous config */

      mpfs_putreg16(0, MPFS_USB_ENDPOINT(epno) +
                    MPFS_USB_ENDPOINT_RX_CSR_OFFSET);

      mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                       MPFS_USB_ENDPOINT_RX_CSR_OFFSET,
                       RXCSRL_REG_EPN_OVERRUN_MASK |
                       RXCSRL_REG_EPN_STALL_SENT_MASK |
                       RXCSRL_REG_EPN_SEND_STALL_MASK,
                       RXCSRL_REG_EPN_RX_PKT_RDY_MASK);

      mpfs_ep_set_fifo_size(epno, dirin, maxpacket);
      mpfs_putreg16(desc->addr, MPFS_USB_RX_FIFO_ADDR);

      /* Disable double buffering for RX, will run into trouble with it.
       * The host will send faster than we can handle and all packets
       * are ACK:ed now, although we should NAK in such situations. We
       * cannot NACK in bulk mode now.
       */

      mpfs_modifyreg16(MPFS_USB_RX_DPBUF_DIS, 0,  (1 << epno));

      mpfs_putreg16(0, MPFS_USB_ENDPOINT(epno) +
                    MPFS_USB_ENDPOINT_RX_MAX_P_OFFSET);
      mpfs_putreg16(MPFS_NUM_USB_PKT - 1, MPFS_USB_ENDPOINT(epno) +
                    MPFS_USB_ENDPOINT_RX_MAX_P_OFFSET);
      mpfs_putreg16((MPFS_NUM_USB_PKT - 1) << RX_MAX_P_REG_NUM_USB_PKT_SHIFT,
                     MPFS_USB_ENDPOINT(epno) +
                     MPFS_USB_ENDPOINT_RX_MAX_P_OFFSET);
      mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                       MPFS_USB_ENDPOINT_RX_MAX_P_OFFSET, 0, maxpacket);

      mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                       MPFS_USB_ENDPOINT_RX_CSR_OFFSET,
                       0,
                       RXCSRL_REG_EPN_CLR_DAT_TOG_MASK |
                       RXCSRL_REG_EPN_RX_PKT_RDY_MASK);

      mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                       MPFS_USB_ENDPOINT_RX_CSR_OFFSET,
                       RXCSRL_REG_EPN_RX_PKT_RDY_MASK,
                       0);
    }

  switch (eptype)
    {
      case USB_EP_ATTR_XFER_CONTROL:
        mpfs_putreg16(0, MPFS_USB_INDEXED_CSR_EP0_CSR0);
        break;

      case USB_EP_ATTR_XFER_BULK:
        if (dirin)
          {
            mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                             MPFS_USB_ENDPOINT_TX_CSR_OFFSET,
                             TXCSRH_REG_EPN_ENABLE_ISO_MASK,
                             0);
          }
        else
          {
            /* RXCSRL_REG_EPN_BI_DIS_NYET_MASK is enabled when cleared */

            mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                             MPFS_USB_ENDPOINT_RX_CSR_OFFSET,
                             RXCSRL_REG_EPN_ENABLE_ISO_MASK,
                             RXCSRL_REG_EPN_ENABLE_AUTOCLR_MASK |
                             RXCSRL_REG_EPN_BI_DIS_NYET_MASK |
                             RXCSRL_REG_EPN_RX_PKT_RDY_MASK);
          }
        break;

      case USB_EP_ATTR_XFER_INT:
        if (dirin)
          {
            mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                             MPFS_USB_ENDPOINT_TX_CSR_OFFSET,
                             TXCSRH_REG_EPN_ENABLE_ISO_MASK |
                             TXCSRH_REG_EPN_ENABLE_AUTOSET_MASK,
                             0);
          }
        else
          {
            /* RXCSRL_REG_EPN_BI_DIS_NYET_MASK disabled when set */

            mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                             MPFS_USB_ENDPOINT_RX_CSR_OFFSET,
                             RXCSRL_REG_EPN_ENABLE_ISO_MASK |
                             RXCSRL_REG_EPN_BI_DIS_NYET_MASK,
                             RXCSRL_REG_EPN_ENABLE_AUTOCLR_MASK |
                             RXCSRL_REG_EPN_RX_PKT_RDY_MASK);
          }
        break;

      default:
        usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_BADEPTYPE), eptype);
        return -EINVAL;
    }

  /* Enable endpoint interrupts */

  if (dirin)
    {
      mpfs_modifyreg16(MPFS_USB_TX_IRQ_ENABLE, 0, (1 << epno));
    }
  else
    {
      mpfs_modifyreg16(MPFS_USB_RX_IRQ_ENABLE, 0, (1 << epno));
    }

  return OK;
}

/****************************************************************************
 * Name: mpfs_ep_configure
 *
 * Description:
 *   This is the endpoint configuration method of the usbdev_ep_s interface.
 *
 * Input Parameters:
 *   ep        - USB device abstraction
 *   desc      - USB descriptor
 *   last     -  Set if the endpoint is the last one
 *
 * Returned Value:
 *   OK on success, a negated error code otherwise
 *
 ****************************************************************************/

static int mpfs_ep_configure(struct usbdev_ep_s *ep,
                             const struct usb_epdesc_s *desc,
                             bool last)
{
  struct mpfs_ep_s *privep = (struct mpfs_ep_s *)ep;
  int ret;

  usbtrace(TRACE_EPCONFIGURE, USB_EPNO(desc->addr));

  ret = mpfs_ep_configure_internal(privep, desc);

  /* If this was the last endpoint, then the class driver is fully
   * configured.
   */

  if (ret == OK && last)
    {
      struct mpfs_usbdev_s *priv = privep->dev;

      /* Go to the configured state (we should have been in the addressed
       * state)
       */

      priv->devstate = USB_DEVSTATE_CONFIGURED;
    }

  return ret;
}

static void mpfs_ep_reset(struct mpfs_usbdev_s *priv, uint8_t epno)
{
  struct mpfs_ep_s *privep = &priv->eplist[epno];

  /* Disable endpoint interrupts */

  mpfs_modifyreg16(MPFS_USB_RX_IRQ_ENABLE, (1 << epno), 0);
  mpfs_modifyreg16(MPFS_USB_TX_IRQ_ENABLE, (1 << epno), 0);

  /* Cancel any queued requests.  Since they are cancelled with status
   * -ESHUTDOWN, then will not be requeued until the configuration is reset.
   * NOTE:  This should not be necessary... the CLASS_DISCONNECT above
   * should result in the class implementation calling mpfs_ep_disable
   * for each of its configured endpoints.
   */

  mpfs_req_cancel(privep, -ESHUTDOWN);

  /* Reset endpoint status */

  privep->epstate   = USB_EPSTATE_DISABLED;
  privep->stalled   = false;
  privep->pending   = false;
  privep->halted    = false;
  privep->zlpsent   = false;
  privep->txbusy    = false;
  privep->rxactive  = false;
}

/****************************************************************************
 * Name: mpfs_setdevaddr
 *
 * Description:
 *   This function is called after the completion of the STATUS phase to
 *   instantiate the device address that was received during the SETUP
 *   phase.  This enters the ADDRESSED state from either the DEFAULT or the
 *   CONFIGURED states.
 *
 *   If called with address == 0, then function will revert to the DEFAULT,
 *   un-configured and un-addressed state.
 *
 * Input Parameters:
 *   priv        - USB device abstraction
 *   address     - Address to be set (or cleared)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_setdevaddr(struct mpfs_usbdev_s *priv, uint8_t address)
{
  uinfo("ENTRY address=0x%x\n", address);

  DEBUGASSERT(address <= 0x7f);

  mpfs_putreg8(address, MPFS_USB_FADDR);

  if (address != 0)
    {
      priv->devstate = USB_DEVSTATE_ADDRESSED;
    }
  else
    {
      /* Revert to the un-addressed, default state */

      priv->devstate = USB_DEVSTATE_DEFAULT;
    }
}

/****************************************************************************
 * Name: mpfs_ep_disable
 *
 * Description:
 *   This is the disable() method of the USB device endpoint structure.
 *
 * Input Parameters:
 *   ep        - USB device endpoint
 *
 * Returned Value:
 *   OK always
 *
 ****************************************************************************/

static int mpfs_ep_disable(struct usbdev_ep_s *ep)
{
  struct mpfs_ep_s *privep = (struct mpfs_ep_s *)ep;
  struct mpfs_usbdev_s *priv;
  irqstate_t flags;
  uint8_t epno;

  epno = USB_EPNO(ep->eplog);

  /* Reset the endpoint and cancel any ongoing activity */

  flags = enter_critical_section();
  priv  = privep->dev;
  mpfs_ep_reset(priv, epno);

  /* Revert to the addressed-but-not-configured state */

  mpfs_setdevaddr(priv, priv->devaddr);
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: mpfs_ep_allocreq
 *
 * Description:
 *   This is the allocreq() method of the USB device endpoint structure.
 *
 * Input Parameters:
 *   ep        - USB device endpoint (unused)
 *
 * Returned Value:
 *   New allocated request struct or NULL if no mempry
 *
 ****************************************************************************/

static struct usbdev_req_s *mpfs_ep_allocreq(struct usbdev_ep_s *ep)
{
  struct mpfs_req_s *privreq;

  privreq = (struct mpfs_req_s *)kmm_malloc(sizeof(struct mpfs_req_s));
  if (privreq == NULL)
    {
      usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct mpfs_req_s));
  return &privreq->req;
}

/****************************************************************************
 * Name: mpfs_ep_freereq
 *
 * Description:
 *   This is the freereq() method of the USB device endpoint structure. This
 *   frees the addressed request.
 *
 * Input Parameters:
 *   ep        - USB device endpoint
 *   req       - Request to free
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ep_freereq(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct mpfs_req_s *privreq = (struct mpfs_req_s *)req;

  kmm_free(privreq);
}

/****************************************************************************
 * Name: mpfs_ep_allocbuffer
 *
 * Description:
 *   This is the allocbuffer() method of the USB device endpoint structure.
 *
 * Input Parameters:
 *   ep        - USB device endpoint
 *   nbytes    - Number of bytes to allocate
 *
 * Returned Value:
 *   Pointer to the data or NULL if no memory
 *
 * Assumptions/Limitations:
 *   DMA is not supported yet
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void *mpfs_ep_allocbuffer(struct usbdev_ep_s *ep, uint16_t nbytes)
{
  /* There is not special buffer allocation requirement */

  return kumm_malloc(nbytes);
}
#endif

/****************************************************************************
 * Name: mpfs_ep_freebuffer
 *
 * Description:
 *   This is the freebuffer() method of the USB device endpoint structure.
 *
 * Input Parameters:
 *   ep        - USB device endpoint
 *   buf       - Pointer to the data
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   DMA is not supported yet
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void mpfs_ep_freebuffer(struct usbdev_ep_s *ep, void *buf)
{
  /* There is not special buffer allocation requirement */

  kumm_free(buf);
}
#endif

/****************************************************************************
 * Name: mpfs_ep_submit
 *
 * Description:
 *   This is the submit() method of the USB device endpoint structure.
 *
 * Input Parameters:
 *   ep        - USB device endpoint
 *   req       - The request to be submitted
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

static int mpfs_ep_submit(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct mpfs_req_s *privreq = (struct mpfs_req_s *)req;
  struct mpfs_ep_s *privep = (struct mpfs_ep_s *)ep;
  struct mpfs_usbdev_s *priv;
  irqstate_t flags;
  uint8_t epno;
  int ret = OK;

  usbtrace(TRACE_EPSUBMIT, USB_EPNO(ep->eplog));

  priv = privep->dev;

  /* Handle the request from the class driver */

  epno              = USB_EPNO(ep->eplog);
  req->result       = -EINPROGRESS;
  req->xfrd         = 0;
  privreq->inflight = 0;
  flags             = enter_critical_section();

  /* Handle IN (device-to-host) requests.  NOTE:  If the class device is
   * using the bi-directional EP0, then we assume that they intend the EP0
   * IN functionality (EP0 SETUP OUT data receipt does not use requests).
   */

  if (USB_ISEPIN(ep->eplog) || epno == EP0)
    {
      /* Check if the endpoint is stalled (or there is a stall pending) */

      if (privep->stalled || privep->pending)
        {
          mpfs_req_enqueue(&privep->pendq, privreq);
          usbtrace(TRACE_INREQQUEUED(epno), req->len);

          ret = OK;
        }

      else
        {
          /* Add the new request to the request queue for the IN endpoint */

          mpfs_req_enqueue(&privep->reqq, privreq);
          usbtrace(TRACE_INREQQUEUED(epno), req->len);

          /* If the IN endpoint is IDLE and there is not write queue
           * processing in progress, then transfer the data now.
           */

          if (privep->epstate == USB_EPSTATE_IDLE && !privep->txbusy)
            {
              ret = mpfs_req_write(priv, privep);
            }
        }
    }

  /* Handle OUT (host-to-device) requests */

  else
    {
      /* Add the new request to the request queue for the OUT endpoint */

      mpfs_req_enqueue(&privep->reqq, privreq);
      usbtrace(TRACE_OUTREQQUEUED(epno), req->len);

      /* Check if we have stopped RX receipt due to lack of read
       * requests.  In that case we are not receiving anything from host.
       * and HW sends NAK to host. see mpfs_req_read()
       * so this "state" is actually not required (at least yet)
       */

      if (privep->epstate == USB_EPSTATE_RXSTOPPED)
        {
          privep->epstate = USB_EPSTATE_IDLE;
          privreq->inflight = privreq->req.len;
          privep->rxactive  = true;
          privreq->req.xfrd = 0;
          ret = mpfs_req_read(priv, privep, req->len);
        }

      /* start new read if no active yet */

      else if (!privep->rxactive)
        {
          ret = mpfs_req_read(priv, privep, req->len);
        }
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: mpfs_ep_cancel
 *
 * Description:
 *   This is the cancel() method of the USB device endpoint structure.
 *
 * Input Parameters:
 *   ep        - USB device endpoint
 *   req       - The request to be canceled
 *
 * Returned Value:
 *   OK unconditionally
 *
 ****************************************************************************/

static int mpfs_ep_cancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct mpfs_ep_s *privep = (struct mpfs_ep_s *)ep;
  irqstate_t flags;

  usbtrace(TRACE_EPCANCEL, USB_EPNO(ep->eplog));

  flags = enter_critical_section();
  mpfs_req_cancel(privep, -EAGAIN);
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: mpfs_ep_resume
 *
 * Description:
 *   This is the resume() method.
 *
 * Input Parameters:
 *   privep    - Endpoint abstraction
 *
 * Returned Value:
 *   OK unconditionally
 *
 ****************************************************************************/

static int mpfs_ep_resume(struct mpfs_ep_s *privep)
{
  struct mpfs_usbdev_s *priv;
  struct mpfs_req_s *req;
  irqstate_t flags;
  uint8_t epno;

  /* Check that endpoint is in Idle state */

  DEBUGASSERT(privep->dev);

  usbtrace(TRACE_EPRESUME, USB_EPNO(privep->ep.eplog));

  flags = enter_critical_section();

  /* Check if the endpoint is stalled */

  if (privep->epstate == USB_EPSTATE_STALLED)
    {
      epno = USB_EPNO(privep->ep.eplog);

      priv = (struct mpfs_usbdev_s *)privep->dev;

      /* Return endpoint to Idle state */

      privep->stalled = false;
      privep->pending = false;
      privep->epstate = USB_EPSTATE_IDLE;

      /* Clear STALLRQx request and reset data toggle if needed */

      if (epno == EP0)
        {
          mpfs_modifyreg16(MPFS_USB_INDEXED_CSR_EP0_CSR0,
                           CSR0L_DEV_STALL_SENT_MASK,
                           0);
        }
      else if (USB_ISEPIN(privep->ep.eplog))
        {
          mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                      MPFS_USB_ENDPOINT_TX_CSR_OFFSET,
                      TXCSRL_REG_EPN_SEND_STALL_MASK,
                      0);
        }
      else
        {
          mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                      MPFS_USB_ENDPOINT_RX_CSR_OFFSET,
                      RXCSRL_REG_EPN_SEND_STALL_MASK,
                      RXCSRL_REG_EPN_RX_PKT_RDY_MASK);
          mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                      MPFS_USB_ENDPOINT_RX_CSR_OFFSET,
                      0,
                      RXCSRL_REG_EPN_CLR_DAT_TOG_MASK |
                      RXCSRL_REG_EPN_RX_PKT_RDY_MASK);
        }

      /* Copy any requests in the pending request queue to the working
       * request queue.
       */

      while ((req = mpfs_req_dequeue(&privep->pendq)) != NULL)
        {
          mpfs_req_enqueue(&privep->reqq, req);
        }

      /* Resuming any blocked data transfers on the endpoint */

      if (epno == 0 || USB_ISEPIN(privep->ep.eplog))
        {
          /* IN endpoint (or EP0).  Restart any queued write requests */

          mpfs_req_write(priv, privep);
        }
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: mpfs_ep_stallresume
 *
 * Description:
 *   This is the stallresume() method.
 *
 * Input Parameters:
 *   ep      - Endpoint abstraction
 *   resume  - Resume (or stall)
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

static int mpfs_ep_stallresume(struct usbdev_ep_s *ep, bool resume)
{
  struct mpfs_ep_s *privep;
  uint8_t epno;
  irqstate_t flags;
  int ret;

  /* Handle the resume condition */

  privep = (struct mpfs_ep_s *)ep;
  if (resume)
    {
      ret = mpfs_ep_resume(privep);
    }

  /* Handle the stall condition */

  else
    {
      /* If this is an IN endpoint (and not EP0) and if there are queued
       * write requests, then we cannot stall now.  Perhaps this is a
       * protocol stall.  In that case, we will need to drain the write
       * requests before sending the stall.
       */

      flags = enter_critical_section();
      epno = USB_EPNO(ep->eplog);
      if (epno != 0 && USB_ISEPIN(ep->eplog))
        {
          /* Are there any unfinished write requests in the request
           * queue?
           */

          if (!mpfs_rqempty(&privep->reqq))
            {
              /* Just set a flag to indicate that the endpoint must be
               * stalled on the next TRCPTX interrupt when the request
               * queue becomes empty.
               */

              privep->pending = true;
              leave_critical_section(flags);
              return OK;
            }
        }

      /* Not an IN endpoint, endpoint 0, or no pending write requests.
       * Stall the endpoint now.
       */

      ret = mpfs_ep_stall(privep);
      leave_critical_section(flags);
    }

  return ret;
}

/****************************************************************************
 * Device Controller Operations
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_ep_reserve
 *
 * Description:
 *   Find and un-reserved endpoint number and reserve it for the caller.
 *
 * Input Parameters:
 *   priv      - USB device abstraction
 *   epset     - Set of bits, one bit reflects one endpoint
 *
 * Returned Value:
 *   Endpoint structure or NULL in case of error
 *
 ****************************************************************************/

static inline struct mpfs_ep_s *
mpfs_ep_reserve(struct mpfs_usbdev_s *priv, uint8_t epset)
{
  struct mpfs_ep_s *privep = NULL;
  irqstate_t flags;
  int epndx = 0;

  flags  = enter_critical_section();
  epset &= priv->epavail;
  if (epset != 0)
    {
      /* Select the lowest bit in the set of matching, available endpoints
       * (skipping EP0)
       */

      for (epndx = 1; epndx < MPFS_USB_NENDPOINTS; epndx++)
        {
          uint8_t bit = MPFS_EP_BIT(epndx);
          if ((epset & bit) != 0)
            {
              /* Mark the endpoint no longer available */

              priv->epavail &= ~bit;

              /* And return the pointer to the standard endpoint
               * structure.
               */

              privep = &priv->eplist[epndx];
              break;
            }
        }
    }

  leave_critical_section(flags);
  return privep;
}

/****************************************************************************
 * Name: mpfs_allocep
 *
 * Description:
 *   This is the allocep() method of the USB device driver interface
 *
 * Input Parameters:
 *   dev       - USB device abstraction
 *   epno      - Endpoint number
 *   in        - Direction
 *   eptype    - EP type, unused
 *
 * Returned Value:
 *   USB device endpoint structure or NULL in case of error
 *
 ****************************************************************************/

static struct usbdev_ep_s *mpfs_allocep(struct usbdev_s *dev, uint8_t epno,
                                        bool in, uint8_t eptype)
{
  struct mpfs_usbdev_s *priv = (struct mpfs_usbdev_s *)dev;
  struct mpfs_ep_s *privep = NULL;
  uint16_t epset = MPFS_EPSET_NOTEP0;

  /* Ignore any direction bits in the logical address */

  epno = USB_EPNO(epno);

  /* A logical address of 0 means that any endpoint will do */

  if (epno > 0)
    {
      /* Otherwise, we will return the endpoint structure only for the
       * requested 'logical' endpoint.  All of the other checks will still
       * be performed.
       */

      if (epno >= MPFS_USB_NENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_BADEPNO), (uint16_t)epno);
          return NULL;
        }

      /* Convert the logical address to a physical OUT endpoint address and
       * remove all of the candidate endpoints from the bitset except for the
       * the IN/OUT pair for this logical address.
       */

      epset = MPFS_EP_BIT(epno);
    }

  /* Check if the selected endpoint number is available */

  privep = mpfs_ep_reserve(priv, epset);
  if (privep == NULL)
    {
      return NULL;
    }

  return &privep->ep;
}

/****************************************************************************
 * Name: mpfs_ep_unreserve
 *
 * Description:
 *   The endpoint is no longer in use.  It will be unreserved and can be
 *   re-used if needed.
 *
 * Input Parameters:
 *   priv       - USB device abstraction
 *   privep     - Endpoint private struct
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void
mpfs_ep_unreserve(struct mpfs_usbdev_s *priv, struct mpfs_ep_s *privep)
{
  irqstate_t flags = enter_critical_section();
  priv->epavail   |= MPFS_EP_BIT(USB_EPNO(privep->ep.eplog));
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: mpfs_freeep
 *
 * Description:
 *   This is the freeep() method of the USB device driver interface
 *
 * Input Parameters:
 *   dev        - USB device abstraction
 *   ep         - Endpoint private struct
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep)
{
  struct mpfs_usbdev_s *priv;
  struct mpfs_ep_s *privep;

  priv   = (struct mpfs_usbdev_s *)dev;
  privep = (struct mpfs_ep_s *)ep;

  if (priv != NULL && privep != NULL)
    {
      /* Mark the endpoint as available */

      mpfs_ep_unreserve(priv, privep);
    }
}

/****************************************************************************
 * Name: mpfs_getframe
 *
 * Description:
 *   This is the getframe() method of the USB device driver interface
 *
 * Input Parameters:
 *   dev        - USB device abstraction
 *
 * Returned Value:
 *   Last known frame number
 *
 ****************************************************************************/

static int mpfs_getframe(struct usbdev_s *dev)
{
  uint16_t frameno;

  /* Return the last frame number detected by the hardware */

  frameno = getreg16(MPFS_USB_FRAME);

  return frameno;
}

void mpfs_usb_suspend(struct usbdev_s *dev, bool resume)
{
}

/****************************************************************************
 * Name: mpfs_resume
 *
 * Description:
 *   Resumes action in contrast to suspend.
 *
 * Input Parameters:
 *   priv        - USB device abstraction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_resume(struct mpfs_usbdev_s *priv)
{
  /* This function is called when either (1) a WAKEUP interrupt is received
   * from the host PC, or (2) the class device implementation calls the
   * wakeup() method.
   */

  /* Don't do anything if the device was not suspended */

  if (priv->devstate == USB_DEVSTATE_SUSPENDED)
    {
      /* Revert to the previous state */

      priv->devstate = priv->prevstate;

      /* Restore clocking to the USB peripheral */

      mpfs_enableclk();

      /* Restore full power -- whatever that means for this
       * particular board
       */

      mpfs_usb_suspend((struct usbdev_s *)priv, true);

      /* Notify the class driver of the resume event */

      if (priv->driver)
        {
          CLASS_RESUME(priv->driver, &priv->usbdev);
        }
    }
}

/****************************************************************************
 * Name: mpfs_wakeup
 *
 * Description:
 *   This is the wakeup() method of the USB device driver interface
 *
 * Input Parameters:
 *   dev        - USB device abstraction
 *
 * Returned Value:
 *   OK unconditionally
 *
 ****************************************************************************/

static int mpfs_wakeup(struct usbdev_s *dev)
{
  struct mpfs_usbdev_s *priv = (struct mpfs_usbdev_s *)dev;
  irqstate_t flags;

  /* Resume normal operation */

  flags = enter_critical_section();

  mpfs_resume(priv);

  /* Device is always self-powered. Remote wakeup is not supported */

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: mpfs_selfpowered
 *
 * Description:
 *   This is the selfpowered() method of the USB device driver interface
 *
 * Input Parameters:
 *   dev          - USB device abstraction
 *   selfpowered  - Selfpowered if set true
 *
 * Returned Value:
 *   OK unconditionally
 *
 ****************************************************************************/

static int mpfs_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct mpfs_usbdev_s *priv = (struct mpfs_usbdev_s *)dev;

  priv->selfpowered = selfpowered;

  return OK;
}

/****************************************************************************
 * Name: mpfs_reset
 *
 * Description:
 *   This resets the USB state but doesn't reset the hardware itself
 *
 * Input Parameters:
 *   priv    - USB device abstraction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_reset(struct mpfs_usbdev_s *priv)
{
  uint8_t epno;

  /* Tell the class driver that we are disconnected.  The class driver
   * should then accept any new configurations.
   */

  CLASS_DISCONNECT(priv->driver, &priv->usbdev);

  /* The device enters the default state (un-addressed and un-configured) */

  priv->devaddr  = 0;
  mpfs_setdevaddr(priv, 0);

  priv->devstate = USB_DEVSTATE_DEFAULT;

  /* Reset and disable all endpoints. Then re-configure EP0 */

  mpfs_putreg16(0, MPFS_USB_RX_IRQ_ENABLE);
  mpfs_putreg16(0, MPFS_USB_TX_IRQ_ENABLE);

  mpfs_epset_reset(priv, MPFS_EPSET_ALL);
  mpfs_ep_configure_internal(&priv->eplist[EP0], &g_ep0desc);

  /* Set EP0 waiting for SETUP */

  mpfs_ep0_ctrlread(priv);

  /* Reset endpoint data structures */

  for (epno = 0; epno < MPFS_USB_NENDPOINTS; epno++)
    {
      struct mpfs_ep_s *privep = &priv->eplist[epno];

      /* Cancel any queued requests.  Since they are cancelled
       * with status -ESHUTDOWN, then will not be requeued
       * until the configuration is reset.  NOTE:  This should
       * not be necessary... the CLASS_DISCONNECT above should
       * result in the class implementation calling mpfs_ep_disable
       * for each of its configured endpoints.
       */

      mpfs_req_cancel(privep, -ESHUTDOWN);

      /* Reset endpoint status */

      privep->stalled   = false;
      privep->pending   = false;
      privep->halted    = false;
      privep->zlpsent   = false;
      privep->txbusy    = false;
      privep->rxactive  = false;
    }

  /* Re-configure the USB controller in its initial, unconnected state */

#ifdef CONFIG_USBDEV_DUALSPEED
  priv->usbdev.speed = USB_SPEED_HIGH;
  priv->usbdev.dualspeed = 0;
#else
  priv->usbdev.speed = USB_SPEED_FULL;
  priv->usbdev.dualspeed = 1;
#endif

  /* Clear all pending interrupt statuses */

  mpfs_putreg8(0, MPFS_USB_INDEX);
  mpfs_putreg8(0, MPFS_USB_IRQ);

  mpfs_putreg16(0, MPFS_USB_INDEXED_CSR_EP0_CSR0);
  mpfs_putreg16(1, MPFS_USB_TX_IRQ_ENABLE);
}

/****************************************************************************
 * Name: mpfs_ep0_wrstatus
 *
 * Description:
 *   Writes the ep0 status reply back to the host.
 *
 * Input Parameters:
 *   priv    - USB device abstraction
 *   buffer  - Data buffer
 *   buflen  - Size of the data buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ep0_wrstatus(struct mpfs_usbdev_s *priv,
                             const uint8_t *buffer, size_t buflen)
{
  struct mpfs_ep_s *privep;
  privep = &priv->eplist[EP0];

  /* We need to make copy of data as source is in stack
   * reusing the static ep0 setup buffer
   */

  DEBUGASSERT(buflen <= MPFS_EP0_MAXPACKET);
  memcpy(&priv->ep0out[0], buffer, buflen);

  /* Setup TX transfer */

  priv->eplist[EP0].descb[1]->addr = (uintptr_t) &priv->ep0out[0];

  if (buflen > 0)
    {
      DEBUGASSERT(privep->epstate != USB_EPSTATE_SENDING);
      mpfs_write_tx_fifo(buffer, buflen, 0);

      mpfs_putreg16(CSR0L_DEV_TX_PKT_RDY_MASK | CSR0L_DEV_DATA_END_MASK,
                    MPFS_USB_INDEXED_CSR_EP0_CSR0);
    }
  else
    {
      mpfs_putreg16(CSR0L_DEV_SERVICED_RX_PKT_RDY_MASK |
                    CSR0L_DEV_DATA_END_MASK,
                    MPFS_USB_INDEXED_CSR_EP0_CSR0);
    }

  /* set read for next setup OUT */

  mpfs_ep0_ctrlread(priv);
}

/****************************************************************************
 * Name: mpfs_ep0_dispatch
 *
 * Description:
 *   Writes the ep0 status reply back to the host.
 *
 * Input Parameters:
 *   priv    - USB device abstraction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ep0_dispatch(struct mpfs_usbdev_s *priv)
{
  uint8_t *dataout;
  size_t outlen;
  int ret;

  usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_DISPATCH), 0);

  if (priv && priv->driver)
    {
      /* Assume IN SETUP (or OUT SETUP with no data) */

      dataout = NULL;
      outlen  = 0;

      /* Was this an OUT SETUP command? */

      if (USB_REQ_ISOUT(priv->ctrl.type))
        {
          uint16_t tmplen = GETUINT16(priv->ctrl.len);
          if (tmplen > 0)
            {
              dataout = priv->ep0out;
              outlen  = tmplen;
            }
        }

      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, &priv->ctrl,
                        dataout, outlen);
      if (ret < 0)
        {
          /* Stall on failure */

          usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_DISPATCHSTALL), 0);
          mpfs_ep_stall(&priv->eplist[EP0]);
        }
    }
}

/****************************************************************************
 * Name: mpfs_ep0_setup
 *
 * Description:
 *   This function is called after the receiving of the SETUP packet
 *   data is ready on usb_ctrlreq_s struct.
 *
 * Input Parameters:
 *   priv    - USB device abstraction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ep0_setup(struct mpfs_usbdev_s *priv)
{
  struct mpfs_ep_s      *ep0 = &priv->eplist[EP0];
  struct mpfs_ep_s      *privep;
  union wb_u            value;
  union wb_u            index;
  union wb_u            len;
  union wb_u            response;
  enum mpfs_ep0setup_e  ep0result;
  uint8_t               epno;
  int                   nbytes = 0; /* Assume zero-length packet */
  int                   ret;

  /* Terminate any pending requests */

  mpfs_req_cancel(ep0, -EPROTO);

  /* Assume NOT stalled; no TX in progress */

  ep0->stalled  = false;
  ep0->pending  = false;
  ep0->epstate  = USB_EPSTATE_IDLE;

  /* And extract the little-endian 16-bit values to host order */

  value.w = GETUINT16(priv->ctrl.value);
  index.w = GETUINT16(priv->ctrl.index);
  len.w   = GETUINT16(priv->ctrl.len);

  uinfo("SETUP: type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        priv->ctrl.type, priv->ctrl.req, value.w, index.w, len.w);

  /* Dispatch any non-standard requests */

  if ((priv->ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      /* Let the class implementation handle all non-standard requests */

      mpfs_ep0_dispatch(priv);
      return;
    }

  /* Handle standard request.  Pick off the things of interest to the
   * USB device controller driver; pass what is left to the class driver
   */

  ep0result = USB_EP0SETUP_SUCCESS;

  switch (priv->ctrl.req)
    {
    case USB_REQ_GETSTATUS:
      {
        /* type:  device-to-host; recipient = device, interface, endpoint
         * value: 0
         * index: zero interface endpoint
         * len:   2; data = status
         */

        usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_GETSTATUS),
                 priv->ctrl.type);
        if (len.w != 2 || (priv->ctrl.type & USB_REQ_DIR_IN) == 0 ||
            index.b[MSB] != 0 || value.w != 0)
          {
            usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_BADEPGETSTATUS), 0);
            ep0result = USB_EP0SETUP_STALL;
          }
        else
          {
            switch (priv->ctrl.type & USB_REQ_RECIPIENT_MASK)
              {
               case USB_REQ_RECIPIENT_ENDPOINT:
                {
                  epno = USB_EPNO(index.b[LSB]);
                  usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_GETSTATUS), epno);
                  if (epno >= MPFS_USB_NENDPOINTS)
                    {
                      usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_BADEPGETSTATUS),
                              epno);
                      ep0result = USB_EP0SETUP_STALL;
                    }
                  else
                    {
                      privep     = &priv->eplist[epno];
                      response.w = 0; /* Not stalled */
                      nbytes     = 2; /* Response size: 2 bytes */

                      if (privep->stalled)
                        {
                          /* Endpoint stalled */

                          response.b[LSB] = 1; /* Stalled */
                        }
                    }
                }
                break;

              case USB_REQ_RECIPIENT_DEVICE:
                {
                 if (index.w == 0)
                    {
                      usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_DEVGETSTATUS),
                               0);

                      /* Features:  Remote Wakeup=YES; selfpowered=? */

                      response.w      = 0;
                      response.b[LSB] = (priv->selfpowered <<
                                         USB_FEATURE_SELFPOWERED) |
                                        (1 << USB_FEATURE_REMOTEWAKEUP);
                      nbytes          = 2; /* Response size: 2 bytes */
                    }
                  else
                    {
                      usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_BADDEVGETSTATUS),
                               0);
                      ep0result = USB_EP0SETUP_STALL;
                    }
                }
                break;

              case USB_REQ_RECIPIENT_INTERFACE:
                {
                  usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_IFGETSTATUS), 0);
                  response.w = 0;
                  nbytes     = 2; /* Response size: 2 bytes */
                }
                break;

              default:
                {
                  usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_BADGETSTATUS), 0);
                  ep0result = USB_EP0SETUP_STALL;
                }
                break;
              }
          }
      }
      break;

    case USB_REQ_CLEARFEATURE:
      {
        /* type:  host-to-device; recipient = device, interface or endpoint
         * value: feature selector
         * index: zero interface endpoint;
         * len:   zero, data = none
         */

        usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_CLEARFEATURE),
                                 priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) !=
                               USB_REQ_RECIPIENT_ENDPOINT)
          {
            /* Let the class implementation handle all
             * recipients (except for the
             * endpoint recipient)
             */

            mpfs_ep0_dispatch(priv);
            ep0result = USB_EP0SETUP_DISPATCHED;
          }
        else
          {
            /* Endpoint recipient */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < MPFS_USB_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = false;

                ret = mpfs_ep_resume(privep);
                if (ret < 0)
                  {
                    ep0result = USB_EP0SETUP_STALL;
                  }
              }
            else
              {
                usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_BADCLEARFEATURE), 0);
                ep0result = USB_EP0SETUP_STALL;
              }
          }
      }
      break;

    case USB_REQ_SETFEATURE:
      {
        /* type:  host-to-device; recipient = device, interface, endpoint
         * value: feature selector
         * index: zero interface endpoint;
         * len:   0; data = none
         */

        usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_SETFEATURE),
                                 priv->ctrl.type);
        if (((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
                                USB_REQ_RECIPIENT_DEVICE) &&
            value.w == USB_FEATURE_TESTMODE)
          {
            /* Special case recipient=device test mode */

            uinfo("test mode: %d\n", index.w);
          }
        else if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) !=
                 USB_REQ_RECIPIENT_ENDPOINT)
          {
           /* The class driver handles all recipients
            * except recipient=endpoint
            */

            mpfs_ep0_dispatch(priv);
            ep0result = USB_EP0SETUP_DISPATCHED;
          }
        else
          {
            /* Handler recipient=endpoint */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < MPFS_USB_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = true;

                ret = mpfs_ep_stall(privep);
                if (ret < 0)
                  {
                    ep0result = USB_EP0SETUP_STALL;
                  }
              }
            else
              {
                usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_BADSETFEATURE), 0);
                ep0result = USB_EP0SETUP_STALL;
              }
          }
      }
      break;

    case USB_REQ_SETADDRESS:
      {
        /* type:  host-to-device; recipient = device
         * value: device address
         * index: 0
         * len:   0; data = none
         */

        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) !=
            USB_REQ_RECIPIENT_DEVICE ||
            index.w != 0 || len.w != 0 || value.w > 127)
          {
            usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_BADSETADDRESS), 0);
            ep0result = USB_EP0SETUP_STALL;
          }
        else
          {
            /* Note that setting of the device address will be deferred.
             * A zero-length packet will be sent and the device address will
             * be set when the zero-length packet transfer completes.
             */

            usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_EP0SETUPSETADDRESS),
                     value.w);

            priv->devaddr = value.w;
            ep0result     = USB_EP0SETUP_ADDRESS;
          }
      }
      break;

    case USB_REQ_GETDESCRIPTOR:
      /* type:  device-to-host; recipient = device
       * value: descriptor type and index
       * index: 0 or language ID;
       * len:   descriptor len; data = descriptor
       */

    case USB_REQ_SETDESCRIPTOR:
      /* type:  host-to-device; recipient = device
       * value: descriptor type and index
       * index: 0 or language ID;
       * len:   descriptor len; data = descriptor
       */

      {
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
            USB_REQ_RECIPIENT_DEVICE)
          {
            /* The request seems valid... let the class implementation
             * handle it.
             */

            mpfs_ep0_dispatch(priv);
            ep0result = USB_EP0SETUP_DISPATCHED;
          }
        else
          {
            ep0result = USB_EP0SETUP_STALL;
          }
      }
      break;

    case USB_REQ_GETCONFIGURATION:
      /* type:  device-to-host; recipient = device
       * value: 0;
       * index: 0;
       * len:   1; data = configuration value
       */

      {
        usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_GETCONFIG),
                 priv->ctrl.type);

        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
            USB_REQ_RECIPIENT_DEVICE &&
            value.w == 0 && index.w == 0 && len.w == 1)
          {
            /* The request seems valid... let the class implementation
             * handle it.
             */

            mpfs_ep0_dispatch(priv);
            ep0result = USB_EP0SETUP_DISPATCHED;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_BADGETCONFIG), 0);
            ep0result = USB_EP0SETUP_STALL;
          }
      }
      break;

    case USB_REQ_SETCONFIGURATION:
      /* type:  host-to-device; recipient = device
       * value: configuration value
       * index: 0;
       * len:   0; data = none
       */

      {
        usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_SETCONFIG),
                 priv->ctrl.type);

        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
            USB_REQ_RECIPIENT_DEVICE &&
            index.w == 0 && len.w == 0)
          {
            /* The request seems valid... let the class implementation
             * handle it.  If the class implementation accepts it new
             * configuration, it will call mpfs_ep_configure() to configure
             * the endpoints.
             */

            mpfs_ep0_dispatch(priv);
            ep0result = USB_EP0SETUP_DISPATCHED;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_BADSETCONFIG), 0);
            ep0result = USB_EP0SETUP_STALL;
          }
      }
      break;

    case USB_REQ_GETINTERFACE:
      /* type:  device-to-host; recipient = interface
       * value: 0
       * index: interface;
       * len:   1; data = alt interface
       */

    case USB_REQ_SETINTERFACE:
      /* type:  host-to-device; recipient = interface
       * value: alternate setting
       * index: interface;
       * len:   0; data = none
       */

      {
        /* Let the class implementation handle the request */

        usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_GETSETIF), priv->ctrl.type);
        mpfs_ep0_dispatch(priv);
        ep0result = USB_EP0SETUP_DISPATCHED;
      }
      break;

    case USB_REQ_SYNCHFRAME:
      /* type:  device-to-host; recipient = endpoint
       * value: 0
       * index: endpoint;
       * len:   2; data = frame number
       */

      {
        usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_SYNCHFRAME), 0);
      }
      break;

    default:
      {
        usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_INVALIDCTRLREQ),
                 priv->ctrl.req);
        ep0result = USB_EP0SETUP_STALL;
      }
      break;
    }

  /* Restrict the data length to the length requested in the setup packet */

  if (nbytes > len.w)
    {
      nbytes = len.w;
    }

  switch (ep0result)
    {
      case USB_EP0SETUP_SUCCESS:
        {
          /* Send the response (might be a zero-length packet) */

          ep0->epstate = USB_EPSTATE_EP0STATUSIN;
          mpfs_ep0_wrstatus(priv, response.b, nbytes);
        }
        break;

      case USB_EP0SETUP_ADDRESS:
        {
          /* Send the response (might be a zero-length packet) */

          ep0->epstate = USB_EPSTATE_EP0ADDRESS;
          mpfs_ep0_wrstatus(priv, response.b, nbytes);
        }
        break;

      case USB_EP0SETUP_STALL:
        {
          /* Stall EP0 */

          usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_EP0SETUPSTALLED),
                   priv->ctrl.req);

          mpfs_ep_stall(&priv->eplist[EP0]);
        }
        break;

      case USB_EP0SETUP_DISPATCHED:
      default:
        break;
    }
}

/****************************************************************************
 * Name: mpfs_ep0_ctrlread
 *
 * Description:
 *   Prepare 8-byte read req for ep0-ctrl setup phase.
 *   data is received on priv->ep0out buffer
 *
 * Input Parameters:
 *   priv    - USB device abstraction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ep0_ctrlread(struct mpfs_usbdev_s *priv)
{
  priv->eplist[EP0].descb[0]->addr    = (uintptr_t)&priv->ep0out[0];
  priv->eplist[EP0].descb[0]->pktsize = 8;
}

/****************************************************************************
 * Name: mpfs_ep_rx_interrupt
 *
 * Description:
 *   Handle the USB endpoint interrupt from the host (OUT).
 *
 * Input Parameters:
 *   priv    - USB device abstraction
 *   epno    - The endpoint number
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ep_rx_interrupt(struct mpfs_usbdev_s *priv, int epno)
{
  struct mpfs_ep_s *privep;
#ifdef CONFIG_HAVE_USBTRACE
  uint16_t reg;
#endif
  uint16_t count;

  privep = &priv->eplist[epno];

  mpfs_putreg8(epno, MPFS_USB_INDEX);

  count = getreg16(MPFS_USB_ENDPOINT(epno) +
                   MPFS_USB_ENDPOINT_RX_COUNT_OFFSET);

#ifdef CONFIG_HAVE_USBTRACE
  reg = getreg16(MPFS_USB_ENDPOINT(epno) + MPFS_USB_ENDPOINT_RX_CSR_OFFSET);
  usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_EP_RX_CSR), reg);
#endif
  usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_EP_RX_COUNT), count);

  if (privep->epstate == USB_EPSTATE_IDLE)
    {
      /* Continue processing the read request. */

      usbtrace(TRACE_READ(USB_EPNO(epno)), count);

      mpfs_req_read(priv, privep, count);
    }
}

/****************************************************************************
 * Name: mpfs_ep_tx_interrupt
 *
 * Description:
 *   Handle the TX endpoint interrupt (IN, to the host)
 *
 * Input Parameters:
 *   priv    - USB device abstraction
 *   epno    - The endpoint number
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ep_tx_interrupt(struct mpfs_usbdev_s *priv, int epno)
{
  struct mpfs_ep_s *privep;
  privep = &priv->eplist[epno];

  mpfs_putreg8(epno, MPFS_USB_INDEX);

  uint16_t tx_csr = getreg16(MPFS_USB_ENDPOINT(epno) +
                             MPFS_USB_ENDPOINT_TX_CSR_OFFSET);

  usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_EP_TX_CSR), tx_csr);

  if ((tx_csr & TXCSRL_REG_EPN_UNDERRUN_MASK) != 0)
    {
      /* Under-run errors should happen only for ISO endpoints. */

      mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                       MPFS_USB_ENDPOINT_TX_CSR_OFFSET,
                       TXCSRL_REG_EPN_UNDERRUN_MASK,
                        0);
    }

  if ((tx_csr & TXCSRL_REG_EPN_STALL_SENT_MASK) != 0)
    {
      mpfs_modifyreg16(MPFS_USB_ENDPOINT(epno) +
                       MPFS_USB_ENDPOINT_TX_CSR_OFFSET,
                       TXCSRL_REG_EPN_STALL_SENT_MASK,
                       0);
    }

  if (privep->epstate == USB_EPSTATE_SENDING ||
      privep->epstate == USB_EPSTATE_EP0STATUSIN)
    {
      /* Continue / resume processing the write requests */

      privep->epstate = USB_EPSTATE_IDLE;
      mpfs_req_write(priv, privep);
    }
}

/****************************************************************************
 * Name: mpfs_ctrl_ep_interrupt
 *
 * Description:
 *   Handle the EP0 USB endpoint interrupt
 *
 * Input Parameters:
 *   priv    - USB device abstraction
 *   epno    - The endpoint number
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ctrl_ep_interrupt(struct mpfs_usbdev_s *priv, int epno)
{
  struct mpfs_ep_s *privep;
  uint16_t count0;
  uint16_t csr0;
  uint8_t mode;

  /* Get the endpoint structure */

  privep = &priv->eplist[epno];

  mpfs_putreg8(epno, MPFS_USB_INDEX);

  /* Make sure we're in device mode */

  mode = getreg8(MPFS_USB_DEV_CTRL) & DEV_CTRL_HOST_MODE_MASK;
  DEBUGASSERT(!mode);

  count0 = getreg16(MPFS_USB_INDEXED_CSR_EP0_COUNT0);
  csr0 = getreg16(MPFS_USB_INDEXED_CSR_EP0_CSR0);

  if (privep->epstate == USB_EPSTATE_EP0ADDRESS)
    {
      mpfs_setdevaddr(priv, priv->devaddr);
      privep->epstate = USB_EPSTATE_IDLE;
    }

  /* Endpoint stall */

  if ((csr0 & CSR0L_DEV_STALL_SENT_MASK) != 0)
    {
      if (privep->epstate == USB_EPSTATE_STALLED)
        {
          usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_EP0_STALLSENT), csr0);
          mpfs_modifyreg16(MPFS_USB_INDEXED_CSR_EP0_CSR0,
                           CSR0L_DEV_STALL_SENT_MASK, 0);
        }
    }

  /* Clear setup end if set */

  if ((csr0 & CSR0L_DEV_SETUP_END_MASK) != 0)
    {
      /* Setting SERVICED_SETUP_END bit clears Setup End bit */

      mpfs_modifyreg16(MPFS_USB_INDEXED_CSR_EP0_CSR0, 0,
                       CSR0L_DEV_SERVICED_SETUP_END_MASK);
    }

  if (privep->epstate == USB_EPSTATE_SENDING ||
      privep->epstate == USB_EPSTATE_EP0STATUSIN)
    {
      /* Continue/resume processing the write requests */

      privep->epstate = USB_EPSTATE_IDLE;
      mpfs_req_write(priv, privep);
    }

  /* RX packet received.  Should not get them here. */

  if ((csr0 & CSR0L_DEV_DATA_END_MASK) != 0)
    {
      /* Premature termination. Control transfer has ended before DATAEND,
       * this indicates some trouble!
       */

      usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_EP0PREMATURETERM), count0);
    }

  /* SETUP packet received */

  if ((csr0 & CSR0L_DEV_RX_PKT_RDY_MASK) != 0)
    {
      uint16_t len;

      usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_EP0_CSR0), csr0);
      usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_EP0_COUNT0), count0);

      /* If a write request transfer was pending, complete it. */

      if (privep->epstate == USB_EPSTATE_SENDING)
        {
          mpfs_req_complete(privep, -EPROTO);
        }

      usbtrace(TRACE_READ(USB_EPNO(EP0)), count0);

      if (count0 > 0)
        {
          if (privep->epstate == USB_EPSTATE_EP0DATAOUT)
            {
              mpfs_read_rx_fifo((uint8_t *)&priv->ep0out, count0, EP0);
              mpfs_putreg16(CSR0L_DEV_DATA_END_MASK |
                            CSR0L_DEV_SERVICED_RX_PKT_RDY_MASK,
                            MPFS_USB_INDEXED_CSR_EP0_CSR0);
            }
          else
            {
              DEBUGASSERT(count0 == sizeof(struct usb_ctrlreq_s));

              mpfs_read_rx_fifo((uint8_t *)&priv->ctrl, count0, EP0);

              mpfs_putreg16(CSR0L_DEV_SERVICED_RX_PKT_RDY_MASK,
                            MPFS_USB_INDEXED_CSR_EP0_CSR0);
            }
        }

      if (privep->epstate == USB_EPSTATE_EP0DATAOUT)
        {
          uint16_t rlen;

          DEBUGASSERT(epno == EP0);

          /* Yes.. back to the IDLE state */

          privep->epstate = USB_EPSTATE_IDLE;

          /* Get the size that we expected to receive */

          rlen = GETUINT16(priv->ctrl.len);

          if (rlen == count0)
            {
              /* And handle the EP0 SETUP now. */

              mpfs_ep0_setup(priv);
            }
          else
            {
              /* Then stall. */

              usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_EP0SETUPOUTSIZE),
                       count0);

              mpfs_ep_stall(privep);
            }

          return;
        }

      /* SETUP data is ready */

      len = GETUINT16(priv->ctrl.len);
      if (USB_REQ_ISOUT(priv->ctrl.type) && len > 0)
        {
          /* Yes.. then we have to wait for the OUT data phase to complete
           * before processing the SETUP command.
           */

          privep->epstate = USB_EPSTATE_EP0DATAOUT;
        }
      else
        {
          /* This is an SETUP IN command (or a SETUP IN with no data). */

          privep->epstate = USB_EPSTATE_IDLE;

          /* Handle the SETUP OUT command now */

          mpfs_ep0_setup(priv);
        }

      /* Ready for next setup data */

      mpfs_ep0_ctrlread(priv);
    }
}

/****************************************************************************
 * Name: mpfs_usb_interrupt
 *
 * Description:
 *   Handle the USB interrupt, for the device mode only.
 *
 * Input Parameters:
 *   irq      - IRQ number (unused)
 *   context  - Context (unused)
 *   arg      - Pointer to the private data
 *
 * Returned Value:
 *   OK unconditionally
 *
 ****************************************************************************/

static int mpfs_usb_interrupt(int irq, void *context, void *arg)
{
  struct mpfs_usbdev_s *priv = (struct mpfs_usbdev_s *)arg;
  uint16_t isr;
  uint16_t pending_rx_ep;
  uint16_t pending_tx_ep;
  int i;

  /* Get the device interrupts */

  isr = getreg8(MPFS_USB_IRQ);

  usbtrace(TRACE_INTDECODE(MPFS_TRACEINTID_INTERRUPT), isr);

  /* Get the endpoint interrupts */

  pending_tx_ep = getreg16(MPFS_USB_TX_IRQ);
  pending_rx_ep = getreg16(MPFS_USB_RX_IRQ);

  if (isr & RESET_IRQ_MASK)
    {
      /* Handle the reset */

      mpfs_reset(priv);

#ifdef CONFIG_USBDEV_DUALSPEED
      priv->usbdev.speed = USB_SPEED_HIGH;
      priv->usbdev.dualspeed = 0;
#else
      priv->usbdev.speed = USB_SPEED_FULL;
      priv->usbdev.dualspeed = 1;
#endif

      return OK;
    }

  /* Serve Endpoint Interrupts first */

  if ((pending_tx_ep & 0x01) != 0)
    {
      mpfs_ctrl_ep_interrupt(priv, 0);
    }

  if (pending_tx_ep != 0)
    {
      for (i = 1; i < MPFS_USB_NENDPOINTS; i++)
        {
          if ((pending_tx_ep & (1 << i)) != 0)
            {
              mpfs_ep_tx_interrupt(priv, i);
            }
        }
    }

  if (pending_rx_ep != 0)
    {
      for (i = 0; i < MPFS_USB_NENDPOINTS; i++)
        {
          if ((pending_rx_ep & (1 << i)) != 0)
            {
              mpfs_ep_rx_interrupt(priv, i);
            }
        }
    }

  if ((isr & SUSPEND_IRQ_MASK) != 0)
    {
      /* Unhandled */

      uinfo("SUSPEND IRQ received\n");
    }

  /* SOF interrupt */

  if ((isr & SOF_IRQ_MASK) != 0)
    {
      /* Unhandled */

      uinfo("SOF IRQ received\n");
    }

  if ((isr & DISCONNECT_IRQ_MASK) != 0)
    {
      /* Unhandled */

      uinfo("Disconnect IRQ\n");
    }

  if ((isr & RESUME_IRQ_MASK) != 0)
    {
      mpfs_resume(priv);
    }

  return OK;
}

/****************************************************************************
 * Name: mpfs_usb_dma_interrupt
 *
 * Description:
 *   Handle the USB DMA interrupts.
 *
 * Input Parameters:
 *   irq      - IRQ number (unused)
 *   context  - Context (unused)
 *   arg      - Pointer to the private data
 *
 * Returned Value:
 *   OK unconditionally
 *
 * Assumptions/Limitations:
 *   DMA is not supported yet
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static int mpfs_usb_dma_interrupt(int irq, void *context, void *arg)
{
  return OK;
}
#endif

/****************************************************************************
 * Endpoint Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_epset_reset
 *
 * Description:
 *   Reset and disable a set of endpoints.
 *
 * Input Parameters:
 *   priv      - USB device private data
 *   epset     - Set of EPs to reset
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_epset_reset(struct mpfs_usbdev_s *priv, uint16_t epset)
{
  uint16_t bit;
  int epno;

  /* Reset each endpoint in the set */

  for (epno = 0, bit = 1, epset &= MPFS_EPSET_ALL;
       epno < MPFS_USB_NENDPOINTS && epset != 0;
       epno++, bit <<= 1)
    {
      /* Is this endpoint in the set? */

      if ((epset & bit) != 0)
        {
          /* Yes, reset and disable it */

          mpfs_ep_reset(priv, epno);
          epset &= ~bit;
        }
    }
}

/****************************************************************************
 * Name: mpfs_pullup
 *
 * Description:
 *   This is the pullup() method of the USB device driver interface. This is
 *   a no-op.
 *
 * Input Parameters:
 *   dev       - USB device
 *   enable    - Enable or disable
 *
 * Returned Value:
 *   OK unconditionally
 *
 ****************************************************************************/

static int mpfs_pullup(struct usbdev_s *dev, bool enable)
{
  return OK;
}

/****************************************************************************
 * Initialization / Reset
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_usb_iomux
 *
 * Description:
 *   This initializes the necessary pin-muxes for the USB controller.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_usb_iomux(void)
{
  mpfs_configgpio(MSSIO_USB_CLK);
  mpfs_configgpio(MSSIO_USB_DIR);
  mpfs_configgpio(MSSIO_USB_NXT);
  mpfs_configgpio(MSSIO_USB_STP);
  mpfs_configgpio(MSSIO_USB_DATA0);
  mpfs_configgpio(MSSIO_USB_DATA1);
  mpfs_configgpio(MSSIO_USB_DATA2);
  mpfs_configgpio(MSSIO_USB_DATA3);
  mpfs_configgpio(MSSIO_USB_DATA4);
  mpfs_configgpio(MSSIO_USB_DATA5);
  mpfs_configgpio(MSSIO_USB_DATA6);
  mpfs_configgpio(MSSIO_USB_DATA7);

#ifdef CONFIG_USBDEV_DMA
  /* DMA operations need to open the USB PMP registers for proper
   * operation. If not configured, apply default settings.
   */

  uint64_t pmpcfg_usb_x;

  pmpcfg_usb_x = getreg64(MPFS_PMPCFG_USB_0);
  if ((pmpcfg_usb_x & 0x1ffffff000000000llu) != 0x1f00000000000000llu)
    {
      uerr("Please check the MPFS_PMPCFG_USB_0 register.\n");
      putreg64(0x1f00000fffffffffllu, MPFS_PMPCFG_USB_0);
    }

  pmpcfg_usb_x = getreg64(MPFS_PMPCFG_USB_1);
  if ((pmpcfg_usb_x & 0x1ffffff000000000llu) != 0x1f00000000000000llu)
    {
      uerr("Please check the MPFS_PMPCFG_USB_1 register.\n");
      putreg64(0x1f00000fffffffffllu, MPFS_PMPCFG_USB_1);
    }

  pmpcfg_usb_x = getreg64(MPFS_PMPCFG_USB_2);
  if ((pmpcfg_usb_x & 0x1ffffff000000000llu) != 0x1f00000000000000llu)
    {
      uerr("Please check the MPFS_PMPCFG_USB_2 register.\n");
      putreg64(0x1f00000fffffffffllu, MPFS_PMPCFG_USB_2);
    }

  pmpcfg_usb_x = getreg64(MPFS_PMPCFG_USB_3);
  if ((pmpcfg_usb_x & 0x1ffffff000000000llu) != 0x1f00000000000000llu)
    {
      uerr("Please check the MPFS_PMPCFG_USB_3 register.\n");
      putreg64(0x1f00000fffffffffllu, MPFS_PMPCFG_USB_3);
    }
#endif
}

/****************************************************************************
 * Name: mpfs_hw_setup
 *
 * Description:
 *   This sets up the hardware by setting the clocks and resetting the proper
 *   modules. When ready, it issues the soft-connect procedure.
 *
 * Input Parameters:
 *   priv   - USB device abstraction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_hw_setup(struct mpfs_usbdev_s *priv)
{
  volatile uint8_t soft_reset;

  /* Prepare the pads properly first */

  mpfs_usb_iomux();

  /* Enable the clock */

  mpfs_enableclk();

  /* Disable USB block reset, also from FPGA */

  modifyreg32(MPFS_SYSREG_SOFT_RESET_CR, SYSREG_SOFT_RESET_CR_USB |
              SYSREG_SOFT_RESET_CR_FPGA, 0);

  /* Reset the controller */

  mpfs_putreg8(SOFT_RESET_REG_MASK, MPFS_USB_SOFT_RST);
  do
    {
      soft_reset = getreg8(MPFS_USB_SOFT_RST);
    }
  while (soft_reset);

  /* Enable main PLIC IRQ */

  up_enable_irq(MPFS_IRQ_USB_MC);

  /* Clear IRQ status */

  mpfs_putreg8(0x0, MPFS_USB_IRQ);

  /* Init descriptor base address */

  memset((uint8_t *)(&priv->ep_descriptors[0]), 0,
         sizeof(priv->ep_descriptors));

  /* Enable interrupts */

  mpfs_putreg8(SUSPEND_IRQ_MASK | RESUME_IRQ_MASK | RESET_IRQ_MASK |
               CONNECT_IRQ_MASK | DISCONNECT_IRQ_MASK,
               MPFS_USB_ENABLE);

  mpfs_putreg16(0x01, MPFS_USB_C_T_HSBT);

  /* Disable EP interrupts */

  mpfs_putreg16(0x0, MPFS_USB_RX_IRQ_ENABLE);
  mpfs_putreg16(0x0, MPFS_USB_TX_IRQ_ENABLE);

  /* Perform soft-connect */

#ifdef CONFIG_USBDEV_DUALSPEED
  mpfs_putreg8(POWER_REG_SOFT_CONN_MASK |
               POWER_REG_ENABLE_HS_MASK, MPFS_USB_POWER);
#else
  mpfs_putreg8(POWER_REG_SOFT_CONN_MASK, MPFS_USB_POWER);
#endif
}

/****************************************************************************
 * Name: mpfs_hw_shutdown
 *
 * Description:
 *   This shuts down the USB device.
 *
 * Input Parameters:
 *   priv   - USB device abstraction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_hw_shutdown(struct mpfs_usbdev_s *priv)
{
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable all interrupts */

  mpfs_putreg8(0, MPFS_USB_ENABLE);

  /* Disable clocking to the peripheral */

  mpfs_disableclk();
}

/****************************************************************************
 * Name: mpfs_sw_setup
 *
 * Description:
 *   This sets up and initializes the software.
 *
 * Input Parameters:
 *   priv   - USB device abstraction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_sw_setup(struct mpfs_usbdev_s *priv)
{
  int epno;

  memset(priv, 0, sizeof(struct mpfs_usbdev_s));
  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->eplist[EP0].ep;
  priv->epavail    = MPFS_EPSET_ALL & ~MPFS_EP_BIT(EP0);
  priv->devstate   = USB_DEVSTATE_SUSPENDED;
  priv->prevstate  = USB_DEVSTATE_POWERED;

  /* Initialize the endpoint list */

  for (epno = 0; epno < MPFS_USB_NENDPOINTS; epno++)
    {
      /* Set endpoint operations, reference to driver structure (not
       * really necessary because there is only one controller), and
       * the (physical) endpoint number which is just the index to the
       * endpoint.
       */

      priv->eplist[epno].ep.ops    = &g_epops;
      priv->eplist[epno].dev       = priv;
      priv->eplist[epno].ep.eplog  = epno;

      /* We will use a maxpacket size for supported for each endpoint */

#ifdef CONFIG_USBDEV_DUALSPEED
      if (epno == EP0)
        {
          priv->eplist[epno].ep.maxpacket = MPFS_USB_MAXPACKETSIZE(epno);
        }
      else
        {
          priv->eplist[epno].ep.maxpacket = MPFS_USB_MAXPACKETSIZE_HS(epno);
        }
#else
      priv->eplist[epno].ep.maxpacket = MPFS_USB_MAXPACKETSIZE(epno);
#endif

      /* set descriptor addresses */

      priv->eplist[epno].descb[0] = &priv->ep_descriptors[(epno << 1)];
      priv->eplist[epno].descb[1] = &priv->ep_descriptors[(epno << 1) + 1];
    }

  /* Select a smaller endpoint size for EP0 */

#if MPFS_EP0_MAXPACKET < 64
  priv->eplist[EP0].ep.maxpacket = MPFS_EP0_MAXPACKET;
#endif
}

/****************************************************************************
 * Name: mpfs_sw_shutdown
 *
 * Description:
 *   This shuts down the sw.  Currently this is a no-operation.
 *
 * Input Parameters:
 *   priv   - USB device abstraction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_sw_shutdown(struct mpfs_usbdev_s *priv)
{
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method
 *   will be called to bind it to a USB device driver.
 *
 * Input Parameters:
 *   driver   - USB device driver abstraction
 *
 * Returned Value:
 *   OK on success, a negated error otherwise
 *
 ****************************************************************************/

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  struct mpfs_usbdev_s *priv = &g_usbd;
  int ret;

  DEBUGASSERT(driver != NULL);

  /* First hook up the driver */

  priv->driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &priv->usbdev);
  if (ret != OK)
    {
      usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_BINDFAILED), (uint16_t)-ret);
      priv->driver = NULL;
    }
  else
    {
      mpfs_hw_setup(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Unregister usbdev class driver.  If the USB device is connected to a
 *   USB host, it will first disconnect().  The driver is also requested to
 *   unbind() and clean up any device state, before this procedure finally
 *   returns.
 *
 * Input Parameters:
 *   driver   - USB device driver abstraction
 *
 * Returned Value:
 *   OK on success, a negated error otherwise
 *
 ****************************************************************************/

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct mpfs_usbdev_s *priv = &g_usbd;
  irqstate_t flags;

  /* Reset the hardware and cancel all requests.  All requests must be
   * canceled while the class driver is still bound.
   */

  flags = enter_critical_section();

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  mpfs_hw_shutdown(priv);
  mpfs_sw_shutdown(priv);

  /* Unhook the driver */

  priv->driver = NULL;
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: mpfs_usbinitialize
 *
 * Description:
 *   This is called from mpfs_usbinitialize() if it fails.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpfs_usbuninitialize(void)
{
  /* Nothing to do */
}

/****************************************************************************
 * Name: mpfs_usbinitialize
 *
 * Description:
 *   This is called in the board startup phase.  This prepares the software
 *   ready for the later phase.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpfs_usbinitialize(void)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct mpfs_usbdev_s *priv = &g_usbd;

  usbtrace(TRACE_DEVINIT, 0);

  /* Software initialization */

  mpfs_sw_setup(priv);

  /* Attach interrupts */

  if (irq_attach(MPFS_IRQ_USB_MC, mpfs_usb_interrupt, priv) != 0)
    {
      usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_IRQREGISTRATION),
                              MPFS_IRQ_USB_MC);
      goto errout;
    }

#ifdef CONFIG_USBDEV_DMA
  if (irq_attach(MPFS_IRQ_USB_DMA, mpfs_usb_dma_interrupt, priv) != 0)
    {
      usbtrace(TRACE_DEVERROR(MPFS_TRACEERR_IRQREGISTRATION),
                              MPFS_IRQ_USB_DMA);
      goto errout;
    }
#endif

  return;

errout:
  mpfs_usbuninitialize();
}


/****************************************************************************
 * arch/arm/src/lc823450/lc823450_usbdev.c
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
#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <stddef.h>
#ifdef CONFIG_SYSTEM_PROPERTY
#  include <system_property.h>
#endif

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/signal.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#ifdef CONFIG_BATTERY
#  include <nuttx/power/battery.h>
#endif
#include <nuttx/power/pm.h>
#ifdef CONFIG_OFFDEEPSLEEP
#  include <nuttx/power/offdeepsleep.h>
#endif
#ifdef CONFIG_WAKELOCK
#  include <nuttx/wakelock.h>
#endif
#include <nuttx/wqueue.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "lc823450_usbdev.h"
#include "lc823450_dma.h"
#include "lc823450_syscontrol.h"

#if defined(CONFIG_HOTPLUG) && defined(CONFIG_HOTPLUG_USB)
#  include <arch/board/hotplug.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a, b) ((a) > (b) ? (b) : (a))
#endif

#if 0
#  define DPRINTF(fmt, args...) uinfo(fmt, ##args)
#else
#  define DPRINTF(fmt, args...) do {} while (0)
#endif

#ifndef container_of
#  define container_of(ptr, type, member) \
    ((type *)((void *)(ptr) - offsetof(type, member)))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lc823450_ep_s;

struct lc823450_req_s
{
  struct usbdev_req_s  req;           /* Standard USB request */
  sq_entry_t           q_ent;
  struct lc823450_ep_s *ep;
};

struct lc823450_ep_s
{
  struct usbdev_ep_s       ep;          /* Standard endpoint structure */
  struct lc823450_usbdev_s *dev;        /* Reference to private driver data */
  uint32_t                 epcmd;
  uint8_t                  epphy;       /* Physical EP address */
  sq_queue_t               req_q;
  uint8_t                  *inbuf;
  uint8_t                  *outbuf;
  uint8_t                  stalled:1;   /* 1: Endpoint is stalled */
#ifdef CONFIG_USBMSC_IGNORE_CLEAR_STALL
  uint8_t                  ignore_clear_stall:1;
#endif /* CONFIG_USBMSC_IGNORE_CLEAR_STALL */
  uint8_t                  in : 1;      /* in = 1, out = 0 */
  uint8_t                  type : 2;    /* 0:cont, 1:iso, 2:bulk, 3:int */
  uint8_t                  disable : 1;
};

struct lc823450_usbdev_s
{
  struct usbdev_s             usbdev;
  struct usbdevclass_driver_s *driver;
  struct lc823450_ep_s        eplist[LC823450_NPHYSENDPOINTS];
  int                         bufoffset;
  uint8_t                     used;         /* used phyep */
#ifdef CONFIG_WAKELOCK
  struct wake_lock            wlock;
#endif /* CONFIG_WAKELOCK */
  uint8_t                     suspended:1;
#ifdef CONFIG_USBDEV_CHARGER
  uint8_t                     charger:1;    /* 1: charger detected */
#endif /* CONFIG_USBDEV_CHARGER */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void lc823450_epack(int epnum, bool ack);
static int epbuf_write(int epnum, void *buf, size_t len);
static void epcmd_write(int epnum, uint32_t val);
static int lc823450_epconfigure(struct usbdev_ep_s *ep,
  const struct usb_epdesc_s *desc, bool last);
static int lc823450_epclearreq(struct usbdev_ep_s *ep);
static int lc823450_epdisable(struct usbdev_ep_s *ep);
static struct usbdev_req_s *lc823450_epallocreq(struct usbdev_ep_s *ep);
static void lc823450_epfreereq(struct usbdev_ep_s *ep,
  struct usbdev_req_s *req);
#ifdef CONFIG_USBDEV_DMA
static void *lc823450_epallocbuffer(struct usbdev_ep_s *ep, uint16_t bytes);
static void lc823450_epfreebuffer(struct usbdev_ep_s *ep, void *buf);
#endif /* CONFIG_USBDEV_DMA */
static int lc823450_epsubmit(struct usbdev_ep_s *ep,
  struct usbdev_req_s *req);
static int lc823450_epcancel(struct usbdev_ep_s *ep,
  struct usbdev_req_s *req);
static int lc823450_epstall(struct usbdev_ep_s *ep, bool resume);
static struct usbdev_ep_s *lc823450_allocep(struct usbdev_s *dev,
  uint8_t eplog, bool in, uint8_t eptype);
static void lc823450_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep);
static int lc823450_getframe(struct usbdev_s *dev);
static int lc823450_wakeup(struct usbdev_s *dev);
static int lc823450_selfpowered(struct usbdev_s *dev, bool selfpowered);
int lc823450_usbpullup(struct usbdev_s *dev, bool enable);
static void subintr_usbdev(void);
static void subintr_ep0(void);
static void subintr_ep(uint8_t epnum);
static int lc823450_usbinterrupt(int irq, void *context, void *arg);
#if defined(CONFIG_BATTERY) && defined(CONFIG_USBDEV_CHARGER)
static void usb_reset_work_func(void *arg);
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_DVFS
#define DVFS_BOOST_TIMEOUT (200) /* 200ms */
extern int lc823450_dvfs_boost(int timeout);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lc823450_usbdev_s g_usbdev;

static DMA_HANDLE g_hdma;
static sem_t dma_wait = SEM_INITIALIZER(0);

#ifdef CONFIG_USBMSC_OPT
static struct lc823450_dma_llist g_dma_list[16];
#endif

#ifdef CONFIG_PM
static void usbdev_pmnotify(struct pm_callback_s *cb,
                            enum pm_state_e pmstate);
static struct pm_callback_s g_pm_cb =
{
  .notify      = usbdev_pmnotify,
};
#endif

static const struct usbdev_epops_s g_epops =
{
  .configure   = lc823450_epconfigure,
  .disable     = lc823450_epdisable,
  .allocreq    = lc823450_epallocreq,
  .freereq     = lc823450_epfreereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = lc823450_epallocbuffer,
  .freebuffer  = lc823450_epfreebuffer,
#endif
  .submit      = lc823450_epsubmit,
  .cancel      = lc823450_epcancel,
  .stall       = lc823450_epstall,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep     = lc823450_allocep,
  .freeep      = lc823450_freeep,
  .getframe    = lc823450_getframe,
  .wakeup      = lc823450_wakeup,
  .selfpowered = lc823450_selfpowered,
  .pullup      = lc823450_usbpullup,
};

#if defined(CONFIG_BATTERY) && defined(CONFIG_USBDEV_CHARGER)
static struct work_s g_reset_work;
#endif

#ifdef CONFIG_WAKELOCK
static struct work_s g_suspend_work;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_epack
 *
 * Description:
 *   ACK or NAK and endpoint
 *
 ****************************************************************************/

static void lc823450_epack(int epnum, bool ack)
{
  struct lc823450_ep_s *privep;

  privep = &g_usbdev.eplist[epnum];

  if (ack)
    {
      privep->epcmd &= ~USB_EPCMD_NAK;
    }
  else
    {
      privep->epcmd |= USB_EPCMD_NAK;
    }

  epcmd_write(epnum, (privep->epcmd));
}

/****************************************************************************
 * Name: epbuf_read
 *
 * Description:
 *   read from RX Endpoint Buffer
 *
 ****************************************************************************/

int epbuf_read(int epnum, void *buf, size_t len)
{
  size_t fifolen;
  struct lc823450_ep_s *privep;
  int bufidx;

  bufidx = (epnum == 0 ? 1 : epnum * 2);

  fifolen = (getreg32(USB_EPCOUNT(bufidx)) & USB_EPCOUNT_PHYCNT_MASK) >>
    USB_EPCOUNT_PHYCNT_SHIFT;

  privep = &g_usbdev.eplist[epnum];

  len = MIN(len, fifolen);

  memcpy(buf, privep->outbuf, len);

  epcmd_write(epnum, USB_EPCMD_BUFRD);

  return len;
}

/****************************************************************************
 * Name: epbuf_write
 *
 * Description:
 *   Write to TX Endpoint Buffer
 *
 ****************************************************************************/

static int epbuf_write(int epnum, void *buf, size_t len)
{
  struct lc823450_ep_s *privep;
  int bufidx;
  int txn;
  int total;
  int tout = 0;

  total = len;
  bufidx = (epnum == 0 ? 0 : epnum * 2);

  privep = &g_usbdev.eplist[epnum];

cont:
  if (epnum == 0)
    {
      while (!(getreg32(USB_EPCTRL(epnum)) & USB_EPCTRL_EMPTYI) &&
             !privep->disable && tout++ < 1000000)
        {
          up_udelay(1);
        }
    }
  else
    {
      while (!(getreg32(USB_EPCTRL(epnum)) & USB_EPCTRL_EMPTY) &&
             !privep->disable && tout++ < 1000000)
        {
          up_udelay(1);
        }
    }

  txn = MIN(len, privep->ep.maxpacket);

  memcpy(privep->inbuf, buf, txn);
  putreg32(txn << USB_EPCOUNT_APPCNT_SHIFT, USB_EPCOUNT(bufidx));

  epcmd_write(epnum, USB_EPCMD_BUFWR);

  lc823450_epack(epnum, true);
  epcmd_write(epnum, USB_EPCMD_NACK_CLR);

  len -= txn;
  buf += txn;
  if (len > 0)
    {
      goto cont;
    }

  return total;
}

/****************************************************************************
 * Name: epcmd_write
 *
 * Description:
 *   Write to EP command register
 *
 ****************************************************************************/

static void epcmd_write(int epnum, uint32_t val)
{
  while (getreg32(USB_EPCMD(epnum)) & USB_EPCMD_BUSY);

  putreg32(val, USB_EPCMD(epnum));
}

/****************************************************************************
 * Name: lc823450_epconfigure
 *
 * Description:
 *   Configure endpoint, making it usable
 *
 * Input Parameters:
 *   ep   - the struct usbdev_ep_s instance obtained from allocep()
 *   desc - A struct usb_epdesc_s instance describing the endpoint
 *   last - true if this is the last endpoint to be configured.  Some
 *          hardware needs to take special action when all of the endpoints
 *          have been configured.
 *
 ****************************************************************************/

static int lc823450_epconfigure(struct usbdev_ep_s *ep,
                                const struct usb_epdesc_s *desc,
                                bool last)
{
  int epnum;
  struct lc823450_ep_s *privep = (struct lc823450_ep_s *)ep;
  struct lc823450_usbdev_s *priv = privep->dev;

  epnum = privep->epphy;
  if (desc)
    {
      priv->eplist[epnum].ep.maxpacket = GETUINT16(desc->mxpacketsize);
    }

  DPRINTF("epnum = %d, ep = %p, max = %d, speed = 0x%x, in = 0x%x\n",
          epnum, ep, priv->eplist[epnum].ep.maxpacket,
          priv->usbdev.speed, priv->eplist[epnum].in);

  epcmd_write(epnum, USB_EPCMD_INIT);

  /* Buffer Index assign
   * IDX0: EP0IN
   * IDX1: EP0OUT
   * IDX2: EP1
   * IDX4: EP2
   * IDX6: EP3
   * ...
   */

  if (epnum == 0)
    {
      putreg32(0 << USB_EPCONF_CIDX_SHIFT |
               64 << USB_EPCONF_SIZE_SHIFT |
               (0x100 >> 2) << USB_EPCONF_BASE_SHIFT,
               USB_EPCONF(0));
      priv->eplist[0].inbuf = (uint8_t *)USB_EPBUF + 0x100;
      priv->eplist[0].outbuf = (uint8_t *)USB_EPBUF + 0x140;
    }
  else
    {
      putreg32((epnum * 2) << USB_EPCONF_CIDX_SHIFT |
               priv->eplist[epnum].ep.maxpacket << USB_EPCONF_SIZE_SHIFT |
               (priv->bufoffset >> 2) << USB_EPCONF_BASE_SHIFT,
               USB_EPCONF(epnum));
      priv->eplist[epnum].inbuf = (uint8_t *)USB_EPBUF + priv->bufoffset;
      priv->eplist[epnum].outbuf = (uint8_t *)USB_EPBUF + priv->bufoffset;
      priv->bufoffset += priv->eplist[epnum].ep.maxpacket;
    }

  priv->eplist[epnum].epcmd =
    priv->eplist[epnum].type << USB_EPCMD_ET_SHIFT |
    (priv->eplist[epnum].in ? 1 : 0) << USB_EPCMD_DIR_SHIFT |
    USB_EPCMD_WRITE_EN | USB_EPCMD_START;

#ifdef CONFIG_USBDEV_NULLRESP_EPNUM
  if (epnum == CONFIG_USBDEV_NULLRESP_EPNUM)
    {
      priv->eplist[epnum].epcmd |= USB_EPCMD_NULL;
    }
#endif /* CONFIG_USBDEV_NULLRESP_EPNUM */

  if (epnum != 0)
    {
      if (priv->eplist[epnum].in)
        {
          priv->eplist[epnum].epcmd |= USB_EPCMD_EMPTY_EN;
        }
      else
        {
          priv->eplist[epnum].epcmd |= USB_EPCMD_READY_EN;
        }
    }

  epcmd_write(epnum, priv->eplist[epnum].epcmd | USB_EPCMD_TGL_CLR |
              USB_EPCMD_STALL_CLR);

  privep->disable = 0;
  return 0;
}

/****************************************************************************
 * Name: lc823450_epdisable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 ****************************************************************************/

static int lc823450_epclearreq(struct usbdev_ep_s *ep)
{
  struct lc823450_ep_s *privep = (struct lc823450_ep_s *)ep;
  irqstate_t flags;

  flags = spin_lock_irqsave(NULL);
  while (privep->req_q.tail)
    {
      struct usbdev_req_s *req;
      sq_entry_t *q_ent;

      /* Dequeue from Reqbuf poll */

      q_ent = sq_remlast(&privep->req_q);
      req = &container_of(q_ent, struct lc823450_req_s, q_ent)->req;

      /* return reqbuf to function driver */

      req->result = -ESHUTDOWN;
      req->callback(ep, req);
    }

  spin_unlock_irqrestore(NULL, flags);
  return 0;
}

/****************************************************************************
 * Name: lc823450_epdisable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 ****************************************************************************/

static int lc823450_epdisable(struct usbdev_ep_s *ep)
{
  struct lc823450_ep_s *privep = (struct lc823450_ep_s *)ep;
  uint8_t epnum;

  epnum = privep->epphy;
  privep->epcmd = 0;
  epcmd_write(epnum, privep->epcmd | USB_EPCMD_STOP);

  if (epnum == 0)
    {
      putreg32(0x500, USB_EPCTRL(0));
    }
  else
    {
      putreg32(0x400, USB_EPCTRL(epnum));
    }

  putreg32(0, USB_EPCONF(epnum));
  putreg32(0, USB_EPCOUNT(epnum * 2));
  putreg32(0, USB_EPCOUNT(epnum * 2 + 1));

  g_usbdev.bufoffset = 0x180;
  privep->disable = 1;
  lc823450_dmastop(g_hdma);
  nxsem_post(&dma_wait);
  return lc823450_epclearreq(ep);
}

/****************************************************************************
 * Name: lc823450_epallocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 ****************************************************************************/

static struct usbdev_req_s *lc823450_epallocreq(struct usbdev_ep_s *ep)
{
  struct lc823450_req_s *privreq;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(LC823450_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif

  usbtrace(TRACE_EPALLOCREQ, ((struct lc823450_ep_s *)ep)->epphy);

  privreq = (struct lc823450_req_s *)
    kmm_zalloc(sizeof(struct lc823450_req_s));

  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(LC823450_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  return &privreq->req;
}

/****************************************************************************
 * Name: lc823450_epfreereq
 *
 * Description:
 *   Free an I/O request
 *
 ****************************************************************************/

static void lc823450_epfreereq(struct usbdev_ep_s *ep,
                               struct usbdev_req_s *req)
{
  struct lc823450_req_s *privreq = (struct lc823450_req_s *)req;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(LC823450_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  usbtrace(TRACE_EPFREEREQ, ((struct lc823450_ep_s *)ep)->epphy);
  kmm_free(privreq);
}

#ifdef CONFIG_USBDEV_DMA
/****************************************************************************
 * Name: lc823450_epallocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 ****************************************************************************/

static void *lc823450_epallocbuffer(struct usbdev_ep_s *ep, uint16_t bytes)
{
  usbtrace(TRACE_EPALLOCBUFFER, privep->epphy);

#  ifdef CONFIG_USBDEV_DMAMEMORY
  return usbdev_dma_alloc(bytes);
#  else
  return kmm_malloc(bytes);
#  endif
}
#endif

#ifdef CONFIG_USBDEV_DMA
/****************************************************************************
 * Name: lc823450_epfreebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 ****************************************************************************/

static void lc823450_epfreebuffer(struct usbdev_ep_s *ep, void *buf)
{
  usbtrace(TRACE_EPFREEBUFFER, privep->epphy);

#  ifdef CONFIG_USBDEV_DMAMEMORY
  usbdev_dma_free(buf);
#  else
  kmm_free(buf);
#  endif
}
#endif

/****************************************************************************
 * Name: lc823450_epsubmit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 ****************************************************************************/

static int lc823450_epsubmit(struct usbdev_ep_s *ep,
                             struct usbdev_req_s *req)
{
  struct lc823450_req_s *privreq = (struct lc823450_req_s *)req;
  struct lc823450_ep_s *privep = (struct lc823450_ep_s *)ep;
  irqstate_t flags;

  req->result = 0;
  if (privep->disable)
    {
      return -EBUSY;
    }

  if (privep->epphy == 0)
    {
      flags = spin_lock_irqsave(NULL);
      req->xfrd = epbuf_write(privep->epphy, req->buf, req->len);
      spin_unlock_irqrestore(NULL, flags);
      req->callback(ep, req);
    }
  else if (privep->in)
    {
      /* Send packet request from function driver */

      flags = spin_lock_irqsave(NULL);

      if ((getreg32(USB_EPCOUNT(privep->epphy * 2)) &
          USB_EPCOUNT_PHYCNT_MASK) >> USB_EPCOUNT_PHYCNT_SHIFT ||
          privep->req_q.tail)
        {
          sq_addfirst(&privreq->q_ent, &privep->req_q); /* non block */
          spin_unlock_irqrestore(NULL, flags);
        }
       else
        {
          spin_unlock_irqrestore(NULL, flags);
          req->xfrd = epbuf_write(privep->epphy, req->buf, req->len);
          req->callback(ep, req);
        }
    }
  else
    {
      /* receive packet buffer from function driver */

      flags = spin_lock_irqsave(NULL);
      sq_addfirst(&privreq->q_ent, &privep->req_q); /* non block */
      spin_unlock_irqrestore(NULL, flags);
      lc823450_epack(privep->epphy, 1);
    }

  return 0;
}

/****************************************************************************
 * Name: lc823450_epcancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 ****************************************************************************/

static int lc823450_epcancel(struct usbdev_ep_s *ep,
                             struct usbdev_req_s *req)
{
  struct lc823450_req_s *privreq = (struct lc823450_req_s *)req;
  struct lc823450_ep_s *privep = (struct lc823450_ep_s *)ep;
  irqstate_t flags;

  /* Remove request from req_queue */

  flags = spin_lock_irqsave(NULL);
  sq_remafter(&privreq->q_ent, &privep->req_q);
  spin_unlock_irqrestore(NULL, flags);
  return 0;
}

/****************************************************************************
 * Name: lc823450_epstall
 *
 * Description:
 *   Stall or resume and endpoint
 *
 ****************************************************************************/

static int lc823450_epstall(struct usbdev_ep_s *ep, bool resume)
{
  struct lc823450_ep_s *privep = (struct lc823450_ep_s *)ep;
  irqstate_t flags;

  /* STALL or RESUME the endpoint */

  flags = spin_lock_irqsave(NULL);
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, privep->epphy);

  if (resume)
    {
      privep->stalled = false;
      epcmd_write(privep->epphy, USB_EPCMD_STALL_CLR | USB_EPCMD_TGL_CLR);
    }
  else
    {
      privep->stalled = true;
      epcmd_write(privep->epphy, USB_EPCMD_STALL_SET | USB_EPCMD_TGL_SET);
    }

  spin_unlock_irqrestore(NULL, flags);
  return OK;
}

#ifdef CONFIG_USBMSC_IGNORE_CLEAR_STALL
void up_epignore_clear_stall(struct usbdev_ep_s *ep, bool ignore)
{
  struct lc823450_ep_s *privep = (struct lc823450_ep_s *)ep;
  irqstate_t flags;
  flags = spin_lock_irqsave(NULL);

  privep->ignore_clear_stall = ignore;

  spin_unlock_irqrestore(NULL, flags);
}
#endif /* CONFIG_USBMSC_IGNORE_CLEAR_STALL */

/****************************************************************************
 * Name: lc823450_allocep
 *
 * Description:
 *   Allocate an endpoint matching the parameters.
 *
 * Input Parameters:
 *   eplog  - 7-bit logical endpoint number (direction bit ignored).  Zero
 *            means that any endpoint matching the other requirements will
 *            suffice. The assigned endpoint can be found in the eplog field.
 *   in     - true: IN (device-to-host) endpoint requested
 *   eptype - Endpoint type.  One of {USB_EP_ATTR_XFER_ISOC,
 *            USB_EP_ATTR_XFER_BULK, USB_EP_ATTR_XFER_INT}
 *
 ****************************************************************************/

static struct usbdev_ep_s *lc823450_allocep(struct usbdev_s *dev,
                                            uint8_t eplog, bool in,
                                            uint8_t eptype)
{
  struct lc823450_usbdev_s *priv = (struct lc823450_usbdev_s *)dev;
  struct lc823450_ep_s *privep;
  int epphy;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)eplog);

  /* Ignore any direction bits in the logical address */

  epphy = USB_EPNO(eplog);

  if (priv->used & 1 << epphy)
    {
      uinfo("ep is still used\n");
      return NULL;
    }

  priv->used |= 1 << epphy;

  privep = &priv->eplist[epphy];
  privep->in = in;
  privep->type = eptype;
  privep->epphy = epphy;
  privep->ep.eplog = epphy;

  return &privep->ep;
}

/****************************************************************************
 * Name: lc823450_freeep
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 ****************************************************************************/

static void lc823450_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep)
{
  struct lc823450_usbdev_s *priv = (struct lc823450_usbdev_s *)dev;
  struct lc823450_ep_s *privep;

  privep = (struct lc823450_ep_s *)ep;

  priv->used &= ~(1 << privep->epphy);

  return;
}

/****************************************************************************
 * Name: lc823450_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 ****************************************************************************/

static int lc823450_getframe(struct usbdev_s *dev)
{
  usbtrace(TRACE_DEVGETFRAME, 0);

  return (int)(getreg32(USB_TSTAMP));
}

/****************************************************************************
 * Name: lc823450_wakeup
 *
 * Description:
 *   Tries to wake up the host connected to this device
 *
 ****************************************************************************/

static int lc823450_wakeup(struct usbdev_s *dev)
{
  usbtrace(TRACE_DEVWAKEUP, 0);

  modifyreg32(USB_DEVC, 0, USB_DEVC_RESUME);
  return OK;
}

/****************************************************************************
 * Name: lc823450_selfpowered
 *
 * Description:
 *   Sets/clears the device selfpowered feature
 *
 ****************************************************************************/

static int lc823450_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  return OK;
}

/****************************************************************************
 * Name: lc823450_usbpullup
 *
 * Description:
 *   Software-controlled connect to/disconnect from USB host
 *
 ****************************************************************************/

int lc823450_usbpullup(struct usbdev_s *dev, bool enable)
{
  if (enable)
    {
      modifyreg32(USB_DEVC, USB_DEVC_DISCON, 0);
    }
  else
    {
      modifyreg32(USB_DEVC, 0 , USB_DEVC_DISCON);
    }

  return 0;
}

#ifdef CONFIG_WAKELOCK
/****************************************************************************
 * Name: subintr_usbdev
 *
 * Description:
 *   USB dev interrupt sub handler
 *
 ****************************************************************************/

static void usb_suspend_work_func(void *arg)
{
  struct lc823450_usbdev_s *priv = arg;
  irqstate_t flags;
  DPRINTF("SUSPENDB MAIN\n");

#ifdef CONFIG_CDCACM_AS_ADB
  if (nxproperty_adb_get())
    {
      return;
    }
#endif

  flags = spin_lock_irqsave(NULL);
  if (getreg32(USB_DEVS) & USB_DEVS_SUSPEND)
    {
      uinfo("USB BUS SUSPEND\n");
#if defined(CONFIG_BATTERY) && !defined(CONFIG_BATTERY_DISABLE_CHARGE)
      battery_sendevent(BATTERY_USBEV_SUSP);
#endif

#ifdef CONFIG_OFFDEEPSLEEP
      g_offdeepsleep = 1;
#endif
      g_usbsuspend = 1;
      wake_unlock(&priv->wlock);
    }

  spin_unlock_irqrestore(NULL, flags);
}
#endif

#if defined(CONFIG_BATTERY) && defined(CONFIG_USBDEV_CHARGER)
/****************************************************************************
 * Name: usb_reset_work_func
 ****************************************************************************/

static void usb_reset_work_func(void *arg)
{
  if (g_usbdev.charger)
    {
      /* Disconnect Charger */

#  if defined(CONFIG_HOTPLUG) && defined(CONFIG_HOTPLUG_USB)
      hotplug_start_usbemu(false);
#  endif

      g_usbdev.charger = 0;
      work_queue(HPWORK, &g_reset_work, usb_reset_work_func, NULL,
                 MSEC2TICK(100));
    }
  else
    {
      /* Connect Host */

#  if defined(CONFIG_HOTPLUG) && defined(CONFIG_HOTPLUG_USB)
      hotplug_start_usbemu(true);
#  endif
    }
}
#endif

/****************************************************************************
 * Name: subintr_usbdev
 ****************************************************************************/

static void subintr_usbdev(void)
{
  uint32_t devs;
  struct lc823450_usbdev_s *priv = &g_usbdev;

  devs = getreg32(USB_DEVS);

#ifdef CONFIG_BATTERY
#ifdef CONFIG_USBDEV_CHARGER
  if (g_usbdev.charger)
    {
      work_cancel(HPWORK, &g_reset_work);
      work_queue(HPWORK, &g_reset_work, usb_reset_work_func, NULL, 0);

      /* Disable interrupts */

      up_disable_irq(LC823450_IRQ_USBDEV);
      putreg32(~devs, USB_DEVS);
      return;
    }
#endif
#endif

  if (devs & USB_DEVC_USBRSTE)
    {
      DPRINTF("BUSRESET\n");
      putreg32(~USB_DEVC_USBRSTE, USB_DEVS);

#if defined(CONFIG_BATTERY) && !defined(CONFIG_BATTERY_DISABLE_CHARGE)
      battery_sendevent(BATTERY_USBEV_BUSRST);
#endif

      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
      if (devs & USB_DEVS_CRTSPEED)
        {
          priv->usbdev.speed = USB_SPEED_FULL;
        }
      else
        {
          priv->usbdev.speed = USB_SPEED_HIGH;
        }
    }

  if (devs & USB_DEVC_SETUP)
    {
      DPRINTF("SETUP\n");
      putreg32(~USB_DEVC_SETUP, USB_DEVS);
      subintr_ep0();
    }

  if (devs & USB_DEVC_SUSPENDB)
    {
      DPRINTF("SUSPENDB\n");
      putreg32(~USB_DEVC_SUSPENDB, USB_DEVS);

#ifdef CONFIG_WAKELOCK
      g_deepsleep_cancel = 0;

      work_cancel(HPWORK, &g_suspend_work);
      work_queue(HPWORK, &g_suspend_work, usb_suspend_work_func, priv,
                 MSEC2TICK(1000));
#endif
    }

  if (devs & USB_DEVC_SUSPENDE)
    {
      DPRINTF("SUSPENDE\n");
      putreg32(~USB_DEVC_SUSPENDE, USB_DEVS);
#ifdef CONFIG_WAKELOCK
      g_deepsleep_cancel = 1;
#endif
#if defined(CONFIG_BATTERY) && !defined(CONFIG_BATTERY_DISABLE_CHARGE)
      battery_sendevent(BATTERY_USBEV_RES);
#endif
#ifdef CONFIG_WAKELOCK
      wake_lock(&priv->wlock);
      work_cancel(HPWORK, &g_suspend_work);
#endif

#if defined(CONFIG_BATTERY) && defined(CONFIG_USBDEV_CHARGER)
      if ((getreg32(USBSTAT) & USBSTAT_LINESTE_MASK) ==
          (USBSTAT_LINESTE_0 | USBSTAT_LINESTE_1))
        {
          g_usbdev.charger = 1;
          work_cancel(HPWORK, &g_reset_work);
          work_queue(HPWORK, &g_reset_work, usb_reset_work_func, NULL, 0);
        }
#endif
    }
}

/****************************************************************************
 * Name: subintr_ep0
 *
 * Description:
 *   Endpoint0 interrupt sub handler
 *
 ****************************************************************************/

static void subintr_ep0(void)
{
  uint32_t epctrl;
  struct lc823450_usbdev_s *priv = &g_usbdev;
  int len;
  int handled = 0;
  char resp[2];
  uint8_t epnum;

  epctrl = getreg32(USB_EPCTRL(0));
  DPRINTF("epctrl = 0x%x\n", epctrl);
  if (epctrl & USB_EPCTRL_READOI)
    {
      struct usb_ctrlreq_s ctrl;
      memset(&ctrl, 0, sizeof(ctrl));
      len = epbuf_read(0, &ctrl, sizeof(ctrl));
      epcmd_write(0, USB_EPCMD_READYO_CLR);

      /* NULL RESP */

      if (!len)
        {
          return;
        }

      DPRINTF("CTRL: type 0x%x, req 0x%x, val 0x%x, 0x%x, "
              "idx 0x%x, %x, len 0x%x, 0x%x\n",
              ctrl.type, ctrl.req, ctrl.value[0], ctrl.value[1],
              ctrl.index[0], ctrl.index[1], ctrl.len[0], ctrl.len[1]);

      if ((ctrl.type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
        {
          switch (ctrl.req)
            {
              case USB_REQ_GETSTATUS:
                resp[1] = 0;
                switch (ctrl.type & USB_REQ_RECIPIENT_MASK)
                  {
                    case USB_REQ_RECIPIENT_ENDPOINT:
                      epnum = USB_EPNO(ctrl.index[0]);
                      if (epnum < LC823450_NPHYSENDPOINTS &&
                          priv->eplist[epnum].stalled == 0)
                        {
                          resp[0] = 0; /* bit0: halt */
                        }
                      else
                        {
                          resp[0] = 1; /* bit0: halt */
                        }
                      break;

                    case USB_REQ_RECIPIENT_DEVICE:
                      resp[0] = 0; /* bit0: selfpowerd, bit1: remote wakeup */
                      break;

                    case USB_REQ_RECIPIENT_INTERFACE:
                      resp[0] = 0; /* reserved */
                      break;
                  }

                epbuf_write(0, &resp, 2);
                handled = 1;
                break;

              case USB_REQ_SETADDRESS:
                modifyreg32(USB_FADDR, USB_FADDR_ADDR_MASK, ctrl.value[0]);
                epbuf_write(0, &resp, 0);
                handled = 1;
                break;

              case USB_REQ_SETCONFIGURATION:
                modifyreg32(USB_FADDR, 0, USB_FADDR_CONFD);
#ifdef CONFIG_WAKELOCK
                wake_lock(&priv->wlock);
#endif
#if defined(CONFIG_BATTERY) && !defined(CONFIG_BATTERY_DISABLE_CHARGE)
                battery_sendevent(BATTERY_USBEV_CHG);
#endif
                break;

              case USB_REQ_SETFEATURE:
                if (ctrl.value[0] == USB_FEATURE_TESTMODE)
                  {
                    epbuf_write(0, &resp, 0);
                    up_udelay(1000);

                    if (ctrl.index[1] == 0x4)
                      {
                        /* TestPacket */

                        putreg32(1 << 0 | USB_TESTC_FORCE_HS, USB_TESTC);
                      }
                    else
                      {
                        putreg32(1 << ctrl.index[1] | USB_TESTC_FORCE_HS,
                                 USB_TESTC);
                      }

                    handled = 1;
                  }
                else if (ctrl.value[0] == USB_FEATURE_ENDPOINTHALT)
                  {
                    epnum = USB_EPNO(ctrl.index[0]);
                    if (epnum < LC823450_NPHYSENDPOINTS)
                      {
                        lc823450_epstall(&priv->eplist[epnum].ep, false);
                        epbuf_write(0, &resp, 0);
                        handled = 1;
                      }
                  }
                break;

              case USB_REQ_CLEARFEATURE:
                if (ctrl.value[0] == USB_FEATURE_ENDPOINTHALT)
                  {
                    epnum = USB_EPNO(ctrl.index[0]);
                    if (epnum < LC823450_NPHYSENDPOINTS)
                      {
                        epbuf_write(0, &resp, 0);
#ifdef CONFIG_USBMSC_IGNORE_CLEAR_STALL
                        if (!priv->eplist[epnum].ignore_clear_stall)
#endif
                          {
                            lc823450_epstall(&priv->eplist[epnum].ep, true);
                          }

                        handled = 1;
                      }
                  }
                break;
            }
        }

      if (!handled)
        {
          uint8_t ctrldat[16];
          int ctrldat_len;

          ctrldat_len = MIN(GETUINT16(ctrl.len), sizeof(ctrldat));
          if (ctrldat_len)
            {
              int tout = 1000;
              do
                {
                  if (getreg32(USB_EPCMD(0)) & USB_EPCMD_READYO_CLR)
                    {
                      break;
                    }

                  up_udelay(10);
                }
              while (tout--);

              if (tout)
                {
                  ctrldat_len = epbuf_read(0, &ctrldat, ctrldat_len);
                }
            }

          if (CLASS_SETUP(priv->driver, &priv->usbdev, &ctrl,
              (uint8_t *)&ctrldat, ctrldat_len) < 0)
            {
              lc823450_epstall(&priv->eplist[0].ep, false);
            }
        }
    }
}

/****************************************************************************
 * Name: subintr_epin
 *
 * Description:
 *   Endpoint interrupt sub handler
 *
 ****************************************************************************/

static void subintr_epin(uint8_t epnum, struct lc823450_ep_s *privep)
{
  /* Send packet done */

  irqstate_t flags;
  flags = spin_lock_irqsave(NULL);

  if (privep->req_q.tail)
    {
      struct usbdev_req_s *req;
      sq_entry_t *q_ent;

      /* Dequeue from TXQ */

      q_ent = sq_remlast(&privep->req_q);

      spin_unlock_irqrestore(NULL, flags);

      req = &container_of(q_ent, struct lc823450_req_s, q_ent)->req;

      /* Write to TX FIFO */

      /* int clear!! before epbuf write */

      epcmd_write(epnum, USB_EPCMD_EMPTY_CLR);
      req->xfrd = epbuf_write(epnum, req->buf, req->len);
      req->callback(&privep->ep, req);

      /* int clear */
    }
  else
    {
      spin_unlock_irqrestore(NULL, flags);
      epcmd_write(epnum, USB_EPCMD_EMPTY_CLR);
    }
}

/****************************************************************************
 * Name: subintr_epout
 *
 * Description:
 *   Endpoint interrupt sub handler
 *
 ****************************************************************************/

static void subintr_epout(uint8_t epnum, struct lc823450_ep_s *privep)
{
  /* Packet receive from host */

  irqstate_t flags;
  flags = spin_lock_irqsave(NULL);

  if (privep->req_q.tail)
    {
      struct usbdev_req_s *req;
      sq_entry_t *q_ent;

      /* Dequeue from Reqbuf poll */

      q_ent = sq_remlast(&privep->req_q);

      req = &container_of(q_ent, struct lc823450_req_s, q_ent)->req;
      if (privep->req_q.tail == NULL)
        {
          /* receive buffer exhaust */

          lc823450_epack(epnum, 0);
        }

      spin_unlock_irqrestore(NULL, flags);

      /* PIO */

      epcmd_write(epnum, USB_EPCMD_READY_CLR);

      /* int clear!! before epbuf read */

      req->xfrd = epbuf_read(epnum, req->buf, req->len);
      req->callback(&privep->ep, req);
    }
  else
    {
      spin_unlock_irqrestore(NULL, flags);
      uinfo("REQ Buffer Exhault\n");
      epcmd_write(epnum, USB_EPCMD_READY_CLR);
    }
}

/****************************************************************************
 * Name: subintr_ep
 *
 * Description:
 *   Endpoint interrupt sub handler
 *
 ****************************************************************************/

static void subintr_ep(uint8_t epnum)
{
  struct lc823450_usbdev_s *priv = &g_usbdev;
  struct lc823450_ep_s *privep;
  uint32_t epctrl;

  epctrl = getreg32(USB_EPCTRL(epnum));

  privep =  &priv->eplist[epnum];

  if ((epctrl & USB_EPCTRL_READI) && !privep->in)
    {
      subintr_epout(epnum, privep);
    }
  else if ((epctrl & USB_EPCTRL_EMPTI) && privep->in)
    {
      subintr_epin(epnum, privep);
    }
}

/****************************************************************************
 * Name: lc823450_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 ****************************************************************************/

static int lc823450_usbinterrupt(int irq, void *context, void *arg)
{
  uint32_t disr;
  int i;

  if (g_usbdev.suspended)
    {
      modifyreg32(USBCNT, USBCNT_RSM_CONT, 0);
    }

  disr = getreg32(USB_INTS);
  if (disr & USB_INT_DEV)
    {
      subintr_usbdev();
    }

  if (disr & USB_INT_EP0)
    {
      subintr_ep0();
    }

  for (i = USB_INT_EP0_SHIFT + 1; i <= USB_INT_EP15_SHIFT; i++)
    {
      if (disr & (1 << i))
        {
          subintr_ep(i - USB_INT_EP0_SHIFT);
        }
    }

  putreg32(~disr, USB_INTS);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_usbinitialize
 *
 * Description:
 *   Initialize the USB driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_usbinitialize(void)
{
  int i;
  struct lc823450_usbdev_s *priv = &g_usbdev;

  memset(&g_usbdev, 0, sizeof(g_usbdev));
  g_usbdev.usbdev.ops = &g_devops;
  g_usbdev.usbdev.ep0 = &g_usbdev.eplist[0].ep;

  for (i = 0; i < LC823450_NPHYSENDPOINTS; i++)
    {
      g_usbdev.eplist[i].ep.ops = &g_epops;
      g_usbdev.eplist[i].ep.maxpacket = 64;
      g_usbdev.eplist[i].dev = &g_usbdev;
      g_usbdev.eplist[i].epphy = i;
      sq_init(&g_usbdev.eplist[i].req_q);
      priv->eplist[i].ep.eplog = i;
    }

  if (irq_attach(LC823450_IRQ_USBDEV, lc823450_usbinterrupt, NULL) != 0)
    {
      usbtrace(TRACE_DEVERROR(LC823450_TRACEERR_IRQREGISTRATION),
               (uint16_t)LC823450_IRQ_USBDEV);
      return;
    }

  g_hdma = lc823450_dmachannel(DMA_CHANNEL_USBDEV);
  lc823450_dmarequest(g_hdma, DMA_REQUEST_USBDEV);

#ifdef CONFIG_WAKELOCK
  wake_lock_init(&g_usbdev.wlock, "usb");
#endif /* CONFIG_WAKELOCK */
}

/****************************************************************************
 * Name: usbdev_register
 ****************************************************************************/

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  int ret = -1;
  int i;

#ifdef CONFIG_DVFS
  lc823450_dvfs_boost(DVFS_BOOST_TIMEOUT * 20);
#endif

#ifdef CONFIG_LC823450_LSISTBY
  /* enable USB */

  mod_stby_regs(LSISTBY_STBYE, 0);
#endif

  g_usbdev.driver = driver;

  /* Clock & Power & Reset */

  modifyreg32(MCLKCNTBASIC, 0, MCLKCNTBASIC_USBDEV_CLKEN);
  modifyreg32(USBCNT, USBCNT_ANPD, 0);

  /* USB PLL REFCLOCK */

  if (XT1OSC_CLK == (24 * 1000 * 1000))
    {
      modifyreg32(USBCNT, USBCNT_CLK_MASK, USBCNT_CLK24MHZ);
      modifyreg32(USBCNT, USBCNT_CRYCNTSW_MASK, USBCNT_CRYCNTSW24MHZ);
    }
  else if (XT1OSC_CLK == (20 * 1000 * 1000))
    {
      modifyreg32(USBCNT, USBCNT_CLK_MASK, USBCNT_CLK20MHZ);
      modifyreg32(USBCNT, USBCNT_CRYCNTSW_MASK, USBCNT_CRYCNTSW20MHZ);
    }
  else
    {
      uinfo("not support\n");
    }

  modifyreg32(MRSTCNTBASIC, 0, MRSTCNTBASIC_USBDEV_RSTB);

  modifyreg32(USBCNT, 0, USBCNT_VBUS_VALID);

  /* SoftReset */

  modifyreg32(USB_CONF, 0, USB_CONF_SOFT_RESET);
  while (getreg32(USB_CONF) & USB_CONF_SOFT_RESET);

  putreg32(0, USB_CONF);

  /* RAM area init */

  for (i = 0; i < 16; i++)
    {
      if (i == 0)
        {
          putreg32(0x500, USB_EPCTRL(i));
        }
      else
        {
          putreg32(0x400, USB_EPCTRL(i));
        }

      putreg32(0, USB_EPCONF(i));
      putreg32(0, USB_EPCOUNT(i * 2));
      putreg32(0, USB_EPCOUNT(i * 2 + 1));
    }

  g_usbdev.bufoffset = 0x180;

  /* Device mode enable */

  modifyreg32(USB_MODE, 0, USB_MODE_DEV_EN);

  /* auto address load mode */

  modifyreg32(USB_MODE, 0, USB_MODE_ADDR_LDMOD);
  modifyreg32(USB_MODE, 0, USB_MODE_DEV_INTMOD);

  ret = CLASS_BIND(driver, &g_usbdev.usbdev);

  if (ret)
    {
      usbtrace(TRACE_DEVERROR(LC823450_TRACEERR_BINDFAILED), (uint16_t)-ret);
      g_usbdev.driver = NULL;
      return ret;
    }
  else
    {
      /* Bus Reset End interrupt */

      modifyreg32(USB_DEVC, 0, USB_DEVC_USBRSTE);

      /* Setup start interrupt */

      modifyreg32(USB_DEVC, 0, USB_DEVC_SETUP);

      /* Setup sus/res interrupt */

      modifyreg32(USB_DEVC, 0, USB_DEVC_SUSPENDB | USB_DEVC_SUSPENDE);

      /* Enable USB controller interrupts */

      putreg32(0xffff0002, USB_INTEN);

      g_usbdev.eplist[0].type = 0;
      g_usbdev.eplist[0].in = 0;
      lc823450_epconfigure(&g_usbdev.eplist[0].ep, NULL, 0);

      lc823450_usbpullup(&g_usbdev.usbdev, true);

      /* Detect AC-Charger */

      /* clear sof intr */

      putreg32(~USB_DEVS_SOF, USB_DEVS);

      nxsig_usleep(100000);

      /* SOF is not arrived & D+/D- is HIGH */

      if (!(getreg32(USB_DEVS) & USB_DEVS_SOF) &&
          (getreg32(USBSTAT) & USBSTAT_LINESTE_MASK) ==
          (USBSTAT_LINESTE_0 | USBSTAT_LINESTE_1))
        {
#ifdef CONFIG_USBDEV_CHARGER
          g_usbdev.charger = 1;
#endif /* CONFIG_USBDEV_CHARGER */
          uinfo("charger detect\n");
#if defined(CONFIG_BATTERY) && !defined(CONFIG_BATTERY_DISABLE_CHARGE)
          battery_sendevent(BATTERY_USBEV_CHG);
#endif
        }

#ifdef CONFIG_PM
      pm_register(&g_pm_cb);
#endif /* CONFIG_PM */
      up_enable_irq(LC823450_IRQ_USBDEV);
    }

#if defined(CONFIG_LC823450_USBDEV_CUSTOM_HSDSEL_5)
  modifyreg32(USB_CUSTOMC, USB_CUSTOMC_HSDSEL_MASK, USB_CUSTOMC_HSDSEL_5);
#elif defined(CONFIG_LC823450_USBDEV_CUSTOM_HSDSEL_10)
  modifyreg32(USB_CUSTOMC, USB_CUSTOMC_HSDSEL_MASK, USB_CUSTOMC_HSDSEL_10);
#endif

  ret = 0;
  return ret;
}

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver. If the USB device is connected to a
 *   USB host, it will first disconnect().  The driver is also requested to
 *   unbind() and clean up any device state, before this procedure finally
 *   returns.
 *
 ****************************************************************************/

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  /* At present, there is only a single USB device support. Hence it is
   * pre-allocated as g_otgfsdev.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple
   * devices.
   */

  struct lc823450_usbdev_s *priv = &g_usbdev;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG
  if (driver != priv->driver)
    {
      usbtrace(TRACE_DEVERROR(LC823450_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Reset the hardware and cancel all requests.  All requests must be
   * canceled while the class driver is still bound.
   */

  flags = spin_lock_irqsave(NULL);

#ifdef CONFIG_WAKELOCK
  /* cancel USB suspend work */

  work_cancel(HPWORK, &g_suspend_work);
#endif

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable interrupts */

  up_disable_irq(LC823450_IRQ_USBDEV);

  /* Disconnect device */

  lc823450_usbpullup(&priv->usbdev, false);

  /* Unhook the driver */

  modifyreg32(MCLKCNTBASIC, MCLKCNTBASIC_USBDEV_CLKEN, 0);
  modifyreg32(USBCNT, 0, USBCNT_ANPD);
  modifyreg32(MRSTCNTBASIC, MRSTCNTBASIC_USBDEV_RSTB, 0);

#ifdef CONFIG_USBDEV_CHARGER
  priv->charger = 0;
#endif /* CONFIG_USBDEV_CHARGER */

  priv->driver = NULL;

#ifdef CONFIG_PM
  pm_unregister(&g_pm_cb);
#endif /* CONFIG_PM */

  spin_unlock_irqrestore(NULL, flags);

#ifdef CONFIG_LC823450_LSISTBY
  /* disable USB */

  mod_stby_regs(0, LSISTBY_STBYE);
#endif

#ifdef CONFIG_WAKELOCK
  wake_unlock(&priv->wlock);
#endif /* CONFIG_WAKELOCK */

  return OK;
}

/* FOR USBMSC optimization */

#ifdef CONFIG_USBMSC_OPT
/****************************************************************************
 * Name: usbdev_msc_read_enter
 ****************************************************************************/

void usbdev_msc_read_enter(void)
{
  struct lc823450_ep_s *privep;
#  ifdef CONFIG_DVFS
  lc823450_dvfs_boost(DVFS_BOOST_TIMEOUT);
#  endif

  privep = &g_usbdev.eplist[CONFIG_USBMSC_EPBULKIN];
  privep->epcmd &= ~USB_EPCMD_EMPTY_EN;
  epcmd_write(CONFIG_USBMSC_EPBULKIN, (privep->epcmd));
  lc823450_dmareauest_dir(g_hdma, DMA_REQUEST_USBDEV, 1);
}

/****************************************************************************
 * Name: usbdev_msc_read_exit
 ****************************************************************************/

void usbdev_msc_read_exit(void)
{
  struct lc823450_ep_s *privep;

  privep = &g_usbdev.eplist[CONFIG_USBMSC_EPBULKIN];
  privep->epcmd |= USB_EPCMD_EMPTY_EN;
  epcmd_write(CONFIG_USBMSC_EPBULKIN, (privep->epcmd));
}

/****************************************************************************
 * Name: usbdev_dma_callback
 ****************************************************************************/

static void usbdev_dma_callback(DMA_HANDLE hd, void *arg, int result)
{
  sem_t *waitsem = (sem_t *)arg;
  nxsem_post(waitsem);
}

/****************************************************************************
 * Name: usbdev_msc_epwrite
 ****************************************************************************/

int usbdev_msc_epwrite(void *buf, int len)
{
  int i;
  struct lc823450_ep_s *privep;
  uint32_t ctrl;
  uint32_t pksz;

  pksz = g_usbdev.eplist[CONFIG_USBMSC_EPBULKIN].ep.maxpacket;

  privep = &g_usbdev.eplist[CONFIG_USBMSC_EPBULKIN];

  ctrl = LC823450_DMA_SRCWIDTH_BYTE |
         LC823450_DMA_DSTWIDTH_BYTE |
         LC823450_DMA_SRCINC | LC823450_DMA_DSTINC |
         LC823450_DMA_BS_64 << LC823450_DMA_SBS_SHIFT |
         LC823450_DMA_BS_64 << LC823450_DMA_DBS_SHIFT;

  /* create dma link list */

  for (i = 1; i < len / pksz; i++)
    {
      g_dma_list[i - 1].srcaddr = (uint32_t)buf + i * pksz;
      g_dma_list[i - 1].dstaddr = (uint32_t)privep->inbuf + 0x8000;
      if (i == (len / pksz) - 1)
        {
          /* last link */

          g_dma_list[i - 1].nextlli = 0;
          g_dma_list[i - 1].ctrl = ctrl | LC823450_DMA_ITC | pksz;
        }
      else
        {
          g_dma_list[i - 1].nextlli = (uint32_t)&g_dma_list[i];
          g_dma_list[i - 1].ctrl = ctrl | pksz;
        }
    }

  lc823450_dmallsetup(g_hdma,
                      ctrl,
                      (uint32_t)buf,
                      (uint32_t)privep->inbuf + 0x8000,
                      pksz,
                      len <= pksz ? 0 : (uint32_t)g_dma_list);

  lc823450_dmastart(g_hdma, usbdev_dma_callback, &dma_wait);

  putreg32(len, USB_DMATCI1);

  putreg32(64 << USB_DMAC_BSIZE_SHIFT |
           CONFIG_USBMSC_EPBULKIN  << USB_DMAC_DMAEP_SHIFT |
           USB_DMAC_START,
           USB_DMAC1);

  nxsem_wait(&dma_wait);
  return 0;
}

/****************************************************************************
 * Name: usbdev_msc_write_enter0
 ****************************************************************************/

void usbdev_msc_write_enter0(void)
{
  struct lc823450_ep_s *privep;

#  ifdef CONFIG_DVFS
  lc823450_dvfs_boost(DVFS_BOOST_TIMEOUT);
#  endif

  privep = &g_usbdev.eplist[CONFIG_USBMSC_EPBULKOUT];
  privep->epcmd &= ~USB_EPCMD_READY_EN;
  epcmd_write(CONFIG_USBMSC_EPBULKOUT, (privep->epcmd));
  lc823450_dmareauest_dir(g_hdma, DMA_REQUEST_USBDEV, 0);
}

/****************************************************************************
 * Name: usbdev_msc_write_enter
 ****************************************************************************/

void usbdev_msc_write_enter(void)
{
}

/****************************************************************************
 * Name: usbdev_msc_write_exit
 ****************************************************************************/

void usbdev_msc_write_exit(void)
{
  struct lc823450_ep_s *privep;

  epcmd_write(CONFIG_USBMSC_EPBULKOUT, USB_EPCMD_READY_CLR);

  /* Discard garbage packet: for USBCV MSCTEST11 */

  epcmd_write(CONFIG_USBMSC_EPBULKOUT, USB_EPCMD_BUFRD);
  privep = &g_usbdev.eplist[CONFIG_USBMSC_EPBULKOUT];
  privep->epcmd |= USB_EPCMD_READY_EN;
  epcmd_write(CONFIG_USBMSC_EPBULKOUT, (privep->epcmd));
}

/****************************************************************************
 * Name: usbdev_msc_epread
 ****************************************************************************/

int usbdev_msc_epread(void *buf, int len)
{
  int i;
  struct lc823450_ep_s *privep;
  uint32_t ctrl;
  uint32_t pksz;

  pksz = g_usbdev.eplist[CONFIG_USBMSC_EPBULKOUT].ep.maxpacket;

  privep = &g_usbdev.eplist[CONFIG_USBMSC_EPBULKOUT];

  ctrl = LC823450_DMA_SRCWIDTH_BYTE |
         LC823450_DMA_DSTWIDTH_BYTE |
         LC823450_DMA_SRCINC | LC823450_DMA_DSTINC |
         LC823450_DMA_BS_64 << LC823450_DMA_SBS_SHIFT |
         LC823450_DMA_BS_64 << LC823450_DMA_DBS_SHIFT;

  /* create dma link list */

  for (i = 1; i < len / pksz; i++)
    {
      g_dma_list[i - 1].srcaddr = (uint32_t)privep->outbuf + 0x8000;
      g_dma_list[i - 1].dstaddr = (uint32_t)buf + i * pksz;
      if (i == (len / pksz) - 1)
        {
          /* last link */

          g_dma_list[i - 1].nextlli = 0;
          g_dma_list[i - 1].ctrl = ctrl | LC823450_DMA_ITC | pksz;
        }
      else
        {
          g_dma_list[i - 1].nextlli = (uint32_t)&g_dma_list[i];
          g_dma_list[i - 1].ctrl = ctrl | pksz;
        }
    }

  lc823450_dmallsetup(g_hdma,
                      ctrl,
                      (uint32_t)privep->outbuf + 0x8000,
                      (uint32_t)buf,
                      pksz,
                      len <= pksz ? 0 : (uint32_t)g_dma_list);

  lc823450_dmastart(g_hdma, usbdev_dma_callback, &dma_wait);

  putreg32(len, USB_DMATCI1);
  putreg32(64 << USB_DMAC_BSIZE_SHIFT |
           CONFIG_USBMSC_EPBULKOUT  << USB_DMAC_DMAEP_SHIFT |
           USB_DMAC_START,
           USB_DMAC1);
  nxsem_wait(&dma_wait);

  return 0;
}

void usbdev_msc_stop(void)
{
  lc823450_dmastop(g_hdma);
  nxsem_post(&dma_wait);
}
#endif /* CONFIG_USBMSC */

#ifdef CONFIG_USBDEV_CHARGER
/****************************************************************************
 * Name: usbdev_is_usbcharger
 *
 * return value : 0 : charger was not detected.
 *               !0 : charger was detected.
 ****************************************************************************/

int usbdev_is_usbcharger(void)
{
  return g_usbdev.charger;
}
#endif

#ifdef CONFIG_PM
static void usbdev_pmnotify(struct pm_callback_s *cb,
                            enum pm_state_e pmstate)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(NULL);

  switch (pmstate)
    {
      case PM_SLEEP:
        modifyreg32(USBCNT, 0, USBCNT_RSM_CONT);
        g_usbdev.suspended = 1;
        break;

      case PM_NORMAL:
        modifyreg32(USBCNT, USBCNT_RSM_CONT, 0);
        g_usbdev.suspended = 0;
        if (g_usbdev.driver)
          {
            CLASS_RESUME(g_usbdev.driver, &g_usbdev.usbdev);
          }

        break;

      default:
        break;
    }

  spin_unlock_irqrestore(NULL, flags);
}
#endif

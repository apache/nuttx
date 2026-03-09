/****************************************************************************
 * arch/arm64/src/imx9/imx9_dwc3dev.c
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
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/spinlock.h>
#include <nuttx/cache.h>

#include <arch/barriers.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm64_internal.h"
#include "hardware/imx9_memorymap.h"
#include "hardware/imx9_dwc3.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cache alignment */

#if !defined(ARMV8A_DCACHE_LINESIZE) || ARMV8A_DCACHE_LINESIZE == 0
#  undef ARMV8A_DCACHE_LINESIZE
#  define ARMV8A_DCACHE_LINESIZE 64
#endif

#define DCACHE_LINEMASK       (ARMV8A_DCACHE_LINESIZE - 1)
#define DCACHE_ALIGN_UP(a)    (((a) + DCACHE_LINEMASK) & ~DCACHE_LINEMASK)

#if !defined(CONFIG_ARM64_DCACHE_DISABLE)
#  define cache_aligned_alloc(s) kmm_memalign(ARMV8A_DCACHE_LINESIZE, (s))
#  define CACHE_ALIGNED_DATA     aligned_data(ARMV8A_DCACHE_LINESIZE)
#else
#  define cache_aligned_alloc    kmm_malloc
#  define CACHE_ALIGNED_DATA
#endif

/* Endpoint configuration */

#define IMX9_DWC3_NLOGENDPOINTS     8
#define IMX9_DWC3_NPHYSENDPOINTS    (IMX9_DWC3_NLOGENDPOINTS * 2)

/* Physical endpoint numbering: even = OUT, odd = IN */

#define IMX9_DWC3_EPPHY_OUT(log)    ((log) * 2)
#define IMX9_DWC3_EPPHY_IN(log)     ((log) * 2 + 1)
#define IMX9_DWC3_EP0_OUT           0
#define IMX9_DWC3_EP0_IN            1

#define IMX9_DWC3_EP_IS_IN(epphy)   (((epphy) & 1) != 0)

/* EP0 max packet size */

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#  define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

#define EP0_MAXPACKET               CONFIG_USBDEV_EP0_MAXSIZE
#define EP0_BUFSIZE                 512

/* Maximum packet sizes */

#define DWC3_SS_BULK_MAXPACKET      1024
#define DWC3_HS_BULK_MAXPACKET      512
#define DWC3_INTR_MAXPACKET         1024
#define DWC3_ISOC_MAXPACKET         1024

/* DEPCMD timeout */

#define DWC3_DEPCMD_TIMEOUT         5000

/* Trace error codes */

#define DWC3_TRACEERR_ALLOCFAIL         0x0101
#define DWC3_TRACEERR_BADEPNO           0x0102
#define DWC3_TRACEERR_BINDFAILED        0x0103
#define DWC3_TRACEERR_DRIVER            0x0104
#define DWC3_TRACEERR_DRIVERREGISTERED  0x0105
#define DWC3_TRACEERR_INVALIDPARMS      0x0106
#define DWC3_TRACEERR_IRQREGISTRATION   0x0107
#define DWC3_TRACEERR_NOEP              0x0108
#define DWC3_TRACEERR_REQABORTED        0x0109
#define DWC3_TRACEERR_DEPCMD_TIMEOUT    0x010a
#define DWC3_TRACEERR_COREID            0x010b

/* Trace interrupt codes */

#define DWC3_TRACEINTID_USB             0x0101
#define DWC3_TRACEINTID_DEVRESET        0x0102
#define DWC3_TRACEINTID_CONNDONE        0x0103
#define DWC3_TRACEINTID_DISCONNECT      0x0104
#define DWC3_TRACEINTID_SUSPEND         0x0105
#define DWC3_TRACEINTID_EP0SETUP        0x0106
#define DWC3_TRACEINTID_EP0COMPLETE     0x0107
#define DWC3_TRACEINTID_EPIN            0x0108
#define DWC3_TRACEINTID_EPOUT           0x0109

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* TRB - Transfer Request Block (hardware format, 16 bytes) */

struct imx9_dwc3_trb_s
{
  uint32_t bpl;   /* Buffer Pointer Low */
  uint32_t bph;   /* Buffer Pointer High */
  uint32_t size;  /* Transfer Size */
  uint32_t ctrl;  /* Control */
};

/* USB request container */

struct imx9_dwc3_req_s
{
  struct usbdev_req_s req;        /* Standard USB request */
  struct imx9_dwc3_req_s *flink;  /* Supports singly linked list */
};

/* Forward declaration */

struct imx9_dwc3_s;

/* Endpoint representation */

struct imx9_dwc3_ep_s
{
  struct usbdev_ep_s ep;                  /* Standard endpoint structure */
  struct imx9_dwc3_s *priv;               /* Reference to driver data */
  struct imx9_dwc3_req_s *head;           /* Request list head */
  struct imx9_dwc3_req_s *tail;           /* Request list tail */
  struct imx9_dwc3_trb_s *trb;            /* TRB array */
  uintptr_t trb_dma;                      /* TRB physical address */
  uint8_t epphy;                          /* Physical endpoint number */
  uint8_t eptype;                         /* Endpoint type */
  uint8_t resource_index;                 /* Transfer resource index */
  uint8_t trb_enqueue;                    /* TRB enqueue pointer */
  uint8_t trb_dequeue;                    /* TRB dequeue pointer */
  bool isin;                              /* true = IN endpoint */
  bool stalled;                           /* Endpoint is stalled */
  bool wedged;                            /* Endpoint is wedged */
  bool xfer_started;                      /* Transfer in progress */
};

/* EP0 state machine */

enum imx9_dwc3_ep0state_e
{
  EP0_IDLE = 0,
  EP0_SETUP_PHASE,
  EP0_DATA_PHASE,
  EP0_STATUS_PHASE,
};

/* Controller state */

struct imx9_dwc3_s
{
  struct usbdev_s usbdev;                  /* NuttX USB device */
  struct usbdevclass_driver_s *driver;     /* Bound class driver */
  uintptr_t base;                          /* DWC3 register base */
  uintptr_t glue_base;                     /* Glue layer base */
  uintptr_t phy_base;                      /* USB3 PHY register base */
  int irq;                                 /* IRQ number */

  /* Event buffer */

  uint32_t *evtbuf;                        /* Event buffer */
  uintptr_t evtbuf_dma;                    /* Event buffer DMA addr */
  unsigned int evt_lpos;                   /* Event read position */

  /* EP0 state */

  struct usb_ctrlreq_s ctrl;               /* Current SETUP packet */
  uint8_t ep0state;                        /* EP0 state machine */
  uint8_t *ep0buf;                         /* EP0 data buffer */
  struct imx9_dwc3_trb_s *ep0trb;          /* EP0 TRBs */
  uintptr_t ep0trb_dma;                    /* EP0 TRB DMA addr */
  uint8_t *ep0_setup_buf;                  /* SETUP packet buffer */
  uintptr_t ep0_setup_dma;                 /* SETUP buffer DMA addr */
  bool ep0_expect_in;                      /* EP0 data direction */
  bool three_stage_setup;                  /* 3-stage (data) or 2-stage */
  uint16_t ep0_xfer_len;                   /* EP0 transfer length */

  /* Endpoints */

  struct imx9_dwc3_ep_s eplist[IMX9_DWC3_NPHYSENDPOINTS];

  /* Device state */

  uint8_t devaddr;                         /* Device address */
  uint8_t speed;                           /* Connection speed */
  bool softconnect;                        /* Soft connect state */
  bool selfpowered;                        /* Self powered */
  uint32_t epavail;                        /* Available endpoints */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register access */

static inline uint32_t dwc3_getreg(struct imx9_dwc3_s *priv,
                                   uint32_t offset);
static inline void dwc3_putreg(struct imx9_dwc3_s *priv,
                               uint32_t offset, uint32_t val);
static inline void dwc3_modifyreg(struct imx9_dwc3_s *priv,
                                  uint32_t offset,
                                  uint32_t clear, uint32_t set);

/* Request queue */

static struct imx9_dwc3_req_s *dwc3_rqdequeue(
    struct imx9_dwc3_ep_s *privep);
static bool dwc3_rqenqueue(struct imx9_dwc3_ep_s *privep,
                           struct imx9_dwc3_req_s *req);

/* Low-level DWC3 operations */

static int dwc3_send_ep_cmd(struct imx9_dwc3_s *priv, uint8_t epphy,
                            uint32_t cmd, uint32_t par0,
                            uint32_t par1, uint32_t par2);
static int dwc3_send_gadget_cmd(struct imx9_dwc3_s *priv,
                                uint32_t cmd, uint32_t par);
static void dwc3_ep_start_transfer(struct imx9_dwc3_s *priv,
                                   struct imx9_dwc3_ep_s *privep,
                                   struct imx9_dwc3_trb_s *trb,
                                   uintptr_t trb_dma);
static void dwc3_ep0_start_setup(struct imx9_dwc3_s *priv);

/* Transfer management */

static void dwc3_reqcomplete(struct imx9_dwc3_ep_s *privep,
                             struct imx9_dwc3_req_s *privreq,
                             int16_t result);
static void dwc3_cancelrequests(struct imx9_dwc3_ep_s *privep,
                                int16_t status);

/* Interrupt handling */

static void dwc3_handle_ep0_setup(struct imx9_dwc3_s *priv);
static void dwc3_handle_ep0_xfer_complete(struct imx9_dwc3_s *priv,
                                          uint8_t epphy);
static void dwc3_handle_ep0_xfer_not_ready(struct imx9_dwc3_s *priv,
                                           uint8_t epphy,
                                           uint32_t event);
static void dwc3_handle_ep_xfer_complete(struct imx9_dwc3_s *priv,
                                         uint8_t epphy);
static void dwc3_handle_ep_event(struct imx9_dwc3_s *priv,
                                 uint32_t event);
static void dwc3_handle_dev_event(struct imx9_dwc3_s *priv,
                                  uint32_t event);
static void dwc3_handle_reset(struct imx9_dwc3_s *priv);
static void dwc3_handle_conndone(struct imx9_dwc3_s *priv);
static int dwc3_interrupt(int irq, void *context, void *arg);

/* Endpoint operations (usbdev_epops_s) */

static int dwc3_epconfigure(struct usbdev_ep_s *ep,
                            const struct usb_epdesc_s *desc, bool last);
static int dwc3_epdisable(struct usbdev_ep_s *ep);
static struct usbdev_req_s *dwc3_epallocreq(struct usbdev_ep_s *ep);
static void dwc3_epfreereq(struct usbdev_ep_s *ep,
                           struct usbdev_req_s *req);
#ifdef CONFIG_USBDEV_DMA
static void *dwc3_epallocbuffer(struct usbdev_ep_s *ep, uint16_t bytes);
static void dwc3_epfreebuffer(struct usbdev_ep_s *ep, void *buf);
#endif
static int dwc3_epsubmit(struct usbdev_ep_s *ep,
                         struct usbdev_req_s *req);
static int dwc3_epcancel(struct usbdev_ep_s *ep,
                         struct usbdev_req_s *req);
static int dwc3_epstall(struct usbdev_ep_s *ep, bool resume);

/* Device operations (usbdev_ops_s) */

static struct usbdev_ep_s *dwc3_allocep(struct usbdev_s *dev,
                                        uint8_t epno, bool in,
                                        uint8_t eptype);
static void dwc3_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep);
static int dwc3_getframe(struct usbdev_s *dev);
static int dwc3_wakeup(struct usbdev_s *dev);
static int dwc3_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int dwc3_pullup(struct usbdev_s *dev, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_dwc3_ep0buf[EP0_BUFSIZE] CACHE_ALIGNED_DATA;

static struct imx9_dwc3_s g_dwc3dev;

static const struct usbdev_epops_s g_dwc3_epops =
{
  .configure   = dwc3_epconfigure,
  .disable     = dwc3_epdisable,
  .allocreq    = dwc3_epallocreq,
  .freereq     = dwc3_epfreereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = dwc3_epallocbuffer,
  .freebuffer  = dwc3_epfreebuffer,
#endif
  .submit      = dwc3_epsubmit,
  .cancel      = dwc3_epcancel,
  .stall       = dwc3_epstall,
};

static const struct usbdev_ops_s g_dwc3_devops =
{
  .allocep     = dwc3_allocep,
  .freeep      = dwc3_freeep,
  .getframe    = dwc3_getframe,
  .wakeup      = dwc3_wakeup,
  .selfpowered = dwc3_selfpowered,
  .pullup      = dwc3_pullup,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Register Access
 ****************************************************************************/

static inline uint32_t dwc3_getreg(struct imx9_dwc3_s *priv,
                                   uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static inline void dwc3_putreg(struct imx9_dwc3_s *priv,
                               uint32_t offset, uint32_t val)
{
  putreg32(val, priv->base + offset);
}

static inline void dwc3_modifyreg(struct imx9_dwc3_s *priv,
                                  uint32_t offset,
                                  uint32_t clear, uint32_t set)
{
  uint32_t reg = dwc3_getreg(priv, offset);

  reg &= ~clear;
  reg |= set;
  dwc3_putreg(priv, offset, reg);
}

static inline uint32_t dwc3_glue_getreg(struct imx9_dwc3_s *priv,
                                        uint32_t offset)
{
  return getreg32(priv->glue_base + offset);
}

static inline void dwc3_glue_putreg(struct imx9_dwc3_s *priv,
                                    uint32_t offset, uint32_t val)
{
  putreg32(val, priv->glue_base + offset);
}

/****************************************************************************
 * Request Queue Operations
 ****************************************************************************/

static struct imx9_dwc3_req_s *dwc3_rqdequeue(
    struct imx9_dwc3_ep_s *privep)
{
  struct imx9_dwc3_req_s *ret = privep->head;

  if (ret)
    {
      privep->head = ret->flink;
      if (!privep->head)
        {
          privep->tail = NULL;
        }

      ret->flink = NULL;
    }

  return ret;
}

static bool dwc3_rqenqueue(struct imx9_dwc3_ep_s *privep,
                           struct imx9_dwc3_req_s *req)
{
  bool is_empty = !privep->head;

  req->flink = NULL;
  if (is_empty)
    {
      privep->head = req;
      privep->tail = req;
    }
  else
    {
      privep->tail->flink = req;
      privep->tail = req;
    }

  return is_empty;
}

/****************************************************************************
 * DWC3 Endpoint Command Interface
 ****************************************************************************/

static int dwc3_send_ep_cmd(struct imx9_dwc3_s *priv, uint8_t epphy,
                            uint32_t cmd, uint32_t par0,
                            uint32_t par1, uint32_t par2)
{
  uint32_t reg;
  int timeout = DWC3_DEPCMD_TIMEOUT;

  dwc3_putreg(priv, DWC3_DEPCMDPAR0(epphy), par0);
  dwc3_putreg(priv, DWC3_DEPCMDPAR1(epphy), par1);
  dwc3_putreg(priv, DWC3_DEPCMDPAR2(epphy), par2);

  dwc3_putreg(priv, DWC3_DEPCMD(epphy), cmd | DWC3_DEPCMD_CMDACT);

  do
    {
      reg = dwc3_getreg(priv, DWC3_DEPCMD(epphy));
      if (!(reg & DWC3_DEPCMD_CMDACT))
        {
          int status = (reg & DWC3_DEPCMD_STATUS_MASK) >>
                       DWC3_DEPCMD_STATUS_SHIFT;

          if (status != 0)
            {
              uerr("DEPCMD ep%d cmd 0x%x failed status %d\n",
                   epphy, cmd & 0xf, status);
              return -EIO;
            }

          return OK;
        }

      up_udelay(1);
    }
  while (--timeout > 0);

  usbtrace(TRACE_DEVERROR(DWC3_TRACEERR_DEPCMD_TIMEOUT), epphy);
  return -ETIMEDOUT;
}

static int unused_code dwc3_send_gadget_cmd(struct imx9_dwc3_s *priv,
                                            uint32_t cmd, uint32_t par)
{
  int timeout = DWC3_DEPCMD_TIMEOUT;
  uint32_t reg;

  dwc3_putreg(priv, DWC3_DGCMDPAR, par);
  dwc3_putreg(priv, DWC3_DGCMD, cmd | DWC3_DGCMD_CMDACT);

  do
    {
      reg = dwc3_getreg(priv, DWC3_DGCMD);
      if (!(reg & DWC3_DGCMD_CMDACT))
        {
          return OK;
        }

      up_udelay(1);
    }
  while (--timeout > 0);

  return -ETIMEDOUT;
}

/****************************************************************************
 * TRB Management
 ****************************************************************************/

static void dwc3_trb_init(struct imx9_dwc3_trb_s *trb,
                          uintptr_t buf_dma, uint32_t len,
                          uint32_t ctrl)
{
  trb->bpl  = (uint32_t)(buf_dma & 0xffffffff);
  trb->bph  = (uint32_t)(buf_dma >> 32);
  trb->size = DWC3_TRB_SIZE_LENGTH(len);
  trb->ctrl = ctrl;

  UP_DSB();
}

/****************************************************************************
 * EP Start/End Transfer
 ****************************************************************************/

static void dwc3_ep_start_transfer(struct imx9_dwc3_s *priv,
                                   struct imx9_dwc3_ep_s *privep,
                                   struct imx9_dwc3_trb_s *trb,
                                   uintptr_t trb_dma)
{
  uint32_t par0;
  uint32_t par1;
  int ret;

  /* Flush TRB to memory so DMA can read it */

  up_clean_dcache((uintptr_t)trb,
                  (uintptr_t)trb + sizeof(struct imx9_dwc3_trb_s));

  /* Also flush the data buffer pointed to by the TRB */

  if (trb->bpl || trb->bph)
    {
      uintptr_t buf = (uintptr_t)trb->bpl |
                      ((uintptr_t)trb->bph << 32);
      uint32_t len = trb->size & DWC3_TRB_SIZE_MASK;

      if (len > 0)
        {
          up_clean_dcache(buf, buf + len);
        }
    }

  par0 = (uint32_t)(trb_dma >> 32);
  par1 = (uint32_t)(trb_dma & 0xffffffff);

  ret = dwc3_send_ep_cmd(priv, privep->epphy,
                         DWC3_DEPCMD_STARTTRANSFER, par0, par1, 0);
  if (ret == OK)
    {
      uint32_t cmd_reg = dwc3_getreg(priv, DWC3_DEPCMD(privep->epphy));

      privep->resource_index = DWC3_DEPCMD_GET_RSC_IDX(cmd_reg);
      privep->xfer_started = true;
    }
  else
    {
      uerr("DWC3: STARTTRANSFER FAILED ep=%d ret=%d\n",
             privep->epphy, ret);
    }
}

static void dwc3_ep_end_transfer(struct imx9_dwc3_s *priv,
                                 struct imx9_dwc3_ep_s *privep)
{
  uint32_t cmd;

  if (!privep->xfer_started)
    {
      return;
    }

  cmd = DWC3_DEPCMD_ENDTRANSFER |
        DWC3_DEPCMD_PARAM(privep->resource_index) |
        DWC3_DEPCMD_CMDIOC;

  dwc3_send_ep_cmd(priv, privep->epphy, cmd, 0, 0, 0);
  privep->xfer_started = false;
  privep->resource_index = 0;
}

/****************************************************************************
 * EP0 Setup Phase
 ****************************************************************************/

static void dwc3_ep0_start_setup(struct imx9_dwc3_s *priv)
{
  struct imx9_dwc3_trb_s *trb = priv->ep0trb;
  uintptr_t dma = priv->ep0_setup_dma;

  /* Prepare TRB for SETUP packet reception */

  dwc3_trb_init(trb, dma, 8,
                DWC3_TRBCTL_CONTROL_SETUP |
                DWC3_TRB_CTRL_LST |
                DWC3_TRB_CTRL_IOC |
                DWC3_TRB_CTRL_HWO);

  priv->ep0state = EP0_SETUP_PHASE;

  dwc3_ep_start_transfer(priv, &priv->eplist[IMX9_DWC3_EP0_OUT],
                         trb, priv->ep0trb_dma);
}

/****************************************************************************
 * EP0 Data/Status Phase
 ****************************************************************************/

static void dwc3_ep0_start_data(struct imx9_dwc3_s *priv,
                                uintptr_t buf_dma, uint32_t len,
                                bool is_in)
{
  struct imx9_dwc3_trb_s *trb = priv->ep0trb;
  uint8_t epphy = is_in ? IMX9_DWC3_EP0_IN : IMX9_DWC3_EP0_OUT;

  dwc3_trb_init(trb, buf_dma, len,
                DWC3_TRBCTL_CONTROL_DATA |
                DWC3_TRB_CTRL_LST |
                DWC3_TRB_CTRL_IOC |
                DWC3_TRB_CTRL_ISP_IMI |
                DWC3_TRB_CTRL_HWO);

  priv->ep0state = EP0_DATA_PHASE;

  dwc3_ep_start_transfer(priv, &priv->eplist[epphy],
                         trb, priv->ep0trb_dma);
}

static void dwc3_ep0_start_status(struct imx9_dwc3_s *priv, bool is_in)
{
  struct imx9_dwc3_trb_s *trb = priv->ep0trb;
  uint8_t epphy = is_in ? IMX9_DWC3_EP0_IN : IMX9_DWC3_EP0_OUT;
  uint32_t trbctl;

  /* Per DWC3 databook and Linux driver:
   * 3-stage (has data phase) -> CONTROL_STATUS3
   * 2-stage (no data, e.g. SET_ADDRESS) -> CONTROL_STATUS2
   */

  trbctl = priv->three_stage_setup ? DWC3_TRBCTL_CONTROL_STATUS3
                                   : DWC3_TRBCTL_CONTROL_STATUS2;

  dwc3_trb_init(trb, priv->ep0trb_dma, 0,
                trbctl |
                DWC3_TRB_CTRL_LST |
                DWC3_TRB_CTRL_IOC |
                DWC3_TRB_CTRL_HWO);

  priv->ep0state = EP0_STATUS_PHASE;

  dwc3_ep_start_transfer(priv, &priv->eplist[epphy],
                         trb, priv->ep0trb_dma);
}

/****************************************************************************
 * Request Complete / Cancel
 ****************************************************************************/

static void dwc3_reqcomplete(struct imx9_dwc3_ep_s *privep,
                             struct imx9_dwc3_req_s *privreq,
                             int16_t result)
{
  bool is_in = privep->isin;

  usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);

  privreq->req.result = result;

  /* Callback */

  if (privreq->req.callback)
    {
      privreq->req.callback(&privep->ep, &privreq->req);
    }

  UNUSED(is_in);
}

static void dwc3_cancelrequests(struct imx9_dwc3_ep_s *privep,
                                int16_t status)
{
  struct imx9_dwc3_req_s *req;

  /* End any active transfer */

  if (privep->xfer_started && privep->priv)
    {
      dwc3_ep_end_transfer(privep->priv, privep);
    }

  while ((req = dwc3_rqdequeue(privep)) != NULL)
    {
      dwc3_reqcomplete(privep, req, status);
    }
}

/****************************************************************************
 * EP0 Setup Handling
 ****************************************************************************/

static void dwc3_dispatch_setup(struct imx9_dwc3_s *priv)
{
  if (priv->driver)
    {
      int ret;

      ret = CLASS_SETUP(priv->driver, &priv->usbdev,
                        &priv->ctrl, priv->ep0buf,
                        priv->ep0_xfer_len);
      if (ret < 0)
        {
          /* Stall EP0 */

          dwc3_epstall(&priv->eplist[IMX9_DWC3_EP0_OUT].ep, false);
          dwc3_epstall(&priv->eplist[IMX9_DWC3_EP0_IN].ep, false);
        }
    }
}

static void dwc3_handle_ep0_setup(struct imx9_dwc3_s *priv)
{
  struct usb_ctrlreq_s *ctrl = &priv->ctrl;
  uint16_t value;
  uint16_t len;

  usbtrace(TRACE_INTDECODE(DWC3_TRACEINTID_EP0SETUP), 0);

  /* Invalidate setup buffer cache - DMA has written to it */

  up_invalidate_dcache((uintptr_t)priv->ep0_setup_buf,
                       (uintptr_t)priv->ep0_setup_buf + 8);

  /* Also invalidate the TRB to read completion status */

  up_invalidate_dcache((uintptr_t)priv->ep0trb,
                       (uintptr_t)priv->ep0trb +
                       2 * sizeof(struct imx9_dwc3_trb_s));

  /* Copy SETUP data from DMA buffer */

  memcpy(ctrl, priv->ep0_setup_buf, USB_SIZEOF_CTRLREQ);

  value = GETUINT16(ctrl->value);
  len   = GETUINT16(ctrl->len);

  priv->ep0_expect_in = (ctrl->type & USB_REQ_DIR_IN) != 0;
  priv->ep0_xfer_len  = len;

  /* Determine 2-stage vs 3-stage setup (like Linux) */

  if (len == 0)
    {
      priv->three_stage_setup = false;
      priv->ep0_expect_in = false;
    }
  else
    {
      priv->three_stage_setup = true;
    }

  /* Handle SET_ADDRESS specially - DWC3 handles address in HW */

  if (ctrl->type == USB_REQ_RECIPIENT_DEVICE &&
      ctrl->req == USB_REQ_SETADDRESS)
    {
      priv->devaddr = value & 0x7f;
      dwc3_modifyreg(priv, DWC3_DCFG,
                     DWC3_DCFG_DEVADDR_MASK,
                     DWC3_DCFG_DEVADDR(priv->devaddr));

      /* For 2-stage (no data) transfers like SET_ADDRESS,
       * we must wait for XferNotReady(STATUS) before sending
       * the status phase. Don't start status here.
       * Don't dispatch to class driver - SET_ADDRESS is handled
       * entirely by the DWC3 driver.
       */

      return;
    }

  /* Dispatch to class driver */

  dwc3_dispatch_setup(priv);
}

/****************************************************************************
 * EP0 Transfer Complete
 ****************************************************************************/

static void dwc3_handle_ep0_xfer_complete(struct imx9_dwc3_s *priv,
                                          uint8_t epphy)
{
  struct imx9_dwc3_ep_s *privep = &priv->eplist[epphy];

  privep->xfer_started = false;

  switch (priv->ep0state)
    {
      case EP0_SETUP_PHASE:
        dwc3_handle_ep0_setup(priv);
        break;

      case EP0_DATA_PHASE:
        {
          /* Data phase complete.
           * Per Linux DWC3 driver, we should wait for
           * XferNotReady(STATUS) before starting status phase.
           * Just mark data phase done; XferNotReady handler
           * will start status.
           */

          /* Complete the EP0 request */

          struct imx9_dwc3_ep_s *ep0ep = &priv->eplist[epphy];
          struct imx9_dwc3_req_s *privreq = dwc3_rqdequeue(ep0ep);

          if (privreq)
            {
              /* Invalidate TRB to read actual transfer size */

              up_invalidate_dcache((uintptr_t)priv->ep0trb,
                                   (uintptr_t)priv->ep0trb +
                                   sizeof(struct imx9_dwc3_trb_s));

              uint32_t remaining = priv->ep0trb->size &
                                   DWC3_TRB_SIZE_MASK;

              privreq->req.xfrd = privreq->req.len - remaining;
              dwc3_reqcomplete(ep0ep, privreq, OK);
            }
        }
        break;

      case EP0_STATUS_PHASE:

        /* Status phase complete, restart setup */

        priv->ep0state = EP0_IDLE;
        dwc3_ep0_start_setup(priv);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * EP0 Transfer Not Ready
 ****************************************************************************/

static void dwc3_handle_ep0_xfer_not_ready(struct imx9_dwc3_s *priv,
                                           uint8_t epphy,
                                           uint32_t event)
{
  uint32_t status = DWC3_DEPEVT_STATUS(event);

  switch (status)
    {
      case DWC3_DEPEVT_STATUS_CONTROL_DATA:
        /* Host wants data phase - we should have already started
         * data transfer from dispatch. If direction mismatch,
         * stall.
         */

        break;

      case DWC3_DEPEVT_STATUS_CONTROL_STATUS:
        /* Host is ready for status phase.
         * For 2-stage (no data): send status now
         * For 3-stage: status is sent after data phase completes
         */

        {
          bool is_in = (epphy == IMX9_DWC3_EP0_IN);

          priv->ep0state = EP0_STATUS_PHASE;
          dwc3_ep0_start_status(priv, is_in);
        }
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Non-EP0 Transfer Complete
 ****************************************************************************/

static void dwc3_handle_ep_xfer_complete(struct imx9_dwc3_s *priv,
                                         uint8_t epphy)
{
  struct imx9_dwc3_ep_s *privep = &priv->eplist[epphy];
  struct imx9_dwc3_req_s *privreq;
  struct imx9_dwc3_trb_s *trb;
  uint32_t remaining;

  privep->xfer_started = false;

  privreq = dwc3_rqdequeue(privep);
  if (!privreq)
    {
      return;
    }

  /* Read back TRB to get actual transfer size */

  trb = &privep->trb[privep->trb_dequeue];

  /* Invalidate TRB cache - DMA has updated it */

  up_invalidate_dcache((uintptr_t)trb,
                       (uintptr_t)trb + sizeof(struct imx9_dwc3_trb_s));
  UP_DSB();

  remaining = DWC3_TRB_SIZE_LENGTH(trb->size);
  privreq->req.xfrd = privreq->req.len - remaining;

  /* Invalidate data buffer if this was an OUT transfer */

  if (!privep->isin && privreq->req.xfrd > 0)
    {
      up_invalidate_dcache((uintptr_t)privreq->req.buf,
                           (uintptr_t)privreq->req.buf +
                           privreq->req.xfrd);
    }

  dwc3_reqcomplete(privep, privreq, OK);

  /* Start next queued request if any */

  privreq = privep->head;
  if (privreq)
    {
      uintptr_t buf_dma = (uintptr_t)privreq->req.buf;
      uint32_t len = privreq->req.len;
      uint32_t ctrl;

      ctrl = DWC3_TRBCTL_NORMAL |
             DWC3_TRB_CTRL_LST |
             DWC3_TRB_CTRL_IOC |
             DWC3_TRB_CTRL_ISP_IMI |
             DWC3_TRB_CTRL_HWO;

      privep->trb_enqueue = 0;
      privep->trb_dequeue = 0;

      dwc3_trb_init(&privep->trb[0], buf_dma, len, ctrl);

      dwc3_ep_start_transfer(priv, privep,
                             &privep->trb[0], privep->trb_dma);
    }
}

/****************************************************************************
 * Event Handling
 ****************************************************************************/

static void dwc3_handle_ep_event(struct imx9_dwc3_s *priv,
                                 uint32_t event)
{
  uint8_t epphy = DWC3_DEPEVT_EPNUM(event);
  uint32_t type = DWC3_DEPEVT_TYPE(event);

  if (epphy >= IMX9_DWC3_NPHYSENDPOINTS)
    {
      return;
    }

  switch (type)
    {
      case DWC3_DEPEVT_XFERCOMPLETE:
        if (epphy <= IMX9_DWC3_EP0_IN)
          {
            dwc3_handle_ep0_xfer_complete(priv, epphy);
          }
        else
          {
            dwc3_handle_ep_xfer_complete(priv, epphy);
          }
        break;

      case DWC3_DEPEVT_XFERNOTREADY:
        if (epphy <= IMX9_DWC3_EP0_IN)
          {
            dwc3_handle_ep0_xfer_not_ready(priv, epphy, event);
          }
        break;

      case DWC3_DEPEVT_EPCMDCMPLT:
        break;

      default:
        break;
    }
}

static void dwc3_handle_reset(struct imx9_dwc3_s *priv)
{
  int ret;
  int i;

  usbtrace(TRACE_INTDECODE(DWC3_TRACEINTID_DEVRESET), 0);

  /* End any active EP0 transfers first - this is critical!
   * STARTTRANSFER will fail with status=1 if there's already
   * an active transfer on the endpoint.
   */

  if (priv->eplist[IMX9_DWC3_EP0_OUT].xfer_started)
    {
      dwc3_ep_end_transfer(priv, &priv->eplist[IMX9_DWC3_EP0_OUT]);
    }

  if (priv->eplist[IMX9_DWC3_EP0_IN].xfer_started)
    {
      dwc3_ep_end_transfer(priv, &priv->eplist[IMX9_DWC3_EP0_IN]);
    }

  /* Cancel all pending requests on non-EP0 endpoints */

  for (i = 2; i < IMX9_DWC3_NPHYSENDPOINTS; i++)
    {
      dwc3_cancelrequests(&priv->eplist[i], -ECONNRESET);
    }

  /* Reset device address */

  priv->devaddr = 0;
  dwc3_modifyreg(priv, DWC3_DCFG, DWC3_DCFG_DEVADDR_MASK, 0);

  /* Clear test mode bits in DCTL */

  dwc3_modifyreg(priv, DWC3_DCTL, DWC3_DCTL_TSTCTRL_MASK, 0);

  /* Clear stall on EP0 */

  ret = dwc3_send_ep_cmd(priv, IMX9_DWC3_EP0_OUT,
                         DWC3_DEPCMD_CLEARSTALL, 0, 0, 0);
  if (ret < 0)
    {
      uwarn("DWC3: EP0 OUT CLEARSTALL failed: %d\n", ret);
    }

  ret = dwc3_send_ep_cmd(priv, IMX9_DWC3_EP0_IN,
                         DWC3_DEPCMD_CLEARSTALL, 0, 0, 0);
  if (ret < 0)
    {
      uwarn("DWC3: EP0 IN CLEARSTALL failed: %d\n", ret);
    }

  /* Notify class driver */

  if (priv->driver)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }

  /* Reset EP0 state and restart setup phase */

  priv->ep0state = EP0_SETUP_PHASE;
  dwc3_ep0_start_setup(priv);
}

static void dwc3_handle_conndone(struct imx9_dwc3_s *priv)
{
  uint32_t dsts;
  uint32_t speed;
  uint16_t ep0_maxpacket;
  int ret;

  usbtrace(TRACE_INTDECODE(DWC3_TRACEINTID_CONNDONE), 0);

  dsts = dwc3_getreg(priv, DWC3_DSTS);
  speed = dsts & DWC3_DSTS_CONNECTSPD_MASK;

  switch (speed)
    {
      case DWC3_DSTS_SUPERSPEED:
        priv->usbdev.speed = USB_SPEED_SUPER;
        ep0_maxpacket = 512;
        break;

      case DWC3_DSTS_HIGHSPEED:
        priv->usbdev.speed = USB_SPEED_HIGH;
        ep0_maxpacket = 64;
        break;

      case DWC3_DSTS_FULLSPEED:
      default:
        priv->usbdev.speed = USB_SPEED_FULL;
        ep0_maxpacket = 64;
        break;
    }

  priv->eplist[IMX9_DWC3_EP0_OUT].ep.maxpacket = ep0_maxpacket;
  priv->eplist[IMX9_DWC3_EP0_IN].ep.maxpacket = ep0_maxpacket;

  /* Reconfigure EP0 with new maxpacket if needed */

  /* Clear stall on EP0 */

  ret = dwc3_send_ep_cmd(priv, IMX9_DWC3_EP0_OUT,
                         DWC3_DEPCMD_CLEARSTALL, 0, 0, 0);
  if (ret < 0)
    {
      uwarn("DWC3: EP0 OUT CLEARSTALL failed: %d\n", ret);
    }

  ret = dwc3_send_ep_cmd(priv, IMX9_DWC3_EP0_IN,
                         DWC3_DEPCMD_CLEARSTALL, 0, 0, 0);
  if (ret < 0)
    {
      uwarn("DWC3: EP0 IN CLEARSTALL failed: %d\n", ret);
    }
}

static void dwc3_handle_dev_event(struct imx9_dwc3_s *priv,
                                  uint32_t event)
{
  uint32_t type = DWC3_DEVT_TYPE(event);

  switch (type)
    {
      case DWC3_DEVICE_EVENT_DISCONNECT:
        usbtrace(TRACE_INTDECODE(DWC3_TRACEINTID_DISCONNECT), 0);
        if (priv->driver)
          {
            CLASS_DISCONNECT(priv->driver, &priv->usbdev);
          }

        priv->usbdev.speed = USB_SPEED_UNKNOWN;
        break;

      case DWC3_DEVICE_EVENT_RESET:
        dwc3_handle_reset(priv);
        break;

      case DWC3_DEVICE_EVENT_CONNECT_DONE:
        dwc3_handle_conndone(priv);
        break;

      case DWC3_DEVICE_EVENT_SUSPEND:
        usbtrace(TRACE_INTDECODE(DWC3_TRACEINTID_SUSPEND), 0);
        if (priv->driver)
          {
            CLASS_SUSPEND(priv->driver, &priv->usbdev);
          }
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Interrupt Handler
 ****************************************************************************/

static int dwc3_interrupt(int irq, void *context, void *arg)
{
  struct imx9_dwc3_s *priv = (struct imx9_dwc3_s *)arg;
  uint32_t count;
  uint32_t evt;
  uint32_t amount;
  uint32_t idx;

  /* Local cache for events - max reasonable batch is ~256 bytes (64 events).
   * If more, we process in chunks.
   */

  uint32_t cache[64];

  usbtrace(TRACE_INTENTRY(DWC3_TRACEINTID_USB), 0);

  count = dwc3_getreg(priv, DWC3_GEVNTCOUNT(0));
  count &= DWC3_GEVNTCOUNT_MASK;

  if (count == 0)
    {
      usbtrace(TRACE_INTEXIT(DWC3_TRACEINTID_USB), 0);
      return OK;
    }

  /* Mask events during processing (like Linux top-half) */

  dwc3_putreg(priv, DWC3_GEVNTSIZ(0),
              DWC3_GEVNTSIZ_INTMASK |
              (DWC3_EVENT_BUFFERS_SIZE & DWC3_GEVNTSIZ_SIZE_MASK));

  /* Invalidate event buffer cache before reading DMA-written events */

  up_invalidate_dcache((uintptr_t)priv->evtbuf,
                       (uintptr_t)priv->evtbuf + DWC3_EVENT_BUFFERS_SIZE);
  UP_DSB();

  /* Copy events from DMA buffer to local cache (like Linux does).
   * This ensures we read from invalidated cache lines atomically.
   */

  amount = count;
  if (amount > sizeof(cache))
    {
      amount = sizeof(cache);
    }

  if (priv->evt_lpos + amount <= DWC3_EVENT_BUFFERS_SIZE)
    {
      memcpy(cache, &priv->evtbuf[priv->evt_lpos / 4], amount);
    }
  else
    {
      uint32_t first = DWC3_EVENT_BUFFERS_SIZE - priv->evt_lpos;

      memcpy(cache, &priv->evtbuf[priv->evt_lpos / 4], first);
      memcpy((uint8_t *)cache + first, priv->evtbuf, amount - first);
    }

  /* ACK all events at once (like Linux) */

  dwc3_putreg(priv, DWC3_GEVNTCOUNT(0), count);

  /* Process events from local cache */

  for (idx = 0; idx < amount / 4; idx++)
    {
      evt = cache[idx];

      if (!DWC3_EVENT_IS_DEV_SPECIFIC(evt))
        {
          /* bit 0 = 0: endpoint event */

          dwc3_handle_ep_event(priv, evt);
        }
      else if ((evt & DWC3_EVENT_TYPE_MASK) == DWC3_EVENT_TYPE_DEV)
        {
          /* bit 0 = 1, bits[7:1] = 0: device event */

          dwc3_handle_dev_event(priv, evt);
        }

      priv->evt_lpos = (priv->evt_lpos + 4) % DWC3_EVENT_BUFFERS_SIZE;
    }

  /* Unmask events after processing */

  dwc3_putreg(priv, DWC3_GEVNTSIZ(0),
              DWC3_EVENT_BUFFERS_SIZE & DWC3_GEVNTSIZ_SIZE_MASK);

  usbtrace(TRACE_INTEXIT(DWC3_TRACEINTID_USB), 0);
  return OK;
}

/****************************************************************************
 * Endpoint Operations (usbdev_epops_s)
 ****************************************************************************/

static int dwc3_epconfigure(struct usbdev_ep_s *ep,
                            const struct usb_epdesc_s *desc, bool last)
{
  struct imx9_dwc3_ep_s *privep = (struct imx9_dwc3_ep_s *)ep;
  struct imx9_dwc3_s *priv = privep->priv;
  uint16_t maxpacket;
  uint8_t eptype;
  uint32_t par0;
  uint32_t par1;
  int ret;

  maxpacket = GETUINT16(desc->mxpacketsize);
  eptype = desc->attr & USB_EP_ATTR_XFERTYPE_MASK;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);

  privep->ep.maxpacket = maxpacket;
  privep->eptype = eptype;

  /* Issue DEPCFG command */

  /* par0: EP type, max packet size, burst size, FIFO number */

  par0 = (eptype << 1) |
         (maxpacket << 3);

  if (privep->isin)
    {
      /* Assign FIFO number = logical EP number */

      uint8_t fifo_num = privep->epphy >> 1;

      par0 |= (fifo_num << 17);             /* TxFIFO Number */
    }

  /* par1: interrupt enables, EP number */

  /* par1: XferComplete enable (bit 10), XferNotReady enable (bit 8),
   * EP number (bits 29:25).
   */

  par1 = (1 << 10) | (1 << 8);
  par1 |= ((uint32_t)privep->epphy << 25);

  ret = dwc3_send_ep_cmd(priv, privep->epphy,
                         DWC3_DEPCMD_SETEPCONFIG, par0, par1, 0);
  if (ret < 0)
    {
      return ret;
    }

  /* Issue DEPXFERCFG command - 1 transfer resource */

  ret = dwc3_send_ep_cmd(priv, privep->epphy,
                         DWC3_DEPCMD_SETTRANSFRESOURCE, 1, 0, 0);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable endpoint in DALEPENA */

  dwc3_modifyreg(priv, DWC3_DALEPENA, 0,
                 DWC3_DALEPENA_EP(privep->epphy));

  return OK;
}

static int dwc3_epdisable(struct usbdev_ep_s *ep)
{
  struct imx9_dwc3_ep_s *privep = (struct imx9_dwc3_ep_s *)ep;
  struct imx9_dwc3_s *priv = privep->priv;
  irqstate_t flags;

  usbtrace(TRACE_EPDISABLE, privep->epphy);

  flags = enter_critical_section();

  /* Cancel all pending requests */

  dwc3_cancelrequests(privep, -ESHUTDOWN);

  /* Disable endpoint in DALEPENA */

  dwc3_modifyreg(priv, DWC3_DALEPENA,
                 DWC3_DALEPENA_EP(privep->epphy), 0);

  leave_critical_section(flags);
  return OK;
}

static struct usbdev_req_s *dwc3_epallocreq(struct usbdev_ep_s *ep)
{
  struct imx9_dwc3_req_s *privreq;

  privreq = kmm_zalloc(sizeof(struct imx9_dwc3_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(DWC3_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  return &privreq->req;
}

static void dwc3_epfreereq(struct usbdev_ep_s *ep,
                           struct usbdev_req_s *req)
{
  struct imx9_dwc3_req_s *privreq = (struct imx9_dwc3_req_s *)req;

  kmm_free(privreq);
}

#ifdef CONFIG_USBDEV_DMA
static void *dwc3_epallocbuffer(struct usbdev_ep_s *ep, uint16_t bytes)
{
  return cache_aligned_alloc(DCACHE_ALIGN_UP(bytes));
}

static void dwc3_epfreebuffer(struct usbdev_ep_s *ep, void *buf)
{
  kmm_free(buf);
}
#endif

static int dwc3_epsubmit(struct usbdev_ep_s *ep,
                         struct usbdev_req_s *req)
{
  struct imx9_dwc3_ep_s *privep = (struct imx9_dwc3_ep_s *)ep;
  struct imx9_dwc3_req_s *privreq = (struct imx9_dwc3_req_s *)req;
  struct imx9_dwc3_s *priv = privep->priv;
  irqstate_t flags;
  bool was_empty;

  DEBUGASSERT(req && req->callback && req->buf);

  usbtrace(TRACE_EPSUBMIT, privep->epphy);

  req->result = -EINPROGRESS;
  req->xfrd = 0;

  flags = enter_critical_section();

  /* Handle EP0 specially */

  if (privep->epphy == IMX9_DWC3_EP0_IN ||
      privep->epphy == IMX9_DWC3_EP0_OUT)
    {
      uintptr_t buf_dma = (uintptr_t)req->buf;
      bool is_in = (privep->epphy == IMX9_DWC3_EP0_IN);

      /* For 2-stage transfers (no data phase, e.g. SET_CONFIGURATION),
       * the NuttX class driver calls EP_SUBMIT with len=0 to indicate
       * "send status". On DWC3, status is sent on XferNotReady(STATUS),
       * so just enqueue and complete immediately - the status phase
       * will be handled by the XferNotReady handler.
       */

      if (!priv->three_stage_setup && req->len == 0)
        {
          dwc3_rqenqueue(privep, privreq);
          privreq->req.xfrd = 0;
          dwc3_reqcomplete(privep, privreq, OK);
          leave_critical_section(flags);
          return OK;
        }

      /* For 3-stage transfers, start data phase directly */

      dwc3_rqenqueue(privep, privreq);
      dwc3_ep0_start_data(priv, buf_dma, req->len, is_in);

      leave_critical_section(flags);
      return OK;
    }

  /* Non-EP0: enqueue request */

  was_empty = dwc3_rqenqueue(privep, privreq);

  if (was_empty && !privep->xfer_started)
    {
      /* Start transfer immediately */

      uintptr_t buf_dma = (uintptr_t)req->buf;
      uint32_t len = req->len;
      uint32_t ctrl;

      ctrl = DWC3_TRBCTL_NORMAL |
             DWC3_TRB_CTRL_LST |
             DWC3_TRB_CTRL_IOC |
             DWC3_TRB_CTRL_ISP_IMI |
             DWC3_TRB_CTRL_HWO;

      privep->trb_enqueue = 0;
      privep->trb_dequeue = 0;

      dwc3_trb_init(&privep->trb[0], buf_dma, len, ctrl);

      dwc3_ep_start_transfer(priv, privep,
                             &privep->trb[0], privep->trb_dma);
    }

  leave_critical_section(flags);
  return OK;
}

static int dwc3_epcancel(struct usbdev_ep_s *ep,
                         struct usbdev_req_s *req)
{
  struct imx9_dwc3_ep_s *privep = (struct imx9_dwc3_ep_s *)ep;
  struct imx9_dwc3_req_s *privreq;
  struct imx9_dwc3_req_s *prev;
  irqstate_t flags;

  usbtrace(TRACE_EPCANCEL, privep->epphy);

  flags = enter_critical_section();

  /* Find and remove the request from the queue */

  prev = NULL;
  for (privreq = privep->head; privreq; privreq = privreq->flink)
    {
      if (&privreq->req == req)
        {
          if (prev)
            {
              prev->flink = privreq->flink;
            }
          else
            {
              privep->head = privreq->flink;
            }

          if (privep->tail == privreq)
            {
              privep->tail = prev;
            }

          dwc3_reqcomplete(privep, privreq, -ECANCELED);
          break;
        }

      prev = privreq;
    }

  leave_critical_section(flags);
  return OK;
}

static int dwc3_epstall(struct usbdev_ep_s *ep, bool resume)
{
  struct imx9_dwc3_ep_s *privep = (struct imx9_dwc3_ep_s *)ep;
  struct imx9_dwc3_s *priv = privep->priv;
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();

  if (resume)
    {
      /* Clear stall */

      ret = dwc3_send_ep_cmd(priv, privep->epphy,
                             DWC3_DEPCMD_CLEARSTALL, 0, 0, 0);
      privep->stalled = false;
      privep->wedged = false;
    }
  else
    {
      /* Set stall */

      ret = dwc3_send_ep_cmd(priv, privep->epphy,
                             DWC3_DEPCMD_SETSTALL, 0, 0, 0);
      privep->stalled = true;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Device Operations (usbdev_ops_s)
 ****************************************************************************/

static struct usbdev_ep_s *dwc3_allocep(struct usbdev_s *dev,
                                        uint8_t epno, bool in,
                                        uint8_t eptype)
{
  struct imx9_dwc3_s *priv = (struct imx9_dwc3_s *)dev;
  struct imx9_dwc3_ep_s *privep;
  irqstate_t flags;
  uint8_t epphy;
  int logep;

  usbtrace(TRACE_DEVALLOCEP, epno);

  flags = enter_critical_section();

  /* Find a free endpoint */

  if (epno > 0)
    {
      logep = epno & USB_EPNO_MASK;
    }
  else
    {
      /* Auto-allocate: find first available */

      for (logep = 1; logep < IMX9_DWC3_NLOGENDPOINTS; logep++)
        {
          epphy = in ? IMX9_DWC3_EPPHY_IN(logep)
                     : IMX9_DWC3_EPPHY_OUT(logep);
          if (priv->epavail & (1 << epphy))
            {
              break;
            }
        }

      if (logep >= IMX9_DWC3_NLOGENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(DWC3_TRACEERR_NOEP), 0);
          leave_critical_section(flags);
          return NULL;
        }
    }

  epphy = in ? IMX9_DWC3_EPPHY_IN(logep) : IMX9_DWC3_EPPHY_OUT(logep);

  if (!(priv->epavail & (1 << epphy)))
    {
      usbtrace(TRACE_DEVERROR(DWC3_TRACEERR_NOEP), epphy);
      leave_critical_section(flags);
      return NULL;
    }

  /* Mark as allocated */

  priv->epavail &= ~(1 << epphy);

  privep = &priv->eplist[epphy];

  /* Allocate TRB pool if not EP0 */

  if (!privep->trb)
    {
      privep->trb = cache_aligned_alloc(
          DWC3_TRB_NUM * sizeof(struct imx9_dwc3_trb_s));
      if (!privep->trb)
        {
          priv->epavail |= (1 << epphy);
          leave_critical_section(flags);
          return NULL;
        }

      memset(privep->trb, 0,
             DWC3_TRB_NUM * sizeof(struct imx9_dwc3_trb_s));
      privep->trb_dma = (uintptr_t)privep->trb;
    }

  leave_critical_section(flags);
  return &privep->ep;
}

static void dwc3_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep)
{
  struct imx9_dwc3_s *priv = (struct imx9_dwc3_s *)dev;
  struct imx9_dwc3_ep_s *privep = (struct imx9_dwc3_ep_s *)ep;
  irqstate_t flags;

  usbtrace(TRACE_DEVFREEEP, privep->epphy);

  flags = enter_critical_section();

  dwc3_cancelrequests(privep, -ESHUTDOWN);

  if (privep->trb && privep->epphy > IMX9_DWC3_EP0_IN)
    {
      kmm_free(privep->trb);
      privep->trb = NULL;
    }

  priv->epavail |= (1 << privep->epphy);

  leave_critical_section(flags);
}

static int dwc3_getframe(struct usbdev_s *dev)
{
  struct imx9_dwc3_s *priv = (struct imx9_dwc3_s *)dev;
  uint32_t dsts;

  dsts = dwc3_getreg(priv, DWC3_DSTS);
  return (dsts & DWC3_DSTS_SOFFN_MASK) >> DWC3_DSTS_SOFFN_SHIFT;
}

static int dwc3_wakeup(struct usbdev_s *dev)
{
  struct imx9_dwc3_s *priv = (struct imx9_dwc3_s *)dev;

  usbtrace(TRACE_DEVWAKEUP, 0);

  /* Set link state to Recovery */

  dwc3_modifyreg(priv, DWC3_DCTL,
                 DWC3_DCTL_ULSTCHNGREQ_MASK,
                 (8 << DWC3_DCTL_ULSTCHNGREQ_SHIFT));

  return OK;
}

static int dwc3_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct imx9_dwc3_s *priv = (struct imx9_dwc3_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);
  priv->selfpowered = selfpowered;
  return OK;
}

static int dwc3_pullup(struct usbdev_s *dev, bool enable)
{
  struct imx9_dwc3_s *priv = (struct imx9_dwc3_s *)dev;
  irqstate_t flags;
  uint32_t reg;
  int timeout;

  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  flags = enter_critical_section();

  /* Skip if already in the requested state */

  if (priv->softconnect == enable)
    {
      leave_critical_section(flags);
      return OK;
    }

  uerr("DWC3: pullup %s\n", enable ? "enable" : "disable");

  /* Clear SUSPHY and ENBLSLPM before run/stop change (Linux workaround) */

  dwc3_modifyreg(priv, DWC3_GUSB2PHYCFG(0),
                 DWC3_GUSB2PHYCFG_SUSPHY |
                 DWC3_GUSB2PHYCFG_ENBLSLPM, 0);

  reg = dwc3_getreg(priv, DWC3_DCTL);

  /* Clear ULSTCHNGREQ to avoid accidental link state change */

  reg &= ~DWC3_DCTL_ULSTCHNGREQ_MASK;

  if (enable)
    {
      /* Clear KEEP_CONNECT, set RUN_STOP */

      reg &= ~DWC3_DCTL_KEEP_CONNECT;
      reg |= DWC3_DCTL_RUN_STOP;
      dwc3_putreg(priv, DWC3_DCTL, reg);

      /* Wait for DEVCTRLHLT to clear (controller running) */

      timeout = 2000;
      do
        {
          up_udelay(1000);
          reg = dwc3_getreg(priv, DWC3_DSTS);
          if (!(reg & DWC3_DSTS_DEVCTRLHLT))
            {
              break;
            }
        }
      while (--timeout > 0);

      if (timeout == 0)
        {
          uerr("DWC3: Run/Stop start timeout, DSTS=0x%08x\n", reg);
        }
      else
        {
          uint32_t linkst = (reg & DWC3_DSTS_USBLNKST_MASK) >>
                            DWC3_DSTS_USBLNKST_SHIFT;
          uint32_t speed = reg & DWC3_DSTS_CONNECTSPD_MASK;

          uinfo("DWC3: Controller running DSTS=0x%08x "
                 "linkst=%d speed=%d\n", reg, (int)linkst, (int)speed);
        }
    }
  else
    {
      reg &= ~DWC3_DCTL_RUN_STOP;
      dwc3_putreg(priv, DWC3_DCTL, reg);

      /* Wait for DEVCTRLHLT to set (controller halted) */

      timeout = 2000;
      do
        {
          up_udelay(1000);
          reg = dwc3_getreg(priv, DWC3_DSTS);
          if (reg & DWC3_DSTS_DEVCTRLHLT)
            {
              break;
            }
        }
      while (--timeout > 0);

      if (timeout == 0)
        {
          uerr("DWC3: Run/Stop stop timeout\n");
        }
    }

  priv->softconnect = enable;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * DWC3 Core Initialization
 ****************************************************************************/

static int dwc3_core_init(struct imx9_dwc3_s *priv)
{
  uint32_t reg;
  int timeout;

  /* Read and verify core ID */

  reg = dwc3_getreg(priv, DWC3_GSNPSID);
  if ((reg & DWC3_GSNPSID_MASK) != 0x55330000 &&
      (reg & DWC3_GSNPSID_MASK) != 0x33310000)
    {
      uerr("DWC3: Invalid core ID 0x%08x\n", reg);
      usbtrace(TRACE_DEVERROR(DWC3_TRACEERR_COREID), (uint16_t)reg);
      return -ENODEV;
    }

  uinfo("DWC3 core ID: 0x%08x revision: 0x%04x\n",
        (reg & DWC3_GSNPSID_MASK) >> 16,
        reg & DWC3_GSNPSREV_MASK);

  /* Core soft reset */

  dwc3_modifyreg(priv, DWC3_GCTL, 0, DWC3_GCTL_CORESOFTRESET);

  /* PHY soft reset */

  dwc3_modifyreg(priv, DWC3_GUSB2PHYCFG(0), 0,
                 DWC3_GUSB2PHYCFG_PHYSOFTRST);
  dwc3_modifyreg(priv, DWC3_GUSB3PIPECTL(0), 0,
                 DWC3_GUSB3PIPECTL_PHYSOFTRST);

  up_mdelay(10);

  /* Deassert PHY resets */

  dwc3_modifyreg(priv, DWC3_GUSB2PHYCFG(0),
                 DWC3_GUSB2PHYCFG_PHYSOFTRST, 0);
  dwc3_modifyreg(priv, DWC3_GUSB3PIPECTL(0),
                 DWC3_GUSB3PIPECTL_PHYSOFTRST, 0);

  up_mdelay(10);

  /* Deassert core reset */

  dwc3_modifyreg(priv, DWC3_GCTL, DWC3_GCTL_CORESOFTRESET, 0);

  /* Wait for core to be ready */

  timeout = 1000;
  do
    {
      reg = dwc3_getreg(priv, DWC3_GCTL);
      if (!(reg & DWC3_GCTL_CORESOFTRESET))
        {
          break;
        }

      up_udelay(10);
    }
  while (--timeout > 0);

  if (timeout == 0)
    {
      uerr("DWC3: Core reset timeout\n");
      return -ETIMEDOUT;
    }

  /* Configure USB2 PHY: UTMI+ 16-bit interface
   * Clear SUSPHY, ENBLSLPM, U2_FREECLK_EXISTS (gfladj-refclk-lpm-sel quirk)
   * USBTRDTIM=5 for 16-bit UTMI+, PHYIF=1 for 16-bit
   */

  dwc3_modifyreg(priv, DWC3_GUSB2PHYCFG(0),
                 DWC3_GUSB2PHYCFG_PHYSOFTRST |
                 DWC3_GUSB2PHYCFG_SUSPHY |
                 DWC3_GUSB2PHYCFG_ENBLSLPM |
                 DWC3_GUSB2PHYCFG_U2_FREECLK |
                 DWC3_GUSB2PHYCFG_USBTRDTIM_MASK,
                 DWC3_GUSB2PHYCFG_USBTRDTIM(5) |
                 DWC3_GUSB2PHYCFG_PHYIF);

#ifdef CONFIG_IMX9_DWC3_USB3
  /* USB 3.0 SuperSpeed mode: keep DISRXDETINP3 cleared (default after
   * reset) so SS Rx detection works normally in P3 state.
   * Set DELAYP1TRANS to avoid P1 transition issues.
   * Note: SS on i.MX95 requires Type-C mux config and PHY tuning.
   */

  dwc3_modifyreg(priv, DWC3_GUSB3PIPECTL(0),
                 DWC3_GUSB3PIPECTL_SUSPHY |
                 DWC3_GUSB3PIPECTL_PHYSOFTRST,
                 DWC3_GUSB3PIPECTL_DELAYP1TRANS);
#else
  /* USB 2.0 only mode: set DISRXDETINP3 to disable SS RX detect in
   * P3 state, forcing the controller to USB 2.0 HighSpeed only.
   * Also set DELAYP1TRANS to avoid P1 transition issues.
   */

  dwc3_modifyreg(priv, DWC3_GUSB3PIPECTL(0),
                 DWC3_GUSB3PIPECTL_SUSPHY |
                 DWC3_GUSB3PIPECTL_PHYSOFTRST,
                 DWC3_GUSB3PIPECTL_DISRXDETINP3 |
                 DWC3_GUSB3PIPECTL_DELAYP1TRANS);
#endif

  /* i.MX95 quirks: disable SS park mode (parkmode-disable-ss-quirk)
   * and decouple L1/L2 events, force 20 clk for 30 clk
   */

  reg = dwc3_getreg(priv, DWC3_GUCTL1);
  reg |= DWC3_GUCTL1_PARKMODE_DISABLE_SS;
  reg |= DWC3_GUCTL1_DEV_DECOUPLE_L1L2_EVT;
  reg |= DWC3_GUCTL1_DEV_FORCE_20_CLK_FOR_30_CLK;
  dwc3_putreg(priv, DWC3_GUCTL1, reg);

  /* Configure GFLADJ for 24MHz ref clock (gfladj-refclk-lpm-sel quirk) */

  reg = dwc3_getreg(priv, DWC3_GFLADJ);
  reg |= DWC3_GFLADJ_REFCLK_LPM_SEL;
  dwc3_putreg(priv, DWC3_GFLADJ, reg);

  /* Set device mode with correct PWRDNSCALE for 24MHz ref clock.
   * PWRDNSCALE = ref_freq_khz / 16 = 24000 / 16 = 1500 = 0x5DC
   * Also set SOFITPSYNC for SOF/ITP synchronization.
   */

  reg = dwc3_getreg(priv, DWC3_GCTL);
  reg &= ~(DWC3_GCTL_PRTCAPDIR_MASK | DWC3_GCTL_PWRDNSCALE_MASK |
            DWC3_GCTL_DISSCRAMBLE | DWC3_GCTL_SCALEDOWN_MASK);
  reg |= DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_DEVICE);
  reg |= DWC3_GCTL_PWRDNSCALE(0x5dc);
  reg |= DWC3_GCTL_U2RSTECN;
  dwc3_putreg(priv, DWC3_GCTL, reg);

  /* Wait for mode switch */

  up_mdelay(1);

  return OK;
}

/****************************************************************************
 * DWC3 Device Mode Initialization
 ****************************************************************************/

static int dwc3_device_init(struct imx9_dwc3_s *priv)
{
  uint32_t pending;
  uint32_t par0;
  uint32_t par1;
  int ret;
  int i;

#ifdef CONFIG_IMX9_DWC3_USB3
  /* Configure DCFG: SuperSpeed, device address 0 */

  dwc3_putreg(priv, DWC3_DCFG,
              DWC3_DCFG_SUPERSPEED | DWC3_DCFG_NUMP(4));
#else
  /* Configure DCFG: HighSpeed, device address 0 */

  dwc3_putreg(priv, DWC3_DCFG,
              DWC3_DCFG_HIGHSPEED | DWC3_DCFG_NUMP(4));
#endif

  /* Allocate event buffer - must be aligned to buffer size (4096)
   * per DWC3 databook requirement for GEVNTADR register.
   */

  priv->evtbuf = kmm_memalign(DWC3_EVENT_BUFFERS_SIZE,
                               DWC3_EVENT_BUFFERS_SIZE);
  if (!priv->evtbuf)
    {
      return -ENOMEM;
    }

  memset(priv->evtbuf, 0, DWC3_EVENT_BUFFERS_SIZE);
  up_clean_dcache((uintptr_t)priv->evtbuf,
                  (uintptr_t)priv->evtbuf + DWC3_EVENT_BUFFERS_SIZE);
  priv->evtbuf_dma = (uintptr_t)priv->evtbuf;
  priv->evt_lpos = 0;

  /* Configure event buffer */

  uinfo("DWC3: evtbuf=%p dma=0x%08lx size=%d align=%d\n",
         priv->evtbuf, (unsigned long)priv->evtbuf_dma,
         DWC3_EVENT_BUFFERS_SIZE,
         (int)((uintptr_t)priv->evtbuf % DWC3_EVENT_BUFFERS_SIZE));

  /* First mask events and clear any pending events */

  dwc3_putreg(priv, DWC3_GEVNTSIZ(0),
              DWC3_GEVNTSIZ_INTMASK |
              (DWC3_EVENT_BUFFERS_SIZE & DWC3_GEVNTSIZ_SIZE_MASK));

  pending = dwc3_getreg(priv, DWC3_GEVNTCOUNT(0));
  pending &= DWC3_GEVNTCOUNT_MASK;
  if (pending > 0)
    {
      uinfo("DWC3: clearing %d pending event bytes\n", (int)pending);
      dwc3_putreg(priv, DWC3_GEVNTCOUNT(0), pending);
    }

  /* Now configure the event buffer address */

  dwc3_putreg(priv, DWC3_GEVNTADRLO(0),
              (uint32_t)(priv->evtbuf_dma & 0xffffffff));
  dwc3_putreg(priv, DWC3_GEVNTADRHI(0),
              (uint32_t)(priv->evtbuf_dma >> 32));

  /* Unmask events - do NOT write 0 to GEVNTCOUNT as it's a no-op
   * (writing N decrements count by N, writing 0 does nothing)
   */

  dwc3_putreg(priv, DWC3_GEVNTSIZ(0),
              DWC3_EVENT_BUFFERS_SIZE & DWC3_GEVNTSIZ_SIZE_MASK);

  /* Enable device events */

  dwc3_putreg(priv, DWC3_DEVTEN,
              DWC3_DEVTEN_USBRSTEN |
              DWC3_DEVTEN_CONNECTDONEEN |
              DWC3_DEVTEN_DISCONNEVTEN |
              DWC3_DEVTEN_U3L2L1SUSPEN |
              DWC3_DEVTEN_ERRTICERREN);

  /* Allocate EP0 TRBs */

  priv->ep0trb = cache_aligned_alloc(
      2 * sizeof(struct imx9_dwc3_trb_s));
  if (!priv->ep0trb)
    {
      kmm_free(priv->evtbuf);
      return -ENOMEM;
    }

  memset(priv->ep0trb, 0, 2 * sizeof(struct imx9_dwc3_trb_s));
  up_clean_dcache((uintptr_t)priv->ep0trb,
                  (uintptr_t)priv->ep0trb +
                  2 * sizeof(struct imx9_dwc3_trb_s));
  priv->ep0trb_dma = (uintptr_t)priv->ep0trb;

  /* Allocate EP0 SETUP buffer */

  priv->ep0_setup_buf = cache_aligned_alloc(8);
  if (!priv->ep0_setup_buf)
    {
      kmm_free(priv->ep0trb);
      kmm_free(priv->evtbuf);
      return -ENOMEM;
    }

  memset(priv->ep0_setup_buf, 0, 8);
  up_clean_dcache((uintptr_t)priv->ep0_setup_buf,
                  (uintptr_t)priv->ep0_setup_buf + 8);
  priv->ep0_setup_dma = (uintptr_t)priv->ep0_setup_buf;

  /* Issue DEPSTARTCFG for EP0 */

  ret = dwc3_send_ep_cmd(priv, 0, DWC3_DEPCMD_DEPSTARTCFG, 0, 0, 0);
  if (ret < 0)
    {
      uerr("DWC3: DEPSTARTCFG failed\n");
      goto err_free_bufs;
    }

  /* Configure EP0 OUT */

  /* par0: EP type, max packet size */

  par0 = (DWC3_DEPCMD_TYPE_CONTROL << 1) |
         (EP0_MAXPACKET << 3);

  /* par1: XferComplete + XferNotReady enable */

  par1 = (1 << 10) | (1 << 8);

  ret = dwc3_send_ep_cmd(priv, IMX9_DWC3_EP0_OUT,
                         DWC3_DEPCMD_SETEPCONFIG, par0, par1, 0);
  if (ret < 0)
    {
      goto err_free_bufs;
    }

  ret = dwc3_send_ep_cmd(priv, IMX9_DWC3_EP0_OUT,
                         DWC3_DEPCMD_SETTRANSFRESOURCE, 1, 0, 0);
  if (ret < 0)
    {
      goto err_free_bufs;
    }

  /* Configure EP0 IN */

  par0 = (DWC3_DEPCMD_TYPE_CONTROL << 1) |
         (EP0_MAXPACKET << 3);

  par1 = (1 << 10) | (1 << 8);
  par1 |= ((uint32_t)IMX9_DWC3_EP0_IN << 25);

  ret = dwc3_send_ep_cmd(priv, IMX9_DWC3_EP0_IN,
                         DWC3_DEPCMD_SETEPCONFIG, par0, par1, 0);
  if (ret < 0)
    {
      goto err_free_bufs;
    }

  ret = dwc3_send_ep_cmd(priv, IMX9_DWC3_EP0_IN,
                         DWC3_DEPCMD_SETTRANSFRESOURCE, 1, 0, 0);
  if (ret < 0)
    {
      goto err_free_bufs;
    }

  /* Enable EP0 OUT and EP0 IN in DALEPENA */

  dwc3_putreg(priv, DWC3_DALEPENA,
              DWC3_DALEPENA_EP(IMX9_DWC3_EP0_OUT) |
              DWC3_DALEPENA_EP(IMX9_DWC3_EP0_IN));

  /* Initialize endpoint list */

  for (i = 0; i < IMX9_DWC3_NPHYSENDPOINTS; i++)
    {
      priv->eplist[i].ep.ops = &g_dwc3_epops;
      priv->eplist[i].priv = priv;
      priv->eplist[i].epphy = i;
      priv->eplist[i].isin = IMX9_DWC3_EP_IS_IN(i);
      priv->eplist[i].head = NULL;
      priv->eplist[i].tail = NULL;
      priv->eplist[i].trb = NULL;
      priv->eplist[i].xfer_started = false;
      priv->eplist[i].stalled = false;

      if (i <= IMX9_DWC3_EP0_IN)
        {
          priv->eplist[i].ep.maxpacket = EP0_MAXPACKET;
          priv->eplist[i].ep.eplog = i == 0 ? USB_DIR_OUT : USB_DIR_IN;
        }
      else
        {
          uint8_t logep = i >> 1;

          priv->eplist[i].ep.maxpacket = DWC3_HS_BULK_MAXPACKET;
          priv->eplist[i].ep.eplog = IMX9_DWC3_EP_IS_IN(i)
              ? (logep | USB_DIR_IN)
              : (logep | USB_DIR_OUT);
        }
    }

  return OK;

err_free_bufs:
  kmm_free(priv->ep0_setup_buf);
  priv->ep0_setup_buf = NULL;
  kmm_free(priv->ep0trb);
  priv->ep0trb = NULL;
  kmm_free(priv->evtbuf);
  priv->evtbuf = NULL;
  return ret;
}

/****************************************************************************
 * PHY Initialization
 ****************************************************************************/

static void dwc3_phy_init(struct imx9_dwc3_s *priv)
{
  uintptr_t phy = priv->phy_base;
  uint32_t reg;

  /* Step 1: Configure reference clock to 24MHz */

  reg = getreg32(phy + IMX9_USB3_PHY_CTRL0);
  reg &= ~IMX9_USB3_PHY_CTRL0_FSEL_MASK;
  reg |= IMX9_USB3_PHY_CTRL0_FSEL_24M;
  putreg32(reg, phy + IMX9_USB3_PHY_CTRL0);

  /* Step 2: Disable alt_clk, use internal MPLL */

  reg = getreg32(phy + IMX9_USB3_PHY_CTRL6);
  reg &= ~(IMX9_USB3_PHY_CTRL6_ALT_CLK_SEL |
            IMX9_USB3_PHY_CTRL6_ALT_CLK_EN);
  putreg32(reg, phy + IMX9_USB3_PHY_CTRL6);

  /* Step 3: Assert PHY reset */

  reg = getreg32(phy + IMX9_USB3_PHY_CTRL1);
  reg &= ~(IMX9_USB3_PHY_CTRL1_VDATSRCENB0 |
            IMX9_USB3_PHY_CTRL1_VDATDETENB0);
  reg |= IMX9_USB3_PHY_CTRL1_RESET | IMX9_USB3_PHY_CTRL1_ATERESET;
  putreg32(reg, phy + IMX9_USB3_PHY_CTRL1);

  /* Step 4: Enable SSP reference clock */

  reg = getreg32(phy + IMX9_USB3_PHY_CTRL0);
  reg |= IMX9_USB3_PHY_CTRL0_REF_SSP_EN;
  putreg32(reg, phy + IMX9_USB3_PHY_CTRL0);

  /* Step 5: Enable TX, disable OTG */

  reg = getreg32(phy + IMX9_USB3_PHY_CTRL2);
  reg |= IMX9_USB3_PHY_CTRL2_TXENABLEN0 |
         IMX9_USB3_PHY_CTRL2_OTG_DISABLE;
  putreg32(reg, phy + IMX9_USB3_PHY_CTRL2);

  /* Step 6: Wait for PHY to stabilize */

  up_udelay(10);

  /* Step 7: Deassert PHY reset */

  reg = getreg32(phy + IMX9_USB3_PHY_CTRL1);
  reg &= ~(IMX9_USB3_PHY_CTRL1_RESET | IMX9_USB3_PHY_CTRL1_ATERESET);
  putreg32(reg, phy + IMX9_USB3_PHY_CTRL1);

  /* Step 8: Disable rx term override (power on) */

  reg = getreg32(phy + IMX9_USB3_PHY_CTRL6);
  reg &= ~IMX9_USB3_PHY_CTRL6_RXTERM_OVERRIDE_SEL;
  putreg32(reg, phy + IMX9_USB3_PHY_CTRL6);

  uinfo("USB3 PHY initialized\n");
}

/****************************************************************************
 * Glue Layer Initialization
 ****************************************************************************/

static void dwc3_glue_init(struct imx9_dwc3_s *priv)
{
  uint32_t reg;

  /* Configure glue layer for device mode */

  reg = dwc3_glue_getreg(priv, IMX9_DWC3_GLUE_CTRL0);
  reg &= ~IMX9_DWC3_GLUE_CTRL0_PORTPWR_EN;
  dwc3_glue_putreg(priv, IMX9_DWC3_GLUE_CTRL0, reg);

  reg = dwc3_glue_getreg(priv, IMX9_DWC3_GLUE_CTRL1);
  reg |= IMX9_DWC3_GLUE_CTRL1_OC_POL;
  dwc3_glue_putreg(priv, IMX9_DWC3_GLUE_CTRL1, reg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_usbinitialize
 *
 * Description:
 *   Initialize the DWC3 USB device controller hardware.
 *
 ****************************************************************************/

void arm64_usbinitialize(void)
{
  struct imx9_dwc3_s *priv = &g_dwc3dev;
  irqstate_t flags;
  int ret;

  uinfo("DWC3: arm64_usbinitialize\n");

  flags = enter_critical_section();

  /* Initialize driver state */

  memset(priv, 0, sizeof(struct imx9_dwc3_s));

  priv->base      = IMX9_DWC3_BASE;
  priv->glue_base = IMX9_DWC3_GLUE_BASE;
  priv->phy_base  = IMX9_DWC3_PHY_BASE;
  priv->irq       = IMX9_IRQ_USB1;
  priv->ep0buf    = g_dwc3_ep0buf;

  priv->usbdev.ops = &g_dwc3_devops;
  priv->usbdev.ep0 = &priv->eplist[IMX9_DWC3_EP0_IN].ep;
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Mark all non-EP0 endpoints as available */

  priv->epavail = 0;
  for (int i = 2; i < IMX9_DWC3_NPHYSENDPOINTS; i++)
    {
      priv->epavail |= (1 << i);
    }

  /* Initialize glue layer */

  dwc3_glue_init(priv);

  /* Initialize USB3 PHY */

  dwc3_phy_init(priv);

  /* Initialize DWC3 core */

  ret = dwc3_core_init(priv);
  if (ret < 0)
    {
      uerr("DWC3 core init failed: %d\n", ret);
      leave_critical_section(flags);
      return;
    }

  /* Initialize device mode */

  ret = dwc3_device_init(priv);
  if (ret < 0)
    {
      uerr("DWC3 device init failed: %d\n", ret);
      leave_critical_section(flags);
      return;
    }

  /* Attach and enable USB interrupt */

  irq_attach(priv->irq, dwc3_interrupt, priv);
  up_enable_irq(priv->irq);

  /* Start EP0 setup phase */

  dwc3_ep0_start_setup(priv);

  leave_critical_section(flags);

  uinfo("DWC3: Init done. GCTL=0x%08x DCFG=0x%08x DSTS=0x%08x\n",
         dwc3_getreg(priv, DWC3_GCTL),
         dwc3_getreg(priv, DWC3_DCFG),
         dwc3_getreg(priv, DWC3_DSTS));
  uinfo("DWC3: GUSB2PHYCFG=0x%08x GUSB3PIPECTL=0x%08x\n",
         dwc3_getreg(priv, DWC3_GUSB2PHYCFG(0)),
         dwc3_getreg(priv, DWC3_GUSB3PIPECTL(0)));

  uinfo("DWC3 USB device initialized\n");
}

/****************************************************************************
 * Name: arm64_usbuninitialize
 *
 * Description:
 *   Uninitialize the DWC3 USB device controller.
 *
 ****************************************************************************/

void arm64_usbuninitialize(void)
{
  struct imx9_dwc3_s *priv = &g_dwc3dev;
  irqstate_t flags;
  int i;

  usbtrace(TRACE_DEVUNINIT, 0);

  if (priv->driver)
    {
      usbdev_unregister(priv->driver);
    }

  flags = enter_critical_section();

  /* Stop the controller */

  dwc3_modifyreg(priv, DWC3_DCTL, DWC3_DCTL_RUN_STOP, 0);

  /* Cancel all pending requests */

  for (i = 0; i < IMX9_DWC3_NPHYSENDPOINTS; i++)
    {
      dwc3_cancelrequests(&priv->eplist[i], -ESHUTDOWN);
    }

  /* Disable and detach IRQ */

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

  /* Disable events */

  dwc3_putreg(priv, DWC3_DEVTEN, 0);
  dwc3_putreg(priv, DWC3_GEVNTSIZ(0),
              DWC3_GEVNTSIZ_INTMASK | DWC3_EVENT_BUFFERS_SIZE);

  /* Free allocated buffers */

  if (priv->ep0_setup_buf)
    {
      kmm_free(priv->ep0_setup_buf);
      priv->ep0_setup_buf = NULL;
    }

  if (priv->ep0trb)
    {
      kmm_free(priv->ep0trb);
      priv->ep0trb = NULL;
    }

  if (priv->evtbuf)
    {
      kmm_free(priv->evtbuf);
      priv->evtbuf = NULL;
    }

  /* Free non-EP0 TRBs */

  for (i = 2; i < IMX9_DWC3_NPHYSENDPOINTS; i++)
    {
      if (priv->eplist[i].trb)
        {
          kmm_free(priv->eplist[i].trb);
          priv->eplist[i].trb = NULL;
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: usbdev_register
 *
 * Description:
 *   Register a USB device class driver.
 *
 ****************************************************************************/

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  struct imx9_dwc3_s *priv = &g_dwc3dev;
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->disconnect || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(DWC3_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(DWC3_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* Hook up the driver */

  priv->driver = driver;

  /* Bind the class driver */

  uinfo("DWC3: Binding class driver\n");
  ret = CLASS_BIND(driver, &priv->usbdev);
  if (ret < 0)
    {
      usbtrace(TRACE_DEVERROR(DWC3_TRACEERR_BINDFAILED), (uint16_t)-ret);
      uerr("DWC3: CLASS_BIND failed: %d\n", ret);
      priv->driver = NULL;
      return ret;
    }

  uinfo("DWC3: CLASS_BIND OK, enabling IRQ and pullup\n");

  /* Enable USB controller interrupt */

  up_enable_irq(priv->irq);

  /* Connect to host (set RUN/STOP) */

  dwc3_pullup(&priv->usbdev, true);

  return OK;
}

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Unregister a USB device class driver.
 *
 ****************************************************************************/

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  struct imx9_dwc3_s *priv = &g_dwc3dev;

  usbtrace(TRACE_DEVUNREGISTER, 0);

  if (driver != priv->driver)
    {
      return -EINVAL;
    }

  /* Disconnect from host */

  dwc3_pullup(&priv->usbdev, false);

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable USB controller interrupt */

  up_disable_irq(priv->irq);

  /* Unhook the driver */

  priv->driver = NULL;

  return OK;
}

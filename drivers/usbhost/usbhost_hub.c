/****************************************************************************
 * drivers/usbhost/usbhost_hub.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Kaushal Parikh <kaushal@dspworks.in>
 *           Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/scsi.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/hub.h>
#include <nuttx/usb/usbhost_devaddr.h>

#ifdef CONFIG_USBHOST_HUB

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* Used in usbhost_cfgdesc() */

#define USBHOST_IFFOUND     0x01 /* Required I/F descriptor found */
#define USBHOST_EPINFOUND   0x02 /* Required interrupt IN EP descriptor found */
#define USBHOST_ALLFOUND    (USBHOST_IFFOUND | USBHOST_EPINFOUND)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure contains the internal, private state of the USB host
 * hub class.
 */

struct usbhost_hubpriv_s
{
  FAR struct usb_ctrlreq_s *ctrlreq;      /* Allocated control request */
  FAR uint8_t              *buffer;       /* Allocated buffer */

  uint8_t                   ifno;         /* Interface number */
  uint8_t                   nports;       /* Number of ports */
  uint8_t                   lpsm;         /* Logical power switching mode */
  uint8_t                   ocmode;       /* Over current protection mode */
  uint8_t                   ctrlcurrent;  /* Control current */
  volatile bool             disconnected; /* TRUE: Device has been disconnected */
  bool                      compounddev;  /* Hub is part of compound device */
  bool                      indicator;    /* Port indicator */

  uint16_t                  pwrondelay;   /* Power on wait time in ms */
  int16_t                   crefs;        /* Reference count on the driver instance */

  sem_t                     exclsem;      /* Used to maintain mutual exclusive access */
  struct work_s             work;         /* Used for deferred callback work */
  usbhost_ep_t              intin;        /* Interrupt IN endpoint */

  struct usbhost_class_s    *childclass[USBHUB_MAX_PORTS];
                                          /* Pointer to child devices */
};

/* This represents the hub class structure.  It must be cast compatible
 * with struct usbhost_class_s.
 */

struct usbhost_hubclass_s
{
  struct usbhost_class_s   hubclass;      /* Publicly visible class data */
  struct usbhost_hubpriv_s hubpriv;       /* Private class data */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Memory allocation services */

static inline FAR struct usbhost_hubport_s *usbhost_allochub(
             FAR struct usbhost_driver_s *drvr,
             FAR struct usbhost_class_s *hubclass, uint8_t speed,
             uint8_t port);
static inline void usbhost_freehub(FAR struct usbhost_hubport_s *hport);
static inline void usbhost_freeclass(FAR struct usbhost_class_s *devclass);

/* Worker thread actions */

static void usbhost_destroy(FAR void *arg);

/* Helpers for usbhost_connect() */

static inline int usbhost_cfgdesc(FAR struct usbhost_class_s *hubclass,
             FAR const uint8_t *configdesc, int desclen);
static inline int usbhost_hubdesc(FAR struct usbhost_class_s *hubclass);
static inline int usbhost_hubpwr(FAR struct usbhost_class_s *hubclass,
             bool on);
static void usbhost_hubevent(FAR void *arg);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(const uint8_t *val);
static void usbhost_putle16(uint8_t *dest, uint16_t val);
static void usbhost_callback(FAR void *arg, int result);

/* struct usbhost_registry_s methods */

static FAR struct usbhost_class_s *usbhost_create(
             FAR struct usbhost_hubport_s *hport,
             FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_connect(FAR struct usbhost_class_s *hubclass,
             FAR const uint8_t *configdesc, int desclen);
static int usbhost_disconnected(FAR struct usbhost_class_s *hubclass);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID information that will  be
 * used to associate the USB host hub class to a connected USB hub.
 */

static const const struct usbhost_id_s g_id =
{
  USB_CLASS_HUB,  /* base     */
  0,              /* subclass */
  0,              /* proto    */
  0,              /* vid      */
  0               /* pid      */
};

/* This is the USB host hub class's registry entry */

static struct usbhost_registry_s g_hub =
{
  NULL,                   /* flink    */
  usbhost_create,         /* create   */
  1,                      /* nids     */
  &g_id                   /* id[]     */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_allochub
 *
 * Description:
 *   This is really part of the logic that implements the create() method
 *   of struct usbhost_registry_s.  This function allocates memory for one
 *   new class instance.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s.  NULL is returned on failure; this function will
 *   will fail only if there are insufficient resources to create another
 *   USB host class instance.
 *
 ****************************************************************************/

static inline FAR struct usbhost_hubport_s *
  usbhost_allochub(FAR struct usbhost_driver_s *drvr,
                     FAR struct usbhost_class_s *hubclass,
                     uint8_t speed, uint8_t port)
{
  FAR struct usbhost_hubport_s *child;
  FAR struct usbhost_hubport_s *parent;

  DEBUGASSERT(hubclass != NULL && hubclass->hport != NULL);
  parent = hubclass->hport;

  /* We are not executing from an interrupt handler so we can just call
   * kmm_malloc() to get memory for the class instance.
   */

  DEBUGASSERT(!up_interrupt_context());
  child = (FAR struct usbhost_hubport_s *)
    kmm_malloc(sizeof(struct usbhost_hubport_s));

  uvdbg("Allocated: %p\n", child);

  if (child != NULL)
    {
      struct usbhost_epdesc_s epdesc;
      int ret;

      child->drvr         = drvr;
      child->parent       = parent;
      child->funcaddr     = 0;
      child->speed        = speed;

      epdesc.hport        = child;
      epdesc.addr         = 0;
      epdesc.in           = 0;
      epdesc.xfrtype      = USB_EP_ATTR_XFER_CONTROL;
      epdesc.interval     = 0;
      epdesc.mxpacketsize = 8;

      ret = DRVR_EPALLOC(drvr, &epdesc, &child->ep0);
      if (ret != OK)
        {
          udbg("ERROR: Failed to allocate ep0: %d\n", ret);
          usbhost_freehub(child);
          child = NULL;
        }
    }

  return child;
}

/****************************************************************************
 * Name: usbhost_freehub
 *
 * Description:
 *   Free a hub instance previously allocated by usbhost_allochub().
 *
 * Input Parameters:
 *   hport - A reference to the hub port instance to be freed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline void usbhost_freehub(FAR struct usbhost_hubport_s *hport)
{
  if (hport && !ROOTHUB(hport))
    {
      if (hport->ep0 != NULL)
        {
          DRVR_EPFREE(hport->drvr, hport->ep0);
          hport->ep0 = NULL;
        }

      usbhost_devaddr_destroy(hport);
      hport->funcaddr = 0;

      /* Free the hport instance */

      uvdbg("Freeing: %p\n", hport);
      kmm_free(hport);
    }
}

/****************************************************************************
 * Name: usbhost_freeclass
 *
 * Description:
 *   Free a class instance previously allocated by usbhost_create().
 *
 * Input Parameters:
 *   devclass - A reference to the class instance to be freed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline void usbhost_freeclass(FAR struct usbhost_class_s *devclass)
{
  DEBUGASSERT(devclass != NULL);

  /* Free the bound hub */

  usbhost_freehub(devclass->hport);

  /* Free the class instance */

  uvdbg("Freeing: %p\n", devclass);
  kmm_free(devclass);
}

/****************************************************************************
 * Name: usbhost_destroy
 *
 * Description:
 *   The USB mass storage device has been disconnected and the reference
 *   count on the USB host class instance has gone to 1.. Time to destroy
 *   the USB host class instance.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_destroy(FAR void *arg)
{
  FAR struct usbhost_class_s *hubclass = (FAR struct usbhost_class_s *)arg;
  FAR struct usbhost_hubpriv_s *priv;
  FAR struct usbhost_hubport_s *hport;
  int i;

  DEBUGASSERT(hubclass != NULL);
  priv = &((FAR struct usbhost_hubclass_s *)hubclass)->hubpriv;

  uvdbg("crefs: %d\n", priv->crefs);

  if (priv->intin)
    {
      DEBUGASSERT(hubclass->hport);
      hport = hubclass->hport;
      DRVR_EPFREE(hport->drvr, priv->intin);
    }

  /* Destroy the semaphores */

  sem_destroy(&priv->exclsem);

  /* Destroy allocated child classes */

  for (i = 0; i < USBHUB_MAX_PORTS; i++)
    {
      if (priv->childclass[i] != NULL)
        {
          CLASS_DISCONNECTED(priv->childclass[i]);
        }
    }

  /* Free the allocated class structure */

  usbhost_freeclass(hubclass);
}

/****************************************************************************
 * Name: usbhost_cfgdesc
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   hubclass - The USB host class instance.
 *   configdesc - A pointer to a uint8_t buffer container the configuration
 *     descriptor.
 *   desclen - The length in bytes of the configuration descriptor.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static inline int usbhost_cfgdesc(FAR struct usbhost_class_s *hubclass,
                                  FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_hubpriv_s *priv;
  FAR struct usbhost_hubport_s *hport;
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  FAR struct usbhost_epdesc_s intindesc;
  int remaining;
  uint8_t found = 0;
  int ret;

  DEBUGASSERT(hubclass != NULL);
  priv = &((FAR struct usbhost_hubclass_s *)hubclass)->hubpriv;

  DEBUGASSERT(hubclass->hport);
  hport = hubclass->hport;

  DEBUGASSERT(configdesc != NULL && desclen >= sizeof(struct usb_cfgdesc_s));

  /* Initialize the interrupt IN endpoint information (only to prevent
   * compiler complaints)
   */

  intindesc.hport        = hport;
  intindesc.addr         = 0;
  intindesc.in           = true;
  intindesc.xfrtype      = USB_EP_ATTR_XFER_INT;
  intindesc.interval     = 0;
  intindesc.mxpacketsize = 0;

  /* Verify that we were passed a configuration descriptor */

  cfgdesc = (FAR struct usb_cfgdesc_s *)configdesc;
  if (cfgdesc->type != USB_DESC_TYPE_CONFIG)
    {
      return -EINVAL;
    }

  /* Get the total length of the configuration descriptor (little endian).
   * It might be a good check to get the number of interfaces here too.
  */

  remaining = (int)usbhost_getle16(cfgdesc->totallen);

  /* Skip to the next entry descriptor */

  configdesc += cfgdesc->len;
  remaining  -= cfgdesc->len;

  /* Loop where there are more descriptors to examine */

  while (remaining >= sizeof(struct usb_desc_s))
    {
      /* What is the next descriptor? */

      desc = (FAR struct usb_desc_s *)configdesc;
      switch (desc->type)
        {
        /* Interface descriptor. We really should get the number of endpoints
         * from this descriptor too.
         */

        case USB_DESC_TYPE_INTERFACE:
          {
            FAR struct usb_ifdesc_s *ifdesc =
              (FAR struct usb_ifdesc_s *)configdesc;

            uvdbg("Interface descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Save the interface number and mark ONLY the interface found */

            priv->ifno = ifdesc->ifno;
            found      = USBHOST_IFFOUND;
          }
          break;

        /* Endpoint descriptor.  Here, we expect two bulk endpoints, an IN
         * and an OUT.
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc =
              (FAR struct usb_epdesc_s *)configdesc;

            uvdbg("Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for an interrupt endpoint. */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
                USB_EP_ATTR_XFER_INT)
              {
                /* Yes.. it is a interrupt endpoint.  IN or OUT? */

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an OUT interrupt endpoint. Ignore */

                    uvdbg("Bulk OUT EP addr:%d mxpacketsize:%d\n",
                          (epdesc->addr & USB_EP_ADDR_NUMBER_MASK),
                          usbhost_getle16(epdesc->mxpacketsize));
                  }
                else
                  {
                    /* It is an IN interrupt endpoint. */

                    found |= USBHOST_EPINFOUND;

                    /* Save the interrupt IN endpoint information */

                    intindesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    intindesc.interval     = epdesc->interval;
                    intindesc.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);

                    uvdbg("Interrupt IN EP: addr=%d interval=%d mxpacketsize=%d\n",
                          intindesc.addr, intindesc.interval, intindesc.mxpacketsize);
                  }
              }
          }
          break;

        /* Other descriptors are just ignored for now */

        default:
          break;
        }

      /* If we found everything we need with this interface, then break out
       * of the loop early.
       */

      if (found == USBHOST_ALLFOUND)
        {
          break;
        }

      /* Increment the address of the next descriptor */

      configdesc += desc->len;
      remaining  -= desc->len;
    }

  /* Sanity checking... did we find all of things that we need? */

  if (found != USBHOST_ALLFOUND)
    {
      ulldbg("ERROR: Found IF=%s EPIN=%s\n",
             (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
             (found & USBHOST_EPINFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the interrupt IN endpoint */

  ret = DRVR_EPALLOC(hport->drvr, &intindesc, &priv->intin);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate Interrupt IN endpoint: %d\n", ret);
      (void)DRVR_EPFREE(hport->drvr, priv->intin);
      return ret;
    }

  ullvdbg("Endpoint allocated\n");
  return OK;
}

/****************************************************************************
 * Name: usbhost_hubdesc
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   hubclass - The USB host class instance.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static inline int usbhost_hubdesc(FAR struct usbhost_class_s *hubclass)
{
  FAR struct usbhost_hubpriv_s *priv;
  FAR struct usbhost_hubport_s *hport;
  FAR struct usb_ctrlreq_s *ctrlreq;
  struct usb_hubdesc_s hubdesc;
  uint16_t hubchar;
  int ret;

  DEBUGASSERT(hubclass != NULL);
  priv = &((FAR struct usbhost_hubclass_s *)hubclass)->hubpriv;

  DEBUGASSERT(hubclass->hport);
  hport  = hubclass->hport;

  /* Get the hub descriptor */

  ctrlreq = priv->ctrlreq;
  DEBUGASSERT(ctrlreq);

  ctrlreq->type = USB_REQ_DIR_IN | USBHUB_REQ_TYPE_HUB;
  ctrlreq->req  = USB_REQ_GETDESCRIPTOR;
  usbhost_putle16(ctrlreq->value, (USB_DESC_TYPE_HUB << 8));
  usbhost_putle16(ctrlreq->index, 0);
  usbhost_putle16(ctrlreq->len, USB_SIZEOF_HUBDESC);

  ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq, (FAR uint8_t *)&hubdesc);
  if (ret != OK)
    {
      udbg("ERROR: Failed to read hub descriptor: %d\n", ret);
      return ret;
    }

  priv->nports      = hubdesc.nports;

  hubchar           = usbhost_getle16(hubdesc.characteristics);
  priv->lpsm        = (hubchar & USBHUB_CHAR_LPSM_MASK) >> USBHUB_CHAR_LPSM_SHIFT;
  priv->compounddev = (hubchar & USBHUB_CHAR_COMPOUND) ? true : false;
  priv->ocmode      = (hubchar & USBHUB_CHAR_OCPM_MASK) >> USBHUB_CHAR_OCPM_SHIFT;
  priv->indicator   = (hubchar & USBHUB_CHAR_PORTIND) ? true : false;

  priv->pwrondelay  = (2 * hubdesc.pwrondelay);
  priv->ctrlcurrent = hubdesc.ctrlcurrent;

  return OK;
}

/****************************************************************************
 * Name: usbhost_hubpwr
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   hubclass - The USB host class instance.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static inline int usbhost_hubpwr(FAR struct usbhost_class_s *hubclass, bool on)
{
  FAR struct usbhost_hubpriv_s *priv;
  FAR struct usbhost_hubport_s *hport;
  FAR struct usb_ctrlreq_s *ctrlreq;

  DEBUGASSERT(hubclass != NULL);
  priv = &((FAR struct usbhost_hubclass_s *)hubclass)->hubpriv;

  DEBUGASSERT(hubclass->hport);
  hport  = hubclass->hport;

  if (on || ROOTHUB(hport))
    {
      uint16_t req;
      int port;
      int ret;

      if (on)
        {
          req = USB_REQ_SETFEATURE;
        }
      else
        {
          req = USB_REQ_CLEARFEATURE;
        }

     /* Set port power  */

     ctrlreq = priv->ctrlreq;
     DEBUGASSERT(ctrlreq);

      for (port = 1; port <= priv->nports; port++)
        {
          ctrlreq->type = USBHUB_REQ_TYPE_PORT;
          ctrlreq->req  = req;
          usbhost_putle16(ctrlreq->value, (USBHUB_PORT_FEAT_POWER << 8));
          usbhost_putle16(ctrlreq->index, port);
          usbhost_putle16(ctrlreq->len, 0);

          ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, NULL);
          if (ret != OK)
            {
              udbg("ERROR: Failed to power %d port %d: %d\n", on, port, ret);
              return ret;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: usbhost_hubevent
 *
 * Description:
 *   Handle a hub event.
 *
 * Input Parameters:
 *   xfer - The USB host class instance.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static void usbhost_hubevent(FAR void *arg)
{
  FAR struct usbhost_class_s *hubclass;
  FAR struct usbhost_hubport_s *newhub;
  FAR struct usbhost_hubport_s *hport;
  FAR struct usbhost_hubpriv_s *priv;
  FAR struct usb_ctrlreq_s *ctrlreq;
  struct usb_portstatus_s portstatus;
  uint16_t status;
  uint16_t change;
  uint16_t mask;
  uint16_t feat;
  uint8_t statusmap;
  int port;
  int ret;

  DEBUGASSERT(arg != NULL);
  hubclass = (FAR struct usbhost_class_s *)arg;
  priv     = &((FAR struct usbhost_hubclass_s *)hubclass)->hubpriv;

  DEBUGASSERT(priv->ctrlreq);
  ctrlreq = priv->ctrlreq;

  DEBUGASSERT(hubclass->hport);
  hport = hubclass->hport;

  statusmap = priv->buffer[0];

  for (port = 1; port <= priv->nports; port++)
    {
      /* Check if port status has changed */

      if (!(statusmap & (0x1 << port)))
        {
          continue;
        }

      /* Port status changed, check what happened */

      statusmap &= (~(0x1 << port));

      /* Read hub port status */

      ctrlreq->type = USB_REQ_DIR_IN | USBHUB_REQ_TYPE_PORT;
      ctrlreq->req  = USB_REQ_GETSTATUS;
      usbhost_putle16(ctrlreq->value, 0);
      usbhost_putle16(ctrlreq->index, port);
      usbhost_putle16(ctrlreq->len, USB_SIZEOF_PORTSTS);

      ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq,
                        (FAR uint8_t *)&portstatus);
      if (ret != OK)
        {
          udbg("ERROR: Failed to read port %d status: %d\n", port, ret);
          continue;
        }

      status = usbhost_getle16(portstatus.status);
      change = usbhost_getle16(portstatus.change);

      /* First, clear all change bits */

      mask = 0x1;
      feat = USBHUB_PORT_FEAT_CCONNECTION;
      while (change)
        {
          if (change & mask)
            {
              ctrlreq->type = USBHUB_REQ_TYPE_PORT;
              ctrlreq->req  = USB_REQ_CLEARFEATURE;
              usbhost_putle16(ctrlreq->value, (feat << 8));
              usbhost_putle16(ctrlreq->index, port);
              usbhost_putle16(ctrlreq->len, 0);

              ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, NULL);
              if (ret != OK)
                {
                  udbg("ERROR: Failed to clear port %d change mask %x: %d\n",
                       port, mask, ret);
                }

              change &= (~mask);
            }

          mask <<= 1;
          feat++;
        }

      change = usbhost_getle16(portstatus.change);

      /* Handle connect or disconnect, no power management */

      if (change & USBHUB_PORT_STAT_CCONNECTION)
        {
          uint16_t debouncetime = 0;
          uint16_t debouncestable = 0;
          uint16_t connection = 0xffff;

          uvdbg("port %d status %x change %x\n", port, status, change);

          /* Debounce */

          while (debouncetime < 1500)
            {
              ctrlreq->type = USB_REQ_DIR_IN | USBHUB_REQ_TYPE_PORT;
              ctrlreq->req  = USB_REQ_GETSTATUS;
              usbhost_putle16(ctrlreq->value, 0);
              usbhost_putle16(ctrlreq->index, port);
              usbhost_putle16(ctrlreq->len, USB_SIZEOF_PORTSTS);

              ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq,
                                (FAR uint8_t *)&portstatus);
              if (ret != OK)
                {
                  break;
                }

              status = usbhost_getle16(portstatus.status);
              change = usbhost_getle16(portstatus.change);

              if (!(change & USBHUB_PORT_STAT_CCONNECTION) &&
                   ((status & USBHUB_PORT_STAT_CONNECTION) == connection))
                {
                  debouncestable += 25;
                  if (debouncestable >= 100)
                    {
                      break;
                    }
                }
              else
                {
                  debouncestable = 0;
                  connection = status & USBHUB_PORT_STAT_CONNECTION;
                }

                if (change & USBHUB_PORT_STAT_CCONNECTION)
                  {
                    ctrlreq->type = USBHUB_REQ_TYPE_PORT;
                    ctrlreq->req  = USB_REQ_CLEARFEATURE;
                    usbhost_putle16(ctrlreq->value, (USBHUB_PORT_FEAT_CCONNECTION << 8));
                    usbhost_putle16(ctrlreq->index, port);
                    usbhost_putle16(ctrlreq->len, 0);

                    (void)DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, NULL);
                  }

              debouncetime += 25;
              up_mdelay(25);
            }

          if ((ret != OK) || (debouncetime >= 1500))
            {
              udbg("ERROR: Failed to debounce port %d: %d\n", port, ret);
              continue;
            }

          if (status & USBHUB_PORT_STAT_CONNECTION)
            {
              /* Connect */

              ctrlreq->type = USBHUB_REQ_TYPE_PORT;
              ctrlreq->req  = USB_REQ_SETFEATURE;
              usbhost_putle16(ctrlreq->value, (USBHUB_PORT_FEAT_RESET << 8));
              usbhost_putle16(ctrlreq->index, port);
              usbhost_putle16(ctrlreq->len, 0);

              ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, NULL);
              if (ret != OK)
                {
                  udbg("ERROR: ailed to reset port %d: %d\n", port, ret);
                  continue;
                }

              up_mdelay(100);

              ctrlreq->type = USB_REQ_DIR_IN | USBHUB_REQ_TYPE_PORT;
              ctrlreq->req  = USB_REQ_GETSTATUS;
              usbhost_putle16(ctrlreq->value, 0);
              usbhost_putle16(ctrlreq->index, port);
              usbhost_putle16(ctrlreq->len, USB_SIZEOF_PORTSTS);

              ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq,
                                (FAR uint8_t *)&portstatus);
              if (ret != OK)
                {
                  udbg("ERROR: Failed to reset port %d: %d\n", port, ret);
                  continue;
                }

              status = usbhost_getle16(portstatus.status);
              change = usbhost_getle16(portstatus.change);

              uvdbg("port %d status %x change %x after reset\n",
                    port, status, change);

              if (!(status & USBHUB_PORT_STAT_RESET) &&
                   (status & USBHUB_PORT_STAT_ENABLE))
                {
                  uint8_t speed;

                  if (change & USBHUB_PORT_STAT_CRESET)
                    {
                      ctrlreq->type = USBHUB_REQ_TYPE_PORT;
                      ctrlreq->req  = USB_REQ_CLEARFEATURE;
                      usbhost_putle16(ctrlreq->value, (USBHUB_PORT_FEAT_CRESET << 8));
                      usbhost_putle16(ctrlreq->index, port);
                      usbhost_putle16(ctrlreq->len, 0);

                      (void)DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, NULL);
                    }

                  if (status & USBHUB_PORT_STAT_HIGH_SPEED)
                    {
                      speed = USB_SPEED_HIGH;
                    }
                  else if (status & USBHUB_PORT_STAT_LOW_SPEED)
                    {
                      speed = USB_SPEED_LOW;
                    }
                  else
                    {
                      speed = USB_SPEED_FULL;
                    }

                  /* Allocate new hub class and enumerate */

                  newhub = usbhost_allochub(hport->drvr, hubclass, speed, port);
                  if (newhub)
                    {
                      udbg("ERROR: Failed to allocated class\n");
                      uvdbg("enumerate port %d speed %d\n", port, speed);

                      ret = usbhost_enumerate(newhub, &priv->childclass[port]);
                      if (ret != OK)
                        {
                          udbg("ERROR: Failed to enumerate port %d: %d\n",
                               port, ret);
                          usbhost_freehub(hport);
                        }
                    }
                }
              else
                {
                  udbg("ERROR: Failed to enable port %d\n", port);
                  continue;
                }
            }
          else
            {
              /* Disconnect */
            }
        }
      else if (change)
        {
          udbg("WARNING: status %x change %x not handled\n", status, change);
        }
    }

  if (statusmap & 0x1)
    {
      /* Hub status changed */

      udbg("WARNING: Hub status changed, not handled\n");
    }

  /* Get the number hub event */

  ret = DRVR_ASYNCH(hport->drvr, priv->intin, (FAR uint8_t *)priv->ctrlreq,
                    sizeof(struct usb_ctrlreq_s), usbhost_callback,
                    hubclass);
  if (ret != OK)
    {
      udbg("ERROR: Failed to queue interrupt endpoint: %d\n", ret);
    }
}

/****************************************************************************
 * Name: usbhost_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Values:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t usbhost_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: usbhost_putle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/****************************************************************************
 * Name: usbhost_callback
 *
 * Description:
 *   Handle end of transfer callback.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_callback(FAR void *arg, int result)
{
  FAR struct usbhost_class_s *hubclass;
  FAR struct usbhost_hubpriv_s *priv;

  DEBUGASSERT(arg != NULL);
  hubclass = (FAR struct usbhost_class_s *)arg;
  priv     = &((FAR struct usbhost_hubclass_s *)hubclass)->hubpriv;

  if (result != OK)
    {
      priv->buffer[0] = 0;
    }

  (void)work_queue(HPWORK, &priv->work, (worker_t)usbhost_hubevent,
                   hubclass, 0);
}

/****************************************************************************
 * struct usbhost_registry_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_create
 *
 * Description:
 *   This function implements the create() method of struct usbhost_registry_s.
 *   The create() method is a callback into the class implementation.  It is
 *   used to (1) create a new instance of the USB host class state and to (2)
 *   bind a USB host driver "session" to the class instance.  Use of this
 *   create() method will support environments where there may be multiple
 *   USB ports and multiple USB devices simultaneously connected.
 *
 * Input Parameters:
 *   hport - The hub port that manages the new class instance.
 *   id - In the case where the device supports multiple base classes,
 *     subclasses, or protocols, this specifies which to configure for.
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s that can be used by the USB host driver to communicate
 *   with the USB host class.  NULL is returned on failure; this function
 *   will fail only if the hport input parameter is NULL or if there are
 *   insufficient resources to create another USB host class instance.
 *
 ****************************************************************************/

static FAR struct usbhost_class_s *
  usbhost_create(FAR struct usbhost_hubport_s *hport,
                 FAR const struct usbhost_id_s *id)
{
  FAR struct usbhost_hubclass_s *alloc;
  FAR struct usbhost_class_s *hubclass;
  FAR struct usbhost_hubpriv_s *priv;
  size_t maxlen;
  int child;
  int ret;

  /* Allocate a USB host class instance */

  alloc = kmm_zalloc(sizeof(struct usbhost_hubclass_s));
  if (alloc == NULL)
    {
      return NULL;
    }

  /* Initialize the public class structure */

  hubclass               = &alloc->hubclass;
  hubclass->hport        = hport;
  hubclass->connect      = usbhost_connect;
  hubclass->disconnected = usbhost_disconnected;

  /* Initialize the private class structure */

  priv = &alloc->hubpriv;

  /* Allocate memory for control requests */

  ret = DRVR_ALLOC(hport->drvr, (FAR uint8_t **)&priv->ctrlreq, &maxlen);
  if (ret != OK)
    {
      udbg("DRVR_ALLOC failed: %d\n", ret);
      goto errout_with_hub;
    }

  /* Allocate buffer for status change (INT) endpoint */

  ret = DRVR_IOALLOC(hport->drvr, &priv->buffer, 1);
  if (ret != OK)
    {
      udbg("DRVR_ALLOC failed: %d\n", ret);
      goto errout_with_ctrlreq;
    }

  /* The initial reference count is 1... One reference is held by the driver */

  priv->crefs = 1;

  /* Initialize semaphores (this works okay in the interrupt context) */

  sem_init(&priv->exclsem, 0, 1);

  /* Initialize child class pointers */

  for (child = 0; child < USBHUB_MAX_PORTS; child++)
    {
      priv->childclass[child] = NULL;
    }

  return hubclass;

errout_with_ctrlreq:
  kmm_free(priv->ctrlreq);

errout_with_hub:
  kmm_free(priv);
  return NULL;
}

/****************************************************************************
 * struct usbhost_class_s methods
 ****************************************************************************/
/****************************************************************************
 * Name: usbhost_connect
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to
 *     create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration
 *     descriptor.
 *   desclen - The length in bytes of the configuration descriptor.
 *   funcaddr - The USB address of the function containing the endpoint that
 *     EP0 controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 *   NOTE that the class instance remains valid upon return with a failure.  It is
 *   the responsibility of the higher level enumeration logic to call
 *   CLASS_DISCONNECTED to free up the class driver resources.
 *
 * Assumptions:
 *   - This function will *not* be called from an interrupt handler.
 *   - If this function returns an error, the USB host controller driver
 *     must call to DISCONNECTED method to recover from the error
 *
 ****************************************************************************/

static int usbhost_connect(FAR struct usbhost_class_s *hubclass,
                           FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_hubpriv_s *priv;
  FAR struct usbhost_hubport_s *hport;
  int ret;

  DEBUGASSERT(hubclass != NULL);
  priv = &((FAR struct usbhost_hubclass_s *)hubclass)->hubpriv;

  DEBUGASSERT(configdesc != NULL && desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the endpoints */

  ret = usbhost_cfgdesc(hubclass, configdesc, desclen);
  if (ret != OK)
    {
      udbg("ERROR: Failed to parse config descriptor: %d\n", ret);
    }
  else
    {
      /* Read the hub descriptor */

      ret = usbhost_hubdesc(hubclass);
      if (ret != OK)
        {
          return ret;
        }

      /* Power on hub (i.e. power on all hub ports) */

      ret = usbhost_hubpwr(hubclass, true);
      if (ret != OK)
        {
          return ret;
        }

      /* INT request to periodically check port status */

      DEBUGASSERT(hubclass->hport);
      hport = hubclass->hport;

      ret = DRVR_ASYNCH(hport->drvr, priv->intin, (FAR uint8_t *)&priv->ctrlreq,
                        sizeof(struct usb_ctrlreq_s), usbhost_callback,
                        hubclass);
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_disconnected
 *
 * Description:
 *   This function implements the disconnected() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to inform the class that the USB device has
 *   been disconnected.
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to
 *     create().
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function cannot be called from an interrupt handler.
 *
 ****************************************************************************/

static int usbhost_disconnected(struct usbhost_class_s *hubclass)
{
  FAR struct usbhost_hubpriv_s *priv;
  FAR struct usbhost_hubport_s *hport;
  irqstate_t flags;

  DEBUGASSERT(hubclass != NULL);
  priv = &((FAR struct usbhost_hubclass_s *)hubclass)->hubpriv;

  /* Set an indication to any users of the device that the device is no
   * longer available.
   */

  flags              = irqsave();
  priv->disconnected = true;

  /* Now check the number of references on the class instance.  If it is one,
   * then we can free the class instance now.  Otherwise, we will have to
   * wait until the holders of the references free them by closing the
   * block driver.
   */

  ullvdbg("crefs: %d\n", priv->crefs);
  if (priv->crefs == 1)
    {
      DEBUGASSERT(hubclass->hport);
      hport = hubclass->hport;

      /* Free buffer for status change (INT) endpoint */

      DRVR_IOFREE(hport->drvr, priv->buffer);

      /* Power off (for root hub only) */

      (void)usbhost_hubpwr(hubclass, false);

      /* Destroy the class instance */

      usbhost_destroy(hubclass);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_hubinit
 *
 * Description:
 *   Initialize the USB hub class.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host storage class.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_hubinit(void)
{
  /* Advertise our availability to support (certain) mass storage devices */

  return usbhost_registerclass(&g_hub);
}

#endif  /* CONFIG_USBHOST_HUB */

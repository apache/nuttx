/****************************************************************************
 * drivers/usbhost/usbhost_hub.c
 *
 *   Copyright (C) 2015-2017 Gregory Nutt. All rights reserved.
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

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/hub.h>
#include <nuttx/usb/usbhost_devaddr.h>

#ifdef CONFIG_USBHOST_HUB

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* It is necessary to perform work on the low-priority work queue (vs. the
 * high priority work queue) because:
 *
 * 1. Deferred work requires some delays and waiting, and
 * 2. There may be dependencies between the waiting and driver interrupt
 *    related work.  Since that interrupt related work will performed on the
 *    high priority work queue, there would be the likelihood of deadlocks
 *    if you wait for events on the high priority work thread that can only
 *    occur if the high priority work thread is available to post those events.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#elif !defined(CONFIG_SCHED_LPWORK)
#  error Low-priority work queue support is required (CONFIG_SCHED_LPWORK)
#endif

#ifndef CONFIG_USBHOST_ASYNCH
#  error Asynchronous transfer support is required (CONFIG_USBHOST_ASYNCH)
#endif

#ifndef CONFIG_USBHOST_HUB_POLLMSEC
#  define CONFIG_USBHOST_HUB_POLLMSEC 400
#endif

/* Perform polling actions with a delay on the low priority work queue, if
 * configured
 */

#define POLL_DELAY          MSEC2TICK(CONFIG_USBHOST_HUB_POLLMSEC)

/* Used in usbhost_cfgdesc() */

#define USBHOST_IFFOUND     0x01 /* Required I/F descriptor found */
#define USBHOST_EPINFOUND   0x02 /* Required interrupt IN EP descriptor found */
#define USBHOST_ALLFOUND    (USBHOST_IFFOUND | USBHOST_EPINFOUND)

/* Maximum size of an interrupt IN transfer */

#define INTIN_BUFSIZE       ((USBHUB_MAX_PORTS + 8) >> 3)

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
  sem_t                     exclsem;      /* Used to maintain mutual exclusive access */
  struct work_s             work;         /* Used for deferred callback work */
  usbhost_ep_t              intin;        /* Interrupt IN endpoint */

  /* Hub port descriptors */

  struct usbhost_hubport_s  hport[USBHUB_MAX_PORTS];
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

/* Helpers for usbhost_connect() */

static inline int usbhost_cfgdesc(FAR struct usbhost_class_s *hubclass,
             FAR const uint8_t *configdesc, int desclen);
static inline int usbhost_hubdesc(FAR struct usbhost_class_s *hubclass);
static inline int usbhost_hubpwr(FAR struct usbhost_hubpriv_s *priv,
                                 FAR struct usbhost_hubport_s *hport,
                                 bool on);
static void usbhost_hub_event(FAR void *arg);
static void usbhost_disconnect_event(FAR void *arg);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(const uint8_t *val);
static void usbhost_putle16(uint8_t *dest, uint16_t val);
static void usbhost_callback(FAR void *arg, ssize_t nbytes);

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
 * Name: usbhost_hport_deactivate
 *
 * Description:
 *   Free a hub resource previously allocated by usbhost_hport_activate().
 *
 * Input Parameters:
 *   hport - A reference to the hub port instance to be freed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_hport_deactivate(FAR struct usbhost_hubport_s *hport)
{
  uinfo("Deactivating: %s port %d\n",
        ROOTHUB(hport) ? "Root" : "Hub", hport->port);

  /* Don't free the control pipe of root hub ports! */

  if (!ROOTHUB(hport) && hport->ep0 != NULL)
    {
      /* Free the control endpoint */

      DRVR_EPFREE(hport->drvr, hport->ep0);
      hport->ep0 = NULL;
    }

  /* Free the function address if one has been assigned */

  usbhost_devaddr_destroy(hport, hport->funcaddr);
  hport->funcaddr = 0;

  /* If this is a downstream hub port, then there should be no class driver
   * associated wit it.
   */

  DEBUGASSERT(ROOTHUB(hport) || hport->devclass == NULL);
}

/****************************************************************************
 * Name: usbhost_hport_activate
 *
 * Description:
 *   Activate a hub port by assigning it a control endpoint.  This actions
 *   only occur when a device is connected to the hub endpoint.
 *
 * Input Parameters:
 *   hport - The hub port to be activated.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int usbhost_hport_activate(FAR struct usbhost_hubport_s *hport)
{
  struct usbhost_epdesc_s epdesc;
  int ret;

  uinfo("Activating port %d\n", hport->port);

  epdesc.hport        = hport;
  epdesc.addr         = 0;
  epdesc.in           = false;
  epdesc.xfrtype      = USB_EP_ATTR_XFER_CONTROL;
  epdesc.interval     = 0;
  epdesc.mxpacketsize = (hport->speed == USB_SPEED_HIGH) ? 64 : 8;

  ret = DRVR_EPALLOC(hport->drvr, &epdesc, &hport->ep0);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate ep0: %d\n", ret);
    }

  return ret;
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
 * Returned Value:
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

            uinfo("Interface descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Save the interface number and mark ONLY the interface found */

            priv->ifno = ifdesc->ifno;
            found      = USBHOST_IFFOUND;
          }
          break;

        /* Endpoint descriptor.  Here, we expect one interrupt IN endpoints. */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc =
              (FAR struct usb_epdesc_s *)configdesc;

            uinfo("Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for an interrupt endpoint. */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
                USB_EP_ATTR_XFER_INT)
              {
                /* Yes.. it is a interrupt endpoint.  IN or OUT? */

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an OUT interrupt endpoint. Ignore */

                    uinfo("Interrupt OUT EP addr:%d mxpacketsize:%d\n",
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

                    uinfo("Interrupt IN EP: addr=%d interval=%d mxpacketsize=%d\n",
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
      uerr("ERROR: Found IF=%s EPIN=%s\n",
           (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
           (found & USBHOST_EPINFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the interrupt IN endpoint */

  ret = DRVR_EPALLOC(hport->drvr, &intindesc, &priv->intin);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate Interrupt IN endpoint: %d\n", ret);
      (void)DRVR_EPFREE(hport->drvr, priv->intin);
      return ret;
    }

  uinfo("Endpoint allocated\n");
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
 * Returned Value:
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

  uinfo("Read hub descriptor\n");

  DEBUGASSERT(hubclass != NULL);
  priv = &((FAR struct usbhost_hubclass_s *)hubclass)->hubpriv;

  DEBUGASSERT(hubclass->hport);
  hport  = hubclass->hport;

  /* Get the hub descriptor */

  ctrlreq = priv->ctrlreq;
  DEBUGASSERT(ctrlreq);

  ctrlreq->type = USB_REQ_DIR_IN | USBHUB_REQ_TYPE_HUB;
  ctrlreq->req  = USBHUB_REQ_GETDESCRIPTOR;
  usbhost_putle16(ctrlreq->value, (USB_DESC_TYPE_HUB << 8));
  usbhost_putle16(ctrlreq->index, 0);
  usbhost_putle16(ctrlreq->len, USB_SIZEOF_HUBDESC);

  ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq, (FAR uint8_t *)&hubdesc);
  if (ret < 0)
    {
      uerr("ERROR: Failed to read hub descriptor: %d\n", ret);
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

  uinfo("Hub Descriptor:\n");
  uinfo("  bDescLength:         %d\n", hubdesc.len);
  uinfo("  bDescriptorType:     0x%02x\n", hubdesc.type);
  uinfo("  bNbrPorts:           %d\n", hubdesc.nports);
  uinfo("  wHubCharacteristics: 0x%04x\n", usbhost_getle16(hubdesc.characteristics));
  uinfo("    lpsm:              %d\n", priv->lpsm);
  uinfo("    compounddev:       %s\n", priv->compounddev ? "TRUE" : "FALSE");
  uinfo("    ocmode:            %d\n", priv->ocmode);
  uinfo("    indicator:         %s\n", priv->indicator ? "TRUE" : "FALSE");
  uinfo("  bPwrOn2PwrGood:      %d\n", hubdesc.pwrondelay);
  uinfo("    pwrondelay:        %d\n", priv->pwrondelay);
  uinfo("  bHubContrCurrent:    %d\n", hubdesc.ctrlcurrent);
  uinfo("  DeviceRemovable:     %d\n", hubdesc.devattached);
  uinfo("  PortPwrCtrlMask:     %d\n", hubdesc.pwrctrlmask);

  return OK;
}

/****************************************************************************
 * Name: usbhost_hubpwr
 *
 * Description:
 *   Self-powered hubs may have power switches that control delivery of
 *   power downstream facing ports but it is not required. Bus-powered hubs
 *   are required to have power switches. A hub with power switches can
 *   switch power to all ports as a group/gang, to each port individually, or
 *   have an arbitrary number of gangs of one or more ports.
 *
 *   A hub indicates whether or not it supports power switching by the
 *   setting of the Logical Power Switching Mode field in wHubCharacteristics.
 *   If a hub supports per-port power switching, then the power to a port is
 *   turned on when a SetPortFeature(PORT_POWER) request is received for the
 *   port. Port power is turned off when the port is in the Powered-off or
 *   Not Configured states. If a hub supports ganged power switching, then
 *   the power to all ports in a gang is turned on when any port in a gang
 *   receives a SetPortFeature(PORT_POWER) request. The power to a gang is
 *   not turned off unless all ports in a gang are in the Powered-off or Not
 *   Configured states. Note, the power to a port is not turned on by a
 *   SetPortFeature(PORT_POWER) if both C_HUB_LOCAL_POWER and Local Power
 *   Status (in wHubStatus) are set to 1B at the time when the request is
 *   executed and the PORT_POWER feature would be turned on.
 *
 * Input Parameters:
 *   priv - The USB hub private data
 *   hport - The port on the parent hub where the this hub is connected.
 *   on - True: enable power; false: Disable power
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int usbhost_hubpwr(FAR struct usbhost_hubpriv_s *priv,
                          FAR struct usbhost_hubport_s *hport,
                          bool on)
{
  FAR struct usb_ctrlreq_s *ctrlreq;
  uint16_t req;
  int port;
  int ret;

  /* Are we enabling or disabling power? */

  if (on)
    {
      req = USBHUB_REQ_SETFEATURE;
    }
  else
    {
      req = USBHUB_REQ_CLEARFEATURE;
    }

  /* Enable/disable power to all downstream ports */

  ctrlreq = priv->ctrlreq;
  DEBUGASSERT(ctrlreq);

   for (port = 1; port <= priv->nports; port++)
     {
       ctrlreq->type = USBHUB_REQ_TYPE_PORT;
       ctrlreq->req  = req;
       usbhost_putle16(ctrlreq->value, USBHUB_PORT_FEAT_POWER);
       usbhost_putle16(ctrlreq->index, port);
       usbhost_putle16(ctrlreq->len, 0);

       ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, NULL);
       if (ret < 0)
         {
           uerr("ERROR: Failed to power %s port %d: %d\n",
                on ? "UP" : "DOWN", port, ret);
           return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: usbhost_hub_event
 *
 * Description:
 *   Handle a hub event.
 *
 * Input Parameters:
 *   xfer - The USB host class instance.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static void usbhost_hub_event(FAR void *arg)
{
  FAR struct usbhost_class_s *hubclass;
  FAR struct usbhost_hubport_s *hport;
  FAR struct usbhost_hubport_s *connport;
  FAR struct usbhost_hubpriv_s *priv;
  FAR struct usb_ctrlreq_s *ctrlreq;
  struct usb_portstatus_s portstatus;
  irqstate_t flags;
  uint16_t status;
  uint16_t change;
  uint16_t mask;
  uint16_t feat;
  uint8_t statuschange;
  int port;
  int ret;

  DEBUGASSERT(arg != NULL);
  hubclass = (FAR struct usbhost_class_s *)arg;
  priv     = &((FAR struct usbhost_hubclass_s *)hubclass)->hubpriv;

  /* Has the hub been disconnected? */

  if (priv->disconnected)
    {
      uinfo("Disconnected\n");
      return;
    }

  /* No.. then set up to process the hub event */

  DEBUGASSERT(priv->ctrlreq);
  ctrlreq = priv->ctrlreq;

  DEBUGASSERT(hubclass->hport);
  hport = hubclass->hport;

  statuschange = priv->buffer[0];
  uinfo("StatusChange: %02x\n", statuschange);

  /* Check for status change on any port */

  for (port = 1; port <= priv->nports; port++)
    {
      /* Check if port status has changed */

      if ((statuschange & (1 << port)) == 0)
        {
          continue;
        }

      uinfo("Port %d status change\n", port);

      /* Port status changed, check what happened */

      statuschange &= ~(1 << port);

      /* Read hub port status */

      ctrlreq->type = USB_REQ_DIR_IN | USBHUB_REQ_TYPE_PORT;
      ctrlreq->req  = USBHUB_REQ_GETSTATUS;
      usbhost_putle16(ctrlreq->value, 0);
      usbhost_putle16(ctrlreq->index, port);
      usbhost_putle16(ctrlreq->len, USB_SIZEOF_PORTSTS);

      ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq,
                        (FAR uint8_t *)&portstatus);
      if (ret < 0)
        {
          uerr("ERROR: Failed to read port %d status: %d\n", port, ret);
          continue;
        }

      status = usbhost_getle16(portstatus.status);
      change = usbhost_getle16(portstatus.change);

      /* First, clear all change bits */

      mask = 1;
      feat = USBHUB_PORT_FEAT_CCONNECTION;
      while (change)
        {
          if (change & mask)
            {
              ctrlreq->type = USBHUB_REQ_TYPE_PORT;
              ctrlreq->req  = USBHUB_REQ_CLEARFEATURE;
              usbhost_putle16(ctrlreq->value, feat);
              usbhost_putle16(ctrlreq->index, port);
              usbhost_putle16(ctrlreq->len, 0);

              ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, NULL);
              if (ret < 0)
                {
                  uerr("ERROR: Failed to clear port %d change mask %04x: %d\n",
                       port, mask, ret);
                }

              change &= (~mask);
            }

          mask <<= 1;
          feat++;
        }

      change = usbhost_getle16(portstatus.change);

      /* Handle connect or disconnect, no power management */

      if ((change & USBHUB_PORT_STAT_CCONNECTION) != 0)
        {
          uint16_t debouncetime = 0;
          uint16_t debouncestable = 0;
          uint16_t connection = 0xffff;

          uinfo("Port %d status %04x change %04x\n", port, status, change);

          /* Debounce */

          while (debouncetime < 1500)
            {
              ctrlreq->type = USB_REQ_DIR_IN | USBHUB_REQ_TYPE_PORT;
              ctrlreq->req  = USBHUB_REQ_GETSTATUS;
              usbhost_putle16(ctrlreq->value, 0);
              usbhost_putle16(ctrlreq->index, port);
              usbhost_putle16(ctrlreq->len, USB_SIZEOF_PORTSTS);

              ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq,
                                (FAR uint8_t *)&portstatus);
              if (ret < 0)
                {
                  uerr("ERROR: Failed to get port %d status: %d\n", port, ret);
                  break;
                }

              status = usbhost_getle16(portstatus.status);
              change = usbhost_getle16(portstatus.change);

              if ((change & USBHUB_PORT_STAT_CCONNECTION) == 0 &&
                  (status & USBHUB_PORT_STAT_CONNECTION)  == connection)
                {
                  debouncestable += 25;
                  if (debouncestable >= 100)
                    {
                      uinfo("Port %d debouncestable=%d\n", port, debouncestable);
                      break;
                    }
                }
              else
                {
                  debouncestable = 0;
                  connection = status & USBHUB_PORT_STAT_CONNECTION;
                }

              if ((change & USBHUB_PORT_STAT_CCONNECTION) != 0)
                {
                  ctrlreq->type = USBHUB_REQ_TYPE_PORT;
                  ctrlreq->req  = USBHUB_REQ_CLEARFEATURE;
                  usbhost_putle16(ctrlreq->value, USBHUB_PORT_FEAT_CCONNECTION);
                  usbhost_putle16(ctrlreq->index, port);
                  usbhost_putle16(ctrlreq->len, 0);

                  (void)DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, NULL);
                }

              debouncetime += 25;
              nxsig_usleep(25*1000);
            }

          if (ret < 0 || debouncetime >= 1500)
            {
              uerr("ERROR: Failed to debounce port %d: %d\n", port, ret);
              continue;
            }

          if ((status & USBHUB_PORT_STAT_CONNECTION) != 0)
            {
              /* Device connected to a port on the hub */

              uinfo("Connection on port %d\n", port);

              ctrlreq->type = USBHUB_REQ_TYPE_PORT;
              ctrlreq->req  = USBHUB_REQ_SETFEATURE;
              usbhost_putle16(ctrlreq->value, USBHUB_PORT_FEAT_RESET);
              usbhost_putle16(ctrlreq->index, port);
              usbhost_putle16(ctrlreq->len, 0);

              ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, NULL);
              if (ret < 0)
                {
                  uerr("ERROR: Failed to reset port %d: %d\n", port, ret);
                  continue;
                }

              nxsig_usleep(100*1000);

              ctrlreq->type = USB_REQ_DIR_IN | USBHUB_REQ_TYPE_PORT;
              ctrlreq->req  = USBHUB_REQ_GETSTATUS;
              usbhost_putle16(ctrlreq->value, 0);
              usbhost_putle16(ctrlreq->index, port);
              usbhost_putle16(ctrlreq->len, USB_SIZEOF_PORTSTS);

              ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq,
                                (FAR uint8_t *)&portstatus);
              if (ret < 0)
                {
                  uerr("ERROR: Failed to get port %d status: %d\n", port, ret);
                  continue;
                }

              status = usbhost_getle16(portstatus.status);
              change = usbhost_getle16(portstatus.change);

              uinfo("port %d status %04x change %04x after reset\n",
                    port, status, change);

              if ((status & USBHUB_PORT_STAT_RESET)  == 0 &&
                  (status & USBHUB_PORT_STAT_ENABLE) != 0)
                {
                  if ((change & USBHUB_PORT_STAT_CRESET) != 0)
                    {
                      ctrlreq->type = USBHUB_REQ_TYPE_PORT;
                      ctrlreq->req  = USBHUB_REQ_CLEARFEATURE;
                      usbhost_putle16(ctrlreq->value, USBHUB_PORT_FEAT_CRESET);
                      usbhost_putle16(ctrlreq->index, port);
                      usbhost_putle16(ctrlreq->len, 0);

                      (void)DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, NULL);
                    }

                  connport = &priv->hport[port];
                  if ((status & USBHUB_PORT_STAT_HIGH_SPEED) != 0)
                    {
                      connport->speed = USB_SPEED_HIGH;
                    }
                  else if ((status & USBHUB_PORT_STAT_LOW_SPEED) != 0)
                    {
                      connport->speed = USB_SPEED_LOW;
                    }
                  else
                    {
                      connport->speed = USB_SPEED_FULL;
                    }

                  /* Activate the hub port by assigning it a control endpoint. */

                  ret = usbhost_hport_activate(connport);
                  if (ret < 0)
                    {
                      uerr("ERROR: usbhost_hport_activate failed: %d\n", ret);
                    }
                  else
                    {
                      /* Inform waiters that a new device has been connected */

                      ret = DRVR_CONNECT(connport->drvr, connport, true);
                      if (ret < 0)
                        {
                          uerr("ERROR: DRVR_CONNECT failed: %d\n", ret);
                          usbhost_hport_deactivate(connport);
                        }
                    }
                }
              else
                {
                  uerr("ERROR: Failed to enable port %d\n", port);
                  continue;
                }
            }
          else
            {
              /* Device disconnected from a port on the hub.  Release port
               * resources.
               */

              uinfo("Disconnection on port %d\n", port);

              /* Free any devices classes connect on this hub port */

              connport = &priv->hport[port];
              if (connport->devclass != NULL)
                {
                  CLASS_DISCONNECTED(connport->devclass);
                  connport->devclass = NULL;
                }

              /* Free any resources used by the hub port */

              usbhost_hport_deactivate(connport);
            }
        }
      else if (change)
        {
          uwarn("WARNING: status %04x change %04x not handled\n", status, change);
        }
    }

  /* Check for hub status change */

  if ((statuschange & 1) != 0)
    {
      /* Hub status changed */

      uwarn("WARNING: Hub status changed, not handled\n");
    }

  /* The preceding sequence of events may take a significant amount of
   * time and it is possible that the hub may have been removed while this
   * logic operated.  In any event, we will get here after several failures.
   * But we do not want to schedule another hub event if the hub has been
   * removed.
   */

  flags = enter_critical_section();
  if (!priv->disconnected)
    {
      /* Wait for the next hub event */

      ret = DRVR_ASYNCH(hport->drvr, priv->intin, (FAR uint8_t *)priv->buffer,
                        INTIN_BUFSIZE, usbhost_callback, hubclass);
      if (ret < 0)
        {
          uerr("ERROR: Failed to queue interrupt endpoint: %d\n", ret);
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: usbhost_disconnect_event
 *
 * Description:
 *   This function performs the disconnect() actions on the worker thread.
 *   This work was scheduled when by the usbhost_disconnected() method after
 *   the HCD informs use that the device has been disconnected.
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to
 *     create().
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   Probably called from an interrupt handler.
 *
 ****************************************************************************/

static void usbhost_disconnect_event(FAR void *arg)
{
  FAR struct usbhost_class_s *hubclass = (FAR struct usbhost_class_s *)arg;
  FAR struct usbhost_hubpriv_s *priv;
  FAR struct usbhost_hubport_s *hport;
  FAR struct usbhost_hubport_s *child;
  irqstate_t flags;
  int port;

  uinfo("Disconnecting\n");

  DEBUGASSERT(hubclass != NULL && hubclass->hport != NULL);
  priv  = &((FAR struct usbhost_hubclass_s *)hubclass)->hubpriv;
  hport = hubclass->hport;

  uinfo("Destroying hub on port  %d\n", hport->port);

  /* Set an indication to any users of the device that the device is no
   * longer available.
   */

  flags = enter_critical_section();

  /* Cancel any pending transfers on the interrupt IN pipe */

  DRVR_CANCEL(hport->drvr, priv->intin);

  /* Cancel any pending port status change events */

  work_cancel(LPWORK, &priv->work);

  /* Disable power to all downstream ports */

  (void)usbhost_hubpwr(priv, hport, false);

  /* Free the allocated control request */

  DRVR_FREE(hport->drvr, (FAR uint8_t *)priv->ctrlreq);

  /* Free buffer for status change (INT) endpoint */

  DRVR_IOFREE(hport->drvr, priv->buffer);

  /* Destroy the interrupt IN endpoint */

  DRVR_EPFREE(hport->drvr, priv->intin);

  /* Release per-port resources */

  for (port = 0; port < USBHUB_MAX_PORTS; port++)
    {
      /* Free any devices classes connect on this hub port */

      child = &priv->hport[port];
      if (child->devclass != NULL)
        {
          CLASS_DISCONNECTED(child->devclass);
          child->devclass = NULL;
        }

      /* Free any resources used by the hub port */

      usbhost_hport_deactivate(child);
    }

  /* Deactivate the parent hub port (unless it is the root hub port) */

  usbhost_hport_deactivate(hport);

  /* Destroy the semaphores */

  nxsem_destroy(&priv->exclsem);

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(hport->drvr, hport);

  /* Free the class instance */

  kmm_free(hubclass);
  hport->devclass = NULL;
  leave_critical_section(flags);
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
 * Returned Value:
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
 * Returned Value:
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
 *   arg - The argument provided with the asynchronous I/O was setup
 *   nbytes - The number of bytes actually transferred (or a negated errno
 *     value;
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Probably called from an interrupt handler.
 *
 ****************************************************************************/

static void usbhost_callback(FAR void *arg, ssize_t nbytes)
{
  FAR struct usbhost_class_s *hubclass;
  FAR struct usbhost_hubpriv_s *priv;
  uint32_t delay = 0;

  DEBUGASSERT(arg != NULL);
  hubclass = (FAR struct usbhost_class_s *)arg;
  priv     = &((FAR struct usbhost_hubclass_s *)hubclass)->hubpriv;

  /* Check for a failure.  On higher end host controllers, the asynchronous
   * transfer will pend until data is available (OHCI and EHCI).  On lower
   * end host controllers (like STM32 and EFM32), the transfer will fail
   * immediately when the device NAKs the first attempted interrupt IN
   * transfer (with nbytes == -EAGAIN).  In that case (or in the case of
   * other errors), we must fall back to polling.
   */

  if (nbytes < 0)
    {
      /* This debug output is good to know, but really a nuisance for
       * those configurations where we have to fall back to polling.
       * FIX:  Don't output the message is the result is -EAGAIN.
       */

#if defined(CONFIG_DEBUG_USB) && !defined(CONFIG_DEBUG_INFO)
      if (nbytes != -EAGAIN)
#endif
        {
          uerr("ERROR: Transfer failed: %d\n", (int)nbytes);
        }

      /* Indicate there there is nothing to do.  So when the work is
       * performed, nothing will happen other than we will set to receive
       * the next event.
       */

      priv->buffer[0] = 0;

      /* We don't know the nature of the failure, but we need to do all that
       * we can do to avoid a CPU hog error loop.
       *
       * Use the low-priority work queue and delay polling for the next
       * event.  We want to use as little CPU bandwidth as possible in this
       * case.
       */

      delay = POLL_DELAY;
    }

  /* The work structure should always be available since hub communications
   * are serialized.  However, there is a remote chance that this may
   * collide with a hub disconnection event.
   */

  if (work_available(&priv->work) && !priv->disconnected)
    {
      (void)work_queue(LPWORK, &priv->work, (worker_t)usbhost_hub_event,
                       hubclass, delay);
    }
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
 * Returned Value:
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
  int port;
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
  if (ret < 0)
    {
      uerr("ERROR: DRVR_ALLOC failed: %d\n", ret);
      goto errout_with_hub;
    }

  /* Allocate buffer for status change (INT) endpoint. */

  ret = DRVR_IOALLOC(hport->drvr, &priv->buffer, INTIN_BUFSIZE);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_IOALLOC failed: %d\n", ret);
      goto errout_with_ctrlreq;
    }

  /* Initialize semaphores (this works okay in the interrupt context) */

  nxsem_init(&priv->exclsem, 0, 1);

  /* Initialize per-port data */

  for (port = 0; port < USBHUB_MAX_PORTS; port++)
    {
      FAR struct usbhost_hubport_s *child;

      /* Initialize the hub port descriptor */

      child               = &priv->hport[port];
      memset(child, 0, sizeof(struct usbhost_hubport_s));

      child->drvr         = hport->drvr;
      child->parent       = hport;
      child->port         = port;
      child->speed        = USB_SPEED_FULL;
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
 *
 * Returned Value:
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

  DEBUGASSERT(hubclass->hport);
  hport = hubclass->hport;

  DEBUGASSERT(configdesc != NULL && desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the endpoints */

  ret = usbhost_cfgdesc(hubclass, configdesc, desclen);
  if (ret < 0)
    {
      uerr("ERROR: Failed to parse config descriptor: %d\n", ret);
      return ret;
    }

  /* Read the hub descriptor */

  ret = usbhost_hubdesc(hubclass);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->nports > USBHUB_MAX_PORTS)
    {
      uerr("ERROR: too many downstream ports: %d\n", priv->nports);
      return -ENOSYS;
    }

  /* Enable power to all downstream ports */

  ret = usbhost_hubpwr(priv, hport, true);
  if (ret < 0)
    {
      uerr("ERROR: usbhost_hubpwr failed: %d\n", ret);
      return ret;
    }

  /* Begin monitoring of port status change events */

  ret = DRVR_ASYNCH(hport->drvr, priv->intin, (FAR uint8_t *)priv->buffer,
                    INTIN_BUFSIZE, usbhost_callback, hubclass);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_ASYNCH failed: %d\n", ret);
      (void)usbhost_hubpwr(priv, hport, false);
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
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   Probably called from an interrupt handler.
 *
 ****************************************************************************/

static int usbhost_disconnected(struct usbhost_class_s *hubclass)
{
  FAR struct usbhost_hubpriv_s *priv;
  irqstate_t flags;
  int ret;

  uinfo("Disconnected\n");

  /* Execute the disconnect action from the worker thread. */

  DEBUGASSERT(hubclass != NULL);
  priv = &((FAR struct usbhost_hubclass_s *)hubclass)->hubpriv;

  /* Mark the driver disconnected.  This will cause the callback to ignore
   * any subsequent completions of asynchronous transfers.
   */

  flags = enter_critical_section();
  priv->disconnected = true;

  /* Cancel any pending work. There may be pending HUB work associated with
   * hub interrupt pipe events.  That work may be lost by this action.
   */

  (void)work_cancel(LPWORK, &priv->work);

  /* Schedule the disconnection work */

  ret = work_queue(LPWORK, &priv->work,
                   (worker_t)usbhost_disconnect_event, hubclass, 0);
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_hub_initialize
 *
 * Description:
 *   Initialize the USB hub class.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host storage class.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_hub_initialize(void)
{
  /* Advertise our availability to support (certain) mass storage devices */

  return usbhost_registerclass(&g_hub);
}

#endif  /* CONFIG_USBHOST_HUB */

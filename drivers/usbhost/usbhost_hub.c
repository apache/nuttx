/****************************************************************************
 * drivers/usbhost/usbhost_hub.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Kaushal Parikh <kaushal@dspworks.in>
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

 /* TODO:
  *   - For EHCI controller, enable use of companion controllers
  *     or transaction translator with root hub. Current assumption
  *     is that device connected on EHCI root hub is a high-speed
  *     device.
  */

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
#define USBHOST_ALLFOUND    (USBHOST_IFFOUND|USBHOST_EPINFOUND)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure contains the internal, private state of the USB host
 * hub class.
 */

struct usbhost_hub_s
{
  volatile bool             disconnected; /* TRUE: Device has been disconnected */
  uint8_t                   ifno;         /* Interface number */  
  int16_t                   crefs;        /* Reference count on the driver instance */
  sem_t                     exclsem;      /* Used to maintain mutual exclusive access */

  uint8_t                   nports;       /* Number of ports */
  uint8_t                   lpsm;         /* Logical power switching mode */
  bool                      compounddev;  /* Hub is part of compound device */
  uint8_t                   ocmode;       /* Over current protection mode */
  bool                      indicator;    /* Port indicator */
  uint16_t                  pwrondelay;   /* Power on wait time in ms */
  uint8_t                   ctrlcurrent;  /* Control current */

  struct usb_hubtt_s        tt;           /* Transaction translator */
  struct usbhost_transfer_s intxfer;      /* Interrupt IN endpoint */

  struct usbhost_class_s    *childclass[USBHUB_MAX_PORTS];
                                          /* Pointer to child devices */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphores */

static void usbhost_takesem(sem_t *sem);
#define usbhost_givesem(s) sem_post(s);

/* Memory allocation services */

static inline uint8_t usbhost_allocaddr(void);
static inline void usbhost_freeaddr(uint8_t addr);
static inline FAR struct usbhost_class_s *
  usbhost_allocclass(FAR struct usbhost_driver_s *drvr,
             FAR struct usbhost_class_s *hclass, uint8_t speed,
             uint8_t port);
static inline void usbhost_freeclass(FAR struct usbhost_class_s *devclass);

/* Worker thread actions */

static void usbhost_destroy(FAR void *arg);

/* Helpers for usbhost_connect() */

static inline int usbhost_cfgdesc(FAR struct usbhost_class_s *hclass,
             FAR const uint8_t *configdesc, int desclen);
static inline int usbhost_hubdesc(FAR struct usbhost_class_s *hclass);
static inline int usbhost_hubpwr(FAR struct usbhost_class_s *hclass,
             bool on);
static void usbhost_hubevent(FAR struct usbhost_transfer_s *xfer);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(const uint8_t *val);
static void usbhost_putle16(uint8_t *dest, uint16_t val);
static void usbhost_callback(FAR struct usbhost_transfer_s *xfer);

/* struct usbhost_registry_s methods */
 
static int usbhost_create(FAR struct usbhost_class_s *hclass,
             FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_connect(FAR struct usbhost_class_s *hclass,
             FAR const uint8_t *configdesc, int desclen);
static int usbhost_disconnected(FAR struct usbhost_class_s *hclass);

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

/* Each bit indicates if corresponding bit number
 * is allocated as usb address
 */

static uint32_t g_addrmap[4];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static void usbhost_takesem(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: usbhost_allocaddr
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static inline uint8_t usbhost_allocaddr(void)
{
  uint8_t addr;

  for (addr = 0; addr < 128; addr++)
    {
      if (!(g_addrmap[addr/32] & (0x1 << (addr % 32))))
        {
          break;
        }
    }

  return (addr + 1);
}

/****************************************************************************
 * Name: usbhost_freecaddr
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static inline void usbhost_freeaddr(uint8_t addr)
{
  addr--;
  g_addrmap[addr/32] &= (~(0x1 << (addr % 32)));
}

/****************************************************************************
 * Name: usbhost_allocclass
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

static inline FAR struct usbhost_class_s *
  usbhost_allocclass(FAR struct usbhost_driver_s *drvr,
                     FAR struct usbhost_class_s *hclass,
                     uint8_t speed, uint8_t port)
{
  FAR struct usbhost_hub_s *hpriv;
  FAR struct usbhost_class_s *devclass;

  DEBUGASSERT(hclass != NULL && hclass->priv != NULL);
  hpriv = (FAR struct usbhost_hub_s *)hclass->priv;

  /* We are not executing from an interrupt handler so we can just call
   * kmalloc() to get memory for the class instance.
   */

  DEBUGASSERT(!up_interrupt_context());
  devclass = (FAR struct usbhost_class_s *)
    kmalloc(sizeof(struct usbhost_class_s));

  uvdbg("Allocated: %p\n", devclass);

  if (devclass != NULL)
    {
      struct usbhost_epdesc_s epdesc;
      int ret;
      
      devclass->addr   = usbhost_allocaddr();
      devclass->speed  = speed;
      devclass->drvr   = drvr;

      devclass->parent = hclass;
      devclass->priv   = NULL;

      devclass->tt     = NULL;
      devclass->ttport = 0;

      if (!ROOTHUB(devclass))
        {
          if (hclass->tt != NULL)
            {
              devclass->tt     = hclass->tt;
              devclass->ttport = hclass->ttport;
            }
          else if ((devclass->speed != USB_SPEED_HIGH) &&
                   (hclass->speed == USB_SPEED_HIGH))
            {
              devclass->tt     = &hpriv->tt;
              devclass->ttport = port;
            }
        }

      epdesc.devclass     = devclass;
      epdesc.addr         = 0;
      epdesc.in           = 0;
      epdesc.xfrtype      = USB_EP_ATTR_XFER_CONTROL;
      epdesc.interval     = 0;
      epdesc.mxpacketsize = 8;

      ret = DRVR_EPALLOC(devclass->drvr, &epdesc, &devclass->ep0);
      if (ret != OK)
        {
          udbg("failed to allocate ep0\n");
          usbhost_freeclass(devclass);
          devclass = NULL;
        }
    }

  return devclass;
}

/****************************************************************************
 * Name: usbhost_freeclass
 *
 * Description:
 *   Free a class instance previously allocated by usbhost_allocclass().
 *
 * Input Parameters:
 *   class - A reference to the class instance to be freed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline void usbhost_freeclass(FAR struct usbhost_class_s *devclass)
{
  DEBUGASSERT(devclass != NULL);

  if (devclass->ep0 != NULL)
    {
      DRVR_EPFREE(devclass->drvr, devclass->ep0);
      devclass->ep0 = NULL;
    }

  usbhost_freeaddr(devclass->addr);

  /* Free the class instance (calling sched_free() in case we are executing
   * from an interrupt handler.
   */

  uvdbg("Freeing: %p\n", devclass);
  kfree(devclass);
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
  FAR struct usbhost_class_s *hclass = (FAR struct usbhost_class_s *)arg;
  FAR struct usbhost_hub_s *hpriv;
  int i;

  DEBUGASSERT(hclass != NULL);
  
  hpriv = (FAR struct usbhost_hub_s *)hclass->priv;
  if (hpriv != NULL)
    {
      uvdbg("crefs: %d\n", hpriv->crefs);
    
      if (hpriv->intxfer.ep)
        {
          DRVR_EPFREE(hclass->drvr, hpriv->intxfer.ep);
        }
    
      /* Destroy the semaphores */
    
      sem_destroy(&hpriv->exclsem);
    
      /* Destroy allocated child classes */
    
      for (i = 0; i < USBHUB_MAX_PORTS; i++)
        {
          if (hpriv->childclass[i] != NULL)
            {
              CLASS_DISCONNECTED(hpriv->childclass[i]);
            }
        }
    
      /* Clear priv class */
    
      kfree(hclass->priv);
    }

  usbhost_freeclass(hclass);
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
 *   priv - The USB host class instance.
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

static inline int usbhost_cfgdesc(FAR struct usbhost_class_s *hclass,
                                  FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_hub_s *hpriv;
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  FAR struct usbhost_epdesc_s intindesc;
  int remaining;
  uint8_t found = 0;
  int ret;

  DEBUGASSERT(hclass != NULL && hclass->priv != NULL);
  hpriv = (FAR struct usbhost_hub_s *)hclass->priv;

  DEBUGASSERT(configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));
  
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

  /* Loop where there are more dscriptors to examine */

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

            hpriv->ifno = ifdesc->ifno;
            found       = USBHOST_IFFOUND;
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

                    /* Save the bulk IN endpoint information */

                    intindesc.devclass     = hclass;
                    intindesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    intindesc.in           = 1;
                    intindesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    intindesc.interval     = epdesc->interval;
                    intindesc.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);

                    uvdbg("Bulk IN EP addr:%d mxpacketsize:%d\n",
                          intindesc.addr, intindesc.mxpacketsize);
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
      ulldbg("ERROR: Found IF:%s BIN:%s BOUT:%s\n",
             (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
             (found & USBHOST_EPINFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the endpoints */

  ret = DRVR_EPALLOC(hclass->drvr, &intindesc, &hpriv->intxfer.ep);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate Interrupt IN endpoint\n");
      (void)DRVR_EPFREE(hclass->drvr, hpriv->intxfer.ep);
      return ret;
    }

  ullvdbg("Endpoints allocated\n");
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
 *   hclass - The USB host class instance.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static inline int usbhost_hubdesc(FAR struct usbhost_class_s *hclass)
{
  FAR struct usbhost_hub_s *hpriv;
  struct usb_hubdesc_s hubdesc;
  uint16_t hubchar;  
  int ret;

  DEBUGASSERT(hclass != NULL && hclass->priv != NULL);
  hpriv = (FAR struct usbhost_hub_s *)hclass->priv;

  ret = usbhost_ctrlxfer(hclass, (USB_REQ_DIR_IN | USBHUB_REQ_TYPE_HUB),
                         USB_REQ_GETDESCRIPTOR, USB_DESC_TYPE_HUB,
                         0, USB_SIZEOF_HUBDESC, (uint8_t *)&hubdesc);
  if (ret != OK)
    {
      udbg("failed to read hub descriptor\n");
      return ret;
    }

  hpriv->nports      = hubdesc.nports;
  
  hubchar            = usbhost_getle16(hubdesc.characteristics);
  hpriv->lpsm        = (hubchar & USBHUB_CHAR_LPSM_MASK) >> USBHUB_CHAR_LPSM_SHIFT;
  hpriv->compounddev = (hubchar & USBHUB_CHAR_COMPOUND) ? true : false;
  hpriv->ocmode      = (hubchar & USBHUB_CHAR_OCPM_MASK) >> USBHUB_CHAR_OCPM_SHIFT;
  /* priv->ttthinktime = (((hubchar & USBHUB_CHAR_TTTT_MASK) >> USBHUB_CHAR_TTTT_SHIFT) + 1) * 666; */
                              /* 8 FS bit times == (8 bits / 12000000 bps) ~= 666ns */
  hpriv->indicator   = (hubchar & USBHUB_CHAR_PORTIND) ? true : false;

  hpriv->pwrondelay  = (2 * hubdesc.pwrondelay);
  hpriv->ctrlcurrent = hubdesc.ctrlcurrent;
      
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
 *   hclass - The USB host class instance.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static inline int usbhost_hubpwr(FAR struct usbhost_class_s *hclass, bool on)
{
  FAR struct usbhost_hub_s *hpriv;

  DEBUGASSERT(hclass != NULL && hclass->priv != NULL);
  hpriv = (FAR struct usbhost_hub_s *)hclass->priv;

  if (on || ROOTHUB(hclass))
    {
      uint16_t req;
      int port, ret;

      if (on)
        {
          req = USB_REQ_SETFEATURE;
        }
      else
        {
          req = USB_REQ_CLEARFEATURE;
        }
    
      for (port = 1; port <= hpriv->nports; port++)
        {
          ret = usbhost_ctrlxfer(hclass, USBHUB_REQ_TYPE_PORT,
                                 req, USBHUB_PORT_FEAT_POWER,
                                 port, 0, NULL);
          if (ret != OK)
            {
              udbg("failed to power %d port %d\n", on, port);
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
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
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

static void usbhost_hubevent(FAR struct usbhost_transfer_s *xfer)
{
  FAR struct usbhost_class_s *hclass;
  FAR struct usbhost_hub_s *hpriv;
  struct usb_portstatus_s portstatus;
  uint16_t status;
  uint16_t change;
  uint16_t mask;
  uint16_t feat;
  uint8_t statusmap;
  int port;
  int ret;

  DEBUGASSERT(xfer != NULL && xfer->devclass != NULL);
  hclass = (FAR struct usbhost_class_s *)xfer->devclass;

  DEBUGASSERT(hclass != NULL && hclass->priv != NULL);
  hpriv = (FAR struct usbhost_hub_s *)hclass->priv;

  statusmap = xfer->buffer[0];

  for (port = 1; port <= hpriv->nports; port++)
    {
      /* Check if port status has changed */

      if (!(statusmap & (0x1 << port)))
        {
          continue;
        }

      /* Port status changed, check what happened */

      statusmap &= (~(0x1 << port));
      
      /* Read hub port status */

      ret = usbhost_ctrlxfer(hclass,
                             (USB_REQ_DIR_IN | USBHUB_REQ_TYPE_PORT),
                             USB_REQ_GETSTATUS, 0, port,
                             USB_SIZEOF_PORTSTS, (uint8_t *)&portstatus);
      if (ret != OK)
        {
          udbg("failed to read port %d status\n", port);
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
              ret = usbhost_ctrlxfer(hclass, USBHUB_REQ_TYPE_PORT,
                                     USB_REQ_CLEARFEATURE, feat,
                                     port, 0, NULL);
              if (ret != OK)
                {
                  udbg("failed to clear port %d change mask %x\n", port, mask);
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

          udbg("port %d status %x change %x\n", port, status, change);

          /* Debounce */

          while (debouncetime < 1500)
            {
               ret = usbhost_ctrlxfer(hclass,
                                      (USB_REQ_DIR_IN | USBHUB_REQ_TYPE_PORT),
                                      USB_REQ_GETSTATUS, 0, port,
                                      USB_SIZEOF_PORTSTS, (uint8_t *)&portstatus);
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
                    (void)usbhost_ctrlxfer(hclass, USBHUB_REQ_TYPE_PORT,
                                           USB_REQ_CLEARFEATURE, USBHUB_PORT_FEAT_CCONNECTION,
                                           port, 0, NULL);
                  }              

              debouncetime += 25;
              up_mdelay(25);
            }

          if ((ret != OK) || (debouncetime >= 1500))
            {
              udbg("failed to debounce port %d\n", port);
              continue;
            }

          if (status & USBHUB_PORT_STAT_CONNECTION)
            {
              /* Connect */
              
              ret = usbhost_ctrlxfer(hclass, USBHUB_REQ_TYPE_PORT,
                                     USB_REQ_SETFEATURE,
                                     USBHUB_PORT_FEAT_RESET, port, 0, NULL);
              if (ret != OK)
                {
                  udbg("failed to reset port %d\n", port);
                  continue;
                }

              up_mdelay(100);

              ret = usbhost_ctrlxfer(hclass,
                                     (USB_REQ_DIR_IN | USBHUB_REQ_TYPE_PORT),
                                     USB_REQ_GETSTATUS, 0, port,
                                     USB_SIZEOF_PORTSTS, (uint8_t *)&portstatus);
              if (ret != OK)
                {
                  udbg("failed to reset port %d\n", port);
                  continue;
                }

              status = usbhost_getle16(portstatus.status);
              change = usbhost_getle16(portstatus.change);

              udbg("port %d status %x change %x after reset\n",
                   port, status, change);

              if (!(status & USBHUB_PORT_STAT_RESET) &&
                   (status & USBHUB_PORT_STAT_ENABLE))
                {
                  uint8_t speed;
                  
                  if (change & USBHUB_PORT_STAT_CRESET)
                    {
                      (void)usbhost_ctrlxfer(hclass, USBHUB_REQ_TYPE_PORT,
                                             USB_REQ_CLEARFEATURE,
                                             USBHUB_PORT_FEAT_CRESET,
                                             port, 0, NULL);
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

                  /* Allocate new class and enumerate */

                  hpriv->childclass[port] =
                    usbhost_allocclass(hclass->drvr, hclass, speed, port);

                  if (hpriv->childclass[port] != NULL)
                    {
                      udbg("enumerate port %d speed %d\n", port, speed);

#if 0
                      ret = usbhost_enumerate(hpriv->childclass[port]);
                      if (ret != OK)
                        {
                          udbg("failed to enumerate port %d\n", port);
                        }
#endif
                    }

                }
              else
                {
                  udbg("failed to enable port %d\n", port);
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
          udbg("status %x change %x not handled\n", status, change);
        }
    }

  if (statusmap & 0x1)
    {
      /* Hub status changed */

      udbg("Hub status changed, not handled\n");
    }

  xfer->status = -EIO;

  ret = usbhost_intxfer(hclass, xfer, usbhost_callback);
  if (ret != OK)
    {
      udbg("failed to queue interrupt endpoint\n");
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

static void usbhost_callback(FAR struct usbhost_transfer_s *xfer)
{
  if (xfer->status != OK)
    {
      xfer->buffer[0] = 0;
    }

  (void)work_queue(HPWORK, &xfer->work, (worker_t)usbhost_hubevent, xfer, 0);
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
 *   drvr - An instance of struct usbhost_driver_s that the class
 *     implementation will "bind" to its state structure and will
 *     subsequently use to communicate with the USB host driver.
 *   id - In the case where the device supports multiple base classes,
 *     subclasses, or protocols, this specifies which to configure for.
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s that can be used by the USB host driver to communicate
 *   with the USB host class.  NULL is returned on failure; this function
 *   will fail only if the drvr input parameter is NULL or if there are
 *   insufficient resources to create another USB host class instance.
 *
 ****************************************************************************/

static int usbhost_create(FAR struct usbhost_class_s *hclass,
                          FAR const struct usbhost_id_s *id)
{
  struct usbhost_hub_s *hpriv;
  int child;
  int ret = -ENOMEM;

  /* Allocate a USB host class instance */

  hpriv = kmalloc(sizeof(struct usbhost_hub_s));

  if (hpriv != NULL)
    {
      /* Initialize the allocated storage class instance */

      memset(hpriv, 0, sizeof(struct usbhost_hub_s));

      /* Initialize class method function pointers */

      hclass->connect      = usbhost_connect;
      hclass->disconnected = usbhost_disconnected;
      hclass->priv         = hpriv;

      /* The initial reference count is 1... One reference is held by the driver */

      hpriv->crefs         = 1;

      /* Initialize semaphores (this works okay in the interrupt context) */

      sem_init(&hpriv->exclsem, 0, 1);

      /* Initialize interrupt end-point */

      hpriv->intxfer.ep   = NULL;

      /* Initialize transaction translator */

      hpriv->tt.class     = NULL;

      /* Initialize child class pointers */

      for (child = 0; child < USBHUB_MAX_PORTS; child++)
        {
          hpriv->childclass[child] = NULL;
        }

      ret = OK;
    }

  return ret;
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

static int usbhost_connect(FAR struct usbhost_class_s *hclass,
                           FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_hub_s *hpriv;
  int ret;

  DEBUGASSERT(hclass != NULL && hclass->priv != NULL);
  hpriv = (FAR struct usbhost_hub_s *)hclass->priv;

  DEBUGASSERT(configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the endpoints */

  ret = usbhost_cfgdesc(hclass, configdesc, desclen);
  if (ret != OK)
    {
      udbg("failed to parse config descriptor\n");
    }
  else
    {
      /* Allocate buffer for status change (INT) endpoint */

      ret = DRVR_IOALLOC(hclass->drvr, &hpriv->intxfer.buffer, 1);
      if (ret != OK)
        {
          return ret;
        }

      /* Now read hub descriptor */
      
      ret = usbhost_hubdesc(hclass);
      if (ret != OK)
        {
          return ret;
        }

      /* Power on hub (i.e. power on all hub ports) */

      ret = usbhost_hubpwr(hclass, true);
      if (ret != OK)
        {
          return ret;
        }

      /* INT request to periodically check port status */

      ret = usbhost_intxfer(hclass, &hpriv->intxfer,
                            usbhost_callback);
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

static int usbhost_disconnected(struct usbhost_class_s *hclass)
{
  FAR struct usbhost_hub_s *hpriv;
  irqstate_t flags;

  DEBUGASSERT(hclass != NULL && hclass->priv != NULL);
  hpriv = (FAR struct usbhost_hub_s *)hclass->priv;

  /* Set an indication to any users of the device that the device is no
   * longer available.
   */

  flags               = irqsave();
  hpriv->disconnected = true;

  /* Now check the number of references on the class instance.  If it is one,
   * then we can free the class instance now.  Otherwise, we will have to
   * wait until the holders of the references free them by closing the
   * block driver.
   */

  ullvdbg("crefs: %d\n", hpriv->crefs);
  if (hpriv->crefs == 1)
    {
      /* Free buffer for status change (INT) endpoint */

      DRVR_IOFREE(hclass->drvr, hpriv->intxfer.buffer);
      
      /* Power off (for root hub only) */
      (void)usbhost_hubpwr(hclass, false);
      
      /* Destroy the class instance */
          
      usbhost_destroy(hclass);
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

/*******************************************************************************
 * Name: usbhost_rh_connect
 *
 * Description:
 *   Connects USB host root hub
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call
 *      to the class create() method.
 *
 * Returned Value:
 *
 * Assumptions:
 *
 *******************************************************************************/

int usbhost_rh_connect(FAR struct usbhost_driver_s *drvr)
{
  struct usbhost_class_s *hclass = NULL;
  int ret = -ENOMEM;

  memset(g_addrmap, 0, 4*4);

  hclass = usbhost_allocclass(drvr, NULL, drvr->speed, 1);
  if (hclass != NULL)
    {
      ret = usbhost_enumerate(hclass);
      if (ret != OK)
        {
          udbg("failed to enumerate root hub\n");
        }
      else
        {
          drvr->roothub = hclass;
          udbg("Total class memory %d+%d\n", sizeof(struct usbhost_class_s),
                                             sizeof(struct usbhost_hub_s));
        }
    }

  return ret;
}

/*******************************************************************************
 * Name: usbhost_rh_disconnect
 *
 * Description:
 *   Disconnects USB host root hub
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call
 *      to the class create() method.
 *
 * Returned Value:
 *
 * Assumptions:
 *
 *******************************************************************************/

int usbhost_rh_disconnect(FAR struct usbhost_driver_s *drvr)
{
  FAR struct usbhost_class_s *hclass = drvr->roothub;

  if (hclass != NULL)
    {
      CLASS_DISCONNECTED(hclass);
      
      drvr->roothub = NULL;
    }

  return OK;
}

#endif  /* CONFIG_USBHOST_HUB */

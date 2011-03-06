/****************************************************************************
 * drivers/usbhost/usbhost_rtl8187.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Authors: Rafael Noronha <rafael@pdsolucoes.com.br>
 *            Gregory Nutt <spudmonkey@racsa.co.cr>
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

/* @TODO TEMPORARILY. REMOVE LATER!!! */
#define CONFIG_WLAN_IRQ 1

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

#include <nuttx/fs.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>

#if defined(CONFIG_NET) && defined(CONFIG_NET_WLAN)

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <wdog.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <net/uip/uip.h>
#include <net/uip/uip-arp.h>
#include <net/uip/uip-arch.h>

#endif /* CONFIG_NET && CONFIG_NET_WLAN */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* Driver support ***********************************************************/
/* This format is used to construct the /dev/skel[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "/dev/wlan%c"
#define DEV_NAMELEN         12


/* Used in usbhost_cfgdesc() */

#define USBHOST_IFFOUND     0x01
#define USBHOST_BINFOUND    0x02
#define USBHOST_BOUTFOUND   0x04
#define USBHOST_ALLFOUND    0x07

#define USBHOST_MAX_CREFS   0x7fff

#if defined(CONFIG_NET) && defined(CONFIG_NET_WLAN)
/* CONFIG_WLAN_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_WLAN_NINTERFACES
# define CONFIG_WLAN_NINTERFACES 1
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define WLAN_WDDELAY   (1*CLK_TCK)
#define WLAN_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define WLAN_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of the WLAN header */

#define BUF ((struct uip_eth_hdr *)wlan->wl_dev.d_buf)

#endif /* CONFIG_NET && CONFIG_NET_WLAN */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure contains the internal, private state of the USB host class
 * driver.
 */

struct usbhost_state_s
{
  /* This is the externally visible portion of the state */

  struct usbhost_class_s  class;

  /* This is an instance of the USB host driver bound to this class instance */

  struct usbhost_driver_s *drvr;

  /* The remainder of the fields are provide to the class driver */
  
  char                    devchar;      /* Character identifying the /dev/wlan[n] device */
  volatile bool           disconnected; /* TRUE: Device has been disconnected */
  uint8_t                 ifno;         /* Interface number */
  int16_t                 crefs;        /* Reference count on the driver instance */
  sem_t                   exclsem;      /* Used to maintain mutual exclusive access */
  struct work_s           work;         /* For interacting with the worker thread */
  FAR uint8_t            *tbuffer;      /* The allocated transfer buffer */
  size_t                  tbuflen;      /* Size of the allocated transfer buffer */
  usbhost_ep_t            epin;         /* IN endpoint */
  usbhost_ep_t            epout;        /* OUT endpoint */
};

#if defined(CONFIG_NET) && defined(CONFIG_NET_WLAN)

 /* The wlan_driver_s encapsulates all state information for a single hardware
  * interface
  */

 struct wlan_driver_s
 {
   bool wl_bifup;               /* true:ifup false:ifdown */
   WDOG_ID wl_txpoll;           /* TX poll timer */
   WDOG_ID wl_txtimeout;        /* TX timeout timer */

   /* This holds the information visible to uIP/NuttX */

   struct uip_driver_s wl_dev;  /* Interface understood by uIP */
 };

#endif /* CONFIG_NET && CONFIG_NET_WLAN */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphores */

static void usbhost_takesem(sem_t *sem);
#define usbhost_givesem(s) sem_post(s);

/* Memory allocation services */

static inline FAR struct usbhost_state_s *usbhost_allocclass(void);
static inline void usbhost_freeclass(FAR struct usbhost_state_s *class);

/* Device name management */

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv);
static void usbhost_freedevno(FAR struct usbhost_state_s *priv);
static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv, char *devname);

/* Worker thread actions */

static void usbhost_destroy(FAR void *arg);

/* Helpers for usbhost_connect() */

static inline int usbhost_cfgdesc(FAR struct usbhost_state_s *priv,
                                  FAR const uint8_t *configdesc, int desclen,
                                  uint8_t funcaddr);
static inline int usbhost_devinit(FAR struct usbhost_state_s *priv);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(const uint8_t *val);
static inline void usbhost_putle16(uint8_t *dest, uint16_t val);
static inline uint32_t usbhost_getle32(const uint8_t *val);
static void usbhost_putle32(uint8_t *dest, uint32_t val);

/* Transfer descriptor memory management */

static inline int usbhost_talloc(FAR struct usbhost_state_s *priv);
static inline int usbhost_tfree(FAR struct usbhost_state_s *priv);

/* struct usbhost_registry_s methods */
 
static struct usbhost_class_s *usbhost_create(FAR struct usbhost_driver_s *drvr,
                                              FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr);
static int usbhost_disconnected(FAR struct usbhost_class_s *class);

/* Driver methods -- depend upon the type of NuttX driver interface exported */

#if defined(CONFIG_NET) && defined(CONFIG_NET_WLAN)

/* Common TX logic */

static int  wlan_transmit(FAR struct wlan_driver_s *wlan);
static int  wlan_uiptxpoll(struct uip_driver_s *dev);

/* Interrupt handling */

static void wlan_receive(FAR struct wlan_driver_s *wlan);
static void wlan_txdone(FAR struct wlan_driver_s *wlan);
static int  wlan_interrupt(int irq, FAR void *context);

/* Watchdog timer expirations */

static void wlan_polltimer(int argc, uint32_t arg, ...);
static void wlan_txtimeout(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int wlan_ifup(struct uip_driver_s *dev);
static int wlan_ifdown(struct uip_driver_s *dev);
static int wlan_txavail(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int wlan_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
static int wlan_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
#endif

#endif /* CONFIG_NET && CONFIG_NET_WLAN */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID informatino that will  be 
 * used to associate the USB class driver to a connected USB device.
 */

static const const struct usbhost_id_s g_id =
{
  USB_CLASS_VENDOR_SPEC,  /* base */
  0xff,    /* subclass */
  0xff,    /* proto  */
  0x0bda,  /* vid      */
  0x8189   /* pid      */
};

/* This is the USB host wireless LAN class's registry entry */

static struct usbhost_registry_s g_wlan =
{
  NULL,                   /* flink    */
  usbhost_create,         /* create   */
  1,                      /* nids     */
  &g_id                   /* id[]     */
};

/* This is a bitmap that is used to allocate device names /dev/wlana-z. */

static uint32_t g_devinuse;

#if defined(CONFIG_NET) && defined(CONFIG_NET_WLAN)

static struct wlan_driver_s g_wlan1[CONFIG_WLAN_NINTERFACES];

#endif /* CONFIG_NET && CONFIG_NET_WLAN */

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

static inline FAR struct usbhost_state_s *usbhost_allocclass(void)
{
  FAR struct usbhost_state_s *priv;

  DEBUGASSERT(!up_interrupt_context());
  priv = (FAR struct usbhost_state_s *)malloc(sizeof(struct usbhost_state_s));
  uvdbg("Allocated: %p\n", priv);;
  return priv;
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

static inline void usbhost_freeclass(FAR struct usbhost_state_s *class)
{
  DEBUGASSERT(class != NULL);

  /* Free the class instance (calling sched_free() in case we are executing
   * from an interrupt handler.
   */

  uvdbg("Freeing: %p\n", class);;
  free(class);
}

/****************************************************************************
 * Name: Device name management
 *
 * Description:
 *   Some tiny functions to coordinate management of device names.
 *
 ****************************************************************************/

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv)
{
  irqstate_t flags;
  int devno;

  flags = irqsave();
  for (devno = 0; devno < 26; devno++)
    {
      uint32_t bitno = 1 << devno;
      if ((g_devinuse & bitno) == 0)
        {
          g_devinuse |= bitno;
          priv->devchar = 'a' + devno;
          irqrestore(flags);
          return OK;
        }
    }

  irqrestore(flags);
  return -EMFILE;
}

static void usbhost_freedevno(FAR struct usbhost_state_s *priv)
{
  int devno = 'a' - priv->devchar;

  if (devno >= 0 && devno < 26)
    {
      irqstate_t flags = irqsave();
      g_devinuse &= ~(1 << devno);
      irqrestore(flags);
    }
}

static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv, char *devname)
{
  (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->devchar);
}

/****************************************************************************
 * Name: usbhost_destroy
 *
 * Description:
 *   The USB device has been disconnected and the refernce count on the USB
 *   host class instance has gone to 1.. Time to destroy the USB host class
 *   instance.
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
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)arg;

  DEBUGASSERT(priv != NULL);
  uvdbg("crefs: %d\n", priv->crefs);
 
  /* Unregister the driver */

  /* Release the device name used by this connection */

  usbhost_freedevno(priv);

  /* Free the endpoints */

  /* Free any transfer buffers */

  /* Destroy the semaphores */

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(priv->drvr);

  /* And free the class instance.  Hmmm.. this may execute on the worker
   * thread and the work structure is part of what is getting freed.  That
   * should be okay because once the work contained is removed from the
   * queue, it should not longer be accessed by the worker thread.
   */

  usbhost_freeclass(priv);
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
 *   configdesc - A pointer to a uint8_t buffer container the configuration descripor.
 *   desclen - The length in bytes of the configuration descriptor.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static inline int usbhost_cfgdesc(FAR struct usbhost_state_s *priv,
                                  FAR const uint8_t *configdesc, int desclen,
                                  uint8_t funcaddr)
{
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  FAR struct usbhost_epdesc_s bindesc;
  FAR struct usbhost_epdesc_s boutdesc;
  int remaining;
  uint8_t found = 0;
  int ret;

  DEBUGASSERT(priv != NULL && 
              configdesc != NULL &&
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
            FAR struct usb_ifdesc_s *ifdesc = (FAR struct usb_ifdesc_s *)configdesc;
 
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
            FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)configdesc;

            uvdbg("Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for a bulk endpoint. */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_BULK)
              {
                /* Yes.. it is a bulk endpoint.  IN or OUT? */

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an OUT bulk endpoint.  There should be only one
                     * bulk OUT endpoint.
                     */

                    if ((found & USBHOST_BOUTFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }
                    found |= USBHOST_BOUTFOUND;

                    /* Save the bulk OUT endpoint information */

                    boutdesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    boutdesc.in           = false;
                    boutdesc.funcaddr     = funcaddr;
                    boutdesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    boutdesc.interval     = epdesc->interval;
                    boutdesc.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);
                    uvdbg("Bulk OUT EP addr:%d mxpacketsize:%d\n",
                          boutdesc.addr, boutdesc.mxpacketsize);
                  }
                else
                  {
                    /* It is an IN bulk endpoint.  There should be only one
                     * bulk IN endpoint.
                     */

                    if ((found & USBHOST_BINFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }
                    found |= USBHOST_BINFOUND;

                    /* Save the bulk IN endpoint information */
                    
                    bindesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    bindesc.in           = 1;
                    bindesc.funcaddr     = funcaddr;
                    bindesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    bindesc.interval     = epdesc->interval;
                    bindesc.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);
                    uvdbg("Bulk IN EP addr:%d mxpacketsize:%d\n",
                          bindesc.addr, bindesc.mxpacketsize);
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
             (found & USBHOST_BINFOUND) != 0 ? "YES" : "NO",
             (found & USBHOST_BOUTFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the endpoints */

  ret = DRVR_EPALLOC(priv->drvr, &boutdesc, &priv->epout);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate Bulk OUT endpoint\n");
      return ret;
    }

  ret = DRVR_EPALLOC(priv->drvr, &bindesc, &priv->epin);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate Bulk IN endpoint\n");
      (void)DRVR_EPFREE(priv->drvr, priv->epout);
      return ret;
    }

  ullvdbg("Endpoints allocated\n");
  return OK;
}

/****************************************************************************
 * Name: usbhost_devinit
 *
 * Description:
 *   The USB device has been successfully connected.  This completes the
 *   initialization operations.  It is first called after the
 *   configuration descriptor has been received.
 *
 *   This function is called from the connect() method.  This function always
 *   executes on the thread of the caller of connect().
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline int usbhost_devinit(FAR struct usbhost_state_s *priv)
{
  int ret = OK;

  /* Set aside a transfer buffer for exclusive use by the class driver */

  /* Increment the reference count.  This will prevent usbhost_destroy() from
   * being called asynchronously if the device is removed.
   */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);

  /* Configure the device */

  /* Register the driver */

  if (ret == OK)
    {
      char devname[DEV_NAMELEN];

      uvdbg("Register block driver\n");
      usbhost_mkdevname(priv, devname);
      // ret = register_blockdriver(devname, &g_bops, 0, priv);
    }

  /* Check if we successfully initialized. We now have to be concerned
   * about asynchronous modification of crefs because the block
   * driver has been registerd.
   */

  if (ret == OK)
    {
      usbhost_takesem(&priv->exclsem);
      DEBUGASSERT(priv->crefs >= 2);

      /* Handle a corner case where (1) open() has been called so the
       * reference count is > 2, but the device has been disconnected.
       * In this case, the class instance needs to persist until close()
       * is called.
       */

      if (priv->crefs <= 2 && priv->disconnected)
        {
          /* We don't have to give the semaphore because it will be
           * destroyed when usb_destroy is called.
           */
  
          ret = -ENODEV;
        }
      else
        {
          /* Ready for normal operation as a block device driver */

          uvdbg("Successfully initialized\n");
          priv->crefs--;
          usbhost_givesem(&priv->exclsem);
        }
    }

  /* Disconnect on any errors detected during volume initialization */

  if (ret != OK)
    {
      udbg("ERROR! Aborting: %d\n", ret);
      usbhost_destroy(priv);
    }

  return ret;
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
 * Name: usbhost_getle32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline uint32_t usbhost_getle32(const uint8_t *val)
{
 /* Little endian means LS halfword first in byte stream */

  return (uint32_t)usbhost_getle16(&val[2]) << 16 | (uint32_t)usbhost_getle16(val);
}

/****************************************************************************
 * Name: usbhost_putle32
 *
 * Description:
 *   Put a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_putle32(uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  usbhost_putle16(dest, (uint16_t)(val & 0xffff));
  usbhost_putle16(dest+2, (uint16_t)(val >> 16));
}

/****************************************************************************
 * Name: usbhost_talloc
 *
 * Description:
 *   Allocate transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_talloc(FAR struct usbhost_state_s *priv)
{
  DEBUGASSERT(priv && priv->tbuffer == NULL);
  return DRVR_ALLOC(priv->drvr, &priv->tbuffer, &priv->tbuflen);
}

/****************************************************************************
 * Name: usbhost_tfree
 *
 * Description:
 *   Free transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_tfree(FAR struct usbhost_state_s *priv)
{
  int result = OK;
  DEBUGASSERT(priv);

  if (priv->tbuffer)
    {
      DEBUGASSERT(priv->drvr);
      result         = DRVR_FREE(priv->drvr, priv->tbuffer);
      priv->tbuffer = NULL;
      priv->tbuflen = 0;
    }
  return result;
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

static FAR struct usbhost_class_s *usbhost_create(FAR struct usbhost_driver_s *drvr,
                                                  FAR const struct usbhost_id_s *id)
{
  FAR struct usbhost_state_s *priv;

  /* Allocate a USB host class instance */

  priv = usbhost_allocclass();
  if (priv)
    {
      /* Initialize the allocated storage class instance */

      memset(priv, 0, sizeof(struct usbhost_state_s));

      /* Assign a device number to this class instance */

      if (usbhost_allocdevno(priv) == OK)
        {
         /* Initialize class method function pointers */

          priv->class.connect      = usbhost_connect;
          priv->class.disconnected = usbhost_disconnected;

          /* The initial reference count is 1... One reference is held by the driver */

          priv->crefs              = 1;

          /* Initialize semphores (this works okay in the interrupt context) */

          sem_init(&priv->exclsem, 0, 1);

          /* Bind the driver to the storage class instance */

          priv->drvr               = drvr;

          /* Return the instance of the USB class driver */
 
          return &priv->class;
        }
    }

  /* An error occurred. Free the allocation and return NULL on all failures */

  if (priv)
    {
      usbhost_freeclass(priv);
    }
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
 *   class - The USB host class entry previously obtained from a call to create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration descripor.
 *   desclen - The length in bytes of the configuration descriptor.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - This function will *not* be called from an interrupt handler.
 *   - If this function returns an error, the USB host controller driver
 *     must call to DISCONNECTED method to recover from the error
 *
 ****************************************************************************/

static int usbhost_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)class;
  int ret;

  DEBUGASSERT(priv != NULL && 
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the endpoints */

  ret = usbhost_cfgdesc(priv, configdesc, desclen, funcaddr);
  if (ret != OK)
    {
      udbg("usbhost_cfgdesc() failed: %d\n", ret);
    }
  else
    {
      /* Now configure the device and register the NuttX driver */

      ret = usbhost_devinit(priv);
      if (ret != OK)
        {
          udbg("usbhost_devinit() failed: %d\n", ret);
        }
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
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

static int usbhost_disconnected(struct usbhost_class_s *class)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)class;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

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
      /* Destroy the class instance.  If we are executing from an interrupt
       * handler, then defer the destruction to the worker thread.
       * Otherwise, destroy the instance now.
       */

      if (up_interrupt_context())
        {
          /* Destroy the instance on the worker thread. */

          uvdbg("Queuing destruction: worker %p->%p\n", priv->work.worker, usbhost_destroy);
          DEBUGASSERT(priv->work.worker == NULL);
          (void)work_queue(&priv->work, usbhost_destroy, priv, 0);
       }
      else
        {
          /* Do the work now */

          usbhost_destroy(priv);
        }
    }

  irqrestore(flags);  
  return OK;
}

#if defined(CONFIG_NET) && defined(CONFIG_NET_WLAN)

/****************************************************************************
 * Function: wlan_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   wlan  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int wlan_transmit(FAR struct wlan_driver_s *wlan)
{
  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is not transmission in progress.
   */

  /* Increment statistics */

  /* Send the packet: address=wlan->wl_dev.d_buf, length=wlan->wl_dev.d_len */

  /* Enable Tx interrupts */

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(wlan->wl_txtimeout, WLAN_TXTIMEOUT, wlan_txtimeout, 1, (uint32_t)wlan);
  return OK;
}

/****************************************************************************
 * Function: wlan_uiptxpoll
 *
 * Description:
 *   The transmitter is available, check if uIP has any outgoing packets ready
 *   to send.  This is a callback from uip_poll().  uip_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
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
 ****************************************************************************/

static int wlan_uiptxpoll(struct uip_driver_s *dev)
{
  FAR struct wlan_driver_s *wlan = (FAR struct wlan_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (wlan->wl_dev.d_len > 0)
    {
      uip_arp_out(&wlan->wl_dev);
      wlan_transmit(wlan);

      /* Check if there is room in the device to hold another packet. If not,
       * return a non-zero value to terminate the poll.
       */
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: wlan_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   wlan  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void wlan_receive(FAR struct wlan_driver_s *wlan)
{
  do
    {
      /* Check for errors and update statistics */

      /* Check if the packet is a valid size for the uIP buffer configuration */

      /* Copy the data data from the hardware to wlan->wl_dev.d_buf.  Set
       * amount of data in wlan->wl_dev.d_len
       */

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
      if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
        {
          uip_arp_ipin(&wlan->wl_dev);
          uip_input(&wlan->wl_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (wlan->wl_dev.d_len > 0)
           {
             uip_arp_out(&wlan->wl_dev);
             wlan_transmit(wlan);
           }
        }
      else if (BUF->type == htons(UIP_ETHTYPE_ARP))
        {
          uip_arp_arpin(&wlan->wl_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (wlan->wl_dev.d_len > 0)
            {
              wlan_transmit(wlan);
            }
        }
    }
  while (0); /* While there are more packets to be processed */
}

/****************************************************************************
 * Function: wlan_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
 *   wlan  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void wlan_txdone(FAR struct wlan_driver_s *wlan)
{
  /* Check for errors and update statistics */

  /* If no further xmits are pending, then cancel the TX timeout and
   * disable further Tx interrupts.
   */

  wd_cancel(wlan->wl_txtimeout);

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&wlan->wl_dev, wlan_uiptxpoll);
}

/****************************************************************************
 * Function: wlan_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int wlan_interrupt(int irq, FAR void *context)
{
  register FAR struct wlan_driver_s *wlan = &g_wlan1[0];

  /* Get and clear interrupt status bits */

  /* Handle interrupts according to status bit settings */

  /* Check if we received an incoming packet, if so, call wlan_receive() */

  wlan_receive(wlan);

  /* Check is a packet transmission just completed.  If so, call wlan_txdone.
   * This may disable further Tx interrupts if there are no pending
   * tansmissions.
   */

  wlan_txdone(wlan);

  return OK;
}

/****************************************************************************
 * Function: wlan_txtimeout
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void wlan_txtimeout(int argc, uint32_t arg, ...)
{
  FAR struct wlan_driver_s *wlan = (FAR struct wlan_driver_s *)arg;

  /* Increment statistics and dump debug info */

  /* Then reset the hardware */

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&wlan->wl_dev, wlan_uiptxpoll);
}

/****************************************************************************
 * Function: wlan_polltimer
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void wlan_polltimer(int argc, uint32_t arg, ...)
{
  FAR struct wlan_driver_s *wlan = (FAR struct wlan_driver_s *)arg;

  /* Check if there is room in the send another TX packet.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  /* If so, update TCP timing states and poll uIP for new XMIT data. Hmmm..
   * might be bug here.  Does this mean if there is a transmit in progress,
   * we will missing TCP time state updates?
   */

  (void)uip_timer(&wlan->wl_dev, wlan_uiptxpoll, WLAN_POLLHSEC);

  /* Setup the watchdog poll timer again */

  (void)wd_start(wlan->wl_txpoll, WLAN_WDDELAY, wlan_polltimer, 1, arg);
}

/****************************************************************************
 * Function: wlan_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the WLAN interface when an IP address is
 *   provided
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int wlan_ifup(struct uip_driver_s *dev)
{
  FAR struct wlan_driver_s *wlan = (FAR struct wlan_driver_s *)dev->d_private;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Initialize PHYs, the WLAN interface, and setup up WLAN interrupts */

  /* Set and activate a timer process */

  (void)wd_start(wlan->wl_txpoll, WLAN_WDDELAY, wlan_polltimer, 1, (uint32_t)wlan);

  /* Enable the WLAN interrupt */

  wlan->wl_bifup = true;
  up_enable_irq(CONFIG_WLAN_IRQ);
  return OK;
}

/****************************************************************************
 * Function: wlan_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int wlan_ifdown(struct uip_driver_s *dev)
{
  FAR struct wlan_driver_s *wlan = (FAR struct wlan_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the WLAN interrupt */

  flags = irqsave();
  up_disable_irq(CONFIG_WLAN_IRQ);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(wlan->wl_txpoll);
  wd_cancel(wlan->wl_txtimeout);

  /* Put the the EMAC is its reset, non-operational state.  This should be
   * a known configuration that will guarantee the wlan_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  wlan->wl_bifup = false;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: wlan_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int wlan_txavail(struct uip_driver_s *dev)
{
  FAR struct wlan_driver_s *wlan = (FAR struct wlan_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable interrupts because this function may be called from interrupt
   * level processing.
   */

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (wlan->wl_bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */

      /* If so, then poll uIP for new XMIT data */

      (void)uip_poll(&wlan->wl_dev, wlan_uiptxpoll);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: wlan_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int wlan_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct wlan_driver_s *wlan = (FAR struct wlan_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: wlan_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int wlan_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct wlan_driver_s *wlan = (FAR struct wlan_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

#endif /* CONFIG_NET && CONFIG_NET_WLAN */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_wlaninit
 *
 * Description:
 *   Initialize the USB class driver.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host class device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_wlaninit(void)
{
  /* Perform any one-time initialization of the class implementation */

  /* Advertise our availability to support (certain) devices */

  return usbhost_registerclass(&g_wlan);
}

#if defined(CONFIG_NET) && defined(CONFIG_NET_WLAN)

/****************************************************************************
 * Function: wlan_initialize
 *
 * Description:
 *   Initialize the WLAN controller and driver
 *
 * Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int wlan_initialize(int intf)
{
  struct wlan_driver_s *priv;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(inf < CONFIG_WLAN_NINTERFACES);
  priv = &g_wlan1[intf];

  /* Check if a WLAN chip is recognized at its I/O base */

  /* Attach the IRQ to the driver */

  if (irq_attach(CONFIG_WLAN_IRQ, wlan_interrupt))
    {
      /* We could not attach the ISR to the the interrupt */

      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct wlan_driver_s));
  priv->wl_dev.d_ifup    = wlan_ifup;     /* I/F down callback */
  priv->wl_dev.d_ifdown  = wlan_ifdown;   /* I/F up (new IP address) callback */
  priv->wl_dev.d_txavail = wlan_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->wl_dev.d_addmac  = wlan_addmac;   /* Add multicast MAC address */
  priv->wl_dev.d_rmmac   = wlan_rmmac;    /* Remove multicast MAC address */
#endif
  priv->wl_dev.d_private = (void*)g_wlan1; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->wl_txpoll       = wd_create();   /* Create periodic poll timer */
  priv->wl_txtimeout    = wd_create();   /* Create TX timeout timer */

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling wlan_ifdown().
   */

  /* Read the MAC address from the hardware into priv->wl_dev.d_mac.ether_addr_octet */

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->wl_dev);
  return OK;
}

/****************************************************************************
 * Name: up_netinitialize
 *
 * Description:
 *   Initialize the first network interface.  If there are more than one
 *   interface in the device, then device-specific logic will have to provide
 *   this function to determine which, if any, WLAN controllers should
 *   be initialized.
 *
 ****************************************************************************/

void up_netinitialize(void)
{
  (void)wlan_initialize(0);
}
#endif /* CONFIG_NET && CONFIG_NET_WLAN */


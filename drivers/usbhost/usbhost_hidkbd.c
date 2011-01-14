/****************************************************************************
 * drivers/usbhost/usbhost_hidkbd.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <semaphore.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/hid.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Worker thread support is required */

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* This determines how often the USB keyboard will be polled in units of
 * of clock ticks.  The default is 100MS.
 */

#ifndef CONFIG_HIDKBD_POLLTICKS
  /* The value CLK_TCK gives the frequency in HZ of the system timer.  This
   * is, the number of ticks in one second.  So one tenth of this would give
   * is the number of ticks required for a 100MS delay between polls.
   */

#  define CONFIG_HIDKBD_POLLTICKS (CLK_TCK/10)
#endif

/* Driver support ***********************************************************/
/* This format is used to construct the /dev/sd[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "/dev/sd%c"
#define DEV_NAMELEN         10

/* Used in usbhost_cfgdesc() */

#define USBHOST_IFFOUND     0x01 /* Required I/F descriptor found */
#define USBHOST_EPINFOUND   0x02 /* Required interrupt IN EP descriptor found */
#define USBHOST_EPOUTFOUND  0x04 /* Optional interrupt OUT EP descriptor found */
#define USBHOST_RQDFOUND    (USBHOST_IFFOUND|USBHOST_EPINFOUND)

#define USBHOST_MAX_CREFS   0x7fff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure contains the internal, private state of the USB host mass
 * storage class.
 */

struct usbhost_state_s
{
  /* This is the externally visible portion of the state */

  struct usbhost_class_s  class;

  /* This is an instance of the USB host driver bound to this class instance */

  struct usbhost_driver_s *drvr;

  /* The remainder of the fields are provide o the mass storage class */
  
  char                    sdchar;       /* Character identifying the /dev/sd[n] device */
  volatile bool           disconnected; /* TRUE: Device has been disconnected */
  int16_t                 crefs;        /* Reference count on the driver instance */
  sem_t                   exclsem;      /* Used to maintain mutual exclusive access */
  struct work_s           work;         /* For interacting with the worker thread */
  FAR uint8_t            *tdbuffer;     /* The allocated transfer descriptor buffer */
  size_t                  tdbuflen;     /* Size of the allocated transfer buffer */

  /* Endpoints:
   * EP0 (Control):
   * - Receiving and responding to requests for USB control and class data.
   * - IN data when polled by the HID class driver (Get_Report)
   * - OUT data from the host.
   * EP Interrupt IN:
   * - Receiving asynchronous (unrequested) IN data from the device.
   * EP Interrrupt OUT (optional):
   * - Transmitting low latency OUT data to the device.
   * - If not present, EP0 used.
   */

  usbhost_ep_t            epin;         /* Interrupt IN endpoint */
  usbhost_ep_t            epout;        /* Optional interrupt OUT endpoint */
};

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

static void usbhost_kbdpoll(FAR void *arg);
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

static inline int usbhost_tdalloc(FAR struct usbhost_state_s *priv);
static inline int usbhost_tdfree(FAR struct usbhost_state_s *priv);

/* struct usbhost_registry_s methods */
 
static struct usbhost_class_s *usbhost_create(FAR struct usbhost_driver_s *drvr,
                                              FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr);
static int usbhost_disconnected(FAR struct usbhost_class_s *class);

/* Driver methods.  We export the keyboard as a standard character driver */

static ssize_t usbhost_read(FAR struct file *filp,
                            FAR char *buffer, size_t len);
static ssize_t usbhost_write(FAR struct file *filp,
                             FAR const char *buffer, size_t len);
#ifndef CONFIG_DISABLE_POLL
static int usbhost_poll(FAR struct file *filp, FAR struct pollfd *fds,
                        bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID informatino that will  be 
 * used to associate the USB host mass storage class to a connected USB
 * device.
 */

static const const struct usbhost_id_s g_id =
{
  USB_CLASS_HID,            /* base     */
  USBHID_SUBCLASS_BOOTIF,   /* subclass */
  USBHID_PROTOCOL_KEYBOARD, /* proto    */
  0,                        /* vid      */
  0                         /* pid      */
};

/* This is the USB host storage class's registry entry */

static struct usbhost_registry_s g_skeleton =
{
  NULL,                    /* flink     */
  usbhost_create,          /* create    */
  1,                       /* nids      */
  &g_id                    /* id[]      */
};

static const struct file_operations usbhost_fops =
{
  0,                       /* open      */
  0,                       /* close     */
  usbhost_read,            /* read      */
  usbhost_write,           /* write     */
  0,                       /* seek      */
  0                        /* ioctl     */
#ifndef CONFIG_DISABLE_POLL
  , usbhost_poll           /* poll      */
#endif
};

/* This is a bitmap that is used to allocate device names /dev/sda-z. */

static uint32_t g_devinuse;

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
          priv->sdchar = 'a' + devno;
          irqrestore(flags);
          return OK;
        }
    }

  irqrestore(flags);
  return -EMFILE;
}

static void usbhost_freedevno(FAR struct usbhost_state_s *priv)
{
  int devno = 'a' - priv->sdchar;

  if (devno >= 0 && devno < 26)
    {
      irqstate_t flags = irqsave();
      g_devinuse &= ~(1 << devno);
      irqrestore(flags);
    }
}

static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv, char *devname)
{
  (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->sdchar);
}

/****************************************************************************
 * Name: usbhost_kbdpoll
 *
 * Description:
 *   Periodically check for new keyboard data.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_kbdpoll(FAR void *arg)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)arg;
  irqstate_t flags;
#ifdef CONFIG_DEBUG_USB
  static unsigned int npolls = 0;
#endif
  int ret;

  /* Poll the keyboard */
#warning "Missing logic"

  /* Setup for the next poll.  We have to be careful here because the work
   * structure may be used by the interrupt handler if the USB device is
   * disconnected.
   */

  flags = irqsave();

  /* Is the device still connected?  If not, we do not reschedule any further
   * polling of the device.
   */

  if (priv->disconnected)
    {
      udbg("Keyboard removed, polling halted\n");
    }
  else
    {
      /* Otherwise, just setup the next poll. */
 
      ret = work_queue(&priv->work, usbhost_kbdpoll, priv, CONFIG_HIDKBD_POLLTICKS);
      if (ret != 0)
        {
          udbg("ERROR: Failed to re-schedule keyboard poll: %d\n", ret);
        }

      /* If USB debug is on, then provide some periodic indication that
       * polling is still happening.
       */

#ifdef CONFIG_DEBUG_USB
      npolls++;
#endif
    }
  irqrestore(flags);

  /* If USB debug is on, then provide some periodic indication that
   * polling is still happening.
   */

#ifdef CONFIG_DEBUG_USB
  if (!priv->disconnected && (npolls & ~31) == 0)
   {
      udbg("Still polling: %d\n", npolls);
   }
#endif
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
  char devname[DEV_NAMELEN];

  DEBUGASSERT(priv != NULL);
  uvdbg("crefs: %d\n", priv->crefs);
 
  /* Unregister the driver */

  uvdbg("Unregister driver\n");
  usbhost_mkdevname(priv, devname);
  (void)unregister_driver(devname);

  /* Release the device name used by this connection */

  usbhost_freedevno(priv);

  /* Free the interrupt endpoints */

  if (priv->epin)
    {
      DRVR_EPFREE(priv->drvr, priv->epin);
    }

  if (priv->epout)
    {
      DRVR_EPFREE(priv->drvr, priv->epout);
    }

  /* Free any transfer buffers */

  usbhost_tdfree(priv);

  /* Destroy the semaphores */

  sem_destroy(&priv->exclsem);

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
  FAR struct usbhost_epdesc_s epindesc;
  FAR struct usbhost_epdesc_s epoutdesc;
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
            uvdbg("Interface descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);
            if ((found & USBHOST_IFFOUND) != 0)
              {
                /* Oops.. more than one interface.  We don't know what to do with this. */

                return -ENOSYS;
              }
            found |= USBHOST_IFFOUND;
          }
          break;

        /* HID descriptor */

        case USBHID_DESCTYPE_HID:
            uvdbg("HID descriptor\n");
            break;

        /* Endpoint descriptor.  We expect one or two interrupt endpoints,
         * a required IN endpoint and an optional OUT endpoint.
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)configdesc;

            uvdbg("Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for an interrupt endpoint. */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_INT)
              {
                /* Yes.. it is a interrupt endpoint.  IN or OUT? */

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an interrupt OUT endpoint.  There not be more than one
                     * interrupt OUT endpoint.
                     */

                    if ((found & USBHOST_EPOUTFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know what to do with this. */

                        return -EINVAL;
                      }
                    found |= USBHOST_EPOUTFOUND;

                    /* Save the interrupt OUT endpoint information */

                    epoutdesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    epoutdesc.in           = false;
                    epoutdesc.funcaddr     = funcaddr;
                    epoutdesc.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);
                    uvdbg("Interrupt OUT EP addr:%d mxpacketsize:%d\n",
                          epoutdesc.addr, epoutdesc.mxpacketsize);
                  }
                else
                  {
                    /* It is an interrupt IN endpoint.  There should be only
                     * one interrupt IN endpoint.
                     */

                    if ((found & USBHOST_EPINFOUND) != 0)
                      {
                        /* Oops.. more than one endpint.  We don't know what
                         * to do with this.
                         */

                        return -EINVAL;
                      }
                    found |= USBHOST_EPINFOUND;

                    /* Save the interrupt IN endpoint information */
                    
                    epindesc.addr        = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    epindesc.in           = 1;
                    epindesc.funcaddr     = funcaddr;
                    epindesc.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);
                    uvdbg("Interrupt IN EP addr:%d mxpacketsize:%d\n",
                          epindesc.addr, epindesc.mxpacketsize);
                  }
              }
          }
          break;

        /* Other descriptors are just ignored for now */

        default:
          uvdbg("Other descriptor: %d\n", desc->type);
          break;
        }

      /* Increment the address of the next descriptor */
 
      configdesc += desc->len;
      remaining  -= desc->len;
    }

  /* Sanity checking... did we find all of things that we need? */
    
  if ((found & USBHOST_RQDFOUND) != USBHOST_RQDFOUND)
    {
      ulldbg("ERROR: Found IF:%s EPIN:%s\n",
             (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
             (found & USBHOST_EPINFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the endpoints.  First, the required interrupt
   * IN endpoint.
   */

  ret = DRVR_EPALLOC(priv->drvr, &epindesc, &priv->epin);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate interrupt IN endpoint\n");
      return ret;
    }

  /* Then the optional interrupt OUT endpoint */

  ullvdbg("Found EPOOUT:%s\n",
         (found & USBHOST_EPOUTFOUND) != 0 ? "YES" : "NO");

  if ((found & USBHOST_EPOUTFOUND) != 0)
    {
      ret = DRVR_EPALLOC(priv->drvr, &epoutdesc, &priv->epout);
      if (ret != OK)
        {
          udbg("ERROR: Failed to allocate interrupt OUT endpoint\n");
          (void)DRVR_EPFREE(priv->drvr, priv->epin);
          return ret;
        }
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
  int ret;

  /* Set aside a transfer buffer for exclusive use by the keyboard class driver */

  ret = usbhost_tdalloc(priv);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate transfer buffer\n");
      return ret;
    }

  /* Increment the reference count.  This will prevent usbhost_destroy() from
   * being called asynchronously if the device is removed.
   */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);

  /* Setup a period worker thread event to poll the USB device. */

  ret = work_queue(&priv->work, usbhost_kbdpoll, priv, CONFIG_HIDKBD_POLLTICKS);
  
  /* Register the driver */

  if (ret == OK)
    {
      char devname[DEV_NAMELEN];

      uvdbg("Register driver\n");
      usbhost_mkdevname(priv, devname);
      (void)register_driver(devname, &usbhost_fops, 0666, NULL);
    }

  /* Check if we successfully initialized. We now have to be concerned
   * about asynchronous modification of crefs because the driver has
   * been registerd.
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
          /* Ready for normal operation as a character device driver */

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
 * Name: usbhost_tdalloc
 *
 * Description:
 *   Allocate transfer descriptor memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_tdalloc(FAR struct usbhost_state_s *priv)
{
  DEBUGASSERT(priv && priv->tdbuffer == NULL);
  return DRVR_ALLOC(priv->drvr, &priv->tdbuffer, &priv->tdbuflen);
}

/****************************************************************************
 * Name: usbhost_tdfree
 *
 * Description:
 *   Free transfer descriptor memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_tdfree(FAR struct usbhost_state_s *priv)
{
  int result = OK;
  DEBUGASSERT(priv);

  if (priv->tdbuffer)
    {
      DEBUGASSERT(priv->drvr);
      result         = DRVR_FREE(priv->drvr, priv->tdbuffer);
      priv->tdbuffer = NULL;
      priv->tdbuflen = 0;
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

          /* NOTE: We do not yet know the geometry of the USB mass storage device */
 
          /* Return the instance of the USB mass storage class */
 
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
 *   This function will *not* be called from an interrupt handler.
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

  /* Set an indication to any users of the mass storage device that the device
   * is no longer available.
   */

  flags              = irqsave();
  priv->disconnected = true;

  /* Now check the number of references on the class instance.  If it is one,
   * then we can free the class instance now.  Otherwise, we will have to
   * wait until the holders of the references free them by closing the
   * driver.
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

          /* Cancel the period polling thread */

          (void)work_cancel(&priv->work);
          DEBUGASSERT(priv->work.worker == NULL);

          /* Then schedule the destruction */

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

/****************************************************************************
 * Character driver methods
 ****************************************************************************/
/****************************************************************************
 * Name: usbhost_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t usbhost_read(FAR struct file *filp, FAR char *buffer, size_t len)
{
  return 0; /* Return EOF for now */
}

/****************************************************************************
 * Name: usbhost_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t usbhost_write(FAR struct file *filp, FAR const char *buffer, size_t len)
{
  return len; /* Say that everything was written for now */
}

/****************************************************************************
 * Name: usbhost_poll
 *
 * Description:
 *   Standard character driver poll method.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int usbhost_poll(FAR struct file *filp, FAR struct pollfd *fds,
                        bool setup)
{
  if (setup)
    {
      fds->revents |= (fds->events & (POLLIN|POLLOUT));
      if (fds->revents != 0)
        {
          sem_post(fds->sem);
        }
    }
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_kbdinit
 *
 * Description:
 *   Initialize the USB storage HID keyboard class driver.  This function
 *   should be called be platform-specific code in order to initialize and
 *   register support for the USB host HID keyboard class device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_kbdinit(void)
{
  /* Perform any one-time initialization of the class implementation */

  /* Advertise our availability to support (certain) devices */

  return usbhost_registerclass(&g_skeleton);
}


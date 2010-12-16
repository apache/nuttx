/****************************************************************************
 * drivers/usbhost/usbhost_storage.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
#include <nuttx/usb/usb_storage.h>

#if defined(CONFIG_USBHOST) && !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* If the create() method is called by the USB host device driver from an
 * interrupt handler, then it will be unable to call malloc() in order to
 * allocate a new class instance.  If the create() method is called from the
 * interrupt level, then class instances must be pre-allocated.
 */

#ifndef CONFIG_USBHOST_NPREALLOC
#  define CONFIG_USBHOST_NPREALLOC 0
#endif

#if CONFIG_USBHOST_NPREALLOC > 26
#  error "Currently limited to 26 devices /dev/sda-z"
#endif

/* Driver support ***********************************************************/
/* This format is used to construct the /dev/sd[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT      "/dev/sd%c"
#define DEV_NAMELEN     10

/* Used in usbhost_configdesc() */

#define USBHOST_IFFOUND   0x01
#define USBHOST_BINFOUND  0x02
#define USBHOST_BOUTFOUND 0x04
#define USBHOST_ALLFOUND  0x07

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
  
  int16_t                 crefs;      /* Reference count on the driver instance */
  char                    sdchar;     /* Character identifying the /dev/sd[n] device */
  uint16_t                blocksize;  /* Block size of USB mass storage device */
  uint32_t                nblocks;    /* Number of blocks on the USB mass storage device */
  sem_t                   sem;        /* Used to maintain mutual exclusive access */
  struct work_s           work;       /* For interacting with the worker thread */
  struct usbhost_epdesc_s bulkin;     /* Bulk IN endpoint */
  struct usbhost_epdesc_s bulkout;    /* Bulk OUT endpoint */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphores */

static void    usbhost_takesem(FAR struct usbhost_state_s *priv);
#define usbhost_givesem(p) sem_post(&priv->sem);

/* Memory allocation services */

static inline struct usbhost_state_s *usbhost_allocclass(void);
static inline void usbhost_freeclass(struct usbhost_state_s *class);

/* Device name management */

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv);
static void usbhost_freedevno(FAR struct usbhost_state_s *priv);
static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv, char *devname);

/* Worker thread actions */

static void usbhost_destroy(FAR void *arg);
static void usbhost_work(FAR struct usbhost_state_s *priv, worker_t worker);

/* Data helpers */

static inline uint16_t usbhost_getint16(const uint8_t *val);

/* struct usbhost_registry_s methods */
 
static struct usbhost_class_s *usbhost_create(FAR struct usbhost_driver_s *drvr,
                                              FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_configdesc(FAR struct usbhost_class_s *class,
                              FAR const uint8_t *configdesc, int desclen);
static int usbhost_disconnected(FAR struct usbhost_class_s *class);

/* struct block_operations methods */

static int usbhost_open(FAR struct inode *inode);
static int usbhost_close(FAR struct inode *inode);
static ssize_t usbhost_read(FAR struct inode *inode, FAR unsigned char *buffer,
                            size_t startsector, unsigned int nsectors);
#ifdef CONFIG_FS_WRITABLE
static ssize_t usbhost_write(FAR struct inode *inode,
                             FAR const unsigned char *buffer, size_t startsector,
                             unsigned int nsectors);
#endif
static int usbhost_geometry(FAR struct inode *inode,
                            FAR struct geometry *geometry);
static int usbhost_ioctl(FAR struct inode *inode, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID informatino that will  be 
 * used to associate the USB host mass storage class to a connected USB
 * device.
 */

static const const struct usbhost_id_s g_id =
{
  USB_CLASS_MASS_STORAGE, /* base     */
  USBSTRG_SUBCLASS_SCSI,  /* subclass */
  USBSTRG_PROTO_BULKONLY, /* proto    */
  0,                      /* vid      */
  0                       /* pid      */
};

/* This is the USB host storage class's registry entry */

static struct usbhost_registry_s g_storage =
{
  NULL,                   /* flink    */
  usbhost_create,         /* create   */
  1,                      /* nids     */
  &g_id                   /* id[]     */
};

/* Block driver operations.  This is the interface exposed to NuttX by the
 * class that permits it to behave like a block driver.
 */

static const struct block_operations g_bops =
{
  usbhost_open,           /* open     */
  usbhost_close,          /* close    */
  usbhost_read,           /* read     */
#ifdef CONFIG_FS_WRITABLE
  usbhost_write,          /* write    */
#else
  NULL,                   /* write    */
#endif
  usbhost_geometry,       /* geometry */
  usbhost_ioctl           /* ioctl    */
};

/* This is an array of pre-allocated USB host storage class instances */

#if CONFIG_USBHOST_NPREALLOC > 0
static struct usbhost_state_s g_prealloc[CONFIG_USBHOST_NPREALLOC];
#endif

/* This is a list of free, pre-allocated USB host storage class instances */

#if CONFIG_USBHOST_NPREALLOC > 0
static struct usbhost_state_s *g_freelist;
#endif

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

static void usbhost_takesem(FAR struct usbhost_state_s *priv)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&priv->sem) != 0)
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

#if CONFIG_USBHOST_NPREALLOC > 0
static inline struct usbhost_state_s *usbhost_allocclass(void)
{
  struct usbhost_state_s *priv;
  irqstate_t flags;

  /* We may be executing from an interrupt handler so we need to take one of
   * our pre-allocated class instances from the free list.
   */

  flags = irqsave();
  priv = g_freelist;
  if (priv)
    {
      g_freelist        = priv->class.flink;
      priv->class.flink = NULL;
    }
  irqrestore(flags);
  return priv;
}
#else
static inline struct usbhost_state_s *usbhost_allocclass(void)
{
  /* We are not executing from an interrupt handler so we can just call
   * malloc() to get memory for the class instance.
   */

  DEBUGASSERT(!up_interrupt_context());
  return (struct usbhost_state_s *)malloc(sizeof(struct usbhost_state_s));
}
#endif

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

#if CONFIG_USBHOST_NPREALLOC > 0
static inline void usbhost_freeclass(struct usbhost_state_s *class)
{
  irqstate_t flags;
  DEBUGASSERT(class != NULL);

  /* Just put the pre-allocated class structure back on the freelist */

  flags = irqsave();
  class->class.flink = g_freelist;
  g_freelist = class;
  irqrestore(flags);  
}
#else
static inline void usbhost_freeclass(struct usbhost_state_s *class)
{
  DEBUGASSERT(class != NULL);

  /* Free the class instance (calling sched_free() in case we are executing
   * from an interrupt handler.
   */

  free(class);
}
#endif

/****************************************************************************
 * Name: Device name management
 *
 * Description:
 *   Some tiny functions to coordinate management of mass storage device names.
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
 * Name: usbhost_destroy
 *
 * Description:
 *   The USB mass storage device has been disconnected and the refernce count
 *   on the USB host class instance has gone to 1.. Time to destroy the USB
 *   host class instance.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be freed.
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
 
  /* Unregister the block driver */

  usbhost_mkdevname(priv, devname);
  (void)unregister_blockdriver(devname);

  /* Release the device name used by this connection */

  usbhost_freedevno(priv);

  /* And free the class instance.  Hmmm.. this may execute on the worker
   * thread and the work structure is part of what is getting freed.
   */

  usbhost_freeclass(priv);
}

/****************************************************************************
 * Name: usbhost_configluns
 *
 * Description:
 *   The USB mass storage device has been successfully connected.  Now get
 *   information about the connect LUNs.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be freed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_configluns(FAR void *arg)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)arg;

  /* Get the maximum logical unit number */

  /* Check if the unit is ready */

  /* Get sense information */

  /* Read the capacity of the volume */

  /* Register the block driver */
#warning "Missing Logic"
}

/****************************************************************************
 * Name: usbhost_work
 *
 * Description:
 *   Perform work, depending on context:  If we are executing from an
 *   interrupt handler, then defer the work to the worker thread.  Otherwise,
 *   just execute the work now.
 *
 * Input Parameters:
 *   priv - A reference to the class instance to be freed.
 *   worker - A reference to the worker function to be executed
 *
 * Returned Values:
 *   A uin16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static void usbhost_work(FAR struct usbhost_state_s *priv, worker_t worker)
{
  /* Are we in an interrupt handler? */

  if (up_interrupt_context())
    {
      /* Yes.. do the work on the worker thread.  Higher level logic should
       * prevent us from over-running the work structure.
       */

      (void)work_queue(&priv->work, worker, priv, 0);
    }
  else
    {
      /* No.. do the work now */

      worker(priv);
    }
}

/****************************************************************************
 * Name: usbhost_getint16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Values:
 *   A uin16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t usbhost_getint16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
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

  /* Allocate a USB host mass storage class instance */

  priv = usbhost_allocclass();
  if (priv)
    {
      /* Initialize the allocated storage class instance */

      memset(priv, 0, sizeof(struct usbhost_state_s));

      /* Assign a device number to this class instance */

      if (usbhost_allocdevno(priv) == OK)
        {
          priv->class.configdesc   = usbhost_configdesc;
          priv->class.disconnected = usbhost_disconnected;
          priv->crefs              = 1;

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
 * Name: usbhost_configdesc
 *
 * Description:
 *   This function implemented the configdesc() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration descripor.
 *   desclen - The length in bytes of the configuration descriptor.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 ****************************************************************************/

static int usbhost_configdesc(FAR struct usbhost_class_s *class,
                              FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)class;
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  int remaining;
  uint8_t found = 0;

  /* Verify that we were passed a configuration descriptor */

  cfgdesc = (FAR struct usb_cfgdesc_s *)configdesc;
  if (cfgdesc->type != USB_DESC_TYPE_CONFIG)
    {
      return -EINVAL;
    }

  /* Get the total length of the configuration descriptor (little endian).
   * It might be a good check to get the number of interfaces here too.
  */

  remaining = (int)usbhost_getint16(cfgdesc->totallen);

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
            DEBUGASSERT(remaining >= sizeof(struct usb_ifdesc_s));
            if ((found & USBHOST_IFFOUND) != 0)
              {
                /* Oops.. more than one interface.  We don't know what to do with this. */

                return -ENOSYS;
              }
            found |= USBHOST_IFFOUND;
          }
          break;

        /* Endpoint descriptor.  We expect two bulk endpoints, an IN and an OUT */
        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)configdesc;
            DEBUGASSERT(remaining >= sizeof(struct usb_epdesc_s));

            /* Check for a bulk endpoint.  We only support the bulk-only
             * protocol so I suppose anything else should really be an error.
             */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_BULK)
              {
                /* Yes.. it is a bulk endpoint.  IN or OUT? */

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an OUT bulk endpoint.  There should be only one bulk OUT endpoint. */

                    if ((found & USBHOST_BOUTFOUND) != 0)
                      {
                        /* Oops.. more than one interface.  We don't know what to do with this. */

                        return -EINVAL;
                      }
                    found |= USBHOST_BOUTFOUND;

                    /* Save the bulk OUT endpoint information */

                    priv->bulkout.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    priv->bulkout.in           = false;
                    priv->bulkout.mxpacketsize = usbhost_getint16(epdesc->mxpacketsize);
                  }
                else
                  {
                    /* It is an IN bulk endpoint.  There should be only one bulk IN endpoint. */

                    if ((found & USBHOST_BINFOUND) != 0)
                      {
                        /* Oops.. more than one interface.  We don't know what to do with this. */

                        return -EINVAL;
                      }
                    found |= USBHOST_BINFOUND;

                    /* Save the bulk IN endpoint information */
                    
                    priv->bulkin.addr          = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    priv->bulkin.in            = true;
                    priv->bulkin.mxpacketsize  = usbhost_getint16(epdesc->mxpacketsize);
                  }
              }
          }
          break;

        /* Other descriptors are just ignored for now */

        default:
          break;
        }

      /* Increment the address of the next descriptor */
 
      configdesc += desc->len;
      remaining  -= desc->len;
    }

  /* Sanity checking... did we find all of things that we need? Hmmm..  I wonder..
   * can we work read-only or write-only if only one bulk endpoint found?
   */
    
  if (found != USBHOST_ALLFOUND)
    {
      ulldbg("ERROR: Found IF:%s BIN:%s BOUT:%s\n",
             (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
             (found & USBHOST_BINFOUND) != 0 ? "YES" : "NO",
             (found & USBHOST_BOUTFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  ullvdbg("Mass Storage device connected\n");

  /* Now configure the LUNs and register the block driver(s) */

  usbhost_work(priv, usbhost_configluns);
  return OK;
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
 ****************************************************************************/

static int usbhost_disconnected(struct usbhost_class_s *class)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)class;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  /* Nullify the driver instance.  This will be our indication to any users
   * of the mass storage device that the device is no longer available.
   */

  flags = irqsave();
  priv->drvr = NULL;

  /* Now check the number of references on the class instance.  If it is one,
   * then we can free the class instance now.  Otherwise, we will have to
   * wait until the holders of the references free them by closing the
   * block driver.
   */

  if (priv->crefs == 1)
    {
      /* Destroy the class instance */

      usbhost_work(priv, usbhost_destroy);
    }
  irqrestore(flags);  
  return OK;
}

/****************************************************************************
 * struct block_operations methods
 ****************************************************************************/
/****************************************************************************
 * Name: usbhost_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int usbhost_open(FAR struct inode *inode)
{
  FAR struct usbhost_state_s *priv;
  int ret;

  uvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct usbhost_state_s *)inode->i_private;

  /* Check if the mass storage device is still connected */

  if (!priv->drvr)
    {
      /* No... the block driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse any further
       * attempts to open the driver.
       */

      ret = -ENODEV;
    }
  else
    {
      /* Otherwise, just increment the reference count on the driver */

      DEBUGASSERT(priv->crefs < MAX_CREFS);
      usbhost_takesem(priv);
      priv->crefs++;
      usbhost_givesem(priv);
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int usbhost_close(FAR struct inode *inode)
{
  FAR struct usbhost_state_s *priv;

  uvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct usbhost_state_s *)inode->i_private;

  /* Decrement the reference count on the block driver */

  DEBUGASSERT(priv->crefs > 0);
  usbhost_takesem(priv);
  priv->crefs--;

  /* Release the semaphore.  The following operations when crefs == 1 are
   * safe because we know that there is no outstanding open references to
   * the block driver.
   */

  usbhost_givesem(priv);

  /* Check if the USB mass storage device is still connected.  If the
   * storage device is not connected and the reference count just
   * decremented to one, then unregister the block driver and free
   * the class instance.
   */

  if (priv->crefs <= 1 && priv->drvr == NULL)
    {
      /* Destroy the class instance */

      usbhost_destroy(priv);
    }
  return OK;
}

/****************************************************************************
 * Name: usbhost_read
 *
 * Description:
 *   Read the specified numer of sectors from the read-ahead buffer or from
 *   the physical device.
 *
 ****************************************************************************/

static ssize_t usbhost_read(FAR struct inode *inode, unsigned char *buffer,
                            size_t startsector, unsigned int nsectors)
{
  FAR struct usbhost_state_s *priv;
  ssize_t ret = 0;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct usbhost_state_s *)inode->i_private;
  uvdbg("startsector: %d nsectors: %d sectorsize: %d\n",
        startsector, nsectors, priv->blocksize);

  /* Check if the mass storage device is still connected */

  if (!priv->drvr)
    {
      /* No... the block driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse any attempt to read
       * from the device.
       */

      ret = -ENODEV;
    }
  else if (nsectors > 0)
    {
      usbhost_takesem(priv);
#warning "Missing logic"
      usbhost_givesem(priv);
    }

  /* On success, return the number of blocks read */

  return ret;
}

/****************************************************************************
 * Name: usbhost_write
 *
 * Description:
 *   Write the specified number of sectors to the write buffer or to the
 *   physical device.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static ssize_t usbhost_write(FAR struct inode *inode, const unsigned char *buffer,
                           size_t startsector, unsigned int nsectors)
{
  FAR struct usbhost_state_s *priv;
  int ret;

  uvdbg("sector: %d nsectors: %d sectorsize: %d\n");
  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct usbhost_state_s *)inode->i_private;

  /* Check if the mass storage device is still connected */

  if (!priv->drvr)
    {
      /* No... the block driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse any attempt to
       * write to the device.
       */

      ret = -ENODEV;
    }
  else
    {
      usbhost_takesem(priv);
#warning "Missing logic"
      usbhost_givesem(priv);
    }

  /* On success, return the number of blocks written */

  return ret;
}
#endif

/****************************************************************************
 * Name: usbhost_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int usbhost_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  FAR struct usbhost_state_s *priv;
  int ret = -EINVAL;

  uvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);

  /* Check if the mass storage device is still connected */

  if (!priv->drvr)
    {
      /* No... the block driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse to return any
       * geometry info.
       */

      ret = -ENODEV;
    }
  else if (geometry)
    {
      /* Return the geometry of the USB mass storage device */

      priv = (FAR struct usbhost_state_s *)inode->i_private;
      usbhost_takesem(priv);

      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
#ifdef CONFIG_FS_WRITABLE
      geometry->geo_writeenabled  = true;
#else
      geometry->geo_writeenabled  = false;
#endif
      geometry->geo_nsectors      = priv->nblocks;
      geometry->geo_sectorsize    = priv->blocksize;
      usbhost_givesem(priv);

      uvdbg("nsectors: %ld sectorsize: %d\n",
             (long)geometry->geo_nsectors, geometry->geo_sectorsize);

      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int usbhost_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{
  FAR struct usbhost_state_s *priv;
  int ret;

  uvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct usbhost_state_s *)inode->i_private;

  /* Check if the mass storage device is still connected */

  if (!priv->drvr)
    {
      /* No... the block driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse to process any
       * ioctl commands.
       */

      ret = -ENODEV;
    }
  else
    {
      /* Process the IOCTL by command */

      usbhost_takesem(priv);
      switch (cmd)
        {
        /* Add support for ioctl commands here */

        default:
          ret = -ENOTTY;
          break;
        }

      usbhost_givesem(priv);
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_storageinit
 *
 * Description:
 *   Initialize the USB host storage class.  This function should be called
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

int usbhost_storageinit(void)
{
  /* If we have been configured to use pre-allocated storage class instances,
   * then place all of the pre-allocated USB host storage class instances
   * into a free list.
   */

#if CONFIG_USBHOST_NPREALLOC > 0
  int i;

  g_freelist = NULL;
  for (i = 0; i < CONFIG_USBHOST_NPREALLOC; i++)
    {
      struct usbhost_state_s *class = &g_prealloc[i];
      class->class.flink = g_freelist;
      g_freelist         = class;
    }
#endif

  /* Advertise our availability to support (certain) mass storage devices */

  return usbhost_registerclass(&g_storage);
}

#endif  /* CONFIG_USBHOST && !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 */

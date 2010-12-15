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

#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs.h>
#include <nuttx/arch.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* If the create() method is called by the USB host device driver from an
 * interrupt handler, then it will be unable to call malloc() in order to
 * allocate a new class instance.  If the create() method is called from the
 * interrupt level, then class instances must be pre-allocated.
 */

#ifndef CONFIG_USBHOST_NPREALLOC
#  define CONFIG_USBHOST_NPREALLOC 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structur contains the internal, private state of the USB host mass
 * storage class.
 */

struct usbhost_state_s
{
  /* This is the externally visible portion of the state */

  struct usbhost_class_s  class;

  /* This is an instance of the USB host driver bound to this class instance */

  struct usbhost_driver_s *drvr;

  /* The remainder of the fields are provide o the mass storage class */
  
  int                     crefs;      /* Reference count on the driver instance */
  uint16_t                blocksize;  /* Block size of USB mass storage device */
  uint32_t                nblocks;    /* Number of blocks on the USB mass storage device */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* struct usbhost_registry_s methods */
 
static inline struct usbhost_state_s *usbhost_allocclass(void);
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
static int     usbhost_geometry(FAR struct inode *inode,
                 FAR struct geometry *geometry);
static int     usbhost_ioctl(FAR struct inode *inode, int cmd,
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
  SUBSTRG_SUBCLASS_SCSI,  /* subclass */
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * struct usbhost_registry_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_allocclass
 *
 * Description:
 *   This is really part of the logic that implementes the create() method
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

static inline struct usbhost_state_s *usbhost_allocclass(void)
{
  struct usbhost_state_s *priv;

#if CONFIG_USBHOST_NPREALLOC > 0
  /* We are executing from an interrupt handler so we need to take one of our
   * pre-allocated class instances from the free list.  No special protection
   * is needed if we are in an interrupt handler.
   */

  priv = g_freelist;
  if (priv)
    {
      g_freelist        = priv->class.flink;
      priv->class.flink = NULL;
    }
#else
  /* We are not executing from an interrupt handler so we can just call
   * malloc() to get memory for the class instance.
   */

  DEBUGASSERT(!up_interrupt_context());
  priv = (struct usbhost_state_s *)malloc(sizeof(struct usbhost_state_s));
#endif
  return priv;
}

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

  priv = usbhost_allocclass(void);
  if (priv)
    {
      /* Initialize the allocated storage class instance */

      memset(priv, 0, sizeof(struct usbhost_state_s);
      priv->class.configdesc   = usbhost_configdesc;
      priv->class.disconnected = usbhost_disconnected;
      priv->crefs              = 1;

      /* Bind the driver to the storage class instance */

      priv->drvr               = drvr;

      /* NOTE: We do not yet know the geometry of the USB mass storage device */
      
      /* Return the instance of the USB mass storage class */
 
      return &priv->class;
    }

  /* Return NULL on all failures */

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

static int usbhost_configdesc(struct usbhost_class_s *class,
                              const uint8_t *configdesc, int desclen)
{
#warning "Missing Implementation"
  return -ENOSYS;
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
  DEBUGASSERT(priv != NULL);

  /* Nullify the driver instance.  This will be our indication to any users
   * of the mass storage device that the device is no longer available.
   */

  priv->drvr = NULL;

  /* Now check the number of references on the class instance.  If it is one,
   * then we can free the class instance now.  Otherwise, we will have to
   * wait until the holders of the references free them by closing the
   * block driver.
   */

#warning "Missing Implementation"

  /* Unregister the block device */

#warning "Missing Implementation"
  return -ENOSYS;
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
       * the USB storage device is no longer connected.
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

  /* Check if the USB mass storage device is still connected.  If the
   * storage device is not connected and the reference count just
   * decremented to one, then unregister the block driver and free
   * the class instance.
   */
#warning "Missing Implementation"
 
  usbhost_givesem(priv);
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
       * the USB storage device is no longer connected.
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
       * the USB storage device is no longer connected.
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
       * the USB storage device is no longer connected.
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
       * the USB storage device is no longer connected.
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


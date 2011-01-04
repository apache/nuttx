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
#include <nuttx/scsi.h>

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

#define DEV_FORMAT          "/dev/sd%c"
#define DEV_NAMELEN         10

/* Used in usbhost_connect() */

#define USBHOST_IFFOUND     0x01
#define USBHOST_BINFOUND    0x02
#define USBHOST_BOUTFOUND   0x04
#define USBHOST_ALLFOUND    0x07

#define USBHOST_MAX_RETRIES 100
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
  
  char                    sdchar;     /* Character identifying the /dev/sd[n] device */
  int16_t                 crefs;      /* Reference count on the driver instance */
  uint16_t                blocksize;  /* Block size of USB mass storage device */
  uint32_t                nblocks;    /* Number of blocks on the USB mass storage device */
  sem_t                   exclsem;    /* Used to maintain mutual exclusive access */
  struct work_s           work;       /* For interacting with the worker thread */
  FAR uint8_t            *tdbuffer;   /* The allocated transfer descriptor buffer */
  size_t                  tdbuflen;   /* Size of the allocated transfer buffer */
  struct usbhost_epdesc_s bulkin;     /* Bulk IN endpoint */
  struct usbhost_epdesc_s bulkout;    /* Bulk OUT endpoint */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphores */

static void usbhost_takesem(sem_t *sem);
#define usbhost_givesem(s) sem_post(s);

/* Memory allocation services */

static inline struct usbhost_state_s *usbhost_allocclass(void);
static inline void usbhost_freeclass(struct usbhost_state_s *class);

/* Device name management */

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv);
static void usbhost_freedevno(FAR struct usbhost_state_s *priv);
static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv, char *devname);

/* CBW/CSW debug helpers */

#if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_VERBOSE)
static void usbhost_dumpcbw(FAR struct usbstrg_cbw_s *cbw);
static void usbhost_dumpcsw(FAR struct usbstrg_csw_s *csw);
#else
#  define usbhost_dumpcbw(cbw);
#  define usbhost_dumpcsw(csw);
#endif

/* CBW helpers */

static inline void usbhost_requestsensecbw(FAR struct usbstrg_cbw_s *cbw);
static inline void usbhost_testunitreadycbw(FAR struct usbstrg_cbw_s *cbw);
static inline void usbhost_readcapacitycbw(FAR struct usbstrg_cbw_s *cbw);
static inline void usbhost_inquirycbw (FAR struct usbstrg_cbw_s *cbw);
static inline void usbhost_readcbw (size_t startsector, uint16_t blocksize,
                                    unsigned int nsectors,
                                    FAR struct usbstrg_cbw_s *cbw);
static inline void usbhost_writecbw(size_t startsector, uint16_t blocksize,
                                    unsigned int nsectors,
                                    FAR struct usbstrg_cbw_s *cbw);
/* Command helpers */

static inline int usbhost_maxlunreq(FAR struct usbhost_state_s *priv);
static inline int usbhost_testunitready(FAR struct usbhost_state_s *priv);
static inline int usbhost_requestsense(FAR struct usbhost_state_s *priv);
static inline int usbhost_readcapacity(FAR struct usbhost_state_s *priv);
static inline int usbhost_inquiry(FAR struct usbhost_state_s *priv);

/* Worker thread actions */

static void usbhost_destroy(FAR void *arg);
static void usbhost_initvolume(FAR void *arg);
static void usbhost_work(FAR struct usbhost_state_s *priv, worker_t worker);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(const uint8_t *val);
static inline uint16_t usbhost_getbe16(const uint8_t *val);
static inline void usbhost_putle16(uint8_t *dest, uint16_t val);
static inline void usbhost_putbe16(uint8_t *dest, uint16_t val);
static inline uint32_t usbhost_getle32(const uint8_t *val);
static inline uint32_t usbhost_getbe32(const uint8_t *val);
static void usbhost_putle32(uint8_t *dest, uint32_t val);
static void usbhost_putbe32(uint8_t *dest, uint32_t val);

/* Transfer descriptor memory management */

static inline int usbhost_tdalloc(FAR struct usbhost_state_s *priv);
static inline int usbhost_tdfree(FAR struct usbhost_state_s *priv);
static FAR struct usbstrg_cbw_s *usbhost_cbwalloc(FAR struct usbhost_state_s *priv);

/* struct usbhost_registry_s methods */
 
static struct usbhost_class_s *usbhost_create(FAR struct usbhost_driver_s *drvr,
                                              FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr);
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
 * Name: CBW/CSW debug helpers
 *
 * Description:
 *   The following functions are helper functions used to dump CBWs and CSWs.
 *
 * Input Parameters:
 *   cbw/csw - A reference to the CBW/CSW to dump.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_VERBOSE)
static void usbhost_dumpcbw(FAR struct usbstrg_cbw_s *cbw)
{
  int i;

  uvdbg("CBW:\n");
  uvdbg("  signature: %08x\n", usbhost_getle32(cbw->signature));
  uvdbg("  tag:       %08x\n", usbhost_getle32(cbw->tag));
  uvdbg("  datlen:    %08x\n", usbhost_getle32(cbw->datlen));
  uvdbg("  flags:     %02x\n", cbw->flags);
  uvdbg("  lun:       %02x\n", cbw->lun);
  uvdbg("  cdblen:    %02x\n", cbw->cdblen);

  uvdbg("CDB:\n");
  for (i = 0; i < cbw->cdblen; i += 8)
    {
      uvdbg("  %02x %02x %02x %02x %02x %02x %02x %02x\n",
            cbw->cdb[i],   cbw->cdb[i+1], cbw->cdb[i+2], cbw->cdb[i+3],
            cbw->cdb[i+4], cbw->cdb[i+5], cbw->cdb[i+6], cbw->cdb[i+7]);
    }
}

static void usbhost_dumpcsw(FAR struct usbstrg_csw_s *csw)
{
  uvdbg("CSW:\n");
  uvdbg("  signature: %08x\n", usbhost_getle32(csw->signature));
  uvdbg("  tag:       %08x\n", usbhost_getle32(csw->tag));
  uvdbg("  residue:   %08x\n", usbhost_getle32(csw->residue));
  uvdbg("  status:    %02x\n", csw->status);
}
#endif

/****************************************************************************
 * Name: CBW helpers
 *
 * Description:
 *   The following functions are helper functions used to format CBWs.
 *
 * Input Parameters:
 *   cbw - A reference to allocated and initialized CBW to be built.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline void usbhost_requestsensecbw(FAR struct usbstrg_cbw_s *cbw)
{
  FAR struct scsicmd_requestsense_s *reqsense;

  /* Format the CBW */

  usbhost_putle32(cbw->datlen, SCSIRESP_FIXEDSENSEDATA_SIZEOF);
  cbw->flags         = USBSTRG_CBWFLAG_IN;
  cbw->cdblen        = SCSICMD_REQUESTSENSE_SIZEOF;

  /* Format the CDB */

  reqsense           = (FAR struct scsicmd_requestsense_s *)cbw->cdb;
  reqsense->opcode   = SCSI_CMD_REQUESTSENSE;
  reqsense->alloclen = SCSIRESP_FIXEDSENSEDATA_SIZEOF;

  usbhost_dumpcbw(cbw);
}

static inline void usbhost_testunitreadycbw(FAR struct usbstrg_cbw_s *cbw)
{
  /* Format the CBW */

  cbw->cdblen = SCSICMD_TESTUNITREADY_SIZEOF;

  /* Format the CDB */

  cbw->cdb[0] = SCSI_CMD_TESTUNITREADY;

  usbhost_dumpcbw(cbw);
}

static inline void usbhost_readcapacitycbw(FAR struct usbstrg_cbw_s *cbw)
{
  FAR struct scsicmd_readcapacity10_s *rcap10;

  /* Format the CBW */

  usbhost_putle32(cbw->datlen, SCSIRESP_READCAPACITY10_SIZEOF);
  cbw->flags     = USBSTRG_CBWFLAG_IN;
  cbw->cdblen    = SCSICMD_READCAPACITY10_SIZEOF;

  /* Format the CDB */

  rcap10         = (FAR struct scsicmd_readcapacity10_s *)cbw->cdb;
  rcap10->opcode = SCSI_CMD_READCAPACITY10;

  usbhost_dumpcbw(cbw);
}

static inline void usbhost_inquirycbw (FAR struct usbstrg_cbw_s *cbw)
{
  FAR struct scscicmd_inquiry_s *inq;

  /* Format the CBW */

  usbhost_putle32(cbw->datlen, SCSIRESP_INQUIRY_SIZEOF);
  cbw->flags    = USBSTRG_CBWFLAG_IN;
  cbw->cdblen   = SCSICMD_INQUIRY_SIZEOF;

  /* Format the CDB */

  inq           = (FAR struct scscicmd_inquiry_s *)cbw->cdb;
  inq->opcode   = SCSI_CMD_INQUIRY;
  usbhost_putbe16(inq->alloclen, SCSIRESP_INQUIRY_SIZEOF);

  usbhost_dumpcbw(cbw);
}

static inline void
usbhost_readcbw (size_t startsector, uint16_t blocksize,
                 unsigned int nsectors, FAR struct usbstrg_cbw_s *cbw)
{
  FAR struct scsicmd_read10_s *rd10;

  /* Format the CBW */

  usbhost_putle32(cbw->datlen, blocksize * nsectors);
  cbw->flags   = USBSTRG_CBWFLAG_IN;
  cbw->cdblen  = SCSICMD_READ10_SIZEOF;

  /* Format the CDB */

  rd10 = (FAR struct scsicmd_read10_s *)cbw->cdb;
  rd10->opcode = SCSI_CMD_READ10;
  usbhost_putbe32(rd10->lba, startsector);
  usbhost_putbe16(rd10->xfrlen, nsectors);

  usbhost_dumpcbw(cbw);
}

static inline void
usbhost_writecbw(size_t startsector, uint16_t blocksize,
                 unsigned int nsectors, FAR struct usbstrg_cbw_s *cbw)
{
  FAR struct scsicmd_write10_s *wr10;

  /* Format the CBW */

  usbhost_putle32(cbw->datlen, blocksize * nsectors);
  cbw->cdblen  = SCSICMD_WRITE10_SIZEOF;

  /* Format the CDB */

  wr10 = (FAR struct scsicmd_write10_s *)cbw->cdb;
  wr10->opcode = SCSI_CMD_WRITE10;
  usbhost_putbe32(wr10->lba, startsector);
  usbhost_putbe16(wr10->xfrlen, nsectors);

  usbhost_dumpcbw(cbw);
}

/****************************************************************************
 * Name: Command helpers
 *
 * Description:
 *   The following functions are helper functions used to send commands.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline int usbhost_maxlunreq(FAR struct usbhost_state_s *priv)
{
  FAR struct usb_ctrlreq_s *req = (FAR struct usb_ctrlreq_s *)priv->tdbuffer;
  DEBUGASSERT(priv && priv->tdbuffer);

  /* Request maximum logical unit number.  NOTE: On an IN transaction, The
   * req and buffer pointers passed to DRVR_CTRLIN may refer to the same
   * allocated memory.
   */

  uvdbg("Request maximum logical unit number\n");
  memset(req, 0, sizeof(struct usb_ctrlreq_s));
  req->type    = USB_DIR_IN|USB_REQ_TYPE_CLASS|USB_REQ_RECIPIENT_INTERFACE;
  req->req     = USBSTRG_REQ_GETMAXLUN;
  usbhost_putle16(req->len, 1);
  return DRVR_CTRLIN(priv->drvr, req, priv->tdbuffer);
}

static inline int usbhost_testunitready(FAR struct usbhost_state_s *priv)
{
  FAR struct usbstrg_cbw_s *cbw;
  int result = -ENOMEM;

  /* Initialize a CBW (re-using the allocated transfer buffer) */
 
  cbw = usbhost_cbwalloc(priv);
  if (cbw)
    {
      /* Construct and send the CBW */
 
      usbhost_testunitreadycbw(cbw);
      result = DRVR_TRANSFER(priv->drvr, &priv->bulkout,
                            (uint8_t*)cbw, USBSTRG_CBW_SIZEOF);
      if (result == OK)
        {
          /* Receive the CSW */

          result = DRVR_TRANSFER(priv->drvr, &priv->bulkin,
                                 priv->tdbuffer, USBSTRG_CSW_SIZEOF);
          if (result == OK)
            {
              usbhost_dumpcsw((FAR struct usbstrg_csw_s *)priv->tdbuffer);
            }
        }
    }
  return result;
}

static inline int usbhost_requestsense(FAR struct usbhost_state_s *priv)
{
  FAR struct usbstrg_cbw_s *cbw;
  int result = -ENOMEM;

  /* Initialize a CBW (re-using the allocated transfer buffer) */
 
  cbw = usbhost_cbwalloc(priv);
  if (cbw)
    {
      /* Construct and send the CBW */
 
      usbhost_requestsensecbw(cbw);
      result = DRVR_TRANSFER(priv->drvr, &priv->bulkout,
                            (uint8_t*)cbw, USBSTRG_CBW_SIZEOF);
      if (result == OK)
        {
          /* Receive the sense data response */

          result = DRVR_TRANSFER(priv->drvr, &priv->bulkin,
                                 priv->tdbuffer, SCSIRESP_FIXEDSENSEDATA_SIZEOF);
          if (result == OK)
            {
              /* Receive the CSW */

              result = DRVR_TRANSFER(priv->drvr, &priv->bulkin,
                                     priv->tdbuffer, USBSTRG_CSW_SIZEOF);
              if (result == OK)
                {
                  usbhost_dumpcsw((FAR struct usbstrg_csw_s *)priv->tdbuffer);
                }
            }
        }
    }
  return result;
}

static inline int usbhost_readcapacity(FAR struct usbhost_state_s *priv)
{
  FAR struct usbstrg_cbw_s *cbw;
  FAR struct scsiresp_readcapacity10_s *resp;
  int result = -ENOMEM;

  /* Initialize a CBW (re-using the allocated transfer buffer) */
 
  cbw = usbhost_cbwalloc(priv);
  if (cbw)
    {
      /* Construct and send the CBW */
 
      usbhost_readcapacitycbw(cbw);
      result = DRVR_TRANSFER(priv->drvr, &priv->bulkout,
                             (uint8_t*)cbw, USBSTRG_CBW_SIZEOF);
      if (result == OK)
        {
          /* Receive the read capacity CBW IN response */

          result = DRVR_TRANSFER(priv->drvr, &priv->bulkin,
                                 priv->tdbuffer, SCSIRESP_READCAPACITY10_SIZEOF);
          if (result == OK)
            {
              /* Save the capacity information */

              resp = (FAR struct scsiresp_readcapacity10_s *)priv->tdbuffer;
              priv->nblocks = usbhost_getbe32(resp->lba);
              priv->blocksize = usbhost_getbe32(resp->blklen);

              /* Receive the CSW */

              result = DRVR_TRANSFER(priv->drvr, &priv->bulkin,
                                     priv->tdbuffer, USBSTRG_CSW_SIZEOF);
              if (result == OK)
                {
                  usbhost_dumpcsw((FAR struct usbstrg_csw_s *)priv->tdbuffer);
                }
            }
        }
    }
  return result;
}

static inline int usbhost_inquiry(FAR struct usbhost_state_s *priv)
{
  FAR struct usbstrg_cbw_s *cbw;
  FAR struct scsiresp_inquiry_s *resp;
  int result = -ENOMEM;

  /* Initialize a CBW (re-using the allocated transfer buffer) */
 
  cbw = usbhost_cbwalloc(priv);
  if (cbw)
    {
      /* Construct and send the CBW */
 
      usbhost_inquirycbw(cbw);
      result = DRVR_TRANSFER(priv->drvr, &priv->bulkout,
                             (uint8_t*)cbw, USBSTRG_CBW_SIZEOF);
      if (result == OK)
        {
          /* Receive the CBW IN response */

          result = DRVR_TRANSFER(priv->drvr, &priv->bulkin,
                                 priv->tdbuffer, SCSIRESP_INQUIRY_SIZEOF);
          if (result == OK)
            {
              /* TODO: If USB debug is enabled, dump the response data here */

              resp = (FAR struct scsiresp_inquiry_s *)priv->tdbuffer;

              /* Receive the CSW */

              result = DRVR_TRANSFER(priv->drvr, &priv->bulkin,
                                     priv->tdbuffer, USBSTRG_CSW_SIZEOF);
              if (result == OK)
                {
                  usbhost_dumpcsw((FAR struct usbstrg_csw_s *)priv->tdbuffer);
                }
            }
        }
    }
  return result;
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
 
  /* Unregister the block driver */

  usbhost_mkdevname(priv, devname);
  (void)unregister_blockdriver(devname);

  /* Release the device name used by this connection */

  usbhost_freedevno(priv);

  /* Free any transfer buffers */

  usbhost_tdfree(priv);

  /* Destroy the semaphores */

  sem_destroy(&priv->exclsem);

  /* And free the class instance.  Hmmm.. this may execute on the worker
   * thread and the work structure is part of what is getting freed.
   */

  usbhost_freeclass(priv);
}

/****************************************************************************
 * Name: usbhost_initvolume
 *
 * Description:
 *   The USB mass storage device has been successfully connected.  This
 *   completes the initialization operations.  It is first called after the
 *   configuration descriptor has been received.
 *
 *   This function is called from the connect() method.  It may either
 *   execute on (1) the thread of the caller of connect(), or (2) if
 *   connect() was called from an interrupt handler, on the worker thread.
 *
 * Input Parameters:
 *   arg - A reference to the class instance.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_initvolume(FAR void *arg)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)arg;
  FAR struct usbstrg_csw_s *csw;
  unsigned int retries;
  int result = OK;

  DEBUGASSERT(priv != NULL);

  /* Set aside a transfer buffer for exclusive use by the mass storage driver */

  result = usbhost_tdalloc(priv);
  if (result != OK)
    {
      udbg("Failed to allocate transfer buffer\n");
      return;
    }

  /* Request the maximum logical unit number */

  uvdbg("Get max LUN\n");
  result = usbhost_maxlunreq(priv);

  /* Wait for the unit to be ready */

  for (retries = 0; retries < USBHOST_MAX_RETRIES && result == OK; retries++)
    {
      uvdbg("Test unit ready, retries=%d\n", retries);

      /* Send TESTUNITREADY to see the unit is ready */
      
      result = usbhost_testunitready(priv);
      if (result == OK)
        {
          /* Is the unit is ready */

          csw = (FAR struct usbstrg_csw_s *)priv->tdbuffer;
          if (csw->status == 0)
            {
              /* Yes... break out of the loop */

              break;
            }

          /* No.. Request mode sense information.  The REQUEST SENSE command
           * is sent only "to clear interlocked unit attention conditions."
           * The returned status is ignored here.
           */

          uvdbg("Request sense\n");
          result = usbhost_requestsense(priv);
        }
    }

  /* Did the unit become ready?  Did an error occur? Or did we time out? */

  if (retries >= USBHOST_MAX_RETRIES)
    {
      udbg("ERROR: Timeout!\n");
      result = -ETIMEDOUT;
    }

  if (result == OK)
    {
      /* Get the capacity of the volume */

      uvdbg("Read capacity\n");
      result = usbhost_readcapacity(priv);
      if (result == OK)
        {
          /* Check the CSW for errors */

          csw = (FAR struct usbstrg_csw_s *)priv->tdbuffer;
          if (csw->status != 0)
            {
              udbg("CSW status error: %d\n", csw->status);
              result = -ENODEV;
            }
        }
    }

  /* Get information about the volume */

  if (result == OK)
    {
      /* Inquiry */

      uvdbg("Inquiry\n");
      result = usbhost_inquiry(priv);
      if (result == OK)
        {
          /* Check the CSW for errors */

          csw = (FAR struct usbstrg_csw_s *)priv->tdbuffer;
          if (csw->status != 0)
            {
              udbg("CSW status error: %d\n", csw->status);
              result = -ENODEV;
            }
        }
    }

  /* Register the block driver */

  if (result == OK)
    {
      char devname[DEV_NAMELEN];

      uvdbg("Register block driver\n");
      usbhost_mkdevname(priv, devname);
      result = register_blockdriver(devname, &g_bops, 0, priv);
    }

  /* Check if we successfully initialized */

  if (result == OK)
    {
      /* Ready for normal operation as a block device driver */

      uvdbg("Successfully initialized\n");
    }
  else
    {
      udbg("ERROR! Aborting: %d\n", result);
      DRVR_DISCONNECT(priv->drvr);
      usbhost_destroy(priv);
    }
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
 *   priv - A reference to the class instance.
 *   worker - A reference to the worker function to be executed
 *
 * Returned Values:
 *   None
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
 * Name: usbhost_getbe16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit big endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the big endian value.
 *
 * Returned Values:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t usbhost_getbe16(const uint8_t *val)
{
  return (uint16_t)val[0] << 8 | (uint16_t)val[1];
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
 * Name: usbhost_putbe16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit big endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_putbe16(uint8_t *dest, uint16_t val)
{
  dest[0] = val >> 8; /* Big endian means MS byte first in byte stream */
  dest[1] = val & 0xff;
}

/****************************************************************************
 * Name: usbhost_getbe32
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
 * Name: usbhost_getbe32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit big endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline uint32_t usbhost_getbe32(const uint8_t *val)
{
  /* Big endian means MS halfword first in byte stream */

  return (uint32_t)usbhost_getbe16(val) << 16 | (uint32_t)usbhost_getbe16(&val[2]);
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
 * Name: usbhost_putbe32
 *
 * Description:
 *   Put a (possibly unaligned) 32-bit big endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_putbe32(uint8_t *dest, uint32_t val)
{
  /* Big endian means MS halfword first in byte stream */

  usbhost_putbe16(dest, (uint16_t)(val >> 16));
  usbhost_putbe16(dest+2, (uint16_t)(val & 0xffff));
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
  int result;
  DEBUGASSERT(priv && priv->tdbuffer != NULL);

  result         = DRVR_FREE(priv->drvr, priv->tdbuffer);
  priv->tdbuffer = NULL;
  priv->tdbuflen = 0;
  return result;
}

/****************************************************************************
 * Name: usbhost_cbwalloc
 *
 * Description:
 *   Initialize a CBW (re-using the allocated transfer buffer). Upon
 *   successful return, the CBW is cleared and has the CBW signature in place.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static FAR struct usbstrg_cbw_s *usbhost_cbwalloc(FAR struct usbhost_state_s *priv)
{
  FAR struct usbstrg_cbw_s *cbw = NULL;

  DEBUGASSERT(priv->tdbuffer && priv->tdbuflen >= sizeof(struct usbstrg_cbw_s))

  /* Intialize the CBW sructure */

  cbw = (FAR struct usbstrg_cbw_s *)priv->tdbuffer;
  memset(cbw, 0, sizeof(struct usbstrg_cbw_s));
  usbhost_putle32(cbw->signature, USBSTRG_CBW_SIGNATURE);
  return cbw;
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
 *   This function is probably called on the same thread that called the driver
 *   enumerate() method.  However, this function may also be called from an
 *   interrupt handler.
 *
 ****************************************************************************/

static int usbhost_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)class;
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  int remaining;
  uint8_t found = 0;

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
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);
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
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

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
                    priv->bulkout.in           = 0;
                    priv->bulkout.funcaddr     = funcaddr;
                    priv->bulkout.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);
                    uvdbg("Bulk OUT EP addr:%d mxpacketsize:%d\n",
                          priv->bulkout.addr, priv->bulkout.mxpacketsize);
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
                    priv->bulkin.in            = 1;
                    priv->bulkin.funcaddr      = funcaddr;
                    priv->bulkin.mxpacketsize  = usbhost_getle16(epdesc->mxpacketsize);
                    uvdbg("Bulk IN EP addr:%d mxpacketsize:%d\n",
                          priv->bulkin.addr, priv->bulkin.mxpacketsize);
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

  usbhost_work(priv, usbhost_initvolume);
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
 * Assumptions:
 *   This function may be called from an interrupt handler.
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

      DEBUGASSERT(priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);
      usbhost_takesem(&priv->exclsem);
      priv->crefs++;
      usbhost_givesem(&priv->exclsem);
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

  DEBUGASSERT(priv->crefs > 1);
  usbhost_takesem(&priv->exclsem);
  priv->crefs--;

  /* Release the semaphore.  The following operations when crefs == 1 are
   * safe because we know that there is no outstanding open references to
   * the block driver.
   */

  usbhost_givesem(&priv->exclsem);

  /* Check if the USB mass storage device is still connected.  If the
   * storage device is not connected and the reference count just
   * decremented to one, then unregister the block driver and free
   * the class instance.
   */

  if (priv->crefs <= 1 && priv->drvr == NULL)
    {
      /* Destroy the class instance */

      DEBUGASSERT(priv->crefs == 1);
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
  int result;

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
      FAR struct usbstrg_cbw_s *cbw;

      usbhost_takesem(&priv->exclsem);

      /* Assume allocation failure */

      ret = -ENOMEM;

      /* Initialize a CBW (re-using the allocated transfer buffer) */
 
      cbw = usbhost_cbwalloc(priv);
      if (cbw)
        {
          /* Assume some device failure */

          ret = -ENODEV;

          /* Construct and send the CBW */
 
          usbhost_readcbw(startsector, priv->blocksize, nsectors, cbw);
          result = DRVR_TRANSFER(priv->drvr, &priv->bulkout,
                                 (uint8_t*)cbw, USBSTRG_CBW_SIZEOF);
          if (result == OK)
            {
              /* Receive the user data */

              result = DRVR_TRANSFER(priv->drvr, &priv->bulkin,
                                     buffer, priv->blocksize * nsectors);
              if (result == OK)
                {
                  /* Receive the CSW */

                  result = DRVR_TRANSFER(priv->drvr, &priv->bulkin,
                                         priv->tdbuffer, USBSTRG_CSW_SIZEOF);
                  if (result == OK)
                    {
                      FAR struct usbstrg_csw_s *csw;

                      /* Check the CSW status */

                      csw = (FAR struct usbstrg_csw_s *)priv->tdbuffer;
                      if (csw->status == 0)
                        {
                          ret = nsectors;
                        }
                    }
                }
            }
        }

      usbhost_givesem(&priv->exclsem);
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
  ssize_t ret;
  int result;

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
      FAR struct usbstrg_cbw_s *cbw;

      usbhost_takesem(&priv->exclsem);

     /* Assume allocation failure */

      ret = -ENOMEM;

      /* Initialize a CBW (re-using the allocated transfer buffer) */
 
      cbw = usbhost_cbwalloc(priv);
      if (cbw)
        {
          /* Assume some device failure */

          ret = -ENODEV;

          /* Construct and send the CBW */
 
          usbhost_writecbw(startsector, priv->blocksize, nsectors, cbw);
          result = DRVR_TRANSFER(priv->drvr, &priv->bulkout,
                                 (uint8_t*)cbw, USBSTRG_CBW_SIZEOF);
          if (result == OK)
            {
              /* Send the user data */

              result = DRVR_TRANSFER(priv->drvr, &priv->bulkout,
                                     (uint8_t*)buffer, priv->blocksize * nsectors);
              if (result == OK)
                {
                  /* Receive the CSW */

                  result = DRVR_TRANSFER(priv->drvr, &priv->bulkin,
                                         priv->tdbuffer, USBSTRG_CSW_SIZEOF);
                  if (result == OK)
                    {
                      FAR struct usbstrg_csw_s *csw;

                      /* Check the CSW status */

                      csw = (FAR struct usbstrg_csw_s *)priv->tdbuffer;
                      if (csw->status == 0)
                        {
                          ret = nsectors;
                        }
                    }
                }
            }
        }

      usbhost_givesem(&priv->exclsem);
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

  priv = (FAR struct usbhost_state_s *)inode->i_private;
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

      usbhost_takesem(&priv->exclsem);

      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
#ifdef CONFIG_FS_WRITABLE
      geometry->geo_writeenabled  = true;
#else
      geometry->geo_writeenabled  = false;
#endif
      geometry->geo_nsectors      = priv->nblocks;
      geometry->geo_sectorsize    = priv->blocksize;
      usbhost_givesem(&priv->exclsem);

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

      usbhost_takesem(&priv->exclsem);
      switch (cmd)
        {
        /* Add support for ioctl commands here */

        default:
          ret = -ENOTTY;
          break;
        }
      usbhost_givesem(&priv->exclsem);
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

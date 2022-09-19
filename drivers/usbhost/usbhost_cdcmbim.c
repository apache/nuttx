/****************************************************************************
 * drivers/usbhost/usbhost_cdcmbim.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <poll.h>
#include <fcntl.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/netdev.h>

#include <nuttx/usb/cdc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>

#define CDCMBIM_NETBUF_SIZE 8192

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* put in cdc.h */

#define USB_CDC_SEND_ENCAPSULATED_COMMAND 0x00
#define USB_CDC_GET_ENCAPSULATED_RESPONSE 0x01
#define USB_CDC_GET_NTB_PARAMETERS        0x80
#define USB_CDC_SET_NTB_INPUT_SIZE        0x86
#define USB_CDC_SET_NTB_FORMAT            0x83
#define USB_CDC_SET_CRC_MODE              0x8a
#define USB_CDC_SET_MAX_DATAGRAM_SIZE     0x88

#define USB_CDC_NCM_NTB16_SUPPORTED       (1 << 0)
#define USB_CDC_NCM_NTB32_SUPPORTED       (1 << 1)
#define USB_CDC_NCM_NTB16_FORMAT          0x00
#define USB_CDC_NCM_NTB32_FORMAT          0x01
#define USB_CDC_NCM_DATAGRAM_FORMAT_CRC   0x30
#define USB_CDC_NCM_DATAGRAM_FORMAT_NOCRC 0x31
#define USB_CDC_NCM_CRC_NOT_APPENDED      0x00
#define USB_CDC_NCM_CRC_APPENDED          0x01
#define USB_CDC_NCM_NTH16_SIGNATURE       0x484D434E

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

#ifndef CONFIG_USBHOST_ASYNCH
#  warning Asynchronous transfer support is required (CONFIG_USBHOST_ASYNCH)
#endif

#ifdef CONFIG_USBHOST_CDCMBIM_NTDELAY
#  define USBHOST_CDCMBIM_NTDELAY MSEC2TICK(CONFIG_USBHOST_CDCMBIM_NTDELAY)
#else
#  define USBHOST_CDCMBIM_NTDELAY MSEC2TICK(200)
#endif

#ifndef CONFIG_USBHOST_CDCMBIM_NPOLLWAITERS
#  define CONFIG_USBHOST_CDCMBIM_NPOLLWAITERS 1
#endif

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/cdc-wdm[n] device driver path.
 * It defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "/dev/cdc-wdm%d"
#define DEV_NAMELEN         16

/* Used in usbhost_cfgdesc() */

#define USBHOST_CTRLIFFOUND 0x01
#define USBHOST_DATAIFFOUND 0x02
#define USBHOST_INTRIFFOUND 0x04
#define USBHOST_BINFOUND    0x08
#define USBHOST_BOUTFOUND   0x10
#define USBHOST_ALLFOUND    0x1f

#define USBHOST_MAX_CREFS   0x7fff

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct usb_cdc_ncm_nth16_s
{
  uint8_t signature[4];
  uint8_t length[2];
  uint8_t sequence[2];
  uint8_t block_length[2];
  uint8_t ndp_index[2];
};

struct usb_cdc_ncm_dpe16_s
{
  uint8_t index[2];
  uint8_t length[2];
};

struct usb_cdc_ncm_ndp16_s
{
  uint8_t signature[4];
  uint8_t length[2];
  uint8_t next_ndp_index[2];
  struct usb_cdc_ncm_dpe16_s dpe16[0];
};

struct usb_cdc_ncm_ntb_params_s
{
  uint8_t length[2];
  uint8_t ntb_formats_supported[2];
  uint8_t ntb_in_max_size[4];
  uint8_t ndp_in_divisor[2];
  uint8_t ndp_in_payload_remainder[2];
  uint8_t ndp_in_alignment[2];
  uint8_t reserved[2];
  uint8_t ntb_out_max_size[4];
  uint8_t ndp_out_divisor[2];
  uint8_t ndp_out_payload_remainder[2];
  uint8_t ndp_out_alignment[2];
  uint8_t ntb_out_max_datagrams[2];
};

struct usb_csifdesc_s
{
  uint8_t len;
  uint8_t type;
  uint8_t subtype;
};

struct usb_mbim_desc_s
{
  uint8_t len;
  uint8_t type;
  uint8_t subtype;
  uint8_t mbim_version[2];
  uint8_t max_ctrl_message[2];
  uint8_t num_filters;
  uint8_t max_filter_size;
  uint8_t max_segment_size[2];
  uint8_t network_capabilities;
};

/* This structure contains the internal, private state of the USB host class
 * driver.
 */

struct usbhost_cdcmbim_s
{
  /* This is the externally visible portion of the state */

  struct usbhost_class_s  usbclass;

  /* The remainder of the fields are provided to the class driver */

  uint8_t                 minor;        /* Minor number identifying the /dev/cdc-wdm[n] device */
  volatile bool           disconnected; /* TRUE: Device has been disconnected */
  uint16_t                ctrlif;       /* Control interface number */
  uint16_t                dataif;       /* Data interface number */
  int16_t                 crefs;        /* Reference count on the driver instance */
  sem_t                   exclsem;      /* Used to maintain mutual exclusive access */
  struct work_s           ntwork;       /* Notification work */
  struct work_s           comm_rxwork;  /* Communication interface RX work */
  struct work_s           bulk_rxwork;
  struct work_s           txpollwork;
  struct work_s           destroywork;
  int16_t                 nnbytes;      /* Number of bytes received in notification */
  int16_t                 bulkinbytes;
  uint16_t                comm_rxlen;   /* Number of bytes in the RX buffer */
  uint16_t                comm_rxmsgs;  /* Number of messages available to be read */
  uint16_t                comm_rxpos;   /* Read position for input buffer */
  uint16_t                maxctrlsize;  /* Maximum size of a ctrl request */
  uint16_t                maxintsize;   /* Maximum size of interrupt IN packet */
  uint32_t                maxntbin;     /* Maximum size of NTB IN message */
  uint32_t                maxntbout;    /* Maximum size of NTB OUT message */
  FAR uint8_t            *ctrlreq;      /* Allocated ctrl request structure */
  FAR uint8_t            *data_txbuf;   /* Allocated TX buffer for network datagrams */
  FAR uint8_t            *data_rxbuf;   /* Allocated RX buffer for network datagrams */
  FAR uint8_t            *comm_rxbuf;   /* Allocated RX buffer comm IN messages */
  FAR uint8_t            *notification; /* Allocated RX buffer for async notifications */
  FAR uint8_t            *rxnetbuf;     /* Allocated RX buffer for NTB frames */
  FAR uint8_t            *txnetbuf;     /* Allocated TX buffer for NTB frames */
  usbhost_ep_t            intin;        /* Interrupt endpoint */
  usbhost_ep_t            bulkin;       /* Bulk IN endpoint */
  usbhost_ep_t            bulkout;      /* Bulk OUT endpoint */
  uint16_t                bulkmxpacket; /* Max packet size for Bulk OUT endpoint */
  uint16_t                ntbseq;       /* NTB sequence number */

  struct pollfd *fds[CONFIG_USBHOST_CDCMBIM_NPOLLWAITERS];

  /* Network device members */

  bool                    bifup;        /* true:ifup false:ifdown */
  struct net_driver_s     netdev;       /* Interface understood by the network */
  uint16_t                txpktbuf[(MAX_NETDEV_PKTSIZE + 1) / 2];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphores */

static void usbhost_takesem(sem_t *sem);
#define usbhost_givesem(s) nxsem_post(s);

/* Memory allocation services */

static inline FAR struct usbhost_cdcmbim_s *usbhost_allocclass(void);
static inline void usbhost_freeclass(FAR struct usbhost_cdcmbim_s *usbclass);

/* Device name management */

static int usbhost_allocdevno(FAR struct usbhost_cdcmbim_s *priv);
static void usbhost_freedevno(FAR struct usbhost_cdcmbim_s *priv);
static inline void usbhost_mkdevname(FAR struct usbhost_cdcmbim_s *priv,
                                     FAR char *devname);

/* Worker thread actions */

static void usbhost_notification_work(FAR void *arg);
static void usbhost_notification_callback(FAR void *arg, ssize_t nbytes);
static void usbhost_rxdata_work(FAR void *arg);
static void usbhost_bulkin_work(FAR void *arg);

static void usbhost_destroy(FAR void *arg);

/* Helpers for usbhost_connect() */

static int usbhost_cfgdesc(FAR struct usbhost_cdcmbim_s *priv,
                           FAR const uint8_t *configdesc, int desclen);
static inline int usbhost_devinit(FAR struct usbhost_cdcmbim_s *priv);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(const uint8_t *val);
static inline void usbhost_putle16(uint8_t *dest, uint16_t val);
static inline uint32_t usbhost_getle32(const uint8_t *val);
static void usbhost_putle32(uint8_t *dest, uint32_t val);

/* Buffer memory management */

static int usbhost_alloc_buffers(FAR struct usbhost_cdcmbim_s *priv);
static void usbhost_free_buffers(FAR struct usbhost_cdcmbim_s *priv);

/* struct usbhost_registry_s methods */

static struct usbhost_class_s
              *usbhost_create(FAR struct usbhost_hubport_s *hport,
                              FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_connect(FAR struct usbhost_class_s *usbclass,
                           FAR const uint8_t *configdesc, int desclen);
static int usbhost_disconnected(FAR struct usbhost_class_s *usbclass);

/* Control interface driver methods */

static ssize_t cdcwdm_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t cdcwdm_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     cdcwdm_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup);

/* NuttX network callback functions */

static int cdcmbim_ifup(struct net_driver_s *dev);
static int cdcmbim_ifdown(struct net_driver_s *dev);
static int cdcmbim_txavail(struct net_driver_s *dev);

/* Network support functions */

static void cdcmbim_receive(struct usbhost_cdcmbim_s *priv, uint8_t *buf,
                            size_t len);

static int cdcmbim_txpoll(struct net_driver_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID information that will  be
 * used to associate the USB class driver to a connected USB device.
 */

static const struct usbhost_id_s g_id =
{
  USB_CLASS_CDC,      /* base */
  CDC_SUBCLASS_MBIM,  /* subclass */
  0,                  /* proto */
  0,                  /* vid */
  0                   /* pid */
};

/* This is the USB host storage class's registry entry */

static struct usbhost_registry_s g_cdcmbim =
{
  NULL,                   /* flink */
  usbhost_create,         /* create */
  1,                      /* nids */
  &g_id                   /* id[] */
};

/* File operations for control channel */

static const struct file_operations cdcwdm_fops =
{
  NULL,          /* open */
  NULL,          /* close */
  cdcwdm_read,   /* read */
  cdcwdm_write,  /* write */
  NULL,          /* seek */
  NULL,          /* ioctl */
  cdcwdm_poll    /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

/* This is a bitmap that is used to allocate device names /dev/cdc-wdm[n]. */

static uint32_t g_devinuse;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int usbhost_ctrl_cmd(FAR struct usbhost_cdcmbim_s *priv,
                            uint8_t type, uint8_t req, uint16_t value,
                            uint16_t iface, uint8_t *payload, uint16_t len)
{
  FAR struct usbhost_hubport_s *hport;
  struct usb_ctrlreq_s *ctrlreq;
  int ret;

  hport = priv->usbclass.hport;

  ctrlreq       = (struct usb_ctrlreq_s *)priv->ctrlreq;
  ctrlreq->type = type;
  ctrlreq->req  = req;

  usbhost_putle16(ctrlreq->value, value);
  usbhost_putle16(ctrlreq->index, iface);
  usbhost_putle16(ctrlreq->len,   len);

  if (type & USB_REQ_DIR_IN)
    {
      ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq, payload);
    }
  else
    {
      ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, payload);
    }

  return ret;
}

static ssize_t usbhost_readmessage(FAR struct usbhost_cdcmbim_s *priv,
                                   FAR char *buffer, size_t buflen)
{
  irqstate_t flags;
  ssize_t ret = -EAGAIN;

  flags = enter_critical_section();

  if (priv->comm_rxlen > 0)
    {
      if (buflen > priv->comm_rxlen)
        {
          buflen = priv->comm_rxlen;
        }

      ret = buflen;

      memcpy(buffer, priv->comm_rxbuf + priv->comm_rxpos, buflen);
      priv->comm_rxlen -= buflen;
      priv->comm_rxpos += buflen;
      if (priv->comm_rxlen == 0)
        {
          priv->comm_rxpos = 0;
        }
    }

  leave_critical_section(flags);
  return ret;
}

static ssize_t cdcwdm_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode;
  FAR struct usbhost_cdcmbim_s *priv;
  ssize_t ret;

  inode = filep->f_inode;
  priv  = inode->i_private;

  usbhost_takesem(&priv->exclsem);

  if (priv->disconnected)
    {
      ret = -ENODEV;
      goto errout;
    }

  ret = usbhost_readmessage(priv, buffer, buflen);
  if (ret < 0)
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto errout;
        }

      /* wait for a message
       * TODO blocking read
       */

      ret = 0;
      goto errout;
    }

  /* If this message has been completely read and there are more
   * messages available, read the next message into the input buffer.
   */

  if (priv->comm_rxlen == 0 && priv->comm_rxmsgs > 0)
    {
      uinfo("Reading next message\n");
      if (work_available(&priv->comm_rxwork))
        {
          work_queue(LPWORK, &priv->comm_rxwork,
                     (worker_t)usbhost_rxdata_work,
                     priv, 0);
        }
    }

errout:
  usbhost_givesem(&priv->exclsem);
  return ret;
}

static ssize_t cdcwdm_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode;
  FAR struct usbhost_cdcmbim_s *priv;
  int ret;

  inode = filep->f_inode;
  priv  = inode->i_private;

  if (priv->disconnected)
    {
      return -ENODEV;
    }

  if (buflen > priv->maxctrlsize)
    {
      buflen = priv->maxctrlsize;
    }

  /* Make sure that we have exclusive access to the private data
   * structure. There may now be other tasks with the character driver
   * open and actively trying to interact with the class driver.
   */

  usbhost_takesem(&priv->exclsem);

  ret = usbhost_ctrl_cmd(priv,
                         USB_REQ_DIR_OUT | USB_REQ_TYPE_CLASS |
                         USB_REQ_RECIPIENT_INTERFACE,
                         USB_CDC_SEND_ENCAPSULATED_COMMAND,
                         0, priv->ctrlif, (uint8_t *)buffer, buflen);

  usbhost_givesem(&priv->exclsem);

  if (ret)
    {
      return ret;
    }

  uinfo("wrote %u bytes\n", buflen);

  return buflen;
}

/****************************************************************************
 * Name: usbhost_poll
 *
 * Description:
 *   Standard character driver poll method.
 *
 ****************************************************************************/

static int cdcwdm_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  FAR struct inode             *inode;
  FAR struct usbhost_cdcmbim_s *priv;
  int                           ret = OK;
  int                           i;

  DEBUGASSERT(filep && filep->f_inode && fds);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv);
  usbhost_takesem(&priv->exclsem);

  if (priv->disconnected)
    {
      ret = -ENODEV;
    }
  else if (setup)
    {
      /* This is a request to set up the poll.  Find an available slot for
       * the poll structure reference
       */

      for (i = 0; i < CONFIG_USBHOST_CDCMBIM_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_USBHOST_CDCMBIM_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? Notify
       * the POLLIN event if there is a buffered message.
       */

      if (priv->comm_rxlen > 0)
        {
          poll_notify(priv->fds, CONFIG_USBHOST_CDCMBIM_NPOLLWAITERS,
                      POLLIN);
        }
    }
  else
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot);

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

errout:
  nxsem_post(&priv->exclsem);
  return ret;
}

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
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(sem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
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
 * Returned Value:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s.  NULL is returned on failure; this function will
 *   will fail only if there are insufficient resources to create another
 *   USB host class instance.
 *
 ****************************************************************************/

static inline FAR struct usbhost_cdcmbim_s *usbhost_allocclass(void)
{
  FAR struct usbhost_cdcmbim_s *priv;

  DEBUGASSERT(!up_interrupt_context());
  priv = (FAR struct usbhost_cdcmbim_s *)kmm_malloc(
                                         sizeof(struct usbhost_cdcmbim_s));
  uinfo("Allocated: %p\n", priv);
  return priv;
}

/****************************************************************************
 * Name: usbhost_freeclass
 *
 * Description:
 *   Free a class instance previously allocated by usbhost_allocclass().
 *
 * Input Parameters:
 *   usbclass - A reference to the class instance to be freed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void usbhost_freeclass(FAR struct usbhost_cdcmbim_s *usbclass)
{
  DEBUGASSERT(usbclass != NULL);

  /* Free the class instance (perhaps calling sched_kmm_free() in case we are
   * executing from an interrupt handler.
   */

  uinfo("Freeing: %p\n", usbclass);
  kmm_free(usbclass);
}

/****************************************************************************
 * Name: Device name management
 *
 * Description:
 *   Some tiny functions to coordinate management of device names.
 *
 ****************************************************************************/

static int usbhost_allocdevno(FAR struct usbhost_cdcmbim_s *priv)
{
  irqstate_t flags;
  int devno;

  flags = enter_critical_section();
  for (devno = 0; devno < 32; devno++)
    {
      uint32_t bitno = 1 << devno;
      if ((g_devinuse & bitno) == 0)
        {
          g_devinuse |= bitno;
          priv->minor = devno;
          leave_critical_section(flags);
          return OK;
        }
    }

  leave_critical_section(flags);
  return -EMFILE;
}

static void usbhost_freedevno(FAR struct usbhost_cdcmbim_s *priv)
{
  int devno = priv->minor;

  if (devno >= 0 && devno < 26)
    {
      irqstate_t flags = enter_critical_section();
      g_devinuse &= ~(1 << devno);
      leave_critical_section(flags);
    }
}

static inline void usbhost_mkdevname(FAR struct usbhost_cdcmbim_s *priv,
                                     FAR char *devname)
{
  snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->minor);
}

static void usbhost_bulkin_callback(FAR void *arg, ssize_t nbytes)
{
  struct usbhost_cdcmbim_s *priv = (struct usbhost_cdcmbim_s *)arg;
  uint32_t delay = 0;

  DEBUGASSERT(priv);

  if (priv->disconnected)
    {
      return;
    }

  priv->bulkinbytes = (int16_t)nbytes;

  if (nbytes < 0)
    {
      if (nbytes != -EAGAIN)
        {
          uerr("ERROR: Transfer failed: %d\n", nbytes);
        }

      delay = MSEC2TICK(30);
    }

  if (work_available(&priv->bulk_rxwork))
    {
      work_queue(LPWORK, &priv->bulk_rxwork,
                 usbhost_bulkin_work, priv, delay);
    }
}

static void usbhost_bulkin_work(FAR void *arg)
{
  struct usbhost_cdcmbim_s *priv;
  struct usbhost_hubport_s *hport;
  struct usb_cdc_ncm_nth16_s *nth;
  uint16_t ndpoffset;
  uint16_t dgram_len;
  uint16_t dgram_off;
  uint16_t block_len;

  priv = (struct usbhost_cdcmbim_s *)arg;
  DEBUGASSERT(priv);

  hport = priv->usbclass.hport;
  DEBUGASSERT(hport);

  if (priv->disconnected || !priv->bifup)
    {
      return;
    }

  usbhost_takesem(&priv->exclsem);

  if (priv->bulkinbytes < (int16_t)(sizeof(struct usb_cdc_ncm_nth16_s) +
                                    sizeof(struct usb_cdc_ncm_ndp16_s)))
    {
      goto out;
    }

  /* Parse the NTB header */

  nth = (struct usb_cdc_ncm_nth16_s *)priv->rxnetbuf;

  if (usbhost_getle32(nth->signature) != USB_CDC_NCM_NTH16_SIGNATURE)
    {
      uerr("Invalid NTH signature\n");
      goto out;
    }

  block_len = usbhost_getle16(nth->block_length);

  if (block_len > priv->bulkinbytes)
    {
      uerr("Block length larger than rx buffer\n");
      goto out;
    }

  ndpoffset = usbhost_getle16(nth->ndp_index);
  if (ndpoffset > priv->bulkinbytes)
    {
      uerr("NDP offset too far %u > %u\n", ndpoffset, priv->bulkinbytes);
      goto out;
    }

  /* Parse each NDP */

  do
    {
      struct usb_cdc_ncm_dpe16_s *dpe;
      struct usb_cdc_ncm_ndp16_s *ndp
        = (struct usb_cdc_ncm_ndp16_s *)(priv->rxnetbuf + ndpoffset);

      ndpoffset = usbhost_getle16(ndp->next_ndp_index);

      /* Parse each DPE */

      for (dpe = ndp->dpe16; usbhost_getle16(dpe->index); dpe++)
        {
          dgram_off = usbhost_getle16(dpe->index);
          dgram_len = usbhost_getle16(dpe->length);

          if (dgram_off + dgram_len <= priv->bulkinbytes)
            {
              cdcmbim_receive(priv, priv->rxnetbuf + dgram_off, dgram_len);
            }
        }
    }
  while (ndpoffset);

out:
    DRVR_ASYNCH(hport->drvr, priv->bulkin,
                (uint8_t *)priv->rxnetbuf, CDCMBIM_NETBUF_SIZE,
                usbhost_bulkin_callback, priv);
    usbhost_givesem(&priv->exclsem);
}

/****************************************************************************
 * Name: usbhost_rxdata_work
 *
 * Description:
 *   Read an available comm message into the RX buffer
 *
 * Input Parameters:
 *   arg - Driver private data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   None
 *
 ****************************************************************************/

static void usbhost_rxdata_work(FAR void *arg)
{
  /* Would be nice if CTRLIN would return how many bytes it read... */

  struct mbim_header_s
  {
    uint8_t type[4];
    uint8_t len[4];
  };

  FAR struct usbhost_cdcmbim_s *priv;
  struct mbim_header_s *hdr;
  uint32_t len;
  int ret;

  priv = (FAR struct usbhost_cdcmbim_s *)arg;
  DEBUGASSERT(priv);

  /* Are we still connected? */

  if (priv->disconnected)
    {
      return;
    }

  usbhost_takesem(&priv->exclsem);

  ret = usbhost_ctrl_cmd(priv,
                         USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS |
                         USB_REQ_RECIPIENT_INTERFACE,
                         USB_CDC_GET_ENCAPSULATED_RESPONSE,
                         0, priv->ctrlif, priv->comm_rxbuf,
                         priv->maxctrlsize);
  if (ret)
    {
      uerr("Failed to read message: %d\n", ret);
      goto errout;
    }

  hdr = (struct mbim_header_s *)priv->comm_rxbuf;
  len = usbhost_getle32(hdr->len);

  if (len > priv->maxctrlsize)
    {
      uerr("Read invalid MBIM packet\n");
      goto errout;
    }

  uinfo("Read MBIM message %u bytes\n", len);

  priv->comm_rxlen = len;
  priv->comm_rxmsgs--;

  /* Notify any poll waiters we have data */

  poll_notify(priv->fds, CONFIG_USBHOST_CDCMBIM_NPOLLWAITERS, POLLIN);

errout:
  usbhost_givesem(&priv->exclsem);
}

/****************************************************************************
 * Name: usbhost_notification_work
 *
 * Description:
 *   Handle receipt of an asynchronous notification from the CDC device
 *
 * Input Parameters:
 *   arg - The argument provided with the asynchronous I/O was setup
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Probably called from an interrupt handler.
 *
 ****************************************************************************/

static void usbhost_notification_work(FAR void *arg)
{
  FAR struct usbhost_cdcmbim_s *priv;
  FAR struct usbhost_hubport_s *hport;
  FAR struct cdc_notification_s *inmsg;
  int ret;

  priv = (FAR struct usbhost_cdcmbim_s *)arg;
  DEBUGASSERT(priv);

  hport = priv->usbclass.hport;
  DEBUGASSERT(hport);

  /* Are we still connected? */

  if (!priv->disconnected && priv->intin)
    {
      /* Yes.. Was an interrupt IN message received correctly? */

      if (priv->nnbytes >= 0)
        {
          /* Yes.. decode the notification */

          inmsg = (FAR struct cdc_notification_s *)priv->notification;

          /* We care only about the ResponseAvailable notification */

          if ((inmsg->type         == (USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS |
                                       USB_REQ_RECIPIENT_INTERFACE)) &&
              (inmsg->notification == ACM_RESPONSE_AVAILABLE))
            {
              priv->comm_rxmsgs++;

              /* If this is the only message available, read it */

              if (priv->comm_rxmsgs == 1)
                {
                  if (work_available(&priv->comm_rxwork))
                    {
                      work_queue(LPWORK, &priv->comm_rxwork,
                                 usbhost_rxdata_work,
                                 priv, 0);
                    }
                }
            }
        }

      /* Setup to receive the next notification */

      ret = DRVR_ASYNCH(hport->drvr, priv->intin,
                        (FAR uint8_t *)priv->notification,
                        SIZEOF_NOTIFICATION_S(0),
                        usbhost_notification_callback,
                        priv);
      if (ret < 0)
        {
          uerr("ERROR: DRVR_ASYNCH failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: usbhost_notification_callback
 *
 * Description:
 *   Handle receipt of Response Available from the CDC/MBIM device
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

static void usbhost_notification_callback(FAR void *arg, ssize_t nbytes)
{
  FAR struct usbhost_cdcmbim_s *priv = (FAR struct usbhost_cdcmbim_s *)arg;
  uint32_t delay = 0;

  DEBUGASSERT(priv);

  /* Are we still connected? */

  if (!priv->disconnected)
    {
      /* Check for a failure.  On higher end host controllers, the
       * asynchronous transfer will pend until data is available (OHCI and
       * EHCI).  On lower end host controllers (like STM32 and EFM32), the
       * transfer will fail immediately when the device NAKs the first
       * attempted interrupt IN transfer (with nbytes == -EAGAIN).  In that
       * case (or in the case of other errors), we must fall back to
       * polling.
       */

      DEBUGASSERT(nbytes >= INT16_MIN && nbytes <= INT16_MAX);
      priv->nnbytes = (int16_t)nbytes;

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
              uerr("ERROR: Transfer failed: %d\n", nbytes);
            }

          /* We don't know the nature of the failure, but we need to do all
           * that we can do to avoid a CPU hog error loop.
           *
           * Use the low-priority work queue and delay polling for the next
           * event.  We want to use as little CPU bandwidth as possible in
           * this case.
           */

          delay = USBHOST_CDCMBIM_NTDELAY;
        }

      /* Make sure that the work structure available.  There is a remote
       * chance that this may collide with a device disconnection event.
       */

      if (work_available(&priv->ntwork))
        {
          work_queue(LPWORK, &priv->ntwork,
                     usbhost_notification_work,
                     priv, delay);
        }
    }
}

/****************************************************************************
 * Name: usbhost_destroy
 *
 * Description:
 *   The USB device has been disconnected and the reference count on the USB
 *   host class instance has gone to 1.. Time to destroy the USB host class
 *   instance.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_destroy(FAR void *arg)
{
  FAR struct usbhost_cdcmbim_s *priv = (FAR struct usbhost_cdcmbim_s *)arg;
  FAR struct usbhost_hubport_s *hport;
  FAR struct usbhost_driver_s *drvr;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL);
  hport = priv->usbclass.hport;

  DEBUGASSERT(hport->drvr);
  drvr = hport->drvr;

  uinfo("crefs: %d\n", priv->crefs);

  /* Unregister the driver */

  /* Release the device name used by this connection */

  usbhost_freedevno(priv);

  /* Free the endpoints */

  if (priv->intin)
    {
      DRVR_EPFREE(hport->drvr, priv->intin);
    }

  if (priv->bulkin)
    {
      DRVR_EPFREE(hport->drvr, priv->bulkin);
    }

  if (priv->bulkout)
    {
      DRVR_EPFREE(hport->drvr, priv->bulkout);
    }

  /* Free any transfer buffers */

  usbhost_free_buffers(priv);

  /* Free the function address assigned to this device */

  usbhost_devaddr_destroy(hport, hport->funcaddr);
  hport->funcaddr = 0;

  /* Destroy the semaphores */

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(drvr, hport);

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

static int usbhost_cfgdesc(FAR struct usbhost_cdcmbim_s *priv,
                           FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_hubport_s *hport;
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  FAR struct usbhost_epdesc_s bindesc;
  FAR struct usbhost_epdesc_s boutdesc;
  FAR struct usbhost_epdesc_s iindesc;
  int remaining;
  uint8_t found = 0;
  int ret;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport &&
              configdesc != NULL && desclen >= sizeof(struct usb_cfgdesc_s));
  hport = priv->usbclass.hport;

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
        case USB_DESC_TYPE_CSINTERFACE:
          {
            FAR struct usb_csifdesc_s *csdesc = (FAR struct usb_csifdesc_s *)
                                                desc;

            /* MBIM functional descriptor */

            if (csdesc->subtype == CDC_DSUBTYPE_MBIM)
              {
                FAR struct usb_mbim_desc_s *mbim =
                                (FAR struct usb_mbim_desc_s *)desc;

                priv->maxctrlsize = usbhost_getle16(mbim->max_ctrl_message);
                uinfo("MBIM max control size: %u\n", priv->maxctrlsize);
                uinfo("MBIM max segment size: %u\n",
                                usbhost_getle16(mbim->max_segment_size));
              }
          }
          break;

        /* Interface descriptor. We really should get the number of endpoints
         * from this descriptor too.
         */

        case USB_DESC_TYPE_INTERFACE:
          {
            FAR struct usb_ifdesc_s *ifdesc = (FAR struct usb_ifdesc_s *)
                                              configdesc;

            uinfo("Interface descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Is this the control interface? */

            if (ifdesc->classid  == CDC_CLASS_COMM &&
                ifdesc->subclass == CDC_SUBCLASS_MBIM &&
                ifdesc->protocol == CDC_PROTO_NONE)
              {
                priv->ctrlif  = ifdesc->ifno;
                found        |= USBHOST_CTRLIFFOUND;
              }

            /* Is this the data interface? */

            else if (ifdesc->classid  == USB_CLASS_CDC_DATA &&
                     ifdesc->subclass == CDC_SUBCLASS_NONE &&
                     ifdesc->protocol == CDC_DATA_PROTO_NTB)
              {
                priv->dataif  = ifdesc->ifno;
                found        |= USBHOST_DATAIFFOUND;
              }
          }
          break;

        /* Endpoint descriptor.  Here, we expect two bulk endpoints, an IN
         * and an OUT.
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)
                                              configdesc;

            uinfo("Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for interrupt endpoint */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
                USB_EP_ATTR_XFER_INT)
              {
                if (USB_ISEPIN(epdesc->addr))
                  {
                    found |= USBHOST_INTRIFFOUND;
                    iindesc.hport        = hport;
                    iindesc.addr         = epdesc->addr &
                                           USB_EP_ADDR_NUMBER_MASK;
                    iindesc.in           = true;
                    iindesc.xfrtype      = USB_EP_ATTR_XFER_INT;
                    iindesc.interval     = epdesc->interval;
                    iindesc.mxpacketsize =
                                usbhost_getle16(epdesc->mxpacketsize);
                    uinfo("Interrupt IN EP addr:%d mxpacketsize:%d\n",
                          iindesc.addr, iindesc.mxpacketsize);

                    priv->maxintsize = iindesc.mxpacketsize;
                  }
              }

            /* Check for a bulk endpoint. */

            else if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
                     USB_EP_ATTR_XFER_BULK)
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

                    boutdesc.hport        = hport;
                    boutdesc.addr         = epdesc->addr &
                                            USB_EP_ADDR_NUMBER_MASK;
                    boutdesc.in           = false;
                    boutdesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    boutdesc.interval     = epdesc->interval;
                    boutdesc.mxpacketsize =
                                usbhost_getle16(epdesc->mxpacketsize);
                    uinfo("Bulk OUT EP addr:%d mxpacketsize:%d\n",
                          boutdesc.addr, boutdesc.mxpacketsize);

                    priv->bulkmxpacket = boutdesc.mxpacketsize;
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

                    bindesc.hport        = hport;
                    bindesc.addr         = epdesc->addr &
                                           USB_EP_ADDR_NUMBER_MASK;
                    bindesc.in           = 1;
                    bindesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    bindesc.interval     = epdesc->interval;
                    bindesc.mxpacketsize =
                                usbhost_getle16(epdesc->mxpacketsize);
                    uinfo("Bulk IN EP addr:%d mxpacketsize:%d\n",
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
      uerr("ERROR: Found CTRLIF:%s DATAIF: %s BIN:%s BOUT:%s\n",
           (found & USBHOST_CTRLIFFOUND) != 0 ? "YES" : "NO",
           (found & USBHOST_DATAIFFOUND) != 0 ? "YES" : "NO",
           (found & USBHOST_BINFOUND) != 0 ? "YES" : "NO",
           (found & USBHOST_BOUTFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the endpoints */

  ret = DRVR_EPALLOC(hport->drvr, &boutdesc, &priv->bulkout);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate Bulk OUT endpoint\n");
      return ret;
    }

  ret = DRVR_EPALLOC(hport->drvr, &bindesc, &priv->bulkin);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate Bulk IN endpoint\n");
      (void)DRVR_EPFREE(hport->drvr, priv->bulkout);
      return ret;
    }

  ret = DRVR_EPALLOC(hport->drvr, &iindesc, &priv->intin);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate Interrupt IN endpoint\n");
      (void)DRVR_EPFREE(hport->drvr, priv->bulkout);
      (void)DRVR_EPFREE(hport->drvr, priv->bulkin);
      return ret;
    }

  uinfo("Endpoints allocated\n");
  return OK;
}

static int usbhost_setinterface(FAR struct usbhost_cdcmbim_s *priv,
                                uint16_t iface, uint16_t setting)
{
  return usbhost_ctrl_cmd(priv,
                          USB_REQ_DIR_OUT | USB_REQ_RECIPIENT_INTERFACE,
                          USB_REQ_SETINTERFACE, setting, iface, NULL, 0);
}

static int cdc_ncm_set_ntb_input_size(FAR struct usbhost_cdcmbim_s *priv,
                                      uint32_t size)
{
  uint8_t buf[4];

  usbhost_putle32(buf, size);

  return usbhost_ctrl_cmd(priv,
                          USB_REQ_DIR_OUT | USB_REQ_TYPE_CLASS |
                          USB_REQ_RECIPIENT_INTERFACE,
                          USB_CDC_SET_NTB_INPUT_SIZE,
                          0, priv->ctrlif, buf, 4);
}

#if 0
static int cdc_ncm_set_max_dgram_size(FAR struct usbhost_cdcmbim_s *priv,
                                      uint16_t size)
{
  uint8_t buf[2];

  usbhost_putle16(buf, size);

  return usbhost_ctrl_cmd(priv,
                          USB_REQ_DIR_OUT | USB_REQ_TYPE_CLASS |
                          USB_REQ_RECIPIENT_INTERFACE,
                          USB_CDC_SET_MAX_DATAGRAM_SIZE,
                          0, priv->ctrlif, buf, 2);
}
#endif

static int cdc_ncm_read_parameters(FAR struct usbhost_cdcmbim_s *priv)
{
  struct usb_cdc_ncm_ntb_params_s params;
  int ret;

  ret = usbhost_ctrl_cmd(priv,
                         USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS |
                         USB_REQ_RECIPIENT_INTERFACE,
                         USB_CDC_GET_NTB_PARAMETERS,
                         0, priv->ctrlif, (uint8_t *)&params,
                         sizeof(params));
  if (ret == OK)
    {
      priv->maxntbin  = usbhost_getle32(params.ntb_in_max_size);
      priv->maxntbout = usbhost_getle32(params.ntb_out_max_size);
    }

  return ret;
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
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline int usbhost_devinit(FAR struct usbhost_cdcmbim_s *priv)
{
  FAR struct usbhost_hubport_s *hport;
  int ret = OK;

  hport = priv->usbclass.hport;

  /* Increment the reference count.  This will prevent usbhost_destroy() from
   * being called asynchronously if the device is removed.
   */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);

  /* Configure the device */

  /* Set aside transfer buffers for exclusive use by the class driver */

  ret = usbhost_alloc_buffers(priv);
  if (ret)
    {
      uerr("ERROR: failed to allocate buffers\n");
      return ret;
    }

  priv->ntbseq = 0;

  /* Read NCM parameters */

  ret = cdc_ncm_read_parameters(priv);
  if (ret)
    {
      uerr("ERROR: failed to read NCM parameters: %d\n", ret);
    }

  /* Set alternate setting 1 on data interface */

  usbhost_setinterface(priv, priv->dataif, 1);

  /* Set NTB Input size to length of rx buffer */

  ret = cdc_ncm_set_ntb_input_size(priv, CDCMBIM_NETBUF_SIZE);
  if (ret)
    {
      printf("set NTB input size failed: %d\n", ret);
    }

  #if 0
  /* Set max datagram size to MTU */

  ret = cdc_ncm_set_max_dgram_size(priv, 2048);
  if (ret)
    {
      printf("Failed to set max dgram size: %d\n", ret);
    }
  #endif

  /* Register the driver */

  if (ret >= 0)
    {
      char devname[DEV_NAMELEN];

      uinfo("Register character driver\n");
      usbhost_mkdevname(priv, devname);
      ret = register_driver(devname, &cdcwdm_fops, 0666, priv);
    }

  if (priv->intin)
    {
      /* Begin monitoring of message available events */

      uinfo("Start notification monitoring\n");
      ret = DRVR_ASYNCH(hport->drvr, priv->intin,
                        (FAR uint8_t *)priv->notification,
                        SIZEOF_NOTIFICATION_S(0),
                        usbhost_notification_callback,
                        priv);
      if (ret < 0)
        {
          uerr("ERROR: DRVR_ASYNCH failed on intin: %d\n", ret);
        }
    }

  /* Setup the network interface */

  memset(&priv->netdev, 0, sizeof(struct net_driver_s));
  priv->netdev.d_ifup    = cdcmbim_ifup;
  priv->netdev.d_ifdown  = cdcmbim_ifdown;
  priv->netdev.d_txavail = cdcmbim_txavail;
  priv->netdev.d_private = priv;

  /* Register the network device */

  netdev_register(&priv->netdev, NET_LL_MBIM);

  /* Check if we successfully initialized. We now have to be concerned
   * about asynchronous modification of crefs because the character
   * driver has been registered.
   */

  if (ret >= 0)
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
          /* Ready for normal operation */

          uinfo("Successfully initialized\n");
          priv->crefs--;
          usbhost_givesem(&priv->exclsem);
        }
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
 * Name: usbhost_getle32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t usbhost_getle32(const uint8_t *val)
{
  /* Little endian means LS halfword first in byte stream */

  return (uint32_t)usbhost_getle16(&val[2]) << 16 |
                                   (uint32_t)usbhost_getle16(val);
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
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_putle32(uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  usbhost_putle16(dest, (uint16_t)(val & 0xffff));
  usbhost_putle16(dest + 2, (uint16_t)(val >> 16));
}

/****************************************************************************
 * Name: usbhost_alloc_buffers
 *
 * Description:
 *   Allocate transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Value:
 *   On success, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static int usbhost_alloc_buffers(FAR struct usbhost_cdcmbim_s *priv)
{
  FAR struct usbhost_hubport_s *hport;
  size_t maxlen;
  int ret;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL &&
              priv->ctrlreq == NULL);
  hport = priv->usbclass.hport;

  /* Allocate memory for control requests */

  ret = DRVR_ALLOC(hport->drvr, (FAR uint8_t **)&priv->ctrlreq, &maxlen);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_ALLOC of ctrlreq failed: %d\n", ret);
      goto errout;
    }

  DEBUGASSERT(maxlen >= sizeof(struct usb_ctrlreq_s));

  /* Allocate buffer for receiving encapsulated data */

  ret = DRVR_IOALLOC(hport->drvr, &priv->comm_rxbuf, priv->maxctrlsize);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_IOALLOC of comm_rxbuf failed: %d (%d bytes)\n", ret,
           priv->maxctrlsize);
      goto errout;
    }

  /* Allocate buffer for interrupt IN notifications */

  ret = DRVR_IOALLOC(hport->drvr, &priv->notification, priv->maxintsize);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_IOALLOC of notification buf failed: %d (%d bytes)\n",
           ret, priv->maxintsize);
      goto errout;
    }

  ret = DRVR_IOALLOC(hport->drvr, &priv->rxnetbuf, CDCMBIM_NETBUF_SIZE);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_IOALLOC of net rx buf failed: %d (%d bytes)\n",
           ret, 2048);
      goto errout;
    }

  ret = DRVR_IOALLOC(hport->drvr, &priv->txnetbuf, 2048);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_IOALLOC of net tx buf failed: %d (%d bytes)\n",
           ret, 2048);
      goto errout;
    }

  return OK;

errout:
  usbhost_free_buffers(priv);
  return ret;
}

/****************************************************************************
 * Name: usbhost_free_buffers
 *
 * Description:
 *   Free transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_free_buffers(FAR struct usbhost_cdcmbim_s *priv)
{
  FAR struct usbhost_hubport_s *hport;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL);
  hport = priv->usbclass.hport;

  if (priv->ctrlreq)
    {
      (void)DRVR_FREE(hport->drvr, priv->ctrlreq);
    }

  if (priv->comm_rxbuf)
    {
      (void)DRVR_IOFREE(hport->drvr, priv->comm_rxbuf);
    }

  if (priv->notification)
    {
      (void)DRVR_IOFREE(hport->drvr, priv->notification);
    }

  if (priv->rxnetbuf)
    {
      (void)DRVR_IOFREE(hport->drvr, priv->rxnetbuf);
    }

  if (priv->txnetbuf)
    {
      (void)DRVR_IOFREE(hport->drvr, priv->txnetbuf);
    }

  priv->ctrlreq      = NULL;
  priv->comm_rxbuf   = NULL;
  priv->notification = NULL;
  priv->rxnetbuf     = NULL;
  priv->txnetbuf     = NULL;
}

/****************************************************************************
 * struct usbhost_registry_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_create
 *
 * Description:
 *   This function implements the create() method of struct
 *   usbhost_registry_s.
 *   The create() method is a callback into the class implementation.  It is
 *   used to (1) create a new instance of the USB host class state and to (2)
 *   bind a USB host driver "session" to the class instance.  Use of this
 *   create() method will support environments where there may be multiple
 *   USB ports and multiple USB devices simultaneously connected.
 *
 * Input Parameters:
 *   hport - The hub hat manages the new class instance.
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

static FAR struct usbhost_class_s
                  *usbhost_create(FAR struct usbhost_hubport_s *hport,
                                  FAR const struct usbhost_id_s *id)
{
  FAR struct usbhost_cdcmbim_s *priv;

  /* Allocate a USB host class instance */

  priv = usbhost_allocclass();
  if (priv)
    {
      /* Initialize the allocated storage class instance */

      memset(priv, 0, sizeof(struct usbhost_cdcmbim_s));

      /* Assign a device number to this class instance */

      if (usbhost_allocdevno(priv) == OK)
        {
          /* Initialize class method function pointers */

          priv->usbclass.hport        = hport;
          priv->usbclass.connect      = usbhost_connect;
          priv->usbclass.disconnected = usbhost_disconnected;

          /* The initial reference count is 1... One reference is held by
           * the driver.
           */

          priv->crefs = 1;

          /* Initialize semaphores (this works in the interrupt context) */

          nxsem_init(&priv->exclsem, 0, 1);

          /* Return the instance of the USB class driver */

          return &priv->usbclass;
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
 *   usbclass - The USB host class entry previously obtained from a call to
 *     create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration
 *     descriptor.
 *   desclen - The length in bytes of the configuration descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 *   NOTE that the class instance remains valid upon return with a failure.
 *   It is the responsibility of the higher level enumeration logic to call
 *   CLASS_DISCONNECTED to free up the class driver resources.
 *
 * Assumptions:
 *   - This function will *not* be called from an interrupt handler.
 *   - If this function returns an error, the USB host controller driver
 *     must call to DISCONNECTED method to recover from the error
 *
 ****************************************************************************/

static int usbhost_connect(FAR struct usbhost_class_s *usbclass,
                           FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_cdcmbim_s *priv = (FAR struct usbhost_cdcmbim_s *)
                                       usbclass;
  int ret;

  DEBUGASSERT(priv != NULL &&
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the endpoints */

  ret = usbhost_cfgdesc(priv, configdesc, desclen);
  if (ret < 0)
    {
      uerr("ERROR: usbhost_cfgdesc() failed: %d\n", ret);
    }
  else
    {
      /* Now configure the device and register the NuttX driver */

      ret = usbhost_devinit(priv);
      if (ret < 0)
        {
          uerr("ERROR: usbhost_devinit() failed: %d\n", ret);
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
 *   usbclass - The USB host class entry previously obtained from a call to
 *     create().
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

static int usbhost_disconnected(struct usbhost_class_s *usbclass)
{
  FAR struct usbhost_cdcmbim_s *priv = (FAR struct usbhost_cdcmbim_s *)
                                       usbclass;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  /* Set an indication to any users of the device that the device is no
   * longer available.
   */

  flags              = enter_critical_section();
  priv->disconnected = true;

  /* Now check the number of references on the class instance.  If it is one,
   * then we can free the class instance now.  Otherwise, we will have to
   * wait until the holders of the references free them by closing the
   * block driver.
   */

  uinfo("crefs: %d\n", priv->crefs);
  if (priv->crefs == 1)
    {
      /* Destroy the class instance.  If we are executing from an interrupt
       * handler, then defer the destruction to the worker thread.
       * Otherwise, destroy the instance now.
       */

      if (up_interrupt_context())
        {
          /* Destroy the instance on the worker thread. */

          uinfo("Queuing destruction: worker %p->%p\n",
                priv->destroywork.worker, usbhost_destroy);
          DEBUGASSERT(priv->destroywork.worker == NULL);
          work_queue(LPWORK, &priv->destroywork,
                           usbhost_destroy, priv, 0);
        }
      else
        {
          /* Do the work now */

          usbhost_destroy(priv);
        }
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: cdcmbim_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int cdcmbim_transmit(struct usbhost_cdcmbim_s *priv)
{
  struct usbhost_hubport_s *hport;
  struct usb_cdc_ncm_nth16_s *nth;
  struct usb_cdc_ncm_ndp16_s *ndp;
  struct usb_cdc_ncm_dpe16_s *dpe;
  ssize_t ret;
  uint16_t len = 0;

  hport = priv->usbclass.hport;

  uinfo("transmit packet: %d bytes\n", priv->netdev.d_len);

  /* Increment statistics */

  NETDEV_TXPACKETS(&priv->netdev);

  len = sizeof(struct usb_cdc_ncm_nth16_s);
  nth = (struct usb_cdc_ncm_nth16_s *)priv->txnetbuf;

  /* Begin filling NTH */

  usbhost_putle32(nth->signature, USB_CDC_NCM_NTH16_SIGNATURE);
  usbhost_putle16(nth->length, sizeof(struct usb_cdc_ncm_nth16_s));
  usbhost_putle16(nth->sequence, priv->ntbseq++);

  /* Payload directly after NTH */

  memcpy(priv->txnetbuf + len, priv->netdev.d_buf, priv->netdev.d_len);
  len += priv->netdev.d_len;

  /* NDP after payload */

  ndp = (struct usb_cdc_ncm_ndp16_s *)(priv->txnetbuf + len);
  usbhost_putle16(ndp->next_ndp_index, 0);
  usbhost_putle32(ndp->signature, 0x00535049);
  usbhost_putle16(ndp->length, 16); /* NDP + (2 * DPE) */

  /* Fill NDP and block info in NTH */

  usbhost_putle16(nth->ndp_index, len);
  len += 16;
  usbhost_putle16(nth->block_length, len);

  /* Fill first DPE */

  dpe = ndp->dpe16;
  usbhost_putle16(dpe->index, 0x0c);
  usbhost_putle16(dpe->length, priv->netdev.d_len);

  /* NULL DPE */

  dpe++;
  usbhost_putle16(dpe->index, 0);
  usbhost_putle16(dpe->length, 0);

  ret = DRVR_TRANSFER(hport->drvr, priv->bulkout, priv->txnetbuf, len);
  if (ret < 0)
    {
      uerr("transfer returned error: %d\n", ret);
      return ret;
    }

  /* If frame is multiple of wMaxPacketSize we must send a ZLP */

  if ((len % priv->bulkmxpacket) == 0)
    {
      ret = DRVR_TRANSFER(hport->drvr, priv->bulkout,
                          priv->txnetbuf, 0);
      if (ret < 0)
        {
          uerr("ERROR: DRVR_TRANSFER for ZLP failed: %d\n", (int)ret);
        }
    }

  NETDEV_TXDONE(&priv->netdev);
  return OK;
}

/****************************************************************************
 * Name: cdcmbim_receive
 *
 * Description:
 *   Handle a received packet.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static void cdcmbim_receive(struct usbhost_cdcmbim_s *priv,
                            uint8_t *buf, size_t len)
{
  uinfo("received packet: %d len\n", len);

  net_lock();

  NETDEV_RXPACKETS(&priv->netdev);

  /* Any ACK or other response packet generated by the network stack
   * will always be shorter than the received packet, therefore it is
   * safe to pass the received frame buffer directly.
   */

  priv->netdev.d_buf = buf;
  priv->netdev.d_len = len;

  switch (((FAR struct ipv4_hdr_s *)buf)->vhl & IP_VERSION_MASK)
    {
#ifdef CONFIG_NET_IPv4
    case IPv4_VERSION:
      NETDEV_RXIPV4(&priv->netdev);
      ipv4_input(&priv->netdev);

      if (priv->netdev.d_len > 0)
        {
          cdcmbim_transmit(priv);
        }
      break;
#endif
#ifdef CONFIG_NET_IPv6
    case IPv6_VERSION:
      NETDEV_RXIPV6(&priv->netdev);
      ipv6_input(&priv->netdev);

      if (priv->dev.d_len > 0)
        {
          cdcmbim_transmit(priv);
        }
      break;
#endif
    default:
      NETDEV_RXERRORS(dev);
    }

  net_unlock();
}

/****************************************************************************
 * Name: cdcmbim_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int cdcmbim_txpoll(struct net_driver_s *dev)
{
  struct usbhost_cdcmbim_s *priv = (struct usbhost_cdcmbim_s *)
                                   dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  DEBUGASSERT(priv->netdev.d_buf == (FAR uint8_t *)priv->txpktbuf);

  usbhost_takesem(&priv->exclsem);

  if (priv->netdev.d_len > 0)
    {
      if (!devif_loopback(&priv->netdev))
        {
          /* Send the packet */

          cdcmbim_transmit(priv);
        }
    }

  usbhost_givesem(&priv->exclsem);

  return 0;
}

/****************************************************************************
 * Name: cdcmbim_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the MBIM interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int cdcmbim_ifup(struct net_driver_s *dev)
{
  struct usbhost_cdcmbim_s *priv = (struct usbhost_cdcmbim_s *)
                                   dev->d_private;
  struct usbhost_hubport_s *hport = priv->usbclass.hport;
  int ret;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Start RX asynch on bulk in */

  if (priv->bulkin)
    {
      ret = DRVR_ASYNCH(hport->drvr, priv->bulkin,
                        priv->rxnetbuf, CDCMBIM_NETBUF_SIZE,
                        usbhost_bulkin_callback, priv);
      if (ret < 0)
        {
          uerr("ERROR: DRVR_ASYNCH failed on bulkin: %d\n", ret);
        }
    }

  priv->bifup = true;
  return OK;
}

/****************************************************************************
 * Name: cdcmbim_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int cdcmbim_ifdown(struct net_driver_s *dev)
{
  struct usbhost_cdcmbim_s *priv = (struct usbhost_cdcmbim_s *)
                                   dev->d_private;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Mark the device "down" */

  priv->bifup = false;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: cdcmbim_txavail_work
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static void cdcmbim_txavail_work(void *arg)
{
  struct usbhost_cdcmbim_s *priv = (struct usbhost_cdcmbim_s *)arg;

  net_lock();

  priv->netdev.d_buf = (FAR uint8_t *)priv->txpktbuf;

  if (priv->bifup)
    {
      devif_poll(&priv->netdev, cdcmbim_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Name: cdcmbim_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from the network stack with the network locked.
 *
 ****************************************************************************/

static int cdcmbim_txavail(struct net_driver_s *dev)
{
  struct usbhost_cdcmbim_s *priv = (struct usbhost_cdcmbim_s *)
                                   dev->d_private;

  if (work_available(&priv->txpollwork))
    {
      work_queue(LPWORK, &priv->txpollwork, cdcmbim_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_cdcmbim_initialize
 *
 * Description:
 *   Initialize the USB class driver.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host class device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_cdcmbim_initialize(void)
{
  /* Perform any one-time initialization of the class implementation */

  /* Advertise our availability to support CDC MBIM devices */

  return usbhost_registerclass(&g_cdcmbim);
}

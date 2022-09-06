/****************************************************************************
 * drivers/usbhost/usbhost_cdcacm.c
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
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mutex.h>
#include <nuttx/serial/serial.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/cdc.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbhost_devaddr.h>

#ifdef CONFIG_USBHOST_CDCACM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_USBHOST
#  warning USB host support not enabled (CONFIG_USBHOST)
#endif

#ifdef CONFIG_USBHOST_BULK_DISABLE
#  warning USB bulk endpoint support is disabled (CONFIG_USBHOST_BULK_DISABLE)
#endif

#ifdef CONFIG_USBHOST_INT_DISABLE
#  warning USB interrupt endpoint support is disabled (CONFIG_USBHOST_INT_DISABLE)
#endif

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  warning Worker thread support is required (CONFIG_SCHED_WORKQUEUE)
#else
#  ifndef CONFIG_SCHED_HPWORK
#    warning High priority work thread support is required (CONFIG_SCHED_HPWORK)
#  endif
#  ifndef CONFIG_SCHED_LPWORK
#    warning Low priority work thread support is required (CONFIG_SCHED_LPWORK)
#  endif
#  if CONFIG_SCHED_LPNTHREADS < 2
#    warning Multiple low priority work threads recommended for performance (CONFIG_SCHED_LPNTHREADS > 1)
#  endif
#endif

#ifndef CONFIG_USBHOST_ASYNCH
#  warning Asynchronous transfer support is required (CONFIG_USBHOST_ASYNCH)
#endif

#ifndef CONFIG_SERIAL_REMOVABLE
#  warning Removable serial device support is required (CONFIG_SERIAL_REMOVABLE)
#endif

#ifdef CONFIG_USBHOST_CDCACM_NTDELAY
#  define USBHOST_CDCACM_NTDELAY MSEC2TICK(CONFIG_USBHOST_CDCACM_NTDELAY)
#else
#  define USBHOST_CDCACM_NTDELAY MSEC2TICK(200)
#endif

#ifdef CONFIG_USBHOST_CDCACM_RXDELAY
#  define USBHOST_CDCACM_RXDELAY MSEC2TICK(CONFIG_USBHOST_CDCACM_RXDELAY)
#else
#  define USBHOST_CDCACM_RXDELAY MSEC2TICK(200)
#endif

#ifdef CONFIG_USBHOST_CDCACM_TXDELAY
#  define USBHOST_CDCACM_TXDELAY MSEC2TICK(CONFIG_USBHOST_CDCACM_TXDELAY)
#else
#  define USBHOST_CDCACM_TXDELAY MSEC2TICK(200)
#endif

/* Supported protocol */

#define HAVE_CLASS_REQUESTS 1
#define HAVE_INTIN_ENDPOINT 1
#define HAVE_CTRL_INTERFACE 1

#if defined(CONFIG_USBHOST_CDCACM_REDUCED)
#  undef CONFIG_USBHOST_CDCACM_BULKONLY
#  undef CONFIG_USBHOST_CDCACM_COMPLIANT
#  undef HAVE_INTIN_ENDPOINT
#elif defined(CONFIG_USBHOST_CDCACM_BULKONLY)
#  undef CONFIG_USBHOST_CDCACM_COMPLIANT
#  undef HAVE_CLASS_REQUESTS
#  undef HAVE_INTIN_ENDPOINT
#  undef HAVE_CTRL_INTERFACE
#endif

/* If the create() method is called by the USB host device driver from an
 * interrupt handler, then it will be unable to call kmm_malloc() in order to
 * allocate a new class instance.  If the create() method is called from the
 * interrupt level, then class instances must be pre-allocated.
 */

#ifndef CONFIG_USBHOST_CDCACM_NPREALLOC
#  define CONFIG_USBHOST_CDCACM_NPREALLOC 0
#endif

#if CONFIG_USBHOST_CDCACM_NPREALLOC > 32
#  error Currently limited to 32 devices /dev/ttyACM[n]
#endif

#ifndef CONFIG_USBHOST_CDCACM_RXBUFSIZE
#  define CONFIG_USBHOST_CDCACM_RXBUFSIZE 128
#endif

#ifndef CONFIG_USBHOST_CDCACM_TXBUFSIZE
#  define CONFIG_USBHOST_CDCACM_TXBUFSIZE 128
#endif

/* Initial line coding */

#ifndef CONFIG_USBHOST_CDCACM_BAUD
#  define CONFIG_USBHOST_CDCACM_BAUD 115200
#endif

#ifndef CONFIG_USBHOST_CDCACM_PARITY
#  define CONFIG_USBHOST_CDCACM_PARITY 0
#endif

#ifndef CONFIG_USBHOST_CDCACM_BITS
#  define CONFIG_USBHOST_CDCACM_BITS 8
#endif

#ifndef CONFIG_USBHOST_CDCACM_2STOP
#  define CONFIG_USBHOST_CDCACM_2STOP 0
#endif

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/sd[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT             "/dev/ttyACM%d"
#define DEV_NAMELEN            16

#define MAX_NOTIFICATION       32

/* Used in usbhost_connect() */

#define USBHOST_DATAIF_FOUND   0x01      /* Data interface found */
#define USBHOST_BULKIN_FOUND   0x02      /* Bulk IN interface found */
#define USBHOST_BULKOUT_FOUND  0x04      /* Bulk OUT interface found */

#if defined(CONFIG_USBHOST_CDCACM_BULKONLY)
#  define USBHOST_MINFOUND     0x07      /* Minimum things needed */
#  define USBHOST_ALLFOUND     0x07      /* All configuration things */
#elif defined(CONFIG_USBHOST_CDCACM_REDUCED)
#  define USBHOST_CTRLIF_FOUND 0x08      /* Control interface found */

#  define USBHOST_MINFOUND     0x07      /* Minimum things needed */
#  define USBHOST_HAVE_CTRLIF  0x08      /* Needed for control interface */
#  define USBHOST_ALLFOUND     0x0f      /* All configuration things */
#else
#  define USBHOST_CTRLIF_FOUND 0x08      /* Control interface found */
#  define USBHOST_INTIN_FOUND  0x10      /* Interrupt IN interface found */

#  define USBHOST_MINFOUND     0x07      /* Minimum things needed */
#  define USBHOST_HAVE_CTRLIF  0x18      /* Needed for control interface */
#  define USBHOST_ALLFOUND     0x1f      /* All configuration things */
#endif

#define USBHOST_MAX_CREFS      INT16_MAX /* Max cref count before signed overflow */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure contains the internal, private state of the USB host
 * CDC/ACM class.
 */

struct usbhost_cdcacm_s
{
  /* This is the externally visible portion of the state.  The usbclass must
   * the first element of the structure.  It is then cast compatible with
   * struct usbhost_cdcacm_s.
   */

  struct usbhost_class_s usbclass;

  /* This is the standard of the lower-half serial interface.  It is not
   * the first element of the structure, but includes a pointer back to the
   * the beginning of this structure.
   */

  struct uart_dev_s uartdev;

  /* The remainder of the fields are provide to the CDC/ACM class */

  volatile bool  disconnected;   /* TRUE: Device has been disconnected */
  bool           stop2;          /* True: 2 stop bits (for line coding) */
  bool           dsr;            /* State of transmission carrier */
  bool           dcd;            /* State of receiver carrier detection */
  bool           txena;          /* True: TX "interrupts" enabled */
  bool           rxena;          /* True: RX "interrupts" enabled */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool           iflow;          /* True: Input flow control (RTS) enabled */
  bool           rts;            /* True: Input flow control is in effect */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  bool           oflow;          /* True: Output flow control (CTS) enabled */
#endif
  uint8_t        minor;          /* Minor number identifying the /dev/ttyACM[n] device */
  uint8_t        dataif;         /* Data interface number */
#ifdef HAVE_CTRL_INTERFACE
  uint8_t        ctrlif;         /* Control interface number */
#endif
  uint8_t        nbits;          /* Number of bits (for line encoding) */
  uint8_t        parity;         /* Parity (for line encoding) */
  uint16_t       pktsize;        /* Allocated size of transfer buffers */
  uint16_t       nrxbytes;       /* Number of bytes in the RX packet buffer */
  uint16_t       rxndx;          /* Index to the next byte in the RX packet buffer */
  int16_t        crefs;          /* Reference count on the driver instance */
  int16_t        nbytes;         /* The number of bytes actually transferred */
  mutex_t        lock;           /* Used to maintain mutual exclusive access */
  struct work_s  ntwork;         /* For asynchronous notification work */
  struct work_s  rxwork;         /* For RX packet work */
  struct work_s  txwork;         /* For TX packet work */
  FAR uint8_t   *ctrlreq;        /* Allocated ctrl request structure */
  FAR uint8_t   *linecode;       /* The allocated buffer for line encoding */
  FAR uint8_t   *notification;   /* The allocated buffer for async notifications */
  FAR uint8_t   *inbuf;          /* Allocated RX buffer for the Bulk IN endpoint */
  FAR uint8_t   *outbuf;         /* Allocated TX buffer for the Bulk OUT endpoint */
  uint32_t       baud;           /* Current baud for line coding */
  usbhost_ep_t   bulkin;         /* Bulk IN endpoint */
  usbhost_ep_t   bulkout;        /* Bulk OUT endpoint */
#ifdef HAVE_INTIN_ENDPOINT
  usbhost_ep_t   intin;          /* Interrupt IN endpoint (optional) */
#endif

  /* This is the serial data buffer */

  char           rxbuffer[CONFIG_USBHOST_CDCACM_RXBUFSIZE];
  char           txbuffer[CONFIG_USBHOST_CDCACM_TXBUFSIZE];
};

/* This is how struct usbhost_cdcacm_s looks to the free list logic */

struct usbhost_freestate_s
{
  FAR struct usbhost_freestate_s *flink;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Memory allocation services */

static FAR struct usbhost_cdcacm_s *usbhost_allocclass(void);
static void usbhost_freeclass(FAR struct usbhost_cdcacm_s *usbclass);

/* Device name management */

static int  usbhost_devno_alloc(FAR struct usbhost_cdcacm_s *priv);
static void usbhost_devno_free(FAR struct usbhost_cdcacm_s *priv);
static inline void usbhost_mkdevname(FAR struct usbhost_cdcacm_s *priv,
              FAR char *devname);

/* CDC/ACM request helpers */

#ifdef HAVE_CTRL_INTERFACE
static int  usbhost_linecoding_send(FAR struct usbhost_cdcacm_s *priv);
#ifdef HAVE_INTIN_ENDPOINT
static void usbhost_notification_work(FAR void *arg);
static void usbhost_notification_callback(FAR void *arg, ssize_t nbytes);
#endif
#endif

/* UART buffer data transfer */

static void usbhost_txdata_work(FAR void *arg);
static void usbhost_rxdata_work(FAR void *arg);

/* Worker thread actions */

static void usbhost_destroy(FAR void *arg);

/* Helpers for usbhost_connect() */

static int  usbhost_cfgdesc(FAR struct usbhost_cdcacm_s *priv,
              FAR const uint8_t *configdesc, int desclen);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(FAR const uint8_t *val);
static inline uint16_t usbhost_getbe16(FAR const uint8_t *val);
static inline void usbhost_putle16(FAR uint8_t *dest, uint16_t val);
#ifdef HAVE_CTRL_INTERFACE
static void usbhost_putle32(FAR uint8_t *dest, uint32_t val);
#endif

/* Transfer descriptor memory management */

static int  usbhost_alloc_buffers(FAR struct usbhost_cdcacm_s *priv);
static void usbhost_free_buffers(FAR struct usbhost_cdcacm_s *priv);

/* struct usbhost_registry_s methods */

static struct usbhost_class_s *usbhost_create(
              FAR struct usbhost_hubport_s *hport,
              FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int  usbhost_connect(FAR struct usbhost_class_s *usbclass,
              FAR const uint8_t *configdesc, int desclen);
static int  usbhost_disconnected(FAR struct usbhost_class_s *usbclass);

/* Serial driver lower-half interfaces */

static int  usbhost_setup(FAR struct uart_dev_s *uartdev);
static void usbhost_shutdown(FAR struct uart_dev_s *uartdev);
static int  usbhost_attach(FAR struct uart_dev_s *uartdev);
static void usbhost_detach(FAR struct uart_dev_s *uartdev);
static int  usbhost_ioctl(FAR struct file *filep, int cmd,
              unsigned long arg);
static void usbhost_rxint(FAR struct uart_dev_s *uartdev, bool enable);
static bool usbhost_rxavailable(FAR struct uart_dev_s *uartdev);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool usbhost_rxflowcontrol(FAR struct uart_dev_s *uartdev,
              unsigned int nbuffered, bool upper);
#endif
static void usbhost_txint(FAR struct uart_dev_s *uartdev, bool enable);
static bool usbhost_txready(FAR struct uart_dev_s *uartdev);
static bool usbhost_txempty(FAR struct uart_dev_s *uartdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID information that will  be
 * used to associate the USB host CDC/ACM class to a connected USB
 * device.
 */

static const struct usbhost_id_s g_id[4] =
{
  {
    USB_CLASS_CDC,          /* base     */
    CDC_SUBCLASS_NONE,      /* subclass */
    CDC_PROTO_NONE,         /* proto    */
    0,                      /* vid      */
    0                       /* pid      */
  },
  {
    USB_CLASS_CDC,          /* base     */
    CDC_SUBCLASS_ACM,       /* subclass */
    CDC_PROTO_ATM,          /* proto    */
    0,                      /* vid      */
    0                       /* pid      */
  },
  {
    USB_CLASS_VENDOR_SPEC,  /* base     */
    CDC_SUBCLASS_NONE,      /* subclass */
    CDC_PROTO_NONE,         /* proto    */
    0x2c7c,                 /* vid      */
    0x0125                  /* pid      */
  },
  {
    USB_CLASS_VENDOR_SPEC,  /* base     */
    CDC_SUBCLASS_ACM,       /* subclass */
    CDC_PROTO_NONE,         /* proto    */
    0x2c7c,                 /* vid      */
    0x0125                  /* pid      */
  }
};

/* This is the USB host CDC/ACM class's registry entry */

static struct usbhost_registry_s g_cdcacm =
{
  NULL,                   /* flink    */
  usbhost_create,         /* create   */
  4,                      /* nids     */
  &g_id[0]                /* id[]     */
};

/* Serial driver lower half interface */

static const struct uart_ops_s g_uart_ops =
{
  usbhost_setup,         /* setup */
  usbhost_shutdown,      /* shutdown */
  usbhost_attach,        /* attach */
  usbhost_detach,        /* detach */
  usbhost_ioctl,         /* ioctl */
  NULL           ,       /* receive */
  usbhost_rxint,         /* rxinit */
  usbhost_rxavailable,   /* rxavailable */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  usbhost_rxflowcontrol, /* rxflowcontrol */
#endif
  NULL,                  /* send */
  usbhost_txint,         /* txinit */
  usbhost_txready,       /* txready */
  usbhost_txempty        /* txempty */
};

/* This is an array of pre-allocated USB host CDC/ACM class instances */

#if CONFIG_USBHOST_CDCACM_NPREALLOC > 0
static struct usbhost_cdcacm_s g_prealloc[CONFIG_USBHOST_CDCACM_NPREALLOC];
#endif

/* This is a list of free, pre-allocated USB host CDC/ACM class instances */

#if CONFIG_USBHOST_CDCACM_NPREALLOC > 0
static FAR struct usbhost_freestate_s *g_freelist;
#endif

/* This is a bitmap that is used to allocate device
 * minor numbers /dev/ttyACM[n].
 */

static uint32_t g_devinuse;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

#if CONFIG_USBHOST_CDCACM_NPREALLOC > 0
static FAR struct usbhost_cdcacm_s *usbhost_allocclass(void)
{
  FAR struct usbhost_freestate_s *entry;
  irqstate_t flags;

  /* We may be executing from an interrupt handler so we need to take one of
   * our pre-allocated class instances from the free list.
   */

  flags = enter_critical_section();
  entry = g_freelist;
  if (entry)
    {
      g_freelist = entry->flink;
    }

  leave_critical_section(flags);
  uinfo("Allocated: %p\n", entry);
  return (FAR struct usbhost_cdcacm_s *)entry;
}
#else
static FAR struct usbhost_cdcacm_s *usbhost_allocclass(void)
{
  FAR struct usbhost_cdcacm_s *priv;

  /* We are not executing from an interrupt handler so we can just call
   * kmm_malloc() to get memory for the class instance.
   */

  DEBUGASSERT(!up_interrupt_context());
  priv = (FAR struct usbhost_cdcacm_s *)
    kmm_malloc(sizeof(struct usbhost_cdcacm_s));

  uinfo("Allocated: %p\n", priv);
  return priv;
}
#endif

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

#if CONFIG_USBHOST_CDCACM_NPREALLOC > 0
static void usbhost_freeclass(FAR struct usbhost_cdcacm_s *usbclass)
{
  FAR struct usbhost_freestate_s *entry =
    (FAR struct usbhost_freestate_s *)usbclass;
  irqstate_t flags;
  DEBUGASSERT(entry != NULL);

  uinfo("Freeing: %p\n", entry);

  /* Just put the pre-allocated class structure back on the freelist */

  flags = enter_critical_section();
  entry->flink = g_freelist;
  g_freelist = entry;
  leave_critical_section(flags);
}
#else
static void usbhost_freeclass(FAR struct usbhost_cdcacm_s *usbclass)
{
  DEBUGASSERT(usbclass != NULL);

  /* Free the class instance (calling kmm_free() in case we are executing
   * from an interrupt handler.
   */

  uinfo("Freeing: %p\n", usbclass);
  kmm_free(usbclass);
}
#endif

/****************************************************************************
 * Name: usbhost_devno_alloc
 *
 * Description:
 *   Allocate a unique /dev/ttyACM[n] minor number in the range 0-31.
 *
 ****************************************************************************/

static int usbhost_devno_alloc(FAR struct usbhost_cdcacm_s *priv)
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

/****************************************************************************
 * Name: usbhost_devno_free
 *
 * Description:
 *   Free a /dev/ttyACM[n] minor number so that it can be used.
 *
 ****************************************************************************/

static void usbhost_devno_free(FAR struct usbhost_cdcacm_s *priv)
{
  int devno = priv->minor;

  if (devno >= 0 && devno < 32)
    {
      irqstate_t flags = enter_critical_section();
      g_devinuse &= ~(1 << devno);
      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: usbhost_mkdevname
 *
 * Description:
 *   Format a /dev/ttyACM[n] device name given a minor number.
 *
 ****************************************************************************/

static inline void usbhost_mkdevname(FAR struct usbhost_cdcacm_s *priv,
                                     FAR char *devname)
{
  snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->minor);
}

/****************************************************************************
 * Name: usbhost_linecoding_send
 *
 * Description:
 *   Format and send the on EP0.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef HAVE_CTRL_INTERFACE
static int usbhost_linecoding_send(FAR struct usbhost_cdcacm_s *priv)
{
  FAR struct usbhost_hubport_s *hport;
  FAR struct cdc_linecoding_s *linecode;
  FAR struct usb_ctrlreq_s *ctrlreq;
  int ret;

  hport = priv->usbclass.hport;
  DEBUGASSERT(hport);

  /* Initialize the line coding structure */

  linecode         = (FAR struct cdc_linecoding_s *)priv->linecode;
  usbhost_putle32(linecode->baud, priv->baud);
  linecode->stop   = (priv->stop2) ? 2 : 0;
  linecode->parity = priv->parity;
  linecode->nbits  = priv->nbits;

  /* Initialize the control request */

  ctrlreq          = (FAR struct usb_ctrlreq_s *)priv->ctrlreq;
  ctrlreq->type    = USB_DIR_OUT | USB_REQ_TYPE_CLASS |
                     USB_REQ_RECIPIENT_INTERFACE;
  ctrlreq->req     = ACM_SET_LINE_CODING;

  usbhost_putle16(ctrlreq->value, 0);
  usbhost_putle16(ctrlreq->index, priv->ctrlif);
  usbhost_putle16(ctrlreq->len,   SIZEOF_CDC_LINECODING);

  /* And send the request */

  ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, priv->linecode);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_CTRLOUT failed: %d\n", ret);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: usbhost_notification_work
 *
 * Description:
 *   Handle receipt of an asynchronous notification from the CDC/ACM device
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

#ifdef HAVE_INTIN_ENDPOINT
static void usbhost_notification_work(FAR void *arg)
{
  FAR struct usbhost_cdcacm_s *priv;
  FAR struct usbhost_hubport_s *hport;
  FAR struct cdc_notification_s *inmsg;
  uint16_t value;
  uint16_t index;
  uint16_t len;
  int ret;

  priv = (FAR struct usbhost_cdcacm_s *)arg;
  DEBUGASSERT(priv);

  hport = priv->usbclass.hport;
  DEBUGASSERT(hport);

  /* Are we still connected? */

  if (!priv->disconnected && priv->intin)
    {
      /* Yes.. Was an interrupt IN message received correctly? */

      if (priv->nbytes >= 0)
        {
          /* Yes.. decode the notification */

          inmsg = (FAR struct cdc_notification_s *)priv->notification;
          value = usbhost_getle16(inmsg->value);
          index = usbhost_getle16(inmsg->index);
          len   = usbhost_getle16(inmsg->len);

          uinfo("type: %02x notification: %02x value: %04x index: %04x "
                "len: %04x\n",
                inmsg->type, inmsg->notification, value, index, len);

          /* We care only about the SerialState notification */

          if ((inmsg->type         == (USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS |
                                       USB_REQ_RECIPIENT_INTERFACE)) &&
              (inmsg->notification == ACM_SERIAL_STATE) &&
              (value               == 0) &&
              (index               == priv->ctrlif) &&
              (len                 == 2))
            {
              uint16_t state = usbhost_getle16(inmsg->data);

              /* And we care only about the state of the DCD and DSR in the
               * the notification.
               */

              priv->dcd = ((state & CDC_UART_RXCARRIER) != 0);
              priv->dsr = ((state & CDC_UART_TXCARRIER) != 0);

              uinfo("ACM_SERIAL_STATE: DCD=%d DSR=%d\n",
                    priv->dcd, priv->dsr);
            }
        }

      /* Setup to receive the next line status change event */

      ret = DRVR_ASYNCH(hport->drvr, priv->intin,
                        (FAR uint8_t *)priv->notification,
                        MAX_NOTIFICATION, usbhost_notification_callback,
                        priv);
      if (ret < 0)
        {
          uerr("ERROR: DRVR_ASYNCH failed: %d\n", ret);
        }
    }
}
#endif

/****************************************************************************
 * Name: usbhost_notification_callback
 *
 * Description:
 *   Handle receipt of line status from the CDC/ACM device
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

#ifdef HAVE_INTIN_ENDPOINT
static void usbhost_notification_callback(FAR void *arg, ssize_t nbytes)
{
  FAR struct usbhost_cdcacm_s *priv = (FAR struct usbhost_cdcacm_s *)arg;
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
      priv->nbytes = (int16_t)nbytes;

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

          delay = USBHOST_CDCACM_NTDELAY;
        }

      /* Make sure that the  work structure available.  There is a remote
       * chance that this may collide with a device disconnection event.
       */

      if (work_available(&priv->ntwork))
        {
          work_queue(HPWORK, &priv->ntwork,
                     usbhost_notification_work,
                     priv, delay);
        }
    }
}
#endif

/****************************************************************************
 * UART buffer data transfer
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_txdata_work
 *
 * Description:
 *   Send more OUT data to the attached CDC/ACM device.
 *
 * Input Parameters:
 *   arg - A reference to the CDC/ACM class private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_txdata_work(FAR void *arg)
{
  FAR struct usbhost_cdcacm_s *priv;
  FAR struct usbhost_hubport_s *hport;
  FAR struct uart_dev_s *uartdev;
  FAR struct uart_buffer_s *txbuf;
  ssize_t nwritten;
  int txndx;
  int txtail;
  int ret;

  priv = (FAR struct usbhost_cdcacm_s *)arg;
  DEBUGASSERT(priv);

  hport = priv->usbclass.hport;
  DEBUGASSERT(hport);

  uartdev = &priv->uartdev;
  txbuf   = &uartdev->xmit;

  /* Do nothing if TX transmission is disabled */

  if (!priv->txena)
    {
      /* Terminate the work now *without* rescheduling */

      return;
    }

  /* Loop until The UART TX buffer is empty (or we become disconnected) */

  txtail = txbuf->tail;
  txndx  = 0;

  while (txtail != txbuf->head && priv->txena && !priv->disconnected)
    {
      /* Copy data from the UART TX buffer until either 1) the UART TX
       * buffer has been emptied, or 2) the Bulk OUT buffer is full.
       */

      txndx = 0;
      while (txtail != txbuf->head && txndx < priv->pktsize)
        {
          /* Copy the next byte */

          priv->outbuf[txndx] = txbuf->buffer[txtail];

          /* Increment counters and indices */

          txndx++;
          if (++txtail >= txbuf->size)
            {
             txtail = 0;
            }
        }

      /* Save the updated tail pointer so that it cannot be sent again */

      txbuf->tail = txtail;

      /* Bytes were removed from the TX buffer.  Inform any waiters that
       * there is space available in the TX buffer.
       */

      uart_datasent(uartdev);

      /* Send the filled TX buffer to the CDC/ACM device */

      nwritten = DRVR_TRANSFER(hport->drvr, priv->bulkout,
                               priv->outbuf, txndx);
      if (nwritten < 0)
        {
          /* The most likely reason for a failure is that CDC/ACM device
           * NAK'ed our packet OR that the device has been disconnected.
           *
           * Just break out of the loop, rescheduling the work (unless
           * the device is disconnected).
           */

          uerr("ERROR: DRVR_TRANSFER for packet failed: %d\n",
               (int)nwritten);
          break;
        }
    }

  /* We get here because: 1) the UART TX buffer is empty and there is
   * nothing more to send, 2) the CDC/ACM device was not ready to accept our
   * data, or the device is no longer available.
   *
   * If the last packet sent was and even multiple of the packet size, then
   * we need to send a zero length packet (ZLP).
   */

  if (txndx == priv->pktsize && !priv->disconnected)
    {
      /* Send the ZLP to the CDC/ACM device */

      nwritten = DRVR_TRANSFER(hport->drvr, priv->bulkout,
                               priv->outbuf, 0);
      if (nwritten < 0)
        {
          /* The most likely reason for a failure is that CDC/ACM device
           * NAK'ed our packet.
           */

          uerr("ERROR: DRVR_TRANSFER for ZLP failed: %d\n", (int)nwritten);
        }
    }

  /* Check again if TX reception is enabled and that the device is still
   * connected.  These states could have changed since we started the
   * transfer.
   */

  if (priv->txena && !priv->disconnected)
    {
      /* Schedule TX data work to occur after a delay. */

      ret = work_queue(LPWORK, &priv->txwork, usbhost_txdata_work, priv,
                       USBHOST_CDCACM_TXDELAY);
      DEBUGASSERT(ret >= 0);
      UNUSED(ret);
    }
}

/****************************************************************************
 * Name: usbhost_rxdata_work
 *
 * Description:
 *   Get more IN data from the attached CDC/ACM device.
 *
 * Input Parameters:
 *   arg - A reference to the CDC/ACM class private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_rxdata_work(FAR void *arg)
{
  FAR struct usbhost_cdcacm_s *priv;
  FAR struct usbhost_hubport_s *hport;
  FAR struct uart_dev_s *uartdev;
  FAR struct uart_buffer_s *rxbuf;
  ssize_t nread;
  int nxfrd;
  int nexthead;
  int rxndx;
  int ret;

  priv    = (FAR struct usbhost_cdcacm_s *)arg;
  DEBUGASSERT(priv);

  hport   = priv->usbclass.hport;
  DEBUGASSERT(hport);

  uartdev = &priv->uartdev;
  rxbuf   = &uartdev->recv;

  /* Get the index in the RX packet buffer where we will take the first
   * byte of data.
   */

  rxndx    = priv->rxndx;
  nxfrd    = 0;

  /* Get the index to the value of the UART RX buffer head AFTER the
   * first character has been stored.  We need to know this in order
   * to test if the UART RX buffer is full.
   */

  nexthead = rxbuf->head + 1;
  if (nexthead >= rxbuf->size)
    {
      nexthead = 0;
    }

  /* Loop until either:
   *
   * 1. The UART RX buffer is full
   * 2. There is no more data available from the CDC/ACM device
   * 3. RX rec
   */

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  while (priv->rxena && priv->rts && !priv->disconnected)
#else
  while (priv->rxena && !priv->disconnected)
#endif
    {
      /* Stop now if there is no room for another
       * character in the RX buffer.
       */

      if (nexthead == rxbuf->tail)
        {
          /* Break out of the loop, rescheduling the work */

          break;
        }

      /* Do we have any buffer RX data to transfer? */

      if (priv->nrxbytes < 1)
        {
          /* No.. Read more data from the CDC/ACM device */

          nread = DRVR_TRANSFER(hport->drvr, priv->bulkin,
                                priv->inbuf, priv->pktsize);
          if (nread < 0)
            {
              /* The most likely reason for a failure is that the has no
               * data available now and NAK'ed the IN token OR that the
               * transfer was cancelled because the device was disconnected.
               *
               * Just break out of the loop, rescheduling the work (if the
               * device was not disconnected.
               */

              uerr("ERROR: DRVR_TRANSFER for packet failed: %d\n",
                   (int)nread);
              break;
            }

          /* Save the number of bytes read.  This might be zero if
           * a Zero Length Packet (ZLP) is received.  The ZLP is
           * part of the bulk transfer protocol, but otherwise of
           * no interest to us.
           */

          priv->nrxbytes = (uint16_t)nread;
          rxndx          = 0;

          /* Ignore ZLPs */

          if (nread == 0)
            {
              continue;
            }
        }

      /* Transfer one byte from the RX packet buffer into UART RX buffer */

      rxbuf->buffer[rxbuf->head] = priv->inbuf[rxndx];
      nxfrd++;

      /* Save the updated indices */

      rxbuf->head = nexthead;
      priv->rxndx = rxndx;

      /* Update the head point for for the next pass through the loop
       * handling. If nexthead incremented to rxbuf->tail, then the
       * RX buffer will and we will exit the loop at the top.
       */

      if (++nexthead >= rxbuf->size)
        {
           nexthead = 0;
        }

      /* Increment the index in the USB IN packet buffer.  If the
       * index becomes equal to the number of bytes in the buffer, then
       * we have consumed all of the RX data.
       */

      if (++rxndx >= priv->nrxbytes)
        {
          /* In that case set the number of bytes in the buffer to zero.
           * This will force re-reading on the next time through the loop.
           */

          priv->nrxbytes = 0;
          priv->rxndx    = 0;

          /* Inform any waiters there there is new incoming data available. */

          uart_datareceived(uartdev);
          nxfrd = 0;
        }
    }

  /* We break out to here:  1) the UART RX buffer is full, 2) the CDC/ACM
   * device is not ready to provide us with more serial data, or 3) the
   * device has been disconnected.
   *
   * Check if the device is still available:  RX enabled, no RX flow
   * control in effect, and that the device is not disconnected. These
   * states could have changed since we started the transfer.
   */

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->rxena && priv->rts && work_available(&priv->rxwork) &&
      !priv->disconnected)
#else
  if (priv->rxena && work_available(&priv->rxwork) && !priv->disconnected)
#endif
    {
      /* Schedule RX data reception work to occur after a delay.  This will
       * affect our responsive in certain cases.  The delayed work, however,
       * will be cancelled and replaced with immediate work when the upper
       * layer demands more data.
       */

      ret = work_queue(LPWORK, &priv->rxwork, usbhost_rxdata_work, priv,
                       USBHOST_CDCACM_RXDELAY);
      DEBUGASSERT(ret >= 0);
      UNUSED(ret);
    }

  /* If any bytes were added to the buffer, inform any waiters there there
   * is new incoming data available.
   */

  if (nxfrd)
    {
      uart_datareceived(uartdev);
    }
}

/****************************************************************************
 * Name: usbhost_destroy
 *
 * Description:
 *   The USB CDC/ACM device has been disconnected and the reference count
 *   on the USB host class instance has gone to 1.. Time to destroy the USB
 *   host class instance.
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
  FAR struct usbhost_cdcacm_s *priv = (FAR struct usbhost_cdcacm_s *)arg;
  FAR struct usbhost_hubport_s *hport;
  char devname[DEV_NAMELEN];

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL);
  hport = priv->usbclass.hport;

  uinfo("crefs: %d\n", priv->crefs);

  /* Unregister the serial lower half driver */

  usbhost_mkdevname(priv, devname);
#warning Missing logic

  /* Release the device name used by this connection */

  usbhost_devno_free(priv);

  /* Free the allocated endpoints */

  if (priv->bulkout)
    {
      DRVR_EPFREE(hport->drvr, priv->bulkout);
    }

  if (priv->bulkin)
    {
      DRVR_EPFREE(hport->drvr, priv->bulkin);
    }

#ifdef HAVE_INTIN_ENDPOINT
  if (priv->intin)
    {
      DRVR_EPFREE(hport->drvr, priv->intin);
    }
#endif

  /* Free any transfer buffers */

  usbhost_free_buffers(priv);

  /* Destroy the mutex */

  nxmutex_destroy(&priv->lock);

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(hport->drvr, hport);

  /* Free the function address assigned to this device */

  usbhost_devaddr_destroy(hport, hport->funcaddr);
  hport->funcaddr = 0;

  /* And free the class instance.  */

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

static int usbhost_cfgdesc(FAR struct usbhost_cdcacm_s *priv,
                           FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_hubport_s *hport;
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  struct usbhost_epdesc_s bindesc;
  struct usbhost_epdesc_s boutdesc;
  struct usbhost_epdesc_s iindesc;
  int remaining;
  uint8_t found  = 0;
  uint8_t currif = 0;
  int ret;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport &&
              configdesc != NULL && desclen >= sizeof(struct usb_cfgdesc_s));
  hport = priv->usbclass.hport;

  /* Keep the compiler from complaining about uninitialized variables */

  memset(&bindesc, 0, sizeof(struct usbhost_epdesc_s));
  memset(&boutdesc, 0, sizeof(struct usbhost_epdesc_s));
  memset(&iindesc, 0, sizeof(struct usbhost_epdesc_s));

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
        /* Interface descriptor. The CDC/ACM device may include two
         * interfaces:
         *
         * 1) A data interface which consists of two endpoints (bulk in +
         *    bulk out) and
         * 2) A control interface which consists of one interrupt in
         *    endpoint.
         */

        case USB_DESC_TYPE_INTERFACE:
          {
            FAR struct usb_ifdesc_s *ifdesc =
              (FAR struct usb_ifdesc_s *)configdesc;

            uinfo("Interface descriptor: class: %d subclass: %d proto: %d\n",
                  ifdesc->classid, ifdesc->subclass, ifdesc->protocol);
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Check for the CDC/ACM data interface */

            if ((ifdesc->classid == USB_CLASS_CDC_DATA ||
                ifdesc->classid == USB_CLASS_VENDOR_SPEC) &&
                (found & USBHOST_DATAIF_FOUND) == 0)
              {
                /* Save the data interface number and mark that the data
                 * interface found has been found.
                 */

                priv->dataif  = ifdesc->ifno;
                found        |= USBHOST_DATAIF_FOUND;
                currif        = USBHOST_DATAIF_FOUND;
              }
#ifdef HAVE_CTRL_INTERFACE
            else if (ifdesc->classid == USB_CLASS_CDC &&
                     (found & USBHOST_CTRLIF_FOUND) == 0)
              {
                /* Otherwise, tentatively assume that this is the control
                 * interface.
                 */

                priv->ctrlif  = ifdesc->ifno;
                currif        = USBHOST_CTRLIF_FOUND;
              }
#endif
            else
              {
                /* Its something else */

                currif        = 0;
              }
          }
          break;

        /* Endpoint descriptor.  We expect two bulk endpoints, an IN and an
         * OUT.
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc =
              (FAR struct usb_epdesc_s *)configdesc;

            uinfo("Endpoint descriptor: currif: %02x attr: %02x\n",
                  currif, epdesc->attr);
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for a bulk endpoint. */

            if (currif == USBHOST_DATAIF_FOUND &&
                (epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
                USB_EP_ATTR_XFER_BULK)
              {
                /* Yes.. it is a bulk endpoint.  IN or OUT? */

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an OUT bulk endpoint.  There should be only one
                     * bulk OUT endpoint.
                     */

                    if ((found & USBHOST_BULKOUT_FOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }

                    found |= USBHOST_BULKOUT_FOUND;

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
                  }
                else
                  {
                    /* It is an IN bulk endpoint.  There should be only one
                     * bulk IN endpoint.
                     */

                    if ((found & USBHOST_BULKIN_FOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }

                    found |= USBHOST_BULKIN_FOUND;

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

#ifdef HAVE_CTRL_INTERFACE
            /* Check for an interrupt IN endpoint. */

            else if (currif == USBHOST_CTRLIF_FOUND &&
                     (epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
                     USB_EP_ATTR_XFER_INT)
              {
                /* Yes.. it is a interrupt endpoint.  IN or OUT? */

                if (USB_ISEPIN(epdesc->addr))
                  {
#ifdef HAVE_INTIN_ENDPOINT
                    /* It is an IN interrupt endpoint.  There should be only
                     * one interrupt IN endpoint.
                     */

                    if ((found & USBHOST_INTIN_FOUND) != 0)
                      {
                        /* Oops.. more than one.  We don't know what to do
                         * with this.
                         */

                        return -EINVAL;
                      }

                    /* Let's pick this interface as the one and only control
                     * interface.
                     */

                    found |= (USBHOST_CTRLIF_FOUND | USBHOST_INTIN_FOUND);

                    /* Save the interrupt IN endpoint information */

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
#else
                    found |= USBHOST_CTRLIF_FOUND;
#endif
                  }
              }
#endif
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

  /* Sanity checking... did we find all of things that we needed for the
   * basic CDC/ACM data interface? NOTE: that the Control interface with
   * the Interrupt IN endpoint is optional.
   */

  if ((found & USBHOST_MINFOUND) != USBHOST_MINFOUND)
    {
      uerr("ERROR: Found DATA IF:%s BULK IN:%s BULK OUT:%s\n",
           (found & USBHOST_DATAIF_FOUND)  != 0  ? "YES" : "NO",
           (found & USBHOST_BULKIN_FOUND)  != 0 ? "YES" : "NO",
           (found & USBHOST_BULKOUT_FOUND) != 0 ? "YES" : "NO");
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
      DRVR_EPFREE(hport->drvr, priv->bulkout);
      return ret;
    }

#ifdef HAVE_INTIN_ENDPOINT
  /* The control interface with interrupt IN endpoint is optional */

  if ((found & USBHOST_HAVE_CTRLIF) == USBHOST_HAVE_CTRLIF)
    {
      ret = DRVR_EPALLOC(hport->drvr, &iindesc, &priv->intin);
      if (ret < 0)
        {
          uerr("ERROR: Failed to allocate Interrupt IN endpoint\n");
          priv->intin = NULL;
        }
    }
#endif

  uinfo("Endpoints allocated\n");
  return OK;
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

static inline uint16_t usbhost_getle16(FAR const uint8_t *val)
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
 * Returned Value:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t usbhost_getbe16(FAR const uint8_t *val)
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
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_putle16(FAR uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
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

#ifdef HAVE_CTRL_INTERFACE
static void usbhost_putle32(FAR uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  usbhost_putle16(dest, (uint16_t)(val & 0xffff));
  usbhost_putle16(dest + 2, (uint16_t)(val >> 16));
}
#endif

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

static int usbhost_alloc_buffers(FAR struct usbhost_cdcacm_s *priv)
{
  FAR struct usbhost_hubport_s *hport;
  size_t maxlen;
  int ret;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL &&
              priv->linecode == NULL);
  hport = priv->usbclass.hport;

  /* Allocate memory for control requests */

  ret = DRVR_ALLOC(hport->drvr, (FAR uint8_t **)&priv->ctrlreq, &maxlen);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_ALLOC of ctrlreq failed: %d\n", ret);
      goto errout;
    }

  DEBUGASSERT(maxlen >= sizeof(struct usb_ctrlreq_s));

  /* Allocate buffer for sending line coding data. */

  ret = DRVR_IOALLOC(hport->drvr, &priv->linecode,
                     sizeof(struct cdc_linecoding_s));
  if (ret < 0)
    {
      uerr("ERROR: DRVR_IOALLOC of line coding failed: %d (%d bytes)\n",
           ret, sizeof(struct cdc_linecoding_s));
      goto errout;
    }

#ifdef HAVE_INTIN_ENDPOINT
  /* Allocate (optional) buffer for receiving line status data. */

  if (priv->intin)
    {
      ret = DRVR_IOALLOC(hport->drvr, &priv->notification, MAX_NOTIFICATION);
      if (ret < 0)
        {
          uerr("ERROR: DRVR_IOALLOC of line status failed: %d (%d bytes)\n",
               ret, MAX_NOTIFICATION);
          goto errout;
        }
    }
#endif

  /* Set the size of Bulk IN and OUT buffers to the max packet size */

  priv->pktsize = (hport->speed == USB_SPEED_HIGH) ? 512 : 64;

  /* Allocate a RX buffer for Bulk IN transfers */

  ret = DRVR_IOALLOC(hport->drvr, &priv->inbuf, priv->pktsize);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_IOALLOC of Bulk IN buffer failed: %d (%d bytes)\n",
           ret, priv->pktsize);
      goto errout;
    }

  /* Allocate a TX buffer for Bulk IN transfers */

  ret = DRVR_IOALLOC(hport->drvr, &priv->outbuf, priv->pktsize);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_IOALLOC of Bulk OUT buffer failed: %d (%d bytes)\n",
           ret, priv->pktsize);
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

static void usbhost_free_buffers(FAR struct usbhost_cdcacm_s *priv)
{
  FAR struct usbhost_hubport_s *hport;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL);
  hport = priv->usbclass.hport;

  if (priv->ctrlreq)
    {
      DRVR_FREE(hport->drvr, priv->ctrlreq);
    }

  if (priv->linecode)
    {
      DRVR_IOFREE(hport->drvr, priv->linecode);
    }

  if (priv->notification)
    {
      DRVR_IOFREE(hport->drvr, priv->notification);
    }

  if (priv->inbuf)
    {
      DRVR_IOFREE(hport->drvr, priv->inbuf);
    }

  if (priv->outbuf)
    {
      DRVR_IOFREE(hport->drvr, priv->outbuf);
    }

  priv->pktsize      = 0;
  priv->ctrlreq      = NULL;
  priv->linecode     = NULL;
  priv->notification = NULL;
  priv->inbuf        = NULL;
  priv->outbuf       = NULL;
}

/****************************************************************************
 * struct usbhost_registry_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_create
 *
 * Description:
 *   This function implements the create() method of struct
 *   usbhost_registry_s.  The create() method is a callback into the class
 *   implementation.  It is used to (1) create a new instance of the USB
 *   host class state and to (2) bind a USB host driver "session" to the
 *   class instance.  Use of this create() method will support environments
 *   where there may be multiple USB ports and multiple USB devices
 *   simultaneously connected.
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
  FAR struct usbhost_cdcacm_s *priv;
  FAR struct uart_dev_s *uartdev;

  /* Allocate a USB host CDC/ACM class instance */

  priv = usbhost_allocclass();
  if (priv)
    {
      /* Initialize the allocated CDC/ACM class instance */

      memset(priv, 0, sizeof(struct usbhost_cdcacm_s));

      /* Assign a device number to this class instance */

      if (usbhost_devno_alloc(priv) == OK)
        {
          /* Initialize class method function pointers */

          priv->usbclass.hport        = hport;
          priv->usbclass.connect      = usbhost_connect;
          priv->usbclass.disconnected = usbhost_disconnected;

          /* The initial reference count is 1...
           * One reference is held by the driver
           */

          priv->crefs = 1;

          /* Initialize mutex
           * (this works okay in the interrupt context)
           */

          nxmutex_init(&priv->lock);

          /* Set up the serial lower-half interface */

          uartdev              = &priv->uartdev;
          uartdev->recv.size   = CONFIG_USBHOST_CDCACM_RXBUFSIZE;
          uartdev->recv.buffer = priv->rxbuffer;
          uartdev->xmit.size   = CONFIG_USBHOST_CDCACM_TXBUFSIZE;
          uartdev->xmit.buffer = priv->txbuffer;
          uartdev->ops         = &g_uart_ops;
          uartdev->priv        = priv;

          /* Set up the initial line status */

          priv->baud           = CONFIG_USBHOST_CDCACM_BAUD;
          priv->nbits          = CONFIG_USBHOST_CDCACM_BITS;
          priv->parity         = CONFIG_USBHOST_CDCACM_PARITY;
          priv->stop2          = CONFIG_USBHOST_CDCACM_2STOP;

#ifdef CONFIG_SERIAL_IFLOWCONTROL
          priv->rts            = true;
#endif

          /* Return the instance of the USB CDC/ACM class */

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
  FAR struct usbhost_cdcacm_s *priv =
    (FAR struct usbhost_cdcacm_s *)usbclass;
#ifdef HAVE_INTIN_ENDPOINT
  FAR struct usbhost_hubport_s *hport;
#endif
  char devname[DEV_NAMELEN];
  int ret;

  DEBUGASSERT(priv != NULL &&
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

#ifdef HAVE_INTIN_ENDPOINT
  hport = priv->usbclass.hport;
  DEBUGASSERT(hport);
#endif

  /* Get exclusive access to the device structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the reference count.  This will prevent usbhost_destroy() from
   * being called asynchronously if the device is removed.
   */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);

  /* Parse the configuration descriptor to get the bulk I/O endpoints */

  ret = usbhost_cfgdesc(priv, configdesc, desclen);
  if (ret < 0)
    {
      uerr("ERROR: usbhost_cfgdesc() failed: %d\n", ret);
      goto errout;
    }

  /* Set aside a transfer buffer for exclusive use by the CDC/ACM driver */

  ret = usbhost_alloc_buffers(priv);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate transfer buffer\n");
      goto errout;
    }

#ifdef HAVE_CTRL_INTERFACE
  /* Send the initial line encoding */

  ret = usbhost_linecoding_send(priv);
  if (ret < 0)
    {
      uerr("ERROR: usbhost_linecoding_send() failed: %d\n", ret);
    }
#endif

  /* Register the lower half serial instance with the upper half serial
   * driver.
   */

  usbhost_mkdevname(priv, devname);
  uinfo("Register device: %s\n", devname);

  ret = uart_register(devname, &priv->uartdev);
  if (ret < 0)
    {
      uerr("ERROR: uart_register() failed: %d\n", ret);
      goto errout;
    }

#ifdef HAVE_INTIN_ENDPOINT
  /* Do we have an interrupt IN endpoint? */

  if (priv->intin)
    {
      /* Begin monitoring of port status change events */

      uinfo("Start notification monitoring\n");
      ret = DRVR_ASYNCH(hport->drvr, priv->intin,
                        (FAR uint8_t *)priv->notification,
                        MAX_NOTIFICATION, usbhost_notification_callback,
                        priv);
      if (ret < 0)
        {
          uerr("ERROR: DRVR_ASYNCH failed: %d\n", ret);
        }
    }
#endif

errout:
  /* Decrement the reference count.  We incremented the reference count
   * above so that usbhost_destroy() could not be called.  We now have to
   * be concerned about asynchronous modification of crefs because the
   * serial driver has been registered.
   */

  DEBUGASSERT(priv->crefs >= 2);
  priv->crefs--;

  /* Release the semaphore... there is a race condition here.
   * Decrementing the reference count and releasing the semaphore
   * allows usbhost_destroy() to execute (on the worker thread);
   * the class driver instance could get destroyed before we are
   * ready to handle it!
   */

  nxmutex_unlock(&priv->lock);
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

static int usbhost_disconnected(FAR struct usbhost_class_s *usbclass)
{
  FAR struct usbhost_cdcacm_s *priv =
    (FAR struct usbhost_cdcacm_s *)usbclass;
  FAR struct usbhost_hubport_s *hport;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL);
  hport = priv->usbclass.hport;

  /* Set an indication to any users of the CDC/ACM device that the device
   * is no longer available.
   */

  flags              = enter_critical_section();
  priv->disconnected = true;

  /* Let the upper half driver know that serial device is no longer
   * connected.
   */

  uart_connected(&priv->uartdev, false);

  /* Cancel any ongoing Bulk transfers */

  ret = DRVR_CANCEL(hport->drvr, priv->bulkin);
  if (ret < 0)
    {
     uerr("ERROR: Bulk IN DRVR_CANCEL failed: %d\n", ret);
    }

  ret = DRVR_CANCEL(hport->drvr, priv->bulkout);
  if (ret < 0)
    {
     uerr("ERROR: Bulk OUT DRVR_CANCEL failed: %d\n", ret);
    }

#ifdef HAVE_INTIN_ENDPOINT
  /* Cancel any pending asynchronous I/O */

  if (priv->intin)
    {
      int ret = DRVR_CANCEL(hport->drvr, priv->intin);
      if (ret < 0)
        {
         uerr("ERROR: Interrupt IN DRVR_CANCEL failed: %d\n", ret);
        }
    }
#endif

  /* Now check the number of references on the class instance.  If it is one,
   * then we can free the class instance now.  Otherwise, we will have to
   * wait until the holders of the references free them by closing the
   * serial driver.
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
                priv->ntwork.worker, usbhost_destroy);

          DEBUGASSERT(work_available(&priv->ntwork));
          work_queue(HPWORK, &priv->ntwork, usbhost_destroy, priv, 0);
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
 * Serial Lower-Half Interfaces
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int usbhost_setup(FAR struct uart_dev_s *uartdev)
{
  FAR struct usbhost_cdcacm_s *priv;
  irqstate_t flags;
  int ret;

  uinfo("Entry\n");
  DEBUGASSERT(uartdev && uartdev->priv);
  priv = (FAR struct usbhost_cdcacm_s *)uartdev->priv;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);
  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if the CDC/ACM device is still connected.  We need to disable
   * interrupts momentarily to assure that there are no asynchronous
   * isconnect events.
   */

  flags = enter_critical_section();
  if (priv->disconnected)
    {
      /* No... the block driver is no longer bound to the class.  That means
       * that the USB CDC/ACM device is no longer connected.  Refuse any
       * further attempts to open the driver.
       */

      ret = -ENODEV;
    }
  else
    {
      /* Otherwise, just increment the reference count on the driver */

      priv->crefs++;
      ret = OK;
    }

  leave_critical_section(flags);
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: usbhost_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void usbhost_shutdown(FAR struct uart_dev_s *uartdev)
{
  FAR struct usbhost_cdcacm_s *priv;
  irqstate_t flags;

  uinfo("Entry\n");
  DEBUGASSERT(uartdev && uartdev->priv);
  priv = (FAR struct usbhost_cdcacm_s *)uartdev->priv;

  /* Decrement the reference count on the block driver */

  DEBUGASSERT(priv->crefs > 1);
  nxmutex_lock(&priv->lock);
  priv->crefs--;

  /* Release the semaphore.  The following operations when crefs == 1 are
   * safe because we know that there is no outstanding open references to
   * the block driver.
   */

  nxmutex_unlock(&priv->lock);

  /* We need to disable interrupts momentarily to assure that there are
   * no asynchronous disconnect events.
   */

  flags = enter_critical_section();

  /* Check if the USB CDC/ACM device is still connected.  If the
   * CDC/ACM device is not connected and the reference count just
   * decremented to one, then unregister the block driver and free
   * the class instance.
   */

  if (priv->crefs <= 1 && priv->disconnected)
    {
      /* Destroy the class instance */

      DEBUGASSERT(priv->crefs == 1);
      usbhost_destroy(priv);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: usbhost_attach
 *
 * Description:
 *   Configure the USART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int usbhost_attach(FAR struct uart_dev_s *uartdev)
{
  return OK;
}

/****************************************************************************
 * Name: usbhost_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void usbhost_detach(FAR struct uart_dev_s *uartdev)
{
}

/****************************************************************************
 * Name: usbhost_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int usbhost_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct usbhost_cdcacm_s *priv;
  FAR struct uart_dev_s *uartdev;
  int ret = 0;

  uinfo("Entry\n");
  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  uartdev = (FAR struct uart_dev_s *)inode->i_private;

  DEBUGASSERT(uartdev && uartdev->priv);
  priv = (FAR struct usbhost_cdcacm_s *)uartdev->priv;

  /* Check if the CDC/ACM device is still connected */

  if (priv->disconnected)
    {
      /* No... the serial device has been disconnecgted.  Refuse to process
       * any ioctl commands.
       */

      ret = -ENODEV;
    }
  else
    {
      /* Process the IOCTL by command */

      ret = nxmutex_lock(&priv->lock);
      if (ret < 0)
        {
          return ret;
        }

      switch (cmd)
        {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
        case TIOCSERGSTRUCT:
          {
            FAR struct usbhost_cdcacm_s *user =
              (FAR struct usbhost_cdcacm_s *)arg;
            if (!user)
              {
                ret = -EINVAL;
              }
            else
              {
                memcpy(user, uartdev, sizeof(struct usbhost_cdcacm_s));
              }
          }
          break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
        case TCGETS:
          {
            FAR struct termios *termiosp = (FAR struct termios *)arg;

            if (!termiosp)
              {
                ret = -EINVAL;
                break;
              }

            cfsetispeed(termiosp, priv->baud);

            termiosp->c_cflag =
              ((priv->parity != 0) ? PARENB : 0) |
              ((priv->parity == 1) ? PARODD : 0) |
              ((priv->stop2) ? CSTOPB : 0)
#ifdef CONFIG_SERIAL_OFLOWCONTROL
              | ((priv->oflow) ? CCTS_OFLOW : 0)
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
              | ((priv->iflow) ? CRTS_IFLOW : 0)
#endif
              ;

            switch (priv->nbits)
              {
              case 5:
                termiosp->c_cflag |= CS5;
                break;

              case 6:
                termiosp->c_cflag |= CS6;
                break;

              case 7:
                termiosp->c_cflag |= CS7;
                break;

              default: /* 16-bits? */
              case 8:
                termiosp->c_cflag |= CS8;
                break;
              }
          }
          break;

        case TCSETS:
          {
            FAR struct termios *termiosp = (FAR struct termios *)arg;
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            bool iflow;
#endif

            if (!termiosp)
              {
                ret = -EINVAL;
                break;
              }

            if (termiosp->c_cflag & PARENB)
              {
                priv->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
              }
            else
               {
                priv->parity = 0;
              }

            priv->stop2 = (termiosp->c_cflag & CSTOPB) != 0;
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            priv->oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            iflow       = priv->iflow;
            priv->iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif

            switch (termiosp->c_cflag & CSIZE)
              {
              case CS5:
                priv->nbits = 5;
                break;

              case CS6:
                priv->nbits = 6;
                break;

              case CS7:
                priv->nbits = 7;
                break;

              default:
              case CS8:
                priv->nbits = 8;
                break;
              }

            /* Note that only cfgetispeed is used because we have knowledge
             * that only one speed is supported.
             */

            priv->baud = cfgetispeed(termiosp);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
            /* Set RTS if input flow control changed */

            if (iflow != !priv->iflow)
              {
                 priv->rts = true;
              }
#endif

#ifdef HAVE_CTRL_INTERFACE
            /* Effect the changes immediately - note that we do not implement
             * TCSADRAIN / TCSAFLUSH
             */

            ret = usbhost_linecoding_send(priv);
#endif
          }
          break;
#endif /* CONFIG_SERIAL_TERMIOS */

#ifdef CONFIG_USBHOST_CDCACM_BREAKS
        case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
          {
#warning Missing logic
          }
          break;

        case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
          {
#warning Missing logic
          }
          break;
#endif

        default:
          ret = -ENOTTY;
          break;
        }

      nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void usbhost_rxint(FAR struct uart_dev_s *uartdev, bool enable)
{
  FAR struct usbhost_cdcacm_s *priv;
  int ret;

  DEBUGASSERT(uartdev && uartdev->priv);
  priv = (FAR struct usbhost_cdcacm_s *)uartdev->priv;

  /* Are we enabling or disabling RX reception? */

  if (enable && !priv->rxena)
    {
      /* Cancel any pending, delayed RX data reception work */

      work_cancel(LPWORK, &priv->rxwork);

      /* Restart immediate RX data reception work (unless RX flow control
       * is in effect.
       */

#ifdef CONFIG_SERIAL_IFLOWCONTROL
      if (priv->rts)
#endif
        {
          ret = work_queue(LPWORK, &priv->rxwork,
                           usbhost_rxdata_work, priv, 0);
          DEBUGASSERT(ret >= 0);
          UNUSED(ret);
        }
    }
  else if (!enable && priv->rxena)
    {
      /* Cancel any pending RX data reception work */

      work_cancel(LPWORK, &priv->rxwork);
    }

  /* Save the new RX enable state */

  priv->rxena = enable;
}

/****************************************************************************
 * Name: usbhost_rxavailable
 *
 * Description:
 *   Return true if the receive buffer is not empty
 *
 ****************************************************************************/

static bool usbhost_rxavailable(FAR struct uart_dev_s *uartdev)
{
  FAR struct usbhost_cdcacm_s *priv;

  DEBUGASSERT(uartdev && uartdev->priv);
  priv = (FAR struct usbhost_cdcacm_s *)uartdev->priv;
  return (priv->nrxbytes > 0);
}

/****************************************************************************
 * Name: usbhost_rxflowcontrol
 *
 * Description:
 *   Called when Rx buffer is full (or exceeds configured watermark levels
 *   if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is defined).
 *   Return true if UART activated RX flow control to block more incoming
 *   data
 *
 * Input Parameters:
 *   uartdev   - UART device instance
 *   nbuffered - the number of characters currently buffered
 *               (if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is
 *               not defined the value will be 0 for an empty buffer or the
 *               defined buffer size for a full buffer)
 *   upper     - true indicates the upper watermark was crossed where
 *               false indicates the lower watermark has been crossed
 *
 * Returned Value:
 *   true if RX flow control activated.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool usbhost_rxflowcontrol(FAR struct uart_dev_s *uartdev,
                                  unsigned int nbuffered, bool upper)
{
  FAR struct usbhost_cdcacm_s *priv;
  bool newrts;
  int ret;

  DEBUGASSERT(uartdev && uartdev->priv);
  priv = (FAR struct usbhost_cdcacm_s *)uartdev->priv

  /* Is RX flow control enabled? */

  if (!priv->iflow)
    {
      /* Now.. make sure that RTS is set */

      priv->rts = true;
      return false;
    }

  /* Are we enabling or disabling RX flow control? */

  if (priv->rts && upper)
    {
      /* RX flow control is not in effect (RTS is true) but we have just
       * crossed the upper threshold, meaning that we should now clear
       * RTS.
       */

      priv->rts = false;

      /* Cancel any pending RX data reception work */

      work_cancel(LPWORK, &priv->rxwork);
      return true;
    }
  else if (!priv->rts && !upper)
    {
      /* RX flow control is in effect (RTS is false) and we have just
       * crossed the lower threshold, meaning that we should now set
       * RTS.
       */

       priv->rts = true;

      /* Restart RX data reception work flow unless RX reception is
       * disabled.
       */

      if (priv->rxena && work_available(&priv->rxwork))
        {
          ret = work_queue(LPWORK, &priv->rxwork,
                           usbhost_rxdata_work, priv, 0);
          DEBUGASSERT(ret >= 0);
          UNUSED(ret);
        }

      return false;
    }
}
#endif

/****************************************************************************
 * Name: usbhost_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void usbhost_txint(FAR struct uart_dev_s *uartdev, bool enable)
{
  FAR struct usbhost_cdcacm_s *priv;
  int ret;

  DEBUGASSERT(uartdev && uartdev->priv);
  priv = (FAR struct usbhost_cdcacm_s *)uartdev->priv;

  /* Are we enabling or disabling TX transmission? */

  if (enable && !priv->txena)
    {
      /* Cancel any pending, delayed TX data transmission work */

      work_cancel(LPWORK, &priv->txwork);

      /* Restart immediate TX data transmission work */

      ret = work_queue(LPWORK, &priv->txwork,
                       usbhost_txdata_work, priv, 0);
      DEBUGASSERT(ret >= 0);
      UNUSED(ret);
    }
  else if (!enable && priv->txena)
    {
      /* Cancel any pending TX data transmission work */

      work_cancel(LPWORK, &priv->txwork);
    }

  /* Save the new RX enable state */

  priv->txena = enable;
}

/****************************************************************************
 * Name: usbhost_txready
 *
 * Description:
 *   Return true if the transmit data register is not full.
 *
 ****************************************************************************/

static bool usbhost_txready(FAR struct uart_dev_s *uartdev)
{
  return true;
}

/****************************************************************************
 * Name: usbhost_txempty
 *
 * Description:
 *   Return true if the transmit data buffer is empty
 *
 ****************************************************************************/

static bool usbhost_txempty(FAR struct uart_dev_s *uartdev)
{
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_cdcacm_initialize
 *
 * Description:
 *   Initialize the USB host CDC/ACM class.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host CDC/ACM class.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_cdcacm_initialize(void)
{
  /* If we have been configured to use pre-allocated CDC/ACM class instances,
   * then place all of the pre-allocated USB host CDC/ACM class instances
   * into a free list.
   */

#if CONFIG_USBHOST_CDCACM_NPREALLOC > 0
  FAR struct usbhost_freestate_s *entry;
  int i;

  g_freelist = NULL;
  for (i = 0; i < CONFIG_USBHOST_CDCACM_NPREALLOC; i++)
    {
      entry        = (FAR struct usbhost_freestate_s *)&g_prealloc[i];
      entry->flink = g_freelist;
      g_freelist   = entry;
    }
#endif

  /* Advertise our availability to support (certain) CDC/ACM devices */

  return usbhost_registerclass(&g_cdcacm);
}

#endif /* CONFIG_USBHOST_CDCACM */

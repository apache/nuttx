/****************************************************************************
 * drivers/usbhost/usbhost_max3421e.c
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

/* References:
 *   "MAX3421E USB Peripheral/Host Controller with SPI Interface",
 *      19-3953, Rev 4, Maxim Integrated, July 2013 (Datasheet).
 *   "MAX3421E Programming Guide", Maxim Integrated, December 2006
 *      (Application Note).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>
#include <nuttx/spi/spi.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_devaddr.h>
#include <nuttx/usb/max3421e.h>
#include <nuttx/usb/usbhost_trace.h>

#include <nuttx/irq.h>

#ifdef CONFIG_USBHOST_MAX3421E

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* MAX3421E USB Host Driver Support
 *
 * Pre-requisites
 *
 *  CONFIG_USBHOST - Enable general USB host support
 *  CONFIG_USBHOST_MAX3421E - Enable the MAX3421E USB host support
 *  CONFIG_SCHED_LPWORK - Low priority work queue support is required.
 *
 * Options:
 *
 *   CONFIG_MAX3421E_DESCSIZE - Maximum size of a descriptor.  Default: 128
 *   CONFIG_MAX3421E_USBHOST_REGDEBUG - Enable very low-level register access
 *     debug.  Depends on CONFIG_DEBUG_USB_INFO.
 *   CONFIG_MAX3421E_USBHOST_PKTDUMP - Dump all incoming and outgoing USB
 *     packets. Depends on CONFIG_DEBUG_USB_INFO.
 */

/* Maximum size of a descriptor */

#ifndef CONFIG_MAX3421E_DESCSIZE
#  define CONFIG_MAX3421E_DESCSIZE 128
#endif

/* Low-priority work queue support is required.  The high priority work
 * queue is not used because this driver requires SPI access and may
 * block or wait for a variety of reasons.
 */

#ifndef CONFIG_SCHED_LPWORK
#  warning Low priority work thread support is necessary (CONFIG_SCHED_LPWORK)
#endif

/* Register/packet debug depends on CONFIG_DEBUG_FEATURES */

#ifndef CONFIG_DEBUG_USB_INFO
#  undef CONFIG_MAX3421E_USBHOST_REGDEBUG
#  undef CONFIG_MAX3421E_USBHOST_PKTDUMP
#endif

/* Delays *******************************************************************/

#define MAX3421E_READY_DELAY         200000      /* In loop counts */
#define MAX3421E_FLUSH_DELAY         200000      /* In loop counts */
#define MAX3421E_SETUP_DELAY         SEC2TICK(5) /* 5 seconds in system ticks */
#define MAX3421E_DATANAK_DELAY       SEC2TICK(5) /* 5 seconds in system ticks */
#define MAX3421E_RETRY_COUNT         5           /* Number of tries before giving up */

/* Ever-present MIN/MAX macros */

#ifndef MIN
#  define  MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define  MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#define NO_HOLDER               (INVALID_PROCESS_ID)

/* Debug ********************************************************************/

#define TR_FMT1 false
#define TR_FMT2 true

#define TRENTRY(id,fmt1,string) {string}

/* Lock *********************************************************************/

#define max3421e_take_exclsem(s) nxrmutex_lock(&(s)->lock)
#define max3421e_give_exclsem(s) nxrmutex_unlock(&(s)->lock);

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The following enumeration represents the various states of the USB host
 * state machine (for debug purposes only)
 */

enum max3421e_smstate_e
{
  SMSTATE_DETACHED = 0,  /* Not attached to a device */
  SMSTATE_ATTACHED,      /* Attached to a device */
  SMSTATE_ENUM,          /* Attached, enumerating */
  SMSTATE_CLASS_BOUND,   /* Enumeration complete, class bound */
};

/* When a transfer completes and the HXFRDN interrupt is received, this
 * informs the HXFRDN interrupt handle of the type of transfer that just
 * completed.
 */

enum mx3421e_hxfrdn_e
{
  HXFRDN_SETUP = 0,      /* A setup transfer just completed */
  HXFRDN_SNDZLP,         /* A zero length IN transfer just completed */
  HXFRDN_SNDFIFO,        /* A normal IN transfer using SNDFIFO just completed */
  HXFRDN_RCVFIFO,        /* A normal OUT transfer using RCVFIFO just completed */
};

/* This structure retains the state of one host channel.  NOTE: Since there
 * is only one channel operation active at a time, some of the fields in
 * in the structure could be moved in struct max3421e_ubhost_s to achieve
 * some memory savings.
 */

struct max3421e_chan_s
{
  bool              inuse;     /* True: This channel is "in use" */
  bool              in;        /* True: IN endpoint */
  uint8_t           chidx;     /* Channel index (0-3) */
  uint8_t           epno;      /* Device endpoint number (0-127) */
  uint8_t           eptype;    /* See MAX3421E_EPTYPE_* definitions */
  uint8_t           funcaddr;  /* Device function address */
  uint8_t           speed;     /* Device speed */
  uint8_t           interval;  /* Interrupt/isochronous EP polling interval */
  uint8_t           maxpacket; /* Max packet size (8 or 64) */
  uint8_t           toggles;   /* Saved data toggles from the HCTL register */
};

/* This structure retains the state of the USB host controller */

struct max3421e_usbhost_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct
   * usbhost_driver_s to struct max3421e_usbhost_s.
   */

  struct usbhost_driver_s drvr;

  /* This is the interface to the max3421e lower-half driver */

  FAR const struct max3421e_lowerhalf_s *lower;

  /* This is the hub port description understood by class drivers */

  struct usbhost_roothubport_s rhport;

  /* Overall driver status */

  bool              connected; /* Connected to device */
  bool              change;    /* Connection change */
  bool              pscwait;   /* True: Thread is waiting for a port event */
#ifdef CONFIG_DEBUG_ASSERTIONS
  bool              locked;    /* The SPI bus is locked */
#endif
  uint8_t           smstate;   /* The state of the USB host state machine */
  uint8_t           irqset;    /* Set of enabled interrupts */
  uint8_t           xfrtype;   /* See enum mx3421e_hxfrdn_e */
  uint8_t           inflight;  /* Number of Tx bytes "in-flight" (<= 128) */
  uint8_t           result;    /* The result of the transfer */
  uint16_t          buflen;    /* Buffer length (at start of transfer) */
  uint16_t          xfrd;      /* Bytes transferred (at end of transfer) */
  rmutex_t          lock;      /* Support mutually exclusive access */
  sem_t             pscsem;    /* Semaphore to wait for a port event */
  sem_t             waitsem;   /* Channel wait semaphore */
  FAR uint8_t      *buffer;    /* Transfer buffer pointer */
#ifdef CONFIG_USBHOST_ASYNCH
  usbhost_asynch_t  callback;  /* Transfer complete callback */
  FAR void         *arg;       /* Argument that accompanies the callback */
#endif
  struct work_s     irqwork;   /* Used to process interrupts */

#ifdef CONFIG_USBHOST_HUB
  /* Used to pass external hub port events */

  FAR struct usbhost_hubport_s *hport;
#endif

  /* The channel waiting for the next event (there will only be one in
   * this design)
   */

  FAR struct max3421e_chan_s *waiter;

  /* The state of each host channel, one for each device endpoint. */

  struct max3421e_chan_s chan[MAX3421E_NHOST_CHANNELS];
};

/* This is the MAX3421E connection structure */

struct max3421e_connection_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct
   * usbhost_connection_s to struct usbhost_connection_s.
   */

  struct usbhost_connection_s conn;

  /* Pointer to the associated state structure */

  FAR struct max3421e_usbhost_s *priv;
};

/* Supports allocation of both structures simultaneously */

struct usbhost_alloc_s
{
  struct max3421e_usbhost_s    priv;
  struct max3421e_connection_s conn;
};

/* Tracing support */

#ifdef HAVE_USBHOST_TRACE
struct max3421e_usbhost_trace_s
{
#if 0
  uint16_t id;
  bool fmt2;
#endif
  FAR const char *string;
};

enum usbhost_trace1codes_e
{
  __TRACE1_BASEVALUE = 0,           /* This will force the first value to be 1 */

  MAX3421E_TRACE1_ALLOC_FAIL,
#ifdef CONFIG_USBHOST_ASYNCH
  MAX3421E_TRACE1_ASYNCHSETUP_FAIL1,
  MAX3421E_TRACE1_ASYNCHSETUP_FAIL2,
#endif
  MAX3421E_TRACE1_BAD_JKSTATE,
  MAX3421E_TRACE1_BADREVISION,
  MAX3421E_TRACE1_CHANALLOC_FAIL,
  MAX3421E_TRACE1_CHANWAIT_FAIL,
  MAX3421E_TRACE1_DEVDISCONN1,
  MAX3421E_TRACE1_DEVDISCONN2,
  MAX3421E_TRACE1_DEVDISCONN3,
  MAX3421E_TRACE1_DEVDISCONN4,
  MAX3421E_TRACE1_DEVDISCONN5,
  MAX3421E_TRACE1_DEVDISCONN6,
  MAX3421E_TRACE1_DEVDISCONN7,
  MAX3421E_TRACE1_DEVDISCONN8,
  MAX3421E_TRACE1_ENUMERATE_FAIL,
  MAX3421E_TRACE1_INSETUP_FAIL1,
#ifdef CONFIG_USBHOST_ASYNCH
  MAX3421E_TRACE1_INSETUP_FAIL2,
  MAX3421E_TRACE1_INSETUP_FAIL3,
#endif
  MAX3421E_TRACE1_IRQATTACH_FAIL,
  MAX3421E_TRACE1_OUTSETUP_FAIL1,
#ifdef CONFIG_USBHOST_ASYNCH
  MAX3421E_TRACE1_OUTSETUP_FAIL2,
  MAX3421E_TRACE1_OUTSETUP_FAIL3,
#endif
  MAX3421E_TRACE1_RECVDATA_FAIL,
  MAX3421E_TRACE1_RECVSTATUS_FAIL,
  MAX3421E_TRACE1_SENDDATA_FAIL,
  MAX3421E_TRACE1_SENDSETUP_FAIL1,
  MAX3421E_TRACE1_SENDSETUP_FAIL2,
  MAX3421E_TRACE1_SENDSTATUS_FAIL,
  MAX3421E_TRACE1_TRANSFER_FAILED1,
  MAX3421E_TRACE1_TRANSFER_FAILED2,
  MAX3421E_TRACE1_TRANSFER_FAILED3,

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  MAX3421E_VTRACE1_CANCEL,
  MAX3421E_VTRACE1_CONNECTED1,
  MAX3421E_VTRACE1_CONNECTED2,
  MAX3421E_VTRACE1_CONNECTED3,
  MAX3421E_VTRACE1_DISCONNECTED1,
  MAX3421E_VTRACE1_DISCONNECTED2,
  MAX3421E_VTRACE1_ENUMERATE,
#ifdef CONFIG_USBHOST_HUB
  MAX3421E_VTRACE1_HUB_CONNECTED,
#endif
  MAX3421E_VTRACE1_INITIALIZED,
#ifdef CONFIG_USBHOST_ASYNCH
  MAX3421E_VTRACE1_TRANSFER_COMPLETE,
#endif

#endif

  __TRACE1_NSTRINGS,                 /* Separates the format 1 from the format 2 strings */

#ifdef HAVE_USBHOST_TRACE_VERBOSE
#ifdef CONFIG_USBHOST_ASYNCH
  MAX3421E_VTRACE2_ASYNCH,
#endif
  MAX3421E_VTRACE2_BULKIN,
  MAX3421E_VTRACE2_BULKOUT,
  MAX3421E_VTRACE2_CHANWAKEUP_IN,
  MAX3421E_VTRACE2_CHANWAKEUP_OUT,
  MAX3421E_VTRACE2_CTRLIN,
  MAX3421E_VTRACE2_CTRLOUT,
#ifdef CONFIG_USBHOST_HUB
  MAX3421E_VTRACE2_HUB_CONNECTED,
#endif
  MAX3421E_VTRACE2_INTRIN,
  MAX3421E_VTRACE2_INTROUT,
  MAX3421E_VTRACE2_ISOCIN,
  MAX3421E_VTRACE2_ISOCOUT,
  MAX3421E_VTRACE2_RECVSTATUS,
  MAX3421E_VTRACE2_SENDSTATUS,
  MAX3421E_VTRACE2_STARTTRANSFER1,
  MAX3421E_VTRACE2_STARTTRANSFER2,
  MAX3421E_VTRACE2_TRANSFER,
#ifdef CONFIG_USBHOST_ASYNCH
  MAX3421E_VTRACE2_XFRCOMPLETE,
#endif
#endif
  __TRACE2_NSTRINGS                  /* Total number of enumeration values */
};

#  define TRACE1_FIRST     ((int)__TRACE1_BASEVALUE + 1)
#  define TRACE1_INDEX(id) ((int)(id) - TRACE1_FIRST)
#  define TRACE1_NSTRINGS  TRACE1_INDEX(__TRACE1_NSTRINGS)

#  define TRACE2_FIRST     ((int)__TRACE1_NSTRINGS + 1)
#  define TRACE2_INDEX(id) ((int)(id) - TRACE2_FIRST)
#  define TRACE2_NSTRINGS  TRACE2_INDEX(__TRACE2_NSTRINGS)

#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI/Register operations **************************************************/

static void max3421e_lock(FAR struct max3421e_usbhost_s *priv);
static void max3421e_unlock(FAR struct max3421e_usbhost_s *priv);

#ifdef CONFIG_MAX3421E_USBHOST_REGDEBUG
static void max3421e_printreg(uint8_t addr, uint8_t val, bool iswrite);
static void max3421e_checkreg(uint8_t addr, uint8_t val, bool iswrite)
#endif

static inline uint8_t max3421e_fmtcmd(FAR struct max3421e_usbhost_s *priv,
              uint8_t addr, uint8_t dir);
static uint32_t max3421e_getreg(FAR struct max3421e_usbhost_s *priv,
              uint8_t addr);
static void max3421e_putreg(FAR struct max3421e_usbhost_s *priv,
              uint8_t addr, uint8_t value);

static inline void max3421e_modifyreg(FAR struct max3421e_usbhost_s *priv,
              uint8_t addr, uint8_t clrbits, uint8_t setbits);

static void max3421e_recvblock(FAR struct max3421e_usbhost_s *priv,
              uint8_t addr, FAR void *buffer, size_t buflen);
static void max3421e_sndblock(FAR struct max3421e_usbhost_s *priv,
              uint8_t addr, FAR const void *buffer, size_t buflen);

#ifdef CONFIG_MAX3421E_USBHOST_PKTDUMP
#  define max3421e_pktdump(m,b,n) lib_dumpbuffer(m,b,n)
#else
#  define max3421e_pktdump(m,b,n)
#endif

/* Semaphores ***************************************************************/

static int max3421e_takesem(FAR sem_t *sem);
#define max3421e_givesem(s) nxsem_post(s);
static int max3421e_take_exclsem(FAR struct max3421e_usbhost_s *priv);
static void max3421e_give_exclsem(FAR struct max3421e_usbhost_s *priv);

/* Byte stream access helper functions **************************************/

static inline uint16_t max3421e_getle16(const uint8_t *val);

/* Channel management *******************************************************/

static int max3421e_chan_alloc(FAR struct max3421e_usbhost_s *priv);
static inline void max3421e_chan_free(FAR struct max3421e_usbhost_s *priv,
              int chidx);
static int max3421e_chan_waitsetup(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
#ifdef CONFIG_USBHOST_ASYNCH
static int max3421e_chan_asynchsetup(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan, usbhost_asynch_t callback,
              FAR void *arg);
#endif
static int max3421e_chan_wait(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static void max3421e_chan_wakeup(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan, int result);

/* Control/data transfer logic **********************************************/

static inline void max3421e_save_toggles(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static inline void max3421e_restore_toggles(
              FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static int  max3421e_transfer_status(FAR struct max3421e_usbhost_s *priv);
static void max3421e_transfer_terminate(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan, int result);

/* OUT transfers */

static void max3421e_put_sndfifo(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static void max3421e_send_continue(FAR struct max3421e_usbhost_s *priv);
static void max3421e_send_start(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static ssize_t max3421e_out_transfer(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan, FAR uint8_t *buffer,
              size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void max3421e_out_next(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static int  max3421e_out_asynch(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan, FAR uint8_t *buffer,
              size_t buflen, usbhost_asynch_t callback, FAR void *arg);
#endif

/* Control transfers */

static int  max3421e_ctrl_sendsetup(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan,
              FAR const struct usb_ctrlreq_s *req);
static int  max3421e_ctrl_senddata(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan,
              FAR uint8_t *buffer, unsigned int buflen);
static int  max3421e_ctrl_sendstatus(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static int  max3421e_ctrl_recvstatus(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static int  max3421e_in_setup(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static int  max3421e_out_setup(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);

/* IN transfers */

static uint8_t max3421e_get_rcvfifo(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static void max3421e_recv_restart(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static void max3421e_recv_continue(FAR struct max3421e_usbhost_s *priv);
static void max3421e_recv_start(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static ssize_t max3421e_in_transfer(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan, FAR uint8_t *buffer,
              size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void max3421e_in_next(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static int  max3421e_in_asynch(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan, FAR uint8_t *buffer,
              size_t buflen, usbhost_asynch_t callback, FAR void *arg);
#endif

/* Interrupt handling *******************************************************/

static void max3421e_connect_event(FAR struct max3421e_usbhost_s *priv);
static void max3421e_disconnect_event(FAR struct max3421e_usbhost_s *priv);
static int  max3421e_connected(FAR struct max3421e_usbhost_s *priv);
static void max3421e_disconnected(FAR struct max3421e_usbhost_s *priv);
static void max3421e_irqwork(FAR void *arg);
static int  max3421e_interrupt(int irq, FAR void *context, FAR void *arg);

/* Interrupt controls */

static inline void max3421e_int_enable(FAR struct max3421e_usbhost_s *priv,
              uint8_t irqset);
static inline void max3421e_int_disable(FAR struct max3421e_usbhost_s *priv,
              uint8_t irqset);
static inline uint8_t max3421e_int_status(
              FAR struct max3421e_usbhost_s *priv);
static inline void max3421e_int_clear(FAR struct max3421e_usbhost_s *priv,
              uint8_t irqset);
static void max3421e_int_wait(FAR struct max3421e_usbhost_s *priv,
              uint8_t irqset, unsigned int usec);

/* USB host controller operations *******************************************/

static int  max3421e_wait(FAR struct usbhost_connection_s *conn,
              FAR struct usbhost_hubport_s **hport);
static int  max3421e_getspeed(FAR struct max3421e_usbhost_s *priv,
              FAR struct usbhost_connection_s *conn,
              FAR struct usbhost_hubport_s *hport);
static int  max3421e_enumerate(FAR struct usbhost_connection_s *conn,
              FAR struct usbhost_hubport_s *hport);

static int  max3421e_ep0configure(FAR struct usbhost_driver_s *drvr,
              usbhost_ep_t ep0, uint8_t funcaddr, uint8_t speed,
              uint16_t maxpacketsize);
static int  max3421e_epalloc(FAR struct usbhost_driver_s *drvr,
              FAR const FAR struct usbhost_epdesc_s *epdesc,
              FAR usbhost_ep_t *ep);
static int  max3421e_epfree(FAR struct usbhost_driver_s *drvr,
              usbhost_ep_t ep);
static int  max3421e_alloc(FAR struct usbhost_driver_s *drvr,
              FAR uint8_t **buffer, FAR size_t *maxlen);
static int  max3421e_free(FAR struct usbhost_driver_s *drvr,
              FAR uint8_t *buffer);
static int  max3421e_ioalloc(FAR struct usbhost_driver_s *drvr,
              FAR uint8_t **buffer, size_t buflen);
static int  max3421e_iofree(FAR struct usbhost_driver_s *drvr,
              FAR uint8_t *buffer);
static int  max3421e_ctrlin(FAR struct usbhost_driver_s *drvr,
              usbhost_ep_t ep0, FAR const struct usb_ctrlreq_s *req,
              FAR uint8_t *buffer);
static int  max3421e_ctrlout(FAR struct usbhost_driver_s *drvr,
              usbhost_ep_t ep0, FAR const struct usb_ctrlreq_s *req,
              FAR const uint8_t *buffer);
static ssize_t max3421e_transfer(FAR struct usbhost_driver_s *drvr,
              usbhost_ep_t ep, FAR uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static int  max3421e_asynch(FAR struct usbhost_driver_s *drvr,
              usbhost_ep_t ep, FAR uint8_t *buffer, size_t buflen,
              usbhost_asynch_t callback, FAR void *arg);
#endif
static int  max3421e_cancel(FAR struct usbhost_driver_s *drvr,
                            usbhost_ep_t ep);
#ifdef CONFIG_USBHOST_HUB
static int  max3421e_connect(FAR struct max3421e_usbhost_s *priv,
              FAR struct usbhost_hubport_s *hport, bool connected);
#endif
static void max3421e_disconnect(FAR struct usbhost_driver_s *drvr,
              FAR struct usbhost_hubport_s *hport);

/* Initialization ***********************************************************/

static void max3421e_busreset(FAR struct max3421e_usbhost_s *priv);
static int  max3421e_startsof(FAR struct max3421e_usbhost_s *priv);

static inline int max3421e_sw_initialize(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_connection_s *conn,
              FAR const struct max3421e_lowerhalf_s *lower);
static inline int max3421e_hw_initialize(
              FAR struct max3421e_usbhost_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_USBHOST_TRACE
/* Trace/debug format strings. */

static const struct max3421e_usbhost_trace_s g_trace1[TRACE1_NSTRINGS] =
{
  TRENTRY(MAX3421E_TRACE1_ALLOC_FAIL, TR_FMT1,
          "INIT: Failed to allocate state structure: %u\n"),
#ifdef CONFIG_USBHOST_ASYNCH
  TRENTRY(MAX3421E_TRACE1_ASYNCHSETUP_FAIL1, TR_FMT1,
          "OUT: Asynch setup failed: %u\n"),
  TRENTRY(MAX3421E_TRACE1_ASYNCHSETUP_FAIL2, TR_FMT1,
          "IN: Asynch setup failed: %u\n"),
#endif
  TRENTRY(MAX3421E_TRACE1_BAD_JKSTATE, TR_FMT1,
          "CONNECT: Bad JK state: %02x\n"),
  TRENTRY(MAX3421E_TRACE1_BADREVISION, TR_FMT1,
          "INIT: Bad revision number:  %02x\n"),
  TRENTRY(MAX3421E_TRACE1_CHANALLOC_FAIL, TR_FMT1,
          "EPALLOC: Channel allocation failed: %u\n"),
  TRENTRY(MAX3421E_TRACE1_CHANWAIT_FAIL, TR_FMT1,
          "OUT: Channel wait failure: %u\n"),
  TRENTRY(MAX3421E_TRACE1_DEVDISCONN1, TR_FMT1,
          "OUT: Disconnected during wait: %u\n"),
  TRENTRY(MAX3421E_TRACE1_DEVDISCONN2, TR_FMT1,
          "CTRL: Disconnected during SETUP phase: %u\n"),
  TRENTRY(MAX3421E_TRACE1_DEVDISCONN3, TR_FMT1,
          "CTRL OUT: Disconnected during DATA phase: %u\n"),
  TRENTRY(MAX3421E_TRACE1_DEVDISCONN4, TR_FMT1,
          "CTRL IN: Disconnected during DATA phase: %u"),
  TRENTRY(MAX3421E_TRACE1_DEVDISCONN5, TR_FMT1,
          "IN: Disconnected during wait: %u\n"),
  TRENTRY(MAX3421E_TRACE1_DEVDISCONN6, TR_FMT1,
          "CONNECT: Device disconnect #1: %u\n"),
  TRENTRY(MAX3421E_TRACE1_DEVDISCONN7, TR_FMT1,
          "CONNECT: Device disconnect #2: %u\n"),
  TRENTRY(MAX3421E_TRACE1_DEVDISCONN8, TR_FMT1,
          "CONNECT: Device disconnect #3: %u\n"),
  TRENTRY(MAX3421E_TRACE1_ENUMERATE_FAIL, TR_FMT1,
          "CONNECT: Enumeration failed: %u\n"),
  TRENTRY(MAX3421E_TRACE1_INSETUP_FAIL1, TR_FMT1,
          "CTRL IN: SETUP phase failure: %u\n"),
#ifdef CONFIG_USBHOST_ASYNCH
  TRENTRY(MAX3421E_TRACE1_INSETUP_FAIL2, TR_FMT1,
          "CTRL IN: Asynch SETUP phase failure #1: %u\n"),
  TRENTRY(MAX3421E_TRACE1_INSETUP_FAIL3, TR_FMT1,
          "CTRL IN: Asynch SETUP phase failure #2: %u\n"),
#endif
  TRENTRY(MAX3421E_TRACE1_IRQATTACH_FAIL, TR_FMT1,
          "INIT: Failed to attach interrupt: %u\n"),
  TRENTRY(MAX3421E_TRACE1_OUTSETUP_FAIL1, TR_FMT1,
          "CTRL OUT: SETUP phase failure: %u\n"),
#ifdef CONFIG_USBHOST_ASYNCH
  TRENTRY(MAX3421E_TRACE1_OUTSETUP_FAIL2, TR_FMT1,
          "CTRL OUT: Asynch SETUP phase failure #1: %u\n"),
  TRENTRY(MAX3421E_TRACE1_OUTSETUP_FAIL3, TR_FMT1,
          "CTRL OUT: Asynch SETUP phase failure #2: %u\n"),
#endif
  TRENTRY(MAX3421E_TRACE1_RECVDATA_FAIL, TR_FMT1,
          "CTRL IN: Data phase failure: %u\n"),
  TRENTRY(MAX3421E_TRACE1_RECVSTATUS_FAIL, TR_FMT1,
          "CTRL OUT: Status phase failure: %u\n"),
  TRENTRY(MAX3421E_TRACE1_SENDDATA_FAIL, TR_FMT1,
          "CTRL OUT: Data phase failure: %u\n"),
  TRENTRY(MAX3421E_TRACE1_SENDSETUP_FAIL1, TR_FMT1,
          "CTRL OUT: SETUP phase failure: %u\n"),
  TRENTRY(MAX3421E_TRACE1_SENDSETUP_FAIL2, TR_FMT1,
          "CTRL IN: SETUP phase failure: %u\n"),
  TRENTRY(MAX3421E_TRACE1_SENDSTATUS_FAIL, TR_FMT1,
          "CTRL IN: Status phase failure: %u\n"),
  TRENTRY(MAX3421E_TRACE1_TRANSFER_FAILED1, TR_FMT1,
          "OUT: Transfer wait returned failure: %u\n"),
  TRENTRY(MAX3421E_TRACE1_TRANSFER_FAILED2, TR_FMT1,
          "CTRL: SETUP wait returned failure: %u\n"),
  TRENTRY(MAX3421E_TRACE1_TRANSFER_FAILED3, TR_FMT1,
          "IN: Transfer wait returned failure: %u\n"),

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  TRENTRY(MAX3421E_VTRACE1_CANCEL, TR_FMT1,
          "Transfer canceled: EP%u\n"),
  TRENTRY(MAX3421E_VTRACE1_CONNECTED1, TR_FMT1,
          "CONNECT: Connection event: %u\n"),
  TRENTRY(MAX3421E_VTRACE1_CONNECTED2, TR_FMT1,
          "CONNECT: Connection change detected: %u\n"),
  TRENTRY(MAX3421E_VTRACE1_CONNECTED3, TR_FMT1,
          "CONNECT: Connected: %u\n"),
  TRENTRY(MAX3421E_VTRACE1_DISCONNECTED1, TR_FMT1,
          "CONNECT: Disconnected: %u\n"),
  TRENTRY(MAX3421E_VTRACE1_DISCONNECTED2, TR_FMT1,
          "CONNECT: Disconnect detected: %u\n"),
  TRENTRY(MAX3421E_VTRACE1_ENUMERATE, TR_FMT1,
          "ENUMERATE: Start: %u\n"),
#ifdef CONFIG_USBHOST_HUB
  TRENTRY(MAX3421E_VTRACE1_HUB_CONNECTED, TR_FMT1,
          "CONNECT: Hub connected: %u\n"),
#endif
  TRENTRY(MAX3421E_VTRACE1_INITIALIZED, TR_FMT1,
          "INIT: Hardware initialized: %u\n"),
#ifdef CONFIG_USBHOST_ASYNCH
  TRENTRY(MAX3421E_VTRACE1_TRANSFER_COMPLETE, TR_FMT1,
          "OUT: Asynch transfer complete: %u\n"),
#endif
#endif
};

static const struct max3421e_usbhost_trace_s g_trace2[TRACE2_NSTRINGS] =
{
#ifdef HAVE_USBHOST_TRACE_VERBOSE
#ifdef CONFIG_USBHOST_ASYNCH
  TRENTRY(MAX3421E_VTRACE2_ASYNCH, TR_FMT2,
          "ASYNCH: Transfer started: EP%u len=%u\n"),
#endif
  TRENTRY(MAX3421E_VTRACE2_BULKIN, TR_FMT2,
          "BULK IN:  SETUP: Chan%u len=%u\n"),
  TRENTRY(MAX3421E_VTRACE2_BULKOUT, TR_FMT2,
          "BULK OUT:  SETUP: Chan%u len=%u\n"),
  TRENTRY(MAX3421E_VTRACE2_CHANWAKEUP_IN, TR_FMT2,
          "IN: Channel wakeup: Chan%, result=%u\n"),
  TRENTRY(MAX3421E_VTRACE2_CHANWAKEUP_OUT, TR_FMT2,
          "OUT: Channel wakeup: Chan%u result=%u\n"),
  TRENTRY(MAX3421E_VTRACE2_CTRLIN, TR_FMT2,
          "CTRL IN: Start: type=%u req=%u\n"),
  TRENTRY(MAX3421E_VTRACE2_CTRLOUT, TR_FMT2,
          "CTRL OUT: Start: type=%u req=%u\n"),
#ifdef CONFIG_USBHOST_HUB
  TRENTRY(MAX3421E_VTRACE2_HUB_CONNECTED, TR_FMT2,
          "CONNECT: Hub connected: port=%u, connected=%u\n"),
#endif
  TRENTRY(MAX3421E_VTRACE2_INTRIN, TR_FMT2,
          "INTR IN: SETUP: Chan%u len=%u\n"),
  TRENTRY(MAX3421E_VTRACE2_INTROUT, TR_FMT2,
          "INTR OUT: SETUP: Chan%u len=%u\n"),
  TRENTRY(MAX3421E_VTRACE2_ISOCIN, TR_FMT2,
          "ISOC IN: SETUP: Chan%u len=%u\n"),
  TRENTRY(MAX3421E_VTRACE2_ISOCOUT, TR_FMT2,
          "ISOC OUT: SETUP: Chan%u len=%u\n"),
  TRENTRY(MAX3421E_VTRACE2_RECVSTATUS, TR_FMT2,
          "CTRL OUT: Receive status: Chan% len=%u\n"),
  TRENTRY(MAX3421E_VTRACE2_SENDSTATUS, TR_FMT2,
          "CTRL IN: Send status: Chan% len=%u\n"),
  TRENTRY(MAX3421E_VTRACE2_STARTTRANSFER1, TR_FMT2,
          "OUT: Send start: Chan% len=%u\n"),
  TRENTRY(MAX3421E_VTRACE2_STARTTRANSFER2, TR_FMT2,
          "IN: Receive start: Chan% len=%u\n"),
  TRENTRY(MAX3421E_VTRACE2_TRANSFER, TR_FMT2,
          "Transfer start: EP%u len=%u\n"),
#ifdef CONFIG_USBHOST_ASYNCH
  TRENTRY(MAX3421E_VTRACE2_XFRCOMPLETE, TR_FMT2,
          "ASYNCH: Transfer complete: EP%u len=%u\n"),
#endif
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max3421e_lock
 *
 * Description:
 *   Lock and configure the SPI bus.
 *
 ****************************************************************************/

static void max3421e_lock(FAR struct max3421e_usbhost_s *priv)
{
  FAR const struct max3421e_lowerhalf_s *lower = priv->lower;
  FAR struct spi_dev_s *spi;

  DEBUGASSERT(lower != NULL && lower->spi != NULL && !priv->locked);
  spi = lower->spi;

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, lower->mode);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, lower->frequency);

#ifdef CONFIG_DEBUG_ASSERTIONS
  /* Mark the SPI bus as locked (for debug only) */

  priv->locked = true;
#endif
}

/****************************************************************************
 * Name: max3421e_unlock
 *
 * Description:
 *   Unlock the SPI bus.
 *
 ****************************************************************************/

static void max3421e_unlock(FAR struct max3421e_usbhost_s *priv)
{
  FAR const struct max3421e_lowerhalf_s *lower = priv->lower;

  DEBUGASSERT(lower != NULL && lower->spi != NULL && priv->locked);
  SPI_LOCK(lower->spi, false);

#ifdef CONFIG_DEBUG_ASSERTIONS
  /* Mark the SPI bus as unlocked (for debug only) */

  priv->locked = false;
#endif
}

/****************************************************************************
 * Name: max3421e_printreg
 *
 * Description:
 *   Print the contents of an MAX3421Exx register operation
 *
 ****************************************************************************/

#ifdef CONFIG_MAX3421E_USBHOST_REGDEBUG
static void max3421e_printreg(uint8_t addr, uint8_t val, bool iswrite)
{
  uinfo("%02x%s%02x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/****************************************************************************
 * Name: max3421e_checkreg
 *
 * Description:
 *   Get the contents of an MAX3421E register
 *
 ****************************************************************************/

#ifdef CONFIG_MAX3421E_USBHOST_REGDEBUG
static void max3421e_checkreg(uint8_t addr, uint8_t val, bool iswrite)
{
  static uint8_t prevaddr = 0;
  static uint8_t preval = 0;
  static unsigned int count = 0;
  static bool prevwrite = false;

  /* Is this the same value that we read from/wrote to the same register
   * last time?  Are we polling the register?  If so, suppress the output.
   */

  if (addr == prevaddr && val == preval && prevwrite == iswrite)
    {
      /* Yes.. Just increment the count */

      count++;
    }
  else
    {
      /* No this is a new address or value or operation. Were there any
       * duplicate accesses before this one?
       */

      if (count > 0)
        {
          /* Yes.. Just one? */

          if (count == 1)
            {
              /* Yes.. Just one */

              max3421e_printreg(prevaddr, preval, prevwrite);
            }
          else
            {
              /* No.. More than one. */

              uinfo("[repeats %d more times]\n", count);
            }
        }

      /* Save the new address, value, count, and operation for next time */

      prevaddr  = addr;
      preval    = val;
      count     = 0;
      prevwrite = iswrite;

      /* Show the new register access */

      max3421e_printreg(addr, val, iswrite);
    }
}
#endif

/****************************************************************************
 * Name: max3421e_fmtcmd
 *
 * Description:
 *   Format a command
 *
 *   The command byte contains the register address, a direction bit, and an
 *   ACKSTAT bit:
 *
 *     Bits 3-7:  Command
 *     Bit 2:     Unused
 *     Bit 1:     Direction (read = 0, write = 1)
 *     Bit 0:     ACKSTAT
 *
 *   The ACKSTAT bit is ignored in host mode.
 *
 ****************************************************************************/

static inline uint8_t max3421e_fmtcmd(FAR struct max3421e_usbhost_s *priv,
                                      uint8_t addr, uint8_t dir)
{
  return addr | dir | MAX3421E_ACKSTAT_FALSE;
}

/****************************************************************************
 * Name: max3421e_getreg
 *
 * Description:
 *   Get the contents of an MAX3421E register
 *
 * Assumption:
 *   SPI bus is locked
 *
 ****************************************************************************/

static uint32_t max3421e_getreg(FAR struct max3421e_usbhost_s *priv,
                                uint8_t addr)
{
  FAR const struct max3421e_lowerhalf_s *lower = priv->lower;
  FAR struct spi_dev_s *spi;
  uint8_t cmd;
  uint8_t value;

  DEBUGASSERT(lower != NULL && lower->spi != NULL && priv->locked);
  spi = lower->spi;

  /* Select the MAX4321E */

  SPI_SELECT(spi, SPIDEV_USBHOST(lower->devid), true);

  /* Send the read command byte */

  cmd = max3421e_fmtcmd(priv, addr, MAX3421E_DIR_READ);
  SPI_SEND(spi, cmd);

  /* Read the value of the register */

  value = SPI_SEND(spi, 0xff);

  /* De-select the MAX4321E */

  SPI_SELECT(spi, SPIDEV_USBHOST(lower->devid), false);

#ifdef CONFIG_MAX3421E_USBHOST_REGDEBUG
  /* Check if we need to print this value */

  max3421e_checkreg(addr, value, false);
#endif

  return value;
}

/****************************************************************************
 * Name: max3421e_putreg
 *
 * Description:
 *   Set the contents of an MAX3421E register to a value
 *
 * Assumption:
 *   SPI bus is locked
 *
 ****************************************************************************/

static void max3421e_putreg(FAR struct max3421e_usbhost_s *priv,
                            uint8_t addr, uint8_t value)
{
  FAR const struct max3421e_lowerhalf_s *lower = priv->lower;
  FAR struct spi_dev_s *spi;
  uint8_t cmd;

  DEBUGASSERT(lower != NULL && lower->spi != NULL && priv->locked);
  spi = lower->spi;

#ifdef CONFIG_MAX3421E_USBHOST_REGDEBUG
  /* Check if we need to print this value */

  max3421e_checkreg(addr, val, true);
#endif

  /* Select the MAX4321E */

  SPI_SELECT(spi, SPIDEV_USBHOST(lower->devid), true);

  /* Send the write command byte */

  cmd = max3421e_fmtcmd(priv, addr, MAX3421E_DIR_WRITE);
  SPI_SEND(spi, cmd);

  /* Send the new value for the register */

  SPI_SEND(spi, value);

  /* De-select the MAX4321E */

  SPI_SELECT(spi, SPIDEV_USBHOST(lower->devid), false);
}

/****************************************************************************
 * Name: max3421e_modifyreg
 *
 * Description:
 *   Modify selected bits of an MAX3421E register.
 *
 * Assumption:
 *   SPI bus is locked
 *
 ****************************************************************************/

static inline void max3421e_modifyreg(FAR struct max3421e_usbhost_s *priv,
                                      uint8_t addr, uint8_t clrbits,
                                      uint8_t setbits)
{
  uint8_t value;

  value  = max3421e_getreg(priv, addr);
  value &= ~clrbits;
  value |= setbits;
  max3421e_putreg(priv, addr, value);
}

/****************************************************************************
 * Name: max3421e_recvblock
 *
 * Description:
 *   Receive a block of data from the MAX341E.
 *
 * Assumption:
 *   SPI bus is locked
 *
 ****************************************************************************/

static void max3421e_recvblock(FAR struct max3421e_usbhost_s *priv,
                               uint8_t addr, FAR void *buffer, size_t buflen)
{
  FAR const struct max3421e_lowerhalf_s *lower = priv->lower;
  FAR struct spi_dev_s *spi;
  uint8_t cmd;

  DEBUGASSERT(lower != NULL && lower->spi != NULL && priv->locked);
  spi = lower->spi;

  /* Select the MAX4321E */

  SPI_SELECT(spi, SPIDEV_USBHOST(lower->devid), true);

  /* Send the read command byte */

  cmd = max3421e_fmtcmd(priv, addr, MAX3421E_DIR_READ);
  SPI_SEND(spi, cmd);

  /* Read the block of values from the register(s) */

  SPI_RECVBLOCK(spi, buffer, buflen);

  /* De-select the MAX4321E */

  SPI_SELECT(spi, SPIDEV_USBHOST(lower->devid), false);

#ifdef CONFIG_MAX3421E_USBHOST_REGDEBUG
  /* Dump the block of data received */

  lib_dumpbuffer("Received:", buffer, buflen);
#endif
}

/****************************************************************************
 * Name: max3421e_sndblock
 *
 * Description:
 *   Send a block of data to the MAX341E.
 *
 * Assumption:
 *   SPI bus is locked
 *
 ****************************************************************************/

static void max3421e_sndblock(FAR struct max3421e_usbhost_s *priv,
                              uint8_t addr, FAR const void *buffer,
                              size_t buflen)
{
  FAR const struct max3421e_lowerhalf_s *lower = priv->lower;
  FAR struct spi_dev_s *spi;
  uint8_t cmd;

  DEBUGASSERT(lower != NULL && lower->spi != NULL && priv->locked);
  spi = lower->spi;

#ifdef CONFIG_MAX3421E_USBHOST_REGDEBUG
  /* Dump the block of data to be sent */

  lib_dumpbuffer("Sending:", buffer, buflen);
#endif

  /* Select the MAX4321E */

  SPI_SELECT(spi, SPIDEV_USBHOST(lower->devid), true);

  /* Send the wrte command byte */

  cmd = max3421e_fmtcmd(priv, addr, MAX3421E_DIR_WRITE);
  SPI_SEND(spi, cmd);

  /* Send the new value for the register */

  SPI_SNDBLOCK(spi, buffer, buflen);

  /* De-select the MAX4321E */

  SPI_SELECT(spi, SPIDEV_USBHOST(lower->devid), false);
}

/****************************************************************************
 * Name: max3421e_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static int max3421e_takesem(FAR sem_t *sem)
{
  return nxsem_wait_uninterruptible(sem);
}

/****************************************************************************
 * Name: max3421e_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 ****************************************************************************/

static inline uint16_t max3421e_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: max3421e_chan_alloc
 *
 * Description:
 *   Allocate a channel.
 *
 ****************************************************************************/

static int max3421e_chan_alloc(FAR struct max3421e_usbhost_s *priv)
{
  int chidx;

  /* Search the table of channels */

  for (chidx = 0; chidx < MAX3421E_NHOST_CHANNELS; chidx++)
    {
      /* Is this channel available? */

      if (!priv->chan[chidx].inuse)
        {
          /* Yes... make it "in use" and return the index */

          priv->chan[chidx].inuse = true;
          return chidx;
        }
    }

  /* All of the channels are "in-use" */

  return -EBUSY;
}

/****************************************************************************
 * Name: max3421e_chan_free
 *
 * Description:
 *   Free a previoiusly allocated channel.
 *
 ****************************************************************************/

static void max3421e_chan_free(FAR struct max3421e_usbhost_s *priv,
                               int chidx)
{
  DEBUGASSERT((unsigned)chidx < MAX3421E_NHOST_CHANNELS);

  /* Halt the channel */
#warning Missing logic

  /* Mark the channel available */

  priv->chan[chidx].inuse = false;
}

/****************************************************************************
 * Name: max3421e_chan_waitsetup
 *
 * Description:
 *   Set the request for the transfer complete event well BEFORE enabling
 *   the transfer (as soon as we are absolutely committed to the transfer).
 *   We do this to minimize race conditions.  This logic would have to be
 *   expanded if we want to have more than one packet in flight at a time!
 *
 * Assumptions:
 *   Called from a normal thread context BEFORE the transfer has been
 *   started.
 *
 ****************************************************************************/

static int max3421e_chan_waitsetup(FAR struct max3421e_usbhost_s *priv,
                                   FAR struct max3421e_chan_s *chan)
{
  irqstate_t flags = enter_critical_section();
  int        ret   = -ENODEV;

  DEBUGASSERT(priv != NULL && chan != NULL && priv->waiter == NULL);

  /* Is the device still connected? */

  if (priv->connected)
    {
      /* Yes.. then set waiter to indicate that we expect to be informed
       * when either (1) the device is disconnected, or (2) the transfer
       * completed.
       */

      priv->waiter   = chan;
#ifdef CONFIG_USBHOST_ASYNCH
      priv->callback = NULL;
      priv->arg      = NULL;
#endif
      ret            = OK;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: max3421e_chan_asynchsetup
 *
 * Description:
 *   Set the request for the transfer complete event well BEFORE enabling the
 *   transfer (as soon as we are absolutely committed to transfer).  We do
 *   this to minimize race conditions.  This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 * Assumptions:
 *   Might be called from the level of an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int max3421e_chan_asynchsetup(FAR struct max3421e_usbhost_s *priv,
                                  FAR struct max3421e_chan_s *chan,
                                  usbhost_asynch_t callback, FAR void *arg)
{
  irqstate_t flags = enter_critical_section();
  int        ret   = -ENODEV;

  DEBUGASSERT(priv != NULL && chan != NULL);

  /* Is the device still connected? */

  if (priv->connected)
    {
      priv->waiter   = NULL;      /* No waiter */
      priv->callback = callback;
      priv->arg      = arg;
      ret            = OK;
    }

  leave_critical_section(flags);
  return ret;
}
#endif

/****************************************************************************
 * Name: max3421e_chan_wait
 *
 * Description:
 *   Wait for a transfer on a channel to complete.
 *
 * Assumptions:
 *   Called from a normal thread context
 *
 ****************************************************************************/

static int max3421e_chan_wait(FAR struct max3421e_usbhost_s *priv,
                           FAR struct max3421e_chan_s *chan)
{
  irqstate_t flags;
  int ret;

  /* Disable interrupts so that the following operations will be atomic.  On
   * the host global interrupt needs to be disabled.  However, here we
   * disable all interrupts to exploit that fact that interrupts will be re-
   * enabled while we wait.
   */

  flags = enter_critical_section();

  /* Loop, testing for an end of transfer condition.  The channel 'result'
   * was set to EBUSY and 'waiter' was set to the channel expecting the
   * response before the transfer was started; 'waiter' will be nullified
   * and 'result' will be set appropriately when the transfer is completed.
   */

  do
    {
      /* Wait for the transfer to complete.  NOTE the transfer may already
       * completed before we get here or the transfer may complete while we
       * wait here.
       */

      ret = nxsem_wait_uninterruptible(&priv->waitsem);
      if (ret < 0)
        {
          leave_critical_section(flags);
          return ret;
        }
    }
  while (priv->waiter != NULL);

  /* The transfer is complete re-enable interrupts and return the result */

  ret = -(int)priv->result;
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: max3421e_chan_wakeup
 *
 * Description:
 *   A channel transfer has completed... wakeup any threads waiting for the
 *   transfer to complete.
 *
 * Assumptions:
 *   This function is called from the transfer complete interrupt handler for
 *   the channel.  Interrupts are disabled.
 *
 ****************************************************************************/

static void max3421e_chan_wakeup(FAR struct max3421e_usbhost_s *priv,
                                 FAR struct max3421e_chan_s *chan,
                                 int result)
{
  /* Save the result of the operation */

  DEBUGASSERT(priv->result != EBUSY);
  priv->result = result;

  /* Is there a thread waiting for this transfer to complete? */

  if (priv->waiter != NULL)
    {
#ifdef CONFIG_USBHOST_ASYNCH
      /* Yes.. there should not also be a callback scheduled */

      DEBUGASSERT(priv->callback == NULL);
#endif
      /* Wake'em up! */

      usbhost_vtrace2(chan->in ? MAX3421E_VTRACE2_CHANWAKEUP_IN :
                                 MAX3421E_VTRACE2_CHANWAKEUP_OUT,
                      chan->chidx, priv->result);

      max3421e_givesem(&priv->waitsem);
      priv->waiter = NULL;
    }

#ifdef CONFIG_USBHOST_ASYNCH
  /* No.. is an asynchronous callback expected when the transfer completes? */

  else if (priv->callback)
    {
      /* Handle continuation of IN/OUT pipes */

      if (chan->in)
        {
          max3421e_in_next(priv, chan);
        }
      else
        {
          max3421e_out_next(priv, chan);
        }
    }
#endif
}

/****************************************************************************
 * Name: max3421e_save_toggles and max3421e_restore_toggles
 *
 * Description:
 *   Save and restore data toggles from/to the HCTL register.  The MAX3421E
 *   will automatically update the toggles for consecutive transfers to the
 *   same endpoint; We need to use these only when we change endpoints.
 *
 ****************************************************************************/

static inline void
  max3421e_save_toggles(FAR struct max3421e_usbhost_s *priv,
                       FAR struct max3421e_chan_s *chan)
{
  chan->toggles = max3421e_getreg(priv, MAX3421E_USBHOST_HCTL);
}

static inline void
  max3421e_restore_toggles(FAR struct max3421e_usbhost_s *priv,
                           FAR struct max3421e_chan_s *chan)
{
  max3421e_modifyreg(priv, MAX3421E_USBHOST_HCTL,
                     USBHOST_HCTL_TOGGLES_MASK,
                     chan->toggles & USBHOST_HCTL_TOGGLES_MASK);
}

/****************************************************************************
 * Name: max3421e_transfer_status
 *
 * Description:
 *   Get the end-of-transfer status from HRSLT register.
 *
 *   REVISIT:   Currently NAKs are treated as errors.  A NAK on the first
 *   packet can probably be treated that way.  But not NAKs after the
 *   transfer is in progress.  We should also need to reset the peripheral
 *   in that case.  Better to try and retry here within the driver.
 *
 * Returned value:
 *   OK     - Transfer successful
 *  -EAGAIN - If devices NAKs the transfer.
 *  -EPERM  - If the endpoint stalls
 *  -BUSY   - The transfer is not complete
 *  -EIO    - Other, undecoded error
 *
 * Assumptions:
 *   The SPI bus is locked.
 *
 ****************************************************************************/

static int max3421e_transfer_status(FAR struct max3421e_usbhost_s *priv)
{
  uint8_t regval;
  int ret;

  /* Get the result of the transfer from the HRSLT register */

  regval = max3421e_getreg(priv, MAX3421E_USBHOST_HRSL);

  /* Make the error result to something that the world knows about */

  switch (regval & USBHOST_HRSL_HRSLT_MASK)
    {
      case USBHOST_HRSL_HRSLT_SUCCESS:
        ret = OK;
        break;

      case USBHOST_HRSL_HRSLT_BUSY:
        ret = -EBUSY;
        break;

      case USBHOST_HRSL_HRSLT_NAK:
        ret = -EAGAIN;
        break;

      case USBHOST_HRSL_HRSLT_STALL:
        ret = -EPERM;
        break;

      default:
        ret = -EIO;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: max3421e_transfer_terminate
 *
 * Description:
 *   Terminate a IN or OUT transfer due to an error (or because a zero-
 *   length OUT transfer occurred).
 *
 * Returned value:
 *   OK     - Transfer successful
 *  -EAGAIN - If devices NAKs the transfer.
 *  -EPERM  - If the endpoint stalls
 *  -BUSY   - The transfer is not complete
 *  -EIO    - Other, undecoded error
 *
 * Assumptions:
 *   The SPI bus is locked.
 *
 ****************************************************************************/

static void max3421e_transfer_terminate(FAR struct max3421e_usbhost_s *priv,
                                        FAR struct max3421e_chan_s *chan,
                                        int result)
{
  /* Disable further SNDBAV, RCVDAV or HXFRDN interrupts */

  max3421e_int_disable(priv, USBHOST_HIRQ_SNDBAVIRQ |
                             USBHOST_HIRQ_RCVDAVIRQ |
                             USBHOST_HIRQ_HXFRDNIRQ);

  /* Save the endpoint toggle settings.
   *
   * REVISIT:  The MAX4321E sends fixed DATA0 and DATA1 PID tokens for the
   * various stages of a CONTROL transfer, regardless of the setting of
   * the internal data toggle.
   */

  max3421e_save_toggles(priv, chan);

  /* Wake up any waiters for the end of transfer event */

  DEBUGASSERT(priv->waiter != NULL);
  max3421e_chan_wakeup(priv, chan, -result);
}

/****************************************************************************
 * Name: max3421e_put_sndfifo
 *
 * Description:
 *   Copy data from the user-provided buffer into the SNDFIFO.
 *
 * Assumptions:
 *   The SPI bus is locked.
 *   There is data to be remaining to be sent.
 *   The caller has already set up for the wait event.
 *
 ****************************************************************************/

static void max3421e_put_sndfifo(FAR struct max3421e_usbhost_s *priv,
                                 FAR struct max3421e_chan_s *chan)
{
  uint16_t committed;
  uint16_t wrsize;
  uint8_t maxpacket;
  int i;

  DEBUGASSERT(priv != NULL && chan != NULL);

  /* The SNDFIFO is double buffered.  We may load up to MAX3421E_SNDFIFO_SIZE
   * into one buffer.  After loading the SNDFIFO buffer, the write the SNDBC
   * (Send Byte Count) register with the number of bytes loaded.  The
   * MAX3421E will clear SNDBAVIRQ (Send Buffer Available IRQ) and commit
   * the FIFO to USB transmission.
   *
   * If the other buffer is available when SNDBC is written, the MAX3421E
   * will clear SNDBAVIRQ then immediately set it to indicate
   * availability of the second buffer.
   *
   * The CPU should load the SNDFIFO only when a SNDBAVIRQ = 1.
   */

  maxpacket = chan->maxpacket;
  DEBUGASSERT(maxpacket <= MAX3421E_SNDFIFO_SIZE);

  committed = priv->xfrd + (uint16_t)priv->inflight;
  DEBUGASSERT(committed < priv->buflen);

  for (i = 0;
       i < 2 && committed < priv->buflen &&
       (max3421e_int_status(priv) & USBHOST_HIRQ_SNDBAVIRQ) != 0;
       i++)
    {
      /* Get the size of the biggest thing that we can put in the current
       * SNDFIFO buffer.
       */

      wrsize  = priv->buflen - committed;
      if (wrsize > maxpacket)
        {
          wrsize = maxpacket;
        }

      /* Write packet into the SNDFIFO. */

      max3421e_sndblock(priv, MAX3421E_USBHOST_SNDFIFO,
                        priv->buffer + committed, wrsize);

      /* Write the byte count to the SNDBC register */

      max3421e_putreg(priv, MAX3421E_USBHOST_SNDBC, wrsize);

      /* Send the OUT token */

      max3421e_putreg(priv, MAX3421E_USBHOST_HXFR,
                      USBHOST_HXFR_TOKEN_OUT | chan->epno);

      /* Increment the count of bytes "in-flight" in the SNDFIFO */

      priv->inflight += wrsize;
      committed       = priv->xfrd + wrsize;
    }

  /* Enable the SNDFIFO interrupt to handle the completion/continuation
   * of transfer.  Enable the HXFRDNIRQ to catch NAKs and transfer errors.
   */

  priv->xfrtype = HXFRDN_SNDFIFO;
  max3421e_int_enable(priv, USBHOST_HIRQ_SNDBAVIRQ | USBHOST_HIRQ_HXFRDNIRQ);
}

/****************************************************************************
 * Name: max3421e_send_continue
 *
 * Description:
 *   Continue the send operation started by max3421e_send_start().  If
 *   max3421e_put_sndfifo() was unable to load the entire outgoing buffer
 *   into the SNDFIFO, it enabled SNDBAV interrupt.
 *
 *   When the SNDBAV interrupt occurred, max3421e_irqwork() disabled that
 *   interrupt and called this function in order to continue that long send
 *   operations.
 *
 * Assumptions:
 *   The SPI bus is locked.
 *   The SNDBAV interrupt has been disabled.
 *
 ****************************************************************************/

static void max3421e_send_continue(FAR struct max3421e_usbhost_s *priv)
{
  FAR struct max3421e_chan_s *chan;
  int result;
  uint8_t xfrd;

  DEBUGASSERT(priv != NULL && priv->waiter != NULL);
  chan = priv->waiter;

  /* Check the result of a transfer */

  result = max3421e_transfer_status(priv);
  if (result < 0)
    {
      /* Terminate the transfer on any error. */

      max3421e_transfer_terminate(priv, chan, result);
      return;
    }

  /* Update the number of bytes transferred.  We have to be a little clever
   * here:  We do not keep track of the number of bytes sent in each of the
   * SNDFIFO buffers, rather only the outstanding number of buffered
   * transferred, 'inflight'.  However, we know that all transfers will be
   * the max packet size (other than the last one perhaps).
   */

  /* If the number inflight is strictly greater than the maxpacket size, then
   * we can infer that the transfer that just completed was maxpacket size.
   */

  if (priv->inflight > chan->maxpacket)
    {
      xfrd = chan->maxpacket;
    }

  /* If the number inflight is less than chan->maxpacket, then that must have
   * been the last packet of the transfer.
   */

  else if (priv->inflight < chan->maxpacket)
    {
      xfrd = priv->inflight;
      DEBUGASSERT((priv->xfrd + xfrd) == priv->buflen);
    }

  /* If the number inflight is exactly the maxpacket size and the transfer
   * is not yet finished, then the the transfer size must have been max
   * packet size.
   */

  else if ((priv->xfrd + chan->maxpacket) >= priv->buflen)
    {
      xfrd = priv->buflen - priv->xfrd;
      DEBUGASSERT((priv->xfrd + xfrd) == priv->buflen);
    }

  /* Otherwise, the transfer must have been the max packet size */

  else
    {
      xfrd = chan->maxpacket;
    }

  priv->xfrd     += xfrd;
  priv->inflight -= xfrd;

  /* Check for the end of transfer */

  if (priv->xfrd >= priv->buflen)
    {
      /* Successful end-of-transfer */

      max3421e_transfer_terminate(priv, chan, OK);
    }
  else
    {
      /* No.. there are more bytes to be sent */

      max3421e_put_sndfifo(priv, chan);
    }
}

/****************************************************************************
 * Name: max3421e_send_start
 *
 * Description:
 *   Start at transfer on the selected IN or OUT channel.
 *
 ****************************************************************************/

static void max3421e_send_start(FAR struct max3421e_usbhost_s *priv,
                                FAR struct max3421e_chan_s *chan)
{
  max3421e_pktdump("Sending", priv->buffer, priv->buflen);

  /* Set up the initial state of the transfer */

  usbhost_vtrace2(MAX3421E_VTRACE2_STARTTRANSFER1,
                  chan->chidx, priv->buflen);

  priv->result   = EBUSY;
  priv->inflight = 0;
  priv->xfrd     = 0;

  /* Make sure the peripheral address is correct */

  max3421e_putreg(priv, MAX3421E_USBHOST_PERADDR, chan->funcaddr);

  /* Checkout for zero length packet */

  if (priv->buflen > 0)
    {
      /* No.. we need to copy the outgoing data into SNDFIFO. */

      max3421e_restore_toggles(priv, chan);

      /* Then load the data into the SNDFIFO and start the transfer */

      max3421e_put_sndfifo(priv, chan);
    }
  else
    {
      /* Write the zero byte count to the SNDBC register */

      max3421e_putreg(priv, MAX3421E_USBHOST_SNDBC, 0);

      /* Send the OUT token */

      max3421e_putreg(priv, MAX3421E_USBHOST_HXFR,
                      USBHOST_HXFR_TOKEN_OUT | chan->epno);

      /* Enable the HXFRDNIRQ to catch completion of the ZLP. */

      priv->xfrtype = HXFRDN_SNDZLP;
      max3421e_int_enable(priv, USBHOST_HIRQ_HXFRDNIRQ);
    }
}

/****************************************************************************
 * Name: max3421e_out_transfer
 *
 * Description:
 *   Transfer the 'buflen' bytes in 'buffer' through an OUT channel.
 *
 * Assumptions:
 *   The SPI is not locked.  This function is called only from the TRANSFER
 *   interface and must manage the SPI lock itself.  The lock, for example,
 *   must be relinquished before waiting.
 *
 ****************************************************************************/

static ssize_t max3421e_out_transfer(FAR struct max3421e_usbhost_s *priv,
                                     FAR struct max3421e_chan_s *chan,
                                     FAR uint8_t *buffer, size_t buflen)
{
  clock_t start;
  clock_t elapsed;
  size_t xfrlen;
  ssize_t xfrd;
  int ret;

  /* Loop until the transfer completes (i.e., buflen is decremented to zero)
   * or a fatal error occurs (any error other than a simple NAK)
   */

  start = clock_systime_ticks();
  xfrd  = 0;

  while (buflen > 0)
    {
      /* Transfer one packet at a time.  The hardware is capable of queueing
       * multiple OUT packets, but I just haven't figured out how to handle
       * the case where a single OUT packet in the group is NAKed.
       */

      xfrlen       = MIN(chan->maxpacket, buflen);
      priv->buffer = buffer;
      priv->buflen = xfrlen;
      priv->xfrd   = 0;

      /* Set up for the wait BEFORE starting the transfer */

      ret = max3421e_chan_waitsetup(priv, chan);
      if (ret < 0)
        {
          usbhost_trace1(MAX3421E_TRACE1_DEVDISCONN1, 0);
          return (ssize_t)ret;
        }

      /* Set up for the transfer based on the direction and the endpoint */

      max3421e_lock(priv);
      ret = max3421e_out_setup(priv, chan);
      max3421e_unlock(priv);

      if (ret < 0)
        {
          usbhost_trace1(MAX3421E_TRACE1_OUTSETUP_FAIL1, -ret);
          return (ssize_t)ret;
        }

      /* Wait for the transfer to complete and get the result */

      ret = max3421e_chan_wait(priv, chan);

      /* Handle transfer failures */

      if (ret < 0)
        {
          usbhost_trace1(MAX3421E_TRACE1_TRANSFER_FAILED1, ret);

          /* Check for a special case:  If (1) the transfer was NAKed and (2)
           * no SNDFIFO empty or Rx FIFO not-empty event occurred, then we
           * should be able to just flush the Rx and SNDFIFOs and try again.
           * We can detect this latter case because then the transfer buffer
           * pointer and buffer size will be unaltered.
           */

          elapsed = clock_systime_ticks() - start;
          if (ret != -EAGAIN ||                     /* Not a NAK condition OR */
              elapsed >= MAX3421E_DATANAK_DELAY ||  /* Timeout has elapsed OR */
              priv->xfrd > 0)                       /* Data has been partially transferred */
            {
              /* Break out and return the error */

              usbhost_trace1(MAX3421E_TRACE1_CHANWAIT_FAIL, -ret);
              return (ssize_t)ret;
            }

          /* Get the device a little time to catch up.  Then retry the
           * transfer using the same buffer pointer and length.
           */

          nxsig_usleep(20 * 1000);
        }
      else
        {
          /* Successfully transferred. Update the buffer pointer/length */

          buffer += xfrlen;
          buflen -= xfrlen;
          xfrd   += priv->xfrd;
        }
    }

  return xfrd;
}

/****************************************************************************
 * Name: max3421e_out_next
 *
 * Description:
 *   Initiate the next of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is always called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void max3421e_out_next(FAR struct max3421e_usbhost_s *priv,
                           FAR struct max3421e_chan_s *chan)
{
  usbhost_asynch_t callback;
  FAR void *arg;
  ssize_t nbytes;
  int result;
  int ret;

  /* Is the full transfer complete? Did the last chunk transfer OK? */

  result = -(int)priv->result;
  if (priv->xfrd < priv->buflen && result == OK)
    {
      /* Yes.. Set up for the next transfer based on the direction and the
       * endpoint type
       */

      ret = max3421e_out_setup(priv, chan);
      if (ret >= 0)
        {
          return;
        }

      usbhost_trace1(MAX3421E_TRACE1_OUTSETUP_FAIL2, -ret);
      result = ret;
    }

  /* The transfer is complete, with or without an error */

  usbhost_vtrace1(MAX3421E_VTRACE1_TRANSFER_COMPLETE, result);

  /* Extract the callback information */

  callback       = priv->callback;
  arg            = priv->arg;
  nbytes         = priv->xfrd;

  priv->callback = NULL;
  priv->arg      = NULL;
  priv->xfrd     = 0;

  /* Then perform the callback */

  if (result < 0)
    {
      nbytes = (ssize_t)result;
    }

  callback(arg, nbytes);
}
#endif

/****************************************************************************
 * Name: max3421e_out_asynch
 *
 * Description:
 *   Initiate the first of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   The SPI is not locked.  This function is called only from the ASYNCH
 *   interface and must manage the SPI lock itself.  The lock, for example,
 *   must be relinquished before waiting.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int max3421e_out_asynch(FAR struct max3421e_usbhost_s *priv,
                               FAR struct max3421e_chan_s *chan,
                               FAR uint8_t *buffer, size_t buflen,
                               usbhost_asynch_t callback, FAR void *arg)
{
  int ret;

  /* Set up for the transfer BEFORE starting the first transfer */

  priv->buffer = buffer;
  priv->buflen = buflen;
  priv->xfrd   = 0;

  ret = max3421e_chan_asynchsetup(priv, chan, callback, arg);
  if (ret < 0)
    {
      usbhost_trace1(MAX3421E_TRACE1_ASYNCHSETUP_FAIL1, -ret);
      return ret;
    }

  /* Set up for the transfer based on the direction and the endpoint type */

  max3421e_lock(priv);
  ret = max3421e_out_setup(priv, chan);
  max3421e_unlock(priv);

  if (ret < 0)
    {
      usbhost_trace1(MAX3421E_TRACE1_OUTSETUP_FAIL3, -ret);
    }

  /* And return with the transfer pending */

  return ret;
}
#endif

/****************************************************************************
 * Name: max3421e_ctrl_sendsetup
 *
 * Description:
 *   Send an IN/OUT SETUP packet.
 *
 * Assumptions:
 *   The SPI is not locked.  This function is called only from the CTRLIN and
 *   CTRLOUT interfaces and must manage the SPI lock itself.  The lock, for
 *   example, must be relinquished before waiting.
 *
 ****************************************************************************/

static int max3421e_ctrl_sendsetup(FAR struct max3421e_usbhost_s *priv,
                                   FAR struct max3421e_chan_s *chan,
                                   FAR const struct usb_ctrlreq_s *req)
{
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Loop while the device reports NAK (and a timeout is not exceeded */

  start = clock_systime_ticks();
  do
    {
      /* Send the  SETUP packet */

      priv->buffer   = (FAR uint8_t *)req;
      priv->buflen   = USB_SIZEOF_CTRLREQ;
      priv->inflight = 0;
      priv->xfrd     = 0;
      priv->result   = EBUSY;

      /* Set up for the wait BEFORE starting the transfer */

      ret = max3421e_chan_waitsetup(priv, chan);
      if (ret < 0)
        {
          usbhost_trace1(MAX3421E_TRACE1_DEVDISCONN2, 0);
          return ret;
        }

      /* Make sure the peripheral address is correct */

      max3421e_lock(priv);
      max3421e_putreg(priv, MAX3421E_USBHOST_PERADDR, chan->funcaddr);

      /* Write packet into the SUDFIFO. */

      max3421e_sndblock(priv, MAX3421E_USBHOST_SUDFIFO, priv->buffer,
                        priv->buflen);

      /* Send the SETUP token (always EP0) */

      max3421e_putreg(priv, MAX3421E_USBHOST_HXFR,
                      USBHOST_HXFR_TOKEN_SETUP);

      /* Increment the count of bytes "in-flight" in the SNDFIFO */

      priv->inflight = priv->buflen;

      /* The MAX3421E waits 18 bit times for the device to respond or time
       * out then terminates the transfer by asserting the HXFRDNIRQ and
       * updating the HSRLT bits.  NOTE:  The USB spec says that a
       * peripheral must always ACK a SETUP packet.
       */

      priv->xfrtype = HXFRDN_SETUP;
      max3421e_int_enable(priv, USBHOST_HIRQ_HXFRDNIRQ);

      /* Wait for the transfer to complete */

      max3421e_unlock(priv);
      ret = max3421e_chan_wait(priv, chan);

      /* Return on success and for all failures other than EAGAIN.  EAGAIN
       * means that the device NAKed the SETUP command and that we should
       * try a few more times.  NOTE:  The USB spec says that a peripheral
       * must always ACK a SETUP packet.
       */

      if (ret != -EAGAIN)
        {
          /* Output some debug information if the transfer failed */

          if (ret < 0)
            {
              usbhost_trace1(MAX3421E_TRACE1_TRANSFER_FAILED2, ret);
            }

          /* Return the result in any event */

          return ret;
        }

      /* Get the elapsed time (in frames) */

      elapsed = clock_systime_ticks() - start;
    }
  while (elapsed < MAX3421E_SETUP_DELAY);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: max3421e_ctrl_senddata
 *
 * Description:
 *   Send data in the data phase of an OUT control transfer.  Or send status
 *   in the status phase of an IN control transfer
 *
 * Assumptions:
 *   The SPI is not locked.  This function is called only from the CTRLOUT
 *   interface and must manage the SPI lock itself.  The lock, for example,
 *   must be relinquished before waiting.
 *
 ****************************************************************************/

static int max3421e_ctrl_senddata(FAR struct max3421e_usbhost_s *priv,
                                  FAR struct max3421e_chan_s *chan,
                                  FAR uint8_t *buffer, unsigned int buflen)
{
  int ret;

  /* Save buffer information */

  chan->in     = false;
  priv->xfrd   = 0;
  priv->buffer = buffer;
  priv->buflen = buflen;

  /* The MAX4321E sends fixed DATA0 and DATA1 PID tokens for the various
   * stages of a CONTROL transfer, regardless of the setting of the internal
   * data toggle.
   */

  /* Set up for the wait BEFORE starting the transfer */

  ret = max3421e_chan_waitsetup(priv, chan);
  if (ret < 0)
    {
      usbhost_trace1(MAX3421E_TRACE1_DEVDISCONN3, 0);
      return ret;
    }

  /* Start the transfer */

  max3421e_lock(priv);
  max3421e_send_start(priv, chan);
  max3421e_unlock(priv);

  /* Wait for the transfer to complete and return the result */

  return max3421e_chan_wait(priv, chan);
}

/****************************************************************************
 * Name: max3421e_ctrl_recvdata
 *
 * Description:
 *   Receive data in the data phase of an IN control transfer.  Or receive
 *   status in the status phase of an OUT control transfer
 *
 * Assumptions:
 *   The SPI is not locked.  This function is called only from the CTRLIN
 *   interface and must manage the SPI lock itself.  The lock, for example,
 *   must be relinquished before waiting.
 *
 ****************************************************************************/

static int max3421e_ctrl_recvdata(FAR struct max3421e_usbhost_s *priv,
                                  FAR struct max3421e_chan_s *chan,
                                  FAR uint8_t *buffer, unsigned int buflen)
{
  int ret;

  /* Save buffer information */

  chan->in     = true;
  priv->buffer = buffer;
  priv->buflen = buflen;
  priv->xfrd   = 0;

  /* The MAX4321E sends fixed DATA0 and DATA1 PID tokens for the various
   * stages of a CONTROL transfer, regardless of the setting of the internal
   * data toggle.
   */

  /* Set up for the wait BEFORE starting the transfer */

  ret = max3421e_chan_waitsetup(priv, chan);
  if (ret < 0)
    {
      usbhost_trace1(MAX3421E_TRACE1_DEVDISCONN4, 0);
      return ret;
    }

  /* Start the transfer */

  max3421e_lock(priv);
  max3421e_recv_start(priv, chan);
  max3421e_unlock(priv);

  /* Wait for the transfer to complete and return the result */

  return max3421e_chan_wait(priv, chan);
}

/****************************************************************************
 * Name: max3421e_ctrl_sendstatus
 *
 * Description:
 *   Send status to complete the status phase of a CTRLIN transfer.
 *
 * Assumptions:
 *   The SPI is not locked.  This function is called only from the CTRLIN
 *   interface and must manage the SPI lock itself.  The lock, for example,
 *   must be relinquished before waiting.
 *
 ****************************************************************************/

static int max3421e_ctrl_sendstatus(FAR struct max3421e_usbhost_s *priv,
                                    FAR struct max3421e_chan_s *chan)
{
  /* Set up the initial state of the transfer */

  usbhost_vtrace2(MAX3421E_VTRACE2_SENDSTATUS, chan->chidx, priv->buflen);

  priv->result   = EBUSY;
  priv->inflight = 0;
  priv->buflen   = 0;
  priv->xfrd     = 0;

  /* Make sure the peripheral address is correct */

  max3421e_lock(priv);
  max3421e_putreg(priv, MAX3421E_USBHOST_PERADDR, chan->funcaddr);

  /* Write the zero byte count to the SNDBC register */

  max3421e_putreg(priv, MAX3421E_USBHOST_SNDBC, 0);

  /* Send the HS-OUT token */

  max3421e_putreg(priv, MAX3421E_USBHOST_HXFR,
                  USBHOST_HXFR_TOKEN_OUTHS | chan->epno);

  /* Enable the HXFRDNIRQ to catch completion of the ZLP. */

  priv->xfrtype = HXFRDN_SNDZLP;
  max3421e_int_enable(priv, USBHOST_HIRQ_HXFRDNIRQ);
  max3421e_unlock(priv);

  /* Wait for the transfer to complete and return the result */

  return max3421e_chan_wait(priv, chan);
}

/****************************************************************************
 * Name: max3421e_ctrl_recvstatus
 *
 * Description:
 *   Receive status to complete the status phase of a CTRLOUT transfer.
 *
 * Assumptions:
 *   The SPI is not locked.  This function is called only from the CTRLOUT
 *   interface and must manage the SPI lock itself.  The lock, for example,
 *   must be relinquished before waiting.
 *
 ****************************************************************************/

static int max3421e_ctrl_recvstatus(FAR struct max3421e_usbhost_s *priv,
                                    FAR struct max3421e_chan_s *chan)
{
  max3421e_pktdump("Sending", priv->buffer, priv->buflen);

  /* Set up the initial state of the transfer */

  usbhost_vtrace2(MAX3421E_VTRACE2_RECVSTATUS, chan->chidx, priv->buflen);

  priv->result   = EBUSY;
  priv->inflight = 0;
  priv->buflen   = 0;
  priv->xfrd     = 0;

  /* Make sure the peripheral address is correct */

  max3421e_lock(priv);
  max3421e_putreg(priv, MAX3421E_USBHOST_PERADDR, chan->funcaddr);

  /* Send the HS-IN token. */

  max3421e_putreg(priv, MAX3421E_USBHOST_HXFR,
                  USBHOST_HXFR_TOKEN_INHS | chan->epno);

  /* Enable the RCVFIFO interrupt to handle the completion/continuation
   * of transfer.  Enable the HXFRDNIRQ to catch NAKs and transfer
   * errors.
   */

  priv->xfrtype = HXFRDN_RCVFIFO;
  max3421e_int_enable(priv, USBHOST_HIRQ_RCVDAVIRQ | USBHOST_HIRQ_HXFRDNIRQ);
  max3421e_unlock(priv);

  /* Wait for the transfer to complete and return the result */

  return max3421e_chan_wait(priv, chan);
}

/****************************************************************************
 * Name: max3421e_in_setup
 *
 * Description:
 *   Initiate an IN transfer on an bulk, interrupt, or isochronous pipe.
 *
 ****************************************************************************/

static int max3421e_in_setup(FAR struct max3421e_usbhost_s *priv,
                             FAR struct max3421e_chan_s *chan)
{
  /* Set up for the transfer based on the direction and the endpoint type */

  switch (chan->eptype)
    {
      default:
      case USB_EP_ATTR_XFER_CONTROL: /* Control */
        {
          /* This kind of transfer on control endpoints other than EP0 are
           * not currently supported
           */

          return -ENOSYS;
        }

      case USB_EP_ATTR_XFER_ISOC: /* Isochronous */
        {
          /* Set up the IN DATA0 PID */

          usbhost_vtrace2(MAX3421E_VTRACE2_ISOCIN,
                          chan->chidx, priv->buflen);

          chan->toggles = USBHOST_HCTL_RCVTOG0 | USBHOST_HCTL_SNDTOG0;
        }
        break;

      case USB_EP_ATTR_XFER_BULK: /* Bulk */
        {
          usbhost_vtrace2(MAX3421E_VTRACE2_BULKIN,
                          chan->chidx, priv->buflen);
        }
        break;

      case USB_EP_ATTR_XFER_INT: /* Interrupt */
        {
          usbhost_vtrace2(MAX3421E_VTRACE2_INTRIN,
                          chan->chidx, priv->buflen);
        }
        break;
    }

  /* Start the transfer. */

  max3421e_recv_start(priv, chan);
  return OK;
}

/****************************************************************************
 * Name: max3421e_out_setup
 *
 * Description:
 *   Initiate an OUT transfer on an bulk, interrupt, or isochronous pipe.
 *
 ****************************************************************************/

static int max3421e_out_setup(FAR struct max3421e_usbhost_s *priv,
                              FAR struct max3421e_chan_s *chan)
{
  /* Set up for the transfer based on the direction and the endpoint type */

  switch (chan->eptype)
    {
      default:
      case USB_EP_ATTR_XFER_CONTROL: /* Control */
        {
          /* This kind of transfer on control endpoints other than EP0 are
           * not currently supported
           */

          return -ENOSYS;
        }

      case USB_EP_ATTR_XFER_ISOC: /* Isochronous */
        {
          /* Set up the IN DATA0 PID */

          usbhost_vtrace2(MAX3421E_VTRACE2_ISOCOUT,
                          chan->chidx, priv->buflen);

          chan->toggles = USBHOST_HCTL_RCVTOG0 | USBHOST_HCTL_SNDTOG0;
        }
        break;

      case USB_EP_ATTR_XFER_BULK: /* Bulk */
        {
          usbhost_vtrace2(MAX3421E_VTRACE2_BULKOUT,
                          chan->chidx, priv->buflen);
        }
        break;

      case USB_EP_ATTR_XFER_INT: /* Interrupt */
        {
          usbhost_vtrace2(MAX3421E_VTRACE2_INTROUT,
                          chan->chidx, priv->buflen);
        }
        break;
    }

  /* Start the transfer */

  max3421e_send_start(priv, chan);
  return OK;
}

/****************************************************************************
 * Name: max3421e_get_rcvfifo
 *
 * Description:
 *   Copy data from the RCVFIFO to the user-provided buffer.
 *
 *   The RCVFIFO is double buffered.  If another packet is available in the
 *   other buffer, the MAX3421E will immediately re-assert RCVDAVIRQ and we
 *   will catch that by interrupt handling logic.
 *
 * Assumptions:
 *   The SPI bus is locked.
 *
 ****************************************************************************/

static uint8_t max3421e_get_rcvfifo(FAR struct max3421e_usbhost_s *priv,
                                    FAR struct max3421e_chan_s *chan)
{
  uint16_t remaining;
  uint8_t navail;
  uint8_t nrcvd;

  /* Get the number of bytes available in the RCVFIFO */

  navail = max3421e_getreg(priv, MAX3421E_USBHOST_RCVBC);

  /* Get the number that will fit into the user-provided buffer */

  remaining = priv->buflen - priv->xfrd;
  if (navail > remaining)
    {
      nrcvd = remaining;
    }
  else
    {
      nrcvd = navail;
    }

  /* Read the received data into the user-buffer. */

  max3421e_recvblock(priv, MAX3421E_USBHOST_RCVFIFO,
                     priv->buffer + priv->xfrd, nrcvd);

  /* Update the number of bytes transferred */

  priv->xfrd += nrcvd;

  /* Discard any byte remaining in the RCVFIFO */

  /* REVISIT:  Is this necessary?  Or the MAX3421E automatically discard any
   * unread data?
   */

#if 0
  if (nrcvd < navail)
    {
      max3421e_discard()
    }
#endif

  return nrcvd;
}

/****************************************************************************
 * Name: max3421e_recv_restart
 *
 * Description:
 *   Start/Re-start the transfer on the selected IN or OUT channel.
 *
 ****************************************************************************/

static void max3421e_recv_restart(FAR struct max3421e_usbhost_s *priv,
                                  FAR struct max3421e_chan_s *chan)
{
  /* Send the IN token. */

  max3421e_putreg(priv, MAX3421E_USBHOST_HXFR,
                  USBHOST_HXFR_TOKEN_IN | chan->epno);

  /* Enable the RCVFIFO interrupt to handle the completion/continuation
   * of transfer.  Enable the HXFRDNIRQ to catch NAKs and transfer
   * errors.
   */

  priv->xfrtype = HXFRDN_RCVFIFO;
  max3421e_int_enable(priv, USBHOST_HIRQ_RCVDAVIRQ | USBHOST_HIRQ_HXFRDNIRQ);
}

/****************************************************************************
 * Name: max3421e_recv_continue
 *
 * Description:
 *   Continue the receive operation started by max3421e_recv_start().  This
 *   function is called from the interrupt handler worker when an interrupt
 *   indicates that new, incoming data is available in the RCVFIFO (RCVDAV)
 *
 *   When the RCBBAV interrupt occurred, max3421e_irqwork() disabled that
 *   interrupt and called this function in order to handle the receipt of
 *   data
 *
 * Assumptions:
 *   The SPI bus is locked.
 *   The RCVDAV interrupt has been disabled.
 *
 ****************************************************************************/

static void max3421e_recv_continue(FAR struct max3421e_usbhost_s *priv)
{
  FAR struct max3421e_chan_s *chan;
  uint8_t nrcvd;
  int result;

  DEBUGASSERT(priv != NULL && priv->waiter != NULL);
  chan = priv->waiter;

  /* Check the result of a transfer */

  result = max3421e_transfer_status(priv);
  if (result < 0)
    {
      /* Terminate the transfer on any error. */

      max3421e_transfer_terminate(priv, chan, result);
      return;
    }

  /* Transfer the data from the RCVFIFO to the user buffer */

  nrcvd = max3421e_get_rcvfifo(priv, chan);

  /* A partial or zero-length packet is an indication that the transfer
   * completed early.  Terminate the transfer in those cases OR if all
   * of the requested data has been received
   */

  if (nrcvd < chan->maxpacket || priv->xfrd >= priv->buflen)
    {
      max3421e_transfer_terminate(priv, chan, OK);
    }

  /* If not all of the data has been received, then setup to receive
   * another packet.
   */

  else
    {
      max3421e_recv_restart(priv, chan);
    }
}

/****************************************************************************
 * Name: max3421e_recv_start
 *
 * Description:
 *   Start at transfer on the selected IN or OUT channel.
 *
 * Assumptions:
 *   The caller has the SPI locked.
 *
 ****************************************************************************/

static void max3421e_recv_start(FAR struct max3421e_usbhost_s *priv,
                                FAR struct max3421e_chan_s *chan)
{
  max3421e_pktdump("Sending", priv->buffer, priv->buflen);

  /* Set up the initial state of the transfer */

  usbhost_vtrace2(MAX3421E_VTRACE2_STARTTRANSFER2,
                  chan->chidx, priv->buflen);

  priv->result   = EBUSY;
  priv->inflight = 0;
  priv->xfrd     = 0;

  /* Make sure the peripheral address is correct */

  max3421e_putreg(priv, MAX3421E_USBHOST_PERADDR, chan->funcaddr);

  /* Start the transfer. */

  max3421e_recv_restart(priv, chan);
}

/****************************************************************************
 * Name: max3421e_in_transfer
 *
 * Description:
 *   Transfer 'buflen' bytes into 'buffer' from an IN channel.
 *
 * Assumptions:
 *   The SPI is not locked.  This function is called only from the TRANSFER
 *   interface and must manage the SPI lock itself.  The lock, for example,
 *   must be relinquished before waiting.
 *
 ****************************************************************************/

static ssize_t max3421e_in_transfer(FAR struct max3421e_usbhost_s *priv,
                                    FAR struct max3421e_chan_s *chan,
                                    FAR uint8_t *buffer, size_t buflen)
{
  clock_t start;
  ssize_t xfrd;
  int ret;

  /* Loop until the transfer completes (i.e., buflen is decremented to zero)
   * or a fatal error occurs any error other than a simple NAK.  NAK would
   * simply indicate the end of the transfer (short-transfer).
   */

  priv->buffer = buffer;
  priv->buflen = buflen;
  priv->xfrd   = 0;
  xfrd         = 0;

  start = clock_systime_ticks();
  while (priv->xfrd < priv->buflen)
    {
      /* Set up for the wait BEFORE starting the transfer */

      ret = max3421e_chan_waitsetup(priv, chan);
      if (ret < 0)
        {
          usbhost_trace1(MAX3421E_TRACE1_DEVDISCONN5, 0);
          return (ssize_t)ret;
        }

      /* Set up for the transfer based on the direction and the endpoint */

      max3421e_lock(priv);
      ret = max3421e_in_setup(priv, chan);
      max3421e_unlock(priv);

      if (ret < 0)
        {
          usbhost_trace1(MAX3421E_TRACE1_INSETUP_FAIL1, -ret);
          return (ssize_t)ret;
        }

      /* Wait for the transfer to complete and get the result */

      ret = max3421e_chan_wait(priv, chan);

      /* EAGAIN indicates that the device NAKed the transfer. */

      if (ret < 0)
        {
          /* The transfer failed.  If we received a NAK, return all data
           * buffered so far (if any).
           */

          if (ret == -EAGAIN)
            {
              /* Was data buffered prior to the NAK? */

              if (xfrd > 0)
                {
                  /* Yes, return the amount of data received.
                   *
                   * REVISIT: This behavior is clearly correct for CDC/ACM
                   * bulk transfers and HID interrupt transfers.  But I am
                   * not so certain for MSC bulk transfers which, I think,
                   * could have NAKed packets in the middle of a transfer.
                   */

                  return xfrd;
                }
              else
                {
                  useconds_t delay;

                  /* Get the elapsed time.  Has the timeout elapsed?
                   * if not then try again.
                   */

                  clock_t elapsed = clock_systime_ticks() - start;
                  if (elapsed >= MAX3421E_DATANAK_DELAY)
                    {
                      /* Timeout out... break out returning the NAK as
                       * as a failure.
                       */

                      return (ssize_t)ret;
                    }

                  /* Wait a bit before retrying after a NAK. */

                  if (chan->eptype == USB_EP_ATTR_XFER_INT)
                    {
                      /* For interrupt (and isochronous) endpoints, the
                       * polling rate is determined by the bInterval field
                       * of the endpoint descriptor (in units of frames
                       * which we treat as milliseconds here).
                       */

                      if (chan->interval > 0)
                        {
                          /* Convert the delay to units of microseconds */

                          delay = (useconds_t)chan->interval * 1000;
                        }
                      else
                        {
                          /* Out of range! For interrupt endpoints, the valid
                           * range is 1-255 frames.  Assume one frame.
                           */

                          delay = 1000;
                        }
                    }
                  else
                    {
                      /* For Isochronous endpoints, bInterval must be 1.
                       * Bulk endpoints do not have a polling interval.
                       * Rather, the should wait until data is received.
                       *
                       * REVISIT:  For bulk endpoints this 1 msec delay is
                       * only intended to give the CPU a break from the bulk
                       * EP tight polling loop.  But are there performance
                       * issues?
                       */

                      delay = 1000;
                    }

                  /* Wait for the next polling interval.  For interrupt and
                   * isochronous endpoints, this is necessary to assure the
                   * polling interval.  It is used in other cases only to
                   * prevent the polling from consuming too much CPU
                   * bandwidth.
                   *
                   * Small delays could require more resolution than is
                   * provided by the system timer.  For example, if the
                   * system timer resolution is 10MS, then nxsig_usleep(1000)
                   * will actually request a delay 20MS (due to both
                   * quantization and rounding).
                   *
                   * REVISIT: So which is better?  To ignore tiny delays and
                   * hog the system bandwidth?  Or to wait for an excessive
                   * amount and destroy system throughput?
                   */

                  if (delay > CONFIG_USEC_PER_TICK)
                    {
                      nxsig_usleep(delay - CONFIG_USEC_PER_TICK);
                    }
                }
            }
          else
            {
              /* Some unexpected, fatal error occurred. */

              usbhost_trace1(MAX3421E_TRACE1_TRANSFER_FAILED3, -ret);

              /* Break out and return the error */

              return (ssize_t)ret;
            }
        }
      else
        {
          /* Successfully received another chunk of data... add that to the
           * running total.  Then continue reading until we read 'buflen'
           * bytes of data or until the devices NAKs (implying a short
           * packet).
           */

          xfrd += priv->xfrd;
        }
    }

  return xfrd;
}

/****************************************************************************
 * Name: max3421e_in_next
 *
 * Description:
 *   Initiate the next of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is always called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void max3421e_in_next(FAR struct max3421e_usbhost_s *priv,
                             FAR struct max3421e_chan_s *chan)
{
  usbhost_asynch_t callback;
  FAR void *arg;
  ssize_t nbytes;
  int result;
  int ret;

  /* Is the full transfer complete? Did the last chunk transfer OK? */

  result = -(int)priv->result;
  if (priv->xfrd < priv->buflen && result == OK)
    {
      /* Yes.. Set up for the next transfer based on the direction and the
       * endpoint type
       */

      ret = max3421e_in_setup(priv, chan);
      if (ret >= 0)
        {
          return;
        }

      usbhost_trace1(MAX3421E_TRACE1_INSETUP_FAIL2, -ret);
      result = ret;
    }

  /* The transfer is complete, with or without an error */

  usbhost_vtrace2(MAX3421E_VTRACE2_XFRCOMPLETE,
                  (unsigned int)ep, buflen);

  /* Extract the callback information */

  callback       = priv->callback;
  arg            = priv->arg;
  nbytes         = priv->xfrd;

  priv->callback = NULL;
  priv->arg      = NULL;
  priv->xfrd     = 0;

  /* Then perform the callback */

  if (result < 0)
    {
      nbytes = (ssize_t)result;
    }

  callback(arg, nbytes);
}
#endif

/****************************************************************************
 * Name: max3421e_in_asynch
 *
 * Description:
 *   Initiate the first of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   The SPI is not locked.  This function is called only from the ASYNCH
 *   interface and must manage the SPI lock itself.  The lock, for example,
 *   must be relinquished before waiting.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int max3421e_in_asynch(FAR struct max3421e_usbhost_s *priv,
                              FAR struct max3421e_chan_s *chan,
                              FAR uint8_t *buffer, size_t buflen,
                              usbhost_asynch_t callback, FAR void *arg)
{
  int ret;

  /* Set up for the transfer BEFORE starting the first transfer */

  priv->buffer = buffer;
  priv->buflen = buflen;
  priv->xfrd   = 0;

  ret = max3421e_chan_asynchsetup(priv, chan, callback, arg);
  if (ret < 0)
    {
      usbhost_trace1(MAX3421E_TRACE1_ASYNCHSETUP_FAIL2, -ret);
      return ret;
    }

  /* Set up for the transfer based on the direction and the endpoint type */

  max3421e_lock(priv);
  ret = max3421e_in_setup(priv, chan);
  max3421e_unlock(priv);

  if (ret < 0)
    {
      usbhost_trace1(MAX3421E_TRACE1_INSETUP_FAIL3, -ret);
    }

  /* And return with the transfer pending */

  return ret;
}
#endif

/****************************************************************************
 * Name: max3421e_connect_event
 *
 * Description:
 *   Handle a connection event.
 *
 ****************************************************************************/

static void max3421e_connect_event(FAR struct max3421e_usbhost_s *priv)
{
  /* Were we previously disconnected? */

  if (!priv->connected)
    {
      /* Yes.. then now we are connected */

      usbhost_vtrace1(MAX3421E_VTRACE1_CONNECTED1, 0);
      priv->connected = true;
      priv->change    = true;
      DEBUGASSERT(priv->smstate == SMSTATE_DETACHED);

      /* Notify any waiters */

      priv->smstate = SMSTATE_ATTACHED;
      if (priv->pscwait)
        {
          max3421e_givesem(&priv->pscsem);
          priv->pscwait = false;
        }
    }
}

/****************************************************************************
 * Name: max3421e_disconnect_event
 *
 * Description:
 *   Handle a disconnection event.
 *
 ****************************************************************************/

static void max3421e_disconnect_event(FAR struct max3421e_usbhost_s *priv)
{
  /* Were we previously connected? */

  if (priv->connected)
    {
      /* Yes.. then we no longer connected */

      usbhost_vtrace1(MAX3421E_VTRACE1_DISCONNECTED1, 0);

      /* Are we bound to a class driver? */

      if (priv->rhport.hport.devclass)
        {
          /* Yes.. Disconnect the class driver */

          CLASS_DISCONNECTED(priv->rhport.hport.devclass);
          priv->rhport.hport.devclass = NULL;
        }

      /* Re-Initialize Host for new Enumeration */

      priv->smstate   = SMSTATE_DETACHED;
      priv->connected = false;
      priv->change    = true;

      priv->rhport.hport.speed = USB_SPEED_FULL;
      priv->rhport.hport.funcaddr = 0;

      /* Notify any waiters that there is a change in the connection state */

      if (priv->pscwait)
        {
          max3421e_givesem(&priv->pscsem);
          priv->pscwait = false;
        }
    }
}

/****************************************************************************
 * Name: max3421e_connected
 *
 * Description:
 *   USB host port interrupt handler
 *
 ****************************************************************************/

static int max3421e_connected(FAR struct max3421e_usbhost_s *priv)
{
  int ret;

  /* Stop SOF generation and reset the bus */

  max3421e_busreset(priv);
  nxsig_sleep(1);

  /* Check for low- or full-speed and restart SOF generation. */

  ret = max3421e_startsof(priv);
  if (ret < 0)
    {
      usbhost_trace1(MAX3421E_TRACE1_DEVDISCONN7, -ret);
      return ret;
    }

  usbhost_vtrace1(MAX3421E_VTRACE1_CONNECTED3, 0);

  /* Were we previously disconnected? */

  max3421e_connect_event(priv);
  return OK;
}

/****************************************************************************
 * Name: max3421e_disconnected
 *
 * Description:
 *   USB disconnect detected interrupt handler
 *
 ****************************************************************************/

static void max3421e_disconnected(FAR struct max3421e_usbhost_s *priv)
{
  usbhost_vtrace1(MAX3421E_VTRACE1_DISCONNECTED2, 0);

  /* Disable the SOF generator */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_MODE, USBHOST_MODE_SOFKAENAB, 0);

  /* Handle the disconnection event */

  max3421e_disconnect_event(priv);
}

/****************************************************************************
 * Name: max3421e_irqwork
 *
 * Description:
 *   MAX3421E interrupt worker.  Perform MAX3421E interrupt processing on the
 *   high priority work queue thread.  Interrupts were disabled by the
 *   interrupt handler when the interrupt was received.  This worker must
 *   re-enable MAX3421E interrupts when interrupt processing is complete.
 *
 ****************************************************************************/

static void max3421e_irqwork(FAR void *arg)
{
  FAR struct max3421e_usbhost_s *priv;
  FAR const struct max3421e_lowerhalf_s *lower;
  uint8_t pending;
  int ret;

  priv  = (FAR struct max3421e_usbhost_s *)arg;
  DEBUGASSERT(priv != NULL && priv->lower != NULL);
  lower = (FAR const struct max3421e_lowerhalf_s *)priv->lower;

  /* Get exclusive access to the SPI bus */

  max3421e_lock(priv);

  /* Loop while there are pending interrupts to process.  This loop may save
   * a little interrupt handling overhead.
   */

  for (; ; )
    {
      /* Get the unmasked bits in the GINT status */

      pending = max3421e_int_status(priv);
      priv->lower->acknowledge(lower);

      /* Break out of the loop when there are no pending interrupts. */

      if (pending == 0)
        {
          break;
        }

      /* Possibilities:
       *
       *   HXFRDNIRQ    - Host Transfer Done Interrupt
       *   FRAMEIRQ     - Frame Generator Interrupt
       *   CONNIRQ      - Peripheral Connect/Disconnect Interrupt
       *   SUSDNIRQ     - Suspend operation Done
       *   SNDBAVIRQ    - SNDFIFO is available
       *   RCVDAVIRQ    - RCVFIFO data available
       *   RSMREQIRQ    - Remote Wakeup Interrupt
       *   BUSEVENTIRQ  - Bus Reset or Bus Resume Interrupt
       *
       * Only CONNIRQ handled here.
       */

      /* HXFRDNIRQ:  Host transfer done interrupt */

      if ((pending & USBHOST_HIRQ_HXFRDNIRQ) != 0 &&
          (pending & USBHOST_HIRQ_SNDBAVIRQ) == 0 &&
          (pending & USBHOST_HIRQ_RCVDAVIRQ) == 0)
        {
          int result;

          /* Disable further SNDBAV (or HXFRDN) interrupts */

          max3421e_int_disable(priv, USBHOST_HIRQ_HXFRDNIRQ);

          /* Clear the pending HXFRDN interrupt */

          max3421e_int_clear(priv, USBHOST_HIRQ_HXFRDNIRQ);

          /* Check transfer status and terminate the transfer if any error
           * occurred.
           */

          result = max3421e_transfer_status(priv);
          if (result < 0 || priv->xfrtype == HXFRDN_SETUP ||
              priv->xfrtype == HXFRDN_SNDZLP)
            {
              FAR struct max3421e_chan_s *chan = priv->waiter;
              DEBUGASSERT(chan != NULL);

              max3421e_transfer_terminate(priv, chan, result);
            }
        }

      /* CONNIRQ: Has a peripheral been connected or disconnected */

      if ((pending & USBHOST_HIRQ_CONNIRQ) != 0)
        {
          /* Clear the pending CONNIRQ interrupt */

          max3421e_int_clear(priv, USBHOST_HIRQ_CONNIRQ);

          /* Check if a peripheral device has been connected */

          ret = max3421e_connected(priv);
          if (ret < 0)
            {
              /* No.. then a device must have been disconnected. */

              max3421e_disconnected(priv);
            }
        }

      /* SNDBAV: The SNDFIFO is available */

      else if ((pending & USBHOST_HIRQ_SNDBAVIRQ) != 0)
        {
          /* Disable further SNDBAV (or HXFRDN) interrupts */

          max3421e_int_disable(priv, USBHOST_HIRQ_SNDBAVIRQ |
                                     USBHOST_HIRQ_HXFRDNIRQ);

          /* Clear the pending SNDBAV interrupt */

          max3421e_int_clear(priv, USBHOST_HIRQ_SNDBAVIRQ);

          /* Finish long transfer, possibly re-enabling the SNDBAV
           * interrupt (see max3421e_send_start)
           */

          max3421e_send_continue(priv);
        }

      /* RCVDAVIRQ: RCVFIFO data available */

      else if ((pending & USBHOST_HIRQ_RCVDAVIRQ) != 0)
        {
          /* Disable further RCVDAV (or HXFRDN) interrupts */

          max3421e_int_disable(priv, USBHOST_HIRQ_RCVDAVIRQ |
                                     USBHOST_HIRQ_HXFRDNIRQ);

          /* Clear the pending RCVDAV interrupt.  The RCVFIFO is double
           * buffered.  If another packet is available in the other buffer,
           * the MAX3421E will immediately re-assert RCVDAVIRQ and we
           * will catch that the next time through this loop.
           */

          max3421e_int_clear(priv, USBHOST_HIRQ_RCVDAVIRQ);

          /* Handle the receipt of data */

          max3421e_recv_continue(priv);
        }
    }

  /* Re-enable interrupts */

  lower->enable(lower, true);
  max3421e_unlock(priv);
}

/****************************************************************************
 * Name: max3421e_interrupt
 *
 * Description:
 *   MAX3421E interrupt handler.  This interrupt handler simply defers
 *   interrupt processing to the high priority work queue thread.  This is
 *   necessary because we cannot perform interrupt/DMA driven SPI accesses
 *   from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct max3421e_usbhost_s *priv;
  FAR const struct max3421e_lowerhalf_s *lower;

  priv = (FAR struct max3421e_usbhost_s *)arg;
  DEBUGASSERT(priv != NULL && priv->lower != NULL);
  lower = (FAR const struct max3421e_lowerhalf_s *)priv->lower;

  /* Disable further interrupts until work associated with this interrupt
   * has been processed.
   */

  lower->enable(lower, false);

  /* And defer interrupt processing to the high priority work queue thread */

  work_queue(LPWORK, &priv->irqwork, max3421e_irqwork, priv, 0);
  return OK;
}

/****************************************************************************
 * Name: max3421e_int_enable, max3421e_int_disable, max3421e_int_status, and
 *       max3421e_int_wait
 *
 * Description:
 *   Respectively enable, disable, or get status of the USB host interrupt
 *   (HIRQ) and a mask of enabled interrupts.
 *
 * Input Parameters:
 *   priv - Private state data
 *   irqset - IRQ bits to be set (max3421e_int_status only)
 *
 * Returned Value:
 *   The current unmasks interrupt status  (max3421e_int_status only)
 *
 ****************************************************************************/

/* Enable a set of interrupts */

static inline void max3421e_int_enable(FAR struct max3421e_usbhost_s *priv,
                                       uint8_t irqset)
{
  priv->irqset |= irqset;
  max3421e_putreg(priv, MAX3421E_USBHOST_HIEN, priv->irqset);
}

/* Disable a set of interrupts */

static inline void max3421e_int_disable(FAR struct max3421e_usbhost_s *priv,
                                        uint8_t irqset)
{
  priv->irqset &= ~irqset;
  max3421e_putreg(priv, MAX3421E_USBHOST_HIEN, priv->irqset);
}

/* Get the set of pending interrupts */

static inline uint8_t
  max3421e_int_status(FAR struct max3421e_usbhost_s *priv)
{
  return max3421e_getreg(priv, MAX3421E_USBHOST_HIRQ) & priv->irqset;
}

/* Clear a set of pending interrupts */

static inline void max3421e_int_clear(FAR struct max3421e_usbhost_s *priv,
                                      uint8_t irqset)
{
  max3421e_putreg(priv, MAX3421E_USBHOST_HIRQ, irqset);
}

/* Wait until any interrupt from a set of interrupts occurs */

static void max3421e_int_wait(FAR struct max3421e_usbhost_s *priv,
                              uint8_t irqset, unsigned int usec)
{
  uint8_t regval;

  do
    {
      regval  =  max3421e_getreg(priv, MAX3421E_USBHOST_HIRQ);
      regval &= irqset;

      if (regval == 0 && usec > 0)
        {
          nxsig_usleep(usec);
        }
    }
  while (regval == 0);
}

/****************************************************************************
 * Name: max3421e_wait
 *
 * Description:
 *   Wait for a device to be connected or disconnected to/from a hub port.
 *
 * Input Parameters:
 *   conn  - The USB host connection instance obtained as a parameter from
 *           the call to the USB driver initialization logic.
 *   hport - The location to return the hub port descriptor that detected the
 *           connection related event.
 *
 * Returned Value:
 *   Zero (OK) is returned on success when a device is connected or
 *   disconnected. This function will not return until either (1) a device is
 *   connected or disconnect to/from any hub port or until (2) some failure
 *   occurs.  On a failure, a negated errno value is returned indicating the
 *   nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_wait(FAR struct usbhost_connection_s *conn,
                         FAR struct usbhost_hubport_s **hport)
{
  FAR struct max3421e_connection_s *maxconn;
  FAR struct max3421e_usbhost_s *priv;
  struct usbhost_hubport_s *connport;
  int ret;

  maxconn = (FAR struct max3421e_connection_s *)conn;
  DEBUGASSERT(maxconn != NULL && maxconn->priv != NULL);
  priv = maxconn->priv;

  /* Loop until a change in connection state is detected */

  for (; ; )
    {
      /* We must have exclusive access to USB host hardware and structures */

      ret = max3421e_take_exclsem(priv);
      if (ret < 0)
        {
          return ret;
        }

      /* Is there a change in the connection state of the single root hub
       * port?
       */

      if (priv->change)
        {
          connport = &priv->rhport.hport;

          /* Yes. Remember the new state */

          connport->connected = priv->connected;
          priv->change = false;

          /* And return the root hub port */

          *hport = connport;

          usbhost_vtrace1(MAX3421E_VTRACE1_CONNECTED2, connport->connected);

          max3421e_give_exclsem(priv);
          return OK;
        }

#ifdef CONFIG_USBHOST_HUB
      /* Is a device connected to an external hub? */

      if (priv->hport)
        {
          /* Yes.. return the external hub port */

          connport = (struct usbhost_hubport_s *)priv->hport;
          priv->hport = NULL;

          *hport = connport;

          usbhost_vtrace1(MAX3421E_VTRACE1_HUB_CONNECTED,
                          connport->connected);

          max3421e_give_exclsem(priv);
          return OK;
        }
#endif

      /* Wait for the next connection event */

      priv->pscwait = true;
      max3421e_give_exclsem(priv);
      ret = max3421e_takesem(&priv->pscsem);
      if (ret < 0)
        {
          return ret;
        }
    }
}

/****************************************************************************
 * Name: max3421e_getspeed
 *
 * Description:
 *   Get the speed of the connected device.
 *
 * Input Parameters:
 *   priv - Driver private state structure
 *   conn - The USB host connection instance obtained as a parameter from
 *      the call to the USB driver initialization logic.
 *   hport - The descriptor of the hub port that has the newly connected
 *      device.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   The caller has the SPI bus locked.
 *
 ****************************************************************************/

static int max3421e_getspeed(FAR struct max3421e_usbhost_s *priv,
                             FAR struct usbhost_connection_s *conn,
                             FAR struct usbhost_hubport_s *hport)
{
  int ret;

  DEBUGASSERT(conn != NULL && hport != NULL && hport->port == 0);

  /* Are we connected to a device?  The caller should have called the wait()
   * method first to be assured that a device is connected.
   */

  if (!priv->connected)
    {
      /* No, return an error */

      usbhost_trace1(MAX3421E_TRACE1_DEVDISCONN6, 0);
      return -ENODEV;
    }

  DEBUGASSERT(priv->smstate == SMSTATE_ATTACHED);

  /* USB 2.0 spec says at least 50ms delay before port reset.  We wait
   * 100ms.
   */

  nxsig_usleep(100 * 1000);

  /* Make sure we are still connected */

  if (!priv->connected)
    {
      /* No, return an error */

      usbhost_trace1(MAX3421E_TRACE1_DEVDISCONN6, 0);
      max3421e_give_exclsem(priv);
      return -ENODEV;
    }

  /* Stop SOF generation and reset the host port */

  max3421e_busreset(priv);
  nxsig_sleep(1);

  /* Get the current device speed */

  ret = max3421e_startsof(priv);
  if (ret < 0)
    {
      usbhost_trace1(MAX3421E_TRACE1_DEVDISCONN8, -ret);
    }

  return ret;
}

/****************************************************************************
 * Name: max3421e_enumerate
 *
 * Description:
 *   Enumerate the connected device.  As part of this enumeration process,
 *   the driver will (1) get the device's configuration descriptor, (2)
 *   extract the class ID info from the configuration descriptor, (3) call
 *   usbhost_findclass() to find the class that supports this device, (4)
 *   call the create() method on the struct usbhost_registry_s interface
 *   to get a class instance, and finally (5) call the connect() method
 *   of the struct usbhost_class_s interface.  After that, the class is in
 *   charge of the sequence of operations.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from
 *      the call to the USB driver initialization logic.
 *   hport - The descriptor of the hub port that has the newly connected
 *      device.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_enumerate(FAR struct usbhost_connection_s *conn,
                              FAR struct usbhost_hubport_s *hport)
{
  FAR struct max3421e_connection_s *maxconn;
  FAR struct max3421e_usbhost_s *priv;
  int ret;

  maxconn = (FAR struct max3421e_connection_s *)conn;
  DEBUGASSERT(maxconn != NULL && maxconn->priv != NULL);
  priv = maxconn->priv;

  /* We must have exclusive access to the USB host hardware and structures */

  ret = max3421e_take_exclsem(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* If this is a connection on the root hub, then we need to go to
   * little more effort to get the device speed.  If it is a connection
   * on an external hub, then we already have that information.
   */

  max3421e_lock(priv);

#warning REVISIT:  Isn't this already done?

#ifdef CONFIG_USBHOST_HUB
  if (ROOTHUB(hport))
#endif
    {
      ret = max3421e_getspeed(priv, conn, hport);
      if (ret < 0)
        {
          max3421e_give_exclsem(priv);
          return ret;
        }
    }

  /* Set enumeration data toggles */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_HCTL,
                     USBHOST_HCTL_TOGGLES_MASK,
                     USBHOST_HCTL_RCVTOG1 | USBHOST_HCTL_SNDTOG1);
  max3421e_unlock(priv);

  /* Then let the common usbhost_enumerate do the real enumeration. */

  usbhost_vtrace1(MAX3421E_VTRACE1_ENUMERATE, 0);
  priv->smstate = SMSTATE_ENUM;

  ret = usbhost_enumerate(hport, &hport->devclass);

  /* The enumeration may fail either because of some HCD interfaces failure
   * or because the device class is not supported.  In either case, we just
   * need to perform the disconnection operation and make ready for a new
   * enumeration.
   */

  if (ret < 0)
    {
      /* Return to the disconnected state */

      usbhost_trace1(MAX3421E_TRACE1_ENUMERATE_FAIL, -ret);
      max3421e_disconnect_event(priv);
    }

  /* Set post-enumeration data toggles (assuming success) */

  max3421e_lock(priv);
  max3421e_modifyreg(priv, MAX3421E_USBHOST_HCTL,
                     USBHOST_HCTL_TOGGLES_MASK,
                     USBHOST_HCTL_RCVTOG0 | USBHOST_HCTL_SNDTOG0);
  max3421e_unlock(priv);

  max3421e_give_exclsem(priv);
  return ret;
}

/****************************************************************************
 * Name: max3421e_ep0configure
 *
 * Description:
 *   Configure endpoint 0.  This method is normally used internally by the
 *   enumerate() method but is made available at the interface to support an
 *   external implementation of the enumeration logic.
 *
 * Input Parameters:
 *   drvr          - The USB host driver instance obtained as a parameter
 *                   from the call to the class create() method.
 *   ep0           - The (opaque) EP0 endpoint instance
 *   funcaddr      - The USB address of the function containing the endpoint
 *                   that EP0 controls
 *   speed         - The speed of the port USB_SPEED_LOW or _FULL
 *   maxpacketsize - The maximum number of bytes that can be sent to or
 *                   received from the endpoint in a single data packet
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_ep0configure(FAR struct usbhost_driver_s *drvr,
                                 usbhost_ep_t ep0,
                                 uint8_t funcaddr, uint8_t speed,
                                 uint16_t maxpacketsize)
{
  FAR struct max3421e_usbhost_s *priv =
    (FAR struct max3421e_usbhost_s *)drvr;
  FAR struct max3421e_chan_s *chan;
  int ret;

  DEBUGASSERT(drvr != NULL && funcaddr < 128 && maxpacketsize <= 64);

  /* We must have exclusive access to the USB host hardware and structures */

  ret = max3421e_take_exclsem(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure the EP0 channel */

  chan            = &priv->chan[(intptr_t)ep0];
  chan->funcaddr  = funcaddr;
  chan->speed     = speed;
  chan->maxpacket = maxpacketsize;
  chan->toggles   = USBHOST_HCTL_RCVTOG0 | USBHOST_HCTL_SNDTOG0;

  max3421e_give_exclsem(priv);
  return OK;
}

/****************************************************************************
 * Name: max3421e_epalloc
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep     - A memory location provided by the caller in which to receive
 *            the allocated endpoint descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_epalloc(FAR struct usbhost_driver_s *drvr,
                            FAR const struct usbhost_epdesc_s *epdesc,
                            FAR usbhost_ep_t *ep)
{
  FAR struct max3421e_usbhost_s *priv =
    (FAR struct max3421e_usbhost_s *)drvr;
  struct usbhost_hubport_s *hport;
  FAR struct max3421e_chan_s *chan;
  int chidx;
  int ret;

  /* Sanity check.  NOTE that this method should only be called if a device
   * is connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(drvr != 0 && epdesc != NULL && ep != NULL);
  hport = epdesc->hport;
  DEBUGASSERT(hport != NULL);

  /* We must have exclusive access to the USB host hardware and structures */

  ret = max3421e_take_exclsem(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate a host channel for the endpoint */

  chidx = max3421e_chan_alloc(priv);
  if (chidx < 0)
    {
      usbhost_trace1(MAX3421E_TRACE1_CHANALLOC_FAIL, -chidx);
      max3421e_give_exclsem(priv);
      return chidx;
    }

  /* Decode the endpoint descriptor to initialize the channel data
   * structures.  Note:  Here we depend on the fact that the endpoint point
   * type is encoded in the same way in the endpoint descriptor as it is in
   * the OTG HS hardware.
   */

  chan            = &priv->chan[chidx];
  chan->epno      = epdesc->addr & USB_EPNO_MASK;
  chan->in        = epdesc->in;
  chan->eptype    = epdesc->xfrtype;
  chan->funcaddr  = hport->funcaddr;
  chan->speed     = hport->speed;
  chan->interval  = epdesc->interval;
  chan->maxpacket = epdesc->mxpacketsize;
  chan->toggles   = USBHOST_HCTL_RCVTOG0 | USBHOST_HCTL_SNDTOG0;

  /* Return the endpoint number as the endpoint "handle" */

  *ep = (usbhost_ep_t)chidx;
  max3421e_give_exclsem(priv);
  return OK;
}

/****************************************************************************
 * Name: max3421e_epfree
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *          call to the class create() method.
 *   ep   - The endpoint to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_epfree(FAR struct usbhost_driver_s *drvr,
                           usbhost_ep_t ep)
{
  FAR struct max3421e_usbhost_s *priv =
    (FAR struct max3421e_usbhost_s *)drvr;
  int ret;

  DEBUGASSERT(priv);

  /* We must have exclusive access to the USB host hardware and structures */

  ret = max3421e_take_exclsem(priv);
  if (ret >= 0)
    {
      /* Halt the channel and mark the channel available */

      max3421e_chan_free(priv, (intptr_t)ep);
      max3421e_give_exclsem(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: max3421e_alloc
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.  This method provides a
 *   mechanism to allocate the request/descriptor memory.  If the underlying
 *   hardware does not support such "special" memory, this functions may
 *   simply map to kmm_malloc.
 *
 *   This interface was optimized under a particular assumption.  It was
 *   assumed that the driver maintains a pool of small, pre-allocated
 *   buffers for descriptor traffic.  NOTE that size is not an input, but
 *   an output:  The size of the pre-allocated buffer is returned.
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   buffer - The address of a memory location provided by the caller in
 *            which to return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in
 *            which to return the maximum size of the allocated buffer
 *             memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_alloc(FAR struct usbhost_driver_s *drvr,
                          FAR uint8_t **buffer, FAR size_t *maxlen)
{
  FAR uint8_t *alloc;

  DEBUGASSERT(drvr && buffer && maxlen);

  /* There is no special memory requirement for the MAX3421E. */

  alloc = (FAR uint8_t *)kmm_malloc(CONFIG_MAX3421E_DESCSIZE);
  if (!alloc)
    {
      return -ENOMEM;
    }

  /* Return the allocated address and size of the descriptor buffer */

  *buffer = alloc;
  *maxlen = CONFIG_MAX3421E_DESCSIZE;
  return OK;
}

/****************************************************************************
 * Name: max3421e_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.  This method provides a
 *   mechanism to free that request/descriptor memory.  If the underlying
 *   hardware does not support such "special" memory, this functions may
 *   simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_free(FAR struct usbhost_driver_s *drvr,
                         FAR uint8_t *buffer)
{
  /* There is no special memory requirement */

  DEBUGASSERT(drvr && buffer);
  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: max3421e_ioalloc
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to
 *   allocate the request/descriptor memory.  If the underlying hardware
 *   does not support such "special" memory, this functions may simply map
 *   to kmm_malloc.
 *
 *   This interface differs from DRVR_ALLOC in that the buffers are
 *   variable-sized.
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   buffer - The address of a memory location provided by the caller in
 *            which to return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_ioalloc(FAR struct usbhost_driver_s *drvr,
                            FAR uint8_t **buffer, size_t buflen)
{
  FAR uint8_t *alloc;

  DEBUGASSERT(drvr && buffer && buflen > 0);

  /* There is no special memory requirement */

  alloc = (FAR uint8_t *)kmm_malloc(buflen);
  if (!alloc)
    {
      return -ENOMEM;
    }

  /* Return the allocated buffer */

  *buffer = alloc;
  return OK;
}

/****************************************************************************
 * Name: max3421e_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be accessed
 *   more efficiently.  This method provides a mechanism to free that IO
 *   buffer memory.  If the underlying hardware does not support such
 *   "special" memory, this functions may simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_iofree(FAR struct usbhost_driver_s *drvr,
                           FAR uint8_t *buffer)
{
  /* There is no special memory requirement */

  DEBUGASSERT(drvr && buffer);
  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: max3421e_ctrlin and max3421e_ctrlout
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  These methods
 *   will enqueue the request and wait for it to complete.  Only one
 *   transfer may be queued; Neither these methods nor the transfer() method
 *   can be called again until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from
 *            the call to the class create() method.
 *   ep0    - The control endpoint to send/receive the control request.
 *   req    - Describes the request to be sent.  This request must lie in
 *            memory created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *            responses.  This buffer must be large enough to hold the
 *            length value in the request description. buffer must have been
 *            allocated using DRVR_ALLOC.
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same
 *   allocated memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_ctrlin(FAR struct usbhost_driver_s *drvr,
                           usbhost_ep_t ep0,
                           FAR const struct usb_ctrlreq_s *req,
                           FAR uint8_t *buffer)
{
  FAR struct max3421e_usbhost_s *priv;
  FAR struct max3421e_chan_s *chan;
  uint16_t buflen;
  clock_t start;
  clock_t elapsed;
  int retries;
  int ret;

  DEBUGASSERT(drvr != NULL && req != NULL);
  priv = (FAR struct max3421e_usbhost_s *)drvr;

  DEBUGASSERT((intptr_t)ep0 >= 0 && (intptr_t)ep0 < MAX3421E_NHOST_CHANNELS);
  chan = &priv->chan[(intptr_t)ep0];

  usbhost_vtrace2(MAX3421E_VTRACE2_CTRLIN, req->type, req->req);
  uinfo("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* Extract values from the request */

  buflen = max3421e_getle16(req->len);

  /* We must have exclusive access to the USB host hardware and structures */

  ret = max3421e_take_exclsem(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Loop, retrying until the retry time expires */

  for (retries = 0; retries < MAX3421E_RETRY_COUNT; retries++)
    {
      /* Send the SETUP request */

      ret = max3421e_ctrl_sendsetup(priv, chan, req);
      if (ret < 0)
        {
          usbhost_trace1(MAX3421E_TRACE1_SENDSETUP_FAIL2, -ret);
          continue;
        }

      /* Get the start time.  Loop again until the timeout expires */

      start = clock_systime_ticks();
      do
        {
          /* Handle the IN data phase (if any) */

          /* The MAX4321E sends fixed DATA0 and DATA1 PID tokens for the
           * various stages of a CONTROL transfer, regardless of the
           * setting of the internal data toggle.
           */

          if (buflen > 0)
            {
              ret = max3421e_ctrl_recvdata(priv, chan, buffer, buflen);
              if (ret < 0)
                {
                  usbhost_trace1(MAX3421E_TRACE1_RECVDATA_FAIL, -ret);
                }
            }

          /* Handle the status OUT phase */

          if (ret >= OK)
            {
              ret = max3421e_ctrl_sendstatus(priv, chan);
              if (ret >= OK)
                {
                  /* All success transactions exit here */

                  max3421e_give_exclsem(priv);
                  return OK;
                }

              usbhost_trace1(MAX3421E_TRACE1_SENDSTATUS_FAIL, -ret);
            }

          /* Get the elapsed time (in frames) */

          elapsed = clock_systime_ticks() - start;
        }
      while (elapsed < MAX3421E_DATANAK_DELAY);
    }

  /* All failures exit here after all retries and timeouts are exhausted */

  max3421e_give_exclsem(priv);
  return -ETIMEDOUT;
}

static int max3421e_ctrlout(FAR struct usbhost_driver_s *drvr,
                            usbhost_ep_t ep0,
                            FAR const struct usb_ctrlreq_s *req,
                            FAR const uint8_t *buffer)
{
  FAR struct max3421e_usbhost_s *priv =
    (FAR struct max3421e_usbhost_s *)drvr;
  FAR struct max3421e_chan_s *chan;
  uint16_t buflen;
  clock_t start;
  clock_t elapsed;
  int retries;
  int ret;

  DEBUGASSERT(drvr != NULL && req != NULL);
  priv = (FAR struct max3421e_usbhost_s *)drvr;

  DEBUGASSERT((intptr_t)ep0 >= 0 && (intptr_t)ep0 < MAX3421E_NHOST_CHANNELS);
  chan = &priv->chan[(intptr_t)ep0];

  usbhost_vtrace2(MAX3421E_VTRACE2_CTRLOUT, req->type, req->req);
  uinfo("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* Extract values from the request */

  buflen = max3421e_getle16(req->len);

  /* We must have exclusive access to the USB host hardware and structures */

  ret = max3421e_take_exclsem(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Loop, retrying until the retry time expires */

  for (retries = 0; retries < MAX3421E_RETRY_COUNT; retries++)
    {
      /* Send the SETUP request */

      ret = max3421e_ctrl_sendsetup(priv, chan, req);
      if (ret < 0)
        {
          usbhost_trace1(MAX3421E_TRACE1_SENDSETUP_FAIL1, -ret);
          continue;
        }

      /* Get the start time.  Loop again until the timeout expires */

      start = clock_systime_ticks();
      do
        {
          /* Handle the data OUT phase (if any) */

          /* The MAX4321E sends fixed DATA0 and DATA1 PID tokens for the
           * various stages of a CONTROL transfer, regardless of the setting
           * of the internal data toggle.
           */

          if (buflen > 0)
            {
              /* Start DATA out transfer (only one DATA packet) */

              ret = max3421e_ctrl_senddata(priv, chan, NULL, 0);
              if (ret < 0)
                {
                  usbhost_trace1(MAX3421E_TRACE1_SENDDATA_FAIL, -ret);
                }
            }

          /* Handle the status IN phase */

          if (ret >= OK)
            {
              ret = max3421e_ctrl_recvstatus(priv, chan);
              if (ret >= OK)
                {
                  /* All success transactions exit here */

                  max3421e_give_exclsem(priv);
                  return OK;
                }

              usbhost_trace1(MAX3421E_TRACE1_RECVSTATUS_FAIL, -ret);
            }

          /* Get the elapsed time (in frames) */

          elapsed = clock_systime_ticks() - start;
        }
      while (elapsed < MAX3421E_DATANAK_DELAY);
    }

  /* All failures exit here after all retries and timeouts are exhausted */

  max3421e_give_exclsem(priv);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: max3421e_transfer
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request, blocking until the transfer completes.
 *   Only one transfer may be  queued; Neither this method nor the ctrlin or
 *   ctrlout methods can be called again until this function returns.
 *
 *   This is a blocking method; this functions will not return until the
 *   transfer has completed.
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   ep     - The IN or OUT endpoint descriptor for the device endpoint on
 *            which to perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or
 *            received (IN endpoint).  buffer must have been allocated
 *            using DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Value:
 *   On success, a non-negative value is returned that indicates the number
 *   of bytes successfully transferred.  On a failure, a negated errno value
 *   is returned that indicates the nature of the failure:
 *
 *     EAGAIN - If devices NAKs the transfer (or NYET or other error where
 *              it may be appropriate to restart the entire transaction).
 *     EPERM  - If the endpoint stalls
 *     EIO    - On a TX or data toggle error
 *     EPIPE  - Overrun errors
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static ssize_t max3421e_transfer(FAR struct usbhost_driver_s *drvr,
                                 usbhost_ep_t ep, FAR uint8_t *buffer,
                                 size_t buflen)
{
  FAR struct max3421e_usbhost_s *priv =
    (FAR struct max3421e_usbhost_s *)drvr;
  FAR struct max3421e_chan_s *chan;
  ssize_t nbytes;
  int ret;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT((intptr_t)ep >= 0 && (intptr_t)ep < MAX3421E_NHOST_CHANNELS);
  chan = &priv->chan[(intptr_t)ep];

  usbhost_vtrace2(MAX3421E_VTRACE2_TRANSFER, (unsigned int)ep, buflen);

  /* We must have exclusive access to the USB host hardware and structures */

  ret = max3421e_take_exclsem(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle IN and OUT transfer differently */

  if (chan->in)
    {
      nbytes = max3421e_in_transfer(priv, chan, buffer, buflen);
    }
  else
    {
      nbytes = max3421e_out_transfer(priv, chan, buffer, buflen);
    }

  max3421e_give_exclsem(priv);
  return nbytes;
}

/****************************************************************************
 * Name: max3421e_asynch
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately.  When the transfer
 *   completes, the callback will be invoked with the provided transfer.
 *   This method is useful for receiving interrupt transfers which may come
 *   infrequently.
 *
 *   Only one transfer may be queued; Neither this method nor the ctrlin or
 *   ctrlout methods can be called again until the transfer completes.
 *
 * Input Parameters:
 *   drvr     - The USB host driver instance obtained as a parameter from
 *              the call to the class create() method.
 *   ep       - The IN or OUT endpoint descriptor for the device endpoint
 *              on which to perform the transfer.
 *   buffer   - A buffer containing the data to be sent (OUT endpoint) or
 *              received (IN endpoint).  buffer must have been allocated
 *              using DRVR_ALLOC
 *   buflen   - The length of the data to be sent or received.
 *   callback - This function will be called when the transfer completes.
 *   arg      - The arbitrary parameter that will be passed to the callback
 *              function when the transfer completes.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int max3421e_asynch(FAR struct usbhost_driver_s *drvr,
                           usbhost_ep_t ep,
                           FAR uint8_t *buffer, size_t buflen,
                           usbhost_asynch_t callback, FAR void *arg)
{
  FAR struct max3421e_usbhost_s *priv =
    (FAR struct max3421e_usbhost_s *)drvr;
  FAR struct max3421e_chan_s *chan;
  int ret;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT((intptr_t)ep >= 0 && ep < MAX3421E_NHOST_CHANNELS);
  chan = &priv->chan[(intptr_t)ep];

  usbhost_vtrace2(MAX3421E_VTRACE2_ASYNCH, (unsigned int)ep, buflen);

  /* We must have exclusive access to the USB host hardware and structures */

  ret = max3421e_take_exclsem(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle IN and OUT transfer slightly differently */

  if (chan->in)
    {
      ret = max3421e_in_asynch(priv, chan, buffer, buflen, callback, arg);
    }
  else
    {
      ret = max3421e_out_asynch(priv, chan, buffer, buflen, callback, arg);
    }

  max3421e_give_exclsem(priv);
  return ret;
}
#endif /* CONFIG_USBHOST_ASYNCH */

/****************************************************************************
 * Name: max3421e_cancel
 *
 * Description:
 *   Cancel a pending transfer on an endpoint.  Canceled synchronous or
 *   asynchronous transfer will complete normally with the error -ESHUTDOWN.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *          call to the class create() method.
 *   ep   - The IN or OUT endpoint descriptor for the device endpoint on
 *          which an asynchronous transfer should be transferred.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 ****************************************************************************/

static int max3421e_cancel(FAR struct usbhost_driver_s *drvr,
                           usbhost_ep_t ep)
{
  FAR struct max3421e_usbhost_s *priv =
    (FAR struct max3421e_usbhost_s *)drvr;
  FAR struct max3421e_chan_s *chan;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT((intptr_t)ep >= 0 && (intptr_t)ep < MAX3421E_NHOST_CHANNELS);
  chan = &priv->chan[(intptr_t)ep];

  usbhost_vtrace1(MAX3421E_VTRACE1_CANCEL, (intptr_t)ep);

  /* We need to disable interrupts to avoid race conditions with the
   * asynchronous completion of the transfer being canceled.
   */

  flags = enter_critical_section();

  /* Halt the channel */
#warning Missing logic
  UNUSED(chan);  /* For now */

  priv->result = -ESHUTDOWN;

  /* Is there a thread waiting for this transfer to complete? */

  if (priv->waiter != NULL)
    {
#ifdef CONFIG_USBHOST_ASYNCH
      /* Yes.. there should not also be a callback scheduled */

      DEBUGASSERT(priv->callback == NULL);
#endif

      /* Wake'em up! */

      max3421e_givesem(&priv->waitsem);
      priv->waiter = NULL;
    }

#ifdef CONFIG_USBHOST_ASYNCH
  /* No.. is an asynchronous callback expected when the transfer
   * completes?
   */

  else if (priv->callback)
    {
      usbhost_asynch_t callback;
      FAR void *arg;

      /* Extract the callback information */

      callback       = priv->callback;
      arg            = priv->arg;

      priv->callback = NULL;
      priv->arg      = NULL;
      priv->xfrd     = 0;

      /* Then perform the callback */

      callback(arg, -ESHUTDOWN);
    }
#endif

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: max3421e_connect
 *
 * Description:
 *   New connections may be detected by an attached hub.  This method is the
 *   mechanism that is used by the hub class to introduce a new connection
 *   and port description to the system.
 *
 * Input Parameters:
 *   drvr      - The USB host driver instance obtained as a parameter from
 *               the call to the class create() method.
 *   hport     - The descriptor of the hub port that detected the connection
 *               related event
 *   connected - True: device connected; false: device disconnected
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_HUB
static int max3421e_connect(FAR struct usbhost_driver_s *drvr,
                            FAR struct usbhost_hubport_s *hport,
                            bool connected)
{
  FAR struct max3421e_usbhost_s *priv =
    (FAR struct max3421e_usbhost_s *)drvr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL && hport != NULL);

  /* Set the connected/disconnected flag */

  hport->connected = connected;
  usbhost_vtrace2(MAX3421E_VTRACE2_HUB_CONNECTED, hport->port, connected);

  /* Report the connection event */

  flags = enter_critical_section();
  priv->hport = hport;
  if (priv->pscwait)
    {
      priv->pscwait = false;
      max3421e_givesem(&priv->pscsem);
    }

  leave_critical_section(flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: max3421e_disconnect
 *
 * Description:
 *   Called by the class when an error occurs and driver has been
 *   disconnected.  The USB host driver should discard the handle to the
 *   class instance (it is stale) and not attempt any further interaction
 *   with the class driver instance (until a new instance is received from
 *   the create() method).  The driver should not called the class'
 *   disconnected() method.
 *
 * Input Parameters:
 *   drvr  - The USB host driver instance obtained as a parameter from the
 *           call to the class create() method.
 *   hport - The port from which the device is being disconnected.  Might be
 *           a port on a hub.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static void max3421e_disconnect(FAR struct usbhost_driver_s *drvr,
                                FAR struct usbhost_hubport_s *hport)
{
  DEBUGASSERT(hport != NULL);
  hport->devclass = NULL;
}

/****************************************************************************
 * Initialization
 ****************************************************************************/

/****************************************************************************
 * Name: max3421e_busreset
 *
 * Description:
 *   Reset the USB host port.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void max3421e_busreset(FAR struct max3421e_usbhost_s *priv)
{
  /* Disable the SOF generator */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_MODE, USBHOST_MODE_SOFKAENAB, 0);

  /* Clear any pending bus event */

  max3421e_int_clear(priv, USBHOST_HIRQ_BUSEVENTIRQ);

  /* Perform the reset */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_HCTL, 0, USBHOST_HCTL_BUSRST);
  max3421e_int_wait(priv, USBHOST_HIRQ_BUSEVENTIRQ, 250);
}

/****************************************************************************
 * Name: max3421e_startsof
 *
 * Description:
 *   Called after bus reset.  Determine bus speed and restart SOFs.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   OK if successfully connect; -ENODEV if not connected.
 *
 ****************************************************************************/

static int max3421e_startsof(FAR struct max3421e_usbhost_s *priv)
{
  uint8_t clrbits;
  uint8_t setbits;
  uint8_t regval;
  bool lowspeed;

  /* Check if we are already in low- or full-speed mode */

  regval = max3421e_getreg(priv, MAX3421E_USBHOST_MODE);
  lowspeed = ((regval & USBHOST_MODE_SPEED) != 0);

  /* Enable SAMPLEBUS */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_HCTL, 0,
                     USBHOST_HCTL_BUSSAMPLE);

  while ((max3421e_getreg(priv, MAX3421E_USBHOST_HCTL) &
          USBHOST_HCTL_BUSSAMPLE) == 0)
    {
      nxsig_usleep(5);
    }

  /* Check for low- or full-speed and start SOF (actually already started
   * by max3421e_busreset).
   */

  clrbits = 0;
  setbits = USBHOST_MODE_HOST | USBHOST_MODE_SOFKAENAB |
            USBHOST_MODE_DMPULLD | USBHOST_MODE_DPPULLDN;

  regval = max3421e_getreg(priv, MAX3421E_USBHOST_HRSL);
  switch (regval & (USBHOST_HRSL_KSTATUS | USBHOST_HRSL_JSTATUS))
    {
      default:
      case (USBHOST_HRSL_KSTATUS | USBHOST_HRSL_JSTATUS):

        /* Invalid state */

        usbhost_trace1(MAX3421E_TRACE1_BAD_JKSTATE, regval);

        /* Fall through */

      case 0:

        /* 0:  Not connected */

        return -ENODEV;

      case USBHOST_HRSL_KSTATUS:

        /* J=0, K=1: low-speed in full-speed (or vice versa) */

        if (lowspeed)
          {
            /* Full speed in low speed */

            clrbits |= USBHOST_MODE_SPEED;
          }
        else
          {
            /* Low speed in full speed */

            setbits |= USBHOST_MODE_SPEED;
          }
        break;

      case USBHOST_HRSL_JSTATUS:

        /* J=1,K=0: full-speed in full-speed (or vice versa) */

        if (lowspeed)
          {
            /* Low speed in low speed */

            setbits |= USBHOST_MODE_SPEED;
          }
        else
          {
            /* Full speed in full speed */

            clrbits |= USBHOST_MODE_SPEED;
          }

        break;
    }

  /* Restart the SOF generator */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_MODE, clrbits, setbits);

  /* Wait for the first SOF received and 20ms has passed */

  max3421e_int_wait(priv, USBHOST_HIRQ_FRAMEIRQ, 0);
  nxsig_usleep(20 * 1000);
  return OK;
}

/****************************************************************************
 * Name: max3421e_sw_initialize
 *
 * Description:
 *   One-time setup of the host driver state structure.
 *
 * Input Parameters:
 *   priv  -- USB host driver private data structure.
 *   conn  -- Custom USB host connection structure.
 *   lower -- The lower half driver instance.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline int max3421e_sw_initialize(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_connection_s *conn,
              FAR const struct max3421e_lowerhalf_s *lower)
{
  FAR struct usbhost_driver_s *drvr;
  FAR struct usbhost_hubport_s *hport;
  int ret;
  int i;

  /* Initialize the device operations */

  drvr                 = &priv->drvr;
  drvr->ep0configure   = max3421e_ep0configure;
  drvr->epalloc        = max3421e_epalloc;
  drvr->epfree         = max3421e_epfree;
  drvr->alloc          = max3421e_alloc;
  drvr->free           = max3421e_free;
  drvr->ioalloc        = max3421e_ioalloc;
  drvr->iofree         = max3421e_iofree;
  drvr->ctrlin         = max3421e_ctrlin;
  drvr->ctrlout        = max3421e_ctrlout;
  drvr->transfer       = max3421e_transfer;
#ifdef CONFIG_USBHOST_ASYNCH
  drvr->asynch         = max3421e_asynch;
#endif
  drvr->cancel         = max3421e_cancel;
#ifdef CONFIG_USBHOST_HUB
  drvr->connect        = max3421e_connect;
#endif
  drvr->disconnect     = max3421e_disconnect;

  /* Initialize the public port representation */

  hport                = &priv->rhport.hport;
  hport->drvr          = drvr;
#ifdef CONFIG_USBHOST_HUB
  hport->parent        = NULL;
#endif
  hport->ep0           = 0;
  hport->speed         = USB_SPEED_FULL;

  /* Initialize function address generation logic */

  usbhost_devaddr_initialize(&priv->rhport);

  /* Initialize semaphores */

  nxsem_init(&priv->pscsem,  0, 0);
  nxsem_init(&priv->waitsem,  0, 0);

  /* Initialize lock */

  nxrmutex_init(&priv->lock);

  /* The pscsem and waitsem semaphores are used for signaling and, hence,
   * should not have
   * priority inheritance enabled.
   */

  nxsem_set_protocol(&priv->pscsem, SEM_PRIO_NONE);
  nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);

  /* Initialize the driver state data */

  priv->lower     = lower;
  priv->smstate   = SMSTATE_DETACHED;
  priv->connected = false;
  priv->irqset    = 0;
  priv->change    = false;
  priv->holder    = NO_HOLDER;

  /* Put all of the channels back in their initial, allocated state */

  memset(priv->chan, 0,
         MAX3421E_NHOST_CHANNELS * sizeof(struct max3421e_chan_s));

  /* Initialize each channel */

  for (i = 0; i < MAX3421E_NHOST_CHANNELS; i++)
    {
      FAR struct max3421e_chan_s *chan = &priv->chan[i];

      chan->chidx = i;
    }

  /* Initialize the connection structure */

  conn->conn.wait      = max3421e_wait;
  conn->conn.enumerate = max3421e_enumerate;
  conn->priv           = priv;

  /* Attach USB host controller interrupt handler */

  ret = lower->attach(lower, max3421e_interrupt, priv);
  if (ret < 0)
    {
      usbhost_trace1(MAX3421E_TRACE1_IRQATTACH_FAIL, 0);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: max3421e_hw_initialize
 *
 * Description:
 *   One-time setup of the host controller hardware for normal operations.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static inline int max3421e_hw_initialize(FAR struct max3421e_usbhost_s *priv)
{
  uint8_t revision;
  uint8_t regval;
  int ret;

  /* Get exclusive access to the SPI bus */

  max3421e_lock(priv);

  /* Configure full duplex SPI, level or edge-active, rising- or falling
   * edge interrupt.
   *
   * NOTE:  Initially, the MAX3421E operations in half-duplex mode.  MISO is
   * tristated and there is no status response to commands.  Writes are not
   * effected:  The MISO pin continues to be high impedance and the master
   * continues to drive MOSI.
   *
   * For reads, however, after the 8-bit command, the max3421e starts driving
   * the MOSI pin.  The  master must turn off its driver to the MOSI pin to
   * avoid contention.
   */

  regval  = priv->lower->intconfig;
  regval &= (USBHOST_PINCTL_INTLEVEL | USBHOST_PINCTL_POSINT);
  regval |= USBHOST_PINCTL_FDUPSPI;
  max3421e_putreg(priv, MAX3421E_USBHOST_PINCTL, regval);

  /* Reset the MAX3421E by toggling the CHIPRES bit in the USBCTRL register.
   *
   * NOTE: The bits that control the SPI interface are not changed by
   * CHIPRES: FDUPSPI, INTLEVEL, and POSINT.
   */

  max3421e_putreg(priv, MAX3421E_USBHOST_USBCTL, USBHOST_USBCTL_CHIPRES);
  max3421e_putreg(priv, MAX3421E_USBHOST_USBCTL, 0);

  /* Wait for the oscillator to become stable */

  while ((max3421e_getreg(priv, MAX3421E_USBHOST_USBIRQ) &
                          USBHOST_USBIRQ_OSCOKIRQ) == 0)
    {
    }

  /* Disable interrupts, clear pending interrupts, and reset the interrupt
   * state.
   */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_CPUCTL, USBHOST_CPUCTL_IE, 0);
  max3421e_putreg(priv, MAX3421E_USBHOST_HIEN, 0);
  max3421e_int_clear(priv, 0xff);
  priv->irqset  = 0;

  /* Configure as full-speed USB host */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_MODE,
                     USBHOST_MODE_SPEED | USBHOST_MODE_SOFKAENAB,
                     USBHOST_MODE_HOST | USBHOST_MODE_DMPULLD |
                     USBHOST_MODE_DPPULLDN);

  /* Clear and enable the connection detected (CONDIRQ) interrupt */

  max3421e_int_clear(priv, USBHOST_HIRQ_CONNIRQ);
  max3421e_int_enable(priv, USBHOST_HIRQ_CONNIRQ);

  /* Enable MAX3412E interrupts */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_CPUCTL, 0, USBHOST_CPUCTL_IE);

  usbhost_vtrace1(MAX3421E_VTRACE1_INITIALIZED, 0);

  revision = max3421e_getreg(priv, MAX3421E_USBHOST_REVISION);
  if (revision != USBHOST_REVISION)
    {
      usbhost_trace1(MAX3421E_TRACE1_BADREVISION, revision);
      max3421e_unlock(priv);
      return -ENODEV;
    }

  /* Perform a bus reset to reconnect after a power down */

  ret = max3421e_connected(priv);
  if (ret < 0)
    {
      /* Nothing connected. */

      max3421e_disconnected(priv);
    }

  max3421e_unlock(priv);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max3421e_usbhost_initialize
 *
 * Description:
 *   Initialize MAX3421E as USB host controller.
 *
 * Input Parameters:
 *   lower - The interface to the lower half driver
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the device
 *   to a class driver.
 *
 * Assumptions:
 * - This function should called in the initialization sequence in order
 *   to initialize the USB device functionality.
 * - Class drivers should be initialized prior to calling this function.
 *   Otherwise, there is a race condition if the device is already connected.
 *
 ****************************************************************************/

FAR struct usbhost_connection_s *
max3421e_usbhost_initialize(FAR const struct max3421e_lowerhalf_s *lower)
{
  FAR struct usbhost_alloc_s *alloc;
  FAR struct max3421e_usbhost_s *priv;
  FAR struct max3421e_connection_s *conn;
  int ret;

  DEBUGASSERT(lower != NULL && lower->spi != NULL && lower->attach != NULL &&
              lower->attach != NULL && lower->acknowledge != NULL &&
              lower->power != NULL);

  /* Allocate and instance of the MAX4321E state structure */

  alloc = (FAR struct usbhost_alloc_s *)
    kmm_zalloc(sizeof(struct usbhost_alloc_s));

  if (alloc < 0)
    {
      usbhost_trace1(MAX3421E_TRACE1_ALLOC_FAIL, 0);
      return NULL;
    }

  priv = &alloc->priv;
  conn = &alloc->conn;

  /* Initialize the state of the host driver */

  ret = max3421e_sw_initialize(priv, conn, lower);
  if (ret < 0)
    {
      goto errout_with_alloc;
    }

  /* Initialize the MAX3421E, putting it into full operational state. */

  ret = max3421e_hw_initialize(priv);
  if (ret < 0)
    {
      goto errout_with_alloc;
    }

  /* Drive Vbus +5V (the smoke test). */

  lower->power(lower, true);

  /* Enable host interrupts */

  lower->enable(lower, true);
  return &conn->conn;

errout_with_alloc:
  kmm_free(alloc);
  return NULL;
}

/****************************************************************************
 * Name: usbhost_trformat1 and usbhost_trformat2
 *
 * Description:
 *   This interface must be provided by platform specific logic that knows
 *   the HCDs encoding of USB trace data.
 *
 *   Given an 9-bit index, return a format string suitable for use with,
 *   say, printf.  The returned format is expected to handle two unsigned
 *   integer values.
 *
 ****************************************************************************/

#ifdef HAVE_USBHOST_TRACE
FAR const char *usbhost_trformat1(uint16_t id)
{
  int ndx = TRACE1_INDEX(id);

  if (ndx < TRACE1_NSTRINGS)
    {
      return g_trace1[ndx].string;
    }

  return NULL;
}

FAR const char *usbhost_trformat2(uint16_t id)
{
  int ndx = TRACE2_INDEX(id);

  if (ndx < TRACE2_NSTRINGS)
    {
      return g_trace2[ndx].string;
    }

  return NULL;
}
#endif

#endif /* CONFIG_USBHOST_MAX3421E */

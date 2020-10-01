/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_usbdev.c
 *
 *   Copyright (C) 2008-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
#include <sys/stat.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/fs/procfs.h>

#include <nuttx/irq.h>
#include <arch/chip/usbdev.h>
#include <arch/chip/pm.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"
#include "cxd56_clock.h"
#include "cxd56_usbdev.h"
#include "hardware/cxd5602_topreg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TOPREG VBUS register */

#define CLR_EDGE (1 << 9)
#define CLR_EN   (1 << 8)
#define VBUS_DET (1 << 0)

/* Configuration ************************************************************/

#ifndef CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100  /* mA */
#endif

/* Vendor ID & Product ID of the USB device */

#ifndef CONFIG_CXD56_VENDORID
#  define CONFIG_CXD56_VENDORID     0x054c
#endif

#ifndef CONFIG_CXD56_PRODUCTID
#  define CONFIG_CXD56_PRODUCTID    0x0bc2
#endif

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#  define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

#ifndef CONFIG_USBDEV_SETUP_MAXDATASIZE
#  define CONFIG_USBDEV_SETUP_MAXDATASIZE (CONFIG_USBDEV_EP0_MAXSIZE * 4)
#endif

#define CONFIG_DEFAULT_PHY_CFG0 \
  (PHY_STAGSELECT | PHY_HSFALLCNTRL | PHY_IHSTX(0xc) | PHY_INHSRFRED | \
   PHY_INHSIPLUS | PHY_INHSDRVSLEW| PHY_INLFSFBCAP)

#ifndef __aligned
#  define __aligned(x) __attribute__((aligned(x)))
#endif

/* Debug ********************************************************************/

/* Trace error codes */

#define CXD56_TRACEERR_ALLOCFAIL         0x0001
#define CXD56_TRACEERR_ATTACHIRQREG      0x0002
#define CXD56_TRACEERR_BINDFAILED        0x0003
#define CXD56_TRACEERR_COREIRQREG        0x0004
#define CXD56_TRACEERR_DRIVER            0x0005
#define CXD56_TRACEERR_DRIVERREGISTERED  0x0006
#define CXD56_TRACEERR_EPREAD            0x0007
#define CXD56_TRACEERR_EWRITE            0x0008
#define CXD56_TRACEERR_INVALIDPARMS      0x0009
#define CXD56_TRACEERR_NOEP              0x000a
#define CXD56_TRACEERR_NOTCONFIGURED     0x000b
#define CXD56_TRACEERR_NULLPACKET        0x000c
#define CXD56_TRACEERR_NULLREQUEST       0x000d
#define CXD56_TRACEERR_REQABORTED        0x000e
#define CXD56_TRACEERR_STALLEDCLRFEATURE 0x000f
#define CXD56_TRACEERR_STALLEDISPATCH    0x0010
#define CXD56_TRACEERR_STALLEDGETST      0x0011
#define CXD56_TRACEERR_STALLEDGETSTEP    0x0012
#define CXD56_TRACEERR_STALLEDGETSTRECIP 0x0013
#define CXD56_TRACEERR_STALLEDREQUEST    0x0014
#define CXD56_TRACEERR_STALLEDSETFEATURE 0x0015
#define CXD56_TRACEERR_TXREQLOST         0x0016
#define CXD56_TRACEERR_RXREQLOST         0x0017
#define CXD56_TRACEERR_VBUSIRQREG        0x0018
#define CXD56_TRACEERR_VBUSNIRQREG       0x0019

/* Trace interrupt codes */

#define CXD56_TRACEINTID_USB             0x0001
#define CXD56_TRACEINTID_SYS             0x0002
#define CXD56_TRACEINTID_VBUS            0x0004
#define CXD56_TRACEINTID_VBUSN           0x0008

#define CXD56_TRACEINTID_RMTWKP          1
#define CXD56_TRACEINTID_ENUM            2
#define CXD56_TRACEINTID_SOF             3
#define CXD56_TRACEINTID_US              4
#define CXD56_TRACEINTID_UR              5
#define CXD56_TRACEINTID_ES              6
#define CXD56_TRACEINTID_SI              7
#define CXD56_TRACEINTID_SC              8
#define CXD56_TRACEINTID_GETSTATUS       9
#define CXD56_TRACEINTID_GETIFDEV        10
#define CXD56_TRACEINTID_CLEARFEATURE    11
#define CXD56_TRACEINTID_SETFEATURE      12
#define CXD56_TRACEINTID_TESTMODE        13
#define CXD56_TRACEINTID_SETADDRESS      14
#define CXD56_TRACEINTID_GETSETDESC      15
#define CXD56_TRACEINTID_GETSETIFCONFIG  16
#define CXD56_TRACEINTID_SYNCHFRAME      17
#define CXD56_TRACEINTID_DISPATCH        18
#define CXD56_TRACEINTID_GETENDPOINT     19
#define CXD56_TRACEINTID_RESUME          20
#define CXD56_TRACEINTID_CDCCLEAR        21
#define CXD56_TRACEINTID_TXDMAERROR      22
#define CXD56_TRACEINTID_RXDMAERROR      23
#define CXD56_TRACEINTID_TXBNA           24
#define CXD56_TRACEINTID_RXBNA           25
#define CXD56_TRACEINTID_XFERDONE        26
#define CXD56_TRACEINTID_TXEMPTY         27
#define CXD56_TRACEINTID_TDC             28
#define CXD56_TRACEINTID_IN              29
#define CXD56_TRACEINTID_EPOUTQEMPTY     31
#define CXD56_TRACEINTID_OUTSETUP        32
#define CXD56_TRACEINTID_OUTDATA         33

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_intdecode[] =
{
  TRACE_STR(CXD56_TRACEINTID_RMTWKP),
  TRACE_STR(CXD56_TRACEINTID_ENUM),
  TRACE_STR(CXD56_TRACEINTID_SOF),
  TRACE_STR(CXD56_TRACEINTID_US),
  TRACE_STR(CXD56_TRACEINTID_UR),
  TRACE_STR(CXD56_TRACEINTID_ES),
  TRACE_STR(CXD56_TRACEINTID_SI),
  TRACE_STR(CXD56_TRACEINTID_SC),
  TRACE_STR(CXD56_TRACEINTID_GETSTATUS),
  TRACE_STR(CXD56_TRACEINTID_GETIFDEV),
  TRACE_STR(CXD56_TRACEINTID_CLEARFEATURE),
  TRACE_STR(CXD56_TRACEINTID_SETFEATURE),
  TRACE_STR(CXD56_TRACEINTID_TESTMODE),
  TRACE_STR(CXD56_TRACEINTID_SETADDRESS),
  TRACE_STR(CXD56_TRACEINTID_GETSETDESC),
  TRACE_STR(CXD56_TRACEINTID_GETSETIFCONFIG),
  TRACE_STR(CXD56_TRACEINTID_SYNCHFRAME),
  TRACE_STR(CXD56_TRACEINTID_DISPATCH),
  TRACE_STR(CXD56_TRACEINTID_GETENDPOINT),
  TRACE_STR(CXD56_TRACEINTID_RESUME),
  TRACE_STR(CXD56_TRACEINTID_CDCCLEAR),
  TRACE_STR(CXD56_TRACEINTID_TXDMAERROR),
  TRACE_STR(CXD56_TRACEINTID_RXDMAERROR),
  TRACE_STR(CXD56_TRACEINTID_TXBNA),
  TRACE_STR(CXD56_TRACEINTID_RXBNA),
  TRACE_STR(CXD56_TRACEINTID_XFERDONE),
  TRACE_STR(CXD56_TRACEINTID_TXEMPTY),
  TRACE_STR(CXD56_TRACEINTID_TDC),
  TRACE_STR(CXD56_TRACEINTID_IN),
  TRACE_STR(CXD56_TRACEINTID_EPOUTQEMPTY),
  TRACE_STR(CXD56_TRACEINTID_OUTSETUP),
  TRACE_STR(CXD56_TRACEINTID_OUTDATA),
  TRACE_STR_END
};

const struct trace_msg_t g_usb_trace_strings_deverror[] =
{
  TRACE_STR(CXD56_TRACEERR_ALLOCFAIL),
  TRACE_STR(CXD56_TRACEERR_ATTACHIRQREG),
  TRACE_STR(CXD56_TRACEERR_BINDFAILED),
  TRACE_STR(CXD56_TRACEERR_COREIRQREG),
  TRACE_STR(CXD56_TRACEERR_DRIVER),
  TRACE_STR(CXD56_TRACEERR_DRIVERREGISTERED),
  TRACE_STR(CXD56_TRACEERR_EPREAD),
  TRACE_STR(CXD56_TRACEERR_EWRITE),
  TRACE_STR(CXD56_TRACEERR_INVALIDPARMS),
  TRACE_STR(CXD56_TRACEERR_NOEP),
  TRACE_STR(CXD56_TRACEERR_NOTCONFIGURED),
  TRACE_STR(CXD56_TRACEERR_NULLPACKET),
  TRACE_STR(CXD56_TRACEERR_NULLREQUEST),
  TRACE_STR(CXD56_TRACEERR_REQABORTED),
  TRACE_STR(CXD56_TRACEERR_STALLEDCLRFEATURE),
  TRACE_STR(CXD56_TRACEERR_STALLEDISPATCH),
  TRACE_STR(CXD56_TRACEERR_STALLEDGETST),
  TRACE_STR(CXD56_TRACEERR_STALLEDGETSTEP),
  TRACE_STR(CXD56_TRACEERR_STALLEDGETSTRECIP),
  TRACE_STR(CXD56_TRACEERR_STALLEDREQUEST),
  TRACE_STR(CXD56_TRACEERR_STALLEDSETFEATURE),
  TRACE_STR(CXD56_TRACEERR_TXREQLOST),
  TRACE_STR(CXD56_TRACEERR_RXREQLOST),
  TRACE_STR(CXD56_TRACEERR_VBUSIRQREG),
  TRACE_STR(CXD56_TRACEERR_VBUSNIRQREG),
  TRACE_STR_END
};
#endif

/* Hardware interface *******************************************************/

/* The CXD56 hardware supports 8 configurable endpoints EP1-4, IN and OUT
 * (in addition to EP0 IN and OUT).  This driver, however, does not exploit
 * the full configuratability of the hardware at this time but, instead,
 * supports the one interrupt IN, one bulk IN and one bulk OUT endpoint.
 */

/* Hardware dependent sizes and numbers */

#define CXD56_EP0MAXPACKET       64   /* EP0 max packet size */
#define CXD56_BULKMAXPACKET      512  /* Bulk endpoint max packet */
#define CXD56_INTRMAXPACKET      64   /* Interrupt endpoint max packet */
#define CXD56_EP0BUFSIZE         64   /* EP0 max packet size */
#define CXD56_BULKBUFSIZE        1024 /* Bulk endpoint max packet */
#define CXD56_INTRBUFSIZE        64   /* Interrupt endpoint max packet */
#define CXD56_NENDPOINTS         7    /* Includes EP0 */

/* Endpoint numbers */

#define CXD56_EP0                0          /* Control endpoint */
#define CXD56_EPBULKIN0          1          /* Bulk EP for send to host */
#define CXD56_EPBULKOUT0         2          /* Bulk EP for recv to host */
#define CXD56_EPINTRIN0          3          /* Intr EP for host poll */
#define CXD56_EPBULKIN1          4          /* Bulk EP for send to host */
#define CXD56_EPBULKOUT1         5          /* Bulk EP for recv to host */
#define CXD56_EPINTRIN1          6          /* Intr EP for host poll */

/* Request queue operations *************************************************/

#define cxd56_rqempty(ep)       ((ep)->head == NULL)
#define cxd56_rqpeek(ep)        ((ep)->head)

#define CXD56_USBDEV_LINELEN 32
#define CXD56_USBDEV_TIMEOUT 1000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cxd56_setup_desc_s
{
  volatile uint32_t status;
  volatile uint32_t reserved;
  volatile uint32_t setup_1;
  volatile uint32_t setup_2;
};

struct cxd56_data_desc_s
{
  volatile uint32_t status;
  volatile uint32_t reserved;
  volatile uint32_t buf;
  volatile uint32_t next;
};

/* A container for a request so that the request make be retained in a list */

struct cxd56_req_s
{
  struct usbdev_req_s req;   /* Standard USB request */
  struct cxd56_req_s *flink; /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct cxd56_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct cxd56_ep_s.
   */

  struct usbdev_ep_s ep; /* Standard endpoint structure */

  /* CXD56-specific fields */

  struct cxd56_usbdev_s *dev; /* Reference to private driver data */
  struct cxd56_req_s *head;   /* Request list for this endpoint */
  struct cxd56_req_s *tail;
  struct cxd56_data_desc_s *desc; /* DMA descriptor */
  void *buffer;                   /* OUT only, receiving data buffer */
  uint8_t epphy;                  /* Physical EP address/index */
  uint8_t stalled : 1;            /* Endpoint is halted */
  uint8_t in : 1;                 /* Endpoint is IN only */
  uint8_t halted : 1;             /* Endpoint feature halted */
  uint8_t txnullpkt : 1;          /* Null packet needed at end of transfer */
  uint8_t txwait : 1;             /* IN transaction already requested from host */
};

/* This structure encapsulates the overall driver state */

struct cxd56_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to struct cxd56_usbdev_s.
   */

  struct usbdev_s usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* CXD56-specific fields */

  uint8_t stalled : 1;     /* 1: Protocol stalled */
  uint8_t selfpowered : 1; /* 1: Device is self powered */
  uint8_t paddrset : 1;    /* 1: Peripheral addr has been set */
  uint8_t attached : 1;    /* 1: Host attached */
  uint8_t paddr;           /* Peripheral address */
  uint8_t avail;

  /* E0 SETUP data buffering.
   *
   * ctrl
   *   The 8-byte SETUP request is received on the EP0 OUT endpoint and is
   *   saved.
   *
   * ep0data
   *   For OUT SETUP requests, the SETUP data phase must also complete before
   *   the SETUP command can be processed.
   *
   * ep0datlen
   *   Length of OUT DATA received in ep0data[]
   */

  struct usb_ctrlreq_s ctrl; /* Last EP0 request */

  uint8_t ep0data[CONFIG_USBDEV_SETUP_MAXDATASIZE];
  uint16_t ep0datlen;
  uint16_t ep0reqlen;

  /* The endpoint list */

  struct cxd56_ep_s eplist[CXD56_NENDPOINTS];

  /* attach status */

  int state;
  int power;

  /* signal */

  int signo;
  int pid;
};

/* For maintaining tables of endpoint info */

struct cxd56_epinfo_s
{
  uint8_t addr;       /* Logical endpoint address */
  uint8_t attr;       /* Endpoint attributes */
  uint16_t maxpacket; /* Max packet size */
  uint16_t bufsize;   /* Buffer size */
};

/* This structure describes one open "file" */

struct cxd56_usbdev_file_s
{
  struct procfs_file_s base; /* Base open file structure */
  unsigned int linesize;     /* Number of valid characters in line[] */

  /* Pre-allocated buffer for formatted lines */

  char line[CXD56_USBDEV_LINELEN];
};

static struct pm_cpu_freqlock_s g_hv_lock =
  PM_CPUFREQLOCK_INIT(PM_CPUFREQLOCK_TAG('U', 'S', 0),
  PM_CPUFREQLOCK_FLAG_HV);
static struct pm_cpu_wakelock_s g_wake_lock =
{
  .count = 0,
  .info  = PM_CPUWAKELOCK_TAG('U', 'S', 0),
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Request queue operations *************************************************/

static FAR struct cxd56_req_s *cxd56_rqdequeue(
                            FAR struct cxd56_ep_s *privep);
static void cxd56_rqenqueue(FAR struct cxd56_ep_s *privep,
                            FAR struct cxd56_req_s *req);

/* Low level data transfers and request operations */

static int cxd56_epwrite(FAR struct cxd56_ep_s *privep, FAR uint8_t *buf,
                         uint16_t nbytes);
static inline void cxd56_abortrequest(FAR struct cxd56_ep_s *privep,
                                      FAR struct cxd56_req_s *privreq,
                                      int16_t result);
static void cxd56_reqcomplete(FAR struct cxd56_ep_s *privep, int16_t result);
static int cxd56_wrrequest(FAR struct cxd56_ep_s *privep);
static int cxd56_rdrequest(FAR struct cxd56_ep_s *privep);
static void cxd56_cancelrequests(FAR struct cxd56_ep_s *privep);
static void cxd56_usbdevreset(FAR struct cxd56_usbdev_s *priv);
static void cxd56_usbreset(FAR struct cxd56_usbdev_s *priv);

/* Interrupt handling */

static FAR struct cxd56_ep_s *
  cxd56_epfindbyaddr(FAR struct cxd56_usbdev_s *priv, uint16_t eplog);
static void cxd56_dispatchrequest(FAR struct cxd56_usbdev_s *priv);
static inline void cxd56_ep0setup(FAR struct cxd56_usbdev_s *priv);
static int cxd56_usbinterrupt(int irq, FAR void *context, FAR void *arg);
static int cxd56_sysinterrupt(int irq, FAR void *context, FAR void *arg);
static int cxd56_vbusinterrupt(int irq, FAR void *context, FAR void *arg);
static int cxd56_vbusninterrupt(int irq, FAR void *context, FAR void *arg);

/* Initialization operations */

static inline void cxd56_ep0hwinitialize(FAR struct cxd56_usbdev_s *priv);
static void cxd56_ctrlinitialize(FAR struct cxd56_usbdev_s *priv);
static void cxd56_epinitialize(FAR struct cxd56_usbdev_s *priv);

/* Un-initialization operations */

static void cxd56_usbhwuninit(void);

/* Endpoint methods */

static int cxd56_epconfigure(FAR struct usbdev_ep_s *ep,
                             FAR const struct usb_epdesc_s *desc, bool last);
static int cxd56_epdisable(FAR struct usbdev_ep_s *ep);
static FAR struct usbdev_req_s *cxd56_epallocreq(FAR struct usbdev_ep_s *ep);
static void cxd56_epfreereq(FAR struct usbdev_ep_s *ep,
                            FAR struct usbdev_req_s *req);
#ifdef CONFIG_USBDEV_DMA
static FAR void *cxd56_epallocbuffer(FAR struct usbdev_ep_s *ep,
                                     uint16_t nbytes);
static void cxd56_epfreebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf);
#endif
static int cxd56_epsubmit(FAR struct usbdev_ep_s *ep,
                          FAR struct usbdev_req_s *privreq);
static int cxd56_epcancel(FAR struct usbdev_ep_s *ep,
                          FAR struct usbdev_req_s *privreq);
static int cxd56_epstall(FAR struct usbdev_ep_s *ep, bool resume);

/* USB device controller methods */

static FAR struct usbdev_ep_s *cxd56_allocep(FAR struct usbdev_s *dev,
                                             uint8_t epno, bool in,
                                             uint8_t eptype);
static void cxd56_freeep(FAR struct usbdev_s *dev,
                         FAR struct usbdev_ep_s *ep);
static int cxd56_getframe(FAR struct usbdev_s *dev);
static int cxd56_wakeup(FAR struct usbdev_s *dev);
static int cxd56_selfpowered(FAR struct usbdev_s *dev, bool selfpowered);
static int cxd56_pullup(FAR struct usbdev_s *dev, bool enable);

/* Notify USB device attach/detach signal */

static void cxd56_notify_signal(uint16_t state, uint16_t power);

#ifdef CONFIG_FS_PROCFS

/* procfs methods */

static int cxd56_usbdev_open(FAR struct file *filep, FAR const char *relpath,
                             int oflags, mode_t mode);
static int cxd56_usbdev_close(FAR struct file *filep);
static ssize_t cxd56_usbdev_read(FAR struct file *filep, FAR char *buffer,
                                 size_t buflen);
static int cxd56_usbdev_dup(FAR const struct file *oldp,
                            FAR struct file *newp);
static int cxd56_usbdev_stat(FAR const char *relpath, FAR struct stat *buf);

#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Endpoint methods */

static const struct usbdev_epops_s g_epops =
{
  .configure   = cxd56_epconfigure,
  .disable     = cxd56_epdisable,
  .allocreq    = cxd56_epallocreq,
  .freereq     = cxd56_epfreereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = cxd56_epallocbuffer,
  .freebuffer  = cxd56_epfreebuffer,
#endif
  .submit      = cxd56_epsubmit,
  .cancel      = cxd56_epcancel,
  .stall       = cxd56_epstall,
};

/* USB controller device methods */

static const struct usbdev_ops_s g_devops =
{
  .allocep     = cxd56_allocep,
  .freeep      = cxd56_freeep,
  .getframe    = cxd56_getframe,
  .wakeup      = cxd56_wakeup,
  .selfpowered = cxd56_selfpowered,
  .pullup      = cxd56_pullup,
};

/* There is only one, single, pre-allocated instance of the driver
 * structure.
 */

static struct cxd56_usbdev_s g_usbdev;

/* DMA Descriptors for each endpoints */

static struct cxd56_setup_desc_s __aligned(4) g_ep0setup;
static struct cxd56_data_desc_s __aligned(4) g_ep0in;
static struct cxd56_data_desc_s __aligned(4) g_ep0out;

/* Summarizes information about all CXD56 endpoints */

static const struct cxd56_epinfo_s g_epinfo[CXD56_NENDPOINTS] =
{
  {
    CXD56_EP0,                /* EP0 */
    USB_EP_ATTR_XFER_CONTROL, /* Type: Control IN/OUT */
    CXD56_EP0MAXPACKET,       /* Max packet size */
    CXD56_EP0BUFSIZE,         /* Buffer size */
  },
  {
    CXD56_EPBULKIN0 | USB_DIR_IN, /* Logical endpoint number: 1 IN */
    USB_EP_ATTR_XFER_BULK,        /* Type: Bulk */
    CXD56_BULKMAXPACKET,          /* Max packet size */
    CXD56_BULKBUFSIZE,            /* Buffer size */
  },
  {
    CXD56_EPBULKOUT0 | USB_DIR_OUT, /* Logical endpoint number: 2 OUT */
    USB_EP_ATTR_XFER_BULK,          /* Type: Bulk */
    CXD56_BULKMAXPACKET,            /* Max packet size */
    CXD56_BULKBUFSIZE,              /* Buffer size */
  },
  {
    CXD56_EPINTRIN0 | USB_DIR_IN, /* Logical endpoint number: 3 IN */
    USB_EP_ATTR_XFER_INT,         /* Type: Interrupt */
    CXD56_INTRMAXPACKET,          /* Max packet size */
    CXD56_INTRBUFSIZE,            /* Buffer size */
  },
  {
    CXD56_EPBULKIN1 | USB_DIR_IN, /* Logical endpoint number: 4 IN */
    USB_EP_ATTR_XFER_BULK,        /* Type: Bulk */
    CXD56_BULKMAXPACKET,          /* Max packet size */
    CXD56_BULKBUFSIZE,            /* Buffer size */
  },
  {
    CXD56_EPBULKOUT1 | USB_DIR_OUT, /* Logical endpoint number: 5 OUT */
    USB_EP_ATTR_XFER_BULK,          /* Type: Bulk */
    CXD56_BULKMAXPACKET,            /* Max packet size */
    CXD56_BULKBUFSIZE,              /* Buffer size */
  },
  {
    CXD56_EPINTRIN1 | USB_DIR_IN, /* Logical endpoint number: 6 IN */
    USB_EP_ATTR_XFER_INT,         /* Type: Interrupt */
    CXD56_INTRMAXPACKET,          /* Max packet size */
    CXD56_INTRBUFSIZE,            /* Buffer size */
  }
};

static uint8_t g_ep0outbuffer[CXD56_EP0MAXPACKET];

#ifdef CONFIG_FS_PROCFS

/* See include/nutts/fs/procfs.h
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct procfs_operations cxd56_usbdev_operations =
{
  cxd56_usbdev_open,  /* open */
  cxd56_usbdev_close, /* close */
  cxd56_usbdev_read,  /* read */
  NULL,               /* write */
  cxd56_usbdev_dup,   /* dup */

  NULL,               /* opendir */
  NULL,               /* closedir */
  NULL,               /* readdir */
  NULL,               /* rewinddir */

  cxd56_usbdev_stat   /* stat */
};

#  ifdef CONFIG_FS_PROCFS_REGISTER
static const struct procfs_entry_s g_procfs_usbdev =
{
  "usbdev",
  &cxd56_usbdev_operations
};
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_cablestatus
 *
 * Description:
 *   Set VBUS connected status to system register
 *
 ****************************************************************************/

static inline void cxd56_cableconnected(bool connected)
{
  uint32_t val;

  val = getreg32(CXD56_TOPREG_USB_VBUS);
  if (connected)
    {
      putreg32(val | VBUS_DET, CXD56_TOPREG_USB_VBUS);
    }
  else
    {
      putreg32(val & ~VBUS_DET, CXD56_TOPREG_USB_VBUS);
    }
}

/****************************************************************************
 * Name: cxd56_iscableconnected
 *
 * Description:
 *   Return the cable status. (true is connected)
 *
 ****************************************************************************/

static inline bool cxd56_iscableconnected(void)
{
  return getreg32(CXD56_TOPREG_USB_VBUS) & VBUS_DET;
}

/****************************************************************************
 * Name: cxd56_rqdequeue
 *
 * Description:
 *   Remove a request from an endpoint request queue
 *
 ****************************************************************************/

static FAR struct cxd56_req_s *cxd56_rqdequeue(FAR struct cxd56_ep_s *privep)
{
  FAR struct cxd56_req_s *ret = privep->head;

  if (ret)
    {
      privep->head = ret->flink;
      if (!privep->head)
        {
          privep->tail = NULL;
        }

      ret->flink = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: cxd56_rqenqueue
 *
 * Description:
 *   Add a request from an endpoint request queue
 *
 ****************************************************************************/

static void cxd56_rqenqueue(FAR struct cxd56_ep_s *privep,
                            FAR struct cxd56_req_s *req)
{
  req->flink = NULL;
  if (!privep->head)
    {
      privep->head = req;
      privep->tail = req;
    }
  else
    {
      privep->tail->flink = req;
      privep->tail        = req;
    }
}

/****************************************************************************
 * Name: cxd56_epwrite
 *
 * Description:
 *   Endpoint write (IN)
 *
 ****************************************************************************/

static int cxd56_epwrite(FAR struct cxd56_ep_s *privep, FAR uint8_t *buf,
                         uint16_t nbytes)
{
  FAR struct cxd56_data_desc_s *desc;
  uint32_t ctrl;
  uint8_t epphy = privep->epphy;

  /* Setup IN descriptor */

  desc = epphy == 0 ? &g_ep0in : privep->desc;

  if (IS_BS_DMA_BUSY(desc))
    {
      return 0;
    }

  desc->buf    = (uint32_t)(uintptr_t)buf;
  desc->status = nbytes | DESC_LAST; /* always last descriptor */

  /* Set Poll bit to ready to send */

  ctrl = getreg32(CXD56_USB_IN_EP_CONTROL(epphy));

  /* Send NULL packet request */

  if (privep->txnullpkt)
    {
      ctrl |= USB_SENDNULL;
    }

  putreg32(ctrl | USB_P | USB_CNAK, CXD56_USB_IN_EP_CONTROL(epphy));

  return nbytes;
}

/****************************************************************************
 * Name: cxd56_abortrequest
 *
 * Description:
 *   Discard a request
 *
 ****************************************************************************/

static inline void cxd56_abortrequest(FAR struct cxd56_ep_s *privep,
                                      FAR struct cxd56_req_s *privreq,
                                      int16_t result)
{
  usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_REQABORTED),
          (uint16_t)privep->epphy);

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);
}

/****************************************************************************
 * Name: cxd56_reqcomplete
 *
 * Description:
 *   Handle termination of a request.
 *
 ****************************************************************************/

static void cxd56_reqcomplete(FAR struct cxd56_ep_s *privep, int16_t result)
{
  FAR struct cxd56_req_s *privreq;
  int stalled = privep->stalled;
  irqstate_t flags;

  /* Remove the completed request at the head of the endpoint request list */

  flags   = enter_critical_section();
  privreq = cxd56_rqdequeue(privep);
  leave_critical_section(flags);

  if (privreq)
    {
      /* If endpoint 0, temporarily reflect the state of protocol stalled
       * in the callback.
       */

      if (privep->epphy == CXD56_EP0)
        {
          privep->stalled = privep->dev->stalled;
        }

      /* Save the result in the request structure */

      privreq->req.result = result;

      /* Callback to the request completion handler */

      privreq->flink = NULL;
      privreq->req.callback(&privep->ep, &privreq->req);

      /* Restore the stalled indication */

      privep->stalled = stalled;
    }
}

/****************************************************************************
 * Name: cxd56_txdmacomplete
 *
 * Description:
 *   DMA transfer to TxFIFO is completed.
 *   if exist queued request, do the next transfer request.
 *
 ****************************************************************************/

static void cxd56_txdmacomplete(FAR struct cxd56_ep_s *privep)
{
  FAR struct cxd56_data_desc_s *desc;
  FAR struct cxd56_req_s *privreq;

  desc = privep->epphy == CXD56_EP0 ? &g_ep0in : privep->desc;

  /* Avoid invalid transfer by USB Core */

  DEBUGASSERT(IS_BS_DMA_DONE(desc));

  desc->status |= DESC_BS_HOST_BUSY;

  privreq = cxd56_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_TXREQLOST), privep->epphy);
    }
  else
    {
      privreq->req.xfrd += desc->status & DESC_SIZE_MASK;

      if (privreq->req.xfrd >= privreq->req.len && !privep->txnullpkt)
        {
          usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
          privep->txnullpkt = 0;
          cxd56_reqcomplete(privep, OK);
        }
    }
}

/****************************************************************************
 * Name: cxd56_wrrequest
 *
 * Description:
 *   Send from the next queued write request
 *
 * Returned Value:
 *  0:not finished; 1:completed; <0:error
 *
 ****************************************************************************/

static int cxd56_wrrequest(FAR struct cxd56_ep_s *privep)
{
  FAR struct cxd56_req_s *privreq;
  FAR uint8_t *buf;
  int nbytes;
  int bytesleft;

  /* Check the request from the head of the endpoint request queue */

  privreq = cxd56_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_NULLREQUEST), 0);
      return OK;
    }

  /* Ignore any attempt to send a zero length packet on anything but EP0IN */

  if (privreq->req.len == 0)
    {
      if (privep->epphy == CXD56_EP0)
        {
          cxd56_epwrite(privep, NULL, 0);
        }
      else
        {
          usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_NULLPACKET), 0);
        }

      return OK;
    }

  /* Get the number of bytes left to be sent in the packet */

  bytesleft = privreq->req.len - privreq->req.xfrd;

  /* Send the next packet if (1) there are more bytes to be sent, or
   * (2) the last packet sent was exactly maxpacketsize (bytesleft == 0)
   */

  usbtrace(TRACE_WRITE(privep->epphy), (uint16_t)bytesleft);
  if (bytesleft > 0 || privep->txnullpkt)
    {
      /* Try to send maxpacketsize -- unless we don't have that many
       * bytes to send.
       */

      privep->txnullpkt = 0;
      if (bytesleft > privep->ep.maxpacket)
        {
          nbytes = privep->ep.maxpacket;
        }
      else
        {
          nbytes = bytesleft;
          if ((privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0)
            {
              privep->txnullpkt = (bytesleft == privep->ep.maxpacket);
            }
        }

      /* Send the largest number of bytes that we can in this packet */

      buf = privreq->req.buf + privreq->req.xfrd;
      cxd56_epwrite(privep, buf, nbytes);
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_rxdmacomplete
 *
 * Description:
 *   Notify the upper layer and continue to next receive request.
 *
 ****************************************************************************/

static void cxd56_rxdmacomplete(FAR struct cxd56_ep_s *privep)
{
  FAR struct cxd56_data_desc_s *desc = privep->desc;
  FAR struct cxd56_req_s *privreq;
  uint32_t status = desc->status;
  uint16_t nrxbytes;

  nrxbytes = status & DESC_SIZE_MASK;

  privreq = cxd56_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_RXREQLOST), privep->epphy);
      return;
    }

  desc->status = DESC_BS_HOST_BUSY;

  if ((status & DESC_BS_MASK) == DESC_BS_DMA_DONE)
    {
      privreq->req.xfrd += nrxbytes;

      if (privreq->req.xfrd >= privreq->req.len ||
          nrxbytes < privep->ep.maxpacket)
        {
          usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
          cxd56_reqcomplete(privep, OK);
        }
    }
  else
    {
      uerr("Descriptor status error %08x\n", status);
    }

  cxd56_rdrequest(privep);
}

/****************************************************************************
 * Name: cxd56_rdrequest
 *
 * Description:
 *   Receive to the next queued read request
 *
 ****************************************************************************/

static int cxd56_rdrequest(FAR struct cxd56_ep_s *privep)
{
  FAR struct cxd56_data_desc_s *desc = privep->desc;
  FAR struct cxd56_req_s *privreq;
  uint32_t ctrl;

  /* Check the request from the head of the endpoint request queue */

  privreq = cxd56_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_EPOUTQEMPTY), 0);
      return OK;
    }

  /* Receive the next packet */

  if (!IS_BS_HOST_BUSY(desc))
    {
      return OK;
    }

  usbtrace(TRACE_READ(privep->epphy), privep->ep.maxpacket);

  desc->buf    = (uint32_t)(uintptr_t)privreq->req.buf;
  desc->status = privep->ep.maxpacket | DESC_LAST;

  /* Ready to receive next packet */

  ctrl = getreg32(CXD56_USB_OUT_EP_CONTROL(privep->epphy));
  putreg32(ctrl | USB_RRDY | USB_CNAK,
           CXD56_USB_OUT_EP_CONTROL(privep->epphy));

  return OK;
}

/****************************************************************************
 * Name: cxd56_stopinep
 *
 * Description:
 *   Stop IN endpoint forcibly
 *
 ****************************************************************************/

static void cxd56_stopinep(FAR struct cxd56_ep_s *privep)
{
  uint32_t ctrl;

  /* Stop TX */

  ctrl = getreg32(CXD56_USB_IN_EP_CONTROL(privep->epphy));
  ctrl |= USB_F;
  putreg32(ctrl, CXD56_USB_IN_EP_CONTROL(privep->epphy));
}

/****************************************************************************
 * Name: cxd56_stopoutep
 *
 * Description:
 *   Stop OUT endpoint forcibly
 *
 ****************************************************************************/

static void cxd56_stopoutep(FAR struct cxd56_ep_s *privep)
{
  uint32_t ctrl;
  uint32_t stat;

  /* Stop RX if FIFO not empty */

  stat = getreg32(CXD56_USB_OUT_EP_STATUS(privep->epphy));
  if (stat & USB_INT_MRXFIFOEMPTY)
    {
      return;
    }

  ctrl = getreg32(CXD56_USB_OUT_EP_CONTROL(privep->epphy));
  ctrl |= USB_CLOSEDESC | USB_MRXFLUSH;
  putreg32(ctrl, CXD56_USB_OUT_EP_CONTROL(privep->epphy));
}

/****************************************************************************
 * Name: cxd56_cancelrequests
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 ****************************************************************************/

static void cxd56_cancelrequests(FAR struct cxd56_ep_s *privep)
{
  if (privep->epphy > 0)
    {
      if (privep->in)
        {
          cxd56_stopinep(privep);
        }
      else
        {
          cxd56_stopoutep(privep);
        }
    }

  while (!cxd56_rqempty(privep))
    {
      usbtrace(TRACE_COMPLETE(privep->epphy),
              (cxd56_rqpeek(privep))->req.xfrd);
      cxd56_reqcomplete(privep, -ESHUTDOWN);
    }

  if (privep->epphy > 0)
    {
      if (privep->in)
        {
          putreg32(0, CXD56_USB_IN_EP_DATADESC(privep->epphy));
        }
      else
        {
          putreg32(0, CXD56_USB_OUT_EP_DATADESC(privep->epphy));
        }
    }
}

/****************************************************************************
 * Name: cxd56_epfindbyaddr
 *
 * Description:
 *   Find the physical endpoint structure corresponding to a logic endpoint
 *   address
 *
 ****************************************************************************/

static FAR struct cxd56_ep_s *
cxd56_epfindbyaddr(FAR struct cxd56_usbdev_s *priv, uint16_t eplog)
{
  FAR struct cxd56_ep_s *privep;
  int i;

  /* Endpoint zero is a special case */

  if (USB_EPNO(eplog) == 0)
    {
      return &priv->eplist[0];
    }

  /* Handle the remaining */

  for (i = 1; i < CXD56_NENDPOINTS; i++)
    {
      privep = &priv->eplist[i];

      /* Same logical endpoint number? (includes direction bit) */

      if (eplog == privep->ep.eplog)
        {
          /* Return endpoint found */

          return privep;
        }
    }

  /* Return endpoint not found */

  return NULL;
}

/****************************************************************************
 * Name: cxd56_dispatchrequest
 *
 * Description:
 *   Provide unhandled setup actions to the class driver
 *
 ****************************************************************************/

static void cxd56_dispatchrequest(FAR struct cxd56_usbdev_s *priv)
{
  int ret;

  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_DISPATCH), 0);
  if (priv && priv->driver)
    {
      ret = CLASS_SETUP(priv->driver, &priv->usbdev, &priv->ctrl,
                        priv->ep0data, priv->ep0datlen);
      if (ret < 0)
        {
          /* Stall on failure */

          usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_STALLEDISPATCH),
                   priv->ctrl.req);
          priv->stalled = 1;
        }
    }
}

/****************************************************************************
 * Name: cxd56_ep0setup
 *
 * Description:
 *   USB Ctrl EP Setup Event
 *
 ****************************************************************************/

static inline void cxd56_ep0setup(FAR struct cxd56_usbdev_s *priv)
{
  FAR struct cxd56_ep_s *ep0      = &priv->eplist[0];
  FAR struct cxd56_req_s *privreq = cxd56_rqpeek(ep0);
  FAR struct cxd56_ep_s *privep;
  uint16_t index;
  uint16_t value;
  uint16_t len;
  uint32_t reg;

  /* Terminate any pending requests */

  while (!cxd56_rqempty(ep0))
    {
      int16_t result = OK;
      if (privreq->req.xfrd != privreq->req.len)
        {
          result = -EPROTO;
        }

      usbtrace(TRACE_COMPLETE(ep0->epphy), privreq->req.xfrd);
      cxd56_reqcomplete(ep0, result);
    }

  /* Assume NOT stalled */

  ep0->stalled  = 0;
  priv->stalled = 0;

  /* Read EP0 SETUP data */

  memcpy(&priv->ctrl, (void *)&g_ep0setup.setup_1, USB_SIZEOF_CTRLREQ);
  memset(&g_ep0setup, 0, USB_SIZEOF_CTRLREQ);

  index = GETUINT16(priv->ctrl.index);
  value = GETUINT16(priv->ctrl.value);
  len   = GETUINT16(priv->ctrl.len);

  priv->ep0reqlen = len;

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        priv->ctrl.type, priv->ctrl.req, value, index, len);

  ep0->in = (priv->ctrl.type & USB_DIR_IN) != 0;

  /* Is this an setup with OUT and data of length > 0 */

  if (USB_REQ_ISOUT(priv->ctrl.type) && len != priv->ep0datlen)
    {
      /* At this point priv->ctrl is the setup packet. */

      return;
    }

  /* Dispatch any non-standard requests */

  if ((priv->ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      cxd56_dispatchrequest(priv);
    }
  else
    {
      /* Handle standard request.  Pick off the things of interest to the
       * USB device controller driver; pass what is left to the class driver
       */

      switch (priv->ctrl.req)
        {
          case USB_REQ_GETSTATUS:
            {
              /* type:  device-to-host;
               *        recipient = device,
               *        interface,
               *        endpoint
               * value: 0
               * index: zero interface endpoint
               * len:   2; data = status
               */

              usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_GETSTATUS), 0);

              if (len != 2 || (priv->ctrl.type & USB_REQ_DIR_IN) == 0 ||
                  value != 0)
                {
                  usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_STALLEDGETST),
                           priv->ctrl.req);
                  priv->stalled = 1;
                }
              else
                {
                  switch (priv->ctrl.type & USB_REQ_RECIPIENT_MASK)
                    {
                      case USB_REQ_RECIPIENT_ENDPOINT:
                        {
                          usbtrace(TRACE_INTDECODE(
                                   CXD56_TRACEINTID_GETENDPOINT),
                                   0);
                          privep = cxd56_epfindbyaddr(priv, index);
                          if (!privep)
                            {
                              usbtrace(
                                TRACE_DEVERROR(
                                CXD56_TRACEERR_STALLEDGETSTEP),
                                priv->ctrl.type);
                              priv->stalled = 1;
                            }
                        }
                        break;

                      case USB_REQ_RECIPIENT_DEVICE:
                      case USB_REQ_RECIPIENT_INTERFACE:
                        usbtrace(TRACE_INTDECODE(
                                 CXD56_TRACEINTID_GETIFDEV),
                                 0);
                        break;

                      default:
                        {
                          usbtrace(TRACE_DEVERROR(
                                   CXD56_TRACEERR_STALLEDGETSTRECIP),
                                   priv->ctrl.type);
                          priv->stalled = 1;
                        }
                        break;
                    }
                }
            }
            break;

          case USB_REQ_CLEARFEATURE:
            {
              /* type:  host-to device;
               *        recipient = device,
               *        interface or endpoint
               * value: feature selector
               * index: zero interface endpoint;
               * len:   zero, data = none
               */

              usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_CLEARFEATURE),
                       (uint16_t)priv->ctrl.req);
              if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) !=
                  USB_REQ_RECIPIENT_ENDPOINT)
                {
                  cxd56_dispatchrequest(priv);
                }
              else if (priv->paddrset != 0 &&
                       value == USB_FEATURE_ENDPOINTHALT &&
                       len == 0 &&
                       (privep = cxd56_epfindbyaddr(priv, index)) != NULL)
                {
                  privep->halted = 0;
                  cxd56_epstall(&privep->ep, true);
                  cxd56_epwrite(ep0, NULL, 0);
                }
              else
                {
                  usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_STALLEDCLRFEATURE),
                           priv->ctrl.type);
                  priv->stalled = 1;
                }
            }
            break;

          case USB_REQ_SETFEATURE:
            {
              /* type:  host-to-device;
               *        recipient = device,
               *        interface,
               *        endpoint
               * value: feature selector
               * index: zero interface endpoint;
               * len:   0; data = none
               */

              usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_SETFEATURE), 0);
              if (priv->ctrl.type == USB_REQ_RECIPIENT_DEVICE &&
                  value == USB_FEATURE_TESTMODE)
                {
                  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_TESTMODE),
                           index);
                }
              else if (priv->ctrl.type != USB_REQ_RECIPIENT_ENDPOINT)
                {
                  cxd56_dispatchrequest(priv);
                }
              else if (value == USB_FEATURE_ENDPOINTHALT && len == 0 &&
                       (privep = cxd56_epfindbyaddr(priv, index)) != NULL)
                {
                  privep->halted = 1;
                }
              else
                {
                  usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_STALLEDSETFEATURE),
                           priv->ctrl.type);
                  priv->stalled = 1;
                }
            }
            break;

          case USB_REQ_SETADDRESS:
            {
              /* type:  host-to-device; recipient = device
               * value: device address
               * index: 0
               * len:   0; data = none
               */

              usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_SETADDRESS), 0);
              priv->paddr = value & 0xff;
            }
            break;

          case USB_REQ_GETDESCRIPTOR:
            /* type:  device-to-host; recipient = device
             * value: descriptor type and index
             * index: 0 or language ID;
             * len:   descriptor len; data = descriptor
             */

          case USB_REQ_SETDESCRIPTOR:
            /* type:  host-to-device; recipient = device
             * value: descriptor type and index
             * index: 0 or language ID;
             * len:   descriptor len; data = descriptor
             */

            {
              usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_GETSETDESC), 0);
              cxd56_dispatchrequest(priv);
            }

            break;

          case USB_REQ_GETCONFIGURATION:
            /* type:  device-to-host; recipient = device
             * value: 0;
             * index: 0;
             * len:   1; data = configuration value
             */

          case USB_REQ_SETCONFIGURATION:
            /* type:  host-to-device; recipient = device
             * value: configuration value
             * index: 0;
             * len:   0; data = none
             */

          case USB_REQ_GETINTERFACE:
            /* type:  device-to-host; recipient = interface
             * value: 0
             * index: interface;
             * len:   1; data = alt interface
             */

          case USB_REQ_SETINTERFACE:
            /* type:  host-to-device; recipient = interface
             * value: alternate setting
             * index: interface;
             * len:   0; data = none
             */

            {
              usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_GETSETIFCONFIG), 0);
              cxd56_dispatchrequest(priv);
            }

            break;

          case USB_REQ_SYNCHFRAME:
            {
              /* type:  device-to-host; recipient = endpoint
               * value: 0
               * index: endpoint;
               * len:   2; data = frame number
               */

              usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_SYNCHFRAME), 0);
            }
            break;

          default:
            {
              usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_STALLEDREQUEST),
                       priv->ctrl.req);
              priv->stalled = 1;
            }
            break;
        }
    }

  /* Check if the setup processing resulted in a STALL */

  if (priv->stalled)
    {
      reg = getreg32(CXD56_USB_IN_EP_CONTROL(0));
      putreg32(reg | USB_STALL, CXD56_USB_IN_EP_CONTROL(0));
      reg = getreg32(CXD56_USB_OUT_EP_CONTROL(0));
      putreg32(reg | USB_STALL, CXD56_USB_OUT_EP_CONTROL(0));
    }
}

/****************************************************************************
 * Name: cxd56_epinterrupt
 *
 * Description:
 *   Handle USB endpoint interrupts
 *
 ****************************************************************************/

static int cxd56_epinterrupt(int irq, FAR void *context)
{
  FAR struct cxd56_usbdev_s *priv = &g_usbdev;
  FAR struct cxd56_ep_s *privep;
  uint32_t eps;
  uint32_t stat;
  uint32_t ctrl;
  uint16_t len;
  int n;

  eps = getreg32(CXD56_USB_DEV_EP_INTR);
    {
      for (n = 0; n < CXD56_NENDPOINTS; n++)
        {
          /* Determine IN endpoint interrupts */

          privep = &priv->eplist[n];

          if (eps & (1 << n))
            {
              stat = getreg32(CXD56_USB_IN_EP_STATUS(n));

              if (stat & USB_INT_RCS)
                {
                  /* Handle Clear_Feature */

                  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_CLEARFEATURE),
                                           n);
                  ctrl = getreg32(CXD56_USB_IN_EP_CONTROL(n));
                  putreg32(ctrl | USB_F, CXD56_USB_IN_EP_CONTROL(n));
                  putreg32(ctrl | USB_CNAK, CXD56_USB_IN_EP_CONTROL(n));
                  ctrl = getreg32(CXD56_USB_IN_EP_CONTROL(n));
                  putreg32(USB_INT_RCS, CXD56_USB_IN_EP_STATUS(n));

                  privep->stalled = 0;
                  privep->halted = 0;
                }

              if (stat & USB_INT_RSS)
                {
                  /* Handle Set_Feature */

                  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_SETFEATURE), n);
                  putreg32(USB_INT_RSS, CXD56_USB_IN_EP_STATUS(n));
                  privep->halted = 1;
                }

              if (stat & USB_INT_TXEMPTY)
                {
                  /* Transmit FIFO Empty detected */

                  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_TXEMPTY), n);
                  putreg32(USB_INT_TXEMPTY, CXD56_USB_IN_EP_STATUS(n));
                }

              if (stat & USB_INT_TDC)
                {
                  /* DMA Transmit complete for TxFIFO */

                  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_TDC), n);
                  putreg32(USB_INT_TDC, CXD56_USB_IN_EP_STATUS(n));
                }

              if (stat & USB_INT_XFERDONE)
                {
                  /* Transfer Done/Transmit FIFO Empty */

                  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_XFERDONE), n);

                  /* Set NAK during processing IN request completion */

                  ctrl = getreg32(CXD56_USB_IN_EP_CONTROL(n));
                  putreg32(ctrl | USB_SNAK, CXD56_USB_IN_EP_CONTROL(n));

                  putreg32(USB_INT_XFERDONE, CXD56_USB_IN_EP_STATUS(n));

                  cxd56_txdmacomplete(privep);

                  /* Clear NAK to raise IN interrupt for send next IN
                   * packets.
                   */

                  putreg32(ctrl | USB_CNAK, CXD56_USB_IN_EP_CONTROL(n));
                }

              if (stat & USB_INT_IN)
                {
                  /* Reply NAK for IN token when TxFIFO empty */

                  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_IN), n);

                  ctrl = getreg32(CXD56_USB_IN_EP_CONTROL(n));
                  putreg32(ctrl | USB_SNAK, CXD56_USB_IN_EP_CONTROL(n));

                  /* If IN request is ready, then send it. */

                  if (!cxd56_rqempty(privep))
                    {
                      cxd56_wrrequest(privep);
                    }
                  else
                    {
                      privep->txwait = 1;
                    }

                  putreg32(USB_INT_IN, CXD56_USB_IN_EP_STATUS(n));
                }

              if (stat & USB_INT_HE)
                {
                  /* Detect AHB Bus error */

                  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_TXDMAERROR), n);
                  putreg32(USB_INT_HE, CXD56_USB_IN_EP_STATUS(n));
                }

              if (stat & USB_INT_BNA)
                {
                  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_TXBNA), n);
                  putreg32(USB_INT_BNA, CXD56_USB_IN_EP_STATUS(n));
                }

              putreg32(1 << n, CXD56_USB_DEV_EP_INTR);
            }

          /* Determine OUT endpoint interrupts */

          if (eps & (1 << (n + 16)))
            {
              stat = getreg32(CXD56_USB_OUT_EP_STATUS(n));

              if (USB_INT_OUT(stat) == USB_INT_OUT_SETUP)
                {
                  putreg32(USB_INT_OUT_SETUP, CXD56_USB_OUT_EP_STATUS(n));
                  if (n == 0)
                    {
                      usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_OUTSETUP),
                                               0);

                      ctrl = getreg32(CXD56_USB_OUT_EP_CONTROL(0));
                      putreg32(ctrl | USB_SNAK, CXD56_USB_OUT_EP_CONTROL(0));

                      cxd56_ep0setup(priv);

                      putreg32(ctrl | USB_CNAK | USB_RRDY,
                               CXD56_USB_OUT_EP_CONTROL(0));

                      ctrl = getreg32(CXD56_USB_IN_EP_CONTROL(0));
                      putreg32(ctrl | USB_CNAK, CXD56_USB_IN_EP_CONTROL(0));
                    }
                }

              if (USB_INT_OUT(stat) == USB_INT_OUT_DATA)
                {
                  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_OUTDATA), n);
                  putreg32(USB_INT_OUT_DATA, CXD56_USB_OUT_EP_STATUS(n));
                  if (n == 0)
                    {
                      len = g_ep0out.status & DESC_SIZE_MASK;

                      /* Reset DMA descriptor for next packet */

                      g_ep0out.status = privep->ep.maxpacket | DESC_LAST;

                      if (0 < len)
                        {
                          ASSERT(priv->ep0datlen + len <=
                                 sizeof(priv->ep0data));

                          memcpy(priv->ep0data + priv->ep0datlen,
                                 (const void *)g_ep0out.buf,
                                 len);

                          priv->ep0datlen += len;
                        }

                      /* Dispatch to cxd56_ep0setup if received all OUT
                       * data.
                       */

                      if (priv->ep0datlen == priv->ep0reqlen)
                        {
                          if (((priv->ctrl.type & USB_REQ_TYPE_MASK) !=
                               USB_REQ_TYPE_STANDARD) &&
                              USB_REQ_ISOUT(priv->ctrl.type))
                            {
                              /* Ready to receive the next setup packet */

                              ctrl = getreg32(CXD56_USB_OUT_EP_CONTROL(0));
                              putreg32(ctrl | USB_SNAK | USB_RRDY,
                                       CXD56_USB_OUT_EP_CONTROL(0));

                              cxd56_ep0setup(priv);
                              priv->ep0datlen = 0;
                            }
                        }
                      else
                        {
                          /* Ready to receive the next OUT packet */

                          ctrl = getreg32(CXD56_USB_OUT_EP_CONTROL(0));
                          putreg32(ctrl | USB_CNAK | USB_RRDY,
                                   CXD56_USB_OUT_EP_CONTROL(0));
                        }
                    }
                  else
                    {
                      cxd56_rxdmacomplete(privep);
                    }
                }

              if (stat & USB_INT_CDC_CLEAR)
                {
                  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_CDCCLEAR), n);
                  putreg32(USB_INT_CDC_CLEAR, CXD56_USB_OUT_EP_STATUS(n));
                }

              if (stat & USB_INT_RSS)
                {
                  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_SETFEATURE), n);
                  ctrl = getreg32(CXD56_USB_OUT_EP_CONTROL(0));
                  putreg32(USB_INT_RSS, CXD56_USB_OUT_EP_STATUS(n));
                  privep->halted = 1;
                }

              if (stat & USB_INT_RCS)
                {
                  uint32_t status;

                  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_CLEARFEATURE),
                                           n);

                  ctrl = getreg32(CXD56_USB_OUT_EP_CONTROL(n));

                  /* Make sure that want the DMA transfer stopped. */

                  /* The S bit needs to be clear by hand */

                  ctrl &= ~USB_STALL;

                  putreg32(ctrl | USB_CLOSEDESC,
                           CXD56_USB_OUT_EP_CONTROL(n));
                  do
                    {
                      status = getreg32(CXD56_USB_OUT_EP_STATUS(n));
                    }
                  while (!(status & USB_INT_CDC_CLEAR));
                  putreg32(USB_INT_CDC_CLEAR, CXD56_USB_OUT_EP_STATUS(n));

                  if (!(stat & USB_INT_MRXFIFOEMPTY))
                    {
                      /* Flush Receive FIFO and clear NAK to finish status
                       * stage.
                       */

                      putreg32(ctrl | USB_MRXFLUSH,
                               CXD56_USB_OUT_EP_CONTROL(n));
                    }

                  putreg32(ctrl | USB_CNAK, CXD56_USB_OUT_EP_CONTROL(n));
                  putreg32(USB_INT_RCS, CXD56_USB_OUT_EP_STATUS(n));
                  privep->stalled = 0;
                  privep->halted = 0;
                }

              if (stat & USB_INT_HE)
                {
                  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_RXDMAERROR), n);
                  putreg32(USB_INT_HE, CXD56_USB_OUT_EP_STATUS(n));
                }

              if (stat & USB_INT_BNA)
                {
                  usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_RXBNA), n);
                  cxd56_rdrequest(privep);
                  putreg32(USB_INT_BNA, CXD56_USB_OUT_EP_STATUS(n));
                }

              putreg32(1 << (n + 16), CXD56_USB_DEV_EP_INTR);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_usbinterrupt
 *
 * Description:
 *   Handle USB controller core interrupts
 *
 ****************************************************************************/

static int cxd56_usbinterrupt(int irq, FAR void *context, FAR void *arg)
{
  struct usb_ctrlreq_s ctrl;
  uint32_t intr;
  uint32_t status;
  int ret;

  intr = getreg32(CXD56_USB_DEV_INTR);
  putreg32(intr, CXD56_USB_DEV_INTR);

  usbtrace(TRACE_INTENTRY(CXD56_TRACEINTID_USB), intr & 0xff);

  /* Set/Clear Remove Wakeup is received by the core */

  if (intr & USB_INT_RMTWKP_STATE)
    {
      usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_RMTWKP), 0);
    }

  /* Speed enumeration is complete */

  if (intr & USB_INT_ENUM)
    {
      FAR struct cxd56_usbdev_s *priv = &g_usbdev;
      uint32_t speed;
      uint32_t config;

      /* Read established speed type (high or full) */

      speed = USB_STATUS_SPD(getreg32(CXD56_USB_DEV_STATUS));
      usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_ENUM), speed);

      /* Set established speed type to device configuration and device
       * instance.
       */

      config = getreg32(CXD56_USB_DEV_CONFIG) & ~USB_CONFIG_SPD_MASK;
      if (speed == USB_CONFIG_HS)
        {
          priv->usbdev.speed = USB_SPEED_HIGH;
          config |= USB_CONFIG_HS;
        }
      else if (speed == USB_CONFIG_FS)
        {
          priv->usbdev.speed = USB_SPEED_FULL;
          config |= USB_CONFIG_FS;
        }

      putreg32(config, CXD56_USB_DEV_CONFIG);
    }

  /* An SOF token is detected */

  if (intr & USB_INT_SOF)
    {
      usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_SOF), 0);
    }

  /* A suspend state is detected */

  if (intr & USB_INT_US)
    {
      usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_US), 0);
    }

  /* A USB Reset is detected */

  if (intr & USB_INT_UR)
    {
      int i;

      usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_UR), 0);

      for (i = 1; i < CXD56_NENDPOINTS; i++)
        {
          FAR struct cxd56_ep_s *privep =
            (FAR struct cxd56_ep_s *)&g_usbdev.eplist[i];

          cxd56_cancelrequests(privep);
        }

      cxd56_pullup(&g_usbdev.usbdev, false);
      if (g_usbdev.driver)
        {
          CLASS_DISCONNECT(g_usbdev.driver, &g_usbdev.usbdev);
        }
    }

  /* An idle state is detected */

  if (intr & USB_INT_ES)
    {
      usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_ES), 0);
    }

  /* The device has received a Set_Interface command */

  if (intr & USB_INT_SI)
    {
      usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_SI), 0);
      status = getreg32(CXD56_USB_DEV_STATUS);
      memset(&ctrl, 0, USB_SIZEOF_CTRLREQ);
      ctrl.type     = USB_REQ_RECIPIENT_INTERFACE;
      ctrl.req      = USB_REQ_SETINTERFACE;
      ctrl.value[0] = USB_STATUS_ALT(status);
      ctrl.index[0] = USB_STATUS_INTF(status);
      g_usbdev.ctrl = ctrl;
      ret = CLASS_SETUP(g_usbdev.driver, &g_usbdev.usbdev, &ctrl, NULL, 0);
      if (ret < 0)
        {
          usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_STALLEDISPATCH),
                   USB_REQ_SETINTERFACE);
          g_usbdev.stalled = 1;
        }

      putreg32(getreg32(CXD56_USB_DEV_CONTROL) | USB_CTRL_CSR_DONE,
               CXD56_USB_DEV_CONTROL);
    }

  /* The device has received a Set_Configuration command */

  if (intr & USB_INT_SC)
    {
      usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_SC), 0);
      status = getreg32(CXD56_USB_DEV_STATUS);
      memset(&ctrl, 0, USB_SIZEOF_CTRLREQ);
      ctrl.req      = USB_REQ_SETCONFIGURATION;
      ctrl.value[0] = USB_STATUS_CFG(status);
      g_usbdev.ctrl = ctrl;
      ret = CLASS_SETUP(g_usbdev.driver, &g_usbdev.usbdev, &ctrl, NULL, 0);
      if (ret < 0)
        {
          usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_STALLEDISPATCH),
                   USB_REQ_SETCONFIGURATION);
          g_usbdev.stalled = 1;
        }

      putreg32(getreg32(CXD56_USB_DEV_CONTROL) | USB_CTRL_CSR_DONE,
               CXD56_USB_DEV_CONTROL);
    }

  /* Handle each EP interrupts */

  cxd56_epinterrupt(irq, context);

  usbtrace(TRACE_INTEXIT(CXD56_TRACEINTID_USB), 0);

  return OK;
}

/****************************************************************************
 * Name: cxd56_sysinterrupt
 *
 * Description:
 *
 ****************************************************************************/

static int cxd56_sysinterrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct cxd56_usbdev_s *priv = (FAR struct cxd56_usbdev_s *)arg;
  uint32_t status;

  UNUSED(priv);

  usbtrace(TRACE_INTENTRY(CXD56_TRACEINTID_SYS), 0);

  status = getreg32(CXD56_USB_SYS_INTR);
  putreg32(status, CXD56_USB_SYS_INTR);

  if (status & USB_INT_RESUME)
    {
      usbtrace(TRACE_INTDECODE(CXD56_TRACEINTID_RESUME), 0);
    }

  usbtrace(TRACE_INTEXIT(CXD56_TRACEINTID_SYS), 0);

  return OK;
}

/****************************************************************************
 * Name: cxd56_ep0hwinitialize
 *
 * Description:
 *   Initialize endpoints.  This is logically a part of cxd56_ctrlinitialize
 *
 ****************************************************************************/

static void cxd56_ep0hwinitialize(FAR struct cxd56_usbdev_s *priv)
{
  uint32_t maxp  = g_epinfo[0].maxpacket;
  uint32_t bufsz = g_epinfo[0].bufsize / 4;
  uint32_t status;

  /* Initialize DMA descriptors */

  memset(&g_ep0setup, 0, sizeof(g_ep0setup));
  memset(&g_ep0in, 0, sizeof(g_ep0in));
  memset(&g_ep0out, 0, sizeof(g_ep0out));

  g_ep0out.buf    = (uint32_t)(uintptr_t)g_ep0outbuffer;
  g_ep0out.status = CXD56_EP0MAXPACKET | DESC_LAST;

  putreg32((uint32_t)(uintptr_t)&g_ep0setup, CXD56_USB_OUT_EP_SETUP(0));
  putreg32((uint32_t)(uintptr_t)&g_ep0in, CXD56_USB_IN_EP_DATADESC(0));
  putreg32((uint32_t)(uintptr_t)&g_ep0out, CXD56_USB_OUT_EP_DATADESC(0));

  /* Clear all interrupts */

  status = getreg32(CXD56_USB_IN_EP_STATUS(0));
  putreg32(status, CXD56_USB_IN_EP_STATUS(0));
  status = getreg32(CXD56_USB_OUT_EP_STATUS(0));
  putreg32(status, CXD56_USB_OUT_EP_STATUS(0));

  /* EP0 setup for control */

  putreg32(maxp, CXD56_USB_IN_EP_MAXPKTSIZE(0));
  putreg32(bufsz, CXD56_USB_IN_EP_BUFSIZE(0));
  putreg32(maxp | (bufsz << 16), CXD56_USB_OUT_EP_BUFSIZE(0));
  putreg32(USB_ET(USB_EP_CONTROL) | USB_F, CXD56_USB_IN_EP_CONTROL(0));
  putreg32(USB_ET(USB_EP_CONTROL) | USB_SNAK, CXD56_USB_OUT_EP_CONTROL(0));
  putreg32(maxp << 19, CXD56_USB_DEV_UDC_EP(0));
}

/****************************************************************************
 * Name: cxd56_ctrlinitialize
 *
 * Description:
 *   Initialize the CXD56 USB controller for peripheral mode operation .
 *
 ****************************************************************************/

static void cxd56_ctrlinitialize(FAR struct cxd56_usbdev_s *priv)
{
  uint32_t ctrl;
  uint32_t config;

  config = USB_CONFIG_CSR_PRG | USB_CONFIG_PI | USB_CONFIG_RWKP;
#ifdef CONFIG_USBDEV_SELFPOWERED
  config |= USB_CONFIG_SP;
#endif
  putreg32(config, CXD56_USB_DEV_CONFIG);
  ctrl = USB_CTRL_SD | USB_CTRL_RES;
  putreg32(ctrl, CXD56_USB_DEV_CONTROL);

  /* Polling wait for resumed */

  while (getreg32(CXD56_USB_DEV_STATUS) & USB_STATUS_SUSP);

  ctrl |= USB_CTRL_MODE | USB_CTRL_RDE | USB_CTRL_TDE;
  putreg32(ctrl, CXD56_USB_DEV_CONTROL);
}

/****************************************************************************
 * Name: cxd56_usbdevreset
 *
 * Description:
 *   Reset USB engine
 *
 ****************************************************************************/

static void cxd56_usbdevreset(FAR struct cxd56_usbdev_s *priv)
{
  uint32_t mask;
  int i;
  int timeout = 0;

  /* Initialize USB Device controller */

  putreg32(0, CXD56_USB_RESET);
  putreg32(1, CXD56_USB_RESET);
  putreg32(getreg32(CXD56_USB_PHY_CONFIG1) | PHY_PLLENABLE,
           CXD56_USB_PHY_CONFIG1);

  while (!(getreg32(CXD56_USB_SYS_INTR) & USB_INT_READY))
    {
      timeout++;
      if (timeout > CXD56_USBDEV_TIMEOUT)
        {
          uinfo("usb reset timeout.\n");
          break;
        }

      up_mdelay(1);
    }

  /* Workaround for recovery from reset to slow issue.
   * Wait to recover from usb reset condition,
   * until any register value can read correctly.
   */

  while (getreg32(CXD56_USB_DEV_INTR_MASK) == 0)
    {
      timeout++;
      if (timeout > CXD56_USBDEV_TIMEOUT)
        {
          uinfo("intr mask register timeout.\n");
          break;
        }

      up_mdelay(1);
    }

  putreg32(USB_INT_READY, CXD56_USB_SYS_INTR);
  putreg32(1 << 24, CXD56_USB_PJ_DEMAND); /* XXX */

  cxd56_pullup(&priv->usbdev, false);

  /* Initialize the CXD56 USB controller for DMA mode operation. */

  cxd56_ctrlinitialize(priv);

  cxd56_ep0hwinitialize(priv);

  for (i = 1; i < CXD56_NENDPOINTS; i++)
    {
      const struct cxd56_epinfo_s *info = &g_epinfo[i];
      uint32_t stat;

      if (USB_ISEPIN(info->addr))
        {
          stat = getreg32(CXD56_USB_IN_EP_STATUS(i));
          putreg32(stat, CXD56_USB_IN_EP_STATUS(i));
          putreg32(info->maxpacket, CXD56_USB_IN_EP_MAXPKTSIZE(i));
          putreg32(info->bufsize / 4, CXD56_USB_IN_EP_BUFSIZE(i));
          putreg32(USB_ET(info->attr) | USB_F, CXD56_USB_IN_EP_CONTROL(i));
        }
      else
        {
          stat = getreg32(CXD56_USB_OUT_EP_STATUS(i));
          putreg32(stat, CXD56_USB_OUT_EP_STATUS(i));
          putreg32(info->maxpacket | ((info->bufsize / 4) << 16),
                   CXD56_USB_OUT_EP_BUFSIZE(i));
          putreg32(USB_ET(info->attr) | USB_SNAK,
                   CXD56_USB_OUT_EP_CONTROL(i));
        }
    }

  /* Enable device interrupts */

  mask = getreg32(CXD56_USB_DEV_INTR_MASK);
  mask &= ~(USB_INT_RMTWKP_STATE | USB_INT_ENUM | USB_INT_UR | USB_INT_SI |
            USB_INT_SC);
  putreg32(mask, CXD56_USB_DEV_INTR_MASK);

  /* Enable EP0 IN/OUT */

  mask = getreg32(CXD56_USB_DEV_EP_INTR_MASK);
  mask &= ~(1 << 16 | 1);
  putreg32(mask, CXD56_USB_DEV_EP_INTR_MASK);
}

/****************************************************************************
 * Endpoint Methods
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_epconfigure
 *
 * Description:
 *   Configure endpoint, making it usable
 *
 * Input Parameters:
 *   ep   - the struct usbdev_ep_s instance obtained from allocep()
 *   desc - A struct usb_epdesc_s instance describing the endpoint
 *   last - true if this is the last endpoint to be configured.  Some
 *          hardware needs to take special action when all of the endpoints
 *          have been configured.
 *
 ****************************************************************************/

static int cxd56_epconfigure(FAR struct usbdev_ep_s *ep,
                             FAR const struct usb_epdesc_s *desc, bool last)
{
  FAR struct cxd56_ep_s *privep = (FAR struct cxd56_ep_s *)ep;
  int n;
  int eptype;
  uint16_t maxpacket;
  uint32_t status;
  uint32_t udc;
  uint32_t addr;
  uint32_t ctrl;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);
  DEBUGASSERT(desc->addr == ep->eplog);

  n         = privep->epphy;
  eptype    = desc->attr & USB_EP_ATTR_XFERTYPE_MASK;
  maxpacket = GETUINT16(desc->mxpacketsize);
  ep->maxpacket = maxpacket;

  status = getreg32(CXD56_USB_DEV_STATUS);

  uinfo("config: EP%d %s %d maxpacket=%d (status: %08x)\n", n,
        privep->in ? "IN" : "OUT", eptype, maxpacket, status);

  udc = n;
  udc |= privep->in ? (1 << 4) : 0;
  udc |= eptype << 5;
  udc |= USB_STATUS_CFG(status) << 7;
  udc |= USB_STATUS_INTF(status) << 11;
  udc |= USB_STATUS_ALT(status) << 15;
  udc |= maxpacket << 19;
  uinfo("UDC: %08x\n", udc);

  /* This register is write-only (why?) */

  putreg32(udc, CXD56_USB_DEV_UDC_EP(n));

  /* Write to UDC EP register takes time, so wait for the USBBusy bit */

  while (getreg32(CXD56_USB_BUSY));

  /* Off STALL bit and enable receive */

  addr = USB_ISEPIN(ep->eplog) ? CXD56_USB_IN_EP_CONTROL(privep->epphy)
                               : CXD56_USB_OUT_EP_CONTROL(privep->epphy);
  ctrl = getreg32(addr);

  putreg32((ctrl & ~USB_STALL), addr);

  privep->stalled = 0;

  /* Clear and setup DMA descriptor */

  privep->desc->status = DESC_BS_HOST_BUSY;
  privep->desc->buf    = 0;

  if (privep->in)
    {
      putreg32((uint32_t)(uintptr_t)privep->desc,
               CXD56_USB_IN_EP_DATADESC(privep->epphy));
    }
  else
    {
      putreg32((uint32_t)(uintptr_t)privep->desc,
               CXD56_USB_OUT_EP_DATADESC(privep->epphy));
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_epdisable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 ****************************************************************************/

static int cxd56_epdisable(FAR struct usbdev_ep_s *ep)
{
  FAR struct cxd56_ep_s *privep = (FAR struct cxd56_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

#endif
  usbtrace(TRACE_EPDISABLE, privep->epphy);
  uinfo("EP%d\n", ((FAR struct cxd56_ep_s *)ep)->epphy);

  /* Cancel any ongoing activity and reset the endpoint */

  flags = enter_critical_section();
  cxd56_epstall(&privep->ep, false);
  cxd56_cancelrequests(privep);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: cxd56_epallocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 ****************************************************************************/

static FAR struct usbdev_req_s *cxd56_epallocreq(FAR struct usbdev_ep_s *ep)
{
  FAR struct cxd56_req_s *privreq;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      return NULL;
    }

#endif
  usbtrace(TRACE_EPALLOCREQ, ((FAR struct cxd56_ep_s *)ep)->epphy);

  privreq = (FAR struct cxd56_req_s *)kmm_malloc(sizeof(struct cxd56_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct cxd56_req_s));
  return &privreq->req;
}

/****************************************************************************
 * Name: cxd56_epfreereq
 *
 * Description:
 *   Free an I/O request
 *
 ****************************************************************************/

static void cxd56_epfreereq(FAR struct usbdev_ep_s *ep,
                            FAR struct usbdev_req_s *req)
{
  FAR struct cxd56_req_s *privreq = (FAR struct cxd56_req_s *)req;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  usbtrace(TRACE_EPFREEREQ, ((FAR struct cxd56_ep_s *)ep)->epphy);
  kmm_free(privreq);
}

/****************************************************************************
 * Name: cxd56_epallocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static FAR void *cxd56_epallocbuffer(FAR struct usbdev_ep_s *ep,
                                     uint16_t bytes)
{
  FAR struct cxd56_ep_s *privep = (FAR struct cxd56_ep_s *)ep;

  UNUSED(privep);
  usbtrace(TRACE_EPALLOCBUFFER, privep->epphy);

  return kmm_malloc(bytes);
}
#endif

/****************************************************************************
 * Name: cxd56_epfreebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void cxd56_epfreebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf)
{
  FAR struct cxd56_ep_s *privep = (FAR struct cxd56_ep_s *)ep;

  UNUSED(privep);
  usbtrace(TRACE_EPFREEBUFFER, privep->epphy);

  kmm_free(buf);
}
#endif

/****************************************************************************
 * Name: cxd56_epsubmit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 ****************************************************************************/

static int cxd56_epsubmit(FAR struct usbdev_ep_s *ep,
                          FAR struct usbdev_req_s *req)
{
  FAR struct cxd56_req_s *privreq = (FAR struct cxd56_req_s *)req;
  FAR struct cxd56_ep_s *privep   = (FAR struct cxd56_ep_s *)ep;
  FAR struct cxd56_usbdev_s *priv;
  uint32_t ctrl;
  irqstate_t flags;
  int ret = OK;

#ifdef CONFIG_DEBUG_FEATURES
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

#endif
  usbtrace(TRACE_EPSUBMIT, privep->epphy);
  priv = privep->dev;

  if (!priv->driver || priv->usbdev.speed == USB_SPEED_UNKNOWN)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_NOTCONFIGURED), 0);
      return -ESHUTDOWN;
    }

  req->result = -EINPROGRESS;
  req->xfrd   = 0;
  flags       = enter_critical_section();

  /* If we are stalled, then drop all requests on the floor, except OUT */

  if (privep->stalled && privep->in)
    {
      cxd56_abortrequest(privep, privreq, -EBUSY);
      ret = -EBUSY;
    }

  /* Handle control requests.
   * Submit on endpoint 0 is always IN request
   */

  else if (privep->epphy == 0)
    {
      cxd56_rqenqueue(privep, privreq);

      /* SetConfiguration and SetInterface are handled by hardware,
       * USB device IP automatically returns NULL packet to host, so I drop
       * this request and indicate complete to upper driver.
       */

      if (priv->ctrl.req == USB_REQ_SETCONFIGURATION ||
          priv->ctrl.req == USB_REQ_SETINTERFACE)
        {
          /* Nothing to transfer -- exit success, with zero bytes
           * transferred
           */

          usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
          cxd56_reqcomplete(privep, OK);
        }

      if (priv->ctrl.req == USB_REQ_SETCONFIGURATION)
        {
          /* Notify attach signal.
           * max supply current is returned in response to GET_CONFIGURATION
           * from the device. host receives the response and determines the
           * supply current value.
           */

          cxd56_notify_signal(USBDEV_STATE_ATTACH, priv->power);
        }

      /* Get max supply current value from GET_CONFIGURATION response.
       * max supply current value is stored in units of 2 mA.
       */

      if (priv->ctrl.req == USB_REQ_GETDESCRIPTOR &&
          priv->ctrl.value[1] == USB_DESC_TYPE_CONFIG)
        {
          FAR struct usb_cfgdesc_s *cfgdesc;

          cfgdesc     = (FAR struct usb_cfgdesc_s *)req->buf;
          priv->power = cfgdesc->mxpower * 2;
        }

      /* If IN transaction has been requested, clear NAK bit to be able
       * to raise IN interrupt to start IN packets.
       */

      if (privep->txwait)
        {
          ctrl = getreg32(CXD56_USB_IN_EP_CONTROL(privep->epphy));
          putreg32(ctrl | USB_CNAK, CXD56_USB_IN_EP_CONTROL(privep->epphy));
        }
    }

  /* Handle IN (device-to-host) requests */

  else if (privep->in)
    {
      /* Add the new request to the request queue for the IN endpoint */

      cxd56_rqenqueue(privep, privreq);
      usbtrace(TRACE_INREQQUEUED(privep->epphy), privreq->req.len);

      /* If IN transaction has been requested, clear NAK bit to be able
       * to raise IN interrupt to start IN packets.
       */

      if (privep->txwait)
        {
          ctrl = getreg32(CXD56_USB_IN_EP_CONTROL(privep->epphy));
          putreg32(ctrl | USB_CNAK, CXD56_USB_IN_EP_CONTROL(privep->epphy));
        }
    }

  /* Handle OUT (host-to-device) requests */

  else
    {
      /* Add the new request to the request queue for the OUT endpoint */

      privep->txnullpkt = 0;
      cxd56_rqenqueue(privep, privreq);
      usbtrace(TRACE_OUTREQQUEUED(privep->epphy), privreq->req.len);

      /* This there a incoming data pending the availability of a request? */

      ret = cxd56_rdrequest(privep);
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: cxd56_epcancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 ****************************************************************************/

static int cxd56_epcancel(FAR struct usbdev_ep_s *ep,
                          FAR struct usbdev_req_s *req)
{
  FAR struct cxd56_ep_s *privep = (FAR struct cxd56_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPCANCEL, privep->epphy);

  flags = enter_critical_section();
  cxd56_cancelrequests(privep);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: cxd56_epstall
 *
 * Description:
 *   Stall or resume and endpoint
 *
 ****************************************************************************/

static int cxd56_epstall(FAR struct usbdev_ep_s *ep, bool resume)
{
  FAR struct cxd56_ep_s *privep = (FAR struct cxd56_ep_s *)ep;
  uint32_t ctrl;
  uint32_t addr;

  addr = USB_ISEPIN(ep->eplog) ? CXD56_USB_IN_EP_CONTROL(privep->epphy)
                               : CXD56_USB_OUT_EP_CONTROL(privep->epphy);

  if (resume)
    {
      usbtrace(TRACE_EPRESUME, privep->epphy);
      ctrl = getreg32(addr);
      putreg32(ctrl & ~USB_STALL, addr);
      privep->stalled = 0;
    }
  else
    {
      usbtrace(TRACE_EPSTALL, privep->epphy);
      ctrl = getreg32(addr);
      putreg32(ctrl | USB_STALL, addr);
      privep->stalled = 1;
    }

  return OK;
}

/****************************************************************************
 * Device Methods
 ****************************************************************************/

static int cxd56_allocepbuffer(FAR struct cxd56_ep_s *privep)
{
  DEBUGASSERT(!privep->desc && !privep->buffer);
  DEBUGASSERT(privep->epphy); /* Do not use for EP0 */

  privep->desc =
    (struct cxd56_data_desc_s *)kmm_malloc(sizeof(struct cxd56_data_desc_s));
  if (!privep->desc)
    {
      return -1;
    }

  privep->buffer       = NULL;
  privep->desc->status = DESC_BS_HOST_BUSY;
  privep->desc->buf    = 0;
  privep->desc->next   = 0;

  if (privep->in)
    {
      putreg32((uint32_t)(uintptr_t)privep->desc,
               CXD56_USB_IN_EP_DATADESC(privep->epphy));
    }
  else
    {
      putreg32((uint32_t)(uintptr_t)privep->desc,
               CXD56_USB_OUT_EP_DATADESC(privep->epphy));
    }

  return 0;
}

static void cxd56_freeepbuffer(FAR struct cxd56_ep_s *privep)
{
  DEBUGASSERT(privep->epphy); /* Do not use for EP0 */

  if (privep->in)
    {
      putreg32(0, CXD56_USB_IN_EP_DATADESC(privep->epphy));
    }
  else
    {
      putreg32(0, CXD56_USB_OUT_EP_DATADESC(privep->epphy));
    }

  if (privep->desc)
    {
      kmm_free(privep->desc);
      privep->desc = NULL;
    }

  if (privep->buffer)
    {
      kmm_free(privep->buffer);
      privep->buffer = NULL;
    }
}

/****************************************************************************
 * Name: cxd56_allocep
 *
 * Description:
 *   Allocate an endpoint matching the parameters
 *
 * Input Parameters:
 *   eplog  - 7-bit logical endpoint number (direction bit ignored).
 *            Zero means that any endpoint matching the other requirements
 *            will suffice.  The assigned endpoint can be found in the eplog
 *            field.
 *   in     - true: IN (device-to-host) endpoint requested
 *   eptype - Endpoint type.
 *            One of {USB_EP_ATTR_XFER_ISOC,
 *                    USB_EP_ATTR_XFER_BULK,
 *                    USB_EP_ATTR_XFER_INT}
 *
 ****************************************************************************/

static FAR struct usbdev_ep_s *cxd56_allocep(FAR struct usbdev_s *dev,
                                             uint8_t eplog, bool in,
                                             uint8_t eptype)
{
  FAR struct cxd56_usbdev_s *priv = (FAR struct cxd56_usbdev_s *)dev;
  int ndx;

  usbtrace(TRACE_DEVALLOCEP, eplog);

  /* Ignore any direction bits in the logical address */

  eplog = USB_EPNO(eplog);

  /* Check all endpoints (except EP0) */

  for (ndx = 1; ndx < CXD56_NENDPOINTS; ndx++)
    {
      /* if not used? */

      if (!(priv->avail & (1 << ndx)))
        {
          continue;
        }

      /* Does this match the endpoint number (if one was provided?) */

      if (eplog != 0 && eplog != USB_EPNO(priv->eplist[ndx].ep.eplog))
        {
          continue;
        }

      /* Does the direction match */

      if (in)
        {
          if (!USB_ISEPIN(g_epinfo[ndx].addr))
            {
              continue;
            }
        }
      else
        {
          if (!USB_ISEPOUT(g_epinfo[ndx].addr))
            {
              continue;
            }
        }

      /* Does the type match? */

      if (g_epinfo[ndx].attr == eptype)
        {
          /* Success! */

          irqstate_t flags;
          uint32_t mask;

          if (cxd56_allocepbuffer(&priv->eplist[ndx]) < 0)
            {
              continue;
            }

          flags = enter_critical_section();
          priv->avail &= ~(1 << ndx);
          mask = getreg32(CXD56_USB_DEV_EP_INTR_MASK);
          mask &= ~(1 << ndx << (in ? 0 : 16));
          putreg32(mask, CXD56_USB_DEV_EP_INTR_MASK);
          leave_critical_section(flags);
          return &priv->eplist[ndx].ep;
        }
    }

  usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_NOEP), 0);
  return NULL;
}

/****************************************************************************
 * Name: cxd56_freeep
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 ****************************************************************************/

static void cxd56_freeep(FAR struct usbdev_s *dev,
                         FAR struct usbdev_ep_s *ep)
{
  FAR struct cxd56_ep_s *privep   = (FAR struct cxd56_ep_s *)ep;
  FAR struct cxd56_usbdev_s *pdev = privep->dev;
  irqstate_t flags;

  usbtrace(TRACE_DEVFREEEP, (uint16_t)privep->epphy);

  cxd56_freeepbuffer(privep);

  flags = enter_critical_section();
  pdev->avail |= 1 << privep->epphy;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: cxd56_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 ****************************************************************************/

static int cxd56_getframe(FAR struct usbdev_s *dev)
{
  irqstate_t flags;
  int ret = 0;

  usbtrace(TRACE_DEVGETFRAME, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  /* Return the contents of the frame register.  Interrupts must be disabled
   * because the operation is not atomic.
   */

  flags = enter_critical_section();
  ret   = getreg32(CXD56_USB_DEV_STATUS) >> 18;
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: cxd56_wakeup
 *
 * Description:
 *   Tries to wake up the host connected to this device
 *
 ****************************************************************************/

static int cxd56_wakeup(FAR struct usbdev_s *dev)
{
  irqstate_t flags;

  usbtrace(TRACE_DEVWAKEUP, 0);

  flags = enter_critical_section();
  putreg32(getreg32(CXD56_USB_DEV_CONTROL) | 1, CXD56_USB_DEV_CONTROL);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: cxd56_selfpowered
 *
 * Description:
 *   Sets/clears the device selfpowered feature
 *
 ****************************************************************************/

static int cxd56_selfpowered(FAR struct usbdev_s *dev, bool selfpowered)
{
  FAR struct cxd56_usbdev_s *priv = &g_usbdev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/****************************************************************************
 * Name: cxd56_pullup
 *
 * Description:
 *    Software-controlled connect to/disconnect from USB host
 *
 ****************************************************************************/

static int cxd56_pullup(FAR struct usbdev_s *dev, bool enable)
{
  uint32_t ctrl;
  uint32_t ep;

  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  ctrl = getreg32(CXD56_USB_DEV_CONTROL);
  ep   = getreg32(CXD56_USB_OUT_EP_CONTROL(0));

  if (enable)
    {
      ctrl &= ~(USB_CTRL_SD | USB_CTRL_RES);
      ep |= USB_RRDY;
    }
  else
    {
      ctrl |= USB_CTRL_SD | USB_CTRL_RES;
      ep &= USB_RRDY;
    }

  putreg32(ctrl, CXD56_USB_DEV_CONTROL);
  putreg32(ep, CXD56_USB_OUT_EP_CONTROL(0));

  return OK;
}

/****************************************************************************
 * Name: cxd56_epinitialize
 *
 * Description:
 *   Initialize all of the endpoint data
 *
 ****************************************************************************/

static void cxd56_epinitialize(FAR struct cxd56_usbdev_s *priv)
{
  int i;

  /* Initialize the device state structure */

  priv->usbdev.ops = &g_devops;

  /* Set up the standard stuff */

  memset(&priv->eplist[0], 0, sizeof(struct cxd56_ep_s));
  priv->eplist[0].ep.ops = &g_epops;
  priv->eplist[0].dev    = priv;

  /* The index, i, is the physical endpoint address;  Map this
   * to a logical endpoint address usable by the class driver.
   */

  priv->eplist[0].epphy    = 0;
  priv->eplist[0].ep.eplog = g_epinfo[0].addr;

  /* Setup the endpoint-specific stuff */

  priv->eplist[0].ep.maxpacket = g_epinfo[0].maxpacket;

  /* Expose only the standard EP0 */

  priv->usbdev.ep0 = &priv->eplist[0].ep;

  /* Initilialize USB hardware */

  for (i = 1; i < CXD56_NENDPOINTS; i++)
    {
      FAR const struct cxd56_epinfo_s *info = &g_epinfo[i];
      FAR struct cxd56_ep_s *privep;

      /* Set up the standard stuff */

      privep = &priv->eplist[i];
      memset(privep, 0, sizeof(struct cxd56_ep_s));
      privep->ep.ops = &g_epops;
      privep->dev    = priv;
      privep->desc   = NULL;
      privep->buffer = NULL;

      /* The index, i, is the physical endpoint address;  Map this
       * to a logical endpoint address usable by the class driver.
       */

      privep->epphy    = i;
      privep->ep.eplog = info->addr;

      /* Setup the endpoint-specific stuff */

      privep->ep.maxpacket = g_epinfo[i].maxpacket;

      if (USB_ISEPIN(info->addr))
        {
          privep->in = 1;
        }
    }
}

/****************************************************************************
 * Name: cxd56_usbhwuninit
 ****************************************************************************/

static void cxd56_usbhwuninit(void)
{
  /* Un-initilialize USB hardware */

  putreg32(getreg32(CXD56_USB_PHY_CONFIG1) & ~PHY_PLLENABLE,
           CXD56_USB_PHY_CONFIG1);
  putreg32(getreg32(CXD56_USB_PJ_DEMAND) & ~(1 << 24), CXD56_USB_PJ_DEMAND);

  /* USB Device Reset */

  putreg32(0, CXD56_USB_RESET);
}

/****************************************************************************
 * Name: cxd56_vbusinterrupt
 ****************************************************************************/

static int cxd56_vbusinterrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct cxd56_usbdev_s *priv = (FAR struct cxd56_usbdev_s *)arg;

  cxd56_cableconnected(true);

  usbtrace(TRACE_INTENTRY(CXD56_TRACEINTID_VBUS), 0);
  uinfo("irq=%d context=%08x\n", irq, context);

  /* Toggle vbus interrupts */

  up_disable_irq(CXD56_IRQ_USB_VBUS);
  up_enable_irq(CXD56_IRQ_USB_VBUSN);

  /* Enable interrupts */

  up_enable_irq(CXD56_IRQ_USB_SYS);
  up_enable_irq(CXD56_IRQ_USB_INT);

  /* reconstruct Endpoints and restart Configuration */

  if (priv->driver)
    {
      cxd56_usbreset(priv);
    }

  /* Notify attach signal.
   * if class driver not binded, can't get supply curret value.
   */

  if (!priv->driver)
    {
      cxd56_notify_signal(USBDEV_STATE_ATTACH, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_vbusninterrupt
 ****************************************************************************/

static int cxd56_vbusninterrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct cxd56_usbdev_s *priv = (FAR struct cxd56_usbdev_s *)arg;
  FAR struct cxd56_ep_s *privep;
  int i;

  cxd56_cableconnected(false);

  usbtrace(TRACE_INTENTRY(CXD56_TRACEINTID_VBUSN), 0);

  uinfo("irq=%d context=%08x\n", irq, context);

  /* Toggle vbus interrupts */

  up_disable_irq(CXD56_IRQ_USB_VBUSN);
  up_enable_irq(CXD56_IRQ_USB_VBUS);

  /* Disconnect device */

  for (i = 1; i < CXD56_NENDPOINTS; i++)
    {
      privep = (FAR struct cxd56_ep_s *)&priv->eplist[i];

      cxd56_epstall(&privep->ep, false);
      cxd56_cancelrequests(privep);
    }

  if (g_usbdev.driver)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }

  /* Disable USB_INT interrupt */

  up_disable_irq(CXD56_IRQ_USB_INT);
  up_disable_irq(CXD56_IRQ_USB_SYS);

  /* Notify detach signal */

  priv->power = 0;
  cxd56_notify_signal(USBDEV_STATE_DETACH, priv->power);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_usbinitialize
 *
 * Description:
 *   Initialize USB hardware
 *
 ****************************************************************************/

void arm_usbinitialize(void)
{
  usbtrace(TRACE_DEVINIT, 0);

  /* Enable USB clock */

  cxd56_usb_clock_enable();

  if (irq_attach(CXD56_IRQ_USB_SYS, cxd56_sysinterrupt, &g_usbdev) != 0)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_ATTACHIRQREG), 0);
      goto errout;
    }

  if (irq_attach(CXD56_IRQ_USB_INT, cxd56_usbinterrupt, &g_usbdev) != 0)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_COREIRQREG), 0);
      goto errout;
    }

  if (irq_attach(CXD56_IRQ_USB_VBUS, cxd56_vbusinterrupt, &g_usbdev) != 0)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_VBUSIRQREG), 0);
      goto errout;
    }

  if (irq_attach(CXD56_IRQ_USB_VBUSN, cxd56_vbusninterrupt, &g_usbdev) != 0)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_VBUSNIRQREG), 0);
      goto errout;
    }

  /* Initialize driver instance */

  memset(&g_usbdev, 0, sizeof(struct cxd56_usbdev_s));
  g_usbdev.avail = 0xff;

  /* Initialize endpoint data */

  cxd56_epinitialize(&g_usbdev);

  /* Enable interrupts */

  up_enable_irq(CXD56_IRQ_USB_VBUS);

  return;

errout:
  arm_usbuninitialize();
}

/****************************************************************************
 * Name: arm_usbuninitialize
 ****************************************************************************/

void arm_usbuninitialize(void)
{
  FAR struct cxd56_usbdev_s *priv = &g_usbdev;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNINIT, 0);
  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  flags = enter_critical_section();
  cxd56_pullup(&priv->usbdev, false);
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable and detach IRQs */

  up_disable_irq(CXD56_IRQ_USB_INT);
  up_disable_irq(CXD56_IRQ_USB_SYS);
  up_disable_irq(CXD56_IRQ_USB_VBUS);
  up_disable_irq(CXD56_IRQ_USB_VBUSN);

  irq_detach(CXD56_IRQ_USB_INT);
  irq_detach(CXD56_IRQ_USB_SYS);
  irq_detach(CXD56_IRQ_USB_VBUS);
  irq_detach(CXD56_IRQ_USB_VBUSN);

  cxd56_usb_clock_disable();
  leave_critical_section(flags);

  /* Clear signal */

  priv->signo = 0;
  priv->pid   = 0;
}

/****************************************************************************
 * Name: usbdevclass_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method
 *   will be called to bind it to a USB device driver.
 *
 ****************************************************************************/

int usbdev_register(FAR struct usbdevclass_driver_s *driver)
{
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* Take freqlock to keep clock faster */

  up_pm_acquire_freqlock(&g_hv_lock);
  up_pm_acquire_wakelock(&g_wake_lock);

  /* Hook up the driver */

  g_usbdev.driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &g_usbdev.usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_BINDFAILED), (uint16_t)-ret);
      g_usbdev.driver = NULL;
      return ret;
    }

  /* Enable interrupts */

  up_enable_irq(CXD56_IRQ_USB_VBUS);

  return OK;
}

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver.If the USB device is connected to a USB
 *   host, it will first disconnect().  The driver is also requested to
 *   unbind() and clean up any device state, before this procedure finally
 *    returns.
 *
 ****************************************************************************/

int usbdev_unregister(FAR struct usbdevclass_driver_s *driver)
{
  FAR struct cxd56_usbdev_s *priv = &g_usbdev;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (driver != g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(CXD56_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &g_usbdev.usbdev);

  flags = enter_critical_section();

  /* Disable IRQs */

  up_disable_irq(CXD56_IRQ_USB_INT);
  up_disable_irq(CXD56_IRQ_USB_SYS);

  /* Disconnect device */

  cxd56_pullup(&priv->usbdev, false);
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Unhook the driver */

  g_usbdev.driver = NULL;

  /* Un-initialize USB hardware */

  cxd56_usbhwuninit();

  leave_critical_section(flags);

  up_pm_release_freqlock(&g_hv_lock);
  up_pm_release_wakelock(&g_wake_lock);

  return OK;
}

/****************************************************************************
 * Name: cxd56_usbreset
 *
 * Description:
 *   Reinitialize the endpoint and restore the EP configuration
 *   before disconnecting the host. Then start the Configuration again.
 *
 ****************************************************************************/

static void cxd56_usbreset(FAR struct cxd56_usbdev_s *priv)
{
  uint32_t mask;
  int i;

  /* USB reset assert */

  cxd56_usbdevreset(priv);

  /* Check all endpoints (except EP0) */

  for (i = 1; i < CXD56_NENDPOINTS; i++)
    {
      /* skip unused EP */

      if (priv->avail & (1 << i))
        {
          continue;
        }

      mask = getreg32(CXD56_USB_DEV_EP_INTR_MASK);
      mask &= ~(1 << i << (priv->eplist[i].in ? 0 : 16));
      putreg32(mask, CXD56_USB_DEV_EP_INTR_MASK);

      /* DMA descripter setting */

      priv->eplist[i].buffer       = NULL;
      priv->eplist[i].desc->status = DESC_BS_HOST_BUSY;
      priv->eplist[i].desc->buf    = 0;
      priv->eplist[i].desc->next   = 0;

      if (priv->eplist[i].in)
        {
          putreg32((uint32_t)(uintptr_t)priv->eplist[i].desc,
                   CXD56_USB_IN_EP_DATADESC(priv->eplist[i].epphy));
        }
      else
        {
          putreg32((uint32_t)(uintptr_t)priv->eplist[i].desc,
                   CXD56_USB_OUT_EP_DATADESC(priv->eplist[i].epphy));
        }

      /* resume EP stall */

      cxd56_epstall(&priv->eplist[i].ep, true);
    }

  cxd56_pullup(&priv->usbdev, true);
}

/****************************************************************************
 * Name: cxd56_usbdev_setsigno
 ****************************************************************************/

int cxd56_usbdev_setsigno(int signo)
{
  FAR struct cxd56_usbdev_s *priv = &g_usbdev;

  uinfo("signo = %d\n", signo);

  priv->signo = signo;
  priv->pid   = getpid();

  return OK;
}

/****************************************************************************
 * Name: cxd56_notify_signal
 *
 * Description:
 *   Notify the application of USB attach/detach event
 *
 ****************************************************************************/

static void cxd56_notify_signal(uint16_t state, uint16_t power)
{
  FAR struct cxd56_usbdev_s *priv = &g_usbdev;

  if (priv->signo > 0)
    {
      union sigval value;
      value.sival_int = state << 16 | power;
      sigqueue(priv->pid, priv->signo, value);
    }
}

#ifdef CONFIG_FS_PROCFS

/****************************************************************************
 * Name: cxd56_usbdev_open
 ****************************************************************************/

static int cxd56_usbdev_open(FAR struct file *filep, FAR const char *relpath,
                             int oflags, mode_t mode)
{
  FAR struct cxd56_usbdev_file_s *priv;

  uinfo("Open '%s'\n", relpath);

  /* PROCFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   *
   * REVISIT:  Write-able proc files could be quite useful.
   */

  if (((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0))
    {
      uerr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* Allocate the open file structure */

  priv = (FAR struct cxd56_usbdev_file_s *)kmm_zalloc(
    sizeof(struct cxd56_usbdev_file_s));
  if (!priv)
    {
      uerr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* Save the open file structure as the open-specific state in
   * filep->f_priv.
   */

  filep->f_priv = (FAR void *)priv;
  return OK;
}

/****************************************************************************
 * Name: modprocfs_close
 ****************************************************************************/

static int cxd56_usbdev_close(FAR struct file *filep)
{
  FAR struct cxd56_usbdev_file_s *priv;

  /* Recover our private data from the struct file instance */

  priv = (FAR struct cxd56_usbdev_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  /* Release the file attributes structure */

  kmm_free(priv);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: cxd56_usbdev_read
 ****************************************************************************/

static ssize_t cxd56_usbdev_read(FAR struct file *filep, FAR char *buffer,
                                 size_t buflen)
{
  FAR struct cxd56_usbdev_file_s *attr;
  FAR struct cxd56_usbdev_s *priv = &g_usbdev;
  off_t offset;
  int ret;

  uinfo("buffer=%p buflen=%lu\n", buffer, (unsigned long)buflen);

  /* Recover our private data from the struct file instance */

  attr = (FAR struct cxd56_usbdev_file_s *)filep->f_priv;
  DEBUGASSERT(attr);

  /* Traverse all installed modules */

  attr->linesize = snprintf(attr->line, CXD56_USBDEV_LINELEN, "%-12s%d mA\n",
                            "Power:", priv->power);

  /* Transfer the system up time to user receive buffer */

  offset = filep->f_pos;
  ret    = procfs_memcpy(attr->line, attr->linesize,
                         buffer, buflen, &offset);

  /* Update the file offset */

  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  return ret;
}

/****************************************************************************
 * Name: cxd56_usbdev_dup
 ****************************************************************************/

static int cxd56_usbdev_dup(FAR const struct file *oldp,
                            FAR struct file *newp)
{
  FAR struct cxd56_usbdev_file_s *oldattr;
  FAR struct cxd56_usbdev_file_s *newattr;

  uinfo("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldattr = (FAR struct cxd56_usbdev_file_s *)oldp->f_priv;
  DEBUGASSERT(oldattr);

  /* Allocate a new container to hold the task and attribute selection */

  newattr = (FAR struct cxd56_usbdev_file_s *)kmm_malloc(
    sizeof(struct cxd56_usbdev_file_s));
  if (!newattr)
    {
      uerr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* The copy the file attributes from the old attributes to the new */

  memcpy(newattr, oldattr, sizeof(struct cxd56_usbdev_file_s));

  /* Save the new attributes in the new file structure */

  newp->f_priv = (FAR void *)newattr;
  return OK;
}

/****************************************************************************
 * Name: cxd56_usbdev_stat
 ****************************************************************************/

static int cxd56_usbdev_stat(FAR const char *relpath, FAR struct stat *buf)
{
  buf->st_mode    = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;
  return OK;
}

/****************************************************************************
 * Name: cxd56_usbdev_procfs_register
 *
 * Description:
 *   Register the usbdev procfs file system entry
 *
 ****************************************************************************/

#ifdef CONFIG_FS_PROCFS_REGISTER
int cxd56_usbdev_procfs_register(void)
{
  return procfs_register(&g_procfs_usbdev);
}
#endif

#endif

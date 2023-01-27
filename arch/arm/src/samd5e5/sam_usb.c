/****************************************************************************
 * arch/arm/src/samd5e5/sam_usb.c
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
 * USB core features:
 *
 * - Compatible with the USB 2.1 specification
 * - USB Embedded Host and Device mode
 * - Supports full (12Mbit/s) and low (1.5Mbit/s) speed communication
 * - Supports Link Power Management (LPM-L1) protocol
 * - On-chip transceivers with built-in pull-ups and pull-downs
 * - On-Chip USB serial resistors
 * - 1kHz SOF clock available on external pin
 *
 *   Device mode
 *   - Supports 8 IN endpoints and 8 OUT endpoints
 *   – No endpoint size limitations
 *   – Built-in DMA with multi-packet and dual bank for all endpoints
 *   – Supports feedback endpoint
 *   – Supports crystal less clock
 *
 *   Host mode
 *   - Supports 8 physical pipes
 *   – No pipe size limitations
 *   – Supports multiplexed virtual pipe on one physical pipe to allow an
 *     unlimited USB tree
 *   – Built-in DMA with multi-packet support and dual bank for all pipes
 *   – Supports feedback endpoint
 *   – Supports the USB 2.0 Phase-locked SOFs feature
 *
 ****************************************************************************/

/****************************************************************************
 * WIP NOTES:
 *
 * DS 38.1 (859)
 * To maximize throughput, an endpoint can be configured for ping-pong
 * operation.
 * When this is done the input and output endpoint with the same address are
 * used in the same direction. The CPU or DMA Controller can then read/write
 * one data buffer while the USB module writes/reads from the other buffer.
 * This gives double buffered communication.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_devaddr.h>
#include <nuttx/usb/usbhost_trace.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/sam_pinmap.h"
#include "hardware/sam_fuses.h"
#include "sam_gclk.h"
#include "sam_port.h"
#include "sam_periphclks.h"
#include "sam_usb.h"
#include "sam_usbhost.h"

#if defined(CONFIG_SAMD5E5_USB)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_USB_EP0_DEFSIZE
#  define CONFIG_USB_EP0_DEFSIZE 8
#endif

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#  define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG_USB
#  undef CONFIG_SAMD5E5_USB_REGDEBUG
#endif

/* Driver Definitions *******************************************************/

#define EP0                 (0)
#define SAM_EPSET_ALL       (0xff)    /* All endpoints */
#define SAM_EPSET_NOTEP0    (0xfe)    /* All endpoints except EP0 */
#define SAM_EP_BIT(ep)      (1 << (ep))
#define SAM_EP0_MAXPACKET   (CONFIG_USBDEV_EP0_MAXSIZE) /* EP0 Max. packet size */
#define SAM_MAX_MULTIPACKET_SIZE  (0x3fff)
#define SAM_RETRY_COUNT     3   /* Number of ctrl transfer retries */

/* Delays */

#define SAM_SETUP_DELAY         SEC2TICK(5) /* 5 seconds in system ticks */
#define SAM_DATANAK_DELAY       SEC2TICK(5) /* 5 seconds in system ticks */
#define USB_CTRL_DPKT_TIMEOUT (500)         /* Timeout between control data packets : 500ms */
#define USB_CTRL_STAT_TIMEOUT (50)          /* Timeout of status packet : 50ms */

/* Maximum size of a descriptor */

#ifndef CONFIG_SAM_DESCSIZE
#  define CONFIG_SAM_DESCSIZE 128
#endif

/* Request queue operations *************************************************/

#define sam_rqempty(q)      ((q)->head == NULL)
#define sam_rqpeek(q)       ((q)->head)

/* USB trace ****************************************************************/

/* Trace error codes */

#define SAM_TRACEERR_ALLOCFAIL            0x0001
#define SAM_TRACEERR_BADCLEARFEATURE      0x0002
#define SAM_TRACEERR_BADDEVGETSTATUS      0x0003
#define SAM_TRACEERR_BADEPGETSTATUS       0x0004
#define SAM_TRACEERR_BADEOBSTATE          0x0005
#define SAM_TRACEERR_BADEPNO              0x0006
#define SAM_TRACEERR_BADEPTYPE            0x0007
#define SAM_TRACEERR_BADGETCONFIG         0x0008
#define SAM_TRACEERR_BADGETSETDESC        0x0009
#define SAM_TRACEERR_BADGETSTATUS         0x000a
#define SAM_TRACEERR_BADSETADDRESS        0x000b
#define SAM_TRACEERR_BADSETCONFIG         0x000c
#define SAM_TRACEERR_BADSETFEATURE        0x000d
#define SAM_TRACEERR_BINDFAILED           0x000e
#define SAM_TRACEERR_DISPATCHSTALL        0x000f
#define SAM_TRACEERR_DRIVER               0x0010
#define SAM_TRACEERR_DRIVERREGISTERED     0x0011
#define SAM_TRACEERR_EP0SETUPOUTSIZE      0x0012
#define SAM_TRACEERR_EP0SETUPSTALLED      0x0013
#define SAM_TRACEERR_EPOUTNULLPACKET      0x0014
#define SAM_TRACEERR_EPRESERVE            0x0015
#define SAM_TRACEERR_INVALIDCTRLREQ       0x0016
#define SAM_TRACEERR_INVALIDPARMS         0x0017
#define SAM_TRACEERR_IRQREGISTRATION      0x0018
#define SAM_TRACEERR_NOTCONFIGURED        0x0019
#define SAM_TRACEERR_REQABORTED           0x001a
#define SAM_TRACEERR_RXDATABKERR          0x001b
#define SAM_TRACEERR_TXCOMPERR            0x001c
#define SAM_TRACEERR_UNSUPPEPTYPE         0x001d

/* Trace interrupt codes */

#define SAM_TRACEINTID_INTERRUPT          0x0001
#define SAM_TRACEINTID_PENDING            0x0002
#define SAM_TRACEINTID_PENDING_EP         0x0003
#define SAM_TRACEINTID_SUSPEND            0x0004
#define SAM_TRACEINTID_SOF                0x0005
#define SAM_TRACEINTID_EORST              0x0006
#define SAM_TRACEINTID_WAKEUP             0x0007
#define SAM_TRACEINTID_EORSM              0x0008
#define SAM_TRACEINTID_UPRSM              0x0009
#define SAM_TRACEINTID_RAMACER            0x000a
#define SAM_TRACEINTID_LPMNYET            0x000b
#define SAM_TRACEINTID_LPMSUSP            0x000c
#define SAM_TRACEINTID_EPNO               0x000d
#define SAM_TRACEINTID_EPINTFLAGS         0x000e
#define SAM_TRACEINTID_EPTRCPT0           0x000f
#define SAM_TRACEINTID_EPTRCPT1           0x0010
#define SAM_TRACEINTID_EPTRFAIL0          0x0011
#define SAM_TRACEINTID_EPTRFAIL1          0x0012
#define SAM_TRACEINTID_EPRXSTP            0x0013
#define SAM_TRACEINTID_EPSTALL0           0x0014
#define SAM_TRACEINTID_EPSTALL1           0x0015
#define SAM_TRACEINTID_EPINQEMPTY         0x0016
#define SAM_TRACEINTID_EPOUTQEMPTY        0x0017
#define SAM_TRACEINTID_EP0SETUPOUT        0x0018
#define SAM_TRACEINTID_EP0SETUPIN         0x0019
#define SAM_TRACEINTID_EP0SETUPSETADDRESS 0x001a
#define SAM_TRACEINTID_NOSTDREQ           0x001b
#define SAM_TRACEINTID_GETSTATUS          0x001c
#define SAM_TRACEINTID_DEVGETSTATUS       0x001d
#define SAM_TRACEINTID_IFGETSTATUS        0x001e
#define SAM_TRACEINTID_CLEARFEATURE       0x001f
#define SAM_TRACEINTID_SETFEATURE         0x0020
#define SAM_TRACEINTID_GETSETDESC         0x0021
#define SAM_TRACEINTID_GETCONFIG          0x0022
#define SAM_TRACEINTID_SETCONFIG          0x0023
#define SAM_TRACEINTID_GETSETIF           0x0024
#define SAM_TRACEINTID_SYNCHFRAME         0x0025
#define SAM_TRACEINTID_DISPATCH           0x0026
#define SAM_TRACEINTID_ADDRESSED          0x0027
#define SAM_TRACEINTID_EPCONF             0x0028
#define SAM_TRACEINTID_EPINTEN            0x0029
#define SAM_TRACEINTID_EP0WRSTATUS        0x002a
#define SAM_TRACEINTID_EPTRCPT0_LEN       0x002b
#define SAM_TRACEINTID_PENDING_PIPE       0x002c
#define SAM_TRACEINTID_PIPENO             0x002d

/* Ever-present MIN and MAX macros */

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

/* Byte ordering in host-based values */

#ifdef CONFIG_ENDIAN_BIG
#  define LSB 1
#  define MSB 0
#else
#  define LSB 0
#  define MSB 1
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

#ifdef CONFIG_USBDEV

/* State of an endpoint */

enum sam_epstate_e
{
  USB_EPSTATE_DISABLED = 0, /* Endpoint is disabled */
  USB_EPSTATE_STALLED,      /* Endpoint is stalled */
  USB_EPSTATE_IDLE,         /* Endpoint is idle (i.e. ready for transmission) */
  USB_EPSTATE_SENDING,      /* Endpoint is sending data */
  USB_EPSTATE_RXSTOPPED,    /* OUT endpoint is stopped waiting for a read request */
                            /* --- Endpoint 0 Only --- */
  USB_EPSTATE_EP0DATAOUT,   /* Endpoint 0 is receiving SETUP OUT data */
  USB_EPSTATE_EP0STATUSIN,  /* Endpoint 0 is sending SETUP status */
  USB_EPSTATE_EP0ADDRESS    /* Address change is pending completion of status */
};

/* The overall state of the device */

enum sam_devstate_e
{
  USB_DEVSTATE_SUSPENDED = 0, /* The device is currently suspended */
  USB_DEVSTATE_POWERED,       /* Host is providing +5V through the USB cable */
  USB_DEVSTATE_DEFAULT,       /* Device has been reset */
  USB_DEVSTATE_ADDRESSED,     /* The device has been given an address on the bus */
  USB_DEVSTATE_CONFIGURED     /* A valid configuration has been selected. */
};

/* The result of EP0 SETUP processing */

enum sam_ep0setup_e
{
  USB_EP0SETUP_SUCCESS = 0,   /* The SETUP was handle without incident */
  USB_EP0SETUP_DISPATCHED,    /* The SETUP was forwarded to the class driver */
  USB_EP0SETUP_ADDRESS,       /* A new device address is pending */
  USB_EP0SETUP_STALL          /* An error occurred */
};

union wb_u
{
  uint16_t w;
  uint8_t  b[2];
};

/* A container for a request so that the request may be retained in a list */

struct sam_req_s
{
  struct usbdev_req_s  req;          /* Standard USB request */
  struct sam_req_s    *flink;        /* Supports a singly linked list */
  uint16_t             inflight;     /* Number of TX bytes tansmitting or
                                      * number of RX bytes we are waiting */
};

/* The head of a queue of requests */

struct sam_rqhead_s
{
  struct sam_req_s    *head;         /* Requests are added to the head of the list */
  struct sam_req_s    *tail;         /* Requests are removed from the tail of the list */
};

/* This is the internal representation of an endpoint */

struct sam_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct sam_ep_s.
   */

  struct usbdev_ep_s   ep;           /* Standard endpoint structure */

  /* SAMD5E5-specific fields */

  struct sam_usbdev_s *dev;          /* Reference to private driver data */
  struct sam_rqhead_s  reqq;         /* Read/write request queue */
  struct sam_rqhead_s  pendq;        /* Write requests pending stall sent */
  struct usbdev_epdesc_s *descb[2];  /* Pointers to this endpoints descriptors */
  volatile uint8_t     epstate;      /* State of the endpoint (see enum sam_epstate_e) */
  uint8_t              stalled:1;    /* true: Endpoint is stalled */
  uint8_t              pending:1;    /* true: IN Endpoint stall is pending */
  uint8_t              halted:1;     /* true: Endpoint feature halted */
  uint8_t              zlpsent:1;    /* Zero length packet has been sent */
  uint8_t              txbusy:1;     /* Write request queue is busy (recursion avoidance kludge) */
  uint8_t              rxactive:1;   /* read request is active (for top of queue) */
};

struct sam_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to struct sam_usbdev_s.
   */

  struct usbdev_s usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* USB-specific fields */

  struct usb_ctrlreq_s ctrl;          /* Last EP0 request */
  uint8_t              devstate;      /* State of the device (see enum sam_devstate_e) */
  uint8_t              prevstate;     /* Previous state of the device before SUSPEND */
  uint8_t              devaddr;       /* Assigned device address */
  uint8_t              selfpowered:1; /* 1: Device is self powered */
  uint16_t             epavail;       /* Bitset of available endpoints */

  /* The endpoint list */

  aligned_data(4) struct sam_ep_s eplist[SAM_USB_NENDPOINTS];

  /* Endpoint descriptors 2 banks for each endpoint */

  aligned_data(4)
  struct usbdev_epdesc_s ep_descriptors[SAM_USB_NENDPOINTS *
                                        SAM_USB_NBANKS()];

  /* EP0 data buffer.  For data that is included in an EP0 SETUP OUT
   * transaction.  In this case, no request is in place from the class
   * driver and the incoming data is caught in this buffer.  The size
   * of valid data in the buffer is given by ctrlreg.len[].  For the
   * case of EP0 SETUP IN transaction, the normal request mechanism is
   * used and the class driver provides the buffering.
   */

  aligned_data(4) uint8_t ep0out[SAM_EP0_MAXPACKET];
};
#endif

#ifdef CONFIG_USBHOST
/* The overall state of the device */

enum sam_hoststate_e
{
  USB_HOSTSTATE_SUSPENDED = 0, /* The device is currently suspended */
  USB_HOSTSTATE_POWERED,       /* Host is providing +5V through the USB cable */
  USB_HOSTSTATE_DEFAULT,       /* Device has been reset */
  USB_HOSTSTATE_ADDRESSED,     /* The device has been given an address on the bus */
  USB_HOSTSTATE_CONFIGURED     /* A valid configuration has been selected. */
};

/**
 * @brief      USB HCD pipe states
 */

enum usb_h_pipe_state
{
  USB_H_PIPE_S_FREE = 0x00,  /** Pipe is free to allocate */
  USB_H_PIPE_S_CFG = 0x01,   /** Pipe is in configuration */
  USB_H_PIPE_S_IDLE = 0x02,  /** Pipe is allocated and idle */
  USB_H_PIPE_S_SETUP = 0x03, /** Pipe in control setup stage */
  USB_H_PIPE_S_DATI = 0x05,  /** Pipe in data IN stage */
  USB_H_PIPE_S_DATO = 0x06,  /** Pipe in data OUT stage */
  USB_H_PIPE_S_ZLPI = 0x07,  /** Pipe in data IN ZLP stage */
  USB_H_PIPE_S_ZLPO = 0x08,  /** Pipe in data OUT ZLP stage */
  USB_H_PIPE_S_STATI = 0x09, /** Pipe in control status IN stage */
  USB_H_PIPE_S_STATO = 0x0a, /** Pipe in control status OUT stage */
  USB_H_PIPE_S_TAKEN = 0x10  /** Taken by physical pipe (in process) */
};

/**
 * @brief      USB HCD status code
 */

enum usb_h_status
{
  USB_H_OK = 0,             /** OK */
  USB_H_BUSY = -4,          /** Busy */
  USB_H_DENIED = -17,       /** Denied */
  USB_H_TIMEOUT = -8,       /** Timeout */
  USB_H_ABORT = -3,         /** Abort */
  USB_H_STALL = -25,        /** Stall protocol */
  USB_H_RESET = -7,         /** Transfer reset by pipe re-configure */
  USB_H_ERR_ARG = -13,      /** Argument error */
  USB_H_ERR_UNSP_OP = -27,  /** Operation not supported */
  USB_H_ERR_NO_RSC = -28,   /** No resource */
  USB_H_ERR_NOT_INIT = -20, /** Not initialized */
  USB_H_ERR = -6            /** Some general error */
};

/* The following enumeration represents the various states of the USB host
 * state machine (for debug purposes only)
 */

enum sam_smstate_e
{
  SMSTATE_DETACHED = 0,  /* Not attached to a device */
  SMSTATE_ATTACHED,      /* Attached to a device */
  SMSTATE_ENUM,          /* Attached, enumerating */
  SMSTATE_CLASS_BOUND,   /* Enumeration complete, class bound */
};

/* This enumeration provides the reason for the channel halt. */

enum sam_chreason_e
{
  CHREASON_IDLE = 0,     /* Inactive (initial state) */
  CHREASON_FREED,        /* Channel is no longer in use */
  CHREASON_XFRC,         /* Transfer complete */
  CHREASON_NAK,          /* NAK received */
  CHREASON_NYET,         /* NotYet received */
  CHREASON_STALL,        /* Endpoint stalled */
  CHREASON_TXERR,        /* Transfer error received */
  CHREASON_DTERR,        /* Data toggle error received */
  CHREASON_FRMOR,        /* Frame overrun */
  CHREASON_CANCELLED     /* Transfer cancelled */
};

/**
 * @brief      Transfer descriptor for control transfer
 *
 * Timing in USB 2.0 spec.:
 * - 9.2.6.1 : USB sets an upper limit of 5 seconds as the upper
 *    limit for any command to be processed.
 * - 9.2.6.3 : if a device receives a SetAddress() request,
 *    the device must be able to complete processing
 *    of the request and be able to
 *    successfully complete the Status stage of the request within
 *    50 ms.
 *    After successful completion of the Status stage, the device is
 *    allowed a SetAddress() recovery interval of 2 ms. At the end of
 *    this interval, the device must be able to accept Setup packets
 *    addressed to the new address.
 * - 9.2.6.4 : For standard device requests that require no Data stage,
 *    must be able to complete the request and be able to successfully
 *    complete the Status stage of the request within 50 ms of receipt
 *    of the request. This limitation applies to requests to the
 *    device, interface, or endpoint. For standard device requests
 *    that require data stage transfer to the host, the
 *    device must be able to return the first data packet
 *    to the host within 500 ms of receipt of the request. For
 *    subsequent data packets, if any, the device must be able to
 *    return them within 500 ms of successful completion of the
 *    transmission of the previous packet. The device must then be
 *    able to successfully complete the status stage
 *    within 50 ms after returning the last data packet.
 *    For standard device requests that require a data stage transfer
 *    to the device, the 5-second limit applies.
 * - 9.2.6.5 : Unless specifically exempted in the class document, all
 *    class-specific requests must meet the timing limitations for
 *    standard device requests.
 *
 * Conclusion:
 * 1. Whole request with data: 5 seconds
 * 2. Whole request without data: 50 ms
 * 3. Data packets: 500 ms
 */

struct usb_h_ctrl_xfer
{
  uint8_t *data;        /* Pointer to transfer data */
  uint8_t *setup;       /* Pointer to setup packet */
  uint16_t size;        /* Expected transfer size */
  uint16_t count;       /* Transfer count */
  int16_t req_timeout;  /* Timeout for request, -1 if disable timeout */
  int16_t pkt_timeout;  /* Timeout between packets */
  uint16_t pkt_size;    /* Packet size during transfer (<= allocate max packet size) */
  uint8_t state;        /* Transfer state */
  int8_t status;        /* Last transfer status */
};

/**
 * Transfer descriptor for bulk / interrupt / iso transfer
 */

struct usb_h_bulk_int_iso_xfer
{
  uint32_t size;  /** Expected transfer size */
  uint32_t count; /** Transfer count */
  uint8_t *data;  /** Pointer to transfer data */
  uint16_t reserved[3];
  uint8_t state;  /** Transfer state */
  int8_t status;  /** Last transfer status */
};

/**
 * Transfer descriptor for periodic high bandwidth transfer
 */

struct usb_h_high_bw_xfer
{
  uint32_t size;         /** Expected transfer size */
  uint32_t count;        /** Transfer count */
  uint8_t *data;         /** Pointer to transfer data */
  uint16_t pkt_size[3];  /** Micro frame packet sizes */
  uint8_t state;         /** Transfer state */
  int8_t status;         /** Last transfer status */
};

/**
 * General transfer descriptor
 */

struct usb_h_xfer
{
  /** Reserved for different transfer */

  union
  {
    uint16_t u16[9];
    uint8_t  u8[18];
  } reserved;
  uint8_t state; /** Transfer state */
  int8_t status; /** Last transfer status */
};

/* USB Host Controller Driver Pipe structure */

/* This is the internal representation of an pipe */

struct sam_pipe_s
{
  struct usbhost_pipedesc_s *descb[2]; /* Pointers to this pipe descriptors */
  volatile uint8_t pipestate;          /* State of the pipe (see enum usb_h_pipe_state) */
  volatile uint8_t pipestatus;         /* Status of the pipe */
  volatile int8_t  pipestatus_general; /* Status of the pipe */
  volatile int8_t pipestate_general;
  int16_t result;                      /* The result of the transfer */
  uint32_t size;                       /* Expected transfer size */
  uint32_t count;                      /* Transfer count */
  uint8_t *data;                       /* Pointer to transfer data */
  int16_t pkt_timeout;                 /* Timeout between packets (500ms for data and 50ms for status), -1 if disabled */
  uint8_t zlp:1;                       /* Transfer ZLP support */

  uint8_t              stalled:1;    /* true: Endpoint is stalled */
  uint8_t              pending:1;    /* true: IN Endpoint stall is pending */
  uint8_t              halted:1;     /* true: Endpoint feature halted */
  uint8_t              zlpsent:1;    /* Zero length packet has been sent */
  uint8_t              txbusy:1;     /* Write request queue is busy (recursion avoidance kludge) */
  uint8_t              rxactive:1;   /* read request is active (for top of queue) */
  bool              inuse;           /* True: This pipe is "in use" */
  bool              in;              /* True: IN endpoint */
  uint8_t           idx;             /* Pipe index */
  uint8_t           epno;            /* Device endpoint number (0-127) */
  uint8_t           eptype;          /* See _EPTYPE_* definitions */
  uint8_t           funcaddr;        /* Device function address */
  uint8_t           speed;           /* Device speed */
  uint8_t           interval;        /* Interrupt/isochronous EP polling interval */
  uint16_t          maxpacket;       /* Max packet size */

  sem_t waitsem;               /* Channel wait semaphore */
  volatile bool waiter;        /* True: Thread is waiting for a channel event */

#ifdef CONFIG_USBHOST_ASYNCH
  usbhost_asynch_t callback;   /* Transfer complete callback */
  void *arg;                   /* Argument that accompanies the callback */
#endif

#ifdef HPL_USB_HOST      /* from: Atmel Start hpl_usb_host.h */
  uint16_t max_pkt_size; /* Endpoint max packet size (bits 10..0) */
  uint8_t ep;            /* Endpoint address */
  uint8_t type;          /* Endpoint type: Control, Isochronous, Bulk or Interrupt */
  uint8_t toggle;        /* Current toggle (driver dependent) */
  uint8_t bank : 2;      /* Endpoint number of banks (HW dependent) */
  uint8_t high_bw_out : 1;
  uint8_t dma : 1;            /* Uses DMA (on transfer) */
  uint8_t periodic_start : 1; /* Transfer periodic */

  /** Transfer status */

  union
  {
    struct usb_h_xfer general;          /* General transfer info */
    struct usb_h_ctrl_xfer ctrl;        /* Control transfer status */
    struct usb_h_bulk_int_iso_xfer bii; /* Bulk interrupt iso transfer status */
    struct usb_h_high_bw_xfer hbw;      /* Periodic high bandwidth transfer status */
  } x;
#endif
};

/* This structure retains the state of the USB host controller */

struct sam_usbhost_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to struct sam_usbhost_s.
   */

  struct usbhost_driver_s drvr;

  /* This is the hub port description understood by class drivers */

  struct usbhost_roothubport_s rhport;

  /* Overall driver status */

  uint8_t           hoststate; /* State of the device (see enum sam_hoststate_e) */
  uint8_t           prevstate; /* Previous state of the device before SUSPEND */
  uint16_t          epavail;   /* Bitset of available endpoints */
  mutex_t           lock;      /* Support mutually exclusive access */
  bool              connected; /* Connected to device */
  bool              change;    /* Connection change */
  bool              pscwait;   /* True: Thread is waiting for a port event */
  uint8_t           smstate;   /* The state of the USB host state machine */
  uint8_t           irqset;    /* Set of enabled interrupts */
  uint8_t           xfrtype;   /* See enum _hxfrdn_e */
  sem_t             pscsem;    /* Semaphore to wait for a port event */

  uint16_t pipes_unfreeze; /** Pipes to unfreeze after wakeup */
  int8_t suspend_start;    /** Delayed suspend time in ms */
  int8_t resume_start;     /** Delayed resume time in ms */
  int8_t n_ctrl_req_user;  /** Control transfer request user count */
  int8_t n_sof_user;       /** SOF user count (callback, suspend, resume, ctrl request) */
  uint8_t pipe_pool_size;  /** Pipe pool size in number of pipes */

#ifdef CONFIG_USBHOST_HUB

  /* Used to pass external hub port events */

  volatile struct usbhost_hubport_s *hport;
#endif

  /* The pipe list */

  aligned_data(4)
  struct sam_pipe_s pipelist[SAM_USB_NENDPOINTS];

  /* Pipe descriptors 2 banks for each pipe */

  aligned_data(4)
  struct usbhost_pipedesc_s pipe_descriptors[SAM_USB_NENDPOINTS *
                                             SAM_USB_NBANKS()];

  /* CTRL */

  usbhost_ep_t ep0; /* Root hub port EP0 description */
  aligned_data(4) uint8_t ctrl_buffer[64];
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations */

#ifdef CONFIG_SAMD5E5_USB_REGDEBUG
static void   sam_printreg(uintptr_t regaddr, uint32_t regval, bool iswrite);
static void   sam_checkreg(uintptr_t regaddr, uint32_t regval, bool iswrite);
static uint32_t sam_getreg32(uintptr_t regaddr);
static void   sam_putreg32(uint32_t regval, uintptr_t regaddr);
static uint32_t sam_getreg16(uintptr_t regaddr);
static void   sam_putreg16(uint16_t regval, uintptr_t regaddr);
static uint32_t sam_getreg8(uintptr_t regaddr);
static void   sam_putreg8(uint8_t regval, uintptr_t regaddr);
static void   sam_dumpep(struct sam_usbdev_s *priv, uint8_t epno);

#ifdef CONFIG_USBHOST
static void   sam_dumppipe(struct sam_usbhost_s *priv, uint8_t epno);
#endif
#else
static inline uint32_t sam_getreg32(uintptr_t regaddr);
static inline void sam_putreg32(uint32_t regval, uintptr_t regaddr);
static inline uint32_t sam_getreg16(uintptr_t regaddr);
static inline void sam_putreg16(uint16_t regval, uintptr_t regaddr);
static inline uint32_t sam_getreg8(uintptr_t regaddr);
static inline void sam_putreg8(uint8_t regval, uintptr_t regaddr);
# define sam_dumpep(priv, epno)
#ifdef CONFIG_USBHOST
# define sam_dumppipe(priv, epno)
#endif
#endif
static inline void sam_modifyreg8(uint32_t clrbits,
                                  uint32_t setbits,
                                  uintptr_t regaddr);

/* Clks */

static void sam_enableclks(void);

#ifdef CONFIG_USBDEV

static void sam_disableclks(void);

/* Suspend/Resume Helpers */

static void   sam_suspend(struct sam_usbdev_s *priv);
static void   sam_resume(struct sam_usbdev_s *priv);

/* Request Helpers */

static struct sam_req_s *
              sam_req_dequeue(struct sam_rqhead_s *queue);
static void   sam_req_enqueue(struct sam_rqhead_s *queue,
                struct sam_req_s *req);
static void   sam_req_complete(struct sam_ep_s *privep, int16_t result);
static void   sam_req_wrsetup(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep, struct sam_req_s *privreq);
static int    sam_req_write(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep);
static int    sam_req_read(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep, uint16_t recvsize);
static void   sam_req_cancel(struct sam_ep_s *privep, int16_t status);

/* Interrupt level processing */

static void   sam_ep0_ctrlread(struct sam_usbdev_s *priv);
static void   sam_ep0_wrstatus(struct sam_usbdev_s *priv,
                const uint8_t *buffer, size_t buflen);
static void   sam_ep0_dispatch(struct sam_usbdev_s *priv);
static void   sam_setdevaddr(struct sam_usbdev_s *priv, uint8_t value);
static void   sam_ep0_setup(struct sam_usbdev_s *priv);
static void   sam_ep_interrupt(struct sam_usbdev_s *priv, int epno);
static int    sam_usb_interrupt(int irq, void *context, void *arg);

/* Endpoint helpers */

static void   sam_ep_reset(struct sam_usbdev_s *priv, uint8_t epno);
static void   sam_epset_reset(struct sam_usbdev_s *priv, uint16_t epset);
static int    sam_ep_stall(struct sam_ep_s *privep);
static int    sam_ep_resume(struct sam_ep_s *privep);
static inline struct sam_ep_s *
              sam_ep_reserve(struct sam_usbdev_s *priv, uint8_t epset);
static inline void
              sam_ep_unreserve(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep);
static int    sam_ep_configure_internal(struct sam_ep_s *privep,
                const struct usb_epdesc_s *desc);

/* Endpoint operations */

static int    sam_ep_configure(struct usbdev_ep_s *ep,
                const struct usb_epdesc_s *desc, bool last);
static int    sam_ep_disable(struct usbdev_ep_s *ep);
static struct usbdev_req_s *
              sam_ep_allocreq(struct usbdev_ep_s *ep);
#ifdef CONFIG_USBDEV_DMA
static void  *sam_ep_allocbuffer(struct usbdev_ep_s *ep, uint16_t nbytes);
static void   sam_ep_freebuffer(struct usbdev_ep_s *ep, void *buf);
#endif
static void   sam_ep_freereq(struct usbdev_ep_s *ep,
                struct usbdev_req_s *);
static int    sam_ep_submit(struct usbdev_ep_s *ep,
                struct usbdev_req_s *req);
static int    sam_ep_cancel(struct usbdev_ep_s *ep,
                struct usbdev_req_s *req);
static int    sam_ep_stallresume(struct usbdev_ep_s *ep, bool resume);

/* USB device controller operations */

static struct usbdev_ep_s *
              sam_allocep(struct usbdev_s *dev, uint8_t epno, bool in,
                uint8_t eptype);
static void   sam_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep);
static int    sam_getframe(struct usbdev_s *dev);
static int    sam_wakeup(struct usbdev_s *dev);
static int    sam_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int    sam_pullup(struct usbdev_s *dev,  bool enable);

/* Initialization/Reset */

static void   sam_reset(struct sam_usbdev_s *priv);
static void   sam_hw_setup(struct sam_usbdev_s *priv);
static void   sam_sw_setup(struct sam_usbdev_s *priv);
static void   sam_hw_shutdown(struct sam_usbdev_s *priv);
static void   sam_sw_shutdown(struct sam_usbdev_s *priv);
#endif

#ifdef CONFIG_USBHOST

#undef CONFIG_SAM_USBHOST_PKTDUMP
#ifdef CONFIG_SAM_USBHOST_PKTDUMP
#  define sam_pktdump(m,b,n) lib_dumpbuffer(m,b,n)
#else
#  define sam_pktdump(m,b,n)
#endif

/* Pipe helpers */

static void   sam_reset_pipes(struct sam_usbhost_s *priv, bool warm_reset);
static void   sam_pipe_reset(struct sam_usbhost_s *priv, uint8_t epno);
static void   sam_pipeset_reset(struct sam_usbhost_s *priv, uint16_t epset);

/* Byte stream access helper functions */

static inline uint16_t sam_getle16(const uint8_t *val);

/* Pipe management */

static int sam_pipe_alloc(struct sam_usbhost_s *priv);
static inline void sam_pipe_free(struct sam_usbhost_s *priv,
              int idx);
static void sam_pipe_configure(struct sam_usbhost_s *priv, int idx);
static int sam_pipe_waitsetup(struct sam_usbhost_s *priv,
              struct sam_pipe_s *pipe);
#ifdef CONFIG_USBHOST_ASYNCH
static int sam_pipe_asynchsetup(struct sam_usbhost_s *priv,
                                struct sam_pipe_s *pipe,
                                usbhost_asynch_t callback, void *arg);
#endif
static int sam_pipe_wait(struct sam_usbhost_s *priv,
              struct sam_pipe_s *pipe);
static void sam_pipe_wakeup(struct sam_usbhost_s *priv,
              struct sam_pipe_s *pipe);
static int sam_ctrlep_alloc(struct sam_usbhost_s *priv,
                            const struct usbhost_epdesc_s *epdesc,
                            usbhost_ep_t *ep);
static int sam_xfrep_alloc(struct sam_usbhost_s *priv,
                           const struct usbhost_epdesc_s *epdesc,
                           usbhost_ep_t *ep);

/* Control/data transfer logic */

static void sam_transfer_terminate(struct sam_usbhost_s *priv,
              struct sam_pipe_s *pipe, int result);
static void sam_transfer_abort(struct sam_usbhost_s *priv,
              struct sam_pipe_s *pipe, int result);

/* OUT transfers */

static void sam_send_continue(struct sam_usbhost_s *priv,
                              struct sam_pipe_s *pipe);
static void sam_send_start(struct sam_usbhost_s *priv,
                           struct sam_pipe_s *pipe);
static ssize_t sam_out_transfer(struct sam_usbhost_s *priv,
                                struct sam_pipe_s *pipe,
                                uint8_t *buffer,
                                size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void sam_out_next(struct sam_usbhost_s *priv,
                         struct sam_pipe_s *pipe);
static int  sam_out_asynch(struct sam_usbhost_s *priv,
                           struct sam_pipe_s *pipe,
                           uint8_t *buffer, size_t buflen,
                           usbhost_asynch_t callback, void *arg);
#endif

/* Control transfers */

static int  sam_ctrl_sendsetup(struct sam_usbhost_s *priv,
                               struct sam_pipe_s *pipe,
                               const struct usb_ctrlreq_s *req);
static int  sam_ctrl_senddata(struct sam_usbhost_s *priv,
                              struct sam_pipe_s *pipe,
                              uint8_t *buffer, unsigned int buflen);
static int  sam_ctrl_recvdata(struct sam_usbhost_s *priv,
                              struct sam_pipe_s *pipe,
                              uint8_t *buffer, unsigned int buflen);
static int  sam_in_setup(struct sam_usbhost_s *priv,
                         struct sam_pipe_s *pipe);
static int  sam_out_setup(struct sam_usbhost_s *priv,
                          struct sam_pipe_s *pipe);

/* IN transfers */

static void sam_recv_continue(struct sam_usbhost_s *priv,
                              struct sam_pipe_s *pipe);
static void sam_recv_restart(struct sam_usbhost_s *priv,
                             struct sam_pipe_s *pipe);
static void sam_recv_start(struct sam_usbhost_s *priv,
                           struct sam_pipe_s *pipe);
static ssize_t sam_in_transfer(struct sam_usbhost_s *priv,
                               struct sam_pipe_s *pipe,
                               uint8_t *buffer,
                               size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void sam_in_next(struct sam_usbhost_s *priv,
                        struct sam_pipe_s *pipe);
static int  sam_in_asynch(struct sam_usbhost_s *priv,
                          struct sam_pipe_s *pipe,
                          uint8_t *buffer, size_t buflen,
                          usbhost_asynch_t callback, void *arg);
#endif

/* Interrupt handling */

/* Lower level interrupt handlers */

static void sam_gint_connected(struct sam_usbhost_s *priv);
static void sam_gint_disconnected(struct sam_usbhost_s *priv);

static void sam_pipe_interrupt(struct sam_usbhost_s *priv, int idx);
static int  sam_usbhost_interrupt(int irq, void *context, void *arg);

/* USB host controller operations */

static int sam_wait(struct usbhost_connection_s *conn,
                    struct usbhost_hubport_s **hport);
static int sam_rh_enumerate(struct sam_usbhost_s *priv,
                            struct usbhost_connection_s *conn,
                            struct usbhost_hubport_s *hport);
static int sam_enumerate(struct usbhost_connection_s *conn,
                         struct usbhost_hubport_s *hport);
static int sam_ep0configure(struct usbhost_driver_s *drvr,
                            usbhost_ep_t ep0, uint8_t funcaddr,
                            uint8_t speed,
                            uint16_t maxpacketsize);
static int sam_epalloc(struct usbhost_driver_s *drvr,
                       const struct usbhost_epdesc_s *epdesc,
                       usbhost_ep_t *ep);
static int sam_epfree(struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int sam_alloc(struct usbhost_driver_s *drvr,
                     uint8_t **buffer, size_t *maxlen);
static int sam_free(struct usbhost_driver_s *drvr,
                    uint8_t *buffer);
static int sam_ioalloc(struct usbhost_driver_s *drvr,
                       uint8_t **buffer, size_t buflen);
static int sam_iofree(struct usbhost_driver_s *drvr,
                      uint8_t *buffer);
static int sam_ctrlin(struct usbhost_driver_s *drvr,
                      usbhost_ep_t ep0,
                      const struct usb_ctrlreq_s *req,
                      uint8_t *buffer);
static int sam_ctrlout(struct usbhost_driver_s *drvr,
                       usbhost_ep_t ep0,
                       const struct usb_ctrlreq_s *req,
                       const uint8_t *buffer);
static ssize_t sam_transfer(struct usbhost_driver_s *drvr,
                            usbhost_ep_t ep,
                            uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static int sam_asynch(struct usbhost_driver_s *drvr,
                      usbhost_ep_t ep,
                      uint8_t *buffer, size_t buflen,
                      usbhost_asynch_t callback, void *arg);
#endif
static int sam_cancel(struct usbhost_driver_s *drvr,
                      usbhost_ep_t ep);
#ifdef CONFIG_USBHOST_HUB
static int sam_connect(struct usbhost_driver_s *drvr,
                       struct usbhost_hubport_s *hport,
                       bool connected);
#endif
static void sam_disconnect(struct usbhost_driver_s *drvr,
                           struct usbhost_hubport_s *hport);

static void sam_hostreset(struct sam_usbhost_s *priv);
static void sam_add_sof_user(struct sam_usbhost_s *priv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_USBHOST

/* In this driver implementation, support is provided for only a single
 * USB host. All status information can be simply retained in a single global
 * instance.
 */

static struct sam_usbhost_s g_usbhost =
{
  .lock = NXMUTEX_INITIALIZER,
  .pscsem = SEM_INITIALIZER(0),
};

/* This is the connection/enumeration interface */

static struct usbhost_connection_s g_usbconn =
{
  .wait      = sam_wait,
  .enumerate = sam_enumerate,
};
#endif

#ifdef CONFIG_USBDEV

/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static struct sam_usbdev_s g_usbd;

static const struct usbdev_epops_s g_epops =
{
  .configure     = sam_ep_configure,
  .disable       = sam_ep_disable,
  .allocreq      = sam_ep_allocreq,
  .freereq       = sam_ep_freereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer   = sam_ep_allocbuffer,
  .freebuffer    = sam_ep_freebuffer,
#endif
  .submit        = sam_ep_submit,
  .cancel        = sam_ep_cancel,
  .stall         = sam_ep_stallresume,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep       = sam_allocep,
  .freeep        = sam_freeep,
  .getframe      = sam_getframe,
  .wakeup        = sam_wakeup,
  .selfpowered   = sam_selfpowered,
  .pullup        = sam_pullup,
};

/* This describes endpoint 0 */

static const struct usb_epdesc_s g_ep0desc =
{
  .len           = USB_SIZEOF_EPDESC,
  .type          = USB_DESC_TYPE_ENDPOINT,
  .addr          = EP0,
  .attr          = USB_EP_ATTR_XFER_CONTROL,
  .mxpacketsize  =
  {
    64, 0
  },
  .interval      = 0
};
#endif

/* Device error strings that may be enabled for more desciptive USB trace
 * output.
 */

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_deverror[] =
{
  TRACE_STR(SAM_TRACEERR_ALLOCFAIL),
  TRACE_STR(SAM_TRACEERR_BADCLEARFEATURE),
  TRACE_STR(SAM_TRACEERR_BADDEVGETSTATUS),
  TRACE_STR(SAM_TRACEERR_BADEPGETSTATUS),
  TRACE_STR(SAM_TRACEERR_BADEOBSTATE),
  TRACE_STR(SAM_TRACEERR_BADEPNO),
  TRACE_STR(SAM_TRACEERR_BADEPTYPE),
  TRACE_STR(SAM_TRACEERR_BADGETCONFIG),
  TRACE_STR(SAM_TRACEERR_BADGETSETDESC),
  TRACE_STR(SAM_TRACEERR_BADGETSTATUS),
  TRACE_STR(SAM_TRACEERR_BADSETADDRESS),
  TRACE_STR(SAM_TRACEERR_BADSETCONFIG),
  TRACE_STR(SAM_TRACEERR_BADSETFEATURE),
  TRACE_STR(SAM_TRACEERR_BINDFAILED),
  TRACE_STR(SAM_TRACEERR_DISPATCHSTALL),
  TRACE_STR(SAM_TRACEERR_DRIVER),
  TRACE_STR(SAM_TRACEERR_DRIVERREGISTERED),
  TRACE_STR(SAM_TRACEERR_EP0SETUPOUTSIZE),
  TRACE_STR(SAM_TRACEERR_EP0SETUPSTALLED),
  TRACE_STR(SAM_TRACEERR_EPOUTNULLPACKET),
  TRACE_STR(SAM_TRACEERR_EPRESERVE),
  TRACE_STR(SAM_TRACEERR_INVALIDCTRLREQ),
  TRACE_STR(SAM_TRACEERR_INVALIDPARMS),
  TRACE_STR(SAM_TRACEERR_IRQREGISTRATION),
  TRACE_STR(SAM_TRACEERR_NOTCONFIGURED),
  TRACE_STR(SAM_TRACEERR_REQABORTED),
  TRACE_STR(SAM_TRACEERR_RXDATABKERR),
  TRACE_STR(SAM_TRACEERR_TXCOMPERR),
  TRACE_STR(SAM_TRACEERR_UNSUPPEPTYPE),
  TRACE_STR_END
};
#endif

/* Interrupt event strings that may be enabled for more desciptive USB trace
 * output.
 */

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_intdecode[] =
{
  TRACE_STR(SAM_TRACEINTID_INTERRUPT),
  TRACE_STR(SAM_TRACEINTID_PENDING),
  TRACE_STR(SAM_TRACEINTID_PENDING_EP),
  TRACE_STR(SAM_TRACEINTID_SUSPEND),
  TRACE_STR(SAM_TRACEINTID_SOF),
  TRACE_STR(SAM_TRACEINTID_EORST),
  TRACE_STR(SAM_TRACEINTID_WAKEUP),
  TRACE_STR(SAM_TRACEINTID_EORSM),
  TRACE_STR(SAM_TRACEINTID_UPRSM),
  TRACE_STR(SAM_TRACEINTID_RAMACER),
  TRACE_STR(SAM_TRACEINTID_LPMNYET),
  TRACE_STR(SAM_TRACEINTID_LPMSUSP),

  TRACE_STR(SAM_TRACEINTID_EPNO),
  TRACE_STR(SAM_TRACEINTID_EPINTFLAGS),
  TRACE_STR(SAM_TRACEINTID_EPTRCPT0),
  TRACE_STR(SAM_TRACEINTID_EPTRCPT1),
  TRACE_STR(SAM_TRACEINTID_EPTRFAIL0),
  TRACE_STR(SAM_TRACEINTID_EPTRFAIL1),
  TRACE_STR(SAM_TRACEINTID_EPRXSTP),
  TRACE_STR(SAM_TRACEINTID_EPSTALL0),
  TRACE_STR(SAM_TRACEINTID_EPSTALL1),

  TRACE_STR(SAM_TRACEINTID_EPINQEMPTY),
  TRACE_STR(SAM_TRACEINTID_EPOUTQEMPTY),
  TRACE_STR(SAM_TRACEINTID_EP0SETUPOUT),
  TRACE_STR(SAM_TRACEINTID_EP0SETUPIN),
  TRACE_STR(SAM_TRACEINTID_EP0SETUPSETADDRESS),
  TRACE_STR(SAM_TRACEINTID_NOSTDREQ),
  TRACE_STR(SAM_TRACEINTID_GETSTATUS),
  TRACE_STR(SAM_TRACEINTID_DEVGETSTATUS),
  TRACE_STR(SAM_TRACEINTID_IFGETSTATUS),
  TRACE_STR(SAM_TRACEINTID_CLEARFEATURE),
  TRACE_STR(SAM_TRACEINTID_SETFEATURE),
  TRACE_STR(SAM_TRACEINTID_GETSETDESC),
  TRACE_STR(SAM_TRACEINTID_GETCONFIG),
  TRACE_STR(SAM_TRACEINTID_SETCONFIG),
  TRACE_STR(SAM_TRACEINTID_GETSETIF),
  TRACE_STR(SAM_TRACEINTID_SYNCHFRAME),

  TRACE_STR(SAM_TRACEINTID_DISPATCH),
  TRACE_STR(SAM_TRACEINTID_ADDRESSED),
  TRACE_STR(SAM_TRACEINTID_EPCONF),
  TRACE_STR(SAM_TRACEINTID_EPINTEN),
  TRACE_STR(SAM_TRACEINTID_EP0WRSTATUS),
  TRACE_STR(SAM_TRACEINTID_EPTRCPT0_LEN),

  TRACE_STR(SAM_TRACEINTID_PENDING_PIPE),
  TRACE_STR(SAM_TRACEINTID_PIPENO),

  TRACE_STR_END
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Register Operations
 ****************************************************************************/

/****************************************************************************
 * Name: sam_printreg
 *
 * Description:
 *   Print the SAMD5E5 USB register access
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD5E5_USB_REGDEBUG
static void sam_printreg(uintptr_t regaddr, uint32_t regval, bool iswrite)
{
  uinfo("%p%s0x%08x\n", regaddr, iswrite ? "<-" : "->", regval);
}
#endif

/****************************************************************************
 * Name: sam_checkreg
 *
 * Description:
 *   Check if it is time to output debug information for accesses
 *   to a SAMD5E5 USB registers.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD5E5_USB_REGDEBUG
static void sam_checkreg(uintptr_t regaddr, uint32_t regval, bool iswrite)
{
  static uintptr_t prevaddr  = 0;
  static uint32_t  preval    = 0;
  static uint32_t  count     = 0;
  static bool      prevwrite = false;

  /* Is this the same value that we read from/wrote
   * to the same register last time?
   * Are we polling the register?  If so, suppress the output.
   */

  if (regaddr == prevaddr && regval == preval && prevwrite == iswrite)
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

              sam_printreg(prevaddr, preval, prevwrite);
            }
          else
            {
              /* No.. More than one. */

              uinfo("[repeats %d more times]\n", count);
            }
        }

      /* Save the new address, value, count, and operation for next time */

      prevaddr  = regaddr;
      preval    = regval;
      count     = 0;
      prevwrite = iswrite;

      /* Show the new register access */

      sam_printreg(regaddr, regval, iswrite);
    }
}
#endif

/****************************************************************************
 * Name: sam_getreg32
 *
 * Description:
 *   Get the contents of an 32-bit SAMD5E5 USB register
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD5E5_USB_REGDEBUG
static uint32_t sam_getreg32(uintptr_t regaddr)
{
  /* Read the value from the register */

  uint32_t regval = getreg32(regaddr);

  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, false);
  return regval;
}
#else
static inline uint32_t sam_getreg32(uintptr_t regaddr)
{
  return getreg32(regaddr);
}
#endif

/****************************************************************************
 * Name: sam_putreg32
 *
 * Description:
 *   Set the contents of an 32-bit SAMD5E5 USB register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD5E5_USB_REGDEBUG
static void sam_putreg32(uint32_t regval, uintptr_t regaddr)
{
  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, true);

  /* Write the value */

  putreg32(regval, regaddr);
}
#else
static inline void sam_putreg32(uint32_t regval, uint32_t regaddr)
{
  putreg32(regval, regaddr);
}
#endif

/****************************************************************************
 * Name: sam_getreg16
 *
 * Description:
 *   Get the contents of an 16-bit SAMD5E5 USB register
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD5E5_USB_REGDEBUG
static uint32_t sam_getreg16(uintptr_t regaddr)
{
  /* Read the value from the register */

  uint32_t regval = getreg16(regaddr);

  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, false);
  return regval;
}
#else
static inline uint32_t sam_getreg16(uintptr_t regaddr)
{
  return getreg16(regaddr);
}
#endif

/****************************************************************************
 * Name: sam_putreg16
 *
 * Description:
 *   Set the contents of an 16-bit SAMD5E5 USB register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD5E5_USB_REGDEBUG
static void sam_putreg16(uint16_t regval, uintptr_t regaddr)
{
  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, true);

  /* Write the value */

  putreg16(regval, regaddr);
}
#else
static inline void sam_putreg16(uint16_t regval, uint32_t regaddr)
{
  putreg16(regval, regaddr);
}
#endif

/****************************************************************************
 * Name: sam_getreg8
 *
 * Description:
 *   Get the contents of an 8-bit SAMD5E5 USB register
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD5E5_USB_REGDEBUG
static uint32_t sam_getreg8(uintptr_t regaddr)
{
  /* Read the value from the register */

  uint32_t regval = getreg8(regaddr);

  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, false);
  return regval;
}
#else
static inline uint32_t sam_getreg8(uintptr_t regaddr)
{
  return getreg8(regaddr);
}
#endif

/****************************************************************************
 * Name: sam_putreg8
 *
 * Description:
 *   Set the contents of an 8-bit SAMD5E5 USB register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD5E5_USB_REGDEBUG
static void sam_putreg8(uint8_t regval, uintptr_t regaddr)
{
  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, true);

  /* Write the value */

  putreg8(regval, regaddr);
}
#else
static inline void sam_putreg8(uint8_t regval, uint32_t regaddr)
{
  putreg8(regval, regaddr);
}
#endif

/****************************************************************************
 * Name: sam_dumpep
 ****************************************************************************/

#if defined(CONFIG_SAMD5E5_USB_REGDEBUG) && defined(CONFIG_DEBUG_USB)
#ifdef CONFIG_USBDEV
static void sam_dumpep(struct sam_usbdev_s *priv, uint8_t epno)
{
  /* Global Registers */

  uinfo("Global Registers:\n");
  uinfo("       CTRLB: 0x%04x\n", sam_getreg16(SAM_USBDEV_CTRLB));
  uinfo("        FNUM: 0x%04x\n", sam_getreg16(SAM_USBDEV_FNUM));
  uinfo("        DADD: 0x%02x\n", sam_getreg8(SAM_USBDEV_DADD));
  uinfo("    INTENSET: 0x%04x\n", sam_getreg16(SAM_USBDEV_INTENSET));
  uinfo("      STATUS: 0x%02x\n", sam_getreg8(SAM_USBDEV_STATUS));
  uinfo("     INTFLAG: 0x%04x\n", sam_getreg16(SAM_USBDEV_INTFLAG));
  uinfo("   EPCFG[%d]: 0x%02x\n", epno, sam_getreg8(SAM_USBDEV_EPCFG(epno)));
  uinfo("EPSTATUS[%d]: 0x%02x\n", epno,
                                  sam_getreg8(SAM_USBDEV_EPSTATUS(epno)));
}
#endif

#ifdef CONFIG_USBHOST
static void sam_dumppipe(struct sam_usbhost_s *priv, uint8_t epno)
{
  /* Global Registers */

  uinfo("Global Host Registers:\n");
  uinfo("       CTRLB: 0x%04x\n", sam_getreg16(SAM_USBHOST_CTRLB));
  uinfo("        FNUM: 0x%04x\n", sam_getreg16(SAM_USBHOST_FNUM));
  uinfo("       HSOFC: 0x%02x\n", sam_getreg8(SAM_USBHOST_HSOFC));
  uinfo("    INTENSET: 0x%04x\n", sam_getreg16(SAM_USBHOST_INTENSET));
  uinfo("      STATUS: 0x%02x\n", sam_getreg8(SAM_USBHOST_STATUS));
  uinfo("     INTFLAG: 0x%04x\n", sam_getreg16(SAM_USBHOST_INTFLAG));
  uinfo(" PIPECFG[%d]: 0x%02x\n", epno, sam_getreg8(SAM_USBHOST_PCFG(epno)));
  uinfo(" PSTATUS[%d]: 0x%02x\n", epno,
                                  sam_getreg8(SAM_USBHOST_PSTATUS(epno)));
}
#endif
#endif

/****************************************************************************
 * Name: sam_modifyreg8
 *
 * Description:
 *   Modify selected bits of an SAM register.
 *
 ****************************************************************************/

static inline void sam_modifyreg8(uint32_t clrbits,
                                  uint32_t setbits, uintptr_t regaddr)
{
  sam_putreg8((((sam_getreg8(regaddr)) & (~clrbits)) | setbits), regaddr);
}

/****************************************************************************
 * Name: sam_enableclks
 * Description:
 *   Enable USB clock
 ****************************************************************************/

static void sam_enableclks(void)
{
  sam_gclk_chan_enable(GCLK_CHAN_USB, BOARD_USB_GCLKGEN, false);
  sam_ahb_usb_enableperiph();
  sam_apb_usb_enableperiph();
}

/****************************************************************************
 * Name: sam_ctrla_write
 *
 * Description:
 *   writes value to CTRLA register some bits needs write-synchronisation
 *
 ****************************************************************************/

static void sam_ctrla_write(uint8_t value)
{
  sam_putreg8(value, SAM_USB_CTRLA);

  if (value & USB_CTRLA_SWRST)
    {
      /* Due to synchronization there is a delay from writing CTRLA.SWRST
       * until the reset is complete. CTRLA.SWRST and SYNCBUSY.SWRST will
       * both be cleared when the reset is complete.
       */

      while ((sam_getreg8(SAM_USB_CTRLA) & USB_CTRLA_SWRST) &&
             (sam_getreg8(SAM_USB_SYNCBUSY) & USB_SYNCBUSY_SWRST))
        ;

      return;
    }

  if (value & USB_CTRLA_ENABLE)
    {
      /* Due to synchronization there is delay from writing CTRLA.ENABLE
       * until the peripheral is enabled/disabled.
       * SYNCBUSY.ENABLE will be cleared when the operation is complete.
       */

      while ((sam_getreg8(SAM_USB_SYNCBUSY) & USB_SYNCBUSY_ENABLE))
        ;
    }
}

#ifdef CONFIG_USBDEV

/****************************************************************************
 * Name: sam_disableclks
 * Description:
 *   Disable USB clock
 ****************************************************************************/

static void sam_disableclks(void)
{
  sam_gclk_chan_disable(GCLK_CHAN_USB);
  sam_apb_usb_disableperiph();
}

/****************************************************************************
 * Request Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: sam_req_dequeue
 ****************************************************************************/

static struct sam_req_s *sam_req_dequeue(struct sam_rqhead_s *queue)
{
  struct sam_req_s *ret = queue->head;

  if (ret)
    {
      queue->head = ret->flink;
      if (!queue->head)
        {
          queue->tail = NULL;
        }

      ret->flink = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: sam_req_enqueue
 ****************************************************************************/

static void sam_req_enqueue(struct sam_rqhead_s *queue,
                            struct sam_req_s *req)
{
  req->flink = NULL;
  if (!queue->head)
    {
      queue->head = req;
      queue->tail = req;
    }
  else
    {
      queue->tail->flink = req;
      queue->tail        = req;
    }
}

/****************************************************************************
 * Name: sam_req_complete
 ****************************************************************************/

static void sam_req_complete(struct sam_ep_s *privep, int16_t result)
{
  struct sam_req_s *privreq;
  irqstate_t flags;

  uinfo("ENTRY\n");

  /* Remove the completed request at the head of the endpoint request list */

  flags = enter_critical_section();
  privreq = sam_req_dequeue(&privep->reqq);
  leave_critical_section(flags);

  if (privreq)
    {
      /* Save the result in the request structure */

      privreq->req.result = result;

      /* Callback to the request completion handler */

      privreq->flink = NULL;
      privreq->req.callback(&privep->ep, &privreq->req);

      /* mark endpoint ready to next transmission */

      privep->epstate = USB_EPSTATE_IDLE;
      privep->zlpsent = false;
    }
}

/****************************************************************************
 * Name: sam_req_wrsetup
 *
 * Description:
 *   Process the next queued write request.
 *
 ****************************************************************************/

static void sam_req_wrsetup(struct sam_usbdev_s *priv,
                            struct sam_ep_s *privep,
                            struct sam_req_s *privreq)
{
  const uint8_t *buf;
  uint8_t epno;
  int nbytes;
  uint32_t packetsize;

  /* Get the unadorned endpoint number */

  epno = USB_EPNO(privep->ep.eplog);

  /* Get the number of bytes remaining to be sent. */

  DEBUGASSERT(privreq->req.xfrd < privreq->req.len);
  nbytes = privreq->req.len - privreq->req.xfrd;

  /* Either send the maxpacketsize(multi) or all of the remaining data in
   * the request.
   */

  if (nbytes >= SAM_MAX_MULTIPACKET_SIZE)
    {
      nbytes = SAM_MAX_MULTIPACKET_SIZE;
    }

  /* This is the new number of bytes "in-flight" */

  privreq->inflight = nbytes;
  usbtrace(TRACE_WRITE(USB_EPNO(privep->ep.eplog)), nbytes);

  /* The new buffer pointer is the start of the buffer plus the number of
   * bytes successfully transferred plus the number of bytes previously
   * "in-flight".
   */

  buf = privreq->req.buf + privreq->req.xfrd;

  /* setup TX transfer using ep configured maxpacket size */

  priv->eplist[epno].descb[1]->addr = (uint32_t) buf;
  uinfo("addr=%p\n", buf);
  packetsize = priv->eplist[epno].descb[1]->pktsize;
  packetsize &= ~USBDEV_PKTSIZE_BCNT_MASK;
  packetsize &= ~USBDEV_PKTSIZE_MPKTSIZE_MASK;
  packetsize |= USBDEV_PKTSIZE_BCNT(nbytes);

  /* also set automatic ZLP sending if requested on req */

  if (privreq->req.flags & USBDEV_REQFLAGS_NULLPKT)
    {
      packetsize |= USBDEV_PKTSIZE_AUTOZLP;
    }

  priv->eplist[epno].descb[1]->pktsize = packetsize;

  /* Indicate that we are in the sending state
   * This indication will be need in interrupt processing (TRCPT1)
   * in order to properly terminate the request.
   */

  privep->epstate = USB_EPSTATE_SENDING;

  /* Set BK1RDY to notify the USB hardware that TX data is ready on
   * descriptor bank1.  We will be notified that the descriptor has been
   * transmitted by the USB device when TRCPT1 in the endpoint's EPINTFLAG
   * register has been set.
   */

  sam_putreg8(USBDEV_EPSTATUS_BK1RDY, SAM_USBDEV_EPSTATUSSET(epno));
}

/****************************************************************************
 * Name: sam_req_write
 *
 * Description:
 *   Process the next queued write request. This function is called in one
 *   of three contexts:
 *     (1) When the endpoint is IDLE and a new write request is submitted
 *         (with interrupts disabled),
 *     (2) from TRCPT1 interrupt handling when the current Tx transfer
 *         completes
 *     (3) when resuming a stalled IN or control endpoint.
 *
 *   Calling rules:
 *
 *     The transfer state must IDLE
 *
 *     When a request is queued, the request 'len' is the number of bytes
 *     to transfer and 'xfrd' and 'inflight' must be zero.
 *
 *     When this function starts a transfer it will update the request
 *     'inflight' field to indicate the size of the transfer.
 *
 *     When the transfer completes, the 'inflight' field must hold the
 *     number of bytes that have completed the transfer.  This function will
 *     update 'xfrd' with the new size of the transfer.
 *
 ****************************************************************************/

static int sam_req_write(struct sam_usbdev_s *priv, struct sam_ep_s *privep)
{
  struct sam_req_s *privreq;
  uint8_t epno;
  int bytesleft;

  /* Get the unadorned endpoint number */

  epno = USB_EPNO(privep->ep.eplog);

  /* We get here when an IN endpoint interrupt occurs.  So now we know that
   * there is no TX transfer in progress (epstate should be IDLE).
   */

  DEBUGASSERT(privep->epstate == USB_EPSTATE_IDLE);
  while (privep->epstate == USB_EPSTATE_IDLE)
    {
      /* Check the request from the head of the endpoint request queue */

      privreq = sam_rqpeek(&privep->reqq);
      if (!privreq)
        {
          /* There is no TX transfer in progress and no new pending TX
           * requests to send.
           */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPINQEMPTY), epno);

          /* Was there a pending endpoint stall? */

          if (privep->pending)
            {
              /* Yes... stall the endpoint now */

              sam_ep_stall(privep);
            }

          return -ENOENT;
        }

      uinfo("epno=%d req=%p: len=%d xfrd=%d inflight=%d\n",
            epno, privreq, privreq->req.len, privreq->req.xfrd,
            privreq->inflight);

      /* Handle any bytes in flight. */

      privreq->req.xfrd += privreq->inflight;
      privreq->inflight  = 0;

      /* Get the number of bytes left to be sent in the packet */

      bytesleft = privreq->req.len - privreq->req.xfrd;
      if (bytesleft > 0)
        {
          /* Perform the write operation.  epstate will become SENDING. */

          sam_req_wrsetup(priv, privep, privreq);
        }

      /* No data to send...
       * This can happen on one of two ways:
       * (1) The last packet sent was the final packet of a transfer.
       *     Or
       * (2) called with a request packet that has len == 0
       *
       * len == 0 means that it is requested to send a zero length packet
       * required by protocoll
       */

      else if ((privreq->req.len == 0) && !privep->zlpsent)
        {
          /* If we get here, we requested to send the zero length packet now.
           */

          privep->epstate   = USB_EPSTATE_SENDING;
          privep->zlpsent   = true;
          privreq->inflight = 0;

          usbtrace(TRACE_WRITE(epno), 0);

          /* setup 0 length TX transfer */

          priv->eplist[0].descb[1]->addr     = (uint32_t) &priv->ep0out[0];
          uinfo("addr=%p\n", &priv->ep0out[0]);
          priv->eplist[0].descb[1]->pktsize &= ~USBDEV_PKTSIZE_BCNT_MASK;
          priv->eplist[0].descb[1]->pktsize &= ~USBDEV_PKTSIZE_MPKTSIZE_MASK;
          priv->eplist[0].descb[1]->pktsize |= USBDEV_PKTSIZE_BCNT(0);
          sam_putreg8(USBDEV_EPSTATUS_BK1RDY, SAM_USBDEV_EPSTATUSSET(epno));
        }

      /* If all of the bytes were sent (including any final zero length
       * packet) then we are finished with the request buffer and we can
       * return the request buffer to the class driver.  The state will
       * remain IDLE only if nothing else was put in flight.
       *
       * Note that we will then loop to check to check the next queued
       * write request.
       */

      if (privep->epstate == USB_EPSTATE_IDLE)
        {
          /* Return the write request to the class driver.  Set the txbusy
           * bit to prevent being called recursively from any new submission
           * generated by returning the write request.
           */

          usbtrace(TRACE_COMPLETE(epno), privreq->req.xfrd);
          DEBUGASSERT(privreq->req.len == privreq->req.xfrd);

          privep->txbusy = true;
          sam_req_complete(privep, OK);
          privep->txbusy = false;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: sam_req_read
 *
 * Description:
 *   Complete the last read request. full or partial.
 *   The USB core has transferred the data to user request buffer.
 *   return the completed read request to the class
 *   implementation, and try to start the next queued read request.
 *
 *   REVISIT:
 *   This function is called in one of two contexts:
 *   The normal case is
 *   (1) When the endpoint is IDLE and a new read request is submitted
 *         (with interrupts disabled),
 *   (2) from interrupt handling when the current RX transfer completes.
 *       But there is also a special case
 *   (3) when the OUT endpoint is stopped because there are no
 *       available read requests.
 *
 *   Calling rules:
 *
 *     The transfer state must IDLE
 *
 *     When a request is queued, the request 'len' is size of the request
 *     buffer.  Any OUT request can be received that will fit in this
 *     buffer.  'xfrd' and 'inflight' in the request must be zero
 *     If sam_req_read() is called to start a new transfer, the recvsize
 *     parameter must be zero.
 *
 *     When the transfer completes, the 'recvsize' is the number of bytes
 *     ready on req->buffer.
 *
 ****************************************************************************/

static int sam_req_read(struct sam_usbdev_s *priv, struct sam_ep_s *privep,
                        uint16_t recvsize)
{
  struct sam_req_s *privreq;
  int epno;

  DEBUGASSERT(priv && privep && privep->epstate == USB_EPSTATE_IDLE);

  /* Check the request from the head of the endpoint request queue */

  epno = USB_EPNO(privep->ep.eplog);
  do
    {
      /* Peek at the next read request in the request queue */

      privreq = sam_rqpeek(&privep->reqq);
      if (!privreq)
        {
          /* No read request to receive data */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPOUTQEMPTY), epno);

          /* When no read requests are pending no EP descriptors are set to
           * ready. HW sends NAK to host if it tries to send something.
           */

          privep->epstate = USB_EPSTATE_RXSTOPPED;
          return -ENOENT;
        }

      uinfo("EP%d: req.len=%d xfrd=%d recvsize=%d\n",
            epno, privreq->req.len, privreq->req.xfrd, recvsize);

      /* Ignore any attempt to receive a zero length packet */

      if (privreq->req.len == 0)
        {
          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EPOUTNULLPACKET), 0);
          sam_req_complete(privep, OK);
          privreq = NULL;
        }

      /* complete read request with available data */

      if ((privreq->inflight) && (recvsize != 0))
        {
          usbtrace(TRACE_READ(USB_EPNO(privep->ep.eplog)), recvsize);

          /* Update the total number of bytes transferred */

          privreq->req.xfrd += recvsize;
          privreq->inflight  = 0;
          usbtrace(TRACE_COMPLETE(epno), privreq->req.xfrd);
          sam_req_complete(privep, OK);

          /* need to set recvsize to zero.  When calling sam_req_complete()
           * class driver could call submit() again and we have new request
           * ready on next while() loop.
           */

          privep->rxactive = false;
          recvsize = 0;
          privreq = NULL;
        }
    }
  while (privreq == NULL);

  DEBUGASSERT(recvsize == 0);

  /* activate new read request from queue */

  privep->rxactive  = true;
  privreq->req.xfrd = 0;
  privreq->inflight = privreq->req.len;
  priv->eplist[epno].descb[0]->addr = (uint32_t) privreq->req.buf;
  uinfo("addr=%p\n", privreq->req.buf);
  sam_putreg8(USBDEV_EPSTATUS_BK0RDY, SAM_USBDEV_EPSTATUSCLR(epno));

  return OK;
}

/****************************************************************************
 * Name: sam_req_cancel
 ****************************************************************************/

static void sam_req_cancel(struct sam_ep_s *privep, int16_t result)
{
  /* Complete every queued request with the specified status */

  while (!sam_rqempty(&privep->reqq))
    {
      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)),
               (sam_rqpeek(&privep->reqq))->req.xfrd);
      sam_req_complete(privep, result);
    }
}

/****************************************************************************
 * Name: sam_ep_configure_internal
 *
 * Description:
 *   This is the internal implementation of the endpoint configuration logic
 *   and implements the endpoint configuration method of the usbdev_ep_s
 *   interface.  As an internal interface, it will be used to configure
 *   endpoint 0 which is not available to the class implementation.
 *
 ****************************************************************************/

static int sam_ep_configure_internal(struct sam_ep_s *privep,
                                     const struct usb_epdesc_s *desc)
{
  uint8_t epconf;
  uint16_t maxpacket;
  uint8_t epno;
  uint8_t eptype;
  uint8_t intflags;
  uint32_t ephwsize;
  bool dirin;

  DEBUGASSERT(privep && privep->dev && desc);

  uinfo("len: 0x%02x type: 0x%02x addr: 0x%02x attr: 0x%02x "
        "maxpacketsize: 0x%02x%02x interval: 0x%02x\n",
        desc->len, desc->type, desc->addr, desc->attr,
        desc->mxpacketsize[1],  desc->mxpacketsize[0],
        desc->interval);

  /* Decode the endpoint descriptor */

  epno      = USB_EPNO(desc->addr);
  dirin     = (desc->addr & USB_DIR_MASK) == USB_REQ_DIR_IN;
  eptype    = (desc->attr & USB_EP_ATTR_XFERTYPE_MASK);
  maxpacket = GETUINT16(desc->mxpacketsize);

  if (maxpacket <= 8)
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_8B;
    }
  else if (maxpacket <= 16)
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_16B;
    }
  else if (maxpacket <= 32)
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_32B;
    }
  else if (maxpacket <= 64)
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_64B;
    }
  else if ((maxpacket <= 128) && (eptype == USB_EP_ATTR_XFER_ISOC))
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_128B;
    }
  else if ((maxpacket <= 256) && (eptype == USB_EP_ATTR_XFER_ISOC))
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_256B;
    }
  else if ((maxpacket <= 512) && (eptype == USB_EP_ATTR_XFER_ISOC))
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_512B;
    }
  else if ((maxpacket <= 1023) && (eptype == USB_EP_ATTR_XFER_ISOC))
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_1023B;
    }
  else
    {
      return -EINVAL;
    }

  /* update endpoint descriptors to correct size */

  privep->descb[0]->pktsize = ephwsize;
  privep->descb[1]->pktsize = ephwsize;

  /* Initialize the endpoint structure */

  privep->ep.eplog     = desc->addr;  /* Includes direction */
  privep->ep.maxpacket = maxpacket;
  privep->epstate      = USB_EPSTATE_IDLE;

  /* get current config IN and OUT */

  epconf = 0x00;
  sam_putreg8(0x00, SAM_USBDEV_EPCFG(epno));

  if (dirin)
    {
      /* Disable bank1 (IN) */

      intflags = USBDEV_EPINT_TRCPT1 | USBDEV_EPINT_STALL1;
    }
  else
    {
      /* Disable bank0 (OUT) */

      intflags = USBDEV_EPINT_TRCPT0 | USBDEV_EPINT_STALL0;
    }

  /* write back disabled config */

  sam_putreg8(0x7f, SAM_USBDEV_EPINTENCLR(epno));
  sam_putreg8(0x7f, SAM_USBDEV_EPINTFLAG(epno));

  /* Re-configure and enable the endpoint */

  switch (eptype)
    {
      case USB_EP_ATTR_XFER_CONTROL:
      {
        epconf = USBDEV_EPCCFG_EPTYPE0_CTRLOUT |
                 USBDEV_EPCCFG_EPTYPE1_CTRLIN;

        /* Also enable IN interrupts */

        intflags =  USBDEV_EPINT_TRCPT0 | USBDEV_EPINT_STALL0;
        intflags |= USBDEV_EPINT_TRCPT1 | USBDEV_EPINT_STALL1;
        intflags |= USBDEV_EPINT_RXSTP;
        sam_putreg8(USBDEV_EPSTATUS_BK0RDY, SAM_USBDEV_EPSTATUSSET(0));
        sam_putreg8(USBDEV_EPSTATUS_BK1RDY, SAM_USBDEV_EPSTATUSCLR(0));
      }
      break;

#ifdef CONFIG_USBDEV_ISOCHRONOUS
      case USB_EP_ATTR_XFER_ISOC:
        if (dirin)
          {
            epconf |= USBDEV_EPCCFG_EPTYPE1_ISOCIN;
          }
        else
          {
            epconf |= USBDEV_EPCCFG_EPTYPE0_ISOCOUT;
          }
      break;
#endif

      case USB_EP_ATTR_XFER_BULK:
        if (dirin)
          {
            epconf |= USBDEV_EPCCFG_EPTYPE1_BULKIN;
          }
        else
          {
            epconf |= USBDEV_EPCCFG_EPTYPE0_BULKOUT;
          }
        break;

      case USB_EP_ATTR_XFER_INT:
        if (dirin)
          {
            epconf |= USBDEV_EPCCFG_EPTYPE1_INTIN;
          }
        else
          {
            epconf |= USBDEV_EPCCFG_EPTYPE0_INTOUT;
          }
        break;

      default:
        usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADEPTYPE),
                 eptype >> USB_EP_ATTR_XFERTYPE_SHIFT);
        return -EINVAL;
    }

  sam_putreg8(epconf, SAM_USBDEV_EPCFG(epno));

  /* Enable endpoint interrupts */

  sam_putreg8(intflags, SAM_USBDEV_EPINTENSET(epno));
  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPCONF),  epno << 8 | epconf);
  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPINTEN), epno << 8 | intflags);

  sam_dumpep(privep->dev, epno);
  return OK;
}

/****************************************************************************
 * Name: sam_ep_reserve
 *
 * Description:
 *   Find and un-reserved endpoint number and reserve it for the caller.
 *
 ****************************************************************************/

static inline struct sam_ep_s *
sam_ep_reserve(struct sam_usbdev_s *priv, uint8_t epset)
{
  struct sam_ep_s *privep = NULL;
  irqstate_t flags;
  int epndx = 0;

  flags  = enter_critical_section();
  epset &= priv->epavail;
  if (epset)
    {
      /* Select the lowest bit in the set of matching, available endpoints
       * (skipping EP0)
       */

      for (epndx = 1; epndx < SAM_USB_NENDPOINTS; epndx++)
        {
          uint8_t bit = SAM_EP_BIT(epndx);
          if ((epset & bit) != 0)
            {
              /* Mark the endpoint no longer available */

              priv->epavail &= ~bit;

              /* And return the pointer to the standard endpoint
               * structure.
               */

              privep = &priv->eplist[epndx];
              break;
            }
        }
    }

  leave_critical_section(flags);
  return privep;
}

/****************************************************************************
 * Name: sam_ep_unreserve
 *
 * Description:
 *   The endpoint is no long in-used.  It will be unreserved and can be
 *   re-used if needed.
 *
 ****************************************************************************/

static inline void
sam_ep_unreserve(struct sam_usbdev_s *priv, struct sam_ep_s *privep)
{
  irqstate_t flags = enter_critical_section();
  priv->epavail   |= SAM_EP_BIT(USB_EPNO(privep->ep.eplog));
  leave_critical_section(flags);
}

/****************************************************************************
 * Endpoint operations
 ****************************************************************************/

/****************************************************************************
 * Name: sam_ep_configure
 *
 * Description:
 *   This is the endpoint configuration method of the usbdev_ep_s interface.
 *
 ****************************************************************************/

static int sam_ep_configure(struct usbdev_ep_s *ep,
                            const struct usb_epdesc_s *desc,
                            bool last)
{
  struct sam_ep_s *privep = (struct sam_ep_s *)ep;
  int ret;

  /* Verify parameters.  Endpoint 0 is not available at this interface */

#if defined(CONFIG_DEBUG_USB) || defined(CONFIG_USBDEV_TRACE)
  uint8_t epno = USB_EPNO(desc->addr);
  usbtrace(TRACE_EPCONFIGURE, (uint16_t)epno);

  DEBUGASSERT(ep && desc && epno > 0 && epno < SAM_USB_NENDPOINTS);
  DEBUGASSERT(epno == USB_EPNO(ep->eplog));
#endif

  /* This logic is implemented in sam_ep_configure_internal */

  ret = sam_ep_configure_internal(privep, desc);

  /* If this was the last endpoint, then the class driver is fully
   * configured.
   */

  if (ret == OK && last)
    {
      struct sam_usbdev_s *priv = privep->dev;

      /* Go to the configured state (we should have been in the addressed
       * state)
       */

      DEBUGASSERT(priv && priv->devstate == USB_DEVSTATE_ADDRESSED);
      priv->devstate = USB_DEVSTATE_CONFIGURED;
    }

  return ret;
}

/****************************************************************************
 * Name: sam_ep_disable
 *
 * Description:
 *   This is the disable() method of the USB device endpoint structure.
 *
 ****************************************************************************/

static int sam_ep_disable(struct usbdev_ep_s *ep)
{
  struct sam_ep_s *privep = (struct sam_ep_s *)ep;
  struct sam_usbdev_s *priv;
  irqstate_t flags;
  uint8_t epno;

#ifdef CONFIG_DEBUG_USB
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      uerr("ERROR: ep=%p\n", ep);
      return -EINVAL;
    }
#endif

  epno = USB_EPNO(ep->eplog);
  usbtrace(TRACE_EPDISABLE, epno);

  /* Reset the endpoint and cancel any ongoing activity */

  flags = enter_critical_section();
  priv  = privep->dev;
  sam_ep_reset(priv, epno);

  /* Revert to the addressed-but-not-configured state */

  sam_setdevaddr(priv, priv->devaddr);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_ep_allocreq
 *
 * Description:
 *   This is the allocreq() method of the USB device endpoint structure.
 *
 ****************************************************************************/

static struct usbdev_req_s *sam_ep_allocreq(struct usbdev_ep_s *ep)
{
  struct sam_req_s *privreq;

#ifdef CONFIG_DEBUG_USB
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif

  usbtrace(TRACE_EPALLOCREQ, USB_EPNO(ep->eplog));

  privreq = (struct sam_req_s *)kmm_zalloc(sizeof(struct sam_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  return &privreq->req;
}

/****************************************************************************
 * Name: sam_ep_freereq
 *
 * Description:
 *   This is the freereq() method of the USB device endpoint structure.
 *
 ****************************************************************************/

static void sam_ep_freereq(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct sam_req_s *privreq = (struct sam_req_s *)req;

#ifdef CONFIG_DEBUG_USB
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  usbtrace(TRACE_EPFREEREQ, USB_EPNO(ep->eplog));

  kmm_free(privreq);
}

/****************************************************************************
 * Name: sam_ep_allocbuffer
 *
 * Description:
 *   This is the allocbuffer() method of the USB device endpoint structure.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void *sam_ep_allocbuffer(struct usbdev_ep_s *ep, uint16_t nbytes)
{
  /* There is not special buffer allocation requirement */

  return kumm_malloc(nbytes);
}
#endif

/****************************************************************************
 * Name: sam_ep_freebuffer
 *
 * Description:
 *   This is the freebuffer() method of the USB device endpoint structure.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void sam_ep_freebuffer(struct usbdev_ep_s *ep, void *buf)
{
  /* There is not special buffer allocation requirement */

  kumm_free(buf);
}
#endif

/****************************************************************************
 * Name: sam_ep_submit
 *
 * Description:
 *   This is the submit() method of the USB device endpoint structure.
 *
 ****************************************************************************/

static int sam_ep_submit(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct sam_req_s *privreq = (struct sam_req_s *)req;
  struct sam_ep_s *privep = (struct sam_ep_s *)ep;
  struct sam_usbdev_s *priv;
  irqstate_t flags;
  uint8_t epno;
  int ret = OK;

#ifdef CONFIG_DEBUG_USB
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      uerr("ERROR: req=%p callback=%p buf=%p ep=%p\n", req, req->callback,
           req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, USB_EPNO(ep->eplog));
  priv = privep->dev;

#ifdef CONFIG_DEBUG_USB
  if (!priv->driver)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_NOTCONFIGURED),
               priv->usbdev.speed);
      uerr("ERROR: driver=%p\n", priv->driver);
      return -ESHUTDOWN;
    }
#endif

  /* Handle the request from the class driver */

  epno              = USB_EPNO(ep->eplog);
  req->result       = -EINPROGRESS;
  req->xfrd         = 0;
  privreq->inflight = 0;
  flags             = enter_critical_section();

  /* Handle IN (device-to-host) requests.  NOTE:  If the class device is
   * using the bi-directional EP0, then we assume that they intend the EP0
   * IN functionality (EP0 SETUP OUT data receipt does not use requests).
   */

  if (USB_ISEPIN(ep->eplog) || epno == EP0)
    {
      /* Check if the endpoint is stalled (or there is a stall pending) */

      if (privep->stalled || privep->pending)
        {
          /* Yes.. in this case, save the new they will get in a special
           * "pending" they will get queue until the stall is cleared.
           */

          uinfo("Pending stall clear\n");
          sam_req_enqueue(&privep->pendq, privreq);
          usbtrace(TRACE_INREQQUEUED(epno), req->len);
          ret = OK;
        }

      else
        {
          /* Add the new request to the request queue for the IN endpoint */

          sam_req_enqueue(&privep->reqq, privreq);
          usbtrace(TRACE_INREQQUEUED(epno), req->len);

          /* If the IN endpoint is IDLE and there is not write queue
           * processing in progress, then transfer the data now.
           */

          if (privep->epstate == USB_EPSTATE_IDLE && !privep->txbusy)
            {
              ret = sam_req_write(priv, privep);
            }
        }
    }

  /* Handle OUT (host-to-device) requests */

  else
    {
      /* Add the new request to the request queue for the OUT endpoint */

      sam_req_enqueue(&privep->reqq, privreq);
      usbtrace(TRACE_OUTREQQUEUED(epno), req->len);

      /* Check if we have stopped RX receipt due to lack of read
       * requests.  In that case we are not receiving anything from host.
       * and HW sends NAK to host. see sam_req_read()
       * so this "state" is actually not required (at least yet)
       */

      if (privep->epstate == USB_EPSTATE_RXSTOPPED)
        {
          privep->epstate = USB_EPSTATE_IDLE;
        }

      /* start new read if no active yet */

      if (!privep->rxactive)
        {
          ret = sam_req_read(priv, privep, 0);
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: sam_ep_cancel
 ****************************************************************************/

static int sam_ep_cancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct sam_ep_s *privep = (struct sam_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_USB
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPCANCEL, USB_EPNO(ep->eplog));

  flags = enter_critical_section();
  sam_req_cancel(privep, -EAGAIN);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_ep_stallresume
 ****************************************************************************/

static int sam_ep_stallresume(struct usbdev_ep_s *ep, bool resume)
{
  struct sam_ep_s *privep;
  uint8_t epno;
  irqstate_t flags;
  int ret;

#ifdef CONFIG_DEBUG_USB
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Handle the resume condition */

  privep = (struct sam_ep_s *)ep;
  if (resume)
    {
      ret = sam_ep_resume(privep);
    }

  /* Handle the stall condition */

  else
    {
      /* If this is an IN endpoint (and not EP0) and if there are queued
       * write requests, then we cannot stall now.  Perhaps this is a
       * protocol stall.  In that case, we will need to drain the write
       * requests before sending the stall.
       */

      flags = enter_critical_section();
      epno = USB_EPNO(ep->eplog);
      if (epno != 0 && USB_ISEPIN(ep->eplog))
        {
          /* Are there any unfinished write requests in the request
           * queue?
           */

          if (!sam_rqempty(&privep->reqq))
            {
              /* Just set a flag to indicate that the endpoint must be
               * stalled on the next TRCPTx interrupt when the request
               * queue becomes empty.
               */

              privep->pending = true;
              leave_critical_section(flags);
              return OK;
            }
        }

      /* Not an IN endpoint, endpoint 0, or no pending write requests.
       * Stall the endpoint now.
       */

      ret = sam_ep_stall(privep);
      leave_critical_section(flags);
    }

  return ret;
}

/****************************************************************************
 * Device Controller Operations
 ****************************************************************************/

/****************************************************************************
 * Name: sam_allocep
 *
 * Description:
 *   This is the allocep() method of the USB device driver interface
 *
 ****************************************************************************/

static struct usbdev_ep_s *sam_allocep(struct usbdev_s *dev, uint8_t epno,
                                         bool in, uint8_t eptype)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)dev;
  struct sam_ep_s *privep = NULL;
  uint16_t epset = SAM_EPSET_NOTEP0;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)epno);
#ifdef CONFIG_DEBUG_USB
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif

  /* Ignore any direction bits in the logical address */

  epno = USB_EPNO(epno);

  /* A logical address of 0 means that any endpoint will do */

  if (epno > 0)
    {
      /* Otherwise, we will return the endpoint structure only for the
       * requested 'logical' endpoint.  All of the other checks will still
       * be performed.
       *
       * First, verify that the logical endpoint is in the range supported by
       * by the hardware.
       */

      if (epno >= SAM_USB_NENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADEPNO), (uint16_t)epno);
          return NULL;
        }

      /* Convert the logical address to a physical OUT endpoint address and
       * remove all of the candidate endpoints from the bitset except for the
       * the IN/OUT pair for this logical address.
       */

      epset = SAM_EP_BIT(epno);
    }

  /* Check if the selected endpoint number is available */

  privep = sam_ep_reserve(priv, epset);
  if (!privep)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EPRESERVE), (uint16_t)epset);
      return NULL;
    }

  return &privep->ep;
}

/****************************************************************************
 * Name: sam_freeep
 *
 * Description:
 *   This is the freeep() method of the USB device driver interface
 *
 ****************************************************************************/

static void sam_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep)
{
  struct sam_usbdev_s *priv;
  struct sam_ep_s *privep;

#ifdef CONFIG_DEBUG_USB
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  priv   = (struct sam_usbdev_s *)dev;
  privep = (struct sam_ep_s *)ep;
  usbtrace(TRACE_DEVFREEEP, (uint16_t)USB_EPNO(ep->eplog));

  if (priv && privep)
    {
      /* Mark the endpoint as available */

      sam_ep_unreserve(priv, privep);
    }
}

/****************************************************************************
 * Name: sam_getframe
 *
 * Description:
 *   This is the getframe() method of the USB device driver interface
 *
 ****************************************************************************/

static int sam_getframe(struct usbdev_s *dev)
{
  uint32_t regval;
  uint16_t frameno;

#ifdef CONFIG_DEBUG_USB
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Return the last frame number detected by the hardware */

  regval  = sam_getreg16(SAM_USBDEV_FNUM);
  frameno = (regval & USBDEV_FNUM_MASK) >> USBDEV_FNUM_SHIFT;

  usbtrace(TRACE_DEVGETFRAME, frameno);
  return frameno;
}

/****************************************************************************
 * Name: sam_wakeup
 *
 * Description:
 *   This is the wakeup() method of the USB device driver interface
 *
 ****************************************************************************/

static int sam_wakeup(struct usbdev_s *dev)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)dev;
  irqstate_t flags;
  uint16_t regval;

  usbtrace(TRACE_DEVWAKEUP, 0);
#ifdef CONFIG_DEBUG_USB
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Resume normal operation */

  flags = enter_critical_section();
  sam_resume(priv);

  /* Activate a remote wakeup. (aka upstream resume)
   * Setting the Remote Wakeup bit in CTRLB.UPRSM starts the
   * Remote Wake Up procedure.
   *
   * This will automatically be done by the controller after 5 ms of
   * inactivity on the USB bus.
   *
   * When the controller sends the Upstream Resume INTFLAG.WAKEUP is set
   * and INTFLAG.SUSPEND is cleared.
   * The CTRLB.UPRSM is cleared at the end of the transmitting Upstream
   * Resume.
   */

  regval  = sam_getreg16(SAM_USBDEV_CTRLB);
  regval |= USBDEV_CTRLB_UPRSM;
  sam_putreg16(regval, SAM_USBDEV_CTRLB);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_selfpowered
 *
 * Description:
 *   This is the selfpowered() method of the USB device driver interface
 *
 ****************************************************************************/

static int sam_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG_USB
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/****************************************************************************
 * Suspend/Resume Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: sam_suspend
 ****************************************************************************/

static void sam_suspend(struct sam_usbdev_s *priv)
{
  /* Don't do anything if the device is already suspended */

  if (priv->devstate != USB_DEVSTATE_SUSPENDED)
    {
      /* Notify the class driver of the suspend event */

      if (priv->driver)
        {
          CLASS_SUSPEND(priv->driver, &priv->usbdev);
        }

      /* Switch to the Suspended state */

      priv->prevstate = priv->devstate;
      priv->devstate  = USB_DEVSTATE_SUSPENDED;

      /* Disable clocking to the USB peripheral */

      sam_disableclks();

      /* Let the board-specific logic know that we have entered the
       * suspend state.  This may trigger additional reduced power
       * consumption measures.
       */

      sam_usb_suspend((struct usbdev_s *)priv, false);
    }
}

/****************************************************************************
 * Name: sam_resume
 ****************************************************************************/

static void sam_resume(struct sam_usbdev_s *priv)
{
  /* This function is called when either (1) a WAKEUP interrupt is received
   * from the host PC, or (2) the class device implementation calls the
   * wakeup() method.
   */

  /* Don't do anything if the device was not suspended */

  if (priv->devstate == USB_DEVSTATE_SUSPENDED)
    {
      /* Revert to the previous state */

      priv->devstate = priv->prevstate;

      /* Restore clocking to the USB peripheral */

      sam_enableclks();

      /* Restore full power -- whatever that means for this
       * particular board
       */

      sam_usb_suspend((struct usbdev_s *)priv, true);

      /* Notify the class driver of the resume event */

      if (priv->driver)
        {
          CLASS_RESUME(priv->driver, &priv->usbdev);
        }
    }
}

/****************************************************************************
 * Initialization/Reset
 ****************************************************************************/

/****************************************************************************
 * Name: sam_reset
 ****************************************************************************/

static void sam_reset(struct sam_usbdev_s *priv)
{
  uint16_t regval;
  uint8_t epno;

  /* Make sure that clocking is enabled to the USB peripheral. */

  sam_enableclks();

  /* Tell the class driver that we are disconnected.  The class driver
   * should then accept any new configurations.
   */

  CLASS_DISCONNECT(priv->driver, &priv->usbdev);

  /* The device enters the Default state (un-addressed and un-configured) */

  priv->devaddr   = 0;
  sam_setdevaddr(priv, 0);

  priv->devstate  = USB_DEVSTATE_DEFAULT;

  /* Reset and disable all endpoints. Then re-configure EP0 */

  sam_epset_reset(priv, SAM_EPSET_ALL);
  sam_ep_configure_internal(&priv->eplist[EP0], &g_ep0desc);

  /* set EP0 waiting for SETUP */

  sam_ep0_ctrlread(priv);

  /* Reset endpoint data structures */

  for (epno = 0; epno < SAM_USB_NENDPOINTS; epno++)
    {
      struct sam_ep_s *privep = &priv->eplist[epno];

      /* Cancel any queued requests.  Since they are cancelled
       * with status -ESHUTDOWN, then will not be requeued
       * until the configuration is reset.  NOTE:  This should
       * not be necessary... the CLASS_DISCONNECT above should
       * result in the class implementation calling sam_ep_disable
       * for each of its configured endpoints.
       */

      sam_req_cancel(privep, -ESHUTDOWN);

      /* Reset endpoint status */

      privep->stalled   = false;
      privep->pending   = false;
      privep->halted    = false;
      privep->zlpsent   = false;
      privep->txbusy    = false;
      privep->rxactive  = false;
    }

  /* Re-configure the USB controller in its initial, unconnected state */

  priv->usbdev.speed = USB_SPEED_FULL;
  priv->usbdev.dualspeed = 0;

  /* Clear all pending interrupt status */

  regval = USBDEV_INT_SUSPEND | USBDEV_INT_SOF     | USBDEV_INT_EORST |
           USBDEV_INT_WAKEUP  | USBDEV_INT_EORSM   | USBDEV_INT_UPRSM |
           USBDEV_INT_RAMACER | USBDEV_INT_LPMNYET | USBDEV_INT_LPMSUSP;

  sam_putreg16(regval, SAM_USBDEV_INTFLAG);

  /* Enable normal operational interrupts
   * endpoint 0 is enabled on sam_ep_configure_internal()
   */

  regval = USBDEV_INT_SOF | USBDEV_INT_WAKEUP | USBDEV_INT_SUSPEND;
  sam_putreg16(regval, SAM_USBDEV_INTENSET);

  sam_dumpep(priv, EP0);
}

/****************************************************************************
 * Interrupt Level Processing
 ****************************************************************************/

/****************************************************************************
 * Name: sam_ep0_wrstatus
 *
 * Description:
 *   write ep0 status reply back to host
 *
 ****************************************************************************/

static void sam_ep0_wrstatus(struct sam_usbdev_s *priv,
                             const uint8_t *buffer, size_t buflen)
{
  uint32_t packetsize;

  /* we need to make copy of data as source is in stack
   * reusing the static ep0 setup buffer
   */

  DEBUGASSERT(buflen < SAM_EP0_MAXPACKET);
  memcpy(&priv->ep0out[0], buffer, buflen);

  /* set read for next setup OUT */

  sam_ep0_ctrlread(priv);

  /* setup TX transfer */

  priv->eplist[0].descb[1]->addr = (uint32_t) &priv->ep0out[0];
  uinfo("addr=%p\n", &priv->ep0out[0]);
  packetsize = priv->eplist[0].descb[1]->pktsize;
  packetsize &= ~USBDEV_PKTSIZE_BCNT_MASK;
  packetsize &= ~USBDEV_PKTSIZE_MPKTSIZE_MASK;
  packetsize |= USBDEV_PKTSIZE_BCNT(buflen);
  priv->eplist[0].descb[1]->pktsize = packetsize;

  /* Set BK1RDY to notify the USB hardware that TX data is ready on
   * descriptor bank1.  We will be notified that the descriptor has been
   * transmitted by the USB device when TRCPT1 in the endpoint's EPINTFLAG
   * register has been set.
   */

  sam_putreg8(USBDEV_EPSTATUS_BK1RDY, SAM_USBDEV_EPSTATUSSET(0));

  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0WRSTATUS), buflen);
}

/****************************************************************************
 * Name: sam_ep0_dispatch
 ****************************************************************************/

static void sam_ep0_dispatch(struct sam_usbdev_s *priv)
{
  uint8_t *dataout;
  size_t outlen;
  int ret;

  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_DISPATCH), 0);
  if (priv && priv->driver)
    {
      /* Assume IN SETUP (or OUT SETUP with no data) */

      dataout = NULL;
      outlen  = 0;

      /* Was this an OUT SETUP command? */

      if (USB_REQ_ISOUT(priv->ctrl.type))
        {
          uint16_t tmplen = GETUINT16(priv->ctrl.len);
          if (tmplen > 0)
            {
              dataout = priv->ep0out;
              outlen  = tmplen;
            }
        }

      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, &priv->ctrl,
                        dataout, outlen);
      if (ret < 0)
        {
          /* Stall on failure */

          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_DISPATCHSTALL), 0);
          sam_ep_stall(&priv->eplist[EP0]);
        }
    }
}

/****************************************************************************
 * Name: sam_setdevaddr
 *
 * Description:
 *   This function is called after the completion of the STATUS phase to
 *   instantiate the device address that was received during the SETUP
 *   phase.  This enters the ADDRESSED state from either the DEFAULT or the
 *   CONFIGURED states.
 *
 *   If called with address == 0, then function will revert to the DEFAULT,
 *   un-configured and un-addressed state.
 *
 ****************************************************************************/

static void sam_setdevaddr(struct sam_usbdev_s *priv, uint8_t address)
{
  uinfo("ENTRY address=0x%x\n", address);

  DEBUGASSERT(address <= 0x7f);
  if (address)
    {
      /* Enable the address */

      address |= USBDEV_DADD_ADDEN;
      sam_putreg8(address, SAM_USBDEV_DADD);

      /* Go to the addressed but not configured state */

      priv->devstate = USB_DEVSTATE_ADDRESSED;
    }
  else
    {
      /* Set address to zero. clear ADDEN bit */

      sam_putreg8(0x00, SAM_USBDEV_DADD);

      /* Revert to the un-addressed, default state */

      priv->devstate = USB_DEVSTATE_DEFAULT;
    }
}

/****************************************************************************
 * Name: sam_ep0_setup
 *
 * Description:
 *   This function is called after the receiving of the SETUP packet
 *   data is ready on usb_ctrlreq_s struct
 *
 ****************************************************************************/

static void sam_ep0_setup(struct sam_usbdev_s *priv)
{
  struct sam_ep_s    *ep0 = &priv->eplist[EP0];
  struct sam_ep_s    *privep;
  union wb_u          value;
  union wb_u          index;
  union wb_u          len;
  union wb_u          response;
  enum sam_ep0setup_e ep0result;
  uint8_t             epno;
  int                 nbytes = 0; /* Assume zero-length packet */
  int                 ret;

  /* Terminate any pending requests */

  sam_req_cancel(ep0, -EPROTO);

  /* Assume NOT stalled; no TX in progress */

  ep0->stalled  = false;
  ep0->pending  = false;
  ep0->epstate  = USB_EPSTATE_IDLE;

  /* And extract the little-endian 16-bit values to host order */

  value.w = GETUINT16(priv->ctrl.value);
  index.w = GETUINT16(priv->ctrl.index);
  len.w   = GETUINT16(priv->ctrl.len);

  uinfo("SETUP: type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        priv->ctrl.type, priv->ctrl.req, value.w, index.w, len.w);

  /* Dispatch any non-standard requests */

  if ((priv->ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_NOSTDREQ), priv->ctrl.type);

      /* Let the class implementation handle all non-standard requests */

      sam_ep0_dispatch(priv);
      return;
    }

  /* Handle standard request.  Pick off the things of interest to the
   * USB device controller driver; pass what is left to the class driver
   */

  ep0result = USB_EP0SETUP_SUCCESS;
  switch (priv->ctrl.req)
    {
    case USB_REQ_GETSTATUS:
      {
        /* type:  device-to-host; recipient = device, interface, endpoint
         * value: 0
         * index: zero interface endpoint
         * len:   2; data = status
         */

        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_GETSTATUS), priv->ctrl.type);
        if (len.w != 2 || (priv->ctrl.type & USB_REQ_DIR_IN) == 0 ||
            index.b[MSB] != 0 || value.w != 0)
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADEPGETSTATUS), 0);
            ep0result = USB_EP0SETUP_STALL;
          }
        else
          {
            switch (priv->ctrl.type & USB_REQ_RECIPIENT_MASK)
              {
               case USB_REQ_RECIPIENT_ENDPOINT:
                {
                  epno = USB_EPNO(index.b[LSB]);
                  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_GETSTATUS), epno);
                  if (epno >= SAM_USB_NENDPOINTS)
                    {
                      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADEPGETSTATUS),
                              epno);
                      ep0result = USB_EP0SETUP_STALL;
                    }
                  else
                    {
                      privep     = &priv->eplist[epno];
                      response.w = 0; /* Not stalled */
                      nbytes     = 2; /* Response size: 2 bytes */

                      if (privep->stalled)
                        {
                          /* Endpoint stalled */

                          response.b[LSB] = 1; /* Stalled */
                        }
                    }
                }
                break;

              case USB_REQ_RECIPIENT_DEVICE:
                {
                 if (index.w == 0)
                    {
                      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_DEVGETSTATUS),
                               0);

                      /* Features:  Remote Wakeup=YES; selfpowered=? */

                      response.w      = 0;
                      response.b[LSB] = (priv->selfpowered <<
                                         USB_FEATURE_SELFPOWERED) |
                                        (1 << USB_FEATURE_REMOTEWAKEUP);
                      nbytes          = 2; /* Response size: 2 bytes */
                    }
                  else
                    {
                      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADDEVGETSTATUS),
                               0);
                      ep0result = USB_EP0SETUP_STALL;
                    }
                }
                break;

              case USB_REQ_RECIPIENT_INTERFACE:
                {
                  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_IFGETSTATUS), 0);
                  response.w = 0;
                  nbytes     = 2; /* Response size: 2 bytes */
                }
                break;

              default:
                {
                  usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADGETSTATUS), 0);
                  ep0result = USB_EP0SETUP_STALL;
                }
                break;
              }
          }
      }
      break;

    case USB_REQ_CLEARFEATURE:
      {
        /* type:  host-to-device; recipient = device, interface or endpoint
         * value: feature selector
         * index: zero interface endpoint;
         * len:   zero, data = none
         */

        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_CLEARFEATURE),
                                 priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) !=
                               USB_REQ_RECIPIENT_ENDPOINT)
          {
            /* Let the class implementation handle all
             * recipients (except for the
             * endpoint recipient)
             */

            sam_ep0_dispatch(priv);
            ep0result = USB_EP0SETUP_DISPATCHED;
          }
        else
          {
            /* Endpoint recipient */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < SAM_USB_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = false;

                ret = sam_ep_resume(privep);
                if (ret < 0)
                  {
                    ep0result = USB_EP0SETUP_STALL;
                  }
              }
            else
              {
                usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADCLEARFEATURE), 0);
                ep0result = USB_EP0SETUP_STALL;
              }
          }
      }
      break;

    case USB_REQ_SETFEATURE:
      {
        /* type:  host-to-device; recipient = device, interface, endpoint
         * value: feature selector
         * index: zero interface endpoint;
         * len:   0; data = none
         */

        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_SETFEATURE),
                                 priv->ctrl.type);
        if (((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
                                USB_REQ_RECIPIENT_DEVICE) &&
            value.w == USB_FEATURE_TESTMODE)
          {
            /* Special case recipient=device test mode */

            uinfo("test mode: %d\n", index.w);
          }
        else if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) !=
                 USB_REQ_RECIPIENT_ENDPOINT)
          {
           /* The class driver handles all recipients
            * except recipient=endpoint
            */

            sam_ep0_dispatch(priv);
            ep0result = USB_EP0SETUP_DISPATCHED;
          }
        else
          {
            /* Handler recipient=endpoint */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < SAM_USB_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = true;

                ret = sam_ep_stall(privep);
                if (ret < 0)
                  {
                    ep0result = USB_EP0SETUP_STALL;
                  }
              }
            else
              {
                usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADSETFEATURE), 0);
                ep0result = USB_EP0SETUP_STALL;
              }
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

        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) !=
            USB_REQ_RECIPIENT_DEVICE ||
            index.w != 0 || len.w != 0 || value.w > 127)
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADSETADDRESS), 0);
            ep0result = USB_EP0SETUP_STALL;
          }
        else
          {
            /* Note that setting of the device address will be deferred.
             * A zero-length packet will be sent and the device address will
             * be set when the zero-length packet transfer completes.
             */

            usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0SETUPSETADDRESS),
                     value.w);

            priv->devaddr = value.w;
            ep0result     = USB_EP0SETUP_ADDRESS;
          }
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
        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_GETSETDESC),
                 priv->ctrl.type);

        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
            USB_REQ_RECIPIENT_DEVICE)
          {
            /* The request seems valid... let the class implementation
             * handle it.
             */

            sam_ep0_dispatch(priv);
            ep0result = USB_EP0SETUP_DISPATCHED;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADGETSETDESC), 0);
            ep0result = USB_EP0SETUP_STALL;
          }
      }
      break;

    case USB_REQ_GETCONFIGURATION:
      /* type:  device-to-host; recipient = device
       * value: 0;
       * index: 0;
       * len:   1; data = configuration value
       */

      {
        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_GETCONFIG),
                 priv->ctrl.type);

        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
            USB_REQ_RECIPIENT_DEVICE &&
            value.w == 0 && index.w == 0 && len.w == 1)
          {
            /* The request seems valid... let the class implementation
             * handle it.
             */

            sam_ep0_dispatch(priv);
            ep0result = USB_EP0SETUP_DISPATCHED;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADGETCONFIG), 0);
            ep0result = USB_EP0SETUP_STALL;
          }
      }
      break;

    case USB_REQ_SETCONFIGURATION:
      /* type:  host-to-device; recipient = device
       * value: configuration value
       * index: 0;
       * len:   0; data = none
       */

      {
        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_SETCONFIG),
                 priv->ctrl.type);

        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
            USB_REQ_RECIPIENT_DEVICE &&
            index.w == 0 && len.w == 0)
          {
            /* The request seems valid... let the class implementation
             * handle it.  If the class implementation accepts it new
             * configuration, it will call sam_ep_configure() to configure
             * the endpoints.
             */

            sam_ep0_dispatch(priv);
            ep0result = USB_EP0SETUP_DISPATCHED;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADSETCONFIG), 0);
            ep0result = USB_EP0SETUP_STALL;
          }
      }
      break;

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
        /* Let the class implementation handle the request */

        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_GETSETIF), priv->ctrl.type);
        sam_ep0_dispatch(priv);
        ep0result = USB_EP0SETUP_DISPATCHED;
      }
      break;

    case USB_REQ_SYNCHFRAME:
      /* type:  device-to-host; recipient = endpoint
       * value: 0
       * index: endpoint;
       * len:   2; data = frame number
       */

      {
        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_SYNCHFRAME), 0);
      }
      break;

    default:
      {
        usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDCTRLREQ),
                 priv->ctrl.req);
        ep0result = USB_EP0SETUP_STALL;
      }
      break;
    }

  /* Restrict the data length to the length requested in the setup packet */

  if (nbytes > len.w)
    {
      nbytes = len.w;
    }

  /* At this point, the request has been handled and there are three
   * (or four) possible outcomes:
   *
   * 1a. ep0result == USB_EP0SETUP_SUCCESS
   *
   *    The setup request was successfully handled above and a response
   *    packet must be sent (may be a zero length packet).
   *
   * 1b. ep0result == USB_EP0SETUP_ADDRESS
   *
   *    A special case is the case where epstate=USB_EPSTATE_EP0ADDRESS.
   *    This means that the above processing generated an additional state
   *    where we need to wait until we complete the status phase before
   *    applying the new device address.
   *
   * 2. ep0result == USB_EP0SETUP_DISPATCHED;
   *
   *    The request was forwarded to the class implementation.  In case,
   *    EP0 IN data may have already been sent and the EP0 IN response
   *    has already been queued?  Or perhaps the endpoint has already
   *    been stalled?  This is all under the control of the class driver.
   *
   *    NOTE that for the case of non-standard SETUP requested, those
   *    requests were forwarded to the class driver and we don't even get
   *    to this logic.
   *
   * 3. ep0result == USB_EP0SETUP_STALL;
   *
   *    An error was detected in either the above logic or by the class
   *    implementation logic.
   */

  switch (ep0result)
    {
      case USB_EP0SETUP_SUCCESS:
        {
          /* Send the response (might be a zero-length packet) */

          ep0->epstate = USB_EPSTATE_EP0STATUSIN;
          sam_ep0_wrstatus(priv, response.b, nbytes);
        }
        break;

      case USB_EP0SETUP_ADDRESS:
        {
          /* Send the response (might be a zero-length packet) */

          ep0->epstate = USB_EPSTATE_EP0ADDRESS;
          sam_ep0_wrstatus(priv, response.b, nbytes);
        }
        break;

      case USB_EP0SETUP_STALL:
        {
          /* Stall EP0 */

          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EP0SETUPSTALLED),
                   priv->ctrl.req);

          sam_ep_stall(&priv->eplist[EP0]);
        }
        break;

      case USB_EP0SETUP_DISPATCHED:
      default:
        break;
    }
}

/****************************************************************************
 * Name: sam_ep_trcpt_interrupt
 *
 * Description:
 *   Transmit completed on Bank 0/1
 *   Normal:
 *     OUT data transmit has been completed bank=0
 *   Ping-Pong:
 *     TODO:
 *
 ****************************************************************************/

static void sam_ep_trcpt_interrupt(struct sam_usbdev_s *priv,
                                   struct sam_ep_s *privep,
                                   uint32_t flags, int bank)
{
  uint32_t eptype;
  uint16_t pktsize;
  uint8_t epno;

  /* Get the endpoint type */

  epno   = USB_EPNO(privep->ep.eplog);
  eptype = sam_getreg8(SAM_USBDEV_EPCFG(epno)) & USBDEV_EPCFG_EPTYPE0_MASK;

  /* Are we receiving data for a read request?  EP0 does not receive data
   * using read requests.
   */

  /* Get the size of the packet that we just received */

  pktsize = privep->descb[bank]->pktsize & USBDEV_PKTSIZE_BCNT_MASK;
  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPTRCPT0_LEN), (uint16_t)pktsize);

  if (privep->epstate == USB_EPSTATE_IDLE && epno != 0)
    {
      /* continue processing the read request. */

      sam_req_read(priv, privep, pktsize);
    }

  /* Did we just receive the data associated with an OUT SETUP command? */

  else if (privep->epstate == USB_EPSTATE_EP0DATAOUT)
    {
      uint16_t len;

      DEBUGASSERT(epno == EP0 && bank == 0);

      /* Yes.. back to the IDLE state */

      privep->epstate = USB_EPSTATE_IDLE;

      /* Get the size that we expected to receive */

      len = GETUINT16(priv->ctrl.len);
      if (len == pktsize)
        {
          /* And handle the EP0 SETUP now. */

          sam_ep0_setup(priv);
        }
      else
        {
          /* Then stall. */

          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EP0SETUPOUTSIZE), pktsize);

          sam_ep_stall(privep);
        }
    }

  /* Check for a EP0 STATUS packet returned by the host at the end of a
   * SETUP status phase
   */

  else if ((eptype == USBDEV_EPCCFG_EPTYPE0_CTRLOUT) && pktsize == 0)
    {
      DEBUGASSERT(epno == EP0 && bank == 0);
    }

  /* Otherwise there is a problem.  Complain an clear the interrupt */

  else
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_RXDATABKERR), privep->epstate);
    }
}

/****************************************************************************
 * Name: sam_ep0_ctrlread
 *
 * Description:
 *   setup 8-byte read req for ep0-ctrl setup phase.
 *   data is received on priv->ep0out buffer.  This is notified by
 *   endpoint TRCPT0 interrupt
 *
 ****************************************************************************/

static void sam_ep0_ctrlread(struct sam_usbdev_s *priv)
{
  priv->eplist[0].descb[0]->addr = (uint32_t) &priv->ep0out[0];
  uinfo("addr=%p\n", &priv->ep0out[0]);
  priv->eplist[0].descb[0]->pktsize = USBDEV_PKTSIZE_MPKTSIZE(8) |
                                      USBDEV_PKTSIZE_SIZE_64B;
  sam_putreg8(USBDEV_EPSTATUS_BK0RDY, SAM_USBDEV_EPSTATUSCLR(0));
}

/****************************************************************************
 * Name: sam_ep_interrupt
 *
 * Description:
 *   Handle the USB endpoint interrupt
 *
 ****************************************************************************/

static void sam_ep_interrupt(struct sam_usbdev_s *priv, int epno)
{
  struct sam_ep_s *privep;
  uint16_t flags;

  DEBUGASSERT((unsigned)epno < SAM_USB_NENDPOINTS);

  /* Get the endpoint structure */

  privep = &priv->eplist[epno];

  /* Get the endpoint irq */

  flags = sam_getreg8(SAM_USBDEV_EPINTFLAG(epno));
  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPINTFLAGS), flags);

  /* TRCPT1: IN packet sent and acknowledged by the host */

  if ((flags & USBDEV_EPINT_TRCPT1) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPTRCPT1), flags);

      /* Clear the TRCPT1 interrupt */

      sam_putreg8(USBDEV_EPINT_TRCPT1, SAM_USBDEV_EPINTFLAG(epno));

      /* Sending state.  This is the completion of a "normal" write request
       * transfer.  In this case, we need to resume request processing in
       * order to send the next outgoing packet.
       */

      if (privep->epstate == USB_EPSTATE_SENDING ||
          privep->epstate == USB_EPSTATE_EP0STATUSIN)
        {
          /* Continue/resume processing the write requests */

          privep->epstate = USB_EPSTATE_IDLE;
          sam_req_write(priv, privep);
        }

      /* Setting of the device address is a special case.  The address was
       * obtained when a preceding SETADDRESS SETUP command was processed.
       * But the address is not set until the final SETUP status phase
       * completes.  This interrupt indicates the completion of that status
       * phase and now we set the address.
       */

      else if (privep->epstate == USB_EPSTATE_EP0ADDRESS)
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_ADDRESSED), priv->devaddr);
          DEBUGASSERT(epno == EP0);

          /* Set the device address */

          privep->epstate = USB_EPSTATE_IDLE;
          sam_setdevaddr(priv, priv->devaddr);
        }
      else
        {
          /* Unexpected TRCPT1 interrupt */

          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_TXCOMPERR), privep->epstate);
        }
    }

  /* OUT packet received  */

  if ((flags & USBDEV_EPINT_TRCPT0) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPTRCPT0), flags);

      /* Clear the TRCPT0 interrupt. */

      sam_putreg8(USBDEV_EPINT_TRCPT0, SAM_USBDEV_EPINTFLAG(epno));

      sam_ep_trcpt_interrupt(priv, privep, flags, 0);
    }

  /* Endpoint stall */

  if ((flags & USBDEV_EPINT_STALL0) != 0)
    {
      sam_putreg8(USBDEV_EPINT_STALL0, SAM_USBDEV_EPINTFLAG(epno));
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPSTALL0), flags);

      /* If EP is not halted, clear STALL */

      if (privep->epstate != USB_EPSTATE_STALLED)
        {
          sam_putreg8(USBDEV_EPSTATUS_STALLRQ0,
                      SAM_USBDEV_EPSTATUSCLR(epno));
        }
    }

  if ((flags & USBDEV_EPINT_STALL1) != 0)
    {
      sam_putreg8(USBDEV_EPINT_STALL1, SAM_USBDEV_EPINTFLAG(epno));
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPSTALL1), flags);

      /* If EP is not halted, clear STALL */

      if (privep->epstate != USB_EPSTATE_STALLED)
        {
          sam_putreg8(USBDEV_EPSTATUS_STALLRQ1,
                      SAM_USBDEV_EPSTATUSCLR(epno));
        }
    }

  /* Transmit FAIL! */

  if ((flags & USBDEV_EPINT_TRFAIL0) != 0)
    {
      sam_putreg8(USBDEV_EPINT_TRFAIL0, SAM_USBDEV_EPINTFLAG(epno));
      privep->descb[0]->stausbk &= ~USBDEV_STATUSBK_ERRORFLOW;
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPTRFAIL0), flags);
    }

  if ((flags & USBDEV_EPINT_TRFAIL1) != 0)
    {
      sam_putreg8(USBDEV_EPINT_TRFAIL1, SAM_USBDEV_EPINTFLAG(epno));
      privep->descb[1]->stausbk &= ~USBDEV_STATUSBK_ERRORFLOW;
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPTRFAIL1), flags);
    }

  /* SETUP packet received */

  if ((flags & USBDEV_EPINT_RXSTP) != 0)
    {
      uint16_t len;

      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPRXSTP), flags);

      /* If a write request transfer was pending, complete it. */

      if (privep->epstate == USB_EPSTATE_SENDING)
        {
          sam_req_complete(privep, -EPROTO);
        }

      /* SETUP data is ready in the ep0out buffer. */

      memcpy((uint8_t *)&priv->ctrl, (uint8_t *)&priv->ep0out[0],
             USB_SIZEOF_CTRLREQ);

      /* Check for a SETUP IN transaction with data. */

      len = GETUINT16(priv->ctrl.len);
      if (USB_REQ_ISOUT(priv->ctrl.type) && len > 0)
        {
          /* Yes.. then we have to wait for the OUT data phase to complete
           * before processing the SETUP command.
           */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0SETUPOUT),
                   priv->ctrl.req);
          privep->epstate = USB_EPSTATE_EP0DATAOUT;

          /* Clear the RXSTP indication. */

          sam_putreg8(USBDEV_EPINT_RXSTP, SAM_USBDEV_EPINTFLAG(epno));
        }
      else
        {
          /* This is an SETUP IN command (or a SETUP IN with no data). */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0SETUPIN), len);
          privep->epstate = USB_EPSTATE_IDLE;

          /* Clear the RXSTP indication. */

          sam_putreg8(USBDEV_EPINT_RXSTP, SAM_USBDEV_EPINTFLAG(epno));

          /* Handle the SETUP OUT command now */

          sam_ep0_setup(priv);
        }

      /* ready for next setup data */

      sam_ep0_ctrlread(priv);
    }
}

/****************************************************************************
 * Name: sam_usb_interrupt
 *
 * Description:
 *   Handle the USB interrupt.
 *   Device Mode only
 *
 ****************************************************************************/

static int sam_usb_interrupt(int irq, void *context, void *arg)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)arg;
  uint16_t isr;
  uint16_t pending;
  uint16_t regval;
  uint16_t pendingep;
  int i;

  /* Get the set of pending device interrupts */

  isr     = sam_getreg16(SAM_USBDEV_INTFLAG);
  regval  = sam_getreg16(SAM_USBDEV_INTENSET);
  pending = isr & regval;

  /* Get the set of pending enpoint interrupts */

  pendingep = sam_getreg16(SAM_USBDEV_EPINTSMRY);

  /* Handle all pending USB interrupts */

  /* Serve Endpoint Interrupts first */

  if (pendingep)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_PENDING_EP), pendingep);

      for (i = 0; i < SAM_USB_NENDPOINTS; i++)
        {
          if ((pendingep & USBDEV_EPINTSMRY_EPINT(i)))
            {
              usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPNO), (uint16_t)i);
              sam_ep_interrupt(priv, i);
            }
        }
    }

  /* Suspend, treated last */

  if (pending == USBDEV_INT_SUSPEND)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_SUSPEND), pending);

      /* Enable wakeup interrupts */

      sam_putreg16(USBDEV_INT_SUSPEND, SAM_USBDEV_INTENCLR);
      sam_putreg16(USBDEV_INT_UPRSM | USBDEV_INT_WAKEUP | USBDEV_INT_EORSM,
                  SAM_USBDEV_INTENSET);

      /* Clear the pending suspend (and any wakeup) interrupts */

      sam_putreg16(USBDEV_INT_SUSPEND | USBDEV_INT_WAKEUP,
                   SAM_USBDEV_INTFLAG);

      /* Perform board-specific suspend operations.
       *
       * The USB device peripheral clocks can be switched off.
       * Resume event is asynchronously detected. MCK and USBCK can be
       * switched off in the Power Management controller and
       * Other board-specific operations could also be performed.
       */
    }

  /* SOF interrupt */

  if ((pending & USBDEV_INT_SOF) != 0)
    {
      /* Clear the pending SOF interrupt */

      sam_putreg16(USBDEV_INT_SOF, SAM_USBDEV_INTFLAG);

      /* TODO: do we need check frame errors FNUM.FNCERR */
    }

  /* Resume or wakeup.  REVISIT:  Treat the same? */

  if ((pending & (USBDEV_INT_WAKEUP | USBDEV_INT_EORSM)) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_WAKEUP), (uint16_t)pending);
      sam_resume(priv);

      /* Clear the pending wakeup, resume, (and any suspend) interrupts */

      sam_putreg16(USBDEV_INT_WAKEUP | USBDEV_INT_EORSM |
                   USBDEV_INT_SUSPEND, SAM_USBDEV_INTFLAG);

      /* Disable wakup and endofresume Enable suspend interrupt */

      sam_putreg16(USBDEV_INT_WAKEUP |
                   USBDEV_INT_EORSM, SAM_USBDEV_INTENCLR);
      sam_putreg16(USBDEV_INT_SUSPEND, SAM_USBDEV_INTENSET);
    }

  /* End of Reset. Set by hardware when an End Of Reset has been
   * detected by the USB controller.
   */

  if ((pending & USBDEV_INT_EORST) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EORST), pending);

      /* Clear the end-of-reset interrupt */

      sam_putreg16(USBDEV_INT_EORST, SAM_USBDEV_INTFLAG);

      /* Handle the reset */

      sam_reset(priv);

      /* REVISIT: Set the device speed Why here ?? */

      priv->usbdev.speed = USB_SPEED_FULL;
      priv->usbdev.dualspeed = 0;
    }

  return OK;
}

/****************************************************************************
 * Endpoint Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: sam_ep_reset
 *
 * Description:
 *   Reset and disable one endpoints.
 *
 ****************************************************************************/

static void sam_ep_reset(struct sam_usbdev_s *priv, uint8_t epno)
{
  struct sam_ep_s *privep = &priv->eplist[epno];

  /* Disable endpoint interrupts */

  sam_putreg8(0x7f, SAM_USBDEV_EPINTENCLR(epno));
  sam_putreg8(0x7f, SAM_USBDEV_EPINTFLAG(epno));

  /* Cancel any queued requests.  Since they are cancelled with status
   * -ESHUTDOWN, then will not be requeued until the configuration is reset.
   * NOTE:  This should not be necessary... the CLASS_DISCONNECT above
   * should result in the class implementation calling sam_ep_disable
   * for each of its configured endpoints.
   */

  sam_req_cancel(privep, -ESHUTDOWN);

  /* Reset endpoint status */

  privep->epstate   = USB_EPSTATE_DISABLED;
  privep->stalled   = false;
  privep->pending   = false;
  privep->halted    = false;
  privep->zlpsent   = false;
  privep->txbusy    = false;
  privep->rxactive  = false;
}

/****************************************************************************
 * Name: sam_epset_reset
 *
 * Description:
 *   Reset and disable a set of endpoints.
 *
 ****************************************************************************/

static void sam_epset_reset(struct sam_usbdev_s *priv, uint16_t epset)
{
  uint32_t bit;
  int epno;

  /* Reset each endpoint in the set */

  for (epno = 0, bit = 1, epset &= SAM_EPSET_ALL;
       epno < SAM_USB_NENDPOINTS && epset != 0;
       epno++, bit <<= 1)
    {
      /* Is this endpoint in the set? */

      if ((epset & bit) != 0)
        {
          /* Yes.. reset and disable it */

          sam_ep_reset(priv, epno);
          epset &= ~bit;
        }
    }
}

/****************************************************************************
 * Name: sam_ep_stall
 ****************************************************************************/

static int sam_ep_stall(struct sam_ep_s *privep)
{
  irqstate_t flags;
  uint8_t epno;

  /* Check that endpoint is in Idle state */

  DEBUGASSERT(/* privep->epstate == UDP_EPSTATE_IDLE && */ privep->dev);

  /* Check that endpoint is enabled and not already in Halt state */

  flags = enter_critical_section();
  if ((privep->epstate != USB_EPSTATE_DISABLED) &&
      (privep->epstate != USB_EPSTATE_STALLED))
    {
      epno = USB_EPNO(privep->ep.eplog);
      usbtrace(TRACE_EPSTALL, epno);

      /* If this is an IN endpoint (or endpoint 0), then cancel any
       * write requests in progress.
       */

      if (epno == 0 || USB_ISEPIN(privep->ep.eplog))
        {
          sam_req_cancel(privep, -EPERM);
        }

      /* Put endpoint into stalled state */

      privep->epstate = USB_EPSTATE_STALLED;
      privep->stalled = true;
      privep->pending = false;

      sam_putreg8(USBDEV_EPSTATUS_STALLRQ0 | USBDEV_EPSTATUS_STALLRQ1,
                  SAM_USBDEV_EPSTATUSSET(epno));
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_ep_resume
 ****************************************************************************/

static int sam_ep_resume(struct sam_ep_s *privep)
{
  struct sam_usbdev_s *priv;
  struct sam_req_s *req;
  irqstate_t flags;
  uint8_t epno;

  /* Check that endpoint is in Idle state */

  DEBUGASSERT(/* privep->epstate == UDP_EPSTATE_IDLE && */ privep->dev);

  flags = enter_critical_section();

  /* Check if the endpoint is stalled */

  if (privep->epstate == USB_EPSTATE_STALLED)
    {
      epno = USB_EPNO(privep->ep.eplog);
      usbtrace(TRACE_EPRESUME, epno);

      priv = (struct sam_usbdev_s *)privep->dev;

      /* Return endpoint to Idle state */

      privep->stalled = false;
      privep->pending = false;
      privep->epstate = USB_EPSTATE_IDLE;

      /* Clear STALLRQx request and reset data toggle */

      if (USB_ISEPIN(privep->ep.eplog))
        {
          sam_putreg8(USBDEV_EPSTATUS_STALLRQ1,
                      SAM_USBDEV_EPSTATUSCLR(epno));
          sam_putreg8(USBDEV_EPSTATUS_DTGLIN,
                      SAM_USBDEV_EPSTATUSCLR(epno));
          sam_putreg8(USBDEV_EPINT_STALL1,
                      SAM_USBDEV_EPINTFLAG(epno));
        }
      else
        {
          sam_putreg8(USBDEV_EPSTATUS_STALLRQ0,
                      SAM_USBDEV_EPSTATUSCLR(epno));
          sam_putreg8(USBDEV_EPSTATUS_DTGLOUT,
                      SAM_USBDEV_EPSTATUSCLR(epno));
          sam_putreg8(USBDEV_EPINT_STALL0,
                      SAM_USBDEV_EPINTFLAG(epno));
        }

      /* Copy any requests in the pending request queue to the working
       * request queue.
       */

      while ((req = sam_req_dequeue(&privep->pendq)) != NULL)
        {
          sam_req_enqueue(&privep->reqq, req);
        }

      /* Resuming any blocked data transfers on the endpoint */

      if (epno == 0 || USB_ISEPIN(privep->ep.eplog))
        {
          /* IN endpoint (or EP0).  Restart any queued write requests */

          sam_req_write(priv, privep);
        }
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_pullup
 *
 * Description:
 *   This is the pullup() method of the USB device driver interface
 *
 ****************************************************************************/

static int sam_pullup(struct usbdev_s *dev, bool enable)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)dev;
  uint32_t regval;

  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  /* Enable/disable the USB pull-up resistor */

  regval = sam_getreg16(SAM_USBDEV_CTRLB);

  if (enable)
    {
      /* Connect the 1.5 KOhm integrated pull-up on USB */

      regval &= ~USBDEV_CTRLB_DETACH;
    }
  else
    {
      /* Disconnect the 1.5 KOhm integrated pull-up on USB */

      regval |= USBDEV_CTRLB_DETACH;

      /* Device returns to the Powered state */

      if (priv->devstate > USB_DEVSTATE_POWERED)
        {
          priv->devstate = USB_DEVSTATE_POWERED;
        }
    }

  sam_putreg16((uint16_t)regval, SAM_USBDEV_CTRLB);

  return OK;
}

/****************************************************************************
 * Initialization/Reset
 ****************************************************************************/

/****************************************************************************
 * Name: sam_hw_setup
 ****************************************************************************/

static void sam_hw_setup(struct sam_usbdev_s *priv)
{
  int i;
  uint16_t regval;
  uint32_t padcalib;

  uint8_t calib_transn;
  uint8_t calib_transp;
  uint8_t calib_trim;

  /* Set up the USB DP/DM pins */

  sam_portconfig(PORT_USB_DP);
  sam_portconfig(PORT_USB_DM);

  /* To use the USB, the programmer must first configure the USB clock
   * input,
   */

  sam_enableclks();

  /* full reset USB */

  sam_ctrla_write(USB_CTRLA_SWRST);
  while (sam_getreg32(SAM_USB_SYNCBUSY) == USB_SYNCBUSY_SWRST);

  /* Enable USB core */

  sam_ctrla_write(USB_CTRLA_ENABLE | USB_CTRLA_RUNSTBY |
                                     USB_CTRLA_MODE_DEVICE);
  while (sam_getreg32(SAM_USB_SYNCBUSY) == USB_SYNCBUSY_ENABLE);

  /* Load USB factory calibration values from NVRAM */

  calib_transn = (getreg32(SAM_FUSES_USBTRANSN_ADDR) &
                  SAM_FUSES_USBTRANSN_MASK) >> SAM_FUSES_USBTRANSN_SHIFT;
  if (calib_transn == 0 || calib_transn == 0x1f)
    calib_transn = 0x9;

  calib_transp = (getreg32(SAM_FUSES_USBTRANSP_ADDR) &
                  SAM_FUSES_USBTRANSP_ADDR) >> SAM_FUSES_USBTRANSP_SHIFT;
  if (calib_transp == 0 || calib_transp == 0x1f)
    calib_transp = 0x19;

  calib_trim   = (getreg32(SAM_FUSES_USBTRIM_ADDR) &
                  SAM_FUSES_USBTRIM_MASK) >> SAM_FUSES_USBTRIM_SHIFT;
  if (calib_trim == 0 || calib_trim == 0x7)
    calib_trim = 0x6;

  padcalib     = USB_PADCAL_TRANSP(calib_transp) |
                 USB_PADCAL_TRANSN(calib_transn) |
                 USB_PADCAL_TRIM(calib_trim);

  sam_putreg16(padcalib, SAM_USB_PADCAL);
  uinfo("PADCAL: 0x%x\n", padcalib);

  /* set config
   * NREPLY = Any transaction to endpoint 0 will be ignored except SETUP
   *          cleared by hardware when receiving a SETUP packet.
   * DETACH = The internal device pull-ups are disabled
   */

  regval = USBDEV_CTRLB_NREPLY | USBDEV_CTRLB_DETACH;

  /* do we need config to set LOW_SPEED mode? */

  regval |= USBDEV_CTRLB_SPDCONF_FULL;

  sam_putreg16(regval, SAM_USBDEV_CTRLB);

  /* Reset and disable endpoints */

  sam_epset_reset(priv, SAM_EPSET_ALL);

  /* Initialize Endpoints */

  for (i = 0; i < SAM_USB_NENDPOINTS; i++)
    {
      /* Reset endpoint configuration */

      sam_putreg8(0, SAM_USBDEV_EPCFG(i));
    }

  /* Init descriptor base address */

  sam_putreg32((uint32_t)&priv->ep_descriptors, SAM_USB_DESCADD);

  /* clear all previous descriptor data so no accidental
   * DMA transfers could happen.
   */

  memset((uint8_t *)(&priv->ep_descriptors[0]), 0,
         sizeof(priv->ep_descriptors));

  /* Disable all interrupts */

  sam_putreg16(USBDEV_INT_SUSPEND | USBDEV_INT_SOF     | USBDEV_INT_EORST |
               USBDEV_INT_WAKEUP  | USBDEV_INT_EORSM   | USBDEV_INT_UPRSM |
               USBDEV_INT_RAMACER | USBDEV_INT_LPMNYET | USBDEV_INT_LPMSUSP,
               SAM_USBDEV_INTENCLR);

  sam_pullup(&priv->usbdev, false);
}

/****************************************************************************
 * Name: sam_hw_shutdown
 ****************************************************************************/

static void sam_hw_shutdown(struct sam_usbdev_s *priv)
{
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable all interrupts */

  sam_putreg16(USBDEV_INT_SUSPEND | USBDEV_INT_SOF     | USBDEV_INT_EORST |
               USBDEV_INT_WAKEUP  | USBDEV_INT_EORSM   | USBDEV_INT_UPRSM |
               USBDEV_INT_RAMACER | USBDEV_INT_LPMNYET | USBDEV_INT_LPMSUSP,
               SAM_USBDEV_INTENCLR);

  /* Clear all pending interrupt status */

  sam_putreg16(USBDEV_INT_SUSPEND | USBDEV_INT_SOF     | USBDEV_INT_EORST |
               USBDEV_INT_WAKEUP  | USBDEV_INT_EORSM   | USBDEV_INT_UPRSM |
               USBDEV_INT_RAMACER | USBDEV_INT_LPMNYET | USBDEV_INT_LPMSUSP,
               SAM_USBDEV_INTFLAG);

  /* Disconnect the device / disable the pull-up */

  sam_pullup(&priv->usbdev, false);

  /* Disable clocking to the UDP peripheral */

  sam_disableclks();
}

/****************************************************************************
 * Name: sam_sw_setup
 ****************************************************************************/

static void sam_sw_setup(struct sam_usbdev_s *priv)
{
  int epno;

  /* Initialize the device state structure.  NOTE: many fields
   * have the initial value of zero and, hence, are not explicitly
   * initialized here.
   */

  memset(priv, 0, sizeof(struct sam_usbdev_s));
  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->eplist[EP0].ep;
  priv->epavail    = SAM_EPSET_ALL & ~SAM_EP_BIT(EP0);
  priv->devstate   = USB_DEVSTATE_SUSPENDED;
  priv->prevstate  = USB_DEVSTATE_POWERED;

  /* Initialize the endpoint list */

  for (epno = 0; epno < SAM_USB_NENDPOINTS; epno++)
    {
      /* Set endpoint operations, reference to driver structure (not
       * really necessary because there is only one controller), and
       * the (physical) endpoint number which is just the index to the
       * endpoint.
       */

      priv->eplist[epno].ep.ops    = &g_epops;
      priv->eplist[epno].dev       = priv;
      priv->eplist[epno].ep.eplog  = epno;

      /* We will use a maxpacket size for supported for each endpoint */

      priv->eplist[epno].ep.maxpacket = SAM_USB_MAXPACKETSIZE(epno);

      /* set descriptor addresses */

      priv->eplist[epno].descb[0] = &priv->ep_descriptors[(epno << 1)];
      priv->eplist[epno].descb[1] = &priv->ep_descriptors[(epno << 1) + 1];
    }

  /* Select a smaller endpoint size for EP0 */

#if SAM_EP0_MAXPACKET < 64
  priv->eplist[EP0].ep.maxpacket = SAM_EP0_MAXPACKET;
#endif
}

/****************************************************************************
 * Name: sam_sw_shutdown
 ****************************************************************************/

static void sam_sw_shutdown(struct sam_usbdev_s *priv)
{
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method
 *   will be called to bind it to a USB device driver.
 *
 ****************************************************************************/

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct sam_usbdev_s *priv = &g_usbd;
  int ret;

  uinfo("driver: 0x%x\n", driver);
  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG_USB
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->disconnect || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  priv->driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &priv->usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BINDFAILED), (uint16_t)-ret);
      priv->driver = NULL;
    }
  else
    {
      /* Enable USB controller interrupts at the NVIC. */

      sam_hw_setup(priv);
      up_enable_irq(SAM_IRQ_USB);
      up_enable_irq(SAM_IRQ_USBSOF);
      up_enable_irq(SAM_IRQ_USBTRCPT0);
      up_enable_irq(SAM_IRQ_USBTRCPT1);

      /* Enable EORST irq */

      sam_putreg16(USBDEV_INT_EORST, SAM_USBDEV_INTENSET);

      /* Enable pull-up to connect the device.  The host should enumerate us
       * some time after this.  The next thing we expect is the EORST
       * interrupt.
       */

      sam_pullup(&priv->usbdev, true);
      priv->usbdev.speed = USB_SPEED_FULL;
      priv->usbdev.dualspeed = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver. If the USB device is connected to a
 *   USB host, it will first disconnect().  The driver is also requested to
 *   unbind() and clean up any device state, before this procedure finally
 *   returns.
 *
 ****************************************************************************/

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct sam_usbdev_s *priv = &g_usbd;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG_USB
  if (driver != priv->driver)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Reset the hardware and cancel all requests.  All requests must be
   * canceled while the class driver is still bound.
   */

  flags = enter_critical_section();

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable USB controller interrupts (but keep them attached) */

  up_disable_irq(SAM_IRQ_USB);
  up_disable_irq(SAM_IRQ_USBSOF);
  up_disable_irq(SAM_IRQ_USBTRCPT0);
  up_disable_irq(SAM_IRQ_USBTRCPT1);

  /* Put the hardware in an inactive state.  Then bring the hardware back up
   * in the initial state.  This is essentially the same state as we were
   * in when arm_usbinitialize() was first called.
   */

  sam_hw_shutdown(priv);
  sam_sw_shutdown(priv);

  sam_sw_setup(priv);
  sam_hw_setup(priv);

  /* Unhook the driver */

  priv->driver = NULL;
  leave_critical_section(flags);
  return OK;
}

void sam_usb_suspend(struct usbdev_s *dev, bool resume)
{
}

void arm_usbuninitialize(void)
{
  uinfo("arm_usbuninitialize()\n");
}

void arm_usbinitialize(void)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct sam_usbdev_s *priv = &g_usbd;

  uinfo("INIT\n");
  usbtrace(TRACE_DEVINIT, 0);

  /* Software initialization */

  sam_sw_setup(priv);

  /* Power up and initialize USB controller.  Interrupt from the USB
   * controller is initialized here, but will not be enabled at the AIC
   * until the class driver is installed.
   */

  sam_hw_setup(priv);

  /* Attach USB controller interrupt handlers.  The hardware will not be
   * initialized and interrupts will not be enabled until the class device
   * driver is bound.  Getting the IRQs here only makes sure that we have
   * them when we need them later.
   */

  if (irq_attach(SAM_IRQ_USB, sam_usb_interrupt, priv) != 0)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_IRQREGISTRATION),
                                     (uint16_t)SAM_IRQ_USB);
      goto errout;
    }

  if (irq_attach(SAM_IRQ_USBSOF, sam_usb_interrupt, priv) != 0)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_IRQREGISTRATION),
                                  (uint16_t)SAM_IRQ_USBSOF);
      goto errout;
    }

  if (irq_attach(SAM_IRQ_USBTRCPT0, sam_usb_interrupt, priv) != 0)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_IRQREGISTRATION),
                                  (uint16_t)SAM_IRQ_USBTRCPT0);
      goto errout;
    }

  if (irq_attach(SAM_IRQ_USBTRCPT1, sam_usb_interrupt, priv) != 0)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_IRQREGISTRATION),
                                  (uint16_t)SAM_IRQ_USBTRCPT1);
      goto errout;
    }

  return;

errout:
  arm_usbuninitialize();
}
#endif /* CONFIG_USBDEV */

#ifdef CONFIG_USBHOST

/****************************************************************************
 * Name: sam_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 ****************************************************************************/

static inline uint16_t sam_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: sam_add_sof_user
 *
 * Description:
 *   Add one SOF IRQ user and enable SOF interrupt
 *
 ****************************************************************************/

static void sam_add_sof_user(struct sam_usbhost_s *priv)
{
  priv->n_sof_user++;
  sam_putreg16(USBHOST_INT_HSOF, SAM_USBHOST_INTENSET);
}

/****************************************************************************
 * Name: sam_pipe_alloc
 *
 * Description:
 *   Allocate a pipe.
 *
 ****************************************************************************/

static int sam_pipe_alloc(struct sam_usbhost_s *priv)
{
  int idx;

  /* Search the table of pipes */

  for (idx = 0; idx < SAM_USB_NENDPOINTS; idx++)
    {
      /* Is this pipe available? */

      if (!priv->pipelist[idx].inuse)
        {
          /* Yes... make it "in use" and return the index */

          priv->pipelist[idx].inuse = true;
          return idx;
        }
    }

  /* All of the pipes are "in-use" */

  return -EBUSY;
}

/****************************************************************************
 * Name: sam_pipe_free
 *
 * Description:
 *   Free a previoiusly allocated pipe.
 *
 ****************************************************************************/

static void sam_pipe_free(struct sam_usbhost_s *priv, int idx)
{
  struct sam_pipe_s *pipe = &priv->pipelist[idx];

  uinfo("pipe%d\n", idx);
  DEBUGASSERT((unsigned)idx < SAM_USB_NENDPOINTS);

  /* Halt the pipe */

  sam_putreg8(0, SAM_USBHOST_PCFG(pipe->idx));

  /* Mark the pipe available */

  priv->pipelist[idx].inuse = false;
}

/* Look up table PSIZE -> size of bytes */

static const uint16_t psize_2_size[] =
{
  8,
  16,
  32,
  64,
  128,
  256,
  512,
  1024
};

/****************************************************************************
 * Name: sam_get_psize
 *
 * Description:
 *   Convert bank size of bytes to PIPCFG.PSIZE -> size Size of bytes
 *
 ****************************************************************************/

int8_t sam_get_psize(uint16_t size)
{
  uint8_t i;

  for (i = 0; i < sizeof(psize_2_size) / sizeof(uint16_t); i++)
    {
      /* Size should be exactly PSIZE values */

      if (size <= psize_2_size[i])
      return i;
    }

  return 7;
}

/****************************************************************************
 * Name: sam_pipe_configure
 *
 * Description:
 *   Configure or re-configure a host pipe.  Host pipes are configured
 *   when pipe is allocated and EP0 (only) is re-configured with the
 *   max packet size or device address changes.
 *
 ****************************************************************************/

static void sam_pipe_configure(struct sam_usbhost_s *priv, int idx)
{
  struct sam_pipe_s *pipe = &priv->pipelist[idx];

  /* Clear any old pending interrupts for this host pipe. */

  sam_putreg8(0x3f, SAM_USBHOST_PINTFLAG(pipe->idx));

  /* Enable pipe interrupts required for transfers on this pipe. */

  switch (pipe->eptype)
    {
    case USB_EP_ATTR_XFER_CONTROL:
    case USB_EP_ATTR_XFER_BULK:
      {
#ifdef HAVE_USBHOST_TRACE_VERBOSE
        uint16_t intrace;
        uint16_t outtrace;

        /* Determine the definitive trace ID to use below */

        if (pipe->eptype == USB_EP_ATTR_XFER_CONTROL)
          {
            intrace  = SAM_VTRACE2_PIPECONF_CTRL_IN;
            outtrace = SAM_VTRACE2_PIPECONF_CTRL_OUT;
          }
        else
          {
            intrace  = SAM_VTRACE2_PIPECONF_BULK_IN;
            outtrace = SAM_VTRACE2_PIPECONF_BULK_OUT;
          }

        /* Interrupts required for CTRL and BULK endpoints */

        /* Additional setting for IN/OUT endpoints */

        if (pipe->in)
          {
            usbhost_vtrace2(intrace, idx, pipe->epno);
          }
        else
          {
            usbhost_vtrace2(outtrace, idx, pipe->epno);
          }
#endif
      }
      break;

    case USB_EP_ATTR_XFER_INT:
      {
        /* Interrupts required for INTR endpoints */

        /* Additional setting for IN endpoints */

#ifdef HAVE_USBHOST_TRACE_VERBOSE
        if (pipe->in)
          {
            usbhost_vtrace2(SAM_VTRACE2_PIPECONF_INTR_IN, idx, pipe->epno);
          }
        else
          {
            usbhost_vtrace2(SAM_VTRACE2_PIPECONF_INTR_OUT, idx, pipe->epno);
          }
#endif
      }
      break;

    case USB_EP_ATTR_XFER_ISOC:
      {
        /* Interrupts required for ISOC endpoints */

        /* Additional setting for IN endpoints */

#ifdef HAVE_USBHOST_TRACE_VERBOSE
        if (pipe->in)
          {
            usbhost_vtrace2(SAM_VTRACE2_PIPECONF_ISOC_IN, idx, pipe->epno);
          }
        else
          {
            usbhost_vtrace2(SAM_VTRACE2_PIPECONF_ISOC_OUT, idx, pipe->epno);
          }
#endif
      }
      break;
    }

  /* Write the pipe configuration */

  pipe->descb[0]->ctrlpipe = USBHOST_CTRLPIPE_PDADDR(pipe->funcaddr) |
                    USBHOST_CTRLPIPE_PEPNUM(pipe->epno & USB_EPNO_MASK);
  pipe->descb[0]->pktsize = USBHOST_PKTSIZE_SIZE(
                            sam_get_psize(pipe->maxpacket));

  sam_putreg8(USBHOST_PCFG_PTYPE(pipe->eptype + 1) |
             (pipe->eptype == USB_EP_ATTR_XFER_CONTROL ?
                              USBHOST_PCFG_PTOKEN_SETUP :
                              (pipe->in ? USBHOST_PCFG_PTOKEN_IN :
                              USBHOST_PCFG_PTOKEN_OUT)),
                              SAM_USBHOST_PCFG(pipe->idx));

  sam_putreg8(pipe->interval, SAM_USBHOST_BINTERVAL(pipe->idx));

  /* Enable general error and stall interrupts */

  pipe->pipestatus_general = 0;
  sam_putreg8((USBHOST_PINTFLAG_TRFAIL |
               USBHOST_PINTFLAG_PERR   |
               USBHOST_PINTFLAG_STALL), SAM_USBHOST_PINTFLAG(pipe->idx));

  sam_putreg8((USBHOST_PINTFLAG_TRFAIL |
               USBHOST_PINTFLAG_PERR   |
               USBHOST_PINTFLAG_STALL), SAM_USBHOST_PINTENSET(pipe->idx));

  pipe->pipestate_general = USB_H_PIPE_S_IDLE;

  uinfo("pipe%d pktsize=0x%x ctrl=0x%x status=0x%x\n", pipe->idx,
                                                 pipe->descb[0]->pktsize,
                                                 pipe->descb[0]->ctrlpipe,
                                                 pipe->descb[0]->statuspipe);
}

/****************************************************************************
 * Name: sam_pipe_waitsetup
 *
 * Description:
 *   Set the request for the transfer complete event well
 *   BEFORE enabling the transfer (as soon as we are
 *   absolutely committed to the to avoid transfer).
 *   We do this to minimize race conditions.
 *   This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 * Assumptions:
 *  Called from a normal thread context BEFORE the transfer has been started.
 *
 ****************************************************************************/

static int sam_pipe_waitsetup(struct sam_usbhost_s *priv,
                              struct sam_pipe_s *pipe)
{
  irqstate_t flags = enter_critical_section();
  int        ret   = -ENODEV;

  DEBUGASSERT(priv != NULL && pipe != NULL);

  /* Is the device still connected? */

  if (priv->connected)
    {
      /* Yes.. then set waiter to indicate that we expect
       * to be informed when either (1) the device is disconnected,
       * or (2) the transfer completed.
       */

      pipe->waiter   = true;
  #ifdef CONFIG_USBHOST_ASYNCH
      pipe->callback = NULL;
      pipe->arg      = NULL;
  #endif
      ret            = OK;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: sam_pipe_asynchsetup
 *
 * Description:
 *   Set the request for the transfer complete event well BEFORE enabling the
 *   transfer.
 *   We do this to minimize race conditions.
 *   This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 * Assumptions:
 *   Might be called from the level of an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int sam_pipe_asynchsetup(struct sam_usbhost_s *priv,
                                struct sam_pipe_s *pipe,
                                usbhost_asynch_t callback, void *arg)
{
  irqstate_t flags = enter_critical_section();
  int ret = -ENODEV;

  DEBUGASSERT(priv != NULL && pipe != NULL);

  /* Is the device still connected? */

  if (priv->connected)
    {
      pipe->waiter   = false;      /* No waiter */
      pipe->callback = callback;
      pipe->arg      = arg;
      ret            = OK;
    }

  leave_critical_section(flags);
  return ret;
}
#endif

/****************************************************************************
 * Name: sam_pipe_wait
 *
 * Description:
 *   Wait for a transfer on a pipe to complete.
 *
 * Assumptions:
 *   Called from a normal thread context
 *
 ****************************************************************************/

static int sam_pipe_wait(struct sam_usbhost_s *priv,
                         struct sam_pipe_s *pipe)
{
  irqstate_t flags;
  int ret;

  /* Disable interrupts so that the following operations will be atomic. On
   * the host global interrupt needs to be disabled. However, here we disable
   * all interrupts to exploit that fact that interrupts will be re-enabled
   * while we wait.
   */

  flags = enter_critical_section();

  /* Loop, testing for an end of transfer condition.  The pipe 'result'
   * was set to EBUSY and 'waiter' was set to the pipe expecting the
   * response before the transfer was started; 'waiter' will be nullified
   * and 'result' will be set appropriately when the transfer is completed.
   */

  do
    {
      /* Wait for the transfer to complete.  NOTE the transfer may already
       * completed before we get here or the transfer may complete while we
       * wait here.
       */

      ret = nxsem_wait(&pipe->waitsem);

      /* nxsem_wait should succeed.  But it is possible that we could be
       * awakened by a signal too.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (pipe->waiter);

  /* The transfer is complete re-enable interrupts and return the result */

  ret = -(int)pipe->result;
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: sam_pipe_wakeup
 *
 * Description:
 *   A pipe transfer has completed... wakeup any threads waiting for the
 *   transfer to complete.
 *
 * Assumptions:
 *   This function is called from the transfer complete interrupt handler for
 *   the pipe.  Interrupts are disabled.
 *
 ****************************************************************************/

static void sam_pipe_wakeup(struct sam_usbhost_s *priv,
                            struct sam_pipe_s *pipe)
{
  /* Is the transfer complete? */

  if (pipe->result != EBUSY)
    {
      /* Is there a thread waiting for this transfer to complete? */

      if (pipe->waiter)
        {
#ifdef CONFIG_USBHOST_ASYNCH
          /* Yes.. there should not also be a callback scheduled */

          DEBUGASSERT(pipe->callback == NULL);
#endif
          /* Wake'em up! */

          usbhost_vtrace2(pipe->in ? SAM_VTRACE2_PIPEWAKEUP_IN :
                                     SAM_VTRACE2_PIPEWAKEUP_OUT,
                          pipe->epno, pipe->result);

          nxsem_post(&pipe->waitsem);
          pipe->waiter = false;
        }

     #ifdef CONFIG_USBHOST_ASYNCH
      /* No.. is an asynchronous callback expected
       * when the transfer completes?
       */

      else if (pipe->callback)
        {
          /* Handle continuation of IN/OUT pipes */

          if (pipe->in)
            {
              sam_in_next(priv, pipe);
            }
          else
            {
              sam_out_next(priv, pipe);
            }
        }
     #endif
    }
}

/****************************************************************************
 * Name: sam_ctrlep_alloc
 *
 * Description:
 *   Allocate a container and pipes for control pipe.
 *
 * Input Parameters:
 *   priv - The private USB host driver state.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_ctrlep_alloc(struct sam_usbhost_s *priv,
                            const struct usbhost_epdesc_s *epdesc,
                            usbhost_ep_t *ep)
{
  struct usbhost_hubport_s *hport;
  struct sam_pipe_s *pipe;
  int idx;

  /* Sanity check.  NOTE that this method should only be called if
   * a device is connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(epdesc->hport != NULL);
  hport = epdesc->hport;

  idx = sam_pipe_alloc(priv);
  if (idx < 0)
    {
      usbhost_trace1(SAM_TRACE1_PIPEALLOC_FAIL, -idx);
      uerr("ERROR: Failed to allocate a host pipe\n");
      return -ENOMEM;
    }

  pipe            = &priv->pipelist[idx];
  pipe->epno      = epdesc->addr & USB_EPNO_MASK;
  pipe->in        = false;
  pipe->eptype    = USB_EP_ATTR_XFER_CONTROL;
  pipe->funcaddr  = hport->funcaddr;
  pipe->speed     = hport->speed;
  pipe->interval  = 0;
  pipe->maxpacket = SAM_EP0_MAXPACKET;

  /* Configure control OUT pipe */

  pipe->pipestate_general = USB_H_PIPE_S_CFG;
  sam_pipe_configure(priv, idx);

  /* Return a pointer to the control pipe container as the pipe "handle" */

  *ep = (usbhost_ep_t)idx;
  return OK;
}

/****************************************************************************
 * Name: sam_xfrep_alloc
 *
 * Description:
 *   Allocate and configure one unidirectional pipe.
 *
 * Input Parameters:
 *   priv - The private USB host driver state.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated
 *   errno value is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_xfrep_alloc(struct sam_usbhost_s *priv,
                           const struct usbhost_epdesc_s *epdesc,
                           usbhost_ep_t *ep)
{
  struct usbhost_hubport_s *hport;
  struct sam_pipe_s *pipe;
  int idx;

  /* Sanity check.  NOTE that this method should only be called if a device
   * is connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(epdesc->hport != NULL);
  hport = epdesc->hport;
  DEBUGASSERT(hport != NULL);

  /* Allocate a host pipe for the endpoint */

  idx = sam_pipe_alloc(priv);
  if (idx < 0)
    {
      usbhost_trace1(SAM_TRACE1_PIPEALLOC_FAIL, -idx);
      uerr("ERROR: Failed to allocate a host pipe\n");
      return -ENOMEM;
    }

  /* Decode the endpoint descriptor to initialize the pipe data structures.
   * Note:  Here we depend on the fact that the endpoint point type is
   * encoded in the same way in the endpoint descriptor as it is in the OTG
   * HS hardware.
   */

  pipe            = &priv->pipelist[idx];
  pipe->epno      = epdesc->addr & USB_EPNO_MASK;
  pipe->in        = epdesc->in;
  pipe->eptype    = epdesc->xfrtype;
  pipe->funcaddr  = hport->funcaddr;
  pipe->speed     = hport->speed;
  pipe->interval  = epdesc->interval;
  pipe->maxpacket = epdesc->mxpacketsize;
  pipe->pipestate_general = pipe->in ? USB_H_PIPE_S_DATI : USB_H_PIPE_S_DATO;

  /* Then configure the endpoint */

  sam_pipe_configure(priv, idx);

  /* Return the endpoint number as the endpoint "handle" */

  *ep = (usbhost_ep_t)idx;
  return OK;
}

/****************************************************************************
 * Name: sam_transfer_terminate
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
 *
 ****************************************************************************/

static void sam_transfer_terminate(struct sam_usbhost_s *priv,
                                   struct sam_pipe_s *pipe,
                                   int result)
{
  /* Wake up any waiters for the end of transfer event */

  sam_pipe_wakeup(priv, pipe);

  if (pipe->pipestate_general < USB_H_PIPE_S_SETUP ||
      pipe->pipestate_general > USB_H_PIPE_S_STATO)
  return; /* Not busy */

  if (pipe->eptype == USB_EP_ATTR_XFER_CONTROL)
    {
      if (priv->n_ctrl_req_user)
        priv->n_ctrl_req_user--;
      if (priv->n_sof_user)
        priv->n_sof_user--;
    }

  pipe->pipestate_general  = USB_H_PIPE_S_IDLE;
  pipe->pipestatus_general = result;

  /* Suspend delayed due to control request: start it */

  if (priv->n_ctrl_req_user == 0 && priv->suspend_start < 0)
    {
      uint8_t i;
      if (priv->n_ctrl_req_user)
        {
          /* Delay suspend after setup requests */

          priv->suspend_start = -1;
          return;
        }

      /* Save pipe freeze states and freeze pipes */

      priv->pipes_unfreeze = 0;
      for (i = 0; i < SAM_USB_NENDPOINTS; i++)
        {
          /* Skip frozen pipes */

          if ((sam_getreg16(SAM_USBHOST_PSTATUS(i)) &
               USBHOST_PSTATUS_PFREEZE) >> 4)
            continue;

          /* Log unfrozen pipes */

          priv->pipes_unfreeze |= 1 << i;

          /* Freeze it to suspend */

          sam_putreg8(USBHOST_PSTATUS_PFREEZE, SAM_USBHOST_PSTATUSSET(i));
        }

      /* Wait 3 SOFs before entering in suspend state */

      sam_add_sof_user(priv); /* SOF user: delayed suspend */
      priv->suspend_start = 3;
    }
}

static void sam_transfer_abort(struct sam_usbhost_s *priv,
                               struct sam_pipe_s *pipe,
                               int code)
{
  /* Stop transfer */

  sam_putreg8(USBHOST_PSTATUS_PFREEZE, SAM_USBHOST_PSTATUSSET(pipe->idx));

  /* Update byte count */

  if (pipe->in == 0)
    pipe->count += (pipe->descb[0]->pktsize &
                    USBHOST_PKTSIZE_MPKTSIZE_MASK) >>
                    USBHOST_PKTSIZE_MPKTSIZE_SHIFT;

  /* Disable interrupts */

  sam_putreg8((USBHOST_PINTFLAG_TRCPT0 |
               USBHOST_PINTFLAG_TRCPT1),
               SAM_USBHOST_PINTENCLR(pipe->idx));

  sam_transfer_terminate(priv, pipe, code);
}

/****************************************************************************
 * Name: sam_send_continue
 *
 * Description:
 *   Continue the send operation started by sam_send_start().
 *
 * Assumptions:
 *
 ****************************************************************************/

static void sam_send_continue(struct sam_usbhost_s *priv,
                              struct sam_pipe_s *pipe)
{
  uint8_t * src;
  uint32_t size;
  uint32_t count;
  uint32_t n_tx = 0;
  uint32_t n_remain;

  if (pipe->pipestate_general == USB_H_PIPE_S_STATO)
    {
      /* Control status : ZLP OUT done */

      sam_transfer_terminate(priv, pipe, OK);
      return;
    }

  else if (pipe->pipestate_general != USB_H_PIPE_S_DATO)
    return;

  /* Reset packet timeout for control pipes */

  if (pipe->eptype == USB_EP_ATTR_XFER_CONTROL)
    pipe->pkt_timeout = USB_CTRL_DPKT_TIMEOUT;

  n_tx = (pipe->descb[0]->pktsize & USBHOST_PKTSIZE_BCNT_MASK) >>
                                    USBHOST_PKTSIZE_BCNT_SHIFT;

  /* ZLP cleared if it's short packet */

  if (n_tx < pipe->maxpacket)
    pipe->zlp = 0;

  src = pipe->data;
  size = pipe->size;
  count = pipe->count;

  if (n_tx)
    {
      count += n_tx;
      pipe->count = count;
    }

  n_remain = size - count;

  /* Now set n_tx to next transfer size */

  if (n_remain > 16320)
    n_tx = 16320;
  else
    n_tx = n_remain;

  /* For Control, all data is done, to STATUS stage */

  if (pipe->eptype == USB_EP_ATTR_XFER_CONTROL &&
                     pipe->count >= pipe->size &&
                     !pipe->zlp)
    {
      pipe->pipestate = USB_H_PIPE_S_STATI;

      /* Start IN ZLP request */

      pipe->pkt_timeout = USB_CTRL_STAT_TIMEOUT;
      sam_putreg8(USBHOST_PSTATUS_DTGL, SAM_USBHOST_PSTATUSSET(pipe->idx));
      sam_recv_restart(priv, pipe);
      return;
    }

  /* All transfer done, including ZLP */

  if (count >= size && !pipe->zlp)
    {
      /* At least one bank there, wait to freeze pipe */

      if (pipe->eptype != USB_EP_ATTR_XFER_CONTROL)
        {
          /* Busy interrupt when all banks are empty */

          sam_transfer_terminate(priv, pipe, OK);
        }
      else /* No busy interrupt for control EPs */
        {
        }
    }
  else
    {
      sam_putreg8((USBHOST_PINTFLAG_TRCPT0 |
                   USBHOST_PINTFLAG_TRCPT1),
                  SAM_USBHOST_PINTFLAG(pipe->idx));

      sam_putreg8((USBHOST_PINTFLAG_TRCPT0 |
                   USBHOST_PINTFLAG_TRCPT1),
                  SAM_USBHOST_PINTENSET(pipe->idx));

      pipe->descb[0]->addr = (uint32_t)&src[count];
      pipe->descb[0]->pktsize &= ~(USBHOST_PKTSIZE_MPKTSIZE_MASK |
                                   USBHOST_PKTSIZE_BCNT_MASK);
      pipe->descb[0]->pktsize |= USBHOST_PKTSIZE_BCNT(n_remain);

      /* Send the OUT token */

      sam_modifyreg8(USBHOST_PCFG_PTOKEN_MASK,
                     USBHOST_PCFG_PTOKEN_OUT,
                     SAM_USBHOST_PCFG(pipe->idx));

      sam_putreg8(USBHOST_PSTATUS_BK0RDY,
                  SAM_USBHOST_PSTATUSSET(pipe->idx));
      sam_putreg8(USBHOST_PSTATUS_PFREEZE,
                  SAM_USBHOST_PSTATUSCLR(pipe->idx));
    }
}

/****************************************************************************
 * Name: sam_send_start
 *
 * Description:
 *   Start at transfer on the selected IN or OUT pipe.
 *
 ****************************************************************************/

static void sam_send_start(struct sam_usbhost_s *priv,
                           struct sam_pipe_s *pipe)
{
  /* Set up the initial state of the transfer */

  usbhost_vtrace2(SAM_VTRACE2_STARTTRANSFER1, pipe->idx, pipe->size);

  pipe->result = EBUSY;
  pipe->count = 0;

  /* Make sure the peripheral address is correct */

  pipe->descb[0]->ctrlpipe &= ~USBHOST_CTRLPIPE_PDADDR_MASK;
  pipe->descb[0]->ctrlpipe |= USBHOST_CTRLPIPE_PDADDR(pipe->funcaddr);

  /* Checkout for zero length packet */

  if (pipe->size > 0)
    {
      /* No.. we need to copy the outgoing data and start the transfer */

      sam_putreg8((USBHOST_PINTFLAG_TRCPT0 |
                   USBHOST_PINTFLAG_TRCPT1),
                   SAM_USBHOST_PINTFLAG(pipe->idx));

      sam_putreg8((USBHOST_PINTFLAG_TRCPT0 |
                   USBHOST_PINTFLAG_TRCPT1),
                   SAM_USBHOST_PINTENSET(pipe->idx));

      pipe->descb[0]->addr = (uint32_t)pipe->data;
      pipe->descb[0]->pktsize &= ~(USBHOST_PKTSIZE_MPKTSIZE_MASK |
                                   USBHOST_PKTSIZE_BCNT_MASK);
      pipe->descb[0]->pktsize |= USBHOST_PKTSIZE_BCNT(pipe->size);

      /* Send the OUT token */

      sam_modifyreg8(USBHOST_PCFG_PTOKEN_MASK,
                     USBHOST_PCFG_PTOKEN_OUT,
                     SAM_USBHOST_PCFG(pipe->idx));

      sam_putreg8(USBHOST_PSTATUS_BK0RDY,
                  SAM_USBHOST_PSTATUSSET(pipe->idx));
      sam_putreg8(USBHOST_PSTATUS_PFREEZE,
                  SAM_USBHOST_PSTATUSCLR(pipe->idx));
    }
  else
    {
      sam_putreg8((USBHOST_PINTFLAG_TRCPT0 |
                   USBHOST_PINTFLAG_TRCPT1),
                   SAM_USBHOST_PINTFLAG(pipe->idx));
      sam_putreg8((USBHOST_PINTFLAG_TRCPT0 |
                   USBHOST_PINTFLAG_TRCPT1),
                   SAM_USBHOST_PINTENSET(pipe->idx));
      sam_putreg8(USBHOST_PSTATUS_DTGL,
                  SAM_USBHOST_PSTATUSSET(pipe->idx));

      /* Write the zero byte count */

      pipe->descb[0]->addr = (uint32_t)priv->ctrl_buffer;
      pipe->descb[0]->pktsize &= ~(USBHOST_PKTSIZE_MPKTSIZE_MASK |
                                   USBHOST_PKTSIZE_BCNT_MASK);
      pipe->descb[0]->pktsize |= USBHOST_PKTSIZE_AUTOZLP;

      /* Send the OUT token */

      sam_modifyreg8(USBHOST_PCFG_PTOKEN_MASK,
                     USBHOST_PCFG_PTOKEN_OUT,
                     SAM_USBHOST_PCFG(pipe->idx));
      sam_putreg8(USBHOST_PSTATUS_BK0RDY,
                  SAM_USBHOST_PSTATUSSET(pipe->idx));
      sam_putreg8(USBHOST_PSTATUS_PFREEZE,
                  SAM_USBHOST_PSTATUSCLR(pipe->idx));
    }
}

/****************************************************************************
 * Name: sam_out_transfer
 *
 * Description:
 *   Transfer the 'buflen' bytes in 'buffer' through an OUT pipe.
 *
 * Assumptions:
 *   This function is called only from the TRANSFER
 *   interface.  The lock, for example,
 *   must be relinquished before waiting.
 *
 ****************************************************************************/

static ssize_t sam_out_transfer(struct sam_usbhost_s *priv,
                                struct sam_pipe_s *pipe,
                                uint8_t *buffer, size_t buflen)
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

      xfrlen = MIN(pipe->maxpacket, buflen);
      pipe->data = buffer;
      pipe->size = xfrlen;
      pipe->count = 0;
      uinfo("pipe%d buffer:%p buflen:%d\n",
                                 pipe->idx,
                                 pipe->data,
                                 pipe->size);

      /* Set up for the wait BEFORE starting the transfer */

      ret = sam_pipe_waitsetup(priv, pipe);
      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_DEVDISCONN1, 0);
          return (ssize_t)ret;
        }

      /* Set up for the transfer based
       * on the direction and the endpoint type
       */

      ret = sam_out_setup(priv, pipe);

      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_OUTSETUP_FAIL1, -ret);
          return (ssize_t)ret;
        }

      /* Wait for the transfer to complete and get the result */

      ret = sam_pipe_wait(priv, pipe);

      /* Handle transfer failures */

      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_TRANSFER_FAILED1, ret);

          /* Check for a special case:  If (1) the transfer was NAKed and (2)
           * no SNDFIFO empty or Rx FIFO not-empty event occurred, then we
           * should be able to just flush the Rx and SNDFIFOs and try again.
           * We can detect this latter case because then the transfer buffer
           * pointer and buffer size will be unaltered.
           */

          elapsed = clock_systime_ticks() - start;
          if (ret != -EAGAIN ||                /* Not a NAK condition OR */
              elapsed >= SAM_DATANAK_DELAY ||  /* Timeout has elapsed OR */
              pipe->count > 0)                 /* Data has been partially transferred */
            {
              /* Break out and return the error */

              usbhost_trace1(SAM_TRACE1_PIPEWAIT_FAIL, -ret);
              return (ssize_t)ret;
            }

          /* Get the device a little time to catch up.
           * Then retry the transfer
           * using the same buffer pointer and length.
           */

          nxsig_usleep(20 * 1000);
        }
      else
        {
          /* Successfully transferred. Update the buffer pointer and length */

          buffer += xfrlen;
          buflen -= xfrlen;
          xfrd   += pipe->count;
        }
    }

  return xfrd;
}

/****************************************************************************
 * Name: sam_out_next
 *
 * Description:
 *   Initiate the next of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is always called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void sam_out_next(struct sam_usbhost_s *priv,
                         struct sam_pipe_s *pipe)
{
  usbhost_asynch_t callback;
  void *arg;
  ssize_t nbytes;
  int result;
  int ret;

  /* Is the full transfer complete?
   * Did the last chunk transfer complete OK?
   */

  result = -(int)pipe->result;
  if (pipe->count < pipe->size && result == OK)
    {
      /* Yes.. Set up for the next transfer based on the direction and the
       * endpoint type
       */

      ret = sam_out_setup(priv, pipe);
      if (ret >= 0)
        {
          return;
        }

      usbhost_trace1(SAM_TRACE1_OUTSETUP_FAIL2, -ret);
      result = ret;
    }

  /* The transfer is complete, with or without an error */

  usbhost_vtrace1(SAM_VTRACE1_TRANSFER_COMPLETE, result);

  /* Extract the callback information */

  callback = pipe->callback;
  arg = pipe->arg;
  nbytes = pipe->count;

  pipe->callback = NULL;
  pipe->arg = NULL;
  pipe->count = 0;

  /* Then perform the callback */

  if (result < 0)
    {
      nbytes = (ssize_t)result;
    }

  callback(arg, nbytes);
}
#endif

/****************************************************************************
 * Name: sam_out_asynch
 *
 * Description:
 *   Initiate the first of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is called only from the ASYNCH.
 *   The lock, for example, must be relinquished before waiting.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int sam_out_asynch(struct sam_usbhost_s *priv,
                          struct sam_pipe_s *pipe,
                          uint8_t *buffer, size_t buflen,
                          usbhost_asynch_t callback, void *arg)
{
  int ret;

  /* Set up for the transfer data and callback
   * BEFORE starting the first transfer
   */

  pipe->data = buffer;
  pipe->size = buflen;
  pipe->count = 0;

  ret = sam_pipe_asynchsetup(priv, pipe, callback, arg);
  if (ret < 0)
    {
      usbhost_trace1(SAM_TRACE1_ASYNCHSETUP_FAIL1, -ret);
      return ret;
    }

  /* Set up for the transfer based on the
   * direction and the endpoint type
   */

  ret = sam_out_setup(priv, pipe);

  if (ret < 0)
    {
      usbhost_trace1(SAM_TRACE1_OUTSETUP_FAIL3, -ret);
    }

  /* And return with the transfer pending */

  return ret;
}
#endif

/****************************************************************************
 * Name: sam_ctrl_sendsetup
 *
 * Description:
 *   Send an IN/OUT SETUP packet.
 *
 * Assumptions:
 *   This function is called only from the CTRLIN and CTRLOUT interfaces.
 *
 ****************************************************************************/

static int sam_ctrl_sendsetup(struct sam_usbhost_s *priv,
                              struct sam_pipe_s *pipe,
                              const struct usb_ctrlreq_s *req)
{
  clock_t start;
  clock_t elapsed;
  int ret;
  int i;

  /* Loop while the device reports NAK (and a timeout is not exceeded */

  start = clock_systime_ticks();
  do
    {
      /* Send the SETUP packet */

      pipe->data = (uint8_t *)req;
      pipe->size = USB_SIZEOF_CTRLREQ;
      pipe->count = 0;
      pipe->result = EBUSY;
      uinfo("pipe%d buffer:%p buflen:%d\n",
                                 pipe->idx,
                                 pipe->data,
                                 pipe->size);
      sam_pktdump("sam_ctrl_sendsetup", pipe->data, pipe->size);

      /* Set up for the wait BEFORE starting the transfer */

      ret = sam_pipe_waitsetup(priv, pipe);
      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_DEVDISCONN2, 0);
          return ret;
        }

      pipe->pipestate_general = USB_H_PIPE_S_SETUP;
      sam_add_sof_user(priv);
      priv->n_ctrl_req_user++;

      /* Make sure the peripheral address is correct */

      pipe->descb[0]->ctrlpipe &= ~USBHOST_CTRLPIPE_PDADDR_MASK;
      pipe->descb[0]->ctrlpipe |= USBHOST_CTRLPIPE_PDADDR(pipe->funcaddr);

      /* Write packet */

      sam_putreg8(USBHOST_PINTFLAG_TXSTP, SAM_USBHOST_PINTFLAG(pipe->idx));
      for (i = 0; i < USB_SIZEOF_CTRLREQ; i++)
          priv->ctrl_buffer[i] = pipe->data[i];

      pipe->descb[0]->addr = (uint32_t)pipe->data;
      pipe->descb[0]->pktsize &= ~(USBHOST_PKTSIZE_MPKTSIZE_MASK |
                                   USBHOST_PKTSIZE_BCNT_MASK);
      pipe->descb[0]->pktsize |= USBHOST_PKTSIZE_BCNT(USB_SIZEOF_CTRLREQ);

      pipe->descb[0]->ctrlpipe = USBHOST_CTRLPIPE_PDADDR(pipe->funcaddr) |
                      USBHOST_CTRLPIPE_PEPNUM(pipe->epno & USB_EPNO_MASK);
      uinfo("pipe%d pktsize=0x%x ctrl=0x%x status=0x%x\n",
                                    pipe->idx,
                                    pipe->descb[0]->pktsize,
                                    pipe->descb[0]->ctrlpipe,
                                    pipe->descb[0]->statuspipe);

      /* Send the SETUP token (always EP0) */

      sam_modifyreg8(USBHOST_PCFG_PTOKEN_MASK,
                     USBHOST_PCFG_PTOKEN_SETUP,
                     SAM_USBHOST_PCFG(pipe->idx));
      sam_putreg8(USBHOST_PSTATUS_DTGL, SAM_USBHOST_PSTATUSCLR(pipe->idx));
      sam_putreg8(USBHOST_PSTATUS_BK0RDY, SAM_USBHOST_PSTATUSSET(pipe->idx));
      sam_putreg8(USBHOST_PINTFLAG_TXSTP, SAM_USBHOST_PINTENSET(pipe->idx));
      sam_putreg8(USBHOST_PSTATUS_PFREEZE,
                  SAM_USBHOST_PSTATUSCLR(pipe->idx));

      /* Wait for the transfer to complete */

      ret = sam_pipe_wait(priv, pipe);

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
              usbhost_trace1(SAM_TRACE1_TRANSFER_FAILED2, ret);
            }

          /* Return the result in any event */

          return ret;
        }

      /* Get the elapsed time (in frames) */

      elapsed = clock_systime_ticks() - start;
    }
  while (elapsed < SAM_SETUP_DELAY);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: sam_ctrl_senddata
 *
 * Description:
 *   Send data in the data phase of an OUT control transfer.  Or send status
 *   in the status phase of an IN control transfer
 *
 * Assumptions:
 *   This function is called only from the CTRLOUT interface.
 *
 ****************************************************************************/

static int sam_ctrl_senddata(struct sam_usbhost_s *priv,
                             struct sam_pipe_s *pipe,
                             uint8_t *buffer, unsigned int buflen)
{
  int ret;

  uinfo("pipe%d buffer:%p buflen:%d\n", pipe->idx, buffer, buflen);

  /* Save buffer information */

  pipe->pipestate_general = USB_H_PIPE_S_DATO;
  pipe->in = false;
  pipe->data = buffer;
  pipe->size = buflen;
  pipe->count = 0;

  /* Set up for the wait BEFORE starting the transfer */

  ret = sam_pipe_waitsetup(priv, pipe);
  if (ret < 0)
    {
      usbhost_trace1(SAM_TRACE1_DEVDISCONN3, 0);
      return ret;
    }

  /* Start the transfer */

  sam_send_start(priv, pipe);

  /* Wait for the transfer to complete and return the result */

  return sam_pipe_wait(priv, pipe);
}

/****************************************************************************
 * Name: sam_ctrl_recvdata
 *
 * Description:
 *   Receive data in the data phase of an IN control transfer.
 *   Or receive status in the status phase of
 *   an OUT control transfer.
 *
 * Assumptions:
 *   This function is called only from the CTRLIN interface.
 *
 ****************************************************************************/

static int sam_ctrl_recvdata(struct sam_usbhost_s *priv,
                             struct sam_pipe_s *pipe,
                             uint8_t *buffer, unsigned int buflen)
{
  int ret;

  /* Save buffer information */

  pipe->pipestate_general = USB_H_PIPE_S_DATI;

  pipe->in = true;
  pipe->data = buffer;
  pipe->size = buflen;
  pipe->count = 0;

  /* Set up for the wait BEFORE starting the transfer */

  ret = sam_pipe_waitsetup(priv, pipe);
  if (ret < 0)
    {
      usbhost_trace1(SAM_TRACE1_DEVDISCONN4, 0);
      return ret;
    }

  /* Start the transfer */

  sam_recv_start(priv, pipe);

  /* Wait for the transfer to complete and return the result */

  ret = sam_pipe_wait(priv, pipe);

  uinfo("pipe%d buffer:%p buflen:%d ADDR=0x%x PKTSIZE=0x%x\n",
        pipe->idx, buffer, buflen,
        pipe->descb[0]->addr,
        pipe->descb[0]->pktsize)

  uinfo("EXTREG=0x%x STATUSBK=0x%x CTRLPIPE=0x%x STATUSPIPE=0x%x\n",
        pipe->descb[0]->extreg,
        pipe->descb[0]->stausbk,
        pipe->descb[0]->ctrlpipe,
        pipe->descb[0]->statuspipe);
  sam_pktdump("sam_ctrl_recvdata", pipe->data, pipe->size);

  return ret;
}

/****************************************************************************
 * Name: sam_in_setup
 *
 * Description:
 *   Initiate an IN transfer on an bulk, interrupt, or isochronous pipe.
 *
 ****************************************************************************/

static int sam_in_setup(struct sam_usbhost_s *priv,
                        struct sam_pipe_s *pipe)
{
  uinfo("pipe%d\n", pipe->idx);

  /* Set up for the transfer based on the direction and the endpoint type */

  switch (pipe->eptype)
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

          usbhost_vtrace2(SAM_VTRACE2_ISOCIN, pipe->idx, pipe->size);
        }
        break;

      case USB_EP_ATTR_XFER_BULK: /* Bulk */
        {
          usbhost_vtrace2(SAM_VTRACE2_BULKIN, pipe->idx, pipe->size);
          pipe->pipestate_general = pipe->in ?
                USB_H_PIPE_S_DATI : USB_H_PIPE_S_DATO;
        }
        break;

      case USB_EP_ATTR_XFER_INT: /* Interrupt */
        {
          usbhost_vtrace2(SAM_VTRACE2_INTRIN, pipe->idx, pipe->size);
        }
        break;
    }

  /* Start the transfer. */

  sam_recv_start(priv, pipe);
  return OK;
}

/****************************************************************************
 * Name: sam_out_setup
 *
 * Description:
 *   Initiate an OUT transfer on an bulk, interrupt, or isochronous pipe.
 *
 ****************************************************************************/

static int sam_out_setup(struct sam_usbhost_s *priv,
                         struct sam_pipe_s *pipe)
{
  /* Set up for the transfer based on the direction and the endpoint type */

  switch (pipe->eptype)
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

          usbhost_vtrace2(SAM_VTRACE2_ISOCOUT,
                          pipe->idx, pipe->size);
        }
        break;

      case USB_EP_ATTR_XFER_BULK: /* Bulk */
        {
          usbhost_vtrace2(SAM_VTRACE2_BULKOUT,
                          pipe->idx, pipe->size);
          pipe->pipestate_general = pipe->in ?
                           USB_H_PIPE_S_DATI : USB_H_PIPE_S_DATO;
        }
        break;

      case USB_EP_ATTR_XFER_INT: /* Interrupt */
        {
          usbhost_vtrace2(SAM_VTRACE2_INTROUT,
                          pipe->idx, pipe->size);
        }
        break;
    }

  /* Start the transfer */

  sam_send_start(priv, pipe);
  return OK;
}

/****************************************************************************
 * Name: sam_recv_continue
 *
 * Description:
 *   Continue the receive operation started by sam_recv_start().  This
 *   function is called from the interrupt handler worker when an interrupt
 *   indicates that new, incoming data is available
 *
 * Assumptions:
 *
 ****************************************************************************/

static void sam_recv_continue(struct sam_usbhost_s *priv,
                              struct sam_pipe_s *pipe)
{
  uint8_t *src;
  uint8_t *dst;
  uint32_t size;
  uint32_t count;
  uint32_t i;
  uint32_t n_rx = 0;
  uint32_t n_remain;
  bool shortpkt = false;
  bool full = false;

  if (pipe->pipestate_general == USB_H_PIPE_S_STATI)
    {
      /* Control status : ZLP IN done */

      sam_transfer_terminate(priv, pipe, OK);
      return;
    }
  else if (pipe->pipestate_general != USB_H_PIPE_S_DATI)
    return;

  /* Read byte count */

  n_rx = (pipe->descb[0]->pktsize & USBHOST_PKTSIZE_BCNT_MASK) >>
                                    USBHOST_PKTSIZE_BCNT_SHIFT;
  if (n_rx < pipe->maxpacket)
    shortpkt = true;

  if (n_rx)
    {
      dst = pipe->data;
      size = pipe->size;
      count = pipe->count;
      n_remain = size - count;
      src = (uint8_t *)pipe->descb[0]->addr;
      dst = &dst[count];
      if (n_rx >= n_remain)
        {
          n_rx = n_remain;
          full = true;
        }

      count += n_rx;
      for (i = 0; i < n_rx; i++)
        *dst++ = *src++;

      pipe->count = count;
    }

  /* Reset timeout for control pipes */

  if (pipe->eptype == USB_EP_ATTR_XFER_CONTROL)
    pipe->pkt_timeout = USB_CTRL_DPKT_TIMEOUT;

  /* Finish on error or short packet */

  if (full || shortpkt)
    {
      if (pipe->eptype == USB_EP_ATTR_XFER_CONTROL)
        {
          pipe->pipestate = USB_H_PIPE_S_STATO;
          pipe->pkt_timeout = USB_CTRL_STAT_TIMEOUT;
          sam_putreg8(USBHOST_PSTATUS_DTGL,
                      SAM_USBHOST_PSTATUSSET(pipe->idx));
          sam_send_start(priv, pipe);
        }
      else
          sam_transfer_terminate(priv, pipe, OK);
    }
  else
    {
      /* Just wait another packet */

      sam_recv_restart(priv, pipe);
    }
}

/****************************************************************************
 * Name: sam_recv_restart
 *
 * Description:
 *   Start/Re-start the transfer on the selected IN or OUT pipe
 *
 ****************************************************************************/

static void sam_recv_restart(struct sam_usbhost_s *priv,
                             struct sam_pipe_s *pipe)
{
  /* Send the IN token. */

  sam_putreg8((USBHOST_PINTFLAG_TRCPT0 |
               USBHOST_PINTFLAG_TRCPT1), SAM_USBHOST_PINTFLAG(pipe->idx));
  sam_putreg8((USBHOST_PINTFLAG_TRCPT0 |
               USBHOST_PINTFLAG_TRCPT1), SAM_USBHOST_PINTENSET(pipe->idx));

  if (pipe->eptype == USB_EP_ATTR_XFER_CONTROL)
    {
      pipe->descb[0]->addr = (uint32_t)pipe->data;
      pipe->descb[0]->pktsize &= ~(USBHOST_PKTSIZE_MPKTSIZE_MASK |
                                   USBHOST_PKTSIZE_BCNT_MASK);
      pipe->descb[0]->pktsize |= USBHOST_PKTSIZE_MPKTSIZE(pipe->maxpacket);
    }
  else
    {
      uint32_t n_next = pipe->size - pipe->count;

      pipe->descb[0]->addr = (uint32_t)&pipe->data[pipe->count];
      if (n_next > 16384)
        n_next = 16384;

      pipe->descb[0]->pktsize &= ~(USBHOST_PKTSIZE_MPKTSIZE_MASK |
                                   USBHOST_PKTSIZE_BCNT_MASK);
      pipe->descb[0]->pktsize |= USBHOST_PKTSIZE_MPKTSIZE(n_next);
    }

  sam_modifyreg8(USBHOST_PCFG_PTOKEN_MASK,
                 USBHOST_PCFG_PTOKEN_IN,
                 SAM_USBHOST_PCFG(pipe->idx));
  sam_putreg8(USBHOST_PSTATUS_BK0RDY, SAM_USBHOST_PSTATUSCLR(pipe->idx));
  sam_putreg8(USBHOST_PSTATUS_PFREEZE, SAM_USBHOST_PSTATUSCLR(pipe->idx));
}

/****************************************************************************
 * Name: sam_recv_start
 *
 * Description:
 *   Start at transfer on the selected IN or OUT pipe.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void sam_recv_start(struct sam_usbhost_s *priv,
                           struct sam_pipe_s *pipe)
{
  /* Set up the initial state of the transfer */

  usbhost_vtrace2(SAM_VTRACE2_STARTTRANSFER2, pipe->idx, pipe->size);

  pipe->result = EBUSY;
  pipe->count = 0;

  /* Make sure the peripheral address is correct */

  pipe->descb[0]->ctrlpipe &= ~USBHOST_CTRLPIPE_PDADDR_MASK;
  pipe->descb[0]->ctrlpipe |= USBHOST_CTRLPIPE_PDADDR(pipe->funcaddr);

  /* Start the transfer. */

  sam_recv_restart(priv, pipe);
}

/****************************************************************************
 * Name: sam_in_transfer
 *
 * Description:
 *   Transfer 'buflen' bytes into 'buffer' from an IN pipe.
 *
 * Assumptions:
 *   This function is called only from the TRANSFER.
 *   The lock, for example, must be relinquished before waiting.
 *
 ****************************************************************************/

static ssize_t sam_in_transfer(struct sam_usbhost_s *priv,
                               struct sam_pipe_s *pipe,
                               uint8_t *buffer, size_t buflen)
{
  clock_t start;
  ssize_t xfrd;
  int ret;

/* Loop until the transfer completes (i.e., buflen is decremented to zero)
 * or a fatal error occurs any error other than a simple NAK.  NAK would
 * simply indicate the end of the transfer (short-transfer).
 */

  pipe->data = buffer;
  pipe->size = buflen;
  pipe->count = 0;
  xfrd = 0;

  start = clock_systime_ticks();
  while (pipe->count < pipe->size)
    {
      /* Set up for the wait BEFORE starting the transfer */

      ret = sam_pipe_waitsetup(priv, pipe);
      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_DEVDISCONN5, 0);
          return (ssize_t)ret;
        }

      /* Set up for the transfer based on the direction
       * and the endpoint type
       */

      ret = sam_in_setup(priv, pipe);

      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_INSETUP_FAIL1, -ret);
          return (ssize_t)ret;
        }

      /* Wait for the transfer to complete and get the result */

      ret = sam_pipe_wait(priv, pipe);

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
                  return xfrd;
              else
                {
                  useconds_t delay;

                  /* Get the elapsed time.
                   * Has the timeout elapsed?
                   * if not then try again.
                   */

                  clock_t elapsed = clock_systime_ticks() - start;
                  if (elapsed >= SAM_DATANAK_DELAY)
                    {
                      /* Timeout out... break out returning the NAK as
                       * as a failure.
                       */

                      return (ssize_t)ret;
                    }

                  /* Wait a bit before retrying after a NAK. */

                  if (pipe->eptype == USB_EP_ATTR_XFER_INT)
                    {
      /* For interrupt (and isochronous) endpoints, the
       * polling rate is determined by the bInterval field
       * of the endpoint descriptor (in units of frames
       * which we treat as milliseconds here).
       */

                      if (pipe->interval > 0)
                        {
                          /* Convert the delay to units of microseconds */

                          delay = (useconds_t)pipe->interval * 1000;
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
      /* For Isochronous endpoints, bInterval must be 1.  Bulk
       * endpoints do not have a polling interval.  Rather,
       * the should wait until data is received.
       *
       * REVISIT:  For bulk endpoints this 1 msec delay is only
       * intended to give the CPU a break from the bulk EP tight
       * polling loop.  But are there performance issues?
       */

                      delay = 1000;
                    }

      /* Wait for the next polling interval.  For interrupt and
       * isochronous endpoints, this is necessary to assure the
       * polling interval.  It is used in other cases only to
       * prevent the polling from consuming too much CPU bandwidth.
       *
       * Small delays could require more resolution than is provided
       * by the system timer.  For example, if the system timer
       * resolution is 10MS, then nxsig_usleep(1000) will actually request
       * a delay 20MS (due to both quantization and rounding).
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

              usbhost_trace1(SAM_TRACE1_TRANSFER_FAILED3, -ret);

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

          xfrd += pipe->count;
        }
    }

  return xfrd;
}

/****************************************************************************
 * Name: sam_in_next
 *
 * Description:
 *   Initiate the next of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is always called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void sam_in_next(struct sam_usbhost_s *priv,
                        struct sam_pipe_s *pipe)
{
  usbhost_asynch_t callback;
  void *arg;
  ssize_t nbytes;
  int result;
  int ret;

  /* Is the full transfer complete?
   * Did the last chunk transfer complete OK?
   */

  result = -(int)pipe->result;
  if (pipe->count < pipe->size && result == OK)
    {
      /* Yes.. Set up for the next transfer based on the direction and the
       * endpoint type
       */

      ret = sam_in_setup(priv, pipe);
      if (ret >= 0)
        {
          return;
        }

      usbhost_trace1(SAM_TRACE1_INSETUP_FAIL2, -ret);
      result = ret;
    }

  /* The transfer is complete, with or without an error */

  usbhost_vtrace2(SAM_VTRACE2_XFRCOMPLETE,
                  (unsigned int)pipe->idx, pipe->size);

  /* Extract the callback information */

  callback       = pipe->callback;
  arg            = pipe->arg;
  nbytes         = pipe->count;

  pipe->callback = NULL;
  pipe->arg = NULL;
  pipe->count = 0;

  /* Then perform the callback */

  if (result < 0)
    {
      nbytes = (ssize_t)result;
    }

  callback(arg, nbytes);
}
#endif

/****************************************************************************
 * Name: sam_in_asynch
 *
 * Description:
 *   Initiate the first of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is called only from the ASYNCH interface.
 *   The lock, for example,
 *   must be relinquished before waiting.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int sam_in_asynch(struct sam_usbhost_s *priv,
                         struct sam_pipe_s *pipe,
                         uint8_t *buffer, size_t buflen,
                         usbhost_asynch_t callback, void *arg)
{
  int ret;

  /* Set up for the transfer data and callback
   * BEFORE starting the first transfer
   */

  pipe->data = buffer;
  pipe->size = buflen;
  pipe->count = 0;

  ret = sam_pipe_asynchsetup(priv, pipe, callback, arg);
  if (ret < 0)
    {
      usbhost_trace1(SAM_TRACE1_ASYNCHSETUP_FAIL2, -ret);
      return ret;
    }

  /* Set up for the transfer based on the direction and the endpoint type */

  ret = sam_in_setup(priv, pipe);

  if (ret < 0)
    {
      usbhost_trace1(SAM_TRACE1_INSETUP_FAIL3, -ret);
    }

  /* And return with the transfer pending */

  return ret;
}
#endif

/****************************************************************************
 * Name: sam_gint_connected
 *
 * Description:
 *   Handle a connection event.
 *
 ****************************************************************************/

static void sam_gint_connected(struct sam_usbhost_s *priv)
{
  /* We we previously disconnected? */

  if (!priv->connected)
    {
      /* Yes.. then now we are connected */

      usbhost_vtrace1(SAM_VTRACE1_CONNECTED1, 0);
      priv->connected = true;
      priv->change    = true;
      DEBUGASSERT(priv->smstate == SMSTATE_DETACHED);

      /* Notify any waiters */

      priv->smstate = SMSTATE_ATTACHED;
      if (priv->pscwait)
        {
          nxsem_post(&priv->pscsem);
          priv->pscwait = false;
        }
    }
}

/****************************************************************************
 * Name: sam_gint_disconnected
 *
 * Description:
 *   Handle a disconnection event.
 *
 ****************************************************************************/

static void sam_gint_disconnected(struct sam_usbhost_s *priv)
{
  /* Were we previously connected? */

  if (priv->connected)
    {
      /* Yes.. then we no longer connected */

      usbhost_vtrace1(SAM_VTRACE1_DISCONNECTED1, 0);

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
      sam_reset_pipes(priv, false);

      priv->rhport.hport.speed = USB_SPEED_FULL;
      priv->rhport.hport.funcaddr = 0;

      /* Notify any waiters that there is a
       * change in the connection state
       */

      if (priv->pscwait)
        {
          nxsem_post(&priv->pscsem);
          priv->pscwait = false;
        }
    }
}

/****************************************************************************
 * USB Host Controller Operations
 ****************************************************************************/

/****************************************************************************
 * Name: sam_wait
 *
 * Description:
 *   Wait for a device to be connected or disconnected to/from a hub port.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter
 *    from the call to the USB driver initialization logic.
 *   hport - The location to return the hub port descriptor that detected the
 *      connection related event.
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

static int sam_wait(struct usbhost_connection_s *conn,
                    struct usbhost_hubport_s **hport)
{
  struct sam_usbhost_s *priv = &g_usbhost;
  struct usbhost_hubport_s *connport;
  irqstate_t flags;

  /* Loop until a change in connection state is detected */

  flags = enter_critical_section();
  for (; ; )
    {
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
          leave_critical_section(flags);

          uinfo("RHport Connected: %s\n", connport->connected ?
                                                  "YES" : "NO");
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
          leave_critical_section(flags);

          uinfo("Hub port Connected: %s\n", connport->connected ?
                                                   "YES" : "NO");
          return OK;
        }
 #endif

      /* Wait for the next connection event */

      priv->pscwait = true;
      nxsem_wait_uninterruptible(&priv->pscsem);
    }
}

/****************************************************************************
 * Name: sam_rh_enumerate
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
 *   On success, zero (OK) is returned. On a failure,
 *   a negated errno value is returned
 *   indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_rh_enumerate(struct sam_usbhost_s *priv,
                            struct usbhost_connection_s *conn,
                            struct usbhost_hubport_s *hport)
{
  uint32_t regval;
  int ret;

  uinfo("ENTRY\n");
  DEBUGASSERT(conn != NULL && hport != NULL && hport->port == 0);

  /* Are we connected to a device?  The caller should have called the wait()
   * method first to be assured that a device is connected.
   */

  while (!priv->connected)
    {
      /* No, return an error */

      usbhost_trace1(TRACE1_DEVDISCONN, 0);
      return -ENODEV;
    }

  DEBUGASSERT(priv->smstate == SMSTATE_ATTACHED);

  /* USB 2.0 spec says at least 50ms delay before port reset. */

  nxsig_usleep(100 * 1000);

  /* Reset the host port */

  sam_hostreset(priv);

  /* Get the current device speed */

  regval = sam_getreg8(SAM_USBHOST_STATUS);
  if ((regval & USBDEV_STATUS_SPEED_MASK) == USBDEV_STATUS_SPEED_LOW)
      priv->rhport.hport.speed = USB_SPEED_LOW;
  else
      priv->rhport.hport.speed = USB_SPEED_FULL;

  /* Allocate and initialize the root hub port EP0 pipes */

  if (priv->pipelist[0].inuse == false)
    {
      struct usbhost_epdesc_s epdesc = {
                                        .hport = hport,
                                        .addr = 0
                                       };

      ret = sam_ctrlep_alloc(priv, &epdesc, &priv->ep0);
      if (ret < 0)
          uerr("ERROR: Failed to allocate a control endpoint: %d\n", ret);
    }
  else
      ret = OK;

  return ret;
}

static int sam_enumerate(struct usbhost_connection_s *conn,
                         struct usbhost_hubport_s *hport)
{
  struct sam_usbhost_s *priv = &g_usbhost;
  int ret;

  uinfo("ENTRY\n");
  DEBUGASSERT(hport);

  /* If this is a connection on the root hub, then we need to go to
   * little more effort to get the device speed.  If it is a connection
   * on an external hub, then we already have that information.
   */

 #ifdef CONFIG_USBHOST_HUB
  if (ROOTHUB(hport))
 #endif
    {
      ret = sam_rh_enumerate(priv, conn, hport);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Then let the common usbhost_enumerate do the real enumeration. */

  uinfo("Enumerate the device\n");
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

      uerr("ERROR: Enumeration failed: %d\n", ret);
      sam_gint_disconnected(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: sam_ep0configure
 *
 * Description:
 *   Configure endpoint 0.  This method is normally used internally by the
 *   enumerate() method but is made available at the interface to support an
 *   external implementation of the enumeration logic.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter
 *          from the call to the class create() method.
 *   ep0 - The (opaque) EP0 endpoint instance
 *   funcaddr - The USB address of the function containing the
 *   endpoint that EP0 controls.
 *   speed - The speed of the port USB_SPEED_LOW, _FULL, or _HIGH
 *   maxpacketsize - The maximum number of bytes that can be sent to or
 *    received from the endpoint in a single data packet
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated
 *   errno value isreturned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_ep0configure(struct usbhost_driver_s *drvr,
                            usbhost_ep_t ep0,
                            uint8_t funcaddr,
                            uint8_t speed,
                            uint16_t maxpacketsize)
{
  struct sam_usbhost_s *priv = (struct sam_usbhost_s *)drvr;
  struct sam_pipe_s *pipe;

  uinfo("funcaddr=%d speed=%d maxpacketsize=%d\n",
                  funcaddr, speed, maxpacketsize);
  DEBUGASSERT(drvr != NULL && funcaddr < 128 && maxpacketsize <= 64 &&
                                (unsigned int)ep0 < SAM_USB_NENDPOINTS);

  /* We must have exclusive access to the USB host
   * hardware and state structures
   */

  nxmutex_lock(&priv->lock);

  /* Configure the EP0 pipe */

  pipe            = &priv->pipelist[(unsigned int)ep0];
  pipe->funcaddr  = funcaddr;
  pipe->speed     = speed;
  pipe->maxpacket = maxpacketsize;
  sam_pipe_configure(priv, pipe->idx);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: sam_epalloc
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a
 *     parameter from the call to the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_epalloc(struct usbhost_driver_s *drvr,
                       const struct usbhost_epdesc_s *epdesc,
                       usbhost_ep_t *ep)
{
  struct sam_usbhost_s *priv = (struct sam_usbhost_s *)drvr;
  int ret;

  uwarn("addr=%d in=%d xfrtype=%d interval=%d mxpacketsize=%d\n",
    epdesc->addr, epdesc->in, epdesc->xfrtype,
    epdesc->interval, epdesc->mxpacketsize);

  /* Sanity check.  NOTE that this method should only be called if a device
   * is connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(drvr != 0 && epdesc != NULL && ep != NULL);

  /* We must have exclusive access to the USB
   * host hardware and state structures
   */

  nxmutex_lock(&priv->lock);

  /* Handler control pipes differently from other endpoint types.  This is
   * because the normal, "transfer" endpoints are unidirectional an require
   * only a single pipe.  Control endpoints, however, are bi-diretional
   * and require two pipes, one for the IN and one for the OUT direction.
   */

  if (epdesc->xfrtype == USB_EP_ATTR_XFER_CONTROL)
    {
      ret = sam_ctrlep_alloc(priv, epdesc, ep);
    }
  else
    {
      ret = sam_xfrep_alloc(priv, epdesc, ep);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: sam_epfree
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *    call to the class create() method.
 *   ep - The endpoint to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_epfree(struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct sam_usbhost_s *priv = (struct sam_usbhost_s *)drvr;

  DEBUGASSERT(priv);

  /* We must have exclusive access to the
   * USB host hardware and state structures
   */

  nxmutex_lock(&priv->lock);

  /* Halt the pipe and mark the pipe available */

  sam_pipe_free(priv, (intptr_t)ep);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: sam_alloc
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.
 *   This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does
 *   not support such "special" memory, this functions
 *   may simply map to kmm_malloc.
 *   This interface was optimized under a particular assumption. It was
 *   assumed that the driver maintains a pool of small, pre-allocated buffers
 *   for descriptor traffic. NOTE that size is not an input, but an output:
 *   The size of the pre-allocated buffer is returned.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter
 *      from the call to the class create() method.
 *   buffer - The address of a memory location provided by
 *     the caller in which to return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by
 *     the caller in which to return the maximum size of the allocated
 *     buffer memory.
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

static int sam_alloc(struct usbhost_driver_s *drvr,
                     uint8_t **buffer, size_t *maxlen)
{
  uint8_t *alloc;

  uinfo("ENTRY\n");
  DEBUGASSERT(drvr && buffer && maxlen);

  /* There is no special memory requirement for the SAM. */

  alloc = (uint8_t *)kmm_malloc(CONFIG_SAM_DESCSIZE);
  if (!alloc)
    {
      return -ENOMEM;
    }

  /* Return the allocated address and size of the descriptor buffer */

  *buffer = alloc;
  *maxlen = CONFIG_SAM_DESCSIZE;
  return OK;
}

/****************************************************************************
 * Name: sam_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.
 *   This method provides a mechanism to free that
 *   request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter
 *     from the call to the class create() method.
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

static int sam_free(struct usbhost_driver_s *drvr, uint8_t *buffer)
{
  /* There is no special memory requirement */

  uinfo("ENTRY\n");
  DEBUGASSERT(drvr && buffer);

  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: sam_ioalloc
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to
 *   allocate the request/descriptor memory.
 *   If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to kmm_malloc.
 *
 * This interface differs from DRVR_ALLOC in that the
 * buffers are variable-sized.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   buffer - The address of a memory location provided by the caller
 *     in which to return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno
 *   value is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_ioalloc(struct usbhost_driver_s *drvr,
                       uint8_t **buffer, size_t buflen)
{
  uint8_t *alloc;

  uinfo("ENTRY\n");
  DEBUGASSERT(drvr && buffer && buflen > 0);

  /* There is no special memory requirement */

  alloc = (uint8_t *)kmm_malloc(buflen);
  if (!alloc)
    {
      return -ENOMEM;
    }

  /* Return the allocated buffer */

  *buffer = alloc;
  return OK;
}

/****************************************************************************
 * Name: sam_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can
 *   be accessed more efficiently.
 *   This method provides a mechanism to free that IO buffer
 *   memory.  If the underlying hardware does not support such
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *       call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno
 *   value is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_iofree(struct usbhost_driver_s *drvr, uint8_t *buffer)
{
  /* There is no special memory requirement */

  uinfo("ENTRY\n");
  DEBUGASSERT(drvr && buffer);

  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: sam_ctrlin and sam_ctrlout
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint. These methods
 *   will enqueue the request and wait for it to complete.
 *   Only one transfer may be queued;
 *   Neither these methods nor the transfer() method can be called again
 *   until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   ep0 - The control endpoint to send/receive the control request.
 *   req - Describes the request to be sent.  This request must lie
 *       in memory created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *     responses.  This buffer must be large enough to hold the length value
 *     in the request description.
 *     Buffer must have been allocated using DRVR_ALLOC.
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same
 *    allocated memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno
 *    value is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_ctrlin(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                      const struct usb_ctrlreq_s *req,
                      uint8_t *buffer)
{
  struct sam_usbhost_s *priv = (struct sam_usbhost_s *)drvr;
  struct sam_pipe_s *pipe;
  uint16_t buflen;
  clock_t start;
  clock_t elapsed;
  int retries;
  int ret;

  DEBUGASSERT(priv != NULL && req != NULL &&
                        (unsigned int)ep0 < SAM_USB_NENDPOINTS);
  usbhost_vtrace2(SAM_VTRACE2_CTRLIN, req->type, req->req);

  pipe = &priv->pipelist[(unsigned int)ep0];

  /* Extract values from the request */

  buflen = sam_getle16(req->len);
  uinfo("type:0x%02x req:0x%02x value:0x%02x%02x index:0x%02x%02x len:%d\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], buflen);

  /* We must have exclusive access to the USB
   * host hardware and state structures
   */

  nxmutex_lock(&priv->lock);

  /* Loop, retrying until the retry time expires */

  for (retries = 0; retries < SAM_RETRY_COUNT; retries++)
    {
      /* Send the SETUP request (TXSTP) */

      ret = sam_ctrl_sendsetup(priv, pipe, req);
      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_SENDSETUP_FAIL2, -ret);
          continue;
        }

      /* Get the start time.  Loop again until the timeout expires */

      start = clock_systime_ticks();
      do
        {
          /* Handle the IN data phase (if any) (TRCPT) */

          if (buflen > 0)
            {
              ret = sam_ctrl_recvdata(priv, pipe, buffer, buflen);
              if (ret < 0)
                {
                  usbhost_trace1(SAM_TRACE1_RECVDATA_FAIL, -ret);
                }
            }

          /* Handle the status OUT phase */

          if (ret == OK)
            {
              ret = sam_ctrl_senddata(priv, pipe, NULL, 0);
              if (ret == OK)
                {
                  /* All success transactions exit here */

                  nxmutex_unlock(&priv->lock);
                  return OK;
                }

              usbhost_trace1(SAM_TRACE1_SENDSTATUS_FAIL, ret < 0 ?
                                                       -ret : ret);
            }

          /* Get the elapsed time (in frames) */

          elapsed = clock_systime_ticks() - start;
        }
      while (elapsed < SAM_DATANAK_DELAY);
    }

  /* All failures exit here after all retries
   * and timeouts have been exhausted
   */

  nxmutex_unlock(&priv->lock);
  return -ETIMEDOUT;
}

static int sam_ctrlout(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                       const struct usb_ctrlreq_s *req,
                       const uint8_t *buffer)
{
  struct sam_usbhost_s *priv = (struct sam_usbhost_s *)drvr;
  struct sam_pipe_s *pipe;
  uint16_t buflen;
  clock_t start;
  clock_t elapsed;
  int retries;
  int ret;

  DEBUGASSERT(priv != NULL && req != NULL &&
             (unsigned int)ep0 < SAM_USB_NENDPOINTS);
  usbhost_vtrace2(SAM_VTRACE2_CTRLOUT, req->type, req->req);

  pipe = &priv->pipelist[(unsigned int)ep0];

  /* Extract values from the request */

  buflen = sam_getle16(req->len);
  uinfo("type:0x%02x req:0x%02x value:0x%02x%02x index:0x%02x%02x len:%d\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], buflen);

  /* We must have exclusive access to the
   * USB host hardware and state structures
   */

  nxmutex_lock(&priv->lock);

  /* Loop, retrying until the retry time expires */

  for (retries = 0; retries < SAM_RETRY_COUNT; retries++)
    {
      /* Send the SETUP request */

      ret = sam_ctrl_sendsetup(priv, pipe, req);
      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_SENDSETUP_FAIL1, -ret);
          continue;
        }

      /* Get the start time.  Loop again until the timeout expires */

      start = clock_systime_ticks();
      do
        {
          /* Handle the data OUT phase (if any) */

          if (buflen > 0)
            {
              /* Start DATA out transfer (only one DATA packet) */

              ret = sam_ctrl_senddata(priv, pipe,
                                     (uint8_t *)buffer, buflen);
              if (ret < 0)
                {
                  usbhost_trace1(SAM_TRACE1_SENDDATA_FAIL, -ret);
                }
            }

          /* Handle the status IN phase */

          if (ret == OK)
            {
              ret = sam_ctrl_recvdata(priv, pipe, NULL, 0);
              if (ret == OK)
                {
                  /* All success transactins exit here */

                  nxmutex_unlock(&priv->lock);
                  return OK;
                }

              usbhost_trace1(SAM_TRACE1_RECVSTATUS_FAIL,
                             ret < 0 ? -ret : ret);
            }

          /* Get the elapsed time (in frames) */

          elapsed = clock_systime_ticks() - start;
        }
      while (elapsed < SAM_DATANAK_DELAY);
    }

  /* All failures exit here after all retries
   * and timeouts have been exhausted
   */

  nxmutex_unlock(&priv->lock);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: sam_transfer
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request, blocking until the transfer completes.
 *   Only one transfer may be  queued; Neither this method nor the ctrlin
 *   or ctrlout methods can be called again until this function returns.
 *
 *   This is a blocking method; this functions will not return until the
 *   transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on
 *     which to perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or
 *    received (IN endpoint). Buffer must have been allocated using
 *    DRVR_ALLOC.
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

static ssize_t sam_transfer(struct usbhost_driver_s *drvr,
                            usbhost_ep_t ep,
                            uint8_t *buffer,
                            size_t buflen)
{
  struct sam_usbhost_s *priv = (struct sam_usbhost_s *)drvr;
  struct sam_pipe_s *pipe;
  unsigned int idx = (unsigned int)ep;
  ssize_t nbytes;

  uwarn("pipe%d buffer:%p buflen:%d\n",  idx, buffer, buflen);

  DEBUGASSERT(priv && buffer && idx < SAM_USB_NENDPOINTS && buflen > 0);
  pipe = &priv->pipelist[idx];

  /* We must have exclusive access to the
   * USB host hardware and state structures
   */

  nxmutex_lock(&priv->lock);

  /* Handle IN and OUT transfer slightly differently */

  if (pipe->in)
    {
      nbytes = sam_in_transfer(priv, pipe, buffer, buflen);
    }
  else
    {
      nbytes = sam_out_transfer(priv, pipe, buffer, buflen);
    }

  nxmutex_unlock(&priv->lock);
  return nbytes;
}

/****************************************************************************
 * Name: sam_asynch
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
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on
 *      which to perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint)
 *      or received (IN endpoint). Buffer must have been allocated using
 *      DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *      callback - This function will be called when the transfer completes.
 *   arg - The arbitrary parameter that will be passed to the callback
 *      function when the transfer completes.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno
 *   value is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int sam_asynch(struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                      uint8_t *buffer, size_t buflen,
                      usbhost_asynch_t callback, void *arg)
{
  struct sam_usbhost_s *priv = (struct sam_usbhost_s *)drvr;
  struct sam_pipe_s *pipe;
  unsigned int idx = (unsigned int)ep;
  int ret;

  uinfo("idx: %d buflen: %d\n",  (unsigned int)ep, buflen);

  DEBUGASSERT(priv && buffer && idx < SAM_USB_NENDPOINTS && buflen > 0);
  pipe = &priv->pipelist[idx];

  /* We must have exclusive access to the
   * USB host hardware and state structures
   */

  nxmutex_lock(&priv->lock);

  /* Handle IN and OUT transfer slightly differently */

  if (pipe->in)
    {
      ret = sam_in_asynch(priv, pipe, buffer, buflen, callback, arg);
    }
  else
    {
      ret = sam_out_asynch(priv, pipe, buffer, buflen, callback, arg);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif /* CONFIG_USBHOST_ASYNCH */

/****************************************************************************
 * Name: sam_cancel
 *
 * Description:
 *   Cancel a pending transfer on an endpoint.  Cancelled synchronous or
 *   asynchronous transfer will complete normally with the error -ESHUTDOWN.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on
 *      which an asynchronous transfer should be transferred.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 ****************************************************************************/

static int sam_cancel(struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct sam_usbhost_s *priv = (struct sam_usbhost_s *)drvr;
  struct sam_pipe_s *pipe;
  unsigned int idx = (unsigned int)ep;
  irqstate_t flags;

  uinfo("idx: %u: %d\n",  idx);

  DEBUGASSERT(priv && idx < SAM_USB_NENDPOINTS);
  pipe = &priv->pipelist[idx];

  /* We need to disable interrupts to avoid race conditions with the
   * asynchronous completion of the transfer being cancelled.
   */

  flags = enter_critical_section();

  /* Halt the pipe */

  sam_transfer_abort(priv, pipe, CHREASON_CANCELLED);
  pipe->result = -ESHUTDOWN;

  /* Is there a thread waiting for this transfer to complete? */

  if (pipe->waiter)
    {
#ifdef CONFIG_USBHOST_ASYNCH
  /* Yes.. there should not also be a callback scheduled */

  DEBUGASSERT(pipe->callback == NULL);
#endif

      /* Wake'em up! */

      nxsem_post(&pipe->waitsem);
      pipe->waiter = false;
    }

#ifdef CONFIG_USBHOST_ASYNCH
  /* No.. is an asynchronous callback expected when the transfer
   * completes?
   */

  else if (pipe->callback)
    {
      usbhost_asynch_t callback;
      void *arg;

      /* Extract the callback information */

      callback       = pipe->callback;
      arg            = pipe->arg;

      pipe->callback = NULL;
      pipe->arg      = NULL;
      pipe->count = 0;

      /* Then perform the callback */

      callback(arg, -ESHUTDOWN);
    }
#endif

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_connect
 *
 * Description:
 *   New connections may be detected by an attached hub.  This method is the
 *   mechanism that is used by the hub class to introduce a new connection
 *   and port description to the system.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   hport - The descriptor of the hub port that detected the connection
 *      related event
 *   connected - True: device connected; false: device disconnected
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_HUB
static int sam_connect(struct usbhost_driver_s *drvr,
                       struct usbhost_hubport_s *hport,
                       bool connected)
{
  struct sam_usbhost_s *priv = (struct sam_usbhost_s *)drvr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL && hport != NULL);

  /* Set the connected/disconnected flag */

  hport->connected = connected;
  uinfo("Hub port %d connected: %s\n",
                                      hport->port, connected ? "YES" : "NO");

  /* Report the connection event */

  flags = enter_critical_section();
  priv->hport = hport;
  if (priv->pscwait)
    {
      priv->pscwait = false;
      nxsem_post(&priv->pscsem);
    }

  leave_critical_section(flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: sam_disconnect
 *
 * Description:
 *   Called by the class when an error occurs and driver has been
 *   disconnected. The USB host driver should discard the handle to the
 *   class instance (it is stale) and not attempt any further interaction
 *   with the class driver instance (until a new instance is received from
 *   the create() method). The driver should not called the class
 *   disconnected() method.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   hport - The port from which the device is being disconnected.
 *      Might be a port on a hub.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static void sam_disconnect(struct usbhost_driver_s *drvr,
                           struct usbhost_hubport_s *hport)
{
  DEBUGASSERT(hport != NULL);
  hport->devclass = NULL;
}

/****************************************************************************
 * Pipe Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: sam_reset_pipes
 *
 * Description:
 *   Reset pipes
 *  \param     priv       Pointer to driver instance
 *  \param[in] warm_reset Handles runtime USB reset
 *
 ****************************************************************************/

static void sam_reset_pipes(struct sam_usbhost_s *priv, bool warm_reset)
{
  struct sam_pipe_s *pipe;
  uint8_t i;

  /* Reset pipes */

  for (i = 0; i < SAM_USB_NENDPOINTS; i++)
    {
      /* Get the pipe structure */

      pipe = &priv->pipelist[i];

      if (warm_reset)
        {
          /* Skip free pipes */

          if (pipe->pipestate_general == USB_H_PIPE_S_FREE)
            continue;

          /* Restore physical pipe configurations */

          sam_pipe_configure(priv, i);

          /* Abort transfer (due to reset) */

          if (pipe->eptype == USB_EP_ATTR_XFER_CONTROL)
            {
              /* Force a callback for control endpoints */

              pipe->pipestate_general = USB_H_PIPE_S_SETUP;
            }

          sam_transfer_terminate(priv, pipe, USB_H_RESET);
        }
      else
        pipe->pipestate_general = USB_H_PIPE_S_FREE;
    }
}

/****************************************************************************
 * Name: sam_pipe_reset
 *
 * Description:
 *   Reset and disable one pipe.
 *
 ****************************************************************************/

static void sam_pipe_reset(struct sam_usbhost_s *priv, uint8_t epno)
{
  struct sam_pipe_s *pipe = &priv->pipelist[epno];

  uinfo("pipe%d\n", pipe->idx);

  /* Disable pipe interrupts */

  sam_putreg8(0x3f, SAM_USBHOST_PINTENCLR(epno));
  sam_putreg8(0x3f, SAM_USBHOST_PINTFLAG(epno));

  /* Reset pipe status */

  pipe->pipestate = USB_H_PIPE_S_FREE;
  pipe->stalled   = false;
  pipe->pending   = false;
  pipe->halted    = false;
  pipe->zlpsent   = false;
  pipe->txbusy    = false;
  pipe->rxactive  = false;
}

/****************************************************************************
 * Name: sam_pipeset_reset
 *
 * Description:
 *   Reset and disable a set of pipes.
 *
 ****************************************************************************/

static void sam_pipeset_reset(struct sam_usbhost_s *priv, uint16_t epset)
{
  uint32_t bit;
  int epno;

  uinfo("ENTRY\n");

  /* Reset each pipe in the set */

  for (epno = 0, bit = 1, epset &= SAM_EPSET_ALL;
       epno < SAM_USB_NENDPOINTS && epset != 0;
       epno++, bit <<= 1)
    {
      /* Is this pipe in the set? */

      if ((epset & bit) != 0)
        {
           sam_pipe_reset(priv, epno);
           epset &= ~bit;
        }
    }
}

/****************************************************************************
 * Name: sam_vbusdrive
 *
 * Description:
 *   Drive the Vbus +5V.
 *
 * Input Parameters:
 *   priv  - USB host driver private data structure.
 *   state - True: Drive, False: Don't drive
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void sam_vbusdrive(struct sam_usbhost_s *priv, bool state)
{
  /* Enable/disable the external charge pump */

  sam_usbhost_vbusdrive(0, state);

  up_mdelay(200);
}

/****************************************************************************
 * Name: sam_pipe_interrupt
 *
 * Description:
 *   Handle the USB pipe interrupt
 *
 ****************************************************************************/

static void sam_pipe_interrupt(struct sam_usbhost_s *priv, int idx)
{
  struct sam_pipe_s *pipe;
  uint8_t pipisr;
  uint8_t pipimr;

  DEBUGASSERT((unsigned)idx < SAM_USB_NENDPOINTS);

  /* Get the pipe structure */

  pipe = &priv->pipelist[idx];

  /* Get the pipe irq */

  pipisr = sam_getreg8(SAM_USBHOST_PINTFLAG(idx));
  pipimr = sam_getreg8(SAM_USBHOST_PINTENSET(idx));
  uinfo("pipe%d PINTFLAG:0x%x PINTENSET:0x%x\n", idx, pipisr, pipimr);

  /* Host pipe stall interrupt */

  if (pipisr & USBHOST_PINTFLAG_STALL)
    {
      /* clear the flag */

      sam_putreg8(USBHOST_PINTFLAG_STALL, SAM_USBHOST_PINTFLAG(idx));
      uwarn("STALL =0x%x pktsize=0x%x ctrlpipe=0x%x statuspipe=0x%x\n",
                                            pipisr,
                                            pipe->descb[0]->pktsize,
                                            pipe->descb[0]->ctrlpipe,
                                            pipe->descb[0]->statuspipe);

      sam_transfer_abort(priv, pipe, USB_H_STALL);
      return;
    }

  /* Host pipe error interrupt */

  if (pipisr & USBHOST_PINTFLAG_PERR)
    {
      /* clear the flag */

      sam_putreg8(USBHOST_PINTFLAG_PERR, SAM_USBHOST_PINTFLAG(idx));
      uwarn("PERR =0x%x pktsize=0x%x ctrlpipe=0x%x statuspipe=0x%x\n",
              pipisr,
              pipe->descb[0]->pktsize,
              pipe->descb[0]->ctrlpipe,
              pipe->descb[0]->statuspipe);

      /* Get and ACK error */

      switch (pipe->descb[0]->statuspipe & (USBHOST_STATUSPIPE_DTGLER |
                                           USBHOST_STATUSPIPE_TOUTER |
                                           USBHOST_STATUSPIPE_DAPIDER))
        {
          case USBHOST_STATUSPIPE_DTGLER:
            pipe->pipestatus_general = USB_H_ERR;
          break;

          case USBHOST_STATUSPIPE_TOUTER:
            pipe->pipestatus_general = USB_H_TIMEOUT;
            pipe->result = -ETIMEDOUT;
          break;

          case USBHOST_STATUSPIPE_PIDER:

          case USBHOST_STATUSPIPE_DAPIDER:
          default:
            pipe->pipestatus_general = USB_H_ERR;
          break;
        }

      pipe->descb[0]->statuspipe = 0;

      sam_transfer_abort(priv, pipe, USB_H_ERR);
      return;
    }

  /* Host pipe transfer fail interrupt */

  if (pipisr & USBHOST_PINTFLAG_TRFAIL)
    {
      /* clear the flag */

      sam_putreg8(USBHOST_PINTFLAG_TRFAIL, SAM_USBHOST_PINTFLAG(idx));
      uwarn("TRFAIL =0x%x pktsize=0x%x ctrlpipe=0x%x statuspipe=0x%x\n",
                    pipisr,
                    pipe->descb[0]->pktsize,
                    pipe->descb[0]->ctrlpipe,
                    pipe->descb[0]->statuspipe);

      uint8_t status_bk = pipe->descb[0]->stausbk;
      if (status_bk)
        {
          pipe->descb[0]->stausbk = 0;

          /* Ignore ERRORFLOW and handle CRCERR */

          if (pipe->eptype != USB_EP_ATTR_XFER_ISOC &&
              status_bk == USBHOST_STATUSBK_ERRORFLOW)
            {
              /* Ignore ERRORFLOW on none ISO pipes */
            }
          else
            {
              pipe->pipestatus_general = USB_H_ERR;
              sam_transfer_abort(priv, pipe, USB_H_ERR);
            }
        }

      return;
    }

  /* TRCPT: transfer complete */

  if ((pipisr & pipimr & (USBHOST_PINTFLAG_TRCPT0 |
                          USBHOST_PINTFLAG_TRCPT1)) != 0)
  {
    /* clear the flag */

    sam_putreg8((USBHOST_PINTFLAG_TRCPT0 |
                 USBHOST_PINTFLAG_TRCPT1), SAM_USBHOST_PINTFLAG(idx));

    if ((sam_getreg8(SAM_USBHOST_PCFG(idx)) & USBHOST_PCFG_PTOKEN_MASK) ==
                                              USBHOST_PCFG_PTOKEN_IN)
      {
        uwarn("pipe%d IN TRCPT pktsize=0x%x\n",
                                idx,
                                pipe->descb[0]->pktsize);
        if (idx > 0)
          sam_recv_continue(priv, pipe);
        pipe->result = 0;
        sam_pipe_wakeup(priv, pipe);
      }
    else
      {
        uwarn("pipe%d OUT TRCPT pktsize=0x%x\n",
                      idx,
                      pipe->descb[0]->pktsize);
        if (idx > 0)
          sam_send_continue(priv, pipe);

        pipe->result = 0;
        sam_pipe_wakeup(priv, pipe);
      }

    return;
  }

  /* Host pipe transmitted setup interrupt */

  if (pipisr & pipimr & USBHOST_PINTFLAG_TXSTP)
    {
      /* clear the flag */

      sam_putreg8(USBHOST_PINTFLAG_TXSTP, SAM_USBHOST_PINTFLAG(idx));
      sam_putreg8(USBHOST_PINTFLAG_TXSTP, SAM_USBHOST_PINTENCLR(idx));

      /* Reset data toggle for DATA */

      sam_putreg8(USBHOST_PSTATUS_DTGL, SAM_USBHOST_PSTATUSSET(idx));

      /* Start DATA phase */

      if (priv->ctrl_buffer[0] & 0x80) /* 1 = Device to Host */
        {
          /* IN */

          uwarn("pipe%d IN TXSTP\n", idx);
          pipe->pipestate = USB_H_PIPE_S_DATI; /* Pipe in data IN stage */

          /* Start IN requests */

          pipe->result = 0;
          sam_pipe_wakeup(priv, pipe);
        }
      else
        {
          /* OUT */

          uwarn("pipe%d OUT TXSTP\n", idx);
            if (priv->ctrl_buffer[6] || priv->ctrl_buffer[7]) /* setup packet wLength[2] */
            {
              pipe->pipestate = USB_H_PIPE_S_DATO; /* Pipe in data OUT stage */

              /* Start OUT */

              pipe->result = 0;
              sam_pipe_wakeup(priv, pipe);
            }
          else
            {
              /* No DATA phase */

              uwarn("pipe%d OUT TXSTP ZLP\n", idx);
              pipe->pipestate = USB_H_PIPE_S_STATI; /* Pipe in control status IN stage */

              /* Start IN ZLP request */

              pipe->result = 0;
              sam_pipe_wakeup(priv, pipe);
            }
        }

      return;
    }
}

/****************************************************************************
 * Name: sam_usbhost_interrupt
 *
 * Description:
 *   Handle the USB interrupt.
 *   Host Mode
 *
 ****************************************************************************/

static int sam_usbhost_interrupt(int irq, void *context, void *arg)
{
  struct sam_usbhost_s *priv = (struct sam_usbhost_s *)arg;
  uint16_t isr;
  uint16_t imr;
  uint16_t regval;
  uint16_t pending;
  uint16_t pendingpipe;
  int i;

  /* Get the set of pending device interrupts */

  isr = sam_getreg16(SAM_USBHOST_INTFLAG);
  imr = sam_getreg16(SAM_USBHOST_INTENSET);
  pending = isr & imr;

  /* Get the set of pending Pipe interrupts */

  pendingpipe = sam_getreg16(SAM_USBHOST_PINTSMRY);

  /* Handle all pending USB interrupts */

  /* Serve Pipe Interrupts first */

  if (pendingpipe)
    {
      usbhost_vtrace1(TRACE_INTDECODE(SAM_TRACEINTID_PENDING_PIPE),
                      pendingpipe);
      for (i = 0; i < SAM_USB_NENDPOINTS; i++)
        {
          if ((pendingpipe & USBHOST_PINTSMRY_PIPEINT(i)))
            {
              usbhost_vtrace1(TRACE_INTDECODE(SAM_TRACEINTID_PIPENO),
                             (uint16_t)i);
              sam_pipe_interrupt(priv, i);
            }
        }
    }
  else
    {
      /* Host SOF interrupt */

      if (isr & USBHOST_INT_HSOF) /* Clear the flag */
        sam_putreg16(USBHOST_INT_HSOF, SAM_USBHOST_INTFLAG);
      if (isr & USBHOST_INT_RST) /* Host reset interrupt */
        {
          sam_putreg16(USBHOST_INT_RST, SAM_USBHOST_INTFLAG);
          sam_reset_pipes(priv, true);
          return OK;
        }
      else if (pending & USBHOST_INT_DDISC)
        {
          /* Host disconnect interrupt */

          /* clear the flag */

          sam_putreg16((USBHOST_INT_DDISC |
                        USBHOST_INT_DCONN), SAM_USBHOST_INTFLAG);

          /* Disable disconnect interrupt.
           * Disable wakeup/resumes interrupts,
           * in case of disconnection during suspend mode
           */

          sam_putreg16((USBHOST_INT_DDISC  |
                        USBHOST_INT_WAKEUP |
                        USBHOST_INT_DNRSM  |
                        USBHOST_INT_UPRSM), SAM_USBHOST_INTENCLR);

          /* Stop reset signal, in case of disconnection during reset */

          regval = sam_getreg16(SAM_USBHOST_CTRLB);
          regval &= ~USBHOST_CTRLB_BUSRESET;
          sam_putreg16(regval, SAM_USBHOST_CTRLB);

          /* Enable connection and wakeup interrupts */

          sam_putreg16((USBHOST_INT_DCONN  |
                        USBHOST_INT_WAKEUP |
                        USBHOST_INT_DNRSM  |
                        USBHOST_INT_UPRSM), SAM_USBHOST_INTFLAG);
          sam_putreg16((USBHOST_INT_DCONN  |
                        USBHOST_INT_WAKEUP |
                        USBHOST_INT_DNRSM  |
                        USBHOST_INT_UPRSM), SAM_USBHOST_INTENSET);

          priv->suspend_start = 0;
          priv->resume_start  = 0;

          /* PORT_CONNECTION: connect status changed */

          sam_gint_disconnected(priv);
          return OK;
        }
      else if (pending & USBHOST_INT_DCONN)
        {
          /* Host connect interrupt */

          /* clear the flag */

          /* Reserve the CONN flag for connection check */

          sam_putreg16 (USBHOST_INT_DCONN, SAM_USBHOST_INTENCLR);

          /* Enable disconnection interrupt */

          sam_putreg16(USBHOST_INT_DDISC, SAM_USBHOST_INTFLAG);
          sam_putreg16(USBHOST_INT_DDISC, SAM_USBHOST_INTENSET);

          /* Enable SOF */

          regval  = sam_getreg16(SAM_USBHOST_CTRLB);
          regval |= USBHOST_CTRLB_SOFE | USBHOST_CTRLB_BUSRESET;
          sam_putreg16(regval, SAM_USBHOST_CTRLB);

          priv->suspend_start = 0;
          priv->resume_start  = 0;

          /* PORT_CONNECTION: connect status changed */

          sam_gint_connected(priv);
          return OK;
        }

          /* Wake up to power */

      if ((isr & USBHOST_INT_WAKEUP) && (imr & USBHOST_INT_DCONN))
        {
          /* clear the flag */

          sam_putreg16(USBHOST_INT_WAKEUP, SAM_USBHOST_INTFLAG);
          sam_vbusdrive(priv, true);
        }

          /* Resume */

      if (pending & (USBHOST_INT_WAKEUP |
                     USBHOST_INT_UPRSM  |
                    USBHOST_INT_DNRSM))
        {
          sam_putreg16((USBHOST_INT_WAKEUP |
                        USBHOST_INT_UPRSM  |
                        USBHOST_INT_DNRSM), SAM_USBHOST_INTFLAG);

          sam_putreg16((USBHOST_INT_WAKEUP |
                        USBHOST_INT_UPRSM  |
                        USBHOST_INT_DNRSM), SAM_USBHOST_INTENCLR);

          sam_putreg16((USBHOST_INT_RST |
                        USBHOST_INT_DDISC), SAM_USBHOST_INTENSET);

          /* Enable SOF */

          regval  = sam_getreg16(SAM_USBHOST_CTRLB);
          regval |= USBHOST_CTRLB_RESUME | USBHOST_CTRLB_SOFE;
          sam_putreg16(regval, SAM_USBHOST_CTRLB);

          /* Wait 50ms before restarting transfer */

          priv->resume_start = 50;
          sam_add_sof_user(priv);
          return OK;
        }
    }

  /* Just ignore unexpected interrupts */

  sam_putreg16(isr, SAM_USBHOST_INTFLAG);
  return OK;
}

/****************************************************************************
 * Initialization/Reset
 ****************************************************************************/

/****************************************************************************
 * Name: sam_hostreset
 ****************************************************************************/

static void sam_hostreset(struct sam_usbhost_s *priv)
{
  uint16_t regval;

  uinfo("ENTRY\n");

  /* Make sure that clocking is enabled to the USB peripheral. */

  sam_enableclks();
  priv->hoststate = USB_HOSTSTATE_DEFAULT;
  regval = sam_getreg16(SAM_USBHOST_CTRLB);
  regval &= ~USBHOST_CTRLB_BUSRESET;
  sam_putreg16(regval, SAM_USBHOST_CTRLB);

  /* Clear all pending interrupt status */

  regval = USBHOST_INT_HSOF | USBHOST_INT_RST | USBHOST_INT_WAKEUP |
           USBHOST_INT_DNRSM | USBHOST_INT_UPRSM | USBHOST_INT_RAMACER |
           USBHOST_INT_DCONN | USBHOST_INT_DDISC;
  sam_putreg16(regval, SAM_USBHOST_INTFLAG);

  /* Enable normal operational interrupts */

  regval = USBHOST_INT_WAKEUP | USBHOST_INT_DDISC;
  sam_putreg16(regval, SAM_USBHOST_INTENSET);

  sam_dumppipe(priv, EP0);
}

/****************************************************************************
 * Name: sam_host_initialize
 *
 * Description:
 *   Initialize/re-initialize hardware for host mode operation.  At present,
 *   this function is called only from sam_hw_initialize().  But if OTG mode
 *   were supported, this function would also be called to swtich between
 *   host and device modes on a connector ID change interrupt.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void sam_host_initialize(struct sam_usbhost_s *priv)
{
  /* Drive Vbus +5V (the smoke test). Should be done elsewhere in OTG mode. */

  sam_vbusdrive(priv, true);

  /* Enable interrupts to detect connection */

  sam_putreg16(USBHOST_INT_DCONN |
               USBHOST_INT_RST   |
               USBHOST_INT_WAKEUP, SAM_USBHOST_INTENSET);
}

/****************************************************************************
 * Name: sam_sw_initialize
 *
 * Description:
 *   One-time setup of the host driver state structure.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void sam_sw_initialize(struct sam_usbhost_s *priv)
{
  struct usbhost_driver_s *drvr;
  struct usbhost_hubport_s *hport;
  int epno;

  /* Initialize the device operations */

  drvr                 = &priv->drvr;
  drvr->ep0configure   = sam_ep0configure;
  drvr->epalloc        = sam_epalloc;
  drvr->epfree         = sam_epfree;
  drvr->alloc          = sam_alloc;
  drvr->free           = sam_free;
  drvr->ioalloc        = sam_ioalloc;
  drvr->iofree         = sam_iofree;
  drvr->ctrlin         = sam_ctrlin;
  drvr->ctrlout        = sam_ctrlout;
  drvr->transfer       = sam_transfer;
#ifdef CONFIG_USBHOST_ASYNCH
  drvr->asynch         = sam_asynch;
#endif
  drvr->cancel         = sam_cancel;
#ifdef CONFIG_USBHOST_HUB
  drvr->connect        = sam_connect;
#endif
  drvr->disconnect     = sam_disconnect;

  /* Initialize the public port representation */

  hport                = &priv->rhport.hport;
  hport->drvr          = drvr;
#ifdef CONFIG_USBHOST_HUB
  hport->parent        = NULL;
#endif
  hport->ep0           = priv->ep0;
  hport->speed         = USB_SPEED_FULL;

  /* Initialize function address generation logic */

  usbhost_devaddr_initialize(&priv->rhport);

  /* Initialize the pipe list */

  for (epno = 0; epno < SAM_USB_NENDPOINTS; epno++)
    {
      priv->pipelist[epno].idx = epno;
      nxsem_init(&priv->pipelist[epno].waitsem, 0, 0);
      sem_setprotocol(&priv->pipelist[epno].waitsem, SEM_PRIO_NONE);

      sam_putreg8(USBHOST_PSTATUS_PFREEZE, SAM_USBHOST_PSTATUSSET(epno));

  /* set descriptor addresses */

    priv->pipelist[epno].descb[0] = &priv->pipe_descriptors[(epno << 1)];
    priv->pipelist[epno].descb[1] = &priv->pipe_descriptors[(epno << 1) + 1];
    }

  sam_reset_pipes(priv, false);

  /* Initialize the driver state data */

  priv->smstate   = SMSTATE_DETACHED;
  priv->connected = false;
  priv->change    = false;

  priv->irqset    = 0;
}

/****************************************************************************
 * Name: sam_hw_initialize
 *
 * Description:
 *   One-time setup of the host controller harware for normal operations.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static inline int sam_hw_initialize(struct sam_usbhost_s *priv)
{
  uint16_t regval;
  uint32_t padcalib;
  uint8_t calib_transn;
  uint8_t calib_transp;
  uint8_t calib_trim;

  /* Set up the USB DP/DM pins */

  sam_portconfig(PORT_USB_DP | PORT_PULL_NONE);
  sam_portconfig(PORT_USB_DM | PORT_PULL_NONE);

  /* To use the USB, the programmer must first configure
   * inputthe USB clock
   */

  sam_enableclks();

  /* full reset USB */

  sam_ctrla_write(USB_CTRLA_SWRST);

  /* Load USB factory calibration values from NVRAM */

  calib_transn = (getreg32(SAM_FUSES_USBTRANSN_ADDR) &
                  SAM_FUSES_USBTRANSN_MASK) >> SAM_FUSES_USBTRANSN_SHIFT;
  if (calib_transn == 0 || calib_transn == 0x1f)
    calib_transn = 0x9;

  calib_transp = (getreg32(SAM_FUSES_USBTRANSP_ADDR) &
                  SAM_FUSES_USBTRANSP_ADDR) >> SAM_FUSES_USBTRANSP_SHIFT;
  if (calib_transp == 0 || calib_transp == 0x1f)
    calib_transp = 0x19;

  calib_trim   = (getreg32(SAM_FUSES_USBTRIM_ADDR) &
                  SAM_FUSES_USBTRIM_MASK) >> SAM_FUSES_USBTRIM_SHIFT;
  if (calib_trim == 0 || calib_trim == 0x7)
    calib_trim = 0x6;

  padcalib     = USB_PADCAL_TRANSP(calib_transp) |
                 USB_PADCAL_TRANSN(calib_transn) |
                 USB_PADCAL_TRIM(calib_trim);

  sam_putreg16(padcalib, SAM_USB_PADCAL);
  uinfo("PADCAL: 0x%x\n", padcalib); /* 0x6259 */

  sam_putreg8(USB_QOSCTRL_CQOS(3) | USB_QOSCTRL_DQOS(3), SAM_USB_QOSCTRL);

  /* Enable USB core */

  sam_ctrla_write(USB_CTRLA_ENABLE |
                 USB_CTRLA_RUNSTBY |
                 USB_CTRLA_MODE_HOST);

  /* Reset and disable pipes */

  sam_pipeset_reset(priv, SAM_EPSET_ALL);

  /* clear all previous descriptor data so no accidental
   * DMA transfers could happen
   */

  memset((uint8_t *)(&priv->pipe_descriptors[0]), 0,
                    sizeof(priv->pipe_descriptors));

  /* Init descriptor base address */

  sam_putreg32((uint32_t)&priv->pipe_descriptors, SAM_USB_DESCADD);

  regval = USBHOST_CTRLB_SOFE |
           USBHOST_CTRLB_SPDCONF_LF |
           USBHOST_CTRLB_VBUSOK;
  sam_putreg16(regval, SAM_USBHOST_CTRLB);

  /* Disable all interrupts */

  sam_putreg16(USBHOST_INT_HSOF | USBHOST_INT_RST | USBHOST_INT_WAKEUP |
               USBHOST_INT_DNRSM | USBHOST_INT_UPRSM | USBHOST_INT_RAMACER |
               USBHOST_INT_DCONN | USBHOST_INT_DDISC,
               SAM_USBHOST_INTENCLR);

  /* Initialize host mode and return success */

  sam_host_initialize(priv);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_usbhost_initialize
 *
 * Description:
 *   Initialize USB host device controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *     this identifies which controller is being initialized.  Normally, this
 *     is just zero.
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

struct usbhost_connection_s *sam_usbhost_initialize(int controller)
{
  /* At present, there is only support for a single OTG FS host. Hence it is
   * pre-allocated as g_usbhost.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any
   * future support for multiple devices.
   */

  struct sam_usbhost_s *priv = &g_usbhost;

  /* Sanity checks */

  DEBUGASSERT(controller == 0);

  /* Reset the state of the host driver */

  sam_sw_initialize(priv);

  /* Initialize the USB core */

  sam_hw_initialize(priv);

  priv->suspend_start   = 0;
  priv->resume_start    = 0;
  priv->pipes_unfreeze  = 0;
  priv->n_ctrl_req_user = 0;
  priv->n_sof_user      = 0;

  /* Attach USB host controller interrupt handler */

  if (irq_attach(SAM_IRQ_USB, sam_usbhost_interrupt, priv) != 0)
    usbhost_vtrace1(TRACE_DEVERROR(SAM_TRACEERR_IRQREGISTRATION),
                                  (uint16_t)SAM_IRQ_USB);

  if (irq_attach(SAM_IRQ_USBSOF, sam_usbhost_interrupt, priv) != 0)
    usbhost_vtrace1(TRACE_DEVERROR(SAM_TRACEERR_IRQREGISTRATION),
                                  (uint16_t)SAM_IRQ_USBSOF);

  if (irq_attach(SAM_IRQ_USBTRCPT0, sam_usbhost_interrupt, priv) != 0)
    usbhost_vtrace1(TRACE_DEVERROR(SAM_TRACEERR_IRQREGISTRATION),
                                  (uint16_t)SAM_IRQ_USBTRCPT0);

  if (irq_attach(SAM_IRQ_USBTRCPT1, sam_usbhost_interrupt, priv) != 0)
    usbhost_vtrace1(TRACE_DEVERROR(SAM_TRACEERR_IRQREGISTRATION),
                                  (uint16_t)SAM_IRQ_USBTRCPT1);

  /* Enable interrupts at the interrupt controller */

  up_enable_irq(SAM_IRQ_USB);
  up_enable_irq(SAM_IRQ_USBSOF);
  up_enable_irq(SAM_IRQ_USBTRCPT0);
  up_enable_irq(SAM_IRQ_USBTRCPT1);

  return &g_usbconn;
}
#endif /* CONFIG_USBHOST */

#endif /* CONFIG_SAMD5E5_USB */

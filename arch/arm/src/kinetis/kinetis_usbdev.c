/****************************************************************************
 * arch/arm/src/kinetis/kinetis_usbdev.c
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
 * References:
 *   This file derives from the STM32 USB device driver with modifications
 *   based on additional information from:
 *
 *   - "USB On-The-Go (OTG)", DS61126E, Microchip Technology Inc., 2009
 *   - Sample code provided with the Sure Electronics PIC32 board
 *     (which seems to have derived from Microchip PICDEM PIC18 code).
 *
 *   K66 Sub-Family Reference Manual, Rev. 2, May 2015
 *   How to Implement USB Suspend/Resume - Document Number: AN5385
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "kinetis.h"
#include "kinetis_usbotg.h"
#include "hardware/kinetis_sim.h"
#include "hardware/kinetis_fmc.h"

#if defined(CONFIG_USBDEV)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#  define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

#ifndef CONFIG_USBDEV_SETUP_MAXDATASIZE
#  define CONFIG_USBDEV_SETUP_MAXDATASIZE (CONFIG_USBDEV_EP0_MAXSIZE * 4)
#endif

/* Extremely detailed register/BDT debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG_USB_INFO
#  undef CONFIG_KHCI_USBDEV_REGDEBUG
#  undef CONFIG_KHCI_USBDEV_BDTDEBUG
#endif

/* Disable this logic because it is buggy.  It works most of the time but
 * has some lurking issues that keep this higher performance solution from
 * being usable.
 */

#undef CONFIG_USBDEV_NOREADAHEAD      /* Makes no difference */

#undef CONFIG_USBDEV_NOWRITEAHEAD
#define CONFIG_USBDEV_NOWRITEAHEAD  /* Fixes some problems with IN transfers */

/* Interrupts ***************************************************************/

/* Initial interrupt sets */

#ifdef CONFIG_USB_SOFINTS
#  define USB_SOF_INTERRUPT USB_INT_SOFTOK
#else
#  define USB_SOF_INTERRUPT 0
#endif

#define ERROR_INTERRUPTS  (USB_ERRSTAT_PIDERR | USB_ERRSTAT_CRC5EOF |  \
                           USB_ERRSTAT_CRC16 | USB_ERRSTAT_DFN8 | USB_ERRSTAT_BTOERR | \
                           USB_ERRSTAT_BTSERR)

#define NORMAL_INTERRUPTS (USB_INT_USBRST | USB_INT_ERROR | USB_SOF_INTERRUPT | \
                           USB_INT_TOKDNE | USB_INT_SLEEP | USB_INT_STALL)

/* Endpoints ****************************************************************/

#define USB_STAT_ENDPT(n)     ((n) << USB_STAT_ENDP_SHIFT) /* Endpoint n, n=0..15 */

#define USB_STAT_ODD_ODD      USB_STAT_ODD /* The last transaction was to the ODD BD bank */
#define USB_STAT_ODD_EVEN     0            /* The last transaction was to the EVEN BD bank */

#define USB_STAT_TX_IN        USB_STAT_TX  /* Last transaction was a transmit transfer (TX) */
#define USB_STAT_TX_OUT       0            /* Last transaction was a receive transfer (RX) */

#define KHCI_NENDPOINTS       (16)
#define EP0                   (0)

#define KHCI_ENDP_BIT(ep)     (1 << (ep))
#define KHCI_ENDP_ALLSET      0xffff

/* BDT Table Indexing.  The BDT is addressed in the hardware as follows:
 *
 *   Bits 9-31:  These come the BDT address bits written into the BDTP3,
 *               BDTP2, and BDTP1 registers
 *   Bits 5-8:   The endpoint number
 *   Bit 4:      Direction:
 *               1 = Transmit: SETUP/OUT for host, IN for function
 *               0 = Receive: IN for host, SETUP/OUT for function
 *   Bit 3:      PPBI, the ping point buffer index bit (0=EVEN, 1=ODD)
 *   Bits 0-2:   Supports 8-byte BDT entries
 */

#define EP0_OUT_EVEN          (0)
#define EP0_OUT_ODD           (1)
#define EP0_IN_EVEN           (2)
#define EP0_IN_ODD            (3)
#define EP_OUT_EVEN(ep)       ((int)(ep) << 2)
#define EP_OUT_ODD(ep)        (((int)(ep) << 2) + 1)
#define EP_IN_EVEN(ep)        (((int)(ep) << 2) + 2)
#define EP_IN_ODD(ep)         (((int)(ep) << 2) + 3)

#define EP(ep,dir,pp)         (((int)(ep) << 2) + ((int)(dir) << 1) + (int)(pp))
#define EP_DIR_OUT            0
#define EP_DIR_IN             1
#define EP_PP_EVEN            0
#define EP_PP_ODD             1

/* Packet sizes.  We use a fixed 64 max packet size for all endpoint types */

#define KHCI_MAXPACKET_SHIFT  (6)
#define KHCI_MAXPACKET_SIZE   (1 << (KHCI_MAXPACKET_SHIFT))

#define KHCI_EP0MAXPACKET     KHCI_MAXPACKET_SIZE

/* Endpoint register initialization parameters */

#define KHCI_EP_DISABLED  (0)
#define KHCI_EP_CONTROL   (USB_ENDPT_EPHSHK | USB_ENDPT_EPTXEN | USB_ENDPT_EPRXEN)
#define KHCI_EP_BULKIN    (USB_ENDPT_EPTXEN | USB_ENDPT_EPCTLDIS | USB_ENDPT_EPHSHK)
#define KHCI_EP_BULKOUT   (USB_ENDPT_EPRXEN | USB_ENDPT_EPCTLDIS | USB_ENDPT_EPHSHK)
#define KHCI_EP_INTIN     (USB_ENDPT_EPTXEN | USB_ENDPT_EPCTLDIS | USB_ENDPT_EPHSHK)
#define KHCI_EP_INTOUT    (USB_ENDPT_EPRXEN | USB_ENDPT_EPCTLDIS | USB_ENDPT_EPHSHK)
#define KHCI_EP_ISOCIN    (USB_ENDPT_EPTXEN | USB_ENDPT_EPCTLDIS)
#define KHCI_EP_ISOCOUT   (USB_ENDPT_EPRXEN | USB_ENDPT_EPCTLDIS)

/* USB-related masks */

#define REQRECIPIENT_MASK (USB_REQ_TYPE_MASK | USB_REQ_RECIPIENT_MASK)

/* Request queue operations *************************************************/

#define khci_rqempty(q)   ((q)->head == NULL)
#define khci_rqhead(q)    ((q)->head)
#define khci_rqtail(q)    ((q)->tail)

#define RESTART_DELAY     (150 * CLOCKS_PER_SEC / 1000)

#define USB0_USBTRC0_BIT6 0x40 /* Undocumented bit that is set in the
                                * Kinetis lib
                                */

/* USB trace ****************************************************************/

/* Trace error codes */

#define KHCI_TRACEERR_ALLOCFAIL            0x0001
#define KHCI_TRACEERR_BADCLEARFEATURE      0x0002
#define KHCI_TRACEERR_BADDEVGETSTATUS      0x0003
#define KHCI_TRACEERR_BADEPGETSTATUS       0x0004
#define KHCI_TRACEERR_BADEPNO              0x0005
#define KHCI_TRACEERR_BADEPTYPE            0x0006
#define KHCI_TRACEERR_BADGETCONFIG         0x0007
#define KHCI_TRACEERR_BADGETSETDESC        0x0008
#define KHCI_TRACEERR_BADGETSTATUS         0x0009
#define KHCI_TRACEERR_BADSETADDRESS        0x000a
#define KHCI_TRACEERR_BADSETCONFIG         0x000b
#define KHCI_TRACEERR_BADSETFEATURE        0x000c
#define KHCI_TRACEERR_BINDFAILED           0x000d
#define KHCI_TRACEERR_DISPATCHSTALL        0x000e
#define KHCI_TRACEERR_DRIVER               0x000f
#define KHCI_TRACEERR_DRIVERREGISTERED     0x0010
#define KHCI_TRACEERR_EP0SETUPSTALLED      0x0011
#define KHCI_TRACEERR_EPDISABLED           0x0012
#define KHCI_TRACEERR_EPOUTNULLPACKET      0x0013
#define KHCI_TRACEERR_EPRESERVE            0x0014
#define KHCI_TRACEERR_INVALIDCTRLREQ       0x0015
#define KHCI_TRACEERR_INVALIDPARMS         0x0016
#define KHCI_TRACEERR_IRQREGISTRATION      0x0017
#define KHCI_TRACEERR_NOTCONFIGURED        0x0018
#define KHCI_TRACEERR_REQABORTED           0x0019
#define KHCI_TRACEERR_INVALIDSTATE         0x001a

/* Trace interrupt codes */

#define KHCI_TRACEINTID_CLEARFEATURE       0x0001
#define KHCI_TRACEINTID_DEVGETSTATUS       0x0002
#define KHCI_TRACEINTID_DISPATCH           0x0003
#define KHCI_TRACEINTID_EP0IN              0x0004
#define KHCI_TRACEINTID_EP0INDONE          0x0005
#define KHCI_TRACEINTID_EP0OUTDONE         0x0006
#define KHCI_TRACEINTID_EP0SETUPDONE       0x0007
#define KHCI_TRACEINTID_EP0SETUPSETADDRESS 0x0008
#define KHCI_TRACEINTID_EP0ADDRESSSET      0x0009
#define KHCI_TRACEINTID_EPGETSTATUS        0x000a
#define KHCI_TRACEINTID_EPINDONE           0x000b
#define KHCI_TRACEINTID_EPINQEMPTY         0x000c
#define KHCI_TRACEINTID_EPOUTDONE          0x000d
#define KHCI_TRACEINTID_EPOUTQEMPTY        0x000e
#define KHCI_TRACEINTID_SOF                0x000f
#define KHCI_TRACEINTID_GETCONFIG          0x0010
#define KHCI_TRACEINTID_GETSETDESC         0x0011
#define KHCI_TRACEINTID_GETSETIF           0x0012
#define KHCI_TRACEINTID_GETSTATUS          0x0013
#define KHCI_TRACEINTID_IFGETSTATUS        0x0014
#define KHCI_TRACEINTID_TRNC               0x0015
#define KHCI_TRACEINTID_TRNCS              0x0016
#define KHCI_TRACEINTID_INTERRUPT          0x0017
#define KHCI_TRACEINTID_NOSTDREQ           0x0018
#define KHCI_TRACEINTID_RESET              0x0019
#define KHCI_TRACEINTID_SETCONFIG          0x001a
#define KHCI_TRACEINTID_SETFEATURE         0x001b
#define KHCI_TRACEINTID_IDLE               0x001c
#define KHCI_TRACEINTID_SYNCHFRAME         0x001d
#define KHCI_TRACEINTID_WKUP               0x001e
#define KHCI_TRACEINTID_T1MSEC             0x001f
#define KHCI_TRACEINTID_OTGID              0x0020
#define KHCI_TRACEINTID_STALL              0x0021
#define KHCI_TRACEINTID_UERR               0x0022
#define KHCI_TRACEINTID_SUSPENDED          0x0023
#define KHCI_TRACEINTID_RESUME             0x0024
#define KHCI_TRACEINTID_WAITRESET          0x0025
#define KHCI_TRACEINTID_EP0SETUPOUT        0x0026
#define KHCI_TRACEINTID_EP0SETUPOUTDATA    0x0027

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_intdecode[] =
{
  TRACE_STR(KHCI_TRACEINTID_CLEARFEATURE),        /* 0x0001 */
  TRACE_STR(KHCI_TRACEINTID_DEVGETSTATUS),        /* 0x0002 */
  TRACE_STR(KHCI_TRACEINTID_DISPATCH),            /* 0x0003 */
  TRACE_STR(KHCI_TRACEINTID_EP0IN),               /* 0x0004 */
  TRACE_STR(KHCI_TRACEINTID_EP0INDONE),           /* 0x0005 */
  TRACE_STR(KHCI_TRACEINTID_EP0OUTDONE),          /* 0x0006 */
  TRACE_STR(KHCI_TRACEINTID_EP0SETUPDONE),        /* 0x0007 */
  TRACE_STR(KHCI_TRACEINTID_EP0SETUPSETADDRESS),  /* 0x0008 */
  TRACE_STR(KHCI_TRACEINTID_EP0ADDRESSSET),       /* 0x0009 */
  TRACE_STR(KHCI_TRACEINTID_EPGETSTATUS),         /* 0x000a */
  TRACE_STR(KHCI_TRACEINTID_EPINDONE),            /* 0x000b */
  TRACE_STR(KHCI_TRACEINTID_EPINQEMPTY),          /* 0x000c */
  TRACE_STR(KHCI_TRACEINTID_EPOUTDONE),           /* 0x000d */
  TRACE_STR(KHCI_TRACEINTID_EPOUTQEMPTY),         /* 0x000e */
  TRACE_STR(KHCI_TRACEINTID_SOF),                 /* 0x000f */
  TRACE_STR(KHCI_TRACEINTID_GETCONFIG),           /* 0x0010 */
  TRACE_STR(KHCI_TRACEINTID_GETSETDESC),          /* 0x0011 */
  TRACE_STR(KHCI_TRACEINTID_GETSETIF),            /* 0x0012 */
  TRACE_STR(KHCI_TRACEINTID_GETSTATUS),           /* 0x0013 */
  TRACE_STR(KHCI_TRACEINTID_IFGETSTATUS),         /* 0x0014 */
  TRACE_STR(KHCI_TRACEINTID_TRNC),                /* 0x0015 */
  TRACE_STR(KHCI_TRACEINTID_TRNCS),               /* 0x0016 */
  TRACE_STR(KHCI_TRACEINTID_INTERRUPT),           /* 0x0017 */
  TRACE_STR(KHCI_TRACEINTID_NOSTDREQ),            /* 0x0018 */
  TRACE_STR(KHCI_TRACEINTID_RESET),               /* 0x0019 */
  TRACE_STR(KHCI_TRACEINTID_SETCONFIG),           /* 0x001a */
  TRACE_STR(KHCI_TRACEINTID_SETFEATURE),          /* 0x001b */
  TRACE_STR(KHCI_TRACEINTID_IDLE),                /* 0x001c */
  TRACE_STR(KHCI_TRACEINTID_SYNCHFRAME),          /* 0x001d */
  TRACE_STR(KHCI_TRACEINTID_WKUP),                /* 0x001e */
  TRACE_STR(KHCI_TRACEINTID_T1MSEC),              /* 0x001f */
  TRACE_STR(KHCI_TRACEINTID_OTGID),               /* 0x0020 */
  TRACE_STR(KHCI_TRACEINTID_STALL),               /* 0x0021 */
  TRACE_STR(KHCI_TRACEINTID_UERR),                /* 0x0022 */
  TRACE_STR(KHCI_TRACEINTID_SUSPENDED),           /* 0x0023 */
  TRACE_STR(KHCI_TRACEINTID_RESUME),              /* 0x0024 */
  TRACE_STR(KHCI_TRACEINTID_WAITRESET),           /* 0x0025 */
  TRACE_STR(KHCI_TRACEINTID_EP0SETUPOUT),         /* 0x0026 */
  TRACE_STR(KHCI_TRACEINTID_EP0SETUPOUTDATA),     /* 0x0027 */
  TRACE_STR_END
};
#endif

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_deverror[] =
{
  TRACE_STR(KHCI_TRACEERR_ALLOCFAIL),             /* 0x0001 */
  TRACE_STR(KHCI_TRACEERR_BADCLEARFEATURE),       /* 0x0002 */
  TRACE_STR(KHCI_TRACEERR_BADDEVGETSTATUS),       /* 0x0003 */
  TRACE_STR(KHCI_TRACEERR_BADEPGETSTATUS),        /* 0x0004 */
  TRACE_STR(KHCI_TRACEERR_BADEPNO),               /* 0x0005 */
  TRACE_STR(KHCI_TRACEERR_BADEPTYPE),             /* 0x0006 */
  TRACE_STR(KHCI_TRACEERR_BADGETCONFIG),          /* 0x0007 */
  TRACE_STR(KHCI_TRACEERR_BADGETSETDESC),         /* 0x0008 */
  TRACE_STR(KHCI_TRACEERR_BADGETSTATUS),          /* 0x0009 */
  TRACE_STR(KHCI_TRACEERR_BADSETADDRESS),         /* 0x000a */
  TRACE_STR(KHCI_TRACEERR_BADSETCONFIG),          /* 0x000b */
  TRACE_STR(KHCI_TRACEERR_BADSETFEATURE),         /* 0x000c */
  TRACE_STR(KHCI_TRACEERR_BINDFAILED),            /* 0x000d */
  TRACE_STR(KHCI_TRACEERR_DISPATCHSTALL),         /* 0x000e */
  TRACE_STR(KHCI_TRACEERR_DRIVER),                /* 0x000f */
  TRACE_STR(KHCI_TRACEERR_DRIVERREGISTERED),      /* 0x0010 */
  TRACE_STR(KHCI_TRACEERR_EP0SETUPSTALLED),       /* 0x0011 */
  TRACE_STR(KHCI_TRACEERR_EPDISABLED),            /* 0x0012 */
  TRACE_STR(KHCI_TRACEERR_EPOUTNULLPACKET),       /* 0x0013 */
  TRACE_STR(KHCI_TRACEERR_EPRESERVE),             /* 0x0014 */
  TRACE_STR(KHCI_TRACEERR_INVALIDCTRLREQ),        /* 0x0015 */
  TRACE_STR(KHCI_TRACEERR_INVALIDPARMS),          /* 0x0016 */
  TRACE_STR(KHCI_TRACEERR_IRQREGISTRATION),       /* 0x0017 */
  TRACE_STR(KHCI_TRACEERR_NOTCONFIGURED),         /* 0x0018 */
  TRACE_STR(KHCI_TRACEERR_REQABORTED),            /* 0x0019 */
  TRACE_STR(KHCI_TRACEERR_INVALIDSTATE),          /* 0x001a */
  TRACE_STR_END
};
#endif

/* Misc Helper Macros *******************************************************/

/* Byte ordering in host-based values */

#ifdef CONFIG_ENDIAN_BIG
#  define LSB 1
#  define MSB 0
#else
#  define LSB 0
#  define MSB 1
#endif

/* Debug ********************************************************************/

/* CONFIG_KHCI_USBDEV_REGDEBUG enables dumping of all low-level register
 * access and BDT accesses.  Normally, this generates so much debug output
 * that USB may not even be functional.
 */

#ifdef CONFIG_KHCI_USBDEV_REGDEBUG
#  undef CONFIG_KHCI_USBDEV_BDTDEBUG
#  define CONFIG_KHCI_USBDEV_BDTDEBUG 1
#else
#  define khci_getreg(addr)      getreg8(addr)
#  define khci_putreg(val,addr)  putreg8(val,addr)
#endif

/* CONFIG_KHCI_USBDEV_BDTDEBUG dumps most BDT settings */

#ifdef CONFIG_KHCI_USBDEV_BDTDEBUG
#  define bdterr  uerr
#  define bdtinfo uinfo
#else
#  define bdterr(x...)
#  define bdtinfo(x...)
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Overvall device state */

enum khci_devstate_e
{
  DEVSTATE_DETACHED = 0,  /* Not connected to a host */
  DEVSTATE_ATTACHED,      /* Connected to a host */
  DEVSTATE_POWERED,       /* Powered */
  DEVSTATE_DEFAULT,       /* Default state */
  DEVSTATE_ADDRPENDING,   /* Waiting for an address */
  DEVSTATE_ADDRESS,       /* Address received */
  DEVSTATE_CONFIGURED,    /* Configuration received */
};

/* The various states of the control pipe */

enum khci_ctrlstate_e
{
  CTRLSTATE_WAITSETUP = 0,  /* No request in progress, waiting for setup */
  CTRLSTATE_SETUP_OUT,      /* Set up received with data for device OUT in progress */
  CTRLSTATE_SETUP_READY,    /* Set up was received prior and is in ctrl,
                             * now the data has arrived */
  CTRLSTATE_RDREQUEST,      /* Read request (OUT) in progress */
  CTRLSTATE_WRREQUEST,      /* Write request (IN) in progress */
  CTRLSTATE_STALL,          /* EP0 stall requested */
  CTRLSTATE_STALLED         /* EP0 is stalled */
};

union wb_u
{
  uint16_t w;
  uint8_t  b[2];
};

/* A container for a request so that the request make be retained in a
 * singly-linked list.
 */

struct khci_req_s
{
  struct usbdev_req_s req;             /* Standard USB request */
#ifdef CONFIG_USBDEV_NOWRITEAHEAD
  uint16_t inflight[1];                /* The number of bytes "in-flight" */
#else
  uint16_t inflight[2];                /* The number of bytes "in-flight" */
#endif
  struct khci_req_s *flink;            /* Supports a singly linked list */
};

/* This structure represents the 'head' of a singly linked list of requests */

struct khci_queue_s
{
  struct khci_req_s *head;             /* Head of the request queue */
  struct khci_req_s *tail;             /* Tail of the request queue */
};

/* This is the internal representation of an endpoint */

struct khci_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct khci_ep_s.
   */

  struct usbdev_ep_s ep;               /* Standard endpoint structure */

  /* KHCI-specific fields */

  struct khci_usbdev_s *dev;           /* Reference to private driver data */
  struct khci_queue_s pend;            /* List of pending (inactive) requests for this endpoint */
  struct khci_queue_s active;          /* List of active requests for this endpoint */
  uint8_t stalled:1;                   /* true: Endpoint is stalled */
  uint8_t halted:1;                    /* true: Endpoint feature halted */
  uint8_t txnullpkt:1;                 /* Null packet needed at end of TX transfer */
  uint8_t txdata1:1;                   /* Data0/1 of next TX transfer */
  uint8_t rxdata1:1;                   /* Data0/1 of next RX transfer */

  volatile struct usbotg_bdtentry_s *bdtin;  /* BDT entry for the IN transaction */
  volatile struct usbotg_bdtentry_s *bdtout; /* BDT entry for the OUT transaction */
};

struct khci_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to structkhci_usbdev_s.
   */

  struct usbdev_s usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* KHCI-specific fields */

  struct usb_ctrlreq_s  ctrl;          /* Last EP0 request */
  uint8_t devstate;                    /* Driver state (see enum khci_devstate_e) */
  uint8_t ctrlstate;                   /* Control EP state (see enum khci_ctrlstate_e) */
  uint8_t selfpowered:1;               /* 1: Device is self powered */
  uint8_t rwakeup:1;                   /* 1: Device supports remote wakeup */
  uint8_t ep0done:1;                   /* EP0 OUT already prepared */
  uint8_t rxbusy:1;                    /* EP0 OUT data transfer in progress */
  uint16_t epavail;                    /* Bitset of available endpoints */
  uint16_t epstalled;                  /* Bitset of stalled endpoints */
  struct wdog_s wdog;                  /* Supports the restart delay */

  uint8_t out0data[2][CONFIG_USBDEV_EP0_MAXSIZE];
  uint8_t ep0data[CONFIG_USBDEV_SETUP_MAXDATASIZE];
  uint16_t ep0datlen;
  uint16_t ep0datreq;

  /* The endpoint list */

  struct khci_ep_s eplist[KHCI_NENDPOINTS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_KHCI_USBDEV_REGDEBUG
static uint16_t khci_getreg(uint32_t addr);
static void khci_putreg(uint32_t val, uint32_t addr);
#endif

/* Suspend/Resume Helpers ***************************************************/

#ifndef CONFIG_KINETIS_USBOTG
static void   khci_suspend(struct khci_usbdev_s *priv);
#endif
static void   khci_remote_resume(struct khci_usbdev_s *priv);
static void   khci_resume(struct khci_usbdev_s *priv);

/* Request Queue Management *************************************************/

static struct khci_req_s *khci_remfirst(struct khci_queue_s *queue);
static struct khci_req_s *khci_remlast(struct khci_queue_s *queue);
static void   khci_addlast(struct khci_queue_s *queue,
                struct khci_req_s *req);
static void   khci_addfirst(struct khci_queue_s *queue,
                struct khci_req_s *req);

/* Request Helpers **********************************************************/

static void   khci_reqreturn(struct khci_ep_s *privep,
                struct khci_req_s *privreq, int16_t result);
static void   khci_reqcomplete(struct khci_ep_s *privep,
                int16_t result);
static void   khci_epwrite(struct khci_ep_s *privep,
                volatile struct usbotg_bdtentry_s *bdt,
                const uint8_t *src, uint32_t nbytes);
static void   khci_wrcomplete(struct khci_usbdev_s *priv,
                struct khci_ep_s *privep);
static void   khci_rqrestart(wdparm_t arg);
static void   khci_delayedrestart(struct khci_usbdev_s *priv,
                uint8_t epno);
static void   khci_rqstop(struct khci_ep_s *privep);
static int    khci_wrstart(struct khci_usbdev_s *priv,
                struct khci_ep_s *privep);
static int    khci_wrrequest(struct khci_usbdev_s *priv,
                struct khci_ep_s *privep);
static int    khci_rdcomplete(struct khci_usbdev_s *priv,
                struct khci_ep_s *privep);
static int    khci_ep0rdsetup(struct khci_usbdev_s *priv,
                uint8_t *dest, int readlen);
static int    khci_rdsetup(struct khci_ep_s *privep, uint8_t *dest,
                int readlen);
static int    khci_rdrequest(struct khci_usbdev_s *priv,
                struct khci_ep_s *privep);
static void   khci_cancelrequests(struct khci_ep_s *privep,
                int16_t result);

/* Interrupt level processing ***********************************************/

static void   khci_dispatchrequest(struct khci_usbdev_s *priv);
static void   khci_ep0stall(struct khci_usbdev_s *priv);
static void   khci_eptransfer(struct khci_usbdev_s *priv, uint8_t epno,
                uint16_t ustat);
static void   khci_ep0nextsetup(struct khci_usbdev_s *priv);
static void   khci_ep0rdcomplete(struct khci_usbdev_s *priv);
static void   khci_ep0setup(struct khci_usbdev_s *priv);
static void   khci_ep0outcomplete(struct khci_usbdev_s *priv);
static void   khci_ep0incomplete(struct khci_usbdev_s *priv);
static void   khci_ep0transfer(struct khci_usbdev_s *priv,
                uint16_t ustat);
static int    khci_interrupt(int irq, void *context, void *arg);

/* Endpoint helpers *********************************************************/

static inline struct khci_ep_s *
              khci_epreserve(struct khci_usbdev_s *priv, uint8_t epset);
static inline void
              khci_epunreserve(struct khci_usbdev_s *priv,
              struct khci_ep_s *privep);
static void  khci_ep0configure(struct khci_usbdev_s *priv);

/* Endpoint operations ******************************************************/

static int    khci_epconfigure(struct usbdev_ep_s *ep,
                const struct usb_epdesc_s *desc, bool last);
static int    khci_epdisable(struct usbdev_ep_s *ep);
static struct usbdev_req_s *
              khci_epallocreq(struct usbdev_ep_s *ep);
static void   khci_epfreereq(struct usbdev_ep_s *ep,
                struct usbdev_req_s *);
static int    khci_epsubmit(struct usbdev_ep_s *ep,
                struct usbdev_req_s *req);
static int    khci_epcancel(struct usbdev_ep_s *ep,
                struct usbdev_req_s *req);
static int    khci_epbdtstall(struct usbdev_ep_s *ep, bool resume,
                bool epin);
static int    khci_epstall(struct usbdev_ep_s *ep, bool resume);

/* USB device controller operations *****************************************/

static struct usbdev_ep_s *
              khci_allocep(struct usbdev_s *dev, uint8_t epno, bool in,
                uint8_t eptype);
static void   khci_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep);
static int    khci_getframe(struct usbdev_s *dev);
static int    khci_wakeup(struct usbdev_s *dev);
static int    khci_selfpowered(struct usbdev_s *dev, bool selfpowered);

/* Initialization/Reset *****************************************************/

static void   khci_reset(struct khci_usbdev_s *priv);
static void   khci_attach(struct khci_usbdev_s *priv);
static void   khci_swreset(struct khci_usbdev_s *priv);
static void   khci_hwreset(struct khci_usbdev_s *priv);
static void   khci_swinitialize(struct khci_usbdev_s *priv);
static void   khci_hwinitialize(struct khci_usbdev_s *priv);
static void   khci_hwshutdown(struct khci_usbdev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static struct khci_usbdev_s g_usbdev;

static const struct usbdev_epops_s g_epops =
{
  .configure   = khci_epconfigure,
  .disable     = khci_epdisable,
  .allocreq    = khci_epallocreq,
  .freereq     = khci_epfreereq,
  .submit      = khci_epsubmit,
  .cancel      = khci_epcancel,
  .stall       = khci_epstall,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep     = khci_allocep,
  .freeep      = khci_freeep,
  .getframe    = khci_getframe,
  .wakeup      = khci_wakeup,
  .selfpowered = khci_selfpowered,
  .pullup      = kinetis_usbpullup,
};

/* Buffer Descriptor Table.  Four BDT entries per endpoint
 *
 * The BDT is addressed in the hardware as follows:
 *
 *   Bits 9-31:  These come the BDT address bits written into the BDTP3,
 *      BDTP2 and BDTP1 registers
 *   Bits 5-8:   The endpoint number
 *   Bit 4:      Direction (0=IN/Tx, 1 = OUT/Rx)
 *   Bit 3:      PPBI, the ping point buffer index bit.
 *   Bits 0-2:   Supports 8-byte BDT entries
 */

static volatile struct usbotg_bdtentry_s g_bdt[4*KHCI_NENDPOINTS]
  aligned_data(512);

/****************************************************************************
 * Private Private Functions
 ****************************************************************************/

/****************************************************************************
 * Register Operations
 ****************************************************************************/

/****************************************************************************
 * Name: khci_getreg
 ****************************************************************************/

#ifdef CONFIG_KHCI_USBDEV_REGDEBUG
static uint16_t khci_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;

  /* Read the value from the register */

  uint16_t val = getreg8(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              uinfo("...\n");
            }
          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          uinfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  uinfo("%08x->%04x\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: khci_putreg
 ****************************************************************************/

#ifdef CONFIG_KHCI_USBDEV_REGDEBUG
static void khci_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  uinfo("%08x<-%04x\n", addr, val);

  /* Write the value */

  putreg8(val, addr);
}
#endif

/****************************************************************************
 * Request Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: khci_remfirst
 ****************************************************************************/

static struct khci_req_s *khci_remfirst(struct khci_queue_s *queue)
{
  struct khci_req_s *ret = queue->head;

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
 * Name: khci_remlast
 ****************************************************************************/

static struct khci_req_s *khci_remlast(struct khci_queue_s *queue)
{
  struct khci_req_s *prev;
  struct khci_req_s *ret = queue->tail;

  ret = queue->tail;
  if (ret)
    {
      if (queue->head == queue->tail)
        {
          queue->head = NULL;
          queue->tail = NULL;
        }
      else
        {
          for (prev = queue->head;
               prev && prev->flink != ret;
               prev = prev->flink);

          if (prev)
            {
              prev->flink = NULL;
              queue->tail = prev;
            }
        }

      ret->flink = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: khci_addlast
 ****************************************************************************/

static void khci_addlast(struct khci_queue_s *queue, struct khci_req_s *req)
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
 * Name: khci_addfirst
 ****************************************************************************/

static void khci_addfirst(struct khci_queue_s *queue, struct khci_req_s *req)
{
  req->flink = queue->head;
  if (!queue->head)
    {
      queue->tail = req;
    }

  queue->head = req;
}

/****************************************************************************
 * Name: khci_reqreturn
 ****************************************************************************/

static void khci_reqreturn(struct khci_ep_s *privep,
                              struct khci_req_s *privreq, int16_t result)
{
  /* If endpoint 0, temporarily reflect the state of protocol stalled
   * in the callback.
   */

  bool stalled = privep->stalled;
  if (USB_EPNO(privep->ep.eplog) == EP0)
    {
      privep->stalled = (privep->dev->ctrlstate == CTRLSTATE_STALLED);
    }

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->flink = NULL;
  privreq->req.callback(&privep->ep, &privreq->req);

  /* Restore the stalled indication */

  privep->stalled = stalled;
}

/****************************************************************************
 * Name: khci_reqcomplete
 ****************************************************************************/

static void khci_reqcomplete(struct khci_ep_s *privep, int16_t result)
{
  struct khci_req_s *privreq;
  irqstate_t flags;

  /* Remove the completed request at the head of the endpoint's active
   * request list.
   */

  flags = enter_critical_section();
  privreq = khci_remfirst(&privep->active);
  leave_critical_section(flags);

  if (privreq)
    {
      /* Return the request to the class driver */

      khci_reqreturn(privep, privreq, result);
    }
}

/****************************************************************************
 * Name: khci_epwrite
 ****************************************************************************/

static void khci_epwrite(struct khci_ep_s *privep,
                            volatile struct usbotg_bdtentry_s *bdt,
                            const uint8_t *src, uint32_t nbytes)
{
  uint32_t status;

  usbtrace(TRACE_WRITE(USB_EPNO(privep->ep.eplog)), nbytes);

  /* Clear all bits in the status (assuring that we own the BDT) */

  bdt->status = 0;

  /* Get the correct data toggle (as well as other BDT bits) */

  if (privep->txdata1)
    {
      status          = (USB_BDT_UOWN | USB_BDT_DATA1 | USB_BDT_DTS);
      privep->txdata1 = 0;
    }
  else
    {
      status          = (USB_BDT_UOWN | USB_BDT_DATA0 | USB_BDT_DTS);
      privep->txdata1 = 1;
    }

  /* Set the data pointer and data length */

  bdt->addr = (uint8_t *)src;
  status   |= (nbytes << USB_BDT_BYTECOUNT_SHIFT) | USB_BDT_DTS;

  /* And, finally, give the BDT to the USB */

  bdtinfo("EP%d BDT IN [%p] {%08x, %08x}\n",
          USB_EPNO(privep->ep.eplog), bdt, status, bdt->addr);

  bdt->status = status;
}

/****************************************************************************
 * Name: khci_wrcomplete
 ****************************************************************************/

static void khci_wrcomplete(struct khci_usbdev_s *priv,
                               struct khci_ep_s *privep)
{
  volatile struct usbotg_bdtentry_s *bdtin;
  struct khci_req_s *privreq;
  int bytesleft;
  int epno;

  /* Check the request at the head of the endpoint's active request queue.
   * Since we got here from a write completion event, the active request
   * queue should not be empty.
   */

  privreq = khci_rqhead(&privep->active);
  DEBUGASSERT(privreq != NULL);

  /* An outgoing IN packet has completed.  bdtin should point to the BDT
   * that just completed.
   */

  bdtin = privep->bdtin;
  epno   = USB_EPNO(privep->ep.eplog);

#ifdef CONFIG_USBDEV_NOWRITEAHEAD
  uinfo("EP%d: len=%d xfrd=%d inflight=%d\n",
        epno, privreq->req.len, privreq->req.xfrd, privreq->inflight[0]);
#else
  uinfo("EP%d: len=%d xfrd=%d inflight={%d, %d}\n",
        epno, privreq->req.len, privreq->req.xfrd,
        privreq->inflight[0], privreq->inflight[1]);
#endif
  bdtinfo("EP%d BDT IN [%p] {%08x, %08x}\n",
        epno, bdtin, bdtin->status, bdtin->addr);

  /* We should own the BDT that just completed. But NULLify the entire BDT
   * IN. Why?  So that we can tell later that the BDT available.  No, it is
   * not sufficient to look at the UOWN bit.  If UOWN==0, then the transfer
   * has been completed BUT it may not yet have been processed.
   * But a completely NULLified BDT is a sure indication
   */

  DEBUGASSERT((bdtin->status & USB_BDT_UOWN) == USB_BDT_COWN);
  bdtin->status = 0;
  bdtin->addr   = 0;

  /* Toggle bdtin to the other BDT.  Is the current bdtin the EVEN bdt? */

  privep->bdtin = &g_bdt[EP_IN_EVEN(epno)];
  if (bdtin == privep->bdtin)
    {
      /* Yes.. Then the other BDT is the ODD BDT */

      privep->bdtin++;
    }

  /* Update the number of bytes transferred. */

  privreq->req.xfrd   += privreq->inflight[0];
#ifdef CONFIG_USBDEV_NOWRITEAHEAD
  privreq->inflight[0] = 0;
#else
  privreq->inflight[0] = privreq->inflight[1];
  privreq->inflight[1] = 0;
#endif
  bytesleft            = privreq->req.len - privreq->req.xfrd;

  /* If all of the bytes were sent (bytesleft == 0) and no NULL packet is
   * needed (!txnullpkt), then we are finished with the transfer
   */

  if (bytesleft == 0 && !privep->txnullpkt)
    {
      /* The transfer is complete.  Give the completed request back to
       * the class driver.
       */

      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)),
               privreq->req.xfrd);
      khci_reqcomplete(privep, OK);

      /* Special case writes to endpoint zero.  If there is no transfer in
       * progress, then we need to configure to received the next SETUP
       * packet.
       */

      if (USB_EPNO(privep->ep.eplog) == 0)
        {
          priv->ctrlstate = CTRLSTATE_WAITSETUP;
        }
    }
}

/****************************************************************************
 * Name: khci_rqrestart
 ****************************************************************************/

static void khci_rqrestart(wdparm_t arg)
{
  struct khci_usbdev_s *priv;
  struct khci_ep_s *privep;
  struct khci_req_s *privreq;
  uint16_t epstalled;
  uint16_t mask;
  int epno;

  /* Recover the pointer to the driver structure */

  priv = (struct khci_usbdev_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Sample and clear the set of endpoints that have recovered from a stall */

  epstalled = priv->epstalled;
  priv->epstalled = 0;

  /* Loop, checking each bit in the epstalled bit set */

  for (epno = 0; epstalled && epno < KHCI_NENDPOINTS; epno++)
    {
      /* Has this encpoint recovered from a stall? */

      mask = (1 << epno);
      if ((epstalled & mask) != 0)
        {
          /* Yes, this endpoint needs to be restarteed */

          epstalled      &= ~mask;
          privep          = &priv->eplist[epno];

          /* Reset some endpoint state variables */

          privep->stalled   = false;
          privep->txnullpkt = false;

          /* Check the request at the head of the endpoint's pending
           * request queue
           */

          privreq = khci_rqhead(&privep->pend);
          if (privreq)
            {
              /* Restart transmission after we have recovered from a stall */

              privreq->req.xfrd    = 0;
              privreq->inflight[0] = 0;
#ifndef CONFIG_USBDEV_NOWRITEAHEAD
              privreq->inflight[1] = 0;
#endif
              khci_wrrequest(priv, privep);
            }
        }
    }
}

/****************************************************************************
 * Name: khci_delayedrestart
 ****************************************************************************/

static void khci_delayedrestart(struct khci_usbdev_s *priv, uint8_t epno)
{
  /* Add endpoint to the set of endpoints that need to be restarted */

  priv->epstalled |= (1 << epno);

  /* And start (or re-start) the watchdog timer */

  wd_start(&priv->wdog, RESTART_DELAY,
           khci_rqrestart, (wdparm_t)priv);
}

/****************************************************************************
 * Name: khci_rqstop
 ****************************************************************************/

static void khci_rqstop(struct khci_ep_s *privep)
{
  struct khci_req_s *privreq;

  /* Move all of the active requests back to the pending request queue */

  while ((privreq = khci_remlast(&privep->active)))
    {
      /* Move the request back to the head of the pending list */

      khci_addfirst(&privep->pend, privreq);
    }
}

/****************************************************************************
 * Name: khci_wrstart
 ****************************************************************************/

static int khci_wrstart(struct khci_usbdev_s *priv,
                           struct khci_ep_s *privep)
{
  volatile struct usbotg_bdtentry_s *bdt;
  struct khci_req_s *privreq;
  uint8_t *buf;
  uint8_t epno;
  int nbytes;
  int bytesleft;
  int xfrd;
  int index;

  /* We get here when either (1) an IN endpoint completion interrupt occurs,
   * or (2) a new write request is reqeived from the class.
   */

  /* Get the endpoint number that we are servicing */

  epno = USB_EPNO(privep->ep.eplog);

  /* Decide which BDT to use.  bdtin points to the "current" BDT.  That is,
   * the one that either (1) available for next transfer, or (2) the one
   * that is currently busy with the current transfer.  If the current
   * BDT is busy, we have the option of setting up the other BDT in advance
   * in order to improve data transfer performance.
   */

  bdt   = privep->bdtin;
  index = 0;

  if (bdt->status || bdt->addr)
    {
#ifdef CONFIG_USBDEV_NOWRITEAHEAD
      /* The current BDT is not available and write ahead is disabled.  There
       * is nothing we can do now.  Return -EBUSY to indicate this condition.
       */

      return -EBUSY;
#else
      /* The current BDT is not available, check the other BDT */

      volatile struct usbotg_bdtentry_s *otherbdt;
      otherbdt = &g_bdt[EP(epno, EP_DIR_IN, EP_PP_EVEN)];
      if (otherbdt == bdt)
        {
          otherbdt++;
        }

      /* Is it available? */

      if (otherbdt->status || otherbdt->addr)
        {
          /* No, neither are available.  We cannot perform the transfer now.
           * Return -EBUSY to indicate this condition.
           */

          return -EBUSY;
        }

      /* Yes... use the other BDT */

      bdt   = otherbdt;
      index = 1;
#endif
    }

  /* A BDT is available.  Which request should we be operating on?  The last
   * incomplete, active request would be at the tail of the active list.
   */

  privreq = khci_rqtail(&privep->active);

  /* This request would be NULL if there is no incomplete, active request. */

  if (privreq)
    {
      /* Get the number of bytes left to be transferred in the request */

      xfrd      = privreq->req.xfrd;
      bytesleft = privreq->req.len - xfrd;

      /* Even if the request is incomplete, transfer of all the requested
       * bytes may already been started.  NOTE: inflight[1] should be zero
       * because we know that there is a BDT available.
       */

#ifndef CONFIG_USBDEV_NOWRITEAHEAD
      DEBUGASSERT(privreq->inflight[1] == 0);
#endif
      /* Has the transfer been initiated for all of the bytes? */

      if (bytesleft > privreq->inflight[0])
        {
          /* No.. we have more work to do with this request */

          xfrd      += privreq->inflight[0];
          bytesleft -=  privreq->inflight[0];
        }

      /* Do we need to send a null packet after this packet? */

      else if (privep->txnullpkt)
        {
          /* Yes... set up for the NULL packet transfer */

          xfrd      = privreq->req.len;
          bytesleft = 0;
        }
      else
        {
          /* No.. We are finished with this request.  We need to get the
           * next request from the head of the pending request list.
           */

          privreq = NULL;
        }
    }

  /* If privreq is NULL here then either (1) there is no active request, or
   * (2) the (only) active request is fully queued.  In either case, we need
   * to get the next request from the head of the pending request list.
   */

  if (!privreq)
    {
      /* Remove the next request from the head of the pending request list */

      privreq = khci_remfirst(&privep->pend);
      if (!privreq)
        {
          /* The pending request list is empty. There are no queued TX
           * requests to be sent.
           */

          usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_EPINQEMPTY), epno);

          /* Return -ENODATA to indicate that there are no further requests
           * to be processed.
           */

          return -ENODATA;
        }

      /* Add this request to the tail of the active request list */

      khci_addlast(&privep->active, privreq);

      /* Set up the first transfer for this request */

      xfrd      = 0;
      bytesleft = privreq->req.len;
    }

  uinfo("epno=%d req=%p: len=%d xfrd=%d index=%d nullpkt=%d\n",
        epno, privreq, privreq->req.len, xfrd, index, privep->txnullpkt);

  /* Get the number of bytes left to be sent in the packet */

  nbytes = bytesleft;
  if (nbytes > 0 || privep->txnullpkt)
    {
      /* Either send the maxpacketsize or all of the remaining data in
       * the request.
       */

      privep->txnullpkt = 0;
      if (nbytes >= privep->ep.maxpacket)
        {
          nbytes =  privep->ep.maxpacket;

          /* Handle the case where this packet is exactly the
           * maxpacketsize.  Do we need to send a zero-length packet
           * in this case?
           */

          if (bytesleft ==  privep->ep.maxpacket &&
             (privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0)
            {
              privep->txnullpkt = 1;
            }
        }
    }

  /* Send the packet (might be a null packet with nbytes == 0) */

  buf = privreq->req.buf + xfrd;

  /* Setup the writes to the endpoints */

  khci_epwrite(privep, bdt, buf, nbytes);

  /* Special case endpoint 0 state information.  The write request is in
   * progress.
   */

  if (epno == 0)
    {
      priv->ctrlstate = CTRLSTATE_WRREQUEST;
    }

  /* Update for the next data IN interrupt */

  privreq->inflight[index] = nbytes;
  return OK;
}

/****************************************************************************
 * Name: khci_wrrequest
 ****************************************************************************/

static int khci_wrrequest(struct khci_usbdev_s *priv,
                          struct khci_ep_s *privep)
{
  int ret;

  /* Always try to start two transfers in order to take advantage of the
   * KHCI's ping pong buffering.
   */

  ret = khci_wrstart(priv, privep);
#ifndef CONFIG_USBDEV_NOWRITEAHEAD
  if (ret == OK)
    {
      /* Note:  We need to return the error condition only if nothing was
       * queued
       */

      khci_wrstart(priv, privep);
    }
#else
  UNUSED(ret);
#endif

  /* We return OK to indicate that a write request is still in progress */

  return khci_rqhead(&privep->active) == NULL ? -ENODATA : OK;
}

/****************************************************************************
 * Name: khci_rdcomplete
 ****************************************************************************/

static int khci_rdcomplete(struct khci_usbdev_s *priv,
                              struct khci_ep_s *privep)
{
  volatile struct usbotg_bdtentry_s *bdtout;
  struct khci_req_s *privreq;
  int epno;

  /* Check the request at the head of the endpoint's active request queue */

  privreq = khci_rqhead(&privep->active);
  if (!privreq)
    {
      /* There is no active packet waiting to receive any data. Then why are
       * we here?
       */

      usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_EPOUTQEMPTY),
               USB_EPNO(privep->ep.eplog));
      return -EINVAL;
    }

  /* bdtout should point to the BDT that just completed */

  bdtout = privep->bdtout;
  epno   = USB_EPNO(privep->ep.eplog);

  uinfo("EP%d: len=%d xfrd=%d\n",
        epno, privreq->req.len, privreq->req.xfrd);
  bdtinfo("EP%d BDT OUT [%p] {%08x, %08x}\n",
        epno, bdtout, bdtout->status, bdtout->addr);

  /* We should own the BDT that just completed */

  DEBUGASSERT((bdtout->status & USB_BDT_UOWN) == USB_BDT_COWN);

  /* Get the length of the data received from the BDT. */

  privreq->req.xfrd = (bdtout->status & USB_BDT_BYTECOUNT_MASK) >>
                       USB_BDT_BYTECOUNT_SHIFT;

  /* Complete the transfer and return the request to the class driver. */

  usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)), privreq->req.xfrd);
  khci_reqcomplete(privep, OK);

  /* Nullify the BDT entry that just completed.  Why?  So that we can tell
   * later that the BDT has been processed.  No, it is not sufficient to look
   * at the UOWN bit.  If UOWN==0, then the transfer has been completed BUT
   * it may not yet have been processed.
   */

  bdtout->status = 0;
  bdtout->addr   = 0;

  /* Toggle bdtout to the other BDT.  Is the current bdtout the EVEN bdt? */

  privep->bdtout = &g_bdt[EP_OUT_EVEN(epno)];
  if (bdtout == privep->bdtout)
    {
      /* Yes.. Then the other BDT is the ODD BDT */

      privep->bdtout++;
    }

  /* Set up the next read operation */

  return khci_rdrequest(priv, privep);
}

/****************************************************************************
 * Name: khci_ep0rdsetup
 ****************************************************************************/

static int khci_ep0rdsetup(struct khci_usbdev_s *priv, uint8_t *dest,
                              int readlen)
{
  volatile struct usbotg_bdtentry_s *bdtout;
  volatile struct usbotg_bdtentry_s *otherbdt;
  struct khci_ep_s *privep;
  uint32_t status;

  /* bdtout refers to the next ping-pong BDT to use. */

  privep = &priv->eplist[EP0];
  bdtout = privep->bdtout;

  /* Get the other BDT.  Check if the current BDT the EVEN BDT? */

  otherbdt = &g_bdt[EP_OUT_EVEN(EP0)];
  if (bdtout == otherbdt)
    {
      /* Yes.. then the other BDT is the ODD BDT. */

      otherbdt++;
    }

  /* If there is no RX transfer in progress, then the other BDT is setup
   * to receive the next setup packet.  There is a race condition here!
   * Stop any setup packet.
   */

  if (!priv->rxbusy)
    {
      /* Nullify all BDT OUT entries.  Why?  So that we can tell later
       * that the BDT available.  No, it is not sufficient to look at the
       * UOWN bit.  If UOWN==0, then the transfer has been completed BUT
       * it may not yet have been processed.  But a completely NULLified
       * BDT is a sure indication
       */

      bdtout->status   = 0;
      bdtout->addr     = 0;
      otherbdt->status = 0;
      otherbdt->addr   = 0;

      /* Reset the other BDT to zero... this will cause any attempted use
       * of the other BDT to be NAKed.  Set the first DATA0/1 value to 1.
       */

      privep->rxdata1  = 1;
    }

  /* Otherwise, there are RX transfers in progress.  bdtout may be
   * unavailable now.  In that case, we are free to setup the other BDT
   * in order to improve performance.  NOTE: That we check if the
   * entire BDT has been NULLified.  That is the only sure indication
   * that the BDT is available (see above).
   */

  if (bdtout->status || bdtout->addr)
    {
#ifdef CONFIG_USBDEV_NOREADAHEAD
      /* We will not try to read ahead */

      return -EBUSY;
#else
      /* bdtout is not available.  Is the other BDT available? */

      if (otherbdt->status || otherbdt->addr)
        {
          /* Neither are available... we cannot accept the request now */

          return -EBUSY;
        }

      /* Use the other BDT */

      bdtout = otherbdt;
#endif
    }

  usbtrace(TRACE_READ(EP0), readlen);

  /* Get the correct data toggle (as well as other BDT bits) */

  if (privep->rxdata1)
    {
      status          = (USB_BDT_UOWN | USB_BDT_DATA1 | USB_BDT_DTS);
      privep->rxdata1 = 0;
    }
  else
    {
      status          = (USB_BDT_UOWN | USB_BDT_DATA0 | USB_BDT_DTS);
      privep->rxdata1 = 1;
    }

  /* Set the data pointer, data length, and enable the endpoint */

  bdtout->addr  = (uint8_t *)dest;
  status       |= ((uint32_t)readlen << USB_BDT_BYTECOUNT_SHIFT);

  /* Then give the BDT to the USB */

  bdtinfo("EP0 BDT OUT [%p] {%08x, %08x}\n", bdtout, status, bdtout->addr);
  bdtout->status = status;

  priv->ctrlstate = CTRLSTATE_RDREQUEST;
  priv->rxbusy    = 1;
  return OK;
}

/****************************************************************************
 * Name: khci_rdsetup
 ****************************************************************************/

static int khci_rdsetup(struct khci_ep_s *privep, uint8_t *dest, int readlen)
{
  volatile struct usbotg_bdtentry_s *bdtout;
  uint32_t status;
  int epno;

  /* Select a BDT.  Check both the even and the ODD BDT and use the first one
   * that we own.
   */

  epno = USB_EPNO(privep->ep.eplog);

  /* bdtout refers to the next ping-pong BDT to use.  However, bdtout may be
   * unavailable now.  But, in that case, we are free to setup the other BDT
   * in order to improve performance.
   *
   * Note that we NULLify the BDT OUT entries.  This is so that we can tell
   * that the BDT readily available.  No, it is not sufficient to look at the
   * UOWN bit.  If UOWN==0, then the transfer has been completed BUT it may
   * not yet have been processed.  But a completely NULLified BDT is a sure
   * indication
   */

  bdtout = privep->bdtout;
  if (bdtout->status || bdtout->addr)
    {
#ifdef CONFIG_USBDEV_NOREADAHEAD
      /* We will not try to read-ahead */

      return -EBUSY;
#else
      volatile struct usbotg_bdtentry_s *otherbdt;

      /* Is the current BDT the EVEN BDT? */

      otherbdt = &g_bdt[EP_OUT_EVEN(epno)];
      if (bdtout == otherbdt)
        {
          /* Yes.. select the ODD BDT */

          otherbdt++;
        }

      /* Is the other BDT available? */

      if (otherbdt->status || otherbdt->addr)
        {
          /* Neither are available... we cannot accept the request now */

          return -EBUSY;
        }

      /* Use the other BDT */

      bdtout = otherbdt;
#endif
    }

  usbtrace(TRACE_READ(USB_EPNO(privep->ep.eplog)), readlen);

  /* Clear status bits (making sure that UOWN is cleared before doing
   * anything else).
   */

  bdtout->status = 0;

  /* Set the data pointer, data length, and enable the endpoint */

  bdtout->addr = (uint8_t *)dest;

  /* Get the correct data toggle. */

  if (privep->rxdata1)
    {
      status          = (USB_BDT_UOWN | USB_BDT_DATA1 | USB_BDT_DTS);
      privep->rxdata1 = 0;
    }
  else
    {
      status          = (USB_BDT_UOWN | USB_BDT_DATA0 | USB_BDT_DTS);
      privep->rxdata1 = 1;
    }

  /* Set the data length (preserving the data toggle). */

  status |= ((uint32_t)readlen << USB_BDT_BYTECOUNT_SHIFT);

  /* Then give the BDT to the USB */

  bdtinfo("EP%d BDT OUT [%p] {%08x, %08x}\n",
          epno, bdtout, status, bdtout->addr);

  bdtout->status = status;
  return OK;
}

/****************************************************************************
 * Name: khci_rdrequest
 ****************************************************************************/

static int khci_rdrequest(struct khci_usbdev_s *priv,
                             struct khci_ep_s *privep)
{
  struct khci_req_s *privreq;
  int readlen;
  int ret;

  /* Check the request at the head of the endpoint request queue */

  privreq = khci_rqhead(&privep->pend);
  if (!privreq)
    {
      /* There is no packet to receive any data. */

      usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_EPOUTQEMPTY),
               USB_EPNO(privep->ep.eplog));

      /* Special case reads from to endpoint zero.  If there is no transfer
       * in progress, then we need to configure to received the next SETUP
       * packet.
       */

      if (USB_EPNO(privep->ep.eplog) == 0 &&
          priv->ctrlstate == CTRLSTATE_RDREQUEST)
        {
          priv->ctrlstate = CTRLSTATE_WAITSETUP;
          priv->rxbusy    = 0;
        }

      return OK;
    }

  uinfo("EP%d: len=%d\n", USB_EPNO(privep->ep.eplog), privreq->req.len);

  /* Ignore any attempt to receive a zero length packet */

  if (privreq->req.len == 0)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_EPOUTNULLPACKET), 0);
      khci_reqcomplete(privep, OK);
      return OK;
    }

  /* Limit the size of the transfer to either the buffer size or the max
   * packet size of the endpoint.
   */

  readlen = MIN(privreq->req.len, privep->ep.maxpacket);

  /* Handle EP0 in a few special ways */

  if (USB_EPNO(privep->ep.eplog) == EP0)
    {
      ret = khci_ep0rdsetup(priv, privreq->req.buf, readlen);
    }
  else
    {
      ret = khci_rdsetup(privep, privreq->req.buf, readlen);
    }

  /* If the read request was successfully setup, then move the request from
   * the head of the pending request queue to the tail of the active request
   * queue.
   */

  if (ret == OK)
    {
      privreq = khci_remfirst(&privep->pend);
      DEBUGASSERT(privreq != NULL);
      khci_addlast(&privep->active, privreq);
    }

  return ret;
}

/****************************************************************************
 * Name: khci_cancelrequests
 ****************************************************************************/

static void khci_cancelrequests(struct khci_ep_s *privep, int16_t result)
{
  struct khci_req_s *privreq;

  while ((privreq = khci_remfirst(&privep->active)))
    {
      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)),
               privreq->req.xfrd);
      khci_reqreturn(privep, privreq, result);
    }

  while ((privreq = khci_remfirst(&privep->pend)))
    {
      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)),
               privreq->req.xfrd);
      khci_reqreturn(privep, privreq, result);
    }
}

/****************************************************************************
 * Interrupt Level Processing
 ****************************************************************************/

/****************************************************************************
 * Name: khci_dispatchrequest
 ****************************************************************************/

static void khci_dispatchrequest(struct khci_usbdev_s *priv)
{
  int ret;

  usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_DISPATCH), 0);
  if (priv && priv->driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, &priv->ctrl,
          priv->ep0data, priv->ep0datlen);
      if (ret < 0)
        {
          /* Stall on failure */

          usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_DISPATCHSTALL), 0);
          priv->ctrlstate = CTRLSTATE_STALL;
        }
    }
}

/****************************************************************************
 * Name: khci_ep0stall
 ****************************************************************************/

static void khci_ep0stall(struct khci_usbdev_s *priv)
{
  uint32_t regval;

  /* Check if EP0 is stalled */

  regval = khci_getreg(KINETIS_USB0_ENDPT0);
  if ((regval & USB_ENDPT_EPSTALL) != 0)
    {
      /* If so, clear the EP0 stall status */

      regval &= ~USB_ENDPT_EPSTALL;
      khci_putreg(regval, KINETIS_USB0_ENDPT0);
    }
}

/****************************************************************************
 * Name: khci_eptransfer
 ****************************************************************************/

static void khci_eptransfer(struct khci_usbdev_s *priv, uint8_t epno,
                               uint16_t ustat)
{
  struct khci_ep_s *privep;
  int ret;

  /* Decode and service non control endpoints interrupt */

  privep = &priv->eplist[epno];

  /* Check if the last transaction was an EP0 OUT transaction */

  if ((ustat & USB_STAT_TX) == USB_STAT_TX_OUT)
    {
      /* OUT: host-to-device */

      usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_EPOUTDONE), ustat);

      /* Handle read requests. Call khci_rdcomplete() to complete the OUT
       * transfer and setup the next out transfer.
       */

      ret = khci_rdcomplete(priv, privep);
#ifdef CONFIG_USBDEV_NOREADAHEAD
      if (ret == OK)
        {
          /* If that succeeds, then try to set up another OUT transfer. */

          khci_rdrequest(priv, privep);
        }
#else
      UNUSED(ret);
#endif
    }
  else
    {
      /* IN: device-to-host */

      usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_EPINDONE), ustat);

      /* An outgoing IN packet has completed.  Update the number of bytes
       * transferred and check for completion of the transfer.
       */

      khci_wrcomplete(priv, privep);

      /* Handle additional queued write requests */

      khci_wrrequest(priv, privep);
    }
}

/****************************************************************************
 * Name: khci_ep0nextsetup
 *
 * Description:
 *   This function is called (1) after successful completion of an EP0 Setup
 *   command, or (2) after receipt of the OUT complete event (for simple
 *   transfers).  It simply sets up the single BDT to accept the next
 *   SETUP command.
 *
 ****************************************************************************/

static void khci_ep0nextsetup(struct khci_usbdev_s *priv)
{
  volatile struct usbotg_bdtentry_s *bdt = priv->eplist[EP0].bdtout;
  uint32_t bytecount;

  /* This operation should be performed no more than once per OUT
   * transaction. priv->ep0done is set to zero at the beginning of processing
   * of each EP0 transfer.  It is set the first time that this function runs
   * after the EP0 transfer.
   */

  if (!priv->ep0done)
    {
      if (bdt == &g_bdt[EP0_OUT_EVEN])
        {
          bdt->addr = priv->out0data[EP0_OUT_EVEN];
        }
      else
        {
          bdt->addr = priv->out0data[EP0_OUT_ODD];
        }

      bytecount     = (CONFIG_USBDEV_EP0_MAXSIZE << USB_BDT_BYTECOUNT_SHIFT);
      bdt->status   = (USB_BDT_UOWN | bytecount);
      priv->ep0done = 1;
    }
}

/****************************************************************************
 * Name: khci_ep0rdcomplete
 *
 * Description:
 *   This function is called after a sequence of read sequence.  In this
 *   context, only one BDT is used.  Both BDTs must be prepared to receive
 *   SETUP packets.
 *
 ****************************************************************************/

static void khci_ep0rdcomplete(struct khci_usbdev_s *priv)
{
  volatile struct usbotg_bdtentry_s *bdt;
  struct khci_ep_s *ep0;
  uint32_t bytecount;

  /* This operation should be performed no more than once per OUT
   * transaction. priv->ep0done is set to zero at the beginning of processing
   * of each EP0 transfer.  It is set the first time that this function runs
   * after the EP0 transfer.
   */

  if (!priv->ep0done)
    {
      bytecount     = (CONFIG_USBDEV_EP0_MAXSIZE << USB_BDT_BYTECOUNT_SHIFT);

      bdt           = &g_bdt[EP0_OUT_EVEN];
      bdt->addr     = priv->out0data[EP0_OUT_EVEN];
      bdt->status   = (USB_BDT_UOWN | bytecount);

      bdt           = &g_bdt[EP0_OUT_ODD];
      bdt->addr     = priv->out0data[EP0_OUT_ODD];
      bdt->status   = (USB_BDT_UOWN | bytecount);

      priv->ep0done = 1;

      /* Data toggling is not used on SETUP transfers.  And IN transfer
       * resulting from a SETUP command should begin with DATA1.
       */

      ep0           = &priv->eplist[EP0];
      ep0->rxdata1  = 0;
      ep0->txdata1  = 1;
    }
}

/****************************************************************************
 * Name: khci_ep0setup
 ****************************************************************************/

static void khci_ep0setup(struct khci_usbdev_s *priv)
{
  volatile struct usbotg_bdtentry_s *bdt;
  struct khci_ep_s *ep0;
  struct khci_ep_s *privep;
  union wb_u value;
  union wb_u index;
  union wb_u len;
  union wb_u response;
  uint32_t regval;
  bool dispatched = false;
  uint8_t epno;
  int nbytes = 0; /* Assume zero-length packet */
  int ret;

  /* Cancel any pending requests. */

  ep0 = &priv->eplist[EP0];
  khci_cancelrequests(ep0, -EPROTO);

  /* Assume NOT stalled; no TX in progress; no RX overrun.  Data 0/1 toggling
   * is not used on SETUP packets, but any following EP0 IN transfer should
   * beginning with DATA1.
   */

  ep0->stalled = false;
  ep0->rxdata1 = 0;
  ep0->txdata1 = 1;

  /* Extract the little-endian 16-bit values to host order */

  value.w = GETUINT16(priv->ctrl.value);
  index.w = GETUINT16(priv->ctrl.index);
  len.w   = GETUINT16(priv->ctrl.len);
  response.w    = 0;

  /* Check to see if called from the DATA phase of a SETUP Transfer */

  if (priv->ctrlstate != CTRLSTATE_SETUP_READY &&
      priv->ctrlstate != CTRLSTATE_SETUP_OUT)
    {
      /* Not the data phase */

      uinfo("SETUP: type=%02x req=%02x value=%04x index=%04x len=%04x\n",
            priv->ctrl.type, priv->ctrl.req, value.w, index.w, len.w);

      /* Is this an setup with OUT and data of length > 0 */

      if (USB_REQ_ISOUT(priv->ctrl.type) && len.w > 0)
        {
          usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_EP0SETUPOUT), len.w);

          priv->ep0datlen = 0;
          priv->ep0datreq = len.w;

          /* At this point priv->ctrl is the setup packet. */

          khci_ep0nextsetup(priv);
          priv->ctrlstate = CTRLSTATE_SETUP_OUT;
        }
      else
        {
          priv->ctrlstate = CTRLSTATE_SETUP_READY;
        }
    }

  if (priv->ctrlstate == CTRLSTATE_SETUP_READY)
    {
      /* Dispatch any non-standard requests */

      if ((priv->ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
        {
          usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_NOSTDREQ),
                   priv->ctrl.type);

          /* Let the class implementation handle all non-standard requests */

          khci_dispatchrequest(priv);
          dispatched = true;
        }
      else
        {
          /* Handle standard request.  Pick off the things of interest to the
           * USB device controller driver; pass what is left to the class
           * driver
           */

          switch (priv->ctrl.req)
            {
            case USB_REQ_GETSTATUS:
              {
                /* type:  device-to-host; recipient = device, interface,
                 *        endpoint
                 * value: 0
                 * index: zero interface endpoint
                 * len:   2; data = status
                 */

                usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_GETSTATUS),
                         priv->ctrl.type);
                if (len.w != 2 || (priv->ctrl.type & USB_REQ_DIR_IN) == 0 ||
                    index.b[MSB] != 0 || value.w != 0)
                  {
                    usbtrace(TRACE_DEVERROR(
                             KHCI_TRACEERR_BADEPGETSTATUS), 0);
                    priv->ctrlstate = CTRLSTATE_STALL;
                  }
                else
                  {
                    switch (priv->ctrl.type & USB_REQ_RECIPIENT_MASK)
                      {
                       case USB_REQ_RECIPIENT_ENDPOINT:
                        {
                          epno = USB_EPNO(index.b[LSB]);
                          usbtrace(TRACE_INTDECODE(
                                   KHCI_TRACEINTID_EPGETSTATUS), epno);
                          if (epno >= KHCI_NENDPOINTS)
                            {
                              usbtrace(TRACE_DEVERROR(
                                       KHCI_TRACEERR_BADEPGETSTATUS), epno);
                              priv->ctrlstate = CTRLSTATE_STALL;
                            }
                          else
                            {
                              privep            = &priv->eplist[epno];
                              response.w        = 0; /* Not stalled */
                              nbytes            = 2; /* Response size: 2 bytes */

                              if (USB_ISEPIN(index.b[LSB]))
                                {
                                  /* IN endpoint */

                                  bdt = privep->bdtin;
                                }
                              else
                                {
                                  /* OUT endpoint */

                                  bdt = privep->bdtout;
                                }

                              /* BSTALL set if stalled */

                              if ((bdt->status & USB_BDT_BSTALL) != 0)
                                {
                                  response.b[LSB] = 1; /* Stalled, set bit 0 */
                                }
                            }
                        }
                        break;

                      case USB_REQ_RECIPIENT_DEVICE:
                        {
                         if (index.w == 0)
                            {
                              usbtrace(TRACE_INTDECODE(
                                       KHCI_TRACEINTID_DEVGETSTATUS), 0);

                              /* Features:
                               * Remote Wakeup=YES;
                               * selfpowered=?
                               */

                              response.w      = 0;
                              response.b[LSB] =
                                  (priv->selfpowered <<
                                   USB_FEATURE_SELFPOWERED) |
                                  (priv->rwakeup <<
                                   USB_FEATURE_REMOTEWAKEUP);
                              nbytes        = 2; /* Response size: 2 bytes */
                            }
                          else
                            {
                              usbtrace(TRACE_DEVERROR(
                                       KHCI_TRACEERR_BADDEVGETSTATUS), 0);
                              priv->ctrlstate = CTRLSTATE_STALL;
                            }
                        }
                        break;

                      case USB_REQ_RECIPIENT_INTERFACE:
                        {
                          usbtrace(TRACE_INTDECODE(
                                   KHCI_TRACEINTID_IFGETSTATUS), 0);
                          response.w        = 0;
                          nbytes            = 2; /* Response size: 2 bytes */
                        }
                        break;

                      default:
                        {
                          usbtrace(TRACE_DEVERROR(
                                   KHCI_TRACEERR_BADGETSTATUS), 0);
                          priv->ctrlstate = CTRLSTATE_STALL;
                        }
                        break;
                      }
                  }
              }
              break;

            case USB_REQ_CLEARFEATURE:
              {
                /* type:  host-to-device; recipient = device, interface or
                 *        endpoint
                 * value: feature selector
                 * index: zero interface endpoint;
                 * len:   zero, data = none
                 */

                usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_CLEARFEATURE),
                         priv->ctrl.type);
                if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
                     USB_REQ_RECIPIENT_DEVICE)
                  {
                    /* Disable B device from performing HNP */

        #ifdef CONFIG_KINETIS_USBOTG
                    if (value.w == USBOTG_FEATURE_B_HNP_ENABLE)
                      {
                        /* Disable HNP */
        #warning Missing Logic
                      }

                    /* Disable A device HNP support */

                    else if (value.w == USBOTG_FEATURE_A_HNP_SUPPORT)
                      {
                        /* Disable HNP support */
        #warning Missing Logic
                      }

                    /* Disable alternate HNP support */

                    else if (value.w == USBOTG_FEATURE_A_ALT_HNP_SUPPORT)
                      {
                        /* Disable alternate HNP */
        #warning Missing Logic
                      }
                    else
        #endif
                    /* Disable remote wakeup */

                    if (value.w == USB_FEATURE_REMOTEWAKEUP)
                      {
                        priv->rwakeup     = 0;
                      }
                    else
                      {
                        /* Let the class implementation handle all other
                         * device features
                         */

                        khci_dispatchrequest(priv);
                        dispatched = true;
                      }
                  }
                else if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
                    USB_REQ_RECIPIENT_ENDPOINT)
                  {
                    epno = USB_EPNO(index.b[LSB]);
                    if (epno > 0 && epno <
                        KHCI_NENDPOINTS && index.b[MSB] == 0 &&
                        value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
                      {
                        privep            = &priv->eplist[epno];
                        privep->halted    = false;
                        ret               = khci_epstall(&privep->ep, true);
                        UNUSED(ret);
                      }
                    else
                      {
                        usbtrace(TRACE_DEVERROR(
                                 KHCI_TRACEERR_BADCLEARFEATURE), 0);
                        priv->ctrlstate = CTRLSTATE_STALL;
                      }
                  }
                else
                  {
                    /* Let the class implementation handle all other
                     * recipients.
                     */

                    khci_dispatchrequest(priv);
                    dispatched = true;
                  }
              }
              break;

            case USB_REQ_SETFEATURE:
              {
                /* type:  host-to-device; recipient = device, interface,
                 *        endpoint
                 * value: feature selector
                 * index: zero interface endpoint;
                 * len:   0; data = none
                 */

                usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_SETFEATURE),
                         priv->ctrl.type);

                if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
                     USB_REQ_RECIPIENT_DEVICE)
                  {
                    /* Enable B device to perform HNP */

        #ifdef CONFIG_KINETIS_USBOTG
                    if (value.w == USBOTG_FEATURE_B_HNP_ENABLE)
                      {
                        /* Enable HNP */
        #warning "Missing logic"
                      }

                    /* Enable A device HNP supports */

                    else if (value.w == USBOTG_FEATURE_A_HNP_SUPPORT)
                      {
                        /* Enable HNP support */
        #warning "Missing logic"
                      }

                    /* Another port on the A device supports HNP */

                    else if (value.w == USBOTG_FEATURE_A_ALT_HNP_SUPPORT)
                      {
                        /* Enable alternate HNP */
        #warning "Missing logic"
                      }
                    else
        #endif

                    if (value.w == USB_FEATURE_REMOTEWAKEUP)
                      {
                        priv->rwakeup     = 0;
                      }
                    else if (value.w == USB_FEATURE_TESTMODE)
                      {
                        /* Special case recipient=device test mode */

                        uinfo("test mode: %d\n", index.w);
                      }
                    else
                      {
                        /* Let the class implementation handle all other
                         * device features
                         */

                        khci_dispatchrequest(priv);
                        dispatched = true;
                      }
                  }
                else if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
                    USB_REQ_RECIPIENT_ENDPOINT)
                  {
                    /* Handler recipient=endpoint */

                    epno = USB_EPNO(index.b[LSB]);
                    if (epno < KHCI_NENDPOINTS && index.b[MSB] == 0 &&
                        value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
                      {
                        privep            = &priv->eplist[epno];
                        privep->halted    = true;
                        ret               = khci_epstall(&privep->ep, false);
                        UNUSED(ret);
                      }
                    else
                      {
                        usbtrace(TRACE_DEVERROR(
                                 KHCI_TRACEERR_BADSETFEATURE), 0);
                        priv->ctrlstate = CTRLSTATE_STALL;
                      }
                  }
                else
                  {
                    /* The class driver handles all recipients except
                     * recipient=endpoint
                     */

                    khci_dispatchrequest(priv);
                    dispatched = true;
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

                usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_EP0SETUPSETADDRESS),
                         value.w);
                if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) !=
                    USB_REQ_RECIPIENT_DEVICE ||
                    index.w != 0 || len.w != 0 || value.w > 127)
                  {
                    usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_BADSETADDRESS), 0);
                    priv->ctrlstate = CTRLSTATE_STALL;
                  }
                else
                  {
                    /* Note that setting of the device address will be
                     * deferred. A zero-length packet will be sent and the
                     * device address will be set when the zerolength packet
                     * transfer completes.
                     */

                    priv->devstate = DEVSTATE_ADDRPENDING;
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
                usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_GETSETDESC),
                         priv->ctrl.type);
                if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
                    USB_REQ_RECIPIENT_DEVICE)
                  {
                    /* The request seems valid...
                     * let the class implementation handle it
                     */

                    khci_dispatchrequest(priv);
                    dispatched = true;
                  }
                else
                  {
                    usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_BADGETSETDESC), 0);
                    priv->ctrlstate = CTRLSTATE_STALL;
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
                usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_GETCONFIG),
                         priv->ctrl.type);
                if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
                    USB_REQ_RECIPIENT_DEVICE && value.w == 0 &&
                    index.w == 0 && len.w == 1)
                  {
                    /* The request seems valid... let the class
                     * implementation handle it
                     */

                    khci_dispatchrequest(priv);
                    dispatched = true;
                  }
                else
                  {
                    usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_BADGETCONFIG), 0);
                    priv->ctrlstate = CTRLSTATE_STALL;
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
                usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_SETCONFIG),
                         priv->ctrl.type);
                if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
                    USB_REQ_RECIPIENT_DEVICE && index.w == 0 && len.w == 0)
                  {
                     /* The request seems valid. Let the class implementation
                      * handle it
                      */

                     khci_dispatchrequest(priv);
                     dispatched = true;
                  }
                else
                  {
                    usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_BADSETCONFIG), 0);
                    priv->ctrlstate = CTRLSTATE_STALL;
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

                usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_GETSETIF),
                         priv->ctrl.type);
                khci_dispatchrequest(priv);
                dispatched = true;
              }
              break;

            case USB_REQ_SYNCHFRAME:
              /* type:  device-to-host; recipient = endpoint
               * value: 0
               * index: endpoint;
               * len:   2; data = frame number
               */

              {
                usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_SYNCHFRAME), 0);
              }
              break;

            default:
              {
                usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDCTRLREQ),
                         priv->ctrl.req);
                priv->ctrlstate = CTRLSTATE_STALL;
              }
              break;
            }
        }
    }

  /* PKTDIS bit is set when a Setup Transaction is received. Clear to resume
   * packet processing.
   */

  regval = khci_getreg(KINETIS_USB0_CTL);
  regval &= ~USB_CTL_TXSUSPENDTOKENBUSY;
  khci_putreg(regval, KINETIS_USB0_CTL);

  /* At this point, the request has been handled and there are three possible
   * outcomes:
   *
   * 1. The setup request was successfully handled above and a response
   *    packet must be sent (may be a zero length packet).
   * 2. The request was successfully handled by the class implementation.
   *    In case, the EP0 IN response has already been queued and the local
   *    variable 'dispatched' will be set to true and ctrlstate !=
   *    CTRLSTATE_STALL;
   * 3. An error was detected in either the above logic or by the class
   *    implementation logic.  In either case, priv->state will be set
   *    CTRLSTATE_STALL to indicate this case.
   *
   * NOTE:
   * Non-standard requests are a special case.  They are handled by the
   * class implementation and this function returned early above, skipping
   * this logic altogether.
   */

  if (priv->ctrlstate == CTRLSTATE_SETUP_READY)
    {
      if (!dispatched)
        {
          /* The SETUP command was not dispatched to the class driver and the
           * SETUP command did not cause a stall. We will respond.  First,
           * restrict the data length to the length requested in the setup
           * packet
           */

          if (nbytes > len.w)
            {
              nbytes = len.w;
            }

          /* Send the EP0 SETUP response (might be a zero-length packet) */

          khci_epwrite(ep0, ep0->bdtin, response.b, nbytes);
        }

      priv->ctrlstate = CTRLSTATE_WAITSETUP;
    }
  else if (priv->ctrlstate == CTRLSTATE_STALL)
    {
      /* Did we stall?
       * This might have occurred from the above logic OR the stall condition
       * may have been set less obviously in khci_dispatchrequest().
       * In either case, we handle the stall condition the same.
       *
       * However, bad things happen if we try to stall a SETUP packet.  So
       * lets So lets not.  If we wait a bit, things will recover.
       * Hmmm.. If we completed the data phase (perhaps by sending a NULL
       * packet), then I think we could stall the endpoint and perhaps speed
       * things up a bit???.
       */

      priv->ctrlstate = CTRLSTATE_WAITSETUP;
    }

  /* Set up the BDT to accept the next setup command. */

  khci_ep0nextsetup(priv);
}

/****************************************************************************
 * Name: khci_ep0incomplete
 ****************************************************************************/

static void khci_ep0incomplete(struct khci_usbdev_s *priv)
{
  struct khci_ep_s *ep0 = &priv->eplist[EP0];
  volatile struct usbotg_bdtentry_s *bdtlast;
  int ret;

  /* Get the last BDT and make sure that we own it. */

  bdtlast = ep0->bdtin;

  /* Make sure that we own the last BDT. */

  bdtlast->status = 0;
  bdtlast->addr   = 0;

  /* Are we processing the completion of one packet of an outgoing request
   * from the class driver?
   */

  if (priv->ctrlstate == CTRLSTATE_WRREQUEST)
    {
      /* An outgoing EP0 transfer has completed.  Update the byte count and
       * check for the completion of the transfer.
       *
       * NOTE: khci_wrcomplete() will toggle bdtin to the other buffer so
       * we do not need to that for this case.
       */

      khci_wrcomplete(priv, &priv->eplist[EP0]);

      /* Handle the next queue IN transfer.  If there are no further queued
       * IN transfers, khci_wrrequest will return -ENODATA and that is the
       * only expected error return value in this context.
       */

      ret = khci_wrrequest(priv, &priv->eplist[EP0]);
      if (ret < 0)
        {
          DEBUGASSERT(ret == -ENODATA);

          /* If there is nothing to be sent, then we need to configure to
           * receive the next SETUP packet.
           */

          priv->ctrlstate = CTRLSTATE_WAITSETUP;
        }
    }

  /* No.. Are we processing the completion of a status response? */

  else if (priv->ctrlstate == CTRLSTATE_WAITSETUP)
    {
      /* Get the next IN BDT */

      if (bdtlast == &g_bdt[EP0_IN_EVEN])
        {
          ep0->bdtin = &g_bdt[EP0_IN_ODD];
        }
      else
        {
          DEBUGASSERT(bdtlast == &g_bdt[EP0_IN_ODD]);
          ep0->bdtin = &g_bdt[EP0_IN_EVEN];
        }

      /* Look at the saved SETUP command.  Was it a SET ADDRESS request?
       * If so, then now is the time to set the address.
       */

      if (priv->devstate == DEVSTATE_ADDRPENDING)
        {
          uint16_t addr = GETUINT16(priv->ctrl.value);
          usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_EP0ADDRESSSET), addr);

          /* This should be the equivalent state */

          DEBUGASSERT(priv->ctrl.req == USB_REQ_SETADDRESS &&
                     (priv->ctrl.type & REQRECIPIENT_MASK) ==
                     (USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE));

          /* Set (or clear) the address */

          khci_putreg(addr, KINETIS_USB0_ADDR);
          if (addr > 0)
            {
              priv->devstate = DEVSTATE_ADDRESS;
            }
          else
            {
              priv->devstate = DEVSTATE_DEFAULT;
            }
        }
    }

  /* No other state is expected in this context */

  else
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDSTATE),
               priv->ctrlstate);
      priv->ctrlstate = CTRLSTATE_STALL;
    }
}

/****************************************************************************
 * Name: khci_ep0outcomplete
 ****************************************************************************/

static void khci_ep0outcomplete(struct khci_usbdev_s *priv)
{
  struct khci_ep_s *ep0 = &priv->eplist[EP0];

  switch (priv->ctrlstate)
    {
      /* Read request in progress */

      case CTRLSTATE_RDREQUEST:

        /* Process the next read request for EP0 */

        khci_rdcomplete(priv, ep0);

        /* Was this the end of the OUT transfer? */

        if (priv->ctrlstate == CTRLSTATE_WAITSETUP)
          {
            /* Prepare EP0 OUT for the next SETUP transaction. */

            khci_ep0rdcomplete(priv);
          }
        break;

      /* No transfer in progress, waiting for SETUP */

      case CTRLSTATE_WAITSETUP:
        {
          /* In this case the last OUT transaction must have been a status
           * stage of a CTRLSTATE_WRREQUEST: Prepare EP0 OUT for the next
           * SETUP transaction.
           */

           khci_ep0nextsetup(priv);
        }
        break;

      /* Unexpected state OR host aborted the OUT transfer before it
       * completed, STALL the endpoint in either case
       */

      default:
        {
          usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDSTATE),
                   priv->ctrlstate);
          priv->ctrlstate = CTRLSTATE_STALL;
        }
        break;
    }
}

/****************************************************************************
 * Name: khci_ep0transfer
 ****************************************************************************/

static void khci_ep0transfer(struct khci_usbdev_s *priv, uint16_t ustat)
{
  volatile struct usbotg_bdtentry_s *bdt;

  /* The following information is available in the status register :
   *
   * ENDPT - The 4 bit endpoint number that cause the interrupt.
   * DIR   - The direction of the endpoint.
   * PPBI  - The ping-pong buffer used in the transaction.
   */

  priv->ep0done = 0;

  /* Check if the last transaction was an EP0 OUT transaction */

  if ((ustat & USB_STAT_TX) == USB_STAT_TX_OUT)
    {
      int index;

      /* It was an EP0 OUT transaction.  Get the index to the BDT. */

      index = ((ustat & USB_STAT_ODD) == 0 ? EP0_OUT_EVEN : EP0_OUT_ODD);
      bdt   = &g_bdt[index];
      priv->eplist[0].bdtout = bdt;

      bdtinfo("EP0 BDT OUT [%p] {%08x, %08x}\n", bdt, bdt->status,
              bdt->addr);

      /* Check the current EP0 OUT buffer contains a SETUP packet */

      if (((bdt->status & USB_BDT_PID_MASK) >>
           USB_BDT_PID_SHIFT) == USB_PID_SETUP_TOKEN)
        {
          void *src = (void *)bdt->addr;
          void *dest = &priv->ctrl;

          memcpy(dest, src, USB_SIZEOF_CTRLREQ);

          /* Handle the control OUT transfer */

          khci_ep0setup(priv);

          if (priv->ctrlstate == CTRLSTATE_WAITSETUP)
            {
              usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_EP0SETUPDONE),
                       bdt->status);
            }
          else
            {
              usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_EP0SETUPOUT),
                       bdt->status);
            }
        }
      else if (priv->ctrlstate == CTRLSTATE_SETUP_OUT)
        {
          void *src = (void *)bdt->addr;
          void *dest = (void *)&priv->ep0data[priv->ep0datlen];
          uint16_t readlen = MIN((priv->ep0datreq - priv->ep0datlen),
                                  priv->eplist[0].ep.maxpacket);

          memcpy(dest, src, readlen);

          priv->ep0datlen += readlen;

          if (priv->ep0datlen == priv->ep0datreq)
            {
              priv->ctrlstate = CTRLSTATE_SETUP_READY;
            }

          khci_ep0setup(priv);

          if (priv->ctrlstate == CTRLSTATE_WAITSETUP)
            {
              usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_EP0SETUPDONE),
                       bdt->status);
            }
          else
            {
              usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_EP0SETUPOUTDATA),
                       bdt->status);
            }
        }
      else
        {
          /* Handle the data OUT transfer */

          usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_EP0OUTDONE), ustat);
          khci_ep0outcomplete(priv);
        }
    }

  /* No.. it was an EP0 IN transfer */

  else /* if ((status & USB_STAT_TX) == USB_STAT_TX_IN) */
    {
      usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_EP0INDONE), ustat);

      /* Handle the IN transfer complete */

      khci_ep0incomplete(priv);
    }

  /* Check for a request to stall EP0 */

  if (priv->ctrlstate == CTRLSTATE_STALL)
    {
      /* Stall EP0 */

      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_EP0SETUPSTALLED),
               priv->ctrlstate);
      khci_epstall(&priv->eplist[EP0].ep, false);
    }
}

/****************************************************************************
 * Name: khci_interrupt
 ****************************************************************************/

static int khci_interrupt(int irq, void *context, void *arg)
{
  uint16_t usbir;
  uint32_t regval;
  int i;
#ifdef CONFIG_KINETIS_USBOTG
  uint16_t otgir;
#endif

  struct khci_usbdev_s *priv = (struct khci_usbdev_s *) arg;

  /* Get the set of pending USB and OTG interrupts interrupts */

  usbir = khci_getreg(KINETIS_USB0_ISTAT) &
                      khci_getreg(KINETIS_USB0_INTEN);

#if !defined(CONFIG_KINETIS_USBOTG)
  usbtrace(TRACE_INTENTRY(KHCI_TRACEINTID_INTERRUPT), usbir);
#else
  otgir = khci_getreg(KINETIS_USB0_OTGISTAT) &
                      khci_getreg(KINETIS_USB0_OTGICR);

  usbtrace(TRACE_INTENTRY(KHCI_TRACEINTID_INTERRUPT), usbir | otgir);

  /* Session Request Protocol (SRP) Time Out Check */

  /* Check if USB OTG SRP is ready */
#  warning "Missing logic"
  {
    /* Check if the 1 millisecond timer has expired */

    if ((otgir & USB_OTGISTAT_ONEMSEC) != 0)
      {
        usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_T1MSEC), otgir);

        /* Check for the USB OTG SRP timeout */
#  warning "Missing logic"
          {
              /* Handle OTG events of the SRP timeout has expired */
#  warning "Missing logic"
          }

          /* Clear Interrupt 1 msec timer Flag */

          khci_putreg(USB_OTGISTAT_ONEMSEC, KINETIS_USB0_OTGISTAT);
      }
  }
#endif

  /* Handle events while we are in the attached state */

  if (priv->devstate == DEVSTATE_ATTACHED)
    {
      /* Now were are in the powered state */

      priv->devstate = DEVSTATE_POWERED;
    }

  /* Service error interrupts */

  if ((usbir & USB_INT_ERROR) != 0)
    {
      usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_UERR), usbir);
      uerr("ERROR: EIR=%04x\n", khci_getreg(KINETIS_USB0_ERRSTAT));

      /* Clear all pending USB error interrupts */

      khci_putreg(USB_EINT_ALL, KINETIS_USB0_ERRSTAT);
      khci_putreg(USB_INT_ERROR, KINETIS_USB0_ISTAT);
    }

  /* Service resume interrupts */

  if ((usbir & USB_INT_RESUME) != 0)
    {
      usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_RESUME), usbir);
      khci_resume(priv);
    }

  /* Service USB Bus Reset Interrupt. */

  if ((usbir & USB_INT_USBRST) != 0)
    {
      usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_RESET), usbir);

      /* Reset interrupt received. Restore our initial state.  NOTE:  the
       * hardware automatically resets the USB address, so we really just
       * need reset any existing configuration/transfer states.
       */

      khci_swreset(priv);
      khci_hwreset(priv);

      /* Configure EP0 */

      khci_ep0configure(priv);
      priv->devstate = DEVSTATE_DEFAULT;

#ifdef CONFIG_KINETIS_USBOTG
        /* Disable and deactivate HNP */
#warning Missing Logic
#endif
      /* Acknowledge the reset interrupt */

      khci_putreg(USB_INT_USBRST, KINETIS_USB0_ISTAT);
      goto interrupt_exit;
    }

#ifdef CONFIG_KINETIS_USBOTG
  /* Check if the ID Pin Changed State */

  if ((otgir & USB_OTGISTAT_IDCHG) != 0)
    {
      usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_OTGID), otgir);

      /* Re-detect and re-initialize */
#warning "Missing logic"

      khci_putreg(USB_OTGISTAT_IDCHG, KINETIS_USB0_OTGISTAT);
    }
#endif

  /*  Service USB Transaction Complete Interrupt */

  if ((usbir & USB_INT_TOKDNE) != 0)
    {
      usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_TRNC), usbir);

      /* Drain the USAT FIFO entries.  If the USB FIFO ever gets full, USB
       * bandwidth utilization can be compromised, and the device won't be
       * able to receive SETUP packets.
       */

      for (i = 0; i < 4; i++)
        {
          uint8_t epno;

          /* Check the pending interrupt register.
           *  Is token processing complete.
           */

          if ((khci_getreg(KINETIS_USB0_ISTAT) & USB_INT_TOKDNE) != 0)
            {
              regval = khci_getreg(KINETIS_USB0_STAT);
              khci_putreg(USB_INT_TOKDNE, KINETIS_USB0_ISTAT);

              usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_TRNCS), regval);

              /* Handle the endpoint transfer complete event. */

              epno = (regval & USB_STAT_ENDP_MASK) >> USB_STAT_ENDP_SHIFT;
              if (epno == 0)
                {
                   khci_ep0transfer(priv, regval);
                }
              else
                {
                  khci_eptransfer(priv, epno, regval);
                }
            }
          else
            {
               /* USTAT FIFO must be empty. */

               break;
            }
        }
    }

  /* Service IDLE interrupts */

  if ((usbir & USB_INT_SLEEP) != 0)
    {
      usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_IDLE), usbir);

#ifdef CONFIG_KINETIS_USBOTG
      /* If Suspended, Try to switch to Host */
#warning "Missing logic"
#else
      khci_suspend(priv);

#endif
      khci_putreg(USB_INT_SLEEP, KINETIS_USB0_ISTAT);
    }

  /* Check for asynchronous resume interrupt */

  if ((khci_getreg(KINETIS_USB0_USBTRC0) & USB_USBTRC0_RESUME_INT) != 0)
    {
      /* Just clear the asynchronous resume interrupt enable */

      regval = khci_getreg(KINETIS_USB0_USBTRC0);
      regval &= ~USB_USBTRC0_USBRESMEN;
      khci_putreg(regval, KINETIS_USB0_USBTRC0);
    }

  /*  It is pointless to continue servicing if the device is in suspend
   * mode.
   */

  if ((khci_getreg(KINETIS_USB0_USBCTRL) & USB_USBCTRL_SUSP) != 0)
    {
      /* If we are still suspended then re-enable asynchronous resume
       * interrupt
       */

      regval = khci_getreg(KINETIS_USB0_USBTRC0);
      regval |= USB_USBTRC0_USBRESMEN;
      khci_putreg(regval, KINETIS_USB0_USBTRC0);

      usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_SUSPENDED),
               khci_getreg(KINETIS_USB0_CTL));
      goto interrupt_exit;
    }

  /* Service SOF interrupts */

#ifdef CONFIG_USB_SOFINTS
  if ((usbir & USB_INT_SOFTOK) != 0)
    {
      usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_SOF), 0);

      /* I am not sure why you would ever enable SOF interrupts */

      khci_putreg(USB_INT_SOFTOK, KINETIS_USB0_ISTAT);
    }
#endif

  /* Service stall interrupts */

  if ((usbir & USB_INT_STALL) != 0)
    {
      usbtrace(TRACE_INTDECODE(KHCI_TRACEINTID_STALL), usbir);

      khci_ep0stall(priv);

      /* Clear the pending STALL interrupt */

      khci_putreg(USB_INT_STALL, KINETIS_USB0_ISTAT);
    }

  /* Clear the pending USB interrupt.  Goto is used in the above to assure
   * that all interrupt exits pass through this logic.
   */

interrupt_exit:
  kinetis_clrpend(KINETIS_IRQ_USBOTG);
#ifdef CONFIG_KINETIS_USBOTG
  usbtrace(TRACE_INTEXIT(KHCI_TRACEINTID_INTERRUPT), usbir | otgir);
#else
  usbtrace(TRACE_INTEXIT(KHCI_TRACEINTID_INTERRUPT), usbir);
#endif
  return OK;
}

/****************************************************************************
 * Suspend/Resume Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: khci_suspend
 ****************************************************************************/

#ifndef CONFIG_KINETIS_USBOTG
static void khci_suspend(struct khci_usbdev_s *priv)
{
  uint32_t regval;

  /* Notify the class driver of the suspend event */

  if (priv->driver)
    {
      CLASS_SUSPEND(priv->driver, &priv->usbdev);
    }

  /* Disable further IDLE interrupts.  Once is enough. */

  regval  = khci_getreg(KINETIS_USB0_INTEN);
  regval &= ~USB_INT_SLEEP;
  khci_putreg(regval, KINETIS_USB0_INTEN);

  /* Enable Resume */

  regval |= USB_INT_RESUME;
  khci_putreg(regval, KINETIS_USB0_INTEN);

  regval  = khci_getreg(KINETIS_USB0_USBCTRL);
  regval |= USB_USBCTRL_SUSP | USB_USBCTRL_PDE;
  khci_putreg(regval, KINETIS_USB0_USBCTRL);

  regval = khci_getreg(KINETIS_USB0_USBTRC0);
  regval |= USB_USBTRC0_USBRESMEN;
  khci_putreg(regval, KINETIS_USB0_USBTRC0);

  /* Invoke a callback into board-specific logic.  The board-specific logic
   * may enter into sleep or idle modes or switch to a slower clock, etc.
   */

  kinetis_usbsuspend((struct usbdev_s *)priv, false);
}
#endif

/****************************************************************************
 * Name: khci_remote_resume
 ****************************************************************************/

static void khci_remote_resume(struct khci_usbdev_s *priv)
{
  uint32_t regval;

  /* Start RESUME signaling */

  regval = khci_getreg(KINETIS_USB0_CTL);
  regval |= USB_CTL_RESUME;
  khci_putreg(regval, KINETIS_USB0_CTL);

  /* Keep the RESUME line set for 1-13 ms */

  up_mdelay(10);

  regval &= ~USB_CTL_RESUME;
  khci_putreg(regval, KINETIS_USB0_CTL);
}

/****************************************************************************
 * Name: khci_resume
 ****************************************************************************/

static void khci_resume(struct khci_usbdev_s *priv)
{
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  /* This function is called when the USB resume interrupt occurs.
   * If using clock switching, this is the place to call out to
   * logic to restore the original MCU core clock frequency.
   */

  kinetis_usbsuspend((struct usbdev_s *)priv, true);

  /* Unsuspend */

  regval  = khci_getreg(KINETIS_USB0_USBCTRL);
  regval &= ~(USB_USBCTRL_SUSP | USB_USBCTRL_PDE);
  khci_putreg(regval, KINETIS_USB0_USBCTRL);

  /* Enable the IDLE interrupt */

  regval  = khci_getreg(KINETIS_USB0_INTEN);
  regval |= USB_INT_SLEEP;
  khci_putreg(regval, KINETIS_USB0_INTEN);

  /* Disable the RESUME interrupt */

  regval &= ~USB_INT_RESUME;
  khci_putreg(regval, KINETIS_USB0_INTEN);

  /* Disable the the async resume interrupt */

  regval = khci_getreg(KINETIS_USB0_USBTRC0);
  regval &= ~USB_USBTRC0_USBRESMEN;
  khci_putreg(regval, KINETIS_USB0_USBTRC0);

  khci_putreg(USB_INT_RESUME, KINETIS_USB0_ISTAT);

  /* Notify the class driver of the resume event */

  if (priv->driver)
    {
      CLASS_RESUME(priv->driver, &priv->usbdev);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Endpoint Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: khci_epreserve
 ****************************************************************************/

static inline struct khci_ep_s *
khci_epreserve(struct khci_usbdev_s *priv, uint8_t epset)
{
  struct khci_ep_s *privep = NULL;
  irqstate_t flags;
  int epndx = 0;

  flags = enter_critical_section();
  epset &= priv->epavail;
  if (epset)
    {
      /* Select the lowest bit in the set of matching, available endpoints
       * (skipping EP0)
       */

      for (epndx = 1; epndx < KHCI_NENDPOINTS; epndx++)
        {
          uint8_t bit = KHCI_ENDP_BIT(epndx);
          if ((epset & bit) != 0)
            {
              /* Mark the endpoint no longer available */

              priv->epavail &= ~bit;

              /* And return the pointer to the standard endpoint structure */

              privep = &priv->eplist[epndx];
              break;
            }
        }
    }

  leave_critical_section(flags);
  return privep;
}

/****************************************************************************
 * Name: khci_epunreserve
 ****************************************************************************/

static inline void
khci_epunreserve(struct khci_usbdev_s *priv, struct khci_ep_s *privep)
{
  irqstate_t flags = enter_critical_section();
  priv->epavail   |= KHCI_ENDP_BIT(USB_EPNO(privep->ep.eplog));
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: khci_ep0configure
 ****************************************************************************/

static void khci_ep0configure(struct khci_usbdev_s *priv)
{
  volatile struct usbotg_bdtentry_s *bdt;
  struct khci_ep_s *ep0;
  uint32_t bytecount;

  /* Configure the OUT BDTs.  We assume that the ping-poing buffer index has
   * just been reset and we expect to receive on the EVEN BDT first.  Data
   * toggle synchronization is not needed for SETUP packets.
   */

  /* Disabled the Endpoint first */

  khci_putreg(KHCI_EP_DISABLED, KINETIS_USB0_ENDPT0);

  ep0         = &priv->eplist[EP0];
  bytecount   = (CONFIG_USBDEV_EP0_MAXSIZE << USB_BDT_BYTECOUNT_SHIFT);

  bdt         = &g_bdt[EP0_OUT_EVEN];
  bdt->addr   = priv->out0data[EP0_OUT_EVEN];
  bdt->status = (USB_BDT_UOWN | bytecount);
  ep0->bdtout = bdt;

  bdt         = &g_bdt[EP0_OUT_ODD];
  bdt->addr   = priv->out0data[EP0_OUT_ODD];
  bdt->status = (USB_BDT_UOWN | bytecount);

  /* Configure the IN BDTs. */

  bdt         = &g_bdt[EP0_IN_EVEN];
  bdt->status = 0;
  bdt->addr   = 0;
  ep0->bdtin  = bdt;

  bdt         = &g_bdt[EP0_IN_ODD];
  bdt->status = 0;
  bdt->addr   = 0;

  /* Data toggling is not used on SETUP transfers.  And IN transfer resulting
   * from a SETUP command should begin with DATA1.
   */

  ep0->rxdata1 = 0;
  ep0->txdata1 = 1;

  /* Enable the EP0 endpoint */

  khci_putreg(KHCI_EP_CONTROL, KINETIS_USB0_ENDPT0);
}

/****************************************************************************
 * Endpoint operations
 ****************************************************************************/

/****************************************************************************
 * Name: khci_epconfigure
 ****************************************************************************/

static int khci_epconfigure(struct usbdev_ep_s *ep,
                            const struct usb_epdesc_s *desc,
                            bool last)
{
  struct khci_ep_s *privep = (struct khci_ep_s *)ep;
  volatile struct usbotg_bdtentry_s *bdt;
  uint16_t maxpacket;
  uint32_t regval;
  uint8_t  epno;
  bool     epin;
  bool     bidi;
  int      index;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !desc)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDPARMS), 0);
      uerr("ERROR: ep=%p desc=%p\n", ep, desc);
      return -EINVAL;
    }
#endif

  /* Get the unadorned endpoint address */

  epno = USB_EPNO(desc->addr);
  epin = USB_ISEPIN(desc->addr);

  usbtrace(TRACE_EPCONFIGURE, (uint16_t)epno);
  DEBUGASSERT(epno == USB_EPNO(ep->eplog));

  /* Set the requested type */

  switch (desc->attr & USB_EP_ATTR_XFERTYPE_MASK)
  {
    case USB_EP_ATTR_XFER_INT: /* Interrupt endpoint */
      regval = epin ? KHCI_EP_INTIN : KHCI_EP_INTOUT;
      break;

    case USB_EP_ATTR_XFER_BULK: /* Bulk endpoint */
      regval = epin ? KHCI_EP_BULKIN : KHCI_EP_BULKOUT;
      break;

    case USB_EP_ATTR_XFER_ISOC: /* Isochronous endpoint */
      regval = epin ? KHCI_EP_ISOCIN : KHCI_EP_ISOCOUT;
      break;

    case USB_EP_ATTR_XFER_CONTROL: /* Control endpoint */
      regval = KHCI_EP_CONTROL;
      bidi   = true;
      break;

    default:
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_BADEPTYPE),
              (uint16_t)desc->type);
      return -EINVAL;
  }

  /* First disable the endpoint */

  khci_putreg(KHCI_EP_DISABLED, KINETIS_USB0_ENDPT(epno));

  /* Setup up buffer descriptor table (BDT) entry/ies for this endpoint */

  if (epin || bidi)
    {
      /* Get the pointer to BDT entry */

      index         =  EP(epno, EP_DIR_IN, EP_PP_EVEN);
      bdt           = &g_bdt[index];
      privep->bdtin = bdt;

      /* Mark that we own the entry */

      bdt->status = 0;
      bdt->addr   = 0;

      bdtinfo("EP%d BDT IN [%p] {%08x, %08x}\n",
              epno, bdt, bdt->status, bdt->addr);

      /* Now do the same for the other buffer. */

      bdt++;
      bdt->status = 0;
      bdt->addr   = 0;

      bdtinfo("EP%d BDT IN [%p] {%08x, %08x}\n",
              epno, bdt, bdt->status, bdt->addr);
    }

  if (!epin || bidi)
    {
      index          =  EP(epno, EP_DIR_OUT, EP_PP_EVEN);
      bdt            = &g_bdt[index];
      privep->bdtout = bdt;

      /* Mark that we own the entry */

      bdt->status = 0;
      bdt->addr   = 0;

      bdtinfo("EP%d BDT OUT [%p] {%08x, %08x}\n",
              epno, bdt, bdt->status, bdt->addr);

      /* Now do the same for the other buffer. */

      bdt++;
      bdt->status = 0;
      bdt->addr   = 0;

      bdtinfo("EP%d BDT OUT [%p] {%08x, %08x}\n",
              epno, bdt, bdt->status, bdt->addr);
    }

  /* Get the maxpacket size of the endpoint. */

  maxpacket = GETUINT16(desc->mxpacketsize);
  DEBUGASSERT(maxpacket <= KHCI_MAXPACKET_SIZE);
  ep->maxpacket = maxpacket;

  /* Set the full, logic EP number
   * (that includes direction encoded in bit 7)
   */

  if (epin)
    {
      ep->eplog = USB_EPIN(epno);
    }
  else
    {
      ep->eplog = USB_EPOUT(epno);
    }

  khci_putreg(regval, KINETIS_USB0_ENDPT(epno));

  return OK;
}

/****************************************************************************
 * Name: khci_epdisable
 ****************************************************************************/

static int khci_epdisable(struct usbdev_ep_s *ep)
{
  struct khci_ep_s *privep;
  volatile uint32_t *ptr;
  int epno;
  int i;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDPARMS), 0);
      uerr("ERROR: ep=%p\n", ep);
      return -EINVAL;
    }
#endif

  privep = (struct khci_ep_s *)ep;
  epno   = USB_EPNO(ep->eplog);
  usbtrace(TRACE_EPDISABLE, epno);

  /* Cancel any ongoing activity */

  flags = enter_critical_section();
  khci_cancelrequests(privep, -ESHUTDOWN);

  /* Disable the endpoint */

  khci_putreg(KHCI_EP_DISABLED, KINETIS_USB0_ENDPT(epno));

  /* Reset the BDTs for the endpoint.  Four BDT entries per endpoint; Two
   * 32-bit words per BDT.
   */

  ptr = (uint32_t *)&g_bdt[EP(epno, EP_DIR_OUT, EP_PP_EVEN)];
  for (i = 0; i < USB_BDT_WORD_SIZE * USB_NBDTS_PER_EP; i++)
    {
      *ptr++ = 0;
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: khci_epallocreq
 ****************************************************************************/

static struct usbdev_req_s *khci_epallocreq(struct usbdev_ep_s *ep)
{
  struct khci_req_s *privreq;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif

  usbtrace(TRACE_EPALLOCREQ, USB_EPNO(ep->eplog));

  privreq = (struct khci_req_s *)kmm_malloc(sizeof(struct khci_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct khci_req_s));
  return &privreq->req;
}

/****************************************************************************
 * Name: khci_epfreereq
 ****************************************************************************/

static void khci_epfreereq(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct khci_req_s *privreq = (struct khci_req_s *)req;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  usbtrace(TRACE_EPFREEREQ, USB_EPNO(ep->eplog));

  kmm_free(privreq);
}

/****************************************************************************
 * Name: khci_epsubmit
 ****************************************************************************/

static int khci_epsubmit(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct khci_req_s *privreq = (struct khci_req_s *)req;
  struct khci_ep_s *privep = (struct khci_ep_s *)ep;
  struct khci_usbdev_s *priv;
  irqstate_t flags;
  uint8_t epno;
  int ret = OK;

#ifdef CONFIG_DEBUG_FEATURES
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDPARMS), 0);
      uerr("ERROR: req=%p callback=%p buf=%p ep=%p\n",
           req, req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, USB_EPNO(ep->eplog));
  priv = privep->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv->driver)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_NOTCONFIGURED),
               priv->usbdev.speed);
      uerr("ERROR: driver=%p\n", priv->driver);
      return -ESHUTDOWN;
    }
#endif

  /* Handle the request from the class driver */

  epno                 = USB_EPNO(ep->eplog);
  req->result          = -EINPROGRESS;
  req->xfrd            = 0;
  privreq->inflight[0] = 0;
#ifndef CONFIG_USBDEV_NOWRITEAHEAD
  privreq->inflight[1] = 0;
#endif
  flags                = enter_critical_section();

  /* Add the new request to the request queue for the OUT endpoint */

  khci_addlast(&privep->pend, privreq);

  /* Handle IN (device-to-host) requests.  NOTE:  If the class device is
   * using the bi-directional EP0, then we assume that they intend the EP0
   * IN functionality.
   */

  if (USB_ISEPIN(ep->eplog) || epno == EP0)
    {
      usbtrace(TRACE_INREQQUEUED(epno), req->len);

      /* If the endpoint is not stalled and an IN endpoint BDT is available,
       * then transfer the data now.
       */

      if (!privep->stalled)
        {
          khci_wrrequest(priv, privep);
        }
    }

  /* Handle OUT (host-to-device) requests */

  else
    {
      usbtrace(TRACE_OUTREQQUEUED(epno), req->len);

      /* Set up the read operation (unless the endpoint is stalled).  Because
       * the KHCI supports ping-pong* buffering.  There may be two pending
       * read requests.  The following call will attempt to setup a read
       * using this request for this endpoint.  It is not harmful if this
       * fails.
       */

      if (!privep->stalled)
        {
          khci_rdrequest(priv, privep);
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: khci_epcancel
 ****************************************************************************/

static int khci_epcancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct khci_ep_s *privep = (struct khci_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPCANCEL, USB_EPNO(ep->eplog));

  flags = enter_critical_section();
  khci_cancelrequests(privep, -EAGAIN);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: khci_epbdtstall
 ****************************************************************************/

static int khci_epbdtstall(struct usbdev_ep_s *ep, bool resume, bool epin)
{
  struct khci_ep_s *privep;
  struct khci_usbdev_s *priv;
  volatile struct usbotg_bdtentry_s *bdt;
  volatile struct usbotg_bdtentry_s *otherbdt;
  uint32_t regaddr;
  uint32_t regval;
  uint8_t epno;

  /* Recover pointers */

  privep = (struct khci_ep_s *)ep;
  priv   = (struct khci_usbdev_s *)privep->dev;
  epno   = USB_EPNO(ep->eplog);

  /* Check for an IN endpoint */

  if (epin)
    {
      /* Get a pointer to the current IN BDT */

      bdt = privep->bdtin;

      /* Get the other BDT */

      otherbdt = &g_bdt[EP(epno, EP_DIR_IN, EP_PP_EVEN)];
      if (otherbdt == bdt)
        {
          otherbdt++;
        }

      /* Reset the data toggle */

      privep->txdata1 = false;
    }

  /* Otherwise it is an an OUT endpoint. */

  else
    {
      /* Get a pointer to the current OUT BDT */

      bdt = privep->bdtout;

      /* Get a pointer to the other BDT */

      otherbdt = &g_bdt[EP(epno, EP_DIR_OUT, EP_PP_EVEN)];
      if (otherbdt == bdt)
        {
          otherbdt++;
        }

      /* Reset the data toggle */

      privep->rxdata1 = false;
    }

  /* Handle the resume condition */

  if (resume)
    {
      /* Resuming a stalled endpoint */

      usbtrace(TRACE_EPRESUME, epno);

      /* Point to the appropriate EP register */

      regaddr = KINETIS_USB0_ENDPT(epno);

      /* Clear the STALL bit in the UEP register */

      regval  = khci_getreg(regaddr);
      regval &= ~USB_ENDPT_EPSTALL;
      khci_putreg(regval, regaddr);

      /* Check for the EP0 OUT endpoint.  This is a special case because we
       * need to set it up to receive the next setup packet (Hmmm... what
       * if there are queued outgoing responses.  We need to revisit this.)
       */

      if (epno == 0 && !epin)
        {
          uint32_t bytecount = (CONFIG_USBDEV_EP0_MAXSIZE <<
                                USB_BDT_BYTECOUNT_SHIFT);

          /* Configure the other BDT to receive a SETUP command. */

          if (otherbdt == &g_bdt[EP0_OUT_EVEN])
            {
              otherbdt->addr = priv->out0data[EP0_OUT_EVEN];
            }
          else
            {
              otherbdt->addr = priv->out0data[EP0_OUT_ODD];
            }

          otherbdt->status   = (USB_BDT_UOWN | bytecount);

          /* Configure the current BDT to receive a SETUP command. */

          if (bdt == &g_bdt[EP0_OUT_EVEN])
            {
              bdt->addr = priv->out0data[EP0_OUT_EVEN];
            }
          else
            {
              bdt->addr = priv->out0data[EP0_OUT_ODD];
            }

          bdt->status        = (USB_BDT_UOWN | bytecount);

          bdtinfo("EP0 BDT IN [%p] {%08x, %08x}\n",
                  bdt, bdt->status, bdt->addr);
          bdtinfo("EP0 BDT IN [%p] {%08x, %08x}\n",
                  otherbdt, otherbdt->status, otherbdt->addr);
        }
      else
        {
          /* Return the other BDT to the CPU. */

          otherbdt->addr   = 0;
          otherbdt->status = 0;

          /* Return the current BDT to the CPU. */

          bdt->addr        = 0;
          bdt->status      = 0;

          bdtinfo("EP%d BDT %s [%p] {%08x, %08x}\n",
                  epno, epin ? "IN" : "OUT", bdt, bdt->status, bdt->addr);
          bdtinfo("EP%d BDT %s [%p] {%08x, %08x}\n",
                  epno, epin ? "IN" : "OUT", otherbdt,
                  otherbdt->status, otherbdt->addr);

          /* Restart any queued requests (after a delay so that we can be
           * assured that the hardware has recovered from the stall -- I
           * don't know of any other way to assure this.).
           */

          khci_delayedrestart(priv, epno);
        }
    }

  /* Handle the stall condition */

  else
    {
      usbtrace(TRACE_EPSTALL, epno);
      privep->stalled  = true;

      /* Stall the other BDT. */

      otherbdt->status = (USB_BDT_UOWN | USB_BDT_BSTALL);
      otherbdt->addr   = 0;

      /* Stall the current BDT. */

      bdt->status      = (USB_BDT_UOWN | USB_BDT_BSTALL);
      bdt->addr        = 0;

      /* Stop any queued requests.  Hmmm.. is there a race condition here? */

      khci_rqstop(privep);

      bdtinfo("EP%d BDT %s [%p] {%08x, %08x}\n",
              epno, epin ? "IN" : "OUT", bdt, bdt->status, bdt->addr);
      bdtinfo("EP%d BDT %s [%p] {%08x, %08x}\n",
              epno, epin ? "IN" : "OUT", otherbdt,
              otherbdt->status, otherbdt->addr);
    }

  return OK;
}

/****************************************************************************
 * Name: khci_epstall
 ****************************************************************************/

static int khci_epstall(struct usbdev_ep_s *ep, bool resume)
{
  struct khci_ep_s *privep;
  irqstate_t flags;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Recover pointers */

  privep = (struct khci_ep_s *)ep;

  /* STALL or RESUME the endpoint */

  flags = enter_critical_section();

  /* Special case EP0.  When we stall EP0 we have to stall both the IN and
   * OUT BDTs.
   */

  if (USB_EPNO(ep->eplog) == 0)
    {
      ret = khci_epbdtstall(ep, resume, true);
      if (ret == OK)
        {
          ret = khci_epbdtstall(ep, resume, false);
        }

      /* Set the EP0 control state appropriately */

      privep->dev->ctrlstate = resume ?
                               CTRLSTATE_WAITSETUP : CTRLSTATE_STALLED;
    }

  /* Otherwise, select the BDT for the endpoint direction */

  else
    {
      /* It is a unidirectional endpoint */

      ret = khci_epbdtstall(ep, resume, USB_ISEPIN(ep->eplog));
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Device Controller Operations
 ****************************************************************************/

/****************************************************************************
 * Name: khci_allocep
 ****************************************************************************/

static struct usbdev_ep_s *khci_allocep(struct usbdev_s *dev, uint8_t epno,
                                           bool epin, uint8_t eptype)
{
  struct khci_usbdev_s *priv = (struct khci_usbdev_s *)dev;
  struct khci_ep_s *privep = NULL;
  uint16_t epset = KHCI_ENDP_ALLSET;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)epno);
#ifdef CONFIG_DEBUG_FEATURES
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDPARMS), 0);
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
       * First, verify that the logical endpoint is in the range supported
       * by by the hardware.
       */

      if (epno >= KHCI_NENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_BADEPNO), (uint16_t)epno);
          return NULL;
        }

      /* Convert the logical address to a physical OUT endpoint address and
       * remove all of the candidate endpoints from the bitset except for the
       * the IN/OUT pair for this logical address.
       */

      epset = KHCI_ENDP_BIT(epno);
    }

  /* Check if the selected endpoint number is available */

  privep = khci_epreserve(priv, epset);
  if (!privep)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_EPRESERVE), (uint16_t)epset);
      return NULL;
    }

  return &privep->ep;
}

/****************************************************************************
 * Name: khci_freeep
 ****************************************************************************/

static void khci_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep)
{
  struct khci_usbdev_s *priv;
  struct khci_ep_s *privep;

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev || !ep)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  priv   = (struct khci_usbdev_s *)dev;
  privep = (struct khci_ep_s *)ep;
  usbtrace(TRACE_DEVFREEEP, (uint16_t)USB_EPNO(ep->eplog));
  DEBUGASSERT(priv && privep);

  /* Disable the endpoint */

  khci_epdisable(ep);

  /* Mark the endpoint as available */

  khci_epunreserve(priv, privep);
}

/****************************************************************************
 * Name: khci_getframe
 ****************************************************************************/

static int khci_getframe(struct usbdev_s *dev)
{
  uint16_t frml;
  uint16_t frmh;
  uint16_t tmp;

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Return the last frame number detected by the hardware.  Thr FRMH/L
   * registers are updated with the current frame number whenever a SOF
   * TOKEN is received.
   */

  do
    {
      /* Loop until we can be sure that there was no wrap from the FRML
       * to the FRMH register.
       */

      frmh = khci_getreg(KINETIS_USB0_FRMNUMH) & USB_FRMNUMH_MASK;
      frml = khci_getreg(KINETIS_USB0_FRMNUML) & USB_FRMNUML_MASK;
      tmp  = khci_getreg(KINETIS_USB0_FRMNUMH) & USB_FRMNUMH_MASK;
    }
  while (frmh != tmp);

  /* Combine to for the full 11-bit value */

  tmp = (frmh) << 8 | frml;
  usbtrace(TRACE_DEVGETFRAME, tmp);
  return tmp;
}

/****************************************************************************
 * Name: khci_wakeup
 ****************************************************************************/

static int khci_wakeup(struct usbdev_s *dev)
{
  struct khci_usbdev_s *priv = (struct khci_usbdev_s *)dev;

  usbtrace(TRACE_DEVWAKEUP, 0);
#ifdef CONFIG_DEBUG_FEATURES
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Resume normal operation. */

  khci_remote_resume(priv);
  return OK;
}

/****************************************************************************
 * Name: khci_selfpowered
 ****************************************************************************/

static int khci_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct khci_usbdev_s *priv = (struct khci_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/****************************************************************************
 * Initialization/Reset
 ****************************************************************************/

/****************************************************************************
 * Name: khci_reset
 *
 * Description:
 *   Reset the software and hardware states.  At the end of this reset, the
 *   hardware should be in the full up, ready-to-run state.
 *
 ****************************************************************************/

static void khci_reset(struct khci_usbdev_s *priv)
{
  /* Reset the software configuration */

  khci_swreset(priv);

  /* Re-configure the USB controller in its initial, unconnected state */

  khci_hwreset(priv);

  /* Do the final hw attach */

  khci_attach(priv);
}

/****************************************************************************
 * Name: khci_attach
 ****************************************************************************/

static void khci_attach(struct khci_usbdev_s *priv)
{
  uint32_t regval;

  /* Check if we are in the detached state */

  if (priv->devstate == DEVSTATE_DETACHED)
    {
      /* Disable USB interrupts at the interrupt controller */

      up_disable_irq(KINETIS_IRQ_USBOTG);

      /* Initialize the controller to known states. */

      khci_putreg(USB_CTL_USBENSOFEN, KINETIS_USB0_CTL);

      /* Configure things like: pull ups, full/low-speed mode,
       * set the ping pong mode, and set internal transceiver
       */

      khci_putreg(0, KINETIS_USB0_USBCTRL);

      /* Enable interrupts at the USB controller */

      khci_putreg(ERROR_INTERRUPTS, KINETIS_USB0_ERREN);
      khci_putreg(NORMAL_INTERRUPTS, KINETIS_USB0_INTEN);

      /* Flush any pending transactions */

      while ((khci_getreg(KINETIS_USB0_ISTAT) & USB_INT_TOKDNE) != 0)
        {
          khci_putreg(USB_INT_TOKDNE, KINETIS_USB0_ISTAT);
        }

      /* Make sure packet processing is enabled */

      regval = khci_getreg(KINETIS_USB0_CTL);
      regval &= ~USB_CTL_TXSUSPENDTOKENBUSY;
      khci_putreg(regval, KINETIS_USB0_CTL);

      /* Enable the USB module and attach to bus */

      do
        {
          regval = khci_getreg(KINETIS_USB0_CTL);
          if ((regval & USB_CTL_USBENSOFEN) == 0)
            {
              khci_putreg(regval | USB_CTL_USBENSOFEN, KINETIS_USB0_CTL);
            }
        }
      while ((regval & USB_CTL_USBENSOFEN) == 0);

      /* Enable OTG */

#ifdef CONFIG_KINETIS_USBOTG
      regval  = khci_getreg(KINETIS_USB0_OTGCTL);
      regval |= (USB_OTGCTL_DPHIGH | USB_OTGCTL_OTGEN);
      khci_putreg(regval, KINETIS_USB0_OTGCTL);
#endif

      /* Configure EP0 */

      khci_ep0configure(priv);

      /* Transition to the attached state */

      priv->devstate     = DEVSTATE_ATTACHED;
      priv->usbdev.speed = USB_SPEED_FULL;

      /* Clear all pending USB interrupts */

      khci_putreg(USB_EINT_ALL, KINETIS_USB0_ERRSTAT);
      khci_putreg(USB_INT_ALL, KINETIS_USB0_ISTAT);

      kinetis_clrpend(KINETIS_IRQ_USBOTG);

      /* Enable USB interrupts at the interrupt controller */

      up_enable_irq(KINETIS_IRQ_USBOTG);
    }
}

/****************************************************************************
 * Name: khci_swreset
 ****************************************************************************/

static void khci_swreset(struct khci_usbdev_s *priv)
{
  int epno;

  /* Tell the class driver that we are disconnected.  The class driver
   * should then accept any new configurations.
   */

  if (priv->driver)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }

  /* Flush and reset endpoint states */

  for (epno = 0; epno < KHCI_NENDPOINTS; epno++)
    {
      struct khci_ep_s *privep = &priv->eplist[epno];

      /* Cancel any queued requests.  Since they are canceled
       * with status -ESHUTDOWN, then will not be requeued
       * until the configuration is reset.  NOTE:  This should
       * not be necessary... the CLASS_DISCONNECT above should
       * result in the class implementation calling khci_epdisable
       * for each of its configured endpoints.
       */

      khci_cancelrequests(privep, -ESHUTDOWN);

      /* Reset endpoint status */

      privep->stalled   = false;
      privep->halted    = false;
      privep->txnullpkt = false;
    }

  priv->devstate = DEVSTATE_DETACHED;

  /* Reset the control state */

  priv->ctrlstate = CTRLSTATE_WAITSETUP;
  priv->rxbusy    = 0;
}

/****************************************************************************
 * Name: khci_hwreset
 *
 * Description:
 *   Reset the hardware and leave it in a known, unready state.
 *
 ****************************************************************************/

static void khci_hwreset(struct khci_usbdev_s *priv)
{
  int epno;
  uint32_t regval;

  /* When bus reset is received during suspend, ensure we resume */

  if ((khci_getreg(KINETIS_USB0_USBCTRL) & USB_USBCTRL_SUSP) != 0)
    {
      khci_resume(priv);
    }

  /* Unconfigure each endpoint by clearing the endpoint control registers */

  for (epno = 0; epno < KHCI_NENDPOINTS; epno++)
    {
      khci_putreg(KHCI_EP_DISABLED, KINETIS_USB0_ENDPT(epno));
    }

  /* Reset the address */

  khci_putreg(0, KINETIS_USB0_ADDR);

  /* Assert reset request to all of the Ping Pong buffer pointers.  This
   * will reset all Even/Odd buffer pointers to the EVEN BD banks.
   */

  regval  = khci_getreg(KINETIS_USB0_CTL);
  regval |= USB_CTL_ODDRST;
  khci_putreg(regval, KINETIS_USB0_CTL);

  /* Bring the ping pong buffer pointers out of reset */

  regval &= ~USB_CTL_ODDRST;
  khci_putreg(regval, KINETIS_USB0_CTL);

  /* Enable interrupts at the USB controller */

  khci_putreg(ERROR_INTERRUPTS, KINETIS_USB0_ERREN);
  khci_putreg(NORMAL_INTERRUPTS, KINETIS_USB0_INTEN);
}

/****************************************************************************
 * Name: khci_hwinitialize
 *
 * Description:
 *   Reset the hardware and leave it in a known, unready state.
 *
 ****************************************************************************/

static void khci_hwinitialize(struct khci_usbdev_s *priv)
{
  uint32_t regval;

  /* Initialize registers to known states. */

  /* Reset USB Module */

  regval = khci_getreg(KINETIS_USB0_USBTRC0);
  regval |= USB_USBTRC0_USBRESET;
  khci_putreg(regval, KINETIS_USB0_USBTRC0);

  /* NOTE: This bit is always read as zero. Wait two
   * USB clock cycles after setting this bit.That is ~42 Ns
   */

  /* Clear all of the buffer descriptor table (BDT) entries */

  memset((void *)g_bdt, 0, sizeof(g_bdt));

  /* Enable the USB-FS to operate */

  khci_putreg(0, KINETIS_USB0_CTL);
  up_udelay(2);
  khci_putreg(USB_CTL_USBENSOFEN, KINETIS_USB0_CTL);

  /* Set the address of the buffer descriptor table (BDT)
   *
   * BDTP1: Bit 1-7: Bits 9-15 of the BDT base address
   * BDTP2: Bit 0-7: Bits 16-23 of the BDT base address
   * BDTP3: Bit 0-7: Bits 24-31 of the BDT base address
   */

  khci_putreg((uint8_t)((uint32_t)g_bdt >> 24), KINETIS_USB0_BDTPAGE3);
  khci_putreg((uint8_t)((uint32_t)g_bdt >> 16), KINETIS_USB0_BDTPAGE2);
  khci_putreg((uint8_t)(((uint32_t)g_bdt >> 8) & USB_BDTPAGE1_MASK),
      KINETIS_USB0_BDTPAGE1);

  uinfo("BDT Address %p\n", (const void *)&g_bdt);
  uinfo("BDTPAGE3 %hhx\n", khci_getreg(KINETIS_USB0_BDTPAGE3));
  uinfo("BDTPAGE2 %hhx\n", khci_getreg(KINETIS_USB0_BDTPAGE2));
  uinfo("BDTPAGE1 %hhx\n", khci_getreg(KINETIS_USB0_BDTPAGE1));

#if defined(USB0_USBTRC0_BIT6)
  /* Undocumented bit */

  regval = khci_getreg(KINETIS_USB0_USBTRC0);
  regval |= USB0_USBTRC0_BIT6;
  khci_putreg(regval, KINETIS_USB0_USBTRC0);
#endif
}

/****************************************************************************
 * Name: khci_swinitialize
 ****************************************************************************/

static void khci_swinitialize(struct khci_usbdev_s *priv)
{
  int epno;

  /* Initialize the device state structure.  NOTE: many fields
   * have the initial value of zero and, hence, are not explicitly
   * initialized here.
   */

  memset(priv, 0, sizeof(struct khci_usbdev_s));
  priv->usbdev.ops   = &g_devops;
  priv->usbdev.ep0   = &priv->eplist[EP0].ep;
  priv->usbdev.speed = USB_SPEED_UNKNOWN;
  priv->epavail      = KHCI_ENDP_ALLSET & ~KHCI_ENDP_BIT(EP0);
  priv->rwakeup      = 1;

  /* Initialize the endpoint list */

  for (epno = 0; epno < KHCI_NENDPOINTS; epno++)
    {
      struct khci_ep_s *privep = &priv->eplist[epno];

      /* Set endpoint operations, reference to driver structure (not
       * really necessary because there is only one controller), and
       * the (physical) endpoint number which is just the index to the
       * endpoint.
       */

      privep->ep.ops           = &g_epops;
      privep->dev              = priv;
      privep->ep.eplog         = epno;

      /* We will use a fixed maxpacket size for all endpoints (perhaps
       * ISOC endpoints could have larger maxpacket???).  A smaller
       * packet size can be selected when the endpoint is configured.
       */

      privep->ep.maxpacket     = KHCI_MAXPACKET_SIZE;
    }

  /* Select a smaller endpoint size for EP0 */

#if KHCI_EP0MAXPACKET < KHCI_MAXPACKET_SIZE
  priv->eplist[EP0].ep.maxpacket = KHCI_EP0MAXPACKET;
#endif
}

/****************************************************************************
 * Name: khci_hwshutdown
 ****************************************************************************/

static void khci_hwshutdown(struct khci_usbdev_s *priv)
{
  /* Disable all interrupts and force the USB controller into reset */

  khci_putreg(0, KINETIS_USB0_ERREN);
  khci_putreg(0, KINETIS_USB0_INTEN);

  /* Clear any pending interrupts */

  khci_putreg(USB_EINT_ALL, KINETIS_USB0_ERRSTAT);
  khci_putreg(USB_INT_ALL, KINETIS_USB0_ISTAT);

  kinetis_clrpend(KINETIS_IRQ_USBOTG);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_usbinitialize
 *
 * Description:
 *   Initialize the USB driver
 *
 * Assumptions:
 * - This function is called very early in the initialization sequence
 * - PLL and GIO pin initialization is not performed here but should been in
 *   the low-level  boot logic: SIM_SOPT2[PLLFLLSEL] and
 *   SIM_CLKDIV2[USBFRAC, USBDIV] will have been configured in
 *   kinetis_pllconfig.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_usbinitialize(void)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct khci_usbdev_s *priv = &g_usbdev;
  uint32_t regval;

  usbtrace(TRACE_DEVINIT, 0);

  /* Initialize the driver state structure */

  khci_swinitialize(priv);

  /* Select clock source:
   * SIM_SOPT2[PLLFLLSEL] and SIM_CLKDIV2[USBFRAC, USBDIV] will have been
   * configured in kinetis_pllconfig. So here we select between USB_CLKIN
   * or the output of SIM_CLKDIV2[USBFRAC, USBDIV]
   */

  /* 1: Select USB clock */

  regval = getreg32(KINETIS_SIM_SOPT2);
  regval &= ~(SIM_SOPT2_USBSRC);
  regval |= BOARD_USB_CLKSRC;
  putreg32(regval, KINETIS_SIM_SOPT2);

  /* 2: Gate USB clock */

  regval = getreg32(KINETIS_SIM_SCGC4);
  regval |= SIM_SCGC4_USBOTG;
  putreg32(regval, KINETIS_SIM_SCGC4);

#if defined(BOARD_USB_FLASHACCESS)
  /* Allow USBOTG-FS Controller to Read from FLASH */

  regval = getreg32(KINETIS_FMC_PFAPR);
  regval &= ~(FMC_PFAPR_M4AP_MASK);
  regval |= (FMC_PFAPR_RDONLY << FMC_PFAPR_M4AP_SHIFT);
  putreg32(regval, KINETIS_FMC_PFAPR);
#endif

  khci_swreset(priv);

  /* Attach USB controller interrupt handler.  The hardware will not be
   * initialized and interrupts will not be enabled until the class device
   * driver is bound.  Getting the IRQs here only makes sure that we have
   * them when we need them later.
   */

  if (irq_attach(KINETIS_IRQ_USBOTG, khci_interrupt, priv) != 0)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_IRQREGISTRATION),
               (uint16_t)KINETIS_IRQ_USBOTG);
      arm_usbuninitialize();
      return;
    }

  khci_hwinitialize(priv);
}

/****************************************************************************
 * Name: arm_usbuninitialize
 * Description:
 *   Initialize the USB driver
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_usbuninitialize(void)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct khci_usbdev_s *priv = &g_usbdev;
  irqstate_t flags;
  uint32_t regval;

  usbtrace(TRACE_DEVUNINIT, 0);

  /* Disconnect the device */

  flags = enter_critical_section();

  khci_swreset(priv);

  kinetis_usbpullup(&priv->usbdev, false);

  wd_cancel(&priv->wdog);

  /* Put the hardware in an inactive state */

  khci_hwreset(priv);
  khci_hwshutdown(priv);

  /* Disable and detach the USB IRQs */

  up_disable_irq(KINETIS_IRQ_USBOTG);
  irq_detach(KINETIS_IRQ_USBOTG);

  /* Gate Off the USB controller */

  regval = getreg32(KINETIS_SIM_SCGC4);
  regval &= ~SIM_SCGC4_USBOTG;
  putreg32(regval, KINETIS_SIM_SCGC4);

  leave_critical_section(flags);
}

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

  struct khci_usbdev_s *priv = &g_usbdev;
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->disconnect || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  priv->driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &priv->usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_BINDFAILED), (uint16_t)-ret);
      priv->driver = NULL;
    }

  /* The class driver has been successfully bound. */

  else
    {
      /* Setup the USB controller in it initial ready-to-run state */

      DEBUGASSERT(priv->devstate == DEVSTATE_DETACHED);
      khci_reset(priv);
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

  struct khci_usbdev_s *priv = &g_usbdev;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (driver != priv->driver)
    {
      usbtrace(TRACE_DEVERROR(KHCI_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Reset the hardware and cancel all requests.  All requests must be
   * canceled while the class driver is still bound.  This will put
   * the hardware back into its initial, unconnected state.
   */

  flags = enter_critical_section();
  khci_swreset(priv);
  kinetis_usbpullup(&priv->usbdev, false);
  khci_hwreset(priv);

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable USB controller interrupts (but keep them attached) */

  up_disable_irq(KINETIS_IRQ_USBOTG);

  /* Put the hardware in an inactive state.  Then bring the hardware back up
   * in the reset state (this is probably not necessary, the khci_hwreset()
   * call above was probably sufficient).
   */

  khci_hwshutdown(priv);
  khci_swinitialize(priv);

  leave_critical_section(flags);
  return OK;
}
#endif /* CONFIG_USBDEV */

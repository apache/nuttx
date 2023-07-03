/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_otg_device.c
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

#include <sys/param.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "xtensa.h"
#include "hardware/esp32s3_otg.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "hardware/esp32s3_usb_wrap.h"
#include "hardware/esp32s3_rtccntl.h"
#include "hardware/esp32s3_pinmap.h"
#include "esp32s3_gpio.h"
#include "esp32s3_irq.h"
#include "esp32s3_otg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* USB hardware definition */

#define ESP32S3_FIFO_SIZE       (1280)
#define ESP32S3_FIFO_WORDS      (ESP32S3_FIFO_SIZE >> 2)

/* EP0 + additional EP */

#define ESP32S3_ENDPOINT_NUM    (CONFIG_ESP32S3_OTG_ENDPOINT_NUM + 1)

/* There is 1.25Kb of FIFO memory.  The default partitions this memory
 * so that there is a TxFIFO allocated for each endpoint and with more
 * memory provided for the common RxFIFO.  A more knowledge-able
 * configuration would not allocate any TxFIFO space to OUT endpoints.
 */

#define ESP32S3_TXFIFO_SIZE \
  (ESP32S3_FIFO_SIZE / 2 / ESP32S3_ENDPOINT_NUM * ESP32S3_ENDPOINT_NUM)

#define ESP32S3_EP_TXFIFO_SIZE \
  (ESP32S3_TXFIFO_SIZE / ESP32S3_ENDPOINT_NUM)

/* EP0 must exist */

#ifdef CONFIG_USBDEV_EP0_TXFIFO_SIZE
#  define ESP32S3_EP0_TXFIFO_SIZE CONFIG_USBDEV_EP0_TXFIFO_SIZE
#else
#  define ESP32S3_EP0_TXFIFO_SIZE ESP32S3_EP_TXFIFO_SIZE
#endif

#if ESP32S3_ENDPOINT_NUM > 1
#  ifdef CONFIG_USBDEV_EP1_TXFIFO_SIZE
#    define ESP32S3_EP1_TXFIFO_SIZE CONFIG_USBDEV_EP1_TXFIFO_SIZE
#  else
#    define ESP32S3_EP1_TXFIFO_SIZE ESP32S3_EP_TXFIFO_SIZE
#  endif
#else
#  define ESP32S3_EP1_TXFIFO_SIZE 0
#endif

#if ESP32S3_ENDPOINT_NUM > 2
#  ifdef CONFIG_USBDEV_EP2_TXFIFO_SIZE
#    define ESP32S3_EP2_TXFIFO_SIZE CONFIG_USBDEV_EP2_TXFIFO_SIZE
#  else
#    define ESP32S3_EP2_TXFIFO_SIZE ESP32S3_EP_TXFIFO_SIZE
#  endif
#else
#  define ESP32S3_EP2_TXFIFO_SIZE 0
#endif

#if ESP32S3_ENDPOINT_NUM > 3
#  ifdef CONFIG_USBDEV_EP3_TXFIFO_SIZE
#    define ESP32S3_EP3_TXFIFO_SIZE CONFIG_USBDEV_EP3_TXFIFO_SIZE
#  else
#    define ESP32S3_EP3_TXFIFO_SIZE ESP32S3_EP_TXFIFO_SIZE
#  endif
#else
#  define ESP32S3_EP3_TXFIFO_SIZE 0
#endif

#if ESP32S3_ENDPOINT_NUM > 4
#  ifdef CONFIG_USBDEV_EP4_TXFIFO_SIZE
#    define ESP32S3_EP4_TXFIFO_SIZE CONFIG_USBDEV_EP4_TXFIFO_SIZE
#  else
#    define ESP32S3_EP4_TXFIFO_SIZE ESP32S3_EP_TXFIFO_SIZE
#  endif
#else
#  define ESP32S3_EP4_TXFIFO_SIZE 0
#endif

#if ESP32S3_ENDPOINT_NUM > 5
#  ifdef CONFIG_USBDEV_EP5_TXFIFO_SIZE
#    define ESP32S3_EP5_TXFIFO_SIZE CONFIG_USBDEV_EP5_TXFIFO_SIZE
#  else
#    define ESP32S3_EP5_TXFIFO_SIZE ESP32S3_EP_TXFIFO_SIZE
#  endif
#else
#  define ESP32S3_EP5_TXFIFO_SIZE 0
#endif

#if ESP32S3_ENDPOINT_NUM > 6
#  ifdef CONFIG_USBDEV_EP6_TXFIFO_SIZE
#    define ESP32S3_EP6_TXFIFO_SIZE CONFIG_USBDEV_EP6_TXFIFO_SIZE
#  else
#    define ESP32S3_EP6_TXFIFO_SIZE ESP32S3_EP_TXFIFO_SIZE
#  endif
#else
#  define ESP32S3_EP6_TXFIFO_SIZE 0
#endif

/* The actual FIFO addresses that we use must be aligned to 4-byte
 * boundaries;
 * FIFO sizes must be provided in units of 32-bit words.
 */

#define ALIGN(v, a) (((v) + (a - 1)) & (~(a - 1)))

#define ESP32S3_EP0_TXFIFO_WORDS (ALIGN(ESP32S3_EP0_TXFIFO_SIZE, 4) >> 2)
#define ESP32S3_EP1_TXFIFO_WORDS (ALIGN(ESP32S3_EP1_TXFIFO_SIZE, 4) >> 2)
#define ESP32S3_EP2_TXFIFO_WORDS (ALIGN(ESP32S3_EP2_TXFIFO_SIZE, 4) >> 2)
#define ESP32S3_EP3_TXFIFO_WORDS (ALIGN(ESP32S3_EP3_TXFIFO_SIZE, 4) >> 2)
#define ESP32S3_EP4_TXFIFO_WORDS (ALIGN(ESP32S3_EP4_TXFIFO_SIZE, 4) >> 2)
#define ESP32S3_EP5_TXFIFO_WORDS (ALIGN(ESP32S3_EP5_TXFIFO_SIZE, 4) >> 2)
#define ESP32S3_EP6_TXFIFO_WORDS (ALIGN(ESP32S3_EP6_TXFIFO_SIZE, 4) >> 2)

#define ESP32S3_RXFIFO_WORDS  \
  (ESP32S3_FIFO_WORDS       - \
   ESP32S3_EP0_TXFIFO_WORDS - \
   ESP32S3_EP1_TXFIFO_WORDS - \
   ESP32S3_EP2_TXFIFO_WORDS - \
   ESP32S3_EP3_TXFIFO_WORDS - \
   ESP32S3_EP4_TXFIFO_WORDS - \
   ESP32S3_EP5_TXFIFO_WORDS - \
   ESP32S3_EP6_TXFIFO_WORDS)

#if (ESP32S3_RXFIFO_WORDS * 4) < 64
#  error "ESP32-S3 OTG device RX FIFO size should >= 64"
#endif

/* EP0 max packet size */

#define ESP32S3_CDCACM_EP0MAXPACKET CONFIG_CDCACM_EP0MAXPACKET

#define OTG_GINT_RESERVED_FS   (OTG_GINT_RES89    | \
                                OTG_GINT_RES1617  | \
                                OTG_GINT_RES22)

#define OTG_GINT_RC_W1_FS      (OTG_GINT_MMIS     | \
                                OTG_GINT_SOF      | \
                                OTG_GINT_ESUSP    | \
                                OTG_GINT_USBSUSP  | \
                                OTG_GINT_USBRST   | \
                                OTG_GINT_ENUMDNE  | \
                                OTG_GINT_ISOODRP  | \
                                OTG_GINT_EOPF     | \
                                OTG_GINT_IISOIXFR | \
                                OTG_GINT_IISOOXFR | \
                                OTG_GINT_RSTDET   | \
                                OTG_GINT_LPMINT   | \
                                OTG_GINT_CIDSCHG  | \
                                OTG_GINT_DISC     | \
                                OTG_GINT_SRQ      | \
                                OTG_GINT_WKUP)

#define OTG_GINT_RESERVED       OTG_GINT_RESERVED_FS
#define OTG_GINT_RC_W1          OTG_GINT_RC_W1_FS

/* Debug ********************************************************************/

/* Trace error codes */

#define ESP32S3_TRACEERR_ALLOCFAIL          0x01
#define ESP32S3_TRACEERR_BADCLEARFEATURE    0x02
#define ESP32S3_TRACEERR_BADDEVGETSTATUS    0x03
#define ESP32S3_TRACEERR_BADEPNO            0x04
#define ESP32S3_TRACEERR_BADEPGETSTATUS     0x05
#define ESP32S3_TRACEERR_BADGETCONFIG       0x06
#define ESP32S3_TRACEERR_BADGETSETDESC      0x07
#define ESP32S3_TRACEERR_BADGETSTATUS       0x08
#define ESP32S3_TRACEERR_BADSETADDRESS      0x09
#define ESP32S3_TRACEERR_BADSETCONFIG       0x0a
#define ESP32S3_TRACEERR_BADSETFEATURE      0x0b
#define ESP32S3_TRACEERR_BADTESTMODE        0x0c
#define ESP32S3_TRACEERR_BINDFAILED         0x0d
#define ESP32S3_TRACEERR_DISPATCHSTALL      0x0e
#define ESP32S3_TRACEERR_DRIVER             0x0f
#define ESP32S3_TRACEERR_DRIVERREGISTERED   0x10
#define ESP32S3_TRACEERR_EP0NOSETUP         0x11
#define ESP32S3_TRACEERR_EP0SETUPSTALLED    0x12
#define ESP32S3_TRACEERR_EPINNULLPACKET     0x13
#define ESP32S3_TRACEERR_EPINUNEXPECTED     0x14
#define ESP32S3_TRACEERR_EPOUTNULLPACKET    0x15
#define ESP32S3_TRACEERR_EPOUTUNEXPECTED    0x16
#define ESP32S3_TRACEERR_INVALIDCTRLREQ     0x17
#define ESP32S3_TRACEERR_INVALIDPARMS       0x18
#define ESP32S3_TRACEERR_IRQREGISTRATION    0x19
#define ESP32S3_TRACEERR_NOEP               0x1a
#define ESP32S3_TRACEERR_NOTCONFIGURED      0x1b
#define ESP32S3_TRACEERR_EPOUTQEMPTY        0x1c
#define ESP32S3_TRACEERR_EPINREQEMPTY       0x1d
#define ESP32S3_TRACEERR_NOOUTSETUP         0x1e
#define ESP32S3_TRACEERR_POLLTIMEOUT        0x1f

/* Trace interrupt codes */

#define ESP32S3_TRACEINTID_USB              1 /* USB Interrupt entry/exit */
#define ESP32S3_TRACEINTID_INTPENDING       2 /* On each pass through the loop */

#define ESP32S3_TRACEINTID_EPOUT            (10 + 0)  /* First level interrupt decode */
#define ESP32S3_TRACEINTID_EPIN             (10 + 1)
#define ESP32S3_TRACEINTID_MISMATCH         (10 + 2)
#define ESP32S3_TRACEINTID_WAKEUP           (10 + 3)
#define ESP32S3_TRACEINTID_SUSPEND          (10 + 4)
#define ESP32S3_TRACEINTID_SOF              (10 + 5)
#define ESP32S3_TRACEINTID_RXFIFO           (10 + 6)
#define ESP32S3_TRACEINTID_DEVRESET         (10 + 7)
#define ESP32S3_TRACEINTID_ENUMDNE          (10 + 8)
#define ESP32S3_TRACEINTID_IISOIXFR         (10 + 9)
#define ESP32S3_TRACEINTID_IISOOXFR         (10 + 10)
#define ESP32S3_TRACEINTID_SRQ              (10 + 11)
#define ESP32S3_TRACEINTID_OTG              (10 + 12)

#define ESP32S3_TRACEINTID_EPOUT_XFRC       (40 + 0)  /* EPOUT second level decode */
#define ESP32S3_TRACEINTID_EPOUT_EPDISD     (40 + 1)
#define ESP32S3_TRACEINTID_EPOUT_SETUP      (40 + 2)
#define ESP32S3_TRACEINTID_DISPATCH         (40 + 3)

#define ESP32S3_TRACEINTID_GETSTATUS        (50 + 0)  /* EPOUT third level decode */
#define ESP32S3_TRACEINTID_EPGETSTATUS      (50 + 1)
#define ESP32S3_TRACEINTID_DEVGETSTATUS     (50 + 2)
#define ESP32S3_TRACEINTID_IFGETSTATUS      (50 + 3)
#define ESP32S3_TRACEINTID_CLEARFEATURE     (50 + 4)
#define ESP32S3_TRACEINTID_SETFEATURE       (50 + 5)
#define ESP32S3_TRACEINTID_SETADDRESS       (50 + 6)
#define ESP32S3_TRACEINTID_GETSETDESC       (50 + 7)
#define ESP32S3_TRACEINTID_GETCONFIG        (50 + 8)
#define ESP32S3_TRACEINTID_SETCONFIG        (50 + 9)
#define ESP32S3_TRACEINTID_GETSETIF         (50 + 10)
#define ESP32S3_TRACEINTID_SYNCHFRAME       (50 + 11)

#define ESP32S3_TRACEINTID_EPIN_XFRC        (70 + 0)  /* EPIN second level decode */
#define ESP32S3_TRACEINTID_EPIN_TOC         (70 + 1)
#define ESP32S3_TRACEINTID_EPIN_ITTXFE      (70 + 2)
#define ESP32S3_TRACEINTID_EPIN_EPDISD      (70 + 3)
#define ESP32S3_TRACEINTID_EPIN_TXFE        (70 + 4)

#define ESP32S3_TRACEINTID_EPIN_EMPWAIT     (80 + 0)  /* EPIN second level decode */

#define ESP32S3_TRACEINTID_OUTNAK           (90 + 0)  /* RXFLVL second level decode */
#define ESP32S3_TRACEINTID_OUTRECVD         (90 + 1)
#define ESP32S3_TRACEINTID_OUTDONE          (90 + 2)
#define ESP32S3_TRACEINTID_SETUPDONE        (90 + 3)
#define ESP32S3_TRACEINTID_SETUPRECVD       (90 + 4)

/* Endpoints ****************************************************************/

/* Odd physical endpoint numbers are IN; even are OUT */

#define ESP32S3_EPPHYIN2LOG(epphy)    ((uint8_t)(epphy) | USB_DIR_IN)
#define ESP32S3_EPPHYOUT2LOG(epphy)   ((uint8_t)(epphy) | USB_DIR_OUT)

/* Endpoint 0 */

#define EP0                           (0)

/* The set of all endpoints available to the class implementation (1-3) */

#define ESP32S3_EP_AVAILABLE          (0xfe)   /* All available endpoints */

/* Delays *******************************************************************/

#define ESP32S3_READY_DELAY           200000
#define ESP32S3_FLUSH_DELAY           200000

/* Request queue operations *************************************************/

#define esp32s3_rqempty(ep)           ((ep)->head == NULL)
#define esp32s3_rqpeek(ep)            ((ep)->head)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Overall device state */

enum esp32s3_devstate_e
{
  DEVSTATE_DEFAULT = 0,       /* Power-up, unconfigured state.  This state
                               * simply means that the device is not yet been
                               * given an address.  SET: At initialization,
                               * uninitialization, reset, and whenever the
                               * device address is set to zero TESTED: Never */
  DEVSTATE_ADDRESSED,         /* Device address has been assigned, not no
                               * configuration has yet been selected.  SET:
                               * When either a non-zero device address is
                               * first assigned or when the device is
                               * unconfigured (with configuration == 0)
                               * TESTED: never */
  DEVSTATE_CONFIGURED,        /* Address assigned and configured: SET: When
                               * the device has been addressed and an
                               * non-zero configuration has been selected.
                               * TESTED: In many places to assure that the
                               * USB device has been properly configured by
                               * the host. */
};

/* Endpoint 0 states */

enum esp32s3_ep0state_e
{
  EP0STATE_IDLE = 0,          /* Idle State, leave on receiving a SETUP
                               * packet or epsubmit: SET: In esp32s3_epin() and
                               * esp32s3_epout() when we revert from request
                               * processing to SETUP processing.  TESTED:
                               * Never */
  EP0STATE_SETUP_OUT,         /* OUT SETUP packet received.  Waiting for the
                               * DATA OUT phase of SETUP Packet to complete
                               * before processing a SETUP command (without a
                               * USB request): SET: Set in
                               * esp32s3_rxinterrupt() when SETUP OUT packet is
                               * received.  TESTED: In esp32s3_ep0out_receive() */
  EP0STATE_SETUP_READY,       /* IN SETUP packet received -OR- OUT SETUP
                               * packet and accompanying data have been
                               * received.  Processing of SETUP command will
                               * happen soon.  SET: (1)
                               * esp32s3_ep0out_receive() when the OUT SETUP
                               * data phase completes, or (2)
                               * esp32s3_rxinterrupt() when an IN SETUP is
                               * packet received.  TESTED: Tested in
                               * esp32s3_epout_interrupt() when SETUP phase is
                               * done to see if the SETUP command is ready to
                               * be processed.  Also tested in
                               * esp32s3_ep0out_setup() just to double-check
                               * that we have a SETUP request and any
                               * accompanying data. */
  EP0STATE_SETUP_PROCESS,     /* SETUP Packet is being processed by
                               * esp32s3_ep0out_setup(): SET: When SETUP packet
                               * received in EP0 OUT TESTED: Never */
  EP0STATE_SETUPRESPONSE,     /* Short SETUP response write (without a USB
                               * request): SET: When SETUP response is sent
                               * by esp32s3_ep0in_setupresponse() TESTED: Never */
  EP0STATE_DATA_IN,           /* Waiting for data out stage (with a USB
                               * request): SET: In esp32s3_epin_request() when
                               * a write request is processed on EP0.
                               * TESTED: In esp32s3_epin() to see if we should
                               * revert to SETUP processing. */
  EP0STATE_DATA_OUT           /* Waiting for data in phase to complete ( with
                               * a USB request) SET: In esp32s3_epout_request()
                               * when a read request is processed on EP0.
                               * TESTED: In esp32s3_epout() to see if we should
                               * revert to SETUP processing */
};

/* Parsed control request */

struct esp32s3_ctrlreq_s
{
  uint8_t type;
  uint8_t req;
  uint16_t value;
  uint16_t index;
  uint16_t len;
};

/* A container for a request so that the request may be retained in a list */

struct esp32s3_req_s
{
  struct usbdev_req_s req;      /* Standard USB request */
  struct esp32s3_req_s *flink;  /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct esp32s3_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct esp32s3_ep_s.
   */

  struct usbdev_ep_s ep;      /* Standard endpoint structure */

  /* ESP32-S3-specific fields */

  struct esp32s3_usbdev_s *dev; /* Reference to private driver data */
  struct esp32s3_req_s *head;   /* Request list for this endpoint */
  struct esp32s3_req_s *tail;
  uint8_t epphy;                /* Physical EP address */
  uint8_t eptype:2;             /* Endpoint type */
  uint8_t active:1;             /* 1: A request is being processed */
  uint8_t stalled:1;            /* 1: Endpoint is stalled */
  uint8_t isin:1;               /* 1: IN Endpoint */
  uint8_t odd:1;                /* 1: Odd frame */
  uint8_t zlp:1;                /* 1: Transmit a zero-length-packet (IN EPs only) */
};

/* This structure retains the state of the USB device controller */

struct esp32s3_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s to
   * struct esp32s3_usbdev_s.
   */

  struct usbdev_s usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* ESP32-S3-specific fields */

  int cpu;                    /* CPU ID */
  int cpuint;                 /* OTG interrupt ID */

  uint8_t stalled:1;          /* 1: Protocol stalled */
  uint8_t selfpowered:1;      /* 1: Device is self powered */
  uint8_t addressed:1;        /* 1: Peripheral address has been set */
  uint8_t configured:1;       /* 1: Class driver has been configured */
  uint8_t wakeup:1;           /* 1: Device remote wake-up */
  uint8_t dotest:1;           /* 1: Test mode selected */

  uint8_t devstate:4;         /* See enum esp32s3_devstate_e */
  uint8_t ep0state:4;         /* See enum esp32s3_ep0state_e */
  uint8_t testmode:4;         /* Selected test mode */
  uint8_t epavail[2];         /* Bitset of available OUT/IN endpoints */

  /* E0 SETUP data buffering.
   *
   * ctrlreq:
   *   The 8-byte SETUP request is received on the EP0 OUT endpoint and is
   *   saved.
   *
   * ep0data:
   *   For OUT SETUP requests, the SETUP data phase must also complete
   *   before the SETUP command can be processed.  The pack receipt logic
   *   will save the accompanying EP0 IN data in ep0data[] before the
   *   SETUP command is processed.
   *
   *   For IN SETUP requests, the DATA phase will occur AFTER the SETUP
   *   control request is processed.  In that case, ep0data[] may be used
   *   as the response buffer.
   *
   * ep0datlen:
   *   Length of OUT DATA received in ep0data[] (Not used with OUT data)
   */

  struct usb_ctrlreq_s ctrlreq;
  uint8_t ep0data[ESP32S3_CDCACM_EP0MAXPACKET];
  uint16_t ep0datlen;

  /* The endpoint lists */

  struct esp32s3_ep_s epin[ESP32S3_ENDPOINT_NUM];
  struct esp32s3_ep_s epout[ESP32S3_ENDPOINT_NUM];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_ESP32S3_OTG_DEBUG_REGISTER
static uint32_t esp32s3_getreg(uint32_t addr);
static void esp32s3_putreg(uint32_t val, uint32_t addr);
#else
#  define esp32s3_getreg(addr)     getreg32(addr)
#  define esp32s3_putreg(val,addr) putreg32(val,addr)
#endif

/* Request queue operations *************************************************/

static struct esp32s3_req_s *esp32s3_req_remfirst(struct esp32s3_ep_s
                                                  *privep);
static bool esp32s3_req_addlast(struct esp32s3_ep_s *privep,
                                struct esp32s3_req_s *req);

/* Low level data transfers and request operations **************************/

/* Special endpoint 0 data transfer logic */

static void esp32s3_ep0in_setupresponse(struct esp32s3_usbdev_s *priv,
                                        uint8_t * data, uint32_t nbytes);
static inline void esp32s3_ep0in_transmitzlp(struct esp32s3_usbdev_s *priv);
static void esp32s3_ep0in_activate(void);

static void esp32s3_ep0out_ctrlsetup(struct esp32s3_usbdev_s *priv);

/* IN request and TxFIFO handling */

static void esp32s3_txfifo_write(struct esp32s3_ep_s *privep,
                                 uint8_t *buf, int nbytes);
static void esp32s3_epin_transfer(struct esp32s3_ep_s *privep,
                                  uint8_t *buf, int nbytes);
static void esp32s3_epin_request(struct esp32s3_usbdev_s *priv,
                                 struct esp32s3_ep_s *privep);

/* OUT request and RxFIFO handling */

static void esp32s3_rxfifo_read(struct esp32s3_ep_s *privep,
                                uint8_t * dest, uint16_t len);
static void esp32s3_rxfifo_discard(struct esp32s3_ep_s *privep, int len);
static void esp32s3_epout_complete(struct esp32s3_usbdev_s *priv,
                                   struct esp32s3_ep_s *privep);
static inline void esp32s3_ep0out_receive(struct esp32s3_ep_s *privep,
                                          int bcnt);
static inline void esp32s3_epout_receive(struct esp32s3_ep_s *privep,
                                         int bcnt);
static void esp32s3_epout_request(struct esp32s3_usbdev_s *priv,
                                  struct esp32s3_ep_s *privep);

/* General request handling */

static void esp32s3_ep_flush(struct esp32s3_ep_s *privep);
static void esp32s3_req_complete(struct esp32s3_ep_s *privep,
                                 int16_t result);
static void esp32s3_req_cancel(struct esp32s3_ep_s *privep, int16_t status);

/* Interrupt handling *******************************************************/

static struct esp32s3_ep_s *esp32s3_ep_findbyaddr(
                                struct esp32s3_usbdev_s *priv,
                                uint16_t eplog);
static int esp32s3_req_dispatch(struct esp32s3_usbdev_s *priv,
                                const struct usb_ctrlreq_s *ctrl);
static void esp32s3_usbreset(struct esp32s3_usbdev_s *priv);

/* Second level OUT endpoint interrupt processing */

static inline void esp32s3_ep0out_testmode(struct esp32s3_usbdev_s *priv,
                                           uint16_t index);
static inline void esp32s3_ep0out_stdrequest(
                        struct esp32s3_usbdev_s *priv,
                        struct esp32s3_ctrlreq_s *ctrlreq);
static inline void esp32s3_ep0out_setup(struct esp32s3_usbdev_s *priv);
static inline void esp32s3_epout(struct esp32s3_usbdev_s *priv,
                                 uint8_t epno);
static inline void esp32s3_epout_interrupt(struct esp32s3_usbdev_s *priv);

/* Second level IN endpoint interrupt processing */

static inline void esp32s3_epin_runtestmode(struct esp32s3_usbdev_s *priv);
static inline void esp32s3_epin(struct esp32s3_usbdev_s *priv, uint8_t epno);
static inline void esp32s3_epin_txfifoempty(struct esp32s3_usbdev_s *priv,
                                            int epno);
static inline void esp32s3_epin_interrupt(struct esp32s3_usbdev_s *priv);

/* Other second level interrupt processing */

static inline void esp32s3_resumeinterrupt(struct esp32s3_usbdev_s *priv);
static inline void esp32s3_suspendinterrupt(struct esp32s3_usbdev_s *priv);
static inline void esp32s3_rxinterrupt(struct esp32s3_usbdev_s *priv);
static inline void esp32s3_enuminterrupt(struct esp32s3_usbdev_s *priv);
#ifdef CONFIG_USBDEV_ISOCHRONOUS
static inline void esp32s3_isocininterrupt(struct esp32s3_usbdev_s *priv);
static inline void esp32s3_isocoutinterrupt(struct esp32s3_usbdev_s *priv);
#endif
#ifdef CONFIG_USBDEV_VBUSSENSING
static inline void esp32s3_sessioninterrupt(struct esp32s3_usbdev_s *priv);
static inline void esp32s3_otginterrupt(struct esp32s3_usbdev_s *priv);
#endif

/* First level interrupt processing */

static int esp32s3_usbinterrupt(int irq, void *context, void *arg);

/* Endpoint operations ******************************************************/

/* Global OUT NAK controls */

static void esp32s3_enablegonak(struct esp32s3_ep_s *privep);
static void esp32s3_disablegonak(struct esp32s3_ep_s *privep);

/* Endpoint configuration */

static int esp32s3_epout_configure(struct esp32s3_ep_s *privep,
                                   uint8_t eptype, uint16_t maxpacket);
static int esp32s3_epin_configure(struct esp32s3_ep_s *privep,
                                  uint8_t eptype, uint16_t maxpacket);
static int esp32s3_ep_configure(struct usbdev_ep_s *ep,
                                const struct usb_epdesc_s *desc,
                                bool last);
static void esp32s3_ep0_configure(struct esp32s3_usbdev_s *priv);

/* Endpoint disable */

static void esp32s3_epout_disable(struct esp32s3_ep_s *privep);
static void esp32s3_epin_disable(struct esp32s3_ep_s *privep);
static int esp32s3_ep_disable(struct usbdev_ep_s *ep);

/* Endpoint request management */

static struct usbdev_req_s *esp32s3_ep_allocreq(
                                struct usbdev_ep_s *ep);
static void esp32s3_ep_freereq(struct usbdev_ep_s *ep,
                               struct usbdev_req_s *);

/* Endpoint buffer management */

#ifdef CONFIG_USBDEV_DMA
static void *esp32s3_ep_allocbuffer(struct usbdev_ep_s *ep,
                                    uint16_t bytes);
static void esp32s3_ep_freebuffer(struct usbdev_ep_s *ep, void *buf);
#endif

/* Endpoint request submission */

static int esp32s3_ep_submit(struct usbdev_ep_s *ep,
                             struct usbdev_req_s *req);

/* Endpoint request cancellation */

static int esp32s3_ep_cancel(struct usbdev_ep_s *ep,
                             struct usbdev_req_s *req);

/* Stall handling */

static int esp32s3_epout_setstall(struct esp32s3_ep_s *privep);
static int esp32s3_epin_setstall(struct esp32s3_ep_s *privep);
static int esp32s3_ep_setstall(struct esp32s3_ep_s *privep);
static int esp32s3_ep_clrstall(struct esp32s3_ep_s *privep);
static int esp32s3_ep_stall(struct usbdev_ep_s *ep, bool resume);
static void esp32s3_ep0_stall(struct esp32s3_usbdev_s *priv);

/* Endpoint allocation */

static struct usbdev_ep_s *esp32s3_ep_alloc(struct usbdev_s *dev,
                                            uint8_t epno, bool in,
                                            uint8_t eptype);
static void esp32s3_ep_free(struct usbdev_s *dev,
                            struct usbdev_ep_s *ep);

/* USB device controller operations *****************************************/

static int esp32s3_getframe(struct usbdev_s *dev);
static int esp32s3_wakeup(struct usbdev_s *dev);
static int esp32s3_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int esp32s3_pullup(struct usbdev_s *dev, bool enable);
static void esp32s3_setaddress(struct esp32s3_usbdev_s *priv,
                               uint16_t address);
static int esp32s3_txfifo_flush(uint32_t txfnum);
static int esp32s3_rxfifo_flush(void);

/* Initialization ***********************************************************/

static void esp32s3_swinitialize(struct esp32s3_usbdev_s *priv);
static void esp32s3_hwinitialize(struct esp32s3_usbdev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static struct esp32s3_usbdev_s g_otghsdev;

static const struct usbdev_epops_s g_epops =
{
  .configure = esp32s3_ep_configure,
  .disable = esp32s3_ep_disable,
  .allocreq = esp32s3_ep_allocreq,
  .freereq = esp32s3_ep_freereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = esp32s3_ep_allocbuffer,
  .freebuffer = esp32s3_ep_freebuffer,
#endif
  .submit = esp32s3_ep_submit,
  .cancel = esp32s3_ep_cancel,
  .stall = esp32s3_ep_stall,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep = esp32s3_ep_alloc,
  .freeep = esp32s3_ep_free,
  .getframe = esp32s3_getframe,
  .wakeup = esp32s3_wakeup,
  .selfpowered = esp32s3_selfpowered,
  .pullup = esp32s3_pullup,
};

/* Device error strings that may be enabled for more descriptive USB trace
 * output.
 */

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_deverror[] =
{
  TRACE_STR(ESP32S3_TRACEERR_ALLOCFAIL),
  TRACE_STR(ESP32S3_TRACEERR_BADCLEARFEATURE),
  TRACE_STR(ESP32S3_TRACEERR_BADDEVGETSTATUS),
  TRACE_STR(ESP32S3_TRACEERR_BADEPNO),
  TRACE_STR(ESP32S3_TRACEERR_BADEPGETSTATUS),
  TRACE_STR(ESP32S3_TRACEERR_BADGETCONFIG),
  TRACE_STR(ESP32S3_TRACEERR_BADGETSETDESC),
  TRACE_STR(ESP32S3_TRACEERR_BADGETSTATUS),
  TRACE_STR(ESP32S3_TRACEERR_BADSETADDRESS),
  TRACE_STR(ESP32S3_TRACEERR_BADSETCONFIG),
  TRACE_STR(ESP32S3_TRACEERR_BADSETFEATURE),
  TRACE_STR(ESP32S3_TRACEERR_BADTESTMODE),
  TRACE_STR(ESP32S3_TRACEERR_BINDFAILED),
  TRACE_STR(ESP32S3_TRACEERR_DISPATCHSTALL),
  TRACE_STR(ESP32S3_TRACEERR_DRIVER),
  TRACE_STR(ESP32S3_TRACEERR_DRIVERREGISTERED),
  TRACE_STR(ESP32S3_TRACEERR_EP0NOSETUP),
  TRACE_STR(ESP32S3_TRACEERR_EP0SETUPSTALLED),
  TRACE_STR(ESP32S3_TRACEERR_EPINNULLPACKET),
  TRACE_STR(ESP32S3_TRACEERR_EPINUNEXPECTED),
  TRACE_STR(ESP32S3_TRACEERR_EPOUTNULLPACKET),
  TRACE_STR(ESP32S3_TRACEERR_EPOUTUNEXPECTED),
  TRACE_STR(ESP32S3_TRACEERR_INVALIDCTRLREQ),
  TRACE_STR(ESP32S3_TRACEERR_INVALIDPARMS),
  TRACE_STR(ESP32S3_TRACEERR_IRQREGISTRATION),
  TRACE_STR(ESP32S3_TRACEERR_NOEP),
  TRACE_STR(ESP32S3_TRACEERR_NOTCONFIGURED),
  TRACE_STR(ESP32S3_TRACEERR_EPOUTQEMPTY),
  TRACE_STR(ESP32S3_TRACEERR_EPINREQEMPTY),
  TRACE_STR(ESP32S3_TRACEERR_NOOUTSETUP),
  TRACE_STR(ESP32S3_TRACEERR_POLLTIMEOUT),
  TRACE_STR_END
};
#endif

/* Interrupt event strings that may be enabled for more descriptive USB trace
 * output.
 */

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_intdecode[] =
{
  TRACE_STR(ESP32S3_TRACEINTID_USB),
  TRACE_STR(ESP32S3_TRACEINTID_INTPENDING),
  TRACE_STR(ESP32S3_TRACEINTID_EPOUT),
  TRACE_STR(ESP32S3_TRACEINTID_EPIN),
  TRACE_STR(ESP32S3_TRACEINTID_MISMATCH),
  TRACE_STR(ESP32S3_TRACEINTID_WAKEUP),
  TRACE_STR(ESP32S3_TRACEINTID_SUSPEND),
  TRACE_STR(ESP32S3_TRACEINTID_SOF),
  TRACE_STR(ESP32S3_TRACEINTID_RXFIFO),
  TRACE_STR(ESP32S3_TRACEINTID_DEVRESET),
  TRACE_STR(ESP32S3_TRACEINTID_ENUMDNE),
  TRACE_STR(ESP32S3_TRACEINTID_IISOIXFR),
  TRACE_STR(ESP32S3_TRACEINTID_IISOOXFR),
  TRACE_STR(ESP32S3_TRACEINTID_SRQ),
  TRACE_STR(ESP32S3_TRACEINTID_OTG),
  TRACE_STR(ESP32S3_TRACEINTID_EPOUT_XFRC),
  TRACE_STR(ESP32S3_TRACEINTID_EPOUT_EPDISD),
  TRACE_STR(ESP32S3_TRACEINTID_EPOUT_SETUP),
  TRACE_STR(ESP32S3_TRACEINTID_DISPATCH),
  TRACE_STR(ESP32S3_TRACEINTID_GETSTATUS),
  TRACE_STR(ESP32S3_TRACEINTID_EPGETSTATUS),
  TRACE_STR(ESP32S3_TRACEINTID_DEVGETSTATUS),
  TRACE_STR(ESP32S3_TRACEINTID_IFGETSTATUS),
  TRACE_STR(ESP32S3_TRACEINTID_CLEARFEATURE),
  TRACE_STR(ESP32S3_TRACEINTID_SETFEATURE),
  TRACE_STR(ESP32S3_TRACEINTID_SETADDRESS),
  TRACE_STR(ESP32S3_TRACEINTID_GETSETDESC),
  TRACE_STR(ESP32S3_TRACEINTID_GETCONFIG),
  TRACE_STR(ESP32S3_TRACEINTID_SETCONFIG),
  TRACE_STR(ESP32S3_TRACEINTID_GETSETIF),
  TRACE_STR(ESP32S3_TRACEINTID_SYNCHFRAME),
  TRACE_STR(ESP32S3_TRACEINTID_EPIN_XFRC),
  TRACE_STR(ESP32S3_TRACEINTID_EPIN_TOC),
  TRACE_STR(ESP32S3_TRACEINTID_EPIN_ITTXFE),
  TRACE_STR(ESP32S3_TRACEINTID_EPIN_EPDISD),
  TRACE_STR(ESP32S3_TRACEINTID_EPIN_TXFE),
  TRACE_STR(ESP32S3_TRACEINTID_EPIN_EMPWAIT),
  TRACE_STR(ESP32S3_TRACEINTID_OUTNAK),
  TRACE_STR(ESP32S3_TRACEINTID_OUTRECVD),
  TRACE_STR(ESP32S3_TRACEINTID_OUTDONE),
  TRACE_STR(ESP32S3_TRACEINTID_SETUPDONE),
  TRACE_STR(ESP32S3_TRACEINTID_SETUPRECVD),
  TRACE_STR_END
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_getreg
 *
 * Description:
 *   Get the contents of an ESP32-S3 register
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_OTG_DEBUG_REGISTER
static uint32_t esp32s3_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register? If so, suppress some of the output.
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
      preval = val;
      count = 1;
    }

  /* Show the register value read */

  uinfo("%08x->%08x\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: esp32s3_putreg
 *
 * Description:
 *   Set the contents of an ESP32-S3 register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_OTG_DEBUG_REGISTER
static void esp32s3_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  uinfo("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: esp32s3_req_remfirst
 *
 * Description:
 *   Remove a request from the head of an endpoint request queue
 *
 ****************************************************************************/

static struct esp32s3_req_s *esp32s3_req_remfirst(
                                struct esp32s3_ep_s *privep)
{
  struct esp32s3_req_s *ret = privep->head;

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
 * Name: esp32s3_req_addlast
 *
 * Description:
 *   Add a request to the end of an endpoint request queue
 *
 ****************************************************************************/

static bool esp32s3_req_addlast(struct esp32s3_ep_s *privep,
                                struct esp32s3_req_s *req)
{
  bool is_empty = !privep->head;

  req->flink = NULL;
  if (is_empty)
    {
      privep->head = req;
      privep->tail = req;
    }
  else
    {
      privep->tail->flink = req;
      privep->tail = req;
    }

  return is_empty;
}

/****************************************************************************
 * Name: esp32s3_ep0in_setupresponse
 *
 * Description:
 *   Schedule a short transfer on Endpoint 0 (IN or OUT)
 *
 ****************************************************************************/

static void esp32s3_ep0in_setupresponse(struct esp32s3_usbdev_s *priv,
                                        uint8_t *buf,
                                        uint32_t nbytes)
{
  esp32s3_epin_transfer(&priv->epin[EP0], buf, nbytes);
  priv->ep0state = EP0STATE_SETUPRESPONSE;
  esp32s3_ep0out_ctrlsetup(priv);
}

/****************************************************************************
 * Name: esp32s3_ep0in_transmitzlp
 *
 * Description:
 *   Send a zero length packet (ZLP) on endpoint 0 IN
 *
 ****************************************************************************/

static inline void esp32s3_ep0in_transmitzlp(struct esp32s3_usbdev_s *priv)
{
  esp32s3_ep0in_setupresponse(priv, NULL, 0);
}

/****************************************************************************
 * Name: esp32s3_ep0in_activate
 *
 * Description:
 *   Activate the endpoint 0 IN endpoint.
 *
 ****************************************************************************/

static void esp32s3_ep0in_activate(void)
{
  uint32_t regval;

  /* Set the max packet size of the IN EP. */

  regval = esp32s3_getreg(ESP32S3_OTG_DIEPCTL(0));
  regval &= ~OTG_DIEPCTL0_MPSIZ_MASK;

#if ESP32S3_CDCACM_EP0MAXPACKET == 8
  regval |= OTG_DIEPCTL0_MPSIZ_8;
#elif ESP32S3_CDCACM_EP0MAXPACKET == 16
  regval |= OTG_DIEPCTL0_MPSIZ_16;
#elif ESP32S3_CDCACM_EP0MAXPACKET == 32
  regval |= OTG_DIEPCTL0_MPSIZ_32;
#elif ESP32S3_CDCACM_EP0MAXPACKET == 64
  regval |= OTG_DIEPCTL0_MPSIZ_64;
#else
#  error "Unsupported value of ESP32S3_CDCACM_EP0MAXPACKET"
#endif

  esp32s3_putreg(regval, ESP32S3_OTG_DIEPCTL(0));

  /* Clear global IN NAK */

  regval = esp32s3_getreg(ESP32S3_OTG_DCTL);
  regval |= OTG_DCTL_CGINAK;
  esp32s3_putreg(regval, ESP32S3_OTG_DCTL);
}

/****************************************************************************
 * Name: esp32s3_ep0out_ctrlsetup
 *
 * Description:
 *   Setup to receive a SETUP packet.
 *
 ****************************************************************************/

static void esp32s3_ep0out_ctrlsetup(struct esp32s3_usbdev_s *priv)
{
  uint32_t regval;

  /* Setup the hardware to perform the SETUP transfer */

  regval = ((USB_SIZEOF_CTRLREQ * 3) << OTG_DOEPTSIZ0_XFRSIZ_SHIFT) |
           (OTG_DOEPTSIZ0_PKTCNT) |
           (3 << OTG_DOEPTSIZ0_STUPCNT_SHIFT);
  esp32s3_putreg(regval, ESP32S3_OTG_DOEPTSIZ(0));

  /* Then clear NAKing and enable the transfer */

  regval = esp32s3_getreg(ESP32S3_OTG_DOEPCTL(0));
  regval |= (OTG_DOEPCTL0_CNAK | OTG_DOEPCTL0_EPENA);
  esp32s3_putreg(regval, ESP32S3_OTG_DOEPCTL(0));
}

/****************************************************************************
 * Name: esp32s3_txfifo_write
 *
 * Description:
 *   Send data to the endpoint's TxFIFO.
 *
 ****************************************************************************/

static void esp32s3_txfifo_write(struct esp32s3_ep_s *privep,
                                 uint8_t *buf,
                                 int nbytes)
{
  uint32_t regaddr;
  uint32_t regval;
  int nwords;
  int i;

  /* Convert the number of bytes to words */

  nwords = (nbytes + 3) >> 2;

  /* Get the TxFIFO for this endpoint (same as the endpoint number) */

  regaddr = ESP32S3_OTG_DFIFO_DEP(privep->epphy);

  /* Then transfer each word to the TxFIFO */

  for (i = 0; i < nwords; i++)
    {
      /* Read four bytes from the source buffer (to avoid unaligned accesses)
       * and pack these into one 32-bit word (little endian).
       */

      regval = (uint32_t) *buf++;
      regval |= ((uint32_t) *buf++) << 8;
      regval |= ((uint32_t) *buf++) << 16;
      regval |= ((uint32_t) *buf++) << 24;

      /* Then write the packet data to the TxFIFO */

      esp32s3_putreg(regval, regaddr);
    }
}

/****************************************************************************
 * Name: esp32s3_epin_transfer
 *
 * Description:
 *   Start the Tx data transfer
 *
 ****************************************************************************/

static void esp32s3_epin_transfer(struct esp32s3_ep_s *privep,
                                  uint8_t *buf,
                                  int nbytes)
{
  uint32_t pktcnt;
  uint32_t regval;

  /* Read the DIEPSIZx register */

  regval = esp32s3_getreg(ESP32S3_OTG_DIEPTSIZ(privep->epphy));

  /* Clear the XFRSIZ, PKTCNT, and MCNT field of the DIEPSIZx register */

  regval &= ~(OTG_DIEPTSIZ_XFRSIZ_MASK | OTG_DIEPTSIZ_PKTCNT_MASK |
              OTG_DIEPTSIZ_MCNT_MASK);

  /* Are we sending a zero length packet (ZLP) */

  if (nbytes == 0)
    {
      /* Yes.. leave the transfer size at zero and set the packet count to
       * 1
       */

      pktcnt = 1;
    }
  else
    {
      /* No.. Program the transfer size and packet count .  First calculate:
       * xfrsize = The total number of bytes to be sent. pktcnt = the number
       * of packets (of maxpacket bytes) required to perform the transfer.
       */

      pktcnt = ((uint32_t)nbytes + (privep->ep.maxpacket - 1)) /
               privep->ep.maxpacket;
    }

  /* Set the XFRSIZ and PKTCNT */

  regval |= (pktcnt << OTG_DIEPTSIZ_PKTCNT_SHIFT);
  regval |= ((uint32_t)nbytes << OTG_DIEPTSIZ_XFRSIZ_SHIFT);

  /* If this is an isochronous endpoint, then set the multi-count field to
   * the PKTCNT as well.
   */

  if (privep->eptype == USB_EP_ATTR_XFER_ISOC)
    {
      regval |= (pktcnt << OTG_DIEPTSIZ_MCNT_SHIFT);
    }

  /* Save DIEPSIZx register value */

  esp32s3_putreg(regval, ESP32S3_OTG_DIEPTSIZ(privep->epphy));

  /* Read the DIEPCTLx register */

  regval = esp32s3_getreg(ESP32S3_OTG_DIEPCTL(privep->epphy));

  /* If this is an isochronous endpoint, then set the even/odd frame bit the
   * DIEPCTLx register.
   */

  if (privep->eptype == USB_EP_ATTR_XFER_ISOC)
    {
      /* Check bit 0 of the frame number of the received SOF and set the
       * even/odd frame to match.
       */

      uint32_t status = esp32s3_getreg(ESP32S3_OTG_DSTS);
      if ((status & OTG_DSTS_SOFFN0) == OTG_DSTS_SOFFN_EVEN)
        {
          regval |= OTG_DIEPCTL_SEVNFRM;
        }
      else
        {
          regval |= OTG_DIEPCTL_SODDFRM;
        }
    }

  /* EP enable, IN data in FIFO */

  regval &= ~OTG_DIEPCTL_EPDIS;
  regval |= (OTG_DIEPCTL_CNAK | OTG_DIEPCTL_EPENA);
  esp32s3_putreg(regval, ESP32S3_OTG_DIEPCTL(privep->epphy));

  /* Transfer the data to the TxFIFO.  At this point, the caller has already
   * assured that there is sufficient space in the TxFIFO to hold the
   * transfer we can just blindly continue.
   */

  esp32s3_txfifo_write(privep, buf, nbytes);
}

/****************************************************************************
 * Name: esp32s3_epin_request
 *
 * Description:
 *   Begin or continue write request processing.
 *
 ****************************************************************************/

static void esp32s3_epin_request(struct esp32s3_usbdev_s *priv,
                                 struct esp32s3_ep_s *privep)
{
  struct esp32s3_req_s *privreq;
  uint32_t regaddr;
  uint32_t regval;
  uint8_t *buf;
  int nbytes;
  int nwords;
  int bytesleft;

  /* We get here in one of four possible ways.  From three interrupting
   * events:
   *
   * 1. From esp32s3_epin as part of the transfer complete interrupt
   *    processing This interrupt indicates that the last transfer has
   *    completed.
   * 2. As part of the ITTXFE interrupt processing.  That interrupt
   *    indicates that an IN token was received when the associated
   *    TxFIFO was empty.
   * 3. From esp32s3_epin_txfifoempty as part of the TXFE interrupt
   *    processing.  The TXFEinterrupt is only enabled when the TxFIFO
   *    is full and the software must wait for space to become available
   *    in the TxFIFO.
   *
   * And this function may be called immediately when the write request
   * is queued to start up the next transaction.
   *
   * 4. From esp32s3_ep_submit when a new write request is received WHILE
   *    the endpoint is not active (privep->active == false).
   */

  /* Check the request from the head of the endpoint request queue */

  privreq = esp32s3_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_EPINREQEMPTY), privep->epphy);

      /* There is no TX transfer in progress and no new pending TX requests
       * to send.  To stop transmitting any data on a particular IN endpoint,
       * the application must set the IN NAK bit. To set this bit, the
       * following field must be programmed.
       */

      regaddr = ESP32S3_OTG_DIEPCTL(privep->epphy);
      regval = esp32s3_getreg(regaddr);
      regval |= OTG_DIEPCTL_SNAK;
      esp32s3_putreg(regval, regaddr);

      /* The endpoint is no longer active */

      privep->active = false;
      return;
    }

  uinfo("EP%d req=%p: len=%d xfrd=%d zlp=%d\n",
        privep->epphy, privreq, privreq->req.len,
        privreq->req.xfrd, privep->zlp);

  /* Check for a special case: If we are just starting a request (xfrd==0)
   * and the class driver is trying to send a zero-length packet (len==0).
   * Then set the ZLP flag so that the packet will be sent.
   */

  if (privreq->req.len == 0)
    {
      /* The ZLP flag is set TRUE whenever we want to force the driver to
       * send a zero-length-packet on the next pass through the loop (below).
       * The flag is cleared whenever a packet is sent in the loop below.
       */

      privep->zlp = true;
    }

  /* Add one more packet to the TxFIFO.  We will wait for the transfer
   * complete event before we add the next packet (or part of a packet to
   * the TxFIFO).
   * The documentation says that we can can multiple packets to the TxFIFO,
   * but it seems that we need to get the transfer complete event before we
   * can add the next (or maybe I have got something wrong?)
   */

#if 0
  while (privreq->req.xfrd < privreq->req.len || privep->zlp)
#else
  if (privreq->req.xfrd < privreq->req.len || privep->zlp)
#endif
    {
      /* Get the number of bytes left to be sent in the request */

      bytesleft = privreq->req.len - privreq->req.xfrd;
      nbytes = bytesleft;

      /* Assume no zero-length-packet on the next pass through this loop */

      privep->zlp = false;

      /* Limit the size of the transfer to one full packet and handle
       * zero-length packets (ZLPs).
       */

      if (nbytes > 0)
        {
          /* Either send the maxpacketsize or all of the remaining data in
           * the request.
           */

          if (nbytes >= privep->ep.maxpacket)
            {
              nbytes = privep->ep.maxpacket;

              /* Handle the case where this packet is exactly the
               * maxpacketsize.  Do we need to send a zero-length packet in
               * this case?
               */

              if (bytesleft == privep->ep.maxpacket &&
                  (privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0)
                {
                  /* The ZLP flag is set TRUE whenever we want to force the
                   * driver to send a zero-length-packet on the next pass
                   * through this loop. The flag is cleared (above) whenever
                   * we are committed to sending any packet and set here when
                   * we want to force one more pass through the loop.
                   */

                  privep->zlp = true;
                }
            }
        }

      /* Get the transfer size in 32-bit words */

      nwords = (nbytes + 3) >> 2;

      /* Get the number of 32-bit words available in the TxFIFO. The DXTFSTS
       * indicates the amount of free space available in the endpoint TxFIFO.
       * Values are in terms of 32-bit words:
       *
       *   0: Endpoint TxFIFO is full
       *   1: 1 word available
       *   2: 2 words available
       *   n: n words available
       */

      regaddr = ESP32S3_OTG_DTXFSTS(privep->epphy);

      /* Check for space in the TxFIFO.  If space in the TxFIFO is not
       * available, then set up an interrupt to resume the transfer when the
       * TxFIFO is empty.
       */

      regval = esp32s3_getreg(regaddr);
      if ((int)(regval & OTG_DTXFSTS_MASK) < nwords)
        {
          usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_EPIN_EMPWAIT),
                   (uint16_t)regval);

          /* There is insufficient space in the TxFIFO.  Wait for a TxFIFO
           * empty interrupt and try again.
           */

          uint32_t empmsk = esp32s3_getreg(ESP32S3_OTG_DIEPEMPMSK);
          empmsk |= OTG_DIEPEMPMSK(privep->epphy);
          esp32s3_putreg(empmsk, ESP32S3_OTG_DIEPEMPMSK);

#ifdef CONFIG_DEBUG_FEATURES
          /* Check if the configured TXFIFO size is sufficient for a given
           * request. If not, raise an assertion here.
           */

          regval = esp32s3_getreg(ESP32S3_OTG_DIEPTXF(privep->epphy));
          regval &= OTG_DIEPTXF_INEPTXFD_MASK;
          regval >>= OTG_DIEPTXF_INEPTXFD_SHIFT;
          uerr("EP%" PRId8 " TXLEN=%" PRId32 " nwords=%d\n",
               privep->epphy, regval, nwords);
          DEBUGASSERT(regval >= nwords);
#endif

          /* Terminate the transfer.  We will try again when the TxFIFO empty
           * interrupt is received.
           */

          return;
        }

      /* Transfer data to the TxFIFO */

      buf = privreq->req.buf + privreq->req.xfrd;
      esp32s3_epin_transfer(privep, buf, nbytes);

      /* If it was not before, the OUT endpoint is now actively transferring
       * data.
       */

      privep->active = true;

      /* EP0 is a special case */

      if (privep->epphy == EP0)
        {
          priv->ep0state = EP0STATE_DATA_IN;
        }

      /* Update for the next time through the loop */

      privreq->req.xfrd += nbytes;
    }

  /* Note that the ZLP, if any, must be sent as a separate transfer.  The
   * need for a ZLP is indicated by privep->zlp.  If all of the bytes were
   * sent (including any final null packet) then we are finished with the
   * transfer
   */

  if (privreq->req.xfrd >= privreq->req.len && !privep->zlp)
    {
      usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);

      /* We are finished with the request (although the transfer has not yet
       * completed).
       */

      esp32s3_req_complete(privep, OK);
    }
}

/****************************************************************************
 * Name: esp32s3_rxfifo_read
 *
 * Description:
 *   Read packet from the RxFIFO into a read request.
 *
 ****************************************************************************/

static void esp32s3_rxfifo_read(struct esp32s3_ep_s *privep,
                                uint8_t *dest,
                                uint16_t len)
{
  uint32_t regaddr;
  int i;

  /* Get the address of the RxFIFO.  Note: there is only one RxFIFO so we
   * might as well use the address associated with EP0.
   */

  regaddr = ESP32S3_OTG_DFIFO_DEP(EP0);

  /* Read 32-bits and write 4 x 8-bits at time (to avoid unaligned
   * accesses)
   */

  for (i = 0; i < len; i += 4)
    {
      union
        {
          uint32_t w;
          uint8_t b[4];
        } data;

      /* Read 1 x 32-bits of EP0 packet data */

      data.w = esp32s3_getreg(regaddr);

      /* Write 4 x 8-bits of EP0 packet data */

      *dest++ = data.b[0];
      *dest++ = data.b[1];
      *dest++ = data.b[2];
      *dest++ = data.b[3];
    }
}

/****************************************************************************
 * Name: esp32s3_rxfifo_discard
 *
 * Description:
 *   Discard packet data from the RxFIFO.
 *
 ****************************************************************************/

static void esp32s3_rxfifo_discard(struct esp32s3_ep_s *privep, int len)
{
  if (len > 0)
    {
      uint32_t regaddr;
      int i;

      /* Get the address of the RxFIFO Note: there is only one RxFIFO so we
       * might as well use the address associated with EP0.
       */

      regaddr = ESP32S3_OTG_DFIFO_DEP(EP0);

      /* Read 32-bits at time */

      for (i = 0; i < len; i += 4)
        {
          volatile uint32_t data = esp32s3_getreg(regaddr);
          UNUSED(data);
        }
    }
}

/****************************************************************************
 * Name: esp32s3_epout_complete
 *
 * Description:
 *   This function is called when an OUT transfer complete interrupt is
 *   received.  It completes the read request at the head of the endpoint's
 *   request queue.
 *
 ****************************************************************************/

static void esp32s3_epout_complete(struct esp32s3_usbdev_s *priv,
                                   struct esp32s3_ep_s *privep)
{
  struct esp32s3_req_s *privreq;

  /* Since a transfer just completed, there must be a read request at the
   * head of the endpoint request queue.
   */

  privreq = esp32s3_rqpeek(privep);
  DEBUGASSERT(privreq);

  if (!privreq)
    {
      /* An OUT transfer completed, but no packet to receive the data.  This
       * should not happen.
       */

      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_EPOUTQEMPTY), privep->epphy);
      privep->active = false;
      return;
    }

  uinfo("EP%d: len=%d xfrd=%d\n",
        privep->epphy, privreq->req.len, privreq->req.xfrd);

  /* Return the completed read request to the class driver and mark the state
   * IDLE.
   */

  usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
  esp32s3_req_complete(privep, OK);
  privep->active = false;

  /* Now set up the next read request (if any) */

  esp32s3_epout_request(priv, privep);
}

/****************************************************************************
 * Name: esp32s3_ep0out_receive
 *
 * Description:
 *   This function is called from the RXFLVL interrupt handler when new
 *   incoming data is available in the endpoint's RxFIFO.  This function
 *   will simply copy the incoming data into pending request's data buffer.
 *
 ****************************************************************************/

static inline void esp32s3_ep0out_receive(struct esp32s3_ep_s *privep,
                                          int bcnt)
{
  struct esp32s3_usbdev_s *priv;

  /* Sanity Checking */

  DEBUGASSERT(privep && privep->dev);
  priv = (struct esp32s3_usbdev_s *)privep->dev;

  uinfo("EP0: bcnt=%d\n", bcnt);
  usbtrace(TRACE_READ(EP0), bcnt);

  /* Verify that an OUT SETUP request as received before this data was
   * received in the RxFIFO.
   */

  if (priv->ep0state == EP0STATE_SETUP_OUT)
    {
      /* Read the data into our special buffer for SETUP data */

      int readlen = MIN(ESP32S3_CDCACM_EP0MAXPACKET, bcnt);
      esp32s3_rxfifo_read(privep, priv->ep0data, readlen);

      /* Do we have to discard any excess bytes? */

      esp32s3_rxfifo_discard(privep, bcnt - readlen);

      /* Now we can process the setup command */

      privep->active = false;
      priv->ep0state = EP0STATE_SETUP_READY;
      priv->ep0datlen = readlen;

      esp32s3_ep0out_setup(priv);
    }
  else
    {
      /* This is an error.  We don't have any idea what to do with the EP0
       * data in this case.  Just read and discard it so that the RxFIFO
       * does not become constipated.
       */

      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_NOOUTSETUP), priv->ep0state);
      esp32s3_rxfifo_discard(privep, bcnt);
      privep->active = false;
    }
}

/****************************************************************************
 * Name: esp32s3_epout_receive
 *
 * Description:
 *   This function is called from the RXFLVL interrupt handler when new
 *   incoming data is available in the endpoint's RxFIFO.  This function
 *   will simply copy the incoming data into pending request's data buffer.
 *
 ****************************************************************************/

static inline void esp32s3_epout_receive(struct esp32s3_ep_s *privep,
                                         int bcnt)
{
  struct esp32s3_req_s *privreq;
  uint8_t *dest;
  int buflen;
  int readlen;

  /* Get a reference to the request at the head of the endpoint's request
   * queue.
   */

  privreq = esp32s3_rqpeek(privep);
  if (!privreq)
    {
      /* Incoming data is available in the RxFIFO, but there is no read setup
       * to receive the receive the data.  This should not happen for data
       * endpoints; those endpoints should have been NAKing any OUT data
       * tokens.
       *
       * We should get here normally on OUT data phase following an OUT
       * SETUP command.  EP0 data will still receive data in this case and
       * it should not be NAKing.
       */

      if (privep->epphy == 0)
        {
          esp32s3_ep0out_receive(privep, bcnt);
        }
      else
        {
          /* Otherwise, the data is lost. This really should not happen if
           * NAKing is working as expected.
           */

          usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_EPOUTQEMPTY),
                   privep->epphy);

          /* Discard the data in the RxFIFO */

          esp32s3_rxfifo_discard(privep, bcnt);
        }

      privep->active = false;
      return;
    }

  uinfo("EP%d: len=%d xfrd=%d\n", privep->epphy, privreq->req.len,
        privreq->req.xfrd);
  usbtrace(TRACE_READ(privep->epphy), bcnt);

  /* Get the number of bytes to transfer from the RxFIFO */

  buflen = privreq->req.len - privreq->req.xfrd;
  DEBUGASSERT(buflen > 0 && buflen >= bcnt);
  readlen = MIN(buflen, bcnt);

  /* Get the destination of the data transfer */

  dest = privreq->req.buf + privreq->req.xfrd;

  /* Transfer the data from the RxFIFO to the request's data buffer */

  esp32s3_rxfifo_read(privep, dest, readlen);

  /* If there were more bytes in the RxFIFO than could be held in the read
   * request, then we will have to discard those.
   */

  esp32s3_rxfifo_discard(privep, bcnt - readlen);

  /* Update the number of bytes transferred */

  privreq->req.xfrd += readlen;
}

/****************************************************************************
 * Name: esp32s3_epout_request
 *
 * Description:
 *   This function is called when either (1) new read request is received, or
 *   (2) a pending receive request completes.  If there is no read in
 *   pending, then this function will initiate the next OUT (read) operation.
 *
 ****************************************************************************/

static void esp32s3_epout_request(struct esp32s3_usbdev_s *priv,
                                  struct esp32s3_ep_s *privep)
{
  struct esp32s3_req_s *privreq;
  uint32_t regaddr;
  uint32_t regval;
  uint32_t xfrsize;
  uint32_t pktcnt;

  /* Make sure that there is not already a pending request request.  If there
   * is, just return, leaving the newly received request in the request
   * queue.
   */

  if (!privep->active)
    {
      /* Loop until a valid request is found (or the request queue is empty).
       * The loop is only need to look at the request queue again is an
       * invalid read request is encountered.
       */

      for (; ; )
        {
          /* Get a reference to the request at the head of the endpoint's
           * request queue
           */

          privreq = esp32s3_rqpeek(privep);
          if (!privreq)
            {
              usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_EPOUTQEMPTY),
                       privep->epphy);

              /* There are no read requests to be setup.  Configure the
               * hardware to NAK any incoming packets.  (This should already
               * be the case.  I think that the hardware will automatically
               * NAK after a transfer is completed until SNAK is cleared).
               */

              regaddr = ESP32S3_OTG_DOEPCTL(privep->epphy);
              regval = esp32s3_getreg(regaddr);
              regval |= OTG_DOEPCTL_SNAK;
              esp32s3_putreg(regval, regaddr);

              /* This endpoint is no longer actively transferring */

              privep->active = false;
              return;
            }

          uinfo("EP%d: len=%d\n", privep->epphy, privreq->req.len);

          /* Ignore any attempt to receive a zero length packet (this really
           * should not happen.
           */

          if (privreq->req.len <= 0)
            {
              usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_EPOUTNULLPACKET), 0);
              esp32s3_req_complete(privep, OK);
            }

          /* Otherwise, we have a usable read request... break out of the
           * loop
           */

          else
            {
              break;
            }
        }

      /* Setup the pending read into the request buffer.  First calculate:
       *
       * pktcnt = the number of packets (of maxpacket bytes) required to
       *   perform the transfer.
       * xfrsize = The total number of bytes required (in units of maxpacket
       *   bytes).
       */

      pktcnt = (privreq->req.len + (privep->ep.maxpacket - 1)) /
               privep->ep.maxpacket;
      xfrsize = pktcnt * privep->ep.maxpacket;

      /* Then setup the hardware to perform this transfer */

      regaddr = ESP32S3_OTG_DOEPTSIZ(privep->epphy);
      regval = esp32s3_getreg(regaddr);
      regval &= ~(OTG_DOEPTSIZ_XFRSIZ_MASK | OTG_DOEPTSIZ_PKTCNT_MASK);
      regval |= (xfrsize << OTG_DOEPTSIZ_XFRSIZ_SHIFT);
      regval |= (pktcnt << OTG_DOEPTSIZ_PKTCNT_SHIFT);
      esp32s3_putreg(regval, regaddr);

      /* Then enable the transfer */

      regaddr = ESP32S3_OTG_DOEPCTL(privep->epphy);
      regval = esp32s3_getreg(regaddr);

      /* When an isochronous transfer is enabled the Even/Odd frame bit must
       * also be set appropriately.
       */

#ifdef CONFIG_USBDEV_ISOCHRONOUS
      if (privep->eptype == USB_EP_ATTR_XFER_ISOC)
        {
          if (privep->odd)
            {
              regval |= OTG_DOEPCTL_SODDFRM;
            }
          else
            {
              regval |= OTG_DOEPCTL_SEVNFRM;
            }
        }
#endif

      /* Clearing NAKing and enable the transfer. */

      regval |= (OTG_DOEPCTL_CNAK | OTG_DOEPCTL_EPENA);
      esp32s3_putreg(regval, regaddr);

      /* A transfer is now active on this endpoint */

      privep->active = true;

      /* EP0 is a special case.  We need to know when to switch back to
       * normal SETUP processing.
       */

      if (privep->epphy == EP0)
        {
          priv->ep0state = EP0STATE_DATA_OUT;
        }
    }
}

/****************************************************************************
 * Name: esp32s3_ep_flush
 *
 * Description:
 *   Flush any primed descriptors from this ep
 *
 ****************************************************************************/

static void esp32s3_ep_flush(struct esp32s3_ep_s *privep)
{
  if (privep->isin)
    {
      esp32s3_txfifo_flush(OTG_GRSTCTL_TXFNUM_D(privep->epphy));
    }
  else
    {
      esp32s3_rxfifo_flush();
    }
}

/****************************************************************************
 * Name: esp32s3_req_complete
 *
 * Description:
 *   Handle termination of the request at the head of the endpoint request
 *   queue.
 *
 ****************************************************************************/

static void esp32s3_req_complete(struct esp32s3_ep_s *privep,
                                 int16_t result)
{
  struct esp32s3_req_s *privreq;

  /* Remove the request at the head of the request list */

  privreq = esp32s3_req_remfirst(privep);
  DEBUGASSERT(privreq != NULL);

  /* If endpoint 0, temporarily reflect the state of protocol stalled in the
   * callback.
   */

  bool stalled = privep->stalled;
  if (privep->epphy == EP0)
    {
      privep->stalled = privep->dev->stalled;
    }

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);

  /* Restore the stalled indication */

  privep->stalled = stalled;
}

/****************************************************************************
 * Name: esp32s3_req_cancel
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 ****************************************************************************/

static void esp32s3_req_cancel(struct esp32s3_ep_s *privep, int16_t status)
{
  if (!esp32s3_rqempty(privep))
    {
      esp32s3_ep_flush(privep);
    }

  while (!esp32s3_rqempty(privep))
    {
      usbtrace(TRACE_COMPLETE(privep->epphy),
               (esp32s3_rqpeek(privep))->req.xfrd);
      esp32s3_req_complete(privep, status);
    }
}

/****************************************************************************
 * Name: esp32s3_ep_findbyaddr
 *
 * Description:
 *   Find the physical endpoint structure corresponding to a logic endpoint
 *   address
 *
 ****************************************************************************/

static struct esp32s3_ep_s *esp32s3_ep_findbyaddr(
                              struct esp32s3_usbdev_s *priv,
                              uint16_t eplog)
{
  struct esp32s3_ep_s *privep;
  uint8_t epphy = USB_EPNO(eplog);

  if (epphy >= ESP32S3_ENDPOINT_NUM)
    {
      return NULL;
    }

  /* Is this an IN or an OUT endpoint? */

  if (USB_ISEPIN(eplog))
    {
      privep = &priv->epin[epphy];
    }
  else
    {
      privep = &priv->epout[epphy];
    }

  /* Return endpoint reference */

  DEBUGASSERT(privep->epphy == epphy);
  return privep;
}

/****************************************************************************
 * Name: esp32s3_req_dispatch
 *
 * Description:
 *   Provide unhandled setup actions to the class driver. This is logically
 *   part of the USB interrupt handler.
 *
 ****************************************************************************/

static int esp32s3_req_dispatch(struct esp32s3_usbdev_s *priv,
                                const struct usb_ctrlreq_s *ctrl)
{
  int ret = -EIO;

  usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_DISPATCH), 0);
  if (priv->driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, ctrl,
                        priv->ep0data, priv->ep0datlen);
    }

  if (ret < 0)
    {
      /* Stall on failure */

      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_DISPATCHSTALL), 0);
      priv->stalled = true;
    }

  return ret;
}

/****************************************************************************
 * Name: esp32s3_usbreset
 *
 * Description:
 *   Reset Usb engine
 *
 ****************************************************************************/

static void esp32s3_usbreset(struct esp32s3_usbdev_s *priv)
{
  struct esp32s3_ep_s *privep;
  uint32_t regval;
  int i;

  /* Clear the Remote Wake-up Signaling */

  regval = esp32s3_getreg(ESP32S3_OTG_DCTL);
  regval &= ~OTG_DCTL_RWUSIG;
  esp32s3_putreg(regval, ESP32S3_OTG_DCTL);

  /* Flush the EP0 Tx FIFO */

  esp32s3_txfifo_flush(OTG_GRSTCTL_TXFNUM_D(EP0));

  /* Tell the class driver that we are disconnected. The class driver should
   * then accept any new configurations.
   */

  if (priv->driver)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }

  /* Mark all endpoints as available */

  priv->epavail[0] = ESP32S3_EP_AVAILABLE;
  priv->epavail[1] = ESP32S3_EP_AVAILABLE;

  /* Disable all end point interrupts */

  for (i = 0; i < ESP32S3_ENDPOINT_NUM; i++)
    {
      /* Disable endpoint interrupts */

      esp32s3_putreg(0xff, ESP32S3_OTG_DIEPINT(i));
      esp32s3_putreg(0xff, ESP32S3_OTG_DOEPINT(i));

      /* Return write requests to the class implementation */

      privep = &priv->epin[i];
      esp32s3_req_cancel(privep, -ESHUTDOWN);

      /* Reset IN endpoint status */

      privep->stalled = false;
      privep->active  = false;
      privep->zlp     = false;

      /* Return read requests to the class implementation */

      privep = &priv->epout[i];
      esp32s3_req_cancel(privep, -ESHUTDOWN);

      /* Reset endpoint status */

      privep->stalled = false;
      privep->active  = false;
      privep->zlp     = false;
    }

  esp32s3_putreg(0xffffffff, ESP32S3_OTG_DAINT);

  /* Mask all device endpoint interrupts except EP0 */

  regval = (OTG_DAINT_IEP(EP0) | OTG_DAINT_OEP(EP0));
  esp32s3_putreg(regval, ESP32S3_OTG_DAINTMSK);

  /* Unmask OUT interrupts */

  regval = (OTG_DOEPMSK_XFRCM | OTG_DOEPMSK_STUPM | OTG_DOEPMSK_EPDM);
  esp32s3_putreg(regval, ESP32S3_OTG_DOEPMSK);

  /* Unmask IN interrupts */

  regval = (OTG_DIEPMSK_XFRCM | OTG_DIEPMSK_EPDM | OTG_DIEPMSK_TOM);
  esp32s3_putreg(regval, ESP32S3_OTG_DIEPMSK);

  /* Reset device address to 0 */

  esp32s3_setaddress(priv, 0);
  priv->devstate = DEVSTATE_DEFAULT;
  priv->usbdev.speed = USB_SPEED_FULL;

  /* Re-configure EP0 */

  esp32s3_ep0_configure(priv);

  /* Setup EP0 to receive SETUP packets */

  esp32s3_ep0out_ctrlsetup(priv);
}

/****************************************************************************
 * Name: esp32s3_ep0out_testmode
 *
 * Description:
 *   Select test mode
 *
 ****************************************************************************/

static void esp32s3_ep0out_testmode(struct esp32s3_usbdev_s *priv,
                                    uint16_t index)
{
  uint8_t testmode;

  testmode = index >> 8;
  switch (testmode)
    {
    case 1:
      priv->testmode = OTG_TESTMODE_J;
      break;

    case 2:
      priv->testmode = OTG_TESTMODE_K;
      break;

    case 3:
      priv->testmode = OTG_TESTMODE_SE0_NAK;
      break;

    case 4:
      priv->testmode = OTG_TESTMODE_PACKET;
      break;

    case 5:
      priv->testmode = OTG_TESTMODE_FORCE;
      break;

    default:
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_BADTESTMODE), testmode);
      priv->dotest = false;
      priv->testmode = OTG_TESTMODE_DISABLED;
      priv->stalled = true;
    }

  priv->dotest = true;
  esp32s3_ep0in_transmitzlp(priv);
}

/****************************************************************************
 * Name: esp32s3_ep0out_stdrequest
 *
 * Description:
 *   Handle a stanard request on EP0.  Pick off the things of interest to the
 *   USB device controller driver; pass what is left to the class driver.
 *
 ****************************************************************************/

static void esp32s3_ep0out_stdrequest(struct esp32s3_usbdev_s *priv,
                                      struct esp32s3_ctrlreq_s *ctrlreq)
{
  struct esp32s3_ep_s *privep;

  /* Handle standard request */

  switch (ctrlreq->req)
    {
    case USB_REQ_GETSTATUS:
      {
        /* type:  device-to-host; recipient = device, interface, endpoint
         * value: 0
         * index: zero interface endpoint
         * len:   2; data = status
         */

        usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_GETSTATUS), 0);
        if (!priv->addressed ||
            ctrlreq->len != 2 ||
            USB_REQ_ISOUT(ctrlreq->type) || ctrlreq->value != 0)
          {
            priv->stalled = true;
          }
        else
          {
            switch (ctrlreq->type & USB_REQ_RECIPIENT_MASK)
              {
              case USB_REQ_RECIPIENT_ENDPOINT:
                {
                  usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_EPGETSTATUS),
                           0);
                  privep = esp32s3_ep_findbyaddr(priv, ctrlreq->index);
                  if (!privep)
                    {
                      usbtrace(TRACE_DEVERROR(
                                  ESP32S3_TRACEERR_BADEPGETSTATUS),
                               0);
                      priv->stalled = true;
                    }
                  else
                    {
                      if (privep->stalled)
                        {
                          priv->ep0data[0] = (1 << USB_FEATURE_ENDPOINTHALT);
                        }
                      else
                        {
                          priv->ep0data[0] = 0; /* Not stalled */
                        }

                      priv->ep0data[1] = 0;
                      esp32s3_ep0in_setupresponse(priv, priv->ep0data, 2);
                    }
                }
                break;

              case USB_REQ_RECIPIENT_DEVICE:
                {
                  if (ctrlreq->index == 0)
                    {
                      usbtrace(
                        TRACE_INTDECODE(ESP32S3_TRACEINTID_DEVGETSTATUS),
                        0);

                      /* Features: Remote Wakeup and self-powered */

                      priv->ep0data[0] =
                        (priv->selfpowered << USB_FEATURE_SELFPOWERED);
                      priv->ep0data[0] |=
                        (priv->wakeup << USB_FEATURE_REMOTEWAKEUP);
                      priv->ep0data[1] = 0;

                      esp32s3_ep0in_setupresponse(priv, priv->ep0data, 2);
                    }
                  else
                    {
                      usbtrace(
                        TRACE_DEVERROR(ESP32S3_TRACEERR_BADDEVGETSTATUS),
                        0);
                      priv->stalled = true;
                    }
                }
                break;

              case USB_REQ_RECIPIENT_INTERFACE:
                {
                  usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_IFGETSTATUS),
                           0);
                  priv->ep0data[0] = 0;
                  priv->ep0data[1] = 0;

                  esp32s3_ep0in_setupresponse(priv, priv->ep0data, 2);
                }
                break;

              default:
                {
                  usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_BADGETSTATUS), 0);
                  priv->stalled = true;
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

        usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_CLEARFEATURE), 0);
        if (priv->addressed != 0 && ctrlreq->len == 0)
          {
            uint8_t recipient = ctrlreq->type & USB_REQ_RECIPIENT_MASK;
            if (recipient == USB_REQ_RECIPIENT_ENDPOINT &&
                ctrlreq->value == USB_FEATURE_ENDPOINTHALT &&
                (privep = esp32s3_ep_findbyaddr(priv, ctrlreq->index))
                != NULL)
              {
                esp32s3_ep_clrstall(privep);
                esp32s3_ep0in_transmitzlp(priv);
              }
            else if (recipient == USB_REQ_RECIPIENT_DEVICE &&
                     ctrlreq->value == USB_FEATURE_REMOTEWAKEUP)
              {
                priv->wakeup = 0;
                esp32s3_ep0in_transmitzlp(priv);
              }
            else
              {
                /* Actually, I think we could just stall here. */

                esp32s3_req_dispatch(priv, &priv->ctrlreq);
              }
          }
        else
          {
            usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_BADCLEARFEATURE), 0);
            priv->stalled = true;
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

        usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_SETFEATURE), 0);
        if (priv->addressed != 0 && ctrlreq->len == 0)
          {
            uint8_t recipient = ctrlreq->type & USB_REQ_RECIPIENT_MASK;
            if (recipient == USB_REQ_RECIPIENT_ENDPOINT &&
                ctrlreq->value == USB_FEATURE_ENDPOINTHALT &&
                (privep = esp32s3_ep_findbyaddr(priv, ctrlreq->index))
                != NULL)
              {
                esp32s3_ep_setstall(privep);
                esp32s3_ep0in_transmitzlp(priv);
              }
            else if (recipient == USB_REQ_RECIPIENT_DEVICE &&
                     ctrlreq->value == USB_FEATURE_REMOTEWAKEUP)
              {
                priv->wakeup = 1;
                esp32s3_ep0in_transmitzlp(priv);
              }
            else if (recipient == USB_REQ_RECIPIENT_DEVICE &&
                     ctrlreq->value == USB_FEATURE_TESTMODE &&
                     ((ctrlreq->index & 0xff) == 0))
              {
                esp32s3_ep0out_testmode(priv, ctrlreq->index);
              }
            else if (priv->configured)
              {
                /* Actually, I think we could just stall here. */

                esp32s3_req_dispatch(priv, &priv->ctrlreq);
              }
            else
              {
                usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_BADSETFEATURE), 0);
                priv->stalled = true;
              }
          }
        else
          {
            usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_BADSETFEATURE), 0);
            priv->stalled = true;
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

        usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_SETADDRESS),
                 ctrlreq->value);
        if ((ctrlreq->type & USB_REQ_RECIPIENT_MASK) ==
            USB_REQ_RECIPIENT_DEVICE
            && ctrlreq->index == 0 && ctrlreq->len == 0
            && ctrlreq->value < 128
            && priv->devstate != DEVSTATE_CONFIGURED)
          {
            /* Save the address.  We cannot actually change to the next
             * address until the completion of the status phase.
             */

            esp32s3_setaddress(priv, (uint16_t)priv->ctrlreq.value[0]);
            esp32s3_ep0in_transmitzlp(priv);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_BADSETADDRESS), 0);
            priv->stalled = true;
          }
      }
      break;

    case USB_REQ_GETDESCRIPTOR:
      /* type:  device-to-host; recipient = device, interface
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
        usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_GETSETDESC), 0);
        if (((ctrlreq->type & USB_REQ_RECIPIENT_MASK) ==
             USB_REQ_RECIPIENT_DEVICE) ||
            ((ctrlreq->type & USB_REQ_RECIPIENT_MASK) ==
             USB_REQ_RECIPIENT_INTERFACE))
          {
            esp32s3_req_dispatch(priv, &priv->ctrlreq);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_BADGETSETDESC), 0);
            priv->stalled = true;
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
        usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_GETCONFIG), 0);
        if (priv->addressed &&
            (ctrlreq->type & USB_REQ_RECIPIENT_MASK) ==
            USB_REQ_RECIPIENT_DEVICE
            && ctrlreq->value == 0 && ctrlreq->index == 0
            && ctrlreq->len == 1)
          {
            esp32s3_req_dispatch(priv, &priv->ctrlreq);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_BADGETCONFIG), 0);
            priv->stalled = true;
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
        usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_SETCONFIG), 0);
        if (priv->addressed &&
            (ctrlreq->type & USB_REQ_RECIPIENT_MASK) ==
            USB_REQ_RECIPIENT_DEVICE
            && ctrlreq->index == 0 && ctrlreq->len == 0)
          {
            /* Give the configuration to the class driver */

            int ret = esp32s3_req_dispatch(priv, &priv->ctrlreq);

            /* If the class driver accepted the configuration, then mark the
             * device state as configured (or not, depending on the
             * configuration).
             */

            if (ret == OK)
              {
                uint8_t cfg = (uint8_t)ctrlreq->value;
                if (cfg != 0)
                  {
                    priv->devstate = DEVSTATE_CONFIGURED;
                    priv->configured = true;
                  }
                else
                  {
                    priv->devstate = DEVSTATE_ADDRESSED;
                    priv->configured = false;
                  }
              }
          }
        else
          {
            usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_BADSETCONFIG), 0);
            priv->stalled = true;
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
        usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_GETSETIF), 0);
        esp32s3_req_dispatch(priv, &priv->ctrlreq);
      }
      break;

    case USB_REQ_SYNCHFRAME:
      /* type:  device-to-host; recipient = endpoint
       * value: 0
       * index: endpoint;
       * len:   2; data = frame number
       */

      {
        usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_SYNCHFRAME), 0);
      }
      break;

    default:
      {
        usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_INVALIDCTRLREQ), 0);
        priv->stalled = true;
      }
      break;
    }
}

/****************************************************************************
 * Name: esp32s3_ep0out_setup
 *
 * Description:
 *   USB Ctrl EP Setup Event. This is logically part of the USB interrupt
 *   handler.  This event occurs when a setup packet is receive on EP0 OUT.
 *
 ****************************************************************************/

static void esp32s3_ep0out_setup(struct esp32s3_usbdev_s *priv)
{
  struct esp32s3_ctrlreq_s ctrlreq;

  /* Verify that a SETUP was received */

  if (priv->ep0state != EP0STATE_SETUP_READY)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_EP0NOSETUP), priv->ep0state);
      return;
    }

  /* Terminate any pending requests */

  esp32s3_req_cancel(&priv->epout[EP0], -EPROTO);
  esp32s3_req_cancel(&priv->epin[EP0], -EPROTO);

  /* Assume NOT stalled */

  priv->epout[EP0].stalled = false;
  priv->epin[EP0].stalled = false;
  priv->stalled = false;

  /* Starting to process a control request - update state */

  priv->ep0state = EP0STATE_SETUP_PROCESS;

  /* And extract the little-endian 16-bit values to host order */

  ctrlreq.type = priv->ctrlreq.type;
  ctrlreq.req = priv->ctrlreq.req;
  ctrlreq.value = GETUINT16(priv->ctrlreq.value);
  ctrlreq.index = GETUINT16(priv->ctrlreq.index);
  ctrlreq.len = GETUINT16(priv->ctrlreq.len);

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        ctrlreq.type, ctrlreq.req, ctrlreq.value, ctrlreq.index,
        ctrlreq.len);

  /* Check for a standard request */

  if ((ctrlreq.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      /* Dispatch any non-standard requests */

      esp32s3_req_dispatch(priv, &priv->ctrlreq);
    }
  else
    {
      /* Handle standard requests. */

      esp32s3_ep0out_stdrequest(priv, &ctrlreq);
    }

  /* Check if the setup processing resulted in a STALL */

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_EP0SETUPSTALLED),
               priv->ep0state);
      esp32s3_ep0_stall(priv);
    }

  /* Reset state/data associated with the SETUP request */

  priv->ep0datlen = 0;
}

/****************************************************************************
 * Name: esp32s3_epout
 *
 * Description:
 *   This is part of the OUT endpoint interrupt processing.  This function
 *   handles the OUT event for a single endpoint.
 *
 ****************************************************************************/

static inline void esp32s3_epout(struct esp32s3_usbdev_s *priv, uint8_t epno)
{
  struct esp32s3_ep_s *privep;

  /* Endpoint 0 is a special case. */

  if (epno == 0)
    {
      privep = &priv->epout[EP0];

      /* In the EP0STATE_DATA_OUT state, we are receiving data into the
       * request buffer.  In that case, we must continue the request
       * processing.
       */

      if (priv->ep0state == EP0STATE_DATA_OUT)
        {
          /* Continue processing data from the EP0 OUT request queue */

          esp32s3_epout_complete(priv, privep);

          /* If we are not actively processing an OUT request, then we need
           * to setup to receive the next control request.
           */

          if (!privep->active)
            {
              esp32s3_ep0out_ctrlsetup(priv);
              priv->ep0state = EP0STATE_IDLE;
            }
        }
    }

  /* For other endpoints, the only possibility is that we are continuing or
   * finishing an OUT request.
   */

  else if (priv->devstate == DEVSTATE_CONFIGURED)
    {
      esp32s3_epout_complete(priv, &priv->epout[epno]);
    }
}

/****************************************************************************
 * Name: esp32s3_epout_interrupt
 *
 * Description:
 *   USB OUT endpoint interrupt handler.  The core generates this interrupt
 *   when there is an interrupt is pending on one of the OUT endpoints of the
 *   core.
 *   The driver must read the OTG DAINT register to determine the exact
 *   number of the OUT endpoint on which the interrupt occurred, and then
 *   read the corresponding OTG DOEPINTx register to determine the exact
 *   cause of the interrupt.
 *
 ****************************************************************************/

static inline void esp32s3_epout_interrupt(struct esp32s3_usbdev_s *priv)
{
  uint32_t daint;
  uint32_t regval;
  uint32_t doepint;
  int epno;

  /* Get the pending, enabled interrupts for the OUT endpoint from the
   * endpoint interrupt status register.
   */

  regval = esp32s3_getreg(ESP32S3_OTG_DAINT);
  regval &= esp32s3_getreg(ESP32S3_OTG_DAINTMSK);
  daint = (regval & OTG_DAINT_OEP_MASK) >> OTG_DAINT_OEP_SHIFT;

  if (daint == 0)
    {
      /* We got an interrupt, but there is no unmasked endpoint that caused
       * it?! When this happens, the interrupt flag never gets cleared and
       * we are stuck in infinite interrupt loop.
       *
       * This shouldn't happen if we are diligent about handling timing
       * issues when masking endpoint interrupts. However, this workaround
       * avoids infinite loop and allows operation to continue normally.
       * It works by clearing each endpoint flags, masked or not.
       */

      regval = esp32s3_getreg(ESP32S3_OTG_DAINT);
      daint = (regval & OTG_DAINT_OEP_MASK) >> OTG_DAINT_OEP_SHIFT;

      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_EPOUTUNEXPECTED),
               (uint16_t)regval);

      epno = 0;
      while (daint)
        {
          if ((daint & 1) != 0)
            {
              regval = esp32s3_getreg(ESP32S3_OTG_DOEPINT(epno));
              uerr("DOEPINT(%d) = %08" PRIx32 "\n", epno, regval);
              esp32s3_putreg(0xff, ESP32S3_OTG_DOEPINT(epno));
            }

          epno++;
          daint >>= 1;
        }

      return;
    }

  /* Process each pending IN endpoint interrupt */

  epno = 0;
  while (daint)
    {
      /* Is an OUT interrupt pending for this endpoint? */

      if ((daint & 1) != 0)
        {
          /* Yes.. get the OUT endpoint interrupt status */

          doepint = esp32s3_getreg(ESP32S3_OTG_DOEPINT(epno));
          doepint &= esp32s3_getreg(ESP32S3_OTG_DOEPMSK);

          /* Transfer completed interrupt.  This interrupt is triggered when
           * esp32s3_rxinterrupt() removes the last packet data from the
           * RxFIFO.
           * In this case, core internally sets the NAK bit for this endpoint
           * to prevent it from receiving any more packets.
           */

          if ((doepint & OTG_DOEPINT_XFRC) != 0)
            {
              usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_EPOUT_XFRC),
                       (uint16_t)doepint);

              /* Clear the bit in DOEPINTn for this interrupt */

              esp32s3_putreg(OTG_DOEPINT_XFRC, ESP32S3_OTG_DOEPINT(epno));

              /* Handle the RX transfer data ready event */

              esp32s3_epout(priv, epno);
            }

          /* Endpoint disabled interrupt (ignored because this interrupt is
           * used in polled mode by the endpoint disable logic).
           */
#if 1
          /* REVISIT: */

          if ((doepint & OTG_DOEPINT_EPDISD) != 0)
            {
              usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_EPOUT_EPDISD),
                       (uint16_t)doepint);

              /* Clear the bit in DOEPINTn for this interrupt */

              esp32s3_putreg(OTG_DOEPINT_EPDISD, ESP32S3_OTG_DOEPINT(epno));
            }
#endif

          /* Setup Phase Done (control EPs) */

          if ((doepint & OTG_DOEPINT_SETUP) != 0)
            {
              usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_EPOUT_SETUP),
                       priv->ep0state);

              /* Handle the receipt of the IN SETUP packets now (OUT setup
               * packet processing may be delayed until the accompanying OUT
               * DATA is received)
               */

              if (priv->ep0state == EP0STATE_SETUP_READY)
                {
                  esp32s3_ep0out_setup(priv);
                }

              esp32s3_putreg(OTG_DOEPINT_SETUP, ESP32S3_OTG_DOEPINT(epno));
            }
        }

      epno++;
      daint >>= 1;
    }
}

/****************************************************************************
 * Name: esp32s3_epin_runtestmode
 *
 * Description:
 *   Execute the test mode setup by the SET FEATURE request
 *
 ****************************************************************************/

static inline void esp32s3_epin_runtestmode(struct esp32s3_usbdev_s *priv)
{
  uint32_t regval = esp32s3_getreg(ESP32S3_OTG_DCTL);
  regval &= OTG_DCTL_TCTL_MASK;
  regval |= (uint32_t)priv->testmode << OTG_DCTL_TCTL_SHIFT;
  esp32s3_putreg(regval, ESP32S3_OTG_DCTL);

  priv->dotest = 0;
  priv->testmode = OTG_TESTMODE_DISABLED;
}

/****************************************************************************
 * Name: esp32s3_epin
 *
 * Description:
 *   This is part of the IN endpoint interrupt processing.  This function
 *   handles the IN event for a single endpoint.
 *
 ****************************************************************************/

static inline void esp32s3_epin(struct esp32s3_usbdev_s *priv, uint8_t epno)
{
  struct esp32s3_ep_s *privep = &priv->epin[epno];

  /* Endpoint 0 is a special case. */

  if (epno == 0)
    {
      /* In the EP0STATE_DATA_IN state, we are sending data from request
       * buffer.  In that case, we must continue the request processing
       */

      if (priv->ep0state == EP0STATE_DATA_IN)
        {
          /* Continue processing data from the EP0 OUT request queue */

          esp32s3_epin_request(priv, privep);

          /* If we are not actively processing an OUT request, then we need
           * to setup to receive the next control request.
           */

          if (!privep->active)
            {
              esp32s3_ep0out_ctrlsetup(priv);
              priv->ep0state = EP0STATE_IDLE;
            }
        }

      /* Test mode is another special case */

      if (priv->dotest)
        {
          esp32s3_epin_runtestmode(priv);
        }
    }

  /* For other endpoints, the only possibility is that we are continuing or
   * finishing an IN request.
   */

  else if (priv->devstate == DEVSTATE_CONFIGURED)
    {
      /* Continue processing data from the endpoint write request queue */

      esp32s3_epin_request(priv, privep);
    }
}

/****************************************************************************
 * Name: esp32s3_epin_txfifoempty
 *
 * Description:
 *   TxFIFO empty interrupt handling
 *
 ****************************************************************************/

static inline void esp32s3_epin_txfifoempty(struct esp32s3_usbdev_s *priv,
                                            int epno)
{
  struct esp32s3_ep_s *privep = &priv->epin[epno];

  /* Continue processing the write request queue.  This may mean sending more
   * data from the existing request or terminating the current requests and
   * (perhaps) starting the IN transfer from the next write request.
   */

  esp32s3_epin_request(priv, privep);
}

/****************************************************************************
 * Name: esp32s3_epin_interrupt
 *
 * Description:
 *   USB IN endpoint interrupt handler.  The core generates this interrupt
 *   when an interrupt is pending on one of the IN endpoints of the core.
 *   The driver must read the OTG DAINT register to determine the exact
 *   number of the IN endpoint on which the interrupt occurred, and then
 *   read the corresponding OTG DIEPINTx register to determine the exact
 *   cause of the interrupt.
 *
 ****************************************************************************/

static inline void esp32s3_epin_interrupt(struct esp32s3_usbdev_s *priv)
{
  uint32_t diepint;
  uint32_t daint;
  uint32_t mask;
  uint32_t empty;
  int epno;

  /* Get the pending, enabled interrupts for the IN endpoint from the
   * endpoint interrupt status register.
   */

  daint = esp32s3_getreg(ESP32S3_OTG_DAINT);
  daint &= esp32s3_getreg(ESP32S3_OTG_DAINTMSK);
  daint &= OTG_DAINT_IEP_MASK;

  if (daint == 0)
    {
      /* We got an interrupt, but there is no unmasked endpoint that caused
       * it?! When this happens, the interrupt flag never gets cleared and
       * we are stuck in infinite interrupt loop.
       *
       * This shouldn't happen if we are diligent about handling timing
       * issues when masking endpoint interrupts. However, this workaround
       * avoids infinite loop and allows operation to continue normally.
       * It works by clearing each endpoint flags, masked or not.
       */

      daint = esp32s3_getreg(ESP32S3_OTG_DAINT);
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_EPINUNEXPECTED),
               (uint16_t)daint);

      daint &= OTG_DAINT_IEP_MASK;
      epno = 0;

      while (daint)
        {
          if ((daint & 1) != 0)
            {
              uerr("DIEPINT(%d) = %08" PRIx32 "\n",
                   epno, esp32s3_getreg(ESP32S3_OTG_DIEPINT(epno)));
              esp32s3_putreg(0xff, ESP32S3_OTG_DIEPINT(epno));
            }

          epno++;
          daint >>= 1;
        }

      return;
    }

  /* Process each pending IN endpoint interrupt */

  epno = 0;
  while (daint)
    {
      /* Is an IN interrupt pending for this endpoint? */

      if ((daint & 1) != 0)
        {
          /* Get IN interrupt mask register.  Bits 0-6 correspond to enabled
           * interrupts as will be found in the DIEPINT interrupt status
           * register.
           */

          mask = esp32s3_getreg(ESP32S3_OTG_DIEPMSK);

          /* Check if the TxFIFO not empty interrupt is enabled for this
           * endpoint in the DIEPMSK register.  Bits n corresponds to
           * endpoint n in the register. That condition corresponds to bit 7
           * of the DIEPINT interrupt status register.  There is no TXFE bit
           * in the mask register, so we fake one here.
           */

          empty = esp32s3_getreg(ESP32S3_OTG_DIEPEMPMSK);
          if ((empty & OTG_DIEPEMPMSK(epno)) != 0)
            {
              mask |= OTG_DIEPINT_TXFE;
            }

          /* Now, read the interrupt status and mask out all disabled
           * interrupts.
           */

          diepint = esp32s3_getreg(ESP32S3_OTG_DIEPINT(epno)) & mask;

          /* Decode and process the enabled, pending interrupts */

          /* Transfer completed interrupt */

          if ((diepint & OTG_DIEPINT_XFRC) != 0)
            {
              usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_EPIN_XFRC),
                       (uint16_t)diepint);

              /* It is possible that logic may be waiting for a the TxFIFO to
               * become empty.  We disable the TxFIFO empty interrupt here;
               * it will be re-enabled if there is still insufficient space
               * in the TxFIFO.
               */

              empty &= ~OTG_DIEPEMPMSK(epno);
              esp32s3_putreg(empty, ESP32S3_OTG_DIEPEMPMSK);
              esp32s3_putreg(OTG_DIEPINT_XFRC, ESP32S3_OTG_DIEPINT(epno));

              /* IN transfer complete */

              esp32s3_epin(priv, epno);
            }

          /* Timeout condition */

          if ((diepint & OTG_DIEPINT_TOC) != 0)
            {
              usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_EPIN_TOC),
                       (uint16_t)diepint);
              esp32s3_putreg(OTG_DIEPINT_TOC, ESP32S3_OTG_DIEPINT(epno));
            }

          /* IN token received when TxFIFO is empty.  Applies to non-periodic
           * IN endpoints only.  This interrupt indicates that an IN token
           * was received when the associated TxFIFO (periodic/non-periodic)
           * was empty. This interrupt is asserted on the endpoint for which
           * the IN token was received.
           */

          if ((diepint & OTG_DIEPINT_ITTXFE) != 0)
            {
              usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_EPIN_ITTXFE),
                       (uint16_t)diepint);
              esp32s3_epin_request(priv, &priv->epin[epno]);
              esp32s3_putreg(OTG_DIEPINT_ITTXFE, ESP32S3_OTG_DIEPINT(epno));
            }

          /* IN endpoint NAK effective (ignored as this used only in polled
           * mode)
           */
#if 0
          if ((diepint & OTG_DIEPINT_INEPNE) != 0)
            {
              usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_EPIN_INEPNE),
                       (uint16_t)diepint);
              esp32s3_putreg(OTG_DIEPINT_INEPNE, ESP32S3_OTG_DIEPINT(epno));
            }
#endif

          /* Endpoint disabled interrupt (ignored as this used only in polled
           * mode)
           */
#if 0
          if ((diepint & OTG_DIEPINT_EPDISD) != 0)
            {
              usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_EPIN_EPDISD),
                       (uint16_t)diepint);
              esp32s3_putreg(OTG_DIEPINT_EPDISD, ESP32S3_OTG_DIEPINT(epno));
            }
#endif

          /* Transmit FIFO empty */

          if ((diepint & OTG_DIEPINT_TXFE) != 0)
            {
              usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_EPIN_TXFE),
                       (uint16_t)diepint);

              /* If we were waiting for TxFIFO to become empty, the we might
               * have both XFRC and TXFE interrupts pending.  Since we do the
               * same thing for both cases, ignore the TXFE if we have
               * already processed the XFRC.
               */

              if ((diepint & OTG_DIEPINT_XFRC) == 0)
                {
                  /* Mask further FIFO empty interrupts.  This will be
                   * re-enabled whenever we need to wait for a FIFO event.
                   */

                  empty &= ~OTG_DIEPEMPMSK(epno);
                  esp32s3_putreg(empty, ESP32S3_OTG_DIEPEMPMSK);

                  /* Handle TxFIFO empty */

                  esp32s3_epin_txfifoempty(priv, epno);
                }

              /* Clear the pending TxFIFO empty interrupt */

              esp32s3_putreg(OTG_DIEPINT_TXFE, ESP32S3_OTG_DIEPINT(epno));
            }
        }

      epno++;
      daint >>= 1;
    }
}

/****************************************************************************
 * Name: esp32s3_resumeinterrupt
 *
 * Description:
 *   Resume/remote wakeup detected interrupt
 *
 ****************************************************************************/

static inline void esp32s3_resumeinterrupt(struct esp32s3_usbdev_s *priv)
{
  uint32_t regval;

  /* Restart the PHY clock and un-gate USB core clock (HCLK) */

#ifdef CONFIG_USBDEV_LOWPOWER
  regval = esp32s3_getreg(ESP32S3_OTG_PCGCCTL);
  regval &= ~(OTG_PCGCCTL_STPPCLK | OTG_PCGCCTL_GATEHCLK);
  esp32s3_putreg(regval, ESP32S3_OTG_PCGCCTL);
#endif

  /* Clear remote wake-up signaling */

  regval = esp32s3_getreg(ESP32S3_OTG_DCTL);
  regval &= ~OTG_DCTL_RWUSIG;
  esp32s3_putreg(regval, ESP32S3_OTG_DCTL);

  /* Restore full power -- whatever that means for this particular board */

  esp32s3_usbsuspend((struct usbdev_s *)priv, true);

  /* Notify the class driver of the resume event */

  if (priv->driver)
    {
      CLASS_RESUME(priv->driver, &priv->usbdev);
    }
}

/****************************************************************************
 * Name: esp32s3_suspendinterrupt
 *
 * Description:
 *   USB suspend interrupt
 *
 ****************************************************************************/

static inline void esp32s3_suspendinterrupt(struct esp32s3_usbdev_s *priv)
{
#ifdef CONFIG_USBDEV_LOWPOWER
  uint32_t regval;
#endif

  /* Notify the class driver of the suspend event */

  if (priv->driver)
    {
      CLASS_SUSPEND(priv->driver, &priv->usbdev);
    }

#ifdef CONFIG_USBDEV_LOWPOWER
  /* OTG_DSTS_SUSPSTS is set as long as the suspend condition is detected on
   * USB.  Check if we are still have the suspend condition, that we are
   * connected to the host, and that we have been configured.
   */

  regval = esp32s3_getreg(ESP32S3_OTG_DSTS);

  if ((regval & OTG_DSTS_SUSPSTS) != 0 && devstate == DEVSTATE_CONFIGURED)
    {
      /* Switch off OTG clocking.  Setting OTG_PCGCCTL_STPPCLK stops the PHY
       * clock.
       */

      regval = esp32s3_getreg(ESP32S3_OTG_PCGCCTL);
      regval |= OTG_PCGCCTL_STPPCLK;
      esp32s3_putreg(regval, ESP32S3_OTG_PCGCCTL);

      /* Setting OTG_PCGCCTL_GATEHCLK gate HCLK to modules other than the AHB
       * Slave and Master and wakeup logic.
       */

      regval |= OTG_PCGCCTL_GATEHCLK;
      esp32s3_putreg(regval, ESP32S3_OTG_PCGCCTL);
    }
#endif

  /* Let the board-specific logic know that we have entered the suspend
   * state
   */

  esp32s3_usbsuspend((struct usbdev_s *)priv, false);
}

/****************************************************************************
 * Name: esp32s3_rxinterrupt
 *
 * Description:
 *   RxFIFO non-empty interrupt.  This interrupt indicates that there is at
 *   least one packet pending to be read from the RxFIFO.
 *
 ****************************************************************************/

static inline void esp32s3_rxinterrupt(struct esp32s3_usbdev_s *priv)
{
  struct esp32s3_ep_s *privep;
  uint32_t regval;
  int bcnt;
  int epphy;

  /* Get the status from the top of the FIFO */

  regval = esp32s3_getreg(ESP32S3_OTG_GRXSTSP);

  /* Decode status fields */

  epphy = (regval & OTG_GRXSTSD_EPNUM_MASK) >> OTG_GRXSTSD_EPNUM_SHIFT;

  if (epphy < ESP32S3_ENDPOINT_NUM)
    {
      privep = &priv->epout[epphy];

      /* Handle the RX event according to the packet status field */

      switch (regval & OTG_GRXSTSD_PKTSTS_MASK)
        {
          /* Global OUT NAK.  This indicate that the global OUT NAK bit has
           * taken effect.
           *
           * PKTSTS = Global OUT NAK, BCNT = 0, EPNUM = Don't Care, DPID =
           * Don't Care.
           */

        case OTG_GRXSTSD_PKTSTS_OUTNAK:
          {
            usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_OUTNAK), 0);
          }
          break;

          /* OUT data packet received. PKTSTS = DataOUT, BCNT = size of the
           * received data OUT packet, EPNUM = EPNUM on which the packet was
           * received, DPID = Actual Data PID.
           */

        case OTG_GRXSTSD_PKTSTS_OUTRECVD:
          {
            usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_OUTRECVD), epphy);
            bcnt = (regval & OTG_GRXSTSD_BCNT_MASK) >>
                   OTG_GRXSTSD_BCNT_SHIFT;
            if (bcnt > 0)
              {
                esp32s3_epout_receive(privep, bcnt);
              }
          }
          break;

          /* OUT transfer completed.  This indicates that an OUT data
           * transfer for the specified OUT endpoint has completed. After
           * this entry is popped from the receive FIFO, the core asserts
           * a Transfer Completed interrupt on the specified OUT endpoint.
           *
           * PKTSTS = Data OUT Transfer Done, BCNT = 0, EPNUM = OUT EP Num on
           * which the data transfer is complete, DPID = Don't Care.
           */

        case OTG_GRXSTSD_PKTSTS_OUTDONE:
          {
            usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_OUTDONE), epphy);
          }
          break;

          /* SETUP transaction completed. This indicates that the Setup stage
           * for the specified endpoint has completed and the Data stage has
           * started. After this entry is popped from the receive FIFO, the
           * core asserts a Setup interrupt on the specified control OUT
           * endpoint (triggers an interrupt).
           *
           * PKTSTS = Setup Stage Done, BCNT = 0, EPNUM = Control EP Num,
           * DPID = Don't Care.
           */

        case OTG_GRXSTSD_PKTSTS_SETUPDONE:
          {
            usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_SETUPDONE), epphy);

            /* Now that the Setup Phase is complete if it was an OUT enable
             * the endpoint (Doing this here prevents the loss of the first
             * FIFO word)
             */

            if (priv->ep0state == EP0STATE_SETUP_OUT)
              {
                /* Clear NAKSTS so that we can receive the data */

                regval = esp32s3_getreg(ESP32S3_OTG_DOEPCTL(0));
                regval |= OTG_DOEPCTL0_CNAK;
                esp32s3_putreg(regval, ESP32S3_OTG_DOEPCTL(0));
              }
          }
          break;

          /* SETUP data packet received.  This indicates that a SETUP packet
           * for the specified endpoint is now available for reading from the
           * receive FIFO.
           *
           * PKTSTS = SETUP, BCNT = 8, EPNUM = Control EP Num, DPID = D0.
           */

        case OTG_GRXSTSD_PKTSTS_SETUPRECVD:
          {
            uint16_t datlen;

            usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_SETUPRECVD), epphy);

            /* Read EP0 setup data.  NOTE: If multiple SETUP packets are
             * received, the last one overwrites the previous setup packets
             * and only that last SETUP packet will be processed.
             */

            esp32s3_rxfifo_read(&priv->epout[EP0],
                                (uint8_t *)&priv->ctrlreq,
                                USB_SIZEOF_CTRLREQ);

            /* Was this an IN or an OUT SETUP packet.  If it is an OUT SETUP,
             * then we need to wait for the completion of the data phase to
             * process the setup command.  If it is an IN SETUP packet, then
             * we must processing the command BEFORE we enter the DATA phase.
             *
             * If the data associated with the OUT SETUP packet is zero
             * length, then, of course, we don't need to wait.
             */

            datlen = GETUINT16(priv->ctrlreq.len);
            if (USB_REQ_ISOUT(priv->ctrlreq.type) && datlen > 0)
              {
                priv->ep0state = EP0STATE_SETUP_OUT;
              }
            else
              {
                /* We can process the setup data as soon as SETUP done word
                 * is popped of the RxFIFO.
                 */

                priv->ep0state = EP0STATE_SETUP_READY;
              }
          }
          break;

        default:
          {
            usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_INVALIDPARMS),
                     (regval & OTG_GRXSTSD_PKTSTS_MASK) >>
                     OTG_GRXSTSD_PKTSTS_SHIFT);
          }
          break;
        }
    }
}

/****************************************************************************
 * Name: esp32s3_enuminterrupt
 *
 * Description:
 *   Enumeration done interrupt
 *
 ****************************************************************************/

static inline void esp32s3_enuminterrupt(struct esp32s3_usbdev_s *priv)
{
  uint32_t regval;

  /* Activate EP0 */

  esp32s3_ep0in_activate();

  /* Set USB turn-around time for the full speed device with internal PHY
   * interface.
   */

  regval = esp32s3_getreg(ESP32S3_OTG_GUSBCFG);
  regval &= ~OTG_GUSBCFG_TRDT_MASK;
  regval |= OTG_GUSBCFG_TRDT(6);
  esp32s3_putreg(regval, ESP32S3_OTG_GUSBCFG);
}

/****************************************************************************
 * Name: esp32s3_isocininterrupt
 *
 * Description:
 *   Incomplete isochronous IN transfer interrupt.  Assertion of the
 *   incomplete isochronous IN transfer interrupt indicates an incomplete
 *   isochronous IN transfer on at least one of the isochronous IN endpoints.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_ISOCHRONOUS
static inline void esp32s3_isocininterrupt(struct esp32s3_usbdev_s *priv)
{
  int i;

  /* The application must read the endpoint control register for all
   * isochronous IN endpoints to detect endpoints with incomplete IN data
   * transfers.
   */

  for (i = 0; i < ESP32S3_ENDPOINT_NUM; i++)
    {
      /* Is this an isochronous IN endpoint? */

      privep = &priv->epin[i];
      if (privep->eptype != USB_EP_ATTR_XFER_ISOC)
        {
          /* No... keep looking */

          continue;
        }

      /* Is there an active read request on the isochronous OUT endpoint? */

      if (!privep->active)
        {
          /* No.. the endpoint is not actively transmitting data */

          continue;
        }

      /* Check if this is the endpoint that had the incomplete transfer */

      regaddr = ESP32S3_OTG_DIEPCTL(privep->epphy);
      doepctl = esp32s3_getreg(regaddr);
      dsts = esp32s3_getreg(ESP32S3_OTG_DSTS);

      /* EONUM = 0:even frame, 1:odd frame SOFFN = Frame number of the
       * received SOF
       */

      eonum = ((doepctl & OTG_DIEPCTL_EONUM) != 0);
      soffn = ((dsts & OTG_DSTS_SOFFN0) != 0);

      if (eonum != soffn)
        {
          /* Not this endpoint */

          continue;
        }

      /* For isochronous IN endpoints with incomplete transfers, the
       * application must discard the data in the memory and disable the
       * endpoint.
       */

      esp32s3_req_complete(privep, -EIO);
#  warning "Will clear OTG_DIEPCTL_USBAEP too"
      esp32s3_epin_disable(privep);
      break;
    }
}
#endif

/****************************************************************************
 * Name: esp32s3_isocoutinterrupt
 *
 * Description:
 *   Incomplete periodic transfer interrupt
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_ISOCHRONOUS
static inline void esp32s3_isocoutinterrupt(struct esp32s3_usbdev_s *priv)
{
  struct esp32s3_ep_s *privep;
  struct esp32s3_req_s *privreq;
  uint32_t regaddr;
  uint32_t doepctl;
  uint32_t dsts;
  bool eonum;
  bool soffn;

  /* When it receives an IISOOXFR interrupt, the application must read the
   * control registers of all isochronous OUT endpoints to determine which
   * endpoints had an incomplete transfer in the current microframe. An
   * endpoint transfer is incomplete if both the following conditions are
   * true:
   *
   * DOEPCTLx:EONUM = DSTS:SOFFN[0], and
   * DOEPCTLx:EPENA = 1
   */

  for (i = 0; i < ESP32S3_ENDPOINT_NUM; i++)
    {
      /* Is this an isochronous OUT endpoint? */

      privep = &priv->epout[i];
      if (privep->eptype != USB_EP_ATTR_XFER_ISOC)
        {
          /* No... keep looking */

          continue;
        }

      /* Is there an active read request on the isochronous OUT endpoint? */

      if (!privep->active)
        {
          /* No.. the endpoint is not actively transmitting data */

          continue;
        }

      /* Check if this is the endpoint that had the incomplete transfer */

      regaddr = ESP32S3_OTG_DOEPCTL(privep->epphy);
      doepctl = esp32s3_getreg(regaddr);
      dsts = esp32s3_getreg(ESP32S3_OTG_DSTS);

      /* EONUM = 0:even frame, 1:odd frame
       * SOFFN = Frame number of the received SOF
       */

      eonum = ((doepctl & OTG_DOEPCTL_EONUM) != 0);
      soffn = ((dsts & OTG_DSTS_SOFFN0) != 0);

      if (eonum != soffn)
        {
          /* Not this endpoint */

          continue;
        }

      /* For isochronous OUT endpoints with incomplete transfers, the
       * application must discard the data in the memory and disable the
       * endpoint.
       */

      esp32s3_req_complete(privep, -EIO);
#  warning "Will clear OTG_DOEPCTL_USBAEP too"
      esp32s3_epout_disable(privep);
      break;
    }
}
#endif

/****************************************************************************
 * Name: esp32s3_sessioninterrupt
 *
 * Description:
 *   Session request/new session detected interrupt
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_VBUSSENSING
static inline void esp32s3_sessioninterrupt(struct esp32s3_usbdev_s *priv)
{
#  warning "Missing logic"
}
#endif

/****************************************************************************
 * Name: esp32s3_otginterrupt
 *
 * Description:
 *   OTG interrupt
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_VBUSSENSING
static inline void esp32s3_otginterrupt(struct esp32s3_usbdev_s *priv)
{
  uint32_t regval;

  /* Check for session end detected */

  regval = esp32s3_getreg(ESP32S3_OTG_GOTGINT);
  if ((regval & OTG_GOTGINT_SEDET) != 0)
    {
#  warning "Missing logic"
    }

  /* Clear OTG interrupt */

  esp32s3_putreg(regval, ESP32S3_OTG_GOTGINT);
}
#endif

/****************************************************************************
 * Name: esp32s3_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 ****************************************************************************/

static int esp32s3_usbinterrupt(int irq, void *context, void *arg)
{
  /* At present, there is only a single OTG device support. Hence it is
   * pre-allocated as g_otghsdev.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple
   * devices.
   */

  struct esp32s3_usbdev_s *priv = &g_otghsdev;
  uint32_t regval;
  uint32_t reserved;

  usbtrace(TRACE_INTENTRY(ESP32S3_TRACEINTID_USB), 0);

  /* Assure that we are in device mode */

  DEBUGASSERT((esp32s3_getreg(ESP32S3_OTG_GINTSTS) & OTG_GINTSTS_CMOD) ==
              OTG_GINTSTS_DEVMODE);

  /* Get the state of all enabled interrupts.  We will do this repeatedly
   * some interrupts (like RXFLVL) will generate additional interrupting
   * events.
   */

  for (; ; )
    {
      /* Get the set of pending, un-masked interrupts */

      regval = esp32s3_getreg(ESP32S3_OTG_GINTSTS);
      reserved = (regval & OTG_GINT_RESERVED);
      regval &= esp32s3_getreg(ESP32S3_OTG_GINTMSK);

      /* With out modifying the reserved bits, acknowledge all **Writable**
       * pending irqs we will service below.
       */

      esp32s3_putreg(((regval | reserved) & OTG_GINT_RC_W1),
                     ESP32S3_OTG_GINTSTS);

      /* Break out of the loop when there are no further pending
       * (and unmasked) interrupts to be processes.
       */

      if (regval == 0)
        {
          break;
        }

      usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_INTPENDING),
               (uint16_t)regval);

      /* OUT endpoint interrupt. The core sets this bit to indicate that an
       * interrupt is pending on one of the OUT endpoints of the core.
       */

      if ((regval & OTG_GINT_OEP) != 0)
        {
          usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_EPOUT),
                   (uint16_t)regval);
          esp32s3_epout_interrupt(priv);
        }

      /* IN endpoint interrupt.  The core sets this bit to indicate that an
       * interrupt is pending on one of the IN endpoints of the core.
       */

      if ((regval & OTG_GINT_IEP) != 0)
        {
          usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_EPIN),
                   (uint16_t)regval);
          esp32s3_epin_interrupt(priv);
        }

      /* Host/device mode mismatch error interrupt */

#ifdef CONFIG_DEBUG_USB
      if ((regval & OTG_GINT_MMIS) != 0)
        {
          usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_MISMATCH),
                   (uint16_t)regval);
        }
#endif

      /* Resume/remote wakeup detected interrupt */

      if ((regval & OTG_GINT_WKUP) != 0)
        {
          usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_WAKEUP),
                   (uint16_t)regval);
          esp32s3_resumeinterrupt(priv);
        }

      /* USB suspend interrupt */

      if ((regval & OTG_GINT_USBSUSP) != 0)
        {
          usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_SUSPEND),
                   (uint16_t)regval);
          esp32s3_suspendinterrupt(priv);
        }

      /* Start of frame interrupt */

#ifdef CONFIG_USBDEV_SOFINTERRUPT
      if ((regval & OTG_GINT_SOF) != 0)
        {
          usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_SOF),
                   (uint16_t)regval);
        }
#endif

      /* RxFIFO non-empty interrupt.  Indicates that there is at least one
       * packet pending to be read from the RxFIFO.
       */

      if ((regval & OTG_GINT_RXFLVL) != 0)
        {
          usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_RXFIFO),
                   (uint16_t)regval);
          esp32s3_rxinterrupt(priv);
        }

      /* USB reset interrupt */

      if ((regval & (OTG_GINT_USBRST | OTG_GINT_RSTDET)) != 0)
        {
          usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_DEVRESET),
                   (uint16_t)regval);

          /* Perform the device reset */

          esp32s3_usbreset(priv);
          usbtrace(TRACE_INTEXIT(ESP32S3_TRACEINTID_USB), 0);
          return OK;
        }

      /* Enumeration done interrupt */

      if ((regval & OTG_GINT_ENUMDNE) != 0)
        {
          usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_ENUMDNE),
                   (uint16_t)regval);
          esp32s3_enuminterrupt(priv);
        }

      /* Incomplete isochronous IN transfer interrupt.  When the core finds
       * non-empty any of the isochronous IN endpoint FIFOs scheduled for the
       * current frame non-empty, the core generates an IISOIXFR interrupt.
       */

#ifdef CONFIG_USBDEV_ISOCHRONOUS
      if ((regval & OTG_GINT_IISOIXFR) != 0)
        {
          usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_IISOIXFR),
                   (uint16_t)regval);
          esp32s3_isocininterrupt(priv);
        }

      /* Incomplete isochronous OUT transfer.  For isochronous OUT endpoints,
       * the XFRC interrupt may not always be asserted. If the core drops
       * isochronous OUT data packets, the application could fail to detect
       * the XFRC interrupt.  The incomplete Isochronous OUT data interrupt
       * indicates that an XFRC interrupt was not asserted on at least one of
       * the isochronous OUT endpoints. At this point, the endpoint with the
       * incomplete transfer remains enabled, but no active transfers remain
       * in progress on this endpoint on the USB.
       */

      if ((regval & OTG_GINT_IISOOXFR) != 0)
        {
          usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_IISOOXFR),
                   (uint16_t)regval);
          esp32s3_isocoutinterrupt(priv);
        }
#endif

      /* Session request/new session detected interrupt */

#ifdef CONFIG_USBDEV_VBUSSENSING
      if ((regval & OTG_GINT_SRQ) != 0)
        {
          usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_SRQ),
                   (uint16_t)regval);
          esp32s3_sessioninterrupt(priv);
        }

      /* OTG interrupt */

      if ((regval & OTG_GINT_OTG) != 0)
        {
          usbtrace(TRACE_INTDECODE(ESP32S3_TRACEINTID_OTG),
                   (uint16_t)regval);
          esp32s3_otginterrupt(priv);
        }
#endif
    }

  usbtrace(TRACE_INTEXIT(ESP32S3_TRACEINTID_USB), 0);
  return OK;
}

/****************************************************************************
 * Endpoint operations
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_enablegonak
 *
 * Description:
 *   Enable global OUT NAK mode
 *
 ****************************************************************************/

static void esp32s3_enablegonak(struct esp32s3_ep_s *privep)
{
  uint32_t regval;

  /* First, make sure that there is no GNOAKEFF interrupt pending. */

#if 0
  esp32s3_putreg(OTG_GINT_GONAKEFF, ESP32S3_OTG_GINTSTS);
#endif

  /* Enable Global OUT NAK mode in the core. */

  regval = esp32s3_getreg(ESP32S3_OTG_DCTL);
  regval |= OTG_DCTL_SGONAK;
  esp32s3_putreg(regval, ESP32S3_OTG_DCTL);

#if 0
  /* Wait for the GONAKEFF interrupt that indicates that the OUT NAK mode is
   * in effect.  When the interrupt handler pops the OUTNAK word from the
   * RxFIFO, the core sets the GONAKEFF interrupt.
   */

  while ((esp32s3_getreg(ESP32S3_OTG_GINTSTS) & OTG_GINT_GONAKEFF) == 0);
  esp32s3_putreg(OTG_GINT_GONAKEFF, ESP32S3_OTG_GINTSTS);

#else
  /* Since we are in the interrupt handler, we cannot wait inline for the
   * GONAKEFF because it cannot occur until service the RXFLVL global
   * interrupt and pop the OUTNAK word from the RxFIFO.
   *
   * Perhaps it is sufficient to wait for Global OUT NAK status to be
   * reported in OTG DCTL register?
   */

  while ((esp32s3_getreg(ESP32S3_OTG_DCTL) & OTG_DCTL_GONSTS) == 0);
#endif
}

/****************************************************************************
 * Name: esp32s3_disablegonak
 *
 * Description:
 *   Disable global OUT NAK mode
 *
 ****************************************************************************/

static void esp32s3_disablegonak(struct esp32s3_ep_s *privep)
{
  uint32_t regval;

  /* Set the "Clear the Global OUT NAK bit" to disable global OUT NAK mode */

  regval = esp32s3_getreg(ESP32S3_OTG_DCTL);
  regval |= OTG_DCTL_CGONAK;
  esp32s3_putreg(regval, ESP32S3_OTG_DCTL);
}

/****************************************************************************
 * Name: esp32s3_epout_configure
 *
 * Description:
 *   Configure an OUT endpoint, making it usable
 *
 * Input Parameters:
 *   privep    - a pointer to an internal endpoint structure
 *   eptype    - The type of the endpoint
 *   maxpacket - The max packet size of the endpoint
 *
 ****************************************************************************/

static int esp32s3_epout_configure(struct esp32s3_ep_s *privep,
                                   uint8_t eptype,
                                   uint16_t maxpacket)
{
  uint32_t mpsiz;
  uint32_t regaddr;
  uint32_t regval;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);

  /* For EP0, the packet size is encoded */

  if (privep->epphy == EP0)
    {
      DEBUGASSERT(eptype == USB_EP_ATTR_XFER_CONTROL);

      /* Map the size in bytes to the encoded value in the register */

      switch (maxpacket)
        {
        case 8:
          mpsiz = OTG_DOEPCTL0_MPSIZ_8;
          break;

        case 16:
          mpsiz = OTG_DOEPCTL0_MPSIZ_16;
          break;

        case 32:
          mpsiz = OTG_DOEPCTL0_MPSIZ_32;
          break;

        case 64:
          mpsiz = OTG_DOEPCTL0_MPSIZ_64;
          break;

        default:
          uerr("Unsupported maxpacket: %d\n", maxpacket);
          return -EINVAL;
        }
    }

  /* For other endpoints, the packet size is in bytes */

  else
    {
      mpsiz = (maxpacket << OTG_DOEPCTL_MPSIZ_SHIFT);
    }

  /* If the endpoint is already active don't change the endpoint control
   * register.
   */

  regaddr = ESP32S3_OTG_DOEPCTL(privep->epphy);
  regval = esp32s3_getreg(regaddr);
  if ((regval & OTG_DOEPCTL_USBAEP) == 0)
    {
      if (regval & OTG_DOEPCTL_NAKSTS)
        {
          regval |= OTG_DOEPCTL_CNAK;
        }

      regval &= ~(OTG_DOEPCTL_MPSIZ_MASK | OTG_DOEPCTL_EPTYP_MASK);
      regval |= mpsiz;
      regval |= (eptype << OTG_DOEPCTL_EPTYP_SHIFT);
      regval |= (OTG_DOEPCTL_SD0PID | OTG_DOEPCTL_USBAEP);
      esp32s3_putreg(regval, regaddr);

      /* Save the endpoint configuration */

      privep->ep.maxpacket = maxpacket;
      privep->eptype       = eptype;
      privep->stalled      = false;
      privep->active       = false;
      privep->zlp          = false;
    }

  /* Enable the interrupt for this endpoint */

  regval = esp32s3_getreg(ESP32S3_OTG_DAINTMSK);
  regval |= OTG_DAINT_OEP(privep->epphy);
  esp32s3_putreg(regval, ESP32S3_OTG_DAINTMSK);
  return OK;
}

/****************************************************************************
 * Name: esp32s3_epin_configure
 *
 * Description:
 *   Configure an IN endpoint, making it usable
 *
 * Input Parameters:
 *   privep    - a pointer to an internal endpoint structure
 *   eptype    - The type of the endpoint
 *   maxpacket - The max packet size of the endpoint
 *
 ****************************************************************************/

static int esp32s3_epin_configure(struct esp32s3_ep_s *privep,
                                  uint8_t eptype,
                                  uint16_t maxpacket)
{
  uint32_t mpsiz;
  uint32_t regaddr;
  uint32_t regval;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);

  /* For EP0, the packet size is encoded */

  if (privep->epphy == EP0)
    {
      DEBUGASSERT(eptype == USB_EP_ATTR_XFER_CONTROL);

      /* Map the size in bytes to the encoded value in the register */

      switch (maxpacket)
        {
        case 8:
          mpsiz = OTG_DIEPCTL0_MPSIZ_8;
          break;

        case 16:
          mpsiz = OTG_DIEPCTL0_MPSIZ_16;
          break;

        case 32:
          mpsiz = OTG_DIEPCTL0_MPSIZ_32;
          break;

        case 64:
          mpsiz = OTG_DIEPCTL0_MPSIZ_64;
          break;

        default:
          uerr("Unsupported maxpacket: %d\n", maxpacket);
          return -EINVAL;
        }
    }

  /* For other endpoints, the packet size is in bytes */

  else
    {
      mpsiz = (maxpacket << OTG_DIEPCTL_MPSIZ_SHIFT);
    }

  /* If the endpoint is already active don't change the endpoint control
   * register.
   */

  regaddr = ESP32S3_OTG_DIEPCTL(privep->epphy);
  regval = esp32s3_getreg(regaddr);
  if ((regval & OTG_DIEPCTL_USBAEP) == 0)
    {
      if (regval & OTG_DIEPCTL_NAKSTS)
        {
          regval |= OTG_DIEPCTL_CNAK;
        }

      regval &= ~(OTG_DIEPCTL_MPSIZ_MASK | OTG_DIEPCTL_EPTYP_MASK |
                  OTG_DIEPCTL_TXFNUM_MASK);
      regval |= mpsiz;
      regval |= (eptype << OTG_DIEPCTL_EPTYP_SHIFT);
      regval |= (privep->epphy << OTG_DIEPCTL_TXFNUM_SHIFT);
      regval |= (OTG_DIEPCTL_SD0PID | OTG_DIEPCTL_USBAEP);
      esp32s3_putreg(regval, regaddr);

      /* Save the endpoint configuration */

      privep->ep.maxpacket = maxpacket;
      privep->eptype       = eptype;
      privep->stalled      = false;
      privep->active       = false;
      privep->zlp          = false;
    }

  /* Enable the interrupt for this endpoint */

  regval = esp32s3_getreg(ESP32S3_OTG_DAINTMSK);
  regval |= OTG_DAINT_IEP(privep->epphy);
  esp32s3_putreg(regval, ESP32S3_OTG_DAINTMSK);

  return OK;
}

/****************************************************************************
 * Name: esp32s3_ep_configure
 *
 * Description:
 *   Configure endpoint, making it usable
 *
 * Input Parameters:
 *   ep   - the struct usbdev_ep_s instance obtained from allocep()
 *   desc - A struct usb_epdesc_s instance describing the endpoint
 *   last - true if this this last endpoint to be configured.  Some hardware
 *          needs to take special action when all of the endpoints have been
 *          configured.
 *
 ****************************************************************************/

static int esp32s3_ep_configure(struct usbdev_ep_s *ep,
                                const struct usb_epdesc_s *desc,
                                bool last)
{
  struct esp32s3_ep_s *privep = (struct esp32s3_ep_s *)ep;
  uint16_t maxpacket;
  uint8_t eptype;
  int ret;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);
  DEBUGASSERT(desc->addr == ep->eplog);

  /* Initialize EP capabilities */

  maxpacket = GETUINT16(desc->mxpacketsize);
  eptype = desc->attr & USB_EP_ATTR_XFERTYPE_MASK;

  /* Setup Endpoint Control Register */

  if (privep->isin)
    {
      ret = esp32s3_epin_configure(privep, eptype, maxpacket);
    }
  else
    {
      ret = esp32s3_epout_configure(privep, eptype, maxpacket);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32s3_ep0_configure
 *
 * Description:
 *   Reset Usb engine
 *
 ****************************************************************************/

static void esp32s3_ep0_configure(struct esp32s3_usbdev_s *priv)
{
  /* Enable EP0 IN and OUT */

  esp32s3_epin_configure(&priv->epin[EP0], USB_EP_ATTR_XFER_CONTROL,
                         ESP32S3_CDCACM_EP0MAXPACKET);
  esp32s3_epout_configure(&priv->epout[EP0], USB_EP_ATTR_XFER_CONTROL,
                          ESP32S3_CDCACM_EP0MAXPACKET);
}

/****************************************************************************
 * Name: esp32s3_epout_disable
 *
 * Description:
 *   Disable an OUT endpoint will no longer be used
 *
 ****************************************************************************/

static void esp32s3_epout_disable(struct esp32s3_ep_s *privep)
{
  uint32_t regaddr;
  uint32_t regval;
  irqstate_t flags;

  usbtrace(TRACE_EPDISABLE, privep->epphy);

  /* Is this an IN or an OUT endpoint */

  /* Before disabling any OUT endpoint, the application must enable Global
   * OUT NAK mode in the core.
   */

  flags = enter_critical_section();
  esp32s3_enablegonak(privep);

  /* Disable the required OUT endpoint by setting the EPDIS and SNAK bits int
   * DOECPTL register.
   */

  regaddr = ESP32S3_OTG_DOEPCTL(privep->epphy);
  regval = esp32s3_getreg(regaddr);
  regval &= ~OTG_DOEPCTL_USBAEP;
  regval |= (OTG_DOEPCTL_EPDIS | OTG_DOEPCTL_SNAK);
  esp32s3_putreg(regval, regaddr);

  /* Wait for the EPDISD interrupt which indicates that the OUT endpoint is
   * completely disabled.
   */

#if 0    /* Doesn't happen */
  regaddr = ESP32S3_OTG_DOEPINT(privep->epphy);
  while ((esp32s3_getreg(regaddr) & OTG_DOEPINT_EPDISD) == 0);
#else
  /* REVISIT: */

  up_udelay(10);
#endif

  /* Clear the EPDISD interrupt indication */

  esp32s3_putreg(OTG_DOEPINT_EPDISD, ESP32S3_OTG_DOEPINT(privep->epphy));

  /* Then disable the Global OUT NAK mode to continue receiving data from
   * other non-disabled OUT endpoints.
   */

  esp32s3_disablegonak(privep);

  /* Disable endpoint interrupts */

  regval = esp32s3_getreg(ESP32S3_OTG_DAINTMSK);
  regval &= ~OTG_DAINT_OEP(privep->epphy);
  esp32s3_putreg(regval, ESP32S3_OTG_DAINTMSK);

  /* Cancel any queued read requests */

  esp32s3_req_cancel(privep, -ESHUTDOWN);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32s3_epin_disable
 *
 * Description:
 *   Disable an IN endpoint when it will no longer be used
 *
 ****************************************************************************/

static void esp32s3_epin_disable(struct esp32s3_ep_s *privep)
{
  uint32_t regaddr;
  uint32_t regval;
  irqstate_t flags;

  usbtrace(TRACE_EPDISABLE, privep->epphy);

  /* After USB reset, the endpoint will already be deactivated by the
   * hardware.
   * Trying to disable again will just hang in the wait.
   */

  regaddr = ESP32S3_OTG_DIEPCTL(privep->epphy);
  regval = esp32s3_getreg(regaddr);
  if ((regval & OTG_DIEPCTL_USBAEP) == 0)
    {
      return;
    }

  /* This INEPNE wait logic is suggested by reference manual, but seems to
   * get stuck to infinite loop.
   */

#if 0
  /* Make sure that there is no pending IPEPNE interrupt (because we are to
   * poll this bit below).
   */

  esp32s3_putreg(OTG_DIEPINT_INEPNE, ESP32S3_OTG_DIEPINT(privep->epphy));

  /* Set the endpoint in NAK mode */

  regaddr = ESP32S3_OTG_DIEPCTL(privep->epphy);
  regval = esp32s3_getreg(regaddr);
  regval &= ~OTG_DIEPCTL_USBAEP;
  regval |= (OTG_DIEPCTL_EPDIS | OTG_DIEPCTL_SNAK);
  esp32s3_putreg(regval, regaddr);

  /* Wait for the INEPNE interrupt that indicates that we are now in NAK
   * mode
   */

  regaddr = ESP32S3_OTG_DIEPINT(privep->epphy);
  while ((esp32s3_getreg(regaddr) & OTG_DIEPINT_INEPNE) == 0);

  /* Clear the INEPNE interrupt indication */

  esp32s3_putreg(OTG_DIEPINT_INEPNE, regaddr);
#endif

  /* Deactivate and disable the endpoint by setting the EPDIS and SNAK bits
   * the DIEPCTLx register.
   */

  flags = enter_critical_section();
  regaddr = ESP32S3_OTG_DIEPCTL(privep->epphy);
  regval = esp32s3_getreg(regaddr);
  regval &= ~OTG_DIEPCTL_USBAEP;
  regval |= (OTG_DIEPCTL_EPDIS | OTG_DIEPCTL_SNAK);
  esp32s3_putreg(regval, regaddr);

  /* Wait for the EPDISD interrupt which indicates that the IN endpoint is
   * completely disabled.
   */

  regaddr = ESP32S3_OTG_DIEPINT(privep->epphy);
  while ((esp32s3_getreg(regaddr) & OTG_DIEPINT_EPDISD) == 0);

  /* Clear the EPDISD interrupt indication */

  regval = esp32s3_getreg(regaddr);
  regval |= OTG_DIEPINT_EPDISD;
  esp32s3_putreg(regval, regaddr);

  /* Flush any data remaining in the TxFIFO */

  esp32s3_txfifo_flush(OTG_GRSTCTL_TXFNUM_D(privep->epphy));

  /* Disable endpoint interrupts */

  regval = esp32s3_getreg(ESP32S3_OTG_DAINTMSK);
  regval &= ~OTG_DAINT_IEP(privep->epphy);
  esp32s3_putreg(regval, ESP32S3_OTG_DAINTMSK);

  /* Cancel any queued write requests */

  esp32s3_req_cancel(privep, -ESHUTDOWN);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32s3_ep_disable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 ****************************************************************************/

static int esp32s3_ep_disable(struct usbdev_ep_s *ep)
{
  struct esp32s3_ep_s *privep = (struct esp32s3_ep_s *)ep;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPDISABLE, privep->epphy);

  /* Is this an IN or an OUT endpoint */

  if (privep->isin)
    {
      /* Disable the IN endpoint */

      esp32s3_epin_disable(privep);
    }
  else
    {
      /* Disable the OUT endpoint */

      esp32s3_epout_disable(privep);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32s3_ep_allocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 ****************************************************************************/

static struct usbdev_req_s *esp32s3_ep_allocreq(struct usbdev_ep_s *ep)
{
  struct esp32s3_req_s *privreq;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif

  usbtrace(TRACE_EPALLOCREQ, ((struct esp32s3_ep_s *)ep)->epphy);

  privreq = (struct esp32s3_req_s *)kmm_zalloc(sizeof(struct esp32s3_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  return &privreq->req;
}

/****************************************************************************
 * Name: esp32s3_ep_freereq
 *
 * Description:
 *   Free an I/O request
 *
 ****************************************************************************/

static void esp32s3_ep_freereq(struct usbdev_ep_s *ep,
                               struct usbdev_req_s *req)
{
  struct esp32s3_req_s *privreq = (struct esp32s3_req_s *)req;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  usbtrace(TRACE_EPFREEREQ, ((struct esp32s3_ep_s *)ep)->epphy);
  kmm_free(privreq);
}

/****************************************************************************
 * Name: esp32s3_ep_allocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void *esp32s3_ep_allocbuffer(struct usbdev_ep_s *ep, uint16_t bytes)
{
  usbtrace(TRACE_EPALLOCBUFFER, ((struct esp32s3_ep_s *)ep)->epphy);

#  ifdef CONFIG_USBDEV_DMAMEMORY
  return usbdev_dma_alloc(bytes);
#  else
  return kmm_malloc(bytes);
#  endif
}
#endif

/****************************************************************************
 * Name: esp32s3_ep_freebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void esp32s3_ep_freebuffer(struct usbdev_ep_s *ep, void *buf)
{
  usbtrace(TRACE_EPALLOCBUFFER, ((struct esp32s3_ep_s *)ep)->epphy);

#  ifdef CONFIG_USBDEV_DMAMEMORY
  usbdev_dma_free(buf);
#  else
  kmm_free(buf);
#  endif
}
#endif

/****************************************************************************
 * Name: esp32s3_ep_submit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 ****************************************************************************/

static int esp32s3_ep_submit(struct usbdev_ep_s *ep,
                             struct usbdev_req_s *req)
{
  struct esp32s3_req_s *privreq = (struct esp32s3_req_s *)req;
  struct esp32s3_ep_s *privep = (struct esp32s3_ep_s *)ep;
  struct esp32s3_usbdev_s *priv;
  irqstate_t flags;
  int ret = OK;

  /* Some sanity checking */

#ifdef CONFIG_DEBUG_FEATURES
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_INVALIDPARMS), 0);
      uinfo("req=%p callback=%p buf=%p ep=%p\n",
            req, req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, privep->epphy);
  priv = privep->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv->driver)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_NOTCONFIGURED),
               priv->usbdev.speed);
      return -ESHUTDOWN;
    }
#endif

  /* Handle the request from the class driver */

  req->result = -EINPROGRESS;
  req->xfrd = 0;

  /* Disable Interrupts */

  flags = enter_critical_section();

  /* If we are stalled, then drop all requests on the floor */

  if (privep->stalled)
    {
      ret = -EBUSY;
    }
  else
    {
      /* Add the new request to the request queue for the endpoint. */

      if (esp32s3_req_addlast(privep, privreq) && !privep->active)
        {
          /* If a request was added to an IN endpoint, then attempt to send
           * the request data buffer now.
           */

          if (privep->isin)
            {
              usbtrace(TRACE_INREQQUEUED(privep->epphy), privreq->req.len);

              /* If the endpoint is not busy with another write request, then
               * process the newly received write request now.
               */

              if (!privep->active)
                {
                  esp32s3_epin_request(priv, privep);
                }
            }

          /* If the request was added to an OUT endpoint, then attempt to
           * setup a read into the request data buffer now (this will,
           * of course, fail if there is already a read in place).
           */

          else
            {
              usbtrace(TRACE_OUTREQQUEUED(privep->epphy), privreq->req.len);
              esp32s3_epout_request(priv, privep);
            }
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: esp32s3_ep_cancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 ****************************************************************************/

static int esp32s3_ep_cancel(struct usbdev_ep_s *ep,
                             struct usbdev_req_s *req)
{
  struct esp32s3_ep_s *privep = (struct esp32s3_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPCANCEL, privep->epphy);

  flags = enter_critical_section();

  /* FIXME: if the request is the first, then we need to flush the EP
   * otherwise just remove it from the list but ... all other
   * implementations cancel all requests ...
   */

  esp32s3_req_cancel(privep, -ESHUTDOWN);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: esp32s3_epout_setstall
 *
 * Description:
 *   Stall an OUT endpoint
 *
 ****************************************************************************/

static int esp32s3_epout_setstall(struct esp32s3_ep_s *privep)
{
#if 1
  /* This implementation follows the requirements from the ESP32-S3 reference
   * manual.
   */

  uint32_t regaddr;
  uint32_t regval;

  /* Put the core in the Global OUT NAK mode */

  esp32s3_enablegonak(privep);

  /* Disable and STALL the OUT endpoint by setting the EPDIS and STALL bits
   * in the DOECPTL register.
   */

  regaddr = ESP32S3_OTG_DOEPCTL(privep->epphy);
  regval = esp32s3_getreg(regaddr);
  regval |= (OTG_DOEPCTL_EPDIS | OTG_DOEPCTL_STALL);
  esp32s3_putreg(regval, regaddr);

  /* Wait for the EPDISD interrupt which indicates that the OUT endpoint is
   * completely disabled.
   */

#  if 0    /* Doesn't happen */
  regaddr = ESP32S3_OTG_DOEPINT(privep->epphy);
  while ((esp32s3_getreg(regaddr) & OTG_DOEPINT_EPDISD) == 0);
#  else
  /* REVISIT: */

  up_udelay(10);
#  endif

  /* Disable Global OUT NAK mode */

  esp32s3_disablegonak(privep);

  /* The endpoint is now stalled */

  privep->stalled = true;
  return OK;
#else
  /* This implementation follows the STMicro code example. */

  /* REVISIT: */

  uint32_t regaddr;
  uint32_t regval;

  /* Stall the OUT endpoint by setting the STALL bit in the DOECPTL
   * register.
   */

  regaddr = ESP32S3_OTG_DOEPCTL(privep->epphy);
  regval = esp32s3_getreg(regaddr);
  regval |= OTG_DOEPCTL_STALL;
  esp32s3_putreg(regval, regaddr);

  /* The endpoint is now stalled */

  privep->stalled = true;
  return OK;
#endif
}

/****************************************************************************
 * Name: esp32s3_epin_setstall
 *
 * Description:
 *   Stall an IN endpoint
 *
 ****************************************************************************/

static int esp32s3_epin_setstall(struct esp32s3_ep_s *privep)
{
  uint32_t regaddr;
  uint32_t regval;

  /* Get the IN endpoint device control register */

  regaddr = ESP32S3_OTG_DIEPCTL(privep->epphy);
  regval = esp32s3_getreg(regaddr);

  /* Then stall the endpoint */

  regval |= OTG_DIEPCTL_STALL;
  esp32s3_putreg(regval, regaddr);

  /* The endpoint is now stalled */

  privep->stalled = true;
  return OK;
}

/****************************************************************************
 * Name: esp32s3_ep_setstall
 *
 * Description:
 *   Stall an endpoint
 *
 ****************************************************************************/

static int esp32s3_ep_setstall(struct esp32s3_ep_s *privep)
{
  usbtrace(TRACE_EPSTALL, privep->epphy);

  /* Is this an IN endpoint? */

  if (privep->isin == 1)
    {
      return esp32s3_epin_setstall(privep);
    }
  else
    {
      return esp32s3_epout_setstall(privep);
    }
}

/****************************************************************************
 * Name: esp32s3_ep_clrstall
 *
 * Description:
 *   Resume a stalled endpoint
 *
 ****************************************************************************/

static int esp32s3_ep_clrstall(struct esp32s3_ep_s *privep)
{
  uint32_t regaddr;
  uint32_t regval;
  uint32_t stallbit;
  uint32_t data0bit;

  usbtrace(TRACE_EPRESUME, privep->epphy);

  /* Is this an IN endpoint? */

  if (privep->isin == 1)
    {
      /* Clear the stall bit in the IN endpoint device control register */

      regaddr = ESP32S3_OTG_DIEPCTL(privep->epphy);
      stallbit = OTG_DIEPCTL_STALL;
      data0bit = OTG_DIEPCTL_SD0PID;
    }
  else
    {
      /* Clear the stall bit in the IN endpoint device control register */

      regaddr = ESP32S3_OTG_DOEPCTL(privep->epphy);
      stallbit = OTG_DOEPCTL_STALL;
      data0bit = OTG_DOEPCTL_SD0PID;
    }

  /* Clear the stall bit */

  regval = esp32s3_getreg(regaddr);
  regval &= ~stallbit;

  /* Set the DATA0 pid for interrupt and bulk endpoints */

  if (privep->eptype == USB_EP_ATTR_XFER_INT ||
      privep->eptype == USB_EP_ATTR_XFER_BULK)
    {
      /* Writing this bit sets the DATA0 PID */

      regval |= data0bit;
    }

  esp32s3_putreg(regval, regaddr);

  /* The endpoint is no longer stalled */

  privep->stalled = false;
  return OK;
}

/****************************************************************************
 * Name: esp32s3_ep_stall
 *
 * Description:
 *   Stall or resume an endpoint
 *
 ****************************************************************************/

static int esp32s3_ep_stall(struct usbdev_ep_s *ep, bool resume)
{
  struct esp32s3_ep_s *privep = (struct esp32s3_ep_s *)ep;
  irqstate_t flags;
  int ret;

  /* Set or clear the stall condition as requested */

  flags = enter_critical_section();
  if (resume)
    {
      ret = esp32s3_ep_clrstall(privep);
    }
  else
    {
      ret = esp32s3_ep_setstall(privep);
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: esp32s3_ep0_stall
 *
 * Description:
 *   Stall endpoint 0
 *
 ****************************************************************************/

static void esp32s3_ep0_stall(struct esp32s3_usbdev_s *priv)
{
  esp32s3_epin_setstall(&priv->epin[EP0]);
  esp32s3_epout_setstall(&priv->epout[EP0]);
  priv->stalled = true;
  esp32s3_ep0out_ctrlsetup(priv);
}

/****************************************************************************
 * Device operations
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_ep_alloc
 *
 * Description:
 *   Allocate an endpoint matching the parameters.
 *
 * Input Parameters:
 *   eplog  - 7-bit logical endpoint number (direction bit ignored).  Zero
 *            means that any endpoint matching the other requirements will
 *            suffice.  The assigned endpoint can be found in the eplog
 *            field.
 *   in     - true: IN (device-to-host) endpoint requested
 *   eptype - Endpoint type.  One of {USB_EP_ATTR_XFER_ISOC,
 *            USB_EP_ATTR_XFER_BULK, USB_EP_ATTR_XFER_INT}
 *
 ****************************************************************************/

static struct usbdev_ep_s *esp32s3_ep_alloc(struct usbdev_s *dev,
                                            uint8_t eplog, bool in,
                                            uint8_t eptype)
{
  struct esp32s3_usbdev_s *priv = (struct esp32s3_usbdev_s *)dev;
  uint8_t epavail;
  irqstate_t flags;
  int epphy;
  int epno = 0;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)eplog);

  /* Ignore any direction bits in the logical address */

  epphy = USB_EPNO(eplog);

  /* Get the set of available endpoints depending on the direction */

  flags = enter_critical_section();
  epavail = priv->epavail[in];

  /* A physical address of 0 means that any endpoint will do */

  if (epphy > 0)
    {
      /* Otherwise, we will return the endpoint structure only for the
       * requested 'logical' endpoint.  All of the other checks will still be
       * performed.
       *
       * First, verify that the logical endpoint is in the range supported by
       * the hardware.
       */

      if (epphy >= ESP32S3_ENDPOINT_NUM)
        {
          usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_BADEPNO),
                   (uint16_t)epphy);
          return NULL;
        }

      /* Remove all of the candidate endpoints from the bitset except for the
       * this physical endpoint number.
       */

      epavail &= (1 << epphy);
    }

  /* Is there an available endpoint? */

  if (epavail)
    {
      /* Yes.. Select the lowest numbered endpoint in the set of available
       * endpoints.
       */

      for (epno = 1; epno < ESP32S3_ENDPOINT_NUM; epno++)
        {
          uint8_t bit = 1 << epno;
          if ((epavail & bit) != 0)
            {
              /* Mark the endpoint no longer available */

              priv->epavail[in] &= ~(1 << epno);

              /* And return the pointer to the standard endpoint structure */

              leave_critical_section(flags);
              return in ? &priv->epin[epno].ep : &priv->epout[epno].ep;
            }
        }

      /* We should not get here */
    }

  usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_NOEP), (uint16_t)eplog);
  leave_critical_section(flags);
  return NULL;
}

/****************************************************************************
 * Name: esp32s3_ep_free
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 ****************************************************************************/

static void esp32s3_ep_free(struct usbdev_s *dev,
                            struct usbdev_ep_s *ep)
{
  struct esp32s3_usbdev_s *priv = (struct esp32s3_usbdev_s *)dev;
  struct esp32s3_ep_s *privep = (struct esp32s3_ep_s *)ep;
  irqstate_t flags;

  usbtrace(TRACE_DEVFREEEP, (uint16_t)privep->epphy);

  if (priv && privep)
    {
      /* Mark the endpoint as available */

      flags = enter_critical_section();
      priv->epavail[privep->isin] |= (1 << privep->epphy);
      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: esp32s3_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 ****************************************************************************/

static int esp32s3_getframe(struct usbdev_s *dev)
{
  uint32_t regval;

  usbtrace(TRACE_DEVGETFRAME, 0);

  /* Return the last frame number of the last SOF detected by the hardware */

  regval = esp32s3_getreg(ESP32S3_OTG_DSTS);
  return (int)((regval & OTG_DSTS_SOFFN_MASK) >> OTG_DSTS_SOFFN_SHIFT);
}

/****************************************************************************
 * Name: esp32s3_wakeup
 *
 * Description:
 *   Exit suspend mode.
 *
 ****************************************************************************/

static int esp32s3_wakeup(struct usbdev_s *dev)
{
  struct esp32s3_usbdev_s *priv = (struct esp32s3_usbdev_s *)dev;
  uint32_t regval;
  irqstate_t flags;

  usbtrace(TRACE_DEVWAKEUP, 0);

  /* Is wakeup enabled? */

  flags = enter_critical_section();
  if (priv->wakeup)
    {
      /* Yes... is the core suspended? */

      regval = esp32s3_getreg(ESP32S3_OTG_DSTS);
      if ((regval & OTG_DSTS_SUSPSTS) != 0)
        {
          /* Re-start the PHY clock and un-gate USB core clock (HCLK) */

#ifdef CONFIG_USBDEV_LOWPOWER
          regval = esp32s3_getreg(ESP32S3_OTG_PCGCCTL);
          regval &= ~(OTG_PCGCCTL_STPPCLK | OTG_PCGCCTL_GATEHCLK);
          esp32s3_putreg(regval, ESP32S3_OTG_PCGCCTL);
#endif
          /* Activate Remote wakeup signaling */

          regval = esp32s3_getreg(ESP32S3_OTG_DCTL);
          regval |= OTG_DCTL_RWUSIG;
          esp32s3_putreg(regval, ESP32S3_OTG_DCTL);
          up_mdelay(5);
          regval &= ~OTG_DCTL_RWUSIG;
          esp32s3_putreg(regval, ESP32S3_OTG_DCTL);
        }
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: esp32s3_selfpowered
 *
 * Description:
 *   Sets/clears the device self-powered feature
 *
 ****************************************************************************/

static int esp32s3_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct esp32s3_usbdev_s *priv = (struct esp32s3_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/****************************************************************************
 * Name: esp32s3_pullup
 *
 * Description:
 *   Software-controlled connect to/disconnect from USB host
 *
 ****************************************************************************/

static int esp32s3_pullup(struct usbdev_s *dev, bool enable)
{
  uint32_t regval;

  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  irqstate_t flags = enter_critical_section();
  regval = esp32s3_getreg(ESP32S3_OTG_DCTL);
  if (enable)
    {
      /* Connect the device by clearing the soft disconnect bit in the DCTL
       * register.
       */

      regval &= ~OTG_DCTL_SDIS;
    }
  else
    {
      /* Disconnect the device by setting the soft disconnect bit in the DCTL
       * register.
       */

      regval |= OTG_DCTL_SDIS;
    }

  esp32s3_putreg(regval, ESP32S3_OTG_DCTL);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: esp32s3_setaddress
 *
 * Description:
 *   Set the devices USB address
 *
 ****************************************************************************/

static void esp32s3_setaddress(struct esp32s3_usbdev_s *priv,
                               uint16_t address)
{
  uint32_t regval;

  /* Set the device address in the DCFG register */

  regval = esp32s3_getreg(ESP32S3_OTG_DCFG);
  regval &= ~OTG_DCFG_DAD_MASK;
  regval |= ((uint32_t)address << OTG_DCFG_DAD_SHIFT);
  esp32s3_putreg(regval, ESP32S3_OTG_DCFG);

  /* Are we now addressed? (i.e., do we have a non-NULL device address?) */

  if (address != 0)
    {
      priv->devstate = DEVSTATE_ADDRESSED;
      priv->addressed = true;
    }
  else
    {
      priv->devstate = DEVSTATE_DEFAULT;
      priv->addressed = false;
    }
}

/****************************************************************************
 * Name: esp32s3_txfifo_flush
 *
 * Description:
 *   Flush the specific TX fifo.
 *
 ****************************************************************************/

static int esp32s3_txfifo_flush(uint32_t txfnum)
{
  uint32_t regval;
  uint32_t timeout;

  /* Initiate the TX FIFO flush operation */

  regval = OTG_GRSTCTL_TXFFLSH | txfnum;
  esp32s3_putreg(regval, ESP32S3_OTG_GRSTCTL);

  /* Wait for the FLUSH to complete */

  for (timeout = 0; timeout < ESP32S3_FLUSH_DELAY; timeout++)
    {
      regval = esp32s3_getreg(ESP32S3_OTG_GRSTCTL);
      if ((regval & OTG_GRSTCTL_TXFFLSH) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);
  return OK;
}

/****************************************************************************
 * Name: esp32s3_rxfifo_flush
 *
 * Description:
 *   Flush the RX fifo.
 *
 ****************************************************************************/

static int esp32s3_rxfifo_flush(void)
{
  uint32_t regval;
  uint32_t timeout;

  /* Initiate the RX FIFO flush operation */

  esp32s3_putreg(OTG_GRSTCTL_RXFFLSH, ESP32S3_OTG_GRSTCTL);

  /* Wait for the FLUSH to complete */

  for (timeout = 0; timeout < ESP32S3_FLUSH_DELAY; timeout++)
    {
      regval = esp32s3_getreg(ESP32S3_OTG_GRSTCTL);
      if ((regval & OTG_GRSTCTL_RXFFLSH) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);
  return OK;
}

/****************************************************************************
 * Name: esp32s3_swinitialize
 *
 * Description:
 *   Initialize all driver data structures.
 *
 ****************************************************************************/

static void esp32s3_swinitialize(struct esp32s3_usbdev_s *priv)
{
  struct esp32s3_ep_s *privep;
  int i;

  /* Initialize the device state structure */

  memset(priv, 0, sizeof(struct esp32s3_usbdev_s));

  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->epin[EP0].ep;

  priv->epavail[0] = ESP32S3_EP_AVAILABLE;
  priv->epavail[1] = ESP32S3_EP_AVAILABLE;

  /* Initialize the endpoint lists */

  for (i = 0; i < ESP32S3_ENDPOINT_NUM; i++)
    {
      /* Set endpoint operations, reference to driver structure (not really
       * necessary because there is only one controller), and the physical
       * endpoint number (which is just the index to the endpoint).
       */

      privep = &priv->epin[i];
      privep->ep.ops = &g_epops;
      privep->dev = priv;
      privep->isin = 1;

      /* The index, i, is the physical endpoint address; Map this to a
       * logical endpoint address usable by the class driver.
       */

      privep->epphy = i;
      privep->ep.eplog = ESP32S3_EPPHYIN2LOG(i);

      /* Control until endpoint is activated */

      privep->eptype = USB_EP_ATTR_XFER_CONTROL;
      privep->ep.maxpacket = ESP32S3_CDCACM_EP0MAXPACKET;
    }

  /* Initialize the endpoint lists */

  for (i = 0; i < ESP32S3_ENDPOINT_NUM; i++)
    {
      /* Set endpoint operations, reference to driver structure (not really
       * necessary because there is only one controller), and the physical
       * endpoint number (which is just the index to the endpoint).
       */

      privep = &priv->epout[i];
      privep->ep.ops = &g_epops;
      privep->dev = priv;

      /* The index, i, is the physical endpoint address; Map this to a
       * logical endpoint address usable by the class driver.
       */

      privep->epphy = i;
      privep->ep.eplog = ESP32S3_EPPHYOUT2LOG(i);

      /* Control until endpoint is activated */

      privep->eptype = USB_EP_ATTR_XFER_CONTROL;
      privep->ep.maxpacket = ESP32S3_CDCACM_EP0MAXPACKET;
    }
}

/****************************************************************************
 * Name: esp32s3_hwinitialize
 *
 * Description:
 *   Configure the OTG  core for operation.
 *
 ****************************************************************************/

static void esp32s3_hwinitialize(struct esp32s3_usbdev_s *priv)
{
  uint32_t regval;
  uint32_t timeout;
  uint32_t address;
  int i;

  /* Enable USB clock and disable reset */

  regval  = esp32s3_getreg(SYSTEM_PERIP_CLK_EN0_REG);
  regval |= SYSTEM_USB_CLK_EN;
  esp32s3_putreg(regval, SYSTEM_PERIP_CLK_EN0_REG);

  regval  = esp32s3_getreg(SYSTEM_PERIP_RST_EN0_REG);
  regval &= ~SYSTEM_USB_RST;
  esp32s3_putreg(regval, SYSTEM_PERIP_RST_EN0_REG);

  /* Configure USB PHY */

  regval  = esp32s3_getreg(RTC_CNTL_RTC_USB_CONF_REG);
  regval |= RTC_CNTL_SW_HW_USB_PHY_SEL | RTC_CNTL_SW_USB_PHY_SEL;
  esp32s3_putreg(regval, RTC_CNTL_RTC_USB_CONF_REG);

  regval  = esp32s3_getreg(USB_WRAP_OTG_CONF_REG);
  regval &= ~(USB_WRAP_PHY_SEL | USB_WRAP_DM_PULLUP |
              USB_WRAP_DP_PULLDOWN | USB_WRAP_DM_PULLDOWN);
  regval |= USB_WRAP_PAD_PULL_OVERRIDE | USB_WRAP_DP_PULLUP |
            USB_WRAP_USB_PAD_ENABLE;
  esp32s3_putreg(regval, USB_WRAP_OTG_CONF_REG);

  /* At start-up the core is in FS mode. */

  /* Disable global interrupts by clearing the GINTMASK bit in the GAHBCFG
   * register; Set the TXFELVL bit in the GAHBCFG register so that TxFIFO
   * interrupts will occur when the TxFIFO is truly empty (not just half
   * full).
   */

  esp32s3_putreg(OTG_GAHBCFG_TXFELVL, ESP32S3_OTG_GAHBCFG);

  /* Common USB OTG core initialization */

  /* Reset after a PHY select and set Host mode.  First, wait for AHB master
   * IDLE state.
   */

  for (timeout = 0; timeout < ESP32S3_READY_DELAY; timeout++)
    {
      up_udelay(3);
      regval = esp32s3_getreg(ESP32S3_OTG_GRSTCTL);
      if ((regval & OTG_GRSTCTL_AHBIDL) != 0)
        {
          break;
        }
    }

  /* Then perform the core soft reset. */

  regval = esp32s3_getreg(ESP32S3_OTG_GRSTCTL);
  regval |= OTG_GRSTCTL_CSRST;
  esp32s3_putreg(regval, ESP32S3_OTG_GRSTCTL);
  for (timeout = 0; timeout < ESP32S3_READY_DELAY; timeout++)
    {
      regval = esp32s3_getreg(ESP32S3_OTG_GRSTCTL);
      if ((regval & OTG_GRSTCTL_CSRST) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);

  /* Deactivate the power down */

  /* Detection Enable when set */

  regval = esp32s3_getreg(ESP32S3_OTG_GCCFG);

  regval |= OTG_GCCFG_PWRDWN;

#ifdef CONFIG_USBDEV_VBUSSENSING
  regval |= OTG_GCCFG_VBDEN;
#endif

  esp32s3_putreg(regval, ESP32S3_OTG_GCCFG);
  up_mdelay(20);

  /* When VBUS sensing is not used we need to force the B session valid */

#ifndef CONFIG_USBDEV_VBUSSENSING
  regval = esp32s3_getreg(ESP32S3_OTG_GOTGCTL);
  regval |= (OTG_GOTGCTL_BVALOEN | OTG_GOTGCTL_BVALOVAL);
  esp32s3_putreg(regval, ESP32S3_OTG_GOTGCTL);
#endif

  /* Force Device Mode */

  regval = esp32s3_getreg(ESP32S3_OTG_GUSBCFG);
  regval &= ~OTG_GUSBCFG_FHMOD;
  regval |= OTG_GUSBCFG_FDMOD;
  esp32s3_putreg(regval, ESP32S3_OTG_GUSBCFG);
  up_mdelay(50);

  /* Initialize device mode */

  /* Restart the PHY Clock */

  esp32s3_putreg(0, ESP32S3_OTG_PCGCCTL);

  /* Device configuration register */

  regval = esp32s3_getreg(ESP32S3_OTG_DCFG);
  regval &= ~OTG_DCFG_PFIVL_MASK;
  regval |= OTG_DCFG_PFIVL_80PCT;
  esp32s3_putreg(regval, ESP32S3_OTG_DCFG);

  /* Set PHY speed */

  regval = esp32s3_getreg(ESP32S3_OTG_DCFG);
  regval &= ~OTG_DCFG_DSPD_MASK;
  regval |= OTG_DCFG_DSPD_FS;
  esp32s3_putreg(regval, ESP32S3_OTG_DCFG);

  /* Set Rx FIFO size */

  esp32s3_putreg(ESP32S3_RXFIFO_WORDS, ESP32S3_OTG_GRXFSIZ);

#if ESP32S3_ENDPOINT_NUM > 0
  address = ESP32S3_RXFIFO_WORDS;
  regval = (address << OTG_DIEPTXF0_TX0FD_SHIFT) |
    (ESP32S3_EP0_TXFIFO_WORDS << OTG_DIEPTXF0_TX0FSA_SHIFT);
  esp32s3_putreg(regval, ESP32S3_OTG_DIEPTXF0);
#endif

#if ESP32S3_ENDPOINT_NUM > 1
  address += ESP32S3_EP0_TXFIFO_WORDS;
  regval = (address << OTG_DIEPTXF_INEPTXSA_SHIFT) |
    (ESP32S3_EP1_TXFIFO_WORDS << OTG_DIEPTXF_INEPTXFD_SHIFT);
  esp32s3_putreg(regval, ESP32S3_OTG_DIEPTXF(1));
#endif

#if ESP32S3_ENDPOINT_NUM > 2
  address += ESP32S3_EP1_TXFIFO_WORDS;
  regval = (address << OTG_DIEPTXF_INEPTXSA_SHIFT) |
    (ESP32S3_EP2_TXFIFO_WORDS << OTG_DIEPTXF_INEPTXFD_SHIFT);
  esp32s3_putreg(regval, ESP32S3_OTG_DIEPTXF(2));
#endif

#if ESP32S3_ENDPOINT_NUM > 3
  address += ESP32S3_EP2_TXFIFO_WORDS;
  regval = (address << OTG_DIEPTXF_INEPTXSA_SHIFT) |
    (ESP32S3_EP3_TXFIFO_WORDS << OTG_DIEPTXF_INEPTXFD_SHIFT);
  esp32s3_putreg(regval, ESP32S3_OTG_DIEPTXF(3));
#endif

#if ESP32S3_ENDPOINT_NUM > 4
  address += ESP32S3_EP3_TXFIFO_WORDS;
  regval = (address << OTG_DIEPTXF_INEPTXSA_SHIFT) |
    (ESP32S3_EP4_TXFIFO_WORDS << OTG_DIEPTXF_INEPTXFD_SHIFT);
  esp32s3_putreg(regval, ESP32S3_OTG_DIEPTXF(4));
#endif

#if ESP32S3_ENDPOINT_NUM > 5
  address += ESP32S3_EP4_TXFIFO_WORDS;
  regval = (address << OTG_DIEPTXF_INEPTXSA_SHIFT) |
    (ESP32S3_EP5_TXFIFO_WORDS << OTG_DIEPTXF_INEPTXFD_SHIFT);
  esp32s3_putreg(regval, ESP32S3_OTG_DIEPTXF(5));
#endif

#if ESP32S3_ENDPOINT_NUM > 6
  address += ESP32S3_EP5_TXFIFO_WORDS;
  regval = (address << OTG_DIEPTXF_INEPTXSA_SHIFT) |
    (ESP32S3_EP6_TXFIFO_WORDS << OTG_DIEPTXF_INEPTXFD_SHIFT);
  esp32s3_putreg(regval, ESP32S3_OTG_DIEPTXF(6));
#endif

  /* Flush the FIFOs */

  esp32s3_txfifo_flush(OTG_GRSTCTL_TXFNUM_DALL);
  esp32s3_rxfifo_flush();

  /* Clear all pending Device Interrupts */

  esp32s3_putreg(0, ESP32S3_OTG_DIEPMSK);
  esp32s3_putreg(0, ESP32S3_OTG_DOEPMSK);
  esp32s3_putreg(0, ESP32S3_OTG_DIEPEMPMSK);
  esp32s3_putreg(0xffffffff, ESP32S3_OTG_DAINT);
  esp32s3_putreg(0, ESP32S3_OTG_DAINTMSK);

  /* Configure all IN endpoints */

  for (i = 0; i < ESP32S3_ENDPOINT_NUM; i++)
    {
      regval = esp32s3_getreg(ESP32S3_OTG_DIEPCTL(i));
      if ((regval & OTG_DIEPCTL_EPENA) != 0)
        {
          /* The endpoint is already enabled */

          regval = OTG_DIEPCTL_EPDIS | OTG_DIEPCTL_SNAK;
        }
      else
        {
          regval = 0;
        }

      esp32s3_putreg(regval, ESP32S3_OTG_DIEPCTL(i));
      esp32s3_putreg(0, ESP32S3_OTG_DIEPTSIZ(i));
      esp32s3_putreg(0xff, ESP32S3_OTG_DIEPINT(i));
    }

  /* Configure all OUT endpoints */

  for (i = 0; i < ESP32S3_ENDPOINT_NUM; i++)
    {
      regval = esp32s3_getreg(ESP32S3_OTG_DOEPCTL(i));
      if ((regval & OTG_DOEPCTL_EPENA) != 0)
        {
          /* The endpoint is already enabled */

          regval = OTG_DOEPCTL_EPDIS | OTG_DOEPCTL_SNAK;
        }
      else
        {
          regval = 0;
        }

      esp32s3_putreg(regval, ESP32S3_OTG_DOEPCTL(i));
      esp32s3_putreg(0, ESP32S3_OTG_DOEPTSIZ(i));
      esp32s3_putreg(0xff, ESP32S3_OTG_DOEPINT(i));
    }

  /* Disable all interrupts. */

  esp32s3_putreg(0, ESP32S3_OTG_GINTMSK);

  /* Clear any pending USB_OTG Interrupts */

  esp32s3_putreg(0xffffffff, ESP32S3_OTG_GOTGINT);

  /* Clear any pending interrupts */

  regval = esp32s3_getreg(ESP32S3_OTG_GINTSTS);
  regval &= OTG_GINT_RESERVED;
  esp32s3_putreg(regval | OTG_GINT_RC_W1, ESP32S3_OTG_GINTSTS);

  /* Enable the interrupts in the INTMSK */

  regval = (OTG_GINT_RXFLVL | OTG_GINT_USBSUSP | OTG_GINT_ENUMDNE |
            OTG_GINT_IEP | OTG_GINT_OEP | OTG_GINT_USBRST);

#ifdef CONFIG_USBDEV_ISOCHRONOUS
  regval |= (OTG_GINT_IISOIXFR | OTG_GINT_IISOOXFR);
#endif

#ifdef CONFIG_USBDEV_SOFINTERRUPT
  regval |= OTG_GINT_SOF;
#endif

#ifdef CONFIG_USBDEV_VBUSSENSING
  regval |= (OTG_GINT_OTG | OTG_GINT_SRQ);
#endif

#ifdef CONFIG_DEBUG_USB
  regval |= OTG_GINT_MMIS;
#endif

  esp32s3_putreg(regval, ESP32S3_OTG_GINTMSK);

  /* Enable the USB global interrupt by setting GINTMSK in the global OTG AHB
   * configuration register; Set the TXFELVL bit in the GAHBCFG register so
   * that TxFIFO interrupts will occur when the TxFIFO is truly empty (not
   * just half full).
   */

  esp32s3_putreg(OTG_GAHBCFG_GINTMSK | OTG_GAHBCFG_TXFELVL,
                 ESP32S3_OTG_GAHBCFG);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_usbinitialize
 *
 * Description:
 *   Initialize USB hardware.
 *
 * Assumptions:
 * - This function is called very early in the initialization sequence
 *
 ****************************************************************************/

void xtensa_usbinitialize(void)
{
  /* At present, there is only a single OTG device support. Hence it is
   * pre-allocated as g_otghsdev.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple
   * devices.
   */

  struct esp32s3_usbdev_s *priv = &g_otghsdev;
  int ret;

  usbtrace(TRACE_DEVINIT, 0);

  /* Here we assume that: 1. GPIOA and OTG peripheral clocking has already
   * been enabled as part of the boot sequence. 2. Board-specific logic has
   * already enabled other board specific GPIOs for things like soft pull-up,
   * VBUS sensing, power controls, and over- current detection.
   */

  /* Configure OTG alternate function pins */

  esp32s3_configgpio(USB_IOMUX_DM, DRIVE_3);
  esp32s3_configgpio(USB_IOMUX_DP, DRIVE_3);

  /**
   * USB_OTG_IDDIG_IN_IDX:     connected connector is mini-B side
   * USB_SRP_BVALID_IN_IDX:    HIGH to force USB device mode
   * USB_OTG_VBUSVALID_IN_IDX: receiving a valid Vbus from device
   * USB_OTG_AVALID_IN_IDX:    HIGH to force USB host mode
   */

  esp32s3_gpio_matrix_in(MATRIX_DETACH_IN_LOW_HIGH,
                         USB_OTG_IDDIG_IN_IDX,
                         false);
  esp32s3_gpio_matrix_in(MATRIX_DETACH_IN_LOW_HIGH,
                         USB_SRP_BVALID_IN_IDX,
                         false);
  esp32s3_gpio_matrix_in(MATRIX_DETACH_IN_LOW_HIGH,
                         USB_OTG_VBUSVALID_IN_IDX,
                         false);
  esp32s3_gpio_matrix_in(MATRIX_DETACH_IN_LOW_PIN,
                         USB_OTG_AVALID_IN_IDX,
                         false);

  /* Uninitialize the hardware so that we know that we are starting from a
   * known state.
   */

  xtensa_usbuninitialize();

  /* Initialie the driver data structure */

  esp32s3_swinitialize(priv);

  /* Attach the OTG interrupt handler */

  priv->cpu = up_cpu_index();
  priv->cpuint = esp32s3_setup_irq(priv->cpu, ESP32S3_PERIPH_USB,
                                   1, ESP32S3_CPUINT_LEVEL);
  DEBUGASSERT(priv->cpuint >= 0);

  ret = irq_attach(ESP32S3_IRQ_USB, esp32s3_usbinterrupt, NULL);
  DEBUGASSERT(ret == 0);

  /* Initialize the USB OTG core */

  esp32s3_hwinitialize(priv);

  /* Disconnect device */

  esp32s3_pullup(&priv->usbdev, false);

  /* Reset/Re-initialize the USB hardware */

  esp32s3_usbreset(priv);

  /* Enable USB controller interrupts at the NVIC */

  up_enable_irq(ESP32S3_IRQ_USB);
}

/****************************************************************************
 * Name: xtensa_usbuninitialize
 ****************************************************************************/

void xtensa_usbuninitialize(void)
{
  /* At present, there is only a single OTG device support. Hence it is
   * pre-allocated as g_otghsdev.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple
   * devices.
   */

  struct esp32s3_usbdev_s *priv = &g_otghsdev;
  irqstate_t flags;
  int i;

  usbtrace(TRACE_DEVUNINIT, 0);

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  /* Disconnect device */

  flags = enter_critical_section();
  esp32s3_pullup(&priv->usbdev, false);
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable and detach IRQs */

  up_disable_irq(ESP32S3_IRQ_USB);
  irq_detach(ESP32S3_IRQ_USB);

  /* Disable all endpoint interrupts */

  for (i = 0; i < ESP32S3_ENDPOINT_NUM; i++)
    {
      esp32s3_putreg(0xff, ESP32S3_OTG_DIEPINT(i));
      esp32s3_putreg(0xff, ESP32S3_OTG_DOEPINT(i));
    }

  esp32s3_putreg(0, ESP32S3_OTG_DIEPMSK);
  esp32s3_putreg(0, ESP32S3_OTG_DOEPMSK);
  esp32s3_putreg(0, ESP32S3_OTG_DIEPEMPMSK);
  esp32s3_putreg(0, ESP32S3_OTG_DAINTMSK);
  esp32s3_putreg(0xffffffff, ESP32S3_OTG_DAINT);

  /* Flush the FIFOs */

  esp32s3_txfifo_flush(OTG_GRSTCTL_TXFNUM_DALL);
  esp32s3_rxfifo_flush();

  /* TODO: Turn off USB power and clocking */

  priv->devstate = DEVSTATE_DEFAULT;
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
  /* At present, there is only a single OTG device support. Hence it is
   * pre-allocated as g_otghsdev.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple
   * devices.
   */

  struct esp32s3_usbdev_s *priv = &g_otghsdev;
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->disconnect || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  priv->driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &priv->usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_BINDFAILED), (uint16_t)-ret);
      priv->driver = NULL;
    }
  else
    {
      /* Enable USB controller interrupts */

      up_enable_irq(ESP32S3_IRQ_USB);

      /* FIXME: nothing seems to call DEV_CONNECT(), but we need to set the
       *        RS bit to enable the controller.  It kind of makes sense to
       *        do this after the class has bound to us...
       * GEN:   This bug is really in the class driver.  It should make the
       *        soft connect when it is ready to be enumerated.  I have
       *        added that logic to the class drivers but left this logic
       *        here.
       */

      esp32s3_pullup(&priv->usbdev, true);
      priv->usbdev.speed = USB_SPEED_FULL;
    }

  return ret;
}

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver.If the USB device is connected to a
 *   USB host, it will first disconnect().  The driver is also requested to
 *   unbind() and clean up any device state, before this procedure finally
 *   returns.
 *
 ****************************************************************************/

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  /* At present, there is only a single OTG device support. Hence it is
   * pre-allocated as g_otghsdev.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple
   * devices.
   */

  struct esp32s3_usbdev_s *priv = &g_otghsdev;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (driver != priv->driver)
    {
      usbtrace(TRACE_DEVERROR(ESP32S3_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Reset the hardware and cancel all requests.  All requests must be
   * canceled while the class driver is still bound.
   */

  flags = enter_critical_section();
  esp32s3_usbreset(priv);
  leave_critical_section(flags);

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable USB controller interrupts */

  flags = enter_critical_section();
  up_disable_irq(ESP32S3_IRQ_USB);

  /* Disconnect device */

  esp32s3_pullup(&priv->usbdev, false);

  /* Unhook the driver */

  priv->driver = NULL;
  leave_critical_section(flags);

  return OK;
}

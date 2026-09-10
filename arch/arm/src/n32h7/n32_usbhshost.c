/****************************************************************************
 * arch/arm/src/n32h7/n32_usbhshost.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_devaddr.h>
#include <nuttx/usb/usbhost_trace.h>

#include <nuttx/irq.h>

#include "chip.h"
#include <arch/board/board.h>

#include "arm_internal.h"
#include "n32_gpio.h"

#if defined(CONFIG_USBHOST) && (defined(CONFIG_N32H7_USBHS1_HOST) || \
    defined(CONFIG_N32H7_USBHS2_HOST))

/* N32_USBHS_BASE and N32_IRQ_USBHS are defined before n32_usbhs.h is
 * included so that the register address macros and the static inline
 * clock/PHY helpers in that header expand for the correct instance.
 * The USBHS GPIO pin definitions (GPIO_USBHS_*) are provided by board.h.
 */

#if defined(CONFIG_N32H7_USBHS1_HOST)
#  define N32_IRQ_USBHS        N32_IRQ_USBHS1_HS
#  define N32_USBHS_BASE       N32_USBCTRL1_BASE
#  define N32_USBHS_WRAPPER_BASE N32_USBCTRL1_WRAPPER_BASE
#  define GPIO_USBHS_DM         GPIO_USBHS1_DM
#  define GPIO_USBHS_DP         GPIO_USBHS1_DP
#elif defined(CONFIG_N32H7_USBHS2_HOST)
#  define N32_IRQ_USBHS        N32_IRQ_USBHS2_HS
#  define N32_USBHS_BASE       N32_USBCTRL2_BASE
#  define N32_USBHS_WRAPPER_BASE N32_USBCTRL2_WRAPPER_BASE
#  define GPIO_USBHS_DM         GPIO_USBHS2_DM
#  define GPIO_USBHS_DP         GPIO_USBHS2_DP
#else
#  error Not selected USBHOST peripheral
#endif

#if defined(CONFIG_N32H7_USBHS1_HOST) && defined(CONFIG_N32H7_USBHS2_HOST)
#  error Only one HOST role supported
#endif

/* N32H7 MOD:
 * The N32H76x USBHS core has no on-chip HS PHY; an external ULPI PHY is
 * the default configuration.  Define CONFIG_N32H7_USBHS_FS to select the
 * FS serial transceiver (GCFG.PHYSEL) instead.
 */

#include "n32_usbhs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* N32 USBHS Host Driver Support
 *
 * Pre-requisites
 *
 *  CONFIG_USBHOST        - Enable general USB host support
 *  CONFIG_N32H7_USBHS1_HOST or
 *  CONFIG_N32H7_USBHS2_HOST - Select the N32H7 USBHS instance
 *
 * Options:
 *
 *  CONFIG_N32H7_USBHS_RXFIFO_SIZE - Size of the RX FIFO in 32-bit words.
 *    Default 128 (512 bytes)
 *  CONFIG_N32H7_USBHS_NPTXFIFO_SIZE - Size of the non-periodic Tx FIFO
 *    in 32-bit words.  Default 96 (384 bytes)
 *  CONFIG_N32H7_USBHS_PTXFIFO_SIZE - Size of the periodic Tx FIFO in 32-bit
 *    words.  Default 96 (384 bytes)
 *  CONFIG_N32H7_USBHS_DESCSIZE - Maximum size of a descriptor.  Default: 128
 *  CONFIG_N32H7_USBHS_SOFINTR - Enable SOF interrupts.  Why would you ever
 *    want to do that?
 *  CONFIG_N32_USBHOST_REGDEBUG - Enable very low-level register access
 *    debug.  Depends on CONFIG_DEBUG_FEATURES.
 *  CONFIG_N32_USBHOST_PKTDUMP - Dump all incoming and outgoing USB
 *    packets. Depends on CONFIG_DEBUG_FEATURES.
 */

/* Default RxFIFO size */

#ifndef CONFIG_N32H7_USBHS_RXFIFO_SIZE
#  define CONFIG_N32H7_USBHS_RXFIFO_SIZE 512
#endif

/* Default host non-periodic Tx FIFO size */

#ifndef CONFIG_N32H7_USBHS_NPTXFIFO_SIZE
#  define CONFIG_N32H7_USBHS_NPTXFIFO_SIZE 256
#endif

/* Default host periodic Tx fifo size register */

#ifndef CONFIG_N32H7_USBHS_PTXFIFO_SIZE
#  define CONFIG_N32H7_USBHS_PTXFIFO_SIZE 256
#endif

/* Maximum size of a descriptor */

#ifndef CONFIG_N32H7_USBHS_DESCSIZE
#  define CONFIG_N32H7_USBHS_DESCSIZE 128
#endif

/* Register/packet debug depends on CONFIG_DEBUG_FEATURES */

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_N32_USBHOST_REGDEBUG
#  undef CONFIG_N32_USBHOST_PKTDUMP
#endif

/* Driver-private constants ************************************************
 *
 * N32H7 MOD:
 * The following constants have no counterpart in hardware/n32h7_usbhs.h
 * (the hardware header provides only the field SHIFT/MASK).  They are
 * driver-internal encodings used by the ported core logic, private to
 * this file.
 */

/* Endpoint type values (unshifted, driver-internal encoding; matches the
 * HCHCTRL EPTYPE field encoding)
 */

#define N32_USBHS_EPTYPE_CTRL       0
#define N32_USBHS_EPTYPE_ISOC       1
#define N32_USBHS_EPTYPE_BULK       2
#define N32_USBHS_EPTYPE_INTR       3

/* Data PID values (unshifted; matches the HCHTXSIZ PID field encoding) */

#define N32_USBHS_PID_DATA0         0
#define N32_USBHS_PID_DATA1         2
#define N32_USBHS_PID_SETUP         3

/* GRSTCTRL TxFIFO number helper (flush all host TxFIFOs) */

#define N32_USBHS_GRSTCTRL_TXFNUM_HALL \
                                    (0x10UL << N32_USBHS_GRSTCTRL_TXFNUM_SHIFT)

/* Host all-channels interrupt helper (HACHINT/HACHINTEN) */

#define N32_USBHS_HACHINT_CH(n)     (1UL << (n))

/****************************************************************************
 * Debug Trace Instrumentation
 ****************************************************************************/

#ifdef HAVE_USBHOST_TRACE
enum usbhost_trace1codes_e
{
  __TRACE1_BASEVALUE = 0,           /* This will force the first value to be 1 */

  N32_USBHS_TRACE1_DEVDISCONN,      /* USBHS ERROR: Host Port Device disconnected */
  N32_USBHS_TRACE1_IRQATTACH,       /* USBHS ERROR: Failed to attach IRQ */
  N32_USBHS_TRACE1_TRNSFRFAILED,    /* USBHS ERROR: Host Port Transfer Failed */
  N32_USBHS_TRACE1_SENDSETUP,       /* USBHS ERROR: sendsetup() failed with: */
  N32_USBHS_TRACE1_SENDDATA,        /* USBHS ERROR: senddata() failed with: */
  N32_USBHS_TRACE1_RECVDATA,        /* USBHS ERROR: recvdata() failed with: */

#ifdef HAVE_USBHOST_TRACE_VERBOSE

  N32_USBHS_VTRACE1_CONNECTED,         /* USBHS Host Port connected */
  N32_USBHS_VTRACE1_DISCONNECTED,      /* USBHS Host Port disconnected */
  N32_USBHS_VTRACE1_GINT,              /* USBHS Handling Interrupt. Entry Point */
  N32_USBHS_VTRACE1_GINT_SOF,          /* USBHS Handle the start of frame interrupt */
  N32_USBHS_VTRACE1_GINT_RXFLVL,       /* USBHS Handle the RxFIFO non-empty interrupt */
  N32_USBHS_VTRACE1_GINT_NPTXFE,       /* USBHS Handle the non-periodic TxFIFO empty interrupt */
  N32_USBHS_VTRACE1_GINT_PTXFE,        /* USBHS Handle the periodic TxFIFO empty interrupt */
  N32_USBHS_VTRACE1_GINT_HC,           /* USBHS Handle the host channels interrupt */
  N32_USBHS_VTRACE1_GINT_HPRT,         /* USBHS Handle the host port interrupt */
  N32_USBHS_VTRACE1_GINT_HPRT_POCCHNG, /* USBHS HPRT: Port Over-Current Change */
  N32_USBHS_VTRACE1_GINT_HPRT_PCDET,   /* USBHS HPRT: Port Connect Detect */
  N32_USBHS_VTRACE1_GINT_HPRT_PENCHNG, /* USBHS HPRT: Port Enable Changed */
  N32_USBHS_VTRACE1_GINT_HPRT_LSDEV,   /* USBHS HPRT: Low Speed Device Connected */
  N32_USBHS_VTRACE1_GINT_HPRT_FSDEV,   /* USBHS HPRT: Full Speed Device Connected */
  N32_USBHS_VTRACE1_GINT_HPRT_LSFSSW,  /* USBHS HPRT: Host Switch: LS -> FS */
  N32_USBHS_VTRACE1_GINT_HPRT_FSLSSW,  /* USBHS HPRT: Host Switch: FS -> LS */
  N32_USBHS_VTRACE1_GINT_DISC,         /* USBHS Handle the disconnect detected interrupt */
  N32_USBHS_VTRACE1_GINT_IPXFR,        /* USBHS Handle the incomplete periodic transfer */

#endif

  __TRACE1_NSTRINGS,                /* Separates the format 1 from the format 2 strings */

  N32_USBHS_TRACE2_CLIP,            /* USBHS CLIP: chidx:  buflen: */

#ifdef HAVE_USBHOST_TRACE_VERBOSE

  N32_USBHS_VTRACE2_CHANWAKEUP_IN,  /* USBHS IN Channel wake up with result */
  N32_USBHS_VTRACE2_CHANWAKEUP_OUT, /* USBHS OUT Channel wake up with result */
  N32_USBHS_VTRACE2_CTRLIN,         /* USBHS CTRLIN */
  N32_USBHS_VTRACE2_CTRLOUT,        /* USBHS CTRLOUT */
  N32_USBHS_VTRACE2_INTRIN,         /* USBHS INTRIN */
  N32_USBHS_VTRACE2_INTROUT,        /* USBHS INTROUT */
  N32_USBHS_VTRACE2_BULKIN,         /* USBHS BULKIN */
  N32_USBHS_VTRACE2_BULKOUT,        /* USBHS BULKOUT */
  N32_USBHS_VTRACE2_ISOCIN,         /* USBHS ISOCIN */
  N32_USBHS_VTRACE2_ISOCOUT,        /* USBHS ISOCOUT */
  N32_USBHS_VTRACE2_STARTTRANSFER,  /* USBHS EP buflen */
  N32_USBHS_VTRACE2_CHANCONF_CTRL_IN,
  N32_USBHS_VTRACE2_CHANCONF_CTRL_OUT,
  N32_USBHS_VTRACE2_CHANCONF_INTR_IN,
  N32_USBHS_VTRACE2_CHANCONF_INTR_OUT,
  N32_USBHS_VTRACE2_CHANCONF_BULK_IN,
  N32_USBHS_VTRACE2_CHANCONF_BULK_OUT,
  N32_USBHS_VTRACE2_CHANCONF_ISOC_IN,
  N32_USBHS_VTRACE2_CHANCONF_ISOC_OUT,
  N32_USBHS_VTRACE2_CHANHALT,       /* Channel halted. chidx: , reason:  */

#endif

  __TRACE2_NSTRINGS                 /* Total number of enumeration values */
};

#  define TRACE1_FIRST     ((int)__TRACE1_BASEVALUE + 1)
#  define TRACE1_INDEX(id) ((int)(id) - TRACE1_FIRST)
#  define TRACE1_NSTRINGS  TRACE1_INDEX(__TRACE1_NSTRINGS + 1)

#  define TRACE2_FIRST     ((int)__TRACE1_NSTRINGS + 1)
#  define TRACE2_INDEX(id) ((int)(id) - TRACE2_FIRST)
#  define TRACE2_NSTRINGS  TRACE2_INDEX(__TRACE2_NSTRINGS)

#endif /* HAVE_USBHOST_TRACE */

/* HCD Setup ****************************************************************/

/* Hardware capabilities */

#define N32_NHOST_CHANNELS      8   /* Number of host channels */
#define N32_MAX_PACKET_SIZE     64  /* Full speed max packet size */
#define N32_EP0_DEF_PACKET_SIZE 8   /* EP0 default packet size */
#define N32_EP0_MAX_PACKET_SIZE 64  /* EP0 FS max packet size */
#define N32_MAX_TX_FIFOS        15  /* Max number of TX FIFOs */
#define N32_MAX_PKTCOUNT        256 /* Max packet count */
#define N32_RETRY_COUNT         3   /* Number of ctrl transfer retries */

/* Delays *******************************************************************/

#define N32_READY_DELAY         200000      /* In loop counts */
#define N32_FLUSH_DELAY         200000      /* In loop counts */
#define N32_SETUP_DELAY         SEC2TICK(5) /* 5 seconds in system ticks */
#define N32_DATANAK_DELAY       SEC2TICK(5) /* 5 seconds in system ticks */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The following enumeration represents the various states of the USB host
 * state machine (for debug purposes only)
 */

enum n32_smstate_e
{
  SMSTATE_DETACHED = 0,  /* Not attached to a device */
  SMSTATE_ATTACHED,      /* Attached to a device */
  SMSTATE_ENUM,          /* Attached, enumerating */
  SMSTATE_CLASS_BOUND,   /* Enumeration complete, class bound */
};

/* This enumeration provides the reason for the channel halt. */

enum n32_chreason_e
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

/* This structure retains the state of one host channel.  NOTE: Since there
 * is only one channel operation active at a time, some of the fields in
 * in the structure could be moved in struct n32_ubhost_s to achieve
 * some memory savings.
 */

struct n32_chan_s
{
  sem_t             waitsem;   /* Channel wait semaphore */
  volatile uint8_t  result;    /* The result of the transfer */
  volatile uint8_t  chreason;  /* Channel halt reason. See enum n32_chreason_e */
  uint8_t           chidx;     /* Channel index */
  uint8_t           epno;      /* Device endpoint number (0-127) */
  uint8_t           eptype;    /* See N32_USBHS_EPTYPE_* definitions */
  uint8_t           funcaddr;  /* Device function address */
  uint8_t           speed;     /* Device speed */
  uint8_t           interval;  /* Interrupt/isochronous EP polling interval */
  uint8_t           pid;       /* Data PID */
  uint8_t           npackets;  /* Number of packets (for data toggle) */
  bool              inuse;     /* True: This channel is "in use" */
  volatile bool     indata1;   /* IN data toggle. True: DATA01 (Bulk and INTR only) */
  volatile bool     outdata1;  /* OUT data toggle.  True: DATA01 */
  bool              in;        /* True: IN endpoint */
  volatile bool     waiter;    /* True: Thread is waiting for a channel event */
  uint16_t          maxpacket; /* Max packet size */
  uint16_t          buflen;    /* Buffer length (at start of transfer) */
  volatile uint16_t xfrd;      /* Bytes transferred (at end of transfer) */
  volatile uint16_t inflight;  /* Number of Tx bytes "in-flight" */
  uint8_t          *buffer;    /* Transfer buffer pointer */
#ifdef CONFIG_USBHOST_ASYNCH
  usbhost_asynch_t  callback;  /* Transfer complete callback */
  void             *arg;       /* Argument that accompanies the callback */
#endif
};

/* A channel represents on uni-directional endpoint.  So, in the case of the
 * bi-directional, control endpoint, there must be two channels to represent
 * the endpoint.
 */

struct n32_ctrlinfo_s
{
  uint8_t           inndx;     /* EP0 IN control channel index */
  uint8_t           outndx;    /* EP0 OUT control channel index */
};

/* This structure retains the state of the USB host controller */

struct n32_usbhost_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to struct n32_usbhost_s.
   */

  struct usbhost_driver_s drvr;

  /* This is the hub port description understood by class drivers */

  struct usbhost_roothubport_s rhport;

  /* Overall driver status */

  volatile uint8_t  smstate;   /* The state of the USB host state machine */
  uint8_t           chidx;     /* ID of channel waiting for space in Tx FIFO */
  volatile bool     connected; /* Connected to device */
  volatile bool     change;    /* Connection change */
  volatile bool     pscwait;   /* True: Thread is waiting for a port event */
  mutex_t           lock;      /* Support mutually exclusive access */
  sem_t             pscsem;    /* Semaphore to wait for a port event */
  struct n32_ctrlinfo_s ep0;   /* Root hub port EP0 description */

#ifdef CONFIG_USBHOST_HUB
  /* Used to pass external hub port events */

  volatile struct usbhost_hubport_s *hport;
#endif

  struct usbhost_devaddr_s devgen;  /* Address generation data */

  /* The state of each host channel */

  struct n32_chan_s chan[N32_MAX_TX_FIFOS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_N32_USBHOST_REGDEBUG
static void n32_printreg(uint32_t addr, uint32_t val, bool iswrite);
static void n32_checkreg(uint32_t addr, uint32_t val, bool iswrite);
static uint32_t n32_getreg(uint32_t addr);
static void n32_putreg(uint32_t addr, uint32_t value);
#else
#  define n32_getreg(addr)     getreg32(addr)
#  define n32_putreg(addr,val) putreg32(val,addr)
#endif

static inline void n32_modifyreg(uint32_t addr, uint32_t clrbits,
                                   uint32_t setbits);

#ifdef CONFIG_N32_USBHOST_PKTDUMP
#  define n32_pktdump(m,b,n) lib_dumpbuffer(m,b,n)
#else
#  define n32_pktdump(m,b,n)
#endif

/* Byte stream access helper functions **************************************/

static inline uint16_t n32_getle16(const uint8_t *val);

/* Channel management *******************************************************/

static int n32_chan_alloc(struct n32_usbhost_s *priv);
static inline void n32_chan_free(struct n32_usbhost_s *priv,
                                   int chidx);
static inline void n32_chan_freeall(struct n32_usbhost_s *priv);
static void n32_chan_configure(struct n32_usbhost_s *priv,
                                 int chidx);
static void n32_chan_halt(struct n32_usbhost_s *priv, int chidx,
                            enum n32_chreason_e chreason);
static int n32_chan_waitsetup(struct n32_usbhost_s *priv,
                                struct n32_chan_s *chan);
#ifdef CONFIG_USBHOST_ASYNCH
static int n32_chan_asynchsetup(struct n32_usbhost_s *priv,
                                  struct n32_chan_s *chan,
                                  usbhost_asynch_t callback, void *arg);
#endif
static int n32_chan_wait(struct n32_usbhost_s *priv,
                           struct n32_chan_s *chan);
static void n32_chan_wakeup(struct n32_usbhost_s *priv,
                              struct n32_chan_s *chan);
static int n32_ctrlchan_alloc(struct n32_usbhost_s *priv,
                                uint8_t epno, uint8_t funcaddr,
                                uint8_t speed,
                                struct n32_ctrlinfo_s *ctrlep);
static int n32_ctrlep_alloc(struct n32_usbhost_s *priv,
                              const struct usbhost_epdesc_s *epdesc,
                              usbhost_ep_t *ep);
static int n32_xfrep_alloc(struct n32_usbhost_s *priv,
                             const struct usbhost_epdesc_s *epdesc,
                             usbhost_ep_t *ep);

/* Control/data transfer logic **********************************************/

static void n32_transfer_start(struct n32_usbhost_s *priv,
                                 int chidx);
#if 0 /* Not used */
static inline uint16_t n32_getframe(void);
#endif
static int n32_ctrl_sendsetup(struct n32_usbhost_s *priv,
                                struct n32_ctrlinfo_s *ep0,
                                const struct usb_ctrlreq_s *req);
static int n32_ctrl_senddata(struct n32_usbhost_s *priv,
                               struct n32_ctrlinfo_s *ep0,
                               uint8_t *buffer, unsigned int buflen);
static int n32_ctrl_recvdata(struct n32_usbhost_s *priv,
                               struct n32_ctrlinfo_s *ep0,
                               uint8_t *buffer, unsigned int buflen);
static int n32_in_setup(struct n32_usbhost_s *priv, int chidx);
static ssize_t n32_in_transfer(struct n32_usbhost_s *priv, int chidx,
                                 uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void n32_in_next(struct n32_usbhost_s *priv,
                          struct n32_chan_s *chan);
static int n32_in_asynch(struct n32_usbhost_s *priv, int chidx,
                           uint8_t *buffer, size_t buflen,
                           usbhost_asynch_t callback, void *arg);
#endif
static int n32_out_setup(struct n32_usbhost_s *priv, int chidx);
static ssize_t n32_out_transfer(struct n32_usbhost_s *priv,
                                  int chidx, uint8_t *buffer,
                                  size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void n32_out_next(struct n32_usbhost_s *priv,
                           struct n32_chan_s *chan);
static int n32_out_asynch(struct n32_usbhost_s *priv, int chidx,
                            uint8_t *buffer, size_t buflen,
                            usbhost_asynch_t callback, void *arg);
#endif

/* Interrupt handling *******************************************************/

/* Lower level interrupt handlers */

static void n32_gint_wrpacket(struct n32_usbhost_s *priv,
                                uint8_t *buffer, int chidx, int buflen);
static inline void n32_gint_hcinisr(struct n32_usbhost_s *priv,
                                      int chidx);
static inline void n32_gint_hcoutisr(struct n32_usbhost_s *priv,
                                       int chidx);
static void n32_gint_connected(struct n32_usbhost_s *priv);
static void n32_gint_disconnected(struct n32_usbhost_s *priv);

/* Second level interrupt handlers */

#ifdef CONFIG_N32H7_USBHS_SOFINTR
static inline void n32_gint_sofisr(struct n32_usbhost_s *priv);
#endif
static inline void n32_gint_rxflvlisr(struct n32_usbhost_s *priv);
static inline void n32_gint_nptxfeisr(struct n32_usbhost_s *priv);
static inline void n32_gint_ptxfeisr(struct n32_usbhost_s *priv);
static inline void n32_gint_hcisr(struct n32_usbhost_s *priv);
static inline void n32_gint_hprtisr(struct n32_usbhost_s *priv);
static inline void n32_gint_discisr(struct n32_usbhost_s *priv);
static inline void n32_gint_ipxfrisr(struct n32_usbhost_s *priv);

/* First level, global interrupt handler */

static int n32_gint_isr(int irq, void *context, void *arg);

/* Interrupt controls */

static void n32_gint_enable(void);
static void n32_gint_disable(void);
static inline void n32_hostinit_enable(void);
static void n32_txfe_enable(struct n32_usbhost_s *priv, int chidx);

/* USB host controller operations *******************************************/

static int n32_wait(struct usbhost_connection_s *conn,
                      struct usbhost_hubport_s **hport);
static int n32_rh_enumerate(struct n32_usbhost_s *priv,
                              struct usbhost_connection_s *conn,
                              struct usbhost_hubport_s *hport);
static int n32_enumerate(struct usbhost_connection_s *conn,
                           struct usbhost_hubport_s *hport);

static int n32_ep0configure(struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep0, uint8_t funcaddr,
                              uint8_t speed, uint16_t maxpacketsize);
static int n32_epalloc(struct usbhost_driver_s *drvr,
                         const struct usbhost_epdesc_s *epdesc,
                         usbhost_ep_t *ep);
static int n32_epfree(struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int n32_alloc(struct usbhost_driver_s *drvr,
                       uint8_t **buffer, size_t *maxlen);
static int n32_free(struct usbhost_driver_s *drvr,
                      uint8_t *buffer);
static int n32_ioalloc(struct usbhost_driver_s *drvr,
                         uint8_t **buffer, size_t buflen);
static int n32_iofree(struct usbhost_driver_s *drvr,
                        uint8_t *buffer);
static int n32_ctrlin(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                        const struct usb_ctrlreq_s *req,
                        uint8_t *buffer);
static int n32_ctrlout(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                         const struct usb_ctrlreq_s *req,
                         const uint8_t *buffer);
static ssize_t n32_transfer(struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep, uint8_t *buffer,
                              size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static int n32_asynch(struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                        uint8_t *buffer, size_t buflen,
                        usbhost_asynch_t callback, void *arg);
#endif
static int n32_cancel(struct usbhost_driver_s *drvr, usbhost_ep_t ep);
#ifdef CONFIG_USBHOST_HUB
static int n32_connect(struct usbhost_driver_s *drvr,
                         struct usbhost_hubport_s *hport,
                         bool connected);
#endif
static void n32_disconnect(struct usbhost_driver_s *drvr,
                             struct usbhost_hubport_s *hport);

/* Initialization ***********************************************************/

static void n32_portreset(struct n32_usbhost_s *priv);
static void n32_flush_txfifos(uint32_t txfnum);
static void n32_flush_rxfifo(void);
static void n32_vbusdrive(struct n32_usbhost_s *priv, bool state);
static void n32_host_initialize(struct n32_usbhost_s *priv);

static inline void n32_sw_initialize(struct n32_usbhost_s *priv);
static inline int n32_hw_initialize(struct n32_usbhost_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* In this driver implementation, support is provided for only a single a
 * single USB device.  All status information can be simply retained in a
 * single global instance.
 */

static struct n32_usbhost_s g_usbhost =
{
  .lock = NXMUTEX_INITIALIZER,
  .pscsem = SEM_INITIALIZER(0),
};

/* This is the connection/enumeration interface */

static struct usbhost_connection_s g_usbconn =
{
  .wait      = n32_wait,
  .enumerate = n32_enumerate,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_printreg
 *
 * Description:
 *   Print the contents of an N32 register operation
 *
 ****************************************************************************/

#ifdef CONFIG_N32_USBHOST_REGDEBUG
static void n32_printreg(uint32_t addr, uint32_t val, bool iswrite)
{
  uinfo("%08" PRIx32 "%s%08" PRIx32 "\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/****************************************************************************
 * Name: n32_checkreg
 *
 * Description:
 *   Get the contents of an N32 register
 *
 ****************************************************************************/

#ifdef CONFIG_N32_USBHOST_REGDEBUG
static void n32_checkreg(uint32_t addr, uint32_t val, bool iswrite)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;
  static bool     prevwrite = false;

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

              n32_printreg(prevaddr, preval, prevwrite);
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

      /* Show the new regisgter access */

      n32_printreg(addr, val, iswrite);
    }
}
#endif

/****************************************************************************
 * Name: n32_getreg
 *
 * Description:
 *   Get the contents of an N32 register
 *
 ****************************************************************************/

#ifdef CONFIG_N32_USBHOST_REGDEBUG
static uint32_t n32_getreg(uint32_t addr)
{
  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Check if we need to print this value */

  n32_checkreg(addr, val, false);
  return val;
}
#endif

/****************************************************************************
 * Name: n32_putreg
 *
 * Description:
 *   Set the contents of an N32 register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_N32_USBHOST_REGDEBUG
static void n32_putreg(uint32_t addr, uint32_t val)
{
  /* Check if we need to print this value */

  n32_checkreg(addr, val, true);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: n32_modifyreg
 *
 * Description:
 *   Modify selected bits of an N32 register.
 *
 ****************************************************************************/

static inline void n32_modifyreg(uint32_t addr, uint32_t clrbits,
                                   uint32_t setbits)
{
  n32_putreg(addr, (((n32_getreg(addr)) & ~clrbits) | setbits));
}

/****************************************************************************
 * Name: n32_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 ****************************************************************************/

static inline uint16_t n32_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: n32_chan_alloc
 *
 * Description:
 *   Allocate a channel.
 *
 ****************************************************************************/

static int n32_chan_alloc(struct n32_usbhost_s *priv)
{
  int chidx;

  /* Search the table of channels */

  for (chidx = 0; chidx < N32_NHOST_CHANNELS; chidx++)
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
 * Name: n32_chan_free
 *
 * Description:
 *   Free a previoiusly allocated channel.
 *
 ****************************************************************************/

static void n32_chan_free(struct n32_usbhost_s *priv, int chidx)
{
  DEBUGASSERT((unsigned)chidx < N32_NHOST_CHANNELS);

  /* Halt the channel */

  n32_chan_halt(priv, chidx, CHREASON_FREED);

  /* Mark the channel available */

  priv->chan[chidx].inuse = false;
}

/****************************************************************************
 * Name: n32_chan_freeall
 *
 * Description:
 *   Free all channels.
 *
 ****************************************************************************/

static inline void n32_chan_freeall(struct n32_usbhost_s *priv)
{
  uint8_t chidx;

  /* Free all host channels */

  for (chidx = 2; chidx < N32_NHOST_CHANNELS; chidx++)
    {
      n32_chan_free(priv, chidx);
    }
}

/****************************************************************************
 * Name: n32_chan_configure
 *
 * Description:
 *   Configure or re-configure a host channel.  Host channels are configured
 *   when endpoint is allocated and EP0 (only) is re-configured with the
 *   max packet size or device address changes.
 *
 ****************************************************************************/

static void n32_chan_configure(struct n32_usbhost_s *priv, int chidx)
{
  struct n32_chan_s *chan = &priv->chan[chidx];
  uint32_t regval;

  /* Clear any old pending interrupts for this host channel. */

  n32_putreg(N32_USBHS_HCHINTSTS(chidx), 0xffffffff);

  /* Enable channel interrupts required for transfers on this channel. */

  regval = 0;

  switch (chan->eptype)
    {
      case N32_USBHS_EPTYPE_CTRL:
      case N32_USBHS_EPTYPE_BULK:
        {
  #ifdef HAVE_USBHOST_TRACE_VERBOSE
          uint16_t intrace;
          uint16_t outtrace;

          /* Determine the definitive trace ID to use below */

          if (chan->eptype == N32_USBHS_EPTYPE_CTRL)
            {
              intrace  = N32_USBHS_VTRACE2_CHANCONF_CTRL_IN;
              outtrace = N32_USBHS_VTRACE2_CHANCONF_CTRL_OUT;
            }
          else
            {
              intrace  = N32_USBHS_VTRACE2_CHANCONF_BULK_IN;
              outtrace = N32_USBHS_VTRACE2_CHANCONF_BULK_OUT;
            }
  #endif

          /* Interrupts required for CTRL and BULK endpoints */

          regval |= (N32_USBHS_HCHINTEN_TXCIEN   |
                    N32_USBHS_HCHINTEN_STALLIEN |
                    N32_USBHS_HCHINTEN_NAKIEN   |
                    N32_USBHS_HCHINTEN_TXERRIEN |
                    N32_USBHS_HCHINTEN_DTERRIEN);

          /* Additional setting for IN/OUT endpoints */

          if (chan->in)
            {
              usbhost_vtrace2(intrace, chidx, chan->epno);
              regval |= N32_USBHS_HCHINTEN_BBERRIEN;
            }
          else
            {
              usbhost_vtrace2(outtrace, chidx, chan->epno);
              regval |= N32_USBHS_HCHINTEN_NYETIEN;
            }
        }
        break;

      case N32_USBHS_EPTYPE_INTR:
        {
          /* Interrupts required for INTR endpoints */

          regval |= (N32_USBHS_HCHINTEN_TXCIEN   |
                    N32_USBHS_HCHINTEN_STALLIEN |
                    N32_USBHS_HCHINTEN_NAKIEN   |
                    N32_USBHS_HCHINTEN_TXERRIEN |
                    N32_USBHS_HCHINTEN_FOVRIEN  |
                    N32_USBHS_HCHINTEN_DTERRIEN);

          /* Additional setting for IN endpoints */

          if (chan->in)
            {
              usbhost_vtrace2(N32_USBHS_VTRACE2_CHANCONF_INTR_IN, chidx,
                              chan->epno);
              regval |= N32_USBHS_HCHINTEN_BBERRIEN;
            }
  #ifdef HAVE_USBHOST_TRACE_VERBOSE
          else
            {
              usbhost_vtrace2(N32_USBHS_VTRACE2_CHANCONF_INTR_OUT, chidx,
                              chan->epno);
            }
  #endif
        }
        break;

      case N32_USBHS_EPTYPE_ISOC:
        {
          /* Interrupts required for ISOC endpoints */

          regval |= (N32_USBHS_HCHINTEN_TXCIEN |
                    N32_USBHS_HCHINTEN_ACKIEN |
                    N32_USBHS_HCHINTEN_FOVRIEN);

          /* Additional setting for IN endpoints */

          if (chan->in)
            {
              usbhost_vtrace2(N32_USBHS_VTRACE2_CHANCONF_ISOC_IN, chidx,
                              chan->epno);
              regval |= (N32_USBHS_HCHINTEN_TXERRIEN |
                        N32_USBHS_HCHINTEN_BBERRIEN);
            }
  #ifdef HAVE_USBHOST_TRACE_VERBOSE
          else
            {
              usbhost_vtrace2(N32_USBHS_VTRACE2_CHANCONF_ISOC_OUT, chidx,
                              chan->epno);
            }
  #endif
        }
        break;
    }

  n32_putreg(N32_USBHS_HCHINTEN(chidx), regval);

  /* Enable the top level host channel interrupt. */

  n32_modifyreg(N32_USBHS_HACHINTEN, 0, N32_USBHS_HACHINT_CH(chidx));

  /* Make sure host channel interrupts are enabled. */

  n32_modifyreg(N32_USBHS_GINTEN, 0, N32_USBHS_GINTEN_HCHIEN);

  /* Program the HCCHAR register */

  regval = ((uint32_t)chan->maxpacket << N32_USBHS_HCHCTRL_MPS_SHIFT) |
           ((uint32_t)chan->epno      << N32_USBHS_HCHCTRL_EPNUM_SHIFT) |
           ((uint32_t)chan->eptype    << N32_USBHS_HCHCTRL_EPTYPE_SHIFT) |
           ((uint32_t)chan->funcaddr  << N32_USBHS_HCHCTRL_DEVADDR_SHIFT);

  /* Special case settings for low speed devices */

  if (chan->speed == USB_SPEED_LOW)
    {
      regval |= N32_USBHS_HCHCTRL_LSPDDEV;
    }

  /* Special case settings for IN endpoints */

  if (chan->in)
    {
      regval |= N32_USBHS_HCHCTRL_EPDIR;
    }

  /* Special case settings for INTR endpoints */

  if (chan->eptype == N32_USBHS_EPTYPE_INTR)
    {
      regval |= N32_USBHS_HCHCTRL_ODDFRM;
    }

  /* Write the channel configuration */

  n32_putreg(N32_USBHS_HCHCTRL(chidx), regval);
}

/****************************************************************************
 * Name: n32_chan_halt
 *
 * Description:
 *   Halt the channel associated with 'chidx' by setting the CHannel DISable
 *   (CHDIS) bit in in the HCCHAR register.
 *
 ****************************************************************************/

static void n32_chan_halt(struct n32_usbhost_s *priv, int chidx,
                            enum n32_chreason_e chreason)
{
  uint32_t hcchar;
  uint32_t intmsk;
  uint32_t eptype;
  unsigned int avail;

  /* Save the reason for the halt.  We need this in the channel halt
   * interrupt handling logic to know what to do next.
   */

  usbhost_vtrace2(N32_USBHS_VTRACE2_CHANHALT, chidx, chreason);

  priv->chan[chidx].chreason = (uint8_t)chreason;

  /* "The application can disable any channel by programming the
   *  N32_USBHS_HCHCTRLx register with the CHDIS and CHENA bits set to 1.
   *  This enables the USBHS host to flush the posted requests (if any)
   *  and generates a channel halted interrupt. The application must wait
   *  for the CHH interrupt in N32_USBHS_HCHINTSTSx before reallocating
   *  the channel for other transactions.
   *  The USBHS host does not interrupt the transaction that has already
   *  been started on the USB."
   */

  hcchar  = n32_getreg(N32_USBHS_HCHCTRL(chidx));
  hcchar |= (N32_USBHS_HCHCTRL_CHDIS | N32_USBHS_HCHCTRL_CHEN);

  /* Get the endpoint type from the HCCHAR register */

  eptype = hcchar & N32_USBHS_HCHCTRL_EPTYPE_MASK;

  /* Check for space in the Tx FIFO to issue the halt.
   *
   * "Before disabling a channel, the application must ensure that there is
   *  at least one free space available in the non-periodic request queue
   *  (when disabling a non-periodic channel) or the periodic request queue
   *  (when disabling a periodic channel). The application can simply flush
   *  the posted requests when the Request queue is full (before disabling
   *  the channel), by programming the N32_USBHS_HCHCTRLx register with the
   *   CHDIS bit set to 1, and the CHENA bit cleared to 0.
   */

  if (eptype == N32_USBHS_HCHCTRL_EPTYPE_CTRL ||
      eptype == N32_USBHS_HCHCTRL_EPTYPE_BULK)
    {
      /* Get the number of words available in the non-periodic Tx FIFO. */

      avail = n32_getreg(N32_USBHS_GNPTXFSTS) &
              N32_USBHS_GNPTXFSTS_NPTXFSAV_MASK;
    }
  else
    {
      /* if (eptype == N32_USBHS_HCHCTRL_EPTYPE_ISOC ||
       *     eptype == N32_USBHS_HCHCTRL_EPTYPE_INTR)
       */

      /* Get the number of words available in the non-periodic Tx FIFO. */

      avail = n32_getreg(N32_USBHS_HPTXFQSTS) &
              N32_USBHS_HPTXFQSTS_PTXFSAVL_MASK;
    }

  /* Check if there is any space available in the Tx FIFO. */

  if (avail == 0)
    {
      /* The Tx FIFO is full... disable the channel to flush the requests */

      hcchar &= ~N32_USBHS_HCHCTRL_CHEN;
    }

  /* Unmask the CHannel Halted (CHH) interrupt */

  intmsk  = n32_getreg(N32_USBHS_HCHINTEN(chidx));
  intmsk |= N32_USBHS_HCHINTEN_CHHTDIEN;
  n32_putreg(N32_USBHS_HCHINTEN(chidx), intmsk);

  /* Halt the channel by setting CHDIS (and maybe CHENA) in the HCCHAR */

  n32_putreg(N32_USBHS_HCHCTRL(chidx), hcchar);
}

/****************************************************************************
 * Name: n32_chan_waitsetup
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

static int n32_chan_waitsetup(struct n32_usbhost_s *priv,
                                struct n32_chan_s *chan)
{
  irqstate_t flags = enter_critical_section();
  int        ret   = -ENODEV;

  /* Is the device still connected? */

  if (priv->connected)
    {
      /* Yes.. then set waiter to indicate that we expect to be informed
       * when either (1) the device is disconnected, or (2) the transfer
       * completed.
       */

      chan->waiter   = true;
#ifdef CONFIG_USBHOST_ASYNCH
      chan->callback = NULL;
      chan->arg      = NULL;
#endif
      ret            = OK;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: n32_chan_asynchsetup
 *
 * Description:
 *   Set the request for the transfer complete event well BEFORE enabling
 *   the transfer (as soon as we are absolutely committed to the to avoid
 *   transfer).  We do this to minimize race conditions.  This logic would
 *   have to be expanded if we want to have more than one packet in flight
 *   at a time!
 *
 * Assumptions:
 *   Might be called from the level of an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int n32_chan_asynchsetup(struct n32_usbhost_s *priv,
                                  struct n32_chan_s *chan,
                                  usbhost_asynch_t callback, void *arg)
{
  irqstate_t flags = enter_critical_section();
  int        ret   = -ENODEV;

  /* Is the device still connected? */

  if (priv->connected)
    {
      /* Yes.. then set waiter to indicate that we expect to be informed
       * when either (1) the device is disconnected, or (2) the transfer
       * completed.
       */

      chan->waiter   = false;
      chan->callback = callback;
      chan->arg      = arg;
      ret            = OK;
    }

  leave_critical_section(flags);
  return ret;
}
#endif

/****************************************************************************
 * Name: n32_chan_wait
 *
 * Description:
 *   Wait for a transfer on a channel to complete.
 *
 * Assumptions:
 *   Called from a normal thread context
 *
 ****************************************************************************/

static int n32_chan_wait(struct n32_usbhost_s *priv,
                           struct n32_chan_s *chan)
{
  irqstate_t flags;
  int ret;

  /* Disable interrupts so that the following operations will be atomic.  On
   * the OTG FS global interrupt needs to be disabled.  However, here we
   * disable all interrupts to exploit that fact that interrupts will be re-
   * enabled while we wait.
   */

  flags = enter_critical_section();

  /* Loop, testing for an end of transfer condition.  The channel 'result'
   * was set to EBUSY and 'waiter' was set to true before the transfer;
   * 'waiter' will be set to false and 'result' will be set appropriately
   * when the transfer is completed.
   */

  do
    {
      /* Wait for the transfer to complete.  NOTE the transfer may already
       * completed before we get here or the transfer may complete while we
       * wait here.
       */

      nxsem_wait_uninterruptible(&chan->waitsem);
    }
  while (chan->waiter);

  /* The transfer is complete re-enable interrupts and return the result */

  ret = -(int)chan->result;
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: n32_chan_wakeup
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

static void n32_chan_wakeup(struct n32_usbhost_s *priv,
                              struct n32_chan_s *chan)
{
  /* Is the transfer complete? */

  if (chan->result != EBUSY)
    {
      /* Is there a thread waiting for this transfer to complete? */

      if (chan->waiter)
        {
#ifdef CONFIG_USBHOST_ASYNCH
          /* Yes.. there should not also be a callback scheduled */

          DEBUGASSERT(chan->callback == NULL);
#endif
          /* Wake'em up! */

          usbhost_vtrace2(chan->in ? N32_USBHS_VTRACE2_CHANWAKEUP_IN :
                                     N32_USBHS_VTRACE2_CHANWAKEUP_OUT,
                          chan->epno, chan->result);

          nxsem_post(&chan->waitsem);
          chan->waiter = false;
        }

#ifdef CONFIG_USBHOST_ASYNCH
      /* No.. is an asynchronous callback expected when the transfer
       * completes?
       */

      else if (chan->callback)
        {
          /* Handle continuation of IN/OUT pipes */

          if (chan->in)
            {
              n32_in_next(priv, chan);
            }
          else
            {
              n32_out_next(priv, chan);
            }
        }
#endif
    }
}

/****************************************************************************
 * Name: n32_ctrlchan_alloc
 *
 * Description:
 *   Allocate and configured channels for a control pipe.
 *
 ****************************************************************************/

static int n32_ctrlchan_alloc(struct n32_usbhost_s *priv,
                                uint8_t epno, uint8_t funcaddr,
                                uint8_t speed,
                                struct n32_ctrlinfo_s *ctrlep)
{
  struct n32_chan_s *chan;
  int inndx;
  int outndx;

  outndx = n32_chan_alloc(priv);
  if (outndx < 0)
    {
      return -ENOMEM;
    }

  ctrlep->outndx  = outndx;
  chan            = &priv->chan[outndx];
  chan->epno      = epno;
  chan->in        = false;
  chan->eptype    = N32_USBHS_EPTYPE_CTRL;
  chan->funcaddr  = funcaddr;
  chan->speed     = speed;
  chan->interval  = 0;
  chan->maxpacket = N32_EP0_DEF_PACKET_SIZE;
  chan->indata1   = false;
  chan->outdata1  = false;

  /* Configure control OUT channels */

  n32_chan_configure(priv, outndx);

  /* Allocate and initialize the control IN channel */

  inndx = n32_chan_alloc(priv);
  if (inndx < 0)
    {
      n32_chan_free(priv, outndx);
      return -ENOMEM;
    }

  ctrlep->inndx   = inndx;
  chan            = &priv->chan[inndx];
  chan->epno      = epno;
  chan->in        = true;
  chan->eptype    = N32_USBHS_EPTYPE_CTRL;
  chan->funcaddr  = funcaddr;
  chan->speed     = speed;
  chan->interval  = 0;
  chan->maxpacket = N32_EP0_DEF_PACKET_SIZE;
  chan->indata1   = false;
  chan->outdata1  = false;

  /* Configure control IN channels */

  n32_chan_configure(priv, inndx);
  return OK;
}

/****************************************************************************
 * Name: n32_ctrlep_alloc
 *
 * Description:
 *   Allocate a container and channels for control pipe.
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

static int n32_ctrlep_alloc(struct n32_usbhost_s *priv,
                              const struct usbhost_epdesc_s *epdesc,
                              usbhost_ep_t *ep)
{
  struct usbhost_hubport_s *hport;
  struct n32_ctrlinfo_s *ctrlep;
  int ret;

  /* Sanity check.  NOTE that this method should only be called if a device
   * is connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(epdesc->hport != NULL);
  hport = epdesc->hport;

  /* Allocate a container for the control endpoint */

  ctrlep = (struct n32_ctrlinfo_s *)
    kmm_malloc(sizeof(struct n32_ctrlinfo_s));
  if (ctrlep == NULL)
    {
      uerr("ERROR: Failed to allocate control endpoint container\n");
      return -ENOMEM;
    }

  /* Then allocate and configure the IN/OUT channels  */

  ret = n32_ctrlchan_alloc(priv, epdesc->addr & USB_EPNO_MASK,
                             hport->funcaddr, hport->speed, ctrlep);
  if (ret < 0)
    {
      uerr("ERROR: n32_ctrlchan_alloc failed: %d\n", ret);
      kmm_free(ctrlep);
      return ret;
    }

  /* Return a pointer to the control pipe container as the pipe "handle" */

  *ep = (usbhost_ep_t)ctrlep;
  return OK;
}

/****************************************************************************
 * Name: n32_xfrep_alloc
 *
 * Description:
 *   Allocate and configure one unidirectional endpoint.
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

static int n32_xfrep_alloc(struct n32_usbhost_s *priv,
                             const struct usbhost_epdesc_s *epdesc,
                             usbhost_ep_t *ep)
{
  struct usbhost_hubport_s *hport;
  struct n32_chan_s *chan;
  int chidx;

  /* Sanity check.  NOTE that this method should only be called if a device
   * is connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(epdesc->hport != NULL);
  hport = epdesc->hport;

  /* Allocate a host channel for the endpoint */

  chidx = n32_chan_alloc(priv);
  if (chidx < 0)
    {
      uerr("ERROR: Failed to allocate a host channel\n");
      return -ENOMEM;
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
  chan->indata1   = false;
  chan->outdata1  = false;

  /* Then configure the endpoint */

  n32_chan_configure(priv, chidx);

  /* Return the index to the allocated channel as the endpoint "handle" */

  *ep = (usbhost_ep_t)chidx;
  return OK;
}

/****************************************************************************
 * Name: n32_transfer_start
 *
 * Description:
 *   Start at transfer on the select IN or OUT channel.
 *
 ****************************************************************************/

static void n32_transfer_start(struct n32_usbhost_s *priv, int chidx)
{
  struct n32_chan_s *chan;
  uint32_t regval;
  unsigned int npackets;
  unsigned int maxpacket;
  unsigned int avail;
  unsigned int wrsize;
  unsigned int minsize;

  /* Set up the initial state of the transfer */

  chan           = &priv->chan[chidx];

  usbhost_vtrace2(N32_USBHS_VTRACE2_STARTTRANSFER, chidx, chan->buflen);

  chan->result   = EBUSY;
  chan->inflight = 0;
  chan->xfrd     = 0;
  priv->chidx    = chidx;

  /* Compute the expected number of packets associated to the transfer.
   * If the transfer length is zero (or less than the size of one maximum
   * size packet), then one packet is expected.
   */

  /* If the transfer size is greater than one packet, then calculate the
   * number of packets that will be received/sent, including any partial
   * final packet.
   */

  maxpacket = chan->maxpacket;

  if (chan->buflen > maxpacket)
    {
      npackets = (chan->buflen + maxpacket - 1) / maxpacket;

      /* Clip if the buffer length if it exceeds the maximum number of
       * packets that can be transferred (this should not happen).
       */

      if (npackets > N32_MAX_PKTCOUNT)
        {
          npackets = N32_MAX_PKTCOUNT;
          chan->buflen = N32_MAX_PKTCOUNT * maxpacket;
          usbhost_trace2(N32_USBHS_TRACE2_CLIP, chidx, chan->buflen);
        }
    }
  else
    {
      /* One packet will be sent/received (might be a zero length packet) */

      npackets = 1;
    }

  /* If it is an IN transfer, then adjust the size of the buffer UP to
   * a full number of packets.  Hmmm... couldn't this cause an overrun
   * into unallocated memory?
   */

#if 0 /* Think about this */
  if (chan->in)
    {
      /* Force the buffer length to an even multiple of maxpacket */

      chan->buflen = npackets * maxpacket;
    }
#endif

  /* Save the number of packets in the transfer.  We will need this in
   * order to set the next data toggle correctly when the transfer
   * completes.
   */

  chan->npackets = (uint8_t)npackets;

  /* Setup the HCTSIZn register */

  regval = ((uint32_t)chan->buflen << N32_USBHS_HCHTXSIZ_TXSIZ_SHIFT) |
           ((uint32_t)npackets << N32_USBHS_HCHTXSIZ_PKCNT_SHIFT) |
           ((uint32_t)chan->pid << N32_USBHS_HCHTXSIZ_PID_SHIFT);
  n32_putreg(N32_USBHS_HCHTXSIZ(chidx), regval);

  /* Setup the HCCHAR register: Frame oddness and host channel enable */

  regval = n32_getreg(N32_USBHS_HCHCTRL(chidx));

  /* Set/clear the Odd Frame bit.  Check for an even frame; if so set Odd
   * Frame. This field is applicable for only periodic (isochronous and
   * interrupt) channels.
   */

  if ((n32_getreg(N32_USBHS_HFNUM) & 1) == 0)
    {
      regval |= N32_USBHS_HCHCTRL_ODDFRM;
    }
  else
    {
      regval &= ~N32_USBHS_HCHCTRL_ODDFRM;
    }

  regval &= ~N32_USBHS_HCHCTRL_CHDIS;
  regval |= N32_USBHS_HCHCTRL_CHEN;
  n32_putreg(N32_USBHS_HCHCTRL(chidx), regval);

  /* If this is an out transfer, then we need to do more.. we need to copy
   * the outgoing data into the correct TxFIFO.
   */

  if (!chan->in && chan->buflen > 0)
    {
      /* Handle non-periodic (CTRL and BULK) OUT transfers differently than
       * periodic (INTR and ISOC) OUT transfers.
       */

      minsize = MIN(chan->buflen, chan->maxpacket);

      switch (chan->eptype)
        {
          case N32_USBHS_EPTYPE_CTRL: /* Non periodic transfer */
          case N32_USBHS_EPTYPE_BULK:
            {
              /* Read the Non-periodic Tx FIFO status register */

              regval = n32_getreg(N32_USBHS_GNPTXFSTS);
              avail  = ((regval & N32_USBHS_GNPTXFSTS_NPTXFSAV_MASK) >>
                        N32_USBHS_GNPTXFSTS_NPTXFSAV_SHIFT) << 2;
            }
            break;

          /* Periodic transfer */

          case N32_USBHS_EPTYPE_INTR:
          case N32_USBHS_EPTYPE_ISOC:
            {
              /* Read the Non-periodic Tx FIFO status register */

              regval = n32_getreg(N32_USBHS_HPTXFQSTS);
              avail  = ((regval & N32_USBHS_HPTXFQSTS_PTXFSAVL_MASK) >>
                        N32_USBHS_HPTXFQSTS_PTXFSAVL_SHIFT) << 2;
            }
            break;

          default:
            DEBUGPANIC();
            return;
        }

      /* Is there space in the TxFIFO to hold the minimum size packet? */

      if (minsize <= avail)
        {
          /* Yes.. Get the size of the biggest thing that we can put
           * in the Tx FIFO now
           */

          wrsize = chan->buflen;
          if (wrsize > avail)
            {
              /* Clip the write size to the number of full, max sized packets
               * that will fit in the Tx FIFO.
               */

              unsigned int wrpackets = avail / chan->maxpacket;

              wrsize = wrpackets * chan->maxpacket;
            }

          /* Write packet into the Tx FIFO. */

          n32_gint_wrpacket(priv, chan->buffer, chidx, wrsize);
        }

      /* Did we put the entire buffer into the Tx FIFO? */

      if (chan->buflen > avail)
        {
          /* No, there was insufficient space to hold the entire transfer ...
           * Enable the Tx FIFO interrupt to handle the transfer when the Tx
           * FIFO becomes empty.
           */

          n32_txfe_enable(priv, chidx);
        }
    }
}

/****************************************************************************
 * Name: n32_getframe
 *
 * Description:
 *   Get the current frame number.  The frame number (FRNUM) field increments
 *   when a new SOF is transmitted on the USB, and is cleared to 0 when it
 *   reaches 0x3fff.
 *
 ****************************************************************************/

#if 0 /* Not used */
static inline uint16_t n32_getframe(void)
{
  return (uint16_t)(n32_getreg(N32_USBHS_HFNUM) &
         N32_USBHS_HFNUM_FRNUM_MASK);
}
#endif

/****************************************************************************
 * Name: n32_ctrl_sendsetup
 *
 * Description:
 *   Send an IN/OUT SETUP packet.
 *
 ****************************************************************************/

static int n32_ctrl_sendsetup(struct n32_usbhost_s *priv,
                                struct n32_ctrlinfo_s *ep0,
                                const struct usb_ctrlreq_s *req)
{
  struct n32_chan_s *chan;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Loop while the device reports NAK (and a timeout is not exceeded */

  chan  = &priv->chan[ep0->outndx];
  start = clock_systime_ticks();

  do
    {
      /* Send the  SETUP packet */

      chan->pid    = N32_USBHS_PID_SETUP;
      chan->buffer = (uint8_t *)req;
      chan->buflen = USB_SIZEOF_CTRLREQ;
      chan->xfrd   = 0;

      /* Set up for the wait BEFORE starting the transfer */

      ret = n32_chan_waitsetup(priv, chan);
      if (ret < 0)
        {
          usbhost_trace1(N32_USBHS_TRACE1_DEVDISCONN, 0);
          return ret;
        }

      /* Start the transfer */

      n32_transfer_start(priv, ep0->outndx);

      /* Wait for the transfer to complete */

      ret = n32_chan_wait(priv, chan);

      /* Return on success and for all failures other than EAGAIN.  EAGAIN
       * means that the device NAKed the SETUP command and that we should
       * try a few more times.
       */

      if (ret != -EAGAIN)
        {
          /* Output some debug information if the transfer failed */

          if (ret < 0)
            {
              usbhost_trace1(N32_USBHS_TRACE1_TRNSFRFAILED, ret);
            }

          /* Return the result in any event */

          return ret;
        }

      /* Get the elapsed time (in frames) */

      elapsed = clock_systime_ticks() - start;
    }
  while (elapsed < N32_SETUP_DELAY);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: n32_ctrl_senddata
 *
 * Description:
 *   Send data in the data phase of an OUT control transfer.  Or send status
 *   in the status phase of an IN control transfer
 *
 ****************************************************************************/

static int n32_ctrl_senddata(struct n32_usbhost_s *priv,
                               struct n32_ctrlinfo_s *ep0,
                               uint8_t *buffer, unsigned int buflen)
{
  struct n32_chan_s *chan = &priv->chan[ep0->outndx];
  int ret;

  /* Save buffer information */

  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;

  /* Set the DATA PID */

  if (buflen == 0)
    {
      /* For status OUT stage with buflen == 0, set PID DATA1 */

      chan->outdata1 = true;
    }

  /* Set the Data PID as per the outdata1 boolean */

  chan->pid = chan->outdata1 ? N32_USBHS_PID_DATA1 : N32_USBHS_PID_DATA0;

  /* Set up for the wait BEFORE starting the transfer */

  ret = n32_chan_waitsetup(priv, chan);
  if (ret < 0)
    {
      usbhost_trace1(N32_USBHS_TRACE1_DEVDISCONN, 0);
      return ret;
    }

  /* Start the transfer */

  n32_transfer_start(priv, ep0->outndx);

  /* Wait for the transfer to complete and return the result */

  return n32_chan_wait(priv, chan);
}

/****************************************************************************
 * Name: n32_ctrl_recvdata
 *
 * Description:
 *   Receive data in the data phase of an IN control transfer.  Or receive
 *   status in the status phase of an OUT control transfer
 *
 ****************************************************************************/

static int n32_ctrl_recvdata(struct n32_usbhost_s *priv,
                               struct n32_ctrlinfo_s *ep0,
                               uint8_t *buffer, unsigned int buflen)
{
  struct n32_chan_s *chan = &priv->chan[ep0->inndx];
  int ret;

  /* Save buffer information */

  chan->pid    = N32_USBHS_PID_DATA1;
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;

  /* Set up for the wait BEFORE starting the transfer */

  ret = n32_chan_waitsetup(priv, chan);
  if (ret < 0)
    {
      usbhost_trace1(N32_USBHS_TRACE1_DEVDISCONN, 0);
      return ret;
    }

  /* Start the transfer */

  n32_transfer_start(priv, ep0->inndx);

  /* Wait for the transfer to complete and return the result */

  return n32_chan_wait(priv, chan);
}

/****************************************************************************
 * Name: n32_in_setup
 *
 * Description:
 *   Initiate an IN transfer on an bulk, interrupt, or isochronous pipe.
 *
 ****************************************************************************/

static int n32_in_setup(struct n32_usbhost_s *priv, int chidx)
{
  struct n32_chan_s *chan;

  /* Set up for the transfer based on the direction and the endpoint type */

  chan = &priv->chan[chidx];
  switch (chan->eptype)
    {
      default:
      case N32_USBHS_EPTYPE_CTRL: /* Control */
        {
          /* This kind of transfer on control endpoints other than EP0 are
           * not currently supported
           */

          return -ENOSYS;
        }

      case N32_USBHS_EPTYPE_ISOC: /* Isochronous */
        {
          /* Set up the IN data PID */

          usbhost_vtrace2(N32_USBHS_VTRACE2_ISOCIN, chidx, chan->buflen);
          chan->pid = N32_USBHS_PID_DATA0;
        }
        break;

      case N32_USBHS_EPTYPE_BULK: /* Bulk */
        {
          /* Setup the IN data PID */

          usbhost_vtrace2(N32_USBHS_VTRACE2_BULKIN, chidx, chan->buflen);
          chan->pid = chan->indata1 ? N32_USBHS_PID_DATA1 :
                      N32_USBHS_PID_DATA0;
        }
        break;

      case N32_USBHS_EPTYPE_INTR: /* Interrupt */
        {
          /* Setup the IN data PID */

          usbhost_vtrace2(N32_USBHS_VTRACE2_INTRIN, chidx, chan->buflen);
          chan->pid = chan->indata1 ? N32_USBHS_PID_DATA1 :
                      N32_USBHS_PID_DATA0;
        }
        break;
    }

  /* Start the transfer */

  n32_transfer_start(priv, chidx);
  return OK;
}

/****************************************************************************
 * Name: n32_in_transfer
 *
 * Description:
 *   Transfer 'buflen' bytes into 'buffer' from an IN channel.
 *
 ****************************************************************************/

static ssize_t n32_in_transfer(struct n32_usbhost_s *priv, int chidx,
                                 uint8_t *buffer, size_t buflen)
{
  struct n32_chan_s *chan;
  clock_t start;
  ssize_t xfrd;
  int ret;

  /* Loop until the transfer completes (i.e., buflen is decremented to zero)
   * or a fatal error occurs any error other than a simple NAK.  NAK would
   * simply indicate the end of the transfer (short-transfer).
   */

  chan         = &priv->chan[chidx];
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;
  xfrd         = 0;

  start = clock_systime_ticks();
  while (chan->xfrd < chan->buflen)
    {
      /* Set up for the wait BEFORE starting the transfer */

      ret = n32_chan_waitsetup(priv, chan);
      if (ret < 0)
        {
          usbhost_trace1(N32_USBHS_TRACE1_DEVDISCONN, 0);
          return (ssize_t)ret;
        }

      /* Set up for the transfer based on the direction and the endpoint */

      ret = n32_in_setup(priv, chidx);
      if (ret < 0)
        {
          uerr("ERROR: n32_in_setup failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Wait for the transfer to complete and get the result */

      ret = n32_chan_wait(priv, chan);

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

                  if (elapsed >= N32_DATANAK_DELAY)
                    {
                      /* Timeout out... break out returning the NAK as
                       * as a failure.
                       */

                      return (ssize_t)ret;
                    }

                  /* Wait a bit before retrying after a NAK. */

                  if (chan->eptype == N32_USBHS_EPTYPE_INTR)
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
                   * system timer resolution is 10MS, then
                   * nxsched_usleep(1000) will actually request a delay 20MS
                   * (due to both quantization and rounding).
                   *
                   * REVISIT: So which is better?  To ignore tiny delays and
                   * hog the system bandwidth?  Or to wait for an excessive
                   * amount and destroy system throughput?
                   */

                  if (delay > CONFIG_USEC_PER_TICK)
                    {
                      nxsched_usleep(delay - CONFIG_USEC_PER_TICK);
                    }
                }
            }
          else
            {
              /* Some unexpected, fatal error occurred. */

              usbhost_trace1(N32_USBHS_TRACE1_TRNSFRFAILED, ret);

              /* Break out and return the error */

              uerr("ERROR: n32_chan_wait failed: %d\n", ret);
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

          xfrd += chan->xfrd;
        }
    }

  return xfrd;
}

/****************************************************************************
 * Name: n32_in_next
 *
 * Description:
 *   Initiate the next of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is always called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void n32_in_next(struct n32_usbhost_s *priv,
                          struct n32_chan_s *chan)
{
  usbhost_asynch_t callback;
  void *arg;
  ssize_t nbytes;
  int result;
  int ret;

  /* Is the full transfer complete? Did the last chunk transfer OK? */

  result = -(int)chan->result;
  if (chan->xfrd < chan->buflen && result == OK)
    {
      /* Yes.. Set up for the next transfer based on the direction and the
       * endpoint type
       */

      ret = n32_in_setup(priv, chan->chidx);
      if (ret >= 0)
        {
          return;
        }

      uerr("ERROR: n32_in_setup failed: %d\n", ret);
      result = ret;
    }

  /* The transfer is complete, with or without an error */

  uinfo("Transfer complete:  %d\n", result);

  /* Extract the callback information */

  callback       = chan->callback;
  arg            = chan->arg;
  nbytes         = chan->xfrd;

  chan->callback = NULL;
  chan->arg      = NULL;
  chan->xfrd     = 0;

  /* Then perform the callback */

  if (result < 0)
    {
      nbytes = (ssize_t)result;
    }

  callback(arg, nbytes);
}
#endif

/****************************************************************************
 * Name: n32_in_asynch
 *
 * Description:
 *   Initiate the first of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is never called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int n32_in_asynch(struct n32_usbhost_s *priv, int chidx,
                           uint8_t *buffer, size_t buflen,
                           usbhost_asynch_t callback, void *arg)
{
  struct n32_chan_s *chan;
  int ret;

  /* Set up for the transfer BEFORE starting the first transfer */

  chan         = &priv->chan[chidx];
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;

  ret = n32_chan_asynchsetup(priv, chan, callback, arg);
  if (ret < 0)
    {
      uerr("ERROR: n32_chan_asynchsetup failed: %d\n", ret);
      return ret;
    }

  /* Set up for the transfer based on the direction and the endpoint type */

  ret = n32_in_setup(priv, chidx);
  if (ret < 0)
    {
      uerr("ERROR: n32_in_setup failed: %d\n", ret);
    }

  /* And return with the transfer pending */

  return ret;
}
#endif

/****************************************************************************
 * Name: n32_out_setup
 *
 * Description:
 *   Initiate an OUT transfer on an bulk, interrupt, or isochronous pipe.
 *
 ****************************************************************************/

static int n32_out_setup(struct n32_usbhost_s *priv, int chidx)
{
  struct n32_chan_s *chan;

  /* Set up for the transfer based on the direction and the endpoint type */

  chan = &priv->chan[chidx];
  switch (chan->eptype)
    {
      default:
      case N32_USBHS_EPTYPE_CTRL: /* Control */
        {
          /* This kind of transfer on control endpoints other than EP0 are
           * not currently supported
           */

          return -ENOSYS;
        }

      case N32_USBHS_EPTYPE_ISOC: /* Isochronous */
        {
          /* Set up the OUT data PID */

          usbhost_vtrace2(N32_USBHS_VTRACE2_ISOCOUT, chidx, chan->buflen);
          chan->pid = N32_USBHS_PID_DATA0;
        }
        break;

      case N32_USBHS_EPTYPE_BULK: /* Bulk */
        {
          /* Setup the OUT data PID */

          usbhost_vtrace2(N32_USBHS_VTRACE2_BULKOUT, chidx, chan->buflen);
          chan->pid = chan->outdata1 ? N32_USBHS_PID_DATA1 :
                      N32_USBHS_PID_DATA0;
        }
        break;

      case N32_USBHS_EPTYPE_INTR: /* Interrupt */
        {
          /* Setup the OUT data PID */

          usbhost_vtrace2(N32_USBHS_VTRACE2_INTROUT, chidx, chan->buflen);
          chan->pid = chan->outdata1 ? N32_USBHS_PID_DATA1 :
                      N32_USBHS_PID_DATA0;

          /* Toggle the OUT data PID for the next transfer */

          chan->outdata1 ^= true;
        }
        break;
    }

  /* Start the transfer */

  n32_transfer_start(priv, chidx);
  return OK;
}

/****************************************************************************
 * Name: n32_out_transfer
 *
 * Description:
 *   Transfer the 'buflen' bytes in 'buffer' through an OUT channel.
 *
 ****************************************************************************/

static ssize_t n32_out_transfer(struct n32_usbhost_s *priv,
                                  int chidx, uint8_t *buffer,
                                  size_t buflen)
{
  struct n32_chan_s *chan;
  clock_t start;
  clock_t elapsed;
  size_t xfrlen;
  ssize_t xfrd;
  int ret;
  bool zlp;

  /* Loop until the transfer completes (i.e., buflen is decremented to zero)
   * or a fatal error occurs (any error other than a simple NAK)
   */

  chan  = &priv->chan[chidx];
  start = clock_systime_ticks();
  xfrd  = 0;
  zlp   = (buflen == 0);

  while (buflen > 0 || zlp)
    {
      /* Transfer one packet at a time.  The hardware is capable of queueing
       * multiple OUT packets, but I just haven't figured out how to handle
       * the case where a single OUT packet in the group is NAKed.
       */

      xfrlen       = MIN(chan->maxpacket, buflen);
      chan->buffer = buffer;
      chan->buflen = xfrlen;
      chan->xfrd   = 0;

      /* Set up for the wait BEFORE starting the transfer */

      ret = n32_chan_waitsetup(priv, chan);
      if (ret < 0)
        {
          usbhost_trace1(N32_USBHS_TRACE1_DEVDISCONN, 0);
          return (ssize_t)ret;
        }

      /* Set up for the transfer based on the direction and the endpoint */

      ret = n32_out_setup(priv, chidx);
      if (ret < 0)
        {
          uerr("ERROR: n32_out_setup failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Wait for the transfer to complete and get the result */

      ret = n32_chan_wait(priv, chan);

      /* Handle transfer failures */

      if (ret < 0)
        {
          usbhost_trace1(N32_USBHS_TRACE1_TRNSFRFAILED, ret);

          /* Check for a special case:  If (1) the transfer was NAKed and (2)
           * no Tx FIFO empty or Rx FIFO not-empty event occurred, then we
           * should be able to just flush the Rx and Tx FIFOs and try again.
           * We can detect this latter case because then the transfer buffer
           * pointer and buffer size will be unaltered.
           */

          elapsed = clock_systime_ticks() - start;
          if (ret != -EAGAIN ||               /* Not a NAK condition OR */
              elapsed >= N32_DATANAK_DELAY || /* Timeout has elapsed OR */
              chan->xfrd > 0)                 /* Data has been partially transferred */
            {
              /* Break out and return the error */

              uerr("ERROR: n32_chan_wait failed: %d\n", ret);
              return (ssize_t)ret;
            }

          /* Is this flush really necessary? What does the hardware do with
           * the data in the FIFO when the NAK occurs?  Does it discard it?
           */

          n32_flush_txfifos(N32_USBHS_GRSTCTRL_TXFNUM_HALL);

          /* Get the device a little time to catch up.  Then retry the
           * transfer using the same buffer pointer and length.
           */

          nxsched_usleep(20 * 1000);
        }
      else
        {
          /* Successfully transferred.  Update the buffer pointe/length */

          buffer += xfrlen;
          buflen -= xfrlen;
          xfrd   += chan->xfrd;
          zlp     = false;
        }
    }

  return xfrd;
}

/****************************************************************************
 * Name: n32_out_next
 *
 * Description:
 *   Initiate the next of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is always called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void n32_out_next(struct n32_usbhost_s *priv,
                           struct n32_chan_s *chan)
{
  usbhost_asynch_t callback;
  void *arg;
  ssize_t nbytes;
  int result;
  int ret;

  /* Is the full transfer complete? Did the last chunk transfer OK? */

  result = -(int)chan->result;
  if (chan->xfrd < chan->buflen && result == OK)
    {
      /* Yes.. Set up for the next transfer based on the direction and the
       * endpoint type
       */

      ret = n32_out_setup(priv, chan->chidx);
      if (ret >= 0)
        {
          return;
        }

      uerr("ERROR: n32_out_setup failed: %d\n", ret);
      result = ret;
    }

  /* The transfer is complete, with or without an error */

  uinfo("Transfer complete:  %d\n", result);

  /* Extract the callback information */

  callback       = chan->callback;
  arg            = chan->arg;
  nbytes         = chan->xfrd;

  chan->callback = NULL;
  chan->arg      = NULL;
  chan->xfrd     = 0;

  /* Then perform the callback */

  if (result < 0)
    {
      nbytes = (ssize_t)result;
    }

  callback(arg, nbytes);
}
#endif

/****************************************************************************
 * Name: n32_out_asynch
 *
 * Description:
 *   Initiate the first of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is never called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int n32_out_asynch(struct n32_usbhost_s *priv, int chidx,
                            uint8_t *buffer, size_t buflen,
                            usbhost_asynch_t callback, void *arg)
{
  struct n32_chan_s *chan;
  int ret;

  /* Set up for the transfer BEFORE starting the first transfer */

  chan         = &priv->chan[chidx];
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;

  ret = n32_chan_asynchsetup(priv, chan, callback, arg);
  if (ret < 0)
    {
      uerr("ERROR: n32_chan_asynchsetup failed: %d\n", ret);
      return ret;
    }

  /* Set up for the transfer based on the direction and the endpoint type */

  ret = n32_out_setup(priv, chidx);
  if (ret < 0)
    {
      uerr("ERROR: n32_out_setup failed: %d\n", ret);
    }

  /* And return with the transfer pending */

  return ret;
}
#endif

/****************************************************************************
 * Name: n32_gint_wrpacket
 *
 * Description:
 *   Transfer the 'buflen' bytes in 'buffer' to the Tx FIFO associated with
 *   'chidx' (non-DMA).
 *
 ****************************************************************************/

static void n32_gint_wrpacket(struct n32_usbhost_s *priv,
                                uint8_t *buffer, int chidx, int buflen)
{
  uint32_t *src;
  uint32_t fifo;
  int buflen32;

  n32_pktdump("Sending", buffer, buflen);

  /* Get the number of 32-byte words associated with this byte size */

  buflen32 = (buflen + 3) >> 2;

  /* Get the address of the Tx FIFO associated with this channel */

  fifo = N32_USBHS_DFIFO_HCH(chidx);

  /* Transfer all of the data into the Tx FIFO */

  src = (uint32_t *)buffer;
  for (; buflen32 > 0; buflen32--)
    {
      uint32_t data = *src++;

      n32_putreg(fifo, data);
    }

  /* Increment the count of bytes "in-flight" in the Tx FIFO */

  priv->chan[chidx].inflight += buflen;
}

/****************************************************************************
 * Name: n32_gint_hcinisr
 *
 * Description:
 *   USB OTG FS host IN channels interrupt handler
 *
 *   One the completion of the transfer, the channel result byte may be set
 *   as follows:
 *
 *     OK     - Transfer completed successfully
 *     EAGAIN - If devices NAKs the transfer or NYET occurs
 *     EPERM  - If the endpoint stalls
 *     EIO    - On a TX or data toggle error
 *     EPIPE  - Frame overrun
 *
 *   EBUSY in the result field indicates that the transfer has not completed.
 *
 ****************************************************************************/

static inline void n32_gint_hcinisr(struct n32_usbhost_s *priv,
                                      int chidx)
{
  struct n32_chan_s *chan = &priv->chan[chidx];
  uint32_t regval;
  uint32_t pending;

  /* Read the HCINT register to get the pending HC interrupts.  Read the
   * HCINTMSK register to get the set of enabled HC interrupts.
   */

  pending = n32_getreg(N32_USBHS_HCHINTSTS(chidx));
  regval  = n32_getreg(N32_USBHS_HCHINTEN(chidx));

  /* AND the two to get the set of enabled, pending HC interrupts */

  pending &= regval;
  uinfo("HCINTMSK%d: %08" PRIx32 " pending: %08" PRIx32 "\n",
        chidx, regval, pending);

  /* Check for a pending ACK response received/transmitted interrupt */

  if ((pending & N32_USBHS_HCHINTSTS_ACKIF) != 0)
    {
      /* Clear the pending the ACK response received/transmitted interrupt */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx), N32_USBHS_HCHINTSTS_ACKIF);
    }

  /* Check for a pending STALL response receive (STALL) interrupt */

  else if ((pending & N32_USBHS_HCHINTSTS_STALLIF) != 0)
    {
      /* Clear the NAK and STALL Conditions. */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx),
                 N32_USBHS_HCHINTSTS_NAKIF | N32_USBHS_HCHINTSTS_STALLIF);

      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      n32_chan_halt(priv, chidx, CHREASON_STALL);

      /* When there is a STALL, clear any pending NAK so that it is not
       * processed below.
       */

      pending &= ~N32_USBHS_HCHINTSTS_NAKIF;
    }

  /* Check for a pending Data Toggle ERRor (DTERR) interrupt */

  else if ((pending & N32_USBHS_HCHINTSTS_DTERRIF) != 0)
    {
      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      n32_chan_halt(priv, chidx, CHREASON_DTERR);

      /* Clear the NAK and data toggle error conditions */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx),
                 N32_USBHS_HCHINTSTS_NAKIF | N32_USBHS_HCHINTSTS_DTERRIF);
    }

  /* Check for a pending FRaMe OverRun (FRMOR) interrupt */

  if ((pending & N32_USBHS_HCHINTSTS_FOVRIF) != 0)
    {
      /* Halt the channel -- the CHH interrupt is expected next */

      n32_chan_halt(priv, chidx, CHREASON_FRMOR);

      /* Clear the FRaMe OverRun (FRMOR) condition */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx), N32_USBHS_HCHINTSTS_FOVRIF);
    }

  /* Check for a pending TransFeR Completed (XFRC) interrupt */

  else if ((pending & N32_USBHS_HCHINTSTS_TXCFIF) != 0)
    {
      /* Clear the TransFeR Completed (XFRC) condition */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx), N32_USBHS_HCHINTSTS_TXCFIF);

      /* Then handle the transfer completion event based on the endpoint */

      if (chan->eptype == N32_USBHS_EPTYPE_CTRL ||
          chan->eptype == N32_USBHS_EPTYPE_BULK)
        {
          /* Halt the channel -- the CHH interrupt is expected next */

          n32_chan_halt(priv, chidx, CHREASON_XFRC);

          /* Clear any pending NAK condition.  The 'indata1' data toggle
           * should have been appropriately updated by the RxFIFO
           * logic as each packet was received.
           */

          n32_putreg(N32_USBHS_HCHINTSTS(chidx), N32_USBHS_HCHINTSTS_NAKIF);
        }
      else if (chan->eptype == N32_USBHS_EPTYPE_INTR)
        {
          /* Force the next transfer on an ODD frame */

          regval = n32_getreg(N32_USBHS_HCHCTRL(chidx));
          regval |= N32_USBHS_HCHCTRL_ODDFRM;
          n32_putreg(N32_USBHS_HCHCTRL(chidx), regval);

          /* Set the request done state */

          chan->result = OK;
        }
    }

  /* Check for a pending CHannel Halted (CHH) interrupt */

  else if ((pending & N32_USBHS_HCHINTSTS_CHHTDIF) != 0)
    {
      /* Mask the CHannel Halted (CHH) interrupt */

      regval  = n32_getreg(N32_USBHS_HCHINTEN(chidx));
      regval &= ~N32_USBHS_HCHINTEN_CHHTDIEN;
      n32_putreg(N32_USBHS_HCHINTEN(chidx), regval);

      /* Update the request state based on the host state machine state */

      if (chan->chreason == CHREASON_XFRC)
        {
          /* Set the request done result */

          chan->result = OK;
        }
      else if (chan->chreason == CHREASON_STALL)
        {
          /* Set the request stall result */

          chan->result = EPERM;
        }
      else if ((chan->chreason == CHREASON_TXERR) ||
               (chan->chreason == CHREASON_DTERR))
        {
          /* Set the request I/O error result */

          chan->result = EIO;
        }
      else if (chan->chreason == CHREASON_NAK)
        {
          /* Set the NAK error result */

          chan->result = EAGAIN;
        }
      else /* if (chan->chreason == CHREASON_FRMOR) */
        {
          /* Set the frame overrun error result */

          chan->result = EPIPE;
        }

      /* Clear the CHannel Halted (CHH) condition */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx), N32_USBHS_HCHINTSTS_CHHTDIF);
    }

  /* Check for a pending Transaction ERror (TXERR) interrupt */

  else if ((pending & N32_USBHS_HCHINTSTS_TXERRIF) != 0)
    {
      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      n32_chan_halt(priv, chidx, CHREASON_TXERR);

      /* Clear the Transaction ERror (TXERR) condition */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx), N32_USBHS_HCHINTSTS_TXERRIF);
    }

  /* Check for a pending NAK response received (NAK) interrupt */

  else if ((pending & N32_USBHS_HCHINTSTS_NAKIF) != 0)
    {
      /* For a BULK transfer, the hardware is capable of retrying
       * automatically on a NAK.  However, this is not always
       * what we need to do.  So we always halt the transfer and
       * return control to high level logic in the event of a NAK.
       */

#if 1
      /* Halt the interrupt channel */

      if (chan->eptype == N32_USBHS_EPTYPE_INTR ||
          chan->eptype == N32_USBHS_EPTYPE_BULK)
        {
          /* Halt the channel -- the CHH interrupt is expected next */

          n32_chan_halt(priv, chidx, CHREASON_NAK);
        }

      /* Re-activate CTRL and BULK channels.
       * REVISIT: This can cause a lot of interrupts!
       * REVISIT: BULK channel is not re-activated.
       */

      else if (chan->eptype == N32_USBHS_EPTYPE_CTRL)
        {
          /* Re-activate the channel by clearing CHDIS and assuring that
           * CHENA is set
           *
           * TODO: set channel reason to NACK?
           */

          regval  = n32_getreg(N32_USBHS_HCHCTRL(chidx));
          regval |= N32_USBHS_HCHCTRL_CHEN;
          regval &= ~N32_USBHS_HCHCTRL_CHDIS;
          n32_putreg(N32_USBHS_HCHCTRL(chidx), regval);
        }

#else
      /* Halt all transfers on the NAK -- CHH interrupt is expected next */

      n32_chan_halt(priv, chidx, CHREASON_NAK);
#endif

      /* Clear the NAK condition */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx), N32_USBHS_HCHINTSTS_NAKIF);
    }

  /* Check for a transfer complete event */

  n32_chan_wakeup(priv, chan);
}

/****************************************************************************
 * Name: n32_gint_hcoutisr
 *
 * Description:
 *   USB OTG FS host OUT channels interrupt handler
 *
 *   One the completion of the transfer, the channel result byte may be set
 *   as follows:
 *
 *     OK     - Transfer completed successfully
 *     EAGAIN - If devices NAKs the transfer or NYET occurs
 *     EPERM  - If the endpoint stalls
 *     EIO    - On a TX or data toggle error
 *     EPIPE  - Frame overrun
 *
 *   EBUSY in the result field indicates that the transfer has not completed.
 *
 ****************************************************************************/

static inline void n32_gint_hcoutisr(struct n32_usbhost_s *priv,
                                       int chidx)
{
  struct n32_chan_s *chan = &priv->chan[chidx];
  uint32_t regval;
  uint32_t pending;

  /* Read the HCINT register to get the pending HC interrupts.  Read the
   * HCINTMSK register to get the set of enabled HC interrupts.
   */

  pending = n32_getreg(N32_USBHS_HCHINTSTS(chidx));
  regval  = n32_getreg(N32_USBHS_HCHINTEN(chidx));

  /* AND the two to get the set of enabled, pending HC interrupts */

  pending &= regval;
  uinfo("HCINTMSK%d: %08" PRIx32 " pending: %08" PRIx32 "\n",
        chidx, regval, pending);

  /* Check for a pending ACK response received/transmitted interrupt */

  if ((pending & N32_USBHS_HCHINTSTS_ACKIF) != 0)
    {
      /* Clear the pending the ACK response received/transmitted interrupt */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx), N32_USBHS_HCHINTSTS_ACKIF);
    }

  /* Check for a pending FRaMe OverRun (FRMOR) interrupt */

  else if ((pending & N32_USBHS_HCHINTSTS_FOVRIF) != 0)
    {
      /* Halt the channel (probably not necessary for FRMOR) */

      n32_chan_halt(priv, chidx, CHREASON_FRMOR);

      /* Clear the pending the FRaMe OverRun (FRMOR) interrupt */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx), N32_USBHS_HCHINTSTS_FOVRIF);
    }

  /* Check for a pending TransFeR Completed (XFRC) interrupt */

  else if ((pending & N32_USBHS_HCHINTSTS_TXCFIF) != 0)
    {
      /* Decrement the number of bytes remaining by the number of
       * bytes that were "in-flight".
       */

      priv->chan[chidx].buffer  += priv->chan[chidx].inflight;
      priv->chan[chidx].xfrd    += priv->chan[chidx].inflight;
      priv->chan[chidx].inflight = 0;

      /* Halt the channel -- the CHH interrupt is expected next */

      n32_chan_halt(priv, chidx, CHREASON_XFRC);

      /* Clear the pending the TransFeR Completed (XFRC) interrupt */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx), N32_USBHS_HCHINTSTS_TXCFIF);
    }

  /* Check for a pending STALL response receive (STALL) interrupt */

  else if ((pending & N32_USBHS_HCHINTSTS_STALLIF) != 0)
    {
      /* Clear the pending STALL response receive (STALL) interrupt */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx), N32_USBHS_HCHINTSTS_STALLIF);

      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      n32_chan_halt(priv, chidx, CHREASON_STALL);
    }

  /* Check for a pending NAK response received (NAK) interrupt */

  else if ((pending & N32_USBHS_HCHINTSTS_NAKIF) != 0)
    {
      /* Halt the channel  -- the CHH interrupt is expected next */

      n32_chan_halt(priv, chidx, CHREASON_NAK);

      /* Clear the pending the NAK response received (NAK) interrupt */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx), N32_USBHS_HCHINTSTS_NAKIF);
    }

  /* Check for a pending Transaction ERror (TXERR) interrupt */

  else if ((pending & N32_USBHS_HCHINTSTS_TXERRIF) != 0)
    {
      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      n32_chan_halt(priv, chidx, CHREASON_TXERR);

      /* Clear the pending the Transaction ERror (TXERR) interrupt */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx), N32_USBHS_HCHINTSTS_TXERRIF);
    }

  /* Check for a NYET interrupt */

#if 0 /* NYET is a reserved bit in the HCINT register */
  else if ((pending & N32_USBHS_HCHINTSTS_NYETIF) != 0)
    {
      /* Halt the channel */

      n32_chan_halt(priv, chidx, CHREASON_NYET);

      /* Clear the pending the NYET interrupt */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx), N32_USBHS_HCHINTSTS_NYETIF);
    }
#endif

  /* Check for a pending Data Toggle ERRor (DTERR) interrupt */

  else if (pending & N32_USBHS_HCHINTSTS_DTERRIF)
    {
      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      n32_chan_halt(priv, chidx, CHREASON_DTERR);

      /* Clear the pending the Data Toggle ERRor (DTERR) and NAK interrupts */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx),
                 N32_USBHS_HCHINTSTS_DTERRIF | N32_USBHS_HCHINTSTS_NAKIF);
    }

  /* Check for a pending CHannel Halted (CHH) interrupt */

  else if ((pending & N32_USBHS_HCHINTSTS_CHHTDIF) != 0)
    {
      /* Mask the CHannel Halted (CHH) interrupt */

      regval  = n32_getreg(N32_USBHS_HCHINTEN(chidx));
      regval &= ~N32_USBHS_HCHINTEN_CHHTDIEN;
      n32_putreg(N32_USBHS_HCHINTEN(chidx), regval);

      if (chan->chreason == CHREASON_XFRC)
        {
          /* Set the request done result */

          chan->result = OK;

          /* Read the HCCHAR register to get the HCCHAR register to get
           * the endpoint type.
           */

          regval = n32_getreg(N32_USBHS_HCHCTRL(chidx));

          /* Is it a bulk endpoint?  Were an odd number of packets
           * transferred?
           */

          if ((regval & N32_USBHS_HCHCTRL_EPTYPE_MASK) ==
              N32_USBHS_HCHCTRL_EPTYPE_BULK && (chan->npackets & 1) != 0)
            {
              /* Yes to both... toggle the data out PID */

              chan->outdata1 ^= true;
            }
        }
      else if (chan->chreason == CHREASON_NAK ||
               chan->chreason == CHREASON_NYET)
        {
          /* Set the try again later result */

          chan->result = EAGAIN;
        }
      else if (chan->chreason == CHREASON_STALL)
        {
          /* Set the request stall result */

          chan->result = EPERM;
        }
      else if ((chan->chreason == CHREASON_TXERR) ||
               (chan->chreason == CHREASON_DTERR))
        {
          /* Set the I/O failure result */

          chan->result = EIO;
        }
      else /* if (chan->chreason == CHREASON_FRMOR) */
        {
          /* Set the frame error result */

          chan->result = EPIPE;
        }

      /* Clear the pending the CHannel Halted (CHH) interrupt */

      n32_putreg(N32_USBHS_HCHINTSTS(chidx), N32_USBHS_HCHINTSTS_CHHTDIF);
    }

  /* Check for a transfer complete event */

  n32_chan_wakeup(priv, chan);
}

/****************************************************************************
 * Name: n32_gint_connected
 *
 * Description:
 *   Handle a connection event.
 *
 ****************************************************************************/

static void n32_gint_connected(struct n32_usbhost_s *priv)
{
  /* We we previously disconnected? */

  if (!priv->connected)
    {
      /* Yes.. then now we are connected */

      usbhost_vtrace1(N32_USBHS_VTRACE1_CONNECTED, 0);
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
 * Name: n32_gint_disconnected
 *
 * Description:
 *   Handle a disconnection event.
 *
 ****************************************************************************/

static void n32_gint_disconnected(struct n32_usbhost_s *priv)
{
  /* Were we previously connected? */

  if (priv->connected)
    {
      /* Yes.. then we no longer connected */

      usbhost_vtrace1(N32_USBHS_VTRACE1_DISCONNECTED, 0);

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
      n32_chan_freeall(priv);

      priv->rhport.hport.speed = USB_SPEED_FULL;
      priv->rhport.hport.funcaddr = 0;

      /* Notify any waiters that there is a change in the connection state */

      if (priv->pscwait)
        {
          nxsem_post(&priv->pscsem);
          priv->pscwait = false;
        }
    }
}

/****************************************************************************
 * Name: n32_gint_sofisr
 *
 * Description:
 *   USB OTG FS start-of-frame interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_N32H7_USBHS_SOFINTR
static inline void n32_gint_sofisr(struct n32_usbhost_s *priv)
{
  /* Handle SOF interrupt */

#warning "Do what?"

  /* Clear pending SOF interrupt */

  n32_putreg(N32_USBHS_GINTSTS, N32_USBHS_GINTSTS_SOFIF);
}
#endif

/****************************************************************************
 * Name: n32_gint_rxflvlisr
 *
 * Description:
 *   USB OTG FS RxFIFO non-empty interrupt handler
 *
 ****************************************************************************/

static inline void n32_gint_rxflvlisr(struct n32_usbhost_s *priv)
{
  uint32_t *dest;
  uint32_t grxsts;
  uint32_t intmsk;
  uint32_t hcchar;
  uint32_t hctsiz;
  uint32_t fifo;
  int bcnt;
  int bcnt32;
  int chidx;
  int i;

  /* Disable the RxFIFO non-empty interrupt */

  intmsk  = n32_getreg(N32_USBHS_GINTEN);
  intmsk &= ~N32_USBHS_GINTEN_RXFNEIEN;
  n32_putreg(N32_USBHS_GINTEN, intmsk);

  /* Read and pop the next status from the Rx FIFO */

  grxsts = n32_getreg(N32_USBHS_GRXSTSP);
  uinfo("GRXSTS: %08" PRIx32 "\n", grxsts);

  /* Isolate the channel number/index in the status word */

  chidx = (grxsts & N32_USBHS_GRXSTS_CHEPNUM_MASK) >>
          N32_USBHS_GRXSTS_CHEPNUM_SHIFT;

  /* Get the host channel characteristics register (HCCHAR) */

  hcchar = n32_getreg(N32_USBHS_HCHCTRL(chidx));

  /* Then process the interrupt according to the packet status */

  switch (grxsts & N32_USBHS_GRXSTS_PKTSTS_MASK)
    {
      case N32_USBHS_GRXSTS_IN_RCVD: /* IN data packet received */
        {
          /* Read the data into the host buffer. */

          bcnt = (grxsts & N32_USBHS_GRXSTS_BCNT_MASK) >>
                N32_USBHS_GRXSTS_BCNT_SHIFT;
          if (bcnt > 0 && priv->chan[chidx].buffer != NULL)
            {
              /* Transfer the packet from the Rx FIFO into the user buffer */

              dest   = (uint32_t *)priv->chan[chidx].buffer;
              fifo   = N32_USBHS_DFIFO_HCH(0);
              bcnt32 = (bcnt + 3) >> 2;

              for (i = 0; i < bcnt32; i++)
                {
                  *dest++ = n32_getreg(fifo);
                }

              n32_pktdump("Received", priv->chan[chidx].buffer, bcnt);

              /* Toggle the IN data pid (Used by Bulk and INTR only) */

              priv->chan[chidx].indata1 ^= true;

              /* Manage multiple packet transfers */

              priv->chan[chidx].buffer += bcnt;
              priv->chan[chidx].xfrd   += bcnt;

              /* Check if more packets are expected */

              hctsiz = n32_getreg(N32_USBHS_HCHTXSIZ(chidx));
              if ((hctsiz & N32_USBHS_HCHTXSIZ_PKCNT_MASK) != 0)
                {
                  /* Re-activate the channel when more packets are expected */

                  hcchar |= N32_USBHS_HCHCTRL_CHEN;
                  hcchar &= ~N32_USBHS_HCHCTRL_CHDIS;
                  n32_putreg(N32_USBHS_HCHCTRL(chidx), hcchar);
                }
            }
        }
        break;

      case N32_USBHS_GRXSTS_IN_CPLT:     /* IN transfer completed */
      case N32_USBHS_GRXSTS_DATA_TOGERR: /* Data toggle error */
      case N32_USBHS_GRXSTS_CHAN_TERMIN: /* Channel halted */
      default:
        break;
    }

  /* Re-enable the RxFIFO non-empty interrupt */

  intmsk |= N32_USBHS_GINTEN_RXFNEIEN;
  n32_putreg(N32_USBHS_GINTEN, intmsk);
}

/****************************************************************************
 * Name: n32_gint_nptxfeisr
 *
 * Description:
 *   USB OTG FS non-periodic TxFIFO empty interrupt handler
 *
 ****************************************************************************/

static inline void n32_gint_nptxfeisr(struct n32_usbhost_s *priv)
{
  struct n32_chan_s *chan;
  uint32_t     regval;
  unsigned int wrsize;
  unsigned int avail;
  unsigned int chidx;

  /* Recover the index of the channel that is waiting for space in the Tx
   * FIFO.
   */

  chidx = priv->chidx;
  chan  = &priv->chan[chidx];

  /* Reduce the buffer size by the number of bytes that were previously
   * placed in the Tx FIFO.
   */

  chan->buffer  += chan->inflight;
  chan->xfrd    += chan->inflight;
  chan->inflight = 0;

  /* If we have now transferred the entire buffer, then this transfer is
   * complete (this case really should never happen because we disable
   * the NPTXFE interrupt on the final packet).
   */

  if (chan->xfrd >= chan->buflen)
    {
      /* Disable further Tx FIFO empty interrupts and bail. */

      n32_modifyreg(N32_USBHS_GINTEN, N32_USBHS_GINTEN_NPTXFEIEN, 0);
      return;
    }

  /* Read the status from the top of the non-periodic TxFIFO */

  regval = n32_getreg(N32_USBHS_GNPTXFSTS);

  /* Extract the number of bytes available in the non-periodic Tx FIFO. */

  avail = ((regval & N32_USBHS_GNPTXFSTS_NPTXFSAV_MASK) >>
           N32_USBHS_GNPTXFSTS_NPTXFSAV_SHIFT) << 2;

  /* Get the size to put in the Tx FIFO now */

  wrsize = chan->buflen - chan->xfrd;

  /* Get minimal size packet that can be sent.  Something is seriously
   * configured wrong if one packet will not fit into the empty Tx FIFO.
   */

  DEBUGASSERT(wrsize > 0 && avail >= MIN(wrsize, chan->maxpacket));
  if (wrsize > avail)
    {
      /* Clip the write size to the number of full, max sized packets
       * that will fit in the Tx FIFO.
       */

      unsigned int wrpackets = avail / chan->maxpacket;

      wrsize = wrpackets * chan->maxpacket;
    }

  /* Otherwise, this will be the last packet to be sent in this transaction.
   * We now need to disable further NPTXFE interrupts.
   */

  else
    {
      n32_modifyreg(N32_USBHS_GINTEN, N32_USBHS_GINTEN_NPTXFEIEN, 0);
    }

  /* Write the next group of packets into the Tx FIFO */

  uinfo("HNPTXSTS: %08" PRIx32 " chidx: %d avail: %d buflen: %d xfrd: %d "
        "wrsize: %d\n",
        regval, chidx, avail, chan->buflen, chan->xfrd, wrsize);

  n32_gint_wrpacket(priv, chan->buffer, chidx, wrsize);
}

/****************************************************************************
 * Name: n32_gint_ptxfeisr
 *
 * Description:
 *   USB OTG FS periodic TxFIFO empty interrupt handler
 *
 ****************************************************************************/

static inline void n32_gint_ptxfeisr(struct n32_usbhost_s *priv)
{
  struct n32_chan_s *chan;
  uint32_t     regval;
  unsigned int wrsize;
  unsigned int avail;
  unsigned int chidx;

  /* Recover the index of the channel that is waiting for space in the Tx
   * FIFO.
   */

  chidx = priv->chidx;
  chan  = &priv->chan[chidx];

  /* Reduce the buffer size by the number of bytes that were previously
   * placed in the Tx FIFO.
   */

  chan->buffer  += chan->inflight;
  chan->xfrd    += chan->inflight;
  chan->inflight = 0;

  /* If we have now transferred the entire buffer, then this transfer is
   * complete (this case really should never happen because we disable
   * the PTXFE interrupt on the final packet).
   */

  if (chan->xfrd >= chan->buflen)
    {
      /* Disable further Tx FIFO empty interrupts and bail. */

      n32_modifyreg(N32_USBHS_GINTEN, N32_USBHS_GINTEN_PTXFEIEN, 0);
      return;
    }

  /* Read the status from the top of the periodic TxFIFO */

  regval = n32_getreg(N32_USBHS_HPTXFQSTS);

  /* Extract the number of bytes available in the periodic Tx FIFO. */

  avail = ((regval & N32_USBHS_HPTXFQSTS_PTXFSAVL_MASK) >>
           N32_USBHS_HPTXFQSTS_PTXFSAVL_SHIFT) << 2;

  /* Get the size to put in the Tx FIFO now */

  wrsize = chan->buflen - chan->xfrd;

  /* Get minimal size packet that can be sent.  Something is seriously
   * configured wrong if one packet will not fit into the empty Tx FIFO.
   */

  DEBUGASSERT(wrsize && avail >= MIN(wrsize, chan->maxpacket));
  if (wrsize > avail)
    {
      /* Clip the write size to the number of full, max sized packets
       * that will fit in the Tx FIFO.
       */

      unsigned int wrpackets = avail / chan->maxpacket;

      wrsize = wrpackets * chan->maxpacket;
    }

  /* Otherwise, this will be the last packet to be sent in this transaction.
   * We now need to disable further PTXFE interrupts.
   */

  else
    {
      n32_modifyreg(N32_USBHS_GINTEN, N32_USBHS_GINTEN_PTXFEIEN, 0);
    }

  /* Write the next group of packets into the Tx FIFO */

  uinfo("HPTXSTS: %08" PRIx32
        " chidx: %d avail: %d buflen: %d xfrd: %d wrsize: %d\n",
        regval, chidx, avail, chan->buflen, chan->xfrd, wrsize);

  n32_gint_wrpacket(priv, chan->buffer, chidx, wrsize);
}

/****************************************************************************
 * Name: n32_gint_hcisr
 *
 * Description:
 *   USB OTG FS host channels interrupt handler
 *
 ****************************************************************************/

static inline void n32_gint_hcisr(struct n32_usbhost_s *priv)
{
  uint32_t haint;
  uint32_t hcchar;
  int i = 0;

  /* Read the Host all channels interrupt register and test each bit in the
   * register. Each bit i, i=0...(N32_NHOST_CHANNELS-1), corresponds to
   * a pending interrupt on channel i.
   */

  haint = n32_getreg(N32_USBHS_HACHINT);
  for (i = 0; i < N32_NHOST_CHANNELS; i++)
    {
      /* Is an interrupt pending on this channel? */

      if ((haint & N32_USBHS_HACHINT_CH(i)) != 0)
        {
          /* Yes... read the HCCHAR register to get the direction bit */

          hcchar = n32_getreg(N32_USBHS_HCHCTRL(i));

          /* Was this an interrupt on an IN or an OUT channel? */

          if ((hcchar & N32_USBHS_HCHCTRL_EPDIR) != 0)
            {
              /* Handle the HC IN channel interrupt */

              n32_gint_hcinisr(priv, i);
            }
          else
            {
              /* Handle the HC OUT channel interrupt */

              n32_gint_hcoutisr(priv, i);
            }
        }
    }
}

/****************************************************************************
 * Name: n32_gint_hprtisr
 *
 * Description:
 *   USB OTG FS host port interrupt handler
 *
 ****************************************************************************/

static inline void n32_gint_hprtisr(struct n32_usbhost_s *priv)
{
  uint32_t hprt;
  uint32_t newhprt;

  usbhost_vtrace1(N32_USBHS_VTRACE1_GINT_HPRT, 0);

  /* Read the port status and control register (HPRT) */

  hprt = n32_getreg(N32_USBHS_HPCS);

  /* Setup to clear the interrupt bits in GINTSTS by setting the
   * corresponding bits in the HPRT.  The HCINT interrupt bit is cleared
   * when the appropriate status bits in the HPRT register are cleared.
   */

  newhprt = hprt & ~(N32_USBHS_HPCS_PEN    | N32_USBHS_HPCS_PCDET  |
                     N32_USBHS_HPCS_PENC | N32_USBHS_HPCS_POCC);

  /* Check for Port Overcurrent CHaNGe (POCCHNG) */

  if ((hprt & N32_USBHS_HPCS_POCC) != 0)
    {
      /* Set up to clear the POCCHNG status in the new HPRT contents. */

      usbhost_vtrace1(N32_USBHS_VTRACE1_GINT_HPRT_POCCHNG, 0);
      newhprt |= N32_USBHS_HPCS_POCC;
    }

  /* Check for Port Connect DETected (PCDET).  The core sets this bit when a
   * device connection is detected.
   */

  if ((hprt & N32_USBHS_HPCS_PCDET) != 0)
    {
      /* Set up to clear the PCDET status in the new HPRT contents. Then
       * process the new connection event.
       */

      usbhost_vtrace1(N32_USBHS_VTRACE1_GINT_HPRT_PCDET, 0);
      newhprt |= N32_USBHS_HPCS_PCDET;
      n32_portreset(priv);
      n32_gint_connected(priv);
    }

  /* Check for Port Enable CHaNGed (PENCHNG) */

  if ((hprt & N32_USBHS_HPCS_PENC) != 0)
    {
      /* Set up to clear the PENCHNG status in the new HPRT contents. */

      usbhost_vtrace1(N32_USBHS_VTRACE1_GINT_HPRT_PENCHNG, 0);
      newhprt |= N32_USBHS_HPCS_PENC;

      /* Was the port enabled? */

      if ((hprt & N32_USBHS_HPCS_PEN) != 0)
        {
          /* Yes.. handle the new connection event */

          n32_gint_connected(priv);

          /* Is this a low speed or full speed connection? */

          if ((hprt & N32_USBHS_HPCS_PSPD_MASK) == N32_USBHS_HPCS_PSPD_LS)
            {
              /* Set the Host Frame Interval Register for the 6KHz speed */

              usbhost_vtrace1(N32_USBHS_VTRACE1_GINT_HPRT_LSDEV, 0);
              n32_putreg(N32_USBHS_HFRI, 6000);
            }
          else
            {
              /* if ((hprt & N32_USBHS_HPCS_PSPD_MASK) ==
               *     N32_USBHS_HPCS_PSPD_FS)
               */

              /* Set the Host Frame Interval Register for the 48MHz speed */

              usbhost_vtrace1(N32_USBHS_VTRACE1_GINT_HPRT_FSDEV, 0);
              n32_putreg(N32_USBHS_HFRI, 48000);
            }
        }
    }

  /* Clear port interrupts by setting bits in the HPRT */

  n32_putreg(N32_USBHS_HPCS, newhprt);
}

/****************************************************************************
 * Name: n32_gint_discisr
 *
 * Description:
 *   USB OTG FS disconnect detected interrupt handler
 *
 ****************************************************************************/

static inline void n32_gint_discisr(struct n32_usbhost_s *priv)
{
  /* Handle the disconnection event */

  n32_gint_disconnected(priv);

  /* Clear the dicsonnect interrupt */

  n32_putreg(N32_USBHS_GINTSTS, N32_USBHS_GINTSTS_DISCIF);
}

/****************************************************************************
 * Name: n32_gint_ipxfrisr
 *
 * Description:
 *   USB OTG FS incomplete periodic interrupt handler
 *
 ****************************************************************************/

static inline void n32_gint_ipxfrisr(struct n32_usbhost_s *priv)
{
  uint32_t regval;

  /* CHENA : Set to enable the channel
   * CHDIS : Set to stop transmitting/receiving data on a channel
   */

  regval = n32_getreg(N32_USBHS_HCHCTRL(0));
  regval |= (N32_USBHS_HCHCTRL_CHDIS | N32_USBHS_HCHCTRL_CHEN);
  n32_putreg(N32_USBHS_HCHCTRL(0), regval);

  /* Clear the incomplete isochronous OUT interrupt */

  n32_putreg(N32_USBHS_GINTSTS, N32_USBHS_GINTSTS_PTNCIF_ISOUTNCIF);
}

/****************************************************************************
 * Name: n32_gint_isr
 *
 * Description:
 *   USB OTG global interrupt handler
 *
 ****************************************************************************/

static int n32_gint_isr(int irq, void *context, void *arg)
{
  /* At present, there is only support for a single OTG host. Hence it is
   * pre-allocated as g_usbhost.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple
   * devices.
   */

  struct n32_usbhost_s *priv = &g_usbhost;
  uint32_t pending;

  /* If OTG were supported, we would need to check if we are in host or
   * device mode when the global interrupt occurs.  Here we support only
   * host mode
   */

  /* Loop while there are pending interrupts to process.  This loop may save
   * a little interrupt handling overhead.
   */

  for (; ; )
    {
      /* Get the unmasked bits in the GINT status */

      pending  = n32_getreg(N32_USBHS_GINTSTS);
      pending &= n32_getreg(N32_USBHS_GINTEN);

      /* Return from the interrupt when there are no further pending
       * interrupts.
       */

      if (pending == 0)
        {
          return OK;
        }

      /* Otherwise, process each pending, unmasked GINT interrupts */

      /* Handle the start of frame interrupt */

#ifdef CONFIG_N32H7_USBHS_SOFINTR
      if ((pending & N32_USBHS_GINTSTS_SOFIF) != 0)
        {
          usbhost_vtrace1(N32_USBHS_VTRACE1_GINT_SOF, 0);
          n32_gint_sofisr(priv);
        }
#endif

      /* Handle the RxFIFO non-empty interrupt */

      if ((pending & N32_USBHS_GINTSTS_RXFNEIF) != 0)
        {
          usbhost_vtrace1(N32_USBHS_VTRACE1_GINT_RXFLVL, 0);
          n32_gint_rxflvlisr(priv);
        }

      /* Handle the non-periodic TxFIFO empty interrupt */

      if ((pending & N32_USBHS_GINTSTS_NPTXFEIF) != 0)
        {
          usbhost_vtrace1(N32_USBHS_VTRACE1_GINT_NPTXFE, 0);
          n32_gint_nptxfeisr(priv);
        }

      /* Handle the periodic TxFIFO empty interrupt */

      if ((pending & N32_USBHS_GINTSTS_PTXFEIF) != 0)
        {
          usbhost_vtrace1(N32_USBHS_VTRACE1_GINT_PTXFE, 0);
          n32_gint_ptxfeisr(priv);
        }

      /* Handle the host channels interrupt */

      if ((pending & N32_USBHS_GINTSTS_HCHIF) != 0)
        {
          usbhost_vtrace1(N32_USBHS_VTRACE1_GINT_HC, 0);
          n32_gint_hcisr(priv);
        }

      /* Handle the host port interrupt */

      if ((pending & N32_USBHS_GINTSTS_HPIF) != 0)
        {
          n32_gint_hprtisr(priv);
        }

      /* Handle the disconnect detected interrupt */

      if ((pending & N32_USBHS_GINTSTS_DISCIF) != 0)
        {
          usbhost_vtrace1(N32_USBHS_VTRACE1_GINT_DISC, 0);
          n32_gint_discisr(priv);
        }

      /* Handle the incomplete periodic transfer */

      if ((pending & N32_USBHS_GINTSTS_PTNCIF_ISOUTNCIF) != 0)
        {
          usbhost_vtrace1(N32_USBHS_VTRACE1_GINT_IPXFR, 0);
          n32_gint_ipxfrisr(priv);
        }
    }

  /* We won't get here */

  return OK;
}

/****************************************************************************
 * Name: n32_gint_enable and n32_gint_disable
 *
 * Description:
 *   Respectively enable or disable the global OTG FS interrupt.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_gint_enable(void)
{
  uint32_t regval;

  /* Set the GINTMSK bit to unmask the interrupt */

  regval  = n32_getreg(N32_USBHS_GAHBCFG);
  regval |= N32_USBHS_GAHBCFG_GINTEN;
  n32_putreg(N32_USBHS_GAHBCFG, regval);
}

static void n32_gint_disable(void)
{
  uint32_t regval;

  /* Clear the GINTMSK bit to mask the interrupt */

  regval  = n32_getreg(N32_USBHS_GAHBCFG);
  regval &= ~N32_USBHS_GAHBCFG_GINTEN;
  n32_putreg(N32_USBHS_GAHBCFG, regval);
}

/****************************************************************************
 * Name: n32_hostinit_enable
 *
 * Description:
 *   Enable host interrupts.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void n32_hostinit_enable(void)
{
  uint32_t regval;

  /* Disable all interrupts. */

  n32_putreg(N32_USBHS_GINTEN, 0);

  /* Clear any pending interrupts. */

  n32_putreg(N32_USBHS_GINTSTS, 0xffffffff);

  /* Enable the host interrupts */

  /* Common interrupts:
   *
   *   N32_USBHS_GINTEN_WKUPIEN     : Resume/remote wakeup detected interrupt
   *   N32_USBHS_GINTEN_USBSUSPIEN  : USB suspend
   */

  regval = (N32_USBHS_GINTEN_WKUPIEN | N32_USBHS_GINTEN_USBSUSPIEN);

  /* If OTG were supported, we would need to enable the following as well:
   *
   *   N32_USBHS_GINTEN_USBHSIEN  : OTG interrupt
   *   N32_USBHS_GINTEN_VBUSVIF   : Session request/new session detected
   *                                interrupt
   *   N32_USBHS_GINTEN_IDSTSCIEN : Connector ID status change
   */

  /* Host-specific interrupts
   *
   *   N32_USBHS_GINTEN_SOFIEN             : Start of frame
   *   N32_USBHS_GINTEN_RXFNEIEN           : RxFIFO non-empty
   *   N32_USBHS_GINTEN_PTNCIEN_ISOUTNCIEN : Incomplete isochronous OUT
   *                                         transfer interrupt
   *   N32_USBHS_GINTEN_HPIEN              : Host port interrupt
   *   N32_USBHS_GINTEN_HCHIEN             : Host channels interrupt
   *   N32_USBHS_GINTEN_DISCIEN            : Disconnect detected interrupt
   */

#ifdef CONFIG_N32H7_USBHS_SOFINTR
  regval |= (N32_USBHS_GINTEN_SOFIEN             |
             N32_USBHS_GINTEN_RXFNEIEN           |
             N32_USBHS_GINTEN_PTNCIEN_ISOUTNCIEN |
             N32_USBHS_GINTEN_HPIEN              |
             N32_USBHS_GINTEN_HCHIEN             |
             N32_USBHS_GINTEN_DISCIEN);
#else
  regval |= (N32_USBHS_GINTEN_RXFNEIEN           |
             N32_USBHS_GINTEN_PTNCIEN_ISOUTNCIEN |
             N32_USBHS_GINTEN_HPIEN              |
             N32_USBHS_GINTEN_HCHIEN             |
             N32_USBHS_GINTEN_DISCIEN);
#endif
  n32_putreg(N32_USBHS_GINTEN, regval);
}

/****************************************************************************
 * Name: n32_txfe_enable
 *
 * Description:
 *   Enable Tx FIFO empty interrupts.  This is necessary when the entire
 *   transfer will not fit into Tx FIFO.  The transfer will then be completed
 *   when the Tx FIFO is empty.  NOTE:  The Tx FIFO interrupt is disabled
 *   the fifo empty interrupt handler when the transfer is complete.
 *
 * Input Parameters:
 *   priv - Driver state structure reference
 *   chidx - The channel that requires the Tx FIFO empty interrupt
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from user task context.  Interrupts must be disabled to assure
 *   exclusive access to the GINTMSK register.
 *
 ****************************************************************************/

static void n32_txfe_enable(struct n32_usbhost_s *priv, int chidx)
{
  struct n32_chan_s *chan = &priv->chan[chidx];
  irqstate_t flags;
  uint32_t regval;

  /* Disable all interrupts so that we have exclusive access to the GINTMSK
   * (it would be sufficient just to disable the GINT interrupt).
   */

  flags = enter_critical_section();

  /* Should we enable the periodic or non-peridic Tx FIFO empty interrupts */

  regval = n32_getreg(N32_USBHS_GINTEN);
  switch (chan->eptype)
    {
      default:
      case N32_USBHS_EPTYPE_CTRL: /* Non periodic transfer */
      case N32_USBHS_EPTYPE_BULK:
        regval |= N32_USBHS_GINTEN_NPTXFEIEN;
        break;

      case N32_USBHS_EPTYPE_INTR: /* Periodic transfer */
      case N32_USBHS_EPTYPE_ISOC:
        regval |= N32_USBHS_GINTEN_PTXFEIEN;
        break;
    }

  /* Enable interrupts */

  n32_putreg(N32_USBHS_GINTEN, regval);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: n32_wait
 *
 * Description:
 *   Wait for a device to be connected or disconnected to/from a hub port.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from
 *     the call to the USB driver initialization logic.
 *   hport - The location to return the hub port descriptor that detected
 *     the connection related event.
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

static int n32_wait(struct usbhost_connection_s *conn,
                      struct usbhost_hubport_s **hport)
{
  struct n32_usbhost_s *priv = &g_usbhost;
  struct usbhost_hubport_s *connport;
  irqstate_t flags;
  int ret;

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

          uinfo("RHport Connected: %s\n",
                connport->connected ? "YES" : "NO");
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

          uinfo("Hub port Connected: %s\n",
                connport->connected ? "YES" : "NO");
          return OK;
        }
#endif

      /* Wait for the next connection event */

      priv->pscwait = true;
      ret = nxsem_wait_uninterruptible(&priv->pscsem);
      if (ret < 0)
        {
          return ret;
        }
    }
}

/****************************************************************************
 * Name: n32_enumerate
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
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int n32_rh_enumerate(struct n32_usbhost_s *priv,
                              struct usbhost_connection_s *conn,
                              struct usbhost_hubport_s *hport)
{
  uint32_t regval;
  int ret;

  DEBUGASSERT(conn != NULL && hport != NULL && hport->port == 0);

  /* Are we connected to a device?  The caller should have called the wait()
   * method first to be assured that a device is connected.
   */

  while (!priv->connected)
    {
      /* No, return an error */

      usbhost_trace1(N32_USBHS_TRACE1_DEVDISCONN, 0);
      return -ENODEV;
    }

  DEBUGASSERT(priv->smstate == SMSTATE_ATTACHED);

  /* USB 2.0 spec says at least 50ms delay before port reset.  We wait
   * 100ms.
   */

  nxsched_usleep(100 * 1000);

  /* Reset the host port */

  n32_portreset(priv);

  /* Get the current device speed */

  regval = n32_getreg(N32_USBHS_HPCS);
  if ((regval & N32_USBHS_HPCS_PSPD_MASK) == N32_USBHS_HPCS_PSPD_LS)
    {
      priv->rhport.hport.speed = USB_SPEED_LOW;
    }
  else
    {
      priv->rhport.hport.speed = USB_SPEED_FULL;
    }

  /* Allocate and initialize the root hub port EP0 channels */

  ret = n32_ctrlchan_alloc(priv, 0, 0, priv->rhport.hport.speed,
                             &priv->ep0);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate a control endpoint: %d\n", ret);
    }

  return ret;
}

static int n32_enumerate(struct usbhost_connection_s *conn,
                           struct usbhost_hubport_s *hport)
{
  struct n32_usbhost_s *priv = &g_usbhost;
  int ret;

  DEBUGASSERT(hport);

  /* If this is a connection on the root hub, then we need to go to
   * little more effort to get the device speed.  If it is a connection
   * on an external hub, then we already have that information.
   */

#ifdef CONFIG_USBHOST_HUB
  if (ROOTHUB(hport))
#endif
    {
      ret = n32_rh_enumerate(priv, conn, hport);
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
      n32_gint_disconnected(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: n32_ep0configure
 *
 * Description:
 *   Configure endpoint 0.  This method is normally used internally by the
 *   enumerate() method but is made available at the interface to support an
 *   external implementation of the enumeration logic.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep0 - The (opaque) EP0 endpoint instance
 *   funcaddr - The USB address of the function containing the endpoint that
 *     EP0 controls
 *   speed - The speed of the port USB_SPEED_LOW, _FULL, or _HIGH
 *   maxpacketsize - The maximum number of bytes that can be sent to or
 *    received from the endpoint in a single data packet
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int n32_ep0configure(struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep0,
                              uint8_t funcaddr, uint8_t speed,
                              uint16_t maxpacketsize)
{
  struct n32_usbhost_s *priv = (struct n32_usbhost_s *)drvr;
  struct n32_ctrlinfo_s *ep0info = (struct n32_ctrlinfo_s *)ep0;
  struct n32_chan_s *chan;
  int ret;

  DEBUGASSERT(drvr != NULL && ep0info != NULL && funcaddr < 128 &&
              maxpacketsize <= 64);

  /* We must have exclusive access to the USB host hardware and structures */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure the EP0 OUT channel */

  chan            = &priv->chan[ep0info->outndx];
  chan->funcaddr  = funcaddr;
  chan->speed     = speed;
  chan->maxpacket = maxpacketsize;

  n32_chan_configure(priv, ep0info->outndx);

  /* Configure the EP0 IN channel */

  chan            = &priv->chan[ep0info->inndx];
  chan->funcaddr  = funcaddr;
  chan->speed     = speed;
  chan->maxpacket = maxpacketsize;

  n32_chan_configure(priv, ep0info->inndx);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: n32_epalloc
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int n32_epalloc(struct usbhost_driver_s *drvr,
                         const struct usbhost_epdesc_s *epdesc,
                         usbhost_ep_t *ep)
{
  struct n32_usbhost_s *priv = (struct n32_usbhost_s *)drvr;
  int ret;

  /* Sanity check.  NOTE that this method should only be called if a device
   * is connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(drvr != 0 && epdesc != NULL && ep != NULL);

  /* We must have exclusive access to the USB host hardware and state
   * structures.
   */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Handler control pipes differently from other endpoint types.  This is
   * because the normal, "transfer" endpoints are unidirectional an require
   * only a single channel.  Control endpoints, however, are bi-directional
   * and require two channels, one for the IN and one for the OUT direction.
   */

  if (epdesc->xfrtype == N32_USBHS_EPTYPE_CTRL)
    {
      ret = n32_ctrlep_alloc(priv, epdesc, ep);
    }
  else
    {
      ret = n32_xfrep_alloc(priv, epdesc, ep);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: n32_epfree
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep - The endpoint to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int n32_epfree(struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct n32_usbhost_s *priv = (struct n32_usbhost_s *)drvr;
  int ret;

  DEBUGASSERT(priv);

  /* We must have exclusive access to the USB host hardware and structures */

  ret = nxmutex_lock(&priv->lock);

  /* A single channel is represent by an index in the range of 0 to
   * N32_MAX_TX_FIFOS.  Otherwise, the ep must be a pointer to an allocated
   * control endpoint structure.
   */

  if ((uintptr_t)ep < N32_MAX_TX_FIFOS)
    {
      /* Halt the channel and mark the channel available */

      n32_chan_free(priv, (int)ep);
    }
  else
    {
      /* Halt both control channel and mark the channels available */

      struct n32_ctrlinfo_s *ctrlep =
        (struct n32_ctrlinfo_s *)ep;

      n32_chan_free(priv, ctrlep->inndx);
      n32_chan_free(priv, ctrlep->outndx);

      /* And free the control endpoint container */

      kmm_free(ctrlep);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: n32_alloc
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
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   buffer - The address of a memory location provided by the caller in
 *     which to return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in
 *     which to return the maximum size of the allocated buffer memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int n32_alloc(struct usbhost_driver_s *drvr,
                       uint8_t **buffer, size_t *maxlen)
{
  uint8_t *alloc;

  DEBUGASSERT(drvr && buffer && maxlen);

  /* There is no special memory requirement for the N32. */

  alloc = kmm_malloc(CONFIG_N32H7_USBHS_DESCSIZE);
  if (!alloc)
    {
      return -ENOMEM;
    }

  /* Return the allocated address and size of the descriptor buffer */

  *buffer = alloc;
  *maxlen = CONFIG_N32H7_USBHS_DESCSIZE;
  return OK;
}

/****************************************************************************
 * Name: n32_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.  This method provides a
 *   mechanism to free that request/descriptor memory.  If the underlying
 *   hardware does not support such "special" memory, this functions may
 *   simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int n32_free(struct usbhost_driver_s *drvr, uint8_t *buffer)
{
  /* There is no special memory requirement */

  DEBUGASSERT(drvr && buffer);
  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: n32_ioalloc
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
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   buffer - The address of a memory location provided by the caller in
 *     which to return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int n32_ioalloc(struct usbhost_driver_s *drvr,
                         uint8_t **buffer, size_t buflen)
{
  uint8_t *alloc;

  DEBUGASSERT(drvr && buffer && buflen > 0);

  /* There is no special memory requirement */

  alloc = kmm_malloc(buflen);
  if (!alloc)
    {
      return -ENOMEM;
    }

  /* Return the allocated buffer */

  *buffer = alloc;
  return OK;
}

/****************************************************************************
 * Name: n32_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be accessed
 *   more efficiently.  This method provides a mechanism to free that IO
 *   buffer memory.  If the underlying hardware does not support such
 *   "special" memory, this functions may simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int n32_iofree(struct usbhost_driver_s *drvr,
                        uint8_t *buffer)
{
  /* There is no special memory requirement */

  DEBUGASSERT(drvr && buffer);
  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: n32_ctrlin and n32_ctrlout
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  These methods
 *   will enqueue the request and wait for it to complete.  Only one
 *   transfer may be queued; Neither these methods nor the transfer()
 *   method can be called again until the control transfer functions
 *   returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep0 - The control endpoint to send/receive the control request.
 *   req - Describes the request to be sent.  This request must lie in
 *     memory created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *     responses.  This buffer must be large enough to hold the length value
 *     in the request description. buffer must have been allocated using
 *     DRVR_ALLOC.
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same
 *   allocated memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int n32_ctrlin(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                        const struct usb_ctrlreq_s *req,
                        uint8_t *buffer)
{
  struct n32_usbhost_s *priv = (struct n32_usbhost_s *)drvr;
  struct n32_ctrlinfo_s *ep0info = (struct n32_ctrlinfo_s *)ep0;
  uint16_t buflen;
  clock_t start;
  clock_t elapsed;
  int retries;
  int ret;

  DEBUGASSERT(priv != NULL && ep0info != NULL && req != NULL);
  usbhost_vtrace2(N32_USBHS_VTRACE2_CTRLIN, req->type, req->req);
  uinfo("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* Extract values from the request */

  buflen = n32_getle16(req->len);

  /* We must have exclusive access to the USB host hardware and structures */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Loop, retrying until the retry time expires */

  for (retries = 0; retries < N32_RETRY_COUNT; retries++)
    {
      /* Send the SETUP request */

      ret = n32_ctrl_sendsetup(priv, ep0info, req);
      if (ret < 0)
        {
          usbhost_trace1(N32_USBHS_TRACE1_SENDSETUP, -ret);
          continue;
        }

      /* Handle the IN data phase (if any) */

      if (buflen > 0)
        {
          ret = n32_ctrl_recvdata(priv, ep0info, buffer, buflen);
          if (ret < 0)
            {
              usbhost_trace1(N32_USBHS_TRACE1_RECVDATA, -ret);
              continue;
            }
        }

      /* Get the start time.  Loop again until the timeout expires */

      start = clock_systime_ticks();
      do
        {
          /* Handle the status OUT phase */

          priv->chan[ep0info->outndx].outdata1 ^= true;
          ret = n32_ctrl_senddata(priv, ep0info, NULL, 0);
          if (ret == OK)
            {
              /* All success transactions exit here */

              nxmutex_unlock(&priv->lock);
              return OK;
            }

          usbhost_trace1(N32_USBHS_TRACE1_SENDDATA, ret < 0 ? -ret : ret);

          /* Get the elapsed time (in frames) */

          elapsed = clock_systime_ticks() - start;
        }
      while (elapsed < N32_DATANAK_DELAY);
    }

  /* All failures exit here after all retries and timeouts are exhausted */

  nxmutex_unlock(&priv->lock);
  return -ETIMEDOUT;
}

static int n32_ctrlout(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                         const struct usb_ctrlreq_s *req,
                         const uint8_t *buffer)
{
  struct n32_usbhost_s *priv = (struct n32_usbhost_s *)drvr;
  struct n32_ctrlinfo_s *ep0info = (struct n32_ctrlinfo_s *)ep0;
  uint16_t buflen;
  clock_t start;
  clock_t elapsed;
  int retries;
  int ret;

  DEBUGASSERT(priv != NULL && ep0info != NULL && req != NULL);
  usbhost_vtrace2(N32_USBHS_VTRACE2_CTRLOUT, req->type, req->req);
  uinfo("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* Extract values from the request */

  buflen = n32_getle16(req->len);

  /* We must have exclusive access to the USB host hardware and structures */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Loop, retrying until the retry time expires */

  for (retries = 0; retries < N32_RETRY_COUNT; retries++)
    {
      /* Send the SETUP request */

      ret = n32_ctrl_sendsetup(priv, ep0info, req);
      if (ret < 0)
        {
          usbhost_trace1(N32_USBHS_TRACE1_SENDSETUP, -ret);
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

              priv->chan[ep0info->outndx].outdata1 = true;
              ret = n32_ctrl_senddata(priv, ep0info, (uint8_t *)buffer,
                                        buflen);
              if (ret < 0)
                {
                  usbhost_trace1(N32_USBHS_TRACE1_SENDDATA, -ret);
                }
            }

          /* Handle the status IN phase */

          if (ret == OK)
            {
              ret = n32_ctrl_recvdata(priv, ep0info, NULL, 0);
              if (ret == OK)
                {
                  /* All success transactins exit here */

                  nxmutex_unlock(&priv->lock);
                  return OK;
                }

              usbhost_trace1(N32_USBHS_TRACE1_RECVDATA,
                             ret < 0 ? -ret : ret);
            }

          /* Get the elapsed time (in frames) */

          elapsed = clock_systime_ticks() - start;
        }
      while (elapsed < N32_DATANAK_DELAY);
    }

  /* All failures exit here after all retries and timeouts are exhausted */

  nxmutex_unlock(&priv->lock);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: n32_transfer
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
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which
 *     to perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or
 *     received (IN endpoint).  buffer must have been allocated using
 *     DRVR_ALLOC
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

static ssize_t n32_transfer(struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep, uint8_t *buffer,
                              size_t buflen)
{
  struct n32_usbhost_s *priv = (struct n32_usbhost_s *)drvr;
  unsigned int chidx = (unsigned int)ep;
  ssize_t nbytes;
  int ret;

  uinfo("chidx: %d buflen: %d\n",  (unsigned int)ep, buflen);

  DEBUGASSERT(priv && buffer && chidx < N32_MAX_TX_FIFOS && buflen > 0);

  /* We must have exclusive access to the USB host hardware and structures */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Handle IN and OUT transfer slightly differently */

  if (priv->chan[chidx].in)
    {
      nbytes = n32_in_transfer(priv, chidx, buffer, buflen);
    }
  else
    {
      nbytes = n32_out_transfer(priv, chidx, buffer, buflen);
    }

  nxmutex_unlock(&priv->lock);
  return nbytes;
}

/****************************************************************************
 * Name: n32_asynch
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
 *     call to the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on
 *     which to perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or
 *      received (IN endpoint).  buffer must have been allocated using
 *      DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *   callback - This function will be called when the transfer completes.
 *   arg - The arbitrary parameter that will be passed to the callback
 *      function when the transfer completes.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int n32_asynch(struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                        uint8_t *buffer, size_t buflen,
                        usbhost_asynch_t callback, void *arg)
{
  struct n32_usbhost_s *priv = (struct n32_usbhost_s *)drvr;
  unsigned int chidx = (unsigned int)ep;
  int ret;

  uinfo("chidx: %d buflen: %d\n",  (unsigned int)ep, buflen);

  DEBUGASSERT(priv && buffer && chidx < N32_MAX_TX_FIFOS && buflen > 0);

  /* We must have exclusive access to the USB host hardware and structures */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle IN and OUT transfer slightly differently */

  if (priv->chan[chidx].in)
    {
      ret = n32_in_asynch(priv, chidx, buffer, buflen, callback, arg);
    }
  else
    {
      ret = n32_out_asynch(priv, chidx, buffer, buflen, callback, arg);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif /* CONFIG_USBHOST_ASYNCH */

/****************************************************************************
 * Name: n32_cancel
 *
 * Description:
 *   Cancel a pending transfer on an endpoint.  Cancelled synchronous or
 *   asynchronous transfer will complete normally with the error -ESHUTDOWN.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on
 *     which an asynchronous transfer should be transferred.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 ****************************************************************************/

static int n32_cancel(struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct n32_usbhost_s *priv = (struct n32_usbhost_s *)drvr;
  struct n32_chan_s *chan;
  unsigned int chidx = (unsigned int)ep;
  irqstate_t flags;

  uinfo("chidx: %u\n",  chidx);

  DEBUGASSERT(priv && chidx < N32_MAX_TX_FIFOS);
  chan = &priv->chan[chidx];

  /* We need to disable interrupts to avoid race conditions with the
   * asynchronous completion of the transfer being canceled.
   */

  flags = enter_critical_section();

  /* Halt the channel */

  n32_chan_halt(priv, chidx, CHREASON_CANCELLED);
  chan->result = -ESHUTDOWN;

  /* Is there a thread waiting for this transfer to complete? */

  if (chan->waiter)
    {
#ifdef CONFIG_USBHOST_ASYNCH
      /* Yes.. there should not also be a callback scheduled */

      DEBUGASSERT(chan->callback == NULL);
#endif

      /* Wake'em up! */

      nxsem_post(&chan->waitsem);
      chan->waiter = false;
    }

#ifdef CONFIG_USBHOST_ASYNCH
  /* No.. is an asynchronous callback expected when the transfer
   * completes?
   */

  else if (chan->callback)
    {
      usbhost_asynch_t callback;
      void *arg;

      /* Extract the callback information */

      callback       = chan->callback;
      arg            = chan->arg;

      chan->callback = NULL;
      chan->arg      = NULL;
      chan->xfrd     = 0;

      /* Then perform the callback */

      callback(arg, -ESHUTDOWN);
    }
#endif

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: n32_connect
 *
 * Description:
 *   New connections may be detected by an attached hub.  This method is the
 *   mechanism that is used by the hub class to introduce a new connection
 *   and port description to the system.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
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
static int n32_connect(struct usbhost_driver_s *drvr,
                         struct usbhost_hubport_s *hport,
                         bool connected)
{
  struct n32_usbhost_s *priv = (struct n32_usbhost_s *)drvr;
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
 * Name: n32_disconnect
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
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   hport - The port from which the device is being disconnected.  Might be
 *      a port on a hub.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static void n32_disconnect(struct usbhost_driver_s *drvr,
                             struct usbhost_hubport_s *hport)
{
  DEBUGASSERT(hport != NULL);
  hport->devclass = NULL;
}

/****************************************************************************
 * Name: n32_portreset
 *
 * Description:
 *   Reset the USB host port.
 *
 *   NOTE: "Before starting to drive a USB reset, the application waits for
 *   the OTG interrupt triggered by the debounce done bit (DBCDNE bit in
 *   N32_USBHS_GPD), which indicates that the bus is stable again after
 *   the electrical debounce caused by the attachment of a pull-up resistor
 *   on DP (FS) or DM (LS).
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_portreset(struct n32_usbhost_s *priv)
{
  uint32_t regval;

  regval  = n32_getreg(N32_USBHS_HPCS);
  regval &= ~(N32_USBHS_HPCS_PEN   |
              N32_USBHS_HPCS_PCDET |
              N32_USBHS_HPCS_PENC  |
              N32_USBHS_HPCS_POCC);
  regval |= N32_USBHS_HPCS_PRST;
  n32_putreg(N32_USBHS_HPCS, regval);

  up_mdelay(100);

  regval &= ~N32_USBHS_HPCS_PRST;
  n32_putreg(N32_USBHS_HPCS, regval);

  up_mdelay(20);
}

/****************************************************************************
 * Name: n32_flush_txfifos
 *
 * Description:
 *   Flush the selected Tx FIFO.
 *
 * Input Parameters:
 *   txfnum -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void n32_flush_txfifos(uint32_t txfnum)
{
  uint32_t regval;
  uint32_t timeout;

  /* Initiate the TX FIFO flush operation */

  regval = N32_USBHS_GRSTCTRL_TXFFLSH | txfnum;
  n32_putreg(N32_USBHS_GRSTCTRL, regval);

  /* Wait for the FLUSH to complete */

  for (timeout = 0; timeout < N32_FLUSH_DELAY; timeout++)
    {
      regval = n32_getreg(N32_USBHS_GRSTCTRL);
      if ((regval & N32_USBHS_GRSTCTRL_TXFFLSH) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);
}

/****************************************************************************
 * Name: n32_flush_rxfifo
 *
 * Description:
 *   Flush the Rx FIFO.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void n32_flush_rxfifo(void)
{
  uint32_t regval;
  uint32_t timeout;

  /* Initiate the RX FIFO flush operation */

  n32_putreg(N32_USBHS_GRSTCTRL, N32_USBHS_GRSTCTRL_RXFFLSH);

  /* Wait for the FLUSH to complete */

  for (timeout = 0; timeout < N32_FLUSH_DELAY; timeout++)
    {
      regval = n32_getreg(N32_USBHS_GRSTCTRL);
      if ((regval & N32_USBHS_GRSTCTRL_RXFFLSH) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);
}

/****************************************************************************
 * Name: n32_vbusdrive
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

static void n32_vbusdrive(struct n32_usbhost_s *priv, bool state)
{
  uint32_t regval;

  /* Enable/disable the external charge pump */

  n32_usbhs_vbusdrive(0, state);

  /* Turn on the Host port power. */

  regval = n32_getreg(N32_USBHS_HPCS);
  regval &= ~(N32_USBHS_HPCS_PEN   |
              N32_USBHS_HPCS_PCDET |
              N32_USBHS_HPCS_PENC  |
              N32_USBHS_HPCS_POCC);

  if (((regval & N32_USBHS_HPCS_PPWR) == 0) && state)
    {
      regval |= N32_USBHS_HPCS_PPWR;
      n32_putreg(N32_USBHS_HPCS, regval);
    }

  if (((regval & N32_USBHS_HPCS_PPWR) != 0) && !state)
    {
      regval &= ~N32_USBHS_HPCS_PPWR;
      n32_putreg(N32_USBHS_HPCS, regval);
    }

  up_mdelay(200);
}

/****************************************************************************
 * Name: n32_host_initialize
 *
 * Description:
 *   Initialize/re-initialize hardware for host mode operation.  At present,
 *   this function is called only from n32_hw_initialize().  But if OTG
 *   mode were supported, this function would also be called to switch
 *   between host and device modes on a connector ID change interrupt.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void n32_host_initialize(struct n32_usbhost_s *priv)
{
  uint32_t regval;
  uint32_t offset;
  int i;

  /* Restart the PHY Clock */

  n32_putreg(N32_USBHS_PWRCTRL, 0);

  /* Reset the host port */

  n32_portreset(priv);

  regval = n32_getreg(N32_USBHS_HPCS);
  regval &= ~N32_USBHS_HCFG_SPSEL;
  n32_putreg(N32_USBHS_HPCS, regval);

  /* Carve up FIFO memory for the Rx FIFO and the periodic and non-periodic
   * Tx FIFOs
   */

  /* Configure Rx FIFO size (GRXFSIZ) */

  n32_putreg(N32_USBHS_GRXFSIZ, CONFIG_N32H7_USBHS_RXFIFO_SIZE);
  offset = CONFIG_N32H7_USBHS_RXFIFO_SIZE;

  /* Setup the host non-periodic Tx FIFO size (HNPTXFSIZ) */

  regval = (offset |
            (CONFIG_N32H7_USBHS_NPTXFIFO_SIZE <<
            N32_USBHS_GNPTXFSIZ_NPTXFDEP_SHIFT));
  n32_putreg(N32_USBHS_GNPTXFSIZ, regval);
  offset += CONFIG_N32H7_USBHS_NPTXFIFO_SIZE;

  /* Set up the host periodic Tx fifo size register (HPTXFSIZ) */

  regval = (offset |
            (CONFIG_N32H7_USBHS_PTXFIFO_SIZE <<
            N32_USBHS_HPTXFSIZ_HPTXFDEP_SHIFT));
  n32_putreg(N32_USBHS_HPTXFSIZ, regval);

  /* If OTG were supported, we should need to clear HNP enable bit in the
   * USB_OTG control register about here.
   */

  /* Flush all FIFOs */

  n32_flush_txfifos(N32_USBHS_GRSTCTRL_TXFNUM_HALL);
  n32_flush_rxfifo();

  /* Clear all pending HC Interrupts */

  for (i = 0; i < N32_NHOST_CHANNELS; i++)
    {
      n32_putreg(N32_USBHS_HCHINTSTS(i), 0xffffffff);
      n32_putreg(N32_USBHS_HCHINTEN(i), 0);
    }

  /* Driver Vbus +5V (the smoke test).  Should be done elsewhere in OTG
   * mode.
   */

  n32_vbusdrive(priv, true);

  /* Enable host interrupts */

  n32_hostinit_enable();
}

/****************************************************************************
 * Name: n32_sw_initialize
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

static inline void n32_sw_initialize(struct n32_usbhost_s *priv)
{
  struct usbhost_driver_s *drvr;
  struct usbhost_hubport_s *hport;
  int i;

  /* Initialize the device operations */

  drvr                 = &priv->drvr;
  drvr->ep0configure   = n32_ep0configure;
  drvr->epalloc        = n32_epalloc;
  drvr->epfree         = n32_epfree;
  drvr->alloc          = n32_alloc;
  drvr->free           = n32_free;
  drvr->ioalloc        = n32_ioalloc;
  drvr->iofree         = n32_iofree;
  drvr->ctrlin         = n32_ctrlin;
  drvr->ctrlout        = n32_ctrlout;
  drvr->transfer       = n32_transfer;
#ifdef CONFIG_USBHOST_ASYNCH
  drvr->asynch         = n32_asynch;
#endif
  drvr->cancel         = n32_cancel;
#ifdef CONFIG_USBHOST_HUB
  drvr->connect        = n32_connect;
#endif
  drvr->disconnect     = n32_disconnect;

  /* Initialize the public port representation */

  hport                = &priv->rhport.hport;
  hport->drvr          = drvr;
#ifdef CONFIG_USBHOST_HUB
  hport->parent        = NULL;
#endif
  hport->ep0           = (usbhost_ep_t)&priv->ep0;
  hport->speed         = USB_SPEED_FULL;

  /* Initialize function address generation logic */

  usbhost_devaddr_initialize(&priv->devgen);
  priv->rhport.pdevgen = &priv->devgen;

  /* Initialize the driver state data */

  priv->smstate   = SMSTATE_DETACHED;
  priv->connected = false;
  priv->change    = false;

  /* Put all of the channels back in their initial, allocated state */

  memset(priv->chan, 0, N32_MAX_TX_FIFOS * sizeof(struct n32_chan_s));

  /* Initialize each channel */

  for (i = 0; i < N32_MAX_TX_FIFOS; i++)
    {
      struct n32_chan_s *chan = &priv->chan[i];

      chan->chidx = i;
      nxsem_init(&chan->waitsem, 0, 0);
    }
}

/****************************************************************************
 * Name: n32_hw_initialize
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

static inline int n32_hw_initialize(struct n32_usbhost_s *priv)
{
  uint32_t regval;
  unsigned long timeout;

  /* Setup PHY PLL */

  regval = getreg32(N32_USBHS_WRPCFG);
  regval &= ~N32_USBHS_WRPCFG_PHYCLKSEL_MASK;
  regval |= N32_USBHS_WRPCFG_PHYCLKSEL;
  regval |= N32_USBHS_WRPCFG_PLLEN;
  regval &= ~N32_USBHS_WRPCFG_IDSIG;
  n32_putreg(N32_USBHS_WRPCFG, regval);

  /* Enable Host Pending */

  regval = n32_getreg(N32_USBHS_WRPCTRL);
  regval |= N32_USBHS_WRPCTRL_PINDETEN;
  n32_putreg(N32_USBHS_WRPCTRL, regval);

  /* Reset the USBHS core */

  if (N32_USBHS_BASE == N32_USBCTRL1_BASE)
    {
      /* Reset the USBHS1 core */

      regval = n32_getreg(N32_RCC_AHB2RST1);
      regval |= RCC_AHB2RST1_USB1PORRST;
      n32_putreg(N32_RCC_AHB2RST1, regval);
      regval &= ~RCC_AHB2RST1_USB1PORRST;
      n32_putreg(N32_RCC_AHB2RST1, regval);
    }
  else
    {
      /* Reset the USBHS2 core */

      regval = n32_getreg(N32_RCC_AHB1RST1);
      regval |= RCC_AHB1RST1_USB2PORRST;
      n32_putreg(N32_RCC_AHB1RST1, regval);
      regval &= ~RCC_AHB1RST1_USB2PORRST;
      n32_putreg(N32_RCC_AHB1RST1, regval);
    }

  up_mdelay(1);

  regval = n32_getreg(N32_USBHS_GCFG);
#ifdef CONFIG_N32H7_USBHS_FS
  /* N32H7 MOD: Set the PHYSEL bit in the GCFG register only when the FS
   * serial transceiver is used.  The N32H76x has no on-chip HS PHY; with
   * the default external ULPI PHY PHYSEL must stay cleared.
   */

  regval |= N32_USBHS_GCFG_PHYSEL;
#endif
  regval |= N32_USBHS_GCFG_PHYIF;
  n32_putreg(N32_USBHS_GCFG, regval);

  /* Reset after a PHY select and set Host mode.  First, wait for AHB master
   * IDLE state.
   */

  for (timeout = 0; timeout < N32_READY_DELAY; timeout++)
    {
      up_udelay(3);
      regval = n32_getreg(N32_USBHS_GRSTCTRL);
      if ((regval & N32_USBHS_GRSTCTRL_AHBIDLE) != 0)
        {
          break;
        }
    }

  /* Then perform the core soft reset. */

  regval = n32_getreg(N32_USBHS_GRSTCTRL);
  regval |= N32_USBHS_GRSTCTRL_CSRST;
  n32_putreg(N32_USBHS_GRSTCTRL, regval);

  for (timeout = 0; timeout < N32_READY_DELAY; timeout++)
    {
      regval = n32_getreg(N32_USBHS_GRSTCTRL);
      if ((regval & N32_USBHS_GRSTCTRL_SRSTDNE) != 0)
        {
          break;
        }
    }

  regval = n32_getreg(N32_USBHS_GRSTCTRL);
  regval &= ~N32_USBHS_GRSTCTRL_CSRST;
  n32_putreg(N32_USBHS_GRSTCTRL, regval);

  /* Wait for 3 PHY Clocks */

  up_udelay(3);

  up_mdelay(20);

  /* Initialize OTG features:  In order to support OTP, the HNPCAP and SRPCAP
   * bits would need to be set in the GUSBCFG register about here.
   */

  /* Force Host Mode */

  regval  = n32_getreg(N32_USBHS_GCFG);
  regval &= ~N32_USBHS_GCFG_FDMODE;
  regval |= N32_USBHS_GCFG_FHMODE;
  n32_putreg(N32_USBHS_GCFG, regval);
  up_mdelay(50);

  /* Initialize host mode and return success */

  n32_host_initialize(priv);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_usbhshost_initialize
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

struct usbhost_connection_s *n32_usbhshost_initialize(int controller)
{
  /* At present, there is only support for a single OTG FS host. Hence it is
   * pre-allocated as g_usbhost.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple
   * devices.
   */

  struct n32_usbhost_s *priv = &g_usbhost;

  /* Sanity checks */

  DEBUGASSERT(controller == 0);

  /* Make sure that interrupts from the OTG FS core are disabled */

  n32_gint_disable();

  /* Reset the state of the host driver */

  n32_sw_initialize(priv);

  /* N32H7 MOD: The N32H76x has no PWR_CR3 USB33DEN / USBREGEN bits.
   * USB power-domain enable and the USBHS peripheral clock are handled
   * by n32_usbhs_enableclock() (PWR IPMEM control + RCC AHB enable);
   * the external ULPI PHY needs no internal HS PHY PLL setup.
   */

  n32_usbhs_enableclock();
  n32_usbhs_phy_initialize();

  /* Alternate function pin configuration.  Here we assume that:
   *
   * 1. GPIO and USBHS peripheral clocking have already been enabled as
   *    part of the boot sequence (see n32_usbhs_enableclock() above).
   * 2. Board-specific logic has already enabled other board specific GPIOs
   *    for things like soft pull-up, VBUS sensing, power controls, and over-
   *    current detection.
   */

  /* N32H7 MOD: Configure the USBHS pins.  The GPIO_USBHS_* pin macros
   * are provided by board.h.  With the default external ULPI PHY, the
   * ULPI interface pins are configured; with CONFIG_N32H7_USBHS_FS only
   * the FS serial transceiver DM/DP pins are needed.
   */

  n32_configgpio(GPIO_USBHS_DM);
  n32_configgpio(GPIO_USBHS_DP);

  /* Initialize the USBHS core */

  n32_hw_initialize(priv);

  /* Attach USB host controller interrupt handler */

  if (irq_attach(N32_IRQ_USBHS, n32_gint_isr, NULL) != 0)
    {
      usbhost_trace1(N32_USBHS_TRACE1_IRQATTACH, 0);
      return NULL;
    }

  /* Enable USBHS global interrupts */

  n32_gint_enable();

  /* Enable interrupts at the interrupt controller */

  up_enable_irq(N32_IRQ_USBHS);
  return &g_usbconn;
}

/****************************************************************************
 * Debug Trace String Tables
 ****************************************************************************/

#ifdef HAVE_USBHOST_TRACE

#define TR_FMT1 false
#define TR_FMT2 true

#define TRENTRY(id,fmt1,string) {string}

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct n32_usbhost_trace_s
{
  const char *string;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct n32_usbhost_trace_s g_trace1[TRACE1_NSTRINGS] =
{
#ifdef HAVE_USBHOST_TRACE

  TRENTRY(N32_USBHS_TRACE1_DEVDISCONN,         TR_FMT1,
          "USBHS ERROR: Host Port %d. Device disconnected\n"),
  TRENTRY(N32_USBHS_TRACE1_IRQATTACH,          TR_FMT1,
          "USBHS ERROR: Failed to attach IRQ\n"),
  TRENTRY(N32_USBHS_TRACE1_TRNSFRFAILED,       TR_FMT1,
          "USBHS ERROR: Transfer Failed. ret=%d\n"),
  TRENTRY(N32_USBHS_TRACE1_SENDSETUP,          TR_FMT1,
          "USBHS ERROR: ctrl_sendsetup() failed with: %d\n"),
  TRENTRY(N32_USBHS_TRACE1_SENDDATA,           TR_FMT1,
          "USBHS ERROR: ctrl_senddata() failed with: %d\n"),
  TRENTRY(N32_USBHS_TRACE1_RECVDATA,           TR_FMT1,
          "USBHS ERROR: ctrl_recvdata() failed with: %d\n"),

#  ifdef HAVE_USBHOST_TRACE_VERBOSE

  TRENTRY(N32_USBHS_VTRACE1_CONNECTED,         TR_FMT1,
          "USBHS Host Port %d connected.\n"),
  TRENTRY(N32_USBHS_VTRACE1_DISCONNECTED,      TR_FMT1,
          "USBHS Host Port %d disconnected.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT,              TR_FMT1,
          "USBHS Handling Interrupt. Entry Point.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT_SOF,          TR_FMT1,
          "USBHS Handle the start of frame interrupt.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT_RXFLVL,       TR_FMT1,
          "USBHS Handle the RxFIFO non-empty interrupt.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT_NPTXFE,       TR_FMT1,
          "USBHS Handle the non-periodic TxFIFO empty interrupt.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT_PTXFE,        TR_FMT1,
          "USBHS Handle the periodic TxFIFO empty interrupt.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT_HC,           TR_FMT1,
          "USBHS Handle the host channels interrupt.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT_HPRT,         TR_FMT1,
          "USBHS Handle the host port interrupt.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT_HPRT_POCCHNG, TR_FMT1,
          "USBHS HPRT: Port Over-Current Change.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT_HPRT_PCDET,   TR_FMT1,
          "USBHS HPRT: Port Connect Detect.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT_HPRT_PENCHNG, TR_FMT1,
          "USBHS HPRT: Port Enable Changed.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT_HPRT_LSDEV,   TR_FMT1,
          "USBHS HPRT: Low Speed Device Connected.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT_HPRT_FSDEV,   TR_FMT1,
          "USBHS HPRT: Full Speed Device Connected.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT_HPRT_LSFSSW,  TR_FMT1,
          "USBHS HPRT: Host Switch: LS -> FS.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT_HPRT_FSLSSW,  TR_FMT1,
          "USBHS HPRT: Host Switch: FS -> LS.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT_DISC,         TR_FMT1,
          "USBHS Handle the disconnect detected interrupt.\n"),
  TRENTRY(N32_USBHS_VTRACE1_GINT_IPXFR,        TR_FMT1,
          "USBHS Handle the incomplete periodic transfer.\n"),

#  endif
#endif
};

static const struct n32_usbhost_trace_s g_trace2[TRACE2_NSTRINGS] =
{
#ifdef HAVE_USBHOST_TRACE

  TRENTRY(N32_USBHS_TRACE2_CLIP,                TR_FMT2,
          "USBHS CLIP: chidx: %d buflen: %d\n"),

#  ifdef HAVE_USBHOST_TRACE_VERBOSE

  TRENTRY(N32_USBHS_VTRACE2_CHANWAKEUP_IN,      TR_FMT2,
          "USBHS EP%d(IN)  wake up with result: %d\n"),
  TRENTRY(N32_USBHS_VTRACE2_CHANWAKEUP_OUT,     TR_FMT2,
          "USBHS EP%d(OUT) wake up with result: %d\n"),
  TRENTRY(N32_USBHS_VTRACE2_CTRLIN,             TR_FMT2,
          "USBHS CTRL_IN  type: %02x req: %02x\n"),
  TRENTRY(N32_USBHS_VTRACE2_CTRLOUT,            TR_FMT2,
          "USBHS CTRL_OUT type: %02x req: %02x\n"),
  TRENTRY(N32_USBHS_VTRACE2_INTRIN,             TR_FMT2,
          "USBHS INTR_IN  chidx: %02x len: %02x\n"),
  TRENTRY(N32_USBHS_VTRACE2_INTROUT,            TR_FMT2,
          "USBHS INTR_OUT chidx: %02x len: %02x\n"),
  TRENTRY(N32_USBHS_VTRACE2_BULKIN,             TR_FMT2,
          "USBHS BULK_IN  chidx: %02x len: %02x\n"),
  TRENTRY(N32_USBHS_VTRACE2_BULKOUT,            TR_FMT2,
          "USBHS BULK_OUT chidx: %02x len: %02x\n"),
  TRENTRY(N32_USBHS_VTRACE2_ISOCIN,             TR_FMT2,
          "USBHS ISOC_IN  chidx: %02x len: %04d\n"),
  TRENTRY(N32_USBHS_VTRACE2_ISOCOUT,            TR_FMT2,
          "USBHS ISOC_OUT chidx: %02x req: %02x\n"),
  TRENTRY(N32_USBHS_VTRACE2_STARTTRANSFER,      TR_FMT2,
          "USBHS Transfer chidx: %d buflen: %d\n"),
  TRENTRY(N32_USBHS_VTRACE2_CHANCONF_CTRL_IN,   TR_FMT2,
          "USBHS Channel configured. chidx: %d: (EP%d,IN ,CTRL)\n"),
  TRENTRY(N32_USBHS_VTRACE2_CHANCONF_CTRL_OUT,  TR_FMT2,
          "USBHS Channel configured. chidx: %d: (EP%d,OUT,CTRL)\n"),
  TRENTRY(N32_USBHS_VTRACE2_CHANCONF_INTR_IN,   TR_FMT2,
          "USBHS Channel configured. chidx: %d: (EP%d,IN ,INTR)\n"),
  TRENTRY(N32_USBHS_VTRACE2_CHANCONF_INTR_OUT,  TR_FMT2,
          "USBHS Channel configured. chidx: %d: (EP%d,OUT,INTR)\n"),
  TRENTRY(N32_USBHS_VTRACE2_CHANCONF_BULK_IN,   TR_FMT2,
          "USBHS Channel configured. chidx: %d: (EP%d,IN ,BULK)\n"),
  TRENTRY(N32_USBHS_VTRACE2_CHANCONF_BULK_OUT,  TR_FMT2,
          "USBHS Channel configured. chidx: %d: (EP%d,OUT,BULK)\n"),
  TRENTRY(N32_USBHS_VTRACE2_CHANCONF_ISOC_IN,   TR_FMT2,
          "USBHS Channel configured. chidx: %d: (EP%d,IN ,ISOC)\n"),
  TRENTRY(N32_USBHS_VTRACE2_CHANCONF_ISOC_OUT,  TR_FMT2,
          "USBHS Channel configured. chidx: %d: (EP%d,OUT,ISOC)\n"),
  TRENTRY(N32_USBHS_VTRACE2_CHANHALT,           TR_FMT2,
          "USBHS Channel halted. chidx: %d, reason: %d\n"),

#  endif
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_trformat1 and usbhost_trformat2
 *
 * Description:
 *   This interface must be provided by platform specific logic that knows
 *   the HCDs encoding of USB trace data.
 *
 *   Given an 9-bit index, return a format string suitable for use with, say,
 *   printf.  The returned format is expected to handle two unsigned integer
 *   values.
 *
 ****************************************************************************/

const char *usbhost_trformat1(uint16_t id)
{
  int ndx = TRACE1_INDEX(id);

  if (ndx < TRACE1_NSTRINGS)
    {
      return g_trace1[ndx].string;
    }

  return NULL;
}

const char *usbhost_trformat2(uint16_t id)
{
  int ndx = TRACE2_INDEX(id);

  if (ndx < TRACE2_NSTRINGS)
    {
      return g_trace2[ndx].string;
    }

  return NULL;
}

#endif /* HAVE_USBHOST_TRACE */

#endif /* CONFIG_USBHOST && (CONFIG_N32H7_USBHS1_HOST || CONFIG_N32H7_USBHS2_HOST) */

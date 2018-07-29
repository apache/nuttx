/****************************************************************************
 * arch/arm/src/efm32/efm32_usbhost.c
 *
 *   Copyright (C) 2014-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_devaddr.h>
#include <nuttx/usb/usbhost_trace.h>

#include <nuttx/irq.h>

#include "chip.h"             /* Includes default GPIO settings */
#include <arch/board/board.h> /* May redefine GPIO settings */

#include "up_arch.h"
#include "up_internal.h"

#include "efm32_usb.h"

#if defined(CONFIG_USBHOST) && defined(CONFIG_EFM32_OTGFS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ***************************************************************/
/* EFM32 USB OTG FS Host Driver Support
 *
 * Pre-requisites
 *
 *  CONFIG_USBHOST      - Enable general USB host support
 *  CONFIG_EFM32_OTGFS  - Enable the EFM32 USB OTG FS block
 *
 * Options:
 *
 *  CONFIG_EFM32_OTGFS_RXFIFO_SIZE - Size of the RX FIFO in 32-bit words.
 *    Default 128 (512 bytes)
 *  CONFIG_EFM32_OTGFS_NPTXFIFO_SIZE - Size of the non-periodic Tx FIFO
 *    in 32-bit words.  Default 96 (384 bytes)
 *  CONFIG_EFM32_OTGFS_PTXFIFO_SIZE - Size of the periodic Tx FIFO in 32-bit
 *    words.  Default 96 (384 bytes)
 *  CONFIG_EFM32_OTGFS_DESCSIZE - Maximum size of a descriptor.  Default: 128
 *  CONFIG_EFM32_OTGFS_SOFINTR - Enable SOF interrupts.  Why would you ever
 *    want to do that?
 *  CONFIG_EFM32_USBHOST_REGDEBUG - Enable very low-level register access
 *    debug.  Depends on CONFIG_DEBUG_FEATURES.
 *  CONFIG_EFM32_USBHOST_PKTDUMP - Dump all incoming and outgoing USB
 *    packets. Depends on CONFIG_DEBUG_FEATURES.
 */

/* Default RxFIFO size */

#ifndef CONFIG_EFM32_OTGFS_RXFIFO_SIZE
#  define CONFIG_EFM32_OTGFS_RXFIFO_SIZE 128
#endif

/* Default host non-periodic Tx FIFO size */

#ifndef CONFIG_EFM32_OTGFS_NPTXFIFO_SIZE
#  define CONFIG_EFM32_OTGFS_NPTXFIFO_SIZE 96
#endif

/* Default host periodic Tx fifo size register */

#ifndef CONFIG_EFM32_OTGFS_PTXFIFO_SIZE
#  define CONFIG_EFM32_OTGFS_PTXFIFO_SIZE 96
#endif

/* Maximum size of a descriptor */

#ifndef CONFIG_EFM32_OTGFS_DESCSIZE
#  define CONFIG_EFM32_OTGFS_DESCSIZE 128
#endif

/* Register/packet debug depends on CONFIG_DEBUG_FEATURES */

#ifndef CONFIG_DEBUG_USB_INFO
#  undef CONFIG_EFM32_USBHOST_REGDEBUG
#  undef CONFIG_EFM32_USBHOST_PKTDUMP
#endif

/* HCD Setup *******************************************************************/
/* Hardware capabilities */

#define EFM32_NHOST_CHANNELS      8   /* Number of host channels */
#define EFM32_MAX_PACKET_SIZE     64  /* Full speed max packet size */
#define EFM32_EP0_DEF_PACKET_SIZE 8   /* EP0 default packet size */
#define EFM32_EP0_MAX_PACKET_SIZE 64  /* EP0 FS max packet size */
#define EFM32_MAX_TX_FIFOS        15  /* Max number of TX FIFOs */
#define EFM32_MAX_PKTCOUNT        256 /* Max packet count */
#define EFM32_RETRY_COUNT         3   /* Number of ctrl transfer retries */

/* Delays **********************************************************************/

#define EFM32_READY_DELAY         200000 /* In loop counts */
#define EFM32_FLUSH_DELAY         200000 /* In loop counts */
#define EFM32_SETUP_DELAY         SEC2TICK(5) /* 5 seconds in system ticks */
#define EFM32_DATANAK_DELAY       SEC2TICK(5) /* 5 seconds in system ticks */

/* Ever-present MIN/MAX macros */

#ifndef MIN
#  define  MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define  MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/* Tracing *********************************************************************/

#define TR_FMT1 false
#define TR_FMT2 true

#define TRENTRY(id,fmt1,string) {string}

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The following enumeration represents the various states of the USB host
 * state machine (for debug purposes only)
 */

enum efm32_smstate_e
{
  SMSTATE_DETACHED = 0,  /* Not attached to a device */
  SMSTATE_ATTACHED,      /* Attached to a device */
  SMSTATE_ENUM,          /* Attached, enumerating */
  SMSTATE_CLASS_BOUND,   /* Enumeration complete, class bound */
};

/* This enumeration provides the reason for the channel halt. */

enum efm32_chreason_e
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
 * in the structure could be moved in struct efm32_ubhost_s to achieve
 * some memory savings.
 */

struct efm32_chan_s
{
  sem_t             waitsem;   /* Channel wait semaphore */
  volatile uint8_t  result;    /* The result of the transfer */
  volatile uint8_t  chreason;  /* Channel halt reason. See enum efm32_chreason_e */
  uint8_t           chidx;     /* Channel index */
  uint8_t           epno;      /* Device endpoint number (0-127) */
  uint8_t           eptype;    /* See EFM32_USB_EPTYPE_* definitions */
  uint8_t           funcaddr;  /* Device function address */
  uint8_t           speed;     /* Device speed */
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
  FAR uint8_t      *buffer;    /* Transfer buffer pointer */
#ifdef CONFIG_USBHOST_ASYNCH
  usbhost_asynch_t  callback;  /* Transfer complete callback */
  FAR void         *arg;       /* Argument that accompanies the callback */
#endif
};

/* A channel represents on uni-directional endpoint.  So, in the case of the
 * bi-directional, control endpoint, there must be two channels to represent
 * the endpoint.
 */

struct efm32_ctrlinfo_s
{
  uint8_t           inndx;     /* EP0 IN control channel index */
  uint8_t           outndx;    /* EP0 OUT control channel index */
};

/* This structure retains the state of the USB host controller */

struct efm32_usbhost_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to structefm32_usbhost_s.
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
  sem_t             exclsem;   /* Support mutually exclusive access */
  sem_t             pscsem;    /* Semaphore to wait for a port event */
  struct efm32_ctrlinfo_s ep0;  /* Root hub port EP0 description */

#ifdef CONFIG_USBHOST_HUB
  /* Used to pass external hub port events */

  volatile struct usbhost_hubport_s *hport;
#endif

  /* The state of each host channel */

  struct efm32_chan_s chan[EFM32_MAX_TX_FIFOS];
};

#ifdef HAVE_USBHOST_TRACE
/* Format of one trace entry */

struct efm32_usbhost_trace_s
{
#if 0
  uint16_t id;
  bool fmt2;
#endif
  FAR const char *string;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ********************************************************/

#ifdef CONFIG_EFM32_USBHOST_REGDEBUG
static void efm32_printreg(uint32_t addr, uint32_t val, bool iswrite);
static void efm32_checkreg(uint32_t addr, uint32_t val, bool iswrite);
static uint32_t efm32_getreg(uint32_t addr);
static void efm32_putreg(uint32_t addr, uint32_t value);
#else
# define efm32_getreg(addr)     getreg32(addr)
# define efm32_putreg(addr,val) putreg32(val,addr)
#endif

static inline void efm32_modifyreg(uint32_t addr, uint32_t clrbits,
                                   uint32_t setbits);

#ifdef CONFIG_EFM32_USBHOST_PKTDUMP
#  define efm32_pktdump(m,b,n) lib_dumpbuffer(m,b,n)
#else
#  define efm32_pktdump(m,b,n)
#endif

/* Semaphores ******************************************************************/

static void efm32_takesem(sem_t *sem);
#define efm32_givesem(s) nxsem_post(s);

/* Byte stream access helper functions *****************************************/

static inline uint16_t efm32_getle16(const uint8_t *val);

/* Channel management **********************************************************/

static int efm32_chan_alloc(FAR struct efm32_usbhost_s *priv);
static inline void efm32_chan_free(FAR struct efm32_usbhost_s *priv, int chidx);
static inline void efm32_chan_freeall(FAR struct efm32_usbhost_s *priv);
static void efm32_chan_configure(FAR struct efm32_usbhost_s *priv, int chidx);
static void efm32_chan_halt(FAR struct efm32_usbhost_s *priv, int chidx,
                            enum efm32_chreason_e chreason);
static int efm32_chan_waitsetup(FAR struct efm32_usbhost_s *priv,
                                FAR struct efm32_chan_s *chan);
#ifdef CONFIG_USBHOST_ASYNCH
static int efm32_chan_asynchsetup(FAR struct efm32_usbhost_s *priv,
                                  FAR struct efm32_chan_s *chan,
                                  usbhost_asynch_t callback, FAR void *arg);
#endif
static int efm32_chan_wait(FAR struct efm32_usbhost_s *priv,
                           FAR struct efm32_chan_s *chan);
static void efm32_chan_wakeup(FAR struct efm32_usbhost_s *priv,
                              FAR struct efm32_chan_s *chan);
static int efm32_ctrlchan_alloc(FAR struct efm32_usbhost_s *priv,
                                uint8_t epno, uint8_t funcaddr, uint8_t speed,
                                FAR struct efm32_ctrlinfo_s *ctrlep);
static int efm32_ctrlep_alloc(FAR struct efm32_usbhost_s *priv,
                              FAR const struct usbhost_epdesc_s *epdesc,
                              FAR usbhost_ep_t *ep);
static int efm32_xfrep_alloc(FAR struct efm32_usbhost_s *priv,
                              FAR const struct usbhost_epdesc_s *epdesc,
                              FAR usbhost_ep_t *ep);

/* Control/data transfer logic *************************************************/

static void efm32_transfer_start(FAR struct efm32_usbhost_s *priv, int chidx);
#if 0 /* Not used */
static inline uint16_t efm32_getframe(void);
#endif
static int efm32_ctrl_sendsetup(FAR struct efm32_usbhost_s *priv,
                                FAR struct efm32_ctrlinfo_s *ep0,
                                FAR const struct usb_ctrlreq_s *req);
static int efm32_ctrl_senddata(FAR struct efm32_usbhost_s *priv,
                               FAR struct efm32_ctrlinfo_s *ep0,
                               FAR uint8_t *buffer, unsigned int buflen);
static int efm32_ctrl_recvdata(FAR struct efm32_usbhost_s *priv,
                               FAR struct efm32_ctrlinfo_s *ep0,
                               FAR uint8_t *buffer, unsigned int buflen);
static int efm32_in_setup(FAR struct efm32_usbhost_s *priv, int chidx);
static ssize_t efm32_in_transfer(FAR struct efm32_usbhost_s *priv, int chidx,
                                 FAR uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void efm32_in_next(FAR struct efm32_usbhost_s *priv,
                          FAR struct efm32_chan_s *chan);
static int efm32_in_asynch(FAR struct efm32_usbhost_s *priv, int chidx,
                           FAR uint8_t *buffer, size_t buflen,
                           usbhost_asynch_t callback, FAR void *arg);
#endif
static int efm32_out_setup(FAR struct efm32_usbhost_s *priv, int chidx);
static ssize_t efm32_out_transfer(FAR struct efm32_usbhost_s *priv, int chidx,
                                  FAR uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void efm32_out_next(FAR struct efm32_usbhost_s *priv,
                           FAR struct efm32_chan_s *chan);
static int efm32_out_asynch(FAR struct efm32_usbhost_s *priv, int chidx,
                            FAR uint8_t *buffer, size_t buflen,
                            usbhost_asynch_t callback, FAR void *arg);
#endif

/* Interrupt handling **********************************************************/
/* Lower level interrupt handlers */

static void efm32_gint_wrpacket(FAR struct efm32_usbhost_s *priv,
                                FAR uint8_t *buffer, int chidx, int buflen);
static inline void efm32_gint_hcinisr(FAR struct efm32_usbhost_s *priv,
                                      int chidx);
static inline void efm32_gint_hcoutisr(FAR struct efm32_usbhost_s *priv,
                                       int chidx);
static void efm32_gint_connected(FAR struct efm32_usbhost_s *priv);
static void efm32_gint_disconnected(FAR struct efm32_usbhost_s *priv);

/* Second level interrupt handlers */

#ifdef CONFIG_EFM32_OTGFS_SOFINTR
static inline void efm32_gint_sofisr(FAR struct efm32_usbhost_s *priv);
#endif
static inline void efm32_gint_rxflvlisr(FAR struct efm32_usbhost_s *priv);
static inline void efm32_gint_nptxfeisr(FAR struct efm32_usbhost_s *priv);
static inline void efm32_gint_ptxfeisr(FAR struct efm32_usbhost_s *priv);
static inline void efm32_gint_hcisr(FAR struct efm32_usbhost_s *priv);
static inline void efm32_gint_hprtisr(FAR struct efm32_usbhost_s *priv);
static inline void efm32_gint_discisr(FAR struct efm32_usbhost_s *priv);
static inline void efm32_gint_ipxfrisr(FAR struct efm32_usbhost_s *priv);

/* First level, global interrupt handler */

static int efm32_gint_isr(int irq, FAR void *context, FAR void *arg);

/* Interrupt controls */

static void efm32_gint_enable(void);
static void efm32_gint_disable(void);
static inline void efm32_hostinit_enable(void);
static void efm32_txfe_enable(FAR struct efm32_usbhost_s *priv, int chidx);

/* USB host controller operations **********************************************/

static int efm32_wait(FAR struct usbhost_connection_s *conn,
                      FAR struct usbhost_hubport_s **hport);
static int efm32_rh_enumerate(FAR struct efm32_usbhost_s *priv,
                              FAR struct usbhost_connection_s *conn,
                              FAR struct usbhost_hubport_s *hport);
static int efm32_enumerate(FAR struct usbhost_connection_s *conn,
                           FAR struct usbhost_hubport_s *hport);

static int efm32_ep0configure(FAR struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep0, uint8_t funcaddr, uint8_t speed,
                              uint16_t maxpacketsize);
static int efm32_epalloc(FAR struct usbhost_driver_s *drvr,
                         FAR const FAR struct usbhost_epdesc_s *epdesc,
                         FAR usbhost_ep_t *ep);
static int efm32_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int efm32_alloc(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t **buffer, FAR size_t *maxlen);
static int efm32_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);
static int efm32_ioalloc(FAR struct usbhost_driver_s *drvr,
                         FAR uint8_t **buffer, size_t buflen);
static int efm32_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);
static int efm32_ctrlin(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                        const struct usb_ctrlreq_s *req,
                        FAR uint8_t *buffer);
static int efm32_ctrlout(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                         FAR const struct usb_ctrlreq_s *req,
                         FAR const uint8_t *buffer);
static ssize_t efm32_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                              FAR uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static int efm32_asynch(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                        FAR uint8_t *buffer, size_t buflen,
                        usbhost_asynch_t callback, FAR void *arg);
#endif
static int efm32_cancel(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
#ifdef CONFIG_USBHOST_HUB
static int efm32_connect(FAR struct usbhost_driver_s *drvr,
                         FAR struct usbhost_hubport_s *hport,
                         bool connected);
#endif
static void efm32_disconnect(FAR struct usbhost_driver_s *drvr,
                             FAR struct usbhost_hubport_s *hport);

/* Initialization **************************************************************/

static void efm32_portreset(FAR struct efm32_usbhost_s *priv);
static void efm32_flush_txfifos(uint32_t txfnum);
static void efm32_flush_rxfifo(void);
static void efm32_vbusdrive(FAR struct efm32_usbhost_s *priv, bool state);
static void efm32_host_initialize(FAR struct efm32_usbhost_s *priv);

static inline void efm32_sw_initialize(FAR struct efm32_usbhost_s *priv);
static inline int efm32_hw_initialize(FAR struct efm32_usbhost_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* In this driver implementation, support is provided for only a single a single
 * USB device.  All status information can be simply retained in a single global
 * instance.
 */

static struct efm32_usbhost_s g_usbhost;

/* This is the connection/enumeration interface */

static struct usbhost_connection_s g_usbconn =
{
  .wait             = efm32_wait,
  .enumerate        = efm32_enumerate,
};

#ifdef HAVE_USBHOST_TRACE
/* Trace strings */

static const struct efm32_usbhost_trace_s g_trace1[TRACE1_NSTRINGS] =
{
  TRENTRY(USBHOST_TRACE1_DEVDISCONN,         TR_FMT1, "OTGFS ERROR: Host Port %d. Device disconnected\n"),
  TRENTRY(USBHOST_TRACE1_IRQATTACH,          TR_FMT1, "OTGFS ERROR: Failed to attach IRQ\n"),
  TRENTRY(USBHOST_TRACE1_TRNSFRFAILED,       TR_FMT1, "OTGFS  ERROR: Transfer Failed. ret=%d\n"),
  TRENTRY(USBHOST_TRACE1_SENDSETUP,          TR_FMT1, "OTGFS  ERROR: ctrl_sendsetup() failed with: %d\n"),
  TRENTRY(USBHOST_TRACE1_SENDDATA,           TR_FMT1, "OTGFS  ERROR: ctrl_senddata() failed with: %d\n"),
  TRENTRY(USBHOST_TRACE1_RECVDATA,           TR_FMT1, "OTGFS  ERROR: ctrl_recvdata() failed with: %d\n"),

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  TRENTRY(USBHOST_VTRACE1_CONNECTED,         TR_FMT1, "OTGFS Host Port %d connected.\n"),
  TRENTRY(USBHOST_VTRACE1_DISCONNECTED,      TR_FMT1, "OTGFS Host Port %d disconnected.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT,              TR_FMT1, "OTGFS Handling Interrupt. Entry Point.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT_SOF,          TR_FMT1, "OTGFS Handle the start of frame interrupt.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT_RXFLVL,       TR_FMT1, "OTGFS Handle the RxFIFO non-empty interrupt.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT_NPTXFE,       TR_FMT1, "OTGFS Handle the non-periodic TxFIFO empty interrupt.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT_PTXFE,        TR_FMT1, "OTGFS Handle the periodic TxFIFO empty interrupt.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT_HC,           TR_FMT1, "OTGFS Handle the host channels interrupt.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT_HPRT,         TR_FMT1, "OTGFS Handle the host port interrupt.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT_HPRT_POCCHNG, TR_FMT1, "OTGFS  HPRT: Port Over-Current Change.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT_HPRT_PCDET,   TR_FMT1, "OTGFS  HPRT: Port Connect Detect.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT_HPRT_PENCHNG, TR_FMT1, "OTGFS  HPRT: Port Enable Changed.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT_HPRT_LSDEV,   TR_FMT1, "OTGFS  HPRT: Low Speed Device Connected.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT_HPRT_FSDEV,   TR_FMT1, "OTGFS  HPRT: Full Speed Device Connected.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT_HPRT_LSFSSW,  TR_FMT1, "OTGFS  HPRT: Host Switch: LS -> FS.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT_HPRT_FSLSSW,  TR_FMT1, "OTGFS  HPRT: Host Switch: FS -> LS.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT_DISC,         TR_FMT1, "OTGFS Handle the disconnect detected interrupt.\n"),
  TRENTRY(USBHOST_VTRACE1_GINT_IPXFR,        TR_FMT1, "OTGFS Handle the incomplete periodic transfer.\n"),
#endif
};

static const struct efm32_usbhost_trace_s g_trace2[TRACE2_NSTRINGS] =
{
  TRENTRY(USBHOST_TRACE2_CLIP,               TR_FMT2, "OTGFS CLIP: chidx: %d buflen: %d\n"),

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  TRENTRY(USBHOST_VTRACE2_CHANWAKEUP_IN,     TR_FMT2, "OTGFS  EP%d(IN)  wake up with result: %d\n"),
  TRENTRY(USBHOST_VTRACE2_CHANWAKEUP_OUT,    TR_FMT2, "OTGFS  EP%d(OUT) wake up with result: %d\n"),
  TRENTRY(USBHOST_VTRACE2_CTRLIN,            TR_FMT2, "OTGFS CTRL_IN  type: %02x req: %02x\n"),
  TRENTRY(USBHOST_VTRACE2_CTRLOUT,           TR_FMT2, "OTGFS CTRL_OUT type: %02x req: %02x\n"),
  TRENTRY(USBHOST_VTRACE2_INTRIN,            TR_FMT2, "OTGFS INTR_IN  chidx: %02x len: %02x\n"),
  TRENTRY(USBHOST_VTRACE2_INTROUT,           TR_FMT2, "OTGFS INTR_OUT chidx: %02x len: %02x\n"),
  TRENTRY(USBHOST_VTRACE2_BULKIN,            TR_FMT2, "OTGFS BULK_IN  chidx: %02x len: %02x\n"),
  TRENTRY(USBHOST_VTRACE2_BULKOUT,           TR_FMT2, "OTGFS BULK_OUT chidx: %02x len: %02x\n"),
  TRENTRY(USBHOST_VTRACE2_ISOCIN,            TR_FMT2, "OTGFS ISOC_IN  chidx: %02x len: %04d\n"),
  TRENTRY(USBHOST_VTRACE2_ISOCOUT,           TR_FMT2, "OTGFS ISOC_OUT chidx: %02x req: %02x\n"),
  TRENTRY(USBHOST_VTRACE2_STARTTRANSFER,     TR_FMT2, "OTGFS  Transfer chidx: %d buflen: %d\n"),
  TRENTRY(USBHOST_VTRACE2_CHANCONF_CTRL_IN,  TR_FMT2, "OTGFS Channel configured. chidx: %d: (EP%d,IN ,CTRL)\n"),
  TRENTRY(USBHOST_VTRACE2_CHANCONF_CTRL_OUT, TR_FMT2, "OTGFS Channel configured. chidx: %d: (EP%d,OUT,CTRL)\n"),
  TRENTRY(USBHOST_VTRACE2_CHANCONF_INTR_IN,  TR_FMT2, "OTGFS Channel configured. chidx: %d: (EP%d,IN ,INTR)\n"),
  TRENTRY(USBHOST_VTRACE2_CHANCONF_INTR_OUT, TR_FMT2, "OTGFS Channel configured. chidx: %d: (EP%d,OUT,INTR)\n"),
  TRENTRY(USBHOST_VTRACE2_CHANCONF_BULK_IN,  TR_FMT2, "OTGFS Channel configured. chidx: %d: (EP%d,IN ,BULK)\n"),
  TRENTRY(USBHOST_VTRACE2_CHANCONF_BULK_OUT, TR_FMT2, "OTGFS Channel configured. chidx: %d: (EP%d,OUT,BULK)\n"),
  TRENTRY(USBHOST_VTRACE2_CHANCONF_ISOC_IN,  TR_FMT2, "OTGFS Channel configured. chidx: %d: (EP%d,IN ,ISOC)\n"),
  TRENTRY(USBHOST_VTRACE2_CHANCONF_ISOC_OUT, TR_FMT2, "OTGFS Channel configured. chidx: %d: (EP%d,OUT,ISOC)\n"),
  TRENTRY(USBHOST_VTRACE2_CHANHALT,          TR_FMT2, "OTGFS Channel halted. chidx: %d, reason: %d\n"),
#endif
};
#endif /* HAVE_USBHOST_TRACE */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_printreg
 *
 * Description:
 *   Print the contents of an EFM32xx register operation
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_USBHOST_REGDEBUG
static void efm32_printreg(uint32_t addr, uint32_t val, bool iswrite)
{
  uinfo("%08x%s%08x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/****************************************************************************
 * Name: efm32_checkreg
 *
 * Description:
 *   Get the contents of an EFM32 register
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_USBHOST_REGDEBUG
static void efm32_checkreg(uint32_t addr, uint32_t val, bool iswrite)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;
  static bool     prevwrite = false;

  /* Is this the same value that we read from/wrote to the same register last time?
   * Are we polling the register?  If so, suppress the output.
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

              efm32_printreg(prevaddr, preval, prevwrite);
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

      efm32_printreg(addr, val, iswrite);
    }
}
#endif

/****************************************************************************
 * Name: efm32_getreg
 *
 * Description:
 *   Get the contents of an EFM32 register
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_USBHOST_REGDEBUG
static uint32_t efm32_getreg(uint32_t addr)
{
  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Check if we need to print this value */

  efm32_checkreg(addr, val, false);
  return val;
}
#endif

/****************************************************************************
 * Name: efm32_putreg
 *
 * Description:
 *   Set the contents of an EFM32 register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_USBHOST_REGDEBUG
static void efm32_putreg(uint32_t addr, uint32_t val)
{
  /* Check if we need to print this value */

  efm32_checkreg(addr, val, true);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: efm32_modifyreg
 *
 * Description:
 *   Modify selected bits of an EFM32 register.
 *
 ****************************************************************************/

static inline void efm32_modifyreg(uint32_t addr, uint32_t clrbits, uint32_t setbits)
{
  efm32_putreg(addr, (((efm32_getreg(addr)) & ~clrbits) | setbits));
}

/****************************************************************************
 * Name: efm32_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static void efm32_takesem(sem_t *sem)
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
 * Name: efm32_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 ****************************************************************************/

static inline uint16_t efm32_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: efm32_chan_alloc
 *
 * Description:
 *   Allocate a channel.
 *
 ****************************************************************************/

static int efm32_chan_alloc(FAR struct efm32_usbhost_s *priv)
{
  int chidx;

  /* Search the table of channels */

  for (chidx = 0; chidx < EFM32_NHOST_CHANNELS; chidx++)
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
 * Name: efm32_chan_free
 *
 * Description:
 *   Free a previoiusly allocated channel.
 *
 ****************************************************************************/

static void efm32_chan_free(FAR struct efm32_usbhost_s *priv, int chidx)
{
  DEBUGASSERT((unsigned)chidx < EFM32_NHOST_CHANNELS);

  /* Halt the channel */

  efm32_chan_halt(priv, chidx, CHREASON_FREED);

  /* Mark the channel available */

  priv->chan[chidx].inuse = false;
}

/****************************************************************************
 * Name: efm32_chan_freeall
 *
 * Description:
 *   Free all channels.
 *
 ****************************************************************************/

static inline void efm32_chan_freeall(FAR struct efm32_usbhost_s *priv)
{
  uint8_t chidx;

  /* Free all host channels */

  for (chidx = 2; chidx < EFM32_NHOST_CHANNELS; chidx ++)
    {
      efm32_chan_free(priv, chidx);
    }
}

/****************************************************************************
 * Name: efm32_chan_configure
 *
 * Description:
 *   Configure or re-configure a host channel.  Host channels are configured
 *   when endpoint is allocated and EP0 (only) is re-configured with the
 *   max packet size or device address changes.
 *
 ****************************************************************************/

static void efm32_chan_configure(FAR struct efm32_usbhost_s *priv, int chidx)
{
  FAR struct efm32_chan_s *chan = &priv->chan[chidx];
  uint32_t regval;

  /* Clear any old pending interrupts for this host channel. */

  efm32_putreg(EFM32_USB_HCn_INT(chidx), 0xffffffff);

  /* Enable channel interrupts required for transfers on this channel. */

  regval = 0;

  switch (chan->eptype)
    {
    case EFM32_USB_EPTYPE_CTRL:
    case EFM32_USB_EPTYPE_BULK:
      {
#ifdef HAVE_USBHOST_TRACE_VERBOSE
        uint16_t intrace;
        uint16_t outtrace;

        /* Determine the definitive trace ID to use below */

        if (chan->eptype == EFM32_USB_EPTYPE_CTRL)
          {
            intrace  = USBHOST_VTRACE2_CHANCONF_CTRL_IN;
            outtrace = USBHOST_VTRACE2_CHANCONF_CTRL_OUT;
          }
        else
          {
            intrace  = USBHOST_VTRACE2_CHANCONF_BULK_IN;
            outtrace = USBHOST_VTRACE2_CHANCONF_BULK_OUT;
          }
#endif

        /* Interrupts required for CTRL and BULK endpoints */

        regval |= (USB_HC_INTMSK_XFERCOMPLMSK  | USB_HC_INTMSK_STALLMSK |
                   USB_HC_INTMSK_NAKMSK | USB_HC_INTMSK_XACTERRMSK |
                   USB_HC_INTMSK_DATATGLERRMSK);

        /* Additional setting for IN/OUT endpoints */

        if (chan->in)
          {
            usbhost_vtrace2(intrace, chidx, chan->epno);
            regval |= USB_HC_INTMSK_BBLERRMSK;
          }
        else
          {
            usbhost_vtrace2(outtrace, chidx, chan->epno);
          }
      }
      break;

    case EFM32_USB_EPTYPE_INTR:
      {
        /* Interrupts required for INTR endpoints */

        regval |= (USB_HC_INTMSK_XFERCOMPLMSK | USB_HC_INTMSK_STALLMSK |
                   USB_HC_INTMSK_NAKMSK | USB_HC_INTMSK_XACTERRMSK |
                   USB_HC_INTMSK_FRMOVRUNMSK | USB_HC_INTMSK_DATATGLERRMSK);

        /* Additional setting for IN endpoints */

        if (chan->in)
          {
            usbhost_vtrace2(USBHOST_VTRACE2_CHANCONF_INTR_IN, chidx,
                            chan->epno);
            regval |= USB_HC_INTMSK_BBLERRMSK;
          }
#ifdef HAVE_USBHOST_TRACE_VERBOSE
        else
          {
            usbhost_vtrace2(USBHOST_VTRACE2_CHANCONF_INTR_OUT, chidx,
                            chan->epno);
          }
#endif
      }
      break;

    case EFM32_USB_EPTYPE_ISOC:
      {
        /* Interrupts required for ISOC endpoints */

        regval |= (USB_HC_INTMSK_XFERCOMPLMSK | USB_HC_INTMSK_ACKMSK |
                   USB_HC_INTMSK_FRMOVRUNMSK);

        /* Additional setting for IN endpoints */

        if (chan->in)
          {
            usbhost_vtrace2(USBHOST_VTRACE2_CHANCONF_ISOC_IN, chidx,
                            chan->epno);
            regval |= (USB_HC_INTMSK_XACTERRMSK | USB_HC_INTMSK_BBLERRMSK);
          }
#ifdef HAVE_USBHOST_TRACE_VERBOSE
        else
          {
            usbhost_vtrace2(USBHOST_VTRACE2_CHANCONF_ISOC_OUT, chidx,
                            chan->epno);
          }
#endif
      }
      break;
    }

  efm32_putreg(EFM32_USB_HCn_INTMSK(chidx), regval);

  /* Enable the top level host channel interrupt. */

  efm32_modifyreg(EFM32_USB_HAINTMSK, 0, USB_HAINT(chidx));

  /* Make sure host channel interrupts are enabled. */

  efm32_modifyreg(EFM32_USB_GINTMSK, 0, USB_GINTMSK_HCHINTMSK);

  /* Program the HCCHAR register */

  regval = ((uint32_t)chan->maxpacket << _USB_HC_CHAR_MPS_SHIFT) |
           ((uint32_t)chan->epno      << _USB_HC_CHAR_EPNUM_SHIFT) |
           ((uint32_t)chan->eptype    << _USB_HC_CHAR_EPTYPE_SHIFT) |
           ((uint32_t)chan->funcaddr  << _USB_HC_CHAR_DEVADDR_SHIFT);

  /* Special case settings for low speed devices */

  if (chan->speed == USB_SPEED_LOW)
    {
      regval |= USB_HC_CHAR_LSPDDEV;
    }

  /* Special case settings for IN endpoints */

  if (chan->in)
    {
      regval |= USB_HC_CHAR_EPDIR_IN;
    }

  /* Special case settings for INTR endpoints */

  if (chan->eptype == EFM32_USB_EPTYPE_INTR)
    {
      regval |= USB_HC_CHAR_ODDFRM;
    }

  /* Write the channel configuration */

  efm32_putreg(EFM32_USB_HCn_CHAR(chidx), regval);
}

/****************************************************************************
 * Name: efm32_chan_halt
 *
 * Description:
 *   Halt the channel associated with 'chidx' by setting the CHannel DISable
 *   (CHDIS) bit in in the HCCHAR register.
 *
 ****************************************************************************/

static void efm32_chan_halt(FAR struct efm32_usbhost_s *priv, int chidx,
                            enum efm32_chreason_e chreason)
{
  uint32_t hcchar;
  uint32_t intmsk;
  uint32_t eptype;
  unsigned int avail;

  /* Save the reason for the halt.  We need this in the channel halt interrupt
   * handling logic to know what to do next.
   */

  usbhost_vtrace2(USBHOST_VTRACE2_CHANHALT, chidx, chreason);

  priv->chan[chidx].chreason = (uint8_t)chreason;

  /* "The application can disable any channel by programming the OTG_FS_HCCHARx
   *  register with the CHDIS and CHENA bits set to 1. This enables the OTG_FS
   *  host to flush the posted requests (if any) and generates a channel halted
   *  interrupt. The application must wait for the CHH interrupt in OTG_FS_HCINTx
   *  before reallocating the channel for other transactions.  The OTG_FS host
   *  does not interrupt the transaction that has already been started on the
   *  USB."
   */

  hcchar  = efm32_getreg(EFM32_USB_HCn_CHAR(chidx));
  hcchar |= (USB_HC_CHAR_CHDIS | USB_HC_CHAR_CHENA);

  /* Get the endpoint type from the HCCHAR register */

  eptype = hcchar & _USB_HC_CHAR_EPTYPE_MASK;

  /* Check for space in the Tx FIFO to issue the halt.
   *
   * "Before disabling a channel, the application must ensure that there is at
   *  least one free space available in the non-periodic request queue (when
   *  disabling a non-periodic channel) or the periodic request queue (when
   *  disabling a periodic channel). The application can simply flush the
   *  posted requests when the Request queue is full (before disabling the
   *  channel), by programming the OTG_FS_HCCHARx register with the CHDIS bit
   *  set to 1, and the CHENA bit cleared to 0.
   */

  if (eptype == USB_HC_CHAR_EPTYPE_CONTROL || eptype == USB_HC_CHAR_EPTYPE_BULK)
    {
      /* Get the number of words available in the non-periodic Tx FIFO. */

      avail = efm32_getreg(EFM32_USB_GNPTXSTS) & _USB_GNPTXSTS_NPTXFSPCAVAIL_MASK;
    }
  else /* if (eptype == USB_HCCHAR_EPTYP_ISOC || eptype == USB_HC_CHAR_EPTYPE_INT) */
    {
      /* Get the number of words available in the non-periodic Tx FIFO. */

      avail = efm32_getreg(EFM32_USB_HPTXSTS) & _USB_HPTXSTS_PTXFSPCAVAIL_MASK;
    }

  /* Check if there is any space available in the Tx FIFO. */

  if (avail == 0)
    {
      /* The Tx FIFO is full... disable the channel to flush the requests */

      hcchar &= ~USB_HC_CHAR_CHENA;
    }

  /* Unmask the CHannel Halted (CHH) interrupt */

  intmsk  = efm32_getreg(EFM32_USB_HCn_INTMSK(chidx));
  intmsk |= USB_HC_INTMSK_CHHLTDMSK;
  efm32_putreg(EFM32_USB_HCn_INTMSK(chidx), intmsk);

  /* Halt the channel by setting CHDIS (and maybe CHENA) in the HCCHAR */

  efm32_putreg(EFM32_USB_HCn_CHAR(chidx), hcchar);
}

/****************************************************************************
 * Name: efm32_chan_waitsetup
 *
 * Description:
 *   Set the request for the transfer complete event well BEFORE enabling the
 *   transfer (as soon as we are absolutely committed to the to avoid transfer).
 *   We do this to minimize race conditions.  This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 * Assumptions:
 *   Called from a normal thread context BEFORE the transfer has been started.
 *
 ****************************************************************************/

static int efm32_chan_waitsetup(FAR struct efm32_usbhost_s *priv,
                                FAR struct efm32_chan_s *chan)
{
  irqstate_t flags = enter_critical_section();
  int        ret   = -ENODEV;

  /* Is the device still connected? */

  if (priv->connected)
    {
      /* Yes.. then set waiter to indicate that we expect to be informed when
       * either (1) the device is disconnected, or (2) the transfer completed.
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
 * Name: efm32_chan_asynchsetup
 *
 * Description:
 *   Set the request for the transfer complete event well BEFORE enabling the
 *   transfer (as soon as we are absolutely committed to the to avoid transfer).
 *   We do this to minimize race conditions.  This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 * Assumptions:
 *   Might be called from the level of an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int efm32_chan_asynchsetup(FAR struct efm32_usbhost_s *priv,
                                  FAR struct efm32_chan_s *chan,
                                  usbhost_asynch_t callback, FAR void *arg)
{
  irqstate_t flags = enter_critical_section();
  int        ret   = -ENODEV;

  /* Is the device still connected? */

  if (priv->connected)
    {
      /* Yes.. then set waiter to indicate that we expect to be informed when
       * either (1) the device is disconnected, or (2) the transfer completed.
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
 * Name: efm32_chan_wait
 *
 * Description:
 *   Wait for a transfer on a channel to complete.
 *
 * Assumptions:
 *   Called from a normal thread context
 *
 ****************************************************************************/

static int efm32_chan_wait(FAR struct efm32_usbhost_s *priv,
                           FAR struct efm32_chan_s *chan)
{
  irqstate_t flags;
  int ret;

  /* Disable interrupts so that the following operations will be atomic.  On
   * the OTG FS global interrupt needs to be disabled.  However, here we disable
   * all interrupts to exploit that fact that interrupts will be re-enabled
   * while we wait.
   */

  flags = enter_critical_section();

  /* Loop, testing for an end of transfer condition.  The channel 'result'
   * was set to EBUSY and 'waiter' was set to true before the transfer; 'waiter'
   * will be set to false and 'result' will be set appropriately when the
   * transfer is completed.
   */

  do
    {
      /* Wait for the transfer to complete.  NOTE the transfer may already
       * completed before we get here or the transfer may complete while we
       * wait here.
       */

      ret = nxsem_wait(&chan->waitsem);

      /* nxsem_wait should succeed.  But it is possible that we could be
       * awakened by a signal too.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (chan->waiter);

  /* The transfer is complete re-enable interrupts and return the result */

  ret = -(int)chan->result;
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: efm32_chan_wakeup
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

static void efm32_chan_wakeup(FAR struct efm32_usbhost_s *priv,
                              FAR struct efm32_chan_s *chan)
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

          usbhost_vtrace2(chan->in ? USBHOST_VTRACE2_CHANWAKEUP_IN :
                                     USBHOST_VTRACE2_CHANWAKEUP_OUT,
                          chan->epno, chan->result);

          efm32_givesem(&chan->waitsem);
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
              efm32_in_next(priv, chan);
            }
          else
            {
              efm32_out_next(priv, chan);
            }
        }
#endif
    }
}

/****************************************************************************
 * Name: efm32_ctrlchan_alloc
 *
 * Description:
 *   Allocate and configured channels for a control pipe.
 *
 ****************************************************************************/

static int efm32_ctrlchan_alloc(FAR struct efm32_usbhost_s *priv,
                                uint8_t epno, uint8_t funcaddr, uint8_t speed,
                                FAR struct efm32_ctrlinfo_s *ctrlep)
{
  FAR struct efm32_chan_s *chan;
  int inndx;
  int outndx;

  outndx = efm32_chan_alloc(priv);
  if (outndx < 0)
    {
      return -ENOMEM;
    }

  ctrlep->outndx  = outndx;
  chan            = &priv->chan[outndx];
  chan->epno      = epno;
  chan->in        = false;
  chan->eptype    = EFM32_USB_EPTYPE_CTRL;
  chan->funcaddr  = funcaddr;
  chan->speed     = speed;
  chan->maxpacket = EFM32_EP0_DEF_PACKET_SIZE;
  chan->indata1   = false;
  chan->outdata1  = false;

  /* Configure control OUT channels */

  efm32_chan_configure(priv, outndx);

  /* Allocate and initialize the control IN channel */

  inndx = efm32_chan_alloc(priv);
  if (inndx < 0)
    {
      efm32_chan_free(priv, outndx);
      return -ENOMEM;
    }

  ctrlep->inndx   = inndx;
  chan            = &priv->chan[inndx];
  chan->epno      = epno;
  chan->in        = true;
  chan->eptype    = EFM32_USB_EPTYPE_CTRL;
  chan->funcaddr  = funcaddr;
  chan->speed     = speed;
  chan->maxpacket = EFM32_EP0_DEF_PACKET_SIZE;
  chan->indata1   = false;
  chan->outdata1  = false;

  /* Configure control IN channels */

  efm32_chan_configure(priv, inndx);
  return OK;
}

/****************************************************************************
 * Name: efm32_ctrlep_alloc
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
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int efm32_ctrlep_alloc(FAR struct efm32_usbhost_s *priv,
                              FAR const struct usbhost_epdesc_s *epdesc,
                              FAR usbhost_ep_t *ep)
{
  FAR struct usbhost_hubport_s *hport;
  FAR struct efm32_ctrlinfo_s *ctrlep;
  int ret;

  /* Sanity check.  NOTE that this method should only be called if a device is
   * connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(epdesc->hport != NULL);
  hport = epdesc->hport;

  /* Allocate a container for the control endpoint */

  ctrlep = (FAR struct efm32_ctrlinfo_s *)kmm_malloc(sizeof(struct efm32_ctrlinfo_s));
  if (ctrlep == NULL)
    {
      uerr("ERROR: Failed to allocate control endpoint container\n");
      return -ENOMEM;
    }

  /* Then allocate and configure the IN/OUT channnels  */

  ret = efm32_ctrlchan_alloc(priv, epdesc->addr & USB_EPNO_MASK,
                             hport->funcaddr, hport->speed, ctrlep);
  if (ret < 0)
    {
      uerr("ERROR: efm32_ctrlchan_alloc failed: %d\n", ret);
      kmm_free(ctrlep);
      return ret;
    }

  /* Return a pointer to the control pipe container as the pipe "handle" */

  *ep = (usbhost_ep_t)ctrlep;
  return OK;
}

/************************************************************************************
 * Name: efm32_xfrep_alloc
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
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int efm32_xfrep_alloc(FAR struct efm32_usbhost_s *priv,
                              FAR const struct usbhost_epdesc_s *epdesc,
                              FAR usbhost_ep_t *ep)
{
  struct usbhost_hubport_s *hport;
  FAR struct efm32_chan_s *chan;
  int chidx;

  /* Sanity check.  NOTE that this method should only be called if a device is
   * connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(epdesc->hport != NULL);
  hport = epdesc->hport;

  /* Allocate a host channel for the endpoint */

  chidx = efm32_chan_alloc(priv);
  if (chidx < 0)
    {
      uerr("ERROR: Failed to allocate a host channel\n");
      return -ENOMEM;
    }

  /* Decode the endpoint descriptor to initialize the channel data structures.
   * Note:  Here we depend on the fact that the endpoint point type is
   * encoded in the same way in the endpoint descriptor as it is in the OTG
   * HS hardware.
   */

  chan            = &priv->chan[chidx];
  chan->epno      = epdesc->addr & USB_EPNO_MASK;
  chan->in        = epdesc->in;
  chan->eptype    = epdesc->xfrtype;
  chan->funcaddr  = hport->funcaddr;
  chan->speed     = hport->speed;
  chan->maxpacket = epdesc->mxpacketsize;
  chan->indata1   = false;
  chan->outdata1  = false;

  /* Then configure the endpoint */

  efm32_chan_configure(priv, chidx);

  /* Return the index to the allocated channel as the endpoint "handle" */

  *ep = (usbhost_ep_t)chidx;
  return OK;
}

/****************************************************************************
 * Name: efm32_transfer_start
 *
 * Description:
 *   Start at transfer on the select IN or OUT channel.
 *
 ****************************************************************************/

static void efm32_transfer_start(FAR struct efm32_usbhost_s *priv, int chidx)
{
  FAR struct efm32_chan_s *chan;
  uint32_t regval;
  unsigned int npackets;
  unsigned int maxpacket;
  unsigned int avail;
  unsigned int wrsize;
  unsigned int minsize;

  /* Set up the initial state of the transfer */

  chan           = &priv->chan[chidx];

  usbhost_vtrace2(USBHOST_VTRACE2_STARTTRANSFER, chidx, chan->buflen);

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

      if (npackets > EFM32_MAX_PKTCOUNT)
        {
          npackets = EFM32_MAX_PKTCOUNT;
          chan->buflen = EFM32_MAX_PKTCOUNT * maxpacket;
          usbhost_trace2(USBHOST_TRACE2_CLIP, chidx, chan->buflen);
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

  /* Setup the HCn_TSIZ register */

  regval = ((uint32_t)chan->buflen << _USB_HC_TSIZ_XFERSIZE_SHIFT) |
           ((uint32_t)npackets << _USB_HC_TSIZ_PKTCNT_SHIFT) |
           ((uint32_t)chan->pid << _USB_HC_TSIZ_PID_SHIFT);
  efm32_putreg(EFM32_USB_HCn_TSIZ(chidx), regval);

  /* Setup the HCCHAR register: Frame oddness and host channel enable */

  regval = efm32_getreg(EFM32_USB_HCn_CHAR(chidx));

  /* Set/clear the Odd Frame bit.  Check for an even frame; if so set Odd
   * Frame. This field is applicable for only periodic (isochronous and
   * interrupt) channels.
   */

  if ((efm32_getreg(EFM32_USB_HFNUM) & 1) == 0)
    {
      regval |= USB_HC_CHAR_ODDFRM;
    }
  else
    {
      regval &= ~USB_HC_CHAR_ODDFRM;
    }

  regval &= ~USB_HC_CHAR_CHDIS;
  regval |= USB_HC_CHAR_CHENA;
  efm32_putreg(EFM32_USB_HCn_CHAR(chidx), regval);

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
        case EFM32_USB_EPTYPE_CTRL: /* Non periodic transfer */
        case EFM32_USB_EPTYPE_BULK:
          {
            /* Read the Non-periodic Tx FIFO status register */

            regval = efm32_getreg(EFM32_USB_GNPTXSTS);
            avail  = ((regval & _USB_GNPTXSTS_NPTXFSPCAVAIL_MASK) >> _USB_GNPTXSTS_NPTXFSPCAVAIL_SHIFT) << 2;
          }
          break;

        /* Periodic transfer */

        case EFM32_USB_EPTYPE_INTR:
        case EFM32_USB_EPTYPE_ISOC:
          {
            /* Read the Non-periodic Tx FIFO status register */

            regval = efm32_getreg(EFM32_USB_HPTXSTS);
            avail  = ((regval & _USB_HPTXSTS_PTXFSPCAVAIL_MASK) >> _USB_HPTXSTS_PTXFSPCAVAIL_SHIFT) << 2;
          }
          break;

        default:
          DEBUGASSERT(false);
          return;
        }

      /* Is there space in the TxFIFO to hold the minimum size packet? */

      if (minsize <= avail)
        {
          /* Yes.. Get the size of the biggest thing that we can put in the Tx FIFO now */

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

          efm32_gint_wrpacket(priv, chan->buffer, chidx, wrsize);
        }

      /* Did we put the entire buffer into the Tx FIFO? */

      if (chan->buflen > avail)
        {
          /* No, there was insufficient space to hold the entire transfer ...
           * Enable the Tx FIFO interrupt to handle the transfer when the Tx
           * FIFO becomes empty.
           */

           efm32_txfe_enable(priv, chidx);
        }
    }
}

/****************************************************************************
 * Name: efm32_getframe
 *
 * Description:
 *   Get the current frame number.  The frame number (FRNUM) field increments
 *   when a new SOF is transmitted on the USB, and is cleared to 0 when it
 *   reaches 0x3fff.
 *
 ****************************************************************************/

#if 0 /* Not used */
static inline uint16_t efm32_getframe(void)
{
  return (uint16_t)(efm32_getreg(EFM32_USB_HFNUM) & _USB_HFNUM_FRNUM_MASK);
}
#endif

/****************************************************************************
 * Name: efm32_ctrl_sendsetup
 *
 * Description:
 *   Send an IN/OUT SETUP packet.
 *
 ****************************************************************************/

static int efm32_ctrl_sendsetup(FAR struct efm32_usbhost_s *priv,
                                FAR struct efm32_ctrlinfo_s *ep0,
                                FAR const struct usb_ctrlreq_s *req)
{
  FAR struct efm32_chan_s *chan;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Loop while the device reports NAK (and a timeout is not exceeded */

  chan  = &priv->chan[ep0->outndx];
  start = clock_systimer();

  do
    {
      /* Send the  SETUP packet */

      chan->pid    = EFM32_USB_PID_SETUP;
      chan->buffer = (FAR uint8_t *)req;
      chan->buflen = USB_SIZEOF_CTRLREQ;
      chan->xfrd   = 0;

      /* Set up for the wait BEFORE starting the transfer */

      ret = efm32_chan_waitsetup(priv, chan);
      if (ret < 0)
        {
          usbhost_trace1(USBHOST_TRACE1_DEVDISCONN, 0);
          return ret;
        }

      /* Start the transfer */

      efm32_transfer_start(priv, ep0->outndx);

      /* Wait for the transfer to complete */

      ret = efm32_chan_wait(priv, chan);

      /* Return on success and for all failures other than EAGAIN.  EAGAIN
       * means that the device NAKed the SETUP command and that we should
       * try a few more times.
       */

      if (ret != -EAGAIN)
        {
          /* Output some debug information if the transfer failed */

          if (ret < 0)
            {
              usbhost_trace1(USBHOST_TRACE1_TRNSFRFAILED, ret);
            }

          /* Return the result in any event */

          return ret;
        }

      /* Get the elapsed time (in frames) */

     elapsed = clock_systimer() - start;
    }
  while (elapsed < EFM32_SETUP_DELAY);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: efm32_ctrl_senddata
 *
 * Description:
 *   Send data in the data phase of an OUT control transfer.  Or send status
 *   in the status phase of an IN control transfer
 *
 ****************************************************************************/

static int efm32_ctrl_senddata(FAR struct efm32_usbhost_s *priv,
                               FAR struct efm32_ctrlinfo_s *ep0,
                               FAR uint8_t *buffer, unsigned int buflen)
{
  FAR struct efm32_chan_s *chan = &priv->chan[ep0->outndx];
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

  chan->pid = chan->outdata1 ? EFM32_USB_PID_DATA1 : EFM32_USB_PID_DATA0;

  /* Set up for the wait BEFORE starting the transfer */

  ret = efm32_chan_waitsetup(priv, chan);
  if (ret < 0)
    {
      usbhost_trace1(USBHOST_TRACE1_DEVDISCONN, 0);
      return ret;
    }

  /* Start the transfer */

  efm32_transfer_start(priv, ep0->outndx);

  /* Wait for the transfer to complete and return the result */

  return efm32_chan_wait(priv, chan);
}

/****************************************************************************
 * Name: efm32_ctrl_recvdata
 *
 * Description:
 *   Receive data in the data phase of an IN control transfer.  Or receive status
 *   in the status phase of an OUT control transfer
 *
 ****************************************************************************/

static int efm32_ctrl_recvdata(FAR struct efm32_usbhost_s *priv,
                               FAR struct efm32_ctrlinfo_s *ep0,
                               FAR uint8_t *buffer, unsigned int buflen)
{
  FAR struct efm32_chan_s *chan = &priv->chan[ep0->inndx];
  int ret;

  /* Save buffer information */

  chan->pid    = EFM32_USB_PID_DATA1;
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;

  /* Set up for the wait BEFORE starting the transfer */

  ret = efm32_chan_waitsetup(priv, chan);
  if (ret < 0)
    {
      usbhost_trace1(USBHOST_TRACE1_DEVDISCONN, 0);
      return ret;
    }

  /* Start the transfer */

  efm32_transfer_start(priv, ep0->inndx);

  /* Wait for the transfer to complete and return the result */

  return efm32_chan_wait(priv, chan);
}

/****************************************************************************
 * Name: efm32_in_setup
 *
 * Description:
 *   Initiate an IN transfer on an bulk, interrupt, or isochronous pipe.
 *
 ****************************************************************************/

static int efm32_in_setup(FAR struct efm32_usbhost_s *priv, int chidx)
{
  FAR struct efm32_chan_s *chan;

  /* Set up for the transfer based on the direction and the endpoint type */

  chan = &priv->chan[chidx];
  switch (chan->eptype)
    {
    default:
    case EFM32_USB_EPTYPE_CTRL: /* Control */
      {
        /* This kind of transfer on control endpoints other than EP0 are not
         * currently supported
         */

        return -ENOSYS;
      }

    case EFM32_USB_EPTYPE_ISOC: /* Isochronous */
      {
        /* Set up the IN data PID */

        usbhost_vtrace2(USBHOST_VTRACE2_ISOCIN, chidx, chan->buflen);
        chan->pid = EFM32_USB_PID_DATA0;
      }
      break;

    case EFM32_USB_EPTYPE_BULK: /* Bulk */
      {
        /* Setup the IN data PID */

        usbhost_vtrace2(USBHOST_VTRACE2_BULKIN, chidx, chan->buflen);
        chan->pid = chan->indata1 ? EFM32_USB_PID_DATA1 : EFM32_USB_PID_DATA0;
      }
      break;

    case EFM32_USB_EPTYPE_INTR: /* Interrupt */
      {
        /* Setup the IN data PID */

        usbhost_vtrace2(USBHOST_VTRACE2_INTRIN, chidx, chan->buflen);
        chan->pid = chan->indata1 ? EFM32_USB_PID_DATA1 : EFM32_USB_PID_DATA0;
      }
      break;
    }

  /* Start the transfer */

  efm32_transfer_start(priv, chidx);
  return OK;
}

/****************************************************************************
 * Name: efm32_in_transfer
 *
 * Description:
 *   Transfer 'buflen' bytes into 'buffer' from an IN channel.
 *
 ****************************************************************************/

static ssize_t efm32_in_transfer(FAR struct efm32_usbhost_s *priv, int chidx,
                                 FAR uint8_t *buffer, size_t buflen)
{
  FAR struct efm32_chan_s *chan;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Loop until the transfer completes (i.e., buflen is decremented to zero)
   * or a fatal error occurs (any error other than a simple NAK)
   */

  chan         = &priv->chan[chidx];
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;

  start = clock_systimer();
  while (chan->xfrd < chan->buflen)
    {
      /* Set up for the wait BEFORE starting the transfer */

      ret = efm32_chan_waitsetup(priv, chan);
      if (ret < 0)
        {
          usbhost_trace1(USBHOST_TRACE1_DEVDISCONN, 0);
          return  (ssize_t)ret;
        }

      /* Set up for the transfer based on the direction and the endpoint type */

      ret = efm32_in_setup(priv, chidx);
      if (ret < 0)
        {
          uerr("ERROR: efm32_in_setup failed: %d\n", ret);
          return  (ssize_t)ret;
        }

      /* Wait for the transfer to complete and get the result */

      ret = efm32_chan_wait(priv, chan);

      /* EAGAIN indicates that the device NAKed the transfer and we need
       * do try again.  Anything else (success or other errors) will
       * cause use to return
       */

      if (ret < 0)
        {
          usbhost_trace1(USBHOST_TRACE1_TRNSFRFAILED, ret);

          /* Check for a special case:  If (1) the transfer was NAKed and (2)
           * no Tx FIFO empty or Rx FIFO not-empty event occurred, then we
           * should be able to just flush the Rx and Tx FIFOs and try again.
           * We can detect this latter case because then the transfer buffer
           * pointer and buffer size will be unaltered.
           */

          elapsed = clock_systimer() - start;
          if (ret != -EAGAIN ||                  /* Not a NAK condition OR */
              elapsed >= EFM32_DATANAK_DELAY ||  /* Timeout has elapsed OR */
              chan->xfrd > 0)                    /* Data has been partially transferred */
            {
              /* Break out and return the error */

              uerr("ERROR: efm32_chan_wait failed: %d\n", ret);
              return (ssize_t)ret;
            }
        }
    }

  return (ssize_t)chan->xfrd;
}

/****************************************************************************
 * Name: efm32_in_next
 *
 * Description:
 *   Initiate the next of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is always called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void efm32_in_next(FAR struct efm32_usbhost_s *priv,
                          FAR struct efm32_chan_s *chan)
{
  usbhost_asynch_t callback;
  FAR void *arg;
  ssize_t nbytes;
  int result;
  int ret;

  /* Is the full transfer complete? Did the last chunk transfer complete OK? */

  result = -(int)chan->result;
  if (chan->xfrd < chan->buflen && result == OK)
    {
      /* Yes.. Set up for the next transfer based on the direction and the
       * endpoint type
       */

      ret = efm32_in_setup(priv, chan->chidx);
      if (ret >= 0)
        {
          return;
        }

      uerr("ERROR: efm32_in_setup failed: %d\n", ret);
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
 * Name: efm32_in_asynch
 *
 * Description:
 *   Initiate the first of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is never called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int efm32_in_asynch(FAR struct efm32_usbhost_s *priv, int chidx,
                           FAR uint8_t *buffer, size_t buflen,
                           usbhost_asynch_t callback, FAR void *arg)
{
  FAR struct efm32_chan_s *chan;
  int ret;

  /* Set up for the transfer data and callback BEFORE starting the first transfer */

  chan         = &priv->chan[chidx];
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;

  ret = efm32_chan_asynchsetup(priv, chan, callback, arg);
  if (ret < 0)
    {
      uerr("ERROR: efm32_chan_asynchsetup failed: %d\n", ret);
      return ret;
    }

  /* Set up for the transfer based on the direction and the endpoint type */

  ret = efm32_in_setup(priv, chidx);
  if (ret < 0)
    {
      uerr("ERROR: efm32_in_setup failed: %d\n", ret);
    }

  /* And return with the transfer pending */

  return ret;
}
#endif

/****************************************************************************
 * Name: efm32_out_setup
 *
 * Description:
 *   Initiate an OUT transfer on an bulk, interrupt, or isochronous pipe.
 *
 ****************************************************************************/

static int efm32_out_setup(FAR struct efm32_usbhost_s *priv, int chidx)
{
  FAR struct efm32_chan_s *chan;

  /* Set up for the transfer based on the direction and the endpoint type */

  chan = &priv->chan[chidx];
  switch (chan->eptype)
    {
    default:
    case EFM32_USB_EPTYPE_CTRL: /* Control */
      {
        /* This kind of transfer on control endpoints other than EP0 are not
         * currently supported
         */

        return -ENOSYS;
      }

    case EFM32_USB_EPTYPE_ISOC: /* Isochronous */
      {
        /* Set up the OUT data PID */

        usbhost_vtrace2(USBHOST_VTRACE2_ISOCOUT, chidx, chan->buflen);
        chan->pid = EFM32_USB_PID_DATA0;
      }
      break;

    case EFM32_USB_EPTYPE_BULK: /* Bulk */
      {
        /* Setup the OUT data PID */

        usbhost_vtrace2(USBHOST_VTRACE2_BULKOUT, chidx, chan->buflen);
        chan->pid = chan->outdata1 ? EFM32_USB_PID_DATA1 : EFM32_USB_PID_DATA0;
      }
      break;

    case EFM32_USB_EPTYPE_INTR: /* Interrupt */
      {
        /* Setup the OUT data PID */

        usbhost_vtrace2(USBHOST_VTRACE2_INTROUT, chidx, chan->buflen);
        chan->pid = chan->outdata1 ? EFM32_USB_PID_DATA1 : EFM32_USB_PID_DATA0;

        /* Toggle the OUT data PID for the next transfer */

        chan->outdata1 ^= true;
      }
      break;
    }

  /* Start the transfer */

  efm32_transfer_start(priv, chidx);
  return OK;
}

/****************************************************************************
 * Name: efm32_out_transfer
 *
 * Description:
 *   Transfer the 'buflen' bytes in 'buffer' through an OUT channel.
 *
 ****************************************************************************/

static ssize_t efm32_out_transfer(FAR struct efm32_usbhost_s *priv, int chidx,
                                  FAR uint8_t *buffer, size_t buflen)
{
  FAR struct efm32_chan_s *chan;
  clock_t start;
  clock_t elapsed;
  size_t xfrlen;
  ssize_t xfrd;
  int ret;

  /* Loop until the transfer completes (i.e., buflen is decremented to zero)
   * or a fatal error occurs (any error other than a simple NAK)
   */

  chan  = &priv->chan[chidx];
  start = clock_systimer();
  xfrd  = 0;

  while (buflen > 0)
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

      ret = efm32_chan_waitsetup(priv, chan);
      if (ret < 0)
        {
          usbhost_trace1(USBHOST_TRACE1_DEVDISCONN, 0);
          return (ssize_t)ret;
        }

      /* Set up for the transfer based on the direction and the endpoint type */

      ret = efm32_out_setup(priv, chidx);
      if (ret < 0)
        {
          uerr("ERROR: efm32_out_setup failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Wait for the transfer to complete and get the result */

      ret = efm32_chan_wait(priv, chan);

      /* Handle transfer failures */

      if (ret < 0)
        {
          usbhost_trace1(USBHOST_TRACE1_TRNSFRFAILED, ret);

          /* Check for a special case:  If (1) the transfer was NAKed and (2)
           * no Tx FIFO empty or Rx FIFO not-empty event occurred, then we
           * should be able to just flush the Rx and Tx FIFOs and try again.
           * We can detect this latter case because then the transfer buffer
           * pointer and buffer size will be unaltered.
           */

          elapsed = clock_systimer() - start;
          if (ret != -EAGAIN ||                  /* Not a NAK condition OR */
              elapsed >= EFM32_DATANAK_DELAY ||  /* Timeout has elapsed OR */
              chan->xfrd > 0)                    /* Data has been partially transferred */
            {
              /* Break out and return the error */

              uerr("ERROR: efm32_chan_wait failed: %d\n", ret);
              return (ssize_t)ret;
            }

          /* Is this flush really necessary? What does the hardware do with the
           * data in the FIFO when the NAK occurs?  Does it discard it?
           */

          efm32_flush_txfifos(USB_GRSTCTL_TXFNUM_FALL);

          /* Get the device a little time to catch up.  Then retry the transfer
           * using the same buffer pointer and length.
           */

          nxsig_usleep(20*1000);
        }
      else
        {
          /* Successfully transferred.  Update the buffer pointer and length */

          buffer += xfrlen;
          buflen -= xfrlen;
          xfrd   += chan->xfrd;
        }
    }

  return xfrd;
}

/****************************************************************************
 * Name: efm32_out_next
 *
 * Description:
 *   Initiate the next of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is always called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void efm32_out_next(FAR struct efm32_usbhost_s *priv,
                           FAR struct efm32_chan_s *chan)
{
  usbhost_asynch_t callback;
  FAR void *arg;
  ssize_t nbytes;
  int result;
  int ret;

  /* Is the full transfer complete? Did the last chunk transfer complete OK? */

  result = -(int)chan->result;
  if (chan->xfrd < chan->buflen && result == OK)
    {
      /* Yes.. Set up for the next transfer based on the direction and the
       * endpoint type
       */

      ret = efm32_out_setup(priv, chan->chidx);
      if (ret >= 0)
        {
          return;
        }

      uerr("ERROR: efm32_out_setup failed: %d\n", ret);
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
 * Name: efm32_out_asynch
 *
 * Description:
 *   Initiate the first of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is never called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int efm32_out_asynch(FAR struct efm32_usbhost_s *priv, int chidx,
                            FAR uint8_t *buffer, size_t buflen,
                            usbhost_asynch_t callback, FAR void *arg)
{
  FAR struct efm32_chan_s *chan;
  int ret;

  /* Set up for the transfer data and callback BEFORE starting the first transfer */

  chan         = &priv->chan[chidx];
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;

  ret = efm32_chan_asynchsetup(priv, chan, callback, arg);
  if (ret < 0)
    {
      uerr("ERROR: efm32_chan_asynchsetup failed: %d\n", ret);
      return ret;
    }

  /* Set up for the transfer based on the direction and the endpoint type */

  ret = efm32_out_setup(priv, chidx);
  if (ret < 0)
    {
      uerr("ERROR: efm32_out_setup failed: %d\n", ret);
    }

  /* And return with the transfer pending */

  return ret;
}
#endif

/****************************************************************************
 * Name: efm32_gint_wrpacket
 *
 * Description:
 *   Transfer the 'buflen' bytes in 'buffer' to the Tx FIFO associated with
 *   'chidx' (non-DMA).
 *
 ****************************************************************************/

static void efm32_gint_wrpacket(FAR struct efm32_usbhost_s *priv,
                                FAR uint8_t *buffer, int chidx, int buflen)
{
  FAR uint32_t *src;
  uint32_t fifo;
  int buflen32;

  efm32_pktdump("Sending", buffer, buflen);

  /* Get the number of 32-byte words associated with this byte size */

  buflen32 = (buflen + 3) >> 2;

  /* Get the address of the Tx FIFO associated with this channel */

  fifo = EFM32_USB_FIFO_BASE(chidx);

  /* Transfer all of the data into the Tx FIFO */

  src = (FAR uint32_t *)buffer;
  for (; buflen32 > 0; buflen32--)
    {
      uint32_t data = *src++;
      efm32_putreg(fifo, data);
    }

  /* Increment the count of bytes "in-flight" in the Tx FIFO */

  priv->chan[chidx].inflight += buflen;
}

/****************************************************************************
 * Name: efm32_gint_hcinisr
 *
 * Description:
 *   USB OTG FS host IN channels interrupt handler
 *
 *   One the completion of the transfer, the channel result byte may be set as
 *   follows:
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

static inline void efm32_gint_hcinisr(FAR struct efm32_usbhost_s *priv,
                                      int chidx)
{
  FAR struct efm32_chan_s *chan = &priv->chan[chidx];
  uint32_t regval;
  uint32_t pending;

  /* Read the HCINT register to get the pending HC interrupts.  Read the
   * HCINTMSK register to get the set of enabled HC interrupts.
   */

  pending = efm32_getreg(EFM32_USB_HCn_INT(chidx));
  regval  = efm32_getreg(EFM32_USB_HCn_INTMSK(chidx));

  /* AND the two to get the set of enabled, pending HC interrupts */

  pending &= regval;
  uinfo("HCINTMSK%d: %08x pending: %08x\n", chidx, regval, pending);

  /* Check for a pending ACK response received/transmitted (ACK) interrupt */

  if ((pending & USB_HC_INT_ACK) != 0)
    {
      /* Clear the pending the ACK response received/transmitted (ACK) interrupt */

      efm32_putreg(EFM32_USB_HCn_INT(chidx), USB_HC_INT_ACK);
    }

  /* Check for a pending STALL response receive (STALL) interrupt */

  else if ((pending & USB_HC_INT_STALL) != 0)
    {
      /* Clear the NAK and STALL Conditions. */

      efm32_putreg(EFM32_USB_HCn_INT(chidx), (USB_HC_INT_NAK | USB_HC_INT_STALL));

      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      efm32_chan_halt(priv, chidx, CHREASON_STALL);

      /* When there is a STALL, clear any pending NAK so that it is not
       * processed below.
       */

      pending &= ~USB_HC_INT_NAK;
    }

  /* Check for a pending Data Toggle ERRor (DTERR) interrupt */

  else if ((pending & USB_HC_INT_DATATGLERR) != 0)
    {
      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      efm32_chan_halt(priv, chidx, CHREASON_DTERR);

      /* Clear the NAK and data toggle error conditions */

      efm32_putreg(EFM32_USB_HCn_INT(chidx), (USB_HC_INT_NAK | USB_HC_INT_DATATGLERR));
    }

  /* Check for a pending FRaMe OverRun (FRMOR) interrupt */

  if ((pending & USB_HC_INT_FRMOVRUN) != 0)
    {
      /* Halt the channel -- the CHH interrupt is expected next */

      efm32_chan_halt(priv, chidx, CHREASON_FRMOR);

      /* Clear the FRaMe OverRun (FRMOR) condition */

      efm32_putreg(EFM32_USB_HCn_INT(chidx), USB_HC_INT_FRMOVRUN);
    }

  /* Check for a pending TransFeR Completed (XFRC) interrupt */

  else if ((pending & USB_HC_INT_XFERCOMPL) != 0)
    {
      /* Clear the TransFeR Completed (XFRC) condition */

      efm32_putreg(EFM32_USB_HCn_INT(chidx), USB_HC_INT_XFERCOMPL);

      /* Then handle the transfer completion event based on the endpoint type */

      if (chan->eptype == EFM32_USB_EPTYPE_CTRL || chan->eptype == EFM32_USB_EPTYPE_BULK)
        {
          /* Halt the channel -- the CHH interrupt is expected next */

          efm32_chan_halt(priv, chidx, CHREASON_XFRC);

          /* Clear any pending NAK condition.  The 'indata1' data toggle
           * should have been appropriately updated by the RxFIFO
           * logic as each packet was received.
           */

          efm32_putreg(EFM32_USB_HCn_INT(chidx), USB_HC_INT_NAK);
        }
      else if (chan->eptype == EFM32_USB_EPTYPE_INTR)
        {
          /* Force the next transfer on an ODD frame */

          regval = efm32_getreg(EFM32_USB_HCn_CHAR(chidx));
          regval |= USB_HC_CHAR_ODDFRM;
          efm32_putreg(EFM32_USB_HCn_CHAR(chidx), regval);

          /* Set the request done state */

          chan->result = OK;
        }
    }

  /* Check for a pending CHannel Halted (CHH) interrupt */

  else if ((pending & USB_HC_INT_CHHLTD) != 0)
    {
      /* Mask the CHannel Halted (CHH) interrupt */

      regval  = efm32_getreg(EFM32_USB_HCn_INTMSK(chidx));
      regval &= ~USB_HC_INT_CHHLTD;
      efm32_putreg(EFM32_USB_HCn_INTMSK(chidx), regval);

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
          /* Halt on NAK only happens on an INTR channel.  Fetch the HCCHAR register
           * and check for an interrupt endpoint.
           */

          regval = efm32_getreg(EFM32_USB_HCn_CHAR(chidx));
          if ((regval & _USB_HC_CHAR_EPTYPE_MASK) == USB_HC_CHAR_EPTYPE_INT)
            {
              /* Toggle the IN data toggle (Used by Bulk and INTR only) */

              chan->indata1 ^= true;
            }

          /* Set the NAK error result */

          chan->result = EAGAIN;
        }
      else /* if (chan->chreason == CHREASON_FRMOR) */
        {
          /* Set the frame overrun error result */

          chan->result = EPIPE;
        }

      /* Clear the CHannel Halted (CHH) condition */

      efm32_putreg(EFM32_USB_HCn_INT(chidx), USB_HC_INT_CHHLTD);
    }

  /* Check for a pending Transaction ERror (TXERR) interrupt */

  else if ((pending & USB_HC_INT_XACTERR) != 0)
    {
      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      efm32_chan_halt(priv, chidx, CHREASON_TXERR);

      /* Clear the Transaction ERror (TXERR) condition */

      efm32_putreg(EFM32_USB_HCn_INT(chidx), USB_HC_INT_XACTERR);
    }

  /* Check for a pending NAK response received (NAK) interrupt */

  else if ((pending & USB_HC_INT_NAK) != 0)
    {
      /* For a BULK transfer, the hardware is capable of retrying
       * automatically on a NAK.  However, this is not always
       * what we need to do.  So we always halt the transfer and
       * return control to high level logic in the event of a NAK.
       */

#if 1
      /* Halt the interrupt channel */

      if (chan->eptype == EFM32_USB_EPTYPE_INTR)
        {
          /* Halt the channel -- the CHH interrupt is expected next */

          efm32_chan_halt(priv, chidx, CHREASON_NAK);
        }

      /* Re-activate CTRL and BULK channels.
       * REVISIT: This can cause a lot of interrupts!
       */

      else if (chan->eptype == EFM32_USB_EPTYPE_CTRL ||
               chan->eptype == EFM32_USB_EPTYPE_BULK)
        {
          /* Re-activate the channel by clearing CHDIS and assuring that
           * CHENA is set
           */

          regval  = efm32_getreg(EFM32_USB_HCn_CHAR(chidx));
          regval |= USB_HC_CHAR_CHENA;
          regval &= ~USB_HC_CHAR_CHDIS;
          efm32_putreg(EFM32_USB_HCn_CHAR(chidx), regval);
        }
#else
      /* Halt all transfers on the NAK -- the CHH interrupt is expected next */

      efm32_chan_halt(priv, chidx, CHREASON_NAK);
#endif

      /* Clear the NAK condition */

      efm32_putreg(EFM32_USB_HCn_INT(chidx), USB_HC_INT_NAK);
    }

  /* Check for a transfer complete event */

  efm32_chan_wakeup(priv, chan);
}

/****************************************************************************
 * Name: efm32_gint_hcoutisr
 *
 * Description:
 *   USB OTG FS host OUT channels interrupt handler
 *
 *   One the completion of the transfer, the channel result byte may be set as
 *   follows:
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

static inline void efm32_gint_hcoutisr(FAR struct efm32_usbhost_s *priv,
                                       int chidx)
{
  FAR struct efm32_chan_s *chan = &priv->chan[chidx];
  uint32_t regval;
  uint32_t pending;

  /* Read the HCINT register to get the pending HC interrupts.  Read the
   * HCINTMSK register to get the set of enabled HC interrupts.
   */

  pending = efm32_getreg(EFM32_USB_HCn_INT(chidx));
  regval  = efm32_getreg(EFM32_USB_HCn_INTMSK(chidx));

  /* AND the two to get the set of enabled, pending HC interrupts */

  pending &= regval;
  uinfo("HCINTMSK%d: %08x pending: %08x\n", chidx, regval, pending);

  /* Check for a pending ACK response received/transmitted (ACK) interrupt */

  if ((pending & USB_HC_INT_ACK) != 0)
    {
      /* Clear the pending the ACK response received/transmitted (ACK) interrupt */

      efm32_putreg(EFM32_USB_HCn_INT(chidx), USB_HC_INT_ACK);
    }

  /* Check for a pending FRaMe OverRun (FRMOR) interrupt */

  else if ((pending & USB_HC_INT_FRMOVRUN) != 0)
    {
      /* Halt the channel (probably not necessary for FRMOR) */

      efm32_chan_halt(priv, chidx, CHREASON_FRMOR);

      /* Clear the pending the FRaMe OverRun (FRMOR) interrupt */

      efm32_putreg(EFM32_USB_HCn_INT(chidx), USB_HC_INT_FRMOVRUN);
    }

  /* Check for a pending TransFeR Completed (XFRC) interrupt */

  else if ((pending & USB_HC_INT_XFERCOMPL) != 0)
    {
      /* Decrement the number of bytes remaining by the number of
       * bytes that were "in-flight".
       */

      priv->chan[chidx].buffer  += priv->chan[chidx].inflight;
      priv->chan[chidx].xfrd    += priv->chan[chidx].inflight;
      priv->chan[chidx].inflight = 0;

      /* Halt the channel -- the CHH interrupt is expected next */

      efm32_chan_halt(priv, chidx, CHREASON_XFRC);

      /* Clear the pending the TransFeR Completed (XFRC) interrupt */

      efm32_putreg(EFM32_USB_HCn_INT(chidx), USB_HC_INT_XFERCOMPL);
    }

  /* Check for a pending STALL response receive (STALL) interrupt */

  else if ((pending & USB_HC_INT_STALL) != 0)
    {
      /* Clear the pending the STALL response receiv (STALL) interrupt */

      efm32_putreg(EFM32_USB_HCn_INT(chidx), USB_HC_INT_STALL);

      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      efm32_chan_halt(priv, chidx, CHREASON_STALL);
    }

  /* Check for a pending NAK response received (NAK) interrupt */

  else if ((pending & USB_HC_INT_NAK) != 0)
    {
      /* Halt the channel  -- the CHH interrupt is expected next */

      efm32_chan_halt(priv, chidx, CHREASON_NAK);

      /* Clear the pending the NAK response received (NAK) interrupt */

      efm32_putreg(EFM32_USB_HCn_INT(chidx), USB_HC_INT_NAK);
    }

  /* Check for a pending Transaction ERror (TXERR) interrupt */

  else if ((pending & USB_HC_INT_XACTERR) != 0)
    {
      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      efm32_chan_halt(priv, chidx, CHREASON_TXERR);

      /* Clear the pending the Transaction ERror (TXERR) interrupt */

      efm32_putreg(EFM32_USB_HCn_INT(chidx), USB_HC_INT_XACTERR);
    }

  /* Check for a pending Data Toggle ERRor (DTERR) interrupt */

  else if (pending & USB_HC_INT_DATATGLERR)
    {
      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      efm32_chan_halt(priv, chidx, CHREASON_DTERR);

      /* Clear the pending the Data Toggle ERRor (DTERR) and NAK interrupts */

      efm32_putreg(EFM32_USB_HCn_INT(chidx), (USB_HC_INT_DATATGLERR | USB_HC_INT_NAK));
    }

  /* Check for a pending CHannel Halted (CHH) interrupt */

  else if ((pending & USB_HC_INT_CHHLTD) != 0)
    {
      /* Mask the CHannel Halted (CHH) interrupt */

      regval  = efm32_getreg(EFM32_USB_HCn_INTMSK(chidx));
      regval &= ~USB_HC_INT_CHHLTD;
      efm32_putreg(EFM32_USB_HCn_INTMSK(chidx), regval);

      if (chan->chreason == CHREASON_XFRC)
        {
          /* Set the request done result */

          chan->result = OK;

          /* Read the HCCHAR register to get the HCCHAR register to get
           * the endpoint type.
           */

          regval = efm32_getreg(EFM32_USB_HCn_CHAR(chidx));

          /* Is it a bulk endpoint?  Were an odd number of packets
           * transferred?
           */

          if ((regval & _USB_HC_CHAR_EPTYPE_MASK) == USB_HC_CHAR_EPTYPE_BULK &&
              (chan->npackets & 1) != 0)
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

      efm32_putreg(EFM32_USB_HCn_INT(chidx), USB_HC_INT_CHHLTD);
    }

  /* Check for a transfer complete event */

  efm32_chan_wakeup(priv, chan);
}

/****************************************************************************
 * Name: efm32_gint_connected
 *
 * Description:
 *   Handle a connection event.
 *
 ****************************************************************************/

static void efm32_gint_connected(FAR struct efm32_usbhost_s *priv)
{
  /* We we previously disconnected? */

  if (!priv->connected)
    {
      /* Yes.. then now we are connected */

      usbhost_vtrace1(USBHOST_VTRACE1_CONNECTED, 0);
      priv->connected = true;
      priv->change    = true;
      DEBUGASSERT(priv->smstate == SMSTATE_DETACHED);

      /* Notify any waiters */

      priv->smstate = SMSTATE_ATTACHED;
      if (priv->pscwait)
        {
          efm32_givesem(&priv->pscsem);
          priv->pscwait = false;
        }
    }
}

/****************************************************************************
 * Name: efm32_gint_disconnected
 *
 * Description:
 *   Handle a disconnection event.
 *
 ****************************************************************************/

static void efm32_gint_disconnected(FAR struct efm32_usbhost_s *priv)
{
  /* Were we previously connected? */

  if (priv->connected)
    {
      /* Yes.. then we no longer connected */

      usbhost_vtrace1(USBHOST_VTRACE1_DISCONNECTED, 0);

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
      efm32_chan_freeall(priv);

      priv->rhport.hport.speed = USB_SPEED_FULL;

      /* Notify any waiters that there is a change in the connection state */

      if (priv->pscwait)
        {
          efm32_givesem(&priv->pscsem);
          priv->pscwait = false;
        }
    }
}

/****************************************************************************
 * Name: efm32_gint_sofisr
 *
 * Description:
 *   USB OTG FS start-of-frame interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_OTGFS_SOFINTR
static inline void efm32_gint_sofisr(FAR struct efm32_usbhost_s *priv)
{
  /* Handle SOF interrupt */
#warning "Do what?"

  /* Clear pending SOF interrupt */

  efm32_putreg(EFM32_USB_GINTSTS, USB_GINTSTS_SOF);
}
#endif

/****************************************************************************
 * Name: efm32_gint_rxflvlisr
 *
 * Description:
 *   USB OTG FS RxFIFO non-empty interrupt handler
 *
 ****************************************************************************/

static inline void efm32_gint_rxflvlisr(FAR struct efm32_usbhost_s *priv)
{
  FAR uint32_t *dest;
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

  intmsk  = efm32_getreg(EFM32_USB_GINTMSK);
  intmsk &= ~USB_GINTMSK_RXFLVLMSK;
  efm32_putreg(EFM32_USB_GINTMSK, intmsk);

  /* Read and pop the next status from the Rx FIFO */

  grxsts = efm32_getreg(EFM32_USB_GRXSTSP);
  uinfo("GRXSTS: %08x\n", grxsts);

  /* Isolate the channel number/index in the status word */

  chidx = (grxsts & _USB_GRXSTSP_CHEPNUM_MASK) >> _USB_GRXSTSP_CHEPNUM_SHIFT;

  /* Get the host channel characteristics register (HCCHAR) for this channel */

  hcchar = efm32_getreg(EFM32_USB_HCn_CHAR(chidx));

  /* Then process the interrupt according to the packet status */

  switch (grxsts & _USB_GRXSTSP_PKTSTS_MASK)
    {
    case USB_GRXSTSP_PKTSTS_PKTRCV: /* IN data packet received */
      {
        /* Read the data into the host buffer. */

        bcnt = (grxsts & _USB_GRXSTSP_BCNT_MASK) >> _USB_GRXSTSP_BCNT_SHIFT;
        if (bcnt > 0 && priv->chan[chidx].buffer != NULL)
          {
            /* Transfer the packet from the Rx FIFO into the user buffer */

            dest   = (FAR uint32_t *)priv->chan[chidx].buffer;
            fifo   = EFM32_USB_FIFO_BASE(0);
            bcnt32 = (bcnt + 3) >> 2;

            for (i = 0; i < bcnt32; i++)
              {
                *dest++ = efm32_getreg(fifo);
              }

            efm32_pktdump("Received", priv->chan[chidx].buffer, bcnt);

            /* Toggle the IN data pid (Used by Bulk and INTR only) */

            priv->chan[chidx].indata1 ^= true;

            /* Manage multiple packet transfers */

            priv->chan[chidx].buffer += bcnt;
            priv->chan[chidx].xfrd   += bcnt;

            /* Check if more packets are expected */

            hctsiz = efm32_getreg(EFM32_USB_HCn_TSIZ(chidx));
            if ((hctsiz & _USB_HC_TSIZ_PKTCNT_MASK) != 0)
              {
                /* Re-activate the channel when more packets are expected */

                hcchar |= USB_HC_CHAR_CHENA;
                hcchar &= ~USB_HC_CHAR_CHDIS;
                efm32_putreg(EFM32_USB_HCn_CHAR(chidx), hcchar);
              }
          }
      }
      break;

    case USB_GRXSTSP_PKTSTS_XFERCOMPL:  /* IN transfer completed */
    case USB_GRXSTSP_PKTSTS_TGLERR:     /* Data toggle error */
    case USB_GRXSTSP_PKTSTS_CHLT:       /* Channel halted */
    default:
      break;
    }

  /* Re-enable the RxFIFO non-empty interrupt */

  intmsk |= USB_GINTMSK_RXFLVLMSK;
  efm32_putreg(EFM32_USB_GINTMSK, intmsk);
}

/****************************************************************************
 * Name: efm32_gint_nptxfeisr
 *
 * Description:
 *   USB OTG FS non-periodic TxFIFO empty interrupt handler
 *
 ****************************************************************************/

static inline void efm32_gint_nptxfeisr(FAR struct efm32_usbhost_s *priv)
{
  FAR struct efm32_chan_s *chan;
  uint32_t     regval;
  unsigned int wrsize;
  unsigned int avail;
  unsigned int chidx;

  /* Recover the index of the channel that is waiting for space in the Tx
   * FIFO.
   */

  chidx = priv->chidx;
  chan  = &priv->chan[chidx];

  /* Reduce the buffer size by the number of bytes that were previously placed
   * in the Tx FIFO.
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

      efm32_modifyreg(EFM32_USB_GINTMSK, USB_GINTMSK_NPTXFEMPMSK, 0);
      return;
    }

  /* Read the status from the top of the non-periodic TxFIFO */

  regval = efm32_getreg(EFM32_USB_GNPTXSTS);

  /* Extract the number of bytes available in the non-periodic Tx FIFO. */

  avail = ((regval & _USB_GNPTXSTS_NPTXFSPCAVAIL_MASK) >>
           _USB_GNPTXSTS_NPTXFSPCAVAIL_SHIFT) << 2;

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
      efm32_modifyreg(EFM32_USB_GINTMSK, USB_GINTMSK_NPTXFEMPMSK, 0);
    }

  /* Write the next group of packets into the Tx FIFO */

  uinfo("HNPTXSTS: %08x chidx: %d avail: %d buflen: %d xfrd: %d wrsize: %d\n",
         regval, chidx, avail, chan->buflen, chan->xfrd, wrsize);

  efm32_gint_wrpacket(priv, chan->buffer, chidx, wrsize);
}

/****************************************************************************
 * Name: efm32_gint_ptxfeisr
 *
 * Description:
 *   USB OTG FS periodic TxFIFO empty interrupt handler
 *
 ****************************************************************************/

static inline void efm32_gint_ptxfeisr(FAR struct efm32_usbhost_s *priv)
{
  FAR struct efm32_chan_s *chan;
  uint32_t     regval;
  unsigned int wrsize;
  unsigned int avail;
  unsigned int chidx;

  /* Recover the index of the channel that is waiting for space in the Tx
   * FIFO.
   */

  chidx = priv->chidx;
  chan  = &priv->chan[chidx];

  /* Reduce the buffer size by the number of bytes that were previously placed
   * in the Tx FIFO.
   */

  chan->buffer  += chan->inflight;
  chan->xfrd    += chan->inflight;
  chan->inflight = 0;

  /* If we have now transfered the entire buffer, then this transfer is
   * complete (this case really should never happen because we disable
   * the PTXFE interrupt on the final packet).
   */

  if (chan->xfrd >= chan->buflen)
    {
      /* Disable further Tx FIFO empty interrupts and bail. */

      efm32_modifyreg(EFM32_USB_GINTMSK, USB_GINTMSK_NPTXFEMPMSK, 0);
      return;
    }

  /* Read the status from the top of the periodic TxFIFO */

  regval = efm32_getreg(EFM32_USB_HPTXSTS);

  /* Extract the number of bytes available in the periodic Tx FIFO. */

  avail = ((regval & _USB_HPTXSTS_PTXFSPCAVAIL_MASK) >> _USB_HPTXSTS_PTXFSPCAVAIL_SHIFT) << 2;

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
      efm32_modifyreg(EFM32_USB_GINTMSK, USB_GINTMSK_NPTXFEMPMSK, 0);
    }

  /* Write the next group of packets into the Tx FIFO */

  uinfo("HPTXSTS: %08x chidx: %d avail: %d buflen: %d xfrd: %d wrsize: %d\n",
        regval, chidx, avail, chan->buflen, chan->xfrd, wrsize);

  efm32_gint_wrpacket(priv, chan->buffer, chidx, wrsize);
}

/****************************************************************************
 * Name: efm32_gint_hcisr
 *
 * Description:
 *   USB OTG FS host channels interrupt handler
 *
 ****************************************************************************/

static inline void efm32_gint_hcisr(FAR struct efm32_usbhost_s *priv)
{
  uint32_t haint;
  uint32_t hcchar;
  int i = 0;

  /* Read the Host all channels interrupt register and test each bit in the
   * register. Each bit i, i=0...(EFM32_NHOST_CHANNELS-1), corresponds to
   * a pending interrupt on channel i.
   */

  haint = efm32_getreg(EFM32_USB_HAINT);
  for (i = 0; i < EFM32_NHOST_CHANNELS; i++)
    {
      /* Is an interrupt pending on this channel? */

      if ((haint & USB_HAINT(i)) != 0)
        {
          /* Yes... read the HCCHAR register to get the direction bit */

          hcchar = efm32_getreg(EFM32_USB_HCn_CHAR(i));

          /* Was this an interrupt on an IN or an OUT channel? */

          if ((hcchar & _USB_HC_CHAR_EPDIR_MASK) != _USB_HC_CHAR_EPDIR_OUT)
            {
              /* Handle the HC IN channel interrupt */

              efm32_gint_hcinisr(priv, i);
            }
          else
            {
              /* Handle the HC OUT channel interrupt */

              efm32_gint_hcoutisr(priv, i);
            }
        }
    }
}

/****************************************************************************
 * Name: efm32_gint_hprtisr
 *
 * Description:
 *   USB OTG FS host port interrupt handler
 *
 ****************************************************************************/

static inline void efm32_gint_hprtisr(FAR struct efm32_usbhost_s *priv)
{
  uint32_t hprt;
  uint32_t newhprt;
  uint32_t hcfg;

  usbhost_vtrace1(USBHOST_VTRACE1_GINT_HPRT, 0);

  /* Read the port status and control register (HPRT) */

  hprt = efm32_getreg(EFM32_USB_HPRT);

  /* Setup to clear the interrupt bits in GINTSTS by setting the corresponding
   * bits in the HPRT.  The HCINT interrupt bit is cleared when the appropriate
   * status bits in the HPRT register are cleared.
   */

  newhprt = hprt & ~(USB_HPRT_PRTENA | USB_HPRT_PRTCONNDET | USB_HPRT_PRTENCHNG |
                     USB_HPRT_PRTOVRCURRCHNG);

  /* Check for Port Overcurrent CHaNGe (POCCHNG) */

  if ((hprt & USB_HPRT_PRTOVRCURRCHNG) != 0)
    {
      /* Set up to clear the POCCHNG status in the new HPRT contents. */

      usbhost_vtrace1(USBHOST_VTRACE1_GINT_HPRT_POCCHNG, 0);
      newhprt |= USB_HPRT_PRTOVRCURRCHNG;
    }

  /* Check for Port Connect DETected (PCDET).  The core sets this bit when a
   * device connection is detected.
   */

  if ((hprt & USB_HPRT_PRTCONNDET) != 0)
    {
      /* Set up to clear the PCDET status in the new HPRT contents. Then
       * process the new connection event.
       */

      usbhost_vtrace1(USBHOST_VTRACE1_GINT_HPRT_PCDET, 0);
      newhprt |= USB_HPRT_PRTCONNDET;
      efm32_portreset(priv);
      efm32_gint_connected(priv);
    }

  /* Check for Port Enable CHaNGed (PENCHNG) */

  if ((hprt & USB_HPRT_PRTENCHNG) != 0)
    {
      /* Set up to clear the PENCHNG status in the new HPRT contents. */

      usbhost_vtrace1(USBHOST_VTRACE1_GINT_HPRT_PENCHNG, 0);
      newhprt |= USB_HPRT_PRTENCHNG;

      /* Was the port enabled? */

      if ((hprt & USB_HPRT_PRTENA) != 0)
        {
          /* Yes.. handle the new connection event */

          efm32_gint_connected(priv);

          /* Check the Host ConFiGuration register (HCFG) */

          hcfg = efm32_getreg(EFM32_USB_HCFG);

          /* Is this a low speed or full speed connection (OTG FS does not
           * support high speed)
           */

          if ((hprt & _USB_HPRT_PRTSPD_MASK) == USB_HPRT_PRTSPD_LS)
            {
              /* Set the Host Frame Interval Register for the 6KHz speed */

              usbhost_vtrace1(USBHOST_VTRACE1_GINT_HPRT_LSDEV, 0);
              efm32_putreg(EFM32_USB_HFIR, 6000);

              /* Are we switching from FS to LS? */

              if ((hcfg & _USB_HCFG_FSLSPCLKSEL_MASK) != USB_HCFG_FSLSPCLKSEL_DIV8)
                {

                  usbhost_vtrace1(USBHOST_VTRACE1_GINT_HPRT_FSLSSW, 0);
                  /* Yes... configure for LS */

                  hcfg &= ~_USB_HCFG_FSLSPCLKSEL_MASK;
                  hcfg |= USB_HCFG_FSLSPCLKSEL_DIV8;
                  efm32_putreg(EFM32_USB_HCFG, hcfg);

                  /* And reset the port */

                  efm32_portreset(priv);
                }
            }
          else /* if ((hprt & _USB_HPRT_PRTSPD_MASK) == USB_HPRT_PSPD_FS) */
            {

              usbhost_vtrace1(USBHOST_VTRACE1_GINT_HPRT_FSDEV, 0);
              efm32_putreg(EFM32_USB_HFIR, 48000);

              /* Are we switching from LS to FS? */

              if ((hcfg & _USB_HCFG_FSLSPCLKSEL_MASK) != USB_HCFG_FSLSPCLKSEL_DIV1)
                {

                  usbhost_vtrace1(USBHOST_VTRACE1_GINT_HPRT_LSFSSW, 0);
                  /* Yes... configure for FS */

                  hcfg &= ~_USB_HCFG_FSLSPCLKSEL_MASK;
                  hcfg |= USB_HCFG_FSLSPCLKSEL_DIV1;
                  efm32_putreg(EFM32_USB_HCFG, hcfg);

                  /* And reset the port */

                  efm32_portreset(priv);
                }
            }
        }
    }

  /* Clear port interrupts by setting bits in the HPRT */

  efm32_putreg(EFM32_USB_HPRT, newhprt);
}

/****************************************************************************
 * Name: efm32_gint_discisr
 *
 * Description:
 *   USB OTG FS disconnect detected interrupt handler
 *
 ****************************************************************************/

static inline void efm32_gint_discisr(FAR struct efm32_usbhost_s *priv)
{
  /* Handle the disconnection event */

  efm32_gint_disconnected(priv);

  /* Clear the dicsonnect interrupt */

  efm32_putreg(EFM32_USB_GINTSTS, USB_GINTSTS_DISCONNINT);
}

/****************************************************************************
 * Name: efm32_gint_ipxfrisr
 *
 * Description:
 *   USB OTG FS incomplete periodic interrupt handler
 *
 ****************************************************************************/

static inline void efm32_gint_ipxfrisr(FAR struct efm32_usbhost_s *priv)
{
  uint32_t regval;

  /* CHENA : Set to enable the channel
   * CHDIS : Set to stop transmitting/receiving data on a channel
   */

  regval = efm32_getreg(EFM32_USB_HCn_CHAR(0));
  regval |= (USB_HC_CHAR_CHDIS | USB_HC_CHAR_CHENA);
  efm32_putreg(EFM32_USB_HCn_CHAR(0), regval);

  /* Clear the incomplete isochronous OUT interrupt */

  efm32_putreg(EFM32_USB_GINTSTS, USB_GINTSTS_INCOMPLP);
}

/****************************************************************************
 * Name: efm32_gint_isr
 *
 * Description:
 *   USB OTG FS global interrupt handler
 *
 ****************************************************************************/

static int efm32_gint_isr(int irq, FAR void *context, FAR void *arg)
{
  /* At present, there is only support for a single OTG FS host. Hence it is
   * pre-allocated as g_usbhost.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple devices.
   */

  FAR struct efm32_usbhost_s *priv = &g_usbhost;
  uint32_t pending;

  /* If OTG were supported, we would need to check if we are in host or
   * device mode when the global interrupt occurs.  Here we support only
   * host mode
   */

  /* Loop while there are pending interrupts to process.  This loop may save a
   * little interrupt handling overhead.
   */

  for (; ; )
    {
      /* Get the unmasked bits in the GINT status */

      pending  = efm32_getreg(EFM32_USB_GINTSTS);
      pending &= efm32_getreg(EFM32_USB_GINTMSK);

      /* Return from the interrupt when there are no further pending
       * interrupts.
       */

      if (pending == 0)
        {
          return OK;
        }

      /* Otherwise, process each pending, unmasked GINT interrupts */

      /* Handle the start of frame interrupt */

#ifdef CONFIG_EFM32_OTGFS_SOFINTR
      if ((pending & USB_GINTSTS_SOF) != 0)
        {
          usbhost_vtrace1(USBHOST_VTRACE1_GINT_SOF, 0);
          efm32_gint_sofisr(priv);
        }
#endif

      /* Handle the RxFIFO non-empty interrupt */

      if ((pending & USB_GINTSTS_RXFLVL) != 0)
        {
          usbhost_vtrace1(USBHOST_VTRACE1_GINT_RXFLVL, 0);
          efm32_gint_rxflvlisr(priv);
        }

      /* Handle the non-periodic TxFIFO empty interrupt */

      if ((pending & USB_GINTSTS_NPTXFEMP) != 0)
        {
          usbhost_vtrace1(USBHOST_VTRACE1_GINT_NPTXFE, 0);
          efm32_gint_nptxfeisr(priv);
        }

      /* Handle the periodic TxFIFO empty interrupt */

      if ((pending & USB_GINTSTS_PTXFEMP) != 0)
        {
          usbhost_vtrace1(USBHOST_VTRACE1_GINT_PTXFE, 0);
          efm32_gint_ptxfeisr(priv);
        }

      /* Handle the host channels interrupt */

      if ((pending & USB_GINTSTS_HCHINT) != 0)
        {
          usbhost_vtrace1(USBHOST_VTRACE1_GINT_HC, 0);
          efm32_gint_hcisr(priv);
        }

      /* Handle the host port interrupt */

      if ((pending & USB_GINTSTS_PRTINT) != 0)
        {
          efm32_gint_hprtisr(priv);
        }

      /* Handle the disconnect detected interrupt */

      if ((pending & USB_GINTSTS_DISCONNINT) != 0)
        {
          usbhost_vtrace1(USBHOST_VTRACE1_GINT_DISC, 0);
          efm32_gint_discisr(priv);
        }

      /* Handle the incomplete periodic transfer */

      if ((pending & USB_GINTSTS_INCOMPLP) != 0)
        {
          usbhost_vtrace1(USBHOST_VTRACE1_GINT_IPXFR, 0);
          efm32_gint_ipxfrisr(priv);
        }
    }

  /* We won't get here */

  return OK;
}

/****************************************************************************
 * Name: efm32_gint_enable and efm32_gint_disable
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

static void efm32_gint_enable(void)
{
  uint32_t regval;

  /* Set the GINTMSK bit to unmask the interrupt */

  regval  = efm32_getreg(EFM32_USB_GAHBCFG);
  regval |= USB_GAHBCFG_GLBLINTRMSK;
  efm32_putreg(EFM32_USB_GAHBCFG, regval);
}

static void efm32_gint_disable(void)
{
  uint32_t regval;

  /* Clear the GINTMSK bit to mask the interrupt */

  regval  = efm32_getreg(EFM32_USB_GAHBCFG);
  regval &= ~USB_GAHBCFG_GLBLINTRMSK;
  efm32_putreg(EFM32_USB_GAHBCFG, regval);
}

/****************************************************************************
 * Name: efm32_hostinit_enable
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

static inline void efm32_hostinit_enable(void)
{
  uint32_t regval;

  /* Disable all interrupts. */

  efm32_putreg(EFM32_USB_GINTMSK, 0);

  /* Clear any pending interrupts. */

  efm32_putreg(EFM32_USB_GINTSTS, 0xffffffff);

  /* Clear any pending USB OTG Interrupts (should be done elsewhere if OTG is supported) */

  efm32_putreg(EFM32_USB_GOTGINT, 0xffffffff);

  /* Clear any pending USB OTG interrupts */

  efm32_putreg(EFM32_USB_GINTSTS, 0xbfffffff);

  /* Enable the host interrupts */
  /* Common interrupts:
   *
   *   USB_GINTMSK_WKUPINTMSK      : Resume/remote wakeup detected interrupt
   *   USB_GINTMSK_USBSUSPMSK      : USB suspend
   */

  regval = (USB_GINTMSK_WKUPINTMSK | USB_GINTMSK_USBSUSPMSK);

  /* If OTG were supported, we would need to enable the following as well:
   *
   *   USB_GINTMSK_OTGINTMSK       : OTG interrupt
   *   USB_GINTMSK_SESSREQINTMSK   : Session request/new session detected interrupt
   *   USB_GINTMSK_CONIDSTSCHNGMSK : Connector ID status change
   */

  /* Host-specific interrupts
   *
   *   USB_GINTMSK_SOFMSK          : Start of frame
   *   USB_GINTMSK_RXFLVLMSK       : RxFIFO non-empty
   *   USB_GINTMSK_INCOMPLPMSK     : Incomplete isochronous OUT transfer
   *   USB_GINTMSK_PRTINTMSK       : Host port interrupt
   *   USB_GINTMSK_HCHINTMSK       : Host channels interrupt
   *   USB_GINTMSK_DISCONNINTMSK   : Disconnect detected interrupt
   */

#ifdef CONFIG_EFM32_OTGFS_SOFINTR
  regval |= (USB_GINTMSK_SOFMSK      | USB_GINTMSK_RXFLVLMSK   |
             USB_GINTMSK_INCOMPLPMSK | USB_GINTMSK_PRTINTMSK   |
             USB_GINTMSK_HCHINTMSK   | USB_GINTMSK_DISCONNINTMSK);
#else
  regval |= (USB_GINTMSK_RXFLVLMSK   | USB_GINTMSK_INCOMPLPMSK |
             USB_GINTMSK_PRTINTMSK   | USB_GINTMSK_HCHINTMSK   |
             USB_GINTMSK_DISCONNINTMSK);
#endif
  efm32_putreg(EFM32_USB_GINTMSK, regval);
}

/****************************************************************************
 * Name: efm32_txfe_enable
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

static void efm32_txfe_enable(FAR struct efm32_usbhost_s *priv, int chidx)
{
  FAR struct efm32_chan_s *chan = &priv->chan[chidx];
  irqstate_t flags;
  uint32_t regval;

  /* Disable all interrupts so that we have exclusive access to the GINTMSK
   * (it would be sufficent just to disable the GINT interrupt).
   */

  flags = enter_critical_section();

  /* Should we enable the periodic or non-peridic Tx FIFO empty interrupts */

  regval = efm32_getreg(EFM32_USB_GINTMSK);
  switch (chan->eptype)
    {
    default:
    case EFM32_USB_EPTYPE_CTRL: /* Non periodic transfer */
    case EFM32_USB_EPTYPE_BULK:
      regval |= USB_GINTMSK_NPTXFEMPMSK;
      break;

    case EFM32_USB_EPTYPE_INTR: /* Periodic transfer */
    case EFM32_USB_EPTYPE_ISOC:
      regval |= USB_GINTMSK_PTXFEMPMSK;
      break;
    }

  /* Enable interrupts */

  efm32_putreg(EFM32_USB_GINTMSK, regval);
  leave_critical_section(flags);
}

/****************************************************************************
 * USB Host Controller Operations
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_wait
 *
 * Description:
 *   Wait for a device to be connected or disconnected to/from a hub port.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from the call to
 *      the USB driver initialization logic.
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

static int efm32_wait(FAR struct usbhost_connection_s *conn,
                      FAR struct usbhost_hubport_s **hport)
{
  FAR struct efm32_usbhost_s *priv = &g_usbhost;
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

          uinfo("RHport Connected: %s\n", connport->connected ? "YES" : "NO");
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

          uinfo("Hub port Connected: %s\n", connport->connected ? "YES" : "NO");
          return OK;
        }
#endif

      /* Wait for the next connection event */

      priv->pscwait = true;
      efm32_takesem(&priv->pscsem);
    }
}

/****************************************************************************
 * Name: efm32_enumerate
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
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int efm32_rh_enumerate(FAR struct efm32_usbhost_s *priv,
                              FAR struct usbhost_connection_s *conn,
                              FAR struct usbhost_hubport_s *hport)
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

      usbhost_trace1(USBHOST_TRACE1_DEVDISCONN, 0);
      return -ENODEV;
    }

  DEBUGASSERT(priv->smstate == SMSTATE_ATTACHED);

  /* USB 2.0 spec says at least 50ms delay before port reset.  We wait 100ms. */

  nxsig_usleep(100*1000);

  /* Reset the host port */

  efm32_portreset(priv);

  /* Get the current device speed */

  regval = efm32_getreg(EFM32_USB_HPRT);
  if ((regval & _USB_HPRT_PRTSPD_MASK) == USB_HPRT_PRTSPD_LS)
    {
      priv->rhport.hport.speed = USB_SPEED_LOW;
    }
  else
    {
      priv->rhport.hport.speed = USB_SPEED_FULL;
    }

  /* Allocate and initialize the root hub port EP0 channels */

  ret = efm32_ctrlchan_alloc(priv, 0, 0, priv->rhport.hport.speed, &priv->ep0);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate a control endpoint: %d\n", ret);
    }

  return ret;
}

static int efm32_enumerate(FAR struct usbhost_connection_s *conn,
                           FAR struct usbhost_hubport_s *hport)
{
  FAR struct efm32_usbhost_s *priv = &g_usbhost;
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
      ret = efm32_rh_enumerate(priv, conn, hport);
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
      efm32_gint_disconnected(priv);
    }

  return ret;
}

/************************************************************************************
 * Name: efm32_ep0configure
 *
 * Description:
 *   Configure endpoint 0.  This method is normally used internally by the
 *   enumerate() method but is made available at the interface to support an
 *   external implementation of the enumeration logic.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep0 - The (opaque) EP0 endpoint instance
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *   speed - The speed of the port USB_SPEED_LOW, _FULL, or _HIGH
 *   maxpacketsize - The maximum number of bytes that can be sent to or
 *    received from the endpoint in a single data packet
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int efm32_ep0configure(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                              uint8_t funcaddr, uint8_t speed,
                              uint16_t maxpacketsize)
{
  FAR struct efm32_usbhost_s *priv = (FAR struct efm32_usbhost_s *)drvr;
  FAR struct efm32_ctrlinfo_s *ep0info = (FAR struct efm32_ctrlinfo_s *)ep0;
  FAR struct efm32_chan_s *chan;

  DEBUGASSERT(drvr != NULL && ep0info != NULL && funcaddr < 128 &&
              maxpacketsize <= 64);

  /* We must have exclusive access to the USB host hardware and state structures */

  efm32_takesem(&priv->exclsem);

  /* Configure the EP0 OUT channel */

  chan            = &priv->chan[ep0info->outndx];
  chan->funcaddr  = funcaddr;
  chan->speed     = speed;
  chan->maxpacket = maxpacketsize;

  efm32_chan_configure(priv, ep0info->outndx);

  /* Configure the EP0 IN channel */

  chan            = &priv->chan[ep0info->inndx];
  chan->funcaddr  = funcaddr;
  chan->speed     = speed;
  chan->maxpacket = maxpacketsize;

  efm32_chan_configure(priv, ep0info->inndx);

  efm32_givesem(&priv->exclsem);
  return OK;
}

/************************************************************************************
 * Name: efm32_epalloc
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int efm32_epalloc(FAR struct usbhost_driver_s *drvr,
                         FAR const struct usbhost_epdesc_s *epdesc,
                         FAR usbhost_ep_t *ep)
{

  FAR struct efm32_usbhost_s *priv = (FAR struct efm32_usbhost_s *)drvr;
  int ret;

  /* Sanity check.  NOTE that this method should only be called if a device is
   * connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(drvr != 0 && epdesc != NULL && ep != NULL);

  /* We must have exclusive access to the USB host hardware and state structures */

  efm32_takesem(&priv->exclsem);

  /* Handler control pipes differently from other endpoint types.  This is
   * because the normal, "transfer" endpoints are unidirectional an require
   * only a single channel.  Control endpoints, however, are bi-diretional
   * and require two channels, one for the IN and one for the OUT direction.
   */

  if (epdesc->xfrtype == EFM32_USB_EPTYPE_CTRL)
    {
      ret = efm32_ctrlep_alloc(priv, epdesc, ep);
    }
  else
    {
      ret = efm32_xfrep_alloc(priv, epdesc, ep);
    }

  efm32_givesem(&priv->exclsem);
  return ret;
}

/************************************************************************************
 * Name: efm32_epfree
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The endpoint to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int efm32_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  FAR struct efm32_usbhost_s *priv = (FAR struct efm32_usbhost_s *)drvr;

  DEBUGASSERT(priv);

  /* We must have exclusive access to the USB host hardware and state structures */

  efm32_takesem(&priv->exclsem);

  /* A single channel is represent by an index in the range of 0 to EFM32_MAX_TX_FIFOS.
   * Otherwise, the ep must be a pointer to an allocated control endpoint structure.
   */

  if ((uintptr_t)ep < EFM32_MAX_TX_FIFOS)
    {
      /* Halt the channel and mark the channel available */

      efm32_chan_free(priv, (int)ep);
    }
  else
    {
      /* Halt both control channel and mark the channels available */

      FAR struct efm32_ctrlinfo_s *ctrlep = (FAR struct efm32_ctrlinfo_s *)ep;
      efm32_chan_free(priv, ctrlep->inndx);
      efm32_chan_free(priv, ctrlep->outndx);

      /* And free the control endpoint container */

      kmm_free(ctrlep);
    }

  efm32_givesem(&priv->exclsem);
  return OK;
}

/****************************************************************************
 * Name: efm32_alloc
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to kmm_malloc.
 *
 *   This interface was optimized under a particular assumption.  It was assumed
 *   that the driver maintains a pool of small, pre-allocated buffers for descriptor
 *   traffic.  NOTE that size is not an input, but an output:  The size of the
 *   pre-allocated buffer is returned.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of a memory location provided by the caller in which to
 *     return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in which to
 *     return the maximum size of the allocated buffer memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int efm32_alloc(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t **buffer, FAR size_t *maxlen)
{
  FAR uint8_t *alloc;

  DEBUGASSERT(drvr && buffer && maxlen);

  /* There is no special memory requirement for the EFM32. */

  alloc = (FAR uint8_t *)kmm_malloc(CONFIG_EFM32_OTGFS_DESCSIZE);
  if (!alloc)
    {
      return -ENOMEM;
    }

  /* Return the allocated address and size of the descriptor buffer */

  *buffer = alloc;
  *maxlen = CONFIG_EFM32_OTGFS_DESCSIZE;
  return OK;
}

/****************************************************************************
 * Name: efm32_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data can
 *   be accessed more efficiently.  This method provides a mechanism to free that
 *   request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int efm32_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  /* There is no special memory requirement */

  DEBUGASSERT(drvr && buffer);
  kmm_free(buffer);
  return OK;
}

/************************************************************************************
 * Name: efm32_ioalloc
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to kmm_malloc.
 *
 *   This interface differs from DRVR_ALLOC in that the buffers are variable-sized.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of a memory location provided by the caller in which to
 *     return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int efm32_ioalloc(FAR struct usbhost_driver_s *drvr,
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

/************************************************************************************
 * Name: efm32_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be accessed more
 *   efficiently.  This method provides a mechanism to free that IO buffer
 *   memory.  If the underlying hardware does not support such "special" memory,
 *   this functions may simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int efm32_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  /* There is no special memory requirement */

  DEBUGASSERT(drvr && buffer);
  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: efm32_ctrlin and efm32_ctrlout
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  These methods
 *   will enqueue the request and wait for it to complete.  Only one transfer may be
 *   queued; Neither these methods nor the transfer() method can be called again
 *   until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep0 - The control endpoint to send/receive the control request.
 *   req - Describes the request to be sent.  This request must lie in memory
 *      created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *     responses.  This buffer must be large enough to hold the length value
 *     in the request description. buffer must have been allocated using DRVR_ALLOC.
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same allocated
 *   memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int efm32_ctrlin(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                        FAR const struct usb_ctrlreq_s *req,
                        FAR uint8_t *buffer)
{
  FAR struct efm32_usbhost_s *priv = (FAR struct efm32_usbhost_s *)drvr;
  FAR struct efm32_ctrlinfo_s *ep0info = (FAR struct efm32_ctrlinfo_s *)ep0;
  uint16_t buflen;
  clock_t start;
  clock_t elapsed;
  int retries;
  int ret;

  DEBUGASSERT(priv != NULL && ep0info != NULL && req != NULL);
  usbhost_vtrace2(USBHOST_VTRACE2_CTRLIN, req->type, req->req);
  uinfo("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* Extract values from the request */

  buflen = efm32_getle16(req->len);

  /* We must have exclusive access to the USB host hardware and state structures */

  efm32_takesem(&priv->exclsem);

  /* Loop, retrying until the retry time expires */

  for (retries = 0; retries < EFM32_RETRY_COUNT; retries++)
    {
      /* Send the SETUP request */

      ret = efm32_ctrl_sendsetup(priv, ep0info, req);
      if (ret < 0)
        {
          usbhost_trace1(USBHOST_TRACE1_SENDSETUP, -ret);
          continue;
        }

      /* Get the start time.  Loop again until the timeout expires */

      start = clock_systimer();
      do
        {
          /* Handle the IN data phase (if any) */

          if (buflen > 0)
            {
              ret = efm32_ctrl_recvdata(priv, ep0info, buffer, buflen);
              if (ret < 0)
                {
                  usbhost_trace1(USBHOST_TRACE1_RECVDATA, -ret);
                }
            }

          /* Handle the status OUT phase */

          if (ret == OK)
            {
              priv->chan[ep0info->outndx].outdata1 ^= true;
              ret = efm32_ctrl_senddata(priv, ep0info, NULL, 0);
              if (ret == OK)
                {
                  /* All success transactions exit here */

                  efm32_givesem(&priv->exclsem);
                  return OK;
                }

              usbhost_trace1(USBHOST_TRACE1_SENDDATA, ret < 0 ? -ret : ret);
            }

          /* Get the elapsed time (in frames) */

          elapsed = clock_systimer() - start;
        }
      while (elapsed < EFM32_DATANAK_DELAY);
    }

  /* All failures exit here after all retries and timeouts have been exhausted */

  efm32_givesem(&priv->exclsem);
  return -ETIMEDOUT;
}

static int efm32_ctrlout(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                         FAR const struct usb_ctrlreq_s *req,
                         FAR const uint8_t *buffer)
{
  FAR struct efm32_usbhost_s *priv = (FAR struct efm32_usbhost_s *)drvr;
  FAR struct efm32_ctrlinfo_s *ep0info = (FAR struct efm32_ctrlinfo_s *)ep0;
  uint16_t buflen;
  clock_t start;
  clock_t elapsed;
  int retries;
  int ret;

  DEBUGASSERT(priv != NULL && ep0info != NULL && req != NULL);
  usbhost_vtrace2(USBHOST_VTRACE2_CTRLOUT, req->type, req->req);
  uinfo("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* Extract values from the request */

  buflen = efm32_getle16(req->len);

  /* We must have exclusive access to the USB host hardware and state structures */

  efm32_takesem(&priv->exclsem);

  /* Loop, retrying until the retry time expires */

  for (retries = 0; retries < EFM32_RETRY_COUNT; retries++)
    {
      /* Send the SETUP request */

      ret = efm32_ctrl_sendsetup(priv, ep0info, req);
      if (ret < 0)
        {
          usbhost_trace1(USBHOST_TRACE1_SENDSETUP, -ret);
          continue;
        }

      /* Get the start time.  Loop again until the timeout expires */

      start = clock_systimer();
      do
        {
          /* Handle the data OUT phase (if any) */

          if (buflen > 0)
            {
              /* Start DATA out transfer (only one DATA packet) */

              priv->chan[ep0info->outndx].outdata1 = true;
              ret = efm32_ctrl_senddata(priv, ep0info, NULL, 0);
              if (ret < 0)
                {
                  usbhost_trace1(USBHOST_TRACE1_SENDDATA, -ret);
                }
            }

          /* Handle the status IN phase */

          if (ret == OK)
            {
              ret = efm32_ctrl_recvdata(priv, ep0info, NULL, 0);
              if (ret == OK)
                {
                  /* All success transactins exit here */

                  efm32_givesem(&priv->exclsem);
                  return OK;
                }

              usbhost_trace1(USBHOST_TRACE1_RECVDATA, ret < 0 ? -ret : ret);
            }

          /* Get the elapsed time (in frames) */

          elapsed = clock_systimer() - start;
        }
      while (elapsed < EFM32_DATANAK_DELAY);
    }

  /* All failures exit here after all retries and timeouts have been exhausted */

  efm32_givesem(&priv->exclsem);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: efm32_transfer
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request, blocking until the transfer completes. Only
 *   one transfer may be  queued; Neither this method nor the ctrlin or
 *   ctrlout methods can be called again until this function returns.
 *
 *   This is a blocking method; this functions will not return until the
 *   transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or received
 *     (IN endpoint).  buffer must have been allocated using DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Value:
 *   On success, a non-negative value is returned that indicates the number
 *   of bytes successfully transferred.  On a failure, a negated errno value is
 *   returned that indicates the nature of the failure:
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

static ssize_t efm32_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                              FAR uint8_t *buffer, size_t buflen)
{
  FAR struct efm32_usbhost_s *priv  = (FAR struct efm32_usbhost_s *)drvr;
  unsigned int chidx = (unsigned int)ep;
  ssize_t nbytes;

  uinfo("chidx: %d buflen: %d\n",  (unsigned int)ep, buflen);

  DEBUGASSERT(priv && buffer && chidx < EFM32_MAX_TX_FIFOS && buflen > 0);

  /* We must have exclusive access to the USB host hardware and state structures */

  efm32_takesem(&priv->exclsem);

  /* Handle IN and OUT transfer slightly differently */

  if (priv->chan[chidx].in)
    {
      nbytes = efm32_in_transfer(priv, chidx, buffer, buflen);
    }
  else
    {
      nbytes = efm32_out_transfer(priv, chidx, buffer, buflen);
    }

  efm32_givesem(&priv->exclsem);
  return nbytes;
}

/****************************************************************************
 * Name: efm32_asynch
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
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or received
 *     (IN endpoint).  buffer must have been allocated using DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *   callback - This function will be called when the transfer completes.
 *   arg - The arbitrary parameter that will be passed to the callback function
 *     when the transfer completes.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int efm32_asynch(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                        FAR uint8_t *buffer, size_t buflen,
                        usbhost_asynch_t callback, FAR void *arg)
{
  FAR struct efm32_usbhost_s *priv  = (FAR struct efm32_usbhost_s *)drvr;
  unsigned int chidx = (unsigned int)ep;
  int ret;

  uinfo("chidx: %d buflen: %d\n",  (unsigned int)ep, buflen);

  DEBUGASSERT(priv && buffer && chidx < EFM32_MAX_TX_FIFOS && buflen > 0);

  /* We must have exclusive access to the USB host hardware and state structures */

  efm32_takesem(&priv->exclsem);

  /* Handle IN and OUT transfer slightly differently */

  if (priv->chan[chidx].in)
    {
      ret = efm32_in_asynch(priv, chidx, buffer, buflen, callback, arg);
    }
  else
    {
      ret = efm32_out_asynch(priv, chidx, buffer, buflen, callback, arg);
    }

  efm32_givesem(&priv->exclsem);
  return ret;
}
#endif /* CONFIG_USBHOST_ASYNCH */

/************************************************************************************
 * Name: efm32_cancel
 *
 * Description:
 *   Cancel a pending transfer on an endpoint.  Cancelled synchronous or
 *   asynchronous transfer will complete normally with the error -ESHUTDOWN.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which an
 *      asynchronous transfer should be transferred.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ************************************************************************************/

static int efm32_cancel(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  FAR struct efm32_usbhost_s *priv  = (FAR struct efm32_usbhost_s *)drvr;
  FAR struct efm32_chan_s *chan;
  unsigned int chidx = (unsigned int)ep;
  irqstate_t flags;

  uinfo("chidx: %u: %d\n",  chidx);

  DEBUGASSERT(priv && chidx < EFM32_MAX_TX_FIFOS);
  chan = &priv->chan[chidx];

  /* We need to disable interrupts to avoid race conditions with the asynchronous
   * completion of the transfer being cancelled.
   */

  flags = enter_critical_section();

  /* Halt the channel */

  efm32_chan_halt(priv, chidx, CHREASON_CANCELLED);
  chan->result = -ESHUTDOWN;

  /* Is there a thread waiting for this transfer to complete? */

  if (chan->waiter)
    {
#ifdef CONFIG_USBHOST_ASYNCH
      /* Yes.. there should not also be a callback scheduled */

      DEBUGASSERT(chan->callback == NULL);
#endif

      /* Wake'em up! */

      efm32_givesem(&chan->waitsem);
      chan->waiter = false;
    }

#ifdef CONFIG_USBHOST_ASYNCH
  /* No.. is an asynchronous callback expected when the transfer
   * completes?
   */

  else if (chan->callback)
    {
      usbhost_asynch_t callback;
      FAR void *arg;

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

/************************************************************************************
 * Name: efm32_connect
 *
 * Description:
 *   New connections may be detected by an attached hub.  This method is the
 *   mechanism that is used by the hub class to introduce a new connection
 *   and port description to the system.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   hport - The descriptor of the hub port that detected the connection
 *      related event
 *   connected - True: device connected; false: device disconnected
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ************************************************************************************/

#ifdef CONFIG_USBHOST_HUB
static int efm32_connect(FAR struct usbhost_driver_s *drvr,
                         FAR struct usbhost_hubport_s *hport,
                         bool connected)
{
  FAR struct efm32_usbhost_s *priv = (FAR struct efm32_usbhost_s *)drvr;
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
      efm32_givesem(&priv->pscsem);
    }

  leave_critical_section(flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: efm32_disconnect
 *
 * Description:
 *   Called by the class when an error occurs and driver has been disconnected.
 *   The USB host driver should discard the handle to the class instance (it is
 *   stale) and not attempt any further interaction with the class driver instance
 *   (until a new instance is received from the create() method).  The driver
 *   should not called the class' disconnected() method.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static void efm32_disconnect(FAR struct usbhost_driver_s *drvr,
                             FAR struct usbhost_hubport_s *hport)
{
  DEBUGASSERT(hport != NULL);
  hport->devclass = NULL;
}

/****************************************************************************
 * Initialization
 ****************************************************************************/
/****************************************************************************
 * Name: efm32_portreset
 *
 * Description:
 *   Reset the USB host port.
 *
 *   NOTE: "Before starting to drive a USB reset, the application waits for the
 *   OTG interrupt triggered by the debounce done bit (DBCDNE bit in
 *   OTG_FS_GOTGINT), which indicates that the bus is stable again after the
 *   electrical debounce caused by the attachment of a pull-up resistor on DP
 *   (FS) or DM (LS).
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void efm32_portreset(FAR struct efm32_usbhost_s *priv)
{
  uint32_t regval;

  regval  = efm32_getreg(EFM32_USB_HPRT);
  regval &= ~(USB_HPRT_PRTENA | USB_HPRT_PRTCONNDET | USB_HPRT_PRTENCHNG |
              USB_HPRT_PRTOVRCURRCHNG);
  regval |= USB_HPRT_PRTRST;
  efm32_putreg(EFM32_USB_HPRT, regval);

  up_mdelay(20);

  regval &= ~USB_HPRT_PRTRST;
  efm32_putreg(EFM32_USB_HPRT, regval);

  up_mdelay(20);
}

/****************************************************************************
 * Name: efm32_flush_txfifos
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

static void efm32_flush_txfifos(uint32_t txfnum)
{
  uint32_t regval;
  uint32_t timeout;

  /* Initiate the TX FIFO flush operation */

  regval = USB_GRSTCTL_TXFFLSH | txfnum;
  efm32_putreg(regval, EFM32_USB_GRSTCTL);

  /* Wait for the FLUSH to complete */

  for (timeout = 0; timeout < EFM32_FLUSH_DELAY; timeout++)
    {
      regval = efm32_getreg(EFM32_USB_GRSTCTL);
      if ((regval & USB_GRSTCTL_TXFFLSH) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);
}

/****************************************************************************
 * Name: efm32_flush_rxfifo
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

static void efm32_flush_rxfifo(void)
{
  uint32_t regval;
  uint32_t timeout;

  /* Initiate the RX FIFO flush operation */

  efm32_putreg(USB_GRSTCTL_RXFFLSH, EFM32_USB_GRSTCTL);

  /* Wait for the FLUSH to complete */

  for (timeout = 0; timeout < EFM32_FLUSH_DELAY; timeout++)
    {
      regval = efm32_getreg(EFM32_USB_GRSTCTL);
      if ((regval & USB_GRSTCTL_RXFFLSH) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);
}

/****************************************************************************
 * Name: efm32_vbusdrive
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

static void efm32_vbusdrive(FAR struct efm32_usbhost_s *priv, bool state)
{
  uint32_t regval;

  /* Enable/disable the external charge pump */

  efm32_usbhost_vbusdrive(0, state);

  /* Turn on the Host port power. */

  regval = efm32_getreg(EFM32_USB_HPRT);
  regval &= ~(USB_HPRT_PRTENA | USB_HPRT_PRTCONNDET | USB_HPRT_PRTENCHNG |
              USB_HPRT_PRTOVRCURRCHNG);

  if (((regval & USB_HPRT_PRTPWR) == 0) && state)
    {
      regval |= USB_HPRT_PRTPWR;
      efm32_putreg(EFM32_USB_HPRT, regval);
    }

  if (((regval & USB_HPRT_PRTPWR) != 0) && !state)
    {
      regval &= ~USB_HPRT_PRTPWR;
      efm32_putreg(EFM32_USB_HPRT, regval);
    }

  up_mdelay(200);
}

/****************************************************************************
 * Name: efm32_host_initialize
 *
 * Description:
 *   Initialize/re-initialize hardware for host mode operation.  At present,
 *   this function is called only from efm32_hw_initialize().  But if OTG mode
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

static void efm32_host_initialize(FAR struct efm32_usbhost_s *priv)
{
  uint32_t regval;
  uint32_t offset;
  int i;

  /* Restart the PHY Clock */

  efm32_putreg(EFM32_USB_PCGCCTL, 0);

  /* Initialize Host Configuration (HCFG) register */

  regval  = efm32_getreg(EFM32_USB_HCFG);
  regval &= ~_USB_HCFG_FSLSPCLKSEL_MASK;
  regval |= USB_HCFG_FSLSPCLKSEL_DIV1;
  efm32_putreg(EFM32_USB_HCFG, regval);

  /* Reset the host port */

  efm32_portreset(priv);

  /* Clear the FS-/LS-only support bit in the HCFG register */

  regval = efm32_getreg(EFM32_USB_HCFG);
  regval &= ~USB_HCFG_FSLSSUPP;
  efm32_putreg(EFM32_USB_HCFG, regval);

  /* Carve up FIFO memory for the Rx FIFO and the periodic and non-periodic Tx FIFOs */
  /* Configure Rx FIFO size (GRXFSIZ) */

  efm32_putreg(EFM32_USB_GRXFSIZ, CONFIG_EFM32_OTGFS_RXFIFO_SIZE);
  offset = CONFIG_EFM32_OTGFS_RXFIFO_SIZE;

  /* Setup the host non-periodic Tx FIFO size (GNPTXFSIZ) */

  regval = (offset | (CONFIG_EFM32_OTGFS_NPTXFIFO_SIZE << _USB_GNPTXFSIZ_NPTXFINEPTXF0DEP_SHIFT));
  efm32_putreg(EFM32_USB_GNPTXFSIZ, regval);
  offset += CONFIG_EFM32_OTGFS_NPTXFIFO_SIZE;

  /* Set up the host periodic Tx FIFO size register (HPTXFSIZ) */

  regval = (offset | (CONFIG_EFM32_OTGFS_PTXFIFO_SIZE << _USB_HPTXFSIZ_PTXFSIZE_SHIFT));
  efm32_putreg(EFM32_USB_HPTXFSIZ, regval);

  /* If OTG were supported, we would need to clear HNP enable bit in the
   * USB_OTG control register about here.
   */

  /* Flush all FIFOs */

  efm32_flush_txfifos(USB_GRSTCTL_TXFNUM_FALL);
  efm32_flush_rxfifo();

  /* Clear all pending HC Interrupts */

  for (i = 0; i < EFM32_NHOST_CHANNELS; i++)
    {
      efm32_putreg(EFM32_USB_HCn_INT(i), 0xffffffff);
      efm32_putreg(EFM32_USB_HCn_INTMSK(i), 0);
    }

  /* Driver Vbus +5V (the smoke test).  Should be done elsewhere in OTG
   * mode.
   */

  efm32_vbusdrive(priv, true);

  /* Enable host interrupts */

  efm32_hostinit_enable();
}

/****************************************************************************
 * Name: efm32_sw_initialize
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

static inline void efm32_sw_initialize(FAR struct efm32_usbhost_s *priv)
{
  FAR struct usbhost_driver_s *drvr;
  FAR struct usbhost_hubport_s *hport;
  int i;

  /* Initialize the device operations */

  drvr                 = &priv->drvr;
  drvr->ep0configure   = efm32_ep0configure;
  drvr->epalloc        = efm32_epalloc;
  drvr->epfree         = efm32_epfree;
  drvr->alloc          = efm32_alloc;
  drvr->free           = efm32_free;
  drvr->ioalloc        = efm32_ioalloc;
  drvr->iofree         = efm32_iofree;
  drvr->ctrlin         = efm32_ctrlin;
  drvr->ctrlout        = efm32_ctrlout;
  drvr->transfer       = efm32_transfer;
#ifdef CONFIG_USBHOST_ASYNCH
  drvr->asynch         = efm32_asynch;
#endif
  drvr->cancel         = efm32_cancel;
#ifdef CONFIG_USBHOST_HUB
  drvr->connect        = efm32_connect;
#endif
  drvr->disconnect     = efm32_disconnect;

  /* Initialize the public port representation */

  hport                = &priv->rhport.hport;
  hport->drvr          = drvr;
#ifdef CONFIG_USBHOST_HUB
  hport->parent        = NULL;
#endif
  hport->ep0           = (usbhost_ep_t)&priv->ep0;
  hport->speed         = USB_SPEED_FULL;

  /* Initialize function address generation logic */

  usbhost_devaddr_initialize(&priv->rhport);

  /* Initialize semaphores */

  nxsem_init(&priv->pscsem,  0, 0);
  nxsem_init(&priv->exclsem, 0, 1);

  /* The pscsem semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_setprotocol(&priv->pscsem, SEM_PRIO_NONE);

  /* Initialize the driver state data */

  priv->smstate   = SMSTATE_DETACHED;
  priv->connected = false;
  priv->change    = false;

  /* Put all of the channels back in their initial, allocated state */

  memset(priv->chan, 0, EFM32_MAX_TX_FIFOS * sizeof(struct efm32_chan_s));

  /* Initialize each channel */

  for (i = 0; i < EFM32_MAX_TX_FIFOS; i++)
    {
      FAR struct efm32_chan_s *chan = &priv->chan[i];

      chan->chidx = i;

      /* The waitsem semaphore is used for signaling and, hence, should not
       * have priority inheritance enabled.
       */

      nxsem_init(&chan->waitsem,  0, 0);
      nxsem_setprotocol(&chan->waitsem, SEM_PRIO_NONE);
    }
}

/****************************************************************************
 * Name: efm32_hw_initialize
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

static inline int efm32_hw_initialize(FAR struct efm32_usbhost_s *priv)
{
  uint32_t regval;
  unsigned long timeout;

  /* "To initialize the core as host, the application must perform the
   *  following steps.
   *  1. Program USB_GINTMSK.PRTINT to unmask.
   *  2. Program the USB_HCFG register to select full-speed host.
   *  3. Program the USB_HPRT.PRTPWR bit to 1. This drives VBUS on the USB.
   *  4. Wait for the USB_HPRT.PRTCONNDET interrupt. This indicates that a
   *     device is connect to the port.
   *  5. Program the USB_HPRT.PRTRST bit to 1. This starts the reset process.
   *  6. Wait at least 10 ms for the reset process to complete.
   *  7. Program the USB_HPRT.PRTRST bit to 0.
   *  8. Wait for the USB_HPRT.PRTENCHNG interrupt.
   *  9. Read the USB_HPRT.PRTSPD field to get the enumerated speed.
   *  10. Program the USB_HFIR register with a value corresponding to the
   *      selected PHY clock. At this point, the host is up and running and
   *      the port register begins to report device disconnects, etc. The
   *      port is active with SOFs occurring down the enabled port.
   *  11. Program the RXFSIZE register to select the size of the receive FIFO.
   *  12. Program the NPTXFSIZE register to select the size and the start
   *      address of the Non-periodic Transmit FIFO for non-periodic
   *      transactions.
   *  13. Program the USB_HPTXFSIZ register to select the size and start
   *      address of the Periodic Transmit FIFO for periodic transactions."
   */
#warning Review for missing logic

  /* Reset after a PHY select and set Host mode.  First, wait for AHB master
   * IDLE state.
   */

  for (timeout = 0; timeout < EFM32_READY_DELAY; timeout++)
    {
      up_udelay(3);
      regval = efm32_getreg(EFM32_USB_GRSTCTL);
      if ((regval & USB_GRSTCTL_AHBIDLE) != 0)
        {
          break;
        }
    }

  /* Then perform the core soft reset. */

  efm32_putreg(EFM32_USB_GRSTCTL, USB_GRSTCTL_CSFTRST);
  for (timeout = 0; timeout < EFM32_READY_DELAY; timeout++)
    {
      regval = efm32_getreg(EFM32_USB_GRSTCTL);
      if ((regval & USB_GRSTCTL_CSFTRST) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);

  /* Initialize OTG features:  In order to support OTP, the HNPCAP and SRPCAP
   * bits would need to be set in the GUSBCFG register about here.
   */

  /* Force Host Mode */

  regval  = efm32_getreg(EFM32_USB_GUSBCFG);
  regval &= ~_USB_GUSBCFG_FORCEDEVMODE_MASK;
  regval |= USB_GUSBCFG_FORCEHSTMODE;
  efm32_putreg(EFM32_USB_GUSBCFG, regval);
  up_mdelay(50);

  /* Initialize host mode and return success */

  efm32_host_initialize(priv);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_usbhost_initialize
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

FAR struct usbhost_connection_s *efm32_usbhost_initialize(int controller)
{
  /* At present, there is only support for a single OTG FS host. Hence it is
   * pre-allocated as g_usbhost.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple devices.
   */

  FAR struct efm32_usbhost_s *priv = &g_usbhost;

  /* Sanity checks */

  DEBUGASSERT(controller == 0);

  /* "The USB requires the device to run from a 48 MHz crystal (2500 ppm or
   *  better). The core part of the USB will always run from HFCORECLKUSBC
   *  which is HFCLK undivided (48 MHz). The current consumption for the
   *  rest of the device can be reduced by dividing down HFCORECLK using
   *  the CMU_HFCORECLKDIV register. ..."
   *
   * "Follow these steps to enable the USB:
   *  1. Enable the clock to the system part by setting USB in
   *     CMU_HFCORECLKEN0.
   *  2. If the internal USB regulator is bypassed (by applying 3.3V on
   *     USB_VREGI and USB_VREGO externally), disable the regulator by
   *     setting VREGDIS in USB_CTRL.
   *  3. If the PHY is powered from VBUS using the internal regulator, the
   *     VREGO sense circuit should be enabled by setting VREGOSEN in
   *     USB_CTRL.
   *  4. Enable the USB PHY pins by setting PHYPEN in USB_ROUTE.
   *  5. If host or OTG dual-role device, set VBUSENAP in USB_CTRL to the
   *     desired value and then enable the USB_VBUSEN pin in USB_ROUTE. Set
   *     the MODE for the pin to PUSHPULL.
   *  6. If low-speed device, set DMPUAP in USB_CTRL to the desired value
   *     and then enable the USB_DMPU pin in USB_ROUTE. Set the MODE for the
   *     pin to PUSHPULL.
   *  7. Make sure HFXO is ready and selected. The core part requires the
   *     undivided HFCLK to be 48 MHz when USB is active (during
   *     suspend/session-off a 32 kHz clock is used)..
   *  8. Enable the clock to the core part by setting USBC in
   *     CMU_HFCORECLKEN0.
   *  9. Wait for the core to come out of reset. This is easiest done by
   *     polling a core register with non-zero reset value until it reads a
   *     non-zero value. This takes approximately 20 48-MHz cycles.
   *  10. Start initializing the USB core ..."
   */
#warning Missing Logic

  /* Make sure that interrupts from the OTG FS core are disabled */

  efm32_gint_disable();

  /* Reset the state of the host driver */

  efm32_sw_initialize(priv);

  /* Initialize the USB OTG FS core */

  efm32_hw_initialize(priv);

  /* Attach USB host controller interrupt handler */

  if (irq_attach(EFM32_IRQ_USB, efm32_gint_isr, NULL) != 0)
    {
      usbhost_trace1(USBHOST_TRACE1_IRQATTACH, 0);
      return NULL;
    }

  /* Enable USB OTG FS global interrupts */

  efm32_gint_enable();

  /* Enable interrupts at the interrupt controller */

  up_enable_irq(EFM32_IRQ_USB);
  return &g_usbconn;
}

#endif /* CONFIG_USBHOST && CONFIG_EFM32_OTGFS */

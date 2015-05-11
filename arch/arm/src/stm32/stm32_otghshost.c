/*******************************************************************************
 * arch/arm/src/stm32/stm32_otghshost.c
 *
 *   Copyright (C) 2012-2015 Gregory Nutt. All rights reserved.
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
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

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
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_devaddr.h>
#include <nuttx/usb/usbhost_trace.h>

#include <arch/irq.h>

#include "chip.h"             /* Includes default GPIO settings */
#include <arch/board/board.h> /* May redefine GPIO settings */

#include "up_arch.h"
#include "up_internal.h"

#include "stm32_usbhost.h"

#if defined(CONFIG_USBHOST) && defined(CONFIG_STM32_OTGHS)

/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************/
/* Configuration ***************************************************************/
/*
 * STM32 USB OTG HS Host Driver Support
 *
 * Pre-requisites
 *
 *  CONFIG_USBHOST      - Enable general USB host support
 *  CONFIG_STM32_OTGHS  - Enable the STM32 USB OTG HS block
 *  CONFIG_STM32_SYSCFG - Needed
 *
 * Options:
 *
 *  CONFIG_STM32_OTGHS_RXFIFO_SIZE - Size of the RX FIFO in 32-bit words.
 *    Default 128 (512 bytes)
 *  CONFIG_STM32_OTGHS_NPTXFIFO_SIZE - Size of the non-periodic Tx FIFO
 *    in 32-bit words.  Default 96 (384 bytes)
 *  CONFIG_STM32_OTGHS_PTXFIFO_SIZE - Size of the periodic Tx FIFO in 32-bit
 *    words.  Default 96 (384 bytes)
 *  CONFIG_STM32_OTGHS_DESCSIZE - Maximum size of a descriptor.  Default: 128
 *  CONFIG_STM32_OTGHS_SOFINTR - Enable SOF interrupts.  Why would you ever
 *    want to do that?
 *  CONFIG_STM32_USBHOST_REGDEBUG - Enable very low-level register access
 *    debug.  Depends on CONFIG_DEBUG.
 *  CONFIG_STM32_USBHOST_PKTDUMP - Dump all incoming and outgoing USB
 *    packets. Depends on CONFIG_DEBUG.
 */

/* Pre-requisites (partial) */

#ifndef CONFIG_STM32_SYSCFG
#  error "CONFIG_STM32_SYSCFG is required"
#endif

/* Default RxFIFO size */

#ifndef CONFIG_STM32_OTGHS_RXFIFO_SIZE
#  define CONFIG_STM32_OTGHS_RXFIFO_SIZE 128
#endif

/* Default host non-periodic Tx FIFO size */

#ifndef CONFIG_STM32_OTGHS_NPTXFIFO_SIZE
#  define CONFIG_STM32_OTGHS_NPTXFIFO_SIZE 96
#endif

/* Default host periodic Tx fifo size register */

#ifndef CONFIG_STM32_OTGHS_PTXFIFO_SIZE
#  define CONFIG_STM32_OTGHS_PTXFIFO_SIZE 96
#endif

/* Maximum size of a descriptor */

#ifndef CONFIG_STM32_OTGHS_DESCSIZE
#  define CONFIG_STM32_OTGHS_DESCSIZE 128
#endif

/* Register/packet debug depends on CONFIG_DEBUG */

#ifndef CONFIG_DEBUG
#  undef CONFIG_STM32_USBHOST_REGDEBUG
#  undef CONFIG_STM32_USBHOST_PKTDUMP
#endif

/* HCD Setup *******************************************************************/
/* Hardware capabilities */

#define STM32_NHOST_CHANNELS      12  /* Number of host channels */
#define STM32_MAX_PACKET_SIZE     64  /* Full speed max packet size */
#define STM32_EP0_DEF_PACKET_SIZE 8   /* EP0 default packet size */
#define STM32_EP0_MAX_PACKET_SIZE 64  /* EP0 HS max packet size */
#define STM32_MAX_TX_FIFOS        12  /* Max number of TX FIFOs */
#define STM32_MAX_PKTCOUNT        256 /* Max packet count */
#define STM32_RETRY_COUNT         3   /* Number of ctrl transfer retries */

/* Delays **********************************************************************/

#define STM32_READY_DELAY         200000 /* In loop counts */
#define STM32_FLUSH_DELAY         200000 /* In loop counts */
#define STM32_SETUP_DELAY         SEC2TICK(5) /* 5 seconds in system ticks */
#define STM32_DATANAK_DELAY       SEC2TICK(5) /* 5 seconds in system ticks */

/* Ever-present MIN/MAX macros */

#ifndef MIN
#  define  MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define  MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/*******************************************************************************
 * Private Types
 *******************************************************************************/

/* The following enumeration represents the various states of the USB host
 * state machine (for debug purposes only)
 */

enum stm32_smstate_e
{
  SMSTATE_DETACHED = 0,  /* Not attached to a device */
  SMSTATE_ATTACHED,      /* Attached to a device */
  SMSTATE_ENUM,          /* Attached, enumerating */
  SMSTATE_CLASS_BOUND,   /* Enumeration complete, class bound */
};

/* This enumeration provides the reason for the channel halt. */

enum stm32_chreason_e
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
 * in the structure could be moved in struct stm32_ubhost_s to achieve
 * some memory savings.
 */

struct stm32_chan_s
{
  sem_t             waitsem;   /* Channel wait semaphore */
  volatile uint8_t  result;    /* The result of the transfer */
  volatile uint8_t  chreason;  /* Channel halt reason. See enum stm32_chreason_e */
  uint8_t           chidx;     /* Channel index */
  uint8_t           epno;      /* Device endpoint number (0-127) */
  uint8_t           eptype;    /* See OTGHS_EPTYPE_* definitions */
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

struct stm32_ctrlinfo_s
{
  uint8_t           inndx;     /* EP0 IN control channel index */
  uint8_t           outndx;    /* EP0 OUT control channel index */
};

/* This structure retains the state of the USB host controller */

struct stm32_usbhost_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to structstm32_usbhost_s.
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
  struct stm32_ctrlinfo_s ep0;  /* Root hub port EP0 description */

#ifdef CONFIG_USBHOST_HUB
  /* Used to pass external hub port events */

  volatile struct usbhost_hubport_s *hport;
#endif

  /* The state of each host channel */

  struct stm32_chan_s chan[STM32_MAX_TX_FIFOS];
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Register operations ********************************************************/

#ifdef CONFIG_STM32_USBHOST_REGDEBUG
static void stm32_printreg(uint32_t addr, uint32_t val, bool iswrite);
static void stm32_checkreg(uint32_t addr, uint32_t val, bool iswrite);
static uint32_t stm32_getreg(uint32_t addr);
static void stm32_putreg(uint32_t addr, uint32_t value);
#else
# define stm32_getreg(addr)     getreg32(addr)
# define stm32_putreg(addr,val) putreg32(val,addr)
#endif

static inline void stm32_modifyreg(uint32_t addr, uint32_t clrbits,
                                   uint32_t setbits);

#ifdef CONFIG_STM32_USBHOST_PKTDUMP
#  define stm32_pktdump(m,b,n) lib_dumpbuffer(m,b,n)
#else
#  define stm32_pktdump(m,b,n)
#endif

/* Semaphores ******************************************************************/

static void stm32_takesem(sem_t *sem);
#define stm32_givesem(s) sem_post(s);

/* Byte stream access helper functions *****************************************/

static inline uint16_t stm32_getle16(const uint8_t *val);

/* Channel management **********************************************************/

static int stm32_chan_alloc(FAR struct stm32_usbhost_s *priv);
static inline void stm32_chan_free(FAR struct stm32_usbhost_s *priv, int chidx);
static inline void stm32_chan_freeall(FAR struct stm32_usbhost_s *priv);
static void stm32_chan_configure(FAR struct stm32_usbhost_s *priv, int chidx);
static void stm32_chan_halt(FAR struct stm32_usbhost_s *priv, int chidx,
                            enum stm32_chreason_e chreason);
static int stm32_chan_waitsetup(FAR struct stm32_usbhost_s *priv,
                                FAR struct stm32_chan_s *chan);
#ifdef CONFIG_USBHOST_ASYNCH
static int stm32_chan_asynchsetup(FAR struct stm32_usbhost_s *priv,
                                  FAR struct stm32_chan_s *chan,
                                  usbhost_asynch_t callback, FAR void *arg);
#endif
static int stm32_chan_wait(FAR struct stm32_usbhost_s *priv,
                           FAR struct stm32_chan_s *chan);
static void stm32_chan_wakeup(FAR struct stm32_usbhost_s *priv,
                              FAR struct stm32_chan_s *chan);
static int stm32_ctrlchan_alloc(FAR struct stm32_usbhost_s *priv,
                                uint8_t epno, uint8_t funcaddr, uint8_t speed,
                                FAR struct stm32_ctrlinfo_s *ctrlep);
static int stm32_ctrlep_alloc(FAR struct stm32_usbhost_s *priv,
                              FAR const struct usbhost_epdesc_s *epdesc,
                              FAR usbhost_ep_t *ep);
static int stm32_xfrep_alloc(FAR struct stm32_usbhost_s *priv,
                              FAR const struct usbhost_epdesc_s *epdesc,
                              FAR usbhost_ep_t *ep);

/* Control/data transfer logic *************************************************/

static void stm32_transfer_start(FAR struct stm32_usbhost_s *priv, int chidx);
#if 0 /* Not used */
static inline uint16_t stm32_getframe(void);
#endif
static int stm32_ctrl_sendsetup(FAR struct stm32_usbhost_s *priv,
                                FAR struct stm32_ctrlinfo_s *ep0,
                                FAR const struct usb_ctrlreq_s *req);
static int stm32_ctrl_senddata(FAR struct stm32_usbhost_s *priv,
                               FAR struct stm32_ctrlinfo_s *ep0,
                               FAR uint8_t *buffer, unsigned int buflen);
static int stm32_ctrl_recvdata(FAR struct stm32_usbhost_s *priv,
                               FAR struct stm32_ctrlinfo_s *ep0,
                               FAR uint8_t *buffer, unsigned int buflen);
static int stm32_in_setup(FAR struct stm32_usbhost_s *priv, int chidx);
static ssize_t stm32_in_transfer(FAR struct stm32_usbhost_s *priv, int chidx,
                                 FAR uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void stm32_in_next(FAR struct stm32_usbhost_s *priv,
                          FAR struct stm32_chan_s *chan);
static int stm32_in_asynch(FAR struct stm32_usbhost_s *priv, int chidx,
                           FAR uint8_t *buffer, size_t buflen,
                           usbhost_asynch_t callback, FAR void *arg);
#endif
static int stm32_out_setup(FAR struct stm32_usbhost_s *priv, int chidx);
static ssize_t stm32_out_transfer(FAR struct stm32_usbhost_s *priv, int chidx,
                                  FAR uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void stm32_out_next(FAR struct stm32_usbhost_s *priv,
                           FAR struct stm32_chan_s *chan);
static int stm32_out_asynch(FAR struct stm32_usbhost_s *priv, int chidx,
                            FAR uint8_t *buffer, size_t buflen,
                            usbhost_asynch_t callback, FAR void *arg);
#endif

/* Interrupt handling **********************************************************/
/* Lower level interrupt handlers */

static void stm32_gint_wrpacket(FAR struct stm32_usbhost_s *priv,
                                FAR uint8_t *buffer, int chidx, int buflen);
static inline void stm32_gint_hcinisr(FAR struct stm32_usbhost_s *priv,
                                      int chidx);
static inline void stm32_gint_hcoutisr(FAR struct stm32_usbhost_s *priv,
                                       int chidx);
static void stm32_gint_connected(FAR struct stm32_usbhost_s *priv);
static void stm32_gint_disconnected(FAR struct stm32_usbhost_s *priv);

/* Second level interrupt handlers */

#ifdef CONFIG_STM32_OTGHS_SOFINTR
static inline void stm32_gint_sofisr(FAR struct stm32_usbhost_s *priv);
#endif
static inline void stm32_gint_rxflvlisr(FAR struct stm32_usbhost_s *priv);
static inline void stm32_gint_nptxfeisr(FAR struct stm32_usbhost_s *priv);
static inline void stm32_gint_ptxfeisr(FAR struct stm32_usbhost_s *priv);
static inline void stm32_gint_hcisr(FAR struct stm32_usbhost_s *priv);
static inline void stm32_gint_hprtisr(FAR struct stm32_usbhost_s *priv);
static inline void stm32_gint_discisr(FAR struct stm32_usbhost_s *priv);
static inline void stm32_gint_ipxfrisr(FAR struct stm32_usbhost_s *priv);

/* First level, global interrupt handler */

static int stm32_gint_isr(int irq, FAR void *context);

/* Interrupt controls */

static void stm32_gint_enable(void);
static void stm32_gint_disable(void);
static inline void stm32_hostinit_enable(void);
static void stm32_txfe_enable(FAR struct stm32_usbhost_s *priv, int chidx);

/* USB host controller operations **********************************************/

static int stm32_wait(FAR struct usbhost_connection_s *conn,
                      FAR struct usbhost_hubport_s **hport);
static int stm32_rh_enumerate(FAR struct stm32_usbhost_s *priv,
                              FAR struct usbhost_connection_s *conn,
                              FAR struct usbhost_hubport_s *hport);
static int stm32_enumerate(FAR struct usbhost_connection_s *conn,
                           FAR struct usbhost_hubport_s *hport);

static int stm32_ep0configure(FAR struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep0, uint8_t funcaddr, uint8_t speed,
                              uint16_t maxpacketsize);
static int stm32_epalloc(FAR struct usbhost_driver_s *drvr,
                         FAR const FAR struct usbhost_epdesc_s *epdesc,
                         FAR usbhost_ep_t *ep);
static int stm32_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int stm32_alloc(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t **buffer, FAR size_t *maxlen);
static int stm32_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);
static int stm32_ioalloc(FAR struct usbhost_driver_s *drvr,
                         FAR uint8_t **buffer, size_t buflen);
static int stm32_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);
static int stm32_ctrlin(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                        const struct usb_ctrlreq_s *req,
                        FAR uint8_t *buffer);
static int stm32_ctrlout(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                         FAR const struct usb_ctrlreq_s *req,
                         FAR const uint8_t *buffer);
static ssize_t stm32_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                              FAR uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static int stm32_asynch(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                        FAR uint8_t *buffer, size_t buflen,
                        usbhost_asynch_t callback, FAR void *arg);
#endif
static int stm32_cancel(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
#ifdef CONFIG_USBHOST_HUB
static int stm32_connect(FAR struct usbhost_driver_s *drvr,
                         FAR struct usbhost_hubport_s *hport,
                         bool connected);
#endif
static void stm32_disconnect(FAR struct usbhost_driver_s *drvr,
                             FAR struct usbhost_hubport_s *hport);

/* Initialization **************************************************************/

static void stm32_portreset(FAR struct stm32_usbhost_s *priv);
static void stm32_flush_txfifos(uint32_t txfnum);
static void stm32_flush_rxfifo(void);
static void stm32_vbusdrive(FAR struct stm32_usbhost_s *priv, bool state);
static void stm32_host_initialize(FAR struct stm32_usbhost_s *priv);

static inline void stm32_sw_initialize(FAR struct stm32_usbhost_s *priv);
static inline int stm32_hw_initialize(FAR struct stm32_usbhost_s *priv);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

/* In this driver implementation, support is provided for only a single a single
 * USB device.  All status information can be simply retained in a single global
 * instance.
 */

static struct stm32_usbhost_s g_usbhost;

/* This is the connection/enumeration interface */

static struct usbhost_connection_s g_usbconn =
{
  .wait             = stm32_wait,
  .enumerate        = stm32_enumerate,
};

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: stm32_printreg
 *
 * Description:
 *   Print the contents of an STM32xx register operation
 *
 *******************************************************************************/

#ifdef CONFIG_STM32_USBHOST_REGDEBUG
static void stm32_printreg(uint32_t addr, uint32_t val, bool iswrite)
{
  lldbg("%08x%s%08x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/*******************************************************************************
 * Name: stm32_checkreg
 *
 * Description:
 *   Get the contents of an STM32 register
 *
 *******************************************************************************/

#ifdef CONFIG_STM32_USBHOST_REGDEBUG
static void stm32_checkreg(uint32_t addr, uint32_t val, bool iswrite)
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

              stm32_printreg(prevaddr, preval, prevwrite);
            }
          else
            {
              /* No.. More than one. */

              lldbg("[repeats %d more times]\n", count);
            }
        }

      /* Save the new address, value, count, and operation for next time */

      prevaddr  = addr;
      preval    = val;
      count     = 0;
      prevwrite = iswrite;

      /* Show the new regisgter access */

      stm32_printreg(addr, val, iswrite);
    }
}
#endif

/*******************************************************************************
 * Name: stm32_getreg
 *
 * Description:
 *   Get the contents of an STM32 register
 *
 *******************************************************************************/

#ifdef CONFIG_STM32_USBHOST_REGDEBUG
static uint32_t stm32_getreg(uint32_t addr)
{
  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Check if we need to print this value */

  stm32_checkreg(addr, val, false);
  return val;
}
#endif

/*******************************************************************************
 * Name: stm32_putreg
 *
 * Description:
 *   Set the contents of an STM32 register to a value
 *
 *******************************************************************************/

#ifdef CONFIG_STM32_USBHOST_REGDEBUG
static void stm32_putreg(uint32_t addr, uint32_t val)
{
  /* Check if we need to print this value */

  stm32_checkreg(addr, val, true);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/*******************************************************************************
 * Name: stm32_modifyreg
 *
 * Description:
 *   Modify selected bits of an STM32 register.
 *
 *******************************************************************************/

static inline void stm32_modifyreg(uint32_t addr, uint32_t clrbits, uint32_t setbits)
{
  stm32_putreg(addr, (((stm32_getreg(addr)) & ~clrbits) | setbits));
}

/****************************************************************************
 * Name: stm32_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 *******************************************************************************/

static void stm32_takesem(sem_t *sem)
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
 * Name: stm32_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 *******************************************************************************/

static inline uint16_t stm32_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/*******************************************************************************
 * Name: stm32_chan_alloc
 *
 * Description:
 *   Allocate a channel.
 *
 *******************************************************************************/

static int stm32_chan_alloc(FAR struct stm32_usbhost_s *priv)
{
  int chidx;

  /* Search the table of channels */

  for (chidx = 0; chidx < STM32_NHOST_CHANNELS; chidx++)
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

/*******************************************************************************
 * Name: stm32_chan_free
 *
 * Description:
 *   Free a previoiusly allocated channel.
 *
 *******************************************************************************/

static void stm32_chan_free(FAR struct stm32_usbhost_s *priv, int chidx)
{
  DEBUGASSERT((unsigned)chidx < STM32_NHOST_CHANNELS);

  /* Halt the channel */

  stm32_chan_halt(priv, chidx, CHREASON_FREED);

  /* Mark the channel available */

  priv->chan[chidx].inuse = false;
}

/*******************************************************************************
 * Name: stm32_chan_freeall
 *
 * Description:
 *   Free all channels.
 *
 *******************************************************************************/

static inline void stm32_chan_freeall(FAR struct stm32_usbhost_s *priv)
{
   uint8_t chidx;

   /* Free all host channels */

   for (chidx = 2; chidx < STM32_NHOST_CHANNELS; chidx ++)
     {
       stm32_chan_free(priv, chidx);
     }
}

/*******************************************************************************
 * Name: stm32_chan_configure
 *
 * Description:
 *   Configure or re-configure a host channel.  Host channels are configured
 *   when endpoint is allocated and EP0 (only) is re-configured with the
 *   max packet size or device address changes.
 *
 *******************************************************************************/

static void stm32_chan_configure(FAR struct stm32_usbhost_s *priv, int chidx)
{
  FAR struct stm32_chan_s *chan = &priv->chan[chidx];
  uint32_t regval;

  /* Clear any old pending interrupts for this host channel. */

  stm32_putreg(STM32_OTGHS_HCINT(chidx), 0xffffffff);

  /* Enable channel interrupts required for transfers on this channel. */

  regval = 0;

  switch (chan->eptype)
    {
    case OTGHS_EPTYPE_CTRL:
    case OTGHS_EPTYPE_BULK:
      {
#ifdef HAVE_USBHOST_TRACE_VERBOSE
        uint16_t intrace;
        uint16_t outtrace;

        /* Determine the definitive trace ID to use below */

        if (chan->eptype == OTGHS_EPTYPE_CTRL)
          {
            intrace  = OTGHS_VTRACE2_CHANCONF_CTRL_IN;
            outtrace = OTGHS_VTRACE2_CHANCONF_CTRL_OUT;
          }
        else
          {
            intrace  = OTGHS_VTRACE2_CHANCONF_BULK_IN;
            outtrace = OTGHS_VTRACE2_CHANCONF_BULK_OUT;
          }
#endif

        /* Interrupts required for CTRL and BULK endpoints */

        regval |= (OTGHS_HCINT_XFRC  | OTGHS_HCINT_STALL | OTGHS_HCINT_NAK |
                   OTGHS_HCINT_TXERR | OTGHS_HCINT_DTERR);

        /* Additional setting for IN/OUT endpoints */

        if (chan->in)
          {
            usbhost_vtrace2(intrace, chidx, chan->epno);
            regval |= OTGHS_HCINT_BBERR;
          }
        else
          {
            usbhost_vtrace2(outtrace, chidx, chan->epno);
            regval |= OTGHS_HCINT_NYET;
          }
      }
      break;

    case OTGHS_EPTYPE_INTR:
      {
        /* Interrupts required for INTR endpoints */

        regval |= (OTGHS_HCINT_XFRC | OTGHS_HCINT_STALL | OTGHS_HCINT_NAK |
                   OTGHS_HCINT_TXERR | OTGHS_HCINT_FRMOR | OTGHS_HCINT_DTERR);

        /* Additional setting for IN endpoints */

        if (chan->in)
          {
            usbhost_vtrace2(OTGHS_VTRACE2_CHANCONF_INTR_IN, chidx,
                            chan->epno);
            regval |= OTGHS_HCINT_BBERR;
          }
#ifdef HAVE_USBHOST_TRACE_VERBOSE
        else
          {
            usbhost_vtrace2(OTGHS_VTRACE2_CHANCONF_INTR_OUT, chidx,
                            chan->epno);
          }
#endif
      }
      break;

    case OTGHS_EPTYPE_ISOC:
      {
        /* Interrupts required for ISOC endpoints */

        regval |= (OTGHS_HCINT_XFRC | OTGHS_HCINT_ACK | OTGHS_HCINT_FRMOR);

        /* Additional setting for IN endpoints */

        if (chan->in)
          {
            usbhost_vtrace2(OTGHS_VTRACE2_CHANCONF_ISOC_IN, chidx,
                            chan->epno);
            regval |= (OTGHS_HCINT_TXERR | OTGHS_HCINT_BBERR);
          }
#ifdef HAVE_USBHOST_TRACE_VERBOSE
        else
          {
            usbhost_vtrace2(OTGHS_VTRACE2_CHANCONF_ISOC_OUT, chidx,
                            chan->epno);
          }
#endif
      }
      break;
    }

  stm32_putreg(STM32_OTGHS_HCINTMSK(chidx), regval);

  /* Enable the top level host channel interrupt. */

  stm32_modifyreg(STM32_OTGHS_HAINTMSK, 0, OTGHS_HAINT(chidx));

  /* Make sure host channel interrupts are enabled. */

  stm32_modifyreg(STM32_OTGHS_GINTMSK, 0, OTGHS_GINT_HC);

  /* Program the HCCHAR register */

  regval = ((uint32_t)chan->maxpacket << OTGHS_HCCHAR_MPSIZ_SHIFT) |
           ((uint32_t)chan->epno      << OTGHS_HCCHAR_EPNUM_SHIFT) |
           ((uint32_t)chan->eptype    << OTGHS_HCCHAR_EPTYP_SHIFT) |
           ((uint32_t)chan->funcaddr  << OTGHS_HCCHAR_DAD_SHIFT);

  /* Special case settings for low speed devices */

  if (chan->speed == USB_SPEED_LOW)
    {
      regval |= OTGHS_HCCHAR_LSDEV;
    }

  /* Special case settings for IN endpoints */

  if (chan->in)
    {
      regval |= OTGHS_HCCHAR_EPDIR_IN;
    }

  /* Special case settings for INTR endpoints */

  if (chan->eptype == OTGHS_EPTYPE_INTR)
    {
      regval |= OTGHS_HCCHAR_ODDFRM;
    }

  /* Write the channel configuration */

  stm32_putreg(STM32_OTGHS_HCCHAR(chidx), regval);
}

/*******************************************************************************
 * Name: stm32_chan_halt
 *
 * Description:
 *   Halt the channel associated with 'chidx' by setting the CHannel DISable
 *   (CHDIS) bit in in the HCCHAR register.
 *
 *******************************************************************************/

static void stm32_chan_halt(FAR struct stm32_usbhost_s *priv, int chidx,
                            enum stm32_chreason_e chreason)
{
  uint32_t hcchar;
  uint32_t intmsk;
  uint32_t eptype;
  unsigned int avail;

  /* Save the reason for the halt.  We need this in the channel halt interrupt
   * handling logic to know what to do next.
   */

  usbhost_vtrace2(OTGHS_VTRACE2_CHANHALT, chidx, chreason);

  priv->chan[chidx].chreason = (uint8_t)chreason;

  /* "The application can disable any channel by programming the OTG_HS_HCCHARx
   *  register with the CHDIS and CHENA bits set to 1. This enables the OTG_HS
   *  host to flush the posted requests (if any) and generates a channel halted
   *  interrupt. The application must wait for the CHH interrupt in OTG_HS_HCINTx
   *  before reallocating the channel for other transactions.  The OTG_HS host
   *  does not interrupt the transaction that has already been started on the
   *  USB."
   */

  hcchar  = stm32_getreg(STM32_OTGHS_HCCHAR(chidx));
  hcchar |= (OTGHS_HCCHAR_CHDIS | OTGHS_HCCHAR_CHENA);

  /* Get the endpoint type from the HCCHAR register */

  eptype = hcchar & OTGHS_HCCHAR_EPTYP_MASK;

  /* Check for space in the Tx FIFO to issue the halt.
   *
   * "Before disabling a channel, the application must ensure that there is at
   *  least one free space available in the non-periodic request queue (when
   *  disabling a non-periodic channel) or the periodic request queue (when
   *  disabling a periodic channel). The application can simply flush the
   *  posted requests when the Request queue is full (before disabling the
   *  channel), by programming the OTG_HS_HCCHARx register with the CHDIS bit
   *  set to 1, and the CHENA bit cleared to 0.
   */

  if (eptype == OTGHS_HCCHAR_EPTYP_CTRL || eptype == OTGHS_HCCHAR_EPTYP_BULK)
    {
      /* Get the number of words available in the non-periodic Tx FIFO. */

      avail = stm32_getreg(STM32_OTGHS_HNPTXSTS) & OTGHS_HNPTXSTS_NPTXFSAV_MASK;
    }
  else /* if (eptype == OTGHS_HCCHAR_EPTYP_ISOC || eptype == OTGHS_HCCHAR_EPTYP_INTR) */
    {
      /* Get the number of words available in the non-periodic Tx FIFO. */

      avail = stm32_getreg(STM32_OTGHS_HPTXSTS) & OTGHS_HPTXSTS_PTXFSAVL_MASK;
    }

  /* Check if there is any space available in the Tx FIFO. */

  if (avail == 0)
    {
      /* The Tx FIFO is full... disable the channel to flush the requests */

      hcchar &= ~OTGHS_HCCHAR_CHENA;
    }

  /* Unmask the CHannel Halted (CHH) interrupt */

  intmsk  = stm32_getreg(STM32_OTGHS_HCINTMSK(chidx));
  intmsk |= OTGHS_HCINT_CHH;
  stm32_putreg(STM32_OTGHS_HCINTMSK(chidx), intmsk);

  /* Halt the channel by setting CHDIS (and maybe CHENA) in the HCCHAR */

  stm32_putreg(STM32_OTGHS_HCCHAR(chidx), hcchar);
}

/*******************************************************************************
 * Name: stm32_chan_waitsetup
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
 *******************************************************************************/

static int stm32_chan_waitsetup(FAR struct stm32_usbhost_s *priv,
                                FAR struct stm32_chan_s *chan)
{
  irqstate_t flags = irqsave();
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

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: stm32_chan_asynchsetup
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
 *******************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int stm32_chan_asynchsetup(FAR struct stm32_usbhost_s *priv,
                                  FAR struct stm32_chan_s *chan,
                                  usbhost_asynch_t callback, FAR void *arg)
{
  irqstate_t flags = irqsave();
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

  irqrestore(flags);
  return ret;
}
#endif

/*******************************************************************************
 * Name: stm32_chan_wait
 *
 * Description:
 *   Wait for a transfer on a channel to complete.
 *
 * Assumptions:
 *   Called from a normal thread context
 *
 *******************************************************************************/

static int stm32_chan_wait(FAR struct stm32_usbhost_s *priv,
                           FAR struct stm32_chan_s *chan)
{
  irqstate_t flags;
  int ret;

  /* Disable interrupts so that the following operations will be atomic.  On
   * the OTG HS global interrupt needs to be disabled.  However, here we disable
   * all interrupts to exploit that fact that interrupts will be re-enabled
   * while we wait.
   */

  flags = irqsave();

  /* Loop, testing for an end of transfer condition.  The channel 'result'
   * was set to EBUSY and 'waiter' was set to true before the transfer; 'waiter'
   * will be set to false and 'result' will be set appropriately when the
   * tranfer is completed.
   */

  do
    {
      /* Wait for the transfer to complete.  NOTE the transfer may already
       * completed before we get here or the transfer may complete while we
       * wait here.
       */

      ret = sem_wait(&chan->waitsem);

      /* sem_wait should succeed.  But it is possible that we could be
       * awakened by a signal too.
       */

      DEBUGASSERT(ret == OK || get_errno() == EINTR);
    }
  while (chan->waiter);

  /* The transfer is complete re-enable interrupts and return the result */

  ret = -(int)chan->result;
  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: stm32_chan_wakeup
 *
 * Description:
 *   A channel transfer has completed... wakeup any threads waiting for the
 *   transfer to complete.
 *
 * Assumptions:
 *   This function is called from the transfer complete interrupt handler for
 *   the channel.  Interrupts are disabled.
 *
 *******************************************************************************/

static void stm32_chan_wakeup(FAR struct stm32_usbhost_s *priv,
                              FAR struct stm32_chan_s *chan)
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

          usbhost_vtrace2(chan->in ? OTGHS_VTRACE2_CHANWAKEUP_IN :
                                     OTGHS_VTRACE2_CHANWAKEUP_OUT,
                          chan->epno, chan->result);

          stm32_givesem(&chan->waitsem);
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
              stm32_in_next(priv, chan);
            }
          else
            {
              stm32_out_next(priv, chan);
            }
        }
#endif
    }
}

/*******************************************************************************
 * Name: stm32_ctrlchan_alloc
 *
 * Description:
 *   Allocate and configured channels for a control pipe.
 *
 *******************************************************************************/

static int stm32_ctrlchan_alloc(FAR struct stm32_usbhost_s *priv,
                                uint8_t epno, uint8_t funcaddr, uint8_t speed,
                                FAR struct stm32_ctrlinfo_s *ctrlep)
{
  FAR struct stm32_chan_s *chan;
  int inndx;
  int outndx;

  outndx = stm32_chan_alloc(priv);
  if (outndx < 0)
    {
      return -ENOMEM;
    }

  ctrlep->outndx  = outndx;
  chan            = &priv->chan[outndx];
  chan->epno      = epno;
  chan->in        = false;
  chan->eptype    = OTGHS_EPTYPE_CTRL;
  chan->funcaddr  = funcaddr;
  chan->speed     = speed;
  chan->maxpacket = STM32_EP0_DEF_PACKET_SIZE;
  chan->indata1   = false;
  chan->outdata1  = false;

  /* Configure control OUT channels */

  stm32_chan_configure(priv, outndx);

  /* Allocate and initialize the control IN channel */

  inndx = stm32_chan_alloc(priv);
  if (inndx < 0)
    {
      stm32_chan_free(priv, outndx);
      return -ENOMEM;
    }

  ctrlep->inndx   = inndx;
  chan            = &priv->chan[inndx];
  chan->epno      = epno;
  chan->in        = true;
  chan->eptype    = OTGHS_EPTYPE_CTRL;
  chan->funcaddr  = funcaddr;
  chan->speed     = speed;
  chan->maxpacket = STM32_EP0_DEF_PACKET_SIZE;
  chan->indata1   = false;
  chan->outdata1  = false;

  /* Configure control IN channels */

  stm32_chan_configure(priv, inndx);
  return OK;
}

/*******************************************************************************
 * Name: stm32_ctrlep_alloc
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_ctrlep_alloc(FAR struct stm32_usbhost_s *priv,
                              FAR const struct usbhost_epdesc_s *epdesc,
                              FAR usbhost_ep_t *ep)
{
  FAR struct usbhost_hubport_s *hport;
  FAR struct stm32_ctrlinfo_s *ctrlep;
  int ret;

  /* Sanity check.  NOTE that this method should only be called if a device is
   * connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(epdesc->hport != NULL);
  hport = epdesc->hport;

  /* Allocate a container for the control endpoint */

  ctrlep = (FAR struct stm32_ctrlinfo_s *)kmm_malloc(sizeof(struct stm32_ctrlinfo_s));
  if (ctrlep == NULL)
    {
      udbg("ERROR: Failed to allocate control endpoint container\n");
      return -ENOMEM;
    }

  /* Then allocate and configure the IN/OUT channnels  */

  ret = stm32_ctrlchan_alloc(priv, epdesc->addr & USB_EPNO_MASK,
                             hport->funcaddr, hport->speed, ctrlep);
  if (ret < 0)
    {
      udbg("ERROR: stm32_ctrlchan_alloc failed: %d\n", ret);
      kmm_free(ctrlep);
      return ret;
    }

  /* Return a pointer to the control pipe container as the pipe "handle" */

  *ep = (usbhost_ep_t)ctrlep;
  return OK;
}

/************************************************************************************
 * Name: stm32_xfrep_alloc
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_xfrep_alloc(FAR struct stm32_usbhost_s *priv,
                              FAR const struct usbhost_epdesc_s *epdesc,
                              FAR usbhost_ep_t *ep)
{
  struct usbhost_hubport_s *hport;
  FAR struct stm32_chan_s *chan;
  int chidx;

  /* Sanity check.  NOTE that this method should only be called if a device is
   * connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(epdesc->hport != NULL);
  hport = epdesc->hport;

  /* Allocate a host channel for the endpoint */

  chidx = stm32_chan_alloc(priv);
  if (chidx < 0)
    {
      udbg("ERROR: Failed to allocate a host channel\n");
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

  stm32_chan_configure(priv, chidx);

  /* Return the index to the allocated channel as the endpoint "handle" */

  *ep = (usbhost_ep_t)chidx;
  return OK;
}

/*******************************************************************************
 * Name: stm32_transfer_start
 *
 * Description:
 *   Start at transfer on the select IN or OUT channel.
 *
 *******************************************************************************/

static void stm32_transfer_start(FAR struct stm32_usbhost_s *priv, int chidx)
{
  FAR struct stm32_chan_s *chan;
  uint32_t regval;
  unsigned int npackets;
  unsigned int maxpacket;
  unsigned int avail;
  unsigned int wrsize;
  unsigned int minsize;

  /* Set up the initial state of the transfer */

  chan           = &priv->chan[chidx];

  usbhost_vtrace2(OTGHS_VTRACE2_STARTTRANSFER, chidx, chan->buflen);

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

      if (npackets > STM32_MAX_PKTCOUNT)
        {
          npackets = STM32_MAX_PKTCOUNT;
          chan->buflen = STM32_MAX_PKTCOUNT * maxpacket;
          usbhost_trace2(OTGHS_TRACE2_CLIP, chidx, chan->buflen);
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

  regval = ((uint32_t)chan->buflen << OTGHS_HCTSIZ_XFRSIZ_SHIFT) |
           ((uint32_t)npackets << OTGHS_HCTSIZ_PKTCNT_SHIFT) |
           ((uint32_t)chan->pid << OTGHS_HCTSIZ_DPID_SHIFT);
  stm32_putreg(STM32_OTGHS_HCTSIZ(chidx), regval);

  /* Setup the HCCHAR register: Frame oddness and host channel enable */

  regval = stm32_getreg(STM32_OTGHS_HCCHAR(chidx));

  /* Set/clear the Odd Frame bit.  Check for an even frame; if so set Odd
   * Frame. This field is applicable for only periodic (isochronous and
   * interrupt) channels.
   */

  if ((stm32_getreg(STM32_OTGHS_HFNUM) & 1) == 0)
    {
      regval |= OTGHS_HCCHAR_ODDFRM;
    }
  else
    {
      regval &= ~OTGHS_HCCHAR_ODDFRM;
    }

  regval &= ~OTGHS_HCCHAR_CHDIS;
  regval |= OTGHS_HCCHAR_CHENA;
  stm32_putreg(STM32_OTGHS_HCCHAR(chidx), regval);

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
        case OTGHS_EPTYPE_CTRL: /* Non periodic transfer */
        case OTGHS_EPTYPE_BULK:
          {
            /* Read the Non-periodic Tx FIFO status register */

            regval = stm32_getreg(STM32_OTGHS_HNPTXSTS);
            avail  = ((regval & OTGHS_HNPTXSTS_NPTXFSAV_MASK) >> OTGHS_HNPTXSTS_NPTXFSAV_SHIFT) << 2;
          }
          break;

        /* Periodic transfer */

        case OTGHS_EPTYPE_INTR:
        case OTGHS_EPTYPE_ISOC:
          {
            /* Read the Non-periodic Tx FIFO status register */

            regval = stm32_getreg(STM32_OTGHS_HPTXSTS);
            avail  = ((regval & OTGHS_HPTXSTS_PTXFSAVL_MASK) >> OTGHS_HPTXSTS_PTXFSAVL_SHIFT) << 2;
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

          stm32_gint_wrpacket(priv, chan->buffer, chidx, wrsize);
        }

      /* Did we put the entire buffer into the Tx FIFO? */

      if (chan->buflen > avail)
        {
          /* No, there was insufficient space to hold the entire transfer ...
           * Enable the Tx FIFO interrupt to handle the transfer when the Tx
           * FIFO becomes empty.
           */

           stm32_txfe_enable(priv, chidx);
        }
    }
}

/*******************************************************************************
 * Name: stm32_getframe
 *
 * Description:
 *   Get the current frame number.  The frame number (FRNUM) field increments
 *   when a new SOF is transmitted on the USB, and is cleared to 0 when it
 *   reaches 0x3fff.
 *
 *******************************************************************************/

#if 0 /* Not used */
static inline uint16_t stm32_getframe(void)
{
  return (uint16_t)(stm32_getreg(STM32_OTGHS_HFNUM) & OTGHS_HFNUM_FRNUM_MASK);
}
#endif

/*******************************************************************************
 * Name: stm32_ctrl_sendsetup
 *
 * Description:
 *   Send an IN/OUT SETUP packet.
 *
 *******************************************************************************/

static int stm32_ctrl_sendsetup(FAR struct stm32_usbhost_s *priv,
                                FAR struct stm32_ctrlinfo_s *ep0,
                                FAR const struct usb_ctrlreq_s *req)
{
  FAR struct stm32_chan_s *chan;
  uint32_t start;
  uint32_t elapsed;
  int ret;

  /* Loop while the device reports NAK (and a timeout is not exceeded */

  chan  = &priv->chan[ep0->outndx];
  start = clock_systimer();

  do
    {
      /* Send the  SETUP packet */

      chan->pid    = OTGHS_PID_SETUP;
      chan->buffer = (FAR uint8_t *)req;
      chan->buflen = USB_SIZEOF_CTRLREQ;
      chan->xfrd   = 0;

      /* Set up for the wait BEFORE starting the transfer */

      ret = stm32_chan_waitsetup(priv, chan);
      if (ret < 0)
        {
          usbhost_trace1(OTGHS_TRACE1_DEVDISCONN, 0);
          return ret;
        }

      /* Start the transfer */

      stm32_transfer_start(priv, ep0->outndx);

      /* Wait for the transfer to complete */

      ret = stm32_chan_wait(priv, chan);

      /* Return on success and for all failures other than EAGAIN.  EAGAIN
       * means that the device NAKed the SETUP command and that we should
       * try a few more times.
       */

      if (ret != -EAGAIN)
        {
          /* Output some debug information if the transfer failed */

          if (ret < 0)
            {
              usbhost_trace1(OTGHS_TRACE1_TRNSFRFAILED, ret);
            }

          /* Return the result in any event */

          return ret;
        }

     /* Get the elapsed time (in frames) */

     elapsed = clock_systimer() - start;
    }
  while (elapsed < STM32_SETUP_DELAY);

  return -ETIMEDOUT;
}

/*******************************************************************************
 * Name: stm32_ctrl_senddata
 *
 * Description:
 *   Send data in the data phase of an OUT control transfer.  Or send status
 *   in the status phase of an IN control transfer
 *
 *******************************************************************************/

static int stm32_ctrl_senddata(FAR struct stm32_usbhost_s *priv,
                               FAR struct stm32_ctrlinfo_s *ep0,
                               FAR uint8_t *buffer, unsigned int buflen)
{
  FAR struct stm32_chan_s *chan = &priv->chan[ep0->outndx];
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

  chan->pid = chan->outdata1 ? OTGHS_PID_DATA1 : OTGHS_PID_DATA0;

  /* Set up for the wait BEFORE starting the transfer */

  ret = stm32_chan_waitsetup(priv, chan);
  if (ret < 0)
    {
      usbhost_trace1(OTGHS_TRACE1_DEVDISCONN, 0);
      return ret;
    }

  /* Start the transfer */

  stm32_transfer_start(priv, ep0->outndx);

  /* Wait for the transfer to complete and return the result */

  return stm32_chan_wait(priv, chan);
}

/*******************************************************************************
 * Name: stm32_ctrl_recvdata
 *
 * Description:
 *   Receive data in the data phase of an IN control transfer.  Or receive status
 *   in the status phase of an OUT control transfer
 *
 *******************************************************************************/

static int stm32_ctrl_recvdata(FAR struct stm32_usbhost_s *priv,
                               FAR struct stm32_ctrlinfo_s *ep0,
                               FAR uint8_t *buffer, unsigned int buflen)
{
  FAR struct stm32_chan_s *chan = &priv->chan[ep0->inndx];
  int ret;

  /* Save buffer information */

  chan->pid    = OTGHS_PID_DATA1;
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;

  /* Set up for the wait BEFORE starting the transfer */

  ret = stm32_chan_waitsetup(priv, chan);
  if (ret < 0)
    {
      usbhost_trace1(OTGHS_TRACE1_DEVDISCONN, 0);
      return ret;
    }

  /* Start the transfer */

  stm32_transfer_start(priv, ep0->inndx);

  /* Wait for the transfer to complete and return the result */

  return stm32_chan_wait(priv, chan);
}

/*******************************************************************************
 * Name: stm32_in_setup
 *
 * Description:
 *   Initiate an IN transfer on an bulk, interrupt, or isochronous pipe.
 *
 *******************************************************************************/

static int stm32_in_setup(FAR struct stm32_usbhost_s *priv, int chidx)
{
  FAR struct stm32_chan_s *chan;

  /* Set up for the transfer based on the direction and the endpoint type */

  chan = &priv->chan[chidx];
  switch (chan->eptype)
    {
    default:
    case OTGHS_EPTYPE_CTRL: /* Control */
      {
        /* This kind of transfer on control endpoints other than EP0 are not
         * currently supported
         */

        return -ENOSYS;
      }

    case OTGHS_EPTYPE_ISOC: /* Isochronous */
      {
        /* Set up the IN data PID */

        usbhost_vtrace2(OTGHS_VTRACE2_ISOCIN, chidx, chan->buflen);
        chan->pid = OTGHS_PID_DATA0;
      }
      break;

    case OTGHS_EPTYPE_BULK: /* Bulk */
      {
        /* Setup the IN data PID */

        usbhost_vtrace2(OTGHS_VTRACE2_BULKIN, chidx, chan->buflen);
        chan->pid = chan->indata1 ? OTGHS_PID_DATA1 : OTGHS_PID_DATA0;
      }
      break;

    case OTGHS_EPTYPE_INTR: /* Interrupt */
      {
        /* Setup the IN data PID */

        usbhost_vtrace2(OTGHS_VTRACE2_INTRIN, chidx, chan->buflen);
        chan->pid = chan->indata1 ? OTGHS_PID_DATA1 : OTGHS_PID_DATA0;
      }
      break;
    }

  /* Start the transfer */

  stm32_transfer_start(priv, chidx);
  return OK;
}

/*******************************************************************************
 * Name: stm32_in_transfer
 *
 * Description:
 *   Transfer 'buflen' bytes into 'buffer' from an IN channel.
 *
 *******************************************************************************/

static ssize_t stm32_in_transfer(FAR struct stm32_usbhost_s *priv, int chidx,
                                 FAR uint8_t *buffer, size_t buflen)
{
  FAR struct stm32_chan_s *chan;
  uint32_t start;
  uint32_t elapsed;
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

      ret = stm32_chan_waitsetup(priv, chan);
      if (ret < 0)
        {
          usbhost_trace1(OTGHS_TRACE1_DEVDISCONN, 0);
          return (ssize_t)ret;
        }

      /* Set up for the transfer based on the direction and the endpoint type */

      ret = stm32_in_setup(priv, chidx);
      if (ret < 0)
        {
          udbg("ERROR: stm32_in_setup failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Wait for the transfer to complete and get the result */

      ret = stm32_chan_wait(priv, chan);

      /* EAGAIN indicates that the device NAKed the transfer and we need
       * do try again.  Anything else (success or other errors) will
       * cause use to return
       */

      if (ret < 0)
        {
          usbhost_trace1(OTGHS_TRACE1_TRNSFRFAILED,ret);

          /* Check for a special case:  If (1) the transfer was NAKed and (2)
           * no Tx FIFO empty or Rx FIFO not-empty event occurred, then we
           * should be able to just flush the Rx and Tx FIFOs and try again.
           * We can detect this latter case because the then the transfer
           * buffer pointer and buffer size will be unaltered.
           */

          elapsed = clock_systimer() - start;
          if (ret != -EAGAIN ||                  /* Not a NAK condition OR */
              elapsed >= STM32_DATANAK_DELAY ||  /* Timeout has elapsed OR */
              chan->xfrd > 0)                    /* Data has been partially transferred */
            {
              /* Break out and return the error */

              udbg("ERROR: stm32_chan_wait failed: %d\n", ret);
              return (ssize_t)ret;
            }
        }
    }

  return (ssize_t)chan->xfrd;
}

/*******************************************************************************
 * Name: stm32_in_next
 *
 * Description:
 *   Initiate the next of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is always called from an interrupt handler
 *
 *******************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void stm32_in_next(FAR struct stm32_usbhost_s *priv,
                          FAR struct stm32_chan_s *chan)
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

      ret = stm32_in_setup(priv, chan->chidx);
      if (ret >= 0)
        {
          return;
        }

      udbg("ERROR: stm32_in_setup failed: %d\n", ret);
      result = ret;
    }

  /* The transfer is complete, with or without an error */

  uvdbg("Transfer complete:  %d\n", result);

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

/*******************************************************************************
 * Name: stm32_in_asynch
 *
 * Description:
 *   Initiate the first of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is never called from an interrupt handler
 *
 *******************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int stm32_in_asynch(FAR struct stm32_usbhost_s *priv, int chidx,
                           FAR uint8_t *buffer, size_t buflen,
                           usbhost_asynch_t callback, FAR void *arg)
{
  FAR struct stm32_chan_s *chan;
  int ret;

  /* Set up for the transfer data and callback BEFORE starting the first transfer */

  chan         = &priv->chan[chidx];
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;

  ret = stm32_chan_asynchsetup(priv, chan, callback, arg);
  if (ret < 0)
    {
      udbg("ERROR: stm32_chan_asynchsetup failed: %d\n", ret);
      return ret;
    }

  /* Set up for the transfer based on the direction and the endpoint type */

  ret = stm32_in_setup(priv, chidx);
  if (ret < 0)
    {
      udbg("ERROR: stm32_in_setup failed: %d\n", ret);
    }

  /* And return with the transfer pending */

  return ret;
}
#endif

/*******************************************************************************
 * Name: stm32_out_setup
 *
 * Description:
 *   Initiate an OUT transfer on an bulk, interrupt, or isochronous pipe.
 *
 *******************************************************************************/

static int stm32_out_setup(FAR struct stm32_usbhost_s *priv, int chidx)
{
  FAR struct stm32_chan_s *chan;

  /* Set up for the transfer based on the direction and the endpoint type */

  chan = &priv->chan[chidx];
  switch (chan->eptype)
    {
    default:
    case OTGHS_EPTYPE_CTRL: /* Control */
      {
        /* This kind of transfer on control endpoints other than EP0 are not
         * currently supported
         */

        return -ENOSYS;
      }

    case OTGHS_EPTYPE_ISOC: /* Isochronous */
      {
        /* Set up the OUT data PID */

        usbhost_vtrace2(OTGHS_VTRACE2_ISOCOUT, chidx, chan->buflen);
        chan->pid = OTGHS_PID_DATA0;
      }
      break;

    case OTGHS_EPTYPE_BULK: /* Bulk */
      {
        /* Setup the OUT data PID */

        usbhost_vtrace2(OTGHS_VTRACE2_BULKOUT, chidx, chan->buflen);
        chan->pid = chan->outdata1 ? OTGHS_PID_DATA1 : OTGHS_PID_DATA0;
      }
      break;

    case OTGHS_EPTYPE_INTR: /* Interrupt */
      {
        /* Setup the OUT data PID */

        usbhost_vtrace2(OTGHS_VTRACE2_INTROUT, chidx, chan->buflen);
        chan->pid = chan->outdata1 ? OTGHS_PID_DATA1 : OTGHS_PID_DATA0;

        /* Toggle the OUT data PID for the next transfer */

        chan->outdata1 ^= true;
      }
      break;
    }

  /* Start the transfer */

  stm32_transfer_start(priv, chidx);
  return OK;
}

/*******************************************************************************
 * Name: stm32_out_transfer
 *
 * Description:
 *   Transfer the 'buflen' bytes in 'buffer' through an OUT channel.
 *
 *******************************************************************************/

static ssize_t stm32_out_transfer(FAR struct stm32_usbhost_s *priv, int chidx,
                                  FAR uint8_t *buffer, size_t buflen)
{
  FAR struct stm32_chan_s *chan;
  uint32_t start;
  uint32_t elapsed;
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

      ret = stm32_chan_waitsetup(priv, chan);
      if (ret < 0)
        {
          usbhost_trace1(OTGHS_TRACE1_DEVDISCONN,0);
          return (ssize_t)ret;
        }

      /* Set up for the transfer based on the direction and the endpoint type */

      ret = stm32_out_setup(priv, chidx);
      if (ret < 0)
        {
          udbg("ERROR: stm32_out_setup failed: %d\n", ret);
          return (ssize_t)ret;
        }

     /* Wait for the transfer to complete and get the result */

      ret = stm32_chan_wait(priv, chan);

      /* Handle transfer failures */

      if (ret < 0)
        {
          usbhost_trace1(OTGHS_TRACE1_TRNSFRFAILED,ret);

          /* Check for a special case:  If (1) the transfer was NAKed and (2)
           * no Tx FIFO empty or Rx FIFO not-empty event occurred, then we
           * should be able to just flush the Rx and Tx FIFOs and try again.
           * We can detect this latter case because the then the transfer
           * buffer pointer and buffer size will be unaltered.
           */

          elapsed = clock_systimer() - start;
          if (ret != -EAGAIN ||                  /* Not a NAK condition OR */
              elapsed >= STM32_DATANAK_DELAY ||  /* Timeout has elapsed OR */
              chan->xfrd > 0)                    /* Data has been partially transferred */
            {
              /* Break out and return the error */

              udbg("ERROR: stm32_chan_wait failed: %d\n", ret);
              return (ssize_t)ret;
            }

          /* Is this flush really necessary? What does the hardware do with the
           * data in the FIFO when the NAK occurs?  Does it discard it?
           */

          stm32_flush_txfifos(OTGHS_GRSTCTL_TXFNUM_HALL);

          /* Get the device a little time to catch up.  Then retry the transfer
           * using the same buffer pointer and length.
           */

          usleep(20*1000);
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

/*******************************************************************************
 * Name: stm32_out_next
 *
 * Description:
 *   Initiate the next of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is always called from an interrupt handler
 *
 *******************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void stm32_out_next(FAR struct stm32_usbhost_s *priv,
                           FAR struct stm32_chan_s *chan)
{
  usbhost_asynch_t callback;
  FAR void *arg;
  ssize_t nbytes;
  int result;
  int ret;

  /* Is the full transfer complete? Did the last chunk transfer complete OK?*/

  result = -(int)chan->result;
  if (chan->xfrd < chan->buflen && result == OK)
    {
      /* Yes.. Set up for the next transfer based on the direction and the
       * endpoint type
       */

      ret = stm32_out_setup(priv, chan->chidx);
      if (ret >= 0)
        {
          return;
        }

      udbg("ERROR: stm32_out_setup failed: %d\n", ret);
      result = ret;
    }

  /* The transfer is complete, with or without an error */

  uvdbg("Transfer complete:  %d\n", result);

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

/*******************************************************************************
 * Name: stm32_out_asynch
 *
 * Description:
 *   Initiate the first of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is never called from an interrupt handler
 *
 *******************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int stm32_out_asynch(FAR struct stm32_usbhost_s *priv, int chidx,
                            FAR uint8_t *buffer, size_t buflen,
                            usbhost_asynch_t callback, FAR void *arg)
{
  FAR struct stm32_chan_s *chan;
  int ret;

  /* Set up for the transfer data and callback BEFORE starting the first transfer */

  chan         = &priv->chan[chidx];
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;

  ret = stm32_chan_asynchsetup(priv, chan, callback, arg);
  if (ret < 0)
    {
      udbg("ERROR: stm32_chan_asynchsetup failed: %d\n", ret);
      return ret;
    }

  /* Set up for the transfer based on the direction and the endpoint type */

  ret = stm32_out_setup(priv, chidx);
  if (ret < 0)
    {
      udbg("ERROR: stm32_out_setup failed: %d\n", ret);
    }

  /* And return with the transfer pending */

  return ret;
}
#endif

/*******************************************************************************
 * Name: stm32_gint_wrpacket
 *
 * Description:
 *   Transfer the 'buflen' bytes in 'buffer' to the Tx FIFO associated with
 *   'chidx' (non-DMA).
 *
 *******************************************************************************/

static void stm32_gint_wrpacket(FAR struct stm32_usbhost_s *priv,
                                FAR uint8_t *buffer, int chidx, int buflen)
{
  FAR uint32_t *src;
  uint32_t fifo;
  int buflen32;

  stm32_pktdump("Sending", buffer, buflen);

  /* Get the number of 32-byte words associated with this byte size */

  buflen32 = (buflen + 3) >> 2;

  /* Get the address of the Tx FIFO associated with this channel */

  fifo = STM32_OTGHS_DFIFO_HCH(chidx);

  /* Transfer all of the data into the Tx FIFO */

  src = (FAR uint32_t *)buffer;
  for (; buflen32 > 0; buflen32--)
    {
      uint32_t data = *src++;
      stm32_putreg(fifo, data);
    }

  /* Increment the count of bytes "in-flight" in the Tx FIFO */

  priv->chan[chidx].inflight += buflen;
}

/*******************************************************************************
 * Name: stm32_gint_hcinisr
 *
 * Description:
 *   USB OTG HS host IN channels interrupt handler
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
 *******************************************************************************/

static inline void stm32_gint_hcinisr(FAR struct stm32_usbhost_s *priv,
                                      int chidx)
{
  FAR struct stm32_chan_s *chan = &priv->chan[chidx];
  uint32_t regval;
  uint32_t pending;

  /* Read the HCINT register to get the pending HC interrupts.  Read the
   * HCINTMSK register to get the set of enabled HC interrupts.
   */

  pending = stm32_getreg(STM32_OTGHS_HCINT(chidx));
  regval  = stm32_getreg(STM32_OTGHS_HCINTMSK(chidx));

  /* AND the two to get the set of enabled, pending HC interrupts */

  pending &= regval;
  ullvdbg("HCINTMSK%d: %08x pending: %08x\n", chidx, regval, pending);

  /* Check for a pending ACK response received/transmitted (ACK) interrupt */

  if ((pending & OTGHS_HCINT_ACK) != 0)
    {
      /* Clear the pending the ACK response received/transmitted (ACK) interrupt */

      stm32_putreg(STM32_OTGHS_HCINT(chidx), OTGHS_HCINT_ACK);
    }

  /* Check for a pending STALL response receive (STALL) interrupt */

  else if ((pending & OTGHS_HCINT_STALL) != 0)
    {
      /* Clear the NAK and STALL Conditions. */

      stm32_putreg(STM32_OTGHS_HCINT(chidx), (OTGHS_HCINT_NAK | OTGHS_HCINT_STALL));

      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      stm32_chan_halt(priv, chidx, CHREASON_STALL);

      /* When there is a STALL, clear any pending NAK so that it is not
       * processed below.
       */

      pending &= ~OTGHS_HCINT_NAK;
    }

  /* Check for a pending Data Toggle ERRor (DTERR) interrupt */

  else if ((pending & OTGHS_HCINT_DTERR) != 0)
    {
      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      stm32_chan_halt(priv, chidx, CHREASON_DTERR);

      /* Clear the NAK and data toggle error conditions */

      stm32_putreg(STM32_OTGHS_HCINT(chidx), (OTGHS_HCINT_NAK | OTGHS_HCINT_DTERR));
    }

  /* Check for a pending FRaMe OverRun (FRMOR) interrupt */

  if ((pending & OTGHS_HCINT_FRMOR) != 0)
    {
      /* Halt the channel -- the CHH interrupt is expected next */

      stm32_chan_halt(priv, chidx, CHREASON_FRMOR);

      /* Clear the FRaMe OverRun (FRMOR) condition */

      stm32_putreg(STM32_OTGHS_HCINT(chidx), OTGHS_HCINT_FRMOR);
    }

  /* Check for a pending TransFeR Completed (XFRC) interrupt */

  else if ((pending & OTGHS_HCINT_XFRC) != 0)
    {
      /* Clear the TransFeR Completed (XFRC) condition */

      stm32_putreg(STM32_OTGHS_HCINT(chidx), OTGHS_HCINT_XFRC);

      /* Then handle the transfer completion event based on the endpoint type */

      if (chan->eptype == OTGHS_EPTYPE_CTRL || chan->eptype == OTGHS_EPTYPE_BULK)
        {
          /* Halt the channel -- the CHH interrupt is expected next */

          stm32_chan_halt(priv, chidx, CHREASON_XFRC);

          /* Clear any pending NAK condition.  The 'indata1' data toggle
           * should have been appropriately updated by the RxFIFO
           * logic as each packet was received.
           */

          stm32_putreg(STM32_OTGHS_HCINT(chidx), OTGHS_HCINT_NAK);
        }
      else if (chan->eptype == OTGHS_EPTYPE_INTR)
        {
          /* Force the next transfer on an ODD frame */

          regval = stm32_getreg(STM32_OTGHS_HCCHAR(chidx));
          regval |= OTGHS_HCCHAR_ODDFRM;
          stm32_putreg(STM32_OTGHS_HCCHAR(chidx), regval);

          /* Set the request done state */

          chan->result = OK;
        }
    }

  /* Check for a pending CHannel Halted (CHH) interrupt */

  else if ((pending & OTGHS_HCINT_CHH) != 0)
    {
      /* Mask the CHannel Halted (CHH) interrupt */

      regval  = stm32_getreg(STM32_OTGHS_HCINTMSK(chidx));
      regval &= ~OTGHS_HCINT_CHH;
      stm32_putreg(STM32_OTGHS_HCINTMSK(chidx), regval);

      /* Update the request state based on the host state machine state */

      if (chan->chreason == CHREASON_XFRC)
        {
          /* Set the request done reult */

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

          regval = stm32_getreg(STM32_OTGHS_HCCHAR(chidx));
          if ((regval & OTGHS_HCCHAR_EPTYP_MASK) == OTGHS_HCCHAR_EPTYP_INTR)
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

      stm32_putreg(STM32_OTGHS_HCINT(chidx), OTGHS_HCINT_CHH);
    }

  /* Check for a pending Transaction ERror (TXERR) interrupt */

  else if ((pending & OTGHS_HCINT_TXERR) != 0)
    {
      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      stm32_chan_halt(priv, chidx, CHREASON_TXERR);

      /* Clear the Transaction ERror (TXERR) condition */

      stm32_putreg(STM32_OTGHS_HCINT(chidx), OTGHS_HCINT_TXERR);
    }

  /* Check for a pending NAK response received (NAK) interrupt */

  else if ((pending & OTGHS_HCINT_NAK) != 0)
    {
      /* For a BULK transfer, the hardware is capable of retrying
       * automatically on a NAK.  However, this is not always
       * what we need to do.  So we always halt the transfer and
       * return control to high level logic in the even of a NAK.
       */

#if 1
      /* Halt the interrupt channel */

      if (chan->eptype == OTGHS_EPTYPE_INTR)
        {
          /* Halt the channel -- the CHH interrupt is expected next */

          stm32_chan_halt(priv, chidx, CHREASON_NAK);
        }

      /* Re-activate CTRL and BULK channels */

      else if (chan->eptype == OTGHS_EPTYPE_CTRL ||
               chan->eptype == OTGHS_EPTYPE_BULK)
        {
          /* Re-activate the channel by clearing CHDIS and assuring that
           * CHENA is set
           */

          regval  = stm32_getreg(STM32_OTGHS_HCCHAR(chidx));
          regval |= OTGHS_HCCHAR_CHENA;
          regval &= ~OTGHS_HCCHAR_CHDIS;
          stm32_putreg(STM32_OTGHS_HCCHAR(chidx), regval);
        }
#else
      /* Halt all transfers on the NAK -- the CHH interrupt is expected next */

      stm32_chan_halt(priv, chidx, CHREASON_NAK);
#endif

      /* Clear the NAK condition */

      stm32_putreg(STM32_OTGHS_HCINT(chidx), OTGHS_HCINT_NAK);
    }

  /* Check for a transfer complete event */

  stm32_chan_wakeup(priv, chan);
}

/*******************************************************************************
 * Name: stm32_gint_hcoutisr
 *
 * Description:
 *   USB OTG HS host OUT channels interrupt handler
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
 *******************************************************************************/

static inline void stm32_gint_hcoutisr(FAR struct stm32_usbhost_s *priv,
                                       int chidx)
{
  FAR struct stm32_chan_s *chan = &priv->chan[chidx];
  uint32_t regval;
  uint32_t pending;

  /* Read the HCINT register to get the pending HC interrupts.  Read the
   * HCINTMSK register to get the set of enabled HC interrupts.
   */

  pending = stm32_getreg(STM32_OTGHS_HCINT(chidx));
  regval  = stm32_getreg(STM32_OTGHS_HCINTMSK(chidx));

  /* AND the two to get the set of enabled, pending HC interrupts */

  pending &= regval;
  ullvdbg("HCINTMSK%d: %08x pending: %08x\n", chidx, regval, pending);

  /* Check for a pending ACK response received/transmitted (ACK) interrupt */

  if ((pending & OTGHS_HCINT_ACK) != 0)
    {
      /* Clear the pending the ACK response received/transmitted (ACK) interrupt */

      stm32_putreg(STM32_OTGHS_HCINT(chidx), OTGHS_HCINT_ACK);
    }

  /* Check for a pending FRaMe OverRun (FRMOR) interrupt */

  else if ((pending & OTGHS_HCINT_FRMOR) != 0)
    {
      /* Halt the channel (probably not necessary for FRMOR) */

      stm32_chan_halt(priv, chidx, CHREASON_FRMOR);

      /* Clear the pending the FRaMe OverRun (FRMOR) interrupt */

      stm32_putreg(STM32_OTGHS_HCINT(chidx), OTGHS_HCINT_FRMOR);
    }

  /* Check for a pending TransFeR Completed (XFRC) interrupt */

  else if ((pending & OTGHS_HCINT_XFRC) != 0)
    {
      /* Decrement the number of bytes remaining by the number of
       * bytes that were "in-flight".
       */

      priv->chan[chidx].buffer  += priv->chan[chidx].inflight;
      priv->chan[chidx].xfrd    += priv->chan[chidx].inflight;
      priv->chan[chidx].inflight = 0;

      /* Halt the channel -- the CHH interrupt is expected next */

      stm32_chan_halt(priv, chidx, CHREASON_XFRC);

      /* Clear the pending the TransFeR Completed (XFRC) interrupt */

      stm32_putreg(STM32_OTGHS_HCINT(chidx), OTGHS_HCINT_XFRC);
    }

  /* Check for a pending STALL response receive (STALL) interrupt */

  else if ((pending & OTGHS_HCINT_STALL) != 0)
    {
      /* Clear the pending the STALL response receiv (STALL) interrupt */

      stm32_putreg(STM32_OTGHS_HCINT(chidx), OTGHS_HCINT_STALL);

      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      stm32_chan_halt(priv, chidx, CHREASON_STALL);
    }

  /* Check for a pending NAK response received (NAK) interrupt */

  else if ((pending & OTGHS_HCINT_NAK) != 0)
    {
      /* Halt the channel  -- the CHH interrupt is expected next */

      stm32_chan_halt(priv, chidx, CHREASON_NAK);

      /* Clear the pending the NAK response received (NAK) interrupt */

      stm32_putreg(STM32_OTGHS_HCINT(chidx), OTGHS_HCINT_NAK);
    }

  /* Check for a pending Transaction ERror (TXERR) interrupt */

  else if ((pending & OTGHS_HCINT_TXERR) != 0)
    {
      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      stm32_chan_halt(priv, chidx, CHREASON_TXERR);

      /* Clear the pending the Transaction ERror (TXERR) interrupt */

      stm32_putreg(STM32_OTGHS_HCINT(chidx), OTGHS_HCINT_TXERR);
    }

  /* Check for a NYET interrupt */

#if 0 /* NYET is a reserved bit in the HCINT register */
  else if ((pending & OTGHS_HCINT_NYET) != 0)
    {
      /* Halt the channel */

      stm32_chan_halt(priv, chidx, CHREASON_NYET);

      /* Clear the pending the NYET interrupt */

      stm32_putreg(STM32_OTGHS_HCINT(chidx), OTGHS_HCINT_NYET);
    }
#endif

  /* Check for a pending Data Toggle ERRor (DTERR) interrupt */

  else if (pending & OTGHS_HCINT_DTERR)
    {
      /* Halt the channel when a STALL, TXERR, BBERR or DTERR interrupt is
       * received on the channel.
       */

      stm32_chan_halt(priv, chidx, CHREASON_DTERR);

      /* Clear the pending the Data Toggle ERRor (DTERR) and NAK interrupts */

      stm32_putreg(STM32_OTGHS_HCINT(chidx), (OTGHS_HCINT_DTERR | OTGHS_HCINT_NAK));
    }

  /* Check for a pending CHannel Halted (CHH) interrupt */

  else if ((pending & OTGHS_HCINT_CHH) != 0)
    {
      /* Mask the CHannel Halted (CHH) interrupt */

      regval  = stm32_getreg(STM32_OTGHS_HCINTMSK(chidx));
      regval &= ~OTGHS_HCINT_CHH;
      stm32_putreg(STM32_OTGHS_HCINTMSK(chidx), regval);

      if (chan->chreason == CHREASON_XFRC)
        {
          /* Set the request done result */

          chan->result = OK;

          /* Read the HCCHAR register to get the HCCHAR register to get
           * the endpoint type.
           */

          regval = stm32_getreg(STM32_OTGHS_HCCHAR(chidx));

          /* Is it a bulk endpoint?  Were an odd number of packets
           * transferred?
           */

          if ((regval & OTGHS_HCCHAR_EPTYP_MASK) == OTGHS_HCCHAR_EPTYP_BULK &&
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

      stm32_putreg(STM32_OTGHS_HCINT(chidx), OTGHS_HCINT_CHH);
    }

  /* Check for a transfer complete event */

  stm32_chan_wakeup(priv, chan);
}

/*******************************************************************************
 * Name: stm32_gint_connected
 *
 * Description:
 *   Handle a connection event.
 *
 *******************************************************************************/

static void stm32_gint_connected(FAR struct stm32_usbhost_s *priv)
{
  /* We we previously disconnected? */

  if (!priv->connected)
    {
      /* Yes.. then now we are connected */

      usbhost_vtrace1(OTGHS_VTRACE1_CONNECTED,0);
      priv->connected = true;
      priv->change    = true;
      DEBUGASSERT(priv->smstate == SMSTATE_DETACHED);

      /* Notify any waiters */

      priv->smstate = SMSTATE_ATTACHED;
      if (priv->pscwait)
        {
          stm32_givesem(&priv->pscsem);
          priv->pscwait = false;
        }
    }
}

/*******************************************************************************
 * Name: stm32_gint_disconnected
 *
 * Description:
 *   Handle a disconnection event.
 *
 *******************************************************************************/

static void stm32_gint_disconnected(FAR struct stm32_usbhost_s *priv)
{
  /* Were we previously connected? */

  if (priv->connected)
    {
      /* Yes.. then we no longer connected */

      usbhost_vtrace1(OTGHS_VTRACE1_DISCONNECTED,0);

      /* Are we bound to a class driver? */

      if ( priv->rhport.hport.devclass)
        {
          /* Yes.. Disconnect the class driver */

          CLASS_DISCONNECTED( priv->rhport.hport.devclass);
           priv->rhport.hport.devclass = NULL;
        }

      /* Re-Initialize Host for new Enumeration */

      priv->smstate   = SMSTATE_DETACHED;
      priv->connected = false;
      priv->change    = true;
      stm32_chan_freeall(priv);

      priv->rhport.hport.speed = USB_SPEED_FULL;

    /* Notify any waiters that there is a change in the connection state */

     if (priv->pscwait)
        {
          stm32_givesem(&priv->pscsem);
          priv->pscwait = false;
        }
    }
}

/*******************************************************************************
 * Name: stm32_gint_sofisr
 *
 * Description:
 *   USB OTG HS start-of-frame interrupt handler
 *
 *******************************************************************************/

#ifdef CONFIG_STM32_OTGHS_SOFINTR
static inline void stm32_gint_sofisr(FAR struct stm32_usbhost_s *priv)
{
  /* Handle SOF interrupt */
#warning "Do what?"

  /* Clear pending SOF interrupt */

  stm32_putreg(STM32_OTGHS_GINTSTS, OTGHS_GINT_SOF);
}
#endif

/*******************************************************************************
 * Name: stm32_gint_rxflvlisr
 *
 * Description:
 *   USB OTG HS RxFIFO non-empty interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_rxflvlisr(FAR struct stm32_usbhost_s *priv)
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

  intmsk  = stm32_getreg(STM32_OTGHS_GINTMSK);
  intmsk &= ~OTGHS_GINT_RXFLVL;
  stm32_putreg(STM32_OTGHS_GINTMSK, intmsk);

  /* Read and pop the next status from the Rx FIFO */

  grxsts = stm32_getreg(STM32_OTGHS_GRXSTSP);
  ullvdbg("GRXSTS: %08x\n", grxsts);

  /* Isolate the channel number/index in the status word */

  chidx = (grxsts & OTGHS_GRXSTSH_CHNUM_MASK) >> OTGHS_GRXSTSH_CHNUM_SHIFT;

  /* Get the host channel characteristics register (HCCHAR) for this channel */

  hcchar = stm32_getreg(STM32_OTGHS_HCCHAR(chidx));

  /* Then process the interrupt according to the packet status */

  switch (grxsts & OTGHS_GRXSTSH_PKTSTS_MASK)
    {
    case OTGHS_GRXSTSH_PKTSTS_INRECVD: /* IN data packet received */
      {
        /* Read the data into the host buffer. */

        bcnt = (grxsts & OTGHS_GRXSTSH_BCNT_MASK) >> OTGHS_GRXSTSH_BCNT_SHIFT;
        if (bcnt > 0 && priv->chan[chidx].buffer != NULL)
          {
            /* Transfer the packet from the Rx FIFO into the user buffer */

            dest   = (FAR uint32_t *)priv->chan[chidx].buffer;
            fifo   = STM32_OTGHS_DFIFO_HCH(0);
            bcnt32 = (bcnt + 3) >> 2;

            for (i = 0; i < bcnt32; i++)
              {
                *dest++ = stm32_getreg(fifo);
              }

            stm32_pktdump("Received", priv->chan[chidx].buffer, bcnt);

            /* Toggle the IN data pid (Used by Bulk and INTR only) */

            priv->chan[chidx].indata1 ^= true;

            /* Manage multiple packet transfers */

            priv->chan[chidx].buffer += bcnt;
            priv->chan[chidx].xfrd   += bcnt;

            /* Check if more packets are expected */

            hctsiz = stm32_getreg(STM32_OTGHS_HCTSIZ(chidx));
            if ((hctsiz & OTGHS_HCTSIZ_PKTCNT_MASK) != 0)
              {
                /* Re-activate the channel when more packets are expected */

                hcchar |= OTGHS_HCCHAR_CHENA;
                hcchar &= ~OTGHS_HCCHAR_CHDIS;
                stm32_putreg(STM32_OTGHS_HCCHAR(chidx), hcchar);
              }
          }
      }
      break;

    case OTGHS_GRXSTSH_PKTSTS_INDONE:  /* IN transfer completed */
    case OTGHS_GRXSTSH_PKTSTS_DTOGERR: /* Data toggle error */
    case OTGHS_GRXSTSH_PKTSTS_HALTED:  /* Channel halted */
    default:
      break;
    }

  /* Re-enable the RxFIFO non-empty interrupt */

  intmsk |= OTGHS_GINT_RXFLVL;
  stm32_putreg(STM32_OTGHS_GINTMSK, intmsk);
}

/*******************************************************************************
 * Name: stm32_gint_nptxfeisr
 *
 * Description:
 *   USB OTG HS non-periodic TxFIFO empty interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_nptxfeisr(FAR struct stm32_usbhost_s *priv)
{
  FAR struct stm32_chan_s *chan;
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

      stm32_modifyreg(STM32_OTGHS_GINTMSK, OTGHS_GINT_NPTXFE, 0);
      return;
    }

  /* Read the status from the top of the non-periodic TxFIFO */

  regval = stm32_getreg(STM32_OTGHS_HNPTXSTS);

  /* Extract the number of bytes available in the non-periodic Tx FIFO. */

  avail = ((regval & OTGHS_HNPTXSTS_NPTXFSAV_MASK) >> OTGHS_HNPTXSTS_NPTXFSAV_SHIFT) << 2;

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
      stm32_modifyreg(STM32_OTGHS_GINTMSK, OTGHS_GINT_NPTXFE, 0);
    }

  /* Write the next group of packets into the Tx FIFO */

  ullvdbg("HNPTXSTS: %08x chidx: %d avail: %d buflen: %d xfrd: %dwrsize: %d\n",
           regval, chidx, avail, chan->buflen, chan->xfrd, wrsize);

  stm32_gint_wrpacket(priv, chan->buffer, chidx, wrsize);
}

/*******************************************************************************
 * Name: stm32_gint_ptxfeisr
 *
 * Description:
 *   USB OTG HS periodic TxFIFO empty interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_ptxfeisr(FAR struct stm32_usbhost_s *priv)
{
  FAR struct stm32_chan_s *chan;
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

      stm32_modifyreg(STM32_OTGHS_GINTMSK, OTGHS_GINT_PTXFE, 0);
      return;
    }

  /* Read the status from the top of the periodic TxFIFO */

  regval = stm32_getreg(STM32_OTGHS_HPTXSTS);

  /* Extract the number of bytes available in the periodic Tx FIFO. */

  avail = ((regval & OTGHS_HPTXSTS_PTXFSAVL_MASK) >> OTGHS_HPTXSTS_PTXFSAVL_SHIFT) << 2;

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
   * We now need to disable further PTXFE interrupts.
   */

  else
    {
      stm32_modifyreg(STM32_OTGHS_GINTMSK, OTGHS_GINT_PTXFE, 0);
    }

  /* Write the next group of packets into the Tx FIFO */

  ullvdbg("HPTXSTS: %08x chidx: %d avail: %d buflen: %d xfrd: %d wrsize: %d\n",
           regval, chidx, avail, chan->buflen, chan->xfrd, wrsize);

  stm32_gint_wrpacket(priv, chan->buffer, chidx, wrsize);
}

/*******************************************************************************
 * Name: stm32_gint_hcisr
 *
 * Description:
 *   USB OTG HS host channels interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_hcisr(FAR struct stm32_usbhost_s *priv)
{
  uint32_t haint;
  uint32_t hcchar;
  int i = 0;

  /* Read the Host all channels interrupt register and test each bit in the
   * register. Each bit i, i=0...(STM32_NHOST_CHANNELS-1), corresponds to
   * a pending interrupt on channel i.
   */

  haint = stm32_getreg(STM32_OTGHS_HAINT);
  for (i = 0; i < STM32_NHOST_CHANNELS; i++)
    {
      /* Is an interrupt pending on this channel? */

      if ((haint & OTGHS_HAINT(i)) != 0)
        {
          /* Yes... read the HCCHAR register to get the direction bit */

          hcchar = stm32_getreg(STM32_OTGHS_HCCHAR(i));

          /* Was this an interrupt on an IN or an OUT channel? */

          if ((hcchar & OTGHS_HCCHAR_EPDIR) != 0)
            {
              /* Handle the HC IN channel interrupt */

              stm32_gint_hcinisr(priv, i);
            }
          else
            {
              /* Handle the HC OUT channel interrupt */

              stm32_gint_hcoutisr(priv, i);
            }
        }
    }
}

/*******************************************************************************
 * Name: stm32_gint_hprtisr
 *
 * Description:
 *   USB OTG HS host port interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_hprtisr(FAR struct stm32_usbhost_s *priv)
{
  uint32_t hprt;
  uint32_t newhprt;
  uint32_t hcfg;

  usbhost_vtrace1(OTGHS_VTRACE1_GINT_HPRT, 0);
  /* Read the port status and control register (HPRT) */

  hprt = stm32_getreg(STM32_OTGHS_HPRT);

  /* Setup to clear the interrupt bits in GINTSTS by setting the corresponding
   * bits in the HPRT.  The HCINT interrupt bit is cleared when the appropriate
   * status bits in the HPRT register are cleared.
   */

  newhprt = hprt & ~(OTGHS_HPRT_PENA    | OTGHS_HPRT_PCDET  |
                     OTGHS_HPRT_PENCHNG | OTGHS_HPRT_POCCHNG);

  /* Check for Port Overcurrent CHaNGe (POCCHNG) */

  if ((hprt & OTGHS_HPRT_POCCHNG) != 0)
    {
      /* Set up to clear the POCCHNG status in the new HPRT contents. */

      usbhost_vtrace1(OTGHS_VTRACE1_GINT_HPRT_POCCHNG, 0);
      newhprt |= OTGHS_HPRT_POCCHNG;
    }

  /* Check for Port Connect DETected (PCDET).  The core sets this bit when a
   * device connection is detected.
   */

  if ((hprt & OTGHS_HPRT_PCDET) != 0)
    {
      /* Set up to clear the PCDET status in the new HPRT contents. Then
       * process the new connection event.
       */

      usbhost_vtrace1(OTGHS_VTRACE1_GINT_HPRT_PCDET, 0);
      newhprt |= OTGHS_HPRT_PCDET;
      stm32_portreset(priv);
      stm32_gint_connected(priv);
    }

  /* Check for Port Enable CHaNGed (PENCHNG) */

  if ((hprt & OTGHS_HPRT_PENCHNG) != 0)
    {
      /* Set up to clear the PENCHNG status in the new HPRT contents. */

      usbhost_vtrace1(OTGHS_VTRACE1_GINT_HPRT_PENCHNG, 0);
      newhprt |= OTGHS_HPRT_PENCHNG;

      /* Was the port enabled? */

      if ((hprt & OTGHS_HPRT_PENA) != 0)
        {
          /* Yes.. handle the new connection event */

          stm32_gint_connected(priv);

          /* Check the Host ConFiGuration register (HCFG) */

          hcfg = stm32_getreg(STM32_OTGHS_HCFG);

          /* Is this a low speed or full speed connection (OTG HS does not
           * support high speed)
           */

          if ((hprt & OTGHS_HPRT_PSPD_MASK) == OTGHS_HPRT_PSPD_LS)
            {
              /* Set the Host Frame Interval Register for the 6KHz speed */

              usbhost_vtrace1(OTGHS_VTRACE1_GINT_HPRT_LSDEV, 0);
              stm32_putreg(STM32_OTGHS_HFIR, 6000);

              /* Are we switching from HS to LS? */

              if ((hcfg & OTGHS_HCFG_FSLSPCS_MASK) != OTGHS_HCFG_FSLSPCS_LS6MHz)
                {
                  usbhost_vtrace1(OTGHS_VTRACE1_GINT_HPRT_FSLSSW, 0);

                  /* Yes... configure for LS */

                  hcfg &= ~OTGHS_HCFG_FSLSPCS_MASK;
                  hcfg |= OTGHS_HCFG_FSLSPCS_LS6MHz;
                  stm32_putreg(STM32_OTGHS_HCFG, hcfg);

                  /* And reset the port */

                  stm32_portreset(priv);
                }
            }
          else /* if ((hprt & OTGHS_HPRT_PSPD_MASK) == OTGHS_HPRT_PSPD_HS) */
            {

              usbhost_vtrace1(OTGHS_VTRACE1_GINT_HPRT_FSDEV, 0);
              stm32_putreg(STM32_OTGHS_HFIR, 48000);

              /* Are we switching from LS to HS? */

              if ((hcfg & OTGHS_HCFG_FSLSPCS_MASK) != OTGHS_HCFG_FSLSPCS_FS48MHz)
                {

                  usbhost_vtrace1(OTGHS_VTRACE1_GINT_HPRT_LSFSSW, 0);
                  /* Yes... configure for HS */

                  hcfg &= ~OTGHS_HCFG_FSLSPCS_MASK;
                  hcfg |= OTGHS_HCFG_FSLSPCS_FS48MHz;
                  stm32_putreg(STM32_OTGHS_HCFG, hcfg);

                  /* And reset the port */

                  stm32_portreset(priv);
                }
            }
        }
    }

  /* Clear port interrupts by setting bits in the HPRT */

  stm32_putreg(STM32_OTGHS_HPRT, newhprt);
}

/*******************************************************************************
 * Name: stm32_gint_discisr
 *
 * Description:
 *   USB OTG HS disconnect detected interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_discisr(FAR struct stm32_usbhost_s *priv)
{
  /* Handle the disconnection event */

  stm32_gint_disconnected(priv);

  /* Clear the dicsonnect interrupt */

  stm32_putreg(STM32_OTGHS_GINTSTS, OTGHS_GINT_DISC);
}

/*******************************************************************************
 * Name: stm32_gint_ipxfrisr
 *
 * Description:
 *   USB OTG HS incomplete periodic interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_ipxfrisr(FAR struct stm32_usbhost_s *priv)
{
  uint32_t regval;

  /* CHENA : Set to enable the channel
   * CHDIS : Set to stop transmitting/receiving data on a channel
   */

  regval = stm32_getreg(STM32_OTGHS_HCCHAR(0));
  regval |= (OTGHS_HCCHAR_CHDIS | OTGHS_HCCHAR_CHENA);
  stm32_putreg(STM32_OTGHS_HCCHAR(0), regval);

  /* Clear the incomplete isochronous OUT interrupt */

  stm32_putreg(STM32_OTGHS_GINTSTS, OTGHS_GINT_IPXFR);
}

/*******************************************************************************
 * Name: stm32_gint_isr
 *
 * Description:
 *   USB OTG HS global interrupt handler
 *
 *******************************************************************************/

static int stm32_gint_isr(int irq, FAR void *context)
{
  /* At present, there is only support for a single OTG HS host. Hence it is
   * pre-allocated as g_usbhost.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple devices.
   */

  FAR struct stm32_usbhost_s *priv = &g_usbhost;
  uint32_t pending;

  /* If OTG were supported, we would need to check if we are in host or
   * device mode when the global interrupt occurs.  Here we support only
   * host mode
   */

  /* Loop while there are pending interrupts to process.  This loop may save a
   * little interrupt handling overhead.
   */

  for (;;)
    {
      /* Get the unmasked bits in the GINT status */

      pending  = stm32_getreg(STM32_OTGHS_GINTSTS);
      pending &= stm32_getreg(STM32_OTGHS_GINTMSK);

      /* Return from the interrupt when there are no furhter pending
       * interrupts.
       */

      if (pending == 0)
        {
          return OK;
        }

      /* Otherwise, process each pending, unmasked GINT interrupts */


      /* Handle the start of frame interrupt */

#ifdef CONFIG_STM32_OTGHS_SOFINTR
      if ((pending & OTGHS_GINT_SOF) != 0)
        {
          usbhost_vtrace1(OTGHS_VTRACE1_GINT_SOF, 0);
          stm32_gint_sofisr(priv);
        }
#endif

      /* Handle the RxFIFO non-empty interrupt */

      if ((pending & OTGHS_GINT_RXFLVL) != 0)
        {
          usbhost_vtrace1(OTGHS_VTRACE1_GINT_RXFLVL, 0);
          stm32_gint_rxflvlisr(priv);
        }

      /* Handle the non-periodic TxFIFO empty interrupt */

      if ((pending & OTGHS_GINT_NPTXFE) != 0)
        {
          usbhost_vtrace1(OTGHS_VTRACE1_GINT_NPTXFE, 0);
          stm32_gint_nptxfeisr(priv);
        }

      /* Handle the periodic TxFIFO empty interrupt */

      if ((pending & OTGHS_GINT_PTXFE) != 0)
        {
          usbhost_vtrace1(OTGHS_VTRACE1_GINT_PTXFE, 0);
          stm32_gint_ptxfeisr(priv);
        }

      /* Handle the host channels interrupt */

      if ((pending & OTGHS_GINT_HC) != 0)
        {
          usbhost_vtrace1(OTGHS_VTRACE1_GINT_HC, 0);
          stm32_gint_hcisr(priv);
        }

      /* Handle the host port interrupt */

      if ((pending & OTGHS_GINT_HPRT) != 0)
        {
          stm32_gint_hprtisr(priv);
        }

      /* Handle the disconnect detected interrupt */

      if ((pending & OTGHS_GINT_DISC) != 0)
        {
          usbhost_vtrace1(OTGHS_VTRACE1_GINT_DISC, 0);
          stm32_gint_discisr(priv);
        }

      /* Handle the incomplete periodic transfer */

      if ((pending & OTGHS_GINT_IPXFR) != 0)
        {
          usbhost_vtrace1(OTGHS_VTRACE1_GINT_IPXFR, 0);
          stm32_gint_ipxfrisr(priv);
        }
    }

  /* We won't get here */

  return OK;
}

/*******************************************************************************
 * Name: stm32_gint_enable and stm32_gint_disable
 *
 * Description:
 *   Respectively enable or disable the global OTG HS interrupt.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 *******************************************************************************/

static void stm32_gint_enable(void)
{
  uint32_t regval;

  /* Set the GINTMSK bit to unmask the interrupt */

  regval  = stm32_getreg(STM32_OTGHS_GAHBCFG);
  regval |= OTGHS_GAHBCFG_GINTMSK;
  stm32_putreg(STM32_OTGHS_GAHBCFG, regval);
}

static void stm32_gint_disable(void)
{
  uint32_t regval;

  /* Clear the GINTMSK bit to mask the interrupt */

  regval  = stm32_getreg(STM32_OTGHS_GAHBCFG);
  regval &= ~OTGHS_GAHBCFG_GINTMSK;
  stm32_putreg(STM32_OTGHS_GAHBCFG, regval);
}

/*******************************************************************************
 * Name: stm32_hostinit_enable
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
 *******************************************************************************/

static inline void stm32_hostinit_enable(void)
{
  uint32_t regval;

  /* Disable all interrupts. */

  stm32_putreg(STM32_OTGHS_GINTMSK, 0);

  /* Clear any pending interrupts. */

  stm32_putreg(STM32_OTGHS_GINTSTS, 0xffffffff);

  /* Clear any pending USB OTG Interrupts (should be done elsewhere if OTG is supported) */

  stm32_putreg(STM32_OTGHS_GOTGINT, 0xffffffff);

  /* Clear any pending USB OTG interrupts */

  stm32_putreg(STM32_OTGHS_GINTSTS, 0xbfffffff);

  /* Enable the host interrupts */
  /* Common interrupts:
   *
   *   OTGHS_GINT_WKUP     : Resume/remote wakeup detected interrupt
   *   OTGHS_GINT_USBSUSP  : USB suspend
   */

  regval = (OTGHS_GINT_WKUP | OTGHS_GINT_USBSUSP);

  /* If OTG were supported, we would need to enable the following as well:
   *
   *   OTGHS_GINT_OTG      : OTG interrupt
   *   OTGHS_GINT_SRQ      : Session request/new session detected interrupt
   *   OTGHS_GINT_CIDSCHG  : Connector ID status change
   */

  /* Host-specific interrupts
   *
   *   OTGHS_GINT_SOF      : Start of frame
   *   OTGHS_GINT_RXFLVL   : RxFIFO non-empty
   *   OTGHS_GINT_IISOOXFR : Incomplete isochronous OUT transfer
   *   OTGHS_GINT_HPRT     : Host port interrupt
   *   OTGHS_GINT_HC       : Host channels interrupt
   *   OTGHS_GINT_DISC     : Disconnect detected interrupt
   */

#ifdef CONFIG_STM32_OTGHS_SOFINTR
  regval |= (OTGHS_GINT_SOF    | OTGHS_GINT_RXFLVL   | OTGHS_GINT_IISOOXFR |
             OTGHS_GINT_HPRT   | OTGHS_GINT_HC       | OTGHS_GINT_DISC);
#else
  regval |= (OTGHS_GINT_RXFLVL | OTGHS_GINT_IPXFR    | OTGHS_GINT_HPRT     |
             OTGHS_GINT_HC     | OTGHS_GINT_DISC);
#endif
  stm32_putreg(STM32_OTGHS_GINTMSK, regval);
}

/*******************************************************************************
 * Name: stm32_txfe_enable
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
 *******************************************************************************/

static void stm32_txfe_enable(FAR struct stm32_usbhost_s *priv, int chidx)
{
  FAR struct stm32_chan_s *chan = &priv->chan[chidx];
  irqstate_t flags;
  uint32_t regval;

  /* Disable all interrupts so that we have exclusive access to the GINTMSK
   * (it would be sufficent just to disable the GINT interrupt).
   */

  flags = irqsave();

  /* Should we enable the periodic or non-peridic Tx FIFO empty interrupts */

  regval = stm32_getreg(STM32_OTGHS_GINTMSK);
  switch (chan->eptype)
    {
    default:
    case OTGHS_EPTYPE_CTRL: /* Non periodic transfer */
    case OTGHS_EPTYPE_BULK:
      regval |= OTGHS_GINT_NPTXFE;
      break;

    case OTGHS_EPTYPE_INTR: /* Periodic transfer */
    case OTGHS_EPTYPE_ISOC:
      regval |= OTGHS_GINT_PTXFE;
      break;
    }

  /* Enable interrupts */

  stm32_putreg(STM32_OTGHS_GINTMSK, regval);
  irqrestore(flags);
}

/*******************************************************************************
 * USB Host Controller Operations
 *******************************************************************************/

/*******************************************************************************
 * Name: stm32_wait
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
 * Returned Values:
 *   Zero (OK) is returned on success when a device in connected or
 *   disconnected. This function will not return until either (1) a device is
 *   connected or disconnect to/from any hub port or until (2) some failure
 *   occurs.  On a failure, a negated errno value is returned indicating the
 *   nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_wait(FAR struct usbhost_connection_s *conn,
                      FAR struct usbhost_hubport_s **hport)
{
  FAR struct stm32_usbhost_s *priv = &g_usbhost;
  struct usbhost_hubport_s *connport;
  irqstate_t flags;

  /* Loop until a change in connection state is detected */

  flags = irqsave();
  for (;;)
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
          irqrestore(flags);

          uvdbg("RHport Connected: %s\n", connport->connected ? "YES" : "NO");
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
          irqrestore(flags);

          uvdbg("Hub port Connected: %s\n", connport->connected ? "YES" : "NO");
          return OK;
        }
#endif

      /* Wait for the next connection event */

      priv->pscwait = true;
      stm32_takesem(&priv->pscsem);
    }
}

/*******************************************************************************
 * Name: stm32_enumerate
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_rh_enumerate(FAR struct stm32_usbhost_s *priv,
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

      usbhost_trace1(OTGHS_TRACE1_DEVDISCONN,0);
      return -ENODEV;
    }

  DEBUGASSERT(priv->smstate == SMSTATE_ATTACHED);

  /* USB 2.0 spec says at least 50ms delay before port reset.  We wait 100ms. */

  usleep(100*1000);

  /* Reset the host port */

  stm32_portreset(priv);

  /* Get the current device speed */

  regval = stm32_getreg(STM32_OTGHS_HPRT);
  if ((regval & OTGHS_HPRT_PSPD_MASK) == OTGHS_HPRT_PSPD_LS)
    {
      priv->rhport.hport.speed = USB_SPEED_LOW;
    }
  else
    {
      priv->rhport.hport.speed = USB_SPEED_FULL;
    }

  /* Allocate and initialize the root hub port EP0 channels */

  ret = stm32_ctrlchan_alloc(priv, 0, 0, priv->rhport.hport.speed, &priv->ep0);
  if (ret < 0)
    {
      udbg("ERROR: Failed to allocate a control endpoint: %d\n", ret);
    }

  return ret;
}

static int stm32_enumerate(FAR struct usbhost_connection_s *conn,
                           FAR struct usbhost_hubport_s *hport)
{
  FAR struct stm32_usbhost_s *priv = &g_usbhost;
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
      ret = stm32_rh_enumerate(priv, conn, hport);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Then let the common usbhost_enumerate do the real enumeration. */

  uvdbg("Enumerate the device\n");
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

      udbg("ERROR: Enumeration failed: %d\n", ret);
      stm32_gint_disconnected(priv);
    }

  return ret;
}

/************************************************************************************
 * Name: stm32_ep0configure
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_ep0configure(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                              uint8_t funcaddr, uint8_t speed,
                              uint16_t maxpacketsize)
{
  FAR struct stm32_usbhost_s *priv = (FAR struct stm32_usbhost_s *)drvr;
  FAR struct stm32_ctrlinfo_s *ep0info = (FAR struct stm32_ctrlinfo_s *)ep0;
  FAR struct stm32_chan_s *chan;

  DEBUGASSERT(drvr != NULL && ep0info != NULL && funcaddr < 128 &&
              maxpacketsize <= 64);

  /* We must have exclusive access to the USB host hardware and state structures */

  stm32_takesem(&priv->exclsem);

  /* Configure the EP0 OUT channel */

  chan            = &priv->chan[ep0info->outndx];
  chan->funcaddr  = funcaddr;
  chan->speed     = speed;
  chan->maxpacket = maxpacketsize;

  stm32_chan_configure(priv, ep0info->outndx);

  /* Configure the EP0 IN channel */

  chan            = &priv->chan[ep0info->inndx];
  chan->funcaddr  = funcaddr;
  chan->speed     = speed;
  chan->maxpacket = maxpacketsize;

  stm32_chan_configure(priv, ep0info->inndx);

  stm32_givesem(&priv->exclsem);
  return OK;
}

/************************************************************************************
 * Name: stm32_epalloc
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_epalloc(FAR struct usbhost_driver_s *drvr,
                         FAR const struct usbhost_epdesc_s *epdesc,
                         FAR usbhost_ep_t *ep)
{
  FAR struct stm32_usbhost_s *priv = (FAR struct stm32_usbhost_s *)drvr;
  int ret;

  /* Sanity check.  NOTE that this method should only be called if a device is
   * connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(drvr != 0 && epdesc != NULL && ep != NULL);

  /* We must have exclusive access to the USB host hardware and state structures */

  stm32_takesem(&priv->exclsem);

  /* Handler control pipes differently from other endpoint types.  This is
   * because the normal, "transfer" endpoints are unidirectional an require
   * only a single channel.  Control endpoints, however, are bi-diretional
   * and require two channels, one for the IN and one for the OUT direction.
   */

  if (epdesc->xfrtype == OTGHS_EPTYPE_CTRL)
    {
      ret = stm32_ctrlep_alloc(priv, epdesc, ep);
    }
  else
    {
      ret = stm32_xfrep_alloc(priv, epdesc, ep);
    }

  stm32_givesem(&priv->exclsem);
  return ret;
}

/************************************************************************************
 * Name: stm32_epfree
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The endpoint to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  FAR struct stm32_usbhost_s *priv = (FAR struct stm32_usbhost_s *)drvr;

  DEBUGASSERT(priv);

  /* We must have exclusive access to the USB host hardware and state structures */

  stm32_takesem(&priv->exclsem);

  /* A single channel is represent by an index in the range of 0 to STM32_MAX_TX_FIFOS.
   * Otherwise, the ep must be a pointer to an allocated control endpoint structure.
   */

  if ((uintptr_t)ep < STM32_MAX_TX_FIFOS)
    {
      /* Halt the channel and mark the channel available */

      stm32_chan_free(priv, (int)ep);
    }
  else
    {
      /* Halt both control channel and mark the channels available */

      FAR struct stm32_ctrlinfo_s *ctrlep = (FAR struct stm32_ctrlinfo_s *)ep;
      stm32_chan_free(priv, ctrlep->inndx);
      stm32_chan_free(priv, ctrlep->outndx);

      /* And free the control endpoint container */

      kmm_free(ctrlep);
    }

  stm32_givesem(&priv->exclsem);
  return OK;
}

/*******************************************************************************
 * Name: stm32_alloc
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_alloc(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t **buffer, FAR size_t *maxlen)
{
  FAR uint8_t *alloc;

  DEBUGASSERT(drvr && buffer && maxlen);

  /* There is no special memory requirement for the STM32. */

  alloc = (FAR uint8_t *)kmm_malloc(CONFIG_STM32_OTGHS_DESCSIZE);
  if (!alloc)
    {
      return -ENOMEM;
    }

  /* Return the allocated address and size of the descriptor buffer */

  *buffer = alloc;
  *maxlen = CONFIG_STM32_OTGHS_DESCSIZE;
  return OK;
}

/*******************************************************************************
 * Name: stm32_free
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  /* There is no special memory requirement */

  DEBUGASSERT(drvr && buffer);
  kmm_free(buffer);
  return OK;
}

/************************************************************************************
 * Name: stm32_ioalloc
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_ioalloc(FAR struct usbhost_driver_s *drvr,
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
 * Name: stm32_iofree
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  /* There is no special memory requirement */

  DEBUGASSERT(drvr && buffer);
  kmm_free(buffer);
  return OK;
}

/*******************************************************************************
 * Name: stm32_ctrlin and stm32_ctrlout
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_ctrlin(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                        FAR const struct usb_ctrlreq_s *req,
                        FAR uint8_t *buffer)
{
  FAR struct stm32_usbhost_s *priv = (FAR struct stm32_usbhost_s *)drvr;
  FAR struct stm32_ctrlinfo_s *ep0info = (FAR struct stm32_ctrlinfo_s *)ep0;
  uint16_t buflen;
  uint32_t start;
  uint32_t elapsed;
  int retries;
  int ret;

  DEBUGASSERT(priv != NULL && ep0info != NULL && req != NULL);
  usbhost_vtrace2(OTGHS_VTRACE2_CTRLIN, req->type, req->req);
  uvdbg("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* Extract values from the request */

  buflen = stm32_getle16(req->len);

  /* We must have exclusive access to the USB host hardware and state structures */

  stm32_takesem(&priv->exclsem);

  /* Loop, retrying until the retry time expires */

  for (retries = 0; retries < STM32_RETRY_COUNT; retries++)
    {
      /* Send the SETUP request */

      ret = stm32_ctrl_sendsetup(priv, ep0info, req);
      if (ret < 0)
       {
          usbhost_trace1(OTGHS_TRACE1_SENDSETUP, -ret);
          continue;
        }

      /* Get the start time.  Loop again until the timeout expires */

      start = clock_systimer();
      do
        {
          /* Handle the IN data phase (if any) */

          if (buflen > 0)
            {
              ret = stm32_ctrl_recvdata(priv, ep0info, buffer, buflen);
              if (ret < 0)
                {
                  usbhost_trace1(OTGHS_TRACE1_RECVDATA, -ret);
                }
            }

          /* Handle the status OUT phase */

          if (ret == OK)
            {
              priv->chan[ep0info->outndx].outdata1 ^= true;
              ret = stm32_ctrl_senddata(priv, ep0info, NULL, 0);
              if (ret == OK)
                {
                  /* All success transactions exit here */

                  stm32_givesem(&priv->exclsem);
                  return OK;
                }

              usbhost_trace1(OTGHS_TRACE1_SENDDATA, ret < 0 ? -ret : ret);
            }

          /* Get the elapsed time (in frames) */

          elapsed = clock_systimer() - start;
        }
      while (elapsed < STM32_DATANAK_DELAY);
    }

  /* All failures exit here after all retries and timeouts have been exhausted */

  stm32_givesem(&priv->exclsem);
  return -ETIMEDOUT;
}

static int stm32_ctrlout(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                         FAR const struct usb_ctrlreq_s *req,
                         FAR const uint8_t *buffer)
{
  FAR struct stm32_usbhost_s *priv = (FAR struct stm32_usbhost_s *)drvr;
  FAR struct stm32_ctrlinfo_s *ep0info = (FAR struct stm32_ctrlinfo_s *)ep0;
  uint16_t buflen;
  uint32_t start;
  uint32_t elapsed;
  int retries;
  int ret;

  DEBUGASSERT(priv != NULL && ep0info != NULL && req != NULL);
  usbhost_vtrace2(OTGHS_VTRACE2_CTRLOUT, req->type, req->req);
  uvdbg("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* Extract values from the request */

  buflen = stm32_getle16(req->len);

  /* We must have exclusive access to the USB host hardware and state structures */

  stm32_takesem(&priv->exclsem);

  /* Loop, retrying until the retry time expires */

  for (retries = 0; retries < STM32_RETRY_COUNT; retries++)
    {
      /* Send the SETUP request */

      ret = stm32_ctrl_sendsetup(priv, ep0info, req);
      if (ret < 0)
        {
          usbhost_trace1(OTGHS_TRACE1_SENDSETUP, -ret);
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
              ret = stm32_ctrl_senddata(priv, ep0info, NULL, 0);
              if (ret < 0)
                {
                  usbhost_trace1(OTGHS_TRACE1_SENDDATA, -ret);
                }
            }

          /* Handle the status IN phase */

          if (ret == OK)
            {
              ret = stm32_ctrl_recvdata(priv, ep0info, NULL, 0);
              if (ret == OK)
                {
                  /* All success transactins exit here */

                  stm32_givesem(&priv->exclsem);
                  return OK;
                }

              usbhost_trace1(OTGHS_TRACE1_RECVDATA, ret < 0 ? -ret : ret);
            }

          /* Get the elapsed time (in frames) */

          elapsed = clock_systimer() - start;
        }
      while (elapsed < STM32_DATANAK_DELAY);
    }

  /* All failures exit here after all retries and timeouts have been exhausted */

  stm32_givesem(&priv->exclsem);
  return -ETIMEDOUT;
}

/*******************************************************************************
 * Name: stm32_transfer
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
 * Returned Values:
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
 *******************************************************************************/

static ssize_t stm32_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                              FAR uint8_t *buffer, size_t buflen)
{
  FAR struct stm32_usbhost_s *priv  = (FAR struct stm32_usbhost_s *)drvr;
  unsigned int chidx = (unsigned int)ep;
  ssize_t nbytes;

  uvdbg("chidx: %d buflen: %d\n",  (unsigned int)ep, buflen);

  DEBUGASSERT(priv && buffer && chidx < STM32_MAX_TX_FIFOS && buflen > 0);

  /* We must have exclusive access to the USB host hardware and state structures */

  stm32_takesem(&priv->exclsem);

  /* Handle IN and OUT transfer slightly differently */

  if (priv->chan[chidx].in)
    {
      nbytes = stm32_in_transfer(priv, chidx, buffer, buflen);
    }
  else
    {
      nbytes = stm32_out_transfer(priv, chidx, buffer, buflen);
    }

  stm32_givesem(&priv->exclsem);
  return nbytes;
}

/*******************************************************************************
 * Name: stm32_asynch
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately.  When the transfer
 *   completes, the the callback will be invoked with the provided transfer.
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int stm32_asynch(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                        FAR uint8_t *buffer, size_t buflen,
                        usbhost_asynch_t callback, FAR void *arg)
{
  FAR struct stm32_usbhost_s *priv  = (FAR struct stm32_usbhost_s *)drvr;
  unsigned int chidx = (unsigned int)ep;
  int ret;

  uvdbg("chidx: %d buflen: %d\n",  (unsigned int)ep, buflen);

  DEBUGASSERT(priv && buffer && chidx < STM32_MAX_TX_FIFOS && buflen > 0);

  /* We must have exclusive access to the USB host hardware and state structures */

  stm32_takesem(&priv->exclsem);

  /* Handle IN and OUT transfer slightly differently */

  if (priv->chan[chidx].in)
    {
      ret = stm32_in_asynch(priv, chidx, buffer, buflen, callback, arg);
    }
  else
    {
      ret = stm32_out_asynch(priv, chidx, buffer, buflen, callback, arg);
    }

  stm32_givesem(&priv->exclsem);
  return ret;
}
#endif /* CONFIG_USBHOST_ASYNCH */

/************************************************************************************
 * Name: stm32_cancel
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ************************************************************************************/

static int stm32_cancel(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  FAR struct stm32_usbhost_s *priv  = (FAR struct stm32_usbhost_s *)drvr;
  FAR struct stm32_chan_s *chan;
  unsigned int chidx = (unsigned int)ep;
  irqstate_t flags;

  uvdbg("chidx: %u: %d\n",  chidx);

  DEBUGASSERT(priv && chidx < STM32_MAX_TX_FIFOS);
  chan = &priv->chan[chidx];

  /* We must have exclusive access to the USB host hardware and state structures.
   * And when we have that, we need to disable interrupts to avoid race conditions
   * with the asynchronous completion of the transfer being cancelled.
   */

  stm32_takesem(&priv->exclsem);
  flags = irqsave();

  /* Halt the channel */

  stm32_chan_halt(priv, chidx, CHREASON_CANCELLED);
  chan->result = -ESHUTDOWN;

  /* Is there a thread waiting for this transfer to complete? */

  if (chan->waiter)
    {
#ifdef CONFIG_USBHOST_ASYNCH
      /* Yes.. there should not also be a callback scheduled */

      DEBUGASSERT(chan->callback == NULL);
#endif

      /* Wake'em up! */

      stm32_givesem(&chan->waitsem);
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

  irqrestore(flags);
  stm32_givesem(&priv->exclsem);
  return OK;
}
#endif /* CONFIG_USBHOST_ASYNCH */

/************************************************************************************
 * Name: stm32_connect
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ************************************************************************************/

#ifdef CONFIG_USBHOST_HUB
static int stm32_connect(FAR struct usbhost_driver_s *drvr,
                         FAR struct usbhost_hubport_s *hport,
                         bool connected)
{
  FAR struct stm32_usbhost_s *priv = (FAR struct stm32_usbhost_s *)drvr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL && hport != NULL);

  /* Set the connected/disconnected flag */

  hport->connected = connected;
  ullvdbg("Hub port %d connected: %s\n", hport->port, connected ? "YES" : "NO");

  /* Report the connection event */

  flags = irqsave();
  priv->hport = hport;
  if (priv->pscwait)
    {
      priv->pscwait = false;
      stm32_givesem(&priv->pscsem);
    }

  irqrestore(flags);
  return OK;
}
#endif

/*******************************************************************************
 * Name: stm32_disconnect
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
 *   hport - The port from which the device is being disconnected.  Might be a port
 *      on a hub.
 *
 * Returned Values:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static void stm32_disconnect(FAR struct usbhost_driver_s *drvr,
                             FAR struct usbhost_hubport_s *hport)
{
  DEBUGASSERT(hport != NULL);
  hport->devclass = NULL;
}

/*******************************************************************************
 * Initialization
 *******************************************************************************/
/*******************************************************************************
 * Name: stm32_portreset
 *
 * Description:
 *   Reset the USB host port.
 *
 *   NOTE: "Before starting to drive a USB reset, the application waits for the
 *   OTG interrupt triggered by the debounce done bit (DBCDNE bit in
 *   OTG_HS_GOTGINT), which indicates that the bus is stable again after the
 *   electrical debounce caused by the attachment of a pull-up resistor on DP
 *   (HS) or DM (LS).
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None
 *
 *******************************************************************************/

static void stm32_portreset(FAR struct stm32_usbhost_s *priv)
{
  uint32_t regval;

  regval  = stm32_getreg(STM32_OTGHS_HPRT);
  regval &= ~(OTGHS_HPRT_PENA|OTGHS_HPRT_PCDET|OTGHS_HPRT_PENCHNG|OTGHS_HPRT_POCCHNG);
  regval |= OTGHS_HPRT_PRST;
  stm32_putreg(STM32_OTGHS_HPRT, regval);

  up_mdelay(20);

  regval &= ~OTGHS_HPRT_PRST;
  stm32_putreg(STM32_OTGHS_HPRT, regval);

  up_mdelay(20);
}

/*******************************************************************************
 * Name: stm32_flush_txfifos
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
 *******************************************************************************/

static void stm32_flush_txfifos(uint32_t txfnum)
{
  uint32_t regval;
  uint32_t timeout;

  /* Initiate the TX FIFO flush operation */

  regval = OTGHS_GRSTCTL_TXFFLSH | txfnum;
  stm32_putreg(regval, STM32_OTGHS_GRSTCTL);

  /* Wait for the FLUSH to complete */

  for (timeout = 0; timeout < STM32_FLUSH_DELAY; timeout++)
    {
      regval = stm32_getreg(STM32_OTGHS_GRSTCTL);
      if ((regval & OTGHS_GRSTCTL_TXFFLSH) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);
}

/*******************************************************************************
 * Name: stm32_flush_rxfifo
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
 *******************************************************************************/

static void stm32_flush_rxfifo(void)
{
  uint32_t regval;
  uint32_t timeout;

  /* Initiate the RX FIFO flush operation */

  stm32_putreg(OTGHS_GRSTCTL_RXFFLSH, STM32_OTGHS_GRSTCTL);

  /* Wait for the FLUSH to complete */

  for (timeout = 0; timeout < STM32_FLUSH_DELAY; timeout++)
    {
      regval = stm32_getreg(STM32_OTGHS_GRSTCTL);
      if ((regval & OTGHS_GRSTCTL_RXFFLSH) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);
}

/*******************************************************************************
 * Name: stm32_vbusdrive
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
 *******************************************************************************/

static void stm32_vbusdrive(FAR struct stm32_usbhost_s *priv, bool state)
{
  uint32_t regval;

  /* Enable/disable the external charge pump */

  stm32_usbhost_vbusdrive(0, state);

  /* Turn on the Host port power. */

  regval = stm32_getreg(STM32_OTGHS_HPRT);
  regval &= ~(OTGHS_HPRT_PENA|OTGHS_HPRT_PCDET|OTGHS_HPRT_PENCHNG|OTGHS_HPRT_POCCHNG);

  if (((regval & OTGHS_HPRT_PPWR) == 0) && state)
    {
      regval |= OTGHS_HPRT_PPWR;
      stm32_putreg(STM32_OTGHS_HPRT, regval);
    }

  if (((regval & OTGHS_HPRT_PPWR) != 0) && !state)
    {
      regval &= ~OTGHS_HPRT_PPWR;
      stm32_putreg(STM32_OTGHS_HPRT, regval);
    }

  up_mdelay(200);
}

/*******************************************************************************
 * Name: stm32_host_initialize
 *
 * Description:
 *   Initialize/re-initialize hardware for host mode operation.  At present,
 *   this function is called only from stm32_hw_initialize().  But if OTG mode
 *   were supported, this function would also be called to swtich between
 *   host and device modes on a connector ID change interrupt.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 *******************************************************************************/

static void stm32_host_initialize(FAR struct stm32_usbhost_s *priv)
{
  uint32_t regval;
  uint32_t offset;
  int i;

  /* Restart the PHY Clock */

  stm32_putreg(STM32_OTGHS_PCGCCTL, 0);

  /* Initialize Host Configuration (HCFG) register */

  regval  = stm32_getreg(STM32_OTGHS_HCFG);
  regval &= ~OTGHS_HCFG_FSLSPCS_MASK;
  regval |= OTGHS_HCFG_FSLSPCS_FS48MHz;
  stm32_putreg(STM32_OTGHS_HCFG, regval);

  /* Reset the host port */

  stm32_portreset(priv);

  /* Clear the HS-/LS-only support bit in the HCFG register */

  regval = stm32_getreg(STM32_OTGHS_HCFG);
  regval &= ~OTGHS_HCFG_FSLSS;
  stm32_putreg(STM32_OTGHS_HCFG, regval);

  /* Carve up FIFO memory for the Rx FIFO and the periodic and non-periodic Tx FIFOs */
  /* Configure Rx FIFO size (GRXHSIZ) */

  stm32_putreg(STM32_OTGHS_GRXFSIZ, CONFIG_STM32_OTGHS_RXFIFO_SIZE);
  offset = CONFIG_STM32_OTGHS_RXFIFO_SIZE;

  /* Setup the host non-periodic Tx FIFO size (HNPTXHSIZ) */

  regval = (offset | (CONFIG_STM32_OTGHS_NPTXFIFO_SIZE << OTGHS_HNPTXFSIZ_NPTXFD_SHIFT));
  stm32_putreg(STM32_OTGHS_HNPTXFSIZ, regval);
  offset += CONFIG_STM32_OTGHS_NPTXFIFO_SIZE;

  /* Set up the host periodic Tx fifo size register (HPTXHSIZ) */

  regval = (offset | (CONFIG_STM32_OTGHS_PTXFIFO_SIZE << OTGHS_HPTXFSIZ_PTXFD_SHIFT));
  stm32_putreg(STM32_OTGHS_HPTXFSIZ, regval);

  /* If OTG were supported, we sould need to clear HNP enable bit in the
   * USB_OTG control register about here.
   */

  /* Flush all FIFOs */

  stm32_flush_txfifos(OTGHS_GRSTCTL_TXFNUM_HALL);
  stm32_flush_rxfifo();

  /* Clear all pending HC Interrupts */

  for (i = 0; i < STM32_NHOST_CHANNELS; i++)
    {
      stm32_putreg(STM32_OTGHS_HCINT(i), 0xffffffff);
      stm32_putreg(STM32_OTGHS_HCINTMSK(i), 0);
    }

  /* Driver Vbus +5V (the smoke test).  Should be done elsewhere in OTG
   * mode.
   */

  stm32_vbusdrive(priv, true);

  /* Enable host interrupts */

  stm32_hostinit_enable();
}

/*******************************************************************************
 * Name: stm32_sw_initialize
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
 *******************************************************************************/

static inline void stm32_sw_initialize(FAR struct stm32_usbhost_s *priv)
{
  FAR struct usbhost_driver_s *drvr;
  FAR struct usbhost_hubport_s *hport;
  int i;

  /* Initialize the device operations */

  drvr                 = &priv->drvr;
  drvr->ep0configure   = stm32_ep0configure;
  drvr->epalloc        = stm32_epalloc;
  drvr->epfree         = stm32_epfree;
  drvr->alloc          = stm32_alloc;
  drvr->free           = stm32_free;
  drvr->ioalloc        = stm32_ioalloc;
  drvr->iofree         = stm32_iofree;
  drvr->ctrlin         = stm32_ctrlin;
  drvr->ctrlout        = stm32_ctrlout;
  drvr->transfer       = stm32_transfer;
#ifdef CONFIG_USBHOST_ASYNCH
  drvr->asynch         = stm32_asynch;
#endif
  drvr->cancel         = stm32_cancel;
#ifdef CONFIG_USBHOST_HUB
  drvr->connect        = stm32_connect;
#endif
  drvr->disconnect     = stm32_disconnect;

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

  sem_init(&priv->pscsem,  0, 0);
  sem_init(&priv->exclsem, 0, 1);

  /* Initialize the driver state data */

  priv->smstate   = SMSTATE_DETACHED;
  priv->connected = false;
  priv->change    = false;

  /* Put all of the channels back in their initial, allocated state */

  memset(priv->chan, 0, STM32_MAX_TX_FIFOS * sizeof(struct stm32_chan_s));

  /* Initialize each channel */

  for (i = 0; i < STM32_MAX_TX_FIFOS; i++)
    {
      FAR struct stm32_chan_s *chan = &priv->chan[i];
      chan->chidx = i;
      sem_init(&chan->waitsem,  0, 0);
    }
}

/*******************************************************************************
 * Name: stm32_hw_initialize
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
 *******************************************************************************/

static inline int stm32_hw_initialize(FAR struct stm32_usbhost_s *priv)
{
  uint32_t regval;
  unsigned long timeout;

  /* Set the PHYSEL bit in the GUSBCFG register to select the OTG HS serial
   * transceiver: "This bit is always 1 with write-only access"
   */

  regval = stm32_getreg(STM32_OTGHS_GUSBCFG);;
  regval |= OTGHS_GUSBCFG_PHYSEL;
  stm32_putreg(STM32_OTGHS_GUSBCFG, regval);

  /* Reset after a PHY select and set Host mode.  First, wait for AHB master
   * IDLE state.
   */

  for (timeout = 0; timeout < STM32_READY_DELAY; timeout++)
    {
      up_udelay(3);
      regval = stm32_getreg(STM32_OTGHS_GRSTCTL);
      if ((regval & OTGHS_GRSTCTL_AHBIDL) != 0)
        {
          break;
        }
    }

  /* Then perform the core soft reset. */

  stm32_putreg(STM32_OTGHS_GRSTCTL, OTGHS_GRSTCTL_CSRST);
  for (timeout = 0; timeout < STM32_READY_DELAY; timeout++)
    {
      regval = stm32_getreg(STM32_OTGHS_GRSTCTL);
      if ((regval & OTGHS_GRSTCTL_CSRST) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);

  /* Deactivate the power down */

  regval  = (OTGHS_GCCFG_PWRDWN | OTGHS_GCCFG_VBUSASEN | OTGHS_GCCFG_VBUSBSEN);
#ifndef CONFIG_USBDEV_VBUSSENSING
  regval |= OTGHS_GCCFG_NOVBUSSENS;
#endif
#ifdef CONFIG_STM32_OTGHS_SOFOUTPUT
  regval |= OTGHS_GCCFG_SOFOUTEN;
#endif
  stm32_putreg(STM32_OTGHS_GCCFG, regval);
  up_mdelay(20);

  /* Initialize OTG features:  In order to support OTP, the HNPCAP and SRPCAP
   * bits would need to be set in the GUSBCFG register about here.
   */

  /* Force Host Mode */

  regval  = stm32_getreg(STM32_OTGHS_GUSBCFG);
  regval &= ~OTGHS_GUSBCFG_FDMOD;
  regval |= OTGHS_GUSBCFG_FHMOD;
  stm32_putreg(STM32_OTGHS_GUSBCFG, regval);
  up_mdelay(50);

  /* Initialize host mode and return success */

  stm32_host_initialize(priv);
  return OK;
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: stm32_otghshost_initialize
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
 *******************************************************************************/

FAR struct usbhost_connection_s *stm32_otghshost_initialize(int controller)
{
  /* At present, there is only support for a single OTG HS host. Hence it is
   * pre-allocated as g_usbhost.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple devices.
   */

  FAR struct stm32_usbhost_s *priv = &g_usbhost;

  /* Sanity checks */

  DEBUGASSERT(controller == 0);

  /* Make sure that interrupts from the OTG HS core are disabled */

  stm32_gint_disable();

  /* Reset the state of the host driver */

  stm32_sw_initialize(priv);

  /* Alternate function pin configuration.  Here we assume that:
   *
   * 1. GPIOA, SYSCFG, and OTG HS peripheral clocking have already been\
   *    enabled as part of the boot sequence.
   * 2. Board-specific logic has already enabled other board specific GPIOs
   *    for things like soft pull-up, VBUS sensing, power controls, and over-
   *    current detection.
   */

  /* Configure OTG HS alternate function pins for DM, DP, ID, and SOF.
   *
   * PIN* SIGNAL      DIRECTION
   * ---- ----------- ----------
   * PA8  OTG_HS_SOF  SOF clock output
   * PA9  OTG_HS_VBUS VBUS input for device, Driven by external regulator by
   *                  host (not an alternate function)
   * PA10 OTG_HS_ID   OTG ID pin (only needed in Dual mode)
   * PA11 OTG_HS_DM   D- I/O
   * PA12 OTG_HS_DP   D+ I/O
   *
   * *Pins may vary from device-to-device.
   */

  stm32_configgpio(GPIO_OTGHSFS_DM);
  stm32_configgpio(GPIO_OTGHSFS_DP);
//  stm32_configgpio(GPIO_OTGHSFS_ID);    /* Only needed for OTG */

  /* SOF output pin configuration is configurable */

#ifdef CONFIG_STM32_OTGHS_SOFOUTPUT
  stm32_configgpio(GPIO_OTGHSFS_SOF);
#endif

  /* Initialize the USB OTG HS core */

  stm32_hw_initialize(priv);

  /* Attach USB host controller interrupt handler */

  if (irq_attach(STM32_IRQ_OTGHS, stm32_gint_isr) != 0)
    {
      usbhost_trace1(OTGHS_TRACE1_IRQATTACH, 0);
      return NULL;
    }

  /* Enable USB OTG HS global interrupts */

  stm32_gint_enable();

  /* Enable interrupts at the interrupt controller */

  up_enable_irq(STM32_IRQ_OTGHS);
  return &g_usbconn;
}

#endif /* CONFIG_USBHOST && CONFIG_STM32_OTGHS */

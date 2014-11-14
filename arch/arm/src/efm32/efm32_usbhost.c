/*******************************************************************************
 * arch/arm/src/efm32/efm32_usbhost.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
#include <nuttx/usb/usbhost_trace.h>

#include <arch/irq.h>

#include "chip.h"             /* Includes default GPIO settings */
#include <arch/board/board.h> /* May redefine GPIO settings */

#include "up_arch.h"
#include "up_internal.h"

#include "efm32_usb.h"

#if defined(CONFIG_USBHOST) && defined(CONFIG_EFM32_OTGFS)

/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************/
/* Configuration ***************************************************************/
/*
 * EFM32 USB OTG FS Host Driver Support
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
 *    debug.  Depends on CONFIG_DEBUG.
 *  CONFIG_EFM32_USBHOST_PKTDUMP - Dump all incoming and outgoing USB
 *    packets. Depends on CONFIG_DEBUG.
 */

/* Pre-requisites (partial) */

#ifndef CONFIG_EFM32_SYSCFG
#  error "CONFIG_EFM32_SYSCFG is required"
#endif

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

/* Register/packet debug depends on CONFIG_DEBUG */

#ifndef CONFIG_DEBUG
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
#define EFM32_DEF_DEVADDR         0   /* Default device address */

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

/*******************************************************************************
 * Private Types
 *******************************************************************************/

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
  CHREASON_FRMOR         /* Frame overrun */
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
  uint8_t           epno;      /* Device endpoint number (0-127) */
  uint8_t           eptype;    /* See USB_EPTYPE_* definitions */
  uint8_t           pid;       /* Data PID */
  uint8_t           npackets;  /* Number of packets (for data toggle) */
  bool              inuse;     /* True: This channel is "in use" */
  volatile bool     indata1;   /* IN data toggle. True: DATA01 (Bulk and INTR only) */
  volatile bool     outdata1;  /* OUT data toggle.  True: DATA01 */
  bool              in;        /* True: IN endpoint */
  volatile bool     waiter;    /* True: Thread is waiting for a channel event */
  uint16_t          maxpacket; /* Max packet size */
  volatile uint16_t buflen;    /* Buffer length (remaining) */
  volatile uint16_t inflight;  /* Number of Tx bytes "in-flight" */
  FAR uint8_t      *buffer;    /* Transfer buffer pointer */
};

/* This structure retains the state of the USB host controller */

struct efm32_usbhost_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to structefm32_usbhost_s.
   */

  struct usbhost_driver_s drvr;

  /* The bound device class driver */

  struct usbhost_class_s *class;

  /* Overall driver status */

  volatile uint8_t  smstate;   /* The state of the USB host state machine */
  uint8_t           devaddr;   /* Device address */
  uint8_t           ep0in;     /* EP0 IN control channel index */
  uint8_t           ep0out;    /* EP0 OUT control channel index */
  uint8_t           ep0size;   /* EP0 max packet size */
  uint8_t           chidx;     /* ID of channel waiting for space in Tx FIFO */
  bool              lowspeed;  /* True: low speed device */
  volatile bool     connected; /* Connected to device */
  volatile bool     eventwait; /* True: Thread is waiting for a port event */
  sem_t             exclsem;   /* Support mutually exclusive access */
  sem_t             eventsem;  /* Semaphore to wait for a port event */

  /* The state of each host channel */

  struct efm32_chan_s chan[EFM32_MAX_TX_FIFOS];
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

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
#define efm32_givesem(s) sem_post(s);

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
static int efm32_chan_wait(FAR struct efm32_usbhost_s *priv,
                           FAR struct efm32_chan_s *chan);
static void efm32_chan_wakeup(FAR struct efm32_usbhost_s *priv,
                              FAR struct efm32_chan_s *chan);

/* Control/data transfer logic *************************************************/

static void efm32_transfer_start(FAR struct efm32_usbhost_s *priv, int chidx);
#if 0 /* Not used */
static inline uint16_t efm32_getframe(void);
#endif
static int efm32_ctrl_sendsetup(FAR struct efm32_usbhost_s *priv,
                                FAR const struct usb_ctrlreq_s *req);
static int efm32_ctrl_senddata(FAR struct efm32_usbhost_s *priv,
                               FAR uint8_t *buffer, unsigned int buflen);
static int efm32_ctrl_recvdata(FAR struct efm32_usbhost_s *priv,
                               FAR uint8_t *buffer, unsigned int buflen);
static int efm32_in_transfer(FAR struct efm32_usbhost_s *priv, int chidx,
                             FAR uint8_t *buffer, size_t buflen);
static int efm32_out_transfer(FAR struct efm32_usbhost_s *priv, int chidx,
                              FAR uint8_t *buffer, size_t buflen);

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

static int efm32_gint_isr(int irq, FAR void *context);

/* Interrupt controls */

static void efm32_gint_enable(void);
static void efm32_gint_disable(void);
static inline void efm32_hostinit_enable(void);
static void efm32_txfe_enable(FAR struct efm32_usbhost_s *priv, int chidx);

/* USB host controller operations **********************************************/

static int efm32_wait(FAR struct usbhost_connection_s *conn,
                      FAR const bool *connected);
static int efm32_enumerate(FAR struct usbhost_connection_s *conn, int rhpndx);

static int efm32_ep0configure(FAR struct usbhost_driver_s *drvr, uint8_t funcaddr,
                              uint16_t maxpacketsize);
static int efm32_getdevinfo(FAR struct usbhost_driver_s *drvr,
                            FAR struct usbhost_devinfo_s *devinfo);
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
static int efm32_ctrlin(FAR struct usbhost_driver_s *drvr,
                        FAR const struct usb_ctrlreq_s *req,
                        FAR uint8_t *buffer);
static int efm32_ctrlout(FAR struct usbhost_driver_s *drvr,
                         FAR const struct usb_ctrlreq_s *req,
                         FAR const uint8_t *buffer);
static int efm32_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                          FAR uint8_t *buffer, size_t buflen);
static void efm32_disconnect(FAR struct usbhost_driver_s *drvr);

/* Initialization **************************************************************/

static void efm32_portreset(FAR struct efm32_usbhost_s *priv);
static void efm32_flush_txfifos(uint32_t txfnum);
static void efm32_flush_rxfifo(void);
static void efm32_vbusdrive(FAR struct efm32_usbhost_s *priv, bool state);
static void efm32_host_initialize(FAR struct efm32_usbhost_s *priv);

static inline void efm32_sw_initialize(FAR struct efm32_usbhost_s *priv);
static inline int efm32_hw_initialize(FAR struct efm32_usbhost_s *priv);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

/* In this driver implementation, support is provided for only a single a single
 * USB device.  All status information can be simply retained in a single global
 * instance.
 */

static struct efm32_usbhost_s g_usbhost =
{
  .drvr             =
    {
      .ep0configure = efm32_ep0configure,
      .getdevinfo   = efm32_getdevinfo,
      .epalloc      = efm32_epalloc,
      .epfree       = efm32_epfree,
      .alloc        = efm32_alloc,
      .free         = efm32_free,
      .ioalloc      = efm32_ioalloc,
      .iofree       = efm32_iofree,
      .ctrlin       = efm32_ctrlin,
      .ctrlout      = efm32_ctrlout,
      .transfer     = efm32_transfer,
      .disconnect   = efm32_disconnect,
    },
  .class            = NULL,
};

/* This is the connection/enumeration interface */

static struct usbhost_connection_s g_usbconn =
{
  .wait             = efm32_wait,
  .enumerate        = efm32_enumerate,
};

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: efm32_printreg
 *
 * Description:
 *   Print the contents of an EFM32xx register operation
 *
 *******************************************************************************/

#ifdef CONFIG_EFM32_USBHOST_REGDEBUG
static void efm32_printreg(uint32_t addr, uint32_t val, bool iswrite)
{
  lldbg("%08x%s%08x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/*******************************************************************************
 * Name: efm32_checkreg
 *
 * Description:
 *   Get the contents of an EFM32 register
 *
 *******************************************************************************/

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

              lldbg("[repeats %d more times]\n", count);
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

/*******************************************************************************
 * Name: efm32_getreg
 *
 * Description:
 *   Get the contents of an EFM32 register
 *
 *******************************************************************************/

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

/*******************************************************************************
 * Name: efm32_putreg
 *
 * Description:
 *   Set the contents of an EFM32 register to a value
 *
 *******************************************************************************/

#ifdef CONFIG_EFM32_USBHOST_REGDEBUG
static void efm32_putreg(uint32_t addr, uint32_t val)
{
  /* Check if we need to print this value */

  efm32_checkreg(addr, val, true);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/*******************************************************************************
 * Name: efm32_modifyreg
 *
 * Description:
 *   Modify selected bits of an EFM32 register.
 *
 *******************************************************************************/

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
 *******************************************************************************/

static void efm32_takesem(sem_t *sem)
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
 * Name: efm32_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 *******************************************************************************/

static inline uint16_t efm32_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/*******************************************************************************
 * Name: efm32_chan_alloc
 *
 * Description:
 *   Allocate a channel.
 *
 *******************************************************************************/

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

/*******************************************************************************
 * Name: efm32_chan_free
 *
 * Description:
 *   Free a previoiusly allocated channel.
 *
 *******************************************************************************/

static void efm32_chan_free(FAR struct efm32_usbhost_s *priv, int chidx)
{
  DEBUGASSERT((unsigned)chidx < EFM32_NHOST_CHANNELS);

  /* Halt the channel */

  efm32_chan_halt(priv, chidx, CHREASON_FREED);

  /* Mark the channel available */

  priv->chan[chidx].inuse = false;
}

/*******************************************************************************
 * Name: efm32_chan_freeall
 *
 * Description:
 *   Free all channels.
 *
 *******************************************************************************/

static inline void efm32_chan_freeall(FAR struct efm32_usbhost_s *priv)
{
   uint8_t chidx;

   /* Free all host channels */

   for (chidx = 2; chidx < EFM32_NHOST_CHANNELS; chidx ++)
     {
       efm32_chan_free(priv, chidx);
     }
}

/*******************************************************************************
 * Name: efm32_chan_configure
 *
 * Description:
 *   Configure or re-configure a host channel.  Host channels are configured
 *   when endpoint is allocated and EP0 (only) is re-configured with the
 *   max packet size or device address changes.
 *
 *******************************************************************************/

static void efm32_chan_configure(FAR struct efm32_usbhost_s *priv, int chidx)
{
  uint32_t regval;

  /* Clear any old pending interrupts for this host channel. */

  efm32_putreg(EFM32_USB_HCn_INT(chidx), 0xffffffff);

  /* Enable channel interrupts required for transfers on this channel. */

  regval = 0;

  switch (priv->chan[chidx].eptype)
    {
    case USB_EPTYPE_CTRL:
    case USB_EPTYPE_BULK:
      {
#ifdef HAVE_USBHOST_TRACE_VERBOSE
        uint16_t intrace;
        uint16_t outtrace;

        /* Determine the definitive trace ID to use below */

        if (priv->chan[chidx].eptype == USB_EPTYPE_CTRL)
          {
            intrace  = OTGFS_VTRACE2_CHANCONF_CTRL_IN;
            outtrace = OTGFS_VTRACE2_CHANCONF_CTRL_OUT;
          }
        else
          {
            intrace  = OTGFS_VTRACE2_CHANCONF_BULK_IN;
            outtrace = OTGFS_VTRACE2_CHANCONF_BULK_OUT;
          }
#endif

        /* Interrupts required for CTRL and BULK endpoints */

        regval |= (USB_HC_INTMSK_XFERCOMPLMSK  | USB_HC_INTMSK_STALLMSK |
                   USB_HC_INTMSK_NAKMSK | USB_HC_INTMSK_XACTERRMSK |
                   USB_HC_INTMSK_DATATGLERRMSK);

        /* Additional setting for IN/OUT endpoints */

        if (priv->chan[chidx].in)
          {
            usbhost_vtrace2(intrace, chidx, priv->chan[chidx].epno);
            regval |= USB_HC_INTMSK_BBLERRMSK;
          }
        else
          {
            usbhost_vtrace2(outtrace, chidx, priv->chan[chidx].epno);
          }
      }
      break;

    case USB_EPTYPE_INTR:
      {
        /* Interrupts required for INTR endpoints */

        regval |= (USB_HC_INTMSK_XFERCOMPLMSK | USB_HC_INTMSK_STALLMSK |
                   USB_HC_INTMSK_NAKMSK | USB_HC_INTMSK_XACTERRMSK |
                   USB_HC_INTMSK_FRMOVRUNMSK | USB_HC_INTMSK_DATATGLERRMSK);

        /* Additional setting for IN endpoints */

        if (priv->chan[chidx].in)
          {
            usbhost_vtrace2(OTGFS_VTRACE2_CHANCONF_INTR_IN, chidx,
                            priv->chan[chidx].epno);
            regval |= USB_HC_INTMSK_BBLERRMSK;
          }
#ifdef HAVE_USBHOST_TRACE_VERBOSE
        else
          {
            usbhost_vtrace2(OTGFS_VTRACE2_CHANCONF_INTR_OUT, chidx,
                            priv->chan[chidx].epno);
          }
#endif
      }
      break;

    case USB_EPTYPE_ISOC:
      {
        /* Interrupts required for ISOC endpoints */

        regval |= (USB_HC_INTMSK_XFERCOMPLMSK | USB_HC_INTMSK_ACKMSK |
                   USB_HC_INTMSK_FRMOVRUNMSK);

        /* Additional setting for IN endpoints */

        if (priv->chan[chidx].in)
          {
            usbhost_vtrace2(OTGFS_VTRACE2_CHANCONF_ISOC_IN, chidx,
                            priv->chan[chidx].epno);
            regval |= (USB_HC_INTMSK_XACTERRMSK | USB_HC_INTMSK_BBLERRMSK);
          }
#ifdef HAVE_USBHOST_TRACE_VERBOSE
        else
          {
            usbhost_vtrace2(OTGFS_VTRACE2_CHANCONF_ISOC_OUT, chidx,
                            priv->chan[chidx].epno);
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

  regval = ((uint32_t)priv->chan[chidx].maxpacket << _USB_HC_CHAR_MPS_SHIFT) |
           ((uint32_t)priv->chan[chidx].epno << _USB_HC_CHAR_EPNUM_SHIFT) |
           ((uint32_t)priv->chan[chidx].eptype << _USB_HC_CHAR_EPTYPE_SHIFT) |
           ((uint32_t)priv->devaddr << _USB_HC_CHAR_DEVADDR_SHIFT);

  /* Special case settings for low speed devices */

  if (priv->lowspeed)
    {
      regval |= USB_HC_CHAR_LSPDDEV;
    }

  /* Special case settings for IN endpoints */

  if (priv->chan[chidx].in)
    {
      regval |= USB_HC_CHAR_EPDIR_IN;
    }

  /* Special case settings for INTR endpoints */

  if (priv->chan[chidx].eptype == USB_EPTYPE_INTR)
    {
      regval |= USB_HC_CHAR_ODDFRM;
    }

  /* Write the channel configuration */

  efm32_putreg(EFM32_USB_HCCHAR(chidx), regval);
}

/*******************************************************************************
 * Name: efm32_chan_halt
 *
 * Description:
 *   Halt the channel associated with 'chidx' by setting the CHannel DISable
 *   (CHDIS) bit in in the HCCHAR register.
 *
 *******************************************************************************/

static void efm32_chan_halt(FAR struct efm32_usbhost_s *priv, int chidx,
                            enum efm32_chreason_e chreason)
{
  uint32_t hcchar;
  uint32_t intmsk;
  uint32_t eptype;
  unsigned int avail;

  /* Save the reason for the halt.  We need this in the channel halt interrrupt
   * handling logic to know what to do next.
   */

  usbhost_vtrace2(OTGFS_VTRACE2_CHANHALT, chidx, chreason);

  priv->chan[chidx].chreason = (uint8_t)chreason;

  /* "The application can disable any channel by programming the OTG_FS_HCCHARx
   *  register with the CHDIS and CHENA bits set to 1. This enables the OTG_FS
   *  host to flush the posted requests (if any) and generates a channel halted
   *  interrupt. The application must wait for the CHH interrupt in OTG_FS_HCINTx
   *  before reallocating the channel for other transactions.  The OTG_FS host
   *  does not interrupt the transaction that has already been started on the
   *  USB."
   */

  hcchar  = efm32_getreg(EFM32_USB_HCCHAR(chidx));
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

      avail = efm32_getreg(EFM32_USB_HNPTXSTS) & _USB_HNPTXSTS_NPTXFSAV_MASK;
    }
  else /* if (eptype == USB_HCCHAR_EPTYP_ISOC || eptype == USB_HC_CHAR_EPTYPE_INT) */
    {
      /* Get the number of words available in the non-periodic Tx FIFO. */

      avail = efm32_getreg(EFM32_USB_HPTXSTS) & _USB_HPTXSTS_PTXFSAVL_MASK;
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

  efm32_putreg(EFM32_USB_HCCHAR(chidx), hcchar);
}

/*******************************************************************************
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
 *******************************************************************************/

static int efm32_chan_waitsetup(FAR struct efm32_usbhost_s *priv,
                                FAR struct efm32_chan_s *chan)
{
  irqstate_t flags = irqsave();
  int        ret   = -ENODEV;

  /* Is the device still connected? */

  if (priv->connected)
    {
      /* Yes.. then set waiter to indicate that we expect to be informed when
       * either (1) the device is disconnected, or (2) the transfer completed.
       */

      chan->waiter = true;
      ret          = OK;
    }

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: efm32_chan_wait
 *
 * Description:
 *   Wait for a transfer on a channel to complete.
 *
 * Assumptions:
 *   Called from a normal thread context
 *
 *******************************************************************************/

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

  flags = irqsave();

  /* Loop, testing for an end of transfer conditino.  The channel 'result'
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

      /* sem_wait should succeeed.  But it is possible that we could be
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
 *******************************************************************************/

static void efm32_chan_wakeup(FAR struct efm32_usbhost_s *priv,
                              FAR struct efm32_chan_s *chan)
{
  /* Is the transfer complete? Is there a thread waiting for this transfer
   * to complete?
   */

  if (chan->result != EBUSY && chan->waiter)
    {
      usbhost_vtrace2(chan->in ? OTGFS_VTRACE2_CHANWAKEUP_IN :
                                 OTGFS_VTRACE2_CHANWAKEUP_OUT,
                      chan->epno, chan->result);

      efm32_givesem(&chan->waitsem);
      chan->waiter = false;
    }
}

/*******************************************************************************
 * Name: efm32_transfer_start
 *
 * Description:
 *   Start at transfer on the select IN or OUT channel.
 *
 *******************************************************************************/

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

  usbhost_vtrace2(OTGFS_VTRACE2_STARTTRANSFER, chidx, chan->buflen);

  chan->result   = EBUSY;
  chan->inflight = 0;
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
          usbhost_trace2(USB_TRACE2_CLIP, chidx, chan->buflen);
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

  regval = ((uint32_t)chan->buflen << _USB_HCTSIZ_XFRSIZ_SHIFT) |
           ((uint32_t)npackets << _USB_HCTSIZ_PKTCNT_SHIFT) |
           ((uint32_t)chan->pid << _USB_HCTSIZ_DPID_SHIFT);
  efm32_putreg(EFM32_USB_HCTSIZ(chidx), regval);

  /* Setup the HCCHAR register: Frame oddness and host channel enable */

  regval = efm32_getreg(EFM32_USB_HCCHAR(chidx));

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
  efm32_putreg(EFM32_USB_HCCHAR(chidx), regval);

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
        case USB_EPTYPE_CTRL: /* Non periodic transfer */
        case USB_EPTYPE_BULK:
          {
            /* Read the Non-periodic Tx FIFO status register */

            regval = efm32_getreg(EFM32_USB_HNPTXSTS);
            avail  = ((regval & _USB_HNPTXSTS_NPTXFSAV_MASK) >> _USB_HNPTXSTS_NPTXFSAV_SHIFT) << 2;
          }
          break;

        /* Periodic transfer */

        case USB_EPTYPE_INTR:
        case USB_EPTYPE_ISOC:
          {
            /* Read the Non-periodic Tx FIFO status register */

            regval = efm32_getreg(EFM32_USB_HPTXSTS);
            avail  = ((regval & _USB_HPTXSTS_PTXFSAVL_MASK) >> _USB_HPTXSTS_PTXFSAVL_SHIFT) << 2;
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

/*******************************************************************************
 * Name: efm32_getframe
 *
 * Description:
 *   Get the current frame number.  The frame number (FRNUM) field increments
 *   when a new SOF is transmitted on the USB, and is cleared to 0 when it
 *   reaches 0x3fff.
 *
 *******************************************************************************/

#if 0 /* Not used */
static inline uint16_t efm32_getframe(void)
{
  return (uint16_t)(efm32_getreg(EFM32_USB_HFNUM) & _USB_HFNUM_FRNUM_MASK);
}
#endif

/*******************************************************************************
 * Name: efm32_ctrl_sendsetup
 *
 * Description:
 *   Send an IN/OUT SETUP packet.
 *
 *******************************************************************************/

static int efm32_ctrl_sendsetup(FAR struct efm32_usbhost_s *priv,
                                FAR const struct usb_ctrlreq_s *req)
{
  FAR struct efm32_chan_s *chan;
  uint32_t start;
  uint32_t elapsed;
  int ret;

  /* Loop while the device reports NAK (and a timeout is not exceeded */

  chan  = &priv->chan[priv->ep0out];
  start = clock_systimer();

  do
    {
      /* Send the  SETUP packet */

      chan->pid    = USB_PID_SETUP;
      chan->buffer = (FAR uint8_t *)req;
      chan->buflen = USB_SIZEOF_CTRLREQ;

      /* Set up for the wait BEFORE starting the transfer */

      ret = efm32_chan_waitsetup(priv, chan);
      if (ret != OK)
        {
          usbhost_trace1(USB_TRACE1_DEVDISCONN, 0);
          return ret;
        }

      /* Start the transfer */

      efm32_transfer_start(priv, priv->ep0out);

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
              usbhost_trace1(USB_TRACE1_TRNSFRFAILED, ret);
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

/*******************************************************************************
 * Name: efm32_ctrl_senddata
 *
 * Description:
 *   Send data in the data phase of an OUT control transfer.  Or send status
 *   in the status phase of an IN control transfer
 *
 *******************************************************************************/

static int efm32_ctrl_senddata(FAR struct efm32_usbhost_s *priv,
                               FAR uint8_t *buffer, unsigned int buflen)
{
  FAR struct efm32_chan_s *chan = &priv->chan[priv->ep0out];
  int ret;

  /* Save buffer information */

  chan->buffer = buffer;
  chan->buflen = buflen;

  /* Set the DATA PID */

  if (buflen == 0)
    {
      /* For status OUT stage with buflen == 0, set PID DATA1 */

      chan->outdata1 = true;
    }

  /* Set the Data PID as per the outdata1 boolean */

  chan->pid = chan->outdata1 ? USB_PID_DATA1 : USB_PID_DATA0;

  /* Set up for the wait BEFORE starting the transfer */

  ret = efm32_chan_waitsetup(priv, chan);
  if (ret != OK)
    {
      usbhost_trace1(USB_TRACE1_DEVDISCONN, 0);
      return ret;
    }

  /* Start the transfer */

  efm32_transfer_start(priv, priv->ep0out);

  /* Wait for the transfer to complete and return the result */

  return efm32_chan_wait(priv, chan);
}

/*******************************************************************************
 * Name: efm32_ctrl_recvdata
 *
 * Description:
 *   Receive data in the data phase of an IN control transfer.  Or receive status
 *   in the status phase of an OUT control transfer
 *
 *******************************************************************************/

static int efm32_ctrl_recvdata(FAR struct efm32_usbhost_s *priv,
                               FAR uint8_t *buffer, unsigned int buflen)
{
  FAR struct efm32_chan_s *chan = &priv->chan[priv->ep0in];
  int ret;

  /* Save buffer information */

  chan->pid    = USB_PID_DATA1;
  chan->buffer = buffer;
  chan->buflen = buflen;

  /* Set up for the wait BEFORE starting the transfer */

  ret = efm32_chan_waitsetup(priv, chan);
  if (ret != OK)
    {
      usbhost_trace1(USB_TRACE1_DEVDISCONN, 0);
      return ret;
    }

  /* Start the transfer */

  efm32_transfer_start(priv, priv->ep0in);

  /* Wait for the transfer to complete and return the result */

  return efm32_chan_wait(priv, chan);
}

/*******************************************************************************
 * Name: efm32_in_transfer
 *
 * Description:
 *   Transfer 'buflen' bytes into 'buffer' from an IN channel.
 *
 *******************************************************************************/

static int efm32_in_transfer(FAR struct efm32_usbhost_s *priv, int chidx,
                             FAR uint8_t *buffer, size_t buflen)
{
  FAR struct efm32_chan_s *chan;
  uint32_t start;
  uint32_t elapsed;
  int ret = OK;

  /* Loop until the transfer completes (i.e., buflen is decremented to zero)
   * or a fatal error occurs (any error other than a simple NAK)
   */

  chan         = &priv->chan[chidx];
  chan->buffer = buffer;
  chan->buflen = buflen;

  start = clock_systimer();
  while (chan->buflen > 0)
    {
      /* Set up for the wait BEFORE starting the transfer */

      ret = efm32_chan_waitsetup(priv, chan);
      if (ret != OK)
        {
          usbhost_trace1(USB_TRACE1_DEVDISCONN, 0);
          return ret;
        }

      /* Set up for the transfer based on the direction and the endpoint type */

      switch (chan->eptype)
        {
        default:
        case USB_EPTYPE_CTRL: /* Control */
          {
            /* This kind of transfer on control endpoints other than EP0 are not
             * currently supported
             */

            return -ENOSYS;
          }

        case USB_EPTYPE_ISOC: /* Isochronous */
          {
            /* Set up the IN data PID */

            usbhost_vtrace2(OTGFS_VTRACE2_ISOCIN, chidx, buflen);
            chan->pid = USB_PID_DATA0;
          }
          break;

        case USB_EPTYPE_BULK: /* Bulk */
          {
            /* Setup the IN data PID */

            usbhost_vtrace2(OTGFS_VTRACE2_BULKIN, chidx, buflen);
            chan->pid = chan->indata1 ? USB_PID_DATA1 : USB_PID_DATA0;
          }
          break;

        case USB_EPTYPE_INTR: /* Interrupt */
          {
            /* Setup the IN data PID */

            usbhost_vtrace2(OTGFS_VTRACE2_INTRIN, chidx, buflen);
            chan->pid = chan->indata1 ? USB_PID_DATA1 : USB_PID_DATA0;
          }
          break;
        }

      /* Start the transfer */

      efm32_transfer_start(priv, chidx);

      /* Wait for the transfer to complete and get the result */

      ret = efm32_chan_wait(priv, chan);

      /* EAGAIN indicates that the device NAKed the transfer and we need
       * do try again.  Anything else (success or other errors) will
       * cause use to return
       */

      if (ret != OK)
        {
          usbhost_trace1(USB_TRACE1_TRNSFRFAILED,ret);

          /* Check for a special case:  If (1) the transfer was NAKed and (2)
           * no Tx FIFO empty or Rx FIFO not-empty event occurred, then we
           * should be able to just flush the Rx and Tx FIFOs and try again.
           * We can detect this latter case becasue the then the transfer
           * buffer pointer and buffer size will be unaltered.
           */

          elapsed = clock_systimer() - start;
          if (ret != -EAGAIN ||                       /* Not a NAK condition OR */
              elapsed >= EFM32_DATANAK_DELAY ||       /* Timeout has elapsed OR */
              chan->buflen != buflen)                 /* Data has been partially transferred */
            {
              /* Break out and return the error */

              break;
            }
        }
    }

  return ret;
}

/*******************************************************************************
 * Name: efm32_out_transfer
 *
 * Description:
 *   Transfer the 'buflen' bytes in 'buffer' through an OUT channel.
 *
 *******************************************************************************/

static int efm32_out_transfer(FAR struct efm32_usbhost_s *priv, int chidx,
                              FAR uint8_t *buffer, size_t buflen)
{
  FAR struct efm32_chan_s *chan;
  uint32_t start;
  uint32_t elapsed;
  size_t xfrlen;
  int ret = OK;

  /* Loop until the transfer completes (i.e., buflen is decremented to zero)
   * or a fatal error occurs (any error other than a simple NAK)
   */

  chan         = &priv->chan[chidx];
  start        = clock_systimer();

  while (buflen > 0)
    {
      /* Transfer one packet at a time.  The hardware is capable of queueing
       * multiple OUT packets, but I just haven't figured out how to handle
       * the case where a single OUT packet in the group is NAKed.
       */

      xfrlen       = MIN(chan->maxpacket, buflen);
      chan->buffer = buffer;
      chan->buflen = xfrlen;

      /* Set up for the wait BEFORE starting the transfer */

      ret = efm32_chan_waitsetup(priv, chan);
      if (ret != OK)
        {
          usbhost_trace1(USB_TRACE1_DEVDISCONN,0);
          return ret;
        }

      /* Set up for the transfer based on the direction and the endpoint type */

      switch (chan->eptype)
        {
        default:
        case USB_EPTYPE_CTRL: /* Control */
          {
            /* This kind of transfer on control endpoints other than EP0 are not
             * currently supported
             */

            return -ENOSYS;
          }

        case USB_EPTYPE_ISOC: /* Isochronous */
          {
            /* Set up the OUT data PID */

            usbhost_vtrace2(OTGFS_VTRACE2_ISOCOUT, chidx, buflen);
            chan->pid = USB_PID_DATA0;
          }
          break;

        case USB_EPTYPE_BULK: /* Bulk */
          {
            /* Setup the OUT data PID */

            usbhost_vtrace2(OTGFS_VTRACE2_BULKOUT, chidx, buflen);
            chan->pid = chan->outdata1 ? USB_PID_DATA1 : USB_PID_DATA0;
          }
          break;

        case USB_EPTYPE_INTR: /* Interrupt */
          {
            /* Setup the OUT data PID */

            usbhost_vtrace2(OTGFS_VTRACE2_INTROUT, chidx, buflen);
            chan->pid = chan->outdata1 ? USB_PID_DATA1 : USB_PID_DATA0;

            /* Toggle the OUT data PID for the next transfer */

            chan->outdata1 ^= true;
          }
          break;
        }

      /* Start the transfer */

      efm32_transfer_start(priv, chidx);

      /* Wait for the transfer to complete and get the result */

      ret = efm32_chan_wait(priv, chan);

      /* Handle transfer failures */

      if (ret != OK)
        {
          usbhost_trace1(USB_TRACE1_TRNSFRFAILED,ret);

          /* Check for a special case:  If (1) the transfer was NAKed and (2)
           * no Tx FIFO empty or Rx FIFO not-empty event occurred, then we
           * should be able to just flush the Rx and Tx FIFOs and try again.
           * We can detect this latter case becasue the then the transfer
           * buffer pointer and buffer size will be unaltered.
           */

          elapsed = clock_systimer() - start;
          if (ret != -EAGAIN ||                       /* Not a NAK condition OR */
              elapsed >= EFM32_DATANAK_DELAY ||       /* Timeout has elapsed OR */
              chan->buflen != xfrlen)                 /* Data has been partially transferred */
            {
              /* Break out and return the error */

              break;
            }

          /* Is this flush really necessary? What does the hardware do with the
           * data in the FIFO when the NAK occurs?  Does it discard it?
           */

          efm32_flush_txfifos(USB_GRSTCTL_TXFNUM_FALL);

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
        }
    }

  return ret;
}

/*******************************************************************************
 * Name: efm32_gint_wrpacket
 *
 * Description:
 *   Transfer the 'buflen' bytes in 'buffer' to the Tx FIFO associated with
 *   'chidx' (non-DMA).
 *
 *******************************************************************************/

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

  fifo = EFM32_USB_DFIFO_HCH(chidx);

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

/*******************************************************************************
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
 *******************************************************************************/

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
  ullvdbg("HCINTMSK%d: %08x pending: %08x\n", chidx, regval, pending);

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
      /* Halt the channel -- the CHH interrrupt is expected next */

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

      if (chan->eptype == USB_EPTYPE_CTRL || chan->eptype == USB_EPTYPE_BULK)
        {
          /* Halt the channel -- the CHH interrrupt is expected next */

          efm32_chan_halt(priv, chidx, CHREASON_XFRC);

          /* Clear any pending NAK condition.  The 'indata1' data toggle
           * should have been appropriately updated by the RxFIFO
           * logic as each packet was received.
           */

          efm32_putreg(EFM32_USB_HCn_INT(chidx), USB_HC_INT_NAK);
        }
      else if (chan->eptype == USB_EPTYPE_INTR)
        {
          /* Force the next transfer on an ODD frame */

          regval = efm32_getreg(EFM32_USB_HCCHAR(chidx));
          regval |= USB_HC_CHAR_ODDFRM;
          efm32_putreg(EFM32_USB_HCCHAR(chidx), regval);

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

          regval = efm32_getreg(EFM32_USB_HCCHAR(chidx));
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
       * return control to high level logic in the even of a NAK.
       */

#if 1
      /* Halt the interrupt channel */

      if (chan->eptype == USB_EPTYPE_INTR)
        {
          /* Halt the channel -- the CHH interrupt is expected next */

          efm32_chan_halt(priv, chidx, CHREASON_NAK);
        }

      /* Re-activate CTRL and BULK channels */

      else if (chan->eptype == USB_EPTYPE_CTRL ||
               chan->eptype == USB_EPTYPE_BULK)
        {
          /* Re-activate the channel by clearing CHDIS and assuring that
           * CHENA is set
           */

          regval  = efm32_getreg(EFM32_USB_HCCHAR(chidx));
          regval |= USB_HC_CHAR_CHENA;
          regval &= ~USB_HC_CHAR_CHDIS;
          efm32_putreg(EFM32_USB_HCCHAR(chidx), regval);
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

/*******************************************************************************
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
 *******************************************************************************/

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
  ullvdbg("HCINTMSK%d: %08x pending: %08x\n", chidx, regval, pending);

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
      priv->chan[chidx].buflen  -= priv->chan[chidx].inflight;
      priv->chan[chidx].inflight = 0;

      /* Halt the channel -- the CHH interrrupt is expected next */

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
      /* Halt the channel  -- the CHH interrrupt is expected next */

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

          regval = efm32_getreg(EFM32_USB_HCCHAR(chidx));

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

/*******************************************************************************
 * Name: efm32_gint_connected
 *
 * Description:
 *   Handle a connection event.
 *
 *******************************************************************************/

static void efm32_gint_connected(FAR struct efm32_usbhost_s *priv)
{
  /* We we previously disconnected? */

  if (!priv->connected)
    {
      /* Yes.. then now we are connected */

      usbhost_vtrace1(OTGFS_VTRACE1_CONNECTED,0);
      priv->connected = true;
      DEBUGASSERT(priv->smstate == SMSTATE_DETACHED);

      /* Notify any waiters */

      priv->smstate = SMSTATE_ATTACHED;
      if (priv->eventwait)
        {
          efm32_givesem(&priv->eventsem);
          priv->eventwait = false;
        }
    }
}

/*******************************************************************************
 * Name: efm32_gint_disconnected
 *
 * Description:
 *   Handle a disconnection event.
 *
 *******************************************************************************/

static void efm32_gint_disconnected(FAR struct efm32_usbhost_s *priv)
{
  /* Were we previously connected? */

  if (priv->connected)
    {
      /* Yes.. then we no longer connected */

      usbhost_vtrace1(OTGFS_VTRACE1_DISCONNECTED,0);

      /* Are we bound to a class driver? */

      if (priv->class)
        {
          /* Yes.. Disconnect the class driver */

          CLASS_DISCONNECTED(priv->class);
          priv->class = NULL;
        }

      /* Re-Initilaize Host for new Enumeration */

      priv->smstate   = SMSTATE_DETACHED;
      priv->ep0size   = EFM32_EP0_MAX_PACKET_SIZE;
      priv->devaddr   = EFM32_DEF_DEVADDR;
      priv->connected = false;
      priv->lowspeed  = false;
      efm32_chan_freeall(priv);

    /* Notify any waiters that there is a change in the connection state */

     if (priv->eventwait)
        {
          efm32_givesem(&priv->eventsem);
          priv->eventwait = false;
        }
    }
}

/*******************************************************************************
 * Name: efm32_gint_sofisr
 *
 * Description:
 *   USB OTG FS start-of-frame interrupt handler
 *
 *******************************************************************************/

#ifdef CONFIG_EFM32_OTGFS_SOFINTR
static inline void efm32_gint_sofisr(FAR struct efm32_usbhost_s *priv)
{
  /* Handle SOF interrupt */
#warning "Do what?"

  /* Clear pending SOF interrupt */

  efm32_putreg(EFM32_USB_GINTSTS, USB_GINTSTS_SOF);
}
#endif

/*******************************************************************************
 * Name: efm32_gint_rxflvlisr
 *
 * Description:
 *   USB OTG FS RxFIFO non-empty interrupt handler
 *
 *******************************************************************************/

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
  intmsk &= ~USB_GINTMSK_PTXFEMPMSKRXFLVLMSK;
  efm32_putreg(EFM32_USB_GINTMSK, intmsk);

  /* Read and pop the next status from the Rx FIFO */

  grxsts = efm32_getreg(EFM32_USB_GRXSTSP);
  ullvdbg("GRXSTS: %08x\n", grxsts);

  /* Isolate the channel number/index in the status word */

  chidx = (grxsts & _USB_GRXSTSP_CHEPNUM_MASK) >> _USB_GRXSTSP_CHEPNUM_SHIFT;

  /* Get the host channel characteristics register (HCCHAR) for this channel */

  hcchar = efm32_getreg(EFM32_USB_HCCHAR(chidx));

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
            fifo   = EFM32_USB_DFIFO_HCH(0);
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
            priv->chan[chidx].buflen -= bcnt;

            /* Check if more packets are expected */

            hctsiz = efm32_getreg(EFM32_USB_HCTSIZ(chidx));
            if ((hctsiz & _USB_HCTSIZ_PKTCNT_MASK) != 0)
              {
                /* Re-activate the channel when more packets are expected */

                hcchar |= USB_HC_CHAR_CHENA;
                hcchar &= ~USB_HC_CHAR_CHDIS;
                efm32_putreg(EFM32_USB_HCCHAR(chidx), hcchar);
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

  intmsk |= USB_GINTMSK_PTXFEMPMSKRXFLVLMSK;
  efm32_putreg(EFM32_USB_GINTMSK, intmsk);
}

/*******************************************************************************
 * Name: efm32_gint_nptxfeisr
 *
 * Description:
 *   USB OTG FS non-periodic TxFIFO empty interrupt handler
 *
 *******************************************************************************/

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
  chan->buflen  -= chan->inflight;
  chan->inflight = 0;

  /* If we have now transfered the entire buffer, then this transfer is
   * complete (this case really should never happen because we disable
   * the NPTXFE interrupt on the final packet).
   */

  if (chan->buflen <= 0)
    {
      /* Disable further Tx FIFO empty interrupts and bail. */

      efm32_modifyreg(EFM32_USB_GINTMSK, USB_GINTMSK_PTXFEMPMSKNPTXFEMPMSK, 0);
      return;
    }

  /* Read the status from the top of the non-periodic TxFIFO */

  regval = efm32_getreg(EFM32_USB_HNPTXSTS);

  /* Extract the number of bytes available in the non-periodic Tx FIFO. */

  avail = ((regval & _USB_HNPTXSTS_NPTXFSAV_MASK) >> _USB_HNPTXSTS_NPTXFSAV_SHIFT) << 2;

  /* Get minimal size packet that can be sent.  Something is seriously
   * configured wrong if one packet will not fit into the empty Tx FIFO.
   */

  DEBUGASSERT(chan->buflen > 0 &&
              avail >= MIN(chan->buflen, chan->maxpacket));

  /* Get the size to put in the Tx FIFO now */

  wrsize = chan->buflen;
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
      efm32_modifyreg(EFM32_USB_GINTMSK, USB_GINTMSK_PTXFEMPMSKNPTXFEMPMSK, 0);
    }

  /* Write the next group of packets into the Tx FIFO */

  ullvdbg("HNPTXSTS: %08x chidx: %d avail: %d buflen: %d wrsize: %d\n",
           regval, chidx, avail, chan->buflen, wrsize);

  efm32_gint_wrpacket(priv, chan->buffer, chidx, wrsize);
}

/*******************************************************************************
 * Name: efm32_gint_ptxfeisr
 *
 * Description:
 *   USB OTG FS periodic TxFIFO empty interrupt handler
 *
 *******************************************************************************/

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
  chan->buflen  -= chan->inflight;
  chan->inflight = 0;

  /* If we have now transfered the entire buffer, then this transfer is
   * complete (this case really should never happen because we disable
   * the PTXFE interrupt on the final packet).
   */

  if (chan->buflen <= 0)
    {
      /* Disable further Tx FIFO empty interrupts and bail. */

      efm32_modifyreg(EFM32_USB_GINTMSK, USB_GINTMSK_PTXFEMPMSKPTXFEMPMSK, 0);
      return;
    }

  /* Read the status from the top of the periodic TxFIFO */

  regval = efm32_getreg(EFM32_USB_HPTXSTS);

  /* Extract the number of bytes available in the periodic Tx FIFO. */

  avail = ((regval & _USB_HPTXSTS_PTXFSAVL_MASK) >> _USB_HPTXSTS_PTXFSAVL_SHIFT) << 2;

  /* Get minimal size packet that can be sent.  Something is seriously
   * configured wrong if one packet will not fit into the empty Tx FIFO.
   */

  DEBUGASSERT(chan->buflen > 0 &&
              avail >= MIN(chan->buflen, chan->maxpacket));

  /* Get the size to put in the Tx FIFO now */

  wrsize = chan->buflen;
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
      efm32_modifyreg(EFM32_USB_GINTMSK, USB_GINTMSK_PTXFEMPMSKPTXFEMPMSK, 0);
    }

  /* Write the next group of packets into the Tx FIFO */

  ullvdbg("HPTXSTS: %08x chidx: %d avail: %d buflen: %d wrsize: %d\n",
           regval, chidx, avail, chan->buflen, wrsize);

  efm32_gint_wrpacket(priv, chan->buffer, chidx, wrsize);
}

/*******************************************************************************
 * Name: efm32_gint_hcisr
 *
 * Description:
 *   USB OTG FS host channels interrupt handler
 *
 *******************************************************************************/

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

          hcchar = efm32_getreg(EFM32_USB_HCCHAR(i));

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

/*******************************************************************************
 * Name: efm32_gint_hprtisr
 *
 * Description:
 *   USB OTG FS host port interrupt handler
 *
 *******************************************************************************/

static inline void efm32_gint_hprtisr(FAR struct efm32_usbhost_s *priv)
{
  uint32_t hprt;
  uint32_t newhprt;
  uint32_t hcfg;

  usbhost_vtrace1(OTGFS_VTRACE1_GINT_HPRT, 0);

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

      usbhost_vtrace1(OTGFS_VTRACE1_GINT_HPRT_POCCHNG, 0);
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

      usbhost_vtrace1(OTGFS_VTRACE1_GINT_HPRT_PCDET, 0);
      newhprt |= USB_HPRT_PRTCONNDET;
      efm32_portreset(priv);
      efm32_gint_connected(priv);
    }

  /* Check for Port Enable CHaNGed (PENCHNG) */

  if ((hprt & USB_HPRT_PRTENCHNG) != 0)
    {
      /* Set up to clear the PENCHNG status in the new HPRT contents. */

      usbhost_vtrace1(OTGFS_VTRACE1_GINT_HPRT_PENCHNG, 0);
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

              usbhost_vtrace1(OTGFS_VTRACE1_GINT_HPRT_LSDEV, 0);
              efm32_putreg(EFM32_USB_HFIR, 6000);

              /* Are we switching from FS to LS? */

              if ((hcfg & _USB_HCFG_FSLSPCS_MASK) != USB_HCFG_FSLSPCS_LS6MHz)
                {

                  usbhost_vtrace1(OTGFS_VTRACE1_GINT_HPRT_FSLSSW, 0);
                  /* Yes... configure for LS */

                  hcfg &= ~_USB_HCFG_FSLSPCS_MASK;
                  hcfg |= USB_HCFG_FSLSPCS_LS6MHz;
                  efm32_putreg(EFM32_USB_HCFG, hcfg);

                  /* And reset the port */

                  efm32_portreset(priv);
                }
            }
          else /* if ((hprt & _USB_HPRT_PRTSPD_MASK) == USB_HPRT_PSPD_FS) */
            {

              usbhost_vtrace1(OTGFS_VTRACE1_GINT_HPRT_FSDEV, 0);
              efm32_putreg(EFM32_USB_HFIR, 48000);

              /* Are we switching from LS to FS? */

              if ((hcfg & _USB_HCFG_FSLSPCS_MASK) != USB_HCFG_FSLSPCS_FS48MHz)
                {

                  usbhost_vtrace1(OTGFS_VTRACE1_GINT_HPRT_LSFSSW, 0);
                  /* Yes... configure for FS */

                  hcfg &= ~_USB_HCFG_FSLSPCS_MASK;
                  hcfg |= USB_HCFG_FSLSPCS_FS48MHz;
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

/*******************************************************************************
 * Name: efm32_gint_discisr
 *
 * Description:
 *   USB OTG FS disconnect detected interrupt handler
 *
 *******************************************************************************/

static inline void efm32_gint_discisr(FAR struct efm32_usbhost_s *priv)
{
  /* Handle the disconnection event */

  efm32_gint_disconnected(priv);

  /* Clear the dicsonnect interrupt */

  efm32_putreg(EFM32_USB_GINTSTS, USB_GINTSTS_DISCONNINT);
}

/*******************************************************************************
 * Name: efm32_gint_ipxfrisr
 *
 * Description:
 *   USB OTG FS incomplete periodic interrupt handler
 *
 *******************************************************************************/

static inline void efm32_gint_ipxfrisr(FAR struct efm32_usbhost_s *priv)
{
  uint32_t regval;

  /* CHENA : Set to enable the channel
   * CHDIS : Set to stop transmitting/receiving data on a channel
   */

  regval = efm32_getreg(EFM32_USB_HCCHAR(0));
  regval |= (USB_HC_CHAR_CHDIS | USB_HC_CHAR_CHENA);
  efm32_putreg(EFM32_USB_HCCHAR(0), regval);

  /* Clear the incomplete isochronous OUT interrupt */

  efm32_putreg(EFM32_USB_GINTSTS, USB_GINTSTS_INCOMPLP);
}

/*******************************************************************************
 * Name: efm32_gint_isr
 *
 * Description:
 *   USB OTG FS global interrupt handler
 *
 *******************************************************************************/

static int efm32_gint_isr(int irq, FAR void *context)
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

  for (;;)
    {
      /* Get the unmasked bits in the GINT status */

      pending  = efm32_getreg(EFM32_USB_GINTSTS);
      pending &= efm32_getreg(EFM32_USB_GINTMSK);

      /* Return from the interrupt when there are no furhter pending
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
          usbhost_vtrace1(OTGFS_VTRACE1_GINT_SOF, 0);
          efm32_gint_sofisr(priv);
        }
#endif

      /* Handle the RxFIFO non-empty interrupt */

      if ((pending & USB_GINTSTS_RXFLVL) != 0)
        {
          usbhost_vtrace1(OTGFS_VTRACE1_GINT_RXFLVL, 0);
          efm32_gint_rxflvlisr(priv);
        }

      /* Handle the non-periodic TxFIFO empty interrupt */

      if ((pending & USB_GINTSTS_NPTXFEMP) != 0)
        {
          usbhost_vtrace1(OTGFS_VTRACE1_GINT_NPTXFE, 0);
          efm32_gint_nptxfeisr(priv);
        }

      /* Handle the periodic TxFIFO empty interrupt */

      if ((pending & USB_GINTSTS_PTXFEMP) != 0)
        {
          usbhost_vtrace1(OTGFS_VTRACE1_GINT_PTXFE, 0);
          efm32_gint_ptxfeisr(priv);
        }

      /* Handle the host channels interrupt */

      if ((pending & USB_GINTSTS_HCHINT) != 0)
        {
          usbhost_vtrace1(OTGFS_VTRACE1_GINT_HC, 0);
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
          usbhost_vtrace1(OTGFS_VTRACE1_GINT_DISC, 0);
          efm32_gint_discisr(priv);
        }

      /* Handle the incomplete periodic transfer */

      if ((pending & USB_GINTSTS_INCOMPLP) != 0)
        {
          usbhost_vtrace1(OTGFS_VTRACE1_GINT_IPXFR, 0);
          efm32_gint_ipxfrisr(priv);
        }
    }

  /* We won't get here */

  return OK;
}

/*******************************************************************************
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
 *******************************************************************************/

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

/*******************************************************************************
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
 *******************************************************************************/

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
  regval |= (USB_GINTMSK_SOFMSK    | USB_GINTMSK_RXFLVLMSK | USB_GINTMSK_INCOMPLPMSK |
             USB_GINTMSK_PRTINTMSK | USB_GINTMSK_HCHINTMSK | USB_GINTMSK_DISCONNINTMSK);
#else
  regval |= (USB_GINTMSK_RXFLVLMSK | USB_GINTMSK_IPXFR     | USB_GINTMSK_PRTINTMSK |
             USB_GINTMSK_HCHINTMSK | USB_GINTMSK_DISCONNINTMSK);
#endif
  efm32_putreg(EFM32_USB_GINTMSK, regval);
}

/*******************************************************************************
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
 *******************************************************************************/

static void efm32_txfe_enable(FAR struct efm32_usbhost_s *priv, int chidx)
{
  FAR struct efm32_chan_s *chan = &priv->chan[chidx];
  irqstate_t flags;
  uint32_t regval;

  /* Disable all interrupts so that we have exclusive access to the GINTMSK
   * (it would be sufficent just to disable the GINT interrupt).
   */

  flags = irqsave();

  /* Should we enable the periodic or non-peridic Tx FIFO empty interrupts */

  regval = efm32_getreg(EFM32_USB_GINTMSK);
  switch (chan->eptype)
    {
    default:
    case USB_EPTYPE_CTRL: /* Non periodic transfer */
    case USB_EPTYPE_BULK:
      regval |= USB_GINTMSK_NPTXFEMPMSK;
      break;

    case USB_EPTYPE_INTR: /* Periodic transfer */
    case USB_EPTYPE_ISOC:
      regval |= USB_GINTMSK_PTXFEMPMSK;
      break;
    }

  /* Enable interrupts */

  efm32_putreg(EFM32_USB_GINTMSK, regval);
  irqrestore(flags);
}

/*******************************************************************************
 * USB Host Controller Operations
 *******************************************************************************/

/*******************************************************************************
 * Name: efm32_wait
 *
 * Description:
 *   Wait for a device to be connected or disconneced.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from the call to
 *      the USB driver initialization logic.
 *   connected - A pointer to a boolean value.  TRUE: Wait for device to be
 *      connected; FALSE: wait for device to be disconnected
 *
 * Returned Values:
 *   Zero (OK) is returned when a device in connected. This function will not
 *   return until either (1) a device is connected or (2) some failure occurs.
 *   On a failure, a negated errno value is returned indicating the nature of
 *   the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int efm32_wait(FAR struct usbhost_connection_s *conn,
                      FAR const bool *connected)
{
  FAR struct efm32_usbhost_s *priv = &g_usbhost;
  irqstate_t flags;

  /* Are we already connected? */

  flags = irqsave();
  while (priv->connected == *connected)
    {
      /* No... wait for the connection/disconnection */

      priv->eventwait = true;
      efm32_takesem(&priv->eventsem);
    }

  irqrestore(flags);

  udbg("Connected:%s\n", priv->connected ? "YES" : "NO");
  return OK;
}

/*******************************************************************************
 * Name: efm32_enumerate
 *
 * Description:
 *   Enumerate the connected device.  As part of this enumeration process,
 *   the driver will (1) get the device's configuration descriptor, (2)
 *   extract the class ID info from the configuration descriptor, (3) call
 *   usbhost_findclass() to find the class that supports this device, (4)
 *   call the create() method on the struct usbhost_registry_s interface
 *   to get a class instance, and finally (5) call the configdesc() method
 *   of the struct usbhost_class_s interface.  After that, the class is in
 *   charge of the sequence of operations.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from the call to
 *      the USB driver initialization logic.
 *   rphndx - Root hub port index.  0-(n-1) corresponds to root hub port 1-n.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int efm32_enumerate(FAR struct usbhost_connection_s *conn, int rhpndx)
{
  FAR struct efm32_usbhost_s *priv = &g_usbhost;
  uint32_t regval;
  int chidx;
  int ret;

  DEBUGASSERT(priv && rhpndx == 0);

  /* Are we connected to a device?  The caller should have called the wait()
   * method first to be assured that a device is connected.
   */

  while (!priv->connected)
    {
      /* No, return an error */

      usbhost_trace1(USB_TRACE1_DEVDISCONN,0);
      return -ENODEV;
    }

  DEBUGASSERT(priv->smstate == SMSTATE_ATTACHED);

  /* Allocate and initialize the control OUT channel */

  chidx = efm32_chan_alloc(priv);
  DEBUGASSERT(chidx >= 0);

  priv->ep0out                = chidx;
  priv->chan[chidx].epno      = 0;
  priv->chan[chidx].in        = false;
  priv->chan[chidx].eptype    = USB_EPTYPE_CTRL;
  priv->chan[chidx].maxpacket = EFM32_EP0_DEF_PACKET_SIZE;
  priv->chan[chidx].indata1   = false;
  priv->chan[chidx].outdata1  = false;

  /* Allocate and initialize the control IN channel */

  chidx = efm32_chan_alloc(priv);
  DEBUGASSERT(chidx >= 0);

  priv->ep0in                 = chidx;
  priv->chan[chidx].epno      = 0;
  priv->chan[chidx].in        = true;
  priv->chan[chidx].eptype    = USB_EPTYPE_CTRL;
  priv->chan[chidx].maxpacket = EFM32_EP0_DEF_PACKET_SIZE;
  priv->chan[chidx].indata1   = false;
  priv->chan[chidx].outdata1  = false;

  /* USB 2.0 spec says at least 50ms delay before port reset.  We wait 100ms. */

  usleep(100*1000);

  /* Reset the host port */

  efm32_portreset(priv);

  /* Get the current device speed */

  regval = efm32_getreg(EFM32_USB_HPRT);
  priv->lowspeed = ((regval & _USB_HPRT_PRTSPD_MASK) == USB_HPRT_PRTSPD_LS);

  /* Configure control channels */

  efm32_chan_configure(priv, priv->ep0out);
  efm32_chan_configure(priv, priv->ep0in);

  /* Let the common usbhost_enumerate do all of the real work.  Note that the
   * FunctionAddress (USB address) is hardcoded to one.
   */

  uvdbg("Enumerate the device\n");
  priv->smstate = SMSTATE_ENUM;
  ret = usbhost_enumerate(&g_usbhost.drvr, 1, &priv->class);

  /* The enumeration may fail either because of some HCD interfaces failure
   * or because the device class is not supported.  In either case, we just
   * need to perform the disconnection operation and make ready for a new
   * enumeration.
   */

  if (ret < 0)
    {
      /* Return to the disconnected state */

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
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
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

static int efm32_ep0configure(FAR struct usbhost_driver_s *drvr, uint8_t funcaddr,
                              uint16_t maxpacketsize)
{
  FAR struct efm32_usbhost_s *priv = (FAR struct efm32_usbhost_s *)drvr;

  DEBUGASSERT(drvr && funcaddr < 128 && maxpacketsize < 2048);

  /* We must have exclusive access to the USB host hardware and state structures */

  efm32_takesem(&priv->exclsem);

  /* Save the device address and EP0 max packet size */

  priv->devaddr = funcaddr;
  priv->ep0size = maxpacketsize;

  /* Configure the EP0 OUT channel */

  priv->chan[priv->ep0out].maxpacket = maxpacketsize;
  efm32_chan_configure(priv, priv->ep0out);

  /* Configure the EP0 IN channel */

  priv->chan[priv->ep0in].maxpacket = maxpacketsize;
  efm32_chan_configure(priv, priv->ep0in);

  efm32_givesem(&priv->exclsem);
  return OK;
}

/************************************************************************************
 * Name: efm32_getdevinfo
 *
 * Description:
 *   Get information about the connected device.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   devinfo - A pointer to memory provided by the caller in which to return the
 *      device information.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int efm32_getdevinfo(FAR struct usbhost_driver_s *drvr,
                            FAR struct usbhost_devinfo_s *devinfo)
{
  FAR struct efm32_usbhost_s *priv = (FAR struct efm32_usbhost_s *)drvr;

  DEBUGASSERT(drvr && devinfo);
  devinfo->speed = priv->lowspeed ? DEVINFO_SPEED_LOW : DEVINFO_SPEED_FULL;
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
 * Returned Values:
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
  FAR struct efm32_chan_s *chan;
  int chidx;
  int ret;

  /* Sanity check.  NOTE that this method should only be called if a device is
   * connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(priv && epdesc && ep && priv->connected);

  /* We must have exclusive access to the USB host hardware and state structures */

  efm32_takesem(&priv->exclsem);

  /* Allocate a host channel for the endpoint */

  chidx = efm32_chan_alloc(priv);
  if (chidx < 0)
    {
      udbg("Failed to allocate a host channel\n");
      ret = -ENOMEM;
      goto errout;
    }

  /* Decode the endpoint descriptor to initialize the channel data structures.
   * Note:  Here we depend on the fact that the endpoint point type is
   * encoded in the same way in the endpoint descriptor as it is in the OTG
   * FS hardware.
   */

  chan = &priv->chan[chidx];
  chan->epno      = epdesc->addr & USB_EPNO_MASK;
  chan->in        = epdesc->in;
  chan->eptype    = epdesc->xfrtype;
  chan->maxpacket = epdesc->mxpacketsize;
  chan->indata1   = false;
  chan->outdata1  = false;

  /* Then configure the endpoint */

  efm32_chan_configure(priv, chidx);

  /* Return the index to the allocated channel as the endpoint "handle" */

  *ep = (usbhost_ep_t)chidx;
  ret = OK;

errout:
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int efm32_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct efm32_usbhost_s *priv = (struct efm32_usbhost_s *)drvr;
  int chidx = (int)ep;

  DEBUGASSERT(priv && chidx < EFM32_MAX_TX_FIFOS);

  /* We must have exclusive access to the USB host hardware and state structures */

  efm32_takesem(&priv->exclsem);

  /* Halt the channel and mark the channel avaiable */

  efm32_chan_free(priv, chidx);

  efm32_givesem(&priv->exclsem);
  return OK;
}

/*******************************************************************************
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

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

/*******************************************************************************
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
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

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
 * Returned Values:
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
 * Returned Values:
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

/*******************************************************************************
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
 *   req - Describes the request to be sent.  This request must lie in memory
 *      created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *     responses.  This buffer must be large enough to hold the length value
 *     in the request description. buffer must have been allocated using DRVR_ALLOC
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same allocated
 *   memory.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int efm32_ctrlin(FAR struct usbhost_driver_s *drvr,
                        FAR const struct usb_ctrlreq_s *req,
                        FAR uint8_t *buffer)
{
  struct efm32_usbhost_s *priv = (struct efm32_usbhost_s *)drvr;
  uint16_t buflen;
  uint32_t start;
  uint32_t elapsed;
  int retries;
  int ret;

  DEBUGASSERT(drvr && req);
  usbhost_vtrace2(OTGFS_VTRACE2_CTRLIN, req->type, req->req);
  uvdbg("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
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

      ret = efm32_ctrl_sendsetup(priv, req);
      if (ret < 0)
       {
          usbhost_trace1(USB_TRACE1_SENDSETUP, -ret);
          continue;
        }

      /* Get the start time.  Loop again until the timeout expires */

      start = clock_systimer();
      do
        {
          /* Handle the IN data phase (if any) */

          if (buflen > 0)
            {
              ret = efm32_ctrl_recvdata(priv, buffer, buflen);
              if (ret < 0)
                {
                  usbhost_trace1(USB_TRACE1_RECVDATA, -ret);
                }
            }

          /* Handle the status OUT phase */

          if (ret == OK)
            {
              priv->chan[priv->ep0out].outdata1 ^= true;
              ret = efm32_ctrl_senddata(priv, NULL, 0);
              if (ret == OK)
                {
                  /* All success transactions exit here */

                  efm32_givesem(&priv->exclsem);
                  return OK;
                }

              usbhost_trace1(USB_TRACE1_SENDDATA, ret < 0 ? -ret : ret);
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

static int efm32_ctrlout(FAR struct usbhost_driver_s *drvr,
                         FAR const struct usb_ctrlreq_s *req,
                         FAR const uint8_t *buffer)
{
  struct efm32_usbhost_s *priv = (struct efm32_usbhost_s *)drvr;
  uint16_t buflen;
  uint32_t start;
  uint32_t elapsed;
  int retries;
  int ret;

  DEBUGASSERT(drvr && req);
  usbhost_vtrace2(OTGFS_VTRACE2_CTRLOUT, req->type, req->req);
  uvdbg("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
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

      /* Send the SETUP request */

      ret = efm32_ctrl_sendsetup(priv, req);
      if (ret < 0)
        {
          usbhost_trace1(USB_TRACE1_SENDSETUP, -ret);
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

              priv->chan[priv->ep0out].outdata1 = true;
              ret = efm32_ctrl_senddata(priv, NULL, 0);
              if (ret < 0)
                {
                  usbhost_trace1(USB_TRACE1_SENDDATA, -ret);
                }
            }

          /* Handle the status IN phase */

          if (ret == OK)
            {
              ret = efm32_ctrl_recvdata(priv, NULL, 0);
              if (ret == OK)
                {
                  /* All success transactins exit here */

                  efm32_givesem(&priv->exclsem);
                  return OK;
                }

              usbhost_trace1(USB_TRACE1_RECVDATA, ret < 0 ? -ret : ret);
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

/*******************************************************************************
 * Name: efm32_transfer
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately.  Only one transfer may be
 *   queued; Neither this method nor the ctrlin or ctrlout methods can be called
 *   again until this function returns.
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
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure:
 *
 *     EAGAIN - If devices NAKs the transfer (or NYET or other error where
 *              it may be appropriate to restart the entire transaction).
 *     EPERM  - If the endpoint stalls
 *     EIO    - On a TX or data toggle error
 *     EPIPE  - Overrun errors
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int efm32_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                          FAR uint8_t *buffer, size_t buflen)
{
  FAR struct efm32_usbhost_s *priv  = (FAR struct efm32_usbhost_s *)drvr;
  unsigned int chidx = (unsigned int)ep;
  int ret;

  uvdbg("chidx: %d buflen: %d\n",  (unsigned int)ep, buflen);

  DEBUGASSERT(priv && buffer && chidx < EFM32_MAX_TX_FIFOS && buflen > 0);

  /* We must have exclusive access to the USB host hardware and state structures */

  efm32_takesem(&priv->exclsem);

  /* Handle IN and OUT transfer slightly differently */

  if (priv->chan[chidx].in)
    {
      ret = efm32_in_transfer(priv, chidx, buffer, buflen);
    }
  else
    {
      ret = efm32_out_transfer(priv, chidx, buffer, buflen);
    }

  efm32_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
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
 * Returned Values:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static void efm32_disconnect(FAR struct usbhost_driver_s *drvr)
{
  struct efm32_usbhost_s *priv = (struct efm32_usbhost_s *)drvr;
  DEBUGASSERT(priv);

  priv->class = NULL;
}

/*******************************************************************************
 * Initialization
 *******************************************************************************/
/*******************************************************************************
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
 *******************************************************************************/

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

/*******************************************************************************
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
 *******************************************************************************/

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

/*******************************************************************************
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
 *******************************************************************************/

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

/*******************************************************************************
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
 *******************************************************************************/

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

/*******************************************************************************
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
 *******************************************************************************/

static void efm32_host_initialize(FAR struct efm32_usbhost_s *priv)
{
  uint32_t regval;
  uint32_t offset;
  int i;

  /* Restart the PHY Clock */

  efm32_putreg(EFM32_USB_PCGCCTL, 0);

  /* Initialize Host Configuration (HCFG) register */

  regval  = efm32_getreg(EFM32_USB_HCFG);
  regval &= ~_USB_HCFG_FSLSPCS_MASK;
  regval |= USB_HCFG_FSLSPCS_FS48MHz;
  efm32_putreg(EFM32_USB_HCFG, regval);

  /* Reset the host port */

  efm32_portreset(priv);

  /* Clear the FS-/LS-only support bit in the HCFG register */

  regval = efm32_getreg(EFM32_USB_HCFG);
  regval &= ~USB_HCFG_FSLSS;
  efm32_putreg(EFM32_USB_HCFG, regval);

  /* Carve up FIFO memory for the Rx FIFO and the periodic and non-periodic Tx FIFOs */
  /* Configure Rx FIFO size (GRXFSIZ) */

  efm32_putreg(EFM32_USB_GRXFSIZ, CONFIG_EFM32_OTGFS_RXFIFO_SIZE);
  offset = CONFIG_EFM32_OTGFS_RXFIFO_SIZE;

  /* Setup the host non-periodic Tx FIFO size (HNPTXFSIZ) */

  regval = (offset | (CONFIG_EFM32_OTGFS_NPTXFIFO_SIZE << _USB_HNPTXFSIZ_NPTXFD_SHIFT));
  efm32_putreg(EFM32_USB_HNPTXFSIZ, regval);
  offset += CONFIG_EFM32_OTGFS_NPTXFIFO_SIZE;

  /* Set up the host periodic Tx FIFO size register (HPTXFSIZ) */

  regval = (offset | (CONFIG_EFM32_OTGFS_PTXFIFO_SIZE << _USB_HPTXFSIZ_PTXFD_SHIFT));
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

/*******************************************************************************
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
 *******************************************************************************/

static inline void efm32_sw_initialize(FAR struct efm32_usbhost_s *priv)
{
  int i;

  /* Initialize the state data structure */

  sem_init(&priv->eventsem,  0, 0);
  sem_init(&priv->exclsem, 0, 1);

  priv->smstate   = SMSTATE_DETACHED;
  priv->ep0size   = EFM32_EP0_MAX_PACKET_SIZE;
  priv->devaddr   = EFM32_DEF_DEVADDR;
  priv->connected = false;
  priv->lowspeed  = false;

  /* Put all of the channels back in their initial, allocated state */

  memset(priv->chan, 0, EFM32_MAX_TX_FIFOS * sizeof(struct efm32_chan_s));

  /* Initialize each channel */

  for (i = 0; i < EFM32_MAX_TX_FIFOS; i++)
    {
      FAR struct efm32_chan_s *chan = &priv->chan[i];
      sem_init(&chan->waitsem,  0, 0);
    }
}

/*******************************************************************************
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
 *******************************************************************************/

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

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: efm32_otgfshost_initialize
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

FAR struct usbhost_connection_s *efm32_otgfshost_initialize(int controller)
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

  if (irq_attach(EFM32_IRQ_USB, efm32_gint_isr) != 0)
    {
      usbhost_trace1(USB_TRACE1_IRQATTACH, 0);
      return NULL;
    }

  /* Enable USB OTG FS global interrupts */

  efm32_gint_enable();

  /* Enable interrupts at the interrupt controller */

  up_enable_irq(EFM32_IRQ_USB);
  return &g_usbconn;
}

#endif /* CONFIG_USBHOST && CONFIG_EFM32_OTGFS */

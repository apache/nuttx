/****************************************************************************
 * arch/arm/src/sama5/sam_ssc.c
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <queue.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>

#include "up_internal.h"
#include "up_arch.h"
#include "cache.h"

#include "chip.h"
#include "sam_pio.h"
#include "sam_dmac.h"
#include "sam_memories.h"
#include "sam_periphclks.h"
#include "sam_ssc.h"
#include "chip/sam_pmc.h"
#include "chip/sam_ssc.h"
#include "chip/sam_pinmap.h"

#if defined(CONFIG_SAMA5_SSC0) || defined(CONFIG_SAMA5_SSC1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO required by this driver
#endif

#ifndef CONFIG_SAMA5_SSC_MAXINFLIGHT
#  define CONFIG_SAMA5_SSC_MAXINFLIGHT 16
#endif

/* Assume no RX/TX support until we learn better */

#undef SSC_HAVE_RX
#undef SSC_HAVE_TX

/* Check for SSC0 support */

#if defined(CONFIG_SAMA5_SSC0)

#  if defined(CONFIG_SAMA5_HAVE_XDMA)
#    if !defined(CONFIG_SAMA5_XDMAC0) && !defined(CONFIG_SAMA5_XDMAC1)
#      error CONFIG_SAMA5_XDMAC0 or XDMAC1 required by SSC0
#    endif
#  else
#    if !defined(CONFIG_SAMA5_DMAC0)
#      error CONFIG_SAMA5_DMAC0 required by SSC0
#    endif
#  endif

  /* The SSC can handle most any bit width from 2 to 32.  However, the DMA
   * logic here is constrained to byte, half-word, and word sizes.
   */

#  ifndef CONFIG_SAMA5_SSC0_DATALEN
#    define CONFIG_SAMA5_SSC0_DATALEN 16
#  endif

#  if CONFIG_SAMA5_SSC0_DATALEN == 8
#    define SAMA5_SSC0_DATAMASK  0
#  elif CONFIG_SAMA5_SSC0_DATALEN == 16
#    define SAMA5_SSC0_DATAMASK  1
#  elif CONFIG_SAMA5_SSC0_DATALEN == 32
#    define SAMA5_SSC0_DATAMASK  3
#  elif  CONFIG_SAMA5_SSC0_DATALEN < 2 || CONFIG_SAMA5_SSC0_DATALEN > 32
#    error Invalid value for CONFIG_SAMA5_SSC0_DATALEN
#  else
#    error Valid but supported value for CONFIG_SAMA5_SSC0_DATALEN
#  endif

/* Check for SSC0 RX support */

#  if defined(CONFIG_SAMA5_SSC0_RX)
#    define SSC_HAVE_RX 1

#    ifndef CONFIG_SSC0_RX_FSLEN
#      define CONFIG_SSC0_RX_FSLEN 1
#    endif

#    if CONFIG_SSC0_RX_FSLEN < 1 || CONFIG_SSC0_RX_FSLEN > 255
#      error Invalid value for CONFIG_SSC0_RX_FSLEN
#    endif

#    ifndef CONFIG_SSC0_RX_STTDLY
#      define CONFIG_SSC0_RX_STTDLY CONFIG_SSC0_RX_FSLEN
#    endif

#    if CONFIG_SSC0_RX_STTDLY < 0 || \
        CONFIG_SSC0_RX_STTDLY < CONFIG_SSC0_RX_FSLEN || \
        CONFIG_SSC0_RX_STTDLY > 255
#      error Invalid value for CONFIG_SSC0_RX_STTDLY
#    endif
#  endif

/* Check for SSC0 TX support */

#  if defined(CONFIG_SAMA5_SSC0_TX)
#    define SSC_HAVE_TX 1

#    ifndef CONFIG_SSC0_TX_FSLEN
#      define CONFIG_SSC0_TX_FSLEN 0
#    endif

#    if CONFIG_SSC0_TX_FSLEN < 0 || CONFIG_SSC0_TX_FSLEN > 255
#      error Invalid value for CONFIG_SSC0_TX_FSLEN
#    endif

#    ifndef CONFIG_SSC0_TX_STTDLY
#      if CONFIG_SSC0_TX_FSLEN > 0
#        define CONFIG_SSC0_TX_STTDLY CONFIG_SSC0_TX_FSLEN
#      else
#        define CONFIG_SSC0_TX_STTDLY 0
#      endif
#    endif

#    if CONFIG_SSC0_TX_STTDLY < 0 || \
        CONFIG_SSC0_TX_STTDLY < CONFIG_SSC0_TX_FSLEN || \
        CONFIG_SSC0_TX_STTDLY > 255
#      error Invalid value for CONFIG_SSC0_TX_STTDLY
#    endif
#  endif

#endif

/* Check for SSC1 support */

#if defined(CONFIG_SAMA5_SSC1)

#  if defined(CONFIG_SAMA5_HAVE_XDMA)
#    if !defined(CONFIG_SAMA5_XDMAC0) && !defined(CONFIG_SAMA5_XDMAC1)
#      error CONFIG_SAMA5_XDMAC1 (or XDMAC0) required by SSC1
#    endif
#  else
#    if !defined(CONFIG_SAMA5_DMAC1)
#      error CONFIG_SAMA5_DMAC0 required by SSC1
#    endif
#  endif

  /* The SSC can handle most any bit width from 2 to 32.  However, the DMA
   * logic here is constrained to byte, half-word, and word sizes.
   */

#  ifndef CONFIG_SAMA5_SSC1_DATALEN
#    define CONFIG_SAMA5_SSC1_DATALEN 16
#  endif

#  if CONFIG_SAMA5_SSC1_DATALEN == 8
#    define SAMA5_SSC1_DATAMASK  0
#  elif CONFIG_SAMA5_SSC1_DATALEN == 16
#    define SAMA5_SSC1_DATAMASK  1
#  elif CONFIG_SAMA5_SSC1_DATALEN == 32
#    define SAMA5_SSC1_DATAMASK  3
#  elif  CONFIG_SAMA5_SSC1_DATALEN < 2 || CONFIG_SAMA5_SSC1_DATALEN > 32
#    error Invalid value for CONFIG_SAMA5_SSC1_DATALEN
#  else
#    error Valid but supported value for CONFIG_SAMA5_SSC1_DATALEN
#  endif

/* Check for SSC1 RX support */

#  if defined(CONFIG_SAMA5_SSC1_RX)
#    define SSC_HAVE_RX 1

#    ifndef CONFIG_SSC1_RX_FSLEN
#      define CONFIG_SSC1_RX_FSLEN 1
#    endif

#    if CONFIG_SSC1_RX_FSLEN < 1 || CONFIG_SSC1_RX_FSLEN > 255
#      error Invalid value for CONFIG_SSC1_RX_FSLEN
#    endif

#    ifndef CONFIG_SSC1_RX_STTDLY
#      define CONFIG_SSC1_RX_STTDLY CONFIG_SSC1_RX_FSLEN
#    endif

#    if CONFIG_SSC1_RX_STTDLY < 0 || \
        CONFIG_SSC1_RX_STTDLY < CONFIG_SSC1_RX_FSLEN || \
        CONFIG_SSC1_RX_STTDLY > 255
#      error Invalid value for CONFIG_SSC1_RX_STTDLY
#    endif

#  endif

/* Check for SSC1 TX support */

#  if defined(CONFIG_SAMA5_SSC1_TX)
#    define SSC_HAVE_TX 1

#    ifndef CONFIG_SSC1_TX_FSLEN
#      define CONFIG_SSC1_TX_FSLEN 0
#    endif

#    if CONFIG_SSC1_TX_FSLEN < 0 || CONFIG_SSC1_TX_FSLEN > 255
#      error Invalid value for CONFIG_SSC1_TX_FSLEN
#    endif

#    ifndef CONFIG_SSC1_TX_STTDLY
#      if CONFIG_SSC1_TX_FSLEN > 0
#        define CONFIG_SSC1_TX_STTDLY CONFIG_SSC1_TX_FSLEN
#      else
#        define CONFIG_SSC1_TX_STTDLY 0
#      endif
#    endif

#    if CONFIG_SSC1_TX_STTDLY < 0 || \
        CONFIG_SSC1_TX_STTDLY < CONFIG_SSC1_TX_FSLEN || \
        CONFIG_SSC1_TX_STTDLY > 255
#      error Invalid value for CONFIG_SSC1_TX_STTDLY
#    endif
#  endif

#endif

/* Check if we need to build RX and/or TX support */

#if defined(SSC_HAVE_RX) || defined(SSC_HAVE_TX)

/* Check if we need the sample rate to set MCK/2 divider */

#undef SSC_HAVE_MCK2
#undef SSC0_HAVE_MCK2
#undef SSC1_HAVE_MCK2

#if (defined(CONFIG_SAMA5_SSC0_RX) && defined(CONFIG_SAMA5_SSC0_RX_MCKDIV)) || \
    (defined(CONFIG_SAMA5_SSC0_TX) && defined(CONFIG_SAMA5_SSC0_TX_MCKDIV))
#  define SSC0_HAVE_MCK2 1
#endif

#if (defined(CONFIG_SAMA5_SSC1_RX) && defined(CONFIG_SAMA5_SSC1_RX_MCKDIV)) || \
    (defined(CONFIG_SAMA5_SSC1_TX) && defined(CONFIG_SAMA5_SSC1_TX_MCKDIV))
#  define SSC1_HAVE_MCK2 1
#endif

#if defined(SSC0_HAVE_MCK2) || defined(SSC1_HAVE_MCK2)
#  define SSC_HAVE_MCK2 1
#endif

/* Waveform:
 *
 *      |<---------------- PERIOD --------------->|
 *  ----+     +-----------------------------------+    +---
 *      |     |                                   |    |
 *      +-----+                                   +----+
 *      |FSLEN|
 *      |<-STTDLY->|<--DATALEN-->|<--DATALEN-->|  |
 *                 |<-----DATALEN * DATNB----->|
 *
 * TK/RK is assumed to be a negative pulse
 * DATALEN is configurable: CONFIG_SAMA5_SSCx_DATALEN
 * FSLEN is configuration:  CONFIG_SAMA5_SSCx_RX/TX_FSLEN
 * FSLEN and STTDLY are fixed at two clocks
 * DATNB is fixed a one work
 *
 * REVISIT:  These will probably need to be configurable
 */

#define SSC_DATNB       (1) /* Number words per per frame */
#define SCC_PERIOD(s,d) ((s) + (d) * SSC_DATNB)

/* Clocking *****************************************************************/

/* Clock source definitions */

#define SSC_CLKSRC_NONE   0 /* No clock */
#define SSC_CLKSRC_MCKDIV 1 /* Clock source is MCK divided down */
#define SSC_CLKSRC_RXOUT  2 /* Transmitter clock source is the receiver clock */
#define SSC_CLKSRC_TXOUT  2 /* Receiver clock source is the transmitter clock */
#define SSC_CLKSRC_TKIN   3 /* Transmitter clock source is TK */
#define SSC_CLKSRC_RKIN   3 /* Receiver clock source is RK */

/* Clock output definitions */

#define SSC_CLKOUT_NONE   0 /* No output clock */
#define SSC_CLKOUT_CONT   1 /* Continuous */
#define SSC_CLKOUT_XFER   2 /* Only output clock during transfers */

/* Bus configuration differ with chip */

#if defined(ATSAMA5D3)
  /* System bus interfaces */

#  define DMACH_FLAG_PERIPH_IF DMACH_FLAG_PERIPHAHB_AHB_IF2
#  define DMACH_FLAG_MEM_IF    DMACH_FLAG_MEMAHB_AHB_IF0

#elif defined(ATSAMA5D4)
  /* System Bus Interfaces
   *
   * Both SSC0 and SSC1 are APB1.  Both are accessible on MATRIX IF1.
   *
   * Memory is available on either port 5 (IF0 for both XDMAC0 and 1) or
   * port 6 (IF1 for both XDMAC0 and 1).
   */

#  define DMACH_FLAG_PERIPH_IF DMACH_FLAG_PERIPHAHB_AHB_IF1
#  define DMACH_FLAG_MEM_IF    DMACH_FLAG_MEMAHB_AHB_IF0
#endif

/* DMA configuration */

#define DMA8_FLAGS \
  (DMACH_FLAG_PERIPH_IF | DMACH_FLAG_PERIPHH2SEL | \
   DMACH_FLAG_PERIPHISPERIPH | DMACH_FLAG_PERIPHWIDTH_8BITS | \
   DMACH_FLAG_PERIPHCHUNKSIZE_1 | DMACH_FLAG_MEMPID_MAX | \
   DMACH_FLAG_MEM_IF | DMACH_FLAG_MEMWIDTH_16BITS | \
   DMACH_FLAG_MEMINCREMENT | DMACH_FLAG_MEMCHUNKSIZE_1| \
   DMACH_FLAG_MEMBURST_4)

#define DMA16_FLAGS \
  (DMACH_FLAG_PERIPH_IF | DMACH_FLAG_PERIPHH2SEL | \
   DMACH_FLAG_PERIPHISPERIPH | DMACH_FLAG_PERIPHWIDTH_16BITS | \
   DMACH_FLAG_PERIPHCHUNKSIZE_1 | DMACH_FLAG_MEMPID_MAX | \
   DMACH_FLAG_MEM_IF | DMACH_FLAG_MEMWIDTH_16BITS | \
   DMACH_FLAG_MEMINCREMENT | DMACH_FLAG_MEMCHUNKSIZE_1 | \
   DMACH_FLAG_MEMBURST_4)

#define DMA32_FLAGS \
  (DMACH_FLAG_PERIPH_IF | DMACH_FLAG_PERIPHH2SEL | \
   DMACH_FLAG_PERIPHISPERIPH | DMACH_FLAG_PERIPHWIDTH_32BITS | \
   DMACH_FLAG_PERIPHCHUNKSIZE_1 | DMACH_FLAG_MEMPID_MAX | \
   DMACH_FLAG_MEM_IF | DMACH_FLAG_MEMWIDTH_32BITS | \
   DMACH_FLAG_MEMINCREMENT | DMACH_FLAG_MEMCHUNKSIZE_1 | \
   DMACH_FLAG_MEMBURST_4)

/* DMA timeout.  The value is not critical; we just don't want the system to
 * hang in the event that a DMA does not finish.  This is set to
 */

#define DMA_TIMEOUT_MS    (800)
#define DMA_TIMEOUT_TICKS MSEC2TICK(DMA_TIMEOUT_MS)

/* Debug *******************************************************************/
/* Check if SSC debut is enabled (non-standard.. no support in
 * include/debug.h
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_I2S
#endif

#ifndef CONFIG_DEBUG_I2S
#  undef CONFIG_SAMA5_SSC_DMADEBUG
#  undef CONFIG_SAMA5_SSC_REGDEBUG
#  undef CONFIG_SAMA5_SSC_QDEBUG
#  undef CONFIG_SAMA5_SSC_DUMPBUFFERS
#endif

#ifndef CONFIG_DEBUG_DMA
#  undef CONFIG_SAMA5_SSC_DMADEBUG
#endif

#ifdef CONFIG_DEBUG_I2S
#  define i2sdbg         dbg
#  define i2slldbg       lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define i2svdbg      dbg
#    define i2sllvdbg    lldbg
#  else
#    define i2svdbg(x...)
#  endif
#else
#  define i2sdbg(x...)
#  define i2slldbg(x...)
#  define i2svdbg(x...)
#  define i2sllvdbg(x...)
#endif

#define DMA_INITIAL      0
#define DMA_AFTER_SETUP  1
#define DMA_AFTER_START  2
#define DMA_CALLBACK     3
#define DMA_TIMEOUT      3
#define DMA_END_TRANSFER 4
#define DMA_NSAMPLES     5

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* I2S buffer container */

struct sam_buffer_s
{
  struct sam_buffer_s *flink;  /* Supports a singly linked list */
  i2s_callback_t callback;     /* Function to call when the transfer completes */
  uint32_t timeout;            /* The timeout value to use with DMA transfers */
  void *arg;                   /* The argument to be returned with the callback */
  struct ap_buffer_s *apb;     /* The audio buffer */
  int result;                  /* The result of the transfer */
};

/* This structure describes the state of one receiver or transmitter transport */

struct sam_transport_s
{
  DMA_HANDLE dma;             /* SSC DMA handle */
  WDOG_ID dog;                /* Watchdog that handles DMA timeouts */
  sq_queue_t pend;            /* A queue of pending transfers */
  sq_queue_t act;             /* A queue of active transfers */
  sq_queue_t done;            /* A queue of completed transfers */
  struct work_s work;         /* Supports worker thread operations */

#ifdef CONFIG_SAMA5_SSC_DMADEBUG
  struct sam_dmaregs_s dmaregs[DMA_NSAMPLES];
#endif
};

/* The state of the one SSC peripheral */

struct sam_ssc_s
{
  struct i2s_dev_s dev;        /* Externally visible I2S interface */
  uintptr_t base;              /* SSC controller register base address */
  sem_t exclsem;               /* Assures mutually exclusive acess to SSC */
  uint8_t datalen;             /* Data width (8, 16, or 32) */
#ifdef CONFIG_DEBUG
  uint8_t align;               /* Log2 of data width (0, 1, or 3) */
#endif
  uint8_t pid;                 /* Peripheral ID */
  uint8_t rxfslen;             /* RX frame sync length */
  uint8_t txfslen;             /* TX frame sync length */
  uint8_t rxsttdly;            /* RX start delay */
  uint8_t txsttdly;            /* TX start delay */
  uint8_t rxenab:1;            /* True: RX transfers enabled */
  uint8_t txenab:1;            /* True: TX transfers enabled */
  uint8_t loopback:1;          /* True: Loopback mode */
  uint8_t sscno:1;             /* SSC controller number (0 or 1) */
  uint8_t rxclk:2;             /* Receiver clock source. See SSC_CLKSRC_* definitions */
  uint8_t txclk:2;             /* Transmitter clock source. See SSC_CLKSRC_* definitions */
  uint8_t rxout:2;             /* Receiver clock output. See SSC_CLKOUT_* definitions */
  uint8_t txout:2;             /* Transmitter clock output. See SSC_CLKOUT_* definitions */
  uint32_t frequency;          /* SSC clock frequency */
#ifdef SSC_HAVE_MCK2
  uint32_t samplerate;         /* Data sample rate (determines only MCK/2 divider) */
#endif

#ifdef SSC_HAVE_RX
  struct sam_transport_s rx;   /* RX transport state */
#endif
#ifdef SSC_HAVE_TX
  struct sam_transport_s tx;   /* TX transport state */
#endif

  /* Pre-allocated pool of buffer containers */

  sem_t bufsem;                   /* Buffer wait semaphore */
  struct sam_buffer_s *freelist;  /* A list a free buffer containers */
  struct sam_buffer_s containers[CONFIG_SAMA5_SSC_MAXINFLIGHT];

  /* Debug stuff */

#ifdef CONFIG_SAMA5_SSC_REGDEBUG
   bool     wr;                /* Last was a write */
   uint32_t regaddr;           /* Last address */
   uint32_t regval;            /* Last value */
   int      count;             /* Number of times */
#endif /* CONFIG_SAMA5_SSC_REGDEBUG */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register helpers */

#ifdef CONFIG_SAMA5_SSC_REGDEBUG
static bool     ssc_checkreg(struct sam_ssc_s *priv, bool wr, uint32_t regval,
                  uint32_t regaddr);
#else
# define        ssc_checkreg(priv,wr,regval,regaddr) (false)
#endif

static inline uint32_t ssc_getreg(struct sam_ssc_s *priv, unsigned int offset);
static inline void ssc_putreg(struct sam_ssc_s *priv, unsigned int offset,
                  uint32_t regval);
static inline uintptr_t ssc_physregaddr(struct sam_ssc_s *priv,
                  unsigned int offset);

#if defined(CONFIG_DEBUG_I2S) && defined(CONFIG_DEBUG_VERBOSE)
static void     scc_dump_regs(struct sam_ssc_s *priv, const char *msg);
#else
#  define       scc_dump_regs(s,m)
#endif

#ifdef CONFIG_SAMA5_SSC_QDEBUG
static void     ssc_dump_queues(struct sam_transport_s *xpt,
                  const char *msg);
#  define       ssc_dump_rxqueues(s,m) ssc_dump_queues(&(s)->rx,m)
#  define       ssc_dump_txqueues(s,m) ssc_dump_queues(&(s)->tx,m)
#else
#  define       ssc_dump_rxqueues(s,m)
#  define       ssc_dump_txqueues(s,m)
#endif

#ifdef CONFIG_SAMA5_SSC_DUMPBUFFERS
#  define       ssc_init_buffer(b,s)   memset(b, 0x55, s);
#  define       ssc_dump_buffer(m,b,s) lib_dumpbuffer(m,b,s)
#else
#  define       ssc_init_buffer(b,s)
#  define       ssc_dump_buffer(m,b,s)
#endif

/* Semaphore helpers */

static void     ssc_exclsem_take(struct sam_ssc_s *priv);
#define         ssc_exclsem_give(priv) sem_post(&priv->exclsem)

static void     ssc_bufsem_take(struct sam_ssc_s *priv);
#define         ssc_bufsem_give(priv) sem_post(&priv->bufsem)

/* Buffer container helpers */

static struct sam_buffer_s *
                ssc_buf_allocate(struct sam_ssc_s *priv);
static void     ssc_buf_free(struct sam_ssc_s *priv,
                  struct sam_buffer_s *bfcontainer);
static void     ssc_buf_initialize(struct sam_ssc_s *priv);

/* DMA support */

#ifdef CONFIG_SAMA5_SSC_DMADEBUG
static void     ssc_dma_sampleinit(struct sam_ssc_s *priv,
                  struct sam_transport_s *xpt);
#endif

#if defined(CONFIG_SAMA5_SSC_DMADEBUG) && defined(SSC_HAVE_RX)
#  define       ssc_rxdma_sample(s,i) sam_dmasample((s)->rx.dma, &(s)->rx.dmaregs[i])
#  define       ssc_rxdma_sampleinit(s) ssc_dma_sampleinit(s, &(s)->rx)
static void     ssc_rxdma_sampledone(struct sam_ssc_s *priv, int result);

#else
#  define       ssc_rxdma_sample(s,i)
#  define       ssc_rxdma_sampleinit(s)
#  define       ssc_rxdma_sampledone(s,r)

#endif

#if defined(CONFIG_SAMA5_SSC_DMADEBUG) && defined(SSC_HAVE_TX)
#  define       ssc_txdma_sample(s,i) sam_dmasample((s)->tx.dma, &(s)->tx.dmaregs[i])
#  define       ssc_txdma_sampleinit(s) ssc_dma_sampleinit(s, &(s)->tx)
static void     ssc_txdma_sampledone(struct sam_ssc_s *priv, int result);

#else
#  define       ssc_txdma_sample(s,i)
#  define       ssc_txdma_sampleinit(s)
#  define       ssc_txdma_sampledone(s,r)

#endif

#ifdef SSC_HAVE_RX
static void     ssc_rxdma_timeout(int argc, uint32_t arg);
static int      ssc_rxdma_setup(struct sam_ssc_s *priv);
static void     ssc_rx_worker(void *arg);
static void     ssc_rx_schedule(struct sam_ssc_s *priv, int result);
static void     ssc_rxdma_callback(DMA_HANDLE handle, void *arg, int result);
#endif
#ifdef SSC_HAVE_TX
static void     ssc_txdma_timeout(int argc, uint32_t arg);
static int      ssc_txdma_setup(struct sam_ssc_s *priv);
static void     ssc_tx_worker(void *arg);
static void     ssc_tx_schedule(struct sam_ssc_s *priv, int result);
static void     ssc_txdma_callback(DMA_HANDLE handle, void *arg, int result);
#endif

/* I2S methods (and close friends) */

static int      ssc_checkwidth(struct sam_ssc_s *priv, int bits);

static uint32_t ssc_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t ssc_rxdatawidth(struct i2s_dev_s *dev, int bits);
static int      ssc_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                  i2s_callback_t callback, void *arg, uint32_t timeout);
static uint32_t ssc_txsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t ssc_txdatawidth(struct i2s_dev_s *dev, int bits);
static int      ssc_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                  i2s_callback_t callback, void *arg,
                  uint32_t timeout);

/* Initialization */

#ifdef SSC_HAVE_RX
static int      ssc_rx_configure(struct sam_ssc_s *priv);
#endif
#ifdef SSC_HAVE_TX
static int      ssc_tx_configure(struct sam_ssc_s *priv);
#endif
static uint32_t ssc_mck2divider(struct sam_ssc_s *priv);
static void     ssc_clocking(struct sam_ssc_s *priv);
static int      ssc_dma_flags(struct sam_ssc_s *priv, uint32_t *dmaflags);
static int      ssc_dma_allocate(struct sam_ssc_s *priv);
static void     ssc_dma_free(struct sam_ssc_s *priv);
#ifdef CONFIG_SAMA5_SSC0
static void     ssc0_configure(struct sam_ssc_s *priv);
#endif
#ifdef CONFIG_SAMA5_SSC1
static void     ssc1_configure(struct sam_ssc_s *priv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* I2S device operations */

static const struct i2s_ops_s g_sscops =
{
  /* Receiver methods */

  .i2s_rxsamplerate = ssc_rxsamplerate,
  .i2s_rxdatawidth  = ssc_rxdatawidth,
  .i2s_receive      = ssc_receive,

  /* Transmitter methods */

  .i2s_txsamplerate = ssc_txsamplerate,
  .i2s_txdatawidth  = ssc_txdatawidth,
  .i2s_send         = ssc_send,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ssc_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   regval  - The value to be written
 *   regaddr - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SSC_REGDEBUG
static bool ssc_checkreg(struct sam_ssc_s *priv, bool wr, uint32_t regval,
                         uint32_t regaddr)
{
  if (wr      == priv->wr &&     /* Same kind of access? */
      regval  == priv->regval && /* Same value? */
      regaddr == priv->regaddr)  /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->count++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->count > 0)
        {
          /* Yes... show how many times we did it */

          lldbg("...[Repeats %d times]...\n", priv->count);
        }

      /* Save information about the new access */

      priv->wr      = wr;
      priv->regval  = regval;
      priv->regaddr = regaddr;
      priv->count   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: ssc_getreg
 *
 * Description:
 *  Read an SSC register
 *
 ****************************************************************************/

static inline uint32_t ssc_getreg(struct sam_ssc_s *priv,
                                  unsigned int offset)
{
  uint32_t regaddr = priv->base + offset;
  uint32_t regval = getreg32(regaddr);

#ifdef CONFIG_SAMA5_SSC_REGDEBUG
  if (ssc_checkreg(priv, false, regval, regaddr))
    {
      lldbg("%08x->%08x\n", regaddr, regval);
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: ssc_putreg
 *
 * Description:
 *  Write a value to an SSC register
 *
 ****************************************************************************/

static inline void ssc_putreg(struct sam_ssc_s *priv, unsigned int offset,
                              uint32_t regval)
{
  uint32_t regaddr = priv->base + offset;

#ifdef CONFIG_SAMA5_SSC_REGDEBUG
  if (ssc_checkreg(priv, true, regval, regaddr))
    {
      lldbg("%08x<-%08x\n", regaddr, regval);
    }
#endif

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: ssc_physregaddr
 *
 * Description:
 *   Return the physical address of an SSC register
 *
 ****************************************************************************/

static inline uintptr_t ssc_physregaddr(struct sam_ssc_s *priv,
                                        unsigned int offset)
{
  return sam_physregaddr(priv->base + offset);
}

/****************************************************************************
 * Name: scc_dump_regs
 *
 * Description:
 *   Dump the contents of all SSC registers
 *
 * Input Parameters:
 *   priv - The SSC controller to dump
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_I2S) && defined(CONFIG_DEBUG_VERBOSE)
static void scc_dump_regs(struct sam_ssc_s *priv, const char *msg)
{
  i2svdbg("SSC%d: %s\n", priv->sscno, msg);
  i2svdbg("   CMR:%08x RCMR:%08x RFMR:%08x TCMR:%08x\n",
          getreg32(priv->base + SAM_SSC_CMR_OFFSET),
          getreg32(priv->base + SAM_SSC_RCMR_OFFSET),
          getreg32(priv->base + SAM_SSC_RFMR_OFFSET),
          getreg32(priv->base + SAM_SSC_TCMR_OFFSET));
  i2svdbg("  TFMR:%08x RC0R:%08x RC1R:%08x   SR:%08x\n",
          getreg32(priv->base + SAM_SSC_TFMR_OFFSET),
          getreg32(priv->base + SAM_SSC_RC0R_OFFSET),
          getreg32(priv->base + SAM_SSC_RC1R_OFFSET),
          getreg32(priv->base + SAM_SSC_SR_OFFSET));
  i2svdbg("   IMR:%08x WPMR:%08x WPSR:%08x\n",
          getreg32(priv->base + SAM_SSC_IMR_OFFSET),
          getreg32(priv->base + SAM_SSC_WPMR_OFFSET),
          getreg32(priv->base + SAM_SSC_WPSR_OFFSET));
}
#endif

/****************************************************************************
 * Name: ssc_dump_queues
 *
 * Description:
 *   Dump the contents of transport queues
 *
 * Input Parameters:
 *   priv - The SSC controller to dump
 *   xpt - The transport to be dumped
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SSC_QDEBUG
static void ssc_dump_queue(sq_queue_t *queue)
{
  struct sam_buffer_s *bfcontainer;
  struct ap_buffer_s *apb;
  sq_entry_t *entry;

  for (entry = queue->head; entry; entry = entry->flink)
    {
      bfcontainer = (struct sam_buffer_s *)entry;
      apb = bfcontainer->apb;

      if (!apb)
        {
          i2sllvdbg("    %p: No buffer\n", bfcontainer);
        }
      else
        {
          i2sllvdbg("    %p: buffer=%p nmaxbytes=%d nbytes=%d\n",
                    bfcontainer, apb, apb->nmaxbytes, apb->nbytes);
        }
    }
}

static void ssc_dump_queues(struct sam_transport_s *xpt, const char *msg)
{
  irqstate_t flags;

  flags = irqsave();
  i2sllvdbg("%s\n", msg);
  i2sllvdbg("  Pending:\n");
  ssc_dump_queue(&xpt->pend);
  i2sllvdbg("  Active:\n");
  ssc_dump_queue(&xpt->act);
  i2sllvdbg("  Done:\n");
  ssc_dump_queue(&xpt->done);
  irqrestore(flags);
}
#endif

/****************************************************************************
 * Name: ssc_exclsem_take
 *
 * Description:
 *   Take the exclusive access semaphore handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the SSC peripheral state
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void ssc_exclsem_take(struct sam_ssc_s *priv)
{
  int ret;

  /* Wait until we successfully get the semaphore.  EINTR is the only
   * expected 'failure' (meaning that the wait for the semaphore was
   * interrupted by a signal.
   */

  do
    {
      ret = sem_wait(&priv->exclsem);
      DEBUGASSERT(ret == 0 || errno == EINTR);
    }
  while (ret < 0);
}

/****************************************************************************
 * Name: ssc_bufsem_take
 *
 * Description:
 *   Take the buffer semaphore handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the SSC peripheral state
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void ssc_bufsem_take(struct sam_ssc_s *priv)
{
  int ret;

  /* Wait until we successfully get the semaphore.  EINTR is the only
   * expected 'failure' (meaning that the wait for the semaphore was
   * interrupted by a signal.
   */

  do
    {
      ret = sem_wait(&priv->bufsem);
      DEBUGASSERT(ret == 0 || errno == EINTR);
    }
  while (ret < 0);
}

/****************************************************************************
 * Name: ssc_buf_allocate
 *
 * Description:
 *   Allocate a buffer container by removing the one at the head of the
 *   free list
 *
 * Input Parameters:
 *   priv - SSC state instance
 *
 * Returned Value:
 *   A non-NULL pointer to the allocate buffer container on success; NULL if
 *   there are no available buffer containers.
 *
 * Assumptions:
 *   The caller does NOT have exclusive access to the SSC state structure.
 *   That would result in a deadlock!
 *
 ****************************************************************************/

static struct sam_buffer_s *ssc_buf_allocate(struct sam_ssc_s *priv)
{
  struct sam_buffer_s *bfcontainer;
  irqstate_t flags;

  /* Set aside a buffer container.  By doing this, we guarantee that we will
   * have at least one free buffer container.
   */

  ssc_bufsem_take(priv);

  /* Get the buffer from the head of the free list */

  flags = irqsave();
  bfcontainer = priv->freelist;
  ASSERT(bfcontainer);

  /* Unlink the buffer from the freelist */

  priv->freelist = bfcontainer->flink;
  irqrestore(flags);
  return bfcontainer;
}

/****************************************************************************
 * Name: ssc_buf_free
 *
 * Description:
 *   Free buffer container by adding it to the head of the free list
 *
 * Input Parameters:
 *   priv - SSC state instance
 *   bfcontainer - The buffer container to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has exclusive access to the SSC state structure
 *
 ****************************************************************************/

static void ssc_buf_free(struct sam_ssc_s *priv, struct sam_buffer_s *bfcontainer)
{
  irqstate_t flags;

  /* Put the buffer container back on the free list */

  flags = irqsave();
  bfcontainer->flink  = priv->freelist;
  priv->freelist = bfcontainer;
  irqrestore(flags);

  /* Wake up any threads waiting for a buffer container */

  ssc_bufsem_give(priv);
}

/****************************************************************************
 * Name: ssc_buf_initialize
 *
 * Description:
 *   Initialize the buffer container allocator by adding all of the
 *   pre-allocated buffer containers to the free list
 *
 * Input Parameters:
 *   priv - SSC state instance
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in SSC initialization so that there are no issues with
 *   concurrency.
 *
 ****************************************************************************/

static void ssc_buf_initialize(struct sam_ssc_s *priv)
{
  int i;

  priv->freelist = NULL;
  sem_init(&priv->bufsem, 0, CONFIG_SAMA5_SSC_MAXINFLIGHT);

  for (i = 0; i < CONFIG_SAMA5_SSC_MAXINFLIGHT; i++)
    {
      ssc_buf_free(priv, &priv->containers[i]);
    }
}

/****************************************************************************
 * Name: ssc_dma_sampleinit
 *
 * Description:
 *   Initialize sampling of DMA registers (if CONFIG_SAMA5_SSC_DMADEBUG)
 *
 * Input Parameters:
 *   priv - SSC state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_SSC_DMADEBUG) && defined(SSC_HAVE_RX)
static void ssc_dma_sampleinit(struct sam_ssc_s *priv,
                               struct sam_transport_s *xpt)
{
  /* Put contents of register samples into a known state */

  memset(xpt->dmaregs, 0xff, DMA_NSAMPLES * sizeof(struct sam_dmaregs_s));

  /* Then get the initial samples */

  sam_dmasample(xpt->dma, &xpt->dmaregs[DMA_INITIAL]);
}
#endif

/****************************************************************************
 * Name: ssc_rxdma_sampledone
 *
 * Description:
 *   Dump sampled RX DMA registers
 *
 * Input Parameters:
 *   priv - SSC state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_SSC_DMADEBUG) && defined(SSC_HAVE_RX)
static void ssc_rxdma_sampledone(struct sam_ssc_s *priv, int result)
{
  lldbg("result: %d\n", result);

  /* Sample the final registers */

  sam_dmasample(priv->rx.dma, &priv->rx.dmaregs[DMA_END_TRANSFER]);

  /* Then dump the sampled DMA registers */
  /* Initial register values */

  sam_dmadump(priv->rx.dma, &priv->rx.dmaregs[DMA_INITIAL],
              "RX: Initial Registers");

  /* Register values after DMA setup */

  sam_dmadump(priv->rx.dma, &priv->rx.dmaregs[DMA_AFTER_SETUP],
              "RX: After DMA Setup");

  /* Register values after DMA start */

  sam_dmadump(priv->rx.dma, &priv->rx.dmaregs[DMA_AFTER_START],
              "RX: After DMA Start");

  /* Register values at the time of the TX and RX DMA callbacks
   * -OR- DMA timeout.
   *
   * If the DMA timedout, then there will not be any RX DMA
   * callback samples.  There is probably no TX DMA callback
   * samples either, but we don't know for sure.
   */

  if (result == -ETIMEDOUT || result == -EINTR)
    {
      sam_dmadump(priv->rx.dma, &priv->rx.dmaregs[DMA_TIMEOUT],
                  "RX: At DMA timeout");
    }
  else
    {
      sam_dmadump(priv->rx.dma, &priv->rx.dmaregs[DMA_CALLBACK],
                  "RX: At DMA callback");
    }

  sam_dmadump(priv->rx.dma, &priv->rx.dmaregs[DMA_END_TRANSFER],
              "RX: At End-of-Transfer");

  scc_dump_regs(priv, "RX: At End-of-Transfer");
}
#endif

/****************************************************************************
 * Name: ssc_txdma_sampledone
 *
 * Description:
 *   Dump sampled DMA registers
 *
 * Input Parameters:
 *   priv - SSC state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_SSC_DMADEBUG) && defined(SSC_HAVE_TX)
static void ssc_txdma_sampledone(struct sam_ssc_s *priv, int result)
{
  lldbg("result: %d\n", result);

  /* Sample the final registers */

  sam_dmasample(priv->tx.dma, &priv->tx.dmaregs[DMA_END_TRANSFER]);

  /* Then dump the sampled DMA registers */
  /* Initial register values */

  sam_dmadump(priv->tx.dma, &priv->tx.dmaregs[DMA_INITIAL],
              "TX: Initial Registers");

  /* Register values after DMA setup */

  sam_dmadump(priv->tx.dma, &priv->tx.dmaregs[DMA_AFTER_SETUP],
              "TX: After DMA Setup");

  /* Register values after DMA start */

  sam_dmadump(priv->tx.dma, &priv->tx.dmaregs[DMA_AFTER_START],
              "TX: After DMA Start");

  /* Register values at the time of the TX and RX DMA callbacks
   * -OR- DMA timeout.
   */

  if (result == -ETIMEDOUT || result == -EINTR)
    {
      sam_dmadump(priv->tx.dma, &priv->tx.dmaregs[DMA_TIMEOUT],
                  "TX: At DMA timeout");
    }
  else
    {
      sam_dmadump(priv->tx.dma, &priv->tx.dmaregs[DMA_CALLBACK],
                  "TX: At DMA callback");
    }

  sam_dmadump(priv->tx.dma, &priv->tx.dmaregs[DMA_END_TRANSFER],
              "TX: At End-of-Transfer");

  scc_dump_regs(priv, "TX: At End-of-Transfer");
}
#endif

/****************************************************************************
 * Name: ssc_rxdma_timeout
 *
 * Description:
 *   The RX watchdog timeout without completion of the RX DMA.
 *
 * Input Parameters:
 *   argc   - The number of arguments (should be 1)
 *   arg    - The argument (state structure reference cast to uint32_t)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

#ifdef SSC_HAVE_RX
static void ssc_rxdma_timeout(int argc, uint32_t arg)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Sample DMA registers at the time of the timeout */

  ssc_rxdma_sample(priv, DMA_TIMEOUT);

  /* Cancel the DMA */

  sam_dmastop(priv->rx.dma);

  /* Then schedule completion of the transfer to occur on the worker thread.
   * NOTE: sam_dmastop() will call the DMA complete callback with an error
   * of -EINTR.  So the following is just insurance and should have no
   * effect if the worker is already schedule.
   */

  ssc_rx_schedule(priv, -ETIMEDOUT);
}
#endif

/****************************************************************************
 * Name: ssc_rxdma_setup
 *
 * Description:
 *   Setup and initiate the next RX DMA transfer
 *
 * Input Parameters:
 *   priv - SSC state instance
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

#ifdef SSC_HAVE_RX
static int ssc_rxdma_setup(struct sam_ssc_s *priv)
{
  struct sam_buffer_s *bfcontainer;
  struct ap_buffer_s *apb;
  uintptr_t paddr;
  uintptr_t maddr;
  uint32_t timeout;
  bool notimeout;
  int ret;

  /* If there is already an active transmission in progress, then bail
   * returning success.
   */

  if (!sq_empty(&priv->rx.act))
    {
      return OK;
    }

  /* If there are no pending transfer, then bail returning success */

  if (sq_empty(&priv->rx.pend))
    {
      return OK;
    }

  /* Initialize DMA register sampling */

  ssc_rxdma_sampleinit(priv);

  /* Loop, adding each pending DMA */

  timeout = 0;
  notimeout = false;

  do
    {
      /* Remove the pending RX transfer at the head of the RX pending queue. */

      bfcontainer = (struct sam_buffer_s *)sq_remfirst(&priv->rx.pend);
      DEBUGASSERT(bfcontainer && bfcontainer->apb);

      apb = bfcontainer->apb;
      DEBUGASSERT(((uintptr_t)apb->samp % priv->align) == 0);

      /* No data received yet */

      apb->nbytes  = 0;
      apb->curbyte = 0;

      /* Physical address of the SSC RHR register and of the buffer location
       * in RAM.
       */

      paddr = ssc_physregaddr(priv, SAM_SSC_RHR_OFFSET);
      maddr = sam_physramaddr((uintptr_t)apb->samp);

      /* Configure the RX DMA */

      sam_dmarxsetup(priv->rx.dma, paddr, maddr, apb->nmaxbytes);

      /* Increment the DMA timeout */

      if (bfcontainer->timeout > 0)
        {
          timeout += bfcontainer->timeout;
        }
      else
        {
          notimeout = true;
        }

      /* Add the container to the list of active DMAs */

      sq_addlast((sq_entry_t *)bfcontainer, &priv->rx.act);

      /* Invalidate the data cache so that nothing gets flush into the
       * DMA buffer after starting the DMA transfer.
       */

      arch_invalidate_dcache((uintptr_t)apb->samp,
                             (uintptr_t)apb->samp + apb->nmaxbytes);

    }
#if 1 /* REVISIT: Chained RX transfers */
  while (0);
#else
  while (!sq_empty(&priv->rx.pend));
#endif

  /* Sample DMA registers */

  ssc_rxdma_sample(priv, DMA_AFTER_SETUP);

  /* Start the DMA, saving the container as the current active transfer */

  sam_dmastart(priv->rx.dma, ssc_rxdma_callback, priv);
  ssc_rxdma_sample(priv, DMA_AFTER_START);

  /* Enable the receiver */

  ssc_putreg(priv, SAM_SSC_CR_OFFSET, SSC_CR_RXEN);

  /* Start a watchdog to catch DMA timeouts */

  if (!notimeout)
    {
      ret = wd_start(priv->rx.dog, timeout, (wdentry_t)ssc_rxdma_timeout,
                     1, (uint32_t)priv);

      /* Check if we have successfully started the watchdog timer.  Note
       * that we do nothing in the case of failure to start the timer.  We
       * are already committed to the DMA anyway.  Let's just hope that the
       * DMA does not hang.
       */

      if (ret < 0)
        {
          i2slldbg("ERROR: wd_start failed: %d\n", errno);
        }
    }

  ssc_dump_rxqueues(priv, "RX DMA started");
  return OK;
}
#endif

/****************************************************************************
 * Name: ssc_rx_worker
 *
 * Description:
 *   RX transfer done worker
 *
 * Input Parameters:
 *   arg - the SSC device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef SSC_HAVE_RX
static void ssc_rx_worker(void *arg)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)arg;
  struct sam_buffer_s *bfcontainer;
  struct ap_buffer_s *apb;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* When the transfer was started, the active buffer containers were removed
   * from the rx.pend queue and saved in the rx.act queue.  We get here when the
   * DMA is finished... either successfully, with a DMA error, or with a DMA
   * timeout.
   *
   * In any case, the buffer containers in rx.act will be moved to the end
   * of the rx.done queue and rx.act queue will be emptied before this worker
   * is started.
   *
   * REVISIT: Normal DMA callback processing should restart the DMA
   * immediately to avoid audio artifacts at the boundaries between DMA
   * transfers.  Unfortunately, the DMA callback occurs at the interrupt
   * level and we cannot call dma_rxsetup() from the interrupt level.
   * So we have to start the next DMA here.
   */

  i2svdbg("rx.act.head=%p rx.done.head=%p\n",
          priv->rx.act.head, priv->rx.done.head);
  ssc_dump_rxqueues(priv, "RX worker start");

  /* Check if the DMA is IDLE */

  if (sq_empty(&priv->rx.act))
    {
#ifdef CONFIG_SAMA5_SSC_DMADEBUG
      bfcontainer = (struct sam_buffer_s *)sq_peek(&priv->rx.done);
      if (bfcontainer)
        {
          /* Dump the DMA registers */

          ssc_rxdma_sampledone(priv, bfcontainer->result);
        }
#endif

      /* Then start the next DMA.  This must be done with interrupts
       * disabled.
       */

      flags = irqsave();
      (void)ssc_rxdma_setup(priv);
      irqrestore(flags);
    }

  /* Process each buffer in the rx.done queue */

  while (sq_peek(&priv->rx.done) != NULL)
    {
      /* Remove the buffer container from the rx.done queue.  NOTE that
       * interrupts must be enabled to do this because the rx.done queue is
       * also modified from the interrupt level.
       */

      flags = irqsave();
      bfcontainer = (struct sam_buffer_s *)sq_remfirst(&priv->rx.done);
      irqrestore(flags);

      DEBUGASSERT(bfcontainer && bfcontainer->apb && bfcontainer->callback);
      apb = bfcontainer->apb;

      /* If the DMA was successful, then update the number of valid bytes in
       * the audio buffer.
       */

      if (bfcontainer->result == OK)
        {
          apb->nbytes = apb->nmaxbytes;
        }

      ssc_dump_buffer("Received", apb->samp, apb->nbytes);

      /* Perform the RX transfer done callback */

      bfcontainer->callback(&priv->dev, apb, bfcontainer->arg,
                            bfcontainer->result);

      /* Release our reference on the audio buffer.  This may very likely
       * cause the audio buffer to be freed.
       */

      apb_free(apb);

      /* And release the buffer container */

      ssc_buf_free(priv, bfcontainer);
    }

  ssc_dump_rxqueues(priv, "RX worker done");
}
#endif

/****************************************************************************
 * Name: ssc_rx_schedule
 *
 * Description:
 *   An RX DMA completion or timeout has occurred.  Schedule processing on
 *   the working thread.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   arg - A pointer to the chip select struction
 *   result - The result of the DMA transfer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

#ifdef SSC_HAVE_RX
static void ssc_rx_schedule(struct sam_ssc_s *priv, int result)
{
  struct sam_buffer_s *bfcontainer;
  int ret;

  /* Upon entry, the transfer(s) that just completed are the ones in the
   * priv->rx.act queue.  NOTE: In certain conditions, this function may
   * be called an additional time, hence, we can't assert this to be true.
   * For example, in the case of a timeout, this function will be called by
   * both indirectly via the sam_dmastop() logic and directly via the
   * ssc_rxdma_timeout() logic.
   */

  ssc_dump_rxqueues(priv, "RX schedule");

  /* Move all entries from the rx.act queue to the rx.done queue */

  while (!sq_empty(&priv->rx.act))
    {
      /* Remove the next buffer container from the rx.act list */

      bfcontainer = (struct sam_buffer_s *)sq_remfirst(&priv->rx.act);

      /* Report the result of the transfer */

      bfcontainer->result = result;

      /* Add the completed buffer container to the tail of the rx.done queue */

      sq_addlast((sq_entry_t *)bfcontainer, &priv->rx.done);
    }

  /* If the worker has completed running, then reschedule the working thread.
   * REVISIT:  There may be a race condition here.  So we do nothing is the
   * worker is not available.
   */

  if (work_available(&priv->rx.work))
    {
      /* Schedule the TX DMA done processing to occur on the worker thread. */

      ret = work_queue(HPWORK, &priv->rx.work, ssc_rx_worker, priv, 0);
      if (ret != 0)
        {
          i2slldbg("ERROR: Failed to queue RX work: %d\n", ret);
        }
    }
}
#endif

/****************************************************************************
 * Name: ssc_rxdma_callback
 *
 * Description:
 *   This callback function is invoked at the completion of the SSC RX DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   arg - A pointer to the chip select struction
 *   result - The result of the DMA transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef SSC_HAVE_RX
static void ssc_rxdma_callback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Cancel the watchdog timeout */

  (void)wd_cancel(priv->rx.dog);

  /* Sample DMA registers at the time of the DMA completion */

  ssc_rxdma_sample(priv, DMA_CALLBACK);

  /* REVISIT:  We would like to the next DMA started here so that we do not
   * get audio glitches at the boundaries between DMA transfers.
   * Unfortunately, we cannot call sam_dmasetup() from an interrupt handler!
   */

  /* Then schedule completion of the transfer to occur on the worker thread */

  ssc_rx_schedule(priv, result);
}
#endif

/****************************************************************************
 * Name: ssc_txdma_timeout
 *
 * Description:
 *   The RX watchdog timeout without completion of the RX DMA.
 *
 * Input Parameters:
 *   argc   - The number of arguments (should be 1)
 *   arg    - The argument (state structure reference cast to uint32_t)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

#ifdef SSC_HAVE_TX
static void ssc_txdma_timeout(int argc, uint32_t arg)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Sample DMA registers at the time of the timeout */

  ssc_txdma_sample(priv, DMA_TIMEOUT);

  /* Cancel the DMA */

  sam_dmastop(priv->tx.dma);

  /* Then schedule completion of the transfer to occur on the worker thread.
   * NOTE: sam_dmastop() will call the DMA complete callback with an error
   * of -EINTR.  So the following is just insurance and should have no
   * effect if the worker is already schedule.
   */

  ssc_tx_schedule(priv, -ETIMEDOUT);
}
#endif

/****************************************************************************
 * Name: ssc_txdma_setup
 *
 * Description:
 *   Setup and initiate the next TX DMA transfer
 *
 * Input Parameters:
 *   priv - SSC state instance
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

#ifdef SSC_HAVE_TX
static int ssc_txdma_setup(struct sam_ssc_s *priv)
{
  struct sam_buffer_s *bfcontainer;
  struct ap_buffer_s *apb;
  uintptr_t samp;
  uintptr_t paddr;
  uintptr_t maddr;
  uint32_t timeout;
  apb_samp_t nbytes;
  bool notimeout;
  int ret;

  /* If there is already an active transmission in progress, then bail
   * returning success.
   */

  if (!sq_empty(&priv->tx.act))
    {
      return OK;
    }

  /* If there are no pending transfer, then bail returning success */

  if (sq_empty(&priv->tx.pend))
    {
      return OK;
    }

  /* Initialize DMA register sampling */

  ssc_txdma_sampleinit(priv);

  /* Loop, adding each pending DMA */

  timeout = 0;
  notimeout = false;

  do
    {
      /* Remove the pending TX transfer at the head of the TX pending queue. */

      bfcontainer = (struct sam_buffer_s *)sq_remfirst(&priv->tx.pend);
      DEBUGASSERT(bfcontainer && bfcontainer->apb);

      apb = bfcontainer->apb;

      /* Get the transfer information, accounting for any data offset */

      samp   = (uintptr_t)&apb->samp[apb->curbyte];
      nbytes = apb->nbytes - apb->curbyte;
      DEBUGASSERT((samp & priv->align) == 0 && (nbytes & priv->align) == 0);

      /* Physical address of the SSC THR register and of the buffer location
       * in RAM.
       */

      paddr = ssc_physregaddr(priv, SAM_SSC_THR_OFFSET);
      maddr = sam_physramaddr(samp);

      /* Configure the TX DMA */

      sam_dmatxsetup(priv->tx.dma, paddr, maddr, nbytes);

      /* Increment the DMA timeout */

      if (bfcontainer->timeout > 0)
        {
          timeout += bfcontainer->timeout;
        }
      else
        {
          notimeout = true;
        }

      /* Add the container to the list of active DMAs */

      sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.act);

      /* Flush the data cache so that everything is in the physical memory
       * before starting the DMA.
       */

      arch_clean_dcache(samp, samp + nbytes);
    }
#if 1 /* REVISIT: Chained TX transfers */
  while (0);
#else
  while (!sq_empty(&priv->tx.pend));
#endif

  /* Sample DMA registers */

  ssc_txdma_sample(priv, DMA_AFTER_SETUP);

  /* Start the DMA, saving the container as the current active transfer */

  sam_dmastart(priv->tx.dma, ssc_txdma_callback, priv);
  ssc_txdma_sample(priv, DMA_AFTER_START);

  /* Enable the transmitter */

  ssc_putreg(priv, SAM_SSC_CR_OFFSET, SSC_CR_TXEN);

  /* Start a watchdog to catch DMA timeouts */

  if (!notimeout)
    {
      ret = wd_start(priv->tx.dog, timeout, (wdentry_t)ssc_txdma_timeout,
                     1, (uint32_t)priv);

      /* Check if we have successfully started the watchdog timer.  Note
       * that we do nothing in the case of failure to start the timer.  We
       * are already committed to the DMA anyway.  Let's just hope that the
       * DMA does not hang.
       */

      if (ret < 0)
        {
          i2slldbg("ERROR: wd_start failed: %d\n", errno);
        }
    }

  ssc_dump_txqueues(priv, "TX DMA started");
  return OK;
}
#endif

/****************************************************************************
 * Name: ssc_tx_worker
 *
 * Description:
 *   TX transfer done worker
 *
 * Input Parameters:
 *   arg - the SSC device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef SSC_HAVE_TX
static void ssc_tx_worker(void *arg)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)arg;
  struct sam_buffer_s *bfcontainer;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* When the transfer was started, the active buffer containers were removed
   * from the tx.pend queue and saved in the tx.act queue.  We get here when the
   * DMA is finished... either successfully, with a DMA error, or with a DMA
   * timeout.
   *
   * In any case, the buffer containers in tx.act will be moved to the end
   * of the tx.done queue and tx.act will be emptied before this worker is
   * started.
   *
   * REVISIT: Normal DMA callback processing should restart the DMA
   * immediately to avoid audio artifacts at the boundaries between DMA
   * transfers.  Unfortunately, the DMA callback occurs at the interrupt
   * level and we cannot call dma_txsetup() from the interrupt level.
   * So we have to start the next DMA here.
   */

  i2svdbg("tx.act.head=%p tx.done.head=%p\n",
           priv->tx.act.head, priv->tx.done.head);
  ssc_dump_txqueues(priv, "TX worker start");

  /* Check if the DMA is IDLE */

  if (sq_empty(&priv->tx.act))
    {
#ifdef CONFIG_SAMA5_SSC_DMADEBUG
      bfcontainer = (struct sam_buffer_s *)sq_peek(&priv->tx.done);
      if (bfcontainer)
        {
          /* Dump the DMA registers */

          ssc_txdma_sampledone(priv, bfcontainer->result);
        }
#endif

      /* Then start the next DMA.  This must be done with interrupts
       * disabled.
       */

      flags = irqsave();
      (void)ssc_txdma_setup(priv);
      irqrestore(flags);
    }

  /* Process each buffer in the tx.done queue */

  while (sq_peek(&priv->tx.done) != NULL)
    {
      /* Remove the buffer container from the tx.done queue.  NOTE that
       * interrupts must be enabled to do this because the tx.done queue is
       * also modified from the interrupt level.
       */

      flags = irqsave();
      bfcontainer = (struct sam_buffer_s *)sq_remfirst(&priv->tx.done);
      irqrestore(flags);

      /* Perform the TX transfer done callback */

      DEBUGASSERT(bfcontainer && bfcontainer->callback);
      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, bfcontainer->result);

      /* Release our reference on the audio buffer.  This may very likely
       * cause the audio buffer to be freed.
       */

      apb_free(bfcontainer->apb);

      /* And release the buffer container */

      ssc_buf_free(priv, bfcontainer);
    }

  ssc_dump_txqueues(priv, "TX worker done");
}
#endif

/****************************************************************************
 * Name: ssc_tx_schedule
 *
 * Description:
 *   An TX DMA completion or timeout has occurred.  Schedule processing on
 *   the working thread.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   arg - A pointer to the chip select struction
 *   result - The result of the DMA transfer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - The TX timeout has been canceled.
 *
 ****************************************************************************/

#ifdef SSC_HAVE_TX
static void ssc_tx_schedule(struct sam_ssc_s *priv, int result)
{
  struct sam_buffer_s *bfcontainer;
  int ret;

  /* Upon entry, the transfer(s) that just completed are the ones in the
   * priv->tx.act queue.  NOTE: In certain conditions, this function may
   * be called an additional time, hence, we can't assert this to be true.
   * For example, in the case of a timeout, this function will be called by
   * both indirectly via the sam_dmastop() logic and directly via the
   * ssc_txdma_timeout() logic.
   */

  ssc_dump_txqueues(priv, "TX schedule");

  /* Move all entries from the tx.act queue to the tx.done queue */

  while (!sq_empty(&priv->tx.act))
    {
      /* Remove the next buffer container from the tx.act list */

      bfcontainer = (struct sam_buffer_s *)sq_remfirst(&priv->tx.act);

      /* Report the result of the transfer */

      bfcontainer->result = result;

      /* Add the completed buffer container to the tail of the tx.done queue */

      sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.done);
    }

  /* If the worker has completed running, then reschedule the working thread.
   * REVISIT:  There may be a race condition here.  So we do nothing is the
   * worker is not available.
   */

  if (work_available(&priv->tx.work))
    {
      /* Schedule the TX DMA done processing to occur on the worker thread. */

      ret = work_queue(HPWORK, &priv->tx.work, ssc_tx_worker, priv, 0);
      if (ret != 0)
        {
          i2slldbg("ERROR: Failed to queue TX work: %d\n", ret);
        }
    }
}
#endif

/****************************************************************************
 * Name: ssc_txdma_callback
 *
 * Description:
 *   This callback function is invoked at the completion of the SSC TX DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   arg - A pointer to the chip select struction
 *   result - The result of the DMA transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef SSC_HAVE_TX
static void ssc_txdma_callback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Cancel the watchdog timeout */

  (void)wd_cancel(priv->tx.dog);

  /* Sample DMA registers at the time of the DMA completion */

  ssc_txdma_sample(priv, DMA_CALLBACK);

  /* REVISIT:  We would like to the next DMA started here so that we do not
   * get audio glitches at the boundaries between DMA transfers.
   * Unfortunately, we cannot call sam_dmasetup() from an interrupt handler!
   */

  /* Then schedule completion of the transfer to occur on the worker thread */

  ssc_tx_schedule(priv, result);
}
#endif

/****************************************************************************
 * Name: ssc_checkwidth
 *
 * Description:
 *   Check for a valid bit width.  The SSC is capable of handling most any
 *   bit width from 2 to 32, but the DMA logic in this driver is constrained
 *   to 8-, 16-, and 32-bit data widths
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static int ssc_checkwidth(struct sam_ssc_s *priv, int bits)
{
  /* The SSC can handle most any bit width from 2 to 32.  However, the DMA
   * logic here is constrained to byte, half-word, and word sizes.
   */

  switch (bits)
    {
    case 8:
#ifdef CONFIG_DEBUG
      priv->align = 0;
#endif
      break;

    case 16:
#ifdef CONFIG_DEBUG
      priv->align = 1;
#endif
      break;

    case 32:
#ifdef CONFIG_DEBUG
      priv->align = 3;
#endif
      break;

    default:
      i2sdbg("ERROR: Unsupported or invalid data width: %d\n", bits);
      return (bits < 2 || bits > 32) ? -EINVAL : -ENOSYS;
    }

  /* Save the new data width */

  priv->datalen = bits;
  return OK;
}

/****************************************************************************
 * Name: ssc_rxsamplerate
 *
 * Description:
 *   Set the I2S RX sample rate.  NOTE:  This will have no effect if (1) the
 *   driver does not support an I2C receiver or if (2) the sample rate is
 *   driven by the I2C frame clock.  This may also have unexpected side-
 *   effects of the RX sample is coupled with the TX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t ssc_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
#if defined(SSC_HAVE_RX) && defined(SSC_HAVE_MCK2)
  struct sam_ssc_s *priv = (struct sam_ssc_s *)dev;
  DEBUGASSERT(priv && priv->samplerate > 0 && rate > 0);

  /* Check if the receiver is driven by the MCK/2 */

  if (priv->rxclk == SSC_CLKSRC_MCKDIV)
    {
      /* Save the new sample rate and update the MCK/2 divider */

      priv->samplerate = rate;
      return ssc_mck2divider(priv);
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: ssc_rxdatawidth
 *
 * Description:
 *   Set the I2S RX data width.  The RX bitrate is determined by
 *   sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The I2S data with in bits.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t ssc_rxdatawidth(struct i2s_dev_s *dev, int bits)
{
#ifdef SSC_HAVE_RX
  struct sam_ssc_s *priv = (struct sam_ssc_s *)dev;
  uint32_t dmaflags;
  int ret;

  DEBUGASSERT(priv && bits > 1);

  /* Check if this is a bit width that we are configured to handle */

  ret = ssc_checkwidth(priv, bits);
  if (ret < 0)
    {
      i2sdbg("ERROR: ssc_checkwidth failed: %d\n", ret);
      return 0;
    }

  /* Update the DMA flags */

  ret = ssc_dma_flags(priv, &dmaflags);
  if (ret < 0)
    {
      i2sdbg("ERROR: ssc_dma_flags failed: %d\n", ret);
      return 0;
    }

  /* Reconfigure the RX DMA (and TX DMA if applicable) */

  sam_dmaconfig(priv->rx.dma, dmaflags);
#ifdef SSC_HAVE_RX
  if (priv->txenab)
    {
      sam_dmaconfig(priv->tx.dma, dmaflags);
    }
#endif

#ifdef SSC_HAVE_MCK2
  /* Check if the receiver is driven by the MCK/2 */

  if (priv->rxclk == SSC_CLKSRC_MCKDIV)
    {
      /* Update the MCK/2 divider.  bitrate is samplerate * datawidth. */

      return ssc_mck2divider(priv);
    }
#endif
#endif

  return 0;
}

/****************************************************************************
 * Name: ssc_receive
 *
 * Description:
 *   Receive a block of data from I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer in which to recieve data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use.  The transfer will be canceled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.  NOTE:  This function
 *   only enqueues the transfer and returns immediately.  Success here only
 *   means that the transfer was enqueued correctly.
 *
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

static int ssc_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                       i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)dev;
#ifdef SSC_HAVE_RX
  struct sam_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;
#endif

  DEBUGASSERT(priv && apb && ((uintptr_t)apb->samp & priv->align) == 0);
  i2svdbg("apb=%p nmaxbytes=%d arg=%p timeout=%d\n",
          apb, apb->nmaxbytes, arg, timeout);

  ssc_init_buffer(apb->samp, apb->nmaxbytes);

#ifdef SSC_HAVE_RX
  /* Allocate a buffer container in advance */

  bfcontainer = ssc_buf_allocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the SSC driver data */

  ssc_exclsem_take(priv);

  /* Has the RX channel been enabled? */

  if (!priv->rxenab)
    {
      i2sdbg("ERROR: SSC%d has no receiver\n", priv->sscno);
      ret = -EAGAIN;
      goto errout_with_exclsem;
    }

  /* Add a reference to the audio buffer */

  apb_reference(apb);

  /* Initialize the buffer container structure */

  bfcontainer->callback = (void *)callback;
  bfcontainer->timeout  = timeout;
  bfcontainer->arg      = arg;
  bfcontainer->apb      = apb;
  bfcontainer->result   = -EBUSY;

  /* Add the buffer container to the end of the RX pending queue */

  flags = irqsave();
  sq_addlast((sq_entry_t *)bfcontainer, &priv->rx.pend);
  ssc_dump_rxqueues(priv, "Receving");

  /* Then start the next transfer.  If there is already a transfer in progess,
   * then this will do nothing.
   */

  ret = ssc_rxdma_setup(priv);
  DEBUGASSERT(ret == OK);
  irqrestore(flags);
  ssc_exclsem_give(priv);
  return OK;

errout_with_exclsem:
  ssc_exclsem_give(priv);
  ssc_buf_free(priv, bfcontainer);
  return ret;

#else
  i2sdbg("ERROR: SSC%d has no receiver\n", priv->sscno);
  UNUSED(priv);
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: ssc_txsamplerate
 *
 * Description:
 *   Set the I2S TX sample rate.  NOTE:  This will have no effect if (1) the
 *   driver does not support an I2C transmitter or if (2) the sample rate is
 *   driven by the I2C frame clock.  This may also have unexpected side-
 *   effects of the TX sample is coupled with the RX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t ssc_txsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
#if defined(SSC_HAVE_TX) && defined(SSC_HAVE_MCK2)
  struct sam_ssc_s *priv = (struct sam_ssc_s *)dev;
  DEBUGASSERT(priv && priv->samplerate > 0 && rate > 0);

  /* Check if the receiver is driven by the MCK/2 */

  if (priv->txclk == SSC_CLKSRC_MCKDIV)
    {
      /* Save the new sample rate and update the MCK/2 divider */

      priv->samplerate = rate;
      return ssc_mck2divider(priv);
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: ssc_txdatawidth
 *
 * Description:
 *   Set the I2S TX data width.  The TX bitrate is determined by
 *   sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The I2S data with in bits.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t ssc_txdatawidth(struct i2s_dev_s *dev, int bits)
{
#ifdef SSC_HAVE_TX
  struct sam_ssc_s *priv = (struct sam_ssc_s *)dev;
  uint32_t dmaflags;
  int ret;

  DEBUGASSERT(priv && bits > 1);

  /* Check if this is a bit width that we are configured to handle */

  ret = ssc_checkwidth(priv, bits);
  if (ret < 0)
    {
      i2sdbg("ERROR: ssc_checkwidth failed: %d\n", ret);
      return 0;
    }

  /* Upate the DMA flags */

  ret = ssc_dma_flags(priv, &dmaflags);
  if (ret < 0)
    {
      i2sdbg("ERROR: ssc_dma_flags failed: %d\n", ret);
      return 0;
    }

  /* Reconfigure the RX DMA (and RX DMA if applicable) */

  sam_dmaconfig(priv->tx.dma, dmaflags);
#ifdef SSC_HAVE_RX
  if (priv->rxenab)
    {
      sam_dmaconfig(priv->rx.dma, dmaflags);
    }
#endif

#ifdef SSC_HAVE_MCK2
  /* Check if the transmitter is driven by the MCK/2 */

  if (priv->txclk == SSC_CLKSRC_MCKDIV)
    {
      /* Update the MCK/2 divider.  bitrate is samplerate * datawidth. */

      return ssc_mck2divider(priv);
    }
#endif
#endif

  return 0;
}

/****************************************************************************
 * Name: ssc_send
 *
 * Description:
 *   Send a block of data on I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer from which to send data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use.  The transfer will be canceled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.  NOTE:  This function
 *   only enqueues the transfer and returns immediately.  Success here only
 *   means that the transfer was enqueued correctly.
 *
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

static int ssc_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                    i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)dev;
#ifdef SSC_HAVE_TX
  struct sam_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;
#endif

  /* Make sure that we have valid pointers that that the data has uint32_t
   * alignment.
   */

  DEBUGASSERT(priv && apb);
  i2svdbg("apb=%p nbytes=%d arg=%p timeout=%d\n",
          apb, apb->nbytes - apb->curbyte, arg, timeout);

  ssc_dump_buffer("Sending", &apb->samp[apb->curbyte],
                  apb->nbytes - apb->curbyte);
  DEBUGASSERT(((uintptr_t)&apb->samp[apb->curbyte] & priv->align) == 0);

#ifdef SSC_HAVE_TX
  /* Allocate a buffer container in advance */

  bfcontainer = ssc_buf_allocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the SSC driver data */

  ssc_exclsem_take(priv);

  /* Has the TX channel been enabled? */

  if (!priv->txenab)
    {
      i2sdbg("ERROR: SSC%d has no transmitter\n", priv->sscno);
      ret = -EAGAIN;
      goto errout_with_exclsem;
    }

  /* Add a reference to the audio buffer */

  apb_reference(apb);

  /* Initialize the buffer container structure */

  bfcontainer->callback = (void *)callback;
  bfcontainer->timeout  = timeout;
  bfcontainer->arg      = arg;
  bfcontainer->apb      = apb;
  bfcontainer->result   = -EBUSY;

  /* Add the buffer container to the end of the TX pending queue */

  flags = irqsave();
  sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.pend);
  ssc_dump_txqueues(priv, "Transmitting");

  /* Then start the next transfer.  If there is already a transfer in progess,
   * then this will do nothing.
   */

  ret = ssc_txdma_setup(priv);
  DEBUGASSERT(ret == OK);
  irqrestore(flags);
  ssc_exclsem_give(priv);
  return OK;

errout_with_exclsem:
  ssc_exclsem_give(priv);
  ssc_buf_free(priv, bfcontainer);
  return ret;

#else
  i2sdbg("ERROR: SSC%d has no transmitter\n", priv->sscno);
  UNUSED(priv);
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: ssc_rx/tx_configure
 *
 * Description:
 *   Configure the SSC receiver and transmitter.
 *
 * Input Parameters:
 *   priv - Fully initialized SSC device structure.
 *
 * Returned Value:
 *   OK is returned on failure.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int ssc_rx_configure(struct sam_ssc_s *priv)
{
#ifdef SSC_HAVE_RX
  uint32_t regval;
  uint32_t fslen;

  /* Get the RX sync time (in RX clocks) */

  DEBUGASSERT(priv->rxfslen > 0);
  fslen  = priv->rxfslen - 1;

  /* RCMR settings */
  /* Configure the receiver input clock */

  regval = 0;
  switch (priv->rxclk)
    {
    case SSC_CLKSRC_RKIN:    /* Receiver clock source is RK */
      regval = SSC_RCMR_CKS_RK;
      break;

    case SSC_CLKSRC_TXOUT:   /* Receiver clock source is the transmitter clock */
      regval = SSC_RCMR_CKS_TK;
      break;

    case SSC_CLKSRC_MCKDIV:  /* Clock source is MCK divided down */
#ifdef SSC_HAVE_MCK2
      DEBUGASSERT(priv->samplerate > 0);
      regval = SSC_RCMR_CKS_MCK;
      break;
#endif

    case SSC_CLKSRC_NONE: /* No clock */
    default:
      i2sdbg("ERROR:  No receiver clock\n");
      return -EINVAL;
    }

  /* Configure the receiver output clock */

  switch (priv->rxout)
    {
    case SSC_CLKOUT_CONT: /* Continuous */
      regval |= SSC_RCMR_CKO_CONT;
      break;

    case SSC_CLKOUT_XFER: /* Only output clock during transfers */
      regval |= SSC_RCMR_CKO_TRANSFER;
      break;

    case SSC_CLKOUT_NONE: /* No output clock */
      regval |= SSC_RCMR_CKO_NONE;
      break;

    default:
      i2sdbg("ERROR: Invalid clock output selection\n");
      return -EINVAL;
    }

  /* REVISIT:  Some of these settings will need to be configurable as well.
   * Currently hardcoded to:
   *
   *   SSC_RCMR_CKI         Receive clock inversion
   *   SSC_RCMR_CKG_CONT    No receive clock gating
   *   SSC_RCMR_START_EDGE  Detection of any edge on RF signal
   *   SSC_RCMR_STOP        Not selected
   *   SSC_RCMR_STTDLY(1)   Receive start delay = 1 (same as FSLEN)
   *   SSC_RCMR_PERIOD(0)   Receive period divider = 0
   *
   * REVISIT:  This implementation assumes that on the transmitter
   * can be the master (i.e, can generate the TK/RK clocking.
   */

  regval |= (SSC_RCMR_CKI | SSC_RCMR_CKG_CONT | SSC_RCMR_START_EDGE |
             SSC_RCMR_STTDLY(priv->rxsttdly) | SSC_RCMR_PERIOD(0));
  ssc_putreg(priv, SAM_SSC_RCMR_OFFSET, regval);

  /* RFMR settings. Some of these settings will need to be configurable as well.
   * Currently hardcoded to:
   *
   *  SSC_RFMR_DATLEN(n)    'n' deterimined by configuration
   *  SSC_RFMR_LOOP         Determined by configuration
   *  SSC_RFMR_MSBF         Most significant bit first
   *  SSC_RFMR_DATNB(n)     Data number 'n' per frame (hard-coded)
   *  SSC_RFMR_FSLEN        Set to LS 4 bits of (CONFIG_SSCx_RX_FSLEN-1)
   *  SSC_RFMR_FSLEN(1)     Pulse length = FSLEN + (FSLEN_EXT * 16) + 1 = 2 clocks
   *  SSC_RFMR_FSOS_NONE    RF pin is always in input
   *  SSC_RFMR_FSEDGE_POS   Positive frame sync edge detection
   *  SSC_RFMR_FSLENEXT   I Set to MS 4 bits of (CONFIG_SSCx_TX_FSLEN-1)
   */

  regval = (SSC_RFMR_DATLEN(CONFIG_SAMA5_SSC0_DATALEN - 1) | SSC_RFMR_MSBF |
            SSC_RFMR_DATNB(SSC_DATNB - 1) | SSC_RFMR_FSOS_NONE);

  /* Set the RX frame synch  */

  regval |= (SSC_RFMR_FSLEN(fslen & 0x0f) | SSC_RFMR_FSLENEXT((fslen >> 4) & 0x0f));

  /* Loopback mode? */

  if (priv->loopback)
    {
      regval |= SSC_RFMR_LOOP;
    }

  ssc_putreg(priv, SAM_SSC_RFMR_OFFSET, regval);

#else
  ssc_putreg(priv, SAM_SSC_RCMR_OFFSET, 0);
  ssc_putreg(priv, SAM_SSC_RFMR_OFFSET, 0);

#endif

  /* Disable the receiver */

  ssc_putreg(priv, SAM_SSC_CR_OFFSET, SSC_CR_RXDIS);
  return OK;
}

static int ssc_tx_configure(struct sam_ssc_s *priv)
{
#ifdef SSC_HAVE_TX
  uint32_t regval;
  uint32_t fslen;
  uint32_t period;

  /* Get the TX synch in (in TX clocks) */

  fslen  = priv->txfslen > 0 ? priv->txfslen - 1 : 0;

  /* From the start delay and the datalength , we can get the full
   * period of the waveform.
   */

  period = SCC_PERIOD(priv->txsttdly, priv->datalen);

  /* TCMR settings */
  /* Configure the transmitter input clock */

  regval = 0;
  switch (priv->txclk)
    {
    case SSC_CLKSRC_TKIN:    /* Transmitter clock source is TK */
      regval = SSC_TCMR_CKS_TK;
      break;

    case SSC_CLKSRC_RXOUT:   /* Transmitter clock source is the receiver clock */
      regval = SSC_TCMR_CKS_RK;
      break;

    case SSC_CLKSRC_MCKDIV:  /* Clock source is MCK divided down */
#ifdef SSC_HAVE_MCK2
      DEBUGASSERT(priv->samplerate > 0);
      regval = SSC_TCMR_CKS_MCK;
      break;
#endif

    case SSC_CLKSRC_NONE: /* No clock */
    default:
      i2sdbg("ERROR:  No transmitter clock\n");
      return -EINVAL;
    }

  /* Configure the receiver output clock */

  switch (priv->txout)
    {
    case SSC_CLKOUT_CONT: /* Continuous */
      regval |= SSC_TCMR_CKO_CONT;
      break;

    case SSC_CLKOUT_XFER: /* Only output clock during transfers */
      regval |= SSC_TCMR_CKO_TRANSFER;
      break;

    case SSC_CLKOUT_NONE: /* No output clock */
      regval |= SSC_TCMR_CKO_NONE;
      break;

    default:
      i2sdbg("ERROR: Invalid clock output selection\n");
      return -EINVAL;
    }

  /* REVISIT:  Some of these settings will need to be configurable as well.
   * Currently hard-coded to:
   *
   *   SSC_RCMR_CKI           No transmitter clock inversion
   *   SSC_RCMR_CKG_CONT      No transmit clock gating
   *   SSC_TCMR_STTDLY(1)     Receive start delay = 2 clocks (same as FSLEN)
   *
   * If master (i.e., provides clocking):
   *   SSC_TCMR_START_CONT    When data written to THR
   *   SSC_TCMR_PERIOD(n)     'n' depends on the datawidth
   *
   * If slave (i.e., receives clocking):
   *   SSC_TCMR_START_EDGE    Detection of any edge on TF signal
   *   SSC_TCMR_PERIOD(0)     Receive period divider = 0
   *
   * The period signal is generated at clocks = 2 x (PERIOD+1), or
   * PERIOD = (clocks / 2) - 1.
   */

  if (priv->txclk == SSC_CLKSRC_MCKDIV)
    {
      regval |= (SSC_TCMR_CKG_CONT | SSC_TCMR_START_CONT |
                 SSC_TCMR_STTDLY(priv->txsttdly) | SSC_TCMR_PERIOD(period / 2 - 1));
    }
  else
    {
      regval |= (SSC_TCMR_CKG_CONT | SSC_TCMR_START_EDGE |
                 SSC_TCMR_STTDLY(priv->txsttdly) | SSC_TCMR_PERIOD(0));
    }

  ssc_putreg(priv, SAM_SSC_TCMR_OFFSET, regval);

  /* TFMR settings. Some of these settings will need to be configurable as well.
   * Currently set to:
   *
   *  SSC_TFMR_DATLEN(n)    'n' determined by configuration
   *  SSC_TFMR_DATDEF        Data default = 0
   *  SSC_TFMR_MSBF          Most significant bit first
   *  SSC_TFMR_DATNB(n)      Data number 'n' per frame (hard-coded)
   *  SSC_TFMR_FSDEN         Enabled if CONFIG_SSCx_TX_FSLEN > 0
   *  SSC_TFMR_FSLEN         If enabled, set to LS 4 bits of (CONFIG_SSCx_TX_FSLEN-1)
   *  SSC_TFMR_FSLENEXT      If enabled, set to MS 4 bits of (CONFIG_SSCx_TX_FSLEN-1)
   *
   * If master (i.e., provides clocking):
   *  SSC_TFMR_FSOS_NEGATIVE Negative pulse TF output
   *
   * If slave (i.e, receives clocking):
   *  SSC_TFMR_FSOS_NONE     TF is an output
   */

  if (priv->txclk == SSC_CLKSRC_MCKDIV)
    {
      regval = (SSC_TFMR_DATLEN(priv->datalen - 1) |
                SSC_TFMR_MSBF | SSC_TFMR_DATNB(SSC_DATNB - 1) |
                SSC_TFMR_FSOS_NEGATIVE);
    }
  else
    {
      regval = (SSC_TFMR_DATLEN(priv->datalen - 1) |
                SSC_TFMR_MSBF | SSC_TFMR_DATNB(SSC_DATNB - 1) |
                SSC_TFMR_FSOS_NONE);
    }

  /* Is the TX frame synch enabled? */

  if (priv->txfslen > 0)
    {
      /* Yes.. Set the FSDEN bit and the FSLEN field */

      regval |= (SSC_TFMR_FSDEN | SSC_TFMR_FSLEN(fslen & 0x0f) |
                 SSC_TFMR_FSLENEXT((fslen >> 4) & 0x0f));
    }

  ssc_putreg(priv, SAM_SSC_TFMR_OFFSET, regval);

#else
  ssc_putreg(priv, SAM_SSC_TCMR_OFFSET, 0);
  ssc_putreg(priv, SAM_SSC_TFMR_OFFSET, 0);

#endif

  /* Disable the transmitter */

  ssc_putreg(priv, SAM_SSC_CR_OFFSET, SSC_CR_TXDIS);
  return OK;
}

/****************************************************************************
 * Name: ssc_mck2divider
 *
 * Description:
 *   Setup the MCK/2 divider based on the currently selected data width and
 *   the sample rate
 *
 * Input Parameter:
 *   priv - I2C device structure (only the sample rate and data length is
 *          needed at this point).
 *
 * Returned Value:
 *  The current bitrate
 *
 ****************************************************************************/

static uint32_t ssc_mck2divider(struct sam_ssc_s *priv)
{
#ifdef SSC_HAVE_MCK2
  uint32_t bitrate;
  uint32_t regval;
  DEBUGASSERT(priv && priv->samplerate > 0 && priv->datalen > 0);

  /* A zero sample rate means to disable the MCK/2 clock */

  if (priv->samplerate == 0)
    {
      bitrate = 0;
      regval  = 0;
    }
  else
    {
      /* Calculate the new bitrate in Hz */

      bitrate = priv->samplerate * priv->datalen;

      /* Calculate the new MCK/2 divider from the bitrate. The divided clock
       * equals:
       *
       *   bitrate = MCK / (2 * div)
       *   div     = MCK / (2 * bitrate)
       *
       * The maximum bit rate is MCK/2. The minimum bit rate is
       * MCK/2 x 4095 = MCK/8190.
       */

      regval =  (BOARD_MCK_FREQUENCY + bitrate) / (bitrate << 1);
    }

  /* Configure MCK/2 divider */

  ssc_putreg(priv, SAM_SSC_CMR_OFFSET, regval);
  return bitrate;
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: ssc_clocking
 *
 * Description:
 *   Enable and configure clocking to the SSC
 *
 * Input Parameter:
 *   priv - Partially initialized I2C device structure (only the PID is
 *          needed at this point).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ssc_clocking(struct sam_ssc_s *priv)
{
  uint32_t regval;
  uint32_t mck;

  /* Determine the maximum SSC peripheral clock frequency */

  mck = BOARD_MCK_FREQUENCY;
#ifdef SAMA5_HAVE_PMC_PCR_DIV
  DEBUGASSERT((mck >> 3) <= SAM_SSC_MAXPERCLK);

  if (mck <= SAM_SSC_MAXPERCLK)
    {
      priv->frequency = mck;
      regval          = PMC_PCR_DIV1;
    }
  else if ((mck >> 1) <= SAM_SSC_MAXPERCLK)
    {
      priv->frequency = (mck >> 1);
      regval          = PMC_PCR_DIV2;
    }
  else if ((mck >> 2) <= SAM_SSC_MAXPERCLK)
    {
      priv->frequency = (mck >> 2);
      regval          = PMC_PCR_DIV4;
    }
  else /* if ((mck >> 3) <= SAM_SSC_MAXPERCLK) */
    {
      priv->frequency = (mck >> 3);
      regval          = PMC_PCR_DIV8;
    }

#else
  /* No PCR_DIV field */

  priv->frequency     = mck;
  regval              = 0;
#endif

  /* Set the maximum SSC peripheral clock frequency */

  regval |= PMC_PCR_PID(priv->pid) | PMC_PCR_CMD | PMC_PCR_EN;
  putreg32(regval, SAM_PMC_PCR);

  /* Reset, disable receiver & transmitter */

  ssc_putreg(priv, SAM_SSC_CR_OFFSET, SSC_CR_RXDIS | SSC_CR_TXDIS | SSC_CR_SWRST);

  /* Configure MCK/2 divider */

  (void)ssc_mck2divider(priv);

  /* Enable peripheral clocking */

  sam_enableperiph1(priv->pid);

  i2svdbg("PCSR1=%08x PCR=%08x CMR=%08x\n",
          getreg32(SAM_PMC_PCSR1), regval,
          ssc_getreg(priv, SAM_SSC_CMR_OFFSET));
}

/****************************************************************************
 * Name: ssc_dma_flags
 *
 * Description:
 *   Determine DMA FLAGS based on PID and data width
 *
 * Input Parameters:
 *   priv - Partially initialized I2C device structure.
 *   dmaflags - Location to return the DMA flags.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int ssc_dma_flags(struct sam_ssc_s *priv, uint32_t *dmaflags)
{
  uint32_t flags;

  switch (priv->datalen)
    {
    case 8:
      flags = DMA8_FLAGS;
      break;

    case 16:
      flags = DMA16_FLAGS;
      break;

    case 32:
      flags = DMA32_FLAGS;
      break;

    default:
      i2sdbg("ERROR: Unsupported data width: %d\n", priv->datalen);
      return -ENOSYS;
    }

  *dmaflags = (flags | DMACH_FLAG_PERIPHPID(priv->pid));
  return OK;
}

/****************************************************************************
 * Name: ssc_dma_allocate
 *
 * Description:
 *   Allocate SCC DMA channels
 *
 * Input Parameters:
 *   priv - Partially initialized I2C device structure.  This function
 *          will complete the DMA specific portions of the initialization
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int ssc_dma_allocate(struct sam_ssc_s *priv)
{
  uint32_t dmaflags;
  int ret;

  /* Get the DMA flags for this channel */

  ret = ssc_dma_flags(priv, &dmaflags);
  if (ret < 0)
    {
      i2sdbg("ERROR: ssc_dma_flags failed: %d\n", ret);
      return ret;
    }

  /* Allocate DMA channels.  These allocations exploit that fact that
   * SSC0 is managed by DMAC0 and SSC1 is managed by DMAC1.  Hence,
   * the SSC number (sscno) is the same as the DMAC number.
   */

#ifdef SSC_HAVE_RX
  if (priv->rxenab)
    {
      /* Allocate an RX DMA channel */

      priv->rx.dma = sam_dmachannel(priv->sscno, dmaflags);
      if (!priv->rx.dma)
        {
          i2sdbg("ERROR: Failed to allocate the RX DMA channel\n");
          goto errout;
        }

      /* Create a watchdog time to catch RX DMA timeouts */

      priv->rx.dog = wd_create();
      if (!priv->rx.dog)
        {
          i2sdbg("ERROR: Failed to create the RX DMA watchdog\n");
          goto errout;
        }
    }
#endif

#ifdef SSC_HAVE_TX
  if (priv->txenab)
    {
      /* Allocate a TX DMA channel */

      priv->tx.dma = sam_dmachannel(priv->sscno, dmaflags);
      if (!priv->tx.dma)
        {
          i2sdbg("ERROR: Failed to allocate the TX DMA channel\n");
          goto errout;
        }

      /* Create a watchdog time to catch TX DMA timeouts */

      priv->tx.dog = wd_create();
      if (!priv->tx.dog)
        {
          i2sdbg("ERROR: Failed to create the TX DMA watchdog\n");
          goto errout;
        }
    }
#endif

  /* Success exit */

  return OK;

  /* Error exit */

errout:
  ssc_dma_free(priv);
  return -ENOMEM;
}

/****************************************************************************
 * Name: ssc_dma_free
 *
 * Description:
 *   Release DMA-related resources allocated by ssc_dma_allocate()
 *
 * Input Parameters:
 *   priv - Partially initialized I2C device structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ssc_dma_free(struct sam_ssc_s *priv)
{
#ifdef SSC_HAVE_TX
  if (priv->tx.dog)
    {
       wd_delete(priv->tx.dog);
    }

  if (priv->tx.dma)
    {
      sam_dmafree(priv->tx.dma);
    }
#endif

#ifdef SSC_HAVE_RX
  if (priv->rx.dog)
    {
       wd_delete(priv->rx.dog);
    }

  if (priv->rx.dma)
    {
      sam_dmafree(priv->rx.dma);
    }
#endif
}

/****************************************************************************
 * Name: ssc0/1_configure
 *
 * Description:
 *   Configure SSC0 and/or SSC1
 *
 * Input Parameters:
 *   priv - Partially initialized I2C device structure.  These functions
 *          will complete the SSC specific portions of the initialization
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SSC0
static void ssc0_configure(struct sam_ssc_s *priv)
{
  /* Configure multiplexed pins as connected on the board.  Chip
   * select pins must be selected by board-specific logic.
   */

#ifdef CONFIG_SAMA5_SSC0_RX
  priv->rxenab = true;

  /* Configure the receiver data (RD) and receiver frame synchro (RF) pins */

  sam_configpio(PIO_SSC0_RD);
  sam_configpio(PIO_SSC0_RF);

#if defined(CONFIG_SAMA5_SSC0_RX_RKINPUT)
  /* Configure the RK pin only if we are using an external clock to drive
   * the receiver clock.
   *
   * REVISIT:  The SSC is also capable of generated the receiver clock
   * output on the RK pin.
   */

  sam_configpio(PIO_SSC0_RK); /* External clock received on the RK I/O pad */
  priv->rxclk = SSC_CLKSRC_RKIN;

#elif defined(CONFIG_SAMA5_SSC0_RX_TXCLK)
  priv->rxclk = SSC_CLKSRC_TXOUT;

#elif defined(CONFIG_SAMA5_SSC0_RX_MCKDIV)
  priv->rxclk = SSC_CLKSRC_MCKDIV;

#else
  priv->rxclk = SSC_CLKSRC_NONE;

#endif

  /* Remember parameters of the configured waveform */

  priv->rxfslen  = CONFIG_SSC0_RX_FSLEN;
  priv->rxsttdly = CONFIG_SSC0_RX_STTDLY;

  /* Remember the configured RX clock output */

#if defined(CONFIG_SAMA5_SSC0_RX_RKOUTPUT_CONT)
  priv->rxout = SSC_CLKOUT_CONT; /* Continuous */
#elif defined(CONFIG_SAMA5_SSC0_RX_RKOUTPUT_XFR)
  priv->rxout = SSC_CLKOUT_XFER; /* Only output clock during transfers */
#else /* if defined(CONFIG_SAMA5_SSC0_RX_RKOUTPUT_NONE) */
  priv->rxout = SSC_CLKOUT_NONE; /* No output clock */
#endif

#else
  priv->rxenab = false;
  priv->rxclk  = SSC_CLKSRC_NONE; /* No input clock */
  priv->rxout  = SSC_CLKOUT_NONE; /* No output clock */

#endif /* CONFIG_SAMA5_SSC0_RX */

#ifdef CONFIG_SAMA5_SSC0_TX
  priv->txenab = true;

  /* Configure the transmitter data (TD) and transmitter frame synchro (TF)
   * pins
   */

  sam_configpio(PIO_SSC0_TD);
  sam_configpio(PIO_SSC0_TF);

#if defined(CONFIG_SAMA5_SSC0_TX_TKINPUT)
  /* Configure the TK pin only if we are using an external clock to drive
   * the transmitter clock.
   *
   * REVISIT:  The SSC is also capable of generated the transmitter clock
   * output on the TK pin.
   */

  sam_configpio(PIO_SSC0_TK); /* External clock received on the TK I/O pad */
  priv->txclk = SSC_CLKSRC_TKIN;

#elif defined(CONFIG_SAMA5_SSC0_TX_RXCLK)
  priv->txclk = SSC_CLKSRC_RXOUT;

#elif defined(CONFIG_SAMA5_SSC0_TX_MCKDIV)
  priv->txclk = SSC_CLKSRC_MCKDIV;

#else
  priv->txclk = SSC_CLKSRC_NONE;

#endif

  /* Remember the configured TX clock output */

#if defined(CONFIG_SAMA5_SSC0_TX_TKOUTPUT_CONT)
  priv->txout = SSC_CLKOUT_CONT; /* Continuous */
#elif defined(CONFIG_SAMA5_SSC0_TX_TKOUTPUT_XFR)
  priv->txout = SSC_CLKOUT_XFER; /* Only output clock during transfers */
#else /* if defined(CONFIG_SAMA5_SSC0_TX_TKOUTPUT_NONE) */
  priv->txout = SSC_CLKOUT_NONE; /* No output clock */
#endif

#else
  priv->txenab = false;
  priv->txclk  = SSC_CLKSRC_NONE; /* No input clock */
  priv->txout  = SSC_CLKOUT_NONE; /* No output clock */

#endif /* CONFIG_SAMA5_SSC0_TX */

  /* Remember parameters of the configured waveform */

  priv->txfslen  = CONFIG_SSC0_TX_FSLEN;
  priv->txsttdly = CONFIG_SSC0_TX_STTDLY;

  /* Set/clear loopback mode */

#if defined(CONFIG_SAMA5_SSC0_RX) && defined(CONFIG_SAMA5_SSC0_TX) && \
    defined(CONFIG_SAMA5_SSC0_LOOPBACK)
  priv->loopback = true;
#else
  priv->loopback = false;
#endif

  /* Does the receiver or transmitter need to have the MCK divider set up? */

#if defined(SSC0_HAVE_MCK2)
  priv->samplerate = CONFIG_SAMA5_SSC0_MCKDIV_SAMPLERATE;
#elif defined(SSC_HAVE_MCK2)
  priv->samplerate = 0;
#endif

  /* Configure driver state specific to this SSC peripheral */

  priv->base    = SAM_SSC0_VBASE;
  priv->datalen = CONFIG_SAMA5_SSC0_DATALEN;
#ifdef CONFIG_DEBUG
  priv->align   = SAMA5_SSC0_DATAMASK;
#endif
  priv->pid     = SAM_PID_SSC0;
}
#endif

#ifdef CONFIG_SAMA5_SSC1
static void ssc1_configure(struct sam_ssc_s *priv)
{
  /* Configure multiplexed pins as connected on the board.  Chip
   * select pins must be selected by board-specific logic.
   */

#ifdef CONFIG_SAMA5_SSC1_RX
  priv->rxenab = true;

  /* Configure the receiver data (RD) and receiver frame synchro (RF) pins */

  sam_configpio(PIO_SSC1_RD);
  sam_configpio(PIO_SSC1_RF);

#ifdef CONFIG_SAMA5_SSC1_RX_RKINPUT
  /* Configure the RK pin only if we are using an external clock to drive
   * the receiver clock.
   *
   * REVISIT:  The SSC is also capable of generated the receiver clock
   * output on the RK pin.
   */

  sam_configpio(PIO_SSC1_RK); /* External clock received on the RK I/O pad */
  priv->rxclk = SSC_CLKSRC_RKIN;

#elif defined(CONFIG_SAMA5_SSC1_RX_TXCLK)
  priv->rxclk = SSC_CLKSRC_TXOUT;

#elif defined(CONFIG_SAMA5_SSC1_RX_MCKDIV)
  priv->rxclk = SSC_CLKSRC_MCKDIV;

#else
  priv->rxclk = SSC_CLKSRC_NONE;

#endif

  /* Remember parameters of the configured waveform */

  priv->rxfslen  = CONFIG_SSC1_RX_FSLEN;
  priv->rxsttdly = CONFIG_SSC1_RX_STTDLY;

  /* Remember the configured RX clock output */

#if defined(CONFIG_SAMA5_SSC1_RX_RKOUTPUT_CONT)
  priv->rxout = SSC_CLKOUT_CONT; /* Continuous */
#elif defined(CONFIG_SAMA5_SSC1_RX_RKOUTPUT_XFR)
  priv->rxout = SSC_CLKOUT_XFER; /* Only output clock during transfers */
#else /* if defined(CONFIG_SAMA5_SSC1_RX_RKOUTPUT_NONE) */
  priv->rxout = SSC_CLKOUT_NONE; /* No output clock */
#endif

#else
  priv->rxenab = false;
  priv->rxclk  = SSC_CLKSRC_NONE; /* No input clock */
  priv->rxout  = SSC_CLKOUT_NONE; /* No output clock */

#endif /* CONFIG_SAMA5_SSC1_RX */

#ifdef CONFIG_SAMA5_SSC1_TX
  priv->txenab = true;

  /* Configure the transmitter data (TD) and transmitter frame synchro (TF)
   * pins
   */

  sam_configpio(PIO_SSC1_TD);
  sam_configpio(PIO_SSC1_TF);

#if defined(CONFIG_SAMA5_SSC1_TX_TKINPUT)
  /* Configure the TK pin only if we are using an external clock to drive
   * the transmitter clock.
   *
   * REVISIT:  The SSC is also capable of generated the transmitter clock
   * output on the TK pin.
   */

  sam_configpio(PIO_SSC1_TK); /* External clock received on the TK I/O pad */
  priv->txclk = SSC_CLKSRC_TKIN;

#elif defined(CONFIG_SAMA5_SSC1_TX_RXCLK)
  priv->txclk = SSC_CLKSRC_RXOUT;

#elif defined(CONFIG_SAMA5_SSC1_TX_MCKDIV)
  priv->txclk = SSC_CLKSRC_MCKDIV;

#else
  priv->txclk = SSC_CLKSRC_NONE;

#endif

  /* Remember the configured TX clock output */

#if defined(CONFIG_SAMA5_SSC1_TX_TKOUTPUT_CONT)
  priv->txout = SSC_CLKOUT_CONT; /* Continuous */
#elif defined(CONFIG_SAMA5_SSC1_TX_TKOUTPUT_XFR)
  priv->txout = SSC_CLKOUT_XFER;/* Only output clock during transfers */
#else /* if defined(CONFIG_SAMA5_SSC1_TX_TKOUTPUT_NONE) */
  priv->txout = SSC_CLKOUT_NONE; /* No output clock */
#endif

#else
  priv->txenab = false;
  priv->txclk  = SSC_CLKSRC_NONE; /* No input clock */
  priv->txout  = SSC_CLKOUT_NONE; /* No output clock */

#endif /* CONFIG_SAMA5_SSC1_TX */

  /* Remember parameters of the configured waveform */

  priv->txfslen  = CONFIG_SSC1_TX_FSLEN;
  priv->txsttdly = CONFIG_SSC1_TX_STTDLY;

  /* Set/clear loopback mode */

#if defined(CONFIG_SAMA5_SSC1_RX) && defined(CONFIG_SAMA5_SSC1_TX) && \
    defined(CONFIG_SAMA5_SSC1_LOOPBACK)
  priv->loopback = true;
#else
  priv->loopback = false;
#endif

  /* Does the receiver or transmitter need to have the MCK divider set up? */

#if defined(SSC1_HAVE_MCK2)
  priv->samplerate = CONFIG_SAMA5_SSC1_MCKDIV_SAMPLERATE;
#elif defined(SSC_HAVE_MCK2)
  priv->samplerate = 0;
#endif

  /* Configure driver state specific to this SSC peripheral */

  priv->base    = SAM_SSC1_VBASE;
  priv->datalen = CONFIG_SAMA5_SSC1_DATALEN;
#ifdef CONFIG_DEBUG
  priv->align   = SAMA5_SSC1_DATAMASK;
#endif
  priv->pid     = SAM_PID_SSC1;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_ssc_initialize
 *
 * Description:
 *   Initialize the selected SSC port
 *
 * Input Parameter:
 *   port - I2S "port" number (identifying the "logical" SSC port)
 *
 * Returned Value:
 *   Valid SSC device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *sam_ssc_initialize(int port)
{
  struct sam_ssc_s *priv;
  irqstate_t flags;
  int ret;

  /* The support SAM parts have only a single SSC port */

  i2svdbg("port: %d\n", port);

  /* Allocate a new state structure for this chip select.  NOTE that there
   * is no protection if the same chip select is used in two different
   * chip select structures.
   */

  priv = (struct sam_ssc_s *)zalloc(sizeof(struct sam_ssc_s));
  if (!priv)
    {
      i2sdbg("ERROR: Failed to allocate a chip select structure\n");
      return NULL;
    }

  /* Set up the initial state for this chip select structure.  Other fields
   * were zeroed by zalloc().
   */

  /* Initialize the common parts for the SSC device structure  */

  sem_init(&priv->exclsem, 0, 1);
  priv->dev.ops = &g_sscops;
  priv->sscno   = port;

  /* Initialize buffering */

  ssc_buf_initialize(priv);

  flags = irqsave();
#ifdef CONFIG_SAMA5_SSC0
  if (port == 0)
    {
      ssc0_configure(priv);
    }
  else
#endif /* CONFIG_SAMA5_SSC0 */
#ifdef CONFIG_SAMA5_SSC1
  if (port == 1)
    {
      ssc1_configure(priv);
    }
  else
#endif /* CONFIG_SAMA5_SSC1 */
    {
      i2sdbg("ERROR:  Unsupported I2S port: %d\n", port);
      goto errout_with_alloc;
    }

  /* Allocate DMA channels */

  ret = ssc_dma_allocate(priv);
  if (ret < 0)
    {
      goto errout_with_alloc;
    }

  /* Configure and enable clocking */

  ssc_clocking(priv);

  /* Configure the receiver */

  ret = ssc_rx_configure(priv);
  if (ret < 0)
    {
      i2sdbg("ERROR: Failed to configure the receiver: %d\n", ret);
      goto errout_with_clocking;
    }

  /* Configure the transmitter */

  ret = ssc_tx_configure(priv);
  if (ret < 0)
    {
      i2sdbg("ERROR: Failed to configure the transmitter: %d\n", ret);
      goto errout_with_clocking;
    }

  irqrestore(flags);
  scc_dump_regs(priv, "After initialization");

  /* Success exit */

  return &priv->dev;

  /* Failure exits */

errout_with_clocking:
  sam_disableperiph1(priv->pid);
  ssc_dma_free(priv);

errout_with_alloc:
  sem_destroy(&priv->exclsem);
  kmm_free(priv);
  return NULL;
}

#endif /* SSC_HAVE_RX || SSC_HAVE_TX */
#endif /* CONFIG_SAMA5_SSC0 || CONFIG_SAMA5_SSC1 */

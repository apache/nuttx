/****************************************************************************
 * arch/arm/src/stm32/stm32_i2s.c
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
 * The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status
 * must be provided by board-specific logic.  They are implementations of
 * the select and status methods of the SPI interface defined by struct
 * spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 * up_spiinitialize()) are provided by common STM32 logic.  To use this
 * common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure I2S chip
 *      select pins.
 *   2. Provide stm32_i2s2/3select() and stm32_i2s2/3status() functions in
 *      your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board
 *      is configured.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by stm32_i2sbus_initialize() may then be used to
 *     bind the I2S driver to higher level logic
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/spi/spi.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32_dma.h"
#include "stm32_spi.h"
#include "stm32_rcc.h"

#if defined(CONFIG_STM32_I2S2) || defined(CONFIG_STM32_I2S3)

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

#ifndef CONFIG_STM32_I2S_MAXINFLIGHT
#  define CONFIG_STM32_I2S_MAXINFLIGHT 16
#endif

/* Assume no RX/TX support until we learn better */

#undef I2S_HAVE_RX
#undef I2S_HAVE_TX

/* Check for I2S RX support */

#  if defined(CONFIG_STM32_I2S3_RX)
#    define I2S_HAVE_RX 1

#    ifdef CONFIG_STM32_I2S_MCK
#      define I2S_HAVE_MCK  1
#    endif

#  endif

/* Check for I2S3 TX support */

#  if defined(CONFIG_STM32_I2S3_TX)
#    define I2S_HAVE_TX 1

#    ifdef CONFIG_STM32_I2S_MCK
#      define I2S_HAVE_MCK  1
#    endif

#  endif

/* Configuration ************************************************************/

/* I2S interrupts */

#ifdef CONFIG_STM32_SPI_INTERRUPTS
#  error "Interrupt driven I2S not yet supported"
#endif

/* Can't have both interrupt driven SPI and SPI DMA */

#if defined(CONFIG_STM32_SPI_INTERRUPTS) && defined(CONFIG_STM32_SPI_DMA)
#  error "Cannot enable both interrupt mode and DMA mode for SPI"
#endif

/* SPI DMA priority */

#ifdef CONFIG_STM32_SPI_DMA

#  if defined(CONFIG_SPI_DMAPRIO)
#    define SPI_DMA_PRIO  CONFIG_SPI_DMAPRIO
#  elif defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32L15XX)
#    define SPI_DMA_PRIO  DMA_CCR_PRIMED
#  elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#    define SPI_DMA_PRIO  DMA_SCR_PRIMED
#  else
#    error "Unknown STM32 DMA"
#  endif

#  if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32L15XX)
#    if (SPI_DMA_PRIO & ~DMA_CCR_PL_MASK) != 0
#      error "Illegal value for CONFIG_SPI_DMAPRIO"
#    endif
#  elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#    if (SPI_DMA_PRIO & ~DMA_SCR_PL_MASK) != 0
#      error "Illegal value for CONFIG_SPI_DMAPRIO"
#    endif
#  else
#    error "Unknown STM32 DMA"
#  endif

#endif

/* DMA channel configuration */

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX) || \
    defined(CONFIG_STM32_STM32L15XX)
#  define SPI_RXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_CCR_MSIZE_16BITS|DMA_CCR_PSIZE_16BITS|DMA_CCR_MINC            )
#  define SPI_RXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS |DMA_CCR_MINC            )
#  define SPI_RXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_16BITS                         )
#  define SPI_RXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS                          )
#  define SPI_TXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_CCR_MSIZE_16BITS|DMA_CCR_PSIZE_16BITS|DMA_CCR_MINC|DMA_CCR_DIR)
#  define SPI_TXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS |DMA_CCR_MINC|DMA_CCR_DIR)
#  define SPI_TXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_16BITS             |DMA_CCR_DIR)
#  define SPI_TXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS              |DMA_CCR_DIR)
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#  define SPI_RXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_SCR_MSIZE_16BITS|DMA_SCR_PSIZE_16BITS|DMA_SCR_MINC|DMA_SCR_DIR_P2M)
#  define SPI_RXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS |DMA_SCR_MINC|DMA_SCR_DIR_P2M)
#  define SPI_RXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_16BITS             |DMA_SCR_DIR_P2M)
#  define SPI_RXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS              |DMA_SCR_DIR_P2M)
#  define SPI_TXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_SCR_MSIZE_16BITS|DMA_SCR_PSIZE_16BITS|DMA_SCR_MINC|DMA_SCR_DIR_M2P)
#  define SPI_TXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS |DMA_SCR_MINC|DMA_SCR_DIR_M2P)
#  define SPI_TXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_16BITS             |DMA_SCR_DIR_M2P)
#  define SPI_TXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS              |DMA_SCR_DIR_M2P)
#else
#  error "Unknown STM32 DMA"
#endif

/* Debug ********************************************************************/

/* Check if SSC debug is enabled (non-standard.. no support in
 * include/debug.h
 */

#ifndef CONFIG_DEBUG_I2S_INFO
#  undef CONFIG_STM32_I2S_DMADEBUG
#  undef CONFIG_STM32_I2S_REGDEBUG
#  undef CONFIG_STM32_I2S_QDEBUG
#  undef CONFIG_STM32_I2S_DUMPBUFFERS
#endif

/* The I2S can handle most any bit width from 8 to 32.  However, the DMA
 * logic here is constrained to byte, half-word, and word sizes.
 */

#ifndef CONFIG_STM32_I2S3_DATALEN
#  define CONFIG_STM32_I2S3_DATALEN 16
#endif

#if CONFIG_STM32_I2S3_DATALEN == 8
#  define STM32_I2S3_DATAMASK  0
#elif CONFIG_STM32_I2S3_DATALEN == 16
#  define STM32_I2S3_DATAMASK  1
#elif  CONFIG_STM32_I2S3_DATALEN < 8 || CONFIG_STM32_I2S3_DATALEN > 16
#  error Invalid value for CONFIG_STM32_I2S3_DATALEN
#else
#  error Valid but supported value for CONFIG_STM32_I2S3_DATALEN
#endif

/* Check if we need to build RX and/or TX support */

#if defined(I2S_HAVE_RX) || defined(I2S_HAVE_TX)

#ifndef CONFIG_DEBUG_DMA
#  undef CONFIG_STM32_I2S_DMADEBUG
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

struct stm32_buffer_s
{
  struct stm32_buffer_s *flink; /* Supports a singly linked list */
  i2s_callback_t callback;      /* Function to call when the transfer
                                 * completes */
  uint32_t timeout;             /* The timeout value to use with DMA
                                 * transfers */
  void *arg;                    /* The argument to be returned with the
                                 * callback */
  struct ap_buffer_s *apb;      /* The audio buffer */
  int result;                   /* The result of the transfer */
};

/* This structure describes the state of one receiver or transmitter
 * transport.
 */

struct stm32_transport_s
{
  DMA_HANDLE dma;               /* I2S DMA handle */
  struct wdog_s dog;            /* Watchdog that handles DMA timeouts */
  sq_queue_t pend;              /* A queue of pending transfers */
  sq_queue_t act;               /* A queue of active transfers */
  sq_queue_t done;              /* A queue of completed transfers */
  struct work_s work;           /* Supports worker thread operations */

#ifdef CONFIG_STM32_I2S_DMADEBUG
  struct stm32_dmaregs_s dmaregs[DMA_NSAMPLES];
#endif
};

/* The state of the one I2S peripheral */

struct stm32_i2s_s
{
  struct i2s_dev_s  dev;          /* Externally visible I2S interface */
  uintptr_t         base;         /* I2S controller register base address */
  mutex_t           lock;         /* Assures mutually exclusive access to I2S */
  bool              initialized;  /* Has I2S interface been initialized */
  uint8_t           datalen;      /* Data width (8 or 16) */
  uint8_t           align;        /* Log2 of data width (0 or 1) */
  uint8_t           rxenab:1;     /* True: RX transfers enabled */
  uint8_t           txenab:1;     /* True: TX transfers enabled */
  uint8_t           i2sno:6;      /* I2S controller number (0 or 1) */
#ifdef I2S_HAVE_MCK
  uint32_t          samplerate;   /* Data sample rate (determines only MCK
                                   * divider) */
#endif
  uint32_t          rxccr;        /* DMA control register for RX transfers */
  uint32_t          txccr;        /* DMA control register for TX transfers */
#ifdef I2S_HAVE_RX
  struct stm32_transport_s rx;    /* RX transport state */
#endif
#ifdef I2S_HAVE_TX
  struct stm32_transport_s tx;    /* TX transport state */
#endif

  /* Pre-allocated pool of buffer containers */

  sem_t bufsem;                    /* Buffer wait semaphore */
  struct stm32_buffer_s *freelist; /* A list a free buffer containers */
  struct stm32_buffer_s containers[CONFIG_STM32_I2S_MAXINFLIGHT];

  /* Debug stuff */

#ifdef CONFIG_STM32_I2S_REGDEBUG
  bool     wr;                     /* Last was a write */
  uint32_t regaddr;                /* Last address */
  uint16_t regval;                 /* Last value */
  int      count;                  /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register helpers */

#ifdef CONFIG_STM32_I2S_REGDEBUG
static bool     i2s_checkreg(struct stm32_i2s_s *priv, bool wr,
                             uint16_t regval, uint32_t regaddr);
#else
# define        i2s_checkreg(priv,wr,regval,regaddr) (false)
#endif

static inline uint16_t i2s_getreg(struct stm32_i2s_s *priv, uint8_t offset);
static inline void i2s_putreg(struct stm32_i2s_s *priv, uint8_t offset,
                  uint16_t regval);

#if defined(CONFIG_DEBUG_I2S_INFO)
static void     i2s_dump_regs(struct stm32_i2s_s *priv, const char *msg);
#else
#  define       i2s_dump_regs(s,m)
#endif

#ifdef CONFIG_STM32_I2S_DUMPBUFFERS
#  define       i2s_init_buffer(b,s)   memset(b, 0x55, s);
#  define       i2s_dump_buffer(m,b,s) lib_dumpbuffer(m,b,s)
#else
#  define       i2s_init_buffer(b,s)
#  define       i2s_dump_buffer(m,b,s)
#endif

/* Buffer container helpers */

static struct stm32_buffer_s *
                i2s_buf_allocate(struct stm32_i2s_s *priv);
static void     i2s_buf_free(struct stm32_i2s_s *priv,
                  struct stm32_buffer_s *bfcontainer);
static void     i2s_buf_initialize(struct stm32_i2s_s *priv);

/* DMA support */

#ifdef CONFIG_STM32_I2S_DMADEBUG
static void     i2s_dma_sampleinit(struct stm32_i2s_s *priv,
                  struct stm32_transport_s *xpt);
#endif

#if defined(CONFIG_STM32_I2S_DMADEBUG) && defined(I2S_HAVE_RX)
#  define       i2s_rxdma_sample(s,i) stm32_dmasample((s)->rx.dma, &(s)->rx.dmaregs[i])
#  define       i2s_rxdma_sampleinit(s) i2s_dma_sampleinit(s, &(s)->rx)
static void     i2s_rxdma_sampledone(struct stm32_i2s_s *priv, int result);

#else
#  define       i2s_rxdma_sample(s,i)
#  define       i2s_rxdma_sampleinit(s)
#  define       i2s_rxdma_sampledone(s,r)

#endif

#if defined(CONFIG_STM32_I2S_DMADEBUG) && defined(I2S_HAVE_TX)
#  define       i2s_txdma_sample(s,i) stm32_dmasample((s)->tx.dma, &(s)->tx.dmaregs[i])
#  define       i2s_txdma_sampleinit(s) i2s_dma_sampleinit(s, &(s)->tx)
static void     i2s_txdma_sampledone(struct stm32_i2s_s *priv, int result);

#else
#  define       i2s_txdma_sample(s,i)
#  define       i2s_txdma_sampleinit(s)
#  define       i2s_txdma_sampledone(s,r)

#endif

#ifdef I2S_HAVE_RX
static void     i2s_rxdma_timeout(wdparm_t arg);
static int      i2s_rxdma_setup(struct stm32_i2s_s *priv);
static void     i2s_rx_worker(void *arg);
static void     i2s_rx_schedule(struct stm32_i2s_s *priv, int result);
static void     i2s_rxdma_callback(DMA_HANDLE handle, uint8_t result,
                                   void *arg);
#endif
#ifdef I2S_HAVE_TX
static void     i2s_txdma_timeout(wdparm_t arg);
static int      i2s_txdma_setup(struct stm32_i2s_s *priv);
static void     i2s_tx_worker(void *arg);
static void     i2s_tx_schedule(struct stm32_i2s_s *priv, int result);
static void     i2s_txdma_callback(DMA_HANDLE handle, uint8_t result,
                                   void *arg);
#endif

/* I2S methods (and close friends) */

static int      i2s_checkwidth(struct stm32_i2s_s *priv, int bits);

static uint32_t stm32_i2s_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t stm32_i2s_rxdatawidth(struct i2s_dev_s *dev, int bits);
static int      stm32_i2s_receive(struct i2s_dev_s *dev,
                                  struct ap_buffer_s *apb,
                                  i2s_callback_t callback,
                                  void *arg, uint32_t timeout);
static uint32_t stm32_i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t stm32_i2s_txdatawidth(struct i2s_dev_s *dev, int bits);
static int      stm32_i2s_send(struct i2s_dev_s *dev,
                               struct ap_buffer_s *apb,
                               i2s_callback_t callback, void *arg,
                               uint32_t timeout);

/* Initialization */

static uint32_t i2s_mckdivider(struct stm32_i2s_s *priv);
static int      i2s_dma_flags(struct stm32_i2s_s *priv);
static int      i2s_dma_allocate(struct stm32_i2s_s *priv);
static void     i2s_dma_free(struct stm32_i2s_s *priv);
#ifdef CONFIG_STM32_I2S2
static void     i2s2_configure(struct stm32_i2s_s *priv);
#endif
#ifdef CONFIG_STM32_I2S3
static void     i2s3_configure(struct stm32_i2s_s *priv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2S device operations */

static const struct i2s_ops_s g_i2sops =
{
  /* Receiver methods */

  .i2s_rxsamplerate = stm32_i2s_rxsamplerate,
  .i2s_rxdatawidth  = stm32_i2s_rxdatawidth,
  .i2s_receive      = stm32_i2s_receive,

  /* Transmitter methods */

  .i2s_txsamplerate = stm32_i2s_txsamplerate,
  .i2s_txdatawidth  = stm32_i2s_txdatawidth,
  .i2s_send         = stm32_i2s_send,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2s_checkreg
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

#ifdef CONFIG_STM32_I2S_REGDEBUG
static bool i2s_checkreg(struct stm32_i2s_s *priv, bool wr, uint16_t regval,
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

          i2sinfo("...[Repeats %d times]...\n", priv->count);
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
 * Name: i2s_getreg
 *
 * Description:
 *   Get the contents of the I2S register at offset
 *
 * Input Parameters:
 *   priv   - private I2S device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ****************************************************************************/

static inline uint16_t i2s_getreg(struct stm32_i2s_s *priv,
                                  uint8_t offset)
{
  uint32_t regaddr = priv->base + offset;
  uint16_t regval = getreg16(regaddr);

#ifdef CONFIG_STM32_I2S_REGDEBUG
  if (i2s_checkreg(priv, false, regval, regaddr))
    {
      i2sinfo("%08x->%04x\n", regaddr, regval);
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 16-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ****************************************************************************/

static inline void i2s_putreg(struct stm32_i2s_s *priv, uint8_t offset,
                              uint16_t regval)
{
  uint32_t regaddr = priv->base + offset;

#ifdef CONFIG_STM32_I2S_REGDEBUG
  if (i2s_checkreg(priv, true, regval, regaddr))
    {
      i2sinfo("%08x<-%04x\n", regaddr, regval);
    }
#endif

  putreg16(regval, regaddr);
}

/****************************************************************************
 * Name: i2s_dump_regs
 *
 * Description:
 *   Dump the contents of all I2S registers
 *
 * Input Parameters:
 *   priv - The I2S controller to dump
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_I2S)
static void i2s_dump_regs(struct stm32_i2s_s *priv, const char *msg)
{
  i2sinfo("I2S%d: %s\n", priv->i2sno, msg);
  i2sinfo("    CR1:%04x    CR2:%04x     SR:%04x      DR:%04x\n",
          i2s_getreg(priv, STM32_SPI_CR1_OFFSET),
          i2s_getreg(priv, STM32_SPI_CR2_OFFSET),
          i2s_getreg(priv, STM32_SPI_SR_OFFSET),
          i2s_getreg(priv, STM32_SPI_DR_OFFSET));
  i2sinfo("    I2SCFGR:%04x    I2SPR:%04x\n",
          i2s_getreg(priv, STM32_SPI_I2SCFGR_OFFSET),
          i2s_getreg(priv, STM32_SPI_I2SPR_OFFSET));
}
#endif

/****************************************************************************
 * Name: i2s_buf_allocate
 *
 * Description:
 *   Allocate a buffer container by removing the one at the head of the
 *   free list
 *
 * Input Parameters:
 *   priv - I2S state instance
 *
 * Returned Value:
 *   A non-NULL pointer to the allocate buffer container on success; NULL if
 *   there are no available buffer containers.
 *
 * Assumptions:
 *   The caller does NOT have exclusive access to the I2S state structure.
 *   That would result in a deadlock!
 *
 ****************************************************************************/

static struct stm32_buffer_s *i2s_buf_allocate(struct stm32_i2s_s *priv)
{
  struct stm32_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;

  /* Set aside a buffer container.  By doing this, we guarantee that we will
   * have at least one free buffer container.
   */

  ret = nxsem_wait_uninterruptible(&priv->bufsem);
  if (ret < 0)
    {
      return NULL;
    }

  /* Get the buffer from the head of the free list */

  flags = enter_critical_section();
  bfcontainer = priv->freelist;
  DEBUGASSERT(bfcontainer);

  /* Unlink the buffer from the freelist */

  priv->freelist = bfcontainer->flink;
  leave_critical_section(flags);
  return bfcontainer;
}

/****************************************************************************
 * Name: i2s_buf_free
 *
 * Description:
 *   Free buffer container by adding it to the head of the free list
 *
 * Input Parameters:
 *   priv - I2S state instance
 *   bfcontainer - The buffer container to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has exclusive access to the I2S state structure
 *
 ****************************************************************************/

static void i2s_buf_free(struct stm32_i2s_s *priv,
                         struct stm32_buffer_s *bfcontainer)
{
  irqstate_t flags;

  /* Put the buffer container back on the free list */

  flags = enter_critical_section();
  bfcontainer->flink  = priv->freelist;
  priv->freelist = bfcontainer;
  leave_critical_section(flags);

  /* Wake up any threads waiting for a buffer container */

  nxsem_post(&priv->bufsem);
}

/****************************************************************************
 * Name: i2s_buf_initialize
 *
 * Description:
 *   Initialize the buffer container allocator by adding all of the
 *   pre-allocated buffer containers to the free list
 *
 * Input Parameters:
 *   priv - I2S state instance
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in I2S initialization so that there are no issues with
 *   concurrency.
 *
 ****************************************************************************/

static void i2s_buf_initialize(struct stm32_i2s_s *priv)
{
  int i;

  priv->freelist = NULL;
  nxsem_init(&priv->bufsem, 0, CONFIG_STM32_I2S_MAXINFLIGHT);

  for (i = 0; i < CONFIG_STM32_I2S_MAXINFLIGHT; i++)
    {
      i2s_buf_free(priv, &priv->containers[i]);
    }
}

/****************************************************************************
 * Name: i2s_dma_sampleinit
 *
 * Description:
 *   Initialize sampling of DMA registers (if CONFIG_STM32_I2S_DMADEBUG)
 *
 * Input Parameters:
 *   priv - I2S state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_I2S_DMADEBUG)
static void i2s_dma_sampleinit(struct stm32_i2s_s *priv,
                               struct stm32_transport_s *xpt)
{
  /* Put contents of register samples into a known state */

  memset(xpt->dmaregs, 0xff, DMA_NSAMPLES * sizeof(struct stm32_dmaregs_s));

  /* Then get the initial samples */

  stm32_dmasample(xpt->dma, &xpt->dmaregs[DMA_INITIAL]);
}
#endif

/****************************************************************************
 * Name: i2s_rxdma_sampledone
 *
 * Description:
 *   Dump sampled RX DMA registers
 *
 * Input Parameters:
 *   priv - I2S state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_I2S_DMADEBUG) && defined(I2S_HAVE_RX)
static void i2s_rxdma_sampledone(struct stm32_i2s_s *priv, int result)
{
  i2sinfo("result: %d\n", result);

  /* Sample the final registers */

  stm32_dmasample(priv->rx.dma, &priv->rx.dmaregs[DMA_END_TRANSFER]);

  /* Then dump the sampled DMA registers */

  /* Initial register values */

  stm32_dmadump(priv->rx.dma, &priv->rx.dmaregs[DMA_INITIAL],
              "RX: Initial Registers");

  /* Register values after DMA setup */

  stm32_dmadump(priv->rx.dma, &priv->rx.dmaregs[DMA_AFTER_SETUP],
              "RX: After DMA Setup");

  /* Register values after DMA start */

  stm32_dmadump(priv->rx.dma, &priv->rx.dmaregs[DMA_AFTER_START],
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
      stm32_dmadump(priv->rx.dma, &priv->rx.dmaregs[DMA_TIMEOUT],
                  "RX: At DMA timeout");
    }
  else
    {
      stm32_dmadump(priv->rx.dma, &priv->rx.dmaregs[DMA_CALLBACK],
                  "RX: At DMA callback");
    }

  stm32_dmadump(priv->rx.dma, &priv->rx.dmaregs[DMA_END_TRANSFER],
              "RX: At End-of-Transfer");

  i2s_dump_regs(priv, "RX: At End-of-Transfer");
}
#endif

/****************************************************************************
 * Name: i2s_txdma_sampledone
 *
 * Description:
 *   Dump sampled DMA registers
 *
 * Input Parameters:
 *   priv - I2S state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_I2S_DMADEBUG) && defined(I2S_HAVE_TX)
static void i2s_txdma_sampledone(struct stm32_i2s_s *priv, int result)
{
  i2sinfo("result: %d\n", result);

  /* Sample the final registers */

  stm32_dmasample(priv->tx.dma, &priv->tx.dmaregs[DMA_END_TRANSFER]);

  /* Then dump the sampled DMA registers */

  /* Initial register values */

  stm32_dmadump(priv->tx.dma, &priv->tx.dmaregs[DMA_INITIAL],
                "TX: Initial Registers");

  /* Register values after DMA setup */

  stm32_dmadump(priv->tx.dma, &priv->tx.dmaregs[DMA_AFTER_SETUP],
                "TX: After DMA Setup");

  /* Register values after DMA start */

  stm32_dmadump(priv->tx.dma, &priv->tx.dmaregs[DMA_AFTER_START],
                "TX: After DMA Start");

  /* Register values at the time of the TX and RX DMA callbacks
   * -OR- DMA timeout.
   */

  if (result == -ETIMEDOUT || result == -EINTR)
    {
      stm32_dmadump(priv->tx.dma, &priv->tx.dmaregs[DMA_TIMEOUT],
                    "TX: At DMA timeout");
    }
  else
    {
      stm32_dmadump(priv->tx.dma, &priv->tx.dmaregs[DMA_CALLBACK],
                    "TX: At DMA callback");
    }

  stm32_dmadump(priv->tx.dma, &priv->tx.dmaregs[DMA_END_TRANSFER],
              "TX: At End-of-Transfer");

  i2s_dump_regs(priv, "TX: At End-of-Transfer");
}
#endif

/****************************************************************************
 * Name: i2s_rxdma_timeout
 *
 * Description:
 *   The RX watchdog timeout without completion of the RX DMA.
 *
 * Input Parameters:
 *   arg    - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static void i2s_rxdma_timeout(wdparm_t arg)
{
  struct stm32_i2s_s *priv = (struct stm32_i2s_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Sample DMA registers at the time of the timeout */

  i2s_rxdma_sample(priv, DMA_TIMEOUT);

  /* Cancel the DMA */

  stm32_dmastop(priv->rx.dma);

  /* Then schedule completion of the transfer to occur on the worker thread.
   * NOTE: stm32_dmastop() will call the DMA complete callback with an error
   * of -EINTR.  So the following is just insurance and should have no
   * effect if the worker is already schedule.
   */

  i2s_rx_schedule(priv, -ETIMEDOUT);
}
#endif

/****************************************************************************
 * Name: i2s_rxdma_setup
 *
 * Description:
 *   Setup and initiate the next RX DMA transfer
 *
 * Input Parameters:
 *   priv - I2S state instance
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static int i2s_rxdma_setup(struct stm32_i2s_s *priv)
{
  struct stm32_buffer_s *bfcontainer;
  struct ap_buffer_s *apb;
  uintptr_t samp;
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

  i2s_rxdma_sampleinit(priv);

  /* Loop, adding each pending DMA */

  timeout = 0;
  notimeout = false;

  do
    {
      /* Remove the pending RX transfer at the head of the RX pending
       * queue.
       */

      bfcontainer = (struct stm32_buffer_s *)sq_remfirst(&priv->rx.pend);
      DEBUGASSERT(bfcontainer && bfcontainer->apb);

      apb = bfcontainer->apb;
      DEBUGASSERT(((uintptr_t)apb->samp % priv->align) == 0);

      /* No data received yet */

      apb->nbytes  = 0;
      apb->curbyte = 0;
      samp   = (uintptr_t)&apb->samp[apb->curbyte];

      /* Configure the RX DMA */

      stm32_dmasetup(priv->rx.dma, priv->base + STM32_SPI_DR_OFFSET,
                    (uint32_t)samp, apb->nmaxbytes, priv->rxccr);

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
    }
#if 1 /* REVISIT: Chained RX transfers */
  while (0);
#else
  while (!sq_empty(&priv->rx.pend));
#endif

  /* Sample DMA registers */

  i2s_rxdma_sample(priv, DMA_AFTER_SETUP);

  /* Start the DMA, saving the container as the current active transfer */

  stm32_dmastart(priv->rx.dma, i2s_rxdma_callback, priv, false);

  i2s_rxdma_sample(priv, DMA_AFTER_START);

  /* Enable the receiver */

  i2s_putreg(priv, STM32_SPI_CR2_OFFSET,
             i2s_getreg(priv, STM32_SPI_CR2_OFFSET) | SPI_CR2_RXDMAEN);

  /* Start a watchdog to catch DMA timeouts */

  if (!notimeout)
    {
      ret = wd_start(&priv->rx.dog, timeout,
                     i2s_rxdma_timeout, (wdparm_t)priv);

      /* Check if we have successfully started the watchdog timer.  Note
       * that we do nothing in the case of failure to start the timer.  We
       * are already committed to the DMA anyway.  Let's just hope that the
       * DMA does not hang.
       */

      if (ret < 0)
        {
          i2serr("ERROR: wd_start failed: %d\n", ret);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: i2s_rx_worker
 *
 * Description:
 *   RX transfer done worker
 *
 * Input Parameters:
 *   arg - the I2S device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static void i2s_rx_worker(void *arg)
{
  struct stm32_i2s_s *priv = (struct stm32_i2s_s *)arg;
  struct stm32_buffer_s *bfcontainer;
  struct ap_buffer_s *apb;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* When the transfer was started, the active buffer containers were removed
   * from the rx.pend queue and saved in the rx.act queue.  We get here when
   * the DMA is finished... either successfully, with a DMA error, or with a
   * DMA timeout.
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

  i2sinfo("rx.act.head=%p rx.done.head=%p\n",
          priv->rx.act.head, priv->rx.done.head);

  /* Check if the DMA is IDLE */

  if (sq_empty(&priv->rx.act))
    {
#ifdef CONFIG_STM32_I2S_DMADEBUG
      bfcontainer = (struct stm32_buffer_s *)sq_peek(&priv->rx.done);
      if (bfcontainer)
        {
          /* Dump the DMA registers */

          i2s_rxdma_sampledone(priv, bfcontainer->result);
        }
#endif

      /* Then start the next DMA.  This must be done with interrupts
       * disabled.
       */

      flags = enter_critical_section();
      i2s_rxdma_setup(priv);
      leave_critical_section(flags);
    }

  /* Process each buffer in the rx.done queue */

  while (sq_peek(&priv->rx.done) != NULL)
    {
      /* Remove the buffer container from the rx.done queue.  NOTE that
       * interrupts must be enabled to do this because the rx.done queue is
       * also modified from the interrupt level.
       */

      flags = enter_critical_section();
      bfcontainer = (struct stm32_buffer_s *)sq_remfirst(&priv->rx.done);
      leave_critical_section(flags);

      DEBUGASSERT(bfcontainer && bfcontainer->apb && bfcontainer->callback);
      apb = bfcontainer->apb;

      /* If the DMA was successful, then update the number of valid bytes in
       * the audio buffer.
       */

      if (bfcontainer->result == OK)
        {
          apb->nbytes = apb->nmaxbytes;
        }

      i2s_dump_buffer("Received", apb->samp, apb->nbytes);

      /* Perform the RX transfer done callback */

      bfcontainer->callback(&priv->dev, apb, bfcontainer->arg,
                            bfcontainer->result);

      /* Release our reference on the audio buffer.  This may very likely
       * cause the audio buffer to be freed.
       */

      apb_free(apb);

      /* And release the buffer container */

      i2s_buf_free(priv, bfcontainer);
    }
}
#endif

/****************************************************************************
 * Name: i2s_rx_schedule
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

#ifdef I2S_HAVE_RX
static void i2s_rx_schedule(struct stm32_i2s_s *priv, int result)
{
  struct stm32_buffer_s *bfcontainer;
  int ret;

  /* Upon entry, the transfer(s) that just completed are the ones in the
   * priv->rx.act queue.  NOTE: In certain conditions, this function may
   * be called an additional time, hence, we can't assert this to be true.
   * For example, in the case of a timeout, this function will be called by
   * both indirectly via the stm32_dmastop() logic and directly via the
   * i2s_rxdma_timeout() logic.
   */

  /* Move all entries from the rx.act queue to the rx.done queue */

  while (!sq_empty(&priv->rx.act))
    {
      /* Remove the next buffer container from the rx.act list */

      bfcontainer = (struct stm32_buffer_s *)sq_remfirst(&priv->rx.act);

      /* Report the result of the transfer */

      bfcontainer->result = result;

      /* Add the completed buffer container to the tail of the rx.done
       * queue
       */

      sq_addlast((sq_entry_t *)bfcontainer, &priv->rx.done);
    }

  /* If the worker has completed running, then reschedule the working thread.
   * REVISIT:  There may be a race condition here.  So we do nothing is the
   * worker is not available.
   */

  if (work_available(&priv->rx.work))
    {
      /* Schedule the TX DMA done processing to occur on the worker thread. */

      ret = work_queue(HPWORK, &priv->rx.work, i2s_rx_worker, priv, 0);
      if (ret != 0)
        {
          i2serr("ERROR: Failed to queue RX work: %d\n", ret);
        }
    }
}
#endif

/****************************************************************************
 * Name: i2s_rxdma_callback
 *
 * Description:
 *   This callback function is invoked at the completion of the I2S RX DMA.
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

#ifdef I2S_HAVE_RX
static void i2s_rxdma_callback(DMA_HANDLE handle, uint8_t result, void *arg)
{
  struct stm32_i2s_s *priv = (struct stm32_i2s_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Cancel the watchdog timeout */

  wd_cancel(&priv->rx.dog);

  /* Sample DMA registers at the time of the DMA completion */

  i2s_rxdma_sample(priv, DMA_CALLBACK);

  /* REVISIT:  We would like to the next DMA started here so that we do not
   * get audio glitches at the boundaries between DMA transfers.
   * Unfortunately, we cannot call stm32_dmasetup() from an interrupt
   * handler!
   */

  /* Then schedule completion of the transfer to occur on the worker thread */

  i2s_rx_schedule(priv, result);
}
#endif

/****************************************************************************
 * Name: i2s_txdma_timeout
 *
 * Description:
 *   The RX watchdog timeout without completion of the RX DMA.
 *
 * Input Parameters:
 *   arg    - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static void i2s_txdma_timeout(wdparm_t arg)
{
  struct stm32_i2s_s *priv = (struct stm32_i2s_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Sample DMA registers at the time of the timeout */

  i2s_txdma_sample(priv, DMA_TIMEOUT);

  /* Cancel the DMA */

  stm32_dmastop(priv->tx.dma);

  /* Then schedule completion of the transfer to occur on the worker thread.
   * NOTE: stm32_dmastop() will call the DMA complete callback with an error
   * of -EINTR.  So the following is just insurance and should have no
   * effect if the worker is already schedule.
   */

  i2s_tx_schedule(priv, -ETIMEDOUT);
}
#endif

/****************************************************************************
 * Name: i2s_txdma_setup
 *
 * Description:
 *   Setup and initiate the next TX DMA transfer
 *
 * Input Parameters:
 *   priv - I2S state instance
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static int i2s_txdma_setup(struct stm32_i2s_s *priv)
{
  struct stm32_buffer_s *bfcontainer;
  struct ap_buffer_s *apb;
  uintptr_t samp;
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

  i2s_txdma_sampleinit(priv);

  /* Loop, adding each pending DMA */

  timeout = 0;
  notimeout = false;

  do
    {
      /* Remove the pending TX transfer at the head of the TX pending
       * queue.
       */

      bfcontainer = (struct stm32_buffer_s *)sq_remfirst(&priv->tx.pend);
      DEBUGASSERT(bfcontainer && bfcontainer->apb);

      apb = bfcontainer->apb;

      /* Get the transfer information, accounting for any data offset */

      samp   = (uintptr_t)&apb->samp[apb->curbyte];
      nbytes = apb->nbytes - apb->curbyte;
      DEBUGASSERT((samp & priv->align) == 0 && (nbytes & priv->align) == 0);

      /* Configure DMA stream */

      stm32_dmasetup(priv->tx.dma, priv->base + STM32_SPI_DR_OFFSET,
                     (uint32_t)samp, nbytes / 2, priv->txccr);

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
    }
#if 1 /* REVISIT: Chained TX transfers */
  while (0);
#else
  while (!sq_empty(&priv->tx.pend));
#endif

  /* Sample DMA registers */

  i2s_txdma_sample(priv, DMA_AFTER_SETUP);

  /* Start the DMA, saving the container as the current active transfer */

  stm32_dmastart(priv->tx.dma, i2s_txdma_callback, priv, true);

  i2s_txdma_sample(priv, DMA_AFTER_START);

  /* Enable the transmitter */

  i2s_putreg(priv, STM32_SPI_CR2_OFFSET,
             i2s_getreg(priv, STM32_SPI_CR2_OFFSET) | SPI_CR2_TXDMAEN);

  /* Start a watchdog to catch DMA timeouts */

  if (!notimeout)
    {
      ret = wd_start(&priv->tx.dog, timeout,
                     i2s_txdma_timeout, (wdparm_t)priv);

      /* Check if we have successfully started the watchdog timer.  Note
       * that we do nothing in the case of failure to start the timer.  We
       * are already committed to the DMA anyway.  Let's just hope that the
       * DMA does not hang.
       */

      if (ret < 0)
        {
          i2serr("ERROR: wd_start failed: %d\n", ret);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: i2s_tx_worker
 *
 * Description:
 *   TX transfer done worker
 *
 * Input Parameters:
 *   arg - the I2S device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static void i2s_tx_worker(void *arg)
{
  struct stm32_i2s_s *priv = (struct stm32_i2s_s *)arg;
  struct stm32_buffer_s *bfcontainer;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* When the transfer was started, the active buffer containers were removed
   * from the tx.pend queue and saved in the tx.act queue.  We get here when
   * the DMA is finished... either successfully, with a DMA error, or with a
   * DMA timeout.
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

  i2sinfo("tx.act.head=%p tx.done.head=%p\n",
           priv->tx.act.head, priv->tx.done.head);

  /* Check if the DMA is IDLE */

  if (sq_empty(&priv->tx.act))
    {
#ifdef CONFIG_STM32_I2S_DMADEBUG
      bfcontainer = (struct stm32_buffer_s *)sq_peek(&priv->tx.done);
      if (bfcontainer)
        {
          /* Dump the DMA registers */

          i2s_txdma_sampledone(priv, bfcontainer->result);
        }
#endif

      /* Then start the next DMA.  This must be done with interrupts
       * disabled.
       */

      flags = enter_critical_section();
      i2s_txdma_setup(priv);
      leave_critical_section(flags);
    }

  /* Process each buffer in the tx.done queue */

  while (sq_peek(&priv->tx.done) != NULL)
    {
      /* Remove the buffer container from the tx.done queue.  NOTE that
       * interrupts must be enabled to do this because the tx.done queue is
       * also modified from the interrupt level.
       */

      flags = enter_critical_section();
      bfcontainer = (struct stm32_buffer_s *)sq_remfirst(&priv->tx.done);
      leave_critical_section(flags);

      /* Perform the TX transfer done callback */

      DEBUGASSERT(bfcontainer && bfcontainer->callback);
      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, bfcontainer->result);

      /* Release our reference on the audio buffer.  This may very likely
       * cause the audio buffer to be freed.
       */

      apb_free(bfcontainer->apb);

      /* And release the buffer container */

      i2s_buf_free(priv, bfcontainer);
    }
}
#endif

/****************************************************************************
 * Name: i2s_tx_schedule
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

#ifdef I2S_HAVE_TX
static void i2s_tx_schedule(struct stm32_i2s_s *priv, int result)
{
  struct stm32_buffer_s *bfcontainer;
  int ret;

  /* Upon entry, the transfer(s) that just completed are the ones in the
   * priv->tx.act queue.  NOTE: In certain conditions, this function may
   * be called an additional time, hence, we can't assert this to be true.
   * For example, in the case of a timeout, this function will be called by
   * both indirectly via the stm32_dmastop() logic and directly via the
   * i2s_txdma_timeout() logic.
   */

  /* Move all entries from the tx.act queue to the tx.done queue */

  while (!sq_empty(&priv->tx.act))
    {
      /* Remove the next buffer container from the tx.act list */

      bfcontainer = (struct stm32_buffer_s *)sq_remfirst(&priv->tx.act);

      /* Report the result of the transfer */

      bfcontainer->result = result;

      /* Add the completed buffer container to the tail of the tx.done
       * queue
       */

      sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.done);
    }

  /* If the worker has completed running, then reschedule the working thread.
   * REVISIT:  There may be a race condition here.  So we do nothing is the
   * worker is not available.
   */

  if (work_available(&priv->tx.work))
    {
      /* Schedule the TX DMA done processing to occur on the worker thread. */

      ret = work_queue(HPWORK, &priv->tx.work, i2s_tx_worker, priv, 0);
      if (ret != 0)
        {
          i2serr("ERROR: Failed to queue TX work: %d\n", ret);
        }
    }
}
#endif

/****************************************************************************
 * Name: i2s_txdma_callback
 *
 * Description:
 *   This callback function is invoked at the completion of the I2S TX DMA.
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

#ifdef I2S_HAVE_TX
static void i2s_txdma_callback(DMA_HANDLE handle, uint8_t result, void *arg)
{
  struct stm32_i2s_s *priv = (struct stm32_i2s_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Cancel the watchdog timeout */

  wd_cancel(&priv->tx.dog);

  /* Sample DMA registers at the time of the DMA completion */

  i2s_txdma_sample(priv, DMA_CALLBACK);

  /* REVISIT:  We would like to the next DMA started here so that we do not
   * get audio glitches at the boundaries between DMA transfers.
   * Unfortunately, we cannot call stm32_dmasetup() from an interrupt
   * handler!
   */

  /* Then schedule completion of the transfer to occur on the worker thread */

  i2s_tx_schedule(priv, result);
}
#endif

/****************************************************************************
 * Name: i2s_checkwidth
 *
 * Description:
 *   Check for a valid bit width.  The I2S is capable of handling most any
 *   bit width from 8 to 16, but the DMA logic in this driver is constrained
 *   to 8- and 16-bit data widths
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static int i2s_checkwidth(struct stm32_i2s_s *priv, int bits)
{
  /* The I2S can handle most any bit width from 8 to 32.  However, the DMA
   * logic here is constrained to byte, half-word, and word sizes.
   */

  switch (bits)
    {
    case 8:
      priv->align = 0;
      break;

    case 16:
      priv->align = 1;
      break;

    default:
      i2serr("ERROR: Unsupported or invalid data width: %d\n", bits);
      return (bits < 8 || bits > 16) ? -EINVAL : -ENOSYS;
    }

  /* Save the new data width */

  priv->datalen = bits;
  return OK;
}

/****************************************************************************
 * Name: stm32_i2s_rxsamplerate
 *
 * Description:
 *   Set the I2S RX sample rate.  NOTE:  This will have no effect if (1) the
 *   driver does not support an I2C receiver or if (2) the sample rate is
 *   driven by the I2S frame clock.  This may also have unexpected side-
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

static uint32_t stm32_i2s_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
#if defined(I2S_HAVE_RX) && defined(I2S_HAVE_MCK)
  struct stm32_i2s_s *priv = (struct stm32_i2s_s *)dev;
  DEBUGASSERT(priv && priv->samplerate >= 0 && rate > 0);

  /* Check if the receiver is driven by the MCK */

  if (priv->samplerate != rate)
    {
      /* Save the new sample rate and update the MCK divider */

      priv->samplerate = rate;
      return i2s_mckdivider(priv);
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: stm32_i2s_rxdatawidth
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

static uint32_t stm32_i2s_rxdatawidth(struct i2s_dev_s *dev, int bits)
{
#ifdef I2S_HAVE_RX
  struct stm32_i2s_s *priv = (struct stm32_i2s_s *)dev;
  int ret;

  DEBUGASSERT(priv && bits > 1);

  /* Check if this is a bit width that we are configured to handle */

  ret = i2s_checkwidth(priv, bits);
  if (ret < 0)
    {
      i2serr("ERROR: i2s_checkwidth failed: %d\n", ret);
      return 0;
    }

  /* Update the DMA flags */

  ret = i2s_dma_flags(priv);
  if (ret < 0)
    {
      i2serr("ERROR: i2s_dma_flags failed: %d\n", ret);
      return 0;
    }

#endif

  return 0;
}

/****************************************************************************
 * Name: stm32_i2s_receive
 *
 * Description:
 *   Receive a block of data from I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer in which to receive data
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

static int stm32_i2s_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                       i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct stm32_i2s_s *priv = (struct stm32_i2s_s *)dev;
#ifdef I2S_HAVE_RX
  struct stm32_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;
#endif

  DEBUGASSERT(priv && apb && ((uintptr_t)apb->samp & priv->align) == 0);
  i2sinfo("apb=%p nmaxbytes=%d arg=%p timeout=%" PRId32 "\n",
          apb, apb->nmaxbytes, arg, timeout);

  i2s_init_buffer(apb->samp, apb->nmaxbytes);

#ifdef I2S_HAVE_RX
  /* Allocate a buffer container in advance */

  bfcontainer = i2s_buf_allocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the I2S driver data */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      goto errout_with_buf;
    }

  /* Has the RX channel been enabled? */

  if (!priv->rxenab)
    {
      i2serr("ERROR: I2S%d has no receiver\n", priv->i2sno);
      ret = -EAGAIN;
      goto errout_with_excllock;
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

  flags = enter_critical_section();
  sq_addlast((sq_entry_t *)bfcontainer, &priv->rx.pend);

  /* Then start the next transfer.  If there is already a transfer in
   * progress, then this will do nothing.
   */

  ret = i2s_rxdma_setup(priv);
  DEBUGASSERT(ret == OK);
  leave_critical_section(flags);
  nxmutex_unlock(&priv->lock);
  return OK;

errout_with_excllock:
  nxmutex_unlock(&priv->lock);

errout_with_buf:
  i2s_buf_free(priv, bfcontainer);
  return ret;

#else
  i2serr("ERROR: I2S%d has no receiver\n", priv->i2sno);
  UNUSED(priv);
  return -ENOSYS;
#endif
}

static int stm32_i2s_roundf(float num)
{
  if (((int)(num + 0.5f)) > num)
    {
      return num + 1;
    }

  return num;
}

/****************************************************************************
 * Name: stm32_i2s_txsamplerate
 *
 * Description:
 *   Set the I2S TX sample rate.  NOTE:  This will have no effect if (1) the
 *   driver does not support an I2S transmitter or if (2) the sample rate is
 *   driven by the I2S frame clock.  This may also have unexpected side-
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

static uint32_t stm32_i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
#if defined(I2S_HAVE_TX) && defined(I2S_HAVE_MCK)
  struct stm32_i2s_s *priv = (struct stm32_i2s_s *)dev;

  DEBUGASSERT(priv && priv->samplerate >= 0 && rate > 0);

  /* Check if the receiver is driven by the MCK/2 */

  if (priv->samplerate != rate)
    {
      /* Save the new sample rate and update the MCK/2 divider */

      priv->samplerate = rate;
      return i2s_mckdivider(priv);
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: stm32_i2s_txdatawidth
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

static uint32_t stm32_i2s_txdatawidth(struct i2s_dev_s *dev, int bits)
{
#ifdef I2S_HAVE_TX
  struct stm32_i2s_s *priv = (struct stm32_i2s_s *)dev;
  int ret;

  i2sinfo("Data width bits of tx = %d\n", bits);
  DEBUGASSERT(priv && bits > 1);

  /* Check if this is a bit width that we are configured to handle */

  ret = i2s_checkwidth(priv, bits);
  if (ret < 0)
    {
      i2serr("ERROR: i2s_checkwidth failed: %d\n", ret);
      return 0;
    }

  /* Update the DMA flags */

  ret = i2s_dma_flags(priv);
  if (ret < 0)
    {
      i2serr("ERROR: i2s_dma_flags failed: %d\n", ret);
      return 0;
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: stm32_i2s_send
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

static int stm32_i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                    i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct stm32_i2s_s *priv = (struct stm32_i2s_s *)dev;
#ifdef I2S_HAVE_TX
  struct stm32_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;
#endif

  /* Make sure that we have valid pointers that that the data has uint32_t
   * alignment.
   */

  DEBUGASSERT(priv && apb);
  i2sinfo("apb=%p nbytes=%d arg=%p timeout=%" PRId32 "\n",
          apb, apb->nbytes - apb->curbyte, arg, timeout);

  i2s_dump_buffer("Sending", &apb->samp[apb->curbyte],
                  apb->nbytes - apb->curbyte);
  DEBUGASSERT(((uintptr_t)&apb->samp[apb->curbyte] & priv->align) == 0);

#ifdef I2S_HAVE_TX
  /* Allocate a buffer container in advance */

  bfcontainer = i2s_buf_allocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the I2S driver data */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      goto errout_with_buf;
    }

  /* Has the TX channel been enabled? */

  if (!priv->txenab)
    {
      i2serr("ERROR: I2S%d has no transmitter\n", priv->i2sno);
      ret = -EAGAIN;
      goto errout_with_excllock;
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

  flags = enter_critical_section();
  sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.pend);

  /* Then start the next transfer.  If there is already a transfer in
   * progress, then this will do nothing.
   */

  ret = i2s_txdma_setup(priv);
  DEBUGASSERT(ret == OK);
  leave_critical_section(flags);
  nxmutex_unlock(&priv->lock);
  return OK;

errout_with_excllock:
  nxmutex_unlock(&priv->lock);

errout_with_buf:
  i2s_buf_free(priv, bfcontainer);
  return ret;

#else
  i2serr("ERROR: I2S%d has no transmitter\n", priv->i2sno);
  UNUSED(priv);
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: i2s_mckdivider
 *
 * Description:
 *   Setup the MCK divider based on the currently selected data width and
 *   the sample rate
 *
 * Input Parameters:
 *   priv - I2C device structure (only the sample rate and data length is
 *          needed at this point).
 *
 * Returned Value:
 *  The current bitrate
 *
 ****************************************************************************/

static uint32_t i2s_mckdivider(struct stm32_i2s_s *priv)
{
#ifdef I2S_HAVE_MCK
  uint32_t bitrate;
  uint32_t regval;

  uint16_t pllr = 5;
  uint16_t plln = 256;
  uint16_t div = 12;
  uint16_t odd = 1;

  DEBUGASSERT(priv && priv->samplerate >= 0 && priv->datalen > 0);

  /* A zero sample rate means to disable the MCK/2 clock */

  if (priv->samplerate == 0)
    {
      bitrate = 0;
      regval  = 0;
    }
  else
    {
      int R;
      int n;
      int od;
      int napprox;
      int diff;
      int diff_min = 500000000;

      for (od = 0; od <= 1; ++od)
        {
          for (R = 2; R <= 7; ++R)
            {
              for (n = 2; n <= 256; ++n)
                {
                  napprox = stm32_i2s_roundf(priv->samplerate / 1000000.0f *
                                             (8 * 32 * R * (2 * n + od)));
                  if ((napprox > 432) || (napprox < 50))
                    {
                      continue;
                    }

                  diff = abs(priv->samplerate - 1000000 * napprox /
                             (8 * 32 * R * (2 * n + od)));
                  if (diff_min > diff)
                    {
                      diff_min = diff;
                      plln = napprox;
                      pllr = R;
                      div = n;
                      odd = od;
                    }
                }
            }
        }

      /* Calculate the new bitrate in Hz */

      bitrate = priv->samplerate * priv->datalen;
    }

  /* Configure MCK divider */

  /* Disable I2S */

  i2s_putreg(priv, STM32_SPI_I2SCFGR_OFFSET, 0);

  /* I2S clock configuration */

  putreg32((getreg32(STM32_RCC_CR) & (~RCC_CR_PLLI2SON)), STM32_RCC_CR);

  /* PLLI2S clock used as I2S clock source */

  putreg32(((getreg32(STM32_RCC_CFGR)) & (~RCC_CFGR_I2SSRC)),
           STM32_RCC_CFGR);
  regval = (pllr << 28) | (plln << 6);
  putreg32(regval, STM32_RCC_PLLI2SCFGR);

  /* Enable PLLI2S and wait until it is ready */

  putreg32((getreg32(STM32_RCC_CR) | RCC_CR_PLLI2SON), STM32_RCC_CR);
  while (!(getreg32(STM32_RCC_CR) & RCC_CR_PLLI2SRDY));

  i2s_putreg(priv, STM32_SPI_I2SPR_OFFSET,
             div | (odd << 8) | SPI_I2SPR_MCKOE);
  i2s_putreg(priv, STM32_SPI_I2SCFGR_OFFSET,
             SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_MTX | SPI_I2SCFGR_I2SE);

#if 0
  putreg32((getreg32(STM32_DMA1_HIFCR) | DMA_HIFCR_CTCIF7),
           STM32_DMA1_HIFCR);
#endif

  putreg32((getreg32(STM32_DMA1_HIFCR) | 0x80000000), STM32_DMA1_HIFCR);

  return bitrate;
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: i2s_dma_flags
 *
 * Description:
 *   Determine DMA FLAGS based on PID and data width
 *
 * Input Parameters:
 *   priv - Partially initialized I2C device structure.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int i2s_dma_flags(struct stm32_i2s_s *priv)
{
  switch (priv->datalen)
    {
    case 8:

      /* Reconfigure the RX DMA (and TX DMA if applicable) */

      priv->rxccr = SPI_RXDMA8_CONFIG;
      priv->txccr = SPI_TXDMA8_CONFIG;
      break;

    case 16:
      priv->rxccr = SPI_RXDMA16_CONFIG;
      priv->txccr = SPI_TXDMA16_CONFIG;
      break;

    default:
      i2serr("ERROR: Unsupported data width: %d\n", priv->datalen);
      return -ENOSYS;
    }

  return OK;
}

/****************************************************************************
 * Name: i2s_dma_allocate
 *
 * Description:
 *   Allocate I2S DMA channels
 *
 * Input Parameters:
 *   priv - Partially initialized I2S device structure.  This function
 *          will complete the DMA specific portions of the initialization
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int i2s_dma_allocate(struct stm32_i2s_s *priv)
{
  int ret;

  /* Get the DMA flags for this channel */

  ret = i2s_dma_flags(priv);
  if (ret < 0)
    {
      i2serr("ERROR: i2s_dma_flags failed: %d\n", ret);
      return ret;
    }

  /* Allocate DMA channels.  These allocations exploit that fact that
   * I2S2 is managed by DMA1 and I2S3 is managed by DMA2.  Hence,
   * the I2S number (i2sno) is the same as the DMA number.
   */

#ifdef I2S_HAVE_RX
  if (priv->rxenab)
    {
      /* Allocate an RX DMA channel */

      priv->rx.dma = stm32_dmachannel(DMACHAN_I2S3_RX);
      if (!priv->rx.dma)
        {
          i2serr("ERROR: Failed to allocate the RX DMA channel\n");
          goto errout;
        }
    }
#endif

#ifdef I2S_HAVE_TX
  if (priv->txenab)
    {
      /* Allocate a TX DMA channel */

      priv->tx.dma = stm32_dmachannel(DMACHAN_I2S3_TX);
      if (!priv->tx.dma)
        {
          i2serr("ERROR: Failed to allocate the TX DMA channel\n");
          goto errout;
        }
    }
#endif

  /* Success exit */

  return OK;

  /* Error exit */

errout:
  i2s_dma_free(priv);
  return -ENOMEM;
}

/****************************************************************************
 * Name: i2s_dma_free
 *
 * Description:
 *   Release DMA-related resources allocated by i2s_dma_allocate()
 *
 * Input Parameters:
 *   priv - Partially initialized I2C device structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2s_dma_free(struct stm32_i2s_s *priv)
{
#ifdef I2S_HAVE_TX
  wd_cancel(&priv->tx.dog);
  if (priv->tx.dma)
    {
      stm32_dmafree(priv->tx.dma);
    }
#endif

#ifdef I2S_HAVE_RX
  wd_cancel(&priv->rx.dog);
  if (priv->rx.dma)
    {
      stm32_dmafree(priv->rx.dma);
    }
#endif
}

/****************************************************************************
 * Name: i2s2_configure
 *
 * Description:
 *   Configure I2S2
 *
 * Input Parameters:
 *   priv - Partially initialized I2C device structure.  These functions
 *          will complete the I2S specific portions of the initialization
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_I2S2
static void i2s2_configure(struct stm32_i2s_s *priv)
{
  /* Configure multiplexed pins as connected on the board.  Chip
   * select pins must be selected by board-specific logic.
   */

  priv->base  = STM32_I2S2_BASE;

#ifdef CONFIG_STM32_I2S2_RX
  priv->rxenab = true;

  if (!priv->initialized)
    {
      /* Configure I2S2 pins: MCK, SD, CK, WS */

      stm32_configgpio(GPIO_I2S2_MCK);
      stm32_configgpio(GPIO_I2S2_SD);
      stm32_configgpio(GPIO_I2S2_CK);
      stm32_configgpio(GPIO_I2S2_WS);
      priv->initialized = true;
    }
#endif /* CONFIG_STM32_I2S2_RX */

#ifdef CONFIG_STM32_I2S2_TX
  priv->txenab = true;

  /* Only configure if the port is not already configured */

  if (!priv->initialized)
    {
      /* Configure I2S2 pins: MCK, SD, CK, WS */

      stm32_configgpio(GPIO_I2S2_MCK);
      stm32_configgpio(GPIO_I2S2_SD);
      stm32_configgpio(GPIO_I2S2_CK);
      stm32_configgpio(GPIO_I2S2_WS);
      priv->initialized = true;
    }
#endif /* CONFIG_STM32_I2S2_TX */

  /* Configure driver state specific to this I2S peripheral */

  priv->datalen = CONFIG_STM32_I2S2_DATALEN;
#ifdef CONFIG_DEBUG
  priv->align   = STM32_I2S2_DATAMASK;
#endif
}
#endif /* CONFIG_STM32_I2S2 */

/****************************************************************************
 * Name: i2s3_configure
 *
 * Description:
 *   Configure I2S3
 *
 * Input Parameters:
 *   priv - Partially initialized I2C device structure.  These functions
 *          will complete the I2S specific portions of the initialization
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_I2S3
static void i2s3_configure(struct stm32_i2s_s *priv)
{
  /* Configure multiplexed pins as connected on the board.  Chip
   * select pins must be selected by board-specific logic.
   */

  priv->base    = STM32_I2S3_BASE;

#ifdef CONFIG_STM32_I2S3_RX
  priv->rxenab = true;

  if (!priv->initialized)
    {
      /* Configure I2S3 pins: MCK, SD, CK, WS */

      stm32_configgpio(GPIO_I2S3_MCK);
      stm32_configgpio(GPIO_I2S3_SD);
      stm32_configgpio(GPIO_I2S3_CK);
      stm32_configgpio(GPIO_I2S3_WS);
      priv->initialized = true;
    }
#endif /* CONFIG_STM32_I2S3_RX */

#ifdef CONFIG_STM32_I2S3_TX
  priv->txenab = true;

  /* Only configure if the port is not already configured */

  if (!priv->initialized)
    {
      /* Configure I2S3 pins: MCK, SD, CK, WS */

      stm32_configgpio(GPIO_I2S3_MCK);
      stm32_configgpio(GPIO_I2S3_SD);
      stm32_configgpio(GPIO_I2S3_CK);
      stm32_configgpio(GPIO_I2S3_WS);
      priv->initialized = true;
    }
#endif /* CONFIG_STM32_I2S3_TX */

  /* Configure driver state specific to this I2S peripheral */

  priv->datalen = CONFIG_STM32_I2S3_DATALEN;
#ifdef CONFIG_DEBUG
  priv->align   = STM32_I2S3_DATAMASK;
#endif
}
#endif /* CONFIG_STM32_I2S3 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2sbus_initialize
 *
 * Description:
 *   Initialize the selected i2S port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple I2S interfaces)
 *
 * Returned Value:
 *   Valid I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *stm32_i2sbus_initialize(int port)
{
  struct stm32_i2s_s *priv = NULL;
  irqstate_t flags;
  int ret;

  /* The support STM32 parts have only a single I2S port */

  i2sinfo("port: %d\n", port);

  /* Allocate a new state structure for this chip select.  NOTE that there
   * is no protection if the same chip select is used in two different
   * chip select structures.
   */

  priv = (struct stm32_i2s_s *)kmm_zalloc(sizeof(struct stm32_i2s_s));
  if (!priv)
    {
      i2serr("ERROR: Failed to allocate a chip select structure\n");
      return NULL;
    }

  /* Set up the initial state for this chip select structure.  Other fields
   * were zeroed by kmm_zalloc().
   */

  /* Initialize the common parts for the I2S device structure */

  nxmutex_init(&priv->lock);
  priv->dev.ops = &g_i2sops;
  priv->i2sno   = port;

  /* Initialize buffering */

  i2s_buf_initialize(priv);

  flags = enter_critical_section();

#ifdef CONFIG_STM32_I2S2
  if (port == 2)
    {
      /* Select I2S2 */

      i2s2_configure(priv);
    }
  else
#endif
#ifdef CONFIG_STM32_I2S3
  if (port == 3)
    {
      /* Select I2S3 */

      i2s3_configure(priv);
    }
  else
#endif
    {
      i2serr("ERROR: Unsupported I2S port: %d\n", port);
      leave_critical_section(flags);
      return NULL;
    }

  /* Allocate DMA channels */

  ret = i2s_dma_allocate(priv);
  if (ret < 0)
    {
      goto errout_with_alloc;
    }

  leave_critical_section(flags);
  i2s_dump_regs(priv, "After initialization");

  /* Success exit */

  return &priv->dev;

  /* Failure exits */

errout_with_alloc:
  leave_critical_section(flags);
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return NULL;
}
#endif /* I2S_HAVE_RX || I2S_HAVE_TX */

#endif /* CONFIG_STM32_I2S2 || CONFIG_STM32_I2S3 */

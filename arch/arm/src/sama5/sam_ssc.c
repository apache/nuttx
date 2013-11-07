/****************************************************************************
 * arch/arm/src/sama5/sam_ssc.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
#include <wdog.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/audio/i2s.h>

#include "up_internal.h"
#include "up_arch.h"

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
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* When SSC DMA is enabled, small DMA transfers will still be performed by
 * polling logic.  But we need a threshold value to determine what is small.
 * That value is provided by CONFIG_SAMA5_SSC_DMATHRESHOLD.
 */

#ifndef CONFIG_SAMA5_SSC_DMATHRESHOLD
#  define CONFIG_SAMA5_SSC_DMATHRESHOLD 4
#endif

#ifdef CONFIG_SAMA5_SSC_DMA

#  if defined(CONFIG_SAMA5_SSC0) && defined(CONFIG_SAMA5_DMAC0)
#    define SAMA5_SSC0_DMA true
#  else
#    define SAMA5_SSC0_DMA false
#  endif

#  if defined(CONFIG_SAMA5_SSC1) && defined(CONFIG_SAMA5_DMAC1)
#    define SAMA5_SSC1_DMA true
#  else
#    define SAMA5_SSC1_DMA false
#  endif
#endif

#ifndef CONFIG_SAMA5_SSC_DMA
#  undef CONFIG_SAMA5_SSC_DMADEBUG
#endif

/* Clocking *****************************************************************/
/* Select MCU-specific settings
 *
 * SSC is driven by the main clock.
 */

#define SAM_SSC_CLOCK     BOARD_MCK_FREQUENCY

/* DMA timeout.  The value is not critical; we just don't want the system to
 * hang in the event that a DMA does not finish.  This is set to
 */

#define DMA_TIMEOUT_MS    (800)
#define DMA_TIMEOUT_TICKS ((DMA_TIMEOUT_MS + (MSEC_PER_TICK-1)) / MSEC_PER_TICK)

/* Debug *******************************************************************/
/* Check if SSC debut is enabled (non-standard.. no support in
 * include/debug.h
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_I2S
#  undef CONFIG_SAMA5_SSC_DMADEBUG
#  undef CONFIG_SAMA5_SSC_REGDEBUG
#endif

#ifndef CONFIG_DEBUG_DMA
#  undef CONFIG_SAMA5_SSC_DMADEBUG
#endif

#ifdef CONFIG_DEBUG_I2S
#  define i2sdbg lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define i2svdbg lldbg
#  else
#    define i2svdbg(x...)
#  endif
#else
#  define i2sdbg(x...)
#  define i2svdbg(x...)
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

/* The state of the one SSC peripheral */

struct sam_i2s_s
{
  struct i2s_dev_s dev;        /* Externally visible I2S interface */
  uint32_t base;               /* SSC controller register base address */
  sem_t exclsem;               /* Assures mutually exclusive acess to SSC */
#if defined(CONFIG_SAMA5_SSC0) || defined(CONFIG_SAMA5_SSC1)
  uint8_t i2sno;               /* SSC controller number (0 or 1) */
#endif

#ifdef CONFIG_SAMA5_SSC_DMA
  uint8_t pid;                 /* Peripheral ID */
  bool candma;                 /* DMA is supported */
  sem_t dmawait;               /* Used to wait for DMA completion */
  WDOG_ID dmadog;              /* Watchdog that handles DMA timeouts */
  int result;                  /* DMA result */
  DMA_HANDLE rxdma;            /* SSC RX DMA handle */
  DMA_HANDLE txdma;            /* SSC TX DMA handle */
#endif

  /* Debug stuff */

#ifdef CONFIG_SAMA5_SSC_REGDEBUG
   bool     wr;                /* Last was a write */
   uint32_t regaddr;           /* Last address */
   uint32_t regval;            /* Last value */
   int      count;             /* Number of times */
#endif

#ifdef CONFIG_SAMA5_SSC_DMADEBUG
  struct sam_dmaregs_s rxdmaregs[DMA_NSAMPLES];
  struct sam_dmaregs_s txdmaregs[DMA_NSAMPLES];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

#ifdef CONFIG_SAMA5_SSC_REGDEBUG
static bool     i2s_checkreg(struct sam_i2s_s *i2s, bool wr, uint32_t value,
                  uint32_t address);
#else
# define        i2s_checkreg(i2s,wr,value,address) (false)
#endif

static inline uint32_t i2s_getreg(struct sam_i2s_s *i2s, unsigned int offset);
static inline void i2s_putreg(struct sam_i2s_s *i2s, uint32_t value,
                  unsigned int offset);
#ifdef CONFIG_SAMA5_SSC_DMA
static inline uintptr_t i2s_physregaddr(struct sam_i2s_s *i2s,
                  unsigned int offset);
#endif
#if defined(CONFIG_DEBUG_I2S) && defined(CONFIG_DEBUG_VERBOSE)
static void     i2s_dumpregs(struct sam_i2s_s *i2s, const char *msg);
#else
# define        i2s_dumpregs(i2s,msg)
#endif

/* DMA support */

#if defined(CONFIG_SAMA5_SSC_DMADEBUG)
#  define i2s_rxdma_sample(s,i) sam_dmasample((s)->rxdma, &(s)->rxdmaregs[i])
#  define i2s_txdma_sample(s,i) sam_dmasample((s)->txdma, &(s)->txdmaregs[i])
static void     i2s_dma_sampleinit(struct sam_i2s_s *i2s);
static void     i2s_dma_sampledone(struct sam_i2s_s *i2s);

#else
#  define i2s_rxdma_sample(s,i)
#  define i2s_txdma_sample(s,i)
#  define i2s_dma_sampleinit(s)
#  define i2s_dma_sampledone(s)

#endif

#ifdef CONFIG_SAMA5_SSC_DMA
static void     i2s_rxcallback(DMA_HANDLE handle, void *arg, int result);
static void     i2s_txcallback(DMA_HANDLE handle, void *arg, int result);
#endif

/* I2S methods */

static uint32_t i2s_frequency(FAR struct i2s_dev_s *dev, uint32_t frequency);
#ifdef CONFIG_SAMA5_SSC_DMA
static void     i2s_send_nodma(FAR struct i2s_dev_s *dev,
                   FAR const void *buffer, size_t nbytes);
static void     i2s_receive_nodma(FAR struct i2s_dev_s *dev, FAR void *buffer,
                   size_t nbytes);
#endif
static void     i2s_send(struct i2s_dev_s *dev,
                   const void *buffer, size_t nbytes);
static void     i2s_receive(struct i2s_dev_s *dev, void *buffer,
                   size_t nbytes);

/* Initialization */

#ifdef CONFIG_SAMA5_SSC0
static void     i2s_ssc0_configure(struct sam_i2s_s *i2s);
#endif
#ifdef CONFIG_SAMA5_SSC1
static void     i2s_ssc1_configure(struct sam_i2s_s *i2s);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* I2S device operations */

static const struct i2s_ops_s g_i2sops =
{
  .i2s_frequency = i2s_frequency,
  .i2s_send      = i2s_send,
  .i2s_receive   = i2s_receive,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 *   value   - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SSC_REGDEBUG
static bool i2s_checkreg(struct sam_i2s_s *i2s, bool wr, uint32_t value,
                         uint32_t address)
{
  if (wr      == i2s->wr &&     /* Same kind of access? */
      value   == i2s->regval &&  /* Same value? */
      address == i2s->regaddr)  /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      i2s->count++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (i2s->count > 0)
        {
          /* Yes... show how many times we did it */

          lldbg("...[Repeats %d times]...\n", i2s->count);
        }

      /* Save information about the new access */

      i2s->wr      = wr;
      i2s->regval   = value;
      i2s->regaddr = address;
      i2s->count      = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: i2s_getreg
 *
 * Description:
 *  Read an SSC register
 *
 ****************************************************************************/

static inline uint32_t i2s_getreg(struct sam_i2s_s *i2s,
                                  unsigned int offset)
{
  uint32_t address = i2s->base + offset;
  uint32_t value = getreg32(address);

#ifdef CONFIG_SAMA5_SSC_REGDEBUG
  if (i2s_checkreg(i2s, false, value, address))
    {
      lldbg("%08x->%08x\n", address, value);
    }
#endif

  return value;
}

/****************************************************************************
 * Name: i2s_putreg
 *
 * Description:
 *  Write a value to an SSC register
 *
 ****************************************************************************/

static inline void i2s_putreg(struct sam_i2s_s *i2s, uint32_t value,
                              unsigned int offset)
{
  uint32_t address = i2s->base + offset;

#ifdef CONFIG_SAMA5_SSC_REGDEBUG
  if (i2s_checkreg(i2s, true, value, address))
    {
      lldbg("%08x<-%08x\n", address, value);
    }
#endif

  putreg32(value, address);
}

/****************************************************************************
 * Name: i2s_physregaddr
 *
 * Description:
 *   Return the physical address of an HSMCI register
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SSC_DMA
static inline uintptr_t i2s_physregaddr(struct sam_i2s_s *i2s,
                                        unsigned int offset)
{
  return sam_physregaddr(i2s->base + offset);
}
#endif

/****************************************************************************
 * Name: i2s_dumpregs
 *
 * Description:
 *   Dump the contents of all SSC registers
 *
 * Input Parameters:
 *   i2s - The SSC controller to dump
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_I2S) && defined(CONFIG_DEBUG_VERBOSE)
static void i2s_dumpregs(struct sam_i2s_s *i2s, const char *msg)
{
  i2svdbg("SSC%d: %s\n", i2s->i2sno, msg);
  i2svdbg("   CMR:%08x RCMR:%08x RFMR:%08x TCMR:%08x\n",
          getreg32(i2s->base + SAM_SSC_CMR_OFFSET),
          getreg32(i2s->base + SAM_SSC_RCMR_OFFSET),
          getreg32(i2s->base + SAM_SSC_RFMR_OFFSET),
          getreg32(i2s->base + SAM_SSC_TCMR_OFFSET));
  i2svdbg("  TFMR:%08x RC0R:%08x RC1R:%08x   SR:%08x\n",
          getreg32(i2s->base + SAM_SSC_TFMR_OFFSET),
          getreg32(i2s->base + SAM_SSC_RC0R_OFFSET),
          getreg32(i2s->base + SAM_SSC_RC1R_OFFSET),
          getreg32(i2s->base + SAM_SSC_SR_OFFSET));
  i2svdbg("   IMR:%08x WPMR:%08x WPSR:%08x\n",
          getreg32(i2s->base + SAM_SSC_IMR_OFFSET),
          getreg32(i2s->base + SAM_SSC_WPMR_OFFSET),
          getreg32(i2s->base + SAM_SSC_WPSR_OFFSET));
}
#endif

/****************************************************************************
 * Name: i2s_dma_sampleinit
 *
 * Description:
 *   Initialize sampling of DMA registers (if CONFIG_SAMA5_SSC_DMADEBUG)
 *
 * Input Parameters:
 *   i2s - Chip select doing the DMA
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SSC_DMADEBUG
static void i2s_dma_sampleinit(struct sam_i2s_s *i2s)
{
  /* Put contents of register samples into a known state */

  memset(i2s->rxdmaregs, 0xff, DMA_NSAMPLES * sizeof(struct sam_dmaregs_s));
  memset(i2s->txdmaregs, 0xff, DMA_NSAMPLES * sizeof(struct sam_dmaregs_s));

  /* Then get the initial samples */

  sam_dmasample(i2s->rxdma, &i2s->rxdmaregs[DMA_INITIAL]);
  sam_dmasample(i2s->txdma, &i2s->txdmaregs[DMA_INITIAL]);
}
#endif

/****************************************************************************
 * Name: i2s_dma_sampledone
 *
 * Description:
 *   Dump sampled DMA registers
 *
 * Input Parameters:
 *   i2s - Chip select doing the DMA
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SSC_DMADEBUG
static void i2s_dma_sampledone(struct sam_i2s_s *i2s)
{
  /* Sample the final registers */

  sam_dmasample(i2s->rxdma, &i2s->rxdmaregs[DMA_END_TRANSFER]);
  sam_dmasample(i2s->txdma, &i2s->txdmaregs[DMA_END_TRANSFER]);

  /* Then dump the sampled DMA registers */
  /* Initial register values */

  sam_dmadump(i2s->txdma, &i2s->txdmaregs[DMA_INITIAL],
              "TX: Initial Registers");
  sam_dmadump(i2s->rxdma, &i2s->rxdmaregs[DMA_INITIAL],
              "RX: Initial Registers");

  /* Register values after DMA setup */

  sam_dmadump(i2s->txdma, &i2s->txdmaregs[DMA_AFTER_SETUP],
              "TX: After DMA Setup");
  sam_dmadump(i2s->rxdma, &i2s->rxdmaregs[DMA_AFTER_SETUP],
              "RX: After DMA Setup");

  /* Register values after DMA start */

  sam_dmadump(i2s->txdma, &i2s->txdmaregs[DMA_AFTER_START],
              "TX: After DMA Start");
  sam_dmadump(i2s->rxdma, &i2s->rxdmaregs[DMA_AFTER_START],
              "RX: After DMA Start");

  /* Register values at the time of the TX and RX DMA callbacks
   * -OR- DMA timeout.
   *
   * If the DMA timedout, then there will not be any RX DMA
   * callback samples.  There is probably no TX DMA callback
   * samples either, but we don't know for sure.
   */

  sam_dmadump(i2s->txdma, &i2s->txdmaregs[DMA_CALLBACK],
              "TX: At DMA callback");

  /* Register values at the end of the DMA */

  if (i2s->result == -ETIMEDOUT)
    {
      sam_dmadump(i2s->rxdma, &i2s->rxdmaregs[DMA_TIMEOUT],
                  "RX: At DMA timeout");
    }
  else
    {
      sam_dmadump(i2s->rxdma, &i2s->rxdmaregs[DMA_CALLBACK],
                  "RX: At DMA callback");
    }

  sam_dmadump(i2s->rxdma, &i2s->rxdmaregs[DMA_END_TRANSFER],
              "RX: At End-of-Transfer");
  sam_dmadump(i2s->txdma, &i2s->txdmaregs[DMA_END_TRANSFER],
              "TX: At End-of-Transfer");
}
#endif

/****************************************************************************
 * Name: i2s_dmatimeout
 *
 * Description:
 *   The watchdog timeout setup when a has expired without completion of a
 *   DMA.
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

#ifdef CONFIG_SAMA5_SSC_DMA
static void i2s_dmatimeout(int argc, uint32_t arg)
{
  struct sam_i2s_s *i2s = (struct sam_i2s_s *)arg;
  DEBUGASSERT(i2s != NULL);

  /* Sample DMA registers at the time of the timeout */

  i2s_rxdma_sample(i2s, DMA_CALLBACK);

  /* Report timeout result, perhaps overwriting any failure reports from
   * the TX callback.
   */

  i2s->result = -ETIMEDOUT;

  /* Then wake up the waiting thread */

  sem_post(&i2s->dmawait);
}
#endif

/****************************************************************************
 * Name: i2s_rxcallback
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

#ifdef CONFIG_SAMA5_SSC_DMA
static void i2s_rxcallback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_i2s_s *i2s = (struct sam_i2s_s *)arg;
  DEBUGASSERT(i2s != NULL);

  /* Cancel the watchdog timeout */

  (void)wd_cancel(i2s->dmadog);

  /* Sample DMA registers at the time of the callback */

  i2s_rxdma_sample(i2s, DMA_CALLBACK);

  /* Report the result of the transfer only if the TX callback has not already
   * reported an error.
   */

  if (i2s->result == -EBUSY)
    {
      /* Save the result of the transfer if no error was previuosly reported */

      i2s->result = result;
    }

  /* Then wake up the waiting thread */

  sem_post(&i2s->dmawait);
}
#endif

/****************************************************************************
 * Name: i2s_txcallback
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

#ifdef CONFIG_SAMA5_SSC_DMA
static void i2s_txcallback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_i2s_s *i2s = (struct sam_i2s_s *)arg;
  DEBUGASSERT(i2s != NULL);

  i2s_txdma_sample(i2s, DMA_CALLBACK);

  /* Do nothing on the TX callback unless an error is reported.  This
   * callback is not really important because the SSC exchange is not
   * complete until the RX callback is received.
   */

  if (result != OK && i2s->result == -EBUSY)
    {
      /* Save the result of the transfer if an error is reported */

      i2s->result = result;
    }
}
#endif

/****************************************************************************
 * Name: i2s_frequency
 *
 * Description:
 *   Set the SSC frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SSC frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t i2s_frequency(struct i2s_dev_s *dev, uint32_t frequency)
{
  struct sam_i2s_s *i2s = (struct sam_i2s_s *)dev;
  uint32_t actual;
  uint32_t regval;

  i2svdbg("Frequency=%d\n", frequency);
#warning Missing logic

  i2sdbg("Frequency %d->%d\n", frequency, actual);
  return actual;
}

/***************************************************************************
 * Name: i2s_send_nodma
 *
 * Description:
 *   Send a block of data on SSC without using DMA
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nbytes - the length of data to send from the buffer in number of bytes.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SSC_DMA
static void i2s_send_nodma(struct i2s_dev_s *dev, const void *buffer,
                           size_t nbytes)
#else
static void i2s_send(struct i2s_dev_s *dev, const void *buffer, size_t nbytes)
#endif
{
  struct sam_i2s_s *i2s = (struct sam_i2s_s *)dev;
  DEBUGASSERT(i2s);

  i2svdbg("buffer=%p nbytes=%d\n", buffer, (int)nbytes);
#warning Missing logic
}

/****************************************************************************
 * Name: i2s_receive_nodma
 *
 * Description:
 *   Receive a block of data from SSC without using DMA
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to recieve data
 *   nbytes - the length of data that can be received in the buffer in number
 *            of bytes.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SSC_DMA
static void i2s_receive_nodma(struct i2s_dev_s *dev, void *buffer,
                              size_t nbytes)
#else
static void i2s_receive(struct i2s_dev_s *dev, void *buffer, size_t nbytes)
#endif
{
  struct sam_i2s_s *i2s = (struct sam_i2s_s *)dev;
  DEBUGASSERT(i2s);

  i2svdbg("buffer=%p nbytes=%d\n", buffer, (int)nbytes);
#warning Missing logic
}

/***************************************************************************
 * Name: i2s_send
 *
 * Description:
 *   Send a block of data on SSC using DMA is possible
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nbytes - the length of data to send from the buffer in number of bytes.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SSC_DMA
static void i2s_send(struct i2s_dev_s *dev, const void *buffer, size_t nbytes)
{
  struct sam_i2s_s *i2s = (struct sam_i2s_s *)dev;
  DEBUGASSERT(i2s);

  i2svdbg("buffer=%p nbytes=%d\n", buffer, (int)nbytes);

  /* If we are not configured to do DMA OR is this is a very small transfer,
   * then defer the operation to i2s_send_nodma().
   */

  if (!i2s->candma || nbytes < CONFIG_SAMA5_SSC_DMATHRESHOLD)
    {
      i2s_send_nodma(dev, buffer, nbytes);
    }

  /* Otherwise, perform the transfer using DMA */

  else
    {
#warning Missing logic
    }
}
#endif

/****************************************************************************
 * Name: i2s_receive
 *
 * Description:
 *   Receive a block of data from SSC using DMA is possible
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to recieve data
 *   nbytes - the length of data that can be received in the buffer in number
 *            of bytes.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SSC_DMA
static void i2s_receive(struct i2s_dev_s *dev, void *buffer, size_t nbytes)
{
  struct sam_i2s_s *i2s = (struct sam_i2s_s *)dev;

  /* If we are not configured to do DMA OR is this is a very small transfer,
   * then defer the operation to i2s_send_nodma().
   */

  if (!i2s->candma || nbytes < CONFIG_SAMA5_SSC_DMATHRESHOLD)
    {
      i2s_send_nodma(dev, buffer, nbytes);
    }

  /* Otherwise, perform the transfer using DMA */

  else
    {
#warning Missing logic
    }
}
#endif

/****************************************************************************
 * Name: i2s_ssc0/1_configure
 *
 * Description:
 *   Configure SSC0 and/or SSC1
 *
 * Input Parameters:
 *   i2c - Partially initialized I2C device structure.  These functions
 *         will compolete the SSC specific portions of the initialization
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SSC0
static void i2s_ssc0_configure(struct sam_i2s_s *i2s)
{
  /* Enable clocking to the SSC0 peripheral */

  sam_ssc0_enableclk();

  /* Configure multiplexed pins as connected on the board.  Chip
   * select pins must be selected by board-specific logic.
   */

#ifdef CONFIG_SAMA5_SSC0_RX
  sam_configpio(PIO_SSC0_RD);
  sam_configpio(PIO_SSC0_RF);
  sam_configpio(PIO_SSC0_RK);
#ifdef CONFIG_SAMA5_SSC0_RX_EXTCLK
  sam_configpio(PIO_SSC0_RK); /* External clock received on the RK I/O pad */
#endif
#endif /* CONFIG_SAMA5_SSC0_RX */

#ifdef CONFIG_SAMA5_SSC0_TX
  sam_configpio(PIO_SSC0_TD);
  sam_configpio(PIO_SSC0_TF);
#ifdef CONFIG_SAMA5_SSC0_TX_EXTCLK
  sam_configpio(PIO_SSC0_TK); /* External clock received on the TK I/O pad */
#endif
#endif /* CONFIG_SAMA5_SSC0_TX */

  /* Configure driver state specific to this SSC peripheral */

  i2s->base   = SAM_SSC0_VBASE;
#ifdef CONFIG_SAMA5_SSC_DMA
  i2s->candma = SAMA5_SSC1_DMA;
  i2s->pid    = SAM_PID_SSC0;
#endif
}
#endif

#ifdef CONFIG_SAMA5_SSC1
static void i2s_ssc1_configure(struct sam_i2s_s *i2s)
{
  /* Enable clocking to the SSC1 peripheral */

  sam_ssc1_enableclk();

  /* Configure multiplexed pins as connected on the board.  Chip
   * select pins must be selected by board-specific logic.
   */

#ifdef CONFIG_SAMA5_SSC1_RX
  sam_configpio(PIO_SSC1_RD);
  sam_configpio(PIO_SSC1_RF);
#ifdef CONFIG_SAMA5_SSC1_RX_EXTCLK
  sam_configpio(PIO_SSC1_RK); /* External clock received on the RK I/O pad */
#endif
#endif /* CONFIG_SAMA5_SSC1_RX */

#ifdef CONFIG_SAMA5_SSC1_TX
  sam_configpio(PIO_SSC1_TD);
  sam_configpio(PIO_SSC1_TF);
#ifdef CONFIG_SAMA5_SSC1_TX_EXTCLK
  sam_configpio(PIO_SSC1_TK); /* External clock received on the TK I/O pad */
#endif
#endif /* CONFIG_SAMA5_SSC1_TX */

  /* Configure driver state specific to this SSC peripheral */

  i2s->base   = SAM_SSC1_VBASE;
#ifdef CONFIG_SAMA5_SSC_DMA
  i2s->candma = SAMA5_SSC1_DMA;
  i2s->pid    = SAM_PID_SSC1;
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_i2sinitialize
 *
 * Description:
 *   Initialize the selected SSC port
 *
 * Input Parameter:
 *   cs - Chip select number (identifying the "logical" SSC port)
 *
 * Returned Value:
 *   Valid SSC device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *up_i2sinitialize(int port)
{
  struct sam_i2s_s *i2s;
  irqstate_t flags;

  /* The support SAM parts have only a single SSC port */

  i2svdbg("port: %d\n", port);

  /* Allocate a new state structure for this chip select.  NOTE that there
   * is no protection if the same chip select is used in two different
   * chip select structures.
   */

  i2s = (struct sam_i2s_s *)zalloc(sizeof(struct sam_i2s_s));
  if (!i2s)
    {
      i2sdbg("ERROR: Failed to allocate a chip select structure\n");
      return NULL;
    }

  /* Set up the initial state for this chip select structure.  Other fields
   * were zeroed by zalloc().
   */

  /* Initialize the common parts for the SSC device structure  */

  sem_init(&i2s->exclsem, 0, 1);
  i2s->dev.ops = &g_i2sops;
  i2s->i2sno   = port;

  flags = irqsave();
#ifdef CONFIG_SAMA5_SSC0
  if (port == 0)
    {
      i2s_ssc0_configure(i2s);
    }
  else
#endif /* CONFIG_SAMA5_SSC0 */
#ifdef CONFIG_SAMA5_SSC1
  if (port == 1)
    {
      i2s_ssc1_configure(i2s);
    }
  else
#endif /* CONFIG_SAMA5_SSC1 */
    {
      i2sdbg("ERROR:  Unsupported I2S port: %d\n", port);
      goto errout_with_alloc;
    }

#ifdef CONFIG_SAMA5_SSC_DMA
  /* Pre-allocate DMA channels.  These allocations exploit that fact that
   * SSC0 is managed by DMAC0 and SSC1 is managed by DMAC1.  Hence,
   * the SSC number (port) is the same as the DMAC number.
   */

  if (i2s->candma)
    {
      i2s->rxdma = sam_dmachannel(port, 0);
      if (!i2s->rxdma)
        {
          i2sdbg("ERROR: Failed to allocate the RX DMA channel\n");
          i2s->candma = false;
        }
    }

  if (i2s->candma)
    {
      i2s->txdma = sam_dmachannel(port, 0);
      if (!i2s->txdma)
        {
          i2sdbg("ERROR: Failed to allocate the TX DMA channel\n");
          sam_dmafree(i2s->rxdma);
          i2s->rxdma  = NULL;
          i2s->candma = false;
        }
    }

  /* Initialize the SSC semaphore that is used to wake up the waiting
   * thread when the DMA transfer completes.
   */

  sem_init(&i2s->dmawait, 0, 0);

  /* Create a watchdog time to catch DMA timeouts */

  i2s->dmadog = wd_create();
  DEBUGASSERT(i2s->dmadog);
#endif

  /* Initialize I2S hardware */
#warning "Missing logic"
  irqrestore(flags);
  i2s_dumpregs(i2s, "After initialization");

  return &i2s->dev;

errout_with_alloc:
  sem_destroy(&i2s->exclsem);
  kfree(i2s);
  return NULL;
}

#endif /* CONFIG_SAMA5_SSC0 || CONFIG_SAMA5_SSC1 */
